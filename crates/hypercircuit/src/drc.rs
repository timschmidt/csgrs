//! Direct handoff from retained PCB intent and materialized geometry to `hyperdrc`.
//!
//! This adapter preserves source/net identity and reports every currently
//! lossy policy conversion. It does not run DRC itself; callers choose the
//! relevant `hyperdrc` checks and release profile.

use std::collections::{BTreeMap, BTreeSet};

use hypercurve::Point2 as CurvePoint2;
use hyperdrc::authoring_intent::{
    AuthoredComponentEnvelope, AuthoredComponentEnvelopeKind, AuthoredComponentSide,
    AuthoredKeepout, AuthoredKeepoutScope, AuthoredRoutedSlot, authored_component_readiness,
    authored_keepout_readiness, authored_routed_slot_readiness,
};
use hyperdrc::checks::{
    NET_IMPEDANCE_TARGET_READINESS_CHECK, minimum_mask_opening, net_constraint_readiness,
    paste_overhang, silkscreen_board_edge_clearance, silkscreen_min_width, silkscreen_overlap,
    solder_mask_board_edge_clearance, solder_mask_expansion, solder_mask_opening_spacing,
    stackup_readiness,
};
use hyperdrc::constraint_policy::{
    DifferentialRole, NetClassConfig, StackupConfig, StackupLayerConfig,
    StackupLayerKind as DrcStackupLayerKind,
};
use hyperdrc::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
use hyperdrc::{LayerMetadata, PcbSketch, Severity, Violation};
use hyperphysics::{
    MaterialPropertyGraph, MaterialPropertyKind, PropertyResolutionStatus, PropertyValue,
    SourceSpec,
};
use hyperreal::RealSign;

use crate::{
    BoardSide, DrillShape, KeepoutScope, LandPatternGraphicPrimitive, LayerRole, PcbLayout,
    PcbMaterializationReport, Plating, ProcessLayerRole, Real, StackupLayerKind,
};

/// HyperPhysics custom-property key for a dimensionless relative permittivity.
pub const PCB_RELATIVE_PERMITTIVITY_PROPERTY: &str = "pcb-relative-permittivity";

/// HyperPhysics custom-property key for a dimensionless dielectric loss tangent.
pub const PCB_LOSS_TANGENT_PROPERTY: &str = "pcb-loss-tangent";

/// PCB dielectric properties understood at the HyperCircuit/HyperPhysics boundary.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PcbDielectricProperty {
    /// Dimensionless relative permittivity, commonly called Dk.
    RelativePermittivity,
    /// Dimensionless dielectric loss tangent, commonly called Df.
    LossTangent,
}

impl PcbDielectricProperty {
    /// Returns the stable HyperPhysics custom-property key.
    pub const fn key(self) -> &'static str {
        match self {
            Self::RelativePermittivity => PCB_RELATIVE_PERMITTIVITY_PROPERTY,
            Self::LossTangent => PCB_LOSS_TANGENT_PROPERTY,
        }
    }
}

/// Source-attributed HyperPhysics property graphs keyed by retained material handle.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PcbMaterialPropertyLibrary {
    materials: BTreeMap<String, MaterialPropertyGraph>,
}

impl PcbMaterialPropertyLibrary {
    /// Adds or replaces the property graph for one retained stackup material handle.
    pub fn insert(
        &mut self,
        material: impl Into<String>,
        graph: MaterialPropertyGraph,
    ) -> Option<MaterialPropertyGraph> {
        self.materials.insert(material.into(), graph)
    }

    /// Adds a property graph and returns the library for fluent construction.
    pub fn with_material(
        mut self,
        material: impl Into<String>,
        graph: MaterialPropertyGraph,
    ) -> Self {
        self.insert(material, graph);
        self
    }

    /// Returns the source-attributed graph registered for a material handle.
    pub fn get(&self, material: &str) -> Option<&MaterialPropertyGraph> {
        self.materials.get(material)
    }

    /// Iterates material handles and their source-attributed property graphs.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &MaterialPropertyGraph)> {
        self.materials
            .iter()
            .map(|(material, graph)| (material.as_str(), graph))
    }
}

/// Why a HyperPhysics PCB material property could not be certified for HyperDRC.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PcbMaterialPropertyIssue {
    /// No matching assertion was present.
    Unknown,
    /// The property was retained only as an interval.
    Interval,
    /// Source assertions conflict.
    Conflict,
    /// The only usable-looking value was an explicitly external proposal.
    ExternalProposal,
    /// A matching assertion did not use the required dimensionless unit `1`.
    NonDimensionlessUnit,
    /// Relative permittivity was not certified strictly positive.
    NonPositive,
    /// Loss tangent was not certified nonnegative.
    Negative,
}

/// Exact, source-attributed dielectric values resolved for one physical layer.
#[derive(Clone, Debug, PartialEq)]
pub struct DrcDielectricMaterialEvidence {
    /// Retained physical stackup layer.
    pub layer: String,
    /// Retained HyperPhysics material handle.
    pub material: String,
    /// Exact dimensionless relative permittivity.
    pub relative_permittivity: Real,
    /// Sources contributing to relative-permittivity resolution.
    pub relative_permittivity_sources: Vec<SourceSpec>,
    /// Exact dimensionless dielectric loss tangent.
    pub loss_tangent: Real,
    /// Sources contributing to loss-tangent resolution.
    pub loss_tangent_sources: Vec<SourceSpec>,
}

/// Explicitly retained information that the current `hyperdrc` model cannot carry.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DrcHandoffOmission {
    /// The DRC drill carrier requires a plating boolean.
    UnspecifiedDrillPlating(String),
    /// An exact per-layer copper union was blocked; individual features remain present.
    BlockedLayerUnion { layer: u16, blocker: String },
    /// An exact mask, paste, or legend union was blocked.
    BlockedProcessLayer {
        role: ProcessLayerRole,
        blocker: String,
    },
    /// Authored keepout could not become a certified profile.
    InvalidKeepoutGeometry(String),
    /// A courtyard primitive cannot currently become a closed placement envelope.
    UnsupportedCourtyardGraphic { instance: String, graphic: String },
    /// Courtyard/body polygons could not become one certified placement envelope.
    InvalidComponentEnvelope(String),
    /// A placed component has neither a usable courtyard nor a body envelope.
    MissingComponentEnvelope(String),
    /// An impedance-controlled stackup has no dielectric layer to characterize.
    MissingDielectricLayer,
    /// An impedance-controlled dielectric layer has no retained material handle.
    MissingDielectricMaterial { layer: String },
    /// A PCB material property could not be certified from its HyperPhysics graph.
    UnresolvedDielectricProperty {
        layer: String,
        material: String,
        property: PcbDielectricProperty,
        issue: PcbMaterialPropertyIssue,
    },
    /// HyperDRC currently accepts one board-wide value, but resolved layers differ.
    HeterogeneousDielectricProperty {
        property: PcbDielectricProperty,
        materials: Vec<String>,
    },
}

/// One certified mask, paste, or legend image handed directly to HyperDRC.
#[derive(Clone, Debug)]
pub struct DrcProcessLayer {
    /// Side/process role.
    pub role: ProcessLayerRole,
    /// Stable source label used in findings.
    pub name: String,
    /// Exact-aware union image.
    pub sketch: PcbSketch,
}

/// One certified copper-layer union used for process-to-copper checks.
#[derive(Clone, Debug)]
pub struct DrcCopperLayerImage {
    /// Retained routing layer.
    pub layer: hyperpath::TraceLayer,
    /// Stable layer name used in findings.
    pub name: String,
    /// Exact-aware union image.
    pub sketch: PcbSketch,
}

/// Typed `hyperdrc` inputs plus an audit of non-representable retained intent.
#[derive(Clone, Debug)]
pub struct HyperDrcHandoff {
    /// Source-addressable geometry in hyperdrc's stable parsed-board model.
    pub board: BoardModel,
    /// Physical stackup readiness input.
    pub stackup: StackupConfig,
    /// Net-class readiness inputs.
    pub net_classes: Vec<NetClassConfig>,
    /// Successful per-layer HyperPhysics dielectric resolutions and their sources.
    pub dielectric_material_evidence: Vec<DrcDielectricMaterialEvidence>,
    /// Certified copper unions for process-layer comparisons.
    pub copper_layers: Vec<DrcCopperLayerImage>,
    /// Certified mask, paste, and legend unions.
    pub process_layers: Vec<DrcProcessLayer>,
    /// Source-addressable authored keepouts retained beyond parsed-board formats.
    pub authored_keepouts: Vec<AuthoredKeepout>,
    /// Exact routed-slot fabrication intent retained beyond circular drill tables.
    pub authored_slots: Vec<AuthoredRoutedSlot>,
    /// Every placed logical component expected by readiness coverage.
    pub expected_component_sources: Vec<String>,
    /// Side-aware placed courtyard/body envelopes.
    pub authored_components: Vec<AuthoredComponentEnvelope>,
    /// Retained conversion limitations.
    pub omissions: Vec<DrcHandoffOmission>,
}

/// How analytical target-impedance findings participate in release readiness.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ImpedanceTargetPolicy {
    /// Preserve HyperDRC findings as non-blocking analytical review warnings.
    Advisory,
    /// Promote target mismatch or unsupported analytical applicability to an error.
    ReleaseBlocking,
}

/// Thresholds for direct native-authoring readiness checks.
#[derive(Clone, Debug, PartialEq)]
pub struct DrcReadinessPolicy {
    /// Minimum routed-slot cutter width.
    pub minimum_route_width: Real,
    /// Minimum keepout/copper intersection area emitted in reports.
    pub minimum_keepout_report_area: Real,
    /// Minimum component-envelope intersection area emitted in reports.
    pub minimum_component_report_area: Real,
    /// Minimum mask-opening dimension.
    pub minimum_mask_opening: Real,
    /// Minimum spacing between mask openings.
    pub minimum_mask_spacing: Real,
    /// Largest permitted mask expansion beyond copper.
    pub maximum_mask_expansion: Real,
    /// Paste allowed beyond the corresponding surface copper image.
    pub paste_overhang_tolerance: Real,
    /// Minimum retained legend stroke width.
    pub minimum_silkscreen_width: Real,
    /// Clearance from legend/mask openings to the board profile.
    pub process_board_edge_clearance: Real,
    /// Minimum process-layer intersection area emitted in findings.
    pub minimum_process_report_area: Real,
    /// Optional selected copper layer names; empty means every layer.
    pub selected_layers: Vec<String>,
    /// Release disposition for HyperDRC's stable target-impedance check family.
    pub impedance_target_policy: ImpedanceTargetPolicy,
}

impl Default for DrcReadinessPolicy {
    fn default() -> Self {
        Self {
            minimum_route_width: (Real::one() / Real::from(5)).expect("nonzero exact denominator"),
            minimum_keepout_report_area: Real::zero(),
            minimum_component_report_area: Real::zero(),
            minimum_mask_opening: Real::zero(),
            minimum_mask_spacing: Real::zero(),
            maximum_mask_expansion: (Real::one() / Real::from(5))
                .expect("nonzero exact denominator"),
            paste_overhang_tolerance: Real::zero(),
            minimum_silkscreen_width: Real::zero(),
            process_board_edge_clearance: Real::zero(),
            minimum_process_report_area: Real::zero(),
            selected_layers: Vec::new(),
            impedance_target_policy: ImpedanceTargetPolicy::ReleaseBlocking,
        }
    }
}

/// HyperDRC findings produced directly from a native hypercircuit handoff.
#[derive(Debug)]
pub struct HyperDrcReadinessReport {
    /// Source-addressable active findings from stackup, net and authoring checks.
    pub violations: Vec<Violation>,
    /// Target-impedance disposition applied to the returned findings.
    pub impedance_target_policy: ImpedanceTargetPolicy,
    /// Number of HyperDRC target-impedance warnings promoted to release errors.
    pub promoted_impedance_findings: usize,
}

impl HyperDrcReadinessReport {
    /// True when no release-blocking HyperDRC error was produced.
    pub fn is_release_clean(&self) -> bool {
        !self
            .violations
            .iter()
            .any(|violation| violation.severity == Severity::Error)
    }
}

impl HyperDrcHandoff {
    /// Converts materialized geometry and its authoritative layout into DRC inputs.
    pub fn from_materialization(
        layout: &PcbLayout,
        materialized: &PcbMaterializationReport,
    ) -> Self {
        Self::from_materialization_with_materials(
            layout,
            materialized,
            &PcbMaterialPropertyLibrary::default(),
        )
    }

    /// Converts materialized geometry while resolving PCB dielectric properties
    /// from source-attributed HyperPhysics graphs.
    pub fn from_materialization_with_materials(
        layout: &PcbLayout,
        materialized: &PcbMaterializationReport,
        materials: &PcbMaterialPropertyLibrary,
    ) -> Self {
        let copper = materialized
            .copper_features
            .iter()
            .map(|feature| CopperFeature {
                layer: routing_layer_name(layout, feature.layer.0),
                net: feature.net.as_ref().map(|net| net.as_str().to_owned()),
                kind: match feature.kind {
                    crate::CopperFeatureKind::Pad => CopperKind::Pad,
                    crate::CopperFeatureKind::Route => CopperKind::Segment,
                    crate::CopperFeatureKind::Via => CopperKind::Via,
                    crate::CopperFeatureKind::Zone => CopperKind::Zone,
                    crate::CopperFeatureKind::Artwork => CopperKind::Artwork,
                },
                sketch: PcbSketch::new(
                    feature.profile.clone(),
                    Some(LayerMetadata {
                        name: feature.source.clone(),
                    }),
                ),
                location: [feature.anchor.x.clone(), feature.anchor.y.clone()],
            })
            .collect();

        let mut omissions = Vec::new();
        let mut drills = Vec::new();
        for drill in &materialized.drills {
            let diameter = match &drill.shape {
                DrillShape::Round { diameter } => diameter.clone(),
                DrillShape::Slot { .. } => continue,
            };
            let plated = match drill.plating {
                Plating::Plated => true,
                Plating::NonPlated => false,
                Plating::Unspecified => {
                    omissions.push(DrcHandoffOmission::UnspecifiedDrillPlating(
                        drill.source.clone(),
                    ));
                    false
                }
            };
            drills.push(DrillFeature {
                location: [drill.center.x.clone(), drill.center.y.clone()],
                diameter,
                net: materialized
                    .copper_features
                    .iter()
                    .find(|feature| feature.source == drill.source)
                    .and_then(|feature| feature.net.as_ref())
                    .map(|net| net.as_str().to_owned()),
                plated,
            });
        }

        for image in &materialized.copper_layers {
            if let Some(blocker) = &image.blocker {
                omissions.push(DrcHandoffOmission::BlockedLayerUnion {
                    layer: image.layer.0,
                    blocker: blocker.clone(),
                });
            }
        }
        let copper_layers = materialized
            .copper_layers
            .iter()
            .filter_map(|image| {
                image.copper.as_ref().map(|copper| {
                    let name = routing_layer_name(layout, image.layer.0);
                    DrcCopperLayerImage {
                        layer: image.layer,
                        sketch: PcbSketch::new(
                            copper.clone(),
                            Some(LayerMetadata { name: name.clone() }),
                        ),
                        name,
                    }
                })
            })
            .collect::<Vec<_>>();
        let process_layers = materialized
            .process_layers
            .iter()
            .filter_map(|image| match &image.image {
                Some(profile) => {
                    let name = process_role_name(image.role).to_owned();
                    Some(DrcProcessLayer {
                        role: image.role,
                        sketch: PcbSketch::new(
                            profile.clone(),
                            Some(LayerMetadata { name: name.clone() }),
                        ),
                        name,
                    })
                }
                None => {
                    if let Some(blocker) = &image.blocker {
                        omissions.push(DrcHandoffOmission::BlockedProcessLayer {
                            role: image.role,
                            blocker: blocker.clone(),
                        });
                    }
                    None
                }
            })
            .collect::<Vec<_>>();
        let authored_slots = materialized
            .drills
            .iter()
            .filter_map(|drill| match &drill.shape {
                DrillShape::Round { .. } => None,
                DrillShape::Slot { start, end, width } => Some(AuthoredRoutedSlot {
                    source: drill.source.clone(),
                    start: [start.x.clone(), start.y.clone()],
                    end: [end.x.clone(), end.y.clone()],
                    width: width.clone(),
                    plated: drill.plating == Plating::Plated,
                }),
            })
            .collect();
        let authored_keepouts = layout
            .keepouts
            .iter()
            .filter_map(|keepout| {
                let profile = csgrs::sketch::Profile::polygon_points(
                    &keepout
                        .boundary
                        .iter()
                        .map(|point| CurvePoint2::new(point.x.clone(), point.y.clone()))
                        .collect::<Vec<_>>(),
                );
                if profile.is_empty() {
                    omissions.push(DrcHandoffOmission::InvalidKeepoutGeometry(
                        keepout.id.as_str().to_owned(),
                    ));
                    return None;
                }
                let scope = match &keepout.scope {
                    KeepoutScope::All => AuthoredKeepoutScope::All,
                    KeepoutScope::Copper(layers) => AuthoredKeepoutScope::Copper(
                        layers
                            .iter()
                            .map(|layer| routing_layer_name(layout, layer.0))
                            .collect(),
                    ),
                    KeepoutScope::Vias => AuthoredKeepoutScope::Vias,
                    KeepoutScope::Components => AuthoredKeepoutScope::Components,
                };
                Some(AuthoredKeepout {
                    source: keepout.id.as_str().to_owned(),
                    sketch: PcbSketch::new(
                        profile,
                        Some(LayerMetadata {
                            name: format!("keepout:{}", keepout.id.as_str()),
                        }),
                    ),
                    scope,
                })
            })
            .collect();
        let expected_component_sources = layout
            .placements
            .iter()
            .map(|placement| placement.instance.as_str().to_owned())
            .collect::<Vec<_>>();
        let authored_components = layout
            .placements
            .iter()
            .filter_map(|placement| component_envelope(layout, placement, &mut omissions))
            .collect::<Vec<_>>();

        let resolved_net_classes = layout
            .rules
            .resolve_net_classes()
            .expect("validated layout has resolvable net classes");
        let mut net_classes = resolved_net_classes
            .iter()
            .map(|class| NetClassConfig {
                name: class.id.as_str().to_owned(),
                nets: class
                    .nets
                    .iter()
                    .map(|net| net.as_str().to_owned())
                    .collect(),
                min_width: class.min_trace_width.clone(),
                min_clearance: class.min_clearance.clone(),
                max_via_count: class.max_via_count,
                max_length: class.max_length.clone(),
                requires_reference_plane: Some(class.requires_reference_plane),
                requires_impedance_control: Some(class.target_impedance_ohms.is_some()),
                target_impedance_ohms: class.target_impedance_ohms.clone(),
                impedance_tolerance_ohms: class.impedance_tolerance_ohms.clone(),
                ..NetClassConfig::default()
            })
            .collect::<Vec<_>>();
        for pair in &layout.rules.differential_pairs {
            for (net, role, suffix) in [
                (&pair.positive, DifferentialRole::Positive, "positive"),
                (&pair.negative, DifferentialRole::Negative, "negative"),
            ] {
                net_classes.push(NetClassConfig {
                    name: format!("differential:{}:{suffix}", pair.id.as_str()),
                    nets: vec![net.as_str().to_owned()],
                    differential_pair: Some(pair.id.as_str().to_owned()),
                    differential_role: Some(role),
                    max_pair_spacing: Some(pair.spacing.clone()),
                    max_pair_skew: pair.max_skew.clone(),
                    requires_impedance_control: Some(pair.target_impedance_ohms.is_some()),
                    target_impedance_ohms: pair.target_impedance_ohms.clone(),
                    impedance_tolerance_ohms: pair.impedance_tolerance_ohms.clone(),
                    ..NetClassConfig::default()
                });
            }
        }

        let mut dielectric_material_evidence = Vec::new();
        let stackup = stackup_config(
            layout,
            materials,
            &mut omissions,
            &mut dielectric_material_evidence,
        );
        Self {
            board: BoardModel {
                source: layout.id.as_str().to_owned(),
                copper,
                drills,
                board_outline: Some(PcbSketch::new(
                    materialized.substrate.clone(),
                    Some(LayerMetadata {
                        name: "board-outline".into(),
                    }),
                )),
                panel_features: None,
            },
            stackup,
            net_classes,
            dielectric_material_evidence,
            copper_layers,
            process_layers,
            authored_keepouts,
            authored_slots,
            expected_component_sources,
            authored_components,
            omissions,
        }
    }

    /// Runs HyperDRC's native stackup/net-class and authored-intent checks.
    pub fn run_readiness(&self, policy: &DrcReadinessPolicy) -> HyperDrcReadinessReport {
        let boards = std::slice::from_ref(&self.board);
        let mut violations = stackup_readiness(Some(&self.stackup), boards);
        violations.extend(net_constraint_readiness(
            &self.net_classes,
            Some(&self.stackup),
            boards,
            &policy.selected_layers,
        ));
        let mut promoted_impedance_findings = 0;
        if policy.impedance_target_policy == ImpedanceTargetPolicy::ReleaseBlocking {
            for violation in &mut violations {
                if violation.check == NET_IMPEDANCE_TARGET_READINESS_CHECK
                    && violation.severity != Severity::Error
                {
                    violation.severity = Severity::Error;
                    promoted_impedance_findings += 1;
                }
            }
        }
        violations.extend(authored_keepout_readiness(
            &self.board,
            &self.authored_keepouts,
            &policy.minimum_keepout_report_area,
        ));
        violations.extend(authored_routed_slot_readiness(
            &self.authored_slots,
            &policy.minimum_route_width,
        ));
        violations.extend(authored_component_readiness(
            &self.expected_component_sources,
            &self.authored_components,
            &self.authored_keepouts,
            &policy.minimum_component_report_area,
        ));
        for process in &self.process_layers {
            match process.role {
                ProcessLayerRole::FrontSolderMask | ProcessLayerRole::BackSolderMask => {
                    violations.extend(minimum_mask_opening(
                        &process.name,
                        &process.sketch,
                        &policy.minimum_mask_opening,
                        &policy.minimum_process_report_area,
                    ));
                    violations.extend(solder_mask_opening_spacing(
                        &process.name,
                        &process.sketch,
                        &policy.minimum_mask_spacing,
                        &policy.minimum_process_report_area,
                    ));
                    if let Some(board_outline) = &self.board.board_outline {
                        violations.extend(solder_mask_board_edge_clearance(
                            &process.name,
                            &process.sketch,
                            "board-outline",
                            board_outline,
                            &policy.process_board_edge_clearance,
                            &policy.minimum_process_report_area,
                        ));
                    }
                    if let Some(copper) = self.surface_copper(process.role) {
                        violations.extend(solder_mask_expansion(
                            &copper.name,
                            &copper.sketch,
                            &process.name,
                            &process.sketch,
                            &policy.maximum_mask_expansion,
                            &policy.minimum_process_report_area,
                        ));
                    }
                }
                ProcessLayerRole::FrontPaste | ProcessLayerRole::BackPaste => {
                    if let Some(copper) = self.surface_copper(process.role) {
                        violations.extend(paste_overhang(
                            &process.name,
                            &process.sketch,
                            &copper.name,
                            &copper.sketch,
                            &policy.paste_overhang_tolerance,
                            &policy.minimum_process_report_area,
                        ));
                    }
                }
                ProcessLayerRole::FrontSilkscreen | ProcessLayerRole::BackSilkscreen => {
                    if let Some(board_outline) = &self.board.board_outline {
                        violations.extend(silkscreen_board_edge_clearance(
                            &process.name,
                            &process.sketch,
                            "board-outline",
                            board_outline,
                            &policy.process_board_edge_clearance,
                            &policy.minimum_process_report_area,
                        ));
                    }
                    violations.extend(silkscreen_min_width(
                        &process.name,
                        &process.sketch,
                        &policy.minimum_silkscreen_width,
                        &policy.minimum_process_report_area,
                    ));
                    if let Some(mask) = self.side_process_layer(process.role, true) {
                        violations.extend(silkscreen_overlap(
                            &process.name,
                            &process.sketch,
                            &mask.name,
                            &mask.sketch,
                            &policy.minimum_process_report_area,
                        ));
                    }
                }
            }
        }
        HyperDrcReadinessReport {
            violations,
            impedance_target_policy: policy.impedance_target_policy,
            promoted_impedance_findings,
        }
    }

    fn surface_copper(&self, role: ProcessLayerRole) -> Option<&DrcCopperLayerImage> {
        match role {
            ProcessLayerRole::FrontSolderMask
            | ProcessLayerRole::FrontPaste
            | ProcessLayerRole::FrontSilkscreen => self.copper_layers.first(),
            ProcessLayerRole::BackSolderMask
            | ProcessLayerRole::BackPaste
            | ProcessLayerRole::BackSilkscreen => self.copper_layers.last(),
        }
    }

    fn side_process_layer(
        &self,
        role: ProcessLayerRole,
        solder_mask: bool,
    ) -> Option<&DrcProcessLayer> {
        let wanted = match (role, solder_mask) {
            (
                ProcessLayerRole::FrontSolderMask
                | ProcessLayerRole::FrontPaste
                | ProcessLayerRole::FrontSilkscreen,
                true,
            ) => ProcessLayerRole::FrontSolderMask,
            (
                ProcessLayerRole::BackSolderMask
                | ProcessLayerRole::BackPaste
                | ProcessLayerRole::BackSilkscreen,
                true,
            ) => ProcessLayerRole::BackSolderMask,
            _ => return None,
        };
        self.process_layers
            .iter()
            .find(|layer| layer.role == wanted)
    }
}

fn component_envelope(
    layout: &PcbLayout,
    placement: &crate::PcbPlacement,
    omissions: &mut Vec<DrcHandoffOmission>,
) -> Option<AuthoredComponentEnvelope> {
    let Some(pattern) = layout
        .land_patterns
        .iter()
        .find(|pattern| pattern.id == placement.land_pattern)
    else {
        omissions.push(DrcHandoffOmission::MissingComponentEnvelope(
            placement.instance.as_str().to_owned(),
        ));
        return None;
    };
    let courtyard_graphics = pattern
        .graphics
        .iter()
        .filter(|graphic| graphic.layer == LayerRole::Courtyard)
        .collect::<Vec<_>>();
    let mut courtyard_complete = true;
    let mut courtyard_profiles = Vec::new();
    for graphic in &courtyard_graphics {
        let LandPatternGraphicPrimitive::Polygon { vertices, .. } = &graphic.primitive else {
            courtyard_complete = false;
            omissions.push(DrcHandoffOmission::UnsupportedCourtyardGraphic {
                instance: placement.instance.as_str().to_owned(),
                graphic: graphic.id.as_str().to_owned(),
            });
            continue;
        };
        let profile = placed_polygon(vertices, placement);
        if profile.is_empty() {
            courtyard_complete = false;
            omissions.push(DrcHandoffOmission::InvalidComponentEnvelope(format!(
                "courtyard:{}:{}",
                placement.instance.as_str(),
                graphic.id.as_str()
            )));
        } else {
            courtyard_profiles.push(profile);
        }
    }
    if !courtyard_graphics.is_empty() && courtyard_complete {
        match union_component_profiles(courtyard_profiles) {
            Ok(Some(profile)) => {
                return Some(authored_component(
                    placement,
                    profile,
                    AuthoredComponentEnvelopeKind::Courtyard,
                ));
            }
            Ok(None) => {}
            Err(error) => omissions.push(DrcHandoffOmission::InvalidComponentEnvelope(format!(
                "courtyard:{}:{error}",
                placement.instance.as_str()
            ))),
        }
    }
    if let Some(body) = &pattern.body {
        let profile = placed_polygon(&body.outline, placement);
        if !profile.is_empty() {
            return Some(authored_component(
                placement,
                profile,
                AuthoredComponentEnvelopeKind::Body,
            ));
        }
        omissions.push(DrcHandoffOmission::InvalidComponentEnvelope(format!(
            "body:{}",
            placement.instance.as_str()
        )));
    }
    omissions.push(DrcHandoffOmission::MissingComponentEnvelope(
        placement.instance.as_str().to_owned(),
    ));
    None
}

fn placed_polygon(
    vertices: &[hyperlattice::Point2],
    placement: &crate::PcbPlacement,
) -> csgrs::sketch::Profile {
    csgrs::sketch::Profile::polygon_points(
        &vertices
            .iter()
            .map(|point| {
                let point = placement.transform_point(point);
                CurvePoint2::new(point.x, point.y)
            })
            .collect::<Vec<_>>(),
    )
}

fn union_component_profiles(
    profiles: Vec<csgrs::sketch::Profile>,
) -> Result<Option<csgrs::sketch::Profile>, String> {
    let mut profiles = profiles.into_iter();
    let Some(mut combined) = profiles.next() else {
        return Ok(None);
    };
    for profile in profiles {
        combined = combined
            .try_union(&profile)
            .map_err(|error| format!("{error:?}"))?;
    }
    Ok(Some(combined))
}

fn authored_component(
    placement: &crate::PcbPlacement,
    profile: csgrs::sketch::Profile,
    kind: AuthoredComponentEnvelopeKind,
) -> AuthoredComponentEnvelope {
    AuthoredComponentEnvelope {
        source: placement.instance.as_str().to_owned(),
        side: match placement.side {
            BoardSide::Front => AuthoredComponentSide::Front,
            BoardSide::Back => AuthoredComponentSide::Back,
        },
        sketch: PcbSketch::new(
            profile,
            Some(LayerMetadata {
                name: format!("component:{}", placement.instance.as_str()),
            }),
        ),
        kind,
    }
}

fn stackup_config(
    layout: &PcbLayout,
    materials: &PcbMaterialPropertyLibrary,
    omissions: &mut Vec<DrcHandoffOmission>,
    evidence: &mut Vec<DrcDielectricMaterialEvidence>,
) -> StackupConfig {
    let copper_layer_count = layout
        .stackup
        .layers
        .iter()
        .filter(|layer| matches!(layer.kind, StackupLayerKind::Conductor(_)))
        .count();
    let finished_thickness = layout
        .stackup
        .layers
        .iter()
        .fold(crate::Real::zero(), |sum, layer| {
            sum + layer.thickness.clone()
        });
    let impedance_controlled = layout
        .rules
        .net_classes
        .iter()
        .any(|class| class.target_impedance_ohms.is_some())
        || layout
            .rules
            .differential_pairs
            .iter()
            .any(|pair| pair.target_impedance_ohms.is_some());
    let properties = impedance_controlled
        .then(|| resolve_dielectric_properties(layout, materials, omissions, evidence))
        .flatten();
    StackupConfig {
        copper_layer_count: Some(copper_layer_count),
        finished_thickness: Some(finished_thickness),
        impedance_controlled: Some(impedance_controlled),
        material_family: properties
            .as_ref()
            .map(|properties| properties.materials.join(",")),
        material_dielectric_constant: properties
            .as_ref()
            .map(|properties| properties.relative_permittivity.clone()),
        material_loss_tangent: properties
            .as_ref()
            .map(|properties| properties.loss_tangent.clone()),
        layers: layout
            .stackup
            .layers
            .iter()
            .map(|layer| StackupLayerConfig {
                name: layer.name.clone(),
                kind: match layer.kind {
                    StackupLayerKind::Conductor(_) => DrcStackupLayerKind::Copper,
                    StackupLayerKind::Dielectric => DrcStackupLayerKind::Dielectric,
                    StackupLayerKind::SolderMask => DrcStackupLayerKind::SolderMask,
                    StackupLayerKind::Custom(_) => DrcStackupLayerKind::Other,
                },
                dielectric_thickness: matches!(layer.kind, StackupLayerKind::Dielectric)
                    .then(|| layer.thickness.clone()),
                ..StackupLayerConfig::default()
            })
            .collect(),
        ..StackupConfig::default()
    }
}

#[derive(Debug)]
struct ResolvedDielectricProperties {
    materials: Vec<String>,
    relative_permittivity: Real,
    loss_tangent: Real,
}

#[derive(Debug)]
struct ResolvedPcbMaterialProperty {
    value: Real,
    sources: Vec<SourceSpec>,
}

fn resolve_dielectric_properties(
    layout: &PcbLayout,
    materials: &PcbMaterialPropertyLibrary,
    omissions: &mut Vec<DrcHandoffOmission>,
    evidence: &mut Vec<DrcDielectricMaterialEvidence>,
) -> Option<ResolvedDielectricProperties> {
    if !layout
        .stackup
        .layers
        .iter()
        .any(|layer| layer.kind == StackupLayerKind::Dielectric)
    {
        omissions.push(DrcHandoffOmission::MissingDielectricLayer);
        return None;
    }
    let mut material_handles = BTreeSet::new();
    let mut relative_permittivities = Vec::new();
    let mut loss_tangents = Vec::new();
    let mut complete = true;

    for layer in layout
        .stackup
        .layers
        .iter()
        .filter(|layer| layer.kind == StackupLayerKind::Dielectric)
    {
        let Some(material) = layer.material.as_deref() else {
            complete = false;
            omissions.push(DrcHandoffOmission::MissingDielectricMaterial {
                layer: layer.name.clone(),
            });
            continue;
        };
        material_handles.insert(material.to_owned());
        let Some(graph) = materials.get(material) else {
            complete = false;
            for property in [
                PcbDielectricProperty::RelativePermittivity,
                PcbDielectricProperty::LossTangent,
            ] {
                omissions.push(DrcHandoffOmission::UnresolvedDielectricProperty {
                    layer: layer.name.clone(),
                    material: material.to_owned(),
                    property,
                    issue: PcbMaterialPropertyIssue::Unknown,
                });
            }
            continue;
        };

        let relative_permittivity = resolve_dimensionless_property(
            graph,
            &layer.name,
            material,
            PcbDielectricProperty::RelativePermittivity,
            true,
            omissions,
        );
        let loss_tangent = resolve_dimensionless_property(
            graph,
            &layer.name,
            material,
            PcbDielectricProperty::LossTangent,
            false,
            omissions,
        );
        match (relative_permittivity, loss_tangent) {
            (Some(relative_permittivity), Some(loss_tangent)) => {
                relative_permittivities
                    .push((material.to_owned(), relative_permittivity.value.clone()));
                loss_tangents.push((material.to_owned(), loss_tangent.value.clone()));
                evidence.push(DrcDielectricMaterialEvidence {
                    layer: layer.name.clone(),
                    material: material.to_owned(),
                    relative_permittivity: relative_permittivity.value,
                    relative_permittivity_sources: relative_permittivity.sources,
                    loss_tangent: loss_tangent.value,
                    loss_tangent_sources: loss_tangent.sources,
                });
            }
            _ => complete = false,
        }
    }

    let relative_permittivity = homogeneous_property(
        PcbDielectricProperty::RelativePermittivity,
        &relative_permittivities,
        omissions,
    );
    let loss_tangent = homogeneous_property(
        PcbDielectricProperty::LossTangent,
        &loss_tangents,
        omissions,
    );
    if !complete {
        return None;
    }
    Some(ResolvedDielectricProperties {
        materials: material_handles.into_iter().collect(),
        relative_permittivity: relative_permittivity?,
        loss_tangent: loss_tangent?,
    })
}

fn resolve_dimensionless_property(
    graph: &MaterialPropertyGraph,
    layer: &str,
    material: &str,
    property: PcbDielectricProperty,
    require_positive: bool,
    omissions: &mut Vec<DrcHandoffOmission>,
) -> Option<ResolvedPcbMaterialProperty> {
    let kind = MaterialPropertyKind::Custom(property.key().to_owned());
    if graph
        .assertions()
        .iter()
        .filter(|assertion| assertion.kind == kind)
        .any(|assertion| assertion.unit.trim() != "1")
    {
        omissions.push(DrcHandoffOmission::UnresolvedDielectricProperty {
            layer: layer.to_owned(),
            material: material.to_owned(),
            property,
            issue: PcbMaterialPropertyIssue::NonDimensionlessUnit,
        });
        return None;
    }
    let resolution = graph.resolve(&kind);
    let issue = match resolution.status {
        PropertyResolutionStatus::ExactKnown => None,
        PropertyResolutionStatus::Interval => Some(PcbMaterialPropertyIssue::Interval),
        PropertyResolutionStatus::Conflict => Some(PcbMaterialPropertyIssue::Conflict),
        PropertyResolutionStatus::ExternalProposal => {
            Some(PcbMaterialPropertyIssue::ExternalProposal)
        }
        PropertyResolutionStatus::Unknown => Some(PcbMaterialPropertyIssue::Unknown),
    };
    if let Some(issue) = issue {
        omissions.push(DrcHandoffOmission::UnresolvedDielectricProperty {
            layer: layer.to_owned(),
            material: material.to_owned(),
            property,
            issue,
        });
        return None;
    }
    let Some(PropertyValue::ExactScalar(value)) = resolution.value else {
        unreachable!("exact HyperPhysics property resolution must carry an exact scalar");
    };
    let sign = value.refine_sign_until(-64);
    let valid = if require_positive {
        sign == Some(RealSign::Positive)
    } else {
        matches!(sign, Some(RealSign::Positive | RealSign::Zero))
    };
    if !valid {
        omissions.push(DrcHandoffOmission::UnresolvedDielectricProperty {
            layer: layer.to_owned(),
            material: material.to_owned(),
            property,
            issue: if require_positive {
                PcbMaterialPropertyIssue::NonPositive
            } else {
                PcbMaterialPropertyIssue::Negative
            },
        });
        return None;
    }
    Some(ResolvedPcbMaterialProperty {
        value: *value,
        sources: resolution.sources,
    })
}

fn homogeneous_property(
    property: PcbDielectricProperty,
    values: &[(String, Real)],
    omissions: &mut Vec<DrcHandoffOmission>,
) -> Option<Real> {
    let (_, first) = values.first()?;
    if values.iter().any(|(_, value)| value != first) {
        omissions.push(DrcHandoffOmission::HeterogeneousDielectricProperty {
            property,
            materials: values
                .iter()
                .map(|(material, _)| material.clone())
                .collect::<BTreeSet<_>>()
                .into_iter()
                .collect(),
        });
        return None;
    }
    Some(first.clone())
}

fn routing_layer_name(layout: &PcbLayout, index: u16) -> String {
    layout
        .stackup
        .layers
        .iter()
        .find_map(|layer| match layer.kind {
            StackupLayerKind::Conductor(candidate) if candidate.0 == index => {
                Some(layer.name.clone())
            }
            _ => None,
        })
        .unwrap_or_else(|| format!("copper-{index}"))
}

fn process_role_name(role: ProcessLayerRole) -> &'static str {
    match role {
        ProcessLayerRole::FrontSolderMask => "F.Mask",
        ProcessLayerRole::BackSolderMask => "B.Mask",
        ProcessLayerRole::FrontPaste => "F.Paste",
        ProcessLayerRole::BackPaste => "B.Paste",
        ProcessLayerRole::FrontSilkscreen => "F.Silkscreen",
        ProcessLayerRole::BackSilkscreen => "B.Silkscreen",
    }
}

#[cfg(test)]
mod tests {
    use hyperpath::TraceLayer;
    use hyperphysics::{MaterialAssertion, MaterialState, PropertyValue, SourceSpec};

    use super::*;
    use crate::{BoardId, BoardOutline, NetClass, NetClassId, NetId, PcbStackup, StackupLayer};

    fn exact_ratio(numerator: i64, denominator: i64) -> Real {
        (Real::from(numerator) / Real::from(denominator)).expect("nonzero denominator")
    }

    fn material_graph(relative_permittivity: Real, loss_tangent: Real) -> MaterialPropertyGraph {
        let mut graph = MaterialPropertyGraph::default();
        for (property, value) in [
            (PCB_RELATIVE_PERMITTIVITY_PROPERTY, relative_permittivity),
            (PCB_LOSS_TANGENT_PROPERTY, loss_tangent),
        ] {
            graph.push(MaterialAssertion {
                kind: MaterialPropertyKind::Custom(property.to_owned()),
                value: PropertyValue::exact_scalar(value),
                unit: "1".into(),
                state: MaterialState::Cured,
                condition: Some("datasheet nominal test condition".into()),
                source: SourceSpec::new("laminate-datasheet", "revision-a"),
            });
        }
        graph
    }

    fn controlled_layout(material: Option<&str>) -> PcbLayout {
        let mut layout = PcbLayout::new(
            BoardId::new("impedance-board").unwrap(),
            BoardOutline::rectangle(Real::from(10), Real::from(10)),
            PcbStackup {
                layers: vec![
                    StackupLayer {
                        name: "F.Cu".into(),
                        kind: StackupLayerKind::Conductor(TraceLayer(0)),
                        thickness: exact_ratio(35, 1_000),
                        material: Some("hyperphysics:material/copper".into()),
                    },
                    StackupLayer {
                        name: "Core".into(),
                        kind: StackupLayerKind::Dielectric,
                        thickness: exact_ratio(18, 100),
                        material: material.map(str::to_owned),
                    },
                    StackupLayer {
                        name: "B.Cu".into(),
                        kind: StackupLayerKind::Conductor(TraceLayer(1)),
                        thickness: exact_ratio(35, 1_000),
                        material: Some("hyperphysics:material/copper".into()),
                    },
                ],
            },
        );
        layout.rules.net_classes.push(NetClass {
            id: NetClassId::new("rf").unwrap(),
            parent: None,
            nets: vec![NetId::new("RF").unwrap()],
            min_trace_width: None,
            preferred_trace_width: None,
            min_clearance: None,
            preferred_via_land_diameter: None,
            preferred_via_drill_diameter: None,
            preferred_via_style: None,
            max_length: None,
            max_via_count: None,
            target_impedance_ohms: Some(Real::from(50)),
            impedance_tolerance_ohms: Some(Real::from(5)),
            requires_reference_plane: true,
        });
        layout
    }

    #[test]
    fn hyperphysics_material_graph_enables_hyperdrc_impedance_estimate() {
        let layout = controlled_layout(Some("hyperphysics:material/fr4"));
        let materials = PcbMaterialPropertyLibrary::default().with_material(
            "hyperphysics:material/fr4",
            material_graph(exact_ratio(42, 10), exact_ratio(18, 1_000)),
        );
        let mut omissions = Vec::new();
        let mut evidence = Vec::new();
        let stackup = stackup_config(&layout, &materials, &mut omissions, &mut evidence);

        assert!(omissions.is_empty());
        assert_eq!(evidence.len(), 1);
        assert_eq!(evidence[0].layer, "Core");
        assert_eq!(
            evidence[0].relative_permittivity_sources[0].authority,
            "laminate-datasheet"
        );
        assert_eq!(
            stackup.material_dielectric_constant,
            Some(exact_ratio(42, 10))
        );
        assert_eq!(stackup.material_loss_tangent, Some(exact_ratio(18, 1_000)));

        let handoff = HyperDrcHandoff {
            board: BoardModel {
                source: "impedance-board".into(),
                copper: vec![CopperFeature {
                    layer: "F.Cu".into(),
                    net: Some("RF".into()),
                    kind: CopperKind::Segment,
                    sketch: PcbSketch::new(
                        csgrs::sketch::Profile::rectangle(exact_ratio(8, 100), Real::one()),
                        Some(LayerMetadata {
                            name: "route:rf".into(),
                        }),
                    ),
                    location: [Real::zero(), Real::zero()],
                }],
                drills: Vec::new(),
                board_outline: None,
                panel_features: None,
            },
            stackup,
            net_classes: vec![NetClassConfig {
                name: "rf".into(),
                nets: vec!["RF".into()],
                requires_impedance_control: Some(true),
                target_impedance_ohms: Some(Real::from(50)),
                impedance_tolerance_ohms: Some(Real::from(5)),
                ..NetClassConfig::default()
            }],
            dielectric_material_evidence: evidence,
            copper_layers: Vec::new(),
            process_layers: Vec::new(),
            authored_keepouts: Vec::new(),
            authored_slots: Vec::new(),
            expected_component_sources: Vec::new(),
            authored_components: Vec::new(),
            omissions,
        };
        let report = handoff.run_readiness(&DrcReadinessPolicy::default());
        assert!(report.violations.iter().any(|violation| {
            violation.check == NET_IMPEDANCE_TARGET_READINESS_CHECK
                && violation.severity == Severity::Error
                && violation.message.as_deref().is_some_and(|message| {
                    message.contains("estimated outer microstrip impedance")
                        && message.contains("outside target")
                })
        }));
        assert!(!report.is_release_clean());
        assert_eq!(
            report.impedance_target_policy,
            ImpedanceTargetPolicy::ReleaseBlocking
        );
        assert_eq!(report.promoted_impedance_findings, 1);

        let advisory = handoff.run_readiness(&DrcReadinessPolicy {
            impedance_target_policy: ImpedanceTargetPolicy::Advisory,
            ..DrcReadinessPolicy::default()
        });
        assert!(advisory.is_release_clean());
        assert_eq!(advisory.promoted_impedance_findings, 0);
        assert!(advisory.violations.iter().any(|violation| {
            violation.check == NET_IMPEDANCE_TARGET_READINESS_CHECK
                && violation.severity == Severity::Warning
        }));
    }

    #[test]
    fn unresolved_material_properties_remain_release_auditable() {
        let layout = controlled_layout(Some("hyperphysics:material/fr4"));
        let mut omissions = Vec::new();
        let mut evidence = Vec::new();
        let stackup = stackup_config(
            &layout,
            &PcbMaterialPropertyLibrary::default(),
            &mut omissions,
            &mut evidence,
        );

        assert!(evidence.is_empty());
        assert!(stackup.material_dielectric_constant.is_none());
        assert!(stackup.material_loss_tangent.is_none());
        assert_eq!(
            omissions,
            vec![
                DrcHandoffOmission::UnresolvedDielectricProperty {
                    layer: "Core".into(),
                    material: "hyperphysics:material/fr4".into(),
                    property: PcbDielectricProperty::RelativePermittivity,
                    issue: PcbMaterialPropertyIssue::Unknown,
                },
                DrcHandoffOmission::UnresolvedDielectricProperty {
                    layer: "Core".into(),
                    material: "hyperphysics:material/fr4".into(),
                    property: PcbDielectricProperty::LossTangent,
                    issue: PcbMaterialPropertyIssue::Unknown,
                },
            ]
        );
    }

    #[test]
    fn heterogeneous_dielectrics_are_not_collapsed_into_false_precision() {
        let mut layout = controlled_layout(Some("hyperphysics:material/fr4-a"));
        layout.stackup.layers.insert(
            2,
            StackupLayer {
                name: "In1.Cu".into(),
                kind: StackupLayerKind::Conductor(TraceLayer(2)),
                thickness: exact_ratio(35, 1_000),
                material: Some("hyperphysics:material/copper".into()),
            },
        );
        layout.stackup.layers.insert(
            3,
            StackupLayer {
                name: "Core B".into(),
                kind: StackupLayerKind::Dielectric,
                thickness: exact_ratio(18, 100),
                material: Some("hyperphysics:material/fr4-b".into()),
            },
        );
        let materials = PcbMaterialPropertyLibrary::default()
            .with_material(
                "hyperphysics:material/fr4-a",
                material_graph(exact_ratio(42, 10), exact_ratio(18, 1_000)),
            )
            .with_material(
                "hyperphysics:material/fr4-b",
                material_graph(exact_ratio(38, 10), exact_ratio(18, 1_000)),
            );
        let mut omissions = Vec::new();
        let mut evidence = Vec::new();
        let stackup = stackup_config(&layout, &materials, &mut omissions, &mut evidence);

        assert_eq!(evidence.len(), 2);
        assert!(stackup.material_dielectric_constant.is_none());
        assert!(omissions.iter().any(|omission| matches!(
            omission,
            DrcHandoffOmission::HeterogeneousDielectricProperty {
                property: PcbDielectricProperty::RelativePermittivity,
                materials,
            } if materials == &vec![
                "hyperphysics:material/fr4-a".to_owned(),
                "hyperphysics:material/fr4-b".to_owned(),
            ]
        )));
    }
}
