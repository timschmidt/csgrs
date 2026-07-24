//! Semantic KiCad PCB subset import with explicit assumptions and losses.
//!
//! `hyperdrc` parses KiCad into verification geometry. This module serves a
//! different boundary: editable circuit/layout intent. It therefore retains
//! source nets, footprint-local pads, placements, route centerlines, vias,
//! zones, and board contours instead of reverse-engineering those nouns from
//! flattened polygons.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::{Display, Formatter};
use std::path::Path;

use hyperlattice::Point2;
use hyperpath::{ArcDirection, ExplicitCircularArc, LinePathSegment, TraceLayer};
use hyperreal::{Real, RealSign};

use crate::sexp;
use crate::{
    AdapterKind, BoardContour, BoardContourSegment, BoardId, BoardOutline, BoardSide, Circuit,
    CircuitId, CircuitInstance, CircuitInstanceId, ComponentId, CopperZone, DeviceModel,
    DeviceModelId, DeviceModelKind, DevicePin, DrillShape, LandPattern, LandPatternId,
    LandPatternPad, Net, NetClass, NetClassId, NetId, PadId, PadPinMap, PadShape, PartRef,
    Pcb3dModelReference, Pcb3dModelTransform, PcbDesignRules, PcbLayout, PcbPlacement, PcbRoute,
    PcbStackup, PcbVia, PinBinding, PinElectricalKind, PinRef, Plating, RouteId, StackupLayer,
    StackupLayerKind, TransientPolicy, ViaId, ZoneId,
};

/// Caller-controlled policy for source facts KiCad does not retain per layer.
#[derive(Clone, Debug, PartialEq)]
pub struct KiCadImportOptions {
    /// Stable identity assigned to the imported circuit graph.
    pub circuit_id: CircuitId,
    /// Stable identity assigned to the imported PCB.
    pub board_id: BoardId,
    /// Positive thickness assigned to each declared copper layer.
    pub assumed_conductor_thickness: Real,
}

impl KiCadImportOptions {
    /// Creates an explicit semantic import policy.
    pub fn new(
        circuit_id: CircuitId,
        board_id: BoardId,
        assumed_conductor_thickness: Real,
    ) -> Self {
        Self {
            circuit_id,
            board_id,
            assumed_conductor_thickness,
        }
    }
}

/// Exact decimal token imported from KiCad syntax.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct KiCadNumericImport {
    /// Semantic destination field.
    pub field: String,
    /// KiCad decimal source token.
    pub source: String,
    /// Exact value retained by hypercircuit.
    pub exact: String,
}

/// Source intent absent from or unsupported by the semantic subset importer.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum KiCadImportOmission {
    /// Per-conductor thickness was supplied by import policy, not the file.
    AssumedConductorThickness { layer: String },
    /// Dielectric/material/finish stackup details were not reconstructed.
    DetailedStackup,
    /// Stable feature ids were generated because KiCad omitted them.
    GeneratedFeatureIds { feature: String, count: usize },
    /// A pad shape was conservatively represented by its declared size box.
    UnsupportedPadShape {
        footprint: String,
        pad: String,
        shape: String,
    },
    /// A footprint graphic or non-pad semantic was not imported.
    FootprintGraphics { footprint: String },
    /// No KiCad custom-design-rule companion was supplied.
    BoardRules,
    /// No KiCad project companion was supplied, so native class defaults were unavailable.
    ProjectNetClasses,
    /// A custom rule was outside the generated net-name constraint subset.
    UnsupportedDesignRule { rule: String },
    /// A native project class field is outside the retained PCB class subset.
    UnsupportedProjectNetClassField { net_class: String, field: String },
    /// An exact project assignment referenced an absent net or class.
    UnsupportedProjectNetClassAssignment { net: String },
    /// KiCad assigned one net to multiple classes, which hypercircuit cannot retain directly.
    CompositeProjectNetClassAssignment {
        net: String,
        net_classes: Vec<String>,
    },
    /// A wildcard-capable project pattern cannot be treated as an exact assignment.
    UnsupportedProjectNetClassPattern { pattern: String, net_class: String },
    /// KiCad net names do not authoritatively identify the circuit ground net.
    GroundRoleNotInferred,
    /// Imported footprint pins use generic passive electrical classes.
    GenericPinElectricalKinds { footprints: usize },
    /// A keepout zone remains outside the imported semantic subset.
    KeepoutZone,
    /// Copper without a declared logical net could not become a routed feature.
    UnconnectedCopper { feature: String },
    /// A non-line Edge.Cuts primitive could not become a polygonal contour.
    UnsupportedEdgePrimitive { primitive: String },
    /// Repeated footprint library names had different retained pad definitions.
    DivergentLandPattern { footprint: String },
    /// The supported via subset did not reconstruct mask opening/tenting intent.
    ViaMaskIntentUnavailable { count: usize },
    /// A KiCad midpoint arc could not be certified as one exact circular route arc.
    UnsupportedRouteArc { index: usize },
    /// Zone priority/fill/clearance/connection policy was defaulted by the subset importer.
    ZonePolicyDefaulted { zone: String },
}

/// Editable semantic design reconstructed from a supported KiCad PCB subset.
#[derive(Clone, Debug, PartialEq)]
pub struct KiCadImportReport {
    /// Reconstructed logical graph with generic passive footprint models.
    pub circuit: Circuit,
    /// Reconstructed retained PCB intent.
    pub layout: PcbLayout,
    /// Exact numeric token audit.
    pub numeric_imports: Vec<KiCadNumericImport>,
    /// Assumptions and unsupported source intent.
    pub omissions: Vec<KiCadImportOmission>,
}

/// Failure to construct a structurally valid editable design.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum KiCadImportError {
    /// The S-expression was malformed.
    Parse(String),
    /// A required decimal token could not be retained exactly.
    InvalidNumber { field: String, token: String },
    /// Import policy supplied a nonpositive or indeterminate thickness.
    InvalidConductorThickness,
    /// No closed polygonal Edge.Cuts contour was found.
    MissingBoardOutline,
    /// A referenced copper layer was absent from the declared layer table.
    UnknownCopperLayer(String),
    /// Imported ids or cross-references did not pass semantic validation.
    InvalidImportedDesign {
        circuit_issues: usize,
        layout_issues: usize,
    },
    /// Source file could not be read.
    Io(String),
}

impl Display for KiCadImportError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Parse(message) => write!(formatter, "KiCad parse error: {message}"),
            Self::InvalidNumber { field, token } => {
                write!(formatter, "invalid KiCad number {token:?} for {field}")
            }
            Self::InvalidConductorThickness => {
                formatter.write_str("assumed KiCad conductor thickness must be positive")
            }
            Self::MissingBoardOutline => {
                formatter.write_str("KiCad board has no closed polygonal Edge.Cuts contour")
            }
            Self::UnknownCopperLayer(layer) => write!(formatter, "unknown KiCad layer {layer}"),
            Self::InvalidImportedDesign {
                circuit_issues,
                layout_issues,
            } => write!(
                formatter,
                "imported design has {circuit_issues} circuit and {layout_issues} layout issue(s)"
            ),
            Self::Io(message) => write!(formatter, "failed to read KiCad board: {message}"),
        }
    }
}

impl std::error::Error for KiCadImportError {}

impl KiCadImportReport {
    /// Imports semantic intent from a KiCad board string.
    pub fn from_str(source: &str, options: KiCadImportOptions) -> Result<Self, KiCadImportError> {
        Self::from_str_with_companions(source, None, None, options)
    }

    /// Imports a board plus an optional `.kicad_dru` custom-rule companion.
    pub fn from_str_with_design_rules(
        source: &str,
        design_rules: Option<&str>,
        options: KiCadImportOptions,
    ) -> Result<Self, KiCadImportError> {
        Self::from_str_with_companions(source, None, design_rules, options)
    }

    /// Imports a board plus optional `.kicad_pro` and `.kicad_dru` companions.
    pub fn from_str_with_companions(
        source: &str,
        project: Option<&str>,
        design_rules: Option<&str>,
        options: KiCadImportOptions,
    ) -> Result<Self, KiCadImportError> {
        if options.assumed_conductor_thickness.structural_facts().sign != Some(RealSign::Positive) {
            return Err(KiCadImportError::InvalidConductorThickness);
        }
        let root = sexp::parse(source).map_err(KiCadImportError::Parse)?;
        if root.list_name() != Some("kicad_pcb") {
            return Err(KiCadImportError::Parse(
                "root expression is not kicad_pcb".into(),
            ));
        }
        let design_rules = design_rules
            .map(sexp::parse_many)
            .transpose()
            .map_err(KiCadImportError::Parse)?;
        let project = project
            .map(serde_json::from_str)
            .transpose()
            .map_err(|error| KiCadImportError::Parse(format!("project JSON: {error}")))?;
        Importer::new(root, project, design_rules, options).import()
    }

    /// Imports semantic intent from a `.kicad_pcb` file.
    pub fn from_path(path: &Path, options: KiCadImportOptions) -> Result<Self, KiCadImportError> {
        let source = std::fs::read_to_string(path)
            .map_err(|error| KiCadImportError::Io(error.to_string()))?;
        Self::from_str(&source, options)
    }

    /// Imports a `.kicad_pcb` and its `.kicad_dru` companion.
    pub fn from_paths(
        board_path: &Path,
        design_rules_path: &Path,
        options: KiCadImportOptions,
    ) -> Result<Self, KiCadImportError> {
        let board = std::fs::read_to_string(board_path)
            .map_err(|error| KiCadImportError::Io(error.to_string()))?;
        let rules = std::fs::read_to_string(design_rules_path)
            .map_err(|error| KiCadImportError::Io(error.to_string()))?;
        Self::from_str_with_design_rules(&board, Some(&rules), options)
    }

    /// Imports a `.kicad_pcb`, `.kicad_pro`, and optional `.kicad_dru` companion.
    pub fn from_project_paths(
        board_path: &Path,
        project_path: &Path,
        design_rules_path: Option<&Path>,
        options: KiCadImportOptions,
    ) -> Result<Self, KiCadImportError> {
        let board = std::fs::read_to_string(board_path)
            .map_err(|error| KiCadImportError::Io(error.to_string()))?;
        let project = std::fs::read_to_string(project_path)
            .map_err(|error| KiCadImportError::Io(error.to_string()))?;
        let rules = design_rules_path
            .map(std::fs::read_to_string)
            .transpose()
            .map_err(|error| KiCadImportError::Io(error.to_string()))?;
        Self::from_str_with_companions(&board, Some(&project), rules.as_deref(), options)
    }
}

struct Importer {
    root: sexp::Sexp,
    project: Option<serde_json::Value>,
    design_rules: Option<Vec<sexp::Sexp>>,
    options: KiCadImportOptions,
    numeric_imports: Vec<KiCadNumericImport>,
    omissions: Vec<KiCadImportOmission>,
    nets: BTreeMap<i32, NetId>,
    layers: BTreeMap<String, TraceLayer>,
}

impl Importer {
    fn new(
        root: sexp::Sexp,
        project: Option<serde_json::Value>,
        design_rules: Option<Vec<sexp::Sexp>>,
        options: KiCadImportOptions,
    ) -> Self {
        Self {
            root,
            project,
            design_rules,
            options,
            numeric_imports: Vec::new(),
            omissions: Vec::new(),
            nets: BTreeMap::new(),
            layers: BTreeMap::new(),
        }
    }

    fn import(mut self) -> Result<KiCadImportReport, KiCadImportError> {
        let net_nodes = self.root.named_children("net").cloned().collect::<Vec<_>>();
        let mut circuit = Circuit::new(
            self.options.circuit_id.clone(),
            TransientPolicy::Static,
            AdapterKind::Dc,
        );
        for net in net_nodes {
            let Some(code) = net.i32_at(1) else {
                continue;
            };
            let Some(name) = net.atom_at(2).filter(|name| !name.is_empty()) else {
                continue;
            };
            let id =
                NetId::new(name).map_err(|error| KiCadImportError::Parse(error.to_string()))?;
            if self.nets.insert(code, id.clone()).is_none()
                && !circuit.nets.iter().any(|net| net.id == id)
            {
                circuit.nets.push(Net {
                    id,
                    is_ground: false,
                });
            }
        }

        let stackup = self.parse_layers()?;
        let outline = self.parse_outline()?;
        let mut layout = PcbLayout {
            id: self.options.board_id.clone(),
            outline,
            stackup,
            land_patterns: Vec::new(),
            placements: Vec::new(),
            placement_constraints: Vec::new(),
            routes: Vec::new(),
            vias: Vec::new(),
            zones: Vec::new(),
            keepouts: Vec::new(),
            rules: PcbDesignRules::default(),
        };
        self.parse_footprints(&mut circuit, &mut layout)?;
        if !layout.placements.is_empty() {
            self.omissions
                .push(KiCadImportOmission::GenericPinElectricalKinds {
                    footprints: layout.placements.len(),
                });
        }
        self.parse_routes(&mut layout)?;
        self.parse_vias(&mut layout)?;
        self.parse_zones(&mut layout)?;
        self.parse_project_net_classes(&circuit, &mut layout.rules)?;
        self.parse_design_rules(&circuit, &mut layout.rules)?;
        self.omissions
            .push(KiCadImportOmission::GroundRoleNotInferred);

        let circuit_report = circuit.validate();
        let layout_report = layout.validate(&circuit);
        if !circuit_report.is_valid() || !layout_report.is_valid() {
            return Err(KiCadImportError::InvalidImportedDesign {
                circuit_issues: circuit_report.issues.len(),
                layout_issues: layout_report.issues.len(),
            });
        }
        Ok(KiCadImportReport {
            circuit,
            layout,
            numeric_imports: self.numeric_imports,
            omissions: self.omissions,
        })
    }

    fn parse_layers(&mut self) -> Result<PcbStackup, KiCadImportError> {
        let layer_nodes = self
            .root
            .named_child("layers")
            .into_iter()
            .flat_map(|layers| layers.children().iter().skip(1))
            .filter(|layer| layer.atom_at(1).is_some_and(|name| name.ends_with(".Cu")))
            .cloned()
            .collect::<Vec<_>>();
        if layer_nodes.is_empty() {
            return Err(KiCadImportError::UnknownCopperLayer(
                "no declared copper layers".into(),
            ));
        }
        let mut declared = Vec::new();
        for (index, node) in layer_nodes.into_iter().enumerate() {
            let name = node
                .atom_at(1)
                .expect("filtered layer has a name")
                .to_owned();
            let index = u16::try_from(index)
                .map_err(|_| KiCadImportError::Parse("too many copper layers".into()))?;
            let trace = TraceLayer(index);
            self.layers.insert(name.clone(), trace);
            declared.push((name, trace));
        }

        let stackup = self
            .root
            .named_child("setup")
            .and_then(|setup| setup.named_child("stackup"))
            .cloned();
        let Some(stackup) = stackup else {
            self.omissions.push(KiCadImportOmission::DetailedStackup);
            let layers = declared
                .into_iter()
                .map(|(name, trace)| {
                    self.omissions
                        .push(KiCadImportOmission::AssumedConductorThickness {
                            layer: name.clone(),
                        });
                    StackupLayer {
                        name,
                        kind: StackupLayerKind::Conductor(trace),
                        thickness: self.options.assumed_conductor_thickness.clone(),
                        material: None,
                    }
                })
                .collect();
            return Ok(PcbStackup { layers });
        };

        let mut layers = Vec::new();
        let mut seen_conductors = BTreeSet::new();
        for (index, node) in stackup.named_children("layer").cloned().enumerate() {
            let Some(name) = node.atom_at(1).map(str::to_owned) else {
                continue;
            };
            let Some(kind_name) = node
                .named_child("type")
                .and_then(|kind| kind.atom_at(1))
                .map(str::to_owned)
            else {
                continue;
            };
            if node.named_child("thickness").is_none() {
                continue;
            }
            let thickness = self.number_child(
                &node,
                "thickness",
                &format!("stackup.layer[{index}].thickness"),
            )?;
            let material = node
                .named_child("material")
                .and_then(|material| material.atom_at(1))
                .map(str::to_owned);
            let kind = if let Some(trace) = self.layers.get(&name).copied() {
                seen_conductors.insert(name.clone());
                StackupLayerKind::Conductor(trace)
            } else {
                match kind_name.to_ascii_lowercase().as_str() {
                    "core" | "prepreg" | "dielectric" => StackupLayerKind::Dielectric,
                    kind if kind.contains("solder mask") => StackupLayerKind::SolderMask,
                    _ => StackupLayerKind::Custom(kind_name),
                }
            };
            layers.push(StackupLayer {
                name,
                kind,
                thickness,
                material,
            });
        }
        for (name, trace) in declared {
            if !seen_conductors.contains(&name) {
                self.omissions
                    .push(KiCadImportOmission::AssumedConductorThickness {
                        layer: name.clone(),
                    });
                layers.push(StackupLayer {
                    name,
                    kind: StackupLayerKind::Conductor(trace),
                    thickness: self.options.assumed_conductor_thickness.clone(),
                    material: None,
                });
            }
        }
        Ok(PcbStackup { layers })
    }

    fn parse_project_net_classes(
        &mut self,
        circuit: &Circuit,
        rules: &mut PcbDesignRules,
    ) -> Result<(), KiCadImportError> {
        let Some(project) = self.project.clone() else {
            self.omissions.push(KiCadImportOmission::ProjectNetClasses);
            return Ok(());
        };
        let settings = project
            .get("net_settings")
            .and_then(serde_json::Value::as_object)
            .ok_or_else(|| {
                KiCadImportError::Parse("project JSON has no net_settings object".into())
            })?;
        let version = settings
            .get("meta")
            .and_then(|meta| meta.get("version"))
            .and_then(serde_json::Value::as_u64)
            .ok_or_else(|| {
                KiCadImportError::Parse("project net_settings has no schema version".into())
            })?;
        if !(3..=4).contains(&version) {
            return Err(KiCadImportError::Parse(format!(
                "unsupported KiCad project net-settings version {version}"
            )));
        }
        let classes = settings
            .get("classes")
            .and_then(serde_json::Value::as_array)
            .ok_or_else(|| {
                KiCadImportError::Parse("project net_settings classes is not an array".into())
            })?;
        let mut class_indices = BTreeMap::new();
        let mut priorities = BTreeMap::new();
        for entry in classes {
            let Some(entry) = entry.as_object() else {
                continue;
            };
            let Some(name) = entry.get("name").and_then(serde_json::Value::as_str) else {
                continue;
            };
            let priority = entry
                .get("priority")
                .and_then(serde_json::Value::as_i64)
                .unwrap_or(i64::MAX);
            let id = NetClassId::new(name)
                .map_err(|error| KiCadImportError::Parse(error.to_string()))?;
            if class_indices.contains_key(name) {
                return Err(KiCadImportError::Parse(format!(
                    "duplicate project net class {name}"
                )));
            }
            let class = NetClass {
                id,
                parent: None,
                nets: Vec::new(),
                min_trace_width: None,
                preferred_trace_width: self.project_dimension(
                    entry,
                    "track_width",
                    &format!("net_class.{name}.preferred_trace_width"),
                )?,
                min_clearance: self.project_dimension(
                    entry,
                    "clearance",
                    &format!("net_class.{name}.min_clearance"),
                )?,
                preferred_via_land_diameter: self.project_dimension(
                    entry,
                    "via_diameter",
                    &format!("net_class.{name}.preferred_via_land_diameter"),
                )?,
                preferred_via_drill_diameter: self.project_dimension(
                    entry,
                    "via_drill",
                    &format!("net_class.{name}.preferred_via_drill_diameter"),
                )?,
                preferred_via_style: None,
                max_length: None,
                max_via_count: None,
                target_impedance_ohms: None,
                impedance_tolerance_ohms: None,
                requires_reference_plane: false,
            };
            for field in [
                "microvia_diameter",
                "microvia_drill",
                "diff_pair_width",
                "diff_pair_gap",
                "diff_pair_via_gap",
            ] {
                if entry.contains_key(field) {
                    self.omissions
                        .push(KiCadImportOmission::UnsupportedProjectNetClassField {
                            net_class: name.into(),
                            field: field.into(),
                        });
                }
            }
            let index = rules.net_classes.len();
            class_indices.insert(name.to_owned(), index);
            priorities.insert(name.to_owned(), priority);
            rules.net_classes.push(class);
        }

        let net_ids = circuit
            .nets
            .iter()
            .map(|net| (net.id.as_str().to_owned(), net.id.clone()))
            .collect::<BTreeMap<_, _>>();
        let mut assigned = BTreeSet::new();
        if let Some(assignments) = settings
            .get("netclass_assignments")
            .and_then(serde_json::Value::as_object)
        {
            for (net_name, value) in assignments {
                let Some(net) = net_ids.get(net_name).cloned() else {
                    self.omissions.push(
                        KiCadImportOmission::UnsupportedProjectNetClassAssignment {
                            net: net_name.clone(),
                        },
                    );
                    continue;
                };
                let Some(names) = value.as_array() else {
                    self.omissions.push(
                        KiCadImportOmission::UnsupportedProjectNetClassAssignment {
                            net: net_name.clone(),
                        },
                    );
                    continue;
                };
                let mut candidates = names
                    .iter()
                    .filter_map(serde_json::Value::as_str)
                    .filter(|name| class_indices.contains_key(*name))
                    .map(str::to_owned)
                    .collect::<Vec<_>>();
                if candidates.is_empty() {
                    self.omissions.push(
                        KiCadImportOmission::UnsupportedProjectNetClassAssignment {
                            net: net_name.clone(),
                        },
                    );
                    continue;
                }
                candidates.sort_by(|left, right| {
                    priorities[left]
                        .cmp(&priorities[right])
                        .then_with(|| left.cmp(right))
                });
                if candidates.len() > 1 {
                    self.omissions
                        .push(KiCadImportOmission::CompositeProjectNetClassAssignment {
                            net: net_name.clone(),
                            net_classes: candidates.clone(),
                        });
                }
                rules.net_classes[class_indices[&candidates[0]]]
                    .nets
                    .push(net);
                assigned.insert(net_name.clone());
            }
        }

        let patterns = settings
            .get("netclass_patterns")
            .and_then(serde_json::Value::as_array);
        if let Some(patterns) = patterns {
            for pattern in patterns {
                let pattern_name = pattern
                    .get("pattern")
                    .and_then(serde_json::Value::as_str)
                    .unwrap_or("<invalid>");
                let net_class = pattern
                    .get("netclass")
                    .and_then(serde_json::Value::as_str)
                    .unwrap_or("<invalid>");
                self.omissions
                    .push(KiCadImportOmission::UnsupportedProjectNetClassPattern {
                        pattern: pattern_name.into(),
                        net_class: net_class.into(),
                    });
            }
        }
        if patterns.is_none_or(Vec::is_empty)
            && let Some(default_index) = class_indices.get("Default").copied()
        {
            for net in &circuit.nets {
                if !assigned.contains(net.id.as_str()) {
                    rules.net_classes[default_index].nets.push(net.id.clone());
                }
            }
        }
        Ok(())
    }

    fn project_dimension(
        &mut self,
        entry: &serde_json::Map<String, serde_json::Value>,
        key: &str,
        field: &str,
    ) -> Result<Option<Real>, KiCadImportError> {
        let Some(value) = entry.get(key) else {
            return Ok(None);
        };
        let Some(number) = value.as_number() else {
            return Err(KiCadImportError::InvalidNumber {
                field: field.into(),
                token: value.to_string(),
            });
        };
        self.number_token(&number.to_string(), field).map(Some)
    }

    fn parse_design_rules(
        &mut self,
        circuit: &Circuit,
        rules: &mut PcbDesignRules,
    ) -> Result<(), KiCadImportError> {
        let Some(nodes) = self.design_rules.clone() else {
            self.omissions.push(KiCadImportOmission::BoardRules);
            return Ok(());
        };
        if !nodes
            .iter()
            .any(|node| node.list_name() == Some("version") && node.atom_at(1) == Some("1"))
        {
            return Err(KiCadImportError::Parse(
                "unsupported KiCad design-rule version".into(),
            ));
        }
        let net_ids = circuit
            .nets
            .iter()
            .map(|net| (net.id.as_str().to_owned(), net.id.clone()))
            .collect::<BTreeMap<_, _>>();
        for node in nodes
            .into_iter()
            .filter(|node| node.list_name() == Some("rule"))
        {
            let rule_name = node.atom_at(1).unwrap_or("<unnamed>").to_owned();
            let Some(class_name) = rule_name.strip_prefix("hypercircuit.netclass.") else {
                self.omissions
                    .push(KiCadImportOmission::UnsupportedDesignRule { rule: rule_name });
                continue;
            };
            let Some(condition) = node
                .named_child("condition")
                .and_then(|condition| condition.atom_at(1))
            else {
                self.omissions
                    .push(KiCadImportOmission::UnsupportedDesignRule { rule: rule_name });
                continue;
            };
            let Some(net_names) = parse_generated_net_condition(condition) else {
                self.omissions
                    .push(KiCadImportOmission::UnsupportedDesignRule { rule: rule_name });
                continue;
            };
            let nets = net_names
                .iter()
                .filter_map(|name| net_ids.get(name).cloned())
                .collect::<Vec<_>>();
            if nets.len() != net_names.len() {
                self.omissions
                    .push(KiCadImportOmission::UnsupportedDesignRule { rule: rule_name });
                continue;
            }
            let id = NetClassId::new(class_name)
                .map_err(|error| KiCadImportError::Parse(error.to_string()))?;
            let mut class = NetClass {
                id,
                parent: None,
                nets,
                min_trace_width: None,
                preferred_trace_width: None,
                min_clearance: None,
                preferred_via_land_diameter: None,
                preferred_via_drill_diameter: None,
                preferred_via_style: None,
                max_length: None,
                max_via_count: None,
                target_impedance_ohms: None,
                impedance_tolerance_ohms: None,
                requires_reference_plane: false,
            };
            let mut unsupported = false;
            for constraint in node.named_children("constraint") {
                match constraint.atom_at(1) {
                    Some("track_width") => {
                        class.min_trace_width = Some(self.rule_dimension(
                            constraint,
                            "min",
                            &format!("net_class.{class_name}.min_trace_width"),
                        )?);
                    }
                    Some("clearance") => {
                        class.min_clearance = Some(self.rule_dimension(
                            constraint,
                            "min",
                            &format!("net_class.{class_name}.min_clearance"),
                        )?);
                    }
                    Some("length") => {
                        class.max_length = Some(self.rule_dimension(
                            constraint,
                            "max",
                            &format!("net_class.{class_name}.max_length"),
                        )?);
                    }
                    Some("via_count") => {
                        class.max_via_count = constraint
                            .named_child("max")
                            .and_then(|maximum| maximum.atom_at(1))
                            .and_then(|value| value.parse::<usize>().ok());
                        unsupported |= class.max_via_count.is_none();
                    }
                    _ => unsupported = true,
                }
            }
            if unsupported {
                self.omissions
                    .push(KiCadImportOmission::UnsupportedDesignRule {
                        rule: rule_name.clone(),
                    });
            }
            if let Some(existing) = rules
                .net_classes
                .iter_mut()
                .find(|existing| existing.id == class.id)
            {
                if existing.nets.is_empty() {
                    existing.nets = class.nets;
                } else if existing.nets != class.nets {
                    self.omissions
                        .push(KiCadImportOmission::UnsupportedDesignRule {
                            rule: rule_name.clone(),
                        });
                }
                if class.min_trace_width.is_some() {
                    existing.min_trace_width = class.min_trace_width;
                }
                if class.min_clearance.is_some() {
                    existing.min_clearance = class.min_clearance;
                }
                if class.max_length.is_some() {
                    existing.max_length = class.max_length;
                }
                if class.max_via_count.is_some() {
                    existing.max_via_count = class.max_via_count;
                }
            } else {
                rules.net_classes.push(class);
            }
        }
        Ok(())
    }

    fn rule_dimension(
        &mut self,
        node: &sexp::Sexp,
        bound: &str,
        field: &str,
    ) -> Result<Real, KiCadImportError> {
        let token = node
            .named_child(bound)
            .and_then(|bound| bound.atom_at(1))
            .ok_or_else(|| KiCadImportError::Parse(format!("missing {field}")))?;
        let Some(decimal) = token.strip_suffix("mm") else {
            return Err(KiCadImportError::InvalidNumber {
                field: field.into(),
                token: token.into(),
            });
        };
        self.number_token(decimal, field)
    }

    fn parse_outline(&mut self) -> Result<BoardOutline, KiCadImportError> {
        let primitives = self.root.children().to_vec();
        let mut edges = Vec::new();
        for (index, primitive) in primitives.iter().enumerate() {
            let on_edge = primitive
                .named_child("layer")
                .and_then(|layer| layer.atom_at(1))
                == Some("Edge.Cuts");
            if !on_edge {
                continue;
            }
            match primitive.list_name() {
                Some("gr_line") => {
                    let start =
                        self.point_child(primitive, "start", &format!("outline[{index}].start"))?;
                    let end =
                        self.point_child(primitive, "end", &format!("outline[{index}].end"))?;
                    edges.push(BoardContourSegment::Line(LinePathSegment::new(start, end)));
                }
                Some("gr_arc") => {
                    let start =
                        self.point_child(primitive, "start", &format!("outline[{index}].start"))?;
                    let mid =
                        self.point_child(primitive, "mid", &format!("outline[{index}].mid"))?;
                    let end =
                        self.point_child(primitive, "end", &format!("outline[{index}].end"))?;
                    let Some(arc) = exact_arc_through(start, mid, end) else {
                        self.omissions
                            .push(KiCadImportOmission::UnsupportedEdgePrimitive {
                                primitive: "gr_arc-invalid".into(),
                            });
                        continue;
                    };
                    edges.push(BoardContourSegment::CircularArc(arc));
                }
                _ => {
                    self.omissions
                        .push(KiCadImportOmission::UnsupportedEdgePrimitive {
                            primitive: primitive.list_name().unwrap_or("unknown").into(),
                        });
                }
            }
        }
        let mut contours = stitch_board_contours(edges);
        if contours.is_empty() {
            return Err(KiCadImportError::MissingBoardOutline);
        }
        contours.sort_by(|left, right| {
            contour_anchor_area(right)
                .partial_cmp(&contour_anchor_area(left))
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let exterior = contours.remove(0);
        Ok(BoardOutline {
            exterior,
            cutouts: contours,
        })
    }

    fn parse_routes(&mut self, layout: &mut PcbLayout) -> Result<(), KiCadImportError> {
        let nodes = self
            .root
            .named_children("segment")
            .cloned()
            .collect::<Vec<_>>();
        for (index, node) in nodes.iter().enumerate() {
            let Some(net) = self.net_for(node) else {
                self.omissions.push(KiCadImportOmission::UnconnectedCopper {
                    feature: format!("segment[{index}]"),
                });
                continue;
            };
            let layer = self.layer_for(node)?;
            let start = self.point_child(node, "start", &format!("route[{index}].start"))?;
            let end = self.point_child(node, "end", &format!("route[{index}].end"))?;
            let width = self.number_child(node, "width", &format!("route[{index}].width"))?;
            layout.routes.push(PcbRoute {
                id: RouteId::new(format!("kicad-segment-{index}"))
                    .expect("generated route id is nonempty"),
                net,
                layer,
                width,
                segments: vec![LinePathSegment::new(start, end).into()],
            });
        }
        let arc_nodes = self.root.named_children("arc").cloned().collect::<Vec<_>>();
        for (index, node) in arc_nodes.iter().enumerate() {
            let Some(net) = self.net_for(node) else {
                self.omissions.push(KiCadImportOmission::UnconnectedCopper {
                    feature: format!("arc[{index}]"),
                });
                continue;
            };
            let layer = self.layer_for(node)?;
            let start = self.point_child(node, "start", &format!("arc[{index}].start"))?;
            let mid = self.point_child(node, "mid", &format!("arc[{index}].mid"))?;
            let end = self.point_child(node, "end", &format!("arc[{index}].end"))?;
            let width = self.number_child(node, "width", &format!("arc[{index}].width"))?;
            let Some(arc) = exact_arc_through(start, mid, end) else {
                self.omissions
                    .push(KiCadImportOmission::UnsupportedRouteArc { index });
                continue;
            };
            layout.routes.push(PcbRoute {
                id: RouteId::new(format!("kicad-arc-{index}"))
                    .expect("generated route id is nonempty"),
                net,
                layer,
                width,
                segments: vec![arc.into()],
            });
        }
        let generated_count = nodes.len() + arc_nodes.len();
        if generated_count != 0 {
            self.omissions
                .push(KiCadImportOmission::GeneratedFeatureIds {
                    feature: "route".into(),
                    count: generated_count,
                });
        }
        Ok(())
    }

    fn parse_vias(&mut self, layout: &mut PcbLayout) -> Result<(), KiCadImportError> {
        let nodes = self.root.named_children("via").cloned().collect::<Vec<_>>();
        for (index, node) in nodes.iter().enumerate() {
            let Some(net) = self.net_for(node) else {
                self.omissions.push(KiCadImportOmission::UnconnectedCopper {
                    feature: format!("via[{index}]"),
                });
                continue;
            };
            let center = self.point_child(node, "at", &format!("via[{index}].at"))?;
            let land_diameter = self.number_child(node, "size", &format!("via[{index}].size"))?;
            let drill_diameter =
                self.number_child(node, "drill", &format!("via[{index}].drill"))?;
            let layer_names = atoms_after_name(node.named_child("layers"));
            let start_layer =
                self.layer_named(layer_names.first().map(String::as_str).unwrap_or("F.Cu"))?;
            let end_layer =
                self.layer_named(layer_names.last().map(String::as_str).unwrap_or("B.Cu"))?;
            layout.vias.push(PcbVia {
                id: ViaId::new(format!("kicad-via-{index}")).expect("generated via id is nonempty"),
                net,
                start_layer,
                end_layer,
                center,
                land_diameter,
                drill_diameter,
                plating: Plating::Plated,
                mask: crate::ViaMaskIntent::default(),
            });
        }
        if !nodes.is_empty() {
            self.omissions
                .push(KiCadImportOmission::GeneratedFeatureIds {
                    feature: "via".into(),
                    count: nodes.len(),
                });
            self.omissions
                .push(KiCadImportOmission::ViaMaskIntentUnavailable { count: nodes.len() });
        }
        Ok(())
    }

    fn parse_zones(&mut self, layout: &mut PcbLayout) -> Result<(), KiCadImportError> {
        let nodes = self
            .root
            .named_children("zone")
            .cloned()
            .collect::<Vec<_>>();
        let mut imported = 0;
        for (index, node) in nodes.iter().enumerate() {
            if node.named_child("keepout").is_some() {
                self.omissions.push(KiCadImportOmission::KeepoutZone);
                continue;
            }
            let Some(net) = self.net_for(node) else {
                self.omissions.push(KiCadImportOmission::UnconnectedCopper {
                    feature: format!("zone[{index}]"),
                });
                continue;
            };
            let layer = self.layer_for(node)?;
            for polygon in node.named_children("polygon") {
                let Some(points) = polygon.named_child("pts") else {
                    continue;
                };
                let boundary = points
                    .named_children("xy")
                    .enumerate()
                    .map(|(point_index, point)| {
                        self.point_atoms(point, &format!("zone[{index}].point[{point_index}]"))
                    })
                    .collect::<Result<Vec<_>, _>>()?;
                if boundary.len() < 3 {
                    continue;
                }
                let zone_id = ZoneId::new(format!("kicad-zone-{index}-{imported}"))
                    .expect("generated zone id is nonempty");
                self.omissions
                    .push(KiCadImportOmission::ZonePolicyDefaulted {
                        zone: zone_id.as_str().into(),
                    });
                layout.zones.push(CopperZone {
                    id: zone_id,
                    net: net.clone(),
                    layer,
                    boundary,
                    clearance: Real::zero(),
                    fill: crate::CopperZoneFill::Solid,
                    connection: crate::CopperZoneConnection::Solid,
                    islands: crate::CopperZoneIslandPolicy::retain_all(),
                    stitching: None,
                    priority: 0,
                });
                imported += 1;
            }
        }
        if imported > 0 {
            self.omissions
                .push(KiCadImportOmission::GeneratedFeatureIds {
                    feature: "zone".into(),
                    count: imported,
                });
        }
        Ok(())
    }

    fn parse_footprints(
        &mut self,
        circuit: &mut Circuit,
        layout: &mut PcbLayout,
    ) -> Result<(), KiCadImportError> {
        let nodes = self
            .root
            .named_children("footprint")
            .cloned()
            .collect::<Vec<_>>();
        let mut used_instances = BTreeSet::new();
        for (index, node) in nodes.iter().enumerate() {
            let library_name = node.atom_at(1).unwrap_or("kicad-footprint");
            let reference = node
                .named_children("property")
                .find(|property| property.atom_at(1) == Some("Reference"))
                .and_then(|property| property.atom_at(2))
                .unwrap_or("U?");
            let instance_name = unique_text(reference, &mut used_instances, index);
            let instance_id = CircuitInstanceId::new(instance_name.clone())
                .expect("generated instance id is nonempty");
            let position = self.point_child(node, "at", &format!("footprint[{index}].at"))?;
            let rotation = node
                .named_child("at")
                .and_then(|at| at.atom_at(3))
                .map(|token| self.number_token(token, &format!("footprint[{index}].rotation")))
                .transpose()?
                .unwrap_or_else(Real::zero);
            let side = if node
                .named_child("layer")
                .and_then(|layer| layer.atom_at(1))
                .is_some_and(|layer| layer.starts_with("B."))
            {
                BoardSide::Back
            } else {
                BoardSide::Front
            };

            let (mut pattern, device_pins, pin_bindings) =
                self.parse_land_pattern(node, index, library_name)?;
            if let Some(existing) = layout
                .land_patterns
                .iter()
                .find(|existing| existing.id == pattern.id)
            {
                if existing != &pattern {
                    self.omissions
                        .push(KiCadImportOmission::DivergentLandPattern {
                            footprint: library_name.into(),
                        });
                    pattern.id =
                        LandPatternId::new(format!("{}@{instance_name}", pattern.id.as_str()))
                            .expect("generated divergent land-pattern id is nonempty");
                    layout.land_patterns.push(pattern.clone());
                }
            } else {
                layout.land_patterns.push(pattern.clone());
            }
            let model_id = DeviceModelId::new(pattern.id.as_str())
                .expect("land-pattern id is a valid model id");
            if !circuit
                .device_models
                .iter()
                .any(|model| model.id == model_id)
            {
                circuit.device_models.push(DeviceModel {
                    id: model_id.clone(),
                    kind: DeviceModelKind::Custom("kicad-footprint".into()),
                    pins: device_pins,
                    parameters: Vec::new(),
                });
            }
            circuit.instances.push(CircuitInstance {
                id: instance_id.clone(),
                component: ComponentId::new(instance_name)
                    .expect("generated component id is nonempty"),
                part: PartRef::new(library_name).ok(),
                model: model_id,
                pins: pin_bindings,
                parameters: Vec::new(),
            });
            layout.placements.push(PcbPlacement {
                instance: instance_id,
                land_pattern: pattern.id,
                position,
                rotation_degrees: rotation,
                side,
            });
            if node.children().iter().any(|child| {
                matches!(
                    child.list_name(),
                    Some("fp_line" | "fp_arc" | "fp_rect" | "fp_poly" | "fp_text")
                )
            }) {
                self.omissions.push(KiCadImportOmission::FootprintGraphics {
                    footprint: library_name.into(),
                });
            }
        }
        Ok(())
    }

    fn parse_land_pattern(
        &mut self,
        footprint: &sexp::Sexp,
        footprint_index: usize,
        library_name: &str,
    ) -> Result<(LandPattern, Vec<DevicePin>, Vec<PinBinding>), KiCadImportError> {
        let mut pads = Vec::new();
        let mut pin_map = Vec::new();
        let mut device_pins = Vec::new();
        let mut pin_bindings = Vec::new();
        let mut used_pad_ids = BTreeSet::new();
        let mut seen_pins = BTreeSet::new();
        for (pad_index, pad) in footprint.named_children("pad").enumerate() {
            let logical_pin = pad.atom_at(1).filter(|pin| !pin.is_empty()).unwrap_or("~");
            let pin = PinRef::new(logical_pin).expect("generated pad pin is nonempty");
            let pad_name = unique_text(logical_pin, &mut used_pad_ids, pad_index);
            let pad_id = PadId::new(pad_name).expect("generated pad id is nonempty");
            let center = pad
                .named_child("at")
                .map(|at| {
                    self.point_atoms(
                        at,
                        &format!("footprint[{footprint_index}].pad[{pad_index}].at"),
                    )
                })
                .transpose()?
                .unwrap_or_else(Point2::origin);
            let rotation_degrees = pad
                .named_child("at")
                .and_then(|at| at.atom_at(3))
                .map(|token| {
                    self.number_token(
                        token,
                        &format!("footprint[{footprint_index}].pad[{pad_index}].rotation_degrees"),
                    )
                })
                .transpose()?
                .unwrap_or_else(Real::zero);
            let size = pad
                .named_child("size")
                .ok_or_else(|| KiCadImportError::Parse("pad is missing size".into()))?;
            let width = self.number_atom(
                size,
                1,
                &format!("footprint[{footprint_index}].pad[{pad_index}].width"),
            )?;
            let height = self.number_atom(
                size,
                2,
                &format!("footprint[{footprint_index}].pad[{pad_index}].height"),
            )?;
            let shape_name = pad.atom_at(3).unwrap_or("rect");
            let shape = match shape_name {
                "circle" => PadShape::Circle {
                    diameter: max_real(&width, &height),
                },
                "rect" => PadShape::Rectangle {
                    width: width.clone(),
                    height: height.clone(),
                },
                "oval" => PadShape::Obround {
                    width: width.clone(),
                    height: height.clone(),
                },
                "roundrect" => {
                    let ratio = pad
                        .named_child("roundrect_rratio")
                        .and_then(|ratio| ratio.atom_at(1))
                        .map(|token| {
                            self.number_token(
                                token,
                                &format!(
                                    "footprint[{footprint_index}].pad[{pad_index}].roundrect_rratio"
                                ),
                            )
                        })
                        .transpose()?
                        .unwrap_or_else(|| (Real::one() / Real::from(4)).expect("four is nonzero"));
                    PadShape::RoundedRectangle {
                        width: width.clone(),
                        height: height.clone(),
                        corner_radius: min_real(&width, &height) * ratio,
                    }
                }
                other => {
                    self.omissions
                        .push(KiCadImportOmission::UnsupportedPadShape {
                            footprint: library_name.into(),
                            pad: logical_pin.into(),
                            shape: other.into(),
                        });
                    PadShape::Rectangle {
                        width: width.clone(),
                        height: height.clone(),
                    }
                }
            };
            let layer_names = atoms_after_name(pad.named_child("layers"));
            let copper_layers = self.expand_layers(&layer_names)?;
            let drill = self.pad_drill(pad, footprint_index, pad_index)?;
            let plating = match (drill.is_some(), pad.atom_at(2)) {
                (true, Some("np_thru_hole")) => Plating::NonPlated,
                (true, _) => Plating::Plated,
                (false, _) => Plating::Unspecified,
            };
            pads.push(LandPatternPad {
                id: pad_id.clone(),
                center,
                rotation_degrees,
                copper_layers,
                shape,
                drill,
                plating,
                solder_mask_margin: None,
                paste_margin: None,
            });
            pin_map.push(PadPinMap {
                pin: pin.clone(),
                pad: pad_id,
            });
            if seen_pins.insert(pin.clone()) {
                device_pins.push(DevicePin {
                    pin: pin.clone(),
                    kind: PinElectricalKind::Passive,
                    optional: true,
                });
                if let Some(net) = self.net_for(pad) {
                    pin_bindings.push(PinBinding { pin, net });
                }
            }
        }
        Ok((
            LandPattern {
                id: LandPatternId::new(library_name)
                    .expect("KiCad footprint library name is nonempty"),
                pads,
                pin_map,
                graphics: Vec::new(),
                body: None,
                models: footprint
                    .named_children("model")
                    .enumerate()
                    .map(|(index, model)| {
                        self.model_reference(
                            model,
                            &format!("footprint[{footprint_index}].model[{index}]"),
                        )
                    })
                    .collect::<Result<Vec<_>, _>>()?,
            },
            device_pins,
            pin_bindings,
        ))
    }

    fn net_for(&self, node: &sexp::Sexp) -> Option<NetId> {
        let net = node.named_child("net")?;
        net.i32_at(1)
            .and_then(|code| self.nets.get(&code).cloned())
            .or_else(|| {
                net.atom_at(2)
                    .filter(|name| !name.is_empty())
                    .and_then(|name| NetId::new(name).ok())
            })
    }

    fn layer_for(&self, node: &sexp::Sexp) -> Result<TraceLayer, KiCadImportError> {
        let name = node
            .named_child("layer")
            .and_then(|layer| layer.atom_at(1))
            .ok_or_else(|| KiCadImportError::UnknownCopperLayer("missing".into()))?;
        self.layer_named(name)
    }

    fn layer_named(&self, name: &str) -> Result<TraceLayer, KiCadImportError> {
        self.layers
            .get(name)
            .copied()
            .ok_or_else(|| KiCadImportError::UnknownCopperLayer(name.into()))
    }

    fn expand_layers(&self, names: &[String]) -> Result<Vec<TraceLayer>, KiCadImportError> {
        let mut layers = Vec::new();
        for name in names {
            if name == "*.Cu" {
                layers.extend(self.layers.values().copied());
            } else if name.ends_with(".Cu") {
                layers.push(self.layer_named(name)?);
            }
        }
        layers.sort_unstable();
        layers.dedup();
        Ok(layers)
    }

    fn point_child(
        &mut self,
        node: &sexp::Sexp,
        name: &str,
        field: &str,
    ) -> Result<Point2, KiCadImportError> {
        let child = node
            .named_child(name)
            .ok_or_else(|| KiCadImportError::Parse(format!("missing {field}")))?;
        self.point_atoms(child, field)
    }

    fn point_atoms(&mut self, node: &sexp::Sexp, field: &str) -> Result<Point2, KiCadImportError> {
        Ok(Point2::new(
            self.number_atom(node, 1, &format!("{field}.x"))?,
            self.number_atom(node, 2, &format!("{field}.y"))?,
        ))
    }

    fn number_child(
        &mut self,
        node: &sexp::Sexp,
        name: &str,
        field: &str,
    ) -> Result<Real, KiCadImportError> {
        let child = node
            .named_child(name)
            .ok_or_else(|| KiCadImportError::Parse(format!("missing {field}")))?;
        self.number_atom(child, 1, field)
    }

    fn pad_drill(
        &mut self,
        pad: &sexp::Sexp,
        footprint_index: usize,
        pad_index: usize,
    ) -> Result<Option<DrillShape>, KiCadImportError> {
        let Some(drill) = pad.named_child("drill") else {
            return Ok(None);
        };
        let field = format!("footprint[{footprint_index}].pad[{pad_index}].drill");
        if drill.atom_at(1) == Some("oval") {
            let width = self.number_atom(drill, 2, &format!("{field}.width"))?;
            let height = self.number_atom(drill, 3, &format!("{field}.height"))?;
            let half_length = ((max_real(&width, &height) - min_real(&width, &height))
                / Real::from(2))
            .map_err(|_| KiCadImportError::Parse(format!("invalid {field}")))?;
            let cutter_width = min_real(&width, &height);
            let (start, end) = if width >= height {
                (
                    Point2::new(-half_length.clone(), Real::zero()),
                    Point2::new(half_length, Real::zero()),
                )
            } else {
                (
                    Point2::new(Real::zero(), -half_length.clone()),
                    Point2::new(Real::zero(), half_length),
                )
            };
            return Ok(Some(DrillShape::Slot {
                start,
                end,
                width: cutter_width,
            }));
        }
        let token_index = usize::from(drill.atom_at(1) == Some("rect")) + 1;
        Ok(Some(DrillShape::Round {
            diameter: self.number_atom(drill, token_index, &field)?,
        }))
    }

    fn number_atom(
        &mut self,
        node: &sexp::Sexp,
        index: usize,
        field: &str,
    ) -> Result<Real, KiCadImportError> {
        let token = node
            .atom_at(index)
            .ok_or_else(|| KiCadImportError::Parse(format!("missing {field}")))?;
        self.number_token(token, field)
    }

    fn number_token(&mut self, token: &str, field: &str) -> Result<Real, KiCadImportError> {
        let value = parse_decimal(token).ok_or_else(|| KiCadImportError::InvalidNumber {
            field: field.into(),
            token: token.into(),
        })?;
        self.numeric_imports.push(KiCadNumericImport {
            field: field.into(),
            source: token.into(),
            exact: value.to_string(),
        });
        Ok(value)
    }

    fn model_reference(
        &mut self,
        model: &sexp::Sexp,
        field: &str,
    ) -> Result<Pcb3dModelReference, KiCadImportError> {
        let uri = model
            .atom_at(1)
            .filter(|uri| !uri.trim().is_empty())
            .ok_or_else(|| KiCadImportError::Parse(format!("{field} has no URI")))?
            .to_owned();
        let offset = self.model_xyz(model, "offset", Real::zero(), &format!("{field}.offset"))?;
        let scale = self.model_xyz(model, "scale", Real::one(), &format!("{field}.scale"))?;
        let rotation = self.model_xyz(model, "rotate", Real::zero(), &format!("{field}.rotate"))?;
        Ok(Pcb3dModelReference {
            format: crate::layout::pcb_3d_model_format_from_uri(&uri),
            uri,
            transform: Pcb3dModelTransform {
                offset_x: offset[0].clone(),
                offset_y: offset[1].clone(),
                offset_z: offset[2].clone(),
                rotate_x_degrees: rotation[0].clone(),
                rotate_y_degrees: rotation[1].clone(),
                rotate_z_degrees: rotation[2].clone(),
                scale_x: scale[0].clone(),
                scale_y: scale[1].clone(),
                scale_z: scale[2].clone(),
            },
        })
    }

    fn model_xyz(
        &mut self,
        model: &sexp::Sexp,
        group: &str,
        default: Real,
        field: &str,
    ) -> Result<[Real; 3], KiCadImportError> {
        let Some(xyz) = model
            .named_child(group)
            .and_then(|group| group.named_child("xyz"))
        else {
            return Ok([default.clone(), default.clone(), default]);
        };
        Ok([
            self.number_atom(xyz, 1, &format!("{field}.x"))?,
            self.number_atom(xyz, 2, &format!("{field}.y"))?,
            self.number_atom(xyz, 3, &format!("{field}.z"))?,
        ])
    }
}

fn atoms_after_name(node: Option<&sexp::Sexp>) -> Vec<String> {
    node.into_iter()
        .flat_map(|node| node.children().iter().skip(1))
        .filter_map(|atom| atom.as_atom().map(str::to_owned))
        .collect()
}

fn parse_generated_net_condition(condition: &str) -> Option<Vec<String>> {
    let names = condition
        .split(" || ")
        .map(|clause| {
            clause
                .strip_prefix("A.NetName == '")?
                .strip_suffix('\'')
                .map(str::to_owned)
        })
        .collect::<Option<Vec<_>>>()?;
    (!names.is_empty()).then_some(names)
}

fn unique_text(base: &str, used: &mut BTreeSet<String>, index: usize) -> String {
    if used.insert(base.to_owned()) {
        return base.to_owned();
    }
    let candidate = format!("{base}#{index}");
    used.insert(candidate.clone());
    candidate
}

fn max_real(left: &Real, right: &Real) -> Real {
    if left >= right {
        left.clone()
    } else {
        right.clone()
    }
}

fn min_real(left: &Real, right: &Real) -> Real {
    if left <= right {
        left.clone()
    } else {
        right.clone()
    }
}

fn stitch_board_contours(mut edges: Vec<BoardContourSegment>) -> Vec<BoardContour> {
    let mut contours = Vec::new();
    while let Some(first) = edges.pop() {
        let start = first.start().clone();
        let mut end = first.end().clone();
        let mut segments = vec![first];
        while end != start {
            let Some(index) = edges
                .iter()
                .position(|candidate| candidate.start() == &end || candidate.end() == &end)
            else {
                segments.clear();
                break;
            };
            let candidate = edges.swap_remove(index);
            let candidate = if candidate.start() == &end {
                candidate
            } else {
                reverse_board_segment(candidate)
            };
            end = candidate.end().clone();
            segments.push(candidate);
        }
        let contour = BoardContour::from_segments(segments);
        if contour.is_valid() {
            contours.push(contour);
        }
    }
    contours
}

fn reverse_board_segment(segment: BoardContourSegment) -> BoardContourSegment {
    match segment {
        BoardContourSegment::Line(line) => BoardContourSegment::Line(LinePathSegment::new(
            line.end().clone(),
            line.start().clone(),
        )),
        BoardContourSegment::CircularArc(arc) => {
            let direction = match arc.direction() {
                ArcDirection::Cw => ArcDirection::Ccw,
                ArcDirection::Ccw => ArcDirection::Cw,
            };
            BoardContourSegment::CircularArc(
                ExplicitCircularArc::new(
                    arc.center().clone(),
                    arc.radius().clone(),
                    arc.end().clone(),
                    arc.start().clone(),
                    direction,
                )
                .expect("reversing a valid arc preserves its circle"),
            )
        }
        BoardContourSegment::CubicBezier(bezier) => {
            BoardContourSegment::CubicBezier(hyperpath::CubicBezier::new(
                bezier.end().clone(),
                bezier.control1().clone(),
                bezier.control0().clone(),
                bezier.start().clone(),
            ))
        }
    }
}

fn contour_anchor_area(contour: &BoardContour) -> f64 {
    polygon_area(
        &contour
            .segments()
            .iter()
            .map(|segment| segment.start().clone())
            .collect::<Vec<_>>(),
    )
}

fn polygon_area(points: &[Point2]) -> f64 {
    let mut twice_area = 0.0;
    for index in 0..points.len() {
        let current = &points[index];
        let next = &points[(index + 1) % points.len()];
        let Some(x0) = current.x.to_f64_lossy() else {
            return 0.0;
        };
        let Some(y0) = current.y.to_f64_lossy() else {
            return 0.0;
        };
        let Some(x1) = next.x.to_f64_lossy() else {
            return 0.0;
        };
        let Some(y1) = next.y.to_f64_lossy() else {
            return 0.0;
        };
        twice_area += x0 * y1 - x1 * y0;
    }
    twice_area.abs() / 2.0
}

fn parse_decimal(token: &str) -> Option<Real> {
    let (mantissa, exponent) = if let Some((mantissa, exponent)) = token.split_once(['e', 'E']) {
        (mantissa, exponent.parse::<i32>().ok()?)
    } else {
        (token, 0_i32)
    };
    let negative = mantissa.starts_with('-');
    let mantissa = mantissa.strip_prefix(['-', '+']).unwrap_or(mantissa);
    let (whole, fraction) = mantissa.split_once('.').unwrap_or((mantissa, ""));
    if whole.is_empty() && fraction.is_empty() {
        return None;
    }
    let digits = format!("{whole}{fraction}");
    if !digits.chars().all(|character| character.is_ascii_digit()) {
        return None;
    }
    let mut numerator = digits.parse::<i64>().ok()?;
    if negative {
        numerator = -numerator;
    }
    let scale = i32::try_from(fraction.len()).ok()? - exponent;
    if scale <= 0 {
        let multiplier = 10_i64.checked_pow(scale.unsigned_abs())?;
        return Some(Real::from(numerator.checked_mul(multiplier)?));
    }
    let denominator = 10_i64.checked_pow(u32::try_from(scale).ok()?)?;
    (Real::from(numerator) / Real::from(denominator)).ok()
}

fn exact_arc_through(start: Point2, mid: Point2, end: Point2) -> Option<ExplicitCircularArc> {
    let two = Real::from(2);
    let denominator = two
        * (start.x.clone() * (mid.y.clone() - end.y.clone())
            + mid.x.clone() * (end.y.clone() - start.y.clone())
            + end.x.clone() * (start.y.clone() - mid.y.clone()));
    if denominator.refine_sign_until(-128) != Some(RealSign::Positive)
        && denominator.refine_sign_until(-128) != Some(RealSign::Negative)
    {
        return None;
    }
    let squared =
        |point: &Point2| point.x.clone() * point.x.clone() + point.y.clone() * point.y.clone();
    let center_x = ((squared(&start) * (mid.y.clone() - end.y.clone())
        + squared(&mid) * (end.y.clone() - start.y.clone())
        + squared(&end) * (start.y.clone() - mid.y.clone()))
        / denominator.clone())
    .ok()?;
    let center_y = ((squared(&start) * (end.x.clone() - mid.x.clone())
        + squared(&mid) * (start.x.clone() - end.x.clone())
        + squared(&end) * (mid.x.clone() - start.x.clone()))
        / denominator)
        .ok()?;
    let center = Point2::new(center_x, center_y);
    let dx = start.x.clone() - center.x.clone();
    let dy = start.y.clone() - center.y.clone();
    let radius = (dx.clone() * dx + dy.clone() * dy).sqrt().ok()?;
    let orientation = (mid.x.clone() - start.x.clone()) * (end.y.clone() - start.y.clone())
        - (mid.y.clone() - start.y.clone()) * (end.x.clone() - start.x.clone());
    let direction = match orientation.refine_sign_until(-128)? {
        RealSign::Positive => ArcDirection::Ccw,
        RealSign::Negative => ArcDirection::Cw,
        RealSign::Zero => return None,
    };
    ExplicitCircularArc::new(center, radius, start, end, direction).ok()
}
