//! Typed readiness inputs from native PCB authoring systems.
//!
//! Parsed KiCad/Gerber models remain useful release evidence, but native CAD
//! callers can retain exact routed-slot, keepout, courtyard, and package-body
//! intent that those formats sometimes flatten. These carriers let authoring
//! crates hand that intent to HyperDRC without teaching geometry engines about
//! PCB semantics.

use crate::checks::intersection_for_check;
use crate::geometry::multipolygon_to_shapes_scalar;
use crate::kicad::{BoardModel, CopperKind};
use crate::report::{FindingSubject, Severity, Violation};
use crate::{PcbRegion, PcbRegionExt, Scalar};

/// Refined electrical kind supplied by a native authoring system.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AuthoredNetKind {
    /// Unrefined electrical node.
    Generic,
    /// Supply rail with a source contract.
    PowerSupply,
    /// Ground or return net.
    Ground,
    /// Routed digital signal.
    DigitalSignal,
    /// Routed analog signal.
    AnalogSignal,
    /// One differential-pair member.
    DifferentialPairMember,
    /// Versioned authoring extension.
    Extension {
        /// Extension namespace.
        namespace: String,
        /// Stable kind name.
        name: String,
        /// Extension catalog version.
        version: String,
    },
}

/// Native authored net semantics available to role-aware DRC checks.
#[derive(Clone, Debug, PartialEq)]
pub struct AuthoredNetIntent {
    /// Stable logical net identity.
    pub net: String,
    /// Refined electrical kind.
    pub kind: AuthoredNetKind,
    /// Optional exact nominal voltage in volts.
    pub nominal_voltage: Option<Scalar>,
    /// Semantic source subject for diagnostics.
    pub subject: FindingSubject,
}

/// Curated functional role supplied by a native authoring system.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AuthoredFunctionalRoleKind {
    /// Passive or active low-pass filter block.
    LowpassFilter,
    /// Passive or active high-pass filter block.
    HighpassFilter,
    /// Passive or active band-pass filter block.
    BandpassFilter,
    /// Conducted-emissions filter block.
    EmiFilter,
    /// Buck regulator block.
    BuckRegulator,
    /// Linear regulator block.
    LdoRegulator,
    /// Resistive voltage-divider block.
    VoltageDivider,
    /// Crystal oscillator block.
    CrystalOscillator,
    /// Operational-amplifier stage.
    OpAmpStage,
    /// Supply-pin decoupling capacitor.
    DecouplingCapacitor,
    /// Rail bulk-energy capacitor.
    BulkCapacitor,
    /// Kelvin-aware current-sense resistor.
    CurrentSenseResistor,
    /// Pull-up resistor.
    PullupResistor,
    /// Pull-down resistor.
    PulldownResistor,
    /// Transmission-line termination resistor.
    TerminationResistor,
    /// Versioned authoring extension.
    Extension {
        /// Extension namespace.
        namespace: String,
        /// Stable role name.
        name: String,
        /// Extension catalog version.
        version: String,
    },
}

/// One physical endpoint participating in an authored function contract.
#[derive(Clone, Debug, PartialEq)]
pub struct AuthoredRoleEndpoint {
    /// Contract-local endpoint name.
    pub name: String,
    /// Optional logical net identity.
    pub net: Option<String>,
    /// Exact board-space anchors for pads, vias, pins, or component origins.
    pub locations: Vec<[Scalar; 2]>,
    /// Optional narrower semantic subject.
    pub subject: Option<FindingSubject>,
}

/// Native authored function contract and physical placement evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct AuthoredFunctionalRole {
    /// Component or subcircuit implementing the role.
    pub subject: FindingSubject,
    /// Curated or extension role.
    pub role: AuthoredFunctionalRoleKind,
    /// Named physical/electrical endpoints.
    pub endpoints: Vec<AuthoredRoleEndpoint>,
    /// Optional exact maximum endpoint separation in millimeters.
    pub maximum_distance: Option<Scalar>,
}

/// Run physical checks that depend on authoritative native functional intent.
///
/// Imported Gerber/KiCad designs can continue using name-based heuristic checks;
/// this path treats missing or violated native contract evidence as a
/// source-addressable release error.
pub fn authored_functional_role_readiness(roles: &[AuthoredFunctionalRole]) -> Vec<Violation> {
    let mut violations = Vec::new();
    for role in roles {
        let required: &[(&str, &str)] = match role.role {
            AuthoredFunctionalRoleKind::DecouplingCapacitor => {
                &[("supply", "target"), ("reference", "target")]
            }
            AuthoredFunctionalRoleKind::CurrentSenseResistor => &[
                ("sense-positive", "line-input"),
                ("sense-negative", "line-output"),
            ],
            AuthoredFunctionalRoleKind::TerminationResistor => &[("component", "target")],
            _ => &[],
        };
        for (left_name, right_name) in required {
            let left = endpoint(role, left_name);
            let right = endpoint(role, right_name);
            let mut subjects = vec![role.subject.clone()];
            subjects.extend(
                left.into_iter()
                    .chain(right)
                    .filter_map(|endpoint| endpoint.subject.clone()),
            );
            let Some(left) = left else {
                violations.push(missing_role_endpoint(role, left_name, subjects));
                continue;
            };
            let Some(right) = right else {
                violations.push(missing_role_endpoint(role, right_name, subjects));
                continue;
            };
            if left.locations.is_empty() || right.locations.is_empty() {
                violations.push(
                    Violation::new(
                        "authored-functional-role-readiness",
                        Severity::Error,
                        vec!["authoring-intent".into()],
                        None,
                        Vec::new(),
                        Vec::new(),
                        Some(format!(
                            "authored {:?} role {} has no physical location evidence for {} or {}",
                            role.role, role.subject.id, left_name, right_name
                        )),
                    )
                    .with_subjects(subjects),
                );
                continue;
            }
            let Some(maximum) = &role.maximum_distance else {
                violations.push(
                    Violation::new(
                        "authored-functional-role-readiness",
                        Severity::Error,
                        vec!["authoring-intent".into()],
                        None,
                        Vec::new(),
                        Vec::new(),
                        Some(format!(
                            "authored {:?} role {} has no maximum physical separation",
                            role.role, role.subject.id
                        )),
                    )
                    .with_subjects(subjects),
                );
                continue;
            };
            if crate::scalar::le(maximum, &Scalar::zero())
                || !locations_within(&left.locations, &right.locations, maximum)
            {
                let locations = left
                    .locations
                    .iter()
                    .chain(&right.locations)
                    .filter_map(|point| Some([point[0].to_f64_lossy()?, point[1].to_f64_lossy()?]))
                    .collect();
                violations.push(
                    Violation::new(
                        "authored-functional-role-readiness",
                        Severity::Error,
                        vec!["authoring-intent".into()],
                        None,
                        Vec::new(),
                        locations,
                        Some(format!(
                            "authored {:?} role {} endpoints {} and {} exceed maximum separation {maximum:#.6}",
                            role.role, role.subject.id, left_name, right_name
                        )),
                    )
                    .with_subjects(subjects),
                );
            }
        }
    }
    violations
}

fn endpoint<'a>(role: &'a AuthoredFunctionalRole, name: &str) -> Option<&'a AuthoredRoleEndpoint> {
    role.endpoints.iter().find(|endpoint| endpoint.name == name)
}

fn missing_role_endpoint(
    role: &AuthoredFunctionalRole,
    endpoint: &str,
    subjects: Vec<FindingSubject>,
) -> Violation {
    Violation::new(
        "authored-functional-role-readiness",
        Severity::Error,
        vec!["authoring-intent".into()],
        None,
        Vec::new(),
        Vec::new(),
        Some(format!(
            "authored {:?} role {} is missing required endpoint {endpoint}",
            role.role, role.subject.id
        )),
    )
    .with_subjects(subjects)
}

fn locations_within(left: &[[Scalar; 2]], right: &[[Scalar; 2]], maximum: &Scalar) -> bool {
    let maximum_squared = maximum * maximum;
    left.iter().any(|left| {
        right.iter().any(|right| {
            let dx = &left[0] - &right[0];
            let dy = &left[1] - &right[1];
            crate::scalar::le(&(&dx * &dx + &dy * &dy), &maximum_squared)
        })
    })
}

/// Feature family excluded by an authored PCB keepout.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AuthoredKeepoutScope {
    /// Every copper/via feature on every layer.
    All,
    /// Copper restricted to named layers.
    Copper(Vec<String>),
    /// Via copper only.
    Vias,
    /// Component placement; retained for mechanical/assembly consumers.
    Components,
}

/// Source-addressable authored keepout geometry.
#[derive(Clone, Debug)]
pub struct AuthoredKeepout {
    /// Stable source identity from the authoring model.
    pub source: String,
    /// Exact-aware keepout profile.
    pub region: PcbRegion,
    /// Excluded feature family/layers.
    pub scope: AuthoredKeepoutScope,
}

/// Source-addressable routed-slot fabrication intent.
#[derive(Clone, Debug, PartialEq)]
pub struct AuthoredRoutedSlot {
    /// Stable pad/drill source identity.
    pub source: String,
    /// Exact slot centerline start.
    pub start: [Scalar; 2],
    /// Exact slot centerline end.
    pub end: [Scalar; 2],
    /// Exact cutter/finished slot width.
    pub width: Scalar,
    /// Whether the slot wall is intended to be plated.
    pub plated: bool,
}

/// Physical board side occupied by one authored component envelope.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AuthoredComponentSide {
    /// Component mounted on the front/top side.
    Front,
    /// Component mounted on the back/bottom side.
    Back,
}

/// Authoring geometry used to certify component placement.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AuthoredComponentEnvelopeKind {
    /// Explicit footprint courtyard supplied by the authoring model.
    Courtyard,
    /// Package-body plan envelope used when no usable courtyard exists.
    Body,
}

/// Source-addressable placed component envelope.
#[derive(Clone, Debug)]
pub struct AuthoredComponentEnvelope {
    /// Stable logical instance/source identity.
    pub source: String,
    /// Board side on which the envelope is mounted.
    pub side: AuthoredComponentSide,
    /// Exact-aware board-space envelope profile.
    pub region: PcbRegion,
    /// Whether the envelope came from courtyard or body intent.
    pub kind: AuthoredComponentEnvelopeKind,
}

/// Check native keepout profiles against source-addressable board copper.
///
/// Component-only keepouts are retained but skipped because [`BoardModel`]
/// currently contains copper and drills rather than component courtyard/body
/// geometry. Copper/via intersections are release-blocking errors.
pub fn authored_keepout_readiness(
    board: &BoardModel,
    keepouts: &[AuthoredKeepout],
    minimum_report_area: &Scalar,
) -> Vec<Violation> {
    let mut violations = Vec::new();
    for keepout in keepouts {
        if keepout.scope == AuthoredKeepoutScope::Components {
            continue;
        }
        for feature in &board.copper {
            if !feature_matches_scope(feature.kind, &feature.layer, &keepout.scope) {
                continue;
            }
            let intersection = match intersection_for_check(
                &keepout.region,
                &feature.region,
                "authored-keepout-readiness",
                vec![feature.layer.clone(), format!("keepout:{}", keepout.source)],
            ) {
                Ok(intersection) => intersection,
                Err(uncertainty) => return vec![*uncertainty],
            };
            if intersection.is_empty() {
                continue;
            }
            let polygons =
                multipolygon_to_shapes_scalar(&intersection.to_multipolygon(), minimum_report_area);
            if polygons.is_empty() {
                continue;
            }
            violations.push(Violation::new(
                "authored-keepout-readiness",
                Severity::Error,
                vec![feature.layer.clone(), format!("keepout:{}", keepout.source)],
                None,
                polygons,
                vec![feature.location_f64_compatibility_required()],
                Some(format!(
                    "authored keepout {} intersects {:?} copper on {}",
                    keepout.source, feature.kind, feature.layer
                )),
            ));
        }
    }
    violations
}

/// Check exact authored routed-slot widths against cutter capability.
pub fn authored_routed_slot_readiness(
    slots: &[AuthoredRoutedSlot],
    minimum_route_width: &Scalar,
) -> Vec<Violation> {
    slots
        .iter()
        .filter_map(|slot| {
            let (severity, message) = if crate::scalar::le(&slot.width, &Scalar::zero()) {
                (
                    Severity::Error,
                    format!(
                        "authored routed slot {} has non-positive width",
                        slot.source
                    ),
                )
            } else if crate::scalar::lt(&slot.width, minimum_route_width) {
                (
                    Severity::Warning,
                    format!(
                        "authored routed slot {} width {:.6} is below minimum cutter width {:.6}",
                        slot.source, slot.width, minimum_route_width
                    ),
                )
            } else {
                return None;
            };
            Some(Violation::new(
                "authored-routed-slot-readiness",
                severity,
                vec![format!("slot:{}", slot.source)],
                None,
                Vec::new(),
                slot_midpoint(slot).into_iter().collect(),
                Some(message),
            ))
        })
        .collect()
}

/// Check envelope coverage, same-side component collisions and component keepouts.
///
/// Opposite-side envelopes may overlap in XY and are therefore not compared.
/// Component-scoped keepouts apply to both board sides. Failed exact booleans
/// produce warnings; certified material intersections produce errors.
pub fn authored_component_readiness(
    expected_sources: &[String],
    components: &[AuthoredComponentEnvelope],
    keepouts: &[AuthoredKeepout],
    minimum_report_area: &Scalar,
) -> Vec<Violation> {
    let mut violations = Vec::new();
    for source in expected_sources {
        let count = components
            .iter()
            .filter(|component| component.source == *source)
            .count();
        if count == 1 {
            continue;
        }
        violations.push(Violation::new(
            "authored-component-readiness",
            Severity::Error,
            vec![format!("component:{source}")],
            None,
            Vec::new(),
            Vec::new(),
            Some(if count == 0 {
                format!("placed component {source} has no usable courtyard or body envelope")
            } else {
                format!("placed component {source} has {count} competing envelopes")
            }),
        ));
    }
    for (left_index, left) in components.iter().enumerate() {
        for right in components.iter().skip(left_index + 1) {
            if left.side != right.side {
                continue;
            }
            if let Err(uncertainty) = append_component_intersection(
                &mut violations,
                &left.region,
                &right.region,
                minimum_report_area,
                vec![
                    format!("component:{}", left.source),
                    format!("component:{}", right.source),
                ],
                format!(
                    "component envelopes {} ({:?}) and {} ({:?}) overlap on {:?}",
                    left.source, left.kind, right.source, right.kind, left.side
                ),
            ) {
                return vec![*uncertainty];
            }
        }
    }
    for keepout in keepouts
        .iter()
        .filter(|keepout| keepout.scope == AuthoredKeepoutScope::Components)
    {
        for component in components {
            if let Err(uncertainty) = append_component_intersection(
                &mut violations,
                &keepout.region,
                &component.region,
                minimum_report_area,
                vec![
                    format!("keepout:{}", keepout.source),
                    format!("component:{}", component.source),
                ],
                format!(
                    "component envelope {} ({:?}) intersects component keepout {}",
                    component.source, component.kind, keepout.source
                ),
            ) {
                return vec![*uncertainty];
            }
        }
    }
    violations
}

fn append_component_intersection(
    violations: &mut Vec<Violation>,
    left: &PcbRegion,
    right: &PcbRegion,
    minimum_report_area: &Scalar,
    layers: Vec<String>,
    message: String,
) -> Result<(), Box<Violation>> {
    let intersection =
        intersection_for_check(left, right, "authored-component-readiness", layers.clone())?;
    if intersection.is_empty() {
        return Ok(());
    }
    let polygons =
        multipolygon_to_shapes_scalar(&intersection.to_multipolygon(), minimum_report_area);
    if polygons.is_empty() {
        return Ok(());
    }
    violations.push(Violation::new(
        "authored-component-readiness",
        Severity::Error,
        layers,
        None,
        polygons,
        Vec::new(),
        Some(message),
    ));
    Ok(())
}

fn feature_matches_scope(kind: CopperKind, layer: &str, scope: &AuthoredKeepoutScope) -> bool {
    match scope {
        AuthoredKeepoutScope::All => true,
        AuthoredKeepoutScope::Copper(layers) => {
            layers.is_empty() || layers.iter().any(|candidate| candidate == layer)
        }
        AuthoredKeepoutScope::Vias => kind == CopperKind::Via,
        AuthoredKeepoutScope::Components => false,
    }
}

fn slot_midpoint(slot: &AuthoredRoutedSlot) -> Option<[f64; 2]> {
    let start_x = slot.start[0].to_f64_lossy()?;
    let start_y = slot.start[1].to_f64_lossy()?;
    let end_x = slot.end[0].to_f64_lossy()?;
    let end_y = slot.end[1].to_f64_lossy()?;
    [(start_x + end_x) / 2.0, (start_y + end_y) / 2.0].into()
}

#[cfg(test)]
mod tests {
    use csgrs::curve;

    use super::*;
    use crate::kicad::{CopperFeature, DrillFeature};
    use crate::{LayerMetadata, scalar::scalar};

    fn square(name: &str) -> PcbRegion {
        PcbRegion::new(
            curve::rectangle(Scalar::from(2), Scalar::from(2)),
            Some(LayerMetadata { name: name.into() }),
        )
    }

    fn subject(kind: &str, id: &str) -> FindingSubject {
        FindingSubject {
            kind: kind.into(),
            id: id.into(),
            source: None,
        }
    }

    fn endpoint_at(name: &str, x: &str, y: &str) -> AuthoredRoleEndpoint {
        AuthoredRoleEndpoint {
            name: name.into(),
            net: None,
            locations: vec![[scalar(x), scalar(y)]],
            subject: Some(subject("pin", name)),
        }
    }

    #[test]
    fn authoritative_decoupling_role_checks_target_and_return_distance() {
        let compliant = AuthoredFunctionalRole {
            subject: subject("instance", "C1"),
            role: AuthoredFunctionalRoleKind::DecouplingCapacitor,
            endpoints: vec![
                endpoint_at("supply", "0", "0"),
                endpoint_at("target", "0.4", "0"),
                endpoint_at("reference", "0", "0.1"),
            ],
            maximum_distance: Some(scalar("0.5")),
        };
        assert!(authored_functional_role_readiness(std::slice::from_ref(&compliant)).is_empty());

        let mut distant = compliant;
        distant.endpoints[0].locations[0][0] = scalar("2");
        let findings = authored_functional_role_readiness(&[distant]);
        assert_eq!(findings.len(), 1);
        assert_eq!(findings[0].severity, Severity::Error);
        assert_eq!(findings[0].subjects[0].id, "C1");
        assert_eq!(findings[0].check, "authored-functional-role-readiness");
    }

    #[test]
    fn incomplete_authoritative_role_is_a_structured_error() {
        let role = AuthoredFunctionalRole {
            subject: subject("instance", "RSENSE"),
            role: AuthoredFunctionalRoleKind::CurrentSenseResistor,
            endpoints: vec![endpoint_at("line-input", "0", "0")],
            maximum_distance: Some(scalar("0.2")),
        };
        let findings = authored_functional_role_readiness(&[role]);

        assert_eq!(findings.len(), 2);
        assert!(
            findings
                .iter()
                .all(|finding| finding.subjects[0].id == "RSENSE")
        );
    }

    #[test]
    fn native_keepouts_and_slots_produce_source_addressable_readiness_findings() {
        let board = BoardModel {
            source: "native".into(),
            copper: vec![CopperFeature {
                layer: "F.Cu".into(),
                net: Some("SIGNAL".into()),
                kind: CopperKind::Segment,
                region: square("route"),
                location: [Scalar::zero(), Scalar::zero()],
            }],
            drills: Vec::<DrillFeature>::new(),
            board_outline: None,
            panel_features: None,
        };
        let keepout = AuthoredKeepout {
            source: "antenna".into(),
            region: square("keepout"),
            scope: AuthoredKeepoutScope::Copper(vec!["F.Cu".into()]),
        };
        let findings = authored_keepout_readiness(&board, &[keepout], &Scalar::zero());
        assert_eq!(findings.len(), 1);
        assert_eq!(findings[0].severity, Severity::Error);
        assert!(
            findings[0]
                .layers
                .iter()
                .any(|layer| layer == "keepout:antenna")
        );

        let slot = AuthoredRoutedSlot {
            source: "J1:mount".into(),
            start: [Scalar::zero(), Scalar::zero()],
            end: [Scalar::one(), Scalar::zero()],
            width: scalar("0.1"),
            plated: false,
        };
        let findings = authored_routed_slot_readiness(&[slot], &scalar("0.2"));
        assert_eq!(findings.len(), 1);
        assert_eq!(findings[0].severity, Severity::Warning);
    }

    #[test]
    fn native_component_envelopes_check_side_and_component_keepouts() {
        let front_a = AuthoredComponentEnvelope {
            source: "U1".into(),
            side: AuthoredComponentSide::Front,
            region: square("U1-courtyard"),
            kind: AuthoredComponentEnvelopeKind::Courtyard,
        };
        let front_b = AuthoredComponentEnvelope {
            source: "U2".into(),
            side: AuthoredComponentSide::Front,
            region: square("U2-body"),
            kind: AuthoredComponentEnvelopeKind::Body,
        };
        let back = AuthoredComponentEnvelope {
            source: "U3".into(),
            side: AuthoredComponentSide::Back,
            region: square("U3-body"),
            kind: AuthoredComponentEnvelopeKind::Body,
        };
        let findings = authored_component_readiness(
            &["U1".into(), "U2".into(), "U3".into()],
            &[front_a.clone(), front_b, back],
            &[],
            &Scalar::zero(),
        );
        assert_eq!(findings.len(), 1);
        assert_eq!(findings[0].severity, Severity::Error);
        assert!(
            findings[0]
                .layers
                .iter()
                .any(|layer| layer == "component:U1")
        );
        assert!(
            findings[0]
                .layers
                .iter()
                .any(|layer| layer == "component:U2")
        );

        let keepout = AuthoredKeepout {
            source: "antenna-body".into(),
            region: square("component-keepout"),
            scope: AuthoredKeepoutScope::Components,
        };
        let findings =
            authored_component_readiness(&["U1".into()], &[front_a], &[keepout], &Scalar::zero());
        assert_eq!(findings.len(), 1);
        assert!(
            findings[0]
                .layers
                .iter()
                .any(|layer| layer == "keepout:antenna-body")
        );

        let findings = authored_component_readiness(&["missing".into()], &[], &[], &Scalar::zero());
        assert_eq!(findings.len(), 1);
        assert_eq!(findings[0].severity, Severity::Error);
    }
}
