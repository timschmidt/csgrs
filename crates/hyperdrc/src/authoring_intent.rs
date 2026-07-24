//! Typed readiness inputs from native PCB authoring systems.
//!
//! Parsed KiCad/Gerber models remain useful release evidence, but native CAD
//! callers can retain exact routed-slot, keepout, courtyard, and package-body
//! intent that those formats sometimes flatten. These carriers let authoring
//! crates hand that intent to HyperDRC without teaching geometry engines about
//! PCB semantics.

use crate::geometry::multipolygon_to_shapes_scalar;
use crate::kicad::{BoardModel, CopperKind};
use crate::report::{Severity, Violation};
use crate::{PcbSketch, PcbSketchExt, Scalar};

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
    pub sketch: PcbSketch,
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
    pub sketch: PcbSketch,
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
            let intersection = match keepout.sketch.try_intersection(&feature.sketch) {
                Ok(intersection) => intersection,
                Err(error) => {
                    violations.push(Violation::new(
                        "authored-keepout-readiness",
                        Severity::Warning,
                        vec![feature.layer.clone(), format!("keepout:{}", keepout.source)],
                        None,
                        Vec::new(),
                        vec![feature.location_f64_compatibility_required()],
                        Some(format!(
                            "could not certify authored keepout intersection for {}: {error}",
                            keepout.source
                        )),
                    ));
                    continue;
                }
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
            let (severity, message) = if slot.width <= Scalar::zero() {
                (
                    Severity::Error,
                    format!(
                        "authored routed slot {} has non-positive width",
                        slot.source
                    ),
                )
            } else if &slot.width < minimum_route_width {
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
            append_component_intersection(
                &mut violations,
                &left.sketch,
                &right.sketch,
                minimum_report_area,
                vec![
                    format!("component:{}", left.source),
                    format!("component:{}", right.source),
                ],
                format!(
                    "component envelopes {} ({:?}) and {} ({:?}) overlap on {:?}",
                    left.source, left.kind, right.source, right.kind, left.side
                ),
            );
        }
    }
    for keepout in keepouts
        .iter()
        .filter(|keepout| keepout.scope == AuthoredKeepoutScope::Components)
    {
        for component in components {
            append_component_intersection(
                &mut violations,
                &keepout.sketch,
                &component.sketch,
                minimum_report_area,
                vec![
                    format!("keepout:{}", keepout.source),
                    format!("component:{}", component.source),
                ],
                format!(
                    "component envelope {} ({:?}) intersects component keepout {}",
                    component.source, component.kind, keepout.source
                ),
            );
        }
    }
    violations
}

fn append_component_intersection(
    violations: &mut Vec<Violation>,
    left: &PcbSketch,
    right: &PcbSketch,
    minimum_report_area: &Scalar,
    layers: Vec<String>,
    message: String,
) {
    let intersection = match left.try_intersection(right) {
        Ok(intersection) => intersection,
        Err(error) => {
            violations.push(Violation::new(
                "authored-component-readiness",
                Severity::Warning,
                layers,
                None,
                Vec::new(),
                Vec::new(),
                Some(format!("could not certify {message}: {error}")),
            ));
            return;
        }
    };
    if intersection.is_empty() {
        return;
    }
    let polygons =
        multipolygon_to_shapes_scalar(&intersection.to_multipolygon(), minimum_report_area);
    if polygons.is_empty() {
        return;
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
    use csgrs::sketch::Profile;

    use super::*;
    use crate::kicad::{CopperFeature, DrillFeature};
    use crate::{LayerMetadata, scalar::scalar};

    fn square(name: &str) -> PcbSketch {
        PcbSketch::new(
            Profile::rectangle(Scalar::from(2), Scalar::from(2)),
            Some(LayerMetadata { name: name.into() }),
        )
    }

    #[test]
    fn native_keepouts_and_slots_produce_source_addressable_readiness_findings() {
        let board = BoardModel {
            source: "native".into(),
            copper: vec![CopperFeature {
                layer: "F.Cu".into(),
                net: Some("SIGNAL".into()),
                kind: CopperKind::Segment,
                sketch: square("route"),
                location: [Scalar::zero(), Scalar::zero()],
            }],
            drills: Vec::<DrillFeature>::new(),
            board_outline: None,
            panel_features: None,
        };
        let keepout = AuthoredKeepout {
            source: "antenna".into(),
            sketch: square("keepout"),
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
            sketch: square("U1-courtyard"),
            kind: AuthoredComponentEnvelopeKind::Courtyard,
        };
        let front_b = AuthoredComponentEnvelope {
            source: "U2".into(),
            side: AuthoredComponentSide::Front,
            sketch: square("U2-body"),
            kind: AuthoredComponentEnvelopeKind::Body,
        };
        let back = AuthoredComponentEnvelope {
            source: "U3".into(),
            side: AuthoredComponentSide::Back,
            sketch: square("U3-body"),
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
            sketch: square("component-keepout"),
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
