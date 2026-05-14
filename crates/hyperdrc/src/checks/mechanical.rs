//! Mechanical board-feature readiness checks.
//!
//! These checks sit between pure drill checks and full board-context checks:
//! they need KiCad drill/copper context, but the primary design question is
//! mechanical production intent around mounting holes, panel fixtures, and
//! chassis attachment features.

use csgrs::csg::CSG;

use super::distance::polygon_boundary_distance;
use crate::LayerMetadata;
use crate::geometry::{circle_polygon, multipolygon_to_shapes, polygons_to_sketch};
use crate::kicad::{BoardModel, CopperFeature, DrillFeature};
use crate::report::{Severity, Violation};

/// Run the `mounting_hole_grounding_readiness` design-readiness check or report helper.
pub fn mounting_hole_grounding_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    grounding_distance: f64,
) -> Vec<Violation> {
    let copper = selected_copper_features(board, selected_layers);
    let grounding_features = copper
        .iter()
        .filter(|feature| {
            feature
                .net
                .as_deref()
                .is_some_and(looks_ground_or_chassis_net)
        })
        .copied()
        .collect::<Vec<_>>();
    let mut violations = Vec::new();

    for drill in likely_mounting_holes(board, grounding_distance) {
        let has_grounding_intent = grounding_features.iter().any(|feature| {
            distance(drill.location, feature.location) <= drill.diameter / 2.0 + grounding_distance
        });
        if has_grounding_intent {
            continue;
        }

        violations.push(Violation::new(
            "mounting-hole-grounding-readiness",
            Severity::Warning,
            vec![board.source.clone()],
            None,
            Vec::new(),
            vec![drill.location],
            Some(format!(
                "likely mounting hole diameter {:.6} has no parsed ground or chassis copper within {:.6}; review chassis bonding, isolation, or keepout intent",
                drill.diameter, grounding_distance
            )),
        ));
    }

    violations
}

/// Run the `mounting_hole_copper_keepout_readiness` design-readiness check or report helper.
pub fn mounting_hole_copper_keepout_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    keepout: f64,
    min_area: f64,
) -> Vec<Violation> {
    let copper = selected_copper_features(board, selected_layers);
    let mut violations = Vec::new();

    for drill in likely_mounting_holes(board, keepout) {
        let keepout_sketch = drill_keepout(drill, keepout);
        for feature in &copper {
            if feature
                .net
                .as_deref()
                .is_some_and(looks_ground_or_chassis_net)
            {
                continue;
            }

            // IPC-2221B treats mounting holes, conductive hardware, and board
            // edge/mechanical constraints as layout-clearance concerns. This is
            // a conservative geometric readiness check: model the screw/hole
            // region as a circular keepout and report nearby non-ground copper
            // so the release package makes the grounding/isolation intent clear.
            let overlap = keepout_sketch.intersection(&feature.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            let fallback_hit = shapes.is_empty()
                && polygon_boundary_distance(
                    &keepout_sketch.to_multipolygon(),
                    &feature.sketch.to_multipolygon(),
                ) <= keepout;
            if shapes.is_empty() && !fallback_hit {
                continue;
            }

            violations.push(Violation::new(
                "mounting-hole-copper-keepout-readiness",
                Severity::Warning,
                vec![feature.layer.clone()],
                None,
                shapes,
                if fallback_hit {
                    vec![drill.location, feature.location]
                } else {
                    Vec::new()
                },
                Some(format!(
                    "non-ground {:?} copper is inside likely mounting-hole keepout {:.6}; review screw, standoff, washer, or chassis clearance",
                    feature.kind, keepout
                )),
            ));
        }
    }

    violations
}

/// Run the `mounting_hole_edge_clearance_readiness` design-readiness check or report helper.
pub fn mounting_hole_edge_clearance_readiness(
    board: &BoardModel,
    edge_clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let Some(outline) = &board.board_outline else {
        return Vec::new();
    };
    let mut violations = Vec::new();

    for drill in likely_mounting_holes(board, edge_clearance) {
        let keepout_sketch = drill_keepout(drill, edge_clearance);
        let outside_outline = keepout_sketch.difference(outline);
        let shapes = multipolygon_to_shapes(&outside_outline.to_multipolygon(), min_area);
        if shapes.is_empty() {
            continue;
        }

        violations.push(Violation::new(
            "mounting-hole-edge-clearance-readiness",
            Severity::Warning,
            vec![board.source.clone()],
            None,
            shapes,
            vec![drill.location],
            Some(format!(
                "likely mounting-hole screw or washer keepout {:.6} extends beyond the board outline; review enclosure, clamp, or edge-clearance intent",
                edge_clearance
            )),
        ));
    }

    violations
}

/// Run the `mounting_hole_plating_intent_readiness` design-readiness check or report helper.
pub fn mounting_hole_plating_intent_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    grounding_distance: f64,
) -> Vec<Violation> {
    let copper = selected_copper_features(board, selected_layers);
    let grounding_features = copper
        .iter()
        .filter(|feature| {
            feature
                .net
                .as_deref()
                .is_some_and(looks_ground_or_chassis_net)
        })
        .copied()
        .collect::<Vec<_>>();
    let mut violations = Vec::new();

    for drill in likely_plated_mounting_holes(board, grounding_distance) {
        let drill_net_is_ground = drill
            .net
            .as_deref()
            .is_some_and(looks_ground_or_chassis_net);
        let has_grounding_copper = grounding_features.iter().any(|feature| {
            distance(drill.location, feature.location) <= drill.diameter / 2.0 + grounding_distance
        });
        if drill_net_is_ground || has_grounding_copper {
            continue;
        }

        violations.push(Violation::new(
            "mounting-hole-plating-intent-readiness",
            Severity::Warning,
            vec![board.source.clone()],
            None,
            Vec::new(),
            vec![drill.location],
            Some(format!(
                "large plated mounting-style hole diameter {:.6} has no parsed ground/chassis net or nearby bonding copper; review plated versus isolated hardware intent",
                drill.diameter
            )),
        ));
    }

    violations
}

/// Run the `mounting_hole_distribution_readiness` design-readiness check or report helper.
pub fn mounting_hole_distribution_readiness(
    board: &BoardModel,
    minimum_spacing: f64,
) -> Vec<Violation> {
    let holes = likely_hardware_holes(board, minimum_spacing);
    if holes.is_empty() {
        return Vec::new();
    }

    if holes.len() == 1 {
        return vec![Violation::new(
            "mounting-hole-distribution-readiness",
            Severity::Warning,
            vec![board.source.clone()],
            None,
            Vec::new(),
            vec![holes[0].location],
            Some(format!(
                "only one likely mounting hole was parsed; review enclosure support, board retention, or intentional single-fastener design"
            )),
        )];
    }

    let mut maximum_spacing = 0.0_f64;
    let mut span_locations = Vec::new();
    for left_index in 0..holes.len() {
        for right in &holes[(left_index + 1)..] {
            let left = holes[left_index];
            let spacing = distance(left.location, right.location);
            if spacing > maximum_spacing {
                maximum_spacing = spacing;
                span_locations = vec![left.location, right.location];
            }
        }
    }

    if maximum_spacing >= minimum_spacing {
        return Vec::new();
    }

    // IPC-2221B frames board mounting and hardware features as part of the
    // mechanical design envelope. A clustered set of fastener holes can be valid
    // for small modules, but it is weak release evidence for larger enclosure or
    // fixture retention, so report it as review-only readiness.
    vec![Violation::new(
        "mounting-hole-distribution-readiness",
        Severity::Warning,
        vec![board.source.clone()],
        None,
        Vec::new(),
        span_locations,
        Some(format!(
            "likely mounting holes span only {maximum_spacing:.6}, below review spacing {minimum_spacing:.6}; review enclosure support and board retention distribution"
        )),
    )]
}

/// Run the `mounting_hole_spacing_readiness` design-readiness check or report helper.
pub fn mounting_hole_spacing_readiness(
    board: &BoardModel,
    minimum_edge_spacing: f64,
) -> Vec<Violation> {
    let holes = likely_hardware_holes(board, minimum_edge_spacing * 4.0);
    let mut violations = Vec::new();

    for left_index in 0..holes.len() {
        for right in &holes[(left_index + 1)..] {
            let left = holes[left_index];
            let center_spacing = distance(left.location, right.location);
            let edge_spacing = center_spacing - (left.diameter + right.diameter) / 2.0;
            if edge_spacing >= minimum_edge_spacing {
                continue;
            }

            violations.push(Violation::new(
                "mounting-hole-spacing-readiness",
                Severity::Warning,
                vec![board.source.clone()],
                None,
                Vec::new(),
                vec![left.location, right.location],
                Some(format!(
                    "likely mounting-hole edge spacing {edge_spacing:.6} is below review spacing {minimum_edge_spacing:.6}; review screw head, washer, standoff, and drill breakout spacing"
                )),
            ));
        }
    }

    violations
}

/// Run the `panel_feature_outline_readiness` design-readiness check or report helper.
pub fn panel_feature_outline_readiness(
    board: &BoardModel,
    edge_distance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let Some(panel_features) = &board.panel_features else {
        return Vec::new();
    };

    let Some(outline) = &board.board_outline else {
        return vec![Violation::new(
            "panel-feature-outline-readiness",
            Severity::Warning,
            vec![board.source.clone()],
            None,
            Vec::new(),
            Vec::new(),
            Some(
                "KiCad panel or rout graphics were parsed, but no Edge.Cuts outline is available for edge proximity review"
                    .to_string(),
            ),
        )];
    };

    let outline_geometry = outline.to_multipolygon();
    let mut violations = Vec::new();
    for polygon in panel_features.to_multipolygon().0 {
        let feature = polygons_to_sketch(
            vec![polygon],
            Some(LayerMetadata {
                name: "KiCad panel feature".to_string(),
            }),
        );
        // IPC-2221B treats board outline and panel/mechanical definition as
        // release-package data, not just drawing decoration. A panel graphic
        // that is not close to the routed outline may be a valid note, but it
        // is weak evidence for tabs, rails, V-scores, or route paths.
        if polygon_boundary_distance(&feature.to_multipolygon(), &outline_geometry) <= edge_distance
        {
            continue;
        }

        let shapes = multipolygon_to_shapes(&feature.to_multipolygon(), min_area);
        if shapes.is_empty() {
            continue;
        }

        violations.push(Violation::new(
            "panel-feature-outline-readiness",
            Severity::Warning,
            vec![board.source.clone()],
            None,
            shapes,
            Vec::new(),
            Some(format!(
                "parsed panel/rout feature is more than {edge_distance:.6} from the board outline; review tab-route, V-score, rail, or mechanical-layer intent"
            )),
        ));
    }

    violations
}

/// Run the `edge_plating_intent_readiness` design-readiness check or report helper.
pub fn edge_plating_intent_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    edge_distance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let Some(outline) = &board.board_outline else {
        return Vec::new();
    };

    let outline_geometry = outline.to_multipolygon();
    let mut violations = Vec::new();
    for feature in selected_copper_features(board, selected_layers) {
        let feature_geometry = feature.sketch.to_multipolygon();
        let outside_outline = feature.sketch.difference(outline);
        let shapes = multipolygon_to_shapes(&outside_outline.to_multipolygon(), min_area);
        let reaches_outline =
            polygon_boundary_distance(&feature_geometry, &outline_geometry) <= edge_distance;
        if !reaches_outline && shapes.is_empty() {
            continue;
        }

        if shapes.is_empty() && !looks_edge_plating_net(feature.net.as_deref()) {
            continue;
        }

        // IPC-2221B treats edge contacts, plated edges, and board-outline copper
        // as fabrication notes that need explicit mechanical intent. This check
        // does not decide whether edge copper is wrong; it catches geometry that
        // should be paired with an edge-plating, castellation, bevel, or copper
        // pullback instruction before release.
        violations.push(Violation::new(
            "edge-plating-intent-readiness",
            Severity::Warning,
            vec![feature.layer.clone()],
            None,
            shapes,
            if reaches_outline {
                vec![feature.location]
            } else {
                Vec::new()
            },
            Some(format!(
                "{:?} copper on net {:?} reaches or crosses the board outline; review edge-plating, castellation, bevel, or copper-pullback fabrication intent",
                feature.kind, feature.net
            )),
        ));
    }

    violations
}

/// Run the `castellation_pitch_readiness` design-readiness check or report helper.
pub fn castellation_pitch_readiness(
    board: &BoardModel,
    minimum_edge_spacing: f64,
) -> Vec<Violation> {
    let holes = plated_edge_holes(board, minimum_edge_spacing);
    let mut violations = Vec::new();

    for left_index in 0..holes.len() {
        for right in &holes[(left_index + 1)..] {
            let left = holes[left_index];
            let edge_spacing =
                distance(left.location, right.location) - (left.diameter + right.diameter) / 2.0;
            if edge_spacing >= minimum_edge_spacing {
                continue;
            }

            violations.push(Violation::new(
                "castellation-pitch-readiness",
                Severity::Warning,
                vec![board.source.clone()],
                None,
                Vec::new(),
                vec![left.location, right.location],
                Some(format!(
                    "plated edge-hole spacing {edge_spacing:.6} is below castellation review spacing {minimum_edge_spacing:.6}; review half-hole pitch, breakout, and routed-edge plating capability"
                )),
            ));
        }
    }

    violations
}

fn likely_mounting_holes(board: &BoardModel, clearance_hint: f64) -> Vec<&DrillFeature> {
    let minimum_diameter = clearance_hint.max(1.0);
    board
        .drills
        .iter()
        .filter(|drill| !drill.plated && drill.diameter >= minimum_diameter)
        .collect()
}

fn likely_plated_mounting_holes(board: &BoardModel, clearance_hint: f64) -> Vec<&DrillFeature> {
    let minimum_diameter = clearance_hint.max(1.0);
    board
        .drills
        .iter()
        .filter(|drill| drill.plated && drill.diameter >= minimum_diameter)
        .collect()
}

fn likely_hardware_holes(board: &BoardModel, spacing_hint: f64) -> Vec<&DrillFeature> {
    let minimum_diameter = (spacing_hint * 0.25).clamp(1.0, 3.2);
    board
        .drills
        .iter()
        .filter(|drill| drill.diameter >= minimum_diameter)
        .collect()
}

fn plated_edge_holes(board: &BoardModel, edge_distance: f64) -> Vec<&DrillFeature> {
    let Some(outline) = &board.board_outline else {
        return Vec::new();
    };
    let outline_geometry = outline.to_multipolygon();

    board
        .drills
        .iter()
        .filter(|drill| {
            drill.plated
                && polygon_boundary_distance(
                    &drill_keepout(drill, 0.0).to_multipolygon(),
                    &outline_geometry,
                ) <= edge_distance
        })
        .collect()
}

fn drill_keepout(drill: &DrillFeature, keepout: f64) -> crate::PcbSketch {
    polygons_to_sketch(
        vec![circle_polygon(
            drill.location,
            drill.diameter / 2.0 + keepout,
            48,
        )],
        Some(LayerMetadata {
            name: "mounting-hole keepout".to_string(),
        }),
    )
}

fn selected_copper_features<'a>(
    board: &'a BoardModel,
    selected_layers: &[String],
) -> Vec<&'a CopperFeature> {
    board
        .copper
        .iter()
        .filter(|feature| {
            selected_layers.is_empty()
                || selected_layers.iter().any(|layer| layer == &feature.layer)
        })
        .collect()
}

fn looks_ground_or_chassis_net(net: &str) -> bool {
    let normalized = net.to_ascii_lowercase();
    normalized.contains("gnd")
        || normalized.contains("ground")
        || normalized.contains("earth")
        || normalized.contains("chassis")
        || normalized.contains("shield")
        || normalized == "0v"
}

fn looks_edge_plating_net(net: Option<&str>) -> bool {
    let Some(net) = net else {
        return false;
    };
    let normalized = net.to_ascii_lowercase();
    normalized.contains("castell")
        || normalized.contains("edge_plate")
        || normalized.contains("edge-plate")
        || normalized.contains("edgeplating")
        || normalized.contains("edge_plating")
        || normalized.contains("plated_edge")
        || normalized.contains("plated-edge")
}

fn distance(start: [f64; 2], end: [f64; 2]) -> f64 {
    let dx = end[0] - start[0];
    let dy = end[1] - start[1];
    (dx * dx + dy * dy).sqrt()
}

#[cfg(test)]
mod tests {
    use crate::LayerMetadata;
    use crate::geometry::{circle_polygon, polygons_to_sketch};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};

    use super::{
        castellation_pitch_readiness, edge_plating_intent_readiness,
        mounting_hole_copper_keepout_readiness, mounting_hole_distribution_readiness,
        mounting_hole_edge_clearance_readiness, mounting_hole_grounding_readiness,
        mounting_hole_plating_intent_readiness, mounting_hole_spacing_readiness,
        panel_feature_outline_readiness,
    };

    #[test]
    fn mounting_hole_grounding_readiness_reports_unreferenced_large_npth() {
        let board = board_with(
            vec![],
            vec![npth([10.0, 10.0], 3.2), npth([20.0, 10.0], 0.4)],
        );

        let violations = mounting_hole_grounding_readiness(&board, &[], 1.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "mounting-hole-grounding-readiness");
        assert_eq!(violations[0].locations, vec![[10.0, 10.0]]);
    }

    #[test]
    fn mounting_hole_grounding_readiness_accepts_nearby_chassis_copper() {
        let board = board_with(
            vec![copper("CHASSIS", CopperKind::Pad, [11.0, 10.0], 0.4)],
            vec![npth([10.0, 10.0], 3.2)],
        );

        assert!(mounting_hole_grounding_readiness(&board, &[], 1.0).is_empty());
    }

    #[test]
    fn mounting_hole_copper_keepout_reports_non_ground_copper_intrusion() {
        let board = board_with(
            vec![
                copper("SIG", CopperKind::Segment, [11.4, 10.0], 0.4),
                copper("GND", CopperKind::Zone, [8.8, 10.0], 0.4),
            ],
            vec![npth([10.0, 10.0], 2.0)],
        );

        let violations = mounting_hole_copper_keepout_readiness(&board, &[], 0.5, 0.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(
            violations[0].check,
            "mounting-hole-copper-keepout-readiness"
        );
        assert_eq!(violations[0].layers, vec!["F.Cu"]);
    }

    #[test]
    fn mounting_hole_copper_keepout_respects_selected_layers() {
        let board = board_with(
            vec![CopperFeature {
                layer: "B.Cu".to_string(),
                net: Some("SIG".to_string()),
                kind: CopperKind::Pad,
                sketch: polygons_to_sketch(
                    vec![circle_polygon([10.8, 10.0], 0.4, 32)],
                    Some(LayerMetadata {
                        name: "B.Cu pad".to_string(),
                    }),
                ),
                location: [10.8, 10.0],
            }],
            vec![npth([10.0, 10.0], 2.0)],
        );

        assert!(
            mounting_hole_copper_keepout_readiness(&board, &["F.Cu".to_string()], 0.5, 0.0)
                .is_empty()
        );
    }

    #[test]
    fn mounting_hole_edge_clearance_reports_keepout_beyond_outline() {
        let mut board = board_with(vec![], vec![npth([1.0, 5.0], 2.0)]);
        board.board_outline = Some(polygons_to_sketch(
            vec![crate::geometry::rect_polygon([5.0, 5.0], [10.0, 10.0], 0.0)],
            Some(LayerMetadata {
                name: "outline".to_string(),
            }),
        ));

        let violations = mounting_hole_edge_clearance_readiness(&board, 0.5, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(
            violations[0].check,
            "mounting-hole-edge-clearance-readiness"
        );
    }

    #[test]
    fn mounting_hole_edge_clearance_allows_inset_or_missing_outline() {
        let mut board = board_with(vec![], vec![npth([5.0, 5.0], 2.0)]);
        assert!(mounting_hole_edge_clearance_readiness(&board, 0.5, 1.0e-9).is_empty());

        board.board_outline = Some(polygons_to_sketch(
            vec![crate::geometry::rect_polygon([5.0, 5.0], [10.0, 10.0], 0.0)],
            Some(LayerMetadata {
                name: "outline".to_string(),
            }),
        ));
        assert!(mounting_hole_edge_clearance_readiness(&board, 0.5, 1.0e-9).is_empty());
    }

    #[test]
    fn mounting_hole_plating_intent_reports_unbonded_large_plated_hole() {
        let board = board_with(vec![], vec![pth([10.0, 10.0], 3.2, Some("MOUNT"))]);

        let violations = mounting_hole_plating_intent_readiness(&board, &[], 1.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(
            violations[0].check,
            "mounting-hole-plating-intent-readiness"
        );
    }

    #[test]
    fn mounting_hole_plating_intent_accepts_ground_net_or_nearby_bonding_copper() {
        let ground_net = board_with(vec![], vec![pth([10.0, 10.0], 3.2, Some("GND"))]);
        assert!(mounting_hole_plating_intent_readiness(&ground_net, &[], 1.0).is_empty());

        let nearby_chassis = board_with(
            vec![copper("CHASSIS", CopperKind::Pad, [11.0, 10.0], 0.4)],
            vec![pth([10.0, 10.0], 3.2, Some("MOUNT"))],
        );
        assert!(mounting_hole_plating_intent_readiness(&nearby_chassis, &[], 1.0).is_empty());
    }

    #[test]
    fn mounting_hole_distribution_reports_single_or_clustered_holes() {
        let single = board_with(vec![], vec![npth([10.0, 10.0], 3.2)]);
        let single_violations = mounting_hole_distribution_readiness(&single, 8.0);
        assert_eq!(single_violations.len(), 1);
        assert_eq!(
            single_violations[0].check,
            "mounting-hole-distribution-readiness"
        );

        let clustered = board_with(
            vec![],
            vec![npth([10.0, 10.0], 3.2), pth([12.0, 10.0], 3.2, Some("GND"))],
        );
        let clustered_violations = mounting_hole_distribution_readiness(&clustered, 8.0);
        assert_eq!(clustered_violations.len(), 1);
        assert_eq!(
            clustered_violations[0].check,
            "mounting-hole-distribution-readiness"
        );
        assert_eq!(
            clustered_violations[0].locations,
            vec![[10.0, 10.0], [12.0, 10.0]]
        );
    }

    #[test]
    fn mounting_hole_distribution_accepts_absent_or_well_spaced_holes() {
        let absent = board_with(vec![], vec![npth([0.0, 0.0], 0.5)]);
        assert!(mounting_hole_distribution_readiness(&absent, 8.0).is_empty());

        let spaced = board_with(
            vec![],
            vec![npth([0.0, 0.0], 3.2), pth([20.0, 0.0], 3.2, Some("GND"))],
        );
        assert!(mounting_hole_distribution_readiness(&spaced, 8.0).is_empty());
    }

    #[test]
    fn mounting_hole_spacing_reports_tight_hardware_holes() {
        let board = board_with(
            vec![],
            vec![npth([0.0, 0.0], 3.0), pth([3.4, 0.0], 3.0, Some("GND"))],
        );

        let violations = mounting_hole_spacing_readiness(&board, 0.5);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "mounting-hole-spacing-readiness");
        assert_eq!(violations[0].locations, vec![[0.0, 0.0], [3.4, 0.0]]);
    }

    #[test]
    fn mounting_hole_spacing_accepts_distant_or_tiny_holes() {
        let board = board_with(
            vec![],
            vec![
                npth([0.0, 0.0], 3.0),
                pth([5.0, 0.0], 3.0, Some("GND")),
                npth([1.0, 1.0], 0.4),
            ],
        );

        assert!(mounting_hole_spacing_readiness(&board, 0.5).is_empty());
    }

    #[test]
    fn panel_feature_outline_readiness_reports_missing_outline_or_interior_feature() {
        let mut missing_outline = board_with(vec![], vec![]);
        missing_outline.panel_features = Some(polygons_to_sketch(
            vec![crate::geometry::rect_polygon([5.0, 5.0], [1.0, 1.0], 0.0)],
            Some(LayerMetadata {
                name: "panel".to_string(),
            }),
        ));
        let missing = panel_feature_outline_readiness(&missing_outline, 0.5, 1.0e-9);
        assert_eq!(missing.len(), 1);
        assert_eq!(missing[0].check, "panel-feature-outline-readiness");

        let mut interior = board_with(vec![], vec![]);
        interior.board_outline = Some(outline());
        interior.panel_features = Some(polygons_to_sketch(
            vec![crate::geometry::rect_polygon([5.0, 5.0], [1.0, 1.0], 0.0)],
            Some(LayerMetadata {
                name: "panel".to_string(),
            }),
        ));
        let interior_violations = panel_feature_outline_readiness(&interior, 0.5, 1.0e-9);
        assert_eq!(interior_violations.len(), 1);
        assert_eq!(
            interior_violations[0].check,
            "panel-feature-outline-readiness"
        );
    }

    #[test]
    fn panel_feature_outline_readiness_accepts_edge_adjacent_or_absent_features() {
        let absent = board_with(vec![], vec![]);
        assert!(panel_feature_outline_readiness(&absent, 0.5, 1.0e-9).is_empty());

        let mut near_edge = board_with(vec![], vec![]);
        near_edge.board_outline = Some(outline());
        near_edge.panel_features = Some(polygons_to_sketch(
            vec![crate::geometry::rect_polygon([0.1, 5.0], [0.2, 2.0], 0.0)],
            Some(LayerMetadata {
                name: "panel".to_string(),
            }),
        ));
        assert!(panel_feature_outline_readiness(&near_edge, 0.5, 1.0e-9).is_empty());
    }

    #[test]
    fn edge_plating_intent_readiness_reports_edge_intent_or_crossing_copper() {
        let mut edge_intent = board_with(
            vec![copper("EDGE_PLATING", CopperKind::Pad, [0.25, 5.0], 0.2)],
            vec![],
        );
        edge_intent.board_outline = Some(outline());
        let intent_violations = edge_plating_intent_readiness(&edge_intent, &[], 0.5, 1.0e-9);
        assert_eq!(intent_violations.len(), 1);
        assert_eq!(intent_violations[0].check, "edge-plating-intent-readiness");

        let mut crossing = board_with(
            vec![copper("SIG", CopperKind::Segment, [-0.1, 5.0], 0.4)],
            vec![],
        );
        crossing.board_outline = Some(outline());
        let crossing_violations = edge_plating_intent_readiness(&crossing, &[], 0.5, 1.0e-9);
        assert_eq!(crossing_violations.len(), 1);
        assert_eq!(
            crossing_violations[0].check,
            "edge-plating-intent-readiness"
        );
    }

    #[test]
    fn edge_plating_intent_readiness_accepts_interior_or_selected_out_copper() {
        let mut interior = board_with(
            vec![copper("SIG", CopperKind::Segment, [5.0, 5.0], 0.4)],
            vec![],
        );
        interior.board_outline = Some(outline());
        assert!(edge_plating_intent_readiness(&interior, &[], 0.5, 1.0e-9).is_empty());

        let mut back_layer = board_with(
            vec![CopperFeature {
                layer: "B.Cu".to_string(),
                net: Some("EDGE_PLATING".to_string()),
                kind: CopperKind::Pad,
                sketch: polygons_to_sketch(
                    vec![circle_polygon([0.25, 5.0], 0.2, 32)],
                    Some(LayerMetadata {
                        name: "B.Cu copper".to_string(),
                    }),
                ),
                location: [0.25, 5.0],
            }],
            vec![],
        );
        back_layer.board_outline = Some(outline());
        assert!(
            edge_plating_intent_readiness(&back_layer, &["F.Cu".to_string()], 0.5, 1.0e-9)
                .is_empty()
        );
    }

    #[test]
    fn castellation_pitch_readiness_reports_tight_edge_plated_holes() {
        let mut board = board_with(
            vec![],
            vec![
                pth([0.0, 3.0], 0.6, Some("CAST")),
                pth([0.0, 3.7], 0.6, Some("CAST")),
            ],
        );
        board.board_outline = Some(outline());

        let violations = castellation_pitch_readiness(&board, 0.5);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "castellation-pitch-readiness");
        assert_eq!(violations[0].locations, vec![[0.0, 3.0], [0.0, 3.7]]);
    }

    #[test]
    fn castellation_pitch_readiness_accepts_distant_or_interior_plated_holes() {
        let mut board = board_with(
            vec![],
            vec![
                pth([0.0, 2.0], 0.6, Some("CAST")),
                pth([0.0, 4.0], 0.6, Some("CAST")),
                pth([5.0, 5.0], 0.6, Some("VIA")),
            ],
        );
        board.board_outline = Some(outline());

        assert!(castellation_pitch_readiness(&board, 0.5).is_empty());
    }

    fn board_with(copper: Vec<CopperFeature>, drills: Vec<DrillFeature>) -> BoardModel {
        BoardModel {
            source: "board.kicad_pcb".to_string(),
            copper,
            drills,
            board_outline: None,
            panel_features: None,
        }
    }

    fn npth(location: [f64; 2], diameter: f64) -> DrillFeature {
        DrillFeature {
            location,
            diameter,
            net: None,
            plated: false,
        }
    }

    fn pth(location: [f64; 2], diameter: f64, net: Option<&str>) -> DrillFeature {
        DrillFeature {
            location,
            diameter,
            net: net.map(str::to_string),
            plated: true,
        }
    }

    fn outline() -> crate::PcbSketch {
        polygons_to_sketch(
            vec![crate::geometry::rect_polygon([5.0, 5.0], [10.0, 10.0], 0.0)],
            Some(LayerMetadata {
                name: "outline".to_string(),
            }),
        )
    }

    fn copper(net: &str, kind: CopperKind, location: [f64; 2], radius: f64) -> CopperFeature {
        CopperFeature {
            layer: "F.Cu".to_string(),
            net: Some(net.to_string()),
            kind,
            sketch: polygons_to_sketch(
                vec![circle_polygon(location, radius, 32)],
                Some(LayerMetadata {
                    name: "F.Cu copper".to_string(),
                }),
            ),
            location,
        }
    }
}
