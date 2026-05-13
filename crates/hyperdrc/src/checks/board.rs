//! Board-level checks that need nets, drills, vias, or panel features.

use std::collections::HashMap;

use csgrs::csg::CSG;
use geo::Area;

use super::distance::polygon_boundary_distance;
use crate::geometry::{circle_polygon, multipolygon_to_shapes, polygons_to_sketch};
use crate::ipc356::Ipc356Point;
use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
use crate::report::{Severity, Violation};
use crate::{LayerMetadata, PcbSketch};

pub fn annular_ring(
    board: &BoardModel,
    minimum_ring: f64,
    selected_layers: &[String],
) -> Vec<Violation> {
    let mut violations = Vec::new();

    for drill in &board.drills {
        if !drill.plated {
            continue;
        }

        let Some(nearest) = nearest_matching_copper(board, drill, selected_layers) else {
            continue;
        };

        // KiCad pad geometry can be rectangular, oval, or custom. For the first
        // pass we use an area-equivalent circular radius so annular-ring checks
        // remain shape-agnostic. This is conservative enough to flag suspect
        // cases, but exact pad-vs-drill containment can be added later.
        let copper_radius = equivalent_radius(&nearest.sketch);
        let ring = copper_radius - drill.diameter / 2.0;
        if ring < minimum_ring {
            violations.push(Violation::new(
                "annular-ring-readiness",
                Severity::Error,
                vec![nearest.layer.clone()],
                None,
                Vec::new(),
                vec![drill.location],
                Some(format!(
                    "annular ring {ring:.6} is below minimum {minimum_ring:.6}"
                )),
            ));
        }
    }

    violations
}

pub fn plating_intent(
    board: &BoardModel,
    selected_layers: &[String],
    tolerance: f64,
) -> Vec<Violation> {
    let copper_features = selected_copper_features(board, selected_layers);
    let mut violations = Vec::new();

    for drill in &board.drills {
        if drill.plated {
            if has_plated_drill_copper(drill, &copper_features, tolerance) {
                continue;
            }

            violations.push(Violation::new(
                "plating-intent",
                Severity::Warning,
                vec!["KiCad drills".to_string()],
                None,
                Vec::new(),
                vec![drill.location],
                Some("plated drill has no nearby same-net pad or via copper".to_string()),
            ));
        } else if has_nearby_copper(
            drill.location,
            &copper_features,
            drill.diameter / 2.0 + tolerance,
        ) {
            violations.push(Violation::new(
                "plating-intent",
                Severity::Warning,
                vec!["KiCad NPTH drills".to_string()],
                None,
                Vec::new(),
                vec![drill.location],
                Some(
                    "non-plated drill has nearby copper that may imply plated-hole intent"
                        .to_string(),
                ),
            ));
        }
    }

    violations
}

pub fn drill_to_copper_clearance(
    board: &BoardModel,
    extra_drills: &[DrillFeature],
    clearance: f64,
    selected_layers: &[String],
    min_area: f64,
) -> Vec<Violation> {
    let mut drills = board.drills.clone();
    drills.extend_from_slice(extra_drills);
    let copper_features = selected_copper_features(board, selected_layers);
    let mut violations = Vec::new();

    for drill in drills {
        let keepout = polygons_to_sketch(
            vec![circle_polygon(
                drill.location,
                drill.diameter / 2.0 + clearance,
                64,
            )],
            Some(LayerMetadata {
                name: "drill keepout".to_string(),
            }),
        );

        for copper in &copper_features {
            if drill.plated && drill.net.is_some() && drill.net == copper.net {
                continue;
            }

            let overlap = keepout.intersection(&copper.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            if shapes.is_empty() {
                continue;
            }

            violations.push(Violation::new(
                "drill-to-copper-clearance",
                Severity::Error,
                vec![copper.layer.clone()],
                None,
                shapes,
                vec![drill.location],
                Some(format!(
                    "drill keepout with clearance {clearance} intersects copper"
                )),
            ));
        }
    }

    violations
}

pub fn drill_spacing(
    board_drills: &[DrillFeature],
    extra_drills: &[DrillFeature],
    clearance: f64,
) -> Vec<Violation> {
    let mut drills = board_drills.to_vec();
    drills.extend_from_slice(extra_drills);
    let mut violations = Vec::new();

    for left_index in 0..drills.len() {
        let left = &drills[left_index];
        for right in &drills[(left_index + 1)..] {
            let edge_gap = distance(left.location, right.location)
                - left.diameter / 2.0
                - right.diameter / 2.0;
            if edge_gap >= clearance {
                continue;
            }

            violations.push(Violation::new(
                "drill-spacing",
                Severity::Error,
                vec!["drills".to_string()],
                None,
                Vec::new(),
                vec![left.location, right.location],
                Some(format!(
                    "drill edge spacing {edge_gap:.6} is below clearance {clearance:.6}"
                )),
            ));
        }
    }

    violations
}

pub fn board_outline_drill_clearance(
    drill_source: &str,
    outline_name: &str,
    outline: &PcbSketch,
    board_drills: &[DrillFeature],
    extra_drills: &[DrillFeature],
    clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let mut drills = board_drills.to_vec();
    drills.extend_from_slice(extra_drills);
    let mut violations = Vec::new();

    for drill in drills {
        let keepout = polygons_to_sketch(
            vec![circle_polygon(
                drill.location,
                drill.diameter / 2.0 + clearance,
                64,
            )],
            Some(LayerMetadata {
                name: "drill edge keepout".to_string(),
            }),
        );
        let outside_outline = keepout.difference(outline);
        let shapes = multipolygon_to_shapes(&outside_outline.to_multipolygon(), min_area);
        if shapes.is_empty() {
            continue;
        }

        violations.push(Violation::new(
            "board-outline-drill-clearance",
            Severity::Error,
            vec![drill_source.to_string(), outline_name.to_string()],
            None,
            shapes,
            vec![drill.location],
            Some(format!(
                "drill edge is within board outline clearance {clearance}"
            )),
        ));
    }

    violations
}

pub fn drill_aspect_ratio(
    source: &str,
    drills: &[DrillFeature],
    board_thickness: f64,
    max_aspect_ratio: f64,
) -> Vec<Violation> {
    let mut violations = Vec::new();

    for drill in drills {
        if drill.diameter <= 0.0 {
            violations.push(Violation::new(
                "drill-aspect-ratio",
                Severity::Warning,
                vec![source.to_string()],
                None,
                Vec::new(),
                vec![drill.location],
                Some("drill diameter is not positive, so aspect ratio is undefined".to_string()),
            ));
            continue;
        }

        let aspect_ratio = board_thickness / drill.diameter;
        if aspect_ratio <= max_aspect_ratio {
            continue;
        }

        violations.push(Violation::new(
            "drill-aspect-ratio",
            Severity::Warning,
            vec![source.to_string()],
            None,
            Vec::new(),
            vec![drill.location],
            Some(format!(
                "drill aspect ratio {aspect_ratio:.3} exceeds maximum {max_aspect_ratio:.3} for board thickness {board_thickness:.3}"
            )),
        ));
    }

    violations
}

pub fn drill_table_consistency(
    board_drills: &[DrillFeature],
    extra_drills: &[DrillFeature],
    ipc356_points: &[Ipc356Point],
    tolerance: f64,
) -> Vec<Violation> {
    let mut violations = Vec::new();

    for board_drill in board_drills {
        for extra_drill in extra_drills {
            if distance(board_drill.location, extra_drill.location) > tolerance {
                continue;
            }
            if !diameters_conflict(board_drill.diameter, extra_drill.diameter, tolerance) {
                continue;
            }

            violations.push(drill_table_violation(
                "KiCad drills",
                board_drill.diameter,
                "Excellon drills",
                extra_drill.diameter,
                vec![board_drill.location, extra_drill.location],
            ));
        }
    }

    for extra_drill in extra_drills {
        for point in ipc356_points {
            if distance(extra_drill.location, point.location) > tolerance {
                continue;
            }
            let Some(ipc_diameter) = point.diameter else {
                continue;
            };
            if !diameters_conflict(extra_drill.diameter, ipc_diameter, tolerance) {
                continue;
            }

            violations.push(drill_table_violation(
                "Excellon drills",
                extra_drill.diameter,
                "IPC-D-356 drills",
                ipc_diameter,
                vec![extra_drill.location, point.location],
            ));
        }
    }

    violations
}

pub fn copper_net_intent(board: &BoardModel, selected_layers: &[String]) -> Vec<Violation> {
    selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.net.is_none())
        .map(|feature| {
            Violation::new(
                "copper-net-intent",
                Severity::Warning,
                vec![feature.layer.clone()],
                None,
                Vec::new(),
                vec![feature.location],
                Some(format!(
                    "parsed {:?} copper has no net after KiCad parsing and IPC-D-356 annotation",
                    feature.kind
                )),
            )
        })
        .collect()
}

fn diameters_conflict(left: f64, right: f64, tolerance: f64) -> bool {
    left > 0.0 && right > 0.0 && (left - right).abs() > tolerance
}

fn drill_table_violation(
    left_source: &str,
    left_diameter: f64,
    right_source: &str,
    right_diameter: f64,
    locations: Vec<[f64; 2]>,
) -> Violation {
    Violation::new(
        "drill-table-consistency",
        Severity::Warning,
        vec![left_source.to_string(), right_source.to_string()],
        None,
        Vec::new(),
        locations,
        Some(format!(
            "{left_source} diameter {left_diameter:.6} differs from {right_source} diameter {right_diameter:.6}"
        )),
    )
}

pub fn net_spacing(
    board: &BoardModel,
    clearance: f64,
    selected_layers: &[String],
    min_area: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let mut violations = Vec::new();

    for left_index in 0..features.len() {
        for right in &features[(left_index + 1)..] {
            let left = &features[left_index];
            if left.layer != right.layer || left.net.is_none() || left.net == right.net {
                continue;
            }

            // Clearance is modeled by a Minkowski sum of the left copper feature
            // with a disk of radius `clearance`, followed by an intersection
            // with the right feature. In computational geometry terms this is a
            // set-membership test against an offset region; see Lee and
            // Preparata, "Computational Geometry - A Survey", IEEE TC, 1984.
            let overlap = left.sketch.offset(clearance).intersection(&right.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            let locations = if shapes.is_empty()
                && polygon_boundary_distance(
                    &left.sketch.to_multipolygon(),
                    &right.sketch.to_multipolygon(),
                ) <= clearance
            {
                vec![left.location, right.location]
            } else {
                Vec::new()
            };
            if shapes.is_empty() && locations.is_empty() {
                continue;
            }

            violations.push(Violation::new(
                "different-net-spacing",
                Severity::Error,
                vec![left.layer.clone()],
                None,
                shapes,
                locations,
                Some(format!(
                    "net {:?} is within {clearance} of net {:?}",
                    left.net, right.net
                )),
            ));
        }
    }

    violations
}

pub fn registration_tolerance(board: &BoardModel, tolerance: f64, min_area: f64) -> Vec<Violation> {
    let mut by_layer = board.copper_layers(&[]);
    by_layer.sort_by(|left, right| left.0.cmp(&right.0));
    let mut violations = Vec::new();

    for index in 0..by_layer.len() {
        for other in &by_layer[(index + 1)..] {
            let left = &by_layer[index];
            let overlap = left.1.offset(tolerance).intersection(&other.1);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            if shapes.is_empty() {
                continue;
            }

            violations.push(Violation::new(
                "layer-registration-tolerance",
                Severity::Warning,
                vec![left.0.clone(), other.0.clone()],
                None,
                shapes,
                Vec::new(),
                Some(format!(
                    "features on paired layers are within registration tolerance {tolerance}"
                )),
            ));
        }
    }

    violations
}

pub fn panelization_clearance(
    board: &BoardModel,
    extra_drills: &[DrillFeature],
    clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let copper = board.all_copper();
    let mut blockers = Vec::new();

    if let Some(panel_features) = &board.panel_features {
        blockers.push(panel_features.clone());
    }

    if !extra_drills.is_empty() {
        blockers.push(drills_to_sketch(extra_drills, "Excellon panel drills"));
    }

    let npth = board
        .drills
        .iter()
        .filter(|drill| !drill.plated)
        .cloned()
        .collect::<Vec<_>>();
    if !npth.is_empty() {
        blockers.push(drills_to_sketch(&npth, "KiCad NPTH panel drills"));
    }

    let mut violations = Vec::new();
    for blocker in blockers {
        let overlap = blocker.offset(clearance).intersection(&copper);
        let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
        let fallback_hit = shapes.is_empty()
            && polygon_boundary_distance(&blocker.to_multipolygon(), &copper.to_multipolygon())
                <= clearance;
        if shapes.is_empty() && !fallback_hit {
            continue;
        }

        violations.push(Violation::new(
            "panelization-clearance",
            Severity::Warning,
            vec!["KiCad copper".to_string()],
            None,
            shapes,
            Vec::new(),
            Some(format!(
                "copper is within panel feature clearance {clearance}"
            )),
        ));
    }

    violations
}

pub fn apply_ipc356_nets(board: &mut BoardModel, points: &[Ipc356Point], tolerance: f64) {
    for point in points {
        for copper in &mut board.copper {
            if copper.net.is_none() && distance(copper.location, point.location) <= tolerance {
                copper.net = Some(point.net.clone());
            }
        }

        for drill in &mut board.drills {
            if drill.net.is_none() && distance(drill.location, point.location) <= tolerance {
                drill.net = Some(point.net.clone());
            }
            if drill.diameter == 0.0
                && let Some(diameter) = point.diameter
            {
                drill.diameter = diameter;
            }
        }
    }
}

pub fn ipc356_coverage(
    board: &BoardModel,
    points: &[Ipc356Point],
    tolerance: f64,
) -> Vec<Violation> {
    let mut violations = Vec::new();

    for point in points {
        let has_copper = board
            .copper
            .iter()
            .any(|feature| distance(feature.location, point.location) <= tolerance);
        if has_copper {
            continue;
        }

        let label = match (&point.reference, &point.pin) {
            (Some(reference), Some(pin)) => format!("{reference}.{pin}"),
            (Some(reference), None) => reference.clone(),
            _ => "IPC-D-356 test record".to_string(),
        };
        violations.push(Violation::new(
            "ipc356-coverage",
            Severity::Warning,
            vec![point.net.clone()],
            None,
            Vec::new(),
            vec![point.location],
            Some(format!(
                "{label} has no parsed KiCad copper feature within {tolerance}"
            )),
        ));
    }

    violations
}

pub fn ipc356_drill_diameter(
    board: &BoardModel,
    points: &[Ipc356Point],
    tolerance: f64,
) -> Vec<Violation> {
    let mut violations = Vec::new();

    for point in points {
        let Some(ipc_diameter) = point.diameter else {
            continue;
        };
        for drill in &board.drills {
            if distance(drill.location, point.location) > tolerance {
                continue;
            }
            if drill.diameter == 0.0 || (drill.diameter - ipc_diameter).abs() <= tolerance {
                continue;
            }

            violations.push(Violation::new(
                "ipc356-drill-diameter",
                Severity::Warning,
                vec![point.net.clone()],
                None,
                Vec::new(),
                vec![drill.location, point.location],
                Some(format!(
                    "drill diameter {:.6} differs from IPC-D-356 diameter {:.6}",
                    drill.diameter, ipc_diameter
                )),
            ));
        }
    }

    violations
}

pub fn drills_to_sketch(drills: &[DrillFeature], name: &str) -> PcbSketch {
    let polygons = drills
        .iter()
        .map(|drill| circle_polygon(drill.location, drill.diameter / 2.0, 48))
        .collect::<Vec<_>>();

    polygons_to_sketch(
        polygons,
        Some(LayerMetadata {
            name: name.to_string(),
        }),
    )
}

fn nearest_matching_copper<'a>(
    board: &'a BoardModel,
    drill: &DrillFeature,
    selected_layers: &[String],
) -> Option<&'a CopperFeature> {
    selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| drill.net.is_none() || feature.net == drill.net)
        .min_by(|left, right| {
            distance(left.location, drill.location)
                .partial_cmp(&distance(right.location, drill.location))
                .unwrap_or(std::cmp::Ordering::Equal)
        })
}

fn has_plated_drill_copper(
    drill: &DrillFeature,
    copper_features: &[&CopperFeature],
    tolerance: f64,
) -> bool {
    copper_features.iter().any(|feature| {
        matches!(feature.kind, CopperKind::Pad | CopperKind::Via)
            && (drill.net.is_none() || feature.net == drill.net)
            && distance(feature.location, drill.location) <= tolerance
    })
}

fn has_nearby_copper(
    location: [f64; 2],
    copper_features: &[&CopperFeature],
    tolerance: f64,
) -> bool {
    copper_features
        .iter()
        .any(|feature| distance(feature.location, location) <= tolerance)
}

fn selected_copper_features<'a>(
    board: &'a BoardModel,
    selected_layers: &[String],
) -> Vec<&'a CopperFeature> {
    board
        .copper
        .iter()
        .filter(|feature| selected_layers.is_empty() || selected_layers.contains(&feature.layer))
        .collect()
}

fn equivalent_radius(sketch: &PcbSketch) -> f64 {
    let area = sketch
        .to_multipolygon()
        .0
        .iter()
        .map(|polygon| polygon.unsigned_area())
        .sum::<f64>();
    (area / std::f64::consts::PI).sqrt()
}

fn distance(left: [f64; 2], right: [f64; 2]) -> f64 {
    let dx = left[0] - right[0];
    let dy = left[1] - right[1];
    (dx * dx + dy * dy).sqrt()
}

pub fn layer_names_csv(board: &BoardModel) -> String {
    let mut counts = HashMap::new();
    for feature in &board.copper {
        *counts
            .entry((feature.layer.clone(), feature.kind))
            .or_insert(0usize) += 1;
    }

    let mut layers = counts.into_iter().collect::<Vec<_>>();
    layers.sort_by(|left, right| left.0.cmp(&right.0));
    layers
        .into_iter()
        .map(|((layer, kind), count)| format!("{layer}:{}({count})", kind.as_str()))
        .collect::<Vec<_>>()
        .join(", ")
}

impl CopperKind {
    fn as_str(self) -> &'static str {
        match self {
            CopperKind::Pad => "pad",
            CopperKind::Via => "via",
            CopperKind::Segment => "segment",
            CopperKind::Zone => "zone",
        }
    }
}

#[cfg(test)]
mod tests {
    use geo::{Coord, LineString, Polygon};

    use crate::geometry::{circle_polygon, line_polygon, polygons_to_sketch};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
    use crate::{LayerMetadata, PcbSketch};

    use crate::ipc356::Ipc356Point;

    use super::{
        annular_ring, apply_ipc356_nets, board_outline_drill_clearance, copper_net_intent,
        drill_aspect_ratio, drill_spacing, drill_table_consistency, drill_to_copper_clearance,
        ipc356_coverage, ipc356_drill_diameter, net_spacing, panelization_clearance,
        plating_intent, registration_tolerance,
    };

    #[test]
    fn annular_ring_flags_small_pad() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![CopperFeature {
                layer: "F.Cu".to_string(),
                net: Some("GND".to_string()),
                kind: CopperKind::Pad,
                location: [0.0, 0.0],
                sketch: polygons_to_sketch(
                    vec![circle_polygon([0.0, 0.0], 0.4, 32)],
                    Some(LayerMetadata {
                        name: "pad".to_string(),
                    }),
                ),
            }],
            drills: vec![DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.7,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };

        assert_eq!(annular_ring(&board, 0.1, &[]).len(), 1);
    }

    #[test]
    fn annular_ring_allows_via_at_minimum_ring() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![CopperFeature {
                layer: "F.Cu".to_string(),
                net: Some("GND".to_string()),
                kind: CopperKind::Via,
                location: [0.0, 0.0],
                sketch: polygons_to_sketch(
                    vec![circle_polygon([0.0, 0.0], 0.5, 64)],
                    Some(LayerMetadata {
                        name: "via".to_string(),
                    }),
                ),
            }],
            drills: vec![DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.6,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };

        assert!(annular_ring(&board, 0.1, &[]).is_empty());
    }

    #[test]
    fn plating_intent_reports_npth_with_nearby_copper() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![copper_disc("GND", CopperKind::Pad, [0.0, 0.0], 0.4)],
            drills: vec![DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.6,
                net: None,
                plated: false,
            }],
            board_outline: None,
            panel_features: None,
        };

        let violations = plating_intent(&board, &[], 0.05);

        assert_eq!(violations.len(), 1);
        assert!(
            violations[0]
                .message
                .as_deref()
                .unwrap()
                .contains("non-plated")
        );
    }

    #[test]
    fn plating_intent_reports_plated_drill_without_pad_or_via_copper() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![copper_line(
                "GND",
                CopperKind::Segment,
                [0.0, 0.0],
                [1.0, 0.0],
                0.1,
            )],
            drills: vec![DrillFeature {
                location: [0.5, 0.0],
                diameter: 0.3,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };

        let violations = plating_intent(&board, &[], 0.05);

        assert_eq!(violations.len(), 1);
        assert!(
            violations[0]
                .message
                .as_deref()
                .unwrap()
                .contains("plated drill")
        );
    }

    #[test]
    fn plating_intent_allows_plated_drill_with_same_net_pad() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![copper_disc("GND", CopperKind::Pad, [0.0, 0.0], 0.4)],
            drills: vec![DrillFeature {
                location: [0.01, 0.0],
                diameter: 0.3,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };

        assert!(plating_intent(&board, &[], 0.05).is_empty());
    }

    #[test]
    fn drill_aspect_ratio_flags_small_holes_for_board_thickness() {
        let drills = vec![
            DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.15,
                net: None,
                plated: true,
            },
            DrillFeature {
                location: [1.0, 0.0],
                diameter: 0.4,
                net: None,
                plated: true,
            },
        ];

        let violations = drill_aspect_ratio("drills", &drills, 1.6, 10.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].locations, vec![[0.0, 0.0]]);
    }

    #[test]
    fn drill_aspect_ratio_reports_zero_diameter_without_dividing() {
        let drills = vec![DrillFeature {
            location: [0.0, 0.0],
            diameter: 0.0,
            net: None,
            plated: true,
        }];

        let violations = drill_aspect_ratio("drills", &drills, 1.6, 10.0);

        assert_eq!(violations.len(), 1);
        assert!(
            violations[0]
                .message
                .as_deref()
                .unwrap()
                .contains("undefined")
        );
    }

    #[test]
    fn drill_table_consistency_reports_kicad_excellon_diameter_conflicts() {
        let board_drills = vec![DrillFeature {
            location: [0.0, 0.0],
            diameter: 0.30,
            net: Some("GND".to_string()),
            plated: true,
        }];
        let excellon_drills = vec![DrillFeature {
            location: [0.01, 0.0],
            diameter: 0.45,
            net: None,
            plated: true,
        }];

        let violations = drill_table_consistency(&board_drills, &excellon_drills, &[], 0.05);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "drill-table-consistency");
    }

    #[test]
    fn drill_table_consistency_reports_excellon_ipc356_diameter_conflicts() {
        let excellon_drills = vec![DrillFeature {
            location: [1.0, 0.0],
            diameter: 0.30,
            net: None,
            plated: true,
        }];
        let points = vec![Ipc356Point {
            net: "GND".to_string(),
            reference: Some("V1".to_string()),
            pin: None,
            location: [1.01, 0.0],
            diameter: Some(0.50),
        }];

        let violations = drill_table_consistency(&[], &excellon_drills, &points, 0.05);

        assert_eq!(violations.len(), 1);
        assert_eq!(
            violations[0].layers,
            vec![
                "Excellon drills".to_string(),
                "IPC-D-356 drills".to_string()
            ]
        );
    }

    #[test]
    fn drill_table_consistency_allows_matching_or_unmatched_records() {
        let board_drills = vec![DrillFeature {
            location: [0.0, 0.0],
            diameter: 0.30,
            net: Some("GND".to_string()),
            plated: true,
        }];
        let excellon_drills = vec![
            DrillFeature {
                location: [0.01, 0.0],
                diameter: 0.31,
                net: None,
                plated: true,
            },
            DrillFeature {
                location: [10.0, 0.0],
                diameter: 0.90,
                net: None,
                plated: true,
            },
        ];

        assert!(drill_table_consistency(&board_drills, &excellon_drills, &[], 0.05).is_empty());
    }

    #[test]
    fn copper_net_intent_reports_unnetted_kicad_copper() {
        let mut unnetted = copper_disc("GND", CopperKind::Zone, [0.0, 0.0], 0.5);
        unnetted.net = None;
        let board = board_with_copper(vec![
            copper_disc("GND", CopperKind::Pad, [1.0, 0.0], 0.5),
            unnetted,
        ]);

        let violations = copper_net_intent(&board, &[]);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "copper-net-intent");
        assert_eq!(violations[0].locations, vec![[0.0, 0.0]]);
    }

    #[test]
    fn copper_net_intent_respects_selected_layers() {
        let mut unnetted_front =
            copper_disc_on_layer("GND", CopperKind::Zone, "F.Cu", [0.0, 0.0], 0.5);
        unnetted_front.net = None;
        let mut unnetted_back =
            copper_disc_on_layer("GND", CopperKind::Zone, "B.Cu", [1.0, 0.0], 0.5);
        unnetted_back.net = None;
        let board = board_with_copper(vec![unnetted_front, unnetted_back]);

        let violations = copper_net_intent(&board, &["B.Cu".to_string()]);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].layers, vec!["B.Cu".to_string()]);
    }

    #[test]
    fn net_spacing_flags_close_different_nets() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![feature("A", [0.0, 0.0]), feature("B", [0.9, 0.0])],
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        };

        assert_eq!(net_spacing(&board, 0.2, &[], 1.0e-9).len(), 1);
    }

    #[test]
    fn net_spacing_covers_pad_via_and_via_spacing() {
        let selected_layers = vec!["F.Cu".to_string()];
        let pad = copper_disc("PAD", CopperKind::Pad, [0.0, 0.0], 0.12);
        let via_a = copper_disc("VIA_A", CopperKind::Via, [0.28, 0.0], 0.12);
        let via_b = copper_disc("VIA_B", CopperKind::Via, [0.56, 0.0], 0.12);
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![pad, via_a, via_b],
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        };

        let violations = net_spacing(&board, 0.20, &selected_layers, 1.0e-9);

        assert_eq!(violations.len(), 2);
    }

    #[test]
    fn net_spacing_distance_fallback_covers_trace_clearances() {
        assert_eq!(
            net_spacing(
                &board_with_copper(vec![
                    copper_line("A", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.1),
                    copper_line("B", CopperKind::Segment, [0.0, 0.18], [1.0, 0.18], 0.1),
                ]),
                0.10,
                &[],
                1.0e-9,
            )
            .len(),
            1
        );

        assert_eq!(
            net_spacing(
                &board_with_copper(vec![
                    copper_line("A", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.1),
                    copper_disc("B", CopperKind::Pad, [1.15, 0.0], 0.06),
                ]),
                0.10,
                &[],
                1.0e-9,
            )
            .len(),
            1
        );

        assert_eq!(
            net_spacing(
                &board_with_copper(vec![
                    copper_line("A", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.1),
                    copper_disc("B", CopperKind::Via, [0.5, 0.20], 0.06),
                ]),
                0.10,
                &[],
                1.0e-9,
            )
            .len(),
            1
        );
    }

    #[test]
    fn net_spacing_allows_trace_clearances_above_threshold() {
        let board = board_with_copper(vec![
            copper_line("A", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.1),
            copper_line("B", CopperKind::Segment, [0.0, 0.30], [1.0, 0.30], 0.1),
        ]);

        assert!(net_spacing(&board, 0.10, &[], 1.0e-9).is_empty());
    }

    #[test]
    fn drill_to_copper_clearance_flags_hole_trace_and_slot_trace_cases() {
        let trace = copper_line("SIG", CopperKind::Segment, [0.0, 0.0], [2.0, 0.0], 0.1);
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![trace],
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        };
        let extra_drills = vec![
            DrillFeature {
                location: [0.5, 0.18],
                diameter: 0.2,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [1.5, 0.32],
                diameter: 0.2,
                net: None,
                plated: false,
            },
        ];

        let violations = drill_to_copper_clearance(&board, &extra_drills, 0.15, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn drill_to_copper_clearance_ignores_same_net_plated_drills() {
        let trace = copper_line("GND", CopperKind::Segment, [0.0, 0.0], [2.0, 0.0], 0.1);
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![trace],
            drills: vec![DrillFeature {
                location: [1.0, 0.0],
                diameter: 0.3,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };

        let violations = drill_to_copper_clearance(&board, &[], 0.15, &[], 1.0e-9);

        assert!(violations.is_empty());
    }

    #[test]
    fn drill_to_copper_clearance_flags_same_net_npth_drills() {
        let trace = copper_line("GND", CopperKind::Segment, [0.0, 0.0], [2.0, 0.0], 0.1);
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![trace],
            drills: vec![DrillFeature {
                location: [1.0, 0.0],
                diameter: 0.3,
                net: Some("GND".to_string()),
                plated: false,
            }],
            board_outline: None,
            panel_features: None,
        };

        let violations = drill_to_copper_clearance(&board, &[], 0.15, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "drill-to-copper-clearance");
    }

    #[test]
    fn drill_spacing_flags_close_holes_and_allows_compliant_holes() {
        let drills = vec![
            DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.4,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [0.55, 0.0],
                diameter: 0.4,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [2.0, 0.0],
                diameter: 0.4,
                net: None,
                plated: false,
            },
        ];

        let violations = drill_spacing(&drills, &[], 0.20);

        assert_eq!(violations.len(), 1);
        assert!(
            violations[0]
                .message
                .as_deref()
                .unwrap()
                .contains("below clearance")
        );
    }

    #[test]
    fn drill_spacing_includes_excellon_sidecar_hits() {
        let board_drills = vec![DrillFeature {
            location: [0.0, 0.0],
            diameter: 0.4,
            net: Some("GND".to_string()),
            plated: true,
        }];
        let excellon_drills = vec![DrillFeature {
            location: [0.5, 0.0],
            diameter: 0.3,
            net: None,
            plated: false,
        }];

        let violations = drill_spacing(&board_drills, &excellon_drills, 0.20);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].locations.len(), 2);
    }

    #[test]
    fn board_outline_drill_clearance_reports_hole_near_edge() {
        let outline = sketch(vec![square(0.0, 0.0, 10.0, 10.0)]);
        let drills = vec![DrillFeature {
            location: [0.4, 5.0],
            diameter: 0.4,
            net: None,
            plated: false,
        }];

        let violations = board_outline_drill_clearance(
            "KiCad drills",
            "KiCad Edge.Cuts",
            &outline,
            &drills,
            &[],
            0.25,
            1.0e-9,
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-drill-clearance");
    }

    #[test]
    fn board_outline_drill_clearance_allows_inset_hole() {
        let outline = sketch(vec![square(0.0, 0.0, 10.0, 10.0)]);
        let drills = vec![DrillFeature {
            location: [1.0, 5.0],
            diameter: 0.4,
            net: None,
            plated: false,
        }];

        assert!(
            board_outline_drill_clearance(
                "KiCad drills",
                "KiCad Edge.Cuts",
                &outline,
                &drills,
                &[],
                0.25,
                1.0e-9,
            )
            .is_empty()
        );
    }

    #[test]
    fn board_outline_drill_clearance_includes_excellon_sidecar_drills() {
        let outline = sketch(vec![square(0.0, 0.0, 10.0, 10.0)]);
        let extra_drills = vec![DrillFeature {
            location: [9.8, 5.0],
            diameter: 0.4,
            net: None,
            plated: false,
        }];

        let violations = board_outline_drill_clearance(
            "KiCad plus Excellon drills",
            "KiCad Edge.Cuts",
            &outline,
            &[],
            &extra_drills,
            0.25,
            1.0e-9,
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].locations, vec![[9.8, 5.0]]);
    }

    #[test]
    fn drill_spacing_flags_conservative_slot_keepouts() {
        let rectangular_slots = vec![
            DrillFeature {
                location: [0.0, 0.0],
                diameter: 1.8,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [2.0, 0.0],
                diameter: 1.7,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [5.0, 0.0],
                diameter: 1.0,
                net: None,
                plated: false,
            },
        ];

        let violations = drill_spacing(&rectangular_slots, &[], 0.30);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "drill-spacing");
    }

    #[test]
    fn panelization_clearance_flags_copper_near_panel_features_and_stamp_holes() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![copper_disc("SIG", CopperKind::Pad, [0.12, 0.0], 0.08)],
            drills: vec![DrillFeature {
                location: [1.0, 0.0],
                diameter: 0.2,
                net: None,
                plated: false,
            }],
            board_outline: None,
            panel_features: Some(polygons_to_sketch(
                vec![line_polygon([0.0, -1.0], [0.0, 1.0], 0.05).unwrap()],
                Some(LayerMetadata {
                    name: "KiCad Panel".to_string(),
                }),
            )),
        };
        let extra_drills = vec![DrillFeature {
            location: [0.2, 0.0],
            diameter: 0.2,
            net: None,
            plated: false,
        }];

        let violations = panelization_clearance(&board, &extra_drills, 0.25, 1.0e-9);

        assert_eq!(violations.len(), 2);
    }

    #[test]
    fn panelization_clearance_flags_copper_near_tab_route_and_v_score() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![
                copper_disc("TAB", CopperKind::Pad, [0.0, 0.0], 0.08),
                copper_disc("VSCORE", CopperKind::Pad, [2.0, 0.0], 0.08),
            ],
            drills: Vec::new(),
            board_outline: None,
            panel_features: Some(polygons_to_sketch(
                vec![
                    line_polygon([0.0, -1.0], [0.0, 1.0], 0.05).unwrap(),
                    line_polygon([2.0, -1.0], [2.0, 1.0], 0.05).unwrap(),
                ],
                Some(LayerMetadata {
                    name: "KiCad panel features".to_string(),
                }),
            )),
        };

        let violations = panelization_clearance(&board, &[], 0.25, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "panelization-clearance");
    }

    #[test]
    fn registration_tolerance_flags_close_features_on_different_layers() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![
                copper_disc_on_layer("TOP", CopperKind::Pad, "F.Cu", [0.0, 0.0], 0.2),
                copper_disc_on_layer("BOT", CopperKind::Pad, "B.Cu", [0.3, 0.0], 0.2),
                copper_disc_on_layer("INNER", CopperKind::Pad, "In1.Cu", [2.0, 0.0], 0.2),
            ],
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        };

        let violations = registration_tolerance(&board, 0.15, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].layers, vec!["B.Cu", "F.Cu"]);
    }

    #[test]
    fn ipc356_points_annotate_nearby_copper() {
        let mut board = BoardModel {
            source: "test".to_string(),
            copper: vec![CopperFeature {
                layer: "F.Cu".to_string(),
                net: None,
                kind: CopperKind::Pad,
                location: [1.0, 2.0],
                sketch: polygons_to_sketch(
                    vec![circle_polygon([1.0, 2.0], 0.5, 32)],
                    Some(LayerMetadata {
                        name: "feature".to_string(),
                    }),
                ),
            }],
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        };
        let points = vec![Ipc356Point {
            net: "GND".to_string(),
            reference: Some("U1".to_string()),
            pin: Some("1".to_string()),
            location: [1.02, 2.0],
            diameter: None,
        }];

        apply_ipc356_nets(&mut board, &points, 0.1);

        assert_eq!(board.copper[0].net.as_deref(), Some("GND"));
        assert!(ipc356_coverage(&board, &points, 0.1).is_empty());
    }

    #[test]
    fn ipc356_points_recover_missing_drill_net_and_diameter() {
        let mut board = BoardModel {
            source: "test".to_string(),
            copper: Vec::new(),
            drills: vec![DrillFeature {
                location: [1.0, 2.0],
                diameter: 0.0,
                net: None,
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };
        let points = vec![Ipc356Point {
            net: "PWR".to_string(),
            reference: Some("TP1".to_string()),
            pin: None,
            location: [1.01, 2.0],
            diameter: Some(0.45),
        }];

        apply_ipc356_nets(&mut board, &points, 0.1);

        assert_eq!(board.drills[0].net.as_deref(), Some("PWR"));
        assert_eq!(board.drills[0].diameter, 0.45);
    }

    #[test]
    fn ipc356_coverage_reports_missing_test_record_copper() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: Vec::new(),
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        };
        let points = vec![Ipc356Point {
            net: "N/C".to_string(),
            reference: Some("J1".to_string()),
            pin: Some("2".to_string()),
            location: [10.0, 20.0],
            diameter: None,
        }];

        let violations = ipc356_coverage(&board, &points, 0.1);

        assert_eq!(violations.len(), 1);
        assert!(violations[0].message.as_deref().unwrap().contains("J1.2"));
    }

    #[test]
    fn ipc356_drill_diameter_reports_conflicting_drill_table_data() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: Vec::new(),
            drills: vec![DrillFeature {
                location: [1.0, 2.0],
                diameter: 0.30,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };
        let points = vec![Ipc356Point {
            net: "GND".to_string(),
            reference: Some("V1".to_string()),
            pin: None,
            location: [1.01, 2.0],
            diameter: Some(0.50),
        }];

        let violations = ipc356_drill_diameter(&board, &points, 0.05);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "ipc356-drill-diameter");
    }

    #[test]
    fn ipc356_drill_diameter_allows_matching_drills_within_tolerance() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: Vec::new(),
            drills: vec![DrillFeature {
                location: [1.0, 2.0],
                diameter: 0.30,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };
        let points = vec![Ipc356Point {
            net: "GND".to_string(),
            reference: Some("V1".to_string()),
            pin: None,
            location: [1.01, 2.0],
            diameter: Some(0.31),
        }];

        assert!(ipc356_drill_diameter(&board, &points, 0.05).is_empty());
    }

    fn feature(net: &str, location: [f64; 2]) -> CopperFeature {
        copper_disc(net, CopperKind::Pad, location, 0.5)
    }

    fn board_with_copper(copper: Vec<CopperFeature>) -> BoardModel {
        BoardModel {
            source: "test".to_string(),
            copper,
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        }
    }

    fn sketch(polygons: Vec<Polygon<f64>>) -> PcbSketch {
        polygons_to_sketch(
            polygons,
            Some(LayerMetadata {
                name: "outline".to_string(),
            }),
        )
    }

    fn square(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Polygon<f64> {
        Polygon::new(
            LineString::from(vec![
                Coord { x: min_x, y: min_y },
                Coord { x: max_x, y: min_y },
                Coord { x: max_x, y: max_y },
                Coord { x: min_x, y: max_y },
                Coord { x: min_x, y: min_y },
            ]),
            Vec::new(),
        )
    }

    fn copper_disc(net: &str, kind: CopperKind, location: [f64; 2], radius: f64) -> CopperFeature {
        copper_disc_on_layer(net, kind, "F.Cu", location, radius)
    }

    fn copper_disc_on_layer(
        net: &str,
        kind: CopperKind,
        layer: &str,
        location: [f64; 2],
        radius: f64,
    ) -> CopperFeature {
        CopperFeature {
            layer: layer.to_string(),
            net: Some(net.to_string()),
            kind,
            location,
            sketch: polygons_to_sketch(
                vec![circle_polygon(location, radius, 32)],
                Some(LayerMetadata {
                    name: "feature".to_string(),
                }),
            ),
        }
    }

    fn copper_line(
        net: &str,
        kind: CopperKind,
        start: [f64; 2],
        end: [f64; 2],
        width: f64,
    ) -> CopperFeature {
        CopperFeature {
            layer: "F.Cu".to_string(),
            net: Some(net.to_string()),
            kind,
            location: [(start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0],
            sketch: polygons_to_sketch(
                vec![line_polygon(start, end, width).unwrap()],
                Some(LayerMetadata {
                    name: "feature".to_string(),
                }),
            ),
        }
    }
}
