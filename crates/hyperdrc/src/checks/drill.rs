//! Drill, hole, slot, and castellation readiness checks.
//!
//! This module owns checks where the primary object is a KiCad, Excellon, or
//! IPC-D-356 drill record. Keeping them separate from board-wide electrical
//! checks makes mechanical fabrication rules easier to find and extend.

use csgrs::csg::CSG;
use geo::Area;

use crate::geometry::{circle_polygon, multipolygon_to_shapes, polygons_to_sketch};
use crate::ipc356::Ipc356Point;
use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
use crate::report::{Severity, Violation};
use crate::{LayerMetadata, PcbSketch};

/// Run the `annular_ring` design-readiness check or report helper.
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
        // remain shape-agnostic. IPC-2221B and IPC-6012D both treat annular ring
        // as a finished-hole-to-land registration margin; exact containment can
        // be tightened once pad stack drill spans are modeled.
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

/// Run the `annular_ring_tolerance` design-readiness check or report helper.
pub fn annular_ring_tolerance(
    board: &BoardModel,
    minimum_ring: f64,
    registration_tolerance: f64,
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

        let copper_radius = equivalent_radius(&nearest.sketch);
        let nominal_ring = copper_radius - drill.diameter / 2.0;
        let worst_case_ring = nominal_ring - registration_tolerance;
        if nominal_ring >= minimum_ring && worst_case_ring < minimum_ring {
            violations.push(Violation::new(
                "annular-ring-tolerance",
                Severity::Warning,
                vec![nearest.layer.clone()],
                None,
                Vec::new(),
                vec![drill.location],
                Some(format!(
                    "nominal annular ring {nominal_ring:.6} passes minimum {minimum_ring:.6}, but worst-case ring {worst_case_ring:.6} after tolerance {registration_tolerance:.6} does not"
                )),
            ));
        }
    }

    violations
}

/// Run the `plating_intent` design-readiness check or report helper.
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

/// Run the `routed_slot_readiness` design-readiness check or report helper.
pub fn routed_slot_readiness(board: &BoardModel, minimum_route_width: f64) -> Vec<Violation> {
    board
        .drills
        .iter()
        .filter(|drill| !drill.plated && drill.diameter > 0.0 && drill.diameter < minimum_route_width)
        .map(|drill| {
            Violation::new(
                "routed-slot-readiness",
                Severity::Warning,
                vec!["KiCad NPTH drills".to_string()],
                None,
                Vec::new(),
                vec![drill.location],
                Some(format!(
                    "non-plated mechanical drill diameter {:.6} is below minimum route width {:.6}; review routed slot or cutter capability",
                    drill.diameter, minimum_route_width
                )),
            )
        })
        .collect()
}

/// Run the `drill_to_copper_clearance` design-readiness check or report helper.
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

/// Run the `drill_spacing` design-readiness check or report helper.
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

/// Run the `board_outline_drill_clearance` design-readiness check or report helper.
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

/// Run the `castellation_intent` design-readiness check or report helper.
pub fn castellation_intent(board: &BoardModel, min_area: f64) -> Vec<Violation> {
    let Some(outline) = &board.board_outline else {
        return Vec::new();
    };
    let mut violations = Vec::new();

    for drill in &board.drills {
        if !drill.plated {
            continue;
        }

        let hole = polygons_to_sketch(
            vec![circle_polygon(drill.location, drill.diameter / 2.0, 64)],
            Some(LayerMetadata {
                name: "plated drill hole".to_string(),
            }),
        );
        let outside_outline = hole.difference(outline);
        let shapes = multipolygon_to_shapes(&outside_outline.to_multipolygon(), min_area);
        if shapes.is_empty() {
            continue;
        }

        violations.push(Violation::new(
            "castellation-intent",
            Severity::Warning,
            vec![board.source.clone(), "KiCad Edge.Cuts".to_string()],
            None,
            shapes,
            vec![drill.location],
            Some(
                "plated drill hole crosses the board outline; confirm castellation or plated-edge intent"
                    .to_string(),
            ),
        ));
    }

    violations
}

/// Run the `castellation_hole_readiness` design-readiness check or report helper.
pub fn castellation_hole_readiness(
    board: &BoardModel,
    minimum_diameter: f64,
    min_area: f64,
) -> Vec<Violation> {
    let Some(outline) = &board.board_outline else {
        return Vec::new();
    };
    let mut violations = Vec::new();

    for drill in &board.drills {
        if !drill.plated || drill.diameter >= minimum_diameter {
            continue;
        }

        let hole = polygons_to_sketch(
            vec![circle_polygon(drill.location, drill.diameter / 2.0, 64)],
            Some(LayerMetadata {
                name: "plated drill hole".to_string(),
            }),
        );
        let outside_outline = hole.difference(outline);
        let shapes = multipolygon_to_shapes(&outside_outline.to_multipolygon(), min_area);
        if shapes.is_empty() {
            continue;
        }

        violations.push(Violation::new(
            "castellation-hole-readiness",
            Severity::Warning,
            vec![board.source.clone(), "KiCad Edge.Cuts".to_string()],
            None,
            shapes,
            vec![drill.location],
            Some(format!(
                "plated drill crossing the board outline has diameter {:.6} below minimum castellation diameter {:.6}",
                drill.diameter, minimum_diameter
            )),
        ));
    }

    violations
}

/// Run the `drill_aspect_ratio` design-readiness check or report helper.
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

/// Run the `drill_table_consistency` design-readiness check or report helper.
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

/// Run the `drills_to_sketch` design-readiness check or report helper.
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
            "{left_source} diameter {left_diameter:.6} conflicts with {right_source} diameter {right_diameter:.6}"
        )),
    )
}

fn distance(left: [f64; 2], right: [f64; 2]) -> f64 {
    let dx = left[0] - right[0];
    let dy = left[1] - right[1];
    (dx * dx + dy * dy).sqrt()
}
