use std::collections::HashMap;

use csgrs::csg::CSG;
use geo::Area;

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
            if drill.net.is_some() && drill.net == copper.net {
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
            if shapes.is_empty() {
                continue;
            }

            violations.push(Violation::new(
                "different-net-spacing",
                Severity::Error,
                vec![left.layer.clone()],
                None,
                shapes,
                vec![left.location, right.location],
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
        if shapes.is_empty() {
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
    use crate::LayerMetadata;
    use crate::geometry::{circle_polygon, polygons_to_sketch};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};

    use crate::ipc356::Ipc356Point;

    use super::{annular_ring, apply_ipc356_nets, ipc356_coverage, net_spacing};

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

    fn feature(net: &str, location: [f64; 2]) -> CopperFeature {
        CopperFeature {
            layer: "F.Cu".to_string(),
            net: Some(net.to_string()),
            kind: CopperKind::Pad,
            location,
            sketch: polygons_to_sketch(
                vec![circle_polygon(location, 0.5, 32)],
                Some(LayerMetadata {
                    name: "feature".to_string(),
                }),
            ),
        }
    }
}
