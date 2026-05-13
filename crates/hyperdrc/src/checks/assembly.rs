//! Assembly, fixture-access, and DFA readiness checks.
//!
//! These checks operate on parsed KiCad pads/drills plus optional sidecars and
//! focus on whether a board package is ready for placement, probing, tooling,
//! and fine-pitch assembly review.

use std::collections::{BTreeMap, BTreeSet};

use csgrs::csg::CSG;
use geo::{Area, BoundingRect};

use crate::checks::distance::polygon_boundary_distance;
use crate::geometry::{circle_polygon, multipolygon_to_shapes, polygons_to_sketch};
use crate::ipc356::Ipc356Point;
use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
use crate::report::{Severity, Violation};
use crate::{LayerMetadata, PcbSketch};

pub fn component_edge_clearance_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    clearance: f64,
) -> Vec<Violation> {
    let Some(outline) = &board.board_outline else {
        return Vec::new();
    };

    selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.kind == CopperKind::Pad)
        .filter(|feature| !likely_fiducial(feature))
        .filter(|feature| {
            !feature
                .net
                .as_deref()
                .is_some_and(looks_edge_intent_net)
        })
        .filter_map(|feature| {
            let edge_gap = polygon_boundary_distance(
                &feature.sketch.to_multipolygon(),
                &outline.to_multipolygon(),
            );
            (edge_gap < clearance).then(|| {
                Violation::new(
                    "component-edge-clearance-readiness",
                    Severity::Warning,
                    vec![feature.layer.clone()],
                    None,
                    Vec::new(),
                    vec![feature.location],
                    Some(format!(
                        "component pad is {edge_gap:.6} from board edge, below assembly edge clearance {clearance:.6}; review pick-and-place, depanelization, clamp, and rework access"
                    )),
                )
            })
        })
        .collect()
}

pub fn component_hole_clearance_readiness(
    board: &BoardModel,
    extra_drills: &[DrillFeature],
    selected_layers: &[String],
    clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let mut mechanical_drills = board
        .drills
        .iter()
        .chain(extra_drills.iter())
        .filter(|drill| !drill.plated)
        .collect::<Vec<_>>();
    mechanical_drills.sort_by(|left, right| {
        left.location[0]
            .total_cmp(&right.location[0])
            .then(left.location[1].total_cmp(&right.location[1]))
            .then(left.diameter.total_cmp(&right.diameter))
    });

    let pads = selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.kind == CopperKind::Pad)
        .filter(|feature| !likely_fiducial(feature))
        .collect::<Vec<_>>();
    let mut violations = Vec::new();

    for drill in mechanical_drills {
        let keepout = polygons_to_sketch(
            vec![circle_polygon(
                drill.location,
                drill.diameter / 2.0 + clearance,
                32,
            )],
            Some(LayerMetadata {
                name: "mechanical hole keepout".to_string(),
            }),
        );

        for pad in &pads {
            let overlap = keepout.intersection(&pad.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            let fallback_hit = shapes.is_empty()
                && polygon_boundary_distance(
                    &keepout.to_multipolygon(),
                    &pad.sketch.to_multipolygon(),
                ) <= 1.0e-9;
            if shapes.is_empty() && !fallback_hit {
                continue;
            }

            violations.push(Violation::new(
                "component-hole-clearance-readiness",
                Severity::Warning,
                vec![pad.layer.clone()],
                None,
                shapes,
                vec![drill.location, pad.location],
                Some(format!(
                    "component pad is within mechanical hole clearance {clearance:.6}; review screw, standoff, slot, chassis, or connector keepout"
                )),
            ));
        }
    }

    violations
}

pub fn connector_rework_clearance_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    clearance: f64,
    minimum_pad_dimension: f64,
) -> Vec<Violation> {
    let pads = selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.kind == CopperKind::Pad)
        .filter(|feature| !likely_fiducial(feature))
        .collect::<Vec<_>>();
    let connector_pads = pads
        .iter()
        .copied()
        .filter(|feature| feature.net.as_deref().is_some_and(looks_connector_net))
        .filter(|feature| minimum_bounding_dimension(&feature.sketch) >= minimum_pad_dimension)
        .collect::<Vec<_>>();
    let mut violations = Vec::new();

    for connector in connector_pads {
        for neighbor in &pads {
            if connector.layer != neighbor.layer || std::ptr::eq(connector, *neighbor) {
                continue;
            }
            if connector.net.is_some() && connector.net == neighbor.net {
                continue;
            }

            let gap = polygon_boundary_distance(
                &connector.sketch.to_multipolygon(),
                &neighbor.sketch.to_multipolygon(),
            );
            if gap >= clearance {
                continue;
            }

            violations.push(Violation::new(
                "connector-rework-clearance-readiness",
                Severity::Warning,
                vec![connector.layer.clone()],
                None,
                Vec::new(),
                vec![connector.location, neighbor.location],
                Some(format!(
                    "likely connector pad on net {:?} is {gap:.6} from neighboring pad, below rework clearance {clearance:.6}; review soldering iron and connector rework access",
                    connector.net
                )),
            ));
        }
    }

    violations
}

/// Warn when a likely two-terminal pad pair has asymmetric copper area.
///
/// The same wetting-force balance that drives tombstoning is sensitive to land
/// geometry: unequal pad areas change solder volume and wetting force. IPC-7351
/// land-pattern guidance and Eurocircuits' tombstoning notes both describe
/// symmetric pad geometry as a mitigation for chip resistors/capacitors.
pub fn pad_pair_asymmetry_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    max_pair_gap: f64,
    max_area_ratio: f64,
    max_pad_dimension: f64,
) -> Vec<Violation> {
    let pads = selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.kind == CopperKind::Pad)
        .filter(|feature| !likely_fiducial(feature))
        .filter_map(|feature| {
            let area = feature.sketch.to_multipolygon().unsigned_area();
            let (_, max_dimension) = bounding_dimensions(&feature.sketch)?;
            (area > 0.0 && max_dimension <= max_pad_dimension).then_some((
                feature,
                area,
                max_dimension,
            ))
        })
        .collect::<Vec<_>>();
    let mut violations = Vec::new();

    for left_index in 0..pads.len() {
        let (left, left_area, _) = pads[left_index];
        for (right, right_area, _) in &pads[(left_index + 1)..] {
            if left.layer != right.layer {
                continue;
            }
            if left.net.is_some() && left.net == right.net {
                continue;
            }

            let gap = polygon_boundary_distance(
                &left.sketch.to_multipolygon(),
                &right.sketch.to_multipolygon(),
            );
            if gap > max_pair_gap {
                continue;
            }

            let area_ratio = left_area.max(*right_area) / left_area.min(*right_area);
            if area_ratio <= max_area_ratio {
                continue;
            }

            violations.push(Violation::new(
                "pad-pair-asymmetry-readiness",
                Severity::Warning,
                vec![left.layer.clone()],
                None,
                Vec::new(),
                vec![left.location, right.location],
                Some(format!(
                    "neighboring small pads have copper area ratio {area_ratio:.3}, above {max_area_ratio:.3}; review two-terminal land pattern symmetry and tombstoning risk"
                )),
            ));
        }
    }

    violations
}

pub fn testpoint_coverage_readiness(
    board: &BoardModel,
    points: &[Ipc356Point],
    selected_layers: &[String],
) -> Vec<Violation> {
    let covered_nets = points
        .iter()
        .map(|point| normalize_net(&point.net))
        .collect::<BTreeSet<_>>();
    let mut required_nets: BTreeMap<String, Vec<[f64; 2]>> = BTreeMap::new();

    for feature in selected_copper_features(board, selected_layers) {
        let Some(net) = feature.net.as_deref() else {
            continue;
        };
        if !looks_testpoint_required_net(net) {
            continue;
        }
        required_nets
            .entry(net.to_string())
            .or_default()
            .push(feature.location);
    }

    required_nets
        .into_iter()
        .filter(|(net, _)| !covered_nets.contains(&normalize_net(net)))
        .map(|(net, locations)| {
            Violation::new(
                "testpoint-coverage-readiness",
                Severity::Warning,
                vec![net.clone()],
                None,
                Vec::new(),
                locations.into_iter().take(3).collect(),
                Some(format!(
                    "critical net {net:?} has parsed KiCad copper but no matching IPC-D-356 test record"
                )),
            )
        })
        .collect()
}

/// Check fixture-probe diameter, edge clearance, and nearest-neighbor spacing.
///
/// Bed-of-nails fixture guidance from FixturFab and IPC-9252-oriented electrical
/// test practices describe the same mechanical idea this check encodes: test
/// probes need reliable pad size, spacing, and fixture clearance so the probe
/// plate can contact every required net repeatably.
pub fn testpoint_accessibility_readiness(
    board: &BoardModel,
    points: &[Ipc356Point],
    minimum_diameter: f64,
    minimum_spacing: f64,
    edge_clearance: f64,
) -> Vec<Violation> {
    let mut violations = Vec::new();

    for point in points {
        match point.diameter {
            Some(diameter) if diameter < minimum_diameter => {
                violations.push(Violation::new(
                    "testpoint-accessibility-readiness",
                    Severity::Warning,
                    vec![format!("net:{}", point.net)],
                    None,
                    Vec::new(),
                    vec![point.location],
                    Some(format!(
                        "IPC-D-356 testpoint diameter {diameter:.6} is below minimum probe diameter {minimum_diameter:.6}"
                    )),
                ));
            }
            Some(_) => {}
            None => {
                violations.push(Violation::new(
                    "testpoint-accessibility-readiness",
                    Severity::Warning,
                    vec![format!("net:{}", point.net)],
                    None,
                    Vec::new(),
                    vec![point.location],
                    Some(
                        "IPC-D-356 testpoint has no parsed probe diameter; review fixture probe access"
                            .to_string(),
                    ),
                ));
            }
        }

        if let Some(outline) = &board.board_outline {
            let probe_diameter = point.diameter.unwrap_or(minimum_diameter);
            let probe = circle_polygon(point.location, probe_diameter / 2.0, 32);
            let probe_sketch = polygons_to_sketch(
                vec![probe],
                Some(LayerMetadata {
                    name: "IPC-D-356 testpoint probe".to_string(),
                }),
            );
            let edge_gap = polygon_boundary_distance(
                &probe_sketch.to_multipolygon(),
                &outline.to_multipolygon(),
            );
            if edge_gap < edge_clearance {
                violations.push(Violation::new(
                    "testpoint-accessibility-readiness",
                    Severity::Warning,
                    vec![format!("net:{}", point.net)],
                    None,
                    Vec::new(),
                    vec![point.location],
                    Some(format!(
                        "IPC-D-356 testpoint is {edge_gap:.6} from board edge, below fixture edge clearance {edge_clearance:.6}"
                    )),
                ));
            }
        }
    }

    for left_index in 0..points.len() {
        let left = &points[left_index];
        let Some(left_diameter) = left.diameter else {
            continue;
        };
        for right in &points[(left_index + 1)..] {
            let Some(right_diameter) = right.diameter else {
                continue;
            };
            let edge_gap = distance(left.location, right.location)
                - left_diameter / 2.0
                - right_diameter / 2.0;
            if edge_gap >= minimum_spacing {
                continue;
            }

            violations.push(Violation::new(
                "testpoint-accessibility-readiness",
                Severity::Warning,
                vec![format!("net:{}", left.net), format!("net:{}", right.net)],
                None,
                Vec::new(),
                vec![left.location, right.location],
                Some(format!(
                    "IPC-D-356 testpoint spacing {edge_gap:.6} is below fixture probe spacing {minimum_spacing:.6}"
                )),
            ));
        }
    }

    violations
}

pub fn tooling_hole_readiness(
    board: &BoardModel,
    extra_drills: &[DrillFeature],
    minimum_diameter: f64,
    maximum_diameter: f64,
    edge_clearance: f64,
) -> Vec<Violation> {
    let mut drills = board.drills.clone();
    drills.extend_from_slice(extra_drills);

    let candidates = drills
        .iter()
        .filter(|drill| !drill.plated)
        .filter(|drill| drill.diameter >= minimum_diameter && drill.diameter <= maximum_diameter)
        .collect::<Vec<_>>();
    let mut violations = Vec::new();

    if candidates.len() < 2 {
        violations.push(Violation::new(
            "tooling-hole-readiness",
            Severity::Warning,
            vec!["tooling-holes".to_string()],
            None,
            Vec::new(),
            candidates.iter().map(|drill| drill.location).collect(),
            Some(format!(
                "found {} likely tooling hole(s); assembly panels usually need at least two non-plated tooling holes between {minimum_diameter:.6} and {maximum_diameter:.6}",
                candidates.len()
            )),
        ));
    }

    if let Some(outline) = &board.board_outline {
        for drill in candidates {
            let keepout = polygons_to_sketch(
                vec![circle_polygon(drill.location, drill.diameter / 2.0, 32)],
                Some(LayerMetadata {
                    name: "tooling hole".to_string(),
                }),
            );
            let edge_gap =
                polygon_boundary_distance(&keepout.to_multipolygon(), &outline.to_multipolygon());
            if edge_gap >= edge_clearance {
                continue;
            }

            violations.push(Violation::new(
                "tooling-hole-readiness",
                Severity::Warning,
                vec!["tooling-holes".to_string()],
                None,
                Vec::new(),
                vec![drill.location],
                Some(format!(
                    "likely tooling hole is {edge_gap:.6} from board edge, below fixture edge clearance {edge_clearance:.6}"
                )),
            ));
        }
    }

    violations
}

pub fn mouse_bite_readiness(
    board: &BoardModel,
    extra_drills: &[DrillFeature],
    minimum_diameter: f64,
    maximum_diameter: f64,
    minimum_spacing: f64,
    maximum_spacing: f64,
) -> Vec<Violation> {
    let mut drills = board.drills.clone();
    drills.extend_from_slice(extra_drills);
    let candidates = drills
        .iter()
        .filter(|drill| !drill.plated && drill.diameter <= maximum_diameter)
        .collect::<Vec<_>>();
    let mut violations = Vec::new();

    for drill in &candidates {
        if drill.diameter >= minimum_diameter {
            continue;
        }

        violations.push(Violation::new(
            "mouse-bite-readiness",
            Severity::Warning,
            vec!["mouse-bites".to_string()],
            None,
            Vec::new(),
            vec![drill.location],
            Some(format!(
                "likely mouse-bite drill diameter {:.6} is below minimum {:.6}",
                drill.diameter, minimum_diameter
            )),
        ));
    }

    for (left_index, left) in candidates.iter().enumerate() {
        let Some((right, center_spacing)) = candidates
            .iter()
            .enumerate()
            .filter(|(right_index, _)| *right_index != left_index)
            .map(|(_, right)| (*right, distance(left.location, right.location)))
            .min_by(|left, right| left.1.total_cmp(&right.1))
        else {
            continue;
        };
        if center_spacing >= minimum_spacing && center_spacing <= maximum_spacing {
            continue;
        }

        violations.push(Violation::new(
            "mouse-bite-readiness",
            Severity::Warning,
            vec!["mouse-bites".to_string()],
            None,
            Vec::new(),
            vec![left.location, right.location],
            Some(format!(
                "likely mouse-bite drill center spacing {center_spacing:.6} is outside expected range {minimum_spacing:.6}..{maximum_spacing:.6}"
            )),
        ));
    }

    violations
}

pub fn fiducial_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    edge_clearance: f64,
) -> Vec<Violation> {
    let mut candidates_by_layer: BTreeMap<String, Vec<&CopperFeature>> = BTreeMap::new();
    for feature in selected_copper_features(board, selected_layers) {
        if likely_fiducial(feature) {
            candidates_by_layer
                .entry(feature.layer.clone())
                .or_default()
                .push(feature);
        }
    }

    let mut expected_layers = board
        .copper
        .iter()
        .filter(|feature| selected_layers.is_empty() || selected_layers.contains(&feature.layer))
        .map(|feature| feature.layer.clone())
        .collect::<BTreeSet<_>>();
    expected_layers.retain(|layer| layer == "F.Cu" || layer == "B.Cu");
    if expected_layers.is_empty() {
        expected_layers.extend(candidates_by_layer.keys().cloned());
    }

    let mut violations = Vec::new();
    for layer in expected_layers {
        let candidates = candidates_by_layer.get(&layer).cloned().unwrap_or_default();
        if candidates.len() < 2 {
            violations.push(Violation::new(
                "fiducial-readiness",
                Severity::Warning,
                vec![layer.clone()],
                None,
                Vec::new(),
                candidates.iter().map(|feature| feature.location).collect(),
                Some(format!(
                    "layer {layer} has {} likely fiducial(s); assembly usually expects at least two per populated side",
                    candidates.len()
                )),
            ));
        }

        if let Some(outline) = &board.board_outline {
            for candidate in candidates {
                let distance_to_edge = polygon_boundary_distance(
                    &candidate.sketch.to_multipolygon(),
                    &outline.to_multipolygon(),
                );
                if distance_to_edge >= edge_clearance {
                    continue;
                }
                violations.push(Violation::new(
                    "fiducial-readiness",
                    Severity::Warning,
                    vec![layer.clone()],
                    None,
                    Vec::new(),
                    vec![candidate.location],
                    Some(format!(
                        "likely fiducial is {:.6} from board edge, below clearance {:.6}",
                        distance_to_edge, edge_clearance
                    )),
                ));
            }
        }
    }

    violations
}

pub fn local_fiducial_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    pitch_threshold: f64,
    search_radius: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let fiducials = features
        .iter()
        .copied()
        .filter(|feature| likely_fiducial(feature))
        .collect::<Vec<_>>();
    let mut pads_by_layer: BTreeMap<String, Vec<&CopperFeature>> = BTreeMap::new();
    for feature in features {
        if feature.kind == CopperKind::Pad && !likely_fiducial(feature) {
            pads_by_layer
                .entry(feature.layer.clone())
                .or_default()
                .push(feature);
        }
    }

    let mut violations = Vec::new();
    for (layer, pads) in pads_by_layer {
        if pads.len() < 16 {
            continue;
        }
        let Some(min_pitch) = minimum_feature_pitch(&pads) else {
            continue;
        };
        if min_pitch > pitch_threshold {
            continue;
        }

        let cluster_center = average_location(&pads);
        let nearby_fiducials = fiducials
            .iter()
            .filter(|fiducial| fiducial.layer == layer)
            .filter(|fiducial| distance(fiducial.location, cluster_center) <= search_radius)
            .count();
        if nearby_fiducials >= 2 {
            continue;
        }

        violations.push(Violation::new(
            "local-fiducial-readiness",
            Severity::Warning,
            vec![layer],
            None,
            Vec::new(),
            vec![cluster_center],
            Some(format!(
                "dense pad cluster has minimum pitch {min_pitch:.6} but only {nearby_fiducials} likely local fiducial(s) within {search_radius:.6}; review local fiducials for fine-pitch assembly"
            )),
        ));
    }

    violations
}

pub fn dense_pad_escape_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    pitch_threshold: f64,
    via_search_radius: f64,
) -> Vec<Violation> {
    let mut pads_by_layer: BTreeMap<String, Vec<&CopperFeature>> = BTreeMap::new();
    let mut vias = Vec::new();
    for feature in selected_copper_features(board, selected_layers) {
        match feature.kind {
            CopperKind::Pad => pads_by_layer
                .entry(feature.layer.clone())
                .or_default()
                .push(feature),
            CopperKind::Via => vias.push(feature),
            CopperKind::Segment | CopperKind::Zone => {}
        }
    }

    let mut violations = Vec::new();
    for (layer, pads) in pads_by_layer {
        if pads.len() < 16 {
            continue;
        }
        let Some(min_pitch) = minimum_feature_pitch(&pads) else {
            continue;
        };
        if min_pitch > pitch_threshold {
            continue;
        }

        let cluster_center = average_location(&pads);
        let has_escape_via = vias
            .iter()
            .any(|via| distance(via.location, cluster_center) <= via_search_radius);
        if has_escape_via {
            continue;
        }

        violations.push(Violation::new(
            "dense-pad-escape-readiness",
            Severity::Warning,
            vec![layer],
            None,
            Vec::new(),
            vec![cluster_center],
            Some(format!(
                "dense pad cluster has minimum pitch {min_pitch:.6} with no parsed escape via within {via_search_radius:.6}; review BGA/fine-pitch escape strategy"
            )),
        ));
    }

    violations
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

fn minimum_bounding_dimension(sketch: &PcbSketch) -> f64 {
    sketch
        .geometry
        .bounding_rect()
        .map(|bounds| (bounds.max().x - bounds.min().x).min(bounds.max().y - bounds.min().y))
        .unwrap_or(0.0)
}

fn bounding_dimensions(sketch: &PcbSketch) -> Option<(f64, f64)> {
    sketch.geometry.bounding_rect().map(|bounds| {
        let width = bounds.max().x - bounds.min().x;
        let height = bounds.max().y - bounds.min().y;
        (width.min(height), width.max(height))
    })
}

fn likely_fiducial(feature: &CopperFeature) -> bool {
    if feature.kind != CopperKind::Pad || feature.net.is_some() {
        return false;
    }

    let Some(bounds) = feature.sketch.geometry.bounding_rect() else {
        return false;
    };
    let width = bounds.max().x - bounds.min().x;
    let height = bounds.max().y - bounds.min().y;
    let min_dimension = width.min(height);
    let max_dimension = width.max(height);

    min_dimension >= 0.5 && max_dimension <= 2.5 && min_dimension / max_dimension >= 0.75
}

fn minimum_feature_pitch(features: &[&CopperFeature]) -> Option<f64> {
    let mut min_pitch = f64::INFINITY;
    for index in 0..features.len() {
        for other in &features[(index + 1)..] {
            min_pitch = min_pitch.min(distance(features[index].location, other.location));
        }
    }

    min_pitch.is_finite().then_some(min_pitch)
}

fn average_location(features: &[&CopperFeature]) -> [f64; 2] {
    let mut sum = [0.0, 0.0];
    for feature in features {
        sum[0] += feature.location[0];
        sum[1] += feature.location[1];
    }
    [
        sum[0] / features.len() as f64,
        sum[1] / features.len() as f64,
    ]
}

fn distance(left: [f64; 2], right: [f64; 2]) -> f64 {
    let dx = left[0] - right[0];
    let dy = left[1] - right[1];
    (dx * dx + dy * dy).sqrt()
}

fn looks_edge_intent_net(net: &str) -> bool {
    looks_gold_finger_net(net) || looks_chassis_net(net)
}

fn looks_gold_finger_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    ["GOLD", "FINGER", "EDGE", "CARD_EDGE", "CONN_EDGE"]
        .iter()
        .any(|token| normalized.contains(token))
}

fn looks_chassis_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    matches!(
        normalized.as_str(),
        "CHASSIS" | "SHIELD" | "EARTH" | "PE" | "PROTECTIVE_EARTH"
    ) || normalized.ends_with("_SHIELD")
        || normalized.ends_with("-SHIELD")
        || normalized.contains("CHASSIS")
}

fn looks_connector_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    [
        "CONN",
        "CONNECTOR",
        "USB",
        "JACK",
        "SOCKET",
        "PLUG",
        "HEADER",
        "VBUS",
        "SHIELD",
        "CHASSIS",
        "CARD_EDGE",
        "EDGE_CONN",
    ]
    .iter()
    .any(|token| normalized.contains(token))
}

fn looks_testpoint_required_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "RESET", "RST", "BOOT", "JTAG", "SWD", "SWCLK", "SWDIO", "TCK", "TMS", "TDI", "TDO",
        "UART", "TXD", "RXD", "DEBUG", "PROG", "TEST",
    ];

    looks_ground_net(net)
        || looks_high_current_net(net)
        || looks_high_speed_net(net)
        || looks_high_voltage_net(net)
        || looks_sensitive_net(net)
        || looks_chassis_net(net)
        || tokens.iter().any(|token| normalized.contains(token))
}

fn looks_ground_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    matches!(
        normalized.as_str(),
        "GND" | "GROUND" | "PGND" | "AGND" | "DGND"
    ) || normalized.ends_with("_GND")
        || normalized.ends_with("-GND")
}

fn looks_high_current_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    [
        "VBAT", "VBUS", "VIN", "VCC", "VDD", "VOUT", "PWR", "POWER", "MOTOR", "PHASE", "+12V",
        "+5V", "+3V3", "12V", "5V", "3V3", "1V8",
    ]
    .iter()
    .any(|token| normalized.contains(token))
}

fn looks_high_speed_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    [
        "USB", "D+", "D-", "DP", "DM", "CLK", "CLOCK", "TX", "RX", "SERDES", "PCIE", "PCI", "MIPI",
        "LVDS", "HDMI", "ETH", "RGMII", "SGMII", "SATA", "CAN",
    ]
    .iter()
    .any(|token| normalized.contains(token))
}

fn looks_high_voltage_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    [
        "HV", "HIGHV", "MAINS", "LINE", "NEUTRAL", "LIVE", "VAC", "AC_L", "AC_N", "RECT", "BULK",
        "400V", "240V", "230V", "120V", "48V",
    ]
    .iter()
    .any(|token| normalized.contains(token))
}

fn looks_sensitive_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    [
        "RF", "ANT", "AUDIO", "MIC", "ADC", "DAC", "AIN", "AOUT", "ANALOG", "SENSE", "SNS", "XTAL",
        "CRYSTAL", "OSC",
    ]
    .iter()
    .any(|token| normalized.contains(token))
}

fn normalize_net(net: &str) -> String {
    net.trim().to_ascii_uppercase()
}

#[cfg(test)]
mod tests {
    use super::pad_pair_asymmetry_readiness;
    use crate::LayerMetadata;
    use crate::geometry::{polygons_to_sketch, rect_polygon};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind};

    #[test]
    fn pad_pair_asymmetry_readiness_reports_mismatched_neighbor_pads() {
        let board = board_with_copper(vec![
            copper_pad("A", [0.0, 0.0], 0.5, 0.5),
            copper_pad("B", [0.7, 0.0], 1.1, 0.5),
        ]);

        let violations = pad_pair_asymmetry_readiness(&board, &[], 0.3, 1.5, 2.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "pad-pair-asymmetry-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("tombstoning"))
        );
    }

    #[test]
    fn pad_pair_asymmetry_readiness_allows_balanced_distant_or_large_pads() {
        let balanced = board_with_copper(vec![
            copper_pad("A", [0.0, 0.0], 0.5, 0.5),
            copper_pad("B", [0.7, 0.0], 0.55, 0.5),
        ]);
        let distant = board_with_copper(vec![
            copper_pad("A", [0.0, 0.0], 0.5, 0.5),
            copper_pad("B", [2.0, 0.0], 1.1, 0.5),
        ]);
        let large_connector = board_with_copper(vec![
            copper_pad("A", [0.0, 0.0], 2.5, 0.5),
            copper_pad("B", [0.8, 0.0], 0.5, 0.5),
        ]);

        assert!(pad_pair_asymmetry_readiness(&balanced, &[], 0.3, 1.5, 2.0).is_empty());
        assert!(pad_pair_asymmetry_readiness(&distant, &[], 0.3, 1.5, 2.0).is_empty());
        assert!(pad_pair_asymmetry_readiness(&large_connector, &[], 0.3, 1.5, 2.0).is_empty());
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

    fn copper_pad(net: &str, location: [f64; 2], width: f64, height: f64) -> CopperFeature {
        CopperFeature {
            layer: "F.Cu".to_string(),
            net: Some(net.to_string()),
            kind: CopperKind::Pad,
            location,
            sketch: polygons_to_sketch(
                vec![rect_polygon(location, [width, height], 0.0)],
                Some(LayerMetadata {
                    name: "pad".to_string(),
                }),
            ),
        }
    }
}
