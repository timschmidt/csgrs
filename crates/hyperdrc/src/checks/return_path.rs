//! Return-path continuity checks for high-speed copper.
//!
//! These checks use parsed copper geometry plus net-name intent to find review
//! targets that ordinary same-net connectivity checks do not see.
//!
//! Reliability note: split-plane findings infer the reference conductor from
//! same-layer ground zones and high-speed net names. That makes them useful for
//! default readiness sweeps, but suspect for waiver-quality decisions until the
//! stackup, adjacent reference layer, and source CAD constraints are reviewed.

use csgrs::csg::CSG;
use geo::BoundingRect;

use super::distance::polygon_boundary_distance;
use super::spatial::CopperSpatialIndex;
use crate::LayerMetadata;
use crate::geometry::{multipolygon_to_shapes, polygons_to_sketch};
use crate::kicad::{BoardModel, CopperFeature, CopperKind};
use crate::report::{Severity, Violation};

/// Warn when a likely high-speed segment crosses separated ground-zone islands.
///
/// A trace crossing a reference-plane split forces return current to detour and
/// increases loop area. IPC-2221B treats return-path control as part of high-
/// speed routing practice, and Bhargava et al., "DC-DC Buck Converter EMI
/// Reduction Using PCB Layout Modification", IEEE Transactions on
/// Electromagnetic Compatibility, 2011, demonstrates how PCB loop geometry
/// changes radiated emissions. The implementation keeps the geometry exact only
/// after an indexed broad phase; Ericson, *Real-Time Collision Detection*
/// (2005), gives the broad-phase grid pattern, while Lin and Canny, "A Fast
/// Algorithm for Incremental Distance Calculation", IEEE ICRA, 1991, is the
/// collision/distance-processing context for doing inexpensive spatial
/// rejection before exact work.
pub fn split_plane_crossing_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    search_distance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let ground_features = features
        .iter()
        .copied()
        .filter(|feature| {
            feature.kind == CopperKind::Zone && feature.net.as_deref().is_some_and(looks_ground_net)
        })
        .collect::<Vec<_>>();
    let ground_zones = ground_features
        .iter()
        .filter_map(|feature| {
            feature
                .sketch
                .to_multipolygon()
                .bounding_rect()
                .map(|bounds| GroundZone { feature, bounds })
        })
        .collect::<Vec<_>>();
    let indexed_ground_features = ground_zones
        .iter()
        .map(|zone| zone.feature)
        .collect::<Vec<_>>();
    let ground_index = CopperSpatialIndex::new(&indexed_ground_features, search_distance);

    let mut candidate_segments = 0usize;
    let mut candidate_ground_zones = 0usize;
    let mut exact_ground_zones = 0usize;
    let mut violations = Vec::new();
    for feature in &features {
        if feature.kind != CopperKind::Segment {
            continue;
        }
        let Some(net) = &feature.net else {
            continue;
        };
        if !looks_high_speed_net(net) {
            continue;
        }
        candidate_segments += 1;
        let Some(segment_bounds) = feature.sketch.to_multipolygon().bounding_rect() else {
            continue;
        };

        let segment_geometry = feature.sketch.to_multipolygon();
        let candidates = ground_index.same_layer_near_feature(feature, search_distance);
        candidate_ground_zones += candidates.len();
        let nearby = candidates
            .into_iter()
            .filter_map(|ground_index| ground_zones.get(ground_index))
            .filter(|zone| expanded_rects_overlap(&segment_bounds, &zone.bounds, search_distance))
            .filter(|zone| {
                exact_ground_zones += 1;
                polygon_boundary_distance(&segment_geometry, &zone.feature.sketch.to_multipolygon())
                    <= search_distance
            })
            .collect::<Vec<_>>();
        if nearby.len() < 2 || !has_separated_ground_islands(&nearby, search_distance) {
            continue;
        }

        let ground_polygons = nearby
            .iter()
            .flat_map(|zone| zone.feature.sketch.to_multipolygon().0)
            .collect::<Vec<_>>();
        let ground = polygons_to_sketch(
            ground_polygons,
            Some(LayerMetadata {
                name: "nearby KiCad ground zones".to_string(),
            }),
        );
        let uncovered = feature.sketch.difference(&ground);
        let shapes = multipolygon_to_shapes(&uncovered.to_multipolygon(), min_area);
        if shapes.is_empty() {
            continue;
        }

        violations.push(Violation::new(
            "split-plane-crossing-readiness",
            Severity::Warning,
            vec![feature.layer.clone(), "nearby KiCad ground zones".to_string()],
            None,
            shapes,
            vec![feature.location],
            Some(format!(
                "likely high-speed net {net} crosses between {} separated same-layer ground-zone islands; review reference-plane continuity, stitching, or route placement",
                nearby.len()
            )),
        ));
    }

    log::trace!(
        "split plane crossing readiness: source={} candidate_segments={} ground_zones={} ground_buckets={} candidate_ground_zones={} exact_ground_zones={} selected_layers={} search_distance={:.6} violations={}",
        board.source,
        candidate_segments,
        ground_zones.len(),
        ground_index.bucket_count(),
        candidate_ground_zones,
        exact_ground_zones,
        selected_layers.len(),
        search_distance,
        violations.len()
    );

    violations
}

/// Warn when likely high-speed copper has no nearby same-layer ground return.
///
/// This is a loop-area readiness proxy rather than a field solve: it looks for
/// high-speed segment or pad copper whose nearest parsed same-layer ground
/// copper exceeds the review distance. IPC-2221B frames high-speed return-path
/// control as layout intent, while Bhargava et al., "DC-DC Buck Converter EMI
/// Reduction Using PCB Layout Modification", IEEE Transactions on
/// Electromagnetic Compatibility, 2011, shows that small PCB loop-geometry
/// changes can materially change emissions. As with split-plane detection, the
/// check uses AABB rejection before exact polygon distance; Lin and Canny,
/// "A Fast Algorithm for Incremental Distance Calculation", IEEE ICRA, 1991,
/// is the distance-processing context for that staged approach. Sparse ground
/// lookup uses the deterministic grid broad phase described by Ericson,
/// *Real-Time Collision Detection* (2005), before exact distance review.
pub fn return_path_proximity_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    maximum_return_distance: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let ground_features = features
        .iter()
        .copied()
        .filter(|feature| feature.net.as_deref().is_some_and(looks_ground_net))
        .collect::<Vec<_>>();
    let ground_index = CopperSpatialIndex::new(&ground_features, maximum_return_distance);

    let mut candidate_features = 0usize;
    let mut exact_pairs = 0usize;
    let mut violations = Vec::new();
    for feature in &features {
        if feature.kind == CopperKind::Via {
            continue;
        }
        let Some(net) = &feature.net else {
            continue;
        };
        if looks_ground_net(net) || !looks_high_speed_net(net) {
            continue;
        }
        candidate_features += 1;
        let candidate_ground_indexes =
            ground_index.same_layer_near_feature(feature, maximum_return_distance);
        if candidate_ground_indexes.is_empty() {
            violations.push(return_path_proximity_violation(
                feature,
                net,
                maximum_return_distance,
                None,
            ));
            continue;
        };

        let feature_geometry = feature.sketch.to_multipolygon();
        let nearest_distance = candidate_ground_indexes
            .into_iter()
            .map(|ground_index| {
                exact_pairs += 1;
                polygon_boundary_distance(
                    &feature_geometry,
                    &ground_features[ground_index].sketch.to_multipolygon(),
                )
            })
            .fold(f64::INFINITY, f64::min);
        if nearest_distance <= maximum_return_distance {
            continue;
        }

        violations.push(return_path_proximity_violation(
            feature,
            net,
            maximum_return_distance,
            nearest_distance.is_finite().then_some(nearest_distance),
        ));
    }

    log::trace!(
        "return path proximity readiness: source={} candidate_features={} ground_features={} ground_buckets={} exact_pairs={} selected_layers={} max_distance={:.6} violations={}",
        board.source,
        candidate_features,
        ground_features.len(),
        ground_index.bucket_count(),
        exact_pairs,
        selected_layers.len(),
        maximum_return_distance,
        violations.len()
    );

    violations
}

fn return_path_proximity_violation(
    feature: &CopperFeature,
    net: &str,
    maximum_return_distance: f64,
    nearest_distance: Option<f64>,
) -> Violation {
    let distance_detail = nearest_distance
        .map(|distance| format!("nearest parsed ground is {distance:.6} away"))
        .unwrap_or_else(|| "no parsed same-layer ground candidate was found".to_string());

    Violation::new(
        "return-path-proximity-readiness",
        Severity::Warning,
        vec![feature.layer.clone()],
        None,
        Vec::new(),
        vec![feature.location],
        Some(format!(
            "likely high-speed net {net} has {:?} copper without nearby same-layer ground return; {distance_detail}, above review distance {maximum_return_distance:.6}",
            feature.kind
        )),
    )
}

#[derive(Clone, Copy)]
struct GroundZone<'a> {
    feature: &'a CopperFeature,
    bounds: geo::Rect<f64>,
}

fn has_separated_ground_islands(zones: &[&GroundZone<'_>], search_distance: f64) -> bool {
    for (index, left) in zones.iter().enumerate() {
        let left_geometry = left.feature.sketch.to_multipolygon();
        for right in zones.iter().skip(index + 1) {
            if !expanded_rects_overlap(&left.bounds, &right.bounds, search_distance) {
                return true;
            }
            let distance =
                polygon_boundary_distance(&left_geometry, &right.feature.sketch.to_multipolygon());
            if distance > search_distance {
                return true;
            }
        }
    }

    false
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

fn looks_ground_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    normalized == "GND"
        || normalized == "GROUND"
        || normalized.starts_with("GND_")
        || normalized.starts_with("GND-")
        || normalized.contains("AGND")
        || normalized.contains("DGND")
        || normalized.contains("PGND")
        || normalized.contains("SHIELD")
        || normalized.contains("CHASSIS")
}

fn looks_high_speed_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "USB", "D+", "D-", "DP", "DM", "CLK", "CLOCK", "TX", "RX", "SERDES", "PCIE", "PCI", "MIPI",
        "LVDS", "HDMI", "ETH", "RGMII", "SGMII", "SATA", "CAN",
    ];

    tokens.iter().any(|token| normalized.contains(token))
}

fn expanded_rects_overlap(left: &geo::Rect<f64>, right: &geo::Rect<f64>, expansion: f64) -> bool {
    left.min().x - expansion <= right.max().x
        && left.max().x + expansion >= right.min().x
        && left.min().y - expansion <= right.max().y
        && left.max().y + expansion >= right.min().y
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use super::*;
    use crate::geometry::{line_polygon, rect_polygon};

    fn board(copper: Vec<CopperFeature>) -> BoardModel {
        BoardModel {
            source: "unit".to_string(),
            copper,
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        }
    }

    fn segment(net: &str, start: [f64; 2], end: [f64; 2], width: f64) -> CopperFeature {
        CopperFeature {
            layer: "F.Cu".to_string(),
            net: Some(net.to_string()),
            kind: CopperKind::Segment,
            location: [(start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0],
            sketch: polygons_to_sketch(
                vec![line_polygon(start, end, width).expect("test segment should be valid")],
                Some(LayerMetadata {
                    name: "test segment".to_string(),
                }),
            ),
        }
    }

    fn zone(net: &str, center: [f64; 2], size: [f64; 2]) -> CopperFeature {
        CopperFeature {
            layer: "F.Cu".to_string(),
            net: Some(net.to_string()),
            kind: CopperKind::Zone,
            location: center,
            sketch: polygons_to_sketch(
                vec![rect_polygon(center, size, 0.0)],
                Some(LayerMetadata {
                    name: "test zone".to_string(),
                }),
            ),
        }
    }

    #[test]
    fn split_plane_crossing_reports_high_speed_segment_across_gap() {
        let board = board(vec![
            zone("GND", [-1.25, 0.0], [1.5, 1.0]),
            zone("GND", [1.25, 0.0], [1.5, 1.0]),
            segment("USB_DP", [-2.0, 0.0], [2.0, 0.0], 0.10),
        ]);

        let violations = split_plane_crossing_readiness(&board, &[], 0.05, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "split-plane-crossing-readiness");
        assert!(!violations[0].polygons.is_empty());
    }

    #[test]
    fn split_plane_crossing_allows_continuous_ground_zone() {
        let board = board(vec![
            zone("GND", [0.0, 0.0], [4.5, 1.0]),
            segment("USB_DP", [-2.0, 0.0], [2.0, 0.0], 0.10),
        ]);

        let violations = split_plane_crossing_readiness(&board, &[], 0.05, 1.0e-9);

        assert!(violations.is_empty());
    }

    #[test]
    fn split_plane_crossing_ignores_low_speed_segment() {
        let board = board(vec![
            zone("GND", [-1.25, 0.0], [1.5, 1.0]),
            zone("GND", [1.25, 0.0], [1.5, 1.0]),
            segment("GPIO_LED", [-2.0, 0.0], [2.0, 0.0], 0.10),
        ]);

        let violations = split_plane_crossing_readiness(&board, &[], 0.05, 1.0e-9);

        assert!(violations.is_empty());
    }

    #[test]
    fn split_plane_crossing_respects_selected_layers() {
        let board = board(vec![
            zone("GND", [-1.25, 0.0], [1.5, 1.0]),
            zone("GND", [1.25, 0.0], [1.5, 1.0]),
            segment("USB_DP", [-2.0, 0.0], [2.0, 0.0], 0.10),
        ]);

        let violations =
            split_plane_crossing_readiness(&board, &[String::from("B.Cu")], 0.05, 1.0e-9);

        assert!(violations.is_empty());
    }

    #[test]
    fn split_plane_crossing_culls_sparse_large_cases() {
        let mut copper = vec![segment("USB_DP", [-2.0, 0.0], [2.0, 0.0], 0.10)];
        for index in 0..2_000 {
            copper.push(zone(
                "GND",
                [
                    100.0 + (index % 100) as f64 * 3.0,
                    100.0 + (index / 100) as f64 * 3.0,
                ],
                [1.0, 1.0],
            ));
        }
        let board = board(copper);

        let started = Instant::now();
        let violations = split_plane_crossing_readiness(&board, &[], 0.05, 1.0e-9);

        assert!(violations.is_empty());
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "split-plane crossing should index sparse ground zones before exact distance review"
        );
    }

    #[test]
    fn return_path_proximity_reports_high_speed_segment_without_ground() {
        let board = board(vec![
            segment("USB_DP", [0.0, 0.0], [1.0, 0.0], 0.10),
            segment("GND", [4.0, 0.0], [5.0, 0.0], 0.10),
        ]);

        let violations = return_path_proximity_readiness(&board, &[], 0.50);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "return-path-proximity-readiness");
    }

    #[test]
    fn return_path_proximity_allows_nearby_ground() {
        let board = board(vec![
            segment("USB_DP", [0.0, 0.0], [1.0, 0.0], 0.10),
            segment("GND", [0.0, 0.30], [1.0, 0.30], 0.10),
        ]);

        let violations = return_path_proximity_readiness(&board, &[], 0.50);

        assert!(violations.is_empty());
    }

    #[test]
    fn return_path_proximity_ignores_vias_ground_and_low_speed_nets() {
        let board = board(vec![
            segment("GPIO_LED", [0.0, 0.0], [1.0, 0.0], 0.10),
            segment("GND", [3.0, 0.0], [4.0, 0.0], 0.10),
            CopperFeature {
                layer: "F.Cu".to_string(),
                net: Some("USB_DP".to_string()),
                kind: CopperKind::Via,
                location: [8.0, 0.0],
                sketch: polygons_to_sketch(
                    vec![rect_polygon([8.0, 0.0], [0.20, 0.20], 0.0)],
                    Some(LayerMetadata {
                        name: "test via".to_string(),
                    }),
                ),
            },
        ]);

        let violations = return_path_proximity_readiness(&board, &[], 0.50);

        assert!(violations.is_empty());
    }

    #[test]
    fn return_path_proximity_respects_selected_layers() {
        let board = board(vec![segment("USB_DP", [0.0, 0.0], [1.0, 0.0], 0.10)]);

        let violations = return_path_proximity_readiness(&board, &[String::from("B.Cu")], 0.50);

        assert!(violations.is_empty());
    }

    #[test]
    fn return_path_proximity_culls_sparse_large_cases() {
        let mut copper = vec![segment("USB_DP", [0.0, 0.0], [1.0, 0.0], 0.10)];
        for index in 0..2_000 {
            copper.push(segment(
                "GND",
                [100.0 + index as f64 * 3.0, 100.0],
                [101.0 + index as f64 * 3.0, 100.0],
                0.10,
            ));
        }
        let board = board(copper);
        let start = Instant::now();

        let violations = return_path_proximity_readiness(&board, &[], 0.50);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "return-path proximity should index sparse same-layer ground fields"
        );
    }
}
