//! Same-net continuity readiness checks.
//!
//! These checks look for geometry patterns that can break an otherwise routed
//! net before bare-board electrical test. They intentionally stay separate from
//! generic drill clearance because the review question is continuity, not just
//! manufacturing spacing.
//!
//! Reliability note: this module uses parsed copper/drill geometry as a proxy
//! for true net connectivity. Verify any release-blocking result against KiCad
//! DRC, generated CAM data, and electrical test outputs.

use csgrs::csg::CSG;
use std::collections::BTreeMap;

use geo::{Area, BoundingRect};

use crate::geometry::{circle_polygon, multipolygon_to_shapes, polygons_to_sketch};
use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
use crate::report::{Severity, Violation};
use crate::{LayerMetadata, PcbSketch};

use super::spatial::CopperSpatialIndex;

/// Warn when different named nets have overlapping copper on the same layer.
///
/// IPC-9252B treats bare-board electrical test as both a continuity and
/// isolation proof. This readiness check implements the isolation side as a
/// direct geometry predicate: after cheap axis-aligned bounding-box culling, it
/// intersects same-layer copper features with different net names and reports
/// any non-trivial overlap. The broad/narrow phase follows the collision-query
/// pattern described by Lin and Canny, "A Fast Algorithm for Incremental
/// Distance Calculation", IEEE ICRA, 1991.
pub fn different_net_short_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    min_area: f64,
) -> Vec<Violation> {
    let mut by_layer: BTreeMap<String, Vec<(&CopperFeature, geo::Rect<f64>)>> = BTreeMap::new();
    for feature in selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.net.is_some())
    {
        let Some(bounds) = feature.sketch.geometry().bounding_rect() else {
            continue;
        };
        by_layer
            .entry(feature.layer.clone())
            .or_default()
            .push((feature, bounds));
    }
    for features in by_layer.values_mut() {
        features.sort_by(|left, right| {
            left.1
                .min()
                .x
                .total_cmp(&right.1.min().x)
                .then(left.1.min().y.total_cmp(&right.1.min().y))
        });
    }
    log::trace!(
        "different-net short readiness: source={} layers={} selected_layers={}",
        board.source,
        by_layer.len(),
        selected_layers.len()
    );

    let mut violations = Vec::new();
    for candidates in by_layer.values() {
        for left_index in 0..candidates.len() {
            let (left, left_bounds) = candidates[left_index];
            for (right, right_bounds) in &candidates[(left_index + 1)..] {
                if right_bounds.min().x > left_bounds.max().x {
                    break;
                }
                if left.net == right.net || !rects_overlap(&left_bounds, right_bounds) {
                    continue;
                }

                let overlap = left.sketch.intersection(&right.sketch);
                let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
                let has_area = !shapes.is_empty()
                    || overlap
                        .to_multipolygon()
                        .0
                        .iter()
                        .any(|polygon| polygon.unsigned_area() > 0.0);
                if !has_area {
                    continue;
                }

                violations.push(Violation::new(
                    "different-net-short-readiness",
                    Severity::Error,
                    vec![left.layer.clone()],
                    None,
                    shapes,
                    vec![left.location, right.location],
                    Some(format!(
                        "net {:?} overlaps net {:?} on {}; review isolation test, netlist parity, and copper assignment",
                        left.net, right.net, left.layer
                    )),
                ));
            }
        }
    }

    violations
}

/// Warn when a non-plated drill or slot keepout intersects netted copper.
///
/// IPC-9252B frames bare-board electrical test around proving continuity and
/// isolation on the fabricated substrate. This readiness check is a conservative
/// pre-test geometry proxy: if an unplated drill/slot cuts through a trace or
/// zone that belongs to a net, the resulting copper may be physically severed
/// even when the source net assignment still looks continuous.
pub fn same_net_drill_break_readiness(
    board: &BoardModel,
    extra_drills: &[DrillFeature],
    selected_layers: &[String],
    min_area: f64,
) -> Vec<Violation> {
    let drills = board
        .drills
        .iter()
        .chain(extra_drills.iter())
        .filter(|drill| !drill.plated)
        .collect::<Vec<_>>();
    if drills.is_empty() {
        return Vec::new();
    }

    let copper = selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.net.is_some())
        .filter(|feature| matches!(feature.kind, CopperKind::Segment | CopperKind::Zone))
        .collect::<Vec<_>>();
    let maximum_drill_radius = drills
        .iter()
        .map(|drill| drill.diameter / 2.0)
        .fold(0.0_f64, f64::max);
    // Drill-break review is a conservative continuity proxy for IPC-9252B
    // electrical-test risk. The grid broad phase follows Ericson, Real-Time
    // Collision Detection, 2005: it only proposes possible drill/copper
    // contacts, while exact CSG intersection below remains authoritative.
    let copper_index = CopperSpatialIndex::new(&copper, maximum_drill_radius);
    let mut violations = Vec::new();
    let mut candidate_pairs = 0_usize;
    let mut keepouts_built = 0_usize;
    for drill in &drills {
        let drill_radius = drill.diameter / 2.0;
        let mut drill_sketch = None;

        for candidate_index in copper_index.all_layers_near_circle(drill.location, drill_radius) {
            candidate_pairs += 1;
            let feature = copper[candidate_index];

            let drill_sketch = drill_sketch.get_or_insert_with(|| {
                keepouts_built += 1;
                drill_keepout_sketch(drill)
            });
            let overlap = drill_sketch.intersection(&feature.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            if shapes.is_empty()
                && !overlap
                    .to_multipolygon()
                    .0
                    .iter()
                    .any(|polygon| polygon.unsigned_area() > 0.0)
            {
                continue;
            }

            violations.push(Violation::new(
                "same-net-drill-break-readiness",
                Severity::Error,
                vec![feature.layer.clone()],
                None,
                shapes,
                vec![drill.location, feature.location],
                Some(format!(
                    "non-plated drill/slot intersects routed copper for net {:?}; review same-net continuity, zone refill, and bare-board electrical test",
                    feature.net
                )),
            ));
        }
    }

    log::trace!(
        "same-net drill break readiness: source={} netted_routing_features={} non_plated_drills={} spatial_buckets={} candidate_pairs={} keepouts_built={} violations={}",
        board.source,
        copper.len(),
        drills.len(),
        copper_index.bucket_count(),
        candidate_pairs,
        keepouts_built,
        violations.len()
    );

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

fn drill_keepout_sketch(drill: &DrillFeature) -> PcbSketch {
    polygons_to_sketch(
        vec![circle_polygon(drill.location, drill.diameter / 2.0, 32)],
        Some(LayerMetadata {
            name: "non-plated drill continuity keepout".to_string(),
        }),
    )
}

fn rects_overlap(left: &geo::Rect<f64>, right: &geo::Rect<f64>) -> bool {
    left.min().x <= right.max().x
        && left.max().x >= right.min().x
        && left.min().y <= right.max().y
        && left.max().y >= right.min().y
}

#[cfg(test)]
mod tests {
    use super::{different_net_short_readiness, same_net_drill_break_readiness};
    use crate::LayerMetadata;
    use crate::geometry::{circle_polygon, line_polygon, polygons_to_sketch, rect_polygon};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};

    #[test]
    fn different_net_short_reports_overlapping_named_nets() {
        let board = board_with_copper(vec![
            pad(Some("A"), [0.0, 0.0], [1.0, 1.0]),
            pad(Some("B"), [0.4, 0.0], [1.0, 1.0]),
        ]);

        let violations = different_net_short_readiness(&board, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "different-net-short-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("A") && message.contains("B"))
        );
    }

    #[test]
    fn different_net_short_allows_same_net_unnetted_distant_or_other_layer() {
        let same_net = board_with_copper(vec![
            pad(Some("A"), [0.0, 0.0], [1.0, 1.0]),
            pad(Some("A"), [0.4, 0.0], [1.0, 1.0]),
        ]);
        assert!(different_net_short_readiness(&same_net, &[], 1.0e-9).is_empty());

        let unnetted = board_with_copper(vec![
            pad(Some("A"), [0.0, 0.0], [1.0, 1.0]),
            pad(None, [0.4, 0.0], [1.0, 1.0]),
        ]);
        assert!(different_net_short_readiness(&unnetted, &[], 1.0e-9).is_empty());

        let distant = board_with_copper(vec![
            pad(Some("A"), [0.0, 0.0], [1.0, 1.0]),
            pad(Some("B"), [2.0, 0.0], [1.0, 1.0]),
        ]);
        assert!(different_net_short_readiness(&distant, &[], 1.0e-9).is_empty());

        let other_layer = board_with_copper(vec![
            pad(Some("A"), [0.0, 0.0], [1.0, 1.0]),
            pad_on_layer("B.Cu", Some("B"), [0.4, 0.0], [1.0, 1.0]),
        ]);
        assert!(different_net_short_readiness(&other_layer, &[], 1.0e-9).is_empty());
    }

    #[test]
    fn different_net_short_respects_selected_layers() {
        let board = board_with_copper(vec![
            pad_on_layer("B.Cu", Some("A"), [0.0, 0.0], [1.0, 1.0]),
            pad_on_layer("B.Cu", Some("B"), [0.4, 0.0], [1.0, 1.0]),
        ]);

        assert!(different_net_short_readiness(&board, &["F.Cu".to_string()], 1.0e-9).is_empty());
        assert_eq!(
            different_net_short_readiness(&board, &["B.Cu".to_string()], 1.0e-9).len(),
            1
        );
    }

    #[test]
    fn different_net_short_culls_large_sparse_feature_sets() {
        let mut copper = Vec::new();
        for index in 0..900 {
            copper.push(pad(
                Some(&format!("N{index}")),
                [10.0 + index as f64 * 2.0, 10.0],
                [0.5, 0.5],
            ));
        }
        copper.push(pad(Some("A"), [0.0, 0.0], [1.0, 1.0]));
        copper.push(pad(Some("B"), [0.4, 0.0], [1.0, 1.0]));
        let board = board_with_copper(copper);

        let start = std::time::Instant::now();
        let violations = different_net_short_readiness(&board, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed() < std::time::Duration::from_secs(2),
            "different-net short checks should cull distant copper by bounds"
        );
    }

    #[test]
    fn same_net_drill_break_reports_npth_cutting_segment() {
        let mut board = board_with_copper(vec![segment("SIG", [-1.0, 0.0], [1.0, 0.0], 0.30)]);
        board.drills = vec![npth([0.0, 0.0], 0.60)];

        let violations = same_net_drill_break_readiness(&board, &[], &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "same-net-drill-break-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("SIG"))
        );
    }

    #[test]
    fn same_net_drill_break_reports_sidecar_drill_cutting_zone() {
        let board = board_with_copper(vec![zone("PWR", [0.0, 0.0], [2.0, 2.0])]);
        let sidecar_drills = vec![npth([0.0, 0.0], 0.75)];

        let violations = same_net_drill_break_readiness(&board, &sidecar_drills, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].layers, vec!["F.Cu"]);
    }

    #[test]
    fn same_net_drill_break_allows_plated_unnetted_pads_distant_or_selected_out() {
        let mut plated = board_with_copper(vec![segment("SIG", [-1.0, 0.0], [1.0, 0.0], 0.30)]);
        plated.drills = vec![DrillFeature {
            location: [0.0, 0.0],
            diameter: 0.60,
            net: Some("SIG".to_string()),
            plated: true,
        }];
        assert!(same_net_drill_break_readiness(&plated, &[], &[], 1.0e-9).is_empty());

        let mut unnetted_pad = board_with_copper(vec![pad(None, [0.0, 0.0], [1.0, 1.0])]);
        unnetted_pad.drills = vec![npth([0.0, 0.0], 0.60)];
        assert!(same_net_drill_break_readiness(&unnetted_pad, &[], &[], 1.0e-9).is_empty());

        let mut distant = board_with_copper(vec![segment("SIG", [2.0, 0.0], [3.0, 0.0], 0.30)]);
        distant.drills = vec![npth([0.0, 0.0], 0.60)];
        assert!(same_net_drill_break_readiness(&distant, &[], &[], 1.0e-9).is_empty());

        let mut selected_out = board_with_copper(vec![segment_on_layer(
            "B.Cu",
            "SIG",
            [-1.0, 0.0],
            [1.0, 0.0],
            0.30,
        )]);
        selected_out.drills = vec![npth([0.0, 0.0], 0.60)];
        assert!(
            same_net_drill_break_readiness(&selected_out, &[], &["F.Cu".to_string()], 1.0e-9)
                .is_empty()
        );
    }

    #[test]
    fn same_net_drill_break_culls_large_sparse_feature_sets() {
        let mut copper = Vec::new();
        for index in 0..900 {
            let y = 5.0 + index as f64;
            copper.push(segment(&format!("N{index}"), [10.0, y], [12.0, y], 0.20));
        }
        copper.push(segment("SIG", [-1.0, 0.0], [1.0, 0.0], 0.30));
        let mut board = board_with_copper(copper);
        board.drills = vec![npth([0.0, 0.0], 0.60)];

        let start = std::time::Instant::now();
        let violations = same_net_drill_break_readiness(&board, &[], &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed() < std::time::Duration::from_secs(2),
            "continuity drill-break checks should cull distant copper by bounds"
        );
    }

    #[test]
    fn same_net_drill_break_culls_large_sparse_drill_fields() {
        let mut board = board_with_copper(vec![segment("SIG", [-1.0, 0.0], [1.0, 0.0], 0.30)]);
        board.drills = (0..2_000)
            .map(|index| npth([10.0 + index as f64 * 2.0, 10.0], 0.60))
            .collect();
        board.drills.push(npth([0.0, 0.0], 0.60));

        let start = std::time::Instant::now();
        let violations = same_net_drill_break_readiness(&board, &[], &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed() < std::time::Duration::from_secs(2),
            "continuity drill-break checks should not build exact keepout CSG for distant drills"
        );
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

    fn segment(net: &str, start: [f64; 2], end: [f64; 2], width: f64) -> CopperFeature {
        segment_on_layer("F.Cu", net, start, end, width)
    }

    fn segment_on_layer(
        layer: &str,
        net: &str,
        start: [f64; 2],
        end: [f64; 2],
        width: f64,
    ) -> CopperFeature {
        CopperFeature {
            layer: layer.to_string(),
            net: Some(net.to_string()),
            kind: CopperKind::Segment,
            location: [(start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0],
            sketch: polygons_to_sketch(
                vec![line_polygon(start, end, width).expect("segment should be valid")],
                Some(LayerMetadata {
                    name: "segment".to_string(),
                }),
            ),
        }
    }

    fn zone(net: &str, location: [f64; 2], size: [f64; 2]) -> CopperFeature {
        CopperFeature {
            layer: "F.Cu".to_string(),
            net: Some(net.to_string()),
            kind: CopperKind::Zone,
            location,
            sketch: polygons_to_sketch(
                vec![rect_polygon(location, size, 0.0)],
                Some(LayerMetadata {
                    name: "zone".to_string(),
                }),
            ),
        }
    }

    fn pad(net: Option<&str>, location: [f64; 2], size: [f64; 2]) -> CopperFeature {
        pad_on_layer("F.Cu", net, location, size)
    }

    fn pad_on_layer(
        layer: &str,
        net: Option<&str>,
        location: [f64; 2],
        size: [f64; 2],
    ) -> CopperFeature {
        CopperFeature {
            layer: layer.to_string(),
            net: net.map(str::to_string),
            kind: CopperKind::Pad,
            location,
            sketch: polygons_to_sketch(
                vec![rect_polygon(location, size, 0.0)],
                Some(LayerMetadata {
                    name: "pad".to_string(),
                }),
            ),
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

    #[allow(dead_code)]
    fn via(net: &str, location: [f64; 2], diameter: f64) -> CopperFeature {
        CopperFeature {
            layer: "F.Cu".to_string(),
            net: Some(net.to_string()),
            kind: CopperKind::Via,
            location,
            sketch: polygons_to_sketch(
                vec![circle_polygon(location, diameter / 2.0, 32)],
                Some(LayerMetadata {
                    name: "via".to_string(),
                }),
            ),
        }
    }
}
