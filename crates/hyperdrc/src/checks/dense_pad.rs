//! Dense fine-pitch pad and BGA readiness checks.
//!
//! These checks operate on parsed KiCad pad and via copper. They stay separate
//! from broader assembly checks because dense pad clusters share a specific set
//! of geometric predicates: cluster pitch, local fiducials, escape vias,
//! pad-to-via spacing, and pad-to-pad mask-web margin.

use std::collections::{BTreeMap, HashMap};

use csgrs::csg::CSG;
use geo::BoundingRect;

use crate::PcbSketch;
use crate::checks::distance::polygon_boundary_distance;
use crate::geometry::multipolygon_to_shapes;
use crate::kicad::{BoardModel, CopperFeature, CopperKind};
use crate::report::{Severity, Violation};

const DENSE_PAD_CLUSTER_MIN_PADS: usize = 16;

/// Run the `local_fiducial_readiness` design-readiness check or report helper.
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
        if pads.len() < DENSE_PAD_CLUSTER_MIN_PADS {
            continue;
        }
        let Some(min_pitch) = minimum_feature_pitch_within(&pads, pitch_threshold) else {
            continue;
        };

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

/// Run the `dense_pad_escape_readiness` design-readiness check or report helper.
pub fn dense_pad_escape_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    pitch_threshold: f64,
    via_search_radius: f64,
) -> Vec<Violation> {
    let (pads_by_layer, vias) = dense_pad_inputs(board, selected_layers);
    let mut violations = Vec::new();

    for (layer, pads) in pads_by_layer {
        let Some((min_pitch, cluster_center)) = dense_cluster_context(&pads, pitch_threshold)
        else {
            continue;
        };
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

/// Review via-to-pad spacing inside dense fine-pitch pad clusters.
///
/// Dense BGA/CSP breakouts often trade dogbone escape vias, via-in-pad, and
/// soldermask web limits against each other. This check is intentionally a
/// geometry readiness gate: it finds dense pad clusters, then reports nearby
/// vias whose copper boundary is closer than the configured clearance to any
/// pad in the cluster. Jonnalagadda, "Reliability of via-in-pad structures in
/// mechanical cycling fatigue," *Microelectronics Reliability* 42.2 (2002),
/// pp. 253-258, <https://doi.org/10.1016/S0026-2714(01)00136-6>, treats
/// via-in-pad as an HDI enabler for high-I/O BGA/CSP products while still
/// requiring reliability review of the surrounding structure. HyperDRC reports
/// close pad/via geometry for that review instead of assuming a specific
/// filled, capped, dogbone, or open-via fabrication process.
pub fn dense_pad_via_spacing_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    pitch_threshold: f64,
    via_search_radius: f64,
    min_via_clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    if min_via_clearance <= 0.0 {
        return Vec::new();
    }

    let (pads_by_layer, vias) = dense_pad_inputs(board, selected_layers);
    log::trace!(
        "dense pad/via spacing readiness: source={} layers={} vias={} pitch_threshold={pitch_threshold:.6}",
        board.source,
        pads_by_layer.len(),
        vias.len()
    );
    let mut violations = Vec::new();

    for (layer, pads) in pads_by_layer {
        let Some((min_pitch, cluster_center)) = dense_cluster_context(&pads, pitch_threshold)
        else {
            continue;
        };

        for via in vias
            .iter()
            .copied()
            .filter(|via| distance(via.location, cluster_center) <= via_search_radius)
        {
            let Some((pad, clearance)) = nearest_pad_to_via(&pads, via) else {
                continue;
            };
            if clearance >= min_via_clearance {
                continue;
            }

            let keepout = via.sketch.offset(min_via_clearance);
            let shapes = multipolygon_to_shapes(
                &keepout.intersection(&pad.sketch).to_multipolygon(),
                min_area,
            );
            violations.push(Violation::new(
                "dense-pad-via-spacing-readiness",
                Severity::Warning,
                vec![layer.clone(), via.layer.clone()],
                None,
                shapes,
                vec![pad.location, via.location, cluster_center],
                Some(format!(
                    "dense pad cluster has minimum pitch {min_pitch:.6}; nearest pad/via clearance {clearance:.6} is below {min_via_clearance:.6}, review BGA escape spacing, soldermask web, and via fill/cap intent"
                )),
            ));
        }
    }

    violations
}

/// Review solder-mask bridge margin between pads in dense fine-pitch clusters.
///
/// This is a copper-derived proxy for BGA mask manufacturability when a mask
/// layer is not available. It does not replace the layer-level
/// `solder-mask-opening-spacing` check, which should be preferred when actual
/// mask openings are parsed. The check exists because escape geometry can turn
/// nominal NSMD BGA pads into partial SMD exposure and change solder-joint
/// behavior; see Chin and Ramakrishna, "Impact of BGA Escape Trace Design on
/// Performance of Solder Joint," *SMTA International* (Cisco Systems), for a
/// thermal-cycling study of BGA escape trace design choices. HyperDRC therefore
/// reports low pad-to-pad mask-web margin as a release-review item rather than
/// inferring the final solder-mask artwork.
pub fn dense_pad_mask_bridge_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    pitch_threshold: f64,
    min_mask_web: f64,
) -> Vec<Violation> {
    if min_mask_web <= 0.0 {
        return Vec::new();
    }

    let mut pads_by_layer: BTreeMap<String, Vec<&CopperFeature>> = BTreeMap::new();
    for feature in selected_copper_features(board, selected_layers) {
        if feature.kind == CopperKind::Pad {
            pads_by_layer
                .entry(feature.layer.clone())
                .or_default()
                .push(feature);
        }
    }
    log::trace!(
        "dense pad mask-bridge readiness: source={} layers={} pitch_threshold={pitch_threshold:.6}",
        board.source,
        pads_by_layer.len()
    );

    let mut violations = Vec::new();
    for (layer, pads) in pads_by_layer {
        let Some((min_pitch, _)) = dense_cluster_context(&pads, pitch_threshold) else {
            continue;
        };
        let Some((left, right, clearance)) = nearest_feature_pair_within(&pads, min_mask_web)
        else {
            continue;
        };

        violations.push(Violation::new(
            "dense-pad-mask-bridge-readiness",
            Severity::Warning,
            vec![layer],
            None,
            Vec::new(),
            vec![left.location, right.location, average_location(&pads)],
            Some(format!(
                "dense pad cluster has minimum pitch {min_pitch:.6}; nearest pad copper spacing {clearance:.6} is below mask web {min_mask_web:.6}, review BGA solder-mask bridge and NSMD/SMD pad definition"
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

fn dense_pad_inputs<'a>(
    board: &'a BoardModel,
    selected_layers: &[String],
) -> (
    BTreeMap<String, Vec<&'a CopperFeature>>,
    Vec<&'a CopperFeature>,
) {
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

    (pads_by_layer, vias)
}

fn dense_cluster_context(pads: &[&CopperFeature], pitch_threshold: f64) -> Option<(f64, [f64; 2])> {
    if pads.len() < DENSE_PAD_CLUSTER_MIN_PADS {
        return None;
    }
    let min_pitch = minimum_feature_pitch_within(pads, pitch_threshold)?;
    Some((min_pitch, average_location(pads)))
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

fn minimum_feature_pitch_within(features: &[&CopperFeature], threshold: f64) -> Option<f64> {
    if threshold <= 0.0 {
        return None;
    }

    let mut min_pitch = f64::INFINITY;
    let mut grid: HashMap<(i64, i64), Vec<&CopperFeature>> = HashMap::new();
    for feature in features {
        let cell = pitch_cell(feature.location, threshold);
        for dx in -1..=1 {
            for dy in -1..=1 {
                let neighbor_cell = (cell.0 + dx, cell.1 + dy);
                let Some(candidates) = grid.get(&neighbor_cell) else {
                    continue;
                };
                for candidate in candidates {
                    let pitch = distance(feature.location, candidate.location);
                    if pitch <= threshold {
                        min_pitch = min_pitch.min(pitch);
                    }
                }
            }
        }
        grid.entry(cell).or_default().push(*feature);
    }

    min_pitch.is_finite().then_some(min_pitch)
}

fn pitch_cell(location: [f64; 2], cell_size: f64) -> (i64, i64) {
    (
        (location[0] / cell_size).floor() as i64,
        (location[1] / cell_size).floor() as i64,
    )
}

fn nearest_feature_pair_within<'a>(
    features: &[&'a CopperFeature],
    threshold: f64,
) -> Option<(&'a CopperFeature, &'a CopperFeature, f64)> {
    if threshold <= 0.0 {
        return None;
    }

    let mut bounded = features
        .iter()
        .filter_map(|feature| {
            feature
                .sketch
                .geometry
                .bounding_rect()
                .map(|bounds| (*feature, bounds))
        })
        .collect::<Vec<_>>();
    bounded.sort_by(|left, right| {
        left.1
            .min()
            .x
            .total_cmp(&right.1.min().x)
            .then(left.1.min().y.total_cmp(&right.1.min().y))
    });

    let mut nearest = None;
    for index in 0..bounded.len() {
        let (left, left_bounds) = bounded[index];
        for (right, right_bounds) in &bounded[(index + 1)..] {
            if right_bounds.min().x - left_bounds.max().x >= threshold {
                break;
            }
            if !rects_within_clearance(&left_bounds, right_bounds, threshold) {
                continue;
            }
            let clearance = copper_clearance(&left.sketch, &right.sketch);
            if clearance >= threshold {
                continue;
            }
            if nearest
                .as_ref()
                .is_none_or(|(_, _, current): &(_, _, f64)| clearance < *current)
            {
                nearest = Some((left, *right, clearance));
            }
        }
    }

    nearest
}

fn rects_within_clearance(left: &geo::Rect<f64>, right: &geo::Rect<f64>, clearance: f64) -> bool {
    left.min().x - clearance <= right.max().x
        && left.max().x + clearance >= right.min().x
        && left.min().y - clearance <= right.max().y
        && left.max().y + clearance >= right.min().y
}

fn nearest_pad_to_via<'a>(
    pads: &[&'a CopperFeature],
    via: &CopperFeature,
) -> Option<(&'a CopperFeature, f64)> {
    pads.iter()
        .copied()
        .map(|pad| (pad, copper_clearance(&pad.sketch, &via.sketch)))
        .min_by(|left, right| {
            left.1
                .partial_cmp(&right.1)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
}

fn copper_clearance(left: &PcbSketch, right: &PcbSketch) -> f64 {
    polygon_boundary_distance(&left.to_multipolygon(), &right.to_multipolygon())
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

#[cfg(test)]
mod tests {
    use super::{
        DENSE_PAD_CLUSTER_MIN_PADS, dense_pad_escape_readiness, dense_pad_mask_bridge_readiness,
        dense_pad_via_spacing_readiness, local_fiducial_readiness,
    };
    use crate::LayerMetadata;
    use crate::geometry::{circle_polygon, polygons_to_sketch, rect_polygon};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind};

    #[test]
    fn local_fiducial_readiness_reports_dense_clusters_without_nearby_fiducials() {
        let board = board_with_copper(dense_pad_cluster());

        let violations = local_fiducial_readiness(&board, &[], 0.8, 5.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "local-fiducial-readiness");
    }

    #[test]
    fn local_fiducial_readiness_accepts_nearby_local_fiducials_or_sparse_pads() {
        let mut copper = dense_pad_cluster();
        copper.push(fiducial("F.Cu", [-1.0, -1.0], 0.8));
        copper.push(fiducial("F.Cu", [2.5, -1.0], 0.8));
        assert!(local_fiducial_readiness(&board_with_copper(copper), &[], 0.8, 5.0).is_empty());

        let mut sparse = Vec::new();
        for index in 0..DENSE_PAD_CLUSTER_MIN_PADS {
            sparse.push(copper_pad(
                &format!("P{index}"),
                [index as f64 * 1.0, 0.0],
                0.20,
                0.20,
            ));
        }
        assert!(local_fiducial_readiness(&board_with_copper(sparse), &[], 0.8, 5.0).is_empty());
    }

    #[test]
    fn dense_pad_via_spacing_readiness_reports_close_escape_via() {
        let mut copper = dense_pad_cluster();
        copper.push(copper_via("ESC", [0.32, 0.0], 0.20));
        let board = board_with_copper(copper);

        let violations = dense_pad_via_spacing_readiness(&board, &[], 0.8, 2.0, 0.15, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "dense-pad-via-spacing-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("pad/via clearance"))
        );
    }

    #[test]
    fn dense_pad_via_spacing_readiness_allows_sparse_far_or_selected_out_vias() {
        let mut sparse = Vec::new();
        for index in 0..DENSE_PAD_CLUSTER_MIN_PADS {
            sparse.push(copper_pad(
                &format!("P{index}"),
                [index as f64 * 1.0, 0.0],
                0.20,
                0.20,
            ));
        }
        sparse.push(copper_via("ESC", [0.32, 0.0], 0.20));
        assert!(
            dense_pad_via_spacing_readiness(
                &board_with_copper(sparse),
                &[],
                0.8,
                2.0,
                0.15,
                1.0e-9
            )
            .is_empty()
        );

        let mut far = dense_pad_cluster();
        far.push(copper_via("ESC", [10.0, 10.0], 0.20));
        assert!(
            dense_pad_via_spacing_readiness(&board_with_copper(far), &[], 0.8, 2.0, 0.15, 1.0e-9)
                .is_empty()
        );

        let mut selected_out = dense_pad_cluster();
        selected_out.push(copper_via_on_layer("B.Cu", "ESC", [0.32, 0.0], 0.20));
        assert!(
            dense_pad_via_spacing_readiness(
                &board_with_copper(selected_out),
                &["F.Cu".to_string()],
                0.8,
                2.0,
                0.15,
                1.0e-9
            )
            .is_empty()
        );
    }

    #[test]
    fn dense_pad_mask_bridge_readiness_reports_tight_dense_pad_web() {
        let board = board_with_copper(dense_pad_cluster_with_size(0.45));

        let violations = dense_pad_mask_bridge_readiness(&board, &[], 0.8, 0.10);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "dense-pad-mask-bridge-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("mask web"))
        );
    }

    #[test]
    fn dense_pad_mask_bridge_readiness_allows_wider_web_sparse_or_selected_out_pads() {
        assert!(
            dense_pad_mask_bridge_readiness(
                &board_with_copper(dense_pad_cluster_with_size(0.25)),
                &[],
                0.8,
                0.10
            )
            .is_empty()
        );

        let mut sparse = Vec::new();
        for index in 0..DENSE_PAD_CLUSTER_MIN_PADS {
            sparse.push(copper_pad(
                &format!("P{index}"),
                [index as f64 * 1.0, 0.0],
                0.45,
                0.45,
            ));
        }
        assert!(
            dense_pad_mask_bridge_readiness(&board_with_copper(sparse), &[], 0.8, 0.10).is_empty()
        );

        let selected_out = dense_pad_cluster_with_size(0.45)
            .into_iter()
            .map(|mut pad| {
                pad.layer = "B.Cu".to_string();
                pad
            })
            .collect::<Vec<_>>();
        assert!(
            dense_pad_mask_bridge_readiness(
                &board_with_copper(selected_out),
                &["F.Cu".to_string()],
                0.8,
                0.10
            )
            .is_empty()
        );
    }

    #[test]
    fn dense_pad_escape_readiness_culls_large_sparse_pad_fields() {
        let mut copper = Vec::new();
        for index in 0..900 {
            copper.push(copper_pad(
                &format!("P{index}"),
                [(index % 30) as f64 * 2.0, (index / 30) as f64 * 2.0],
                0.20,
                0.20,
            ));
        }
        let board = board_with_copper(copper);

        let start = std::time::Instant::now();
        let violations = dense_pad_escape_readiness(&board, &[], 0.8, 2.0);

        assert!(violations.is_empty());
        assert!(
            start.elapsed() < std::time::Duration::from_secs(2),
            "large sparse dense-pad fields should not require all-pairs pitch checks"
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

    fn dense_pad_cluster() -> Vec<CopperFeature> {
        dense_pad_cluster_with_size(0.25)
    }

    fn dense_pad_cluster_with_size(size: f64) -> Vec<CopperFeature> {
        let mut copper = Vec::new();
        for x in 0..4 {
            for y in 0..4 {
                copper.push(copper_pad(
                    &format!("BGA_{x}_{y}"),
                    [x as f64 * 0.5, y as f64 * 0.5],
                    size,
                    size,
                ));
            }
        }
        copper
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

    fn fiducial(layer: &str, location: [f64; 2], diameter: f64) -> CopperFeature {
        CopperFeature {
            layer: layer.to_string(),
            net: None,
            kind: CopperKind::Pad,
            location,
            sketch: polygons_to_sketch(
                vec![rect_polygon(location, [diameter, diameter], 0.0)],
                Some(LayerMetadata {
                    name: "fiducial".to_string(),
                }),
            ),
        }
    }

    fn copper_via(net: &str, location: [f64; 2], diameter: f64) -> CopperFeature {
        copper_via_on_layer("F.Cu", net, location, diameter)
    }

    fn copper_via_on_layer(
        layer: &str,
        net: &str,
        location: [f64; 2],
        diameter: f64,
    ) -> CopperFeature {
        CopperFeature {
            layer: layer.to_string(),
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
