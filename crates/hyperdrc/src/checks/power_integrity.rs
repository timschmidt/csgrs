//! Power-integrity readiness checks around high-current pad entry geometry.
//!
//! These checks look for layout patterns that often need engineering review
//! before release: connector/regulator pads carrying power through narrow,
//! single-entry copper rather than a pour, parallel vias, or a wide neck.
//!
//! Reliability note: these checks infer current intent from net names and
//! estimate conductor support from parsed 2D copper. They are release-readiness
//! prompts, not ampacity, temperature-rise, or electromigration calculations.

use super::distance::polygon_boundary_distance_scalar;
use super::spatial::CopperSpatialIndex;
use crate::geometry::Rect;
use crate::kicad::{BoardModel, CopperFeature, CopperKind};
use crate::report::{Severity, Violation};
use crate::{PcbRegion, PcbRegionExt, Scalar};

/// Warn when a likely high-current pad has weak local same-net copper support.
///
/// The check treats a high-current pad as supported when nearby same-net copper
/// includes a zone/pour, enough parallel vias, or a segment whose approximate
/// width meets the preferred entry width. IPC-2221B and IPC-2152 frame copper
/// width, copper area, and thermal rise as design constraints. Same-layer
/// support candidates use a deterministic grid before exact boundary-distance
/// review.
pub fn power_pad_entry_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    support_distance: &Scalar,
    minimum_entry_width: &Scalar,
    minimum_parallel_vias: usize,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let broad_phase_distance = scalar_broad_phase_radius(support_distance);
    let feature_index = CopperSpatialIndex::new(&features, broad_phase_distance);
    let mut candidate_pads = 0usize;
    let mut candidate_supports = 0usize;
    let mut exact_supports = 0usize;
    let mut violations = Vec::new();

    for pad in features
        .iter()
        .copied()
        .filter(|feature| feature.kind == CopperKind::Pad)
    {
        let Some(net) = &pad.net else {
            continue;
        };
        if !looks_high_current_net(net) || looks_ground_net(net) {
            continue;
        }
        candidate_pads += 1;
        let (support, support_candidates, support_exact) = local_pad_support(
            pad,
            &features,
            &feature_index,
            support_distance,
            broad_phase_distance,
        );
        candidate_supports += support_candidates;
        exact_supports += support_exact;
        if support.has_zone
            || support.via_count >= minimum_parallel_vias
            || crate::scalar::ge(&support.maximum_segment_width, minimum_entry_width)
        {
            continue;
        }

        violations.push(Violation::new(
            "power-pad-entry-readiness",
            Severity::Warning,
            vec![pad.layer.clone()],
            None,
            Vec::new(),
            vec![pad.location_f64_compatibility_required()],
            Some(format!(
                "likely high-current pad on net {net} has no local same-net pour, only {} nearby same-net via(s), and widest entry segment {:.6} below preferred width {:.6}; review connector/regulator pad current density and copper spreading",
                support.via_count,
                support.maximum_segment_width,
                minimum_entry_width
            )),
        ));
    }

    log::trace!(
        "power pad entry readiness: source={} features={} buckets={} candidate_pads={} candidate_supports={} exact_support_checks={} selected_layers={} support_distance={:.6} min_width={:.6} min_vias={} violations={}",
        board.source,
        features.len(),
        feature_index.bucket_count(),
        candidate_pads,
        candidate_supports,
        exact_supports,
        selected_layers.len(),
        support_distance,
        minimum_entry_width,
        minimum_parallel_vias,
        violations.len()
    );
    debug_assert!(exact_supports <= candidate_supports);

    violations
}

/// Warn when likely high-current vias lack a nearby same-layer return feature.
///
/// This check complements `power-via-array-readiness`: via arrays address
/// current sharing, while this readiness proxy checks local return geometry near
/// each high-current via. IPC-2221B and IPC-2152 frame conductor geometry and
/// current capacity as layout constraints.
///
/// Ground-return candidates use `CopperSpatialIndex` as a conservative broad
/// phase; the
/// return decision still uses exact polygon boundary distance.
pub fn power_via_return_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    return_distance: &Scalar,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let return_features = features
        .iter()
        .copied()
        .filter(|feature| feature.net.as_deref().is_some_and(looks_ground_net))
        .collect::<Vec<_>>();
    let broad_phase_distance = scalar_broad_phase_radius(return_distance);
    let return_index = CopperSpatialIndex::new(&return_features, broad_phase_distance);
    let mut candidate_vias = 0usize;
    let mut candidate_returns = 0usize;
    let mut exact_returns = 0usize;
    let mut violations = Vec::new();

    for via in features
        .iter()
        .copied()
        .filter(|feature| feature.kind == CopperKind::Via)
    {
        let Some(net) = &via.net else {
            continue;
        };
        if !looks_high_current_net(net) || looks_ground_net(net) {
            continue;
        }
        candidate_vias += 1;
        let (has_return, return_candidates, return_exact) = has_nearby_return(
            via,
            &return_features,
            &return_index,
            return_distance,
            broad_phase_distance,
        );
        candidate_returns += return_candidates;
        exact_returns += return_exact;
        if has_return {
            continue;
        }

        violations.push(Violation::new(
            "power-via-return-readiness",
            Severity::Warning,
            vec![via.layer.clone()],
            None,
            Vec::new(),
            vec![via.location_f64_compatibility_required()],
            Some(format!(
                "likely high-current via on net {net} has no parsed same-layer ground return copper within {return_distance:.6}; review power-loop area, decoupling return, and stitching intent"
            )),
        ));
    }

    log::trace!(
        "power via return readiness: source={} candidate_vias={} return_features={} return_buckets={} candidate_returns={} exact_return_checks={} selected_layers={} return_distance={:.6} violations={}",
        board.source,
        candidate_vias,
        return_features.len(),
        return_index.bucket_count(),
        candidate_returns,
        exact_returns,
        selected_layers.len(),
        return_distance,
        violations.len()
    );
    debug_assert!(exact_returns <= candidate_returns);

    violations
}

struct PadSupport {
    has_zone: bool,
    via_count: usize,
    maximum_segment_width: Scalar,
}

impl Default for PadSupport {
    fn default() -> Self {
        Self {
            has_zone: false,
            via_count: 0,
            maximum_segment_width: Scalar::zero(),
        }
    }
}

fn local_pad_support(
    pad: &CopperFeature,
    features: &[&CopperFeature],
    feature_index: &CopperSpatialIndex<'_>,
    support_distance: &Scalar,
    broad_phase_distance: f64,
) -> (PadSupport, usize, usize) {
    let mut support = PadSupport::default();
    let Some(pad_bounds) = pad.region.to_multipolygon().bounding_rect() else {
        return (support, 0, 0);
    };
    let pad_geometry = pad.region.to_multipolygon();
    let candidates = feature_index.same_layer_near_feature(pad, broad_phase_distance);
    let candidate_count = candidates.len();
    let mut exact_count = 0usize;

    for feature_index in candidates {
        let feature = features[feature_index];
        if std::ptr::eq(pad, feature)
            || feature.net != pad.net
            || !matches!(
                feature.kind,
                CopperKind::Segment | CopperKind::Zone | CopperKind::Via
            )
        {
            continue;
        }
        let Some(feature_bounds) = feature.region.to_multipolygon().bounding_rect() else {
            continue;
        };
        if !expanded_rects_overlap(&pad_bounds, &feature_bounds, broad_phase_distance) {
            continue;
        }
        exact_count += 1;
        if polygon_boundary_distance_scalar(&pad_geometry, &feature.region.to_multipolygon())
            .is_none_or(|distance| crate::scalar::gt(&distance, support_distance))
        {
            continue;
        }

        match feature.kind {
            CopperKind::Zone => support.has_zone = true,
            CopperKind::Via => support.via_count += 1,
            CopperKind::Segment => {
                let width = minimum_bounding_dimension_scalar(&feature.region);
                if crate::scalar::gt(&width, &support.maximum_segment_width) {
                    support.maximum_segment_width = width;
                }
            }
            CopperKind::Pad | CopperKind::Artwork => {}
        }
    }

    (support, candidate_count, exact_count)
}

fn has_nearby_return(
    via: &CopperFeature,
    return_features: &[&CopperFeature],
    return_index: &CopperSpatialIndex<'_>,
    return_distance: &Scalar,
    broad_phase_distance: f64,
) -> (bool, usize, usize) {
    let Some(via_bounds) = via.region.to_multipolygon().bounding_rect() else {
        return (false, 0, 0);
    };
    let via_geometry = via.region.to_multipolygon();
    let candidates = return_index.same_layer_near_feature(via, broad_phase_distance);
    let candidate_count = candidates.len();
    let mut exact_count = 0usize;

    let mut has_return = false;
    for feature_index in candidates {
        let feature = return_features[feature_index];
        let Some(feature_bounds) = feature.region.to_multipolygon().bounding_rect() else {
            continue;
        };
        if !expanded_rects_overlap(&via_bounds, &feature_bounds, broad_phase_distance) {
            continue;
        }

        exact_count += 1;
        if polygon_boundary_distance_scalar(&via_geometry, &feature.region.to_multipolygon())
            .is_some_and(|distance| crate::scalar::le(&distance, return_distance))
        {
            has_return = true;
            break;
        }
    }

    (has_return, candidate_count, exact_count)
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

fn minimum_bounding_dimension_scalar(region: &PcbRegion) -> Scalar {
    if let Some(bounds) = region.exact_bounds() {
        let width = &bounds[2] - &bounds[0];
        let height = &bounds[3] - &bounds[1];
        return if crate::scalar::le(&width, &height) {
            width
        } else {
            height
        };
    }
    let bounds = region.bounding_box();
    let width = &bounds.maxs.x - &bounds.mins.x;
    let height = &bounds.maxs.y - &bounds.mins.y;
    if crate::scalar::le(&width, &height) {
        width
    } else {
        height
    }
}

fn looks_high_current_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "VBAT", "VBUS", "VIN", "VCC", "VDD", "VOUT", "PWR", "POWER", "MOTOR", "PHASE", "+12V",
        "+5V", "+3V3", "12V", "5V", "3V3", "1V8",
    ];

    tokens.iter().any(|token| normalized.contains(token))
}

fn looks_ground_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    matches!(
        normalized.as_str(),
        "GND" | "GROUND" | "PGND" | "AGND" | "DGND"
    ) || normalized.ends_with("_GND")
        || normalized.ends_with("-GND")
}

fn expanded_rects_overlap(left: &Rect<f64>, right: &Rect<f64>, expansion: f64) -> bool {
    left.min().x - expansion <= right.max().x
        && left.max().x + expansion >= right.min().x
        && left.min().y - expansion <= right.max().y
        && left.max().y + expansion >= right.min().y
}

fn scalar_broad_phase_radius(value: &Scalar) -> f64 {
    let projected = value
        .to_f64_lossy()
        .expect("power-integrity broad-phase radius must fit the finite compatibility index");
    if projected > 0.0 {
        projected.next_up()
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::LayerMetadata;
    use crate::geometry::{circle_polygon, line_polygon, polygons_to_profile, rect_polygon};

    fn board(copper: Vec<CopperFeature>) -> BoardModel {
        BoardModel {
            source: "unit".to_string(),
            copper,
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        }
    }

    fn pad(net: &str, location: [f64; 2], size: [f64; 2]) -> CopperFeature {
        CopperFeature {
            layer: "F.Cu".to_string(),
            net: Some(net.to_string()),
            kind: CopperKind::Pad,
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
            region: polygons_to_profile(
                vec![rect_polygon(location, size, 0.0)],
                Some(LayerMetadata {
                    name: "test pad".to_string(),
                }),
            ),
        }
    }

    fn segment(net: &str, start: [f64; 2], end: [f64; 2], width: f64) -> CopperFeature {
        CopperFeature {
            layer: "F.Cu".to_string(),
            net: Some(net.to_string()),
            kind: CopperKind::Segment,
            location: [
                crate::geometry::exact_real((start[0] + end[0]) / 2.0),
                crate::geometry::exact_real((start[1] + end[1]) / 2.0),
            ],
            region: polygons_to_profile(
                vec![line_polygon(start, end, width).expect("test segment should be valid")],
                Some(LayerMetadata {
                    name: "test segment".to_string(),
                }),
            ),
        }
    }

    fn zone(net: &str, location: [f64; 2], size: [f64; 2]) -> CopperFeature {
        CopperFeature {
            layer: "F.Cu".to_string(),
            net: Some(net.to_string()),
            kind: CopperKind::Zone,
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
            region: polygons_to_profile(
                vec![rect_polygon(location, size, 0.0)],
                Some(LayerMetadata {
                    name: "test zone".to_string(),
                }),
            ),
        }
    }

    fn via(net: &str, location: [f64; 2]) -> CopperFeature {
        CopperFeature {
            layer: "F.Cu".to_string(),
            net: Some(net.to_string()),
            kind: CopperKind::Via,
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
            region: polygons_to_profile(
                vec![circle_polygon(location, 0.10, 32)],
                Some(LayerMetadata {
                    name: "test via".to_string(),
                }),
            ),
        }
    }

    #[test]
    fn power_pad_entry_reports_unsupported_high_current_pad() {
        let board = board(vec![
            pad("VIN", [0.0, 0.0], [1.0, 1.0]),
            segment("VIN", [0.5, 0.0], [2.0, 0.0], 0.12),
        ]);

        let violations = power_pad_entry_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.20"),
            &crate::scalar::scalar("0.30"),
            2,
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "power-pad-entry-readiness");
    }

    #[test]
    fn power_pad_entry_allows_pour_wide_segment_or_parallel_vias() {
        let poured = board(vec![
            pad("VIN", [0.0, 0.0], [1.0, 1.0]),
            zone("VIN", [0.7, 0.0], [1.0, 1.0]),
        ]);
        let wide = board(vec![
            pad("VIN", [0.0, 0.0], [1.0, 1.0]),
            segment("VIN", [0.5, 0.0], [2.0, 0.0], 0.40),
        ]);
        let vias = board(vec![
            pad("VIN", [0.0, 0.0], [1.0, 1.0]),
            via("VIN", [0.2, 0.0]),
            via("VIN", [-0.2, 0.0]),
        ]);

        assert!(
            power_pad_entry_readiness(
                &poured,
                &[],
                &crate::scalar::scalar("0.20"),
                &crate::scalar::scalar("0.30"),
                2,
            )
            .is_empty()
        );
        assert!(
            power_pad_entry_readiness(
                &wide,
                &[],
                &crate::scalar::scalar("0.20"),
                &crate::scalar::scalar("0.30"),
                2,
            )
            .is_empty()
        );
        assert!(
            power_pad_entry_readiness(
                &vias,
                &[],
                &crate::scalar::scalar("0.20"),
                &crate::scalar::scalar("0.30"),
                2,
            )
            .is_empty()
        );
    }

    #[test]
    fn power_pad_entry_ignores_ground_and_low_current_pads() {
        let board = board(vec![
            pad("GND", [0.0, 0.0], [1.0, 1.0]),
            pad("GPIO_LED", [2.0, 0.0], [1.0, 1.0]),
        ]);

        let violations = power_pad_entry_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.20"),
            &crate::scalar::scalar("0.30"),
            2,
        );

        assert!(violations.is_empty());
    }

    #[test]
    fn power_pad_entry_respects_selected_layers() {
        let board = board(vec![pad("VIN", [0.0, 0.0], [1.0, 1.0])]);

        let violations = power_pad_entry_readiness(
            &board,
            &[String::from("B.Cu")],
            &crate::scalar::scalar("0.20"),
            &crate::scalar::scalar("0.30"),
            2,
        );

        assert!(violations.is_empty());
    }

    #[test]
    fn power_pad_entry_culls_sparse_support_features() {
        let mut copper = vec![pad("VIN", [0.0, 0.0], [1.0, 1.0])];
        for index in 0..2_000 {
            copper.push(segment(
                "VIN",
                [
                    100.0 + (index % 100) as f64 * 2.0,
                    100.0 + (index / 100) as f64 * 2.0,
                ],
                [
                    101.0 + (index % 100) as f64 * 2.0,
                    100.0 + (index / 100) as f64 * 2.0,
                ],
                0.50,
            ));
        }
        let board = board(copper);

        let started = std::time::Instant::now();
        let violations = power_pad_entry_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.20"),
            &crate::scalar::scalar("0.30"),
            2,
        );

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "power-pad entry should index sparse support features before exact distance review"
        );
    }

    #[test]
    fn power_via_return_reports_high_current_via_without_return() {
        let board = board(vec![
            via("VIN", [0.0, 0.0]),
            segment("GND", [2.0, 0.0], [3.0, 0.0], 0.20),
        ]);

        let violations = power_via_return_readiness(&board, &[], &crate::scalar::scalar("0.50"));

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "power-via-return-readiness");
    }

    #[test]
    fn power_via_return_allows_nearby_ground_return() {
        let board = board(vec![
            via("VIN", [0.0, 0.0]),
            segment("GND", [0.25, 0.0], [1.0, 0.0], 0.20),
        ]);

        let violations = power_via_return_readiness(&board, &[], &crate::scalar::scalar("0.50"));

        assert!(violations.is_empty());
    }

    #[test]
    fn power_via_return_ignores_low_current_ground_or_selected_out_vias() {
        let board = board(vec![
            via("GPIO_LED", [0.0, 0.0]),
            via("GND", [1.0, 0.0]),
            via("VIN", [2.0, 0.0]),
        ]);

        assert_eq!(
            power_via_return_readiness(&board, &[], &crate::scalar::scalar("0.50")).len(),
            1
        );
        assert!(
            power_via_return_readiness(
                &board,
                &[String::from("B.Cu")],
                &crate::scalar::scalar("0.50"),
            )
            .is_empty()
        );
    }

    #[test]
    fn power_via_return_culls_sparse_return_features() {
        let mut copper = vec![via("VIN", [0.0, 0.0])];
        for index in 0..2_000 {
            copper.push(segment(
                "GND",
                [
                    100.0 + (index % 100) as f64 * 2.0,
                    100.0 + (index / 100) as f64 * 2.0,
                ],
                [
                    101.0 + (index % 100) as f64 * 2.0,
                    100.0 + (index / 100) as f64 * 2.0,
                ],
                0.20,
            ));
        }
        let board = board(copper);

        let started = std::time::Instant::now();
        let violations = power_via_return_readiness(&board, &[], &crate::scalar::scalar("0.50"));

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "power-via return should index sparse ground returns before exact distance review"
        );
    }
}
