//! RF launch, via-fence, and antenna keepout readiness checks.
//!
//! These checks operate on parsed KiCad copper and use net-name heuristics to
//! find likely RF feeds and antenna regions. They intentionally stay separate
//! from broad board checks because RF review is driven by different geometry:
//! same-layer coupling, nearby copper-free regions, and parsed ground-stitching
//! evidence around launches and feedlines.
//!
//! Reliability note: RF intent inferred from names like `RF` or `ANT` is
//! suspect for custom naming schemes, shields, and intentional copper near
//! radiators. Verify findings against the RF layout plan or measured constraints.

use csgrs::csg::CSG;
use geo::BoundingRect;

use crate::PcbSketchExt;
use crate::Scalar;
use crate::checks::distance::polygon_boundary_distance_scalar;
use crate::checks::offset_for_check;
use crate::checks::spatial::CopperSpatialIndex;
use crate::geometry::multipolygon_to_shapes_scalar;
use crate::kicad::{BoardModel, CopperFeature, CopperKind};
use crate::report::{Severity, Violation};

/// Warn when likely RF/antenna copper is too near non-ground copper on the same
/// selected layer.
///
/// RF layout review is strongly geometry-dependent. This check uses the shared
/// spatial index as a broad phase and only reports after exact offset/CSG or
/// boundary-distance confirmation. Verify findings against the RF layout plan.
pub fn rf_keepout_readiness(
    board: &BoardModel,
    clearance: &Scalar,
    selected_layers: &[String],
    min_area: &Scalar,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let broad_phase_clearance = scalar_broad_phase_radius(clearance);
    let feature_index = CopperSpatialIndex::new(&features, broad_phase_clearance);
    log::trace!(
        "RF keepout readiness: source={} features={} rf_buckets={} clearance={clearance:.6}",
        board.source,
        features.len(),
        feature_index.bucket_count()
    );
    let rf_indices = features
        .iter()
        .enumerate()
        .filter_map(|(index, feature)| {
            feature
                .net
                .as_deref()
                .is_some_and(looks_rf_or_antenna_net)
                .then_some(index)
        })
        .collect::<Vec<_>>();
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;
    let mut exact_pair_count = 0_usize;

    for &rf_index in &rf_indices {
        let rf = features[rf_index];
        for neighbor_index in feature_index.same_layer_near_feature(rf, broad_phase_clearance) {
            candidate_count += 1;
            let neighbor = features[neighbor_index];
            if rf_index == neighbor_index {
                continue;
            }
            if rf.net.is_none() || rf.net == neighbor.net {
                continue;
            }
            if neighbor.net.as_deref().is_some_and(looks_ground_net) {
                continue;
            }
            if neighbor.net.as_deref().is_some_and(looks_rf_or_antenna_net)
                && neighbor_index < rf_index
            {
                continue;
            }
            if !sketches_within_clearance(&rf.sketch, &neighbor.sketch, broad_phase_clearance) {
                continue;
            }
            exact_pair_count += 1;

            let expanded = match offset_for_check(
                &rf.sketch,
                clearance.clone(),
                "rf-keepout-readiness",
                vec![rf.layer.clone()],
            ) {
                Ok(expanded) => expanded,
                Err(uncertainty) => return vec![*uncertainty],
            };
            let overlap = expanded.intersection(&neighbor.sketch);
            let shapes = multipolygon_to_shapes_scalar(&overlap.to_multipolygon(), min_area);
            let locations = if shapes.is_empty()
                && polygon_boundary_distance_scalar(
                    &rf.sketch.to_multipolygon(),
                    &neighbor.sketch.to_multipolygon(),
                )
                .is_some_and(|distance| &distance <= clearance)
            {
                vec![
                    rf.location_f64_compatibility_required(),
                    neighbor.location_f64_compatibility_required(),
                ]
            } else {
                Vec::new()
            };
            if shapes.is_empty() && locations.is_empty() {
                continue;
            }

            violations.push(Violation::new(
                "rf-keepout-readiness",
                Severity::Warning,
                vec![rf.layer.clone()],
                None,
                shapes,
                locations,
                Some(format!(
                    "likely RF or antenna net {:?} is within {clearance:.6} of non-ground net {:?}; review antenna keepout, guard, and coupling intent",
                    rf.net, neighbor.net
                )),
            ));
        }
    }

    log::trace!(
        "RF keepout readiness: source={} rf_features={} candidate_pairs={} exact_pairs={} violations={}",
        board.source,
        rf_indices.len(),
        candidate_count,
        exact_pair_count,
        violations.len()
    );
    debug_assert!(exact_pair_count <= candidate_count);

    violations
}

/// Warn when likely antenna copper has any other copper inside its local
/// copper-free review region.
///
/// Antenna feeds and radiators are especially sensitive to nearby conductors,
/// including ground copper, because the board ground plane and clearance around
/// the radiator are part of the radiating structure. The readiness heuristic is
/// deliberately conservative and reports geometry for human review rather than
/// trying to solve an antenna. Nearby copper remains an RF design-readiness
/// concern because coupling and ground-plane geometry affect the radiator.
pub fn antenna_copper_keepout_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    keepout: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    if keepout <= &Scalar::zero() {
        return Vec::new();
    }

    let features = selected_copper_features(board, selected_layers);
    let broad_phase_keepout = scalar_broad_phase_radius(keepout);
    let feature_index = CopperSpatialIndex::new(&features, broad_phase_keepout);
    log::trace!(
        "antenna copper keepout readiness: source={} features={} buckets={} keepout={keepout:.6}",
        board.source,
        features.len(),
        feature_index.bucket_count()
    );
    let mut violations = Vec::new();
    let mut antenna_count = 0_usize;
    let mut candidate_count = 0_usize;
    let mut exact_pair_count = 0_usize;

    for antenna_index in features.iter().enumerate().filter_map(|(index, feature)| {
        feature
            .net
            .as_deref()
            .is_some_and(looks_antenna_net)
            .then_some(index)
    }) {
        antenna_count += 1;
        let antenna = features[antenna_index];
        for neighbor_index in feature_index.same_layer_near_feature(antenna, broad_phase_keepout) {
            candidate_count += 1;
            let neighbor = features[neighbor_index];
            if antenna_index == neighbor_index || antenna.net == neighbor.net {
                continue;
            }
            if !sketches_within_clearance(&antenna.sketch, &neighbor.sketch, broad_phase_keepout) {
                continue;
            }
            exact_pair_count += 1;

            let expanded = match offset_for_check(
                &antenna.sketch,
                keepout.clone(),
                "antenna-copper-keepout-readiness",
                vec![antenna.layer.clone()],
            ) {
                Ok(expanded) => expanded,
                Err(uncertainty) => return vec![*uncertainty],
            };
            let overlap = expanded.intersection(&neighbor.sketch);
            let shapes = multipolygon_to_shapes_scalar(&overlap.to_multipolygon(), min_area);
            let locations = if shapes.is_empty()
                && polygon_boundary_distance_scalar(
                    &antenna.sketch.to_multipolygon(),
                    &neighbor.sketch.to_multipolygon(),
                )
                .is_some_and(|distance| &distance <= keepout)
            {
                vec![
                    antenna.location_f64_compatibility_required(),
                    neighbor.location_f64_compatibility_required(),
                ]
            } else {
                Vec::new()
            };
            if shapes.is_empty() && locations.is_empty() {
                continue;
            }

            violations.push(Violation::new(
                "antenna-copper-keepout-readiness",
                Severity::Warning,
                vec![antenna.layer.clone()],
                None,
                shapes,
                locations,
                Some(format!(
                    "likely antenna net {:?} has copper from net {:?} inside keepout {keepout:.6}; review antenna copper-free region, ground clearance, and matching-network layout",
                    antenna.net, neighbor.net
                )),
            ));
        }
    }

    log::trace!(
        "antenna copper keepout readiness: source={} antenna_features={} candidate_pairs={} exact_pairs={} violations={}",
        board.source,
        antenna_count,
        candidate_count,
        exact_pair_count,
        violations.len()
    );
    debug_assert!(exact_pair_count <= candidate_count);

    violations
}

/// Warn when likely RF/antenna copper has no nearby ground via fence.
///
/// This is a deliberately conservative readiness heuristic: it does not try to
/// solve an RF launch or antenna structure, but it does catch the common handoff
/// gap where an RF trace/connector is present and there is no parsed ground-via
/// stitching nearby for shielding or return-current review. This follows the
/// same practical DFM/EMC framing as IPC-2221B board-design guidance: geometry
/// gets reviewed as evidence of intent before production release.
pub fn rf_via_fence_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    fence_distance: &Scalar,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let ground_vias = features
        .iter()
        .copied()
        .filter(|feature| feature.kind == CopperKind::Via)
        .filter(|feature| feature.net.as_deref().is_some_and(looks_ground_net))
        .collect::<Vec<_>>();
    let broad_phase_fence = scalar_broad_phase_radius(fence_distance);
    let ground_index = CopperSpatialIndex::new(&ground_vias, broad_phase_fence);
    log::trace!(
        "RF via-fence readiness: source={} features={} ground_vias={} buckets={} fence_distance={fence_distance:.6}",
        board.source,
        features.len(),
        ground_vias.len(),
        ground_index.bucket_count()
    );
    let mut violations = Vec::new();
    let mut rf_feature_count = 0_usize;
    let mut candidate_count = 0_usize;
    let mut exact_distance_count = 0_usize;

    for feature in features {
        let Some(net) = &feature.net else {
            continue;
        };
        if !looks_rf_or_antenna_net(net) || feature.kind == CopperKind::Via {
            continue;
        }
        rf_feature_count += 1;

        let candidates = ground_index.same_layer_candidate_centers_near(
            feature.location_f64_compatibility_required(),
            &feature.layer,
            broad_phase_fence,
        );
        candidate_count += candidates.len();
        let has_fence = candidates.into_iter().any(|ground_index| {
            exact_distance_count += 1;
            let ground = ground_vias[ground_index];
            ground.net.as_deref().is_some_and(looks_ground_net)
                && distance_scalar(&feature.location, &ground.location)
                    .is_some_and(|distance| &distance <= fence_distance)
        });
        if has_fence {
            continue;
        }

        violations.push(Violation::new(
            "rf-via-fence-readiness",
            Severity::Warning,
            vec![feature.layer.clone()],
            None,
            Vec::new(),
            vec![feature.location_f64_compatibility_required()],
            Some(format!(
                "likely RF or antenna net {net} has no parsed same-layer ground via within {fence_distance:.6}; review via fence, coplanar ground, and launch shielding intent"
            )),
        ));
    }

    log::trace!(
        "RF via-fence readiness: source={} rf_features={} candidate_ground_vias={} exact_distance_checks={} violations={}",
        board.source,
        rf_feature_count,
        candidate_count,
        exact_distance_count,
        violations.len()
    );
    debug_assert!(exact_distance_count <= candidate_count);

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

fn sketches_within_clearance(
    left: &crate::PcbSketch,
    right: &crate::PcbSketch,
    clearance: f64,
) -> bool {
    let Some(left_bounds) = left.geometry().bounding_rect() else {
        return true;
    };
    let Some(right_bounds) = right.geometry().bounding_rect() else {
        return true;
    };

    left_bounds.min().x - clearance <= right_bounds.max().x
        && left_bounds.max().x + clearance >= right_bounds.min().x
        && left_bounds.min().y - clearance <= right_bounds.max().y
        && left_bounds.max().y + clearance >= right_bounds.min().y
}

fn distance_scalar(left: &[Scalar; 2], right: &[Scalar; 2]) -> Option<Scalar> {
    let dx = &left[0] - &right[0];
    let dy = &left[1] - &right[1];
    (&dx * &dx + &dy * &dy).sqrt().ok()
}

fn scalar_broad_phase_radius(value: &Scalar) -> f64 {
    let projected = value
        .to_f64_lossy()
        .expect("RF broad-phase radius must fit the finite compatibility index");
    if projected > 0.0 {
        projected.next_up()
    } else {
        0.0
    }
}

fn looks_rf_or_antenna_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "RF", "ANT", "ANTENNA", "GNSS", "GPS", "WIFI", "BT_", "BLE", "LTE",
    ];

    tokens.iter().any(|token| normalized.contains(token))
}

fn looks_antenna_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = ["ANT", "ANTENNA", "GNSS", "GPS", "WIFI", "BT_", "BLE", "LTE"];

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

#[cfg(test)]
mod tests {
    use crate::LayerMetadata;
    use crate::geometry::{circle_polygon, line_polygon, polygons_to_profile};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind};

    use super::{antenna_copper_keepout_readiness, rf_keepout_readiness, rf_via_fence_readiness};

    #[test]
    fn rf_keepout_readiness_reports_antenna_near_non_ground_copper() {
        let board = board_with_copper(vec![
            copper_line("RF_ANT", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.10),
            copper_line("GPIO1", CopperKind::Segment, [0.0, 0.50], [1.0, 0.50], 0.10),
        ]);

        let violations = rf_keepout_readiness(
            &board,
            &crate::scalar::scalar("0.60"),
            &[],
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "rf-keepout-readiness");
    }

    #[test]
    fn rf_keepout_readiness_allows_ground_or_distant_copper() {
        let board = board_with_copper(vec![
            copper_line("RF_ANT", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.10),
            copper_line("GND", CopperKind::Segment, [0.0, 0.20], [1.0, 0.20], 0.10),
            copper_line("GPIO1", CopperKind::Segment, [0.0, 2.0], [1.0, 2.0], 0.10),
        ]);

        assert!(
            rf_keepout_readiness(
                &board,
                &crate::scalar::scalar("0.60"),
                &[],
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
    }

    #[test]
    fn rf_keepout_readiness_culls_sparse_neighbor_fields() {
        let mut copper = (0..2_000)
            .map(|index| {
                copper_line(
                    &format!("GPIO{index}"),
                    CopperKind::Segment,
                    [
                        100.0 + (index % 100) as f64 * 3.0,
                        (index / 100) as f64 * 3.0,
                    ],
                    [
                        101.0 + (index % 100) as f64 * 3.0,
                        (index / 100) as f64 * 3.0,
                    ],
                    0.10,
                )
            })
            .collect::<Vec<_>>();
        copper.push(copper_line(
            "RF_ANT",
            CopperKind::Segment,
            [0.0, 0.0],
            [1.0, 0.0],
            0.10,
        ));
        copper.push(copper_line(
            "GPIO_NEAR",
            CopperKind::Segment,
            [0.0, 0.45],
            [1.0, 0.45],
            0.10,
        ));
        copper.push(copper_line_on_layer(
            "GPIO_OTHER_LAYER",
            CopperKind::Segment,
            "B.Cu",
            [0.0, 0.45],
            [1.0, 0.45],
            0.10,
        ));
        let board = board_with_copper(copper);

        let violations = rf_keepout_readiness(
            &board,
            &crate::scalar::scalar("0.60"),
            &[],
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(violations[0].message.as_ref().is_some_and(|message| {
            message.contains("GPIO_NEAR") && !message.contains("GPIO_OTHER_LAYER")
        }));
    }

    #[test]
    fn antenna_copper_keepout_readiness_reports_ground_copper() {
        let board = board_with_copper(vec![
            copper_line(
                "WIFI_ANT",
                CopperKind::Segment,
                [0.0, 0.0],
                [1.0, 0.0],
                0.10,
            ),
            copper_line("GND", CopperKind::Segment, [0.0, 0.35], [1.0, 0.35], 0.10),
        ]);

        let violations = antenna_copper_keepout_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.50"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "antenna-copper-keepout-readiness");
    }

    #[test]
    fn antenna_copper_keepout_readiness_culls_sparse_ground_fields() {
        let mut copper = (0..2_000)
            .map(|index| {
                copper_line(
                    &format!("SIG{index}"),
                    CopperKind::Segment,
                    [
                        100.0 + (index % 100) as f64 * 3.0,
                        (index / 100) as f64 * 3.0,
                    ],
                    [
                        101.0 + (index % 100) as f64 * 3.0,
                        (index / 100) as f64 * 3.0,
                    ],
                    0.10,
                )
            })
            .collect::<Vec<_>>();
        copper.push(copper_line(
            "WIFI_ANT",
            CopperKind::Segment,
            [0.0, 0.0],
            [1.0, 0.0],
            0.10,
        ));
        copper.push(copper_line(
            "GND",
            CopperKind::Segment,
            [0.0, 0.35],
            [1.0, 0.35],
            0.10,
        ));
        let board = board_with_copper(copper);

        let violations = antenna_copper_keepout_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.50"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "antenna-copper-keepout-readiness");
    }

    #[test]
    fn antenna_copper_keepout_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![
            copper_line_on_layer(
                "WIFI_ANT",
                CopperKind::Segment,
                "B.Cu",
                [0.0, 0.0],
                [1.0, 0.0],
                0.10,
            ),
            copper_line_on_layer(
                "GND",
                CopperKind::Segment,
                "B.Cu",
                [0.0, 0.35],
                [1.0, 0.35],
                0.10,
            ),
        ]);

        assert!(
            antenna_copper_keepout_readiness(
                &board,
                &["F.Cu".to_string()],
                &crate::scalar::scalar("0.50"),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
        assert_eq!(
            antenna_copper_keepout_readiness(
                &board,
                &["B.Cu".to_string()],
                &crate::scalar::scalar("0.50"),
                &crate::scalar::scalar("1.0e-9")
            )
            .len(),
            1
        );
    }

    #[test]
    fn antenna_copper_keepout_readiness_ignores_plain_rf_feed_name() {
        let board = board_with_copper(vec![
            copper_line("RF_FEED", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.10),
            copper_line("GND", CopperKind::Segment, [0.0, 0.35], [1.0, 0.35], 0.10),
        ]);

        assert!(
            antenna_copper_keepout_readiness(
                &board,
                &[],
                &crate::scalar::scalar("0.50"),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
    }

    #[test]
    fn rf_via_fence_readiness_reports_rf_copper_without_nearby_ground_via() {
        let board = board_with_copper(vec![copper_line(
            "RF_ANT",
            CopperKind::Segment,
            [0.0, 0.0],
            [1.0, 0.0],
            0.10,
        )]);

        let violations = rf_via_fence_readiness(&board, &[], &crate::scalar::scalar("0.60"));

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "rf-via-fence-readiness");
    }

    #[test]
    fn rf_via_fence_readiness_allows_nearby_ground_via_or_non_rf_copper() {
        let board = board_with_copper(vec![
            copper_line("RF_ANT", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.10),
            copper_disc("GND", CopperKind::Via, [0.2, 0.2], 0.12),
            copper_line("GPIO1", CopperKind::Segment, [3.0, 0.0], [4.0, 0.0], 0.10),
        ]);

        assert!(rf_via_fence_readiness(&board, &[], &crate::scalar::scalar("0.60")).is_empty());
    }

    #[test]
    fn rf_via_fence_readiness_culls_sparse_ground_vias() {
        let mut copper = (0..2_000)
            .map(|index| {
                copper_disc(
                    "GND",
                    CopperKind::Via,
                    [
                        100.0 + (index % 100) as f64 * 3.0,
                        (index / 100) as f64 * 3.0,
                    ],
                    0.12,
                )
            })
            .collect::<Vec<_>>();
        copper.push(copper_line(
            "RF_ANT",
            CopperKind::Segment,
            [0.0, 0.0],
            [1.0, 0.0],
            0.10,
        ));
        copper.push(copper_disc("GND", CopperKind::Via, [0.2, 0.2], 0.12));
        let board = board_with_copper(copper);

        assert!(rf_via_fence_readiness(&board, &[], &crate::scalar::scalar("0.60")).is_empty());
    }

    #[test]
    fn rf_via_fence_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![copper_line_on_layer(
            "RF_FEED",
            CopperKind::Segment,
            "B.Cu",
            [0.0, 0.0],
            [1.0, 0.0],
            0.10,
        )]);

        assert!(
            rf_via_fence_readiness(
                &board,
                &["F.Cu".to_string()],
                &crate::scalar::scalar("0.60")
            )
            .is_empty()
        );
        assert_eq!(
            rf_via_fence_readiness(
                &board,
                &["B.Cu".to_string()],
                &crate::scalar::scalar("0.60")
            )
            .len(),
            1
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

    fn copper_line(
        net: &str,
        kind: CopperKind,
        start: [f64; 2],
        end: [f64; 2],
        width: f64,
    ) -> CopperFeature {
        copper_line_on_layer(net, kind, "F.Cu", start, end, width)
    }

    fn copper_line_on_layer(
        net: &str,
        kind: CopperKind,
        layer: &str,
        start: [f64; 2],
        end: [f64; 2],
        width: f64,
    ) -> CopperFeature {
        CopperFeature {
            layer: layer.to_string(),
            net: Some(net.to_string()),
            kind,
            location: [
                crate::geometry::exact_real((start[0] + end[0]) / 2.0),
                crate::geometry::exact_real((start[1] + end[1]) / 2.0),
            ],
            sketch: polygons_to_profile(
                vec![line_polygon(start, end, width).expect("test line should be valid")],
                Some(LayerMetadata {
                    name: "test line".to_string(),
                }),
            ),
        }
    }

    fn copper_disc(
        net: &str,
        kind: CopperKind,
        location: [f64; 2],
        diameter: f64,
    ) -> CopperFeature {
        CopperFeature {
            layer: "F.Cu".to_string(),
            net: Some(net.to_string()),
            kind,
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
            sketch: polygons_to_profile(
                vec![circle_polygon(location, diameter / 2.0, 32)],
                Some(LayerMetadata {
                    name: "test disc".to_string(),
                }),
            ),
        }
    }
}
