//! Dense fine-pitch pad and BGA readiness checks.
//!
//! Authored native centers qualify dense clusters and nearby registration/escape
//! evidence. Copper boundaries decide pad/via and mask-web clearance. These are
//! placement and fabrication review proxies, not reconstructed package intent.

use std::collections::BTreeMap;

use crate::Scalar;
use crate::checks::{measurements, spatial::NativePointSpatialIndex};
use crate::kicad::{BoardModel, CopperFeature, CopperKind};
use crate::report::{Severity, Violation};

mod cluster;
mod spacing;
use cluster::{capped_match_count, dense_cluster_context, likely_fiducial};
#[cfg(test)]
use spacing::dense_pad_via_spacing_readiness_with_stats;
pub use spacing::{dense_pad_mask_bridge_readiness, dense_pad_via_spacing_readiness};

const DENSE_PAD_CLUSTER_MIN_PADS: usize = 16;

/// Review dense fine-pitch pad clusters for nearby local fiducials.
/// IPC-7351B local registration is represented by the retained two-fiducial
/// placement proxy. Native dimensions decide fiducial eligibility, native pitch
/// qualifies clusters, and checked center distances count same-layer matches.
/// Unknown candidates cannot become an exact count below the required two.
pub fn local_fiducial_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    pitch_threshold: &Scalar,
    search_radius: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("local-fiducial-readiness", &[&board.source], || {
        crate::operations::evaluate("local-fiducial-readiness", Some(&board.source), || {
            let mut fiducials = Vec::new();
            let mut pads_by_layer: BTreeMap<String, Vec<&CopperFeature>> = BTreeMap::new();
            for feature in selected_copper_features(board, selected_layers) {
                if likely_fiducial(feature)? {
                    fiducials.push(feature);
                } else if feature.kind == CopperKind::Pad {
                    pads_by_layer
                        .entry(feature.layer.clone())
                        .or_default()
                        .push(feature);
                }
            }
            let index = NativePointSpatialIndex::new(
                fiducials.iter().map(|feature| &feature.location),
                search_radius,
            );
            let mut violations = Vec::new();
            let mut candidate_count = 0;
            let mut exact_distance_count = 0;
            for (layer, pads) in pads_by_layer {
                let Some(cluster) = dense_cluster_context(&pads, pitch_threshold)? else {
                    continue;
                };
                let candidates = index.candidates_near(&cluster.center);
                candidate_count += candidates.len();
                let count = capped_match_count(candidates, 2, |other| {
                    let feature = fiducials[other];
                    if feature.layer != layer {
                        return Ok(false);
                    }
                    exact_distance_count += 1;
                    measurements::points_within_radius(
                        &cluster.center,
                        &feature.location,
                        search_radius,
                    )
                })?;
                if count >= 2 {
                    continue;
                }
                violations.push(Violation::new(
                    "local-fiducial-readiness", Severity::Warning, vec![layer], None, Vec::new(),
                    vec![measurements::report_point(&cluster.center)?],
                    Some(format!(
                        "dense pad cluster has {} but only {count} likely local fiducial(s) within {:.6}; review local fiducials for fine-pitch assembly",
                        cluster.pitch.description("minimum pitch", "certified pitch"), search_radius.fixed_decimal(),
                    )),
                ));
            }
            log::trace!(
                "local fiducial readiness: source={} fiducials={} buckets={} candidates={} exact_distance_checks={} violations={}",
                board.source,
                fiducials.len(),
                index.bucket_count(),
                candidate_count,
                exact_distance_count,
                violations.len()
            );
            debug_assert!(exact_distance_count <= candidate_count);
            Ok(violations)
        })
    })
}

/// Review dense pad clusters for nearby escape-via evidence.
/// The native center proxy applies across selected copper layers. A certified
/// alternative proves an escape via exists; unresolved candidates cannot prove
/// absence. Required pitch/marker failures retain an uncertainty receipt.
pub fn dense_pad_escape_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    pitch_threshold: &Scalar,
    via_search_radius: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("dense-pad-escape-readiness", &[&board.source], || {
        crate::operations::evaluate("dense-pad-escape-readiness", Some(&board.source), || {
            let (pads_by_layer, vias) = dense_pad_inputs(board, selected_layers);
            let index = NativePointSpatialIndex::new(
                vias.iter().map(|via| &via.location),
                via_search_radius,
            );
            let mut violations = Vec::new();
            let mut candidate_count = 0;
            let mut exact_distance_count = 0;
            for (layer, pads) in pads_by_layer {
                let Some(cluster) = dense_cluster_context(&pads, pitch_threshold)? else {
                    continue;
                };
                let candidates = index.candidates_near(&cluster.center);
                candidate_count += candidates.len();
                let has_via = measurements::any_match(candidates, |other| {
                    exact_distance_count += 1;
                    measurements::points_within_radius(
                        &cluster.center,
                        &vias[other].location,
                        via_search_radius,
                    )
                })?;
                if has_via {
                    continue;
                }
                violations.push(Violation::new(
                    "dense-pad-escape-readiness", Severity::Warning, vec![layer], None, Vec::new(),
                    vec![measurements::report_point(&cluster.center)?],
                    Some(format!(
                        "dense pad cluster has {} with no parsed escape via within {:.6}; review BGA/fine-pitch escape strategy",
                        cluster.pitch.description("minimum pitch", "certified pitch"), via_search_radius.fixed_decimal(),
                    )),
                ));
            }
            log::trace!(
                "dense-pad escape readiness: source={} vias={} buckets={} candidates={} exact_distance_checks={} violations={}",
                board.source,
                vias.len(),
                index.bucket_count(),
                candidate_count,
                exact_distance_count,
                violations.len()
            );
            debug_assert!(exact_distance_count <= candidate_count);
            Ok(violations)
        })
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
            CopperKind::Segment | CopperKind::Zone | CopperKind::Artwork => {}
        }
    }
    (pads_by_layer, vias)
}
#[cfg(test)]
mod tests {
    use super::{
        DENSE_PAD_CLUSTER_MIN_PADS, dense_pad_escape_readiness, dense_pad_mask_bridge_readiness,
        dense_pad_via_spacing_readiness, dense_pad_via_spacing_readiness_with_stats,
        local_fiducial_readiness,
    };
    use crate::LayerMetadata;
    use crate::geometry::{circle_polygon, polygons_to_profile, rect_polygon};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind};

    fn s(value: f64) -> crate::Scalar {
        crate::geometry::exact_real(value)
    }

    #[test]
    fn local_fiducial_readiness_reports_dense_clusters_without_nearby_fiducials() {
        let board = board_with_copper(dense_pad_cluster());

        let violations = local_fiducial_readiness(&board, &[], &s(0.8), &s(5.0));

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "local-fiducial-readiness");
    }

    #[test]
    fn local_fiducial_readiness_accepts_nearby_local_fiducials_or_sparse_pads() {
        let mut copper = dense_pad_cluster();
        copper.push(fiducial("F.Cu", [-1.0, -1.0], 0.8));
        copper.push(fiducial("F.Cu", [2.5, -1.0], 0.8));
        assert!(
            local_fiducial_readiness(&board_with_copper(copper), &[], &s(0.8), &s(5.0)).is_empty()
        );

        let mut sparse = Vec::new();
        for index in 0..DENSE_PAD_CLUSTER_MIN_PADS {
            sparse.push(copper_pad(
                &format!("P{index}"),
                [index as f64 * 1.0, 0.0],
                0.20,
                0.20,
            ));
        }
        assert!(
            local_fiducial_readiness(&board_with_copper(sparse), &[], &s(0.8), &s(5.0)).is_empty()
        );
    }

    #[test]
    fn local_fiducial_readiness_culls_sparse_fiducial_fields() {
        let mut copper = dense_pad_cluster();
        copper.extend((0..2_000).map(|index| {
            fiducial(
                "F.Cu",
                [100.0 + index as f64 * 4.0, 100.0 + index as f64 * 0.01],
                0.8,
            )
        }));
        copper.push(fiducial("F.Cu", [-1.0, -1.0], 0.8));
        copper.push(fiducial("F.Cu", [2.5, -1.0], 0.8));

        let started = std::time::Instant::now();
        let violations =
            local_fiducial_readiness(&board_with_copper(copper), &[], &s(0.8), &s(5.0));

        assert!(violations.is_empty());
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "local fiducial review should spatially cull distant fiducials"
        );
    }

    #[test]
    fn dense_pad_escape_readiness_reports_missing_escape_via() {
        let board = board_with_copper(dense_pad_cluster());

        let violations = dense_pad_escape_readiness(&board, &[], &s(0.8), &s(2.0));

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "dense-pad-escape-readiness");
    }

    #[test]
    fn dense_pad_escape_readiness_culls_sparse_via_fields() {
        let mut copper = dense_pad_cluster();
        copper.extend(
            (0..2_000).map(|index| copper_via("ESC", [100.0 + index as f64 * 4.0, 100.0], 0.20)),
        );
        copper.push(copper_via("ESC", [0.75, 0.75], 0.20));

        let started = std::time::Instant::now();
        let violations =
            dense_pad_escape_readiness(&board_with_copper(copper), &[], &s(0.8), &s(2.0));

        assert!(violations.is_empty());
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "dense-pad escape review should spatially cull distant vias"
        );
    }

    #[test]
    fn dense_pad_via_spacing_readiness_reports_close_escape_via() {
        let mut copper = dense_pad_cluster();
        copper.push(copper_via("ESC", [0.32, 0.0], 0.20));
        let board = board_with_copper(copper);

        let violations = dense_pad_via_spacing_readiness(
            &board,
            &[],
            &s(0.8),
            &s(2.0),
            &s(0.15),
            &crate::scalar::scalar("1.0e-9"),
        );

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
                &s(0.8),
                &s(2.0),
                &s(0.15),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );

        let mut far = dense_pad_cluster();
        far.push(copper_via("ESC", [10.0, 10.0], 0.20));
        assert!(
            dense_pad_via_spacing_readiness(
                &board_with_copper(far),
                &[],
                &s(0.8),
                &s(2.0),
                &s(0.15),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );

        let mut selected_out = dense_pad_cluster();
        selected_out.push(copper_via_on_layer("B.Cu", "ESC", [0.32, 0.0], 0.20));
        assert!(
            dense_pad_via_spacing_readiness(
                &board_with_copper(selected_out),
                &["F.Cu".to_string()],
                &s(0.8),
                &s(2.0),
                &s(0.15),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
    }

    #[test]
    fn dense_pad_via_spacing_readiness_culls_sparse_via_fields() {
        let mut copper = dense_pad_cluster();
        copper.extend(
            (0..2_000).map(|index| copper_via("ESC", [100.0 + index as f64 * 4.0, 100.0], 0.20)),
        );
        copper.push(copper_via("ESC_NEAR", [0.32, 0.0], 0.20));

        let (violations, stats) = dense_pad_via_spacing_readiness_with_stats(
            &board_with_copper(copper),
            &[],
            &s(0.8),
            &s(2.0),
            &s(0.15),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            stats.via_candidates <= 4,
            "spatial index admitted {} of 2001 vias",
            stats.via_candidates
        );
        assert_eq!(stats.exact_via_distances, stats.via_candidates);
        assert!(stats.pad_candidates <= DENSE_PAD_CLUSTER_MIN_PADS);
        assert_eq!(stats.exact_pad_clearances, stats.pad_candidates);
    }

    #[test]
    fn dense_pad_via_spacing_readiness_culls_large_pad_clusters_per_via() {
        let mut copper = (0..2_000)
            .map(|index| {
                copper_pad(
                    &format!("P{index}"),
                    [(index % 50) as f64 * 0.5, (index / 50) as f64 * 0.5],
                    0.20,
                    0.20,
                )
            })
            .collect::<Vec<_>>();
        copper.push(copper_via("ESC_NEAR", [0.32, 0.0], 0.20));

        let started = std::time::Instant::now();
        let violations = dense_pad_via_spacing_readiness(
            &board_with_copper(copper),
            &[],
            &s(0.8),
            &s(25.0),
            &s(0.15),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "dense-pad via spacing should index large pad clusters before exact pad/via review"
        );
    }

    #[test]
    fn dense_pad_mask_bridge_readiness_reports_tight_dense_pad_web() {
        let board = board_with_copper(dense_pad_cluster_with_size(0.45));

        let violations = dense_pad_mask_bridge_readiness(&board, &[], &s(0.8), &s(0.10));

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
                &s(0.8),
                &s(0.10)
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
            dense_pad_mask_bridge_readiness(&board_with_copper(sparse), &[], &s(0.8), &s(0.10))
                .is_empty()
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
                &s(0.8),
                &s(0.10)
            )
            .is_empty()
        );
    }

    #[test]
    fn dense_pad_mask_bridge_readiness_culls_sparse_pad_fields() {
        let mut copper = (0..2_000)
            .map(|index| {
                copper_pad(
                    &format!("P{index}"),
                    [
                        100.0 + (index % 50) as f64 * 1.0,
                        100.0 + (index / 50) as f64 * 1.0,
                    ],
                    0.25,
                    0.25,
                )
            })
            .collect::<Vec<_>>();
        copper.extend(dense_pad_cluster_with_size(0.45));

        let started = std::time::Instant::now();
        let violations =
            dense_pad_mask_bridge_readiness(&board_with_copper(copper), &[], &s(0.8), &s(0.10));

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "dense-pad mask bridge should spatially cull distant pad fields before exact boundary distance"
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
        let violations = dense_pad_escape_readiness(&board, &[], &s(0.8), &s(2.0));

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
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
            region: polygons_to_profile(
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
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
            region: polygons_to_profile(
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
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
            region: polygons_to_profile(
                vec![circle_polygon(location, diameter / 2.0, 32)],
                Some(LayerMetadata {
                    name: "via".to_string(),
                }),
            ),
        }
    }
}
