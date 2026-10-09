//! Native copper-boundary review after authored-center cluster qualification.

use super::cluster::{dense_cluster_context, select_below};
use super::dense_pad_inputs;
use crate::checks::distance::region_boundary_distance;
use crate::checks::measurements;
use crate::checks::spatial::{NativePointSpatialIndex, NativeRegionSpatialIndex};
use crate::geometry::multipolygon_to_shapes_scalar;
use crate::kicad::BoardModel;
use crate::report::{Severity, Violation};
use crate::{PcbRegionExt, Scalar};

/// Review native via/pad copper clearance in dense authored-center clusters.
/// A shape enclosure schedules candidates independently of authored anchors.
/// Required center and clearance predicates retain uncertainty. A proved bad
/// pair can support a finding when optional nearest-pair ordering is unknown;
/// the diagnostic distinguishes a certified witness from a certified minimum.
pub fn dense_pad_via_spacing_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    pitch_threshold: &Scalar,
    via_search_radius: &Scalar,
    min_via_clearance: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    dense_pad_via_spacing_readiness_with_stats(
        board,
        selected_layers,
        pitch_threshold,
        via_search_radius,
        min_via_clearance,
        min_area,
    )
    .0
}

#[derive(Clone, Copy, Debug, Default)]
pub(super) struct DensePadViaSpacingStats {
    pub(super) via_candidates: usize,
    pub(super) exact_via_distances: usize,
    pub(super) pad_candidates: usize,
    pub(super) exact_pad_clearances: usize,
}

pub(super) fn dense_pad_via_spacing_readiness_with_stats(
    board: &BoardModel,
    selected_layers: &[String],
    pitch_threshold: &Scalar,
    via_search_radius: &Scalar,
    min_via_clearance: &Scalar,
    min_area: &Scalar,
) -> (Vec<Violation>, DensePadViaSpacingStats) {
    let mut stats = DensePadViaSpacingStats::default();
    let findings = crate::checks::run_geometry_check(
        "dense-pad-via-spacing-readiness",
        &[&board.source],
        || {
            crate::operations::evaluate(
                "dense-pad-via-spacing-readiness",
                Some(&board.source),
                || {
                    if crate::scalar::compare(min_via_clearance, &Scalar::zero())
                        .is_some_and(|order| order.is_le())
                    {
                        return Ok(Vec::new());
                    }
                    let (pads_by_layer, vias) = dense_pad_inputs(board, selected_layers);
                    let via_index = NativePointSpatialIndex::new(
                        vias.iter().map(|via| &via.location),
                        via_search_radius,
                    );
                    let mut violations = Vec::new();
                    let mut pad_buckets = 0;
                    for (layer, pads) in pads_by_layer {
                        let Some(cluster) = dense_cluster_context(&pads, pitch_threshold)? else {
                            continue;
                        };
                        let pad_index = NativeRegionSpatialIndex::new(
                            pads.iter().map(|pad| &pad.region),
                            min_via_clearance,
                        );
                        pad_buckets += pad_index.bucket_count();
                        let candidates = via_index.candidates_near(&cluster.center);
                        stats.via_candidates += candidates.len();
                        for other in candidates {
                            stats.exact_via_distances += 1;
                            let via = vias[other];
                            if !measurements::points_within_radius(
                                &cluster.center,
                                &via.location,
                                via_search_radius,
                            )? {
                                continue;
                            }
                            let candidates =
                                pad_index.candidates_near(&via.region, min_via_clearance);
                            stats.pad_candidates += candidates.len();
                            let nearest = select_below(
                                candidates.into_iter().map(|index| {
                                    stats.exact_pad_clearances += 1;
                                    region_boundary_distance(&pads[index].region, &via.region)
                                        .map(|value| value.map(|value| (index, value)))
                                }),
                                min_via_clearance,
                                false,
                            )?;
                            let Some(nearest) = nearest else {
                                continue;
                            };
                            let pad = pads[nearest.item];
                            let keepout = via.region.offset(min_via_clearance.clone())?;
                            let overlap = keepout.try_intersection(&pad.region)?;
                            let shapes = multipolygon_to_shapes_scalar(
                                &overlap.try_to_multipolygon()?,
                                min_area,
                            )?;
                            violations.push(Violation::new(
                        "dense-pad-via-spacing-readiness", Severity::Warning, vec![layer.clone(), via.layer.clone()], None, shapes,
                        vec![measurements::report_point(&pad.location)?, measurements::report_point(&via.location)?, measurements::report_point(&cluster.center)?],
                        Some(format!(
                            "dense pad cluster has {}; {} is below {:.6}, review BGA escape spacing, soldermask web, and via fill/cap intent",
                            cluster.pitch.description("minimum pitch", "certified pitch"),
                            nearest.description("nearest pad/via clearance", "certified pad/via clearance"),
                            min_via_clearance.fixed_decimal(),
                        )),
                    ));
                        }
                    }
                    log::trace!(
                        "dense pad/via spacing readiness: source={} via_candidates={} exact_via_distances={} pad_buckets={} pad_candidates={} exact_pad_clearances={} violations={}",
                        board.source,
                        stats.via_candidates,
                        stats.exact_via_distances,
                        pad_buckets,
                        stats.pad_candidates,
                        stats.exact_pad_clearances,
                        violations.len()
                    );
                    debug_assert!(stats.exact_via_distances <= stats.via_candidates);
                    debug_assert!(stats.exact_pad_clearances <= stats.pad_candidates);
                    Ok(violations)
                },
            )
        },
    );
    (findings, stats)
}

/// Review native pad-boundary spacing in dense clusters as a mask-web proxy.
/// Actual parsed solder-mask openings remain the preferred manufacturing input.
/// Native shape enclosures never depend on report rings or authored anchors;
/// required failures stay typed, and optional minimum ordering is qualified.
pub fn dense_pad_mask_bridge_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    pitch_threshold: &Scalar,
    min_mask_web: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("dense-pad-mask-bridge-readiness", &[&board.source], || {
        crate::operations::evaluate(
            "dense-pad-mask-bridge-readiness",
            Some(&board.source),
            || {
                if crate::scalar::compare(min_mask_web, &Scalar::zero())
                    .is_some_and(|order| order.is_le())
                {
                    return Ok(Vec::new());
                }
                let (pads_by_layer, _) = dense_pad_inputs(board, selected_layers);
                let mut violations = Vec::new();
                for (layer, pads) in pads_by_layer {
                    let Some(cluster) = dense_cluster_context(&pads, pitch_threshold)? else {
                        continue;
                    };
                    let index = NativeRegionSpatialIndex::new(
                        pads.iter().map(|pad| &pad.region),
                        min_mask_web,
                    );
                    let mut candidate_count = 0;
                    let pairs = (0..pads.len()).flat_map(|left| {
                        index
                            .later_candidates_near(left, min_mask_web)
                            .into_iter()
                            .map(move |right| (left, right))
                    });
                    let nearest = select_below(
                        pairs.map(|(left, right)| {
                            candidate_count += 1;
                            region_boundary_distance(&pads[left].region, &pads[right].region)
                                .map(|value| value.map(|value| ((left, right), value)))
                        }),
                        min_mask_web,
                        false,
                    )?;
                    log::trace!(
                        "dense pad mask-bridge readiness: source={} layer={layer} pads={} buckets={} candidate_pairs={candidate_count} qualified={}",
                        board.source,
                        pads.len(),
                        index.bucket_count(),
                        nearest.is_some()
                    );
                    let Some(nearest) = nearest else {
                        continue;
                    };
                    violations.push(Violation::new(
                    "dense-pad-mask-bridge-readiness", Severity::Warning, vec![layer], None, Vec::new(),
                    vec![measurements::report_point(&pads[nearest.item.0].location)?, measurements::report_point(&pads[nearest.item.1].location)?, measurements::report_point(&cluster.center)?],
                    Some(format!(
                        "dense pad cluster has {}; {} is below mask web {:.6}, review BGA solder-mask bridge and NSMD/SMD pad definition",
                        cluster.pitch.description("minimum pitch", "certified pitch"),
                        nearest.description("nearest pad copper spacing", "certified pad copper spacing"),
                        min_mask_web.fixed_decimal(),
                    )),
                ));
                }
                Ok(violations)
            },
        )
    })
}
