//! Native authored-center proximity for RF ground-via fences.

use super::{looks_ground_net, looks_rf_or_antenna_net, selected_copper_features};
use crate::Scalar;
use crate::checks::{measurements, spatial::NativePointSpatialIndex};
use crate::kicad::{BoardModel, CopperKind};
use crate::report::{Severity, Violation};

/// Review non-via RF/antenna features for nearby same-layer ground via centers.
/// Native enclosures only schedule candidates; checked closed-radius predicates
/// decide matches. An unresolved candidate remains uncertainty unless another
/// candidate proves the required match. This is a placement review proxy.
pub fn rf_via_fence_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    fence_distance: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("rf-via-fence-readiness", &[&board.source], || {
        crate::operations::evaluate("rf-via-fence-readiness", Some(&board.source), || {
            let features = selected_copper_features(board, selected_layers);
            let ground_vias = features
                .iter()
                .copied()
                .filter(|feature| {
                    feature.kind == CopperKind::Via
                        && feature.net.as_deref().is_some_and(looks_ground_net)
                })
                .collect::<Vec<_>>();
            let index = NativePointSpatialIndex::new(
                ground_vias.iter().map(|feature| &feature.location),
                fence_distance,
            );
            let mut violations = Vec::new();
            let mut rf_feature_count = 0;
            let mut candidate_count = 0;
            let mut exact_distance_count = 0;
            for feature in features {
                let Some(net) = feature
                    .net
                    .as_deref()
                    .filter(|net| looks_rf_or_antenna_net(net))
                else {
                    continue;
                };
                if feature.kind == CopperKind::Via {
                    continue;
                }
                rf_feature_count += 1;
                let candidates = index
                    .candidates_near(&feature.location)
                    .into_iter()
                    .filter(|&other| ground_vias[other].layer == feature.layer)
                    .collect::<Vec<_>>();
                candidate_count += candidates.len();
                let has_fence = measurements::any_match(candidates, |other| {
                    exact_distance_count += 1;
                    measurements::points_within_radius(
                        &feature.location,
                        &ground_vias[other].location,
                        fence_distance,
                    )
                })?;
                if has_fence {
                    continue;
                }
                let message = format!(
                    "likely RF or antenna net {net} has no parsed same-layer ground via within {fence_distance:.6}; review via fence, coplanar ground, and launch shielding intent"
                );
                violations.push(Violation::new(
                    "rf-via-fence-readiness",
                    Severity::Warning,
                    vec![feature.layer.clone()],
                    None,
                    Vec::new(),
                    vec![measurements::report_point(&feature.location)?],
                    Some(message),
                ));
            }
            log::trace!(
                "RF via-fence readiness: source={} features={} ground_vias={} buckets={} candidate_ground_vias={} exact_distance_checks={} fence_distance={fence_distance:.6} violations={}",
                board.source,
                rf_feature_count,
                ground_vias.len(),
                index.bucket_count(),
                candidate_count,
                exact_distance_count,
                violations.len(),
            );
            debug_assert!(exact_distance_count <= candidate_count);
            Ok(violations)
        })
    })
}
