//! Authored mounting-hole centers and explicit ground/chassis bonding intent.

use super::{looks_ground_or_chassis_net, selected_copper_features};
use crate::checks::{measurements, spatial::NativePointSpatialIndex};
use crate::kicad::{BoardModel, DrillFeature};
use crate::report::{Severity, Violation};
use crate::{PcbGeometryUncertainty, Scalar};

/// Check the native mounting-size proxy without ordering the two thresholds.
/// `diameter >= max(1, hint)` needs both bounds; a certified failure of either
/// bound proves ineligibility even if the other comparison is unresolved.
pub(super) fn mounting_hole_eligible(
    drill: &DrillFeature,
    hint: &Scalar,
) -> Result<bool, PcbGeometryUncertainty> {
    measurements::any_match([&Scalar::one(), hint], |minimum| {
        Ok(crate::scalar::try_compare(&drill.diameter, minimum)?.is_lt())
    })
    .map(|too_small| !too_small)
}

/// Review native NPTH mounting centers for ground/chassis copper proximity.
/// Hole eligibility, radius construction and required matches retain typed
/// failure. The result remains a mechanical bonding-intent proxy.
pub fn mounting_hole_grounding_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    grounding_distance: &Scalar,
) -> Vec<Violation> {
    bonding_readiness(board, selected_layers, grounding_distance, false)
}

/// Review large plated mounting-style holes for explicit bonding intent.
/// An authored ground/chassis net is sufficient by itself; otherwise nearby
/// copper must certify the native center/radius match.
pub fn mounting_hole_plating_intent_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    grounding_distance: &Scalar,
) -> Vec<Violation> {
    bonding_readiness(board, selected_layers, grounding_distance, true)
}

fn bonding_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    grounding_distance: &Scalar,
    plated: bool,
) -> Vec<Violation> {
    let check = if plated {
        "mounting-hole-plating-intent-readiness"
    } else {
        "mounting-hole-grounding-readiness"
    };
    crate::checks::run_geometry_check(check, &[&board.source], || {
        crate::operations::evaluate(check, Some(&board.source), || {
            let ground = selected_copper_features(board, selected_layers)
                .into_iter()
                .filter(|feature| {
                    feature
                        .net
                        .as_deref()
                        .is_some_and(looks_ground_or_chassis_net)
                })
                .collect::<Vec<_>>();
            let index = NativePointSpatialIndex::new(
                ground.iter().map(|feature| &feature.location),
                grounding_distance,
            );
            let mut violations = Vec::new();
            let mut candidate_holes = 0;
            let mut candidate_count = 0;
            let mut exact_distance_count = 0;
            for drill in board.drills.iter().filter(|drill| drill.plated == plated) {
                // This metadata proves the plated-hole intent regardless of
                // eligibility or proximity; no optional geometry is required.
                if plated
                    && drill
                        .net
                        .as_deref()
                        .is_some_and(looks_ground_or_chassis_net)
                {
                    continue;
                }
                if !mounting_hole_eligible(drill, grounding_distance)? {
                    continue;
                }
                candidate_holes += 1;
                let radius = measurements::drill_radius(&drill.diameter)? + grounding_distance;
                let candidates = index.candidates_near_with_radius(&drill.location, &radius);
                candidate_count += candidates.len();
                let has_ground = measurements::any_match(candidates, |other| {
                    exact_distance_count += 1;
                    measurements::points_within_radius(
                        &drill.location,
                        &ground[other].location,
                        &radius,
                    )
                })?;
                if has_ground {
                    continue;
                }
                let message = if plated {
                    format!(
                        "large plated mounting-style hole diameter {:.6} has no parsed ground/chassis net or nearby bonding copper; review plated versus isolated hardware intent",
                        drill.diameter.fixed_decimal(),
                    )
                } else {
                    format!(
                        "likely mounting hole diameter {:.6} has no parsed ground or chassis copper within {:.6}; review chassis bonding, isolation, or keepout intent",
                        drill.diameter.fixed_decimal(),
                        grounding_distance.fixed_decimal(),
                    )
                };
                violations.push(Violation::new(
                    check,
                    Severity::Warning,
                    vec![board.source.clone()],
                    None,
                    Vec::new(),
                    vec![measurements::report_point(&drill.location)?],
                    Some(message),
                ));
            }
            log::trace!(
                "{check}: source={} candidate_holes={} grounding_features={} buckets={} candidate_pairs={} exact_distance_checks={} grounding_distance={:.6} violations={}",
                board.source,
                candidate_holes,
                ground.len(),
                index.bucket_count(),
                candidate_count,
                exact_distance_count,
                grounding_distance.fixed_decimal(),
                violations.len(),
            );
            debug_assert!(exact_distance_count <= candidate_count);
            Ok(violations)
        })
    })
}
