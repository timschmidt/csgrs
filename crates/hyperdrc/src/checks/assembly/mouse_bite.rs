//! Native diameter and center-spacing decisions for mouse-bite drill rows.

use crate::checks::{measurements, spatial::NativePointSpatialIndex};
use crate::kicad::{BoardModel, DrillFeature};
use crate::report::{Severity, Violation};
use crate::{PcbGeometryUncertainty, Scalar};

/// Review small nonplated drill diameters and native neighbor spacing.
///
/// Certified native enclosures schedule candidates. Diameter eligibility,
/// closed maximum pitch and strict minimum pitch use checked predicates.
/// A valid row needs a certified neighbor and no unresolved too-close candidate;
/// it does not need to order equally distant neighbors. A certified too-close
/// pair proves a spacing finding even if another candidate is unresolved.
/// Prefer the nearest certified bad pair when its order can be proved, keeping
/// the first certified bad pair when only that optional ordering is unavailable.
pub fn mouse_bite_readiness(
    board: &BoardModel,
    extra_drills: &[DrillFeature],
    minimum_diameter: &Scalar,
    maximum_diameter: &Scalar,
    minimum_spacing: &Scalar,
    maximum_spacing: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("mouse-bite-readiness", &[&board.source], || {
        crate::operations::evaluate("mouse-bite-readiness", Some(&board.source), || {
            // Diagnostic decimals need absolute output precision, not a proof
            // of a nonzero leading digit in a possibly cancelled native value.
            let minimum_diameter_text = minimum_diameter.fixed_decimal();
            let maximum_diameter_text = maximum_diameter.fixed_decimal();
            let minimum_spacing_text = minimum_spacing.fixed_decimal();
            let maximum_spacing_text = maximum_spacing.fixed_decimal();
            let mut candidates = Vec::new();
            for drill in board.drills.iter().chain(extra_drills) {
                if !drill.plated
                    && crate::scalar::try_compare(&drill.diameter, maximum_diameter)?.is_le()
                {
                    candidates.push(drill);
                }
            }
            let index = NativePointSpatialIndex::new(
                candidates.iter().map(|drill| &drill.location),
                maximum_spacing,
            );
            let mut violations = Vec::new();
            for drill in &candidates {
                if crate::scalar::try_compare(&drill.diameter, minimum_diameter)?.is_ge() {
                    continue;
                }
                violations.push(Violation::new(
                    "mouse-bite-readiness",
                    Severity::Warning,
                    vec!["mouse-bites".into()],
                    None,
                    Vec::new(),
                    vec![measurements::report_point(&drill.location)?],
                    Some(format!(
                        "likely mouse-bite drill diameter {:.6} is below minimum {minimum_diameter_text:.6}",
                        drill.diameter.fixed_decimal()
                    )),
                ));
            }
            let mut neighbor_hits = 0;
            for (left_index, left) in candidates.iter().enumerate() {
                let nearby = index
                    .candidates_near(&left.location)
                    .into_iter()
                    .filter(|&other| other != left_index)
                    .collect::<Vec<_>>();
                neighbor_hits += nearby.len();
                match inspect_neighbors(
                    left,
                    &candidates,
                    nearby,
                    minimum_spacing,
                    maximum_spacing,
                )? {
                    Neighbors::Valid => {}
                    Neighbors::Missing => {
                        let message = format!(
                            "likely mouse-bite drill has no neighboring mouse-bite center within maximum expected spacing {maximum_spacing_text:.6}"
                        );
                        violations.push(Violation::new(
                            "mouse-bite-readiness",
                            Severity::Warning,
                            vec!["mouse-bites".into()],
                            None,
                            Vec::new(),
                            vec![measurements::report_point(&left.location)?],
                            Some(message),
                        ));
                    }
                    Neighbors::TooClose { right, distance } => {
                        let message = format!(
                            "likely mouse-bite drill center spacing {:.6} is outside expected range {minimum_spacing_text:.6}..{maximum_spacing_text:.6}",
                            distance.fixed_decimal()
                        );
                        violations.push(Violation::new(
                            "mouse-bite-readiness",
                            Severity::Warning,
                            vec!["mouse-bites".into()],
                            None,
                            Vec::new(),
                            vec![
                                measurements::report_point(&left.location)?,
                                measurements::report_point(&candidates[right].location)?,
                            ],
                            Some(message),
                        ));
                    }
                }
            }
            log::trace!(
                "mouse-bite readiness: source={} candidates={} point_buckets={} neighbor_hits={} minimum_diameter={minimum_diameter_text:.6} maximum_diameter={maximum_diameter_text:.6} minimum_spacing={minimum_spacing_text:.6} maximum_spacing={maximum_spacing_text:.6} violations={}",
                board.source,
                candidates.len(),
                index.bucket_count(),
                neighbor_hits,
                violations.len()
            );
            Ok(violations)
        })
    })
}

enum Neighbors {
    Valid,
    Missing,
    TooClose { right: usize, distance: Scalar },
}

fn inspect_neighbors(
    left: &DrillFeature,
    candidates: &[&DrillFeature],
    nearby: Vec<usize>,
    minimum: &Scalar,
    maximum: &Scalar,
) -> Result<Neighbors, PcbGeometryUncertainty> {
    let mut has_neighbor = false;
    let mut membership_uncertainty = None;
    let mut spacing_uncertainty = None;
    let mut too_close: Option<(usize, Scalar)> = None;
    for right in nearby {
        let point = &candidates[right].location;
        match measurements::points_within_radius(&left.location, point, maximum) {
            Ok(false) => continue,
            Err(error) => {
                // An unresolved membership cannot prevent a valid alternative
                // when this candidate is independently proved not too close.
                if !matches!(
                    measurements::points_closer_than_radius(&left.location, point, minimum),
                    Ok(false)
                ) {
                    spacing_uncertainty.get_or_insert_with(|| error.clone());
                }
                membership_uncertainty.get_or_insert(error);
                continue;
            }
            Ok(true) => {
                has_neighbor = true;
            }
        }
        match measurements::points_closer_than_radius(&left.location, point, minimum) {
            Ok(false) => {}
            Err(error) => {
                spacing_uncertainty.get_or_insert(error);
            }
            Ok(true) => {
                let distance = measurements::point_distance(&left.location, point)?;
                // This ordering selects an optional marker pair. A previously
                // certified too-close pair already proves the finding; an
                // unresolved preference cannot erase that proof. The outer
                // operation still rejects shared-budget exhaustion.
                if too_close.as_ref().is_none_or(|(_, best)| {
                    matches!(
                        crate::scalar::try_compare(&distance, best),
                        Ok(std::cmp::Ordering::Less)
                    )
                }) {
                    too_close = Some((right, distance));
                }
            }
        }
    }
    if let Some((right, distance)) = too_close {
        return Ok(Neighbors::TooClose { right, distance });
    }
    if has_neighbor {
        spacing_uncertainty.map_or(Ok(Neighbors::Valid), Err)
    } else {
        membership_uncertainty.map_or(Ok(Neighbors::Missing), Err)
    }
}
