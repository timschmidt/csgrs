//! Exact Real and point ordering helpers.
//!
//! These helpers are predicates rather than algebra: ordering is a decision
//! about the sign of a Real difference, so provenance belongs in
//! [`PredicateOutcome`].

use crate::predicate::PredicatePolicy;
use core::cmp::Ordering;

use crate::geometry::{Point2, Point3};
use crate::predicate::{
    Certainty, Escalation, PredicateOutcome, PredicateReport, RefinementNeed, Sign,
};
use crate::real::sub_ref;
use crate::resolve::{map_outcome, resolve_real_sign};
use hyperreal::Real;

/// Decide the sign of one Real value through the predicate pipeline.
pub fn classify_real_sign(value: &Real) -> PredicateOutcome<Sign> {
    classify_real_sign_with_policy(value, PredicatePolicy::default())
}

/// Decide the sign of one Real value and return predicate provenance.
pub fn classify_real_sign_report(value: &Real) -> PredicateReport<Sign> {
    classify_real_sign_report_with_policy(value, PredicatePolicy::default())
}

/// Decide the sign of one Real value with an explicit predicate policy.
pub(crate) fn classify_real_sign_with_policy(
    value: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    classify_real_sign_report_with_policy(value, policy).outcome
}

/// Decide the sign of one Real value with an explicit predicate policy and
/// provenance certificate.
pub(crate) fn classify_real_sign_report_with_policy(
    value: &Real,
    policy: PredicatePolicy,
) -> PredicateReport<Sign> {
    PredicateReport::from_outcome(resolve_real_sign(
        value,
        policy,
        || None,
        || None,
        RefinementNeed::RealRefinement,
    ))
}

/// Compare two Real values by deciding the sign of `left - right`.
pub fn compare_reals(left: &Real, right: &Real) -> PredicateOutcome<Ordering> {
    compare_reals_with_policy(left, right, PredicatePolicy::default())
}

/// Compare two Real values and return predicate provenance.
pub fn compare_reals_report(left: &Real, right: &Real) -> PredicateReport<Ordering> {
    compare_reals_report_with_policy(left, right, PredicatePolicy::default())
}

/// Compare two Real values with an explicit predicate escalation policy.
///
/// This keeps Real ordering on the same exact predicate pipeline as
/// orientation and incidence tests. Higher crates use it for leftmost-vertex
/// selection, ray-crossing tests, interval comparisons, and deterministic tie
/// breaking without importing primitive-float ordering into topology code. The
/// design follows Yap's exact-geometric-computation split: numerical structure
/// may be carried by Real objects, while geometric decisions ask a predicate
/// layer to certify signs. See Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997).
pub fn compare_reals_with_policy(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    compare_reals_report_with_policy(left, right, policy).outcome
}

/// Compare two Real values with an explicit policy and provenance certificate.
///
/// Ordering predicates are often used as sub-decisions in intervals,
/// sweep-line queues, and boundary classifiers. Returning a report makes that
/// sub-decision auditable without changing the lightweight
/// [`compare_reals_with_policy`] API. This is the report-level side of Yap's
/// exact-geometric-computation boundary: approximate views remain outside the
/// topology path, while certified sign decisions expose how they were decided.
/// See Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997).
pub(crate) fn compare_reals_report_with_policy(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> PredicateReport<Ordering> {
    crate::trace_dispatch!("hyperlimit", "compare_reals", "difference-sign");
    let difference = sub_ref(left, right);
    PredicateReport::from_outcome(map_outcome(
        resolve_real_sign(
            &difference,
            policy,
            || None,
            || None,
            RefinementNeed::RealRefinement,
        ),
        ordering_from_sign,
    ))
}

/// Return whether `left <= right` under the exact Real ordering predicate.
pub fn real_le(left: &Real, right: &Real) -> PredicateOutcome<bool> {
    real_le_with_policy(left, right, PredicatePolicy::default())
}

/// Policy-controlled variant of [`real_le`].
pub(crate) fn real_le_with_policy(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    map_outcome(compare_reals_with_policy(left, right, policy), |ordering| {
        matches!(ordering, Ordering::Less | Ordering::Equal)
    })
}

/// Return whether `left >= right` under the exact Real ordering predicate.
pub fn real_ge(left: &Real, right: &Real) -> PredicateOutcome<bool> {
    real_ge_with_policy(left, right, PredicatePolicy::default())
}

/// Policy-controlled variant of [`real_ge`].
pub(crate) fn real_ge_with_policy(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    map_outcome(compare_reals_with_policy(left, right, policy), |ordering| {
        matches!(ordering, Ordering::Greater | Ordering::Equal)
    })
}

/// Return the smaller of two Real references using the exact ordering predicate.
pub fn real_min<'a>(left: &'a Real, right: &'a Real) -> PredicateOutcome<&'a Real> {
    real_min_with_policy(left, right, PredicatePolicy::default())
}

/// Policy-controlled variant of [`real_min`].
pub(crate) fn real_min_with_policy<'a>(
    left: &'a Real,
    right: &'a Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<&'a Real> {
    map_outcome(compare_reals_with_policy(left, right, policy), |ordering| {
        if ordering == Ordering::Greater {
            right
        } else {
            left
        }
    })
}

/// Return the larger of two Real references using the exact ordering predicate.
pub fn real_max<'a>(left: &'a Real, right: &'a Real) -> PredicateOutcome<&'a Real> {
    real_max_with_policy(left, right, PredicatePolicy::default())
}

/// Policy-controlled variant of [`real_max`].
pub(crate) fn real_max_with_policy<'a>(
    left: &'a Real,
    right: &'a Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<&'a Real> {
    map_outcome(compare_reals_with_policy(left, right, policy), |ordering| {
        if ordering == Ordering::Less {
            right
        } else {
            left
        }
    })
}

/// Clamp a Real value to an exact Real interval.
pub fn real_clamp(value: Real, min: &Real, max: &Real) -> PredicateOutcome<Real> {
    real_clamp_with_policy(value, min, max, PredicatePolicy::default())
}

/// Policy-controlled variant of [`real_clamp`].
pub(crate) fn real_clamp_with_policy(
    value: Real,
    min: &Real,
    max: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Real> {
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;

    let min_max = match compare_reals_with_policy(min, max, policy) {
        PredicateOutcome::Decided {
            value,
            certainty: value_certainty,
            stage: value_stage,
        } => {
            certainty = max_certainty(certainty, value_certainty);
            stage = max_stage(stage, value_stage);
            value
        }
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };
    if min_max == Ordering::Greater {
        return PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided);
    }

    match compare_reals_with_policy(&value, min, policy) {
        PredicateOutcome::Decided {
            value: Ordering::Less,
            certainty: value_certainty,
            stage: value_stage,
        } => {
            certainty = max_certainty(certainty, value_certainty);
            stage = max_stage(stage, value_stage);
            return PredicateOutcome::decided(min.clone(), certainty, stage);
        }
        PredicateOutcome::Decided {
            certainty: value_certainty,
            stage: value_stage,
            ..
        } => {
            certainty = max_certainty(certainty, value_certainty);
            stage = max_stage(stage, value_stage);
        }
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    }

    match compare_reals_with_policy(&value, max, policy) {
        PredicateOutcome::Decided {
            value: Ordering::Greater,
            certainty: value_certainty,
            stage: value_stage,
        } => {
            certainty = max_certainty(certainty, value_certainty);
            stage = max_stage(stage, value_stage);
            PredicateOutcome::decided(max.clone(), certainty, stage)
        }
        PredicateOutcome::Decided {
            certainty: value_certainty,
            stage: value_stage,
            ..
        } => {
            certainty = max_certainty(certainty, value_certainty);
            stage = max_stage(stage, value_stage);
            PredicateOutcome::decided(value, certainty, stage)
        }
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Compare two 2D points lexicographically by `(x, y)`.
pub fn compare_point2_lexicographic(left: &Point2, right: &Point2) -> PredicateOutcome<Ordering> {
    compare_point2_lexicographic_with_policy(left, right, PredicatePolicy::default())
}

/// Compare two 2D points lexicographically with predicate provenance.
pub fn compare_point2_lexicographic_report(
    left: &Point2,
    right: &Point2,
) -> PredicateReport<Ordering> {
    compare_point2_lexicographic_report_with_policy(left, right, PredicatePolicy::default())
}

/// Compare two 2D points lexicographically by `(x, y)` with an explicit policy.
///
/// This is useful for deterministic exact event queues and canonical endpoint
/// ordering. It deliberately does not impose polygon, segment, or sweep-line
/// topology; it only composes two Real ordering predicates.
pub(crate) fn compare_point2_lexicographic_with_policy(
    left: &Point2,
    right: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    compare_point2_lexicographic_report_with_policy(left, right, policy).outcome
}

/// Compare two 2D points lexicographically with an explicit policy and report.
pub(crate) fn compare_point2_lexicographic_report_with_policy(
    left: &Point2,
    right: &Point2,
    policy: PredicatePolicy,
) -> PredicateReport<Ordering> {
    match compare_reals_report_with_policy(&left.x, &right.x, policy).outcome {
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            certainty: x_certainty,
            stage: x_stage,
        } => match compare_reals_report_with_policy(&left.y, &right.y, policy).outcome {
            PredicateOutcome::Decided {
                value,
                certainty: y_certainty,
                stage: y_stage,
            } => PredicateReport::from_outcome(PredicateOutcome::decided(
                value,
                max_certainty(x_certainty, y_certainty),
                max_stage(x_stage, y_stage),
            )),
            PredicateOutcome::Unknown { needed, stage } => {
                PredicateReport::from_outcome(PredicateOutcome::unknown(needed, stage))
            }
        },
        decided_or_unknown => PredicateReport::from_outcome(decided_or_unknown),
    }
}

/// Return whether two 2D points have equal coordinates.
pub fn point2_equal(left: &Point2, right: &Point2) -> PredicateOutcome<bool> {
    point2_equal_with_policy(left, right, PredicatePolicy::default())
}

/// Return whether two 2D points have equal coordinates with provenance.
pub fn point2_equal_report(left: &Point2, right: &Point2) -> PredicateReport<bool> {
    point2_equal_report_with_policy(left, right, PredicatePolicy::default())
}

/// Return whether two 2D points have equal coordinates with an explicit
/// predicate escalation policy.
///
/// Point equality is an exact predicate over Real coordinate differences.
/// Keeping it here avoids each arrangement, curve, or triangulation crate
/// reimplementing "compare x, then compare y" with slightly different
/// uncertainty handling. The equality decision follows the exact computation
/// boundary described by Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997).
pub(crate) fn point2_equal_with_policy(
    left: &Point2,
    right: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    point2_equal_report_with_policy(left, right, policy).outcome
}

/// Return whether two 2D points have equal coordinates with explicit policy
/// and provenance.
pub(crate) fn point2_equal_report_with_policy(
    left: &Point2,
    right: &Point2,
    policy: PredicatePolicy,
) -> PredicateReport<bool> {
    PredicateReport::from_outcome(map_outcome(
        compare_point2_lexicographic_report_with_policy(left, right, policy).outcome,
        |ordering| ordering == Ordering::Equal,
    ))
}

/// Compare two 3D points lexicographically by `(x, y, z)`.
pub fn compare_point3_lexicographic(left: &Point3, right: &Point3) -> PredicateOutcome<Ordering> {
    compare_point3_lexicographic_with_policy(left, right, PredicatePolicy::default())
}

/// Compare two 3D points lexicographically with predicate provenance.
pub fn compare_point3_lexicographic_report(
    left: &Point3,
    right: &Point3,
) -> PredicateReport<Ordering> {
    compare_point3_lexicographic_report_with_policy(left, right, PredicatePolicy::default())
}

/// Compare two 3D points lexicographically by `(x, y, z)` with an explicit
/// policy.
///
/// This is the 3D counterpart to [`compare_point2_lexicographic`]. It composes
/// exact Real ordering predicates for deterministic canonicalization and
/// equality decisions without routing coordinate equality through an unrelated
/// geometric primitive such as a zero-radius sphere.
pub(crate) fn compare_point3_lexicographic_with_policy(
    left: &Point3,
    right: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    compare_point3_lexicographic_report_with_policy(left, right, policy).outcome
}

/// Compare two 3D points lexicographically with an explicit policy and report.
pub(crate) fn compare_point3_lexicographic_report_with_policy(
    left: &Point3,
    right: &Point3,
    policy: PredicatePolicy,
) -> PredicateReport<Ordering> {
    match compare_reals_report_with_policy(&left.x, &right.x, policy).outcome {
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            certainty: x_certainty,
            stage: x_stage,
        } => match compare_reals_report_with_policy(&left.y, &right.y, policy).outcome {
            PredicateOutcome::Decided {
                value: Ordering::Equal,
                certainty: y_certainty,
                stage: y_stage,
            } => match compare_reals_report_with_policy(&left.z, &right.z, policy).outcome {
                PredicateOutcome::Decided {
                    value,
                    certainty: z_certainty,
                    stage: z_stage,
                } => PredicateReport::from_outcome(PredicateOutcome::decided(
                    value,
                    max_certainty(max_certainty(x_certainty, y_certainty), z_certainty),
                    max_stage(max_stage(x_stage, y_stage), z_stage),
                )),
                PredicateOutcome::Unknown { needed, stage } => {
                    PredicateReport::from_outcome(PredicateOutcome::unknown(needed, stage))
                }
            },
            PredicateOutcome::Decided {
                value,
                certainty: y_certainty,
                stage: y_stage,
            } => PredicateReport::from_outcome(PredicateOutcome::decided(
                value,
                max_certainty(x_certainty, y_certainty),
                max_stage(x_stage, y_stage),
            )),
            PredicateOutcome::Unknown { needed, stage } => {
                PredicateReport::from_outcome(PredicateOutcome::unknown(needed, stage))
            }
        },
        decided_or_unknown => PredicateReport::from_outcome(decided_or_unknown),
    }
}

/// Return whether two 3D points have equal coordinates.
pub fn point3_equal(left: &Point3, right: &Point3) -> PredicateOutcome<bool> {
    point3_equal_with_policy(left, right, PredicatePolicy::default())
}

/// Return whether two 3D points have equal coordinates with provenance.
pub fn point3_equal_report(left: &Point3, right: &Point3) -> PredicateReport<bool> {
    point3_equal_report_with_policy(left, right, PredicatePolicy::default())
}

/// Return whether two 3D points have equal coordinates with an explicit
/// predicate escalation policy.
///
/// Point equality is an exact predicate over Real coordinate differences.
/// Keeping the 3D form beside [`point2_equal`] gives callers a direct semantic
/// API for vertex identity and normal-row deduplication instead of requiring a
/// zero-radius sphere classification.
pub(crate) fn point3_equal_with_policy(
    left: &Point3,
    right: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    point3_equal_report_with_policy(left, right, policy).outcome
}

/// Return whether two 3D points have equal coordinates with explicit policy
/// and provenance.
pub(crate) fn point3_equal_report_with_policy(
    left: &Point3,
    right: &Point3,
    policy: PredicatePolicy,
) -> PredicateReport<bool> {
    PredicateReport::from_outcome(map_outcome(
        compare_point3_lexicographic_report_with_policy(left, right, policy).outcome,
        |ordering| ordering == Ordering::Equal,
    ))
}

#[inline(always)]
fn ordering_from_sign(sign: Sign) -> Ordering {
    match sign {
        Sign::Negative => Ordering::Less,
        Sign::Zero => Ordering::Equal,
        Sign::Positive => Ordering::Greater,
    }
}

fn max_certainty(left: Certainty, right: Certainty) -> Certainty {
    if certainty_rank(left) >= certainty_rank(right) {
        left
    } else {
        right
    }
}

fn certainty_rank(certainty: Certainty) -> u8 {
    match certainty {
        Certainty::Exact => 0,
        Certainty::Filtered => 1,
    }
}

fn max_stage(left: Escalation, right: Escalation) -> Escalation {
    if stage_rank(left) >= stage_rank(right) {
        left
    } else {
        right
    }
}

fn stage_rank(stage: Escalation) -> u8 {
    match stage {
        Escalation::Structural => 0,
        Escalation::Filter => 1,
        Escalation::Exact => 2,
        Escalation::Refined => 3,
        Escalation::Undecided => 4,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn real(value: i32) -> hyperreal::Real {
        hyperreal::Real::from(value)
    }

    #[test]
    fn real_ordering_uses_exact_difference_sign() {
        assert_eq!(
            compare_reals(&real(1), &real(2)).value(),
            Some(Ordering::Less)
        );
        assert_eq!(
            compare_reals(&real(2), &real(2)).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(&real(3), &real(2)).value(),
            Some(Ordering::Greater)
        );
    }

    #[test]
    fn real_ordering_report_exposes_sign_decision_certificate() {
        let report = compare_reals_report(&real(7), &real(3));

        assert_eq!(report.value(), Some(Ordering::Greater));
        assert_eq!(
            report.certificate,
            crate::PredicateCertificate::StructuralFact
        );
    }

    #[test]
    fn point2_lexicographic_ordering_uses_y_as_tie_breaker() {
        let left = Point2::new(real(1), real(4));
        let right = Point2::new(real(1), real(5));

        assert_eq!(
            compare_point2_lexicographic(&left, &right).value(),
            Some(Ordering::Less)
        );
    }

    #[test]
    fn point2_equal_uses_exact_coordinate_ordering() {
        let left = Point2::new(real(1), real(4));
        let same = Point2::new(real(1), real(4));
        let different = Point2::new(real(1), real(5));

        assert_eq!(point2_equal(&left, &same).value(), Some(true));
        assert_eq!(point2_equal(&left, &different).value(), Some(false));
    }

    #[test]
    fn point3_lexicographic_ordering_uses_z_as_second_tie_breaker() {
        let left = Point3::new(real(1), real(4), real(6));
        let right = Point3::new(real(1), real(4), real(7));

        assert_eq!(
            compare_point3_lexicographic(&left, &right).value(),
            Some(Ordering::Less)
        );
    }

    #[test]
    fn point3_equal_uses_exact_coordinate_ordering() {
        let left = Point3::new(real(1), real(4), real(6));
        let same = Point3::new(real(1), real(4), real(6));
        let different = Point3::new(real(1), real(4), real(7));

        assert_eq!(point3_equal(&left, &same).value(), Some(true));
        assert_eq!(point3_equal(&left, &different).value(), Some(false));
    }

    #[test]
    fn point_ordering_and_equality_reports_match_outcome_apis() {
        let left = Point2::new(real(1), real(4));
        let right = Point2::new(real(1), real(5));
        let left3 = Point3::new(real(1), real(4), real(6));
        let right3 = Point3::new(real(1), real(4), real(7));

        assert_eq!(
            compare_point2_lexicographic_report(&left, &right).value(),
            compare_point2_lexicographic(&left, &right).value()
        );
        assert_eq!(
            point2_equal_report(&left, &right).value(),
            point2_equal(&left, &right).value()
        );
        assert_eq!(
            compare_point3_lexicographic_report(&left3, &right3).value(),
            compare_point3_lexicographic(&left3, &right3).value()
        );
        assert_eq!(
            point3_equal_report(&left3, &right3).value(),
            point3_equal(&left3, &right3).value()
        );
    }

    #[test]
    fn real_min_max_clamp_and_bounds_use_order_predicates() {
        let low = real(1);
        let mid = real(2);
        let high = real(3);

        assert_eq!(real_le(&low, &mid).value(), Some(true));
        assert_eq!(real_ge(&high, &mid).value(), Some(true));
        assert_eq!(real_min(&high, &low).value(), Some(&low));
        assert_eq!(real_max(&high, &low).value(), Some(&high));
        assert_eq!(real_clamp(mid.clone(), &low, &high).value(), Some(mid));
        assert_eq!(real_clamp(real(0), &low, &high).value(), Some(low.clone()));
        assert_eq!(real_clamp(real(4), &low, &high).value(), Some(high));
    }
}
