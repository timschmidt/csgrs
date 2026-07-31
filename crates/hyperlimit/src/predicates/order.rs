//! Exact Real and point ordering helpers.
//!
//! These helpers are predicates rather than algebra: ordering is a decision
//! about the sign of a Real difference, so provenance belongs in
//! [`PredicateOutcome`].

use crate::predicate::PredicatePolicy;
use core::cmp::Ordering;

use crate::geometry::{Point2, Point3};
use crate::predicate::{Certainty, Escalation, PredicateOutcome, RefinementNeed, Sign};
use crate::real::sub_ref;
use crate::resolve::{map_outcome, resolve_real_sign_direct};
use hyperreal::{CertifiedRealOrdering, Problem, Rational, Real, RealOrderingCertificate};

/// Decide the sign of one Real value with an explicit predicate policy.
pub fn classify_real_sign_with_policy(
    value: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    resolve_real_sign_direct(value, policy, RefinementNeed::RealRefinement)
}

/// Construct a reciprocal after deciding nonzero status through the supplied
/// predicate policy.
///
/// The returned outcome retains the certainty and escalation stage of the
/// denominator decision. This is the construction counterpart to
/// [`classify_real_sign_with_policy`]: downstream exact geometry can reuse one policy
/// decision instead of asking `Real` division to repeat the same potentially
/// expensive refinement for every coordinate.
pub fn reciprocal_real_with_policy(
    value: &Real,
    policy: PredicatePolicy,
) -> Result<PredicateOutcome<Real>, Problem> {
    match classify_real_sign_with_policy(value, policy) {
        PredicateOutcome::Decided {
            value: Sign::Zero, ..
        } => Err(Problem::DivideByZero),
        PredicateOutcome::Decided {
            certainty, stage, ..
        } => Ok(PredicateOutcome::decided(
            value.inverse_ref_assuming_nonzero()?,
            certainty,
            stage,
        )),
        PredicateOutcome::Unknown { needed, stage } => Ok(PredicateOutcome::unknown(needed, stage)),
    }
}

/// Decide two Real signs through one predicate cascade.
///
/// This is useful for paired domain checks and vector-like sign queries. It
/// batches the common exact-rational path while preserving the weakest
/// certainty and latest escalation stage when either value needs the general
/// resolver.
/// Decide two Real signs with an explicit predicate policy.
pub fn classify_real_sign_pair_with_policy(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<(Sign, Sign)> {
    if let (Some(left), Some(right)) = (left.exact_rational_ref(), right.exact_rational_ref()) {
        crate::trace_dispatch!("hyperlimit", "classify_real_sign_pair", "exact-rational");
        return PredicateOutcome::decided(
            (exact_rational_sign(left), exact_rational_sign(right)),
            Certainty::Exact,
            Escalation::Exact,
        );
    }

    crate::trace_dispatch!("hyperlimit", "classify_real_sign_pair", "scalar-cascades");
    match (
        classify_real_sign_with_policy(left, policy),
        classify_real_sign_with_policy(right, policy),
    ) {
        (
            PredicateOutcome::Decided {
                value: left,
                certainty: left_certainty,
                stage: left_stage,
            },
            PredicateOutcome::Decided {
                value: right,
                certainty: right_certainty,
                stage: right_stage,
            },
        ) => PredicateOutcome::decided(
            (left, right),
            max_certainty(left_certainty, right_certainty),
            max_stage(left_stage, right_stage),
        ),
        (PredicateOutcome::Unknown { needed, stage }, _)
        | (_, PredicateOutcome::Unknown { needed, stage }) => {
            PredicateOutcome::unknown(needed, stage)
        }
    }
}

/// Compare two Real values with an explicit predicate escalation policy.
///
/// This keeps Real ordering on the same exact predicate pipeline as
/// orientation and incidence tests. Higher crates use it for leftmost-vertex
/// selection, ray-crossing tests, interval comparisons, and deterministic tie
/// breaking without importing primitive-float ordering into topology code.
/// Numerical structure may be carried by Real objects, while geometric
/// decisions ask a predicate layer to certify signs.
#[inline]
pub fn compare_reals_with_policy(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    if let (Some(left), Some(right)) = (left.exact_rational_ref(), right.exact_rational_ref()) {
        crate::trace_dispatch!("hyperlimit", "compare_reals", "exact-rational");
        return PredicateOutcome::decided(
            left.partial_cmp(right)
                .expect("exact rational ordering is total"),
            Certainty::Exact,
            Escalation::Exact,
        );
    }

    match left.certified_cmp_until(right, PredicatePolicy::MAX_REFINEMENT_PRECISION) {
        CertifiedRealOrdering::Known {
            ordering,
            certificate,
        } => {
            let stage = match certificate {
                RealOrderingCertificate::StructuralEquality
                | RealOrderingCertificate::StructuralFacts
                | RealOrderingCertificate::DifferenceStructuralFacts => Escalation::Structural,
                RealOrderingCertificate::ExactRationalComparison => Escalation::Exact,
                RealOrderingCertificate::BoundedRefinement { .. } => Escalation::Refined,
            };
            crate::trace_dispatch!("hyperlimit", "compare_reals", "certified-real-ordering");
            PredicateOutcome::decided(ordering, Certainty::Exact, stage)
        }
        CertifiedRealOrdering::Unknown { .. } => {
            let difference = sub_ref(left, right);
            crate::trace_dispatch!("hyperlimit", "compare_reals", "difference-sign-cascade");
            map_outcome(
                classify_real_sign_with_policy(&difference, policy),
                |sign| match sign {
                    Sign::Negative => Ordering::Less,
                    Sign::Zero => Ordering::Equal,
                    Sign::Positive => Ordering::Greater,
                },
            )
        }
    }
}

/// Return whether `left <= right` under the policy-controlled Real ordering predicate.
pub fn real_le_with_policy(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    map_outcome(compare_reals_with_policy(left, right, policy), |ordering| {
        matches!(ordering, Ordering::Less | Ordering::Equal)
    })
}

/// Return whether `left >= right` under the policy-controlled Real ordering predicate.
pub fn real_ge_with_policy(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    map_outcome(compare_reals_with_policy(left, right, policy), |ordering| {
        matches!(ordering, Ordering::Greater | Ordering::Equal)
    })
}

/// Return the smaller of two Real references using the policy-controlled ordering predicate.
pub fn real_min_with_policy<'a>(
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

/// Return the larger of two Real references using the policy-controlled ordering predicate.
pub fn real_max_with_policy<'a>(
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

/// Clamp a Real value to an interval under an explicit predicate policy.
pub fn real_clamp_with_policy(
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

/// Compare two 2D points lexicographically by `(x, y)` with an explicit policy.
///
/// This is useful for deterministic exact event queues and canonical endpoint
/// ordering. It deliberately does not impose polygon, segment, or sweep-line
/// topology; it only composes two Real ordering predicates.
pub fn compare_point2_lexicographic_with_policy(
    left: &Point2,
    right: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    match compare_reals_with_policy(&left.x, &right.x, policy) {
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            certainty: x_certainty,
            stage: x_stage,
        } => match compare_reals_with_policy(&left.y, &right.y, policy) {
            PredicateOutcome::Decided {
                value,
                certainty: y_certainty,
                stage: y_stage,
            } => PredicateOutcome::decided(
                value,
                max_certainty(x_certainty, y_certainty),
                max_stage(x_stage, y_stage),
            ),
            PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
        },
        decided_or_unknown => decided_or_unknown,
    }
}

/// Return whether two 2D points have equal coordinates with an explicit
/// predicate escalation policy.
///
/// Point equality is an exact predicate over Real coordinate differences.
/// Keeping it here avoids each arrangement, curve, or triangulation crate
/// reimplementing "compare x, then compare y" with slightly different
/// uncertainty handling while preserving the exact-computation boundary.
pub fn point2_equal_with_policy(
    left: &Point2,
    right: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    if let (Some(left_x), Some(right_x)) =
        (left.x.exact_rational_ref(), right.x.exact_rational_ref())
    {
        if left_x != right_x {
            crate::trace_dispatch!("hyperlimit", "point2_equal", "exact-rational-x");
            return PredicateOutcome::decided(false, Certainty::Exact, Escalation::Exact);
        }
        if let (Some(left_y), Some(right_y)) =
            (left.y.exact_rational_ref(), right.y.exact_rational_ref())
        {
            crate::trace_dispatch!("hyperlimit", "point2_equal", "exact-rational-xy");
            return PredicateOutcome::decided(
                left_y == right_y,
                Certainty::Exact,
                Escalation::Exact,
            );
        }
    }

    crate::trace_dispatch!("hyperlimit", "point2_equal", "lexicographic-cascade");
    map_outcome(
        compare_point2_lexicographic_with_policy(left, right, policy),
        |ordering| ordering == Ordering::Equal,
    )
}

/// Compare two 3D points lexicographically by `(x, y, z)` with an explicit
/// policy.
///
/// This is the 3D counterpart to [`compare_point2_lexicographic_with_policy`]. It composes
/// exact Real ordering predicates for deterministic canonicalization and
/// equality decisions without routing coordinate equality through an unrelated
/// geometric primitive such as a zero-radius sphere.
pub fn compare_point3_lexicographic_with_policy(
    left: &Point3,
    right: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    match compare_reals_with_policy(&left.x, &right.x, policy) {
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            certainty: x_certainty,
            stage: x_stage,
        } => match compare_reals_with_policy(&left.y, &right.y, policy) {
            PredicateOutcome::Decided {
                value: Ordering::Equal,
                certainty: y_certainty,
                stage: y_stage,
            } => match compare_reals_with_policy(&left.z, &right.z, policy) {
                PredicateOutcome::Decided {
                    value,
                    certainty: z_certainty,
                    stage: z_stage,
                } => PredicateOutcome::decided(
                    value,
                    max_certainty(max_certainty(x_certainty, y_certainty), z_certainty),
                    max_stage(max_stage(x_stage, y_stage), z_stage),
                ),
                PredicateOutcome::Unknown { needed, stage } => {
                    PredicateOutcome::unknown(needed, stage)
                }
            },
            PredicateOutcome::Decided {
                value,
                certainty: y_certainty,
                stage: y_stage,
            } => PredicateOutcome::decided(
                value,
                max_certainty(x_certainty, y_certainty),
                max_stage(x_stage, y_stage),
            ),
            PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
        },
        decided_or_unknown => decided_or_unknown,
    }
}

/// Return whether two 3D points have equal coordinates with an explicit
/// predicate escalation policy.
///
/// Point equality is an exact predicate over Real coordinate differences.
/// Keeping the 3D form beside [`point2_equal_with_policy`] gives callers a direct semantic
/// API for vertex identity and normal-row deduplication instead of requiring a
/// zero-radius sphere classification.
pub fn point3_equal_with_policy(
    left: &Point3,
    right: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    map_outcome(
        compare_point3_lexicographic_with_policy(left, right, policy),
        |ordering| ordering == Ordering::Equal,
    )
}

#[inline(always)]
fn exact_rational_sign(value: &Rational) -> Sign {
    match value
        .partial_cmp(&Rational::zero())
        .expect("exact rational ordering is total")
    {
        Ordering::Less => Sign::Negative,
        Ordering::Equal => Sign::Zero,
        Ordering::Greater => Sign::Positive,
    }
}

#[inline(always)]
fn max_certainty(left: Certainty, right: Certainty) -> Certainty {
    if certainty_rank(left) >= certainty_rank(right) {
        left
    } else {
        right
    }
}

#[inline(always)]
fn certainty_rank(certainty: Certainty) -> u8 {
    match certainty {
        Certainty::Exact => 0,
        Certainty::Filtered => 1,
        Certainty::Approximate => 2,
    }
}

#[inline(always)]
fn max_stage(left: Escalation, right: Escalation) -> Escalation {
    if stage_rank(left) >= stage_rank(right) {
        left
    } else {
        right
    }
}

#[inline(always)]
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

    const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

    fn real(value: i32) -> hyperreal::Real {
        hyperreal::Real::from(value)
    }

    #[test]
    fn paired_signs_batch_exact_rationals_and_preserve_unknowns() {
        assert_eq!(
            crate::classify_real_sign_pair(&real(-1), &real(0), APPROX).value(),
            Some((Sign::Negative, Sign::Zero))
        );

        let undecidable = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
        assert!(matches!(
            classify_real_sign_pair_with_policy(&real(1), &undecidable, PredicatePolicy::STRICT),
            PredicateOutcome::Unknown { .. }
        ));
    }

    #[test]
    fn real_ordering_uses_exact_difference_sign() {
        assert_eq!(
            crate::compare_reals(&real(1), &real(2), APPROX).value(),
            Some(Ordering::Less)
        );
        assert_eq!(
            crate::compare_reals(&real(2), &real(2), APPROX).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            crate::compare_reals(&real(3), &real(2), APPROX).value(),
            Some(Ordering::Greater)
        );
    }

    #[test]
    fn exact_normal_form_resolves_nested_parameter_replay() {
        let root_two = real(2).sqrt().unwrap();
        let root_two_over_pi = (root_two.clone() / Real::pi()).unwrap();
        let half = (real(1) / real(2)).unwrap();
        let shared_offset = root_two.clone() * real(3) + half;
        let contact = (((root_two.clone() * real(4) - shared_offset.clone()) * Real::pi())
            * root_two_over_pi.clone()
            / real(4))
        .unwrap();
        let domain = (((root_two * real(2) - shared_offset) * Real::pi()) * root_two_over_pi
            / real(4))
        .unwrap()
            + real(1);

        assert_eq!(
            compare_reals_with_policy(&contact, &domain, PredicatePolicy::STRICT).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            classify_real_sign_with_policy(&(contact - domain), PredicatePolicy::STRICT).value(),
            Some(Sign::Zero)
        );
    }

    #[test]
    fn exact_normal_form_precedes_terminal_approximate_equality() {
        let root_two = real(2).sqrt().unwrap();
        let root_two_over_pi = (root_two.clone() / Real::pi()).unwrap();
        let half = (real(1) / real(2)).unwrap();
        let shared_offset = root_two.clone() * real(3) + half;
        let contact = (((root_two.clone() * real(4) - shared_offset.clone()) * Real::pi())
            * root_two_over_pi.clone()
            / real(4))
        .unwrap();
        let domain = (((root_two * real(2) - shared_offset) * Real::pi()) * root_two_over_pi
            / real(4))
        .unwrap()
            + real(1);
        let tiny = real(2).powi_i64(-600).unwrap();
        let positive = contact - domain + tiny;

        assert_eq!(
            classify_real_sign_with_policy(&positive, APPROX),
            PredicateOutcome::decided(Sign::Positive, Certainty::Exact, Escalation::Exact)
        );
        assert_eq!(
            compare_reals_with_policy(&positive, &Real::zero(), APPROX),
            PredicateOutcome::decided(Ordering::Greater, Certainty::Exact, Escalation::Exact)
        );

        let left = Point2::new(positive, Real::zero());
        let right = Point2::new(Real::zero(), Real::zero());
        assert_eq!(
            point2_equal_with_policy(&left, &right, APPROX),
            PredicateOutcome::decided(false, Certainty::Exact, Escalation::Exact)
        );
    }

    #[test]
    fn point2_lexicographic_ordering_uses_y_as_tie_breaker() {
        let left = Point2::new(real(1), real(4));
        let right = Point2::new(real(1), real(5));

        assert_eq!(
            crate::compare_point2_lexicographic(&left, &right, APPROX).value(),
            Some(Ordering::Less)
        );
    }

    #[test]
    fn point2_equal_uses_exact_coordinate_ordering() {
        let left = Point2::new(real(1), real(4));
        let same = Point2::new(real(1), real(4));
        let different = Point2::new(real(1), real(5));

        assert_eq!(
            crate::point2_equal(&left, &same, APPROX).value(),
            Some(true)
        );
        assert_eq!(
            crate::point2_equal(&left, &different, APPROX).value(),
            Some(false)
        );
    }

    #[test]
    fn point3_lexicographic_ordering_uses_z_as_second_tie_breaker() {
        let left = Point3::new(real(1), real(4), real(6));
        let right = Point3::new(real(1), real(4), real(7));

        assert_eq!(
            crate::compare_point3_lexicographic(&left, &right, APPROX).value(),
            Some(Ordering::Less)
        );
    }

    #[test]
    fn point3_equal_uses_exact_coordinate_ordering() {
        let left = Point3::new(real(1), real(4), real(6));
        let same = Point3::new(real(1), real(4), real(6));
        let different = Point3::new(real(1), real(4), real(7));

        assert_eq!(
            crate::point3_equal(&left, &same, APPROX).value(),
            Some(true)
        );
        assert_eq!(
            crate::point3_equal(&left, &different, APPROX).value(),
            Some(false)
        );
    }

    #[test]
    fn real_min_max_clamp_and_bounds_use_order_predicates() {
        let low = real(1);
        let mid = real(2);
        let high = real(3);

        assert_eq!(crate::real_le(&low, &mid, APPROX).value(), Some(true));
        assert_eq!(crate::real_ge(&high, &mid, APPROX).value(), Some(true));
        assert_eq!(crate::real_min(&high, &low, APPROX).value(), Some(&low));
        assert_eq!(crate::real_max(&high, &low, APPROX).value(), Some(&high));
        assert_eq!(
            crate::real_clamp(mid.clone(), &low, &high, APPROX).value(),
            Some(mid)
        );
        assert_eq!(
            crate::real_clamp(real(0), &low, &high, APPROX).value(),
            Some(low.clone())
        );
        assert_eq!(
            crate::real_clamp(real(4), &low, &high, APPROX).value(),
            Some(high)
        );
    }

    #[test]
    fn reciprocal_reuses_the_policy_nonzero_decision() {
        let reciprocal = crate::reciprocal_real(&real(2), APPROX)
            .expect("two is nonzero")
            .value()
            .expect("the reciprocal is decided");
        assert_eq!(
            crate::compare_reals(&(real(2) * reciprocal), &real(1), APPROX).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            crate::reciprocal_real(&real(0), APPROX),
            Err(Problem::DivideByZero)
        );

        let undecidable = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
        assert!(matches!(
            reciprocal_real_with_policy(&undecidable, PredicatePolicy::STRICT),
            Ok(PredicateOutcome::Unknown { .. })
        ));
    }
}
