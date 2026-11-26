//! Policy-controlled signs of closed interval and ball enclosures.
//!
//! Endpoint comparisons can certify a uniform sign or consume an approximate
//! terminal allowed by the policy. Results preserve that qualification; the
//! `certified_*` names alone do not imply a certified outcome. Callers supplying
//! enclosures must establish that their target value lies within them: these
//! functions receive no target or source relation to validate.
//!
//! A filter-stage result summarizes the local enclosure decision, not the
//! deepest endpoint-comparison stage or inherited source provenance. A decline
//! is inconclusive and supplies no total-work or memory-bound guarantee.

use crate::predicate::PredicatePolicy;
use core::cmp::Ordering;

use hyperreal::Real;

use crate::predicate::{Certainty, Escalation, PredicateOutcome, RefinementNeed, Sign};
use crate::predicates::order::compare_reals_with_policy;

/// Classify a closed ball enclosure, preserving invalid-radius uncertainty.
///
/// For a nonnegative radius the enclosure is `[center-radius, center+radius]`.
/// A negative radius comparison yields Unknown/Unsupported; an unresolved
/// radius or an interval with no uniform decided sign yields
/// Unknown/RealRefinement. Both use Filter stage and discard child failure
/// details. Decided signs retain the weakest qualification of radius and
/// endpoint comparisons, including Approximate when the policy permits it.
pub fn classify_ball_sign_with_policy(
    center: &Real,
    radius: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    match certified_ball_sign_outcome_with_policy(center, radius, policy) {
        BallFilterResult::Decided(outcome) => outcome,
        BallFilterResult::Uncertain => {
            PredicateOutcome::unknown(RefinementNeed::RealRefinement, Escalation::Filter)
        }
        BallFilterResult::InvalidRadius => {
            PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Filter)
        }
    }
}

/// Tries to classify a closed ball's sign under the supplied policy.
///
/// `Some` contains a Decided outcome, which can be Approximate; inspect its
/// certainty before using it as proof. `None` merges invalid-radius and
/// inconclusive cases. Use [`classify_ball_sign_with_policy`] to distinguish
/// their Unsupported and RealRefinement hints.
pub fn certified_ball_sign_with_policy(
    center: &Real,
    radius: &Real,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    match certified_ball_sign_outcome_with_policy(center, radius, policy) {
        BallFilterResult::Decided(outcome) => Some(outcome),
        BallFilterResult::Uncertain | BallFilterResult::InvalidRadius => None,
    }
}

/// Tries to classify the uniform sign of a closed interval's endpoint values.
///
/// Endpoints may arrive in either order. Compares the first and then the second
/// with zero, stopping at an Unknown. Two positive, two negative, or two zero
/// comparisons yield that sign; mixed signs, including a single zero endpoint,
/// return `None`. It does not compare endpoints with each other.
///
/// Successful results use Filter stage and Filtered certainty when both child
/// comparisons certify, or Approximate if either consumed the policy terminal.
/// Thus an approximate Zero is not an equality proof. `None` discards child
/// uncertainty details and also covers intervals that cross or touch zero.
/// This optional shape is suitable for a predicate resolver's filter callback.
pub fn certified_interval_sign_with_policy(
    first: &Real,
    second: &Real,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    crate::trace_dispatch!("hyperlimit", "certified_interval_sign", "start");
    let zero = Real::from(0);

    // Consume report-bearing comparisons so terminal qualification survives.
    // The enclosure result retains certainty, but not each child's stage.
    let (first_cmp, first_certainty) =
        ordering_and_certainty(compare_reals_with_policy(first, &zero, policy))?;
    let (second_cmp, second_certainty) =
        ordering_and_certainty(compare_reals_with_policy(second, &zero, policy))?;
    let certainty = filter_certainty(first_certainty, second_certainty);
    let lower_cmp = min_ordering(first_cmp, second_cmp);
    let upper_cmp = max_ordering(first_cmp, second_cmp);

    match (lower_cmp, upper_cmp) {
        (Ordering::Greater, Ordering::Greater) => {
            crate::trace_dispatch!("hyperlimit", "certified_interval_sign", "positive");
            Some(filtered(Sign::Positive, certainty))
        }
        (Ordering::Less, Ordering::Less) => {
            crate::trace_dispatch!("hyperlimit", "certified_interval_sign", "negative");
            Some(filtered(Sign::Negative, certainty))
        }
        (Ordering::Equal, Ordering::Equal) => {
            crate::trace_dispatch!("hyperlimit", "certified_interval_sign", "zero");
            Some(filtered(Sign::Zero, certainty))
        }
        _ => {
            crate::trace_dispatch!("hyperlimit", "certified_interval_sign", "crosses-zero");
            None
        }
    }
}

/// Internal ball classification, separating a negative radius from a decline.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum BallFilterResult {
    /// Radius and endpoint decisions supplied a uniform sign.
    Decided(
        /// Local sign receipt; the constructor path supplies a Decided outcome.
        PredicateOutcome<Sign>,
    ),
    /// Radius ordering was unresolved or endpoint signs did not give one sign.
    Uncertain,
    /// The radius comparison returned Less; its child receipt is not retained.
    InvalidRadius,
}

/// Checks radius sign before constructing the two exact ball endpoints.
///
/// A nonnegative decided radius permits interval classification, whose certainty
/// is joined with the radius receipt. Failure paths collapse child diagnostics
/// into this module's result categories; the caller chooses Option or Unknown.
fn certified_ball_sign_outcome_with_policy(
    center: &Real,
    radius: &Real,
    policy: PredicatePolicy,
) -> BallFilterResult {
    crate::trace_dispatch!("hyperlimit", "certified_ball_sign", "start");
    let zero = Real::from(0);
    let radius_certainty = match compare_reals_with_policy(radius, &zero, policy) {
        PredicateOutcome::Decided {
            value: Ordering::Less,
            ..
        } => {
            crate::trace_dispatch!("hyperlimit", "certified_ball_sign", "invalid-radius");
            return BallFilterResult::InvalidRadius;
        }
        PredicateOutcome::Decided {
            value: Ordering::Equal | Ordering::Greater,
            certainty,
            ..
        } => certainty,
        PredicateOutcome::Unknown { .. } => {
            crate::trace_dispatch!("hyperlimit", "certified_ball_sign", "radius-unknown");
            return BallFilterResult::Uncertain;
        }
    };

    let lower = center - radius;
    let upper = center + radius;
    match certified_interval_sign_with_policy(&lower, &upper, policy) {
        Some(PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        }) => {
            crate::trace_dispatch!("hyperlimit", "certified_ball_sign", "decided");
            BallFilterResult::Decided(PredicateOutcome::decided(
                value,
                weaker_certainty(radius_certainty, certainty),
                stage,
            ))
        }
        Some(PredicateOutcome::Unknown { .. }) | None => {
            crate::trace_dispatch!("hyperlimit", "certified_ball_sign", "uncertain");
            BallFilterResult::Uncertain
        }
    }
}

/// Attaches Filter stage to the supplied sign and already-composed certainty.
#[inline(always)]
fn filtered(sign: Sign, certainty: Certainty) -> PredicateOutcome<Sign> {
    PredicateOutcome::decided(sign, certainty, Escalation::Filter)
}

/// Extracts a decided ordering/qualification, dropping stage and Unknown details.
#[inline(always)]
fn ordering_and_certainty(outcome: PredicateOutcome<Ordering>) -> Option<(Ordering, Certainty)> {
    match outcome {
        PredicateOutcome::Decided {
            value, certainty, ..
        } => Some((value, certainty)),
        PredicateOutcome::Unknown { .. } => None,
    }
}

/// Marks a certified endpoint pair Filtered, retaining any Approximate child.
#[inline(always)]
const fn filter_certainty(left: Certainty, right: Certainty) -> Certainty {
    match weaker_certainty(left, right) {
        Certainty::Approximate => Certainty::Approximate,
        Certainty::Exact | Certainty::Filtered => Certainty::Filtered,
    }
}

/// Joins qualifications in the order Exact, Filtered, Approximate.
#[inline(always)]
const fn weaker_certainty(left: Certainty, right: Certainty) -> Certainty {
    match (left, right) {
        (Certainty::Approximate, _) | (_, Certainty::Approximate) => Certainty::Approximate,
        (Certainty::Filtered, _) | (_, Certainty::Filtered) => Certainty::Filtered,
        (Certainty::Exact, Certainty::Exact) => Certainty::Exact,
    }
}

/// Selects the lower sign-order rank, choosing the left operand on ties.
#[inline(always)]
fn min_ordering(left: Ordering, right: Ordering) -> Ordering {
    if ordering_rank(left) <= ordering_rank(right) {
        left
    } else {
        right
    }
}

/// Selects the higher sign-order rank, choosing the left operand on ties.
#[inline(always)]
fn max_ordering(left: Ordering, right: Ordering) -> Ordering {
    if ordering_rank(left) >= ordering_rank(right) {
        left
    } else {
        right
    }
}

/// Encodes Less, Equal and Greater as -1, 0 and 1 for endpoint sign bounds.
#[inline(always)]
fn ordering_rank(ordering: Ordering) -> i8 {
    match ordering {
        Ordering::Less => -1,
        Ordering::Equal => 0,
        Ordering::Greater => 1,
    }
}

/// Integer enclosure certificates and symbolic terminal-qualification controls.
#[cfg(test)]
mod tests {
    use super::*;

    /// Permits terminal decisions while leaving exact integer cases certified.
    const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

    /// Preserves a proved unit-circle identity before returning the separate
    /// angle-addition zero used to exercise terminal-policy behavior.
    fn terminally_unresolved_zero() -> Real {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let original = &sine * &sine + &cosine * &cosine - Real::one();
        // Preserve the original identity as a newly certified proof case.
        assert_eq!(
            original.exact_rational_normal_form(),
            Some(hyperreal::Rational::zero())
        );
        // A separate angle-addition identity retains the terminal-policy control.
        crate::test_support::terminally_unresolved_zero()
    }

    /// Checks positive, negative and singleton-zero Filtered receipts and a crossing decline.
    #[test]
    fn certified_interval_sign_decides_strict_and_zero_enclosures() {
        assert_eq!(
            crate::certified_interval_sign(&Real::from(1), &Real::from(3), APPROX),
            Some(PredicateOutcome::decided(
                Sign::Positive,
                Certainty::Filtered,
                Escalation::Filter
            ))
        );
        assert_eq!(
            crate::certified_interval_sign(&Real::from(-7), &Real::from(-2), APPROX),
            Some(PredicateOutcome::decided(
                Sign::Negative,
                Certainty::Filtered,
                Escalation::Filter
            ))
        );
        assert_eq!(
            crate::certified_interval_sign(&Real::from(0), &Real::from(0), APPROX),
            Some(PredicateOutcome::decided(
                Sign::Zero,
                Certainty::Filtered,
                Escalation::Filter
            ))
        );
        assert_eq!(
            crate::certified_interval_sign(&Real::from(-1), &Real::from(1), APPROX),
            None
        );
    }

    /// Checks ball Filtered receipts for separated/zero cases and a crossing decline.
    #[test]
    fn certified_ball_sign_decides_strict_zero_and_crossing_balls() {
        assert_eq!(
            crate::certified_ball_sign(&Real::from(5), &Real::from(2), APPROX),
            Some(PredicateOutcome::decided(
                Sign::Positive,
                Certainty::Filtered,
                Escalation::Filter
            ))
        );
        assert_eq!(
            crate::certified_ball_sign(&Real::from(-5), &Real::from(2), APPROX),
            Some(PredicateOutcome::decided(
                Sign::Negative,
                Certainty::Filtered,
                Escalation::Filter
            ))
        );
        assert_eq!(
            crate::certified_ball_sign(&Real::from(0), &Real::from(0), APPROX),
            Some(PredicateOutcome::decided(
                Sign::Zero,
                Certainty::Filtered,
                Escalation::Filter
            ))
        );
        assert_eq!(
            crate::certified_ball_sign(&Real::from(1), &Real::from(2), APPROX),
            None
        );
    }

    /// Checks strict declines, approximate zero receipts, invalid-radius hint/Option
    /// behavior and the lower-rank ordering helper on a reversed pair.
    #[test]
    fn interval_and_ball_filters_preserve_terminal_approximation_certainty() {
        let undecidable = terminally_unresolved_zero();

        assert_eq!(
            certified_interval_sign_with_policy(
                &undecidable,
                &undecidable,
                PredicatePolicy::STRICT
            ),
            None
        );
        assert_eq!(
            certified_interval_sign_with_policy(&undecidable, &undecidable, APPROX,),
            Some(PredicateOutcome::decided(
                Sign::Zero,
                Certainty::Approximate,
                Escalation::Filter,
            ))
        );
        assert_eq!(
            classify_ball_sign_with_policy(&undecidable, &Real::zero(), APPROX,),
            PredicateOutcome::decided(Sign::Zero, Certainty::Approximate, Escalation::Filter,)
        );
        assert_eq!(
            classify_ball_sign_with_policy(&Real::zero(), &undecidable, PredicatePolicy::STRICT,),
            PredicateOutcome::unknown(RefinementNeed::RealRefinement, Escalation::Filter)
        );
        assert_eq!(
            classify_ball_sign_with_policy(&Real::zero(), &Real::from(-1), APPROX),
            PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Filter)
        );
        assert_eq!(
            certified_ball_sign_with_policy(&Real::zero(), &Real::from(-1), APPROX),
            None
        );

        assert_eq!(
            min_ordering(Ordering::Greater, Ordering::Less),
            Ordering::Less
        );
    }
}
