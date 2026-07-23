//! Certified predicate filters.
//!
//! Filters in this module are exact, policy-visible shortcuts. They may decide
//! a predicate before expensive refinement, or return explicit uncertainty.
//! They are not primitive-float tolerances. Approximate or interval information is useful only
//! when it produces a certificate or a bounded non-decision.

use crate::predicate::PredicatePolicy;
use core::cmp::Ordering;

use hyperreal::Real;

use crate::predicate::{Certainty, Escalation, PredicateOutcome, RefinementNeed, Sign};
use crate::predicates::order::compare_reals_with_policy;

/// Classify a closed ball enclosure, preserving invalid-radius uncertainty.
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

/// Try to certify a sign from an exact closed ball enclosure.
///
/// Returns `Some` only when the nonnegative ball certifies a sign. Use
/// [`classify_ball_sign_with_policy`] when invalid-radius and inconclusive
/// outcomes must remain distinct.
pub fn certified_ball_sign(center: &Real, radius: &Real) -> Option<PredicateOutcome<Sign>> {
    certified_ball_sign_with_policy(center, radius, PredicatePolicy)
}

/// Try to certify a sign from an exact closed ball enclosure with policy.
pub(crate) fn certified_ball_sign_with_policy(
    center: &Real,
    radius: &Real,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    match certified_ball_sign_outcome_with_policy(center, radius, policy) {
        BallFilterResult::Decided(outcome) => Some(outcome),
        BallFilterResult::Uncertain | BallFilterResult::InvalidRadius => None,
    }
}

/// Try to certify a sign from an exact closed interval enclosure.
///
/// Returns `Some` only when the interval proves a sign. This shape is intended
/// for predicate filter callbacks such as `resolve_real_sign(..., || {
/// certified_interval_sign_with_policy(...) }, ...)`.
pub fn certified_interval_sign(first: &Real, second: &Real) -> Option<PredicateOutcome<Sign>> {
    certified_interval_sign_with_policy(first, second, PredicatePolicy)
}

/// Try to certify a sign from an exact closed interval enclosure with policy.
pub(crate) fn certified_interval_sign_with_policy(
    first: &Real,
    second: &Real,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    crate::trace_dispatch!("hyperlimit", "certified_interval_sign", "start");
    let zero = Real::from(0);

    // Endpoint comparisons are themselves exact predicates. Use their
    // report-bearing forms so trace/report users can audit the sub-decisions
    // that fed this interval certificate, keeping endpoint ordering inside the
    // certified-filter layer rather than treating it as anonymous scalar work.
    let first_cmp = compare_reals_with_policy(first, &zero, policy).value()?;
    let second_cmp = compare_reals_with_policy(second, &zero, policy).value()?;
    let lower_cmp = min_ordering(first_cmp, second_cmp);
    let upper_cmp = max_ordering(first_cmp, second_cmp);

    match (lower_cmp, upper_cmp) {
        (Ordering::Greater, Ordering::Greater) => {
            crate::trace_dispatch!("hyperlimit", "certified_interval_sign", "positive");
            Some(filtered(Sign::Positive))
        }
        (Ordering::Less, Ordering::Less) => {
            crate::trace_dispatch!("hyperlimit", "certified_interval_sign", "negative");
            Some(filtered(Sign::Negative))
        }
        (Ordering::Equal, Ordering::Equal) => {
            crate::trace_dispatch!("hyperlimit", "certified_interval_sign", "zero");
            Some(filtered(Sign::Zero))
        }
        _ => {
            crate::trace_dispatch!("hyperlimit", "certified_interval_sign", "crosses-zero");
            None
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum BallFilterResult {
    Decided(PredicateOutcome<Sign>),
    Uncertain,
    InvalidRadius,
}

fn certified_ball_sign_outcome_with_policy(
    center: &Real,
    radius: &Real,
    policy: PredicatePolicy,
) -> BallFilterResult {
    crate::trace_dispatch!("hyperlimit", "certified_ball_sign", "start");
    let zero = Real::from(0);
    match compare_reals_with_policy(radius, &zero, policy).value() {
        Some(Ordering::Less) => {
            crate::trace_dispatch!("hyperlimit", "certified_ball_sign", "invalid-radius");
            return BallFilterResult::InvalidRadius;
        }
        Some(Ordering::Equal | Ordering::Greater) => {}
        None => {
            crate::trace_dispatch!("hyperlimit", "certified_ball_sign", "radius-unknown");
            return BallFilterResult::Uncertain;
        }
    }

    let lower = center - radius;
    let upper = center + radius;
    match certified_interval_sign_with_policy(&lower, &upper, policy) {
        Some(outcome) => {
            crate::trace_dispatch!("hyperlimit", "certified_ball_sign", "decided");
            BallFilterResult::Decided(outcome)
        }
        None => {
            crate::trace_dispatch!("hyperlimit", "certified_ball_sign", "uncertain");
            BallFilterResult::Uncertain
        }
    }
}

#[inline(always)]
fn filtered(sign: Sign) -> PredicateOutcome<Sign> {
    PredicateOutcome::decided(sign, Certainty::Filtered, Escalation::Filter)
}

#[inline(always)]
fn min_ordering(left: Ordering, right: Ordering) -> Ordering {
    if ordering_rank(left) <= ordering_rank(right) {
        left
    } else {
        right
    }
}

#[inline(always)]
fn max_ordering(left: Ordering, right: Ordering) -> Ordering {
    if ordering_rank(left) >= ordering_rank(right) {
        left
    } else {
        right
    }
}

#[inline(always)]
fn ordering_rank(ordering: Ordering) -> i8 {
    match ordering {
        Ordering::Less => -1,
        Ordering::Equal => 0,
        Ordering::Greater => 1,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn certified_interval_sign_decides_strict_and_zero_enclosures() {
        assert_eq!(
            certified_interval_sign(&Real::from(1), &Real::from(3)),
            Some(PredicateOutcome::decided(
                Sign::Positive,
                Certainty::Filtered,
                Escalation::Filter
            ))
        );
        assert_eq!(
            certified_interval_sign(&Real::from(-7), &Real::from(-2)),
            Some(PredicateOutcome::decided(
                Sign::Negative,
                Certainty::Filtered,
                Escalation::Filter
            ))
        );
        assert_eq!(
            certified_interval_sign(&Real::from(0), &Real::from(0)),
            Some(PredicateOutcome::decided(
                Sign::Zero,
                Certainty::Filtered,
                Escalation::Filter
            ))
        );
        assert_eq!(
            certified_interval_sign(&Real::from(-1), &Real::from(1)),
            None
        );
    }

    #[test]
    fn certified_ball_sign_decides_strict_zero_and_crossing_balls() {
        assert_eq!(
            certified_ball_sign(&Real::from(5), &Real::from(2)),
            Some(PredicateOutcome::decided(
                Sign::Positive,
                Certainty::Filtered,
                Escalation::Filter
            ))
        );
        assert_eq!(
            certified_ball_sign(&Real::from(-5), &Real::from(2)),
            Some(PredicateOutcome::decided(
                Sign::Negative,
                Certainty::Filtered,
                Escalation::Filter
            ))
        );
        assert_eq!(
            certified_ball_sign(&Real::from(0), &Real::from(0)),
            Some(PredicateOutcome::decided(
                Sign::Zero,
                Certainty::Filtered,
                Escalation::Filter
            ))
        );
        assert_eq!(certified_ball_sign(&Real::from(1), &Real::from(2)), None);
    }
}
