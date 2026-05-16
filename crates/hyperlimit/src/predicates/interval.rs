//! Exact closed-interval classifiers.
//!
//! The functions here classify Real ranges by exact sign decisions over
//! endpoint differences. They are intentionally Real predicates, not bounding
//! box or sweep-line data structures; higher crates own those objects and call
//! this module for certified interval decisions.

use core::cmp::Ordering;

use crate::classify::{ClosedIntervalIntersection, RealIntervalLocation};
use crate::predicate::{Certainty, Escalation, PredicateOutcome, PredicatePolicy, RefinementNeed};
use crate::predicates::order::compare_reals_with_policy;
use hyperreal::Real;

/// Classify `value` relative to the closed interval with endpoints `first` and
/// `second`.
pub fn classify_real_closed_interval(
    value: &Real,
    first: &Real,
    second: &Real,
) -> PredicateOutcome<RealIntervalLocation> {
    classify_real_closed_interval_with_policy(value, first, second, PredicatePolicy::default())
}

/// Classify `value` relative to a closed interval with an explicit predicate
/// escalation policy.
///
/// Endpoints may be supplied in either order; this function first normalizes
/// them by exact Real comparison. The result is useful for segment parameter
/// checks, bounding-box point tests, and candidate filtering in arrangement or
/// triangulation code. The interval itself is not a geometric object here. As
/// in Bentley and Ottmann's intersection-reporting setting, interval tests are
/// candidate filters; final topology must still be certified by geometric
/// predicates. See Bentley and Ottmann, "Algorithms for Reporting and Counting
/// Geometric Intersections," *IEEE Transactions on Computers* C-28.9 (1979).
/// The numerical boundary follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): every comparison is a certified sign,
/// not a primitive-float tolerance.
pub fn classify_real_closed_interval_with_policy(
    value: &Real,
    first: &Real,
    second: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<RealIntervalLocation> {
    let mut trace = DecisionTrace::default();
    let (lower, upper) = match ordered_pair(first, second, policy, &mut trace) {
        Ok(pair) => pair,
        Err(unknown) => return unknown.into_outcome(),
    };

    let lower_cmp = match decided(compare_reals_with_policy(value, lower, policy), &mut trace) {
        Ok(ordering) => ordering,
        Err(unknown) => return unknown.into_outcome(),
    };
    if lower_cmp == Ordering::Less {
        return PredicateOutcome::decided(
            RealIntervalLocation::Below,
            trace.certainty,
            trace.stage,
        );
    }
    if lower_cmp == Ordering::Equal {
        return PredicateOutcome::decided(
            RealIntervalLocation::AtLowerEndpoint,
            trace.certainty,
            trace.stage,
        );
    }

    let upper_cmp = match decided(compare_reals_with_policy(value, upper, policy), &mut trace) {
        Ok(ordering) => ordering,
        Err(unknown) => return unknown.into_outcome(),
    };
    let location = match upper_cmp {
        Ordering::Less => RealIntervalLocation::Interior,
        Ordering::Equal => RealIntervalLocation::AtUpperEndpoint,
        Ordering::Greater => RealIntervalLocation::Above,
    };

    PredicateOutcome::decided(location, trace.certainty, trace.stage)
}

/// Return whether `value` lies in the closed interval with endpoints `first` and
/// `second`.
pub fn real_in_closed_interval(
    value: &Real,
    first: &Real,
    second: &Real,
) -> PredicateOutcome<bool> {
    real_in_closed_interval_with_policy(value, first, second, PredicatePolicy::default())
}

/// Return whether `value` lies in a closed interval with an explicit predicate
/// escalation policy.
pub fn real_in_closed_interval_with_policy(
    value: &Real,
    first: &Real,
    second: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    match classify_real_closed_interval_with_policy(value, first, second, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.is_inside_or_boundary(), certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Classify the intersection relation between two closed Real intervals.
pub fn classify_closed_interval_intersection(
    first_start: &Real,
    first_end: &Real,
    second_start: &Real,
    second_end: &Real,
) -> PredicateOutcome<ClosedIntervalIntersection> {
    classify_closed_interval_intersection_with_policy(
        first_start,
        first_end,
        second_start,
        second_end,
        PredicatePolicy::default(),
    )
}

/// Classify two closed Real intervals with an explicit predicate escalation
/// policy.
///
/// Endpoint order does not matter for either interval. `Touching` means the
/// intervals share exactly one endpoint value, which is a useful distinction for
/// curve splitting, sweep events, and conservative broad-phase pruning.
pub fn classify_closed_interval_intersection_with_policy(
    first_start: &Real,
    first_end: &Real,
    second_start: &Real,
    second_end: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<ClosedIntervalIntersection> {
    let mut trace = DecisionTrace::default();
    let (first_lower, first_upper) = match ordered_pair(first_start, first_end, policy, &mut trace)
    {
        Ok(pair) => pair,
        Err(unknown) => return unknown.into_outcome(),
    };
    let (second_lower, second_upper) =
        match ordered_pair(second_start, second_end, policy, &mut trace) {
            Ok(pair) => pair,
            Err(unknown) => return unknown.into_outcome(),
        };

    match decided(
        compare_reals_with_policy(first_upper, second_lower, policy),
        &mut trace,
    ) {
        Ok(Ordering::Less) => {
            return PredicateOutcome::decided(
                ClosedIntervalIntersection::Disjoint,
                trace.certainty,
                trace.stage,
            );
        }
        Ok(Ordering::Equal) => {
            return PredicateOutcome::decided(
                ClosedIntervalIntersection::Touching,
                trace.certainty,
                trace.stage,
            );
        }
        Ok(Ordering::Greater) => {}
        Err(unknown) => return unknown.into_outcome(),
    }

    match decided(
        compare_reals_with_policy(second_upper, first_lower, policy),
        &mut trace,
    ) {
        Ok(Ordering::Less) => PredicateOutcome::decided(
            ClosedIntervalIntersection::Disjoint,
            trace.certainty,
            trace.stage,
        ),
        Ok(Ordering::Equal) => PredicateOutcome::decided(
            ClosedIntervalIntersection::Touching,
            trace.certainty,
            trace.stage,
        ),
        Ok(Ordering::Greater) => PredicateOutcome::decided(
            ClosedIntervalIntersection::Overlapping,
            trace.certainty,
            trace.stage,
        ),
        Err(unknown) => unknown.into_outcome(),
    }
}

/// Return whether two closed Real intervals intersect.
pub fn closed_intervals_intersect(
    first_start: &Real,
    first_end: &Real,
    second_start: &Real,
    second_end: &Real,
) -> PredicateOutcome<bool> {
    closed_intervals_intersect_with_policy(
        first_start,
        first_end,
        second_start,
        second_end,
        PredicatePolicy::default(),
    )
}

/// Return whether two closed Real intervals intersect with an explicit
/// predicate escalation policy.
pub fn closed_intervals_intersect_with_policy(
    first_start: &Real,
    first_end: &Real,
    second_start: &Real,
    second_end: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    match classify_closed_interval_intersection_with_policy(
        first_start,
        first_end,
        second_start,
        second_end,
        policy,
    ) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.intersects(), certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn ordered_pair<'a>(
    first: &'a Real,
    second: &'a Real,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<(&'a Real, &'a Real), UnknownDecision> {
    match decided(compare_reals_with_policy(first, second, policy), trace)? {
        Ordering::Greater => Ok((second, first)),
        Ordering::Less | Ordering::Equal => Ok((first, second)),
    }
}

#[derive(Clone, Copy)]
struct DecisionTrace {
    certainty: Certainty,
    stage: Escalation,
}

impl Default for DecisionTrace {
    fn default() -> Self {
        Self {
            certainty: Certainty::Exact,
            stage: Escalation::Structural,
        }
    }
}

#[derive(Clone, Copy)]
struct UnknownDecision {
    needed: RefinementNeed,
    stage: Escalation,
}

impl UnknownDecision {
    fn into_outcome<T>(self) -> PredicateOutcome<T> {
        PredicateOutcome::unknown(self.needed, self.stage)
    }
}

fn decided<T>(
    outcome: PredicateOutcome<T>,
    trace: &mut DecisionTrace,
) -> Result<T, UnknownDecision> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => {
            trace.certainty = max_certainty(trace.certainty, certainty);
            trace.stage = max_stage(trace.stage, stage);
            Ok(value)
        }
        PredicateOutcome::Unknown { needed, stage } => Err(UnknownDecision { needed, stage }),
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
    fn real_interval_classifies_ordered_and_reversed_endpoints() {
        assert_eq!(
            classify_real_closed_interval(&real(1), &real(1), &real(4)).value(),
            Some(RealIntervalLocation::AtLowerEndpoint)
        );
        assert_eq!(
            classify_real_closed_interval(&real(3), &real(4), &real(1)).value(),
            Some(RealIntervalLocation::Interior)
        );
        assert_eq!(
            classify_real_closed_interval(&real(5), &real(1), &real(4)).value(),
            Some(RealIntervalLocation::Above)
        );
        assert_eq!(
            real_in_closed_interval(&real(4), &real(1), &real(4)).value(),
            Some(true)
        );
    }

    #[test]
    fn closed_interval_intersection_distinguishes_disjoint_touching_and_overlap() {
        assert_eq!(
            classify_closed_interval_intersection(&real(0), &real(1), &real(2), &real(3)).value(),
            Some(ClosedIntervalIntersection::Disjoint)
        );
        assert_eq!(
            classify_closed_interval_intersection(&real(0), &real(2), &real(2), &real(3)).value(),
            Some(ClosedIntervalIntersection::Touching)
        );
        assert_eq!(
            classify_closed_interval_intersection(&real(0), &real(3), &real(2), &real(4)).value(),
            Some(ClosedIntervalIntersection::Overlapping)
        );
        assert_eq!(
            closed_intervals_intersect(&real(4), &real(2), &real(1), &real(2)).value(),
            Some(true)
        );
    }
}
