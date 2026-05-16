//! Certified predicate filters.
//!
//! Filters in this module are exact, policy-visible shortcuts. They may decide
//! a predicate before expensive refinement, or return explicit uncertainty.
//! They are not primitive-float tolerances. This follows Yap's exact geometric
//! computation pipeline: approximate or interval information is useful only
//! when it produces a certificate or a bounded non-decision; see Yap, "Towards
//! Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997).

use core::cmp::Ordering;

use hyperreal::Real;

use crate::predicate::{
    Certainty, Escalation, PredicateCertificate, PredicateOutcome, PredicatePolicy,
    PredicateReport, RefinementNeed, Sign,
};
use crate::predicates::order::compare_reals_with_policy;

/// Try to certify a sign from an exact closed interval enclosure.
///
/// The interval endpoints may be supplied in either order. Internally this
/// function disables Real refinement and uses only structural/exact comparison
/// routes allowed by `policy`; that makes it suitable as a pre-refinement
/// filter. A returned decision is a proof that every value in the interval has
/// the reported sign. If the interval crosses zero, or if endpoint comparison
/// itself cannot be certified without refinement, the report is explicitly
/// unknown.
///
/// This is the predicate-layer companion to interval arithmetic enclosures:
/// callers own how the interval was produced, while `hyperlimit` owns the exact
/// sign certificate. That boundary follows Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).
pub fn certified_interval_sign_report(first: &Real, second: &Real) -> PredicateReport<Sign> {
    certified_interval_sign_report_with_policy(first, second, PredicatePolicy::default())
}

/// Try to certify a sign from an exact closed interval enclosure with policy.
pub fn certified_interval_sign_report_with_policy(
    first: &Real,
    second: &Real,
    policy: PredicatePolicy,
) -> PredicateReport<Sign> {
    match certified_interval_sign_with_policy(first, second, policy) {
        Some(outcome) => {
            PredicateReport::new(outcome, PredicateCertificate::CertifiedIntervalFilter)
        }
        None => PredicateReport::new(
            PredicateOutcome::unknown(RefinementNeed::RealRefinement, Escalation::Filter),
            PredicateCertificate::Unknown,
        ),
    }
}

/// Try to certify a sign from an exact closed interval enclosure.
///
/// Returns `Some` only when the interval proves a sign. This shape is intended
/// for predicate filter callbacks such as `resolve_real_sign(..., || {
/// certified_interval_sign_with_policy(...) }, ...)`.
pub fn certified_interval_sign(first: &Real, second: &Real) -> Option<PredicateOutcome<Sign>> {
    certified_interval_sign_with_policy(first, second, PredicatePolicy::default())
}

/// Try to certify a sign from an exact closed interval enclosure with policy.
pub fn certified_interval_sign_with_policy(
    first: &Real,
    second: &Real,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    crate::trace_dispatch!("hyperlimit", "certified_interval_sign", "start");
    let policy = PredicatePolicy {
        allow_refinement: false,
        ..policy
    };
    let zero = Real::from(0);

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
    use hyperreal::Rational;

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
    fn certified_interval_report_carries_interval_filter_certificate() {
        let report = certified_interval_sign_report(&Real::from(2), &Real::from(5));
        assert_eq!(report.value(), Some(Sign::Positive));
        assert_eq!(
            report.certificate,
            PredicateCertificate::CertifiedIntervalFilter
        );

        let unknown = certified_interval_sign_report(&Real::from(-2), &Real::from(5));
        assert_eq!(unknown.value(), None);
        assert_eq!(unknown.certificate, PredicateCertificate::Unknown);
    }

    #[test]
    fn certified_interval_filter_does_not_force_refinement() {
        let pi_minus_approx = Real::pi() - Real::new(Rational::fraction(103_993, 33_102).unwrap());
        let positive = Real::from(1);
        let policy = PredicatePolicy {
            allow_exact: false,
            allow_refinement: true,
            ..PredicatePolicy::STRICT
        };

        assert_eq!(
            certified_interval_sign_with_policy(&pi_minus_approx, &positive, policy),
            None
        );
    }
}
