//! Shared sign-resolution helpers for predicate pipelines.

use crate::predicate::{
    Certainty, Escalation, PredicateOutcome, PredicatePolicy, RefinementNeed, Sign, SignKnowledge,
};
use crate::scalar::{MagnitudeBounds, PredicateScalar};

/// Resolve a scalar sign through the common predicate pipeline.
///
/// `exact` is the predicate-level exact evaluation hook. It should do actual
/// exact determinant/sign work for the whole predicate, while scalar facts only
/// certify signs that are already exposed by the computed scalar value.
pub(crate) fn resolve_scalar_sign<S: PredicateScalar>(
    value: &S,
    policy: PredicatePolicy,
    filter: impl FnOnce() -> Option<PredicateOutcome<Sign>>,
    exact: impl FnOnce() -> Option<Sign>,
    fallback: impl FnOnce() -> Option<PredicateOutcome<Sign>>,
    unknown_need: RefinementNeed,
) -> PredicateOutcome<Sign> {
    // The ordering is the performance policy for every predicate: use facts
    // already attached to the scalar, then determinant-specific cheap filters,
    // then exact/refinement/fallback stages. Reordering this can easily move
    // exact symbolic backends from nanosecond fact checks to expression builds.
    if let Some(outcome) = decide_scalar_sign(value, Escalation::Structural) {
        crate::trace_dispatch!(
            "predicated",
            "resolve_scalar_sign",
            "structural-scalar-facts"
        );
        return outcome;
    }

    if let Some(outcome) = filter() {
        crate::trace_dispatch!("predicated", "resolve_scalar_sign", "predicate-filter");
        return outcome;
    }

    if let Some(outcome) = exact_scalar_sign_if_allowed(value, policy) {
        crate::trace_dispatch!("predicated", "resolve_scalar_sign", "exact-scalar-facts");
        return outcome;
    }

    if let Some(outcome) = exact_evaluation_if_allowed(policy, exact) {
        crate::trace_dispatch!("predicated", "resolve_scalar_sign", "exact-predicate");
        return outcome;
    }

    if let Some(outcome) = refine_scalar_sign_if_allowed(value, policy) {
        crate::trace_dispatch!("predicated", "resolve_scalar_sign", "scalar-refinement");
        return outcome;
    }

    if let Some(outcome) = fallback() {
        crate::trace_dispatch!("predicated", "resolve_scalar_sign", "robust-fallback");
        return outcome;
    }

    if let Some(outcome) = approximate_if_allowed(value, policy) {
        crate::trace_dispatch!("predicated", "resolve_scalar_sign", "approximate");
        return outcome;
    }

    crate::trace_dispatch!("predicated", "resolve_scalar_sign", "unknown");
    PredicateOutcome::unknown(unknown_need, Escalation::Undecided)
}

pub(crate) fn decide_scalar_sign<S: PredicateScalar>(
    value: &S,
    stage: Escalation,
) -> Option<PredicateOutcome<Sign>> {
    match value.known_sign() {
        SignKnowledge::Known { sign, certainty } => {
            crate::trace_dispatch!("predicated", "decide_scalar_sign", "known-sign");
            Some(PredicateOutcome::decided(sign, certainty, stage))
        }
        SignKnowledge::NonZero => {
            crate::trace_dispatch!("predicated", "decide_scalar_sign", "nonzero-no-sign");
            None
        }
        SignKnowledge::Unknown => {
            crate::trace_dispatch!("predicated", "decide_scalar_sign", "unknown");
            None
        }
    }
}

pub(crate) fn approximate_if_allowed<S: PredicateScalar>(
    value: &S,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    if !policy.allow_approximate {
        crate::trace_dispatch!("predicated", "approximate_if_allowed", "disabled");
        return None;
    }

    let Some(value) = value.to_f64() else {
        crate::trace_dispatch!("predicated", "approximate_if_allowed", "no-f64");
        return None;
    };
    let Some(sign) = Sign::from_f64(value) else {
        crate::trace_dispatch!("predicated", "approximate_if_allowed", "zero-or-nan");
        return None;
    };
    crate::trace_dispatch!("predicated", "approximate_if_allowed", "decided");
    Some(PredicateOutcome::decided(
        sign,
        Certainty::Approximate,
        Escalation::Undecided,
    ))
}

pub(crate) fn map_outcome<T, U>(
    outcome: PredicateOutcome<T>,
    map: impl FnOnce(T) -> U,
) -> PredicateOutcome<U> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(map(value), certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn exact_scalar_sign_if_allowed<S: PredicateScalar>(
    value: &S,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    if !policy.allow_exact {
        crate::trace_dispatch!("predicated", "exact_scalar_sign", "disabled");
        return None;
    }

    let facts = value.scalar_facts();
    if facts.exact != Some(true) && facts.rational_only != Some(true) {
        crate::trace_dispatch!("predicated", "exact_scalar_sign", "not-exact-scalar");
        return None;
    }

    match facts.sign_knowledge() {
        SignKnowledge::Known { sign, .. } => {
            crate::trace_dispatch!("predicated", "exact_scalar_sign", "decided");
            Some(PredicateOutcome::decided(
                sign,
                Certainty::Exact,
                Escalation::Exact,
            ))
        }
        SignKnowledge::NonZero => {
            crate::trace_dispatch!("predicated", "exact_scalar_sign", "nonzero-no-sign");
            None
        }
        SignKnowledge::Unknown => {
            crate::trace_dispatch!("predicated", "exact_scalar_sign", "unknown");
            None
        }
    }
}

fn exact_evaluation_if_allowed(
    policy: PredicatePolicy,
    exact: impl FnOnce() -> Option<Sign>,
) -> Option<PredicateOutcome<Sign>> {
    if !policy.allow_exact {
        crate::trace_dispatch!("predicated", "exact_evaluation", "disabled");
        return None;
    }

    match exact() {
        Some(sign) => {
            crate::trace_dispatch!("predicated", "exact_evaluation", "decided");
            Some(PredicateOutcome::decided(
                sign,
                Certainty::Exact,
                Escalation::Exact,
            ))
        }
        None => {
            crate::trace_dispatch!("predicated", "exact_evaluation", "unavailable");
            None
        }
    }
}

fn refine_scalar_sign_if_allowed<S: PredicateScalar>(
    value: &S,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    if !policy.allow_refinement {
        crate::trace_dispatch!("predicated", "refine_scalar_sign", "disabled");
        return None;
    }

    let Some(precision) = policy.max_refinement_precision else {
        crate::trace_dispatch!("predicated", "refine_scalar_sign", "no-precision-budget");
        return None;
    };
    match value.refine_sign_until(precision) {
        SignKnowledge::Known { sign, certainty } => {
            crate::trace_dispatch!("predicated", "refine_scalar_sign", "decided");
            Some(PredicateOutcome::decided(
                sign,
                certainty,
                Escalation::Refined,
            ))
        }
        SignKnowledge::NonZero => {
            crate::trace_dispatch!("predicated", "refine_scalar_sign", "nonzero-no-sign");
            None
        }
        SignKnowledge::Unknown => {
            crate::trace_dispatch!("predicated", "refine_scalar_sign", "unknown");
            None
        }
    }
}

/// Try to decide the sign of a sum of signed terms using structural signs and
/// magnitude bounds. Each input term is `(term, sign_multiplier)`.
pub(crate) fn signed_term_filter<S: PredicateScalar>(
    terms: &[(&S, Sign)],
) -> Option<PredicateOutcome<Sign>> {
    // This filter is a performance shortcut ahead of exact predicate fallback.
    // It intentionally uses only cheap scalar facts: exact zero, structural
    // sign, and conservative magnitude. If any needed fact is missing, we stop
    // and let the normal predicate pipeline refine or fall back.
    let mut nonzero = Vec::new();

    for (term, multiplier) in terms {
        let facts = term.scalar_facts();
        if facts.exact_zero == Some(true) || facts.sign == Some(Sign::Zero) {
            crate::trace_dispatch!("predicated", "signed_term_filter", "zero-term");
            continue;
        }

        let Some(sign) = facts.sign else {
            crate::trace_dispatch!("predicated", "signed_term_filter", "missing-sign");
            return None;
        };
        let sign = multiply_sign(sign, *multiplier);
        if sign == Sign::Zero {
            crate::trace_dispatch!("predicated", "signed_term_filter", "zero-after-multiplier");
            continue;
        }
        let Some(magnitude) = facts.magnitude else {
            crate::trace_dispatch!("predicated", "signed_term_filter", "missing-magnitude");
            return None;
        };
        nonzero.push((sign, magnitude));
    }

    if nonzero.is_empty() {
        crate::trace_dispatch!("predicated", "signed_term_filter", "all-zero");
        return Some(PredicateOutcome::decided(
            Sign::Zero,
            Certainty::Filtered,
            Escalation::Filter,
        ));
    }

    let first = nonzero[0].0;
    if nonzero.iter().all(|(sign, _)| *sign == first) {
        crate::trace_dispatch!("predicated", "signed_term_filter", "same-sign");
        return Some(PredicateOutcome::decided(
            first,
            Certainty::Filtered,
            Escalation::Filter,
        ));
    }

    match dominance_sign(&nonzero) {
        Some(sign) => {
            crate::trace_dispatch!("predicated", "signed_term_filter", "dominant-term");
            Some(PredicateOutcome::decided(
                sign,
                Certainty::Filtered,
                Escalation::Filter,
            ))
        }
        None => {
            crate::trace_dispatch!("predicated", "signed_term_filter", "mixed-no-dominance");
            None
        }
    }
}

fn dominance_sign(terms: &[(Sign, MagnitudeBounds)]) -> Option<Sign> {
    // Dominant-term detection catches expressions like `pi - 3` without
    // constructing exact fallback objects. The two-bit gap is conservative:
    // it leaves ambiguous near-cancellation to the slower but safer path.
    for (index, (sign, magnitude)) in terms.iter().enumerate() {
        if magnitude.abs_lower <= 0.0 {
            crate::trace_dispatch!("predicated", "dominance_sign", "nonpositive-lower-bound");
            continue;
        }

        let mut others_upper = 0.0;
        for (other_index, (_, other_magnitude)) in terms.iter().enumerate() {
            if other_index == index {
                continue;
            }
            others_upper += other_magnitude.abs_upper;
        }

        if magnitude.abs_lower > others_upper {
            crate::trace_dispatch!("predicated", "dominance_sign", "decided");
            return Some(*sign);
        }
    }

    crate::trace_dispatch!("predicated", "dominance_sign", "none");
    None
}

fn multiply_sign(left: Sign, right: Sign) -> Sign {
    match (left, right) {
        (Sign::Zero, _) | (_, Sign::Zero) => Sign::Zero,
        (Sign::Positive, Sign::Positive) | (Sign::Negative, Sign::Negative) => Sign::Positive,
        (Sign::Positive, Sign::Negative) | (Sign::Negative, Sign::Positive) => Sign::Negative,
    }
}

#[cfg(test)]
mod tests {
    use core::cell::Cell;
    use core::ops::{Add, Mul, Sub};

    use super::*;
    use crate::scalar::{ScalarFacts, StructuralScalar};

    #[derive(Clone, Debug, PartialEq)]
    struct FactScalar {
        facts: ScalarFacts,
        value: f64,
    }

    impl FactScalar {
        fn new(sign: Sign, abs_lower: f64, abs_upper: f64) -> Self {
            Self {
                facts: ScalarFacts {
                    sign: Some(sign),
                    exact_zero: Some(sign == Sign::Zero),
                    provably_nonzero: Some(sign != Sign::Zero),
                    exact: Some(false),
                    rational_only: Some(false),
                    magnitude: Some(MagnitudeBounds {
                        abs_lower,
                        abs_upper,
                    }),
                },
                value: match sign {
                    Sign::Negative => -abs_lower,
                    Sign::Zero => 0.0,
                    Sign::Positive => abs_lower,
                },
            }
        }

        fn exact_without_known_sign(value: f64) -> Self {
            Self {
                facts: ScalarFacts {
                    sign: None,
                    exact_zero: Some(false),
                    provably_nonzero: None,
                    exact: Some(true),
                    rational_only: Some(true),
                    magnitude: Some(MagnitudeBounds::exact(value.abs())),
                },
                value,
            }
        }
    }

    impl StructuralScalar for FactScalar {
        fn scalar_facts(&self) -> ScalarFacts {
            self.facts
        }
    }

    impl crate::scalar::PredicateScalar for FactScalar {
        fn to_f64(&self) -> Option<f64> {
            Some(self.value)
        }
    }

    impl Add for FactScalar {
        type Output = Self;

        fn add(self, rhs: Self) -> Self::Output {
            Self {
                facts: ScalarFacts::default(),
                value: self.value + rhs.value,
            }
        }
    }

    impl Sub for FactScalar {
        type Output = Self;

        fn sub(self, rhs: Self) -> Self::Output {
            Self {
                facts: ScalarFacts::default(),
                value: self.value - rhs.value,
            }
        }
    }

    impl Mul for FactScalar {
        type Output = Self;

        fn mul(self, rhs: Self) -> Self::Output {
            Self {
                facts: ScalarFacts::default(),
                value: self.value * rhs.value,
            }
        }
    }

    #[test]
    fn signed_term_filter_uses_magnitude_dominance() {
        let large = FactScalar::new(Sign::Positive, 10.0, 12.0);
        let small = FactScalar::new(Sign::Positive, 1.0, 2.0);

        assert_eq!(
            signed_term_filter(&[(&large, Sign::Positive), (&small, Sign::Negative)]),
            Some(PredicateOutcome::decided(
                Sign::Positive,
                Certainty::Filtered,
                Escalation::Filter
            ))
        );
    }

    #[test]
    fn resolve_scalar_sign_uses_exact_evaluation_callback() {
        let value = FactScalar::exact_without_known_sign(3.0);

        assert_eq!(
            resolve_scalar_sign(
                &value,
                PredicatePolicy::STRICT,
                || None,
                || Some(Sign::Positive),
                || None,
                RefinementNeed::ExactArithmetic,
            ),
            PredicateOutcome::decided(Sign::Positive, Certainty::Exact, Escalation::Exact)
        );
    }

    #[test]
    fn resolve_scalar_sign_does_not_call_exact_evaluation_when_policy_disallows_exact() {
        let value = FactScalar::exact_without_known_sign(3.0);
        let called = Cell::new(false);
        let policy = PredicatePolicy {
            allow_exact: false,
            allow_refinement: false,
            allow_robust_fallback: false,
            allow_approximate: false,
            ..PredicatePolicy::STRICT
        };

        assert_eq!(
            resolve_scalar_sign(
                &value,
                policy,
                || None,
                || {
                    called.set(true);
                    Some(Sign::Positive)
                },
                || None,
                RefinementNeed::ExactArithmetic,
            ),
            PredicateOutcome::unknown(RefinementNeed::ExactArithmetic, Escalation::Undecided)
        );
        assert!(!called.get());
    }
}
