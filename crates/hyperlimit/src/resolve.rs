//! Shared sign-resolution helpers for predicate pipelines.

use crate::predicate::{
    Certainty, Escalation, PredicateOutcome, PredicatePolicy, RefinementNeed, Sign, SignKnowledge,
};
use crate::real::{RealPredicateExt, sign_knowledge_from_real_facts};
use hyperreal::{Real, ZeroKnowledge};

/// Resolve a Real sign through the common predicate pipeline.
///
/// `exact` is the predicate-level exact evaluation hook. It should do actual
/// exact determinant/sign work for the whole predicate, while Real facts only
/// certify signs that are already exposed by the computed Real value.
pub(crate) fn resolve_real_sign(
    value: &Real,
    policy: PredicatePolicy,
    filter: impl FnOnce() -> Option<PredicateOutcome<Sign>>,
    exact: impl FnOnce() -> Option<Sign>,
    unknown_need: RefinementNeed,
) -> PredicateOutcome<Sign> {
    // The ordering is the performance policy for every predicate: use facts
    // already attached to the Real, then determinant-specific cheap filters,
    // then exact/refinement stages. Reordering this can easily move
    // exact symbolic Real values from nanosecond fact checks to expression builds.
    if let Some(outcome) = decide_real_sign(value, Escalation::Structural) {
        crate::trace_dispatch!("hyperlimit", "resolve_real_sign", "structural-real-facts");
        return outcome;
    }

    // Structural-dispatch note: richer Real metadata should be preserved up
    // to this boundary. Exact-rational kind, dyadic denominator class, sparse
    // zero masks, symbolic-class tags, and coordinate-grid facts can select
    // faster exact determinant expansions before the predicate allocates a full
    // symbolic expression. Those future dispatches must remain exact; do not
    // reintroduce primitive-float dominance predicates here.
    if let Some(outcome) = filter() {
        crate::trace_dispatch!("hyperlimit", "resolve_real_sign", "predicate-filter");
        return outcome;
    }

    if let Some(outcome) = exact_real_sign_if_allowed(value, policy) {
        crate::trace_dispatch!("hyperlimit", "resolve_real_sign", "exact-real-facts");
        return outcome;
    }

    if let Some(outcome) = exact_evaluation_if_allowed(policy, exact) {
        crate::trace_dispatch!("hyperlimit", "resolve_real_sign", "exact-predicate");
        return outcome;
    }

    if let Some(outcome) = refine_real_sign_if_allowed(value, policy) {
        crate::trace_dispatch!("hyperlimit", "resolve_real_sign", "real-refinement");
        return outcome;
    }

    crate::trace_dispatch!("hyperlimit", "resolve_real_sign", "unknown");
    PredicateOutcome::unknown(unknown_need, Escalation::Undecided)
}

pub(crate) fn decide_real_sign(value: &Real, stage: Escalation) -> Option<PredicateOutcome<Sign>> {
    match value.known_sign() {
        SignKnowledge::Known { sign, certainty } => {
            crate::trace_dispatch!("hyperlimit", "decide_real_sign", "known-sign");
            Some(PredicateOutcome::decided(sign, certainty, stage))
        }
        SignKnowledge::NonZero => {
            crate::trace_dispatch!("hyperlimit", "decide_real_sign", "nonzero-no-sign");
            None
        }
        SignKnowledge::Unknown => {
            crate::trace_dispatch!("hyperlimit", "decide_real_sign", "unknown");
            None
        }
    }
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

fn exact_real_sign_if_allowed(
    value: &Real,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    if !policy.allow_exact {
        crate::trace_dispatch!("hyperlimit", "exact_real_sign", "disabled");
        return None;
    }

    let facts = value.real_facts();
    if !facts.exact_rational {
        crate::trace_dispatch!("hyperlimit", "exact_real_sign", "not-exact-real");
        return None;
    }

    match sign_knowledge_from_real_facts(facts) {
        SignKnowledge::Known { sign, .. } => {
            crate::trace_dispatch!("hyperlimit", "exact_real_sign", "decided");
            Some(PredicateOutcome::decided(
                sign,
                Certainty::Exact,
                Escalation::Exact,
            ))
        }
        SignKnowledge::NonZero => {
            crate::trace_dispatch!("hyperlimit", "exact_real_sign", "nonzero-no-sign");
            None
        }
        SignKnowledge::Unknown => {
            crate::trace_dispatch!("hyperlimit", "exact_real_sign", "unknown");
            None
        }
    }
}

fn exact_evaluation_if_allowed(
    policy: PredicatePolicy,
    exact: impl FnOnce() -> Option<Sign>,
) -> Option<PredicateOutcome<Sign>> {
    if !policy.allow_exact {
        crate::trace_dispatch!("hyperlimit", "exact_evaluation", "disabled");
        return None;
    }

    match exact() {
        Some(sign) => {
            crate::trace_dispatch!("hyperlimit", "exact_evaluation", "decided");
            Some(PredicateOutcome::decided(
                sign,
                Certainty::Exact,
                Escalation::Exact,
            ))
        }
        None => {
            crate::trace_dispatch!("hyperlimit", "exact_evaluation", "unavailable");
            None
        }
    }
}

fn refine_real_sign_if_allowed(
    value: &Real,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    if !policy.allow_refinement {
        crate::trace_dispatch!("hyperlimit", "refine_real_sign", "disabled");
        return None;
    }

    let Some(precision) = policy.max_refinement_precision else {
        crate::trace_dispatch!("hyperlimit", "refine_real_sign", "no-precision-budget");
        return None;
    };
    match value.refine_sign_knowledge_until(precision) {
        SignKnowledge::Known { sign, certainty } => {
            crate::trace_dispatch!("hyperlimit", "refine_real_sign", "decided");
            Some(PredicateOutcome::decided(
                sign,
                certainty,
                Escalation::Refined,
            ))
        }
        SignKnowledge::NonZero => {
            crate::trace_dispatch!("hyperlimit", "refine_real_sign", "nonzero-no-sign");
            None
        }
        SignKnowledge::Unknown => {
            crate::trace_dispatch!("hyperlimit", "refine_real_sign", "unknown");
            None
        }
    }
}

/// Try to decide the sign of a sum of signed terms using structural zero/sign
/// facts only. Each input term is `(term, sign_multiplier)`.
#[inline(always)]
pub(crate) fn signed_term_filter(terms: &[(&Real, Sign)]) -> Option<PredicateOutcome<Sign>> {
    // This filter is a performance shortcut ahead of exact predicate fallback.
    // It intentionally uses only exact structural zero/sign facts. Primitive
    // float magnitude dominance used to live here; it was removed so
    // `hyperlimit` predicates operate entirely through hyperreal-backed exact
    // signs, exact arithmetic, and bounded refinement.
    if terms.len() > 4 {
        return signed_term_filter_dynamic(terms);
    }

    let mut nonzero = [Sign::Zero; 4];
    let mut nonzero_len = 0usize;
    for (term, multiplier) in terms {
        let Some(sign) = signed_nonzero_term(*term, *multiplier)? else {
            continue;
        };
        nonzero[nonzero_len] = sign;
        nonzero_len += 1;
    }

    finish_signed_term_filter(&nonzero[..nonzero_len])
}

#[inline]
fn signed_term_filter_dynamic(terms: &[(&Real, Sign)]) -> Option<PredicateOutcome<Sign>> {
    let mut nonzero = Vec::with_capacity(terms.len());

    for (term, multiplier) in terms {
        let Some(sign) = signed_nonzero_term(*term, *multiplier)? else {
            continue;
        };
        nonzero.push(sign);
    }

    finish_signed_term_filter(&nonzero)
}

#[inline(always)]
fn signed_nonzero_term(term: &Real, multiplier: Sign) -> Option<Option<Sign>> {
    let facts = term.real_facts();
    if matches!(facts.zero, ZeroKnowledge::Zero) {
        crate::trace_dispatch!("hyperlimit", "signed_term_filter", "zero-term");
        return Some(None);
    }

    let Some(sign) = facts.sign.map(crate::real::map_real_sign) else {
        crate::trace_dispatch!("hyperlimit", "signed_term_filter", "missing-sign");
        return None;
    };
    let sign = multiply_sign(sign, multiplier);
    if sign == Sign::Zero {
        crate::trace_dispatch!("hyperlimit", "signed_term_filter", "zero-after-multiplier");
        return Some(None);
    }
    Some(Some(sign))
}

#[inline(always)]
fn finish_signed_term_filter(nonzero: &[Sign]) -> Option<PredicateOutcome<Sign>> {
    if nonzero.is_empty() {
        crate::trace_dispatch!("hyperlimit", "signed_term_filter", "all-zero");
        return Some(PredicateOutcome::decided(
            Sign::Zero,
            Certainty::Filtered,
            Escalation::Filter,
        ));
    }
    let first = nonzero[0];
    if nonzero.iter().all(|sign| *sign == first) {
        crate::trace_dispatch!("hyperlimit", "signed_term_filter", "same-sign");
        return Some(PredicateOutcome::decided(
            first,
            Certainty::Filtered,
            Escalation::Filter,
        ));
    }

    crate::trace_dispatch!("hyperlimit", "signed_term_filter", "mixed-signs");
    None
}

#[inline(always)]
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

    use super::*;
    use hyperreal::Rational;

    #[test]
    fn signed_term_filter_decides_same_sign_terms_without_magnitude() {
        let large = Real::from(10);
        let small = Real::from(1);

        assert_eq!(
            signed_term_filter(&[(&large, Sign::Positive), (&small, Sign::Positive)]),
            Some(PredicateOutcome::decided(
                Sign::Positive,
                Certainty::Filtered,
                Escalation::Filter
            ))
        );
    }

    #[test]
    fn signed_term_filter_leaves_mixed_signs_to_exact_pipeline() {
        let large = Real::from(10);
        let small = Real::from(1);

        assert_eq!(
            signed_term_filter(&[(&large, Sign::Positive), (&small, Sign::Negative)]),
            None
        );
    }

    #[test]
    fn resolve_real_sign_uses_exact_evaluation_callback() {
        // Use a deliberately tight rational approximation to pi so cheap
        // structural Real facts cannot decide the sign before the predicate-level
        // exact callback is reached.
        let value = Real::pi() - Real::new(Rational::fraction(103_993, 33_102).unwrap());

        assert_eq!(
            resolve_real_sign(
                &value,
                PredicatePolicy::STRICT,
                || None,
                || Some(Sign::Positive),
                RefinementNeed::ExactArithmetic,
            ),
            PredicateOutcome::decided(Sign::Positive, Certainty::Exact, Escalation::Exact)
        );
    }

    #[test]
    fn resolve_real_sign_does_not_call_exact_evaluation_when_policy_disallows_exact() {
        // The value is not structurally sign-known, which keeps this test
        // focused on the exact-callback policy gate instead of the earlier
        // structural short-circuit.
        let value = Real::pi() - Real::new(Rational::fraction(103_993, 33_102).unwrap());
        let called = Cell::new(false);
        let policy = PredicatePolicy {
            allow_exact: false,
            allow_refinement: false,
            ..PredicatePolicy::STRICT
        };

        assert_eq!(
            resolve_real_sign(
                &value,
                policy,
                || None,
                || {
                    called.set(true);
                    Some(Sign::Positive)
                },
                RefinementNeed::ExactArithmetic,
            ),
            PredicateOutcome::unknown(RefinementNeed::ExactArithmetic, Escalation::Undecided)
        );
        assert!(!called.get());
    }
}
