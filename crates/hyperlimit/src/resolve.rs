//! Shared sign-resolution helpers for predicate pipelines.

use crate::predicate::PredicatePolicy;
use crate::predicate::{
    Certainty, Escalation, PredicateOutcome, RefinementNeed, Sign, SignKnowledge,
};
use crate::real::{RealPredicateExt, map_real_sign};
use hyperreal::{CertifiedRealSign, Real, RealSignCertificate, ZeroKnowledge};

/// Resolve a Real sign through the common predicate pipeline.
///
/// `exact` is the predicate-level exact evaluation hook. It should do actual
/// exact determinant/sign work for the whole predicate, while Real facts only
/// certify signs that are already exposed by the computed Real value. If both
/// stages and bounded refinement are inconclusive, an exact rational normal
/// form is attempted before policy-authorized approximation.
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

    if let Some(outcome) = exact_evaluation(exact) {
        crate::trace_dispatch!("hyperlimit", "resolve_real_sign", "exact-predicate");
        return outcome;
    }

    if let Some(outcome) = refine_real_sign(value) {
        crate::trace_dispatch!("hyperlimit", "resolve_real_sign", "real-refinement");
        return outcome;
    }

    if let Some(sign) = exact_rational_normal_form_sign(value) {
        crate::trace_dispatch!("hyperlimit", "resolve_real_sign", "exact-normal-form");
        return PredicateOutcome::decided(sign, Certainty::Exact, Escalation::Exact);
    }

    if let Some(outcome) = approximate_real_sign(value, policy) {
        crate::trace_dispatch!(
            "hyperlimit",
            "resolve_real_sign",
            "policy-final-approximation"
        );
        return outcome;
    }

    crate::trace_dispatch!("hyperlimit", "resolve_real_sign", "unknown");
    PredicateOutcome::unknown(unknown_need, Escalation::Undecided)
}

/// Resolve a Real sign when no predicate-specific filter or exact callback
/// exists between structural inspection and bounded refinement.
///
/// Calling `certified_sign_until` once avoids the duplicate structural-facts
/// pass in the general resolver. If bounded refinement is inconclusive, an
/// exact rational normal form is still attempted before any policy-authorized
/// terminal approximation so approximate zero cannot mask an exact nonzero
/// result.
pub(crate) fn resolve_real_sign_direct(
    value: &Real,
    policy: PredicatePolicy,
    unknown_need: RefinementNeed,
) -> PredicateOutcome<Sign> {
    match value.certified_sign_until(PredicatePolicy::MAX_REFINEMENT_PRECISION) {
        CertifiedRealSign::Known { sign, certificate } => {
            let stage = match certificate {
                RealSignCertificate::StructuralFacts | RealSignCertificate::ExactZeroScale => {
                    crate::trace_dispatch!(
                        "hyperlimit",
                        "resolve_real_sign_direct",
                        "structural-real-facts"
                    );
                    Escalation::Structural
                }
                RealSignCertificate::BoundedRefinement { .. } => {
                    crate::trace_dispatch!(
                        "hyperlimit",
                        "resolve_real_sign_direct",
                        "real-refinement"
                    );
                    Escalation::Refined
                }
            };
            PredicateOutcome::decided(map_real_sign(sign), Certainty::Exact, stage)
        }
        CertifiedRealSign::Unknown { .. } => {
            if let Some(sign) = exact_rational_normal_form_sign(value) {
                crate::trace_dispatch!(
                    "hyperlimit",
                    "resolve_real_sign_direct",
                    "exact-normal-form"
                );
                return PredicateOutcome::decided(sign, Certainty::Exact, Escalation::Exact);
            }
            approximate_real_sign(value, policy).unwrap_or_else(|| {
                crate::trace_dispatch!("hyperlimit", "resolve_real_sign_direct", "unknown");
                PredicateOutcome::unknown(unknown_need, Escalation::Undecided)
            })
        }
    }
}

fn exact_rational_normal_form_sign(value: &Real) -> Option<Sign> {
    let rational = value.exact_rational_normal_form()?;
    let zero = hyperreal::Rational::zero();
    Some(
        match rational
            .partial_cmp(&zero)
            .expect("exact rational ordering is total")
        {
            core::cmp::Ordering::Less => Sign::Negative,
            core::cmp::Ordering::Equal => Sign::Zero,
            core::cmp::Ordering::Greater => Sign::Positive,
        },
    )
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

/// Evaluate a composite predicate without allowing child certainty to vanish.
///
/// Composite classifiers often have several sufficient decision paths. Running
/// the strict policy first means a result reached without terminal
/// approximation keeps its original evidence. Only when strict evaluation is
/// undecided do we replay with the caller-authorized approximation and mark the
/// whole result approximate. This is reserved for composite implementations
/// whose internal reports do not carry per-child certainty.
pub(crate) fn resolve_composite_policy<T>(
    policy: PredicatePolicy,
    mut evaluate: impl FnMut(PredicatePolicy) -> PredicateOutcome<T>,
) -> PredicateOutcome<T> {
    if policy != PredicatePolicy::APPROXIMATE_512 {
        return evaluate(policy);
    }

    match evaluate(PredicatePolicy::STRICT) {
        decided @ PredicateOutcome::Decided { .. } => decided,
        PredicateOutcome::Unknown { .. } => match evaluate(policy) {
            PredicateOutcome::Decided { value, .. } => {
                PredicateOutcome::decided(value, Certainty::Approximate, Escalation::Refined)
            }
            unknown @ PredicateOutcome::Unknown { .. } => unknown,
        },
    }
}

fn exact_evaluation(exact: impl FnOnce() -> Option<Sign>) -> Option<PredicateOutcome<Sign>> {
    exact().map(|sign| {
        crate::trace_dispatch!("hyperlimit", "exact_evaluation", "decided");
        PredicateOutcome::decided(sign, Certainty::Exact, Escalation::Exact)
    })
}

fn refine_real_sign(value: &Real) -> Option<PredicateOutcome<Sign>> {
    match value.refine_sign_knowledge_until(PredicatePolicy::MAX_REFINEMENT_PRECISION) {
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

fn approximate_real_sign(value: &Real, policy: PredicatePolicy) -> Option<PredicateOutcome<Sign>> {
    let precision = policy.final_approximation_precision()?;
    let [lower, upper] = value.certified_dyadic_interval(precision)?;
    let zero = hyperreal::Rational::zero();
    let sign = if upper < zero {
        Sign::Negative
    } else if lower > zero {
        Sign::Positive
    } else {
        Sign::Zero
    };
    crate::trace_dispatch!("hyperlimit", "approximate_real_sign", "decided");
    Some(PredicateOutcome::decided(
        sign,
        Certainty::Approximate,
        Escalation::Refined,
    ))
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
        let Some(sign) = signed_nonzero_term(term, *multiplier)? else {
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
        let Some(sign) = signed_nonzero_term(term, *multiplier)? else {
            continue;
        };
        nonzero.push(sign);
    }

    finish_signed_term_filter(&nonzero)
}

#[inline(always)]
fn signed_nonzero_term(term: &Real, multiplier: Sign) -> Option<Option<Sign>> {
    let facts = term.structural_facts();
    if matches!(facts.zero, ZeroKnowledge::Zero) {
        crate::trace_dispatch!("hyperlimit", "signed_term_filter", "zero-term");
        return Some(None);
    }

    let sign = facts.sign.map(crate::real::map_real_sign)?;
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
    use super::*;
    use hyperreal::Rational;

    fn terminally_unresolved_zero() -> Real {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        &sine * &sine + &cosine * &cosine - Real::one()
    }

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
    fn terminal_approximation_is_policy_controlled_and_reports_its_certainty() {
        // This exact identity deliberately remains outside bounded structural
        // normalization so only the terminal policy may consume it.
        let value = terminally_unresolved_zero();

        assert!(matches!(
            resolve_real_sign_direct(
                &value,
                PredicatePolicy::STRICT,
                RefinementNeed::RealRefinement,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert_eq!(
            resolve_real_sign_direct(
                &value,
                PredicatePolicy::APPROXIMATE_512,
                RefinementNeed::RealRefinement,
            ),
            PredicateOutcome::decided(Sign::Zero, Certainty::Approximate, Escalation::Refined,)
        );
    }

    #[test]
    fn general_resolver_normalizes_exact_rationals_before_approximation() {
        let root_two = Real::from(2).sqrt().unwrap();
        let root_two_over_pi = (root_two.clone() / Real::pi()).unwrap();
        let half = (Real::from(1) / Real::from(2)).unwrap();
        let shared_offset = root_two.clone() * Real::from(3) + half;
        let contact = (((root_two.clone() * Real::from(4) - shared_offset.clone()) * Real::pi())
            * root_two_over_pi.clone()
            / Real::from(4))
        .unwrap();
        let domain = (((root_two * Real::from(2) - shared_offset) * Real::pi()) * root_two_over_pi
            / Real::from(4))
        .unwrap()
            + Real::from(1);
        let tiny = Real::from(2).powi_i64(-600).unwrap();
        let positive = contact - domain + tiny;

        assert_eq!(
            resolve_real_sign(
                &positive,
                PredicatePolicy::APPROXIMATE_512,
                || None,
                || None,
                RefinementNeed::RealRefinement,
            ),
            PredicateOutcome::decided(Sign::Positive, Certainty::Exact, Escalation::Exact)
        );
    }
}
