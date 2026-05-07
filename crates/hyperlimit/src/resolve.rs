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
    decide_scalar_sign(value, Escalation::Structural)
        .or_else(filter)
        .or_else(|| exact_scalar_sign_if_allowed(value, policy))
        .or_else(|| exact_evaluation_if_allowed(policy, exact))
        .or_else(|| refine_scalar_sign_if_allowed(value, policy))
        .or_else(fallback)
        .or_else(|| approximate_if_allowed(value, policy))
        .unwrap_or_else(|| PredicateOutcome::unknown(unknown_need, Escalation::Undecided))
}

pub(crate) fn decide_scalar_sign<S: PredicateScalar>(
    value: &S,
    stage: Escalation,
) -> Option<PredicateOutcome<Sign>> {
    match value.known_sign() {
        SignKnowledge::Known { sign, certainty } => {
            Some(PredicateOutcome::decided(sign, certainty, stage))
        }
        SignKnowledge::NonZero | SignKnowledge::Unknown => None,
    }
}

pub(crate) fn approximate_if_allowed<S: PredicateScalar>(
    value: &S,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    if !policy.allow_approximate {
        return None;
    }

    let sign = Sign::from_f64(value.to_f64()?)?;
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
        return None;
    }

    let facts = value.scalar_facts();
    if facts.exact != Some(true) && facts.rational_only != Some(true) {
        return None;
    }

    match facts.sign_knowledge() {
        SignKnowledge::Known { sign, .. } => Some(PredicateOutcome::decided(
            sign,
            Certainty::Exact,
            Escalation::Exact,
        )),
        SignKnowledge::NonZero | SignKnowledge::Unknown => None,
    }
}

fn exact_evaluation_if_allowed(
    policy: PredicatePolicy,
    exact: impl FnOnce() -> Option<Sign>,
) -> Option<PredicateOutcome<Sign>> {
    if !policy.allow_exact {
        return None;
    }

    exact().map(|sign| PredicateOutcome::decided(sign, Certainty::Exact, Escalation::Exact))
}

fn refine_scalar_sign_if_allowed<S: PredicateScalar>(
    value: &S,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    if !policy.allow_refinement {
        return None;
    }

    let precision = policy.max_refinement_precision?;
    match value.refine_sign_until(precision) {
        SignKnowledge::Known { sign, certainty } => Some(PredicateOutcome::decided(
            sign,
            certainty,
            Escalation::Refined,
        )),
        SignKnowledge::NonZero | SignKnowledge::Unknown => None,
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
            continue;
        }

        let sign = facts.sign?;
        let sign = multiply_sign(sign, *multiplier);
        if sign == Sign::Zero {
            continue;
        }
        nonzero.push((sign, facts.magnitude?));
    }

    if nonzero.is_empty() {
        return Some(PredicateOutcome::decided(
            Sign::Zero,
            Certainty::Filtered,
            Escalation::Filter,
        ));
    }

    let first = nonzero[0].0;
    if nonzero.iter().all(|(sign, _)| *sign == first) {
        return Some(PredicateOutcome::decided(
            first,
            Certainty::Filtered,
            Escalation::Filter,
        ));
    }

    dominance_sign(&nonzero)
        .map(|sign| PredicateOutcome::decided(sign, Certainty::Filtered, Escalation::Filter))
}

fn dominance_sign(terms: &[(Sign, MagnitudeBounds)]) -> Option<Sign> {
    // Dominant-term detection catches expressions like `pi - 3` without
    // constructing exact fallback objects. The two-bit gap is conservative:
    // it leaves ambiguous near-cancellation to the slower but safer path.
    for (index, (sign, magnitude)) in terms.iter().enumerate() {
        if magnitude.abs_lower <= 0.0 {
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
            return Some(*sign);
        }
    }

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
