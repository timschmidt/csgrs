use core::cmp::Ordering;

use hyperlimit::{PredicateOutcome, PredicatePolicy, Sign, compare_reals};
use hyperreal::{CertifiedRealSign, Problem, Real, RealSign, ZeroKnowledge};

/// Finish a sign query through the strict exact predicate cascade after a
/// caller-selected scalar-refinement bound has been exhausted. `None` remains
/// an explicit unsupported identity; this helper never enables approximation.
#[inline]
pub(crate) fn strict_sign_after_refinement(value: &Real, min_precision: i32) -> Option<RealSign> {
    if let CertifiedRealSign::Known { sign, .. } = value.certified_sign_until(min_precision) {
        return Some(sign);
    }
    strict_sign_after_refinement_failure(value)
}

#[cold]
pub(crate) fn strict_sign_after_refinement_failure(value: &Real) -> Option<RealSign> {
    match hyperlimit::classify_real_sign(value, PredicatePolicy::STRICT) {
        PredicateOutcome::Decided { value, .. } => Some(match value {
            Sign::Negative => RealSign::Negative,
            Sign::Zero => RealSign::Zero,
            Sign::Positive => RealSign::Positive,
        }),
        PredicateOutcome::Unknown { .. } => None,
    }
}

/// Returns true only when removing one scalar from exact polynomial storage is
/// justified. Unsupported identities are deliberately retained.
#[inline]
pub(crate) fn strict_exact_zero_for_storage(value: &Real) -> bool {
    match value.zero_status() {
        ZeroKnowledge::Zero => true,
        ZeroKnowledge::NonZero => false,
        ZeroKnowledge::Unknown => matches!(
            strict_sign_after_refinement_failure(value),
            Some(RealSign::Zero)
        ),
    }
}

/// Construct a reciprocal after the caller has already retained an exact
/// nonzero decision, preserving the ordinary inverse fast path when it works.
#[inline]
pub(crate) fn reciprocal_after_certified_nonzero(value: &Real) -> Result<Real, Problem> {
    match value.inverse_ref() {
        Ok(reciprocal) => Ok(reciprocal),
        Err(_) => reciprocal_after_certified_nonzero_fallback(value),
    }
}

#[cold]
fn reciprocal_after_certified_nonzero_fallback(value: &Real) -> Result<Real, Problem> {
    value.inverse_ref_assuming_nonzero()
}

/// Preserve `Real`'s structural inverse fast paths, then use the caller's
/// predicate policy only when ordinary inversion cannot decide nonzero status.
#[inline]
pub(crate) fn reciprocal_with_policy(value: &Real, policy: PredicatePolicy) -> Option<Real> {
    match value.inverse_ref() {
        Ok(reciprocal) => Some(reciprocal),
        Err(_) => reciprocal_with_policy_fallback(value, policy),
    }
}

#[cold]
fn reciprocal_with_policy_fallback(value: &Real, policy: PredicatePolicy) -> Option<Real> {
    match compare_reals(value, &Real::zero(), policy).value() {
        Some(Ordering::Less | Ordering::Greater) => value.inverse_ref_assuming_nonzero().ok(),
        Some(Ordering::Equal) | None => None,
    }
}
