//! [`Real`](crate::Real) constants, elementary functions, and zero-classification helpers.

use crate::complex::Complex;
use crate::{
    AbortSignal, BlasResult, CheckedBlasResult, Problem, Real, RealDomainStatus, ZeroStatus,
};
use std::sync::atomic::Ordering;

#[inline(always)]
pub(crate) fn with_abort(mut value: Real, signal: &AbortSignal) -> Real {
    crate::trace_dispatch!("hyperlattice", "abort", "attach-owned-real");
    value.abort(signal.clone());
    value
}

#[inline(always)]
pub(crate) fn clone_with_abort(value: &Real, signal: &AbortSignal) -> Real {
    crate::trace_dispatch!("hyperlattice", "abort", "clone-and-attach");
    with_abort(value.clone(), signal)
}

/// Classifies a real value as zero, non-zero, or unknown.
#[inline(always)]
pub fn zero_status(value: &Real) -> ZeroStatus {
    crate::trace_dispatch!("hyperlattice", "zero_status", "real-query");
    value.zero_status()
}

/// Classifies a real value after attaching an abort signal.
///
/// This lets long-running zero checks on opaque computable reals observe
/// cancellation while keeping the structural fast path allocation-free.
#[inline(always)]
pub fn zero_status_with_abort(value: &Real, signal: &AbortSignal) -> ZeroStatus {
    let status = zero_status(value);
    if status != ZeroStatus::Unknown || !signal.load(Ordering::Relaxed) {
        crate::trace_dispatch!("hyperlattice", "zero_status_abort", "no-clone-fast-path");
        return status;
    }

    crate::trace_dispatch!(
        "hyperlattice",
        "zero_status_abort",
        "clone-with-active-abort"
    );
    zero_status(&clone_with_abort(value, signal))
}

#[inline(always)]
pub(crate) fn reject_definite_zero(value: &Real) -> BlasResult<()> {
    if value.definitely_zero() {
        crate::trace_dispatch!("hyperlattice", "zero_guard", "definite-zero-rejected");
        Err(Problem::DivideByZero)
    } else {
        crate::trace_dispatch!("hyperlattice", "zero_guard", "not-definitely-zero");
        Ok(())
    }
}

#[inline(always)]
pub(crate) fn require_known_nonzero(value: &Real) -> CheckedBlasResult<()> {
    match zero_status(value) {
        ZeroStatus::Zero => {
            crate::trace_dispatch!("hyperlattice", "zero_guard", "checked-zero-rejected");
            Err(Problem::DivideByZero)
        }
        ZeroStatus::NonZero => {
            crate::trace_dispatch!("hyperlattice", "zero_guard", "checked-nonzero");
            Ok(())
        }
        ZeroStatus::Unknown => {
            crate::trace_dispatch!("hyperlattice", "zero_guard", "checked-unknown-rejected");
            Err(Problem::UnknownZero)
        }
    }
}

#[inline(always)]
pub(crate) fn require_known_nonzero_with_abort(
    value: &Real,
    signal: &AbortSignal,
) -> CheckedBlasResult<()> {
    match zero_status_with_abort(value, signal) {
        ZeroStatus::Zero => Err(Problem::DivideByZero),
        ZeroStatus::NonZero => Ok(()),
        ZeroStatus::Unknown => Err(Problem::UnknownZero),
    }
}

/// Returns the additive identity.
pub fn zero() -> Real {
    crate::trace_dispatch!("hyperlattice", "free_function", "zero");
    Real::zero()
}

/// Returns the multiplicative identity.
pub fn one() -> Real {
    crate::trace_dispatch!("hyperlattice", "free_function", "one");
    Real::one()
}

/// Returns Euler's number.
pub fn e() -> Real {
    crate::trace_dispatch!("hyperlattice", "free_function", "e");
    Real::e()
}

/// Returns pi.
pub fn pi() -> Real {
    crate::trace_dispatch!("hyperlattice", "free_function", "pi");
    Real::pi()
}

/// Returns tau, equal to `2 * pi`.
pub fn tau() -> Real {
    crate::trace_dispatch!("hyperlattice", "free_function", "tau");
    Real::tau()
}

/// Returns the imaginary unit as a complex scalar.
pub fn i() -> Complex {
    Complex::i()
}

/// Returns the multiplicative inverse of `value`.
pub fn reciprocal(value: Real) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "reciprocal-owned");
    value.inverse()
}

/// Returns the multiplicative inverse of `value` without consuming it.
pub fn reciprocal_ref(value: &Real) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "reciprocal-ref");
    value.inverse_ref()
}

/// Returns the multiplicative inverse after rejecting zero and unknown-zero values.
pub fn reciprocal_checked(value: Real) -> CheckedBlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "reciprocal-checked-owned");
    require_known_nonzero(&value)?;
    value.inverse()
}

/// Returns the checked multiplicative inverse without consuming `value`.
pub fn reciprocal_ref_checked(value: &Real) -> CheckedBlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "reciprocal-checked-ref");
    require_known_nonzero(value)?;
    value.inverse_ref()
}

/// Returns the checked multiplicative inverse after attaching an abort signal.
pub fn reciprocal_checked_with_abort(value: Real, signal: &AbortSignal) -> CheckedBlasResult<Real> {
    crate::trace_dispatch!(
        "hyperlattice",
        "free_function",
        "reciprocal-checked-with-abort"
    );
    let value = with_abort(value, signal);
    require_known_nonzero_with_abort(&value, signal)?;
    value.inverse()
}

/// Raises `base` to a scalar exponent.
pub fn pow(base: Real, exponent: Real) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "pow");
    base.pow(exponent)
}

/// Raises `base` to an integer exponent using exponentiation by squaring.
///
/// Negative exponents require the result to be invertible. `0^0` returns
/// [`Problem::NotANumber`].
pub fn powi(base: Real, exponent: i64) -> BlasResult<Real> {
    if exponent == 0 {
        if base.definitely_zero() {
            crate::trace_dispatch!("hyperlattice", "powi", "zero-to-zero-domain-error");
            return Err(Problem::NotANumber);
        }
        crate::trace_dispatch!("hyperlattice", "powi", "exponent-zero-one");
        return Ok(Real::one());
    }

    let exp = exponent.unsigned_abs();
    let positive = match exp {
        1 => {
            crate::trace_dispatch!("hyperlattice", "powi", "exponent-one");
            base
        }
        2 => {
            crate::trace_dispatch!("hyperlattice", "powi", "specialized-square");
            base.clone() * base
        }
        3 => {
            crate::trace_dispatch!("hyperlattice", "powi", "specialized-cube");
            let square = base.clone() * base.clone();
            square * base
        }
        4 => {
            crate::trace_dispatch!("hyperlattice", "powi", "specialized-fourth");
            let square = base.clone() * base;
            square.clone() * square
        }
        5 => {
            crate::trace_dispatch!("hyperlattice", "powi", "specialized-fifth");
            let square = base.clone() * base.clone();
            let fourth = square.clone() * square;
            fourth * base
        }
        _ => {
            crate::trace_dispatch!("hyperlattice", "powi", "generic-squaring");
            powi_by_squaring(base, exp)
        }
    };

    if exponent < 0 {
        crate::trace_dispatch!("hyperlattice", "powi", "negative-inverse");
        positive.inverse()
    } else {
        Ok(positive)
    }
}

fn powi_by_squaring(base: Real, exponent: u64) -> Real {
    let mut exp = exponent;
    let mut result = None;
    let mut factor = base;
    while exp > 0 {
        if exp & 1 == 1 {
            result = Some(match result {
                Some(result) => result * factor.clone(),
                None => factor.clone(),
            });
        }
        exp >>= 1;
        if exp > 0 {
            factor = factor.clone() * factor;
        }
    }

    result.expect("non-zero exponent sets at least one result bit")
}

/// Returns `e` raised to `value`.
pub fn exp(value: Real) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "exp");
    value.exp()
}

/// Returns the natural logarithm of `value`.
pub fn ln(value: Real) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "ln");
    reject_invalid_domain(value.log_domain(), Problem::NotANumber)?;
    value.ln()
}

/// Returns the base-10 logarithm of `value`.
pub fn log10(value: Real) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "log10");
    reject_invalid_domain(value.log_domain(), Problem::NotANumber)?;
    value.log10()
}

/// Returns the base-10 logarithm after attaching an abort signal.
pub fn log10_with_abort(value: Real, signal: &AbortSignal) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "log10-with-abort");
    reject_invalid_domain(value.log_domain(), Problem::NotANumber)?;
    with_abort(value, signal).log10()
}

/// Returns the principal square root of `value`.
pub fn sqrt(value: Real) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "sqrt");
    reject_invalid_domain(value.sqrt_domain(), Problem::SqrtNegative)?;
    value.sqrt()
}

/// Returns the sine of `value`.
pub fn sin(value: Real) -> Real {
    crate::trace_dispatch!("hyperlattice", "free_function", "sin");
    value.sin()
}

/// Returns the cosine of `value`.
pub fn cos(value: Real) -> Real {
    crate::trace_dispatch!("hyperlattice", "free_function", "cos");
    value.cos()
}

/// Returns the tangent of `value`.
pub fn tan(value: Real) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "tan");
    value.tan()
}

/// Returns the hyperbolic sine of `value`.
pub fn sinh(value: Real) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "sinh-exp-formula");
    let positive = value.clone().exp()?;
    let negative = (-value).exp()?;
    (positive - negative) / Real::from(2_i8)
}

/// Returns the hyperbolic cosine of `value`.
pub fn cosh(value: Real) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "cosh-exp-formula");
    let positive = value.clone().exp()?;
    let negative = (-value).exp()?;
    (positive + negative) / Real::from(2_i8)
}

/// Returns the hyperbolic tangent of `value`.
pub fn tanh(value: Real) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "tanh-exp-formula");
    let positive = value.clone().exp()?;
    let negative = (-value).exp()?;
    (positive.clone() - negative.clone()) / (positive + negative)
}

/// Returns the inverse sine of `value`.
pub fn asin(value: Real) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "asin");
    reject_invalid_domain(value.asin_acos_domain(), Problem::NotANumber)?;
    value.asin()
}

/// Returns the inverse sine after attaching an abort signal.
pub fn asin_with_abort(value: Real, signal: &AbortSignal) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "asin-with-abort");
    reject_invalid_domain(value.asin_acos_domain(), Problem::NotANumber)?;
    with_abort(value, signal).asin()
}

/// Returns the inverse cosine of `value`.
pub fn acos(value: Real) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "acos");
    reject_invalid_domain(value.asin_acos_domain(), Problem::NotANumber)?;
    value.acos()
}

/// Returns the inverse cosine after attaching an abort signal.
pub fn acos_with_abort(value: Real, signal: &AbortSignal) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "acos-with-abort");
    reject_invalid_domain(value.asin_acos_domain(), Problem::NotANumber)?;
    with_abort(value, signal).acos()
}

/// Returns the inverse tangent of `value`.
pub fn atan(value: Real) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "atan");
    value.atan()
}

/// Returns the inverse tangent after attaching an abort signal.
pub fn atan_with_abort(value: Real, signal: &AbortSignal) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "atan-with-abort");
    with_abort(value, signal).atan()
}

/// Returns the inverse hyperbolic sine of `value`.
pub fn asinh(value: Real) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "asinh");
    value.asinh()
}

/// Returns the inverse hyperbolic sine after attaching an abort signal.
pub fn asinh_with_abort(value: Real, signal: &AbortSignal) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "asinh-with-abort");
    with_abort(value, signal).asinh()
}

/// Returns the inverse hyperbolic cosine of `value`.
pub fn acosh(value: Real) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "acosh");
    reject_invalid_domain(value.acosh_domain(), Problem::NotANumber)?;
    value.acosh()
}

/// Returns the inverse hyperbolic cosine after attaching an abort signal.
pub fn acosh_with_abort(value: Real, signal: &AbortSignal) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "acosh-with-abort");
    reject_invalid_domain(value.acosh_domain(), Problem::NotANumber)?;
    with_abort(value, signal).acosh()
}

/// Returns the inverse hyperbolic tangent of `value`.
pub fn atanh(value: Real) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "atanh");
    value.atanh()
}

/// Returns the inverse hyperbolic tangent after attaching an abort signal.
pub fn atanh_with_abort(value: Real, signal: &AbortSignal) -> BlasResult<Real> {
    crate::trace_dispatch!("hyperlattice", "free_function", "atanh-with-abort");
    with_abort(value, signal).atanh()
}

#[inline(always)]
fn reject_invalid_domain(status: RealDomainStatus, problem: Problem) -> BlasResult<()> {
    match status {
        RealDomainStatus::Invalid => {
            crate::trace_dispatch!("hyperlattice", "domain", "structural-invalid");
            Err(problem)
        }
        RealDomainStatus::Valid => {
            crate::trace_dispatch!("hyperlattice", "domain", "structural-valid");
            Ok(())
        }
        RealDomainStatus::Unknown => {
            crate::trace_dispatch!("hyperlattice", "domain", "structural-unknown");
            Ok(())
        }
    }
}
