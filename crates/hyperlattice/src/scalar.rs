//! Scalar constants, elementary functions, and zero-classification helpers.

use crate::complex::Complex;
use crate::{AbortSignal, Backend, BlasResult, CheckedBlasResult, Problem, Scalar};

/// Classification of whether a scalar is zero.
///
/// `Unknown` is important for computable reals and approximate intervals:
/// ordinary APIs may choose optimistic behavior, while checked APIs reject
/// unknown-zero values.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ZeroStatus {
    /// The value is definitely zero.
    Zero,
    /// The value is definitely non-zero.
    NonZero,
    /// The backend cannot prove whether the value is zero.
    Unknown,
}

pub(crate) fn two<B: Backend>() -> Scalar<B> {
    2.into()
}

pub(crate) fn with_abort<B: Backend>(mut value: Scalar<B>, signal: &AbortSignal) -> Scalar<B> {
    value.abort(signal.clone());
    value
}

pub(crate) fn clone_with_abort<B: Backend>(value: &Scalar<B>, signal: &AbortSignal) -> Scalar<B> {
    with_abort(value.clone(), signal)
}

/// Classifies a scalar as zero, non-zero, or unknown.
pub fn zero_status<B: Backend>(value: &Scalar<B>) -> ZeroStatus {
    value.zero_status()
}

/// Classifies a scalar after attaching an abort signal.
///
/// With the realistic backend, this allows long-running zero checks to observe
/// cancellation. With the approx backend, the signal is accepted as a no-op.
pub fn zero_status_with_abort<B: Backend>(value: &Scalar<B>, signal: &AbortSignal) -> ZeroStatus {
    zero_status(&clone_with_abort(value, signal))
}

pub(crate) fn reject_definite_zero<B: Backend>(value: &Scalar<B>) -> BlasResult<()> {
    if value.definitely_zero() {
        Err(Problem::DivideByZero)
    } else {
        Ok(())
    }
}

pub(crate) fn require_known_nonzero<B: Backend>(value: &Scalar<B>) -> CheckedBlasResult<()> {
    match zero_status(value) {
        ZeroStatus::Zero => Err(Problem::DivideByZero),
        ZeroStatus::NonZero => Ok(()),
        ZeroStatus::Unknown => Err(Problem::UnknownZero),
    }
}

pub(crate) fn require_known_nonzero_with_abort<B: Backend>(
    value: &Scalar<B>,
    signal: &AbortSignal,
) -> CheckedBlasResult<()> {
    match zero_status_with_abort(value, signal) {
        ZeroStatus::Zero => Err(Problem::DivideByZero),
        ZeroStatus::NonZero => Ok(()),
        ZeroStatus::Unknown => Err(Problem::UnknownZero),
    }
}

fn real_from_f64<B: Backend>(value: f64) -> BlasResult<Scalar<B>> {
    if value.is_nan() {
        return Err(Problem::NotANumber);
    }
    if value.is_infinite() {
        return Err(Problem::Infinity);
    }
    Scalar::<B>::try_from(value)
}

/// Returns the additive identity.
pub fn zero() -> Scalar {
    Scalar::zero()
}

/// Returns the multiplicative identity.
pub fn one() -> Scalar {
    Scalar::one()
}

/// Returns Euler's number.
pub fn e() -> Scalar {
    Scalar::e()
}

/// Returns pi.
pub fn pi() -> Scalar {
    Scalar::pi()
}

/// Returns tau, equal to `2 * pi`.
pub fn tau() -> Scalar {
    two() * Scalar::pi()
}

/// Returns the imaginary unit as a complex scalar.
pub fn i() -> Complex {
    Complex::i()
}

/// Returns the multiplicative inverse of `value`.
pub fn reciprocal<B: Backend>(value: Scalar<B>) -> BlasResult<Scalar<B>> {
    value.inverse()
}

/// Returns the multiplicative inverse after rejecting zero and unknown-zero values.
pub fn reciprocal_checked<B: Backend>(value: Scalar<B>) -> CheckedBlasResult<Scalar<B>> {
    require_known_nonzero(&value)?;
    value.inverse()
}

/// Returns the checked multiplicative inverse after attaching an abort signal.
pub fn reciprocal_checked_with_abort<B: Backend>(
    value: Scalar<B>,
    signal: &AbortSignal,
) -> CheckedBlasResult<Scalar<B>> {
    let value = with_abort(value, signal);
    require_known_nonzero_with_abort(&value, signal)?;
    value.inverse()
}

/// Raises `base` to a scalar exponent.
pub fn pow<B: Backend>(base: Scalar<B>, exponent: Scalar<B>) -> BlasResult<Scalar<B>> {
    base.pow(exponent)
}

/// Raises `base` to an integer exponent using exponentiation by squaring.
///
/// Negative exponents require the result to be invertible. `0^0` returns
/// [`Problem::NotANumber`].
pub fn powi<B: Backend>(base: Scalar<B>, exponent: i64) -> BlasResult<Scalar<B>> {
    if exponent == 0 {
        if base.definitely_zero() {
            return Err(Problem::NotANumber);
        }
        return Ok(Scalar::<B>::one());
    }

    let mut exp = exponent.unsigned_abs();
    let mut result = Scalar::<B>::one();
    let mut factor = base;
    while exp > 0 {
        if exp & 1 == 1 {
            result = result * factor.clone();
        }
        exp >>= 1;
        if exp > 0 {
            factor = factor.clone() * factor;
        }
    }

    if exponent < 0 {
        result.inverse()
    } else {
        Ok(result)
    }
}

/// Returns `e` raised to `value`.
pub fn exp<B: Backend>(value: Scalar<B>) -> BlasResult<Scalar<B>> {
    value.exp()
}

/// Returns the natural logarithm of `value`.
pub fn ln<B: Backend>(value: Scalar<B>) -> BlasResult<Scalar<B>> {
    value.ln()
}

/// Returns the base-10 logarithm of `value`.
pub fn log10<B: Backend>(value: Scalar<B>) -> BlasResult<Scalar<B>> {
    value.log10()
}

/// Returns the base-10 logarithm after attaching an abort signal.
pub fn log10_with_abort<B: Backend>(
    value: Scalar<B>,
    signal: &AbortSignal,
) -> BlasResult<Scalar<B>> {
    with_abort(value, signal).log10()
}

/// Returns the principal square root of `value`.
pub fn sqrt<B: Backend>(value: Scalar<B>) -> BlasResult<Scalar<B>> {
    value.sqrt()
}

/// Returns the sine of `value`.
pub fn sin<B: Backend>(value: Scalar<B>) -> Scalar<B> {
    value.sin()
}

/// Returns the cosine of `value`.
pub fn cos<B: Backend>(value: Scalar<B>) -> Scalar<B> {
    value.cos()
}

/// Returns the tangent of `value`.
pub fn tan<B: Backend>(value: Scalar<B>) -> BlasResult<Scalar<B>> {
    value.tan()
}

/// Returns the hyperbolic sine of `value`.
pub fn sinh<B: Backend>(value: Scalar<B>) -> BlasResult<Scalar<B>> {
    let positive = value.clone().exp()?;
    let negative = (-value).exp()?;
    (positive - negative) / two()
}

/// Returns the hyperbolic cosine of `value`.
pub fn cosh<B: Backend>(value: Scalar<B>) -> BlasResult<Scalar<B>> {
    let positive = value.clone().exp()?;
    let negative = (-value).exp()?;
    (positive + negative) / two()
}

/// Returns the hyperbolic tangent of `value`.
pub fn tanh<B: Backend>(value: Scalar<B>) -> BlasResult<Scalar<B>> {
    let positive = value.clone().exp()?;
    let negative = (-value).exp()?;
    (positive.clone() - negative.clone()) / (positive + negative)
}

/// Returns the inverse sine of `value`.
///
/// This helper converts through `f64`, so it is approximate rather than
/// symbolic with the realistic backend.
pub fn asin<B: Backend>(value: Scalar<B>) -> BlasResult<Scalar<B>> {
    real_from_f64(f64::from(value).asin())
}

/// Returns the inverse sine after attaching an abort signal.
pub fn asin_with_abort<B: Backend>(
    value: Scalar<B>,
    signal: &AbortSignal,
) -> BlasResult<Scalar<B>> {
    real_from_f64(f64::from(with_abort(value, signal)).asin())
}

/// Returns the inverse cosine of `value`.
///
/// This helper converts through `f64`, so it is approximate rather than
/// symbolic with the realistic backend.
pub fn acos<B: Backend>(value: Scalar<B>) -> BlasResult<Scalar<B>> {
    real_from_f64(f64::from(value).acos())
}

/// Returns the inverse cosine after attaching an abort signal.
pub fn acos_with_abort<B: Backend>(
    value: Scalar<B>,
    signal: &AbortSignal,
) -> BlasResult<Scalar<B>> {
    real_from_f64(f64::from(with_abort(value, signal)).acos())
}

/// Returns the inverse tangent of `value`.
///
/// This helper converts through `f64`, so it is approximate rather than
/// symbolic with the realistic backend.
pub fn atan<B: Backend>(value: Scalar<B>) -> BlasResult<Scalar<B>> {
    real_from_f64(f64::from(value).atan())
}

/// Returns the inverse tangent after attaching an abort signal.
pub fn atan_with_abort<B: Backend>(
    value: Scalar<B>,
    signal: &AbortSignal,
) -> BlasResult<Scalar<B>> {
    real_from_f64(f64::from(with_abort(value, signal)).atan())
}

/// Returns the inverse hyperbolic sine of `value`.
///
/// This helper converts through `f64`, so it is approximate rather than
/// symbolic with the realistic backend.
pub fn asinh<B: Backend>(value: Scalar<B>) -> BlasResult<Scalar<B>> {
    real_from_f64(f64::from(value).asinh())
}

/// Returns the inverse hyperbolic sine after attaching an abort signal.
pub fn asinh_with_abort<B: Backend>(
    value: Scalar<B>,
    signal: &AbortSignal,
) -> BlasResult<Scalar<B>> {
    real_from_f64(f64::from(with_abort(value, signal)).asinh())
}

/// Returns the inverse hyperbolic cosine of `value`.
///
/// This helper converts through `f64`, so it is approximate rather than
/// symbolic with the realistic backend.
pub fn acosh<B: Backend>(value: Scalar<B>) -> BlasResult<Scalar<B>> {
    real_from_f64(f64::from(value).acosh())
}

/// Returns the inverse hyperbolic cosine after attaching an abort signal.
pub fn acosh_with_abort<B: Backend>(
    value: Scalar<B>,
    signal: &AbortSignal,
) -> BlasResult<Scalar<B>> {
    real_from_f64(f64::from(with_abort(value, signal)).acosh())
}

/// Returns the inverse hyperbolic tangent of `value`.
///
/// This helper converts through `f64`, so it is approximate rather than
/// symbolic with the realistic backend.
pub fn atanh<B: Backend>(value: Scalar<B>) -> BlasResult<Scalar<B>> {
    real_from_f64(f64::from(value).atanh())
}

/// Returns the inverse hyperbolic tangent after attaching an abort signal.
pub fn atanh_with_abort<B: Backend>(
    value: Scalar<B>,
    signal: &AbortSignal,
) -> BlasResult<Scalar<B>> {
    real_from_f64(f64::from(with_abort(value, signal)).atanh())
}
