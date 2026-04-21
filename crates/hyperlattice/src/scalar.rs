use crate::complex::Complex;
use crate::{AbortSignal, BlasResult, CheckedBlasResult, Problem, Scalar};

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ZeroStatus {
    Zero,
    NonZero,
    Unknown,
}

pub(crate) fn two() -> Scalar {
    2.into()
}

pub(crate) fn with_abort(mut value: Scalar, signal: &AbortSignal) -> Scalar {
    value.abort(signal.clone());
    value
}

pub(crate) fn clone_with_abort(value: &Scalar, signal: &AbortSignal) -> Scalar {
    with_abort(value.clone(), signal)
}

pub fn zero_status(value: &Scalar) -> ZeroStatus {
    value.zero_status()
}

pub fn zero_status_with_abort(value: &Scalar, signal: &AbortSignal) -> ZeroStatus {
    zero_status(&clone_with_abort(value, signal))
}

pub(crate) fn reject_definite_zero(value: &Scalar) -> BlasResult<()> {
    if value.definitely_zero() {
        Err(Problem::DivideByZero)
    } else {
        Ok(())
    }
}

pub(crate) fn require_known_nonzero(value: &Scalar) -> CheckedBlasResult<()> {
    match zero_status(value) {
        ZeroStatus::Zero => Err(Problem::DivideByZero),
        ZeroStatus::NonZero => Ok(()),
        ZeroStatus::Unknown => Err(Problem::UnknownZero),
    }
}

pub(crate) fn require_known_nonzero_with_abort(
    value: &Scalar,
    signal: &AbortSignal,
) -> CheckedBlasResult<()> {
    match zero_status_with_abort(value, signal) {
        ZeroStatus::Zero => Err(Problem::DivideByZero),
        ZeroStatus::NonZero => Ok(()),
        ZeroStatus::Unknown => Err(Problem::UnknownZero),
    }
}

fn real_from_f64(value: f64) -> BlasResult<Scalar> {
    if value.is_nan() {
        return Err(Problem::NotANumber);
    }
    if value.is_infinite() {
        return Err(Problem::Infinity);
    }
    Scalar::try_from(value)
}

pub fn zero() -> Scalar {
    Scalar::zero()
}

pub fn one() -> Scalar {
    Scalar::one()
}

pub fn e() -> Scalar {
    Scalar::e()
}

pub fn pi() -> Scalar {
    Scalar::pi()
}

pub fn tau() -> Scalar {
    two() * Scalar::pi()
}

pub fn i() -> Complex {
    Complex::i()
}

pub fn reciprocal(value: Scalar) -> BlasResult<Scalar> {
    value.inverse()
}

pub fn reciprocal_checked(value: Scalar) -> CheckedBlasResult<Scalar> {
    require_known_nonzero(&value)?;
    value.inverse()
}

pub fn reciprocal_checked_with_abort(
    value: Scalar,
    signal: &AbortSignal,
) -> CheckedBlasResult<Scalar> {
    let value = with_abort(value, signal);
    require_known_nonzero_with_abort(&value, signal)?;
    value.inverse()
}

pub fn pow(base: Scalar, exponent: Scalar) -> BlasResult<Scalar> {
    base.pow(exponent)
}

pub fn powi(base: Scalar, exponent: i64) -> BlasResult<Scalar> {
    if exponent == 0 {
        if base.definitely_zero() {
            return Err(Problem::NotANumber);
        }
        return Ok(one());
    }

    let mut exp = exponent.unsigned_abs();
    let mut result = one();
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

pub fn exp(value: Scalar) -> BlasResult<Scalar> {
    value.exp()
}

pub fn ln(value: Scalar) -> BlasResult<Scalar> {
    value.ln()
}

pub fn log10(value: Scalar) -> BlasResult<Scalar> {
    value.log10()
}

pub fn log10_with_abort(value: Scalar, signal: &AbortSignal) -> BlasResult<Scalar> {
    with_abort(value, signal).log10()
}

pub fn sqrt(value: Scalar) -> BlasResult<Scalar> {
    value.sqrt()
}

pub fn sin(value: Scalar) -> Scalar {
    value.sin()
}

pub fn cos(value: Scalar) -> Scalar {
    value.cos()
}

pub fn tan(value: Scalar) -> BlasResult<Scalar> {
    value.tan()
}

pub fn sinh(value: Scalar) -> BlasResult<Scalar> {
    let positive = value.clone().exp()?;
    let negative = (-value).exp()?;
    (positive - negative) / two()
}

pub fn cosh(value: Scalar) -> BlasResult<Scalar> {
    let positive = value.clone().exp()?;
    let negative = (-value).exp()?;
    (positive + negative) / two()
}

pub fn tanh(value: Scalar) -> BlasResult<Scalar> {
    sinh(value.clone())? / cosh(value)?
}

pub fn asin(value: Scalar) -> BlasResult<Scalar> {
    real_from_f64(f64::from(value).asin())
}

pub fn asin_with_abort(value: Scalar, signal: &AbortSignal) -> BlasResult<Scalar> {
    real_from_f64(f64::from(with_abort(value, signal)).asin())
}

pub fn acos(value: Scalar) -> BlasResult<Scalar> {
    real_from_f64(f64::from(value).acos())
}

pub fn acos_with_abort(value: Scalar, signal: &AbortSignal) -> BlasResult<Scalar> {
    real_from_f64(f64::from(with_abort(value, signal)).acos())
}

pub fn atan(value: Scalar) -> BlasResult<Scalar> {
    real_from_f64(f64::from(value).atan())
}

pub fn atan_with_abort(value: Scalar, signal: &AbortSignal) -> BlasResult<Scalar> {
    real_from_f64(f64::from(with_abort(value, signal)).atan())
}

pub fn asinh(value: Scalar) -> BlasResult<Scalar> {
    real_from_f64(f64::from(value).asinh())
}

pub fn asinh_with_abort(value: Scalar, signal: &AbortSignal) -> BlasResult<Scalar> {
    real_from_f64(f64::from(with_abort(value, signal)).asinh())
}

pub fn acosh(value: Scalar) -> BlasResult<Scalar> {
    real_from_f64(f64::from(value).acosh())
}

pub fn acosh_with_abort(value: Scalar, signal: &AbortSignal) -> BlasResult<Scalar> {
    real_from_f64(f64::from(with_abort(value, signal)).acosh())
}

pub fn atanh(value: Scalar) -> BlasResult<Scalar> {
    real_from_f64(f64::from(value).atanh())
}

pub fn atanh_with_abort(value: Scalar, signal: &AbortSignal) -> BlasResult<Scalar> {
    real_from_f64(f64::from(with_abort(value, signal)).atanh())
}
