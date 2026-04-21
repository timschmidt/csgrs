use num::bigint::Sign;

use crate::complex::Complex;
use crate::{AbortSignal, BlasProblem, BlasResult, CheckedBlasResult, Problem, Real};

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ZeroStatus {
    Zero,
    NonZero,
    Unknown,
}

pub(crate) fn two() -> Real {
    2.into()
}

pub(crate) fn with_abort(mut value: Real, signal: &AbortSignal) -> Real {
    value.abort(signal.clone());
    value
}

pub(crate) fn clone_with_abort(value: &Real, signal: &AbortSignal) -> Real {
    with_abort(value.clone(), signal)
}

pub fn zero_status(value: &Real) -> ZeroStatus {
    if value.definitely_zero() {
        ZeroStatus::Zero
    } else {
        match value.best_sign() {
            Sign::Plus | Sign::Minus => ZeroStatus::NonZero,
            Sign::NoSign => ZeroStatus::Unknown,
        }
    }
}

pub fn zero_status_with_abort(value: &Real, signal: &AbortSignal) -> ZeroStatus {
    zero_status(&clone_with_abort(value, signal))
}

pub(crate) fn reject_definite_zero(value: &Real) -> BlasResult<()> {
    if value.definitely_zero() {
        Err(Problem::DivideByZero)
    } else {
        Ok(())
    }
}

pub(crate) fn require_known_nonzero(value: &Real) -> CheckedBlasResult<()> {
    match zero_status(value) {
        ZeroStatus::Zero => Err(BlasProblem::Real(Problem::DivideByZero)),
        ZeroStatus::NonZero => Ok(()),
        ZeroStatus::Unknown => Err(BlasProblem::UnknownZero),
    }
}

pub(crate) fn require_known_nonzero_with_abort(
    value: &Real,
    signal: &AbortSignal,
) -> CheckedBlasResult<()> {
    match zero_status_with_abort(value, signal) {
        ZeroStatus::Zero => Err(BlasProblem::Real(Problem::DivideByZero)),
        ZeroStatus::NonZero => Ok(()),
        ZeroStatus::Unknown => Err(BlasProblem::UnknownZero),
    }
}

fn real_from_f64(value: f64) -> BlasResult<Real> {
    if value.is_nan() {
        return Err(Problem::NotANumber);
    }
    if value.is_infinite() {
        return Err(Problem::Infinity);
    }
    Real::try_from(value)
}

pub fn zero() -> Real {
    Real::zero()
}

pub fn one() -> Real {
    1.into()
}

pub fn e() -> Real {
    Real::e()
}

pub fn pi() -> Real {
    Real::pi()
}

pub fn tau() -> Real {
    two() * Real::pi()
}

pub fn i() -> Complex {
    Complex::i()
}

pub fn reciprocal(value: Real) -> BlasResult<Real> {
    value.inverse()
}

pub fn reciprocal_checked(value: Real) -> CheckedBlasResult<Real> {
    require_known_nonzero(&value)?;
    value.inverse().map_err(BlasProblem::from)
}

pub fn reciprocal_checked_with_abort(value: Real, signal: &AbortSignal) -> CheckedBlasResult<Real> {
    let value = with_abort(value, signal);
    require_known_nonzero_with_abort(&value, signal)?;
    value.inverse().map_err(BlasProblem::from)
}

pub fn pow(base: Real, exponent: Real) -> BlasResult<Real> {
    base.pow(exponent)
}

pub fn powi(base: Real, exponent: i64) -> BlasResult<Real> {
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

pub fn exp(value: Real) -> BlasResult<Real> {
    value.exp()
}

pub fn ln(value: Real) -> BlasResult<Real> {
    value.ln()
}

pub fn log10(value: Real) -> BlasResult<Real> {
    value.log10()
}

pub fn log10_with_abort(value: Real, signal: &AbortSignal) -> BlasResult<Real> {
    with_abort(value, signal).log10()
}

pub fn sqrt(value: Real) -> BlasResult<Real> {
    value.sqrt()
}

pub fn sin(value: Real) -> Real {
    value.sin()
}

pub fn cos(value: Real) -> Real {
    value.cos()
}

pub fn tan(value: Real) -> BlasResult<Real> {
    value.tan()
}

pub fn sinh(value: Real) -> BlasResult<Real> {
    let positive = value.clone().exp()?;
    let negative = (-value).exp()?;
    (positive - negative) / two()
}

pub fn cosh(value: Real) -> BlasResult<Real> {
    let positive = value.clone().exp()?;
    let negative = (-value).exp()?;
    (positive + negative) / two()
}

pub fn tanh(value: Real) -> BlasResult<Real> {
    sinh(value.clone())? / cosh(value)?
}

pub fn asin(value: Real) -> BlasResult<Real> {
    real_from_f64(f64::from(value).asin())
}

pub fn asin_with_abort(value: Real, signal: &AbortSignal) -> BlasResult<Real> {
    real_from_f64(f64::from(with_abort(value, signal)).asin())
}

pub fn acos(value: Real) -> BlasResult<Real> {
    real_from_f64(f64::from(value).acos())
}

pub fn acos_with_abort(value: Real, signal: &AbortSignal) -> BlasResult<Real> {
    real_from_f64(f64::from(with_abort(value, signal)).acos())
}

pub fn atan(value: Real) -> BlasResult<Real> {
    real_from_f64(f64::from(value).atan())
}

pub fn atan_with_abort(value: Real, signal: &AbortSignal) -> BlasResult<Real> {
    real_from_f64(f64::from(with_abort(value, signal)).atan())
}

pub fn asinh(value: Real) -> BlasResult<Real> {
    real_from_f64(f64::from(value).asinh())
}

pub fn asinh_with_abort(value: Real, signal: &AbortSignal) -> BlasResult<Real> {
    real_from_f64(f64::from(with_abort(value, signal)).asinh())
}

pub fn acosh(value: Real) -> BlasResult<Real> {
    real_from_f64(f64::from(value).acosh())
}

pub fn acosh_with_abort(value: Real, signal: &AbortSignal) -> BlasResult<Real> {
    real_from_f64(f64::from(with_abort(value, signal)).acosh())
}

pub fn atanh(value: Real) -> BlasResult<Real> {
    real_from_f64(f64::from(value).atanh())
}

pub fn atanh_with_abort(value: Real, signal: &AbortSignal) -> BlasResult<Real> {
    real_from_f64(f64::from(with_abort(value, signal)).atanh())
}
