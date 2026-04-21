use std::fmt;
use std::ops::{Add, Mul, Neg, Sub};

use num::bigint::Sign;

use crate::{AbortSignal, BlasResult, Problem, ZeroStatus};

#[derive(Clone, Debug)]
pub(crate) struct BackendScalar(pub(crate) realistic::Real);

impl BackendScalar {
    pub(crate) fn zero() -> Self {
        Self(realistic::Real::zero())
    }

    pub(crate) fn one() -> Self {
        Self(1.into())
    }

    pub(crate) fn e() -> Self {
        Self(realistic::Real::e())
    }

    pub(crate) fn pi() -> Self {
        Self(realistic::Real::pi())
    }

    pub(crate) fn inverse(self) -> BlasResult<Self> {
        self.0.inverse().map(Self).map_err(Problem::from)
    }

    pub(crate) fn pow(self, exponent: Self) -> BlasResult<Self> {
        self.0.pow(exponent.0).map(Self).map_err(Problem::from)
    }

    pub(crate) fn exp(self) -> BlasResult<Self> {
        self.0.exp().map(Self).map_err(Problem::from)
    }

    pub(crate) fn ln(self) -> BlasResult<Self> {
        self.0.ln().map(Self).map_err(Problem::from)
    }

    pub(crate) fn log10(self) -> BlasResult<Self> {
        self.0.log10().map(Self).map_err(Problem::from)
    }

    pub(crate) fn sqrt(self) -> BlasResult<Self> {
        self.0.sqrt().map(Self).map_err(Problem::from)
    }

    pub(crate) fn sin(self) -> Self {
        Self(self.0.sin())
    }

    pub(crate) fn cos(self) -> Self {
        Self(self.0.cos())
    }

    pub(crate) fn tan(self) -> BlasResult<Self> {
        self.0.tan().map(Self).map_err(Problem::from)
    }

    pub(crate) fn div(self, rhs: Self) -> BlasResult<Self> {
        (self.0 / rhs.0).map(Self).map_err(Problem::from)
    }

    pub(crate) fn definitely_zero(&self) -> bool {
        self.0.definitely_zero()
    }

    pub(crate) fn zero_status(&self) -> ZeroStatus {
        if self.0.definitely_zero() {
            ZeroStatus::Zero
        } else {
            match self.0.best_sign() {
                Sign::Plus | Sign::Minus => ZeroStatus::NonZero,
                Sign::NoSign => ZeroStatus::Unknown,
            }
        }
    }

    pub(crate) fn abort(&mut self, signal: AbortSignal) {
        self.0.abort(signal);
    }

    pub(crate) fn into_f64(self) -> f64 {
        f64::from(self.0)
    }
}

impl PartialEq for BackendScalar {
    fn eq(&self, rhs: &Self) -> bool {
        self.0 == rhs.0
    }
}

impl fmt::Display for BackendScalar {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if f.alternate() {
            write!(f, "{:#}", self.0)
        } else {
            write!(f, "{}", self.0)
        }
    }
}

impl From<realistic::Real> for BackendScalar {
    fn from(value: realistic::Real) -> Self {
        Self(value)
    }
}

impl From<realistic::Rational> for BackendScalar {
    fn from(value: realistic::Rational) -> Self {
        Self(value.into())
    }
}

macro_rules! impl_integer_conversion {
    ($($ty:ty),* $(,)?) => {
        $(
            impl From<$ty> for BackendScalar {
                fn from(value: $ty) -> Self {
                    Self(value.into())
                }
            }
        )*
    };
}

impl_integer_conversion!(i8, i16, i32, i64, i128, u8, u16, u32, u64, u128);

impl TryFrom<f32> for BackendScalar {
    type Error = Problem;

    fn try_from(value: f32) -> Result<Self, Self::Error> {
        realistic::Real::try_from(value)
            .map(Self)
            .map_err(Problem::from)
    }
}

impl TryFrom<f64> for BackendScalar {
    type Error = Problem;

    fn try_from(value: f64) -> Result<Self, Self::Error> {
        realistic::Real::try_from(value)
            .map(Self)
            .map_err(Problem::from)
    }
}

impl Add for BackendScalar {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Sub for BackendScalar {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

impl Neg for BackendScalar {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self(-self.0)
    }
}

impl Mul for BackendScalar {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self(self.0 * rhs.0)
    }
}
