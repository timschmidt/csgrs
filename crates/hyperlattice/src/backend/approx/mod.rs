use std::fmt;
use std::ops::{Add, Mul, Neg, Sub};

use crate::{AbortSignal, BlasResult, Problem, ZeroStatus};

const ROUNDING_EPSILON: f64 = f64::EPSILON;

#[derive(Clone, Debug)]
pub(crate) struct BackendScalar {
    pub(crate) value: f64,
    pub(crate) epsilon: f64,
}

impl BackendScalar {
    pub(crate) fn new(value: f64, epsilon: f64) -> BlasResult<Self> {
        if value.is_nan() || epsilon.is_nan() || epsilon < 0.0 {
            return Err(Problem::NotANumber);
        }
        if value.is_infinite() || epsilon.is_infinite() {
            return Err(Problem::Infinity);
        }
        Ok(Self { value, epsilon })
    }

    pub(crate) fn zero() -> Self {
        Self {
            value: 0.0,
            epsilon: 0.0,
        }
    }

    pub(crate) fn one() -> Self {
        Self {
            value: 1.0,
            epsilon: 0.0,
        }
    }

    pub(crate) fn e() -> Self {
        Self::rounded(std::f64::consts::E)
    }

    pub(crate) fn pi() -> Self {
        Self::rounded(std::f64::consts::PI)
    }

    pub(crate) fn inverse(self) -> BlasResult<Self> {
        Self::one().div(self)
    }

    pub(crate) fn pow(self, exponent: Self) -> BlasResult<Self> {
        if self.value - self.epsilon < 0.0 && exponent.epsilon > 0.0 {
            return Err(Problem::NotANumber);
        }
        let center = self.value.powf(exponent.value);
        Self::from_unary(center, self.epsilon + exponent.epsilon)
    }

    pub(crate) fn exp(self) -> BlasResult<Self> {
        Self::from_unary(self.value.exp(), self.epsilon)
    }

    pub(crate) fn ln(self) -> BlasResult<Self> {
        if self.value + self.epsilon <= 0.0 {
            return Err(Problem::NotANumber);
        }
        if self.value - self.epsilon <= 0.0 {
            return Err(Problem::UnknownZero);
        }
        Self::from_unary(self.value.ln(), self.epsilon / self.value.abs())
    }

    pub(crate) fn log10(self) -> BlasResult<Self> {
        if self.value + self.epsilon <= 0.0 {
            return Err(Problem::NotANumber);
        }
        if self.value - self.epsilon <= 0.0 {
            return Err(Problem::UnknownZero);
        }
        Self::from_unary(
            self.value.log10(),
            self.epsilon / (self.value.abs() * std::f64::consts::LN_10),
        )
    }

    pub(crate) fn sqrt(self) -> BlasResult<Self> {
        if self.value + self.epsilon < 0.0 {
            return Err(Problem::SqrtNegative);
        }
        if self.value - self.epsilon < 0.0 {
            return Err(Problem::UnknownZero);
        }
        let center = self.value.sqrt();
        let slope = if center == 0.0 {
            0.0
        } else {
            1.0 / (2.0 * center)
        };
        Self::from_unary(center, self.epsilon * slope)
    }

    pub(crate) fn sin(self) -> Self {
        Self::rounded_with_input(self.value.sin(), self.epsilon)
    }

    pub(crate) fn cos(self) -> Self {
        Self::rounded_with_input(self.value.cos(), self.epsilon)
    }

    pub(crate) fn tan(self) -> BlasResult<Self> {
        let cos = self.value.cos();
        if cos.abs() <= self.epsilon {
            return Err(Problem::NotANumber);
        }
        Self::from_unary(self.value.tan(), self.epsilon / (cos * cos).abs())
    }

    pub(crate) fn div(self, rhs: Self) -> BlasResult<Self> {
        match rhs.zero_status() {
            ZeroStatus::Zero => return Err(Problem::DivideByZero),
            ZeroStatus::Unknown => return Err(Problem::UnknownZero),
            ZeroStatus::NonZero => {}
        }

        let center = self.value / rhs.value;
        let denom = rhs.value.abs() - rhs.epsilon;
        let epsilon = self.epsilon / denom + self.value.abs() * rhs.epsilon / (denom * denom);
        Self::new(center, epsilon + ROUNDING_EPSILON * center.abs())
    }

    pub(crate) fn definitely_zero(&self) -> bool {
        self.value == 0.0 && self.epsilon == 0.0
    }

    pub(crate) fn zero_status(&self) -> ZeroStatus {
        if self.definitely_zero() {
            ZeroStatus::Zero
        } else if self.value.abs() > self.epsilon {
            ZeroStatus::NonZero
        } else {
            ZeroStatus::Unknown
        }
    }

    pub(crate) fn abort(&mut self, _signal: AbortSignal) {}

    pub(crate) fn into_f64(self) -> f64 {
        self.value
    }

    fn rounded(value: f64) -> Self {
        Self {
            value,
            epsilon: ROUNDING_EPSILON * value.abs(),
        }
    }

    fn rounded_with_input(value: f64, input_epsilon: f64) -> Self {
        Self {
            value,
            epsilon: input_epsilon + ROUNDING_EPSILON * value.abs(),
        }
    }

    fn from_unary(value: f64, propagated_epsilon: f64) -> BlasResult<Self> {
        Self::new(
            value,
            propagated_epsilon.abs() + ROUNDING_EPSILON * value.abs(),
        )
    }
}

impl PartialEq for BackendScalar {
    fn eq(&self, rhs: &Self) -> bool {
        (self.value - rhs.value).abs() <= self.epsilon + rhs.epsilon
    }
}

impl fmt::Display for BackendScalar {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.value)
    }
}

impl From<realistic::Real> for BackendScalar {
    fn from(value: realistic::Real) -> Self {
        Self::rounded(f64::from(value))
    }
}

impl From<realistic::Rational> for BackendScalar {
    fn from(value: realistic::Rational) -> Self {
        Self::from(realistic::Real::from(value))
    }
}

macro_rules! impl_integer_conversion {
    ($($ty:ty),* $(,)?) => {
        $(
            impl From<$ty> for BackendScalar {
                fn from(value: $ty) -> Self {
                    Self {
                        value: value as f64,
                        epsilon: 0.0,
                    }
                }
            }
        )*
    };
}

impl_integer_conversion!(i8, i16, i32, i64, i128, u8, u16, u32, u64, u128);

impl TryFrom<f32> for BackendScalar {
    type Error = Problem;

    fn try_from(value: f32) -> Result<Self, Self::Error> {
        Self::new(value.into(), 0.0)
    }
}

impl TryFrom<f64> for BackendScalar {
    type Error = Problem;

    fn try_from(value: f64) -> Result<Self, Self::Error> {
        Self::new(value, 0.0)
    }
}

impl Add for BackendScalar {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        let value = self.value + rhs.value;
        Self {
            value,
            epsilon: self.epsilon + rhs.epsilon + ROUNDING_EPSILON * value.abs(),
        }
    }
}

impl Sub for BackendScalar {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        let value = self.value - rhs.value;
        Self {
            value,
            epsilon: self.epsilon + rhs.epsilon + ROUNDING_EPSILON * value.abs(),
        }
    }
}

impl Neg for BackendScalar {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self {
            value: -self.value,
            epsilon: self.epsilon,
        }
    }
}

impl Mul for BackendScalar {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        let value = self.value * rhs.value;
        let epsilon = self.value.abs() * rhs.epsilon
            + rhs.value.abs() * self.epsilon
            + self.epsilon * rhs.epsilon
            + ROUNDING_EPSILON * value.abs();
        Self { value, epsilon }
    }
}
