use std::fmt;
use std::ops::{Add, Mul, Neg, Sub};

use crate::backend::{Backend, BackendScalar as BackendScalarTrait};
use crate::{
    AbortSignal, BlasResult, Problem, ScalarFacts, ScalarMagnitudeBits, ScalarSign, ZeroStatus,
};

const ROUNDING_EPSILON: f64 = f64::EPSILON;

#[derive(Clone, Debug)]
pub struct BackendScalar {
    pub(crate) value: f64,
    pub(crate) epsilon: f64,
}

/// Backend marker for approximate `f64` values with absolute error bounds.
#[derive(Clone, Debug, PartialEq)]
pub struct ApproxBackend;

impl Backend for ApproxBackend {
    type Repr = BackendScalar;
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

fn product_epsilon(left: &BackendScalar, right: &BackendScalar, product: f64) -> f64 {
    left.value.abs() * right.epsilon
        + right.value.abs() * left.epsilon
        + left.epsilon * right.epsilon
        + ROUNDING_EPSILON * product.abs()
}

impl BackendScalarTrait for BackendScalar {
    fn zero() -> Self {
        Self {
            value: 0.0,
            epsilon: 0.0,
        }
    }

    fn one() -> Self {
        Self {
            value: 1.0,
            epsilon: 0.0,
        }
    }

    fn e() -> Self {
        Self::rounded(std::f64::consts::E)
    }

    fn pi() -> Self {
        Self::rounded(std::f64::consts::PI)
    }

    fn inverse(self) -> BlasResult<Self> {
        Self::one().div(self)
    }

    #[inline]
    fn inverse_ref(&self) -> BlasResult<Self> {
        Self::div_refs(&Self::one(), self)
    }

    fn pow(self, exponent: Self) -> BlasResult<Self> {
        let lower = self.value - self.epsilon;
        let upper = self.value + self.epsilon;
        let exponent_is_known_integer = exponent.epsilon == 0.0 && exponent.value.fract() == 0.0;

        if lower < 0.0 && !exponent_is_known_integer {
            return if upper < 0.0 {
                Err(Problem::NotANumber)
            } else {
                Err(Problem::UnknownZero)
            };
        }
        if exponent.value < 0.0 {
            match self.zero_status() {
                ZeroStatus::Zero => return Err(Problem::DivideByZero),
                ZeroStatus::Unknown => return Err(Problem::UnknownZero),
                ZeroStatus::NonZero => {}
            }
        }
        let center = self.value.powf(exponent.value);
        Self::from_unary(center, self.epsilon + exponent.epsilon)
    }

    #[inline]
    fn add_refs(left: &Self, right: &Self) -> Self {
        let value = left.value + right.value;
        Self {
            value,
            epsilon: left.epsilon + right.epsilon + ROUNDING_EPSILON * value.abs(),
        }
    }

    #[inline]
    fn add_owned_ref(left: Self, right: &Self) -> Self {
        Self::add_refs(&left, right)
    }

    #[inline]
    fn add_ref_owned(left: &Self, right: Self) -> Self {
        Self::add_refs(left, &right)
    }

    #[inline]
    fn sub_refs(left: &Self, right: &Self) -> Self {
        let value = left.value - right.value;
        Self {
            value,
            epsilon: left.epsilon + right.epsilon + ROUNDING_EPSILON * value.abs(),
        }
    }

    #[inline]
    fn sub_owned_ref(left: Self, right: &Self) -> Self {
        Self::sub_refs(&left, right)
    }

    #[inline]
    fn sub_ref_owned(left: &Self, right: Self) -> Self {
        Self::sub_refs(left, &right)
    }

    #[inline]
    fn mul_refs(left: &Self, right: &Self) -> Self {
        let value = left.value * right.value;
        let epsilon = product_epsilon(left, right, value);
        Self { value, epsilon }
    }

    #[inline]
    fn mul_owned_ref(left: Self, right: &Self) -> Self {
        Self::mul_refs(&left, right)
    }

    #[inline]
    fn mul_ref_owned(left: &Self, right: Self) -> Self {
        Self::mul_refs(left, &right)
    }

    #[inline]
    fn div_refs(left: &Self, right: &Self) -> BlasResult<Self> {
        match right.zero_status() {
            ZeroStatus::Zero => return Err(Problem::DivideByZero),
            ZeroStatus::Unknown => return Err(Problem::UnknownZero),
            ZeroStatus::NonZero => {}
        }

        let center = left.value / right.value;
        let denom = right.value.abs() - right.epsilon;
        let epsilon = left.epsilon / denom + left.value.abs() * right.epsilon / (denom * denom);
        Self::new(center, epsilon + ROUNDING_EPSILON * center.abs())
    }

    #[inline]
    fn div_owned_ref(left: Self, right: &Self) -> BlasResult<Self> {
        Self::div_refs(&left, right)
    }

    #[inline]
    fn div_ref_owned(left: &Self, right: Self) -> BlasResult<Self> {
        Self::div_refs(left, &right)
    }

    #[inline]
    fn dot3(left: [&Self; 3], right: [&Self; 3]) -> Self {
        // Keep the approximate backend's dot product fully scalar and unrolled. This avoids
        // temporary arrays/iterators while still accumulating the uncertainty terms once.
        let p0 = left[0].value * right[0].value;
        let p1 = left[1].value * right[1].value;
        let p2 = left[2].value * right[2].value;
        let sum01 = p0 + p1;
        let value = sum01 + p2;
        let epsilon = product_epsilon(left[0], right[0], p0)
            + product_epsilon(left[1], right[1], p1)
            + product_epsilon(left[2], right[2], p2)
            + ROUNDING_EPSILON * (sum01.abs() + value.abs());
        Self { value, epsilon }
    }

    #[inline]
    fn dot4(left: [&Self; 4], right: [&Self; 4]) -> Self {
        // Pairwise summation mirrors the hyperreal dot4 shape and keeps the rounding
        // envelope tighter than a longer left-associated chain.
        let p0 = left[0].value * right[0].value;
        let p1 = left[1].value * right[1].value;
        let p2 = left[2].value * right[2].value;
        let p3 = left[3].value * right[3].value;
        let sum01 = p0 + p1;
        let sum23 = p2 + p3;
        let value = sum01 + sum23;
        let epsilon = product_epsilon(left[0], right[0], p0)
            + product_epsilon(left[1], right[1], p1)
            + product_epsilon(left[2], right[2], p2)
            + product_epsilon(left[3], right[3], p3)
            + ROUNDING_EPSILON * (sum01.abs() + sum23.abs() + value.abs());
        Self { value, epsilon }
    }

    fn exp(self) -> BlasResult<Self> {
        Self::from_unary(self.value.exp(), self.epsilon)
    }

    fn ln(self) -> BlasResult<Self> {
        if self.value + self.epsilon <= 0.0 {
            return Err(Problem::NotANumber);
        }
        if self.value - self.epsilon <= 0.0 {
            return Err(Problem::UnknownZero);
        }
        Self::from_unary(self.value.ln(), self.epsilon / self.value.abs())
    }

    fn log10(self) -> BlasResult<Self> {
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

    fn sqrt(self) -> BlasResult<Self> {
        let lower = self.value - self.epsilon;
        let upper = self.value + self.epsilon;
        if upper < 0.0 {
            return Err(Problem::SqrtNegative);
        }
        if lower < 0.0 {
            return Err(Problem::UnknownZero);
        }

        let lower_sqrt = lower.sqrt();
        let upper_sqrt = upper.sqrt();
        let value = (lower_sqrt + upper_sqrt) / 2.0;
        let epsilon = (upper_sqrt - lower_sqrt) / 2.0;
        Self::from_unary(value, epsilon)
    }

    fn sin(self) -> Self {
        Self::rounded_with_input(self.value.sin(), self.epsilon)
    }

    fn cos(self) -> Self {
        Self::rounded_with_input(self.value.cos(), self.epsilon)
    }

    fn tan(self) -> BlasResult<Self> {
        let cos = self.value.cos();
        if cos.abs() <= self.epsilon {
            return Err(Problem::NotANumber);
        }
        Self::from_unary(self.value.tan(), self.epsilon / (cos * cos).abs())
    }

    fn asin(self) -> BlasResult<Self> {
        let lower = self.value - self.epsilon;
        let upper = self.value + self.epsilon;
        if upper < -1.0 || lower > 1.0 {
            return Err(Problem::NotANumber);
        }
        if lower < -1.0 || upper > 1.0 {
            return Err(Problem::UnknownZero);
        }
        if self.epsilon == 0.0 {
            return Self::from_unary(self.value.asin(), 0.0);
        }
        Self::from_unary(
            self.value.asin(),
            self.epsilon / (1.0 - self.value * self.value).sqrt(),
        )
    }

    fn acos(self) -> BlasResult<Self> {
        let lower = self.value - self.epsilon;
        let upper = self.value + self.epsilon;
        if upper < -1.0 || lower > 1.0 {
            return Err(Problem::NotANumber);
        }
        if lower < -1.0 || upper > 1.0 {
            return Err(Problem::UnknownZero);
        }
        if self.epsilon == 0.0 {
            return Self::from_unary(self.value.acos(), 0.0);
        }
        Self::from_unary(
            self.value.acos(),
            self.epsilon / (1.0 - self.value * self.value).sqrt(),
        )
    }

    fn atan(self) -> BlasResult<Self> {
        Self::from_unary(
            self.value.atan(),
            self.epsilon / (1.0 + self.value * self.value),
        )
    }

    fn asinh(self) -> BlasResult<Self> {
        Self::from_unary(
            self.value.asinh(),
            self.epsilon / (1.0 + self.value * self.value).sqrt(),
        )
    }

    fn acosh(self) -> BlasResult<Self> {
        let lower = self.value - self.epsilon;
        let upper = self.value + self.epsilon;
        if upper < 1.0 {
            return Err(Problem::NotANumber);
        }
        if lower < 1.0 {
            return Err(Problem::UnknownZero);
        }
        if self.epsilon == 0.0 {
            return Self::from_unary(self.value.acosh(), 0.0);
        }
        Self::from_unary(
            self.value.acosh(),
            self.epsilon / ((self.value - 1.0).sqrt() * (self.value + 1.0).sqrt()),
        )
    }

    fn atanh(self) -> BlasResult<Self> {
        let lower = self.value - self.epsilon;
        let upper = self.value + self.epsilon;
        if lower <= -1.0 || upper >= 1.0 {
            if self.epsilon == 0.0 && self.value.abs() == 1.0 {
                return Err(Problem::Infinity);
            }
            return if upper < -1.0 || lower > 1.0 {
                Err(Problem::NotANumber)
            } else {
                Err(Problem::UnknownZero)
            };
        }
        Self::from_unary(
            self.value.atanh(),
            self.epsilon / (1.0 - self.value * self.value).abs(),
        )
    }

    fn div(self, rhs: Self) -> BlasResult<Self> {
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

    fn definitely_zero(&self) -> bool {
        self.value == 0.0 && self.epsilon == 0.0
    }

    fn zero_status(&self) -> ZeroStatus {
        if self.definitely_zero() {
            ZeroStatus::Zero
        } else if self.value.abs() > self.epsilon {
            ZeroStatus::NonZero
        } else {
            ZeroStatus::Unknown
        }
    }

    fn structural_facts(&self) -> ScalarFacts {
        let zero = self.zero_status();
        let sign = if self.definitely_zero() {
            Some(ScalarSign::Zero)
        } else if self.value - self.epsilon > 0.0 {
            Some(ScalarSign::Positive)
        } else if self.value + self.epsilon < 0.0 {
            Some(ScalarSign::Negative)
        } else {
            None
        };
        let magnitude = match zero {
            ZeroStatus::NonZero => {
                let lower = (self.value.abs() - self.epsilon).max(0.0);
                if lower > 0.0 && lower.is_finite() {
                    Some(ScalarMagnitudeBits {
                        msd: lower.log2().floor() as i32,
                        exact_msd: self.epsilon == 0.0,
                    })
                } else {
                    None
                }
            }
            ZeroStatus::Zero | ZeroStatus::Unknown => None,
        };

        ScalarFacts {
            sign,
            zero,
            exact_rational: false,
            magnitude,
        }
    }

    fn abort(&mut self, _signal: AbortSignal) {}

    fn into_f64(self) -> f64 {
        self.value
    }

    fn to_f64_approx(&self) -> Option<f64> {
        self.value.is_finite().then_some(self.value)
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

    #[inline]
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

    #[inline]
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

    #[inline]
    fn neg(self) -> Self::Output {
        Self {
            value: -self.value,
            epsilon: self.epsilon,
        }
    }
}

impl Mul for BackendScalar {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: Self) -> Self::Output {
        let value = self.value * rhs.value;
        let epsilon = product_epsilon(&self, &rhs, value);
        Self { value, epsilon }
    }
}
