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
    const USE_BACKEND_TANH: bool = true;

    type Repr = BackendScalar;
}

impl BackendScalar {
    pub(crate) fn new(value: f64, epsilon: f64) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_approx_backend", "constructor", "new");
        if value.is_nan() || epsilon.is_nan() || epsilon < 0.0 {
            return Err(Problem::NotANumber);
        }
        if value.is_infinite() || epsilon.is_infinite() {
            return Err(Problem::Infinity);
        }
        Ok(Self { value, epsilon })
    }

    fn rounded(value: f64) -> Self {
        crate::trace_dispatch!("hyperlattice_approx_backend", "constructor", "rounded");
        Self {
            value,
            epsilon: ROUNDING_EPSILON * value.abs(),
        }
    }

    fn rounded_with_input(value: f64, input_epsilon: f64) -> Self {
        crate::trace_dispatch!(
            "hyperlattice_approx_backend",
            "constructor",
            "rounded-with-input"
        );
        Self {
            value,
            epsilon: input_epsilon + ROUNDING_EPSILON * value.abs(),
        }
    }

    fn from_unary(value: f64, propagated_epsilon: f64) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_approx_backend", "constructor", "from-unary");
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
        crate::trace_dispatch!("hyperlattice_approx_backend", "constructor", "zero");
        Self {
            value: 0.0,
            epsilon: 0.0,
        }
    }

    fn one() -> Self {
        crate::trace_dispatch!("hyperlattice_approx_backend", "constructor", "one");
        Self {
            value: 1.0,
            epsilon: 0.0,
        }
    }

    fn e() -> Self {
        crate::trace_dispatch!("hyperlattice_approx_backend", "constructor", "e");
        Self::rounded(std::f64::consts::E)
    }

    fn pi() -> Self {
        crate::trace_dispatch!("hyperlattice_approx_backend", "constructor", "pi");
        Self::rounded(std::f64::consts::PI)
    }

    fn inverse(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_approx_backend", "method", "inverse-owned");
        Self::one().div(self)
    }

    #[inline]
    fn inverse_ref(&self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_approx_backend", "method", "inverse-ref");
        Self::div_refs(&Self::one(), self)
    }

    fn pow(self, exponent: Self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_approx_backend", "method", "pow");
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
        crate::trace_dispatch!("hyperlattice_approx_backend", "op", "add-ref-ref");
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
        crate::trace_dispatch!("hyperlattice_approx_backend", "op", "sub-ref-ref");
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
        crate::trace_dispatch!("hyperlattice_approx_backend", "op", "mul-ref-ref");
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
        crate::trace_dispatch!("hyperlattice_approx_backend", "op", "div-ref-ref");
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
        crate::trace_dispatch!("hyperlattice_approx_backend", "op", "dot3-specialized");
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
        crate::trace_dispatch!("hyperlattice_approx_backend", "op", "dot4-specialized");
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

    #[inline]
    fn linear_combination3(left: [&Self; 3], right: [&Self; 3]) -> Self {
        // Keep approximate backend behavior as shared accumulation for the
        // coefficient/value matrix-vector common case while avoiding additional
        // temporary constructors.
        crate::trace_dispatch!(
            "hyperlattice_approx_backend",
            "op",
            "linear-combination3-specialized"
        );
        Self::dot3(left, right)
    }

    #[inline]
    fn linear_combination4(left: [&Self; 4], right: [&Self; 4]) -> Self {
        // Same shape as dot4; the dedicated hook keeps benchmarking and call
        // sites stable without changing numeric behavior.
        crate::trace_dispatch!(
            "hyperlattice_approx_backend",
            "op",
            "linear-combination4-specialized"
        );
        Self::dot4(left, right)
    }

    #[inline]
    fn affine_combination4(coeffs: [&Self; 4], values: [&Self; 4], offset: &Self) -> Self {
        let zero0 = coeffs[0].definitely_zero() || values[0].definitely_zero();
        let zero1 = coeffs[1].definitely_zero() || values[1].definitely_zero();
        let zero2 = coeffs[2].definitely_zero() || values[2].definitely_zero();
        let zero3 = coeffs[3].definitely_zero() || values[3].definitely_zero();
        if zero0 && zero1 && zero2 && zero3 {
            if offset.definitely_zero() {
                crate::trace_dispatch!(
                    "hyperlattice_approx_backend",
                    "op",
                    "affine-combination4-all-zero"
                );
                return Self::zero();
            }
            crate::trace_dispatch!(
                "hyperlattice_approx_backend",
                "op",
                "affine-combination4-all-zero-offset"
            );
            return offset.clone();
        }

        // Affine 4-arity is a linear 4-form plus pre-composed offset.
        if offset.definitely_zero() {
            // Preserve linear-specialized behavior for no-offset homogeneous
            // transforms where addition would only add rounding noise.
            crate::trace_dispatch!(
                "hyperlattice_approx_backend",
                "op",
                "affine-combination4-offset-zero"
            );
            return Self::dot4(coeffs, values);
        }

        // Keep the same exact shared accumulation from the linear helper and
        // only add the offset when it is definitely non-zero.
        crate::trace_dispatch!(
            "hyperlattice_approx_backend",
            "op",
            "affine-combination4-specialized"
        );
        Self::dot4(coeffs, values).add_ref(offset)
    }

    fn exp(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_approx_backend", "method", "exp");
        Self::from_unary(self.value.exp(), self.epsilon)
    }

    fn ln(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_approx_backend", "method", "ln");
        if self.value + self.epsilon <= 0.0 {
            return Err(Problem::NotANumber);
        }
        if self.value - self.epsilon <= 0.0 {
            return Err(Problem::UnknownZero);
        }
        Self::from_unary(self.value.ln(), self.epsilon / self.value.abs())
    }

    fn log10(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_approx_backend", "method", "log10");
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
        crate::trace_dispatch!("hyperlattice_approx_backend", "method", "sqrt");
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
        crate::trace_dispatch!("hyperlattice_approx_backend", "method", "sin");
        Self::rounded_with_input(self.value.sin(), self.epsilon)
    }

    fn cos(self) -> Self {
        crate::trace_dispatch!("hyperlattice_approx_backend", "method", "cos");
        Self::rounded_with_input(self.value.cos(), self.epsilon)
    }

    fn tan(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_approx_backend", "method", "tan");
        let cos = self.value.cos();
        if cos.abs() <= self.epsilon {
            return Err(Problem::NotANumber);
        }
        Self::from_unary(self.value.tan(), self.epsilon / (cos * cos).abs())
    }

    fn sinh(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_approx_backend", "method", "sinh");
        let positive = self.clone().exp()?;
        let negative = (-self).exp()?;
        (positive - negative).div(Self::from(2_i8))
    }

    fn cosh(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_approx_backend", "method", "cosh");
        let positive = self.clone().exp()?;
        let negative = (-self).exp()?;
        (positive + negative).div(Self::from(2_i8))
    }

    fn tanh(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_approx_backend", "method", "tanh");
        let positive = self.clone().exp()?;
        let negative = (-self).exp()?;
        (positive.clone() - negative.clone()).div(positive + negative)
    }

    fn asin(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_approx_backend", "method", "asin");
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
        crate::trace_dispatch!("hyperlattice_approx_backend", "method", "acos");
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
        crate::trace_dispatch!("hyperlattice_approx_backend", "method", "atan");
        Self::from_unary(
            self.value.atan(),
            self.epsilon / (1.0 + self.value * self.value),
        )
    }

    fn asinh(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_approx_backend", "method", "asinh");
        Self::from_unary(
            self.value.asinh(),
            self.epsilon / (1.0 + self.value * self.value).sqrt(),
        )
    }

    fn acosh(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_approx_backend", "method", "acosh");
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
        crate::trace_dispatch!("hyperlattice_approx_backend", "method", "atanh");
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
        crate::trace_dispatch!("hyperlattice_approx_backend", "op", "div-owned-owned");
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

    #[inline(always)]
    fn definitely_zero(&self) -> bool {
        crate::trace_dispatch!("hyperlattice_approx_backend", "query", "definitely-zero");
        self.value == 0.0 && self.epsilon == 0.0
    }

    #[inline(always)]
    fn definitely_one(&self) -> bool {
        crate::trace_dispatch!("hyperlattice_approx_backend", "query", "definitely-one");
        self.value == 1.0 && self.epsilon == 0.0
    }

    #[inline(always)]
    fn zero_or_one(&self) -> Option<bool> {
        crate::trace_dispatch!("hyperlattice_approx_backend", "query", "zero-or-one");
        if self.value == 0.0 && self.epsilon == 0.0 {
            Some(false)
        } else if self.value == 1.0 && self.epsilon == 0.0 {
            Some(true)
        } else {
            None
        }
    }

    #[inline]
    fn zero_status(&self) -> ZeroStatus {
        crate::trace_dispatch!("hyperlattice_approx_backend", "query", "zero-status");
        if self.definitely_zero() {
            ZeroStatus::Zero
        } else if self.value.abs() > self.epsilon {
            ZeroStatus::NonZero
        } else {
            ZeroStatus::Unknown
        }
    }

    #[inline]
    fn structural_facts(&self) -> ScalarFacts {
        crate::trace_dispatch!("hyperlattice_approx_backend", "query", "structural-facts");
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

    fn abort(&mut self, _signal: AbortSignal) {
        crate::trace_dispatch!("hyperlattice_approx_backend", "query", "attach-abort-noop");
    }

    #[inline(always)]
    fn into_f64(self) -> f64 {
        crate::trace_dispatch!("hyperlattice_approx_backend", "conversion", "into-f64");
        self.value
    }

    #[inline]
    fn to_f64_approx(&self) -> Option<f64> {
        crate::trace_dispatch!("hyperlattice_approx_backend", "conversion", "to-f64-approx");
        self.value.is_finite().then_some(self.value)
    }
}

impl PartialEq for BackendScalar {
    fn eq(&self, rhs: &Self) -> bool {
        // Avoid `inf - inf` in the tolerance check; it produces NaN and would
        // make equal overflowed centers compare false.
        if self.value == rhs.value {
            return true;
        }
        if !self.value.is_finite() || !rhs.value.is_finite() {
            return false;
        }
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
        crate::trace_dispatch!("hyperlattice_approx_backend", "constructor", "try-from-f32");
        Self::new(value.into(), 0.0)
    }
}

impl TryFrom<f64> for BackendScalar {
    type Error = Problem;

    fn try_from(value: f64) -> Result<Self, Self::Error> {
        crate::trace_dispatch!("hyperlattice_approx_backend", "constructor", "try-from-f64");
        Self::new(value, 0.0)
    }
}

impl Add for BackendScalar {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_approx_backend", "trait_op", "add-owned-owned");
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
        crate::trace_dispatch!("hyperlattice_approx_backend", "trait_op", "sub-owned-owned");
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
        crate::trace_dispatch!("hyperlattice_approx_backend", "trait_op", "neg-owned");
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
        crate::trace_dispatch!("hyperlattice_approx_backend", "trait_op", "mul-owned-owned");
        let value = self.value * rhs.value;
        let epsilon = product_epsilon(&self, &rhs, value);
        Self { value, epsilon }
    }
}
