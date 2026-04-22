//! Scalar-centered linear algebra primitives with selectable numeric backends.
//!
//! `realistic_blas` exposes a crate-owned [`Scalar`] type, complex numbers,
//! 3D/4D vectors, and 3x3/4x4 matrices. The default `realistic-backend` stores
//! scalars as `realistic::Real` values and re-exports `Real` and `Rational` for
//! explicit interop. The `approx-backend` stores an `f64` center value plus an
//! absolute `f64` error bound, which lets tests and callers exercise
//! unknown-zero paths without depending on computable-real evaluation.
//!
//! Backend features are mutually exclusive. The default feature set enables
//! `realistic-backend`; use `default-features = false, features =
//! ["approx-backend"]` to select the approximate backend.
//!
//! Most arithmetic that can fail returns [`BlasResult`]. Checked APIs use
//! [`ZeroStatus`] and reject both definite zero and unknown-zero divisors,
//! returning [`Problem::UnknownZero`] for the latter.
//!
//! # Examples
//!
//! ```
//! use realistic_blas::{Matrix3, Scalar, Vector3, sqrt};
//!
//! fn s(value: i32) -> Scalar {
//!     value.into()
//! }
//!
//! let v = Vector3::new([s(3), s(4), s(0)]);
//! assert_eq!(v.dot(&v), s(25));
//! assert_eq!(sqrt(v.dot(&v)).unwrap(), s(5));
//!
//! let identity = Matrix3::identity();
//! assert_eq!(identity * v.clone(), v);
//! ```

#![warn(missing_docs)]

use std::error::Error;
use std::fmt;
use std::ops::{Add, Div, Mul, Neg, Sub};
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

#[cfg(feature = "realistic-backend")]
pub use realistic::{Rational, Real};

/// Shared cancellation signal used by abort-aware APIs.
///
/// With the realistic backend, the signal is attached to cloned `Real` values
/// before operations that may evaluate unknown computable reals. The approx
/// backend accepts the same API as a no-op.
pub type AbortSignal = Arc<AtomicBool>;

/// Error type returned by fallible scalar, complex, vector, and matrix APIs.
///
/// Most variants mirror errors from `realistic::Problem`; `UnknownZero` is
/// crate-owned and indicates that a checked operation could not prove a divisor
/// or pivot was non-zero.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Problem {
    /// Parsing a numeric value failed.
    ParseError,
    /// A square root was requested for a definitely negative value.
    SqrtNegative,
    /// Division by a definitely zero value was requested.
    DivideByZero,
    /// A requested value could not be found.
    NotFound,
    /// A numeric operation did not receive enough parameters.
    InsufficientParameters,
    /// A conversion or operation produced a NaN.
    NotANumber,
    /// A conversion or operation produced infinity.
    Infinity,
    /// Fraction construction failed.
    BadFraction,
    /// Decimal construction failed.
    BadDecimal,
    /// Integer construction or conversion failed.
    BadInteger,
    /// A conversion was outside the supported numeric range.
    OutOfRange,
    /// An integer-only operation received a non-integer value.
    NotAnInteger,
    /// Evaluation exhausted the available precision or work budget.
    Exhausted,
    /// A checked operation could not determine whether a value was zero.
    UnknownZero,
}

#[cfg(feature = "realistic-backend")]
impl From<realistic::Problem> for Problem {
    fn from(problem: realistic::Problem) -> Self {
        match problem {
            realistic::Problem::ParseError => Self::ParseError,
            realistic::Problem::SqrtNegative => Self::SqrtNegative,
            realistic::Problem::DivideByZero => Self::DivideByZero,
            realistic::Problem::NotFound => Self::NotFound,
            realistic::Problem::InsufficientParameters => Self::InsufficientParameters,
            realistic::Problem::NotANumber => Self::NotANumber,
            realistic::Problem::Infinity => Self::Infinity,
            realistic::Problem::BadFraction => Self::BadFraction,
            realistic::Problem::BadDecimal => Self::BadDecimal,
            realistic::Problem::BadInteger => Self::BadInteger,
            realistic::Problem::OutOfRange => Self::OutOfRange,
            realistic::Problem::NotAnInteger => Self::NotAnInteger,
            realistic::Problem::Exhausted => Self::Exhausted,
            _ => Self::Exhausted,
        }
    }
}

impl fmt::Display for Problem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(self, f)
    }
}

impl Error for Problem {}

/// Result type used by fallible operations in this crate.
pub type BlasResult<T> = Result<T, Problem>;

/// Result type used by APIs that reject unknown-zero conditions.
pub type CheckedBlasResult<T> = BlasResult<T>;

/// Backwards-compatible alias for [`Problem`].
pub type BlasProblem = Problem;

mod backend;

/// Crate-owned scalar value used throughout the public API.
///
/// `Scalar` hides the selected backend while preserving a shared arithmetic
/// surface. The default realistic backend wraps `realistic::Real`; the approx
/// backend stores a center value plus an absolute error bound.
#[derive(Clone, Debug, PartialEq)]
pub struct Scalar(crate::backend::BackendScalar);

#[cfg(feature = "realistic-backend")]
impl PartialEq<Rational> for Scalar {
    fn eq(&self, rhs: &Rational) -> bool {
        self == &Self::from(rhs.clone())
    }
}

#[cfg(feature = "realistic-backend")]
impl PartialEq<Scalar> for Rational {
    fn eq(&self, rhs: &Scalar) -> bool {
        rhs == self
    }
}

impl Scalar {
    /// Constructs a scalar from a realistic rational value.
    ///
    /// This constructor is only available with the `realistic-backend` feature.
    #[cfg(feature = "realistic-backend")]
    pub fn new(rational: Rational) -> Self {
        rational.into()
    }

    /// Constructs a scalar from an approximate center value and error bound.
    ///
    /// With the approx backend, `epsilon` must be finite and non-negative and
    /// is stored as the scalar's absolute error bound. With the realistic
    /// backend, `epsilon` is ignored and `value` is converted to `Real`.
    pub fn approx(value: f64, epsilon: f64) -> BlasResult<Self> {
        #[cfg(all(not(feature = "realistic-backend"), feature = "approx-backend"))]
        {
            crate::backend::BackendScalar::new(value, epsilon).map(Self)
        }

        #[cfg(feature = "realistic-backend")]
        {
            let _ = epsilon;
            Self::try_from(value)
        }
    }

    /// Returns the additive identity.
    pub fn zero() -> Self {
        Self(crate::backend::BackendScalar::zero())
    }

    /// Returns the multiplicative identity.
    pub fn one() -> Self {
        Self(crate::backend::BackendScalar::one())
    }

    /// Returns Euler's number.
    pub fn e() -> Self {
        Self(crate::backend::BackendScalar::e())
    }

    /// Returns pi.
    pub fn pi() -> Self {
        Self(crate::backend::BackendScalar::pi())
    }

    /// Returns the multiplicative inverse of this scalar.
    pub fn inverse(self) -> BlasResult<Self> {
        self.0.inverse().map(Self)
    }

    pub(crate) fn mul_cached(self, factor: &Self) -> Self {
        self * factor.clone()
    }

    /// Raises this scalar to a scalar exponent.
    pub fn pow(self, exponent: Self) -> BlasResult<Self> {
        self.0.pow(exponent.0).map(Self)
    }

    /// Returns `e` raised to this scalar.
    pub fn exp(self) -> BlasResult<Self> {
        self.0.exp().map(Self)
    }

    /// Returns the natural logarithm.
    pub fn ln(self) -> BlasResult<Self> {
        self.0.ln().map(Self)
    }

    /// Returns the base-10 logarithm.
    pub fn log10(self) -> BlasResult<Self> {
        self.0.log10().map(Self)
    }

    /// Returns the principal square root.
    pub fn sqrt(self) -> BlasResult<Self> {
        self.0.sqrt().map(Self)
    }

    /// Returns the sine.
    pub fn sin(self) -> Self {
        Self(self.0.sin())
    }

    /// Returns the cosine.
    pub fn cos(self) -> Self {
        Self(self.0.cos())
    }

    /// Returns the tangent.
    pub fn tan(self) -> BlasResult<Self> {
        self.0.tan().map(Self)
    }

    /// Returns whether this scalar is definitely zero.
    ///
    /// This is an optimistic predicate. Use [`Scalar::zero_status`] or
    /// [`zero_status`] when unknown-zero conditions must be distinguished.
    pub fn definitely_zero(&self) -> bool {
        self.0.definitely_zero()
    }

    /// Classifies this scalar as zero, non-zero, or unknown.
    pub fn zero_status(&self) -> ZeroStatus {
        self.0.zero_status()
    }

    /// Attaches a cancellation signal to this scalar where the backend supports it.
    ///
    /// This affects realistic backend evaluation. It is a no-op on the approx
    /// backend.
    pub fn abort(&mut self, signal: AbortSignal) {
        self.0.abort(signal);
    }
}

impl fmt::Display for Scalar {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

#[cfg(feature = "realistic-backend")]
impl From<Real> for Scalar {
    fn from(value: Real) -> Self {
        Self(value.into())
    }
}

#[cfg(feature = "realistic-backend")]
impl From<Rational> for Scalar {
    fn from(value: Rational) -> Self {
        Self(value.into())
    }
}

macro_rules! impl_integer_conversion {
    ($($ty:ty),* $(,)?) => {
        $(
            impl From<$ty> for Scalar {
                fn from(value: $ty) -> Self {
                    Self(value.into())
                }
            }
        )*
    };
}

impl_integer_conversion!(i8, i16, i32, i64, i128, u8, u16, u32, u64, u128);

impl TryFrom<f32> for Scalar {
    type Error = Problem;

    fn try_from(value: f32) -> Result<Self, Self::Error> {
        crate::backend::BackendScalar::try_from(value).map(Self)
    }
}

impl TryFrom<f64> for Scalar {
    type Error = Problem;

    fn try_from(value: f64) -> Result<Self, Self::Error> {
        crate::backend::BackendScalar::try_from(value).map(Self)
    }
}

impl From<Scalar> for f64 {
    fn from(value: Scalar) -> Self {
        value.0.into_f64()
    }
}

impl Add for Scalar {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Sub for Scalar {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

impl Neg for Scalar {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self(-self.0)
    }
}

impl Mul for Scalar {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self(self.0 * rhs.0)
    }
}

impl Div for Scalar {
    type Output = BlasResult<Self>;

    fn div(self, rhs: Self) -> Self::Output {
        self.0.div(rhs.0).map(Self)
    }
}

mod complex;
mod matrix;
mod scalar;
mod vector;

pub use complex::Complex;
pub use matrix::{Matrix3, Matrix4};
pub use scalar::*;
pub use vector::{Vector3, Vector4};
