//! Scalar-centered linear algebra primitives with selectable numeric backends.
//!
//! `realistic_blas` exposes a crate-owned [`Scalar`] type, complex numbers,
//! 3D/4D vectors, and 3x3/4x4 matrices. These types are generic over a backend
//! marker and default to [`DefaultBackend`]. The default `realistic-backend`
//! stores scalars as `realistic::Real` values and re-exports `Real` and
//! `Rational` for explicit interop. The `approx-backend` stores an `f64` center
//! value plus an absolute `f64` error bound, which lets tests and callers
//! exercise unknown-zero paths without depending on computable-real evaluation.
//!
//! Backend features gate backend availability. The default feature set enables
//! `realistic-backend`; use `default-features = false, features =
//! ["approx-backend"]` to make the approximate backend the default, or enable
//! both features and use [`Scalar`] with [`RealisticBackend`] and
//! [`ApproxBackend`] explicitly.
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

mod backend;

#[cfg(feature = "approx-backend")]
pub use backend::ApproxBackend;
pub use backend::DefaultBackend;
#[cfg(feature = "realistic-backend")]
pub use backend::RealisticBackend;
pub use backend::{Backend, BackendScalar};

/// Crate-owned scalar value used throughout the public API.
///
/// The backend type parameter selects the representation and arithmetic
/// behavior. The default backend is [`DefaultBackend`], which resolves to the
/// realistic backend when the default feature set is enabled.
pub struct Scalar<B: Backend = DefaultBackend>(B::Repr);

impl<B: Backend> Clone for Scalar<B> {
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl<B: Backend> fmt::Debug for Scalar<B> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

impl<B: Backend> PartialEq for Scalar<B> {
    fn eq(&self, rhs: &Self) -> bool {
        self.0 == rhs.0
    }
}

#[cfg(feature = "realistic-backend")]
impl PartialEq<Rational> for Scalar<RealisticBackend> {
    fn eq(&self, rhs: &Rational) -> bool {
        self == &Self::from(rhs.clone())
    }
}

#[cfg(feature = "realistic-backend")]
impl PartialEq<Scalar<RealisticBackend>> for Rational {
    fn eq(&self, rhs: &Scalar<RealisticBackend>) -> bool {
        rhs == self
    }
}

impl<B: Backend> Scalar<B> {
    /// Returns the additive identity.
    pub fn zero() -> Self {
        Self(B::Repr::zero())
    }

    /// Returns the multiplicative identity.
    pub fn one() -> Self {
        Self(B::Repr::one())
    }

    /// Returns Euler's number.
    pub fn e() -> Self {
        Self(B::Repr::e())
    }

    /// Returns pi.
    pub fn pi() -> Self {
        Self(B::Repr::pi())
    }

    /// Returns the multiplicative inverse of this scalar.
    pub fn inverse(self) -> BlasResult<Self> {
        self.0.inverse().map(Self)
    }

    #[inline]
    pub(crate) fn mul_cached(self, factor: &Self) -> Self {
        Self(self.0.mul_ref(&factor.0))
    }

    #[inline]
    pub(crate) fn add_cached(self, rhs: &Self) -> Self {
        Self(self.0.add_ref(&rhs.0))
    }

    #[inline]
    pub(crate) fn sub_cached(self, rhs: &Self) -> Self {
        Self(self.0.sub_ref(&rhs.0))
    }

    #[inline]
    pub(crate) fn dot3(left: [&Self; 3], right: [&Self; 3]) -> Self {
        Self(B::Repr::dot3(
            [&left[0].0, &left[1].0, &left[2].0],
            [&right[0].0, &right[1].0, &right[2].0],
        ))
    }

    #[inline]
    pub(crate) fn dot4(left: [&Self; 4], right: [&Self; 4]) -> Self {
        Self(B::Repr::dot4(
            [&left[0].0, &left[1].0, &left[2].0, &left[3].0],
            [&right[0].0, &right[1].0, &right[2].0, &right[3].0],
        ))
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

#[cfg(feature = "realistic-backend")]
impl Scalar<RealisticBackend> {
    /// Constructs a scalar from a realistic rational value.
    pub fn new(rational: Rational) -> Self {
        rational.into()
    }
}

#[cfg(feature = "approx-backend")]
impl Scalar<ApproxBackend> {
    /// Constructs an approximate scalar from a center value and error bound.
    ///
    /// `epsilon` must be finite and non-negative and is stored as the scalar's
    /// absolute error bound.
    pub fn approx(value: f64, epsilon: f64) -> BlasResult<Self> {
        crate::backend::ApproxScalarRepr::new(value, epsilon).map(Self)
    }
}

#[cfg(feature = "realistic-backend")]
impl Scalar<RealisticBackend> {
    /// Constructs a realistic scalar from an approximate center value.
    ///
    /// The `epsilon` argument is accepted for API compatibility and ignored
    /// because `Real` values do not store an interval error bound.
    pub fn approx(value: f64, epsilon: f64) -> BlasResult<Self> {
        let _ = epsilon;
        Self::try_from(value)
    }
}

impl<B: Backend> fmt::Display for Scalar<B> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

#[cfg(feature = "realistic-backend")]
impl From<Real> for Scalar<RealisticBackend> {
    fn from(value: Real) -> Self {
        Self(value.into())
    }
}

#[cfg(feature = "realistic-backend")]
impl From<Rational> for Scalar<RealisticBackend> {
    fn from(value: Rational) -> Self {
        Self(value.into())
    }
}

macro_rules! impl_integer_conversion {
    ($($ty:ty),* $(,)?) => {
        $(
            impl<B: Backend> From<$ty> for Scalar<B> {
                fn from(value: $ty) -> Self {
                    Self(value.into())
                }
            }
        )*
    };
}

impl_integer_conversion!(i8, i16, i32, i64, i128, u8, u16, u32, u64, u128);

impl<B: Backend> TryFrom<f32> for Scalar<B> {
    type Error = Problem;

    fn try_from(value: f32) -> Result<Self, Self::Error> {
        B::Repr::try_from(value).map(Self)
    }
}

impl<B: Backend> TryFrom<f64> for Scalar<B> {
    type Error = Problem;

    fn try_from(value: f64) -> Result<Self, Self::Error> {
        B::Repr::try_from(value).map(Self)
    }
}

impl<B: Backend> From<Scalar<B>> for f64 {
    fn from(value: Scalar<B>) -> Self {
        value.0.into_f64()
    }
}

impl<B: Backend> Add for Scalar<B> {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl<B: Backend> Sub for Scalar<B> {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

impl<B: Backend> Neg for Scalar<B> {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self::Output {
        Self(-self.0)
    }
}

impl<B: Backend> Mul for Scalar<B> {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: Self) -> Self::Output {
        Self(self.0 * rhs.0)
    }
}

impl<B: Backend> Div for Scalar<B> {
    type Output = BlasResult<Self>;

    #[inline]
    fn div(self, rhs: Self) -> Self::Output {
        self.0.div(rhs.0).map(Self)
    }
}

impl<B: Backend> Add<&Scalar<B>> for Scalar<B> {
    type Output = Self;

    #[inline]
    fn add(self, rhs: &Scalar<B>) -> Self::Output {
        Self(self.0.add_ref(&rhs.0))
    }
}

impl<B: Backend> Add<Scalar<B>> for &Scalar<B> {
    type Output = Scalar<B>;

    #[inline]
    fn add(self, rhs: Scalar<B>) -> Self::Output {
        rhs + self
    }
}

impl<B: Backend> Add<&Scalar<B>> for &Scalar<B> {
    type Output = Scalar<B>;

    #[inline]
    fn add(self, rhs: &Scalar<B>) -> Self::Output {
        self.clone() + rhs
    }
}

impl<B: Backend> Sub<&Scalar<B>> for Scalar<B> {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: &Scalar<B>) -> Self::Output {
        Self(self.0.sub_ref(&rhs.0))
    }
}

impl<B: Backend> Sub<Scalar<B>> for &Scalar<B> {
    type Output = Scalar<B>;

    #[inline]
    fn sub(self, rhs: Scalar<B>) -> Self::Output {
        self.clone() - rhs
    }
}

impl<B: Backend> Sub<&Scalar<B>> for &Scalar<B> {
    type Output = Scalar<B>;

    #[inline]
    fn sub(self, rhs: &Scalar<B>) -> Self::Output {
        self.clone() - rhs
    }
}

impl<B: Backend> Mul<&Scalar<B>> for Scalar<B> {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: &Scalar<B>) -> Self::Output {
        Self(self.0.mul_ref(&rhs.0))
    }
}

impl<B: Backend> Mul<Scalar<B>> for &Scalar<B> {
    type Output = Scalar<B>;

    #[inline]
    fn mul(self, rhs: Scalar<B>) -> Self::Output {
        rhs * self
    }
}

impl<B: Backend> Mul<&Scalar<B>> for &Scalar<B> {
    type Output = Scalar<B>;

    #[inline]
    fn mul(self, rhs: &Scalar<B>) -> Self::Output {
        self.clone() * rhs
    }
}

impl<B: Backend> Div<&Scalar<B>> for Scalar<B> {
    type Output = BlasResult<Self>;

    #[inline]
    fn div(self, rhs: &Scalar<B>) -> Self::Output {
        self.0.div_ref(&rhs.0).map(Self)
    }
}

impl<B: Backend> Div<Scalar<B>> for &Scalar<B> {
    type Output = BlasResult<Scalar<B>>;

    #[inline]
    fn div(self, rhs: Scalar<B>) -> Self::Output {
        self.clone() / rhs
    }
}

impl<B: Backend> Div<&Scalar<B>> for &Scalar<B> {
    type Output = BlasResult<Scalar<B>>;

    #[inline]
    fn div(self, rhs: &Scalar<B>) -> Self::Output {
        self.clone() / rhs
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
