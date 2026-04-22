#[cfg(feature = "realistic-backend")]
mod realistic;

#[cfg(feature = "approx-backend")]
mod approx;

use std::fmt;
use std::ops::{Add, Mul, Neg, Sub};

use crate::{AbortSignal, BlasResult, Problem, ZeroStatus};

#[cfg(feature = "approx-backend")]
pub use approx::ApproxBackend;
#[cfg(feature = "approx-backend")]
pub(crate) use approx::BackendScalar as ApproxScalarRepr;
#[cfg(feature = "realistic-backend")]
pub use realistic::RealisticBackend;

#[cfg(feature = "realistic-backend")]
/// Default backend used when no explicit backend type parameter is supplied.
pub type DefaultBackend = RealisticBackend;
#[cfg(all(not(feature = "realistic-backend"), feature = "approx-backend"))]
/// Default backend used when only the approx backend feature is enabled.
pub type DefaultBackend = ApproxBackend;

#[cfg(not(any(feature = "realistic-backend", feature = "approx-backend")))]
compile_error!("enable either realistic-backend or approx-backend");

/// Numeric backend marker used by [`Scalar`](crate::Scalar).
///
/// Backend marker types select the concrete representation and arithmetic
/// behavior used by scalar, vector, complex, and matrix types. Backends are
/// still controlled by Cargo features; enabling a feature makes its marker
/// type available.
pub trait Backend: Clone + fmt::Debug + PartialEq + 'static {
    /// Opaque scalar representation owned by the backend.
    type Repr: BackendScalar;
}

/// Shared scalar behavior required by crate-owned numeric backends.
///
/// This trait is primarily an implementation boundary for this crate. Users
/// normally choose one of the provided backend marker types instead of
/// implementing a custom backend.
pub trait BackendScalar:
    Clone
    + fmt::Debug
    + PartialEq
    + fmt::Display
    + Add<Output = Self>
    + Sub<Output = Self>
    + Neg<Output = Self>
    + Mul<Output = Self>
    + From<i8>
    + From<i16>
    + From<i32>
    + From<i64>
    + From<i128>
    + From<u8>
    + From<u16>
    + From<u32>
    + From<u64>
    + From<u128>
    + TryFrom<f32, Error = Problem>
    + TryFrom<f64, Error = Problem>
{
    /// Constructs the additive identity.
    fn zero() -> Self;
    /// Constructs the multiplicative identity.
    fn one() -> Self;
    /// Constructs Euler's number.
    fn e() -> Self;
    /// Constructs pi.
    fn pi() -> Self;
    /// Returns the multiplicative inverse.
    fn inverse(self) -> BlasResult<Self>;
    /// Raises this value to a scalar exponent.
    fn pow(self, exponent: Self) -> BlasResult<Self>;
    /// Returns `e` raised to this value.
    fn exp(self) -> BlasResult<Self>;
    /// Returns the natural logarithm.
    fn ln(self) -> BlasResult<Self>;
    /// Returns the base-10 logarithm.
    fn log10(self) -> BlasResult<Self>;
    /// Returns the principal square root.
    fn sqrt(self) -> BlasResult<Self>;
    /// Returns the sine.
    fn sin(self) -> Self;
    /// Returns the cosine.
    fn cos(self) -> Self;
    /// Returns the tangent.
    fn tan(self) -> BlasResult<Self>;
    /// Divides this value by another value.
    fn div(self, rhs: Self) -> BlasResult<Self>;
    /// Returns whether this value is definitely zero.
    fn definitely_zero(&self) -> bool;
    /// Classifies whether this value is zero.
    fn zero_status(&self) -> ZeroStatus;
    /// Attaches a backend-specific abort signal.
    fn abort(&mut self, signal: AbortSignal);
    /// Converts this value into an `f64`.
    fn into_f64(self) -> f64;
}
