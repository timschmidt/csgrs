#[cfg(feature = "hyperreal-backend")]
mod hyperreal;

#[cfg(feature = "approx-backend")]
mod approx;

use std::fmt;
use std::ops::{Add, Mul, Neg, Sub};

use crate::{AbortSignal, BlasResult, Problem, ScalarFacts, ScalarSign, ZeroStatus};

#[cfg(feature = "approx-backend")]
pub use approx::ApproxBackend;
#[cfg(feature = "approx-backend")]
pub(super) use approx::BackendScalar as ApproxScalarRepr;
#[cfg(feature = "hyperreal-backend")]
pub use hyperreal::HyperrealBackend;

#[cfg(feature = "hyperreal-backend")]
/// Default backend used when no explicit backend type parameter is supplied.
pub type DefaultBackend = HyperrealBackend;
#[cfg(all(not(feature = "hyperreal-backend"), feature = "approx-backend"))]
/// Default backend used when only the approx backend feature is enabled.
pub type DefaultBackend = ApproxBackend;

#[cfg(not(any(feature = "hyperreal-backend", feature = "approx-backend")))]
compile_error!("enable either hyperreal-backend or approx-backend");

/// Numeric backend marker used by [`Scalar`](crate::Scalar).
///
/// Backend marker types select the concrete representation and arithmetic
/// behavior used by scalar, vector, complex, and matrix types. Backends are
/// still controlled by Cargo features; enabling a feature makes its marker
/// type available.
pub trait Backend: Clone + fmt::Debug + PartialEq + 'static {
    /// Whether owned elementwise vector and matrix operators should move
    /// entries instead of cloning from indexed arrays.
    ///
    /// This is faster for backends where cloning scalar expression graphs is
    /// expensive, but slower for compact scalar representations where indexed
    /// fixed-size arrays optimize better.
    const MOVE_ELEMENTWISE: bool = false;

    /// Whether scalar integer powers should use hand-specialized small
    /// exponents before falling back to exponentiation by squaring.
    ///
    /// This helps backends with expensive scalar clones and expression graph
    /// construction, but can be slower for compact scalar representations.
    const SPECIALIZE_SCALAR_POWI: bool = false;

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
    /// Adds a borrowed right-hand operand.
    #[inline]
    fn add_ref(self, rhs: &Self) -> Self {
        self + rhs.clone()
    }
    /// Adds an owned left-hand operand and a borrowed right-hand operand.
    #[inline]
    fn add_owned_ref(left: Self, right: &Self) -> Self {
        left.add_ref(right)
    }
    /// Adds a borrowed left-hand operand and an owned right-hand operand.
    #[inline]
    fn add_ref_owned(left: &Self, right: Self) -> Self {
        Self::add_refs(left, &right)
    }
    /// Adds two borrowed operands.
    #[inline]
    fn add_refs(left: &Self, right: &Self) -> Self {
        left.clone().add_ref(right)
    }
    /// Subtracts a borrowed right-hand operand.
    #[inline]
    fn sub_ref(self, rhs: &Self) -> Self {
        self - rhs.clone()
    }
    /// Subtracts a borrowed right-hand operand from an owned left-hand operand.
    #[inline]
    fn sub_owned_ref(left: Self, right: &Self) -> Self {
        left.sub_ref(right)
    }
    /// Subtracts an owned right-hand operand from a borrowed left-hand operand.
    #[inline]
    fn sub_ref_owned(left: &Self, right: Self) -> Self {
        Self::sub_refs(left, &right)
    }
    /// Subtracts two borrowed operands.
    #[inline]
    fn sub_refs(left: &Self, right: &Self) -> Self {
        left.clone().sub_ref(right)
    }
    /// Multiplies by a borrowed right-hand operand.
    #[inline]
    fn mul_ref(self, rhs: &Self) -> Self {
        self * rhs.clone()
    }
    /// Multiplies an owned left-hand operand by a borrowed right-hand operand.
    #[inline]
    fn mul_owned_ref(left: Self, right: &Self) -> Self {
        left.mul_ref(right)
    }
    /// Multiplies a borrowed left-hand operand by an owned right-hand operand.
    #[inline]
    fn mul_ref_owned(left: &Self, right: Self) -> Self {
        Self::mul_refs(left, &right)
    }
    /// Multiplies two borrowed operands.
    #[inline]
    fn mul_refs(left: &Self, right: &Self) -> Self {
        left.clone().mul_ref(right)
    }
    /// Divides by a borrowed right-hand operand.
    #[inline]
    fn div_ref(self, rhs: &Self) -> BlasResult<Self> {
        self.div(rhs.clone())
    }
    /// Divides an owned left-hand operand by a borrowed right-hand operand.
    #[inline]
    fn div_owned_ref(left: Self, right: &Self) -> BlasResult<Self> {
        left.div_ref(right)
    }
    /// Divides a borrowed left-hand operand by an owned right-hand operand.
    #[inline]
    fn div_ref_owned(left: &Self, right: Self) -> BlasResult<Self> {
        Self::div_refs(left, &right)
    }
    /// Divides two borrowed operands.
    #[inline]
    fn div_refs(left: &Self, right: &Self) -> BlasResult<Self> {
        left.clone().div_ref(right)
    }
    /// Returns the three-lane dot product.
    #[inline]
    fn dot3(left: [&Self; 3], right: [&Self; 3]) -> Self {
        let p0 = left[0].clone().mul_ref(right[0]);
        let p1 = left[1].clone().mul_ref(right[1]);
        let p2 = left[2].clone().mul_ref(right[2]);
        p0.add_ref(&p1).add_ref(&p2)
    }
    /// Returns the four-lane dot product.
    #[inline]
    fn dot4(left: [&Self; 4], right: [&Self; 4]) -> Self {
        let p0 = left[0].clone().mul_ref(right[0]);
        let p1 = left[1].clone().mul_ref(right[1]);
        let p2 = left[2].clone().mul_ref(right[2]);
        let p3 = left[3].clone().mul_ref(right[3]);
        p0.add_ref(&p1).add_ref(&p2.add_ref(&p3))
    }
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
    /// Returns the inverse sine.
    fn asin(self) -> BlasResult<Self>;
    /// Returns the inverse cosine.
    fn acos(self) -> BlasResult<Self>;
    /// Returns the inverse tangent.
    fn atan(self) -> BlasResult<Self>;
    /// Returns the inverse hyperbolic sine.
    fn asinh(self) -> BlasResult<Self>;
    /// Returns the inverse hyperbolic cosine.
    fn acosh(self) -> BlasResult<Self>;
    /// Returns the inverse hyperbolic tangent.
    fn atanh(self) -> BlasResult<Self>;
    /// Divides this value by another value.
    fn div(self, rhs: Self) -> BlasResult<Self>;
    /// Returns whether this value is definitely zero.
    fn definitely_zero(&self) -> bool;
    /// Classifies whether this value is zero.
    fn zero_status(&self) -> ZeroStatus;
    /// Returns conservative structural facts about this value.
    fn structural_facts(&self) -> ScalarFacts {
        let zero = self.zero_status();
        let sign = match zero {
            ZeroStatus::Zero => Some(ScalarSign::Zero),
            ZeroStatus::NonZero | ZeroStatus::Unknown => None,
        };
        ScalarFacts {
            sign,
            zero,
            exact_rational: false,
            magnitude: None,
        }
    }
    /// Tries to prove the sign without refining beyond the requested precision.
    fn refine_sign_until(&self, _min_precision: i32) -> Option<ScalarSign> {
        self.structural_facts().sign
    }
    /// Attaches a backend-specific abort signal.
    fn abort(&mut self, signal: AbortSignal);
    /// Converts this value into an `f64`.
    fn into_f64(self) -> f64;
    /// Returns a borrowed finite `f64` approximation when one is available.
    fn to_f64_approx(&self) -> Option<f64> {
        let value = self.clone().into_f64();
        value.is_finite().then_some(value)
    }
}
