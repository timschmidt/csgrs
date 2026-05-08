use std::fmt;
use std::ops::{Add, Div, Mul, Neg, Sub};

use crate::backend::{Backend, BackendScalar as BackendScalarTrait};
use crate::{
    AbortSignal, BlasResult, Problem, ScalarFacts, ScalarMagnitudeBits, ScalarSign, ZeroStatus,
};

fn map_sign(sign: hyperreal::RealSign) -> ScalarSign {
    match sign {
        hyperreal::RealSign::Negative => ScalarSign::Negative,
        hyperreal::RealSign::Zero => ScalarSign::Zero,
        hyperreal::RealSign::Positive => ScalarSign::Positive,
    }
}

fn map_zero(zero: hyperreal::ZeroKnowledge) -> ZeroStatus {
    match zero {
        hyperreal::ZeroKnowledge::Zero => ZeroStatus::Zero,
        hyperreal::ZeroKnowledge::NonZero => ZeroStatus::NonZero,
        hyperreal::ZeroKnowledge::Unknown => ZeroStatus::Unknown,
    }
}

#[derive(Clone, Debug)]
pub struct BackendScalar(pub(crate) hyperreal::Real);

/// Backend marker for exact computable reals from the `hyperreal` crate.
#[derive(Clone, Debug, PartialEq)]
pub struct HyperrealBackend;

impl Backend for HyperrealBackend {
    // Hyperreal `Real` clones can copy expression graphs. Moving owned matrix
    // entries is faster for elementwise operations despite being a little more
    // verbose than indexing from fixed arrays.
    const MOVE_ELEMENTWISE: bool = true;
    // Small integer powers appear in vector/matrix helpers; explicit low
    // exponents avoid clone-heavy generic exponentiation by squaring.
    const SPECIALIZE_SCALAR_POWI: bool = true;

    type Repr = BackendScalar;
}

impl BackendScalarTrait for BackendScalar {
    fn zero() -> Self {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "constructor", "zero");
        Self(hyperreal::Real::zero())
    }

    fn one() -> Self {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "constructor", "one");
        Self(hyperreal::Real::one())
    }

    fn e() -> Self {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "constructor", "e");
        Self(hyperreal::Real::e())
    }

    fn pi() -> Self {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "constructor", "pi");
        Self(hyperreal::Real::pi())
    }

    fn tau() -> Self {
        // Use hyperreal's cached internal `tau` representation instead of
        // rebuilding `2 * pi` through public scalar multiplication.
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "constructor", "tau");
        Self(hyperreal::Real::tau())
    }

    fn inverse(self) -> BlasResult<Self> {
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "method",
            "inverse-owned"
        );
        self.0.inverse().map(Self).map_err(Problem::from)
    }

    #[inline]
    fn inverse_ref(&self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "method", "inverse-ref");
        self.0.inverse_ref().map(Self).map_err(Problem::from)
    }

    fn pow(self, exponent: Self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "method", "pow");
        self.0.pow(exponent.0).map(Self).map_err(Problem::from)
    }

    #[inline]
    fn add_ref(self, rhs: &Self) -> Self {
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "op",
            "add-owned-method-ref"
        );
        Self(&self.0 + &rhs.0)
    }

    #[inline]
    fn add_owned_ref(left: Self, right: &Self) -> Self {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "op", "add-owned-ref");
        Self(left.0 + &right.0)
    }

    #[inline]
    fn add_ref_owned(left: &Self, right: Self) -> Self {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "op", "add-ref-owned");
        Self(&left.0 + right.0)
    }

    #[inline]
    fn add_refs(left: &Self, right: &Self) -> Self {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "op", "add-ref-ref");
        Self(&left.0 + &right.0)
    }

    #[inline]
    fn sub_ref(self, rhs: &Self) -> Self {
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "op",
            "sub-owned-method-ref"
        );
        Self(&self.0 - &rhs.0)
    }

    #[inline]
    fn sub_owned_ref(left: Self, right: &Self) -> Self {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "op", "sub-owned-ref");
        Self(left.0 - &right.0)
    }

    #[inline]
    fn sub_ref_owned(left: &Self, right: Self) -> Self {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "op", "sub-ref-owned");
        Self(&left.0 - right.0)
    }

    #[inline]
    fn sub_refs(left: &Self, right: &Self) -> Self {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "op", "sub-ref-ref");
        Self(&left.0 - &right.0)
    }

    #[inline]
    fn mul_ref(self, rhs: &Self) -> Self {
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "op",
            "mul-owned-method-ref"
        );
        Self(&self.0 * &rhs.0)
    }

    #[inline]
    fn mul_owned_ref(left: Self, right: &Self) -> Self {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "op", "mul-owned-ref");
        Self(left.0 * &right.0)
    }

    #[inline]
    fn mul_ref_owned(left: &Self, right: Self) -> Self {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "op", "mul-ref-owned");
        Self(&left.0 * right.0)
    }

    #[inline]
    fn mul_refs(left: &Self, right: &Self) -> Self {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "op", "mul-ref-ref");
        Self(&left.0 * &right.0)
    }

    #[inline]
    fn div_ref(self, rhs: &Self) -> BlasResult<Self> {
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "op",
            "div-owned-method-ref"
        );
        (&self.0 / &rhs.0).map(Self).map_err(Problem::from)
    }

    #[inline]
    fn div_owned_ref(left: Self, right: &Self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "op", "div-owned-ref");
        (left.0 / &right.0).map(Self).map_err(Problem::from)
    }

    #[inline]
    fn div_ref_owned(left: &Self, right: Self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "op", "div-ref-owned");
        (&left.0 / right.0).map(Self).map_err(Problem::from)
    }

    #[inline]
    fn div_refs(left: &Self, right: &Self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "op", "div-ref-ref");
        (&left.0 / &right.0).map(Self).map_err(Problem::from)
    }

    #[inline]
    fn dot3(left: [&Self; 3], right: [&Self; 3]) -> Self {
        // Build the three products first, then add by reference. This avoids
        // cloning intermediate hyperreal products in dense matrix multiply.
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "op", "dot3-specialized");
        let p0 = &left[0].0 * &right[0].0;
        let p1 = &left[1].0 * &right[1].0;
        let p2 = &left[2].0 * &right[2].0;
        let sum01 = &p0 + &p1;
        Self(&sum01 + &p2)
    }

    #[inline]
    fn dot4(left: [&Self; 4], right: [&Self; 4]) -> Self {
        // Pairwise summation is both a small expression tree and a benchmarked
        // win for 4-lane matrix/complex kernels.
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "op", "dot4-specialized");
        let p0 = &left[0].0 * &right[0].0;
        let p1 = &left[1].0 * &right[1].0;
        let p2 = &left[2].0 * &right[2].0;
        let p3 = &left[3].0 * &right[3].0;
        let sum01 = &p0 + &p1;
        let sum23 = &p2 + &p3;
        Self(&sum01 + &sum23)
    }

    fn exp(self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "method", "exp");
        self.0.exp().map(Self).map_err(Problem::from)
    }

    fn ln(self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "method", "ln");
        self.0.ln().map(Self).map_err(Problem::from)
    }

    fn log10(self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "method", "log10");
        self.0.log10().map(Self).map_err(Problem::from)
    }

    fn sqrt(self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "method", "sqrt");
        self.0.sqrt().map(Self).map_err(Problem::from)
    }

    fn sin(self) -> Self {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "method", "sin");
        Self(self.0.sin())
    }

    fn cos(self) -> Self {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "method", "cos");
        Self(self.0.cos())
    }

    fn tan(self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "method", "tan");
        self.0.tan().map(Self).map_err(Problem::from)
    }

    fn asin(self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "method", "asin");
        self.0.asin().map(Self).map_err(Problem::from)
    }

    fn acos(self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "method", "acos");
        self.0.acos().map(Self).map_err(Problem::from)
    }

    fn atan(self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "method", "atan");
        self.0.atan().map(Self).map_err(Problem::from)
    }

    fn asinh(self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "method", "asinh");
        self.0.asinh().map(Self).map_err(Problem::from)
    }

    fn acosh(self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "method", "acosh");
        self.0.acosh().map(Self).map_err(Problem::from)
    }

    fn atanh(self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "method", "atanh");
        self.0.atanh().map(Self).map_err(Problem::from)
    }

    fn div(self, rhs: Self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "op", "div-owned-owned");
        (self.0 / rhs.0).map(Self).map_err(Problem::from)
    }

    fn definitely_zero(&self) -> bool {
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "query",
            "definitely-zero"
        );
        self.0.definitely_zero()
    }

    fn zero_status(&self) -> ZeroStatus {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "query", "zero-status");
        map_zero(self.0.zero_status())
    }

    fn structural_facts(&self) -> ScalarFacts {
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "query",
            "structural-facts"
        );
        let facts = self.0.structural_facts();
        ScalarFacts {
            sign: facts.sign.map(map_sign),
            zero: map_zero(facts.zero),
            exact_rational: facts.exact_rational,
            magnitude: facts.magnitude.map(|m| ScalarMagnitudeBits {
                msd: m.msd,
                exact_msd: m.exact_msd,
            }),
        }
    }

    fn refine_sign_until(&self, min_precision: i32) -> Option<ScalarSign> {
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "query",
            "refine-sign-until"
        );
        self.0.refine_sign_until(min_precision).map(map_sign)
    }

    fn abort(&mut self, signal: AbortSignal) {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "query", "attach-abort");
        self.0.abort(signal);
    }

    fn into_f64(self) -> f64 {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "conversion", "into-f64");
        f64::from(self.0)
    }

    fn to_f64_approx(&self) -> Option<f64> {
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "conversion",
            "to-f64-approx"
        );
        self.0.to_f64_approx()
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

impl From<hyperreal::Real> for BackendScalar {
    fn from(value: hyperreal::Real) -> Self {
        Self(value)
    }
}

impl From<hyperreal::Rational> for BackendScalar {
    fn from(value: hyperreal::Rational) -> Self {
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
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "constructor",
            "try-from-f32"
        );
        hyperreal::Real::try_from(value)
            .map(Self)
            .map_err(Problem::from)
    }
}

impl TryFrom<f64> for BackendScalar {
    type Error = Problem;

    fn try_from(value: f64) -> Result<Self, Self::Error> {
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "constructor",
            "try-from-f64"
        );
        hyperreal::Real::try_from(value)
            .map(Self)
            .map_err(Problem::from)
    }
}

impl Add for BackendScalar {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "trait_op",
            "add-owned-owned"
        );
        Self(self.0 + rhs.0)
    }
}

impl Sub for BackendScalar {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "trait_op",
            "sub-owned-owned"
        );
        Self(self.0 - rhs.0)
    }
}

impl Neg for BackendScalar {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self::Output {
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "trait_op", "neg-owned");
        Self(-self.0)
    }
}

impl Mul for BackendScalar {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: Self) -> Self::Output {
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "trait_op",
            "mul-owned-owned"
        );
        Self(self.0 * rhs.0)
    }
}

impl Add<&BackendScalar> for BackendScalar {
    type Output = Self;

    #[inline]
    fn add(self, rhs: &BackendScalar) -> Self::Output {
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "trait_op",
            "add-owned-ref"
        );
        Self(&self.0 + &rhs.0)
    }
}

impl Sub<&BackendScalar> for BackendScalar {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: &BackendScalar) -> Self::Output {
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "trait_op",
            "sub-owned-ref"
        );
        Self(&self.0 - &rhs.0)
    }
}

impl Mul<&BackendScalar> for BackendScalar {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: &BackendScalar) -> Self::Output {
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "trait_op",
            "mul-owned-ref"
        );
        Self(&self.0 * &rhs.0)
    }
}

impl Div<&BackendScalar> for BackendScalar {
    type Output = Result<Self, Problem>;

    #[inline]
    fn div(self, rhs: &BackendScalar) -> Self::Output {
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "trait_op",
            "div-owned-ref"
        );
        (&self.0 / &rhs.0).map(Self).map_err(Problem::from)
    }
}
