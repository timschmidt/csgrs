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
    // Exact rational cofactors are short signed product sums. Fusing them in
    // hyperreal lets Rational share one denominator across the whole minor;
    // approximate backends keep the direct arithmetic expression instead.
    const FUSE_SIGNED_PRODUCT_SUM: bool = true;

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
        // Delegate to hyperreal so exact-rational lanes can delay denominator
        // canonicalization until the final dot-product result. Non-rational
        // lanes still use the previous product/tree shape inside hyperreal.
        // 2026-05 benches: borrowed mat3 mul refs moved from roughly 4.99 us
        // to 2.29 us and vec3 dot from roughly 695 ns to 253 ns.
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "op", "dot3-specialized");
        Self(hyperreal::Real::dot3_refs(
            [&left[0].0, &left[1].0, &left[2].0],
            [&right[0].0, &right[1].0, &right[2].0],
        ))
    }

    #[inline]
    fn dot4(left: [&Self; 4], right: [&Self; 4]) -> Self {
        // See `dot3`; this is the hottest matrix multiply shape and benefits
        // most from hyperreal's shared-denominator exact-rational path. Keep
        // this specialization unless traces show the exact-rational constructor
        // count or benchmark time regressing beyond normal Criterion noise.
        crate::trace_dispatch!("realistic_blas_hyperreal_backend", "op", "dot4-specialized");
        Self(hyperreal::Real::dot4_refs(
            [&left[0].0, &left[1].0, &left[2].0, &left[3].0],
            [&right[0].0, &right[1].0, &right[2].0, &right[3].0],
        ))
    }

    #[inline]
    fn linear_combination3(coeffs: [&Self; 3], values: [&Self; 3]) -> Self {
        // Route through `Real`'s fixed-arity form to let affine-structure
        // opportunities in symbolic form live longer than immediate expansion.
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "op",
            "linear-combination3-specialized"
        );
        Self(hyperreal::Real::dot3_refs(
            [&coeffs[0].0, &coeffs[1].0, &coeffs[2].0],
            [&values[0].0, &values[1].0, &values[2].0],
        ))
    }

    #[inline]
    fn linear_combination4(coeffs: [&Self; 4], values: [&Self; 4]) -> Self {
        // Same rationale as `dot4`; a dedicated 4-ary linear form can keep exact
        // rational denominator factorizations aligned with the matrix row.
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "op",
            "linear-combination4-specialized"
        );
        Self(hyperreal::Real::dot4_refs(
            [&coeffs[0].0, &coeffs[1].0, &coeffs[2].0, &coeffs[3].0],
            [&values[0].0, &values[1].0, &values[2].0, &values[3].0],
        ))
    }

    #[inline]
    fn affine_combination3(coeffs: [&Self; 3], values: [&Self; 3], offset: &Self) -> Self {
        // Keep affine shape explicit so repeated matrix->vector geometry can be
        // interpreted as one offset plus shared coefficients downstream.
        // Current `hyperreal` does not expose a dedicated 3-ary affine
        // constructor, so preserve this shape by keeping linear and offset stages
        // separate.
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "op",
            "affine-combination3-specialized"
        );
        Self(Self::linear_combination3(coeffs, values).0 + &offset.0)
    }

    #[inline]
    fn affine_combination4(coeffs: [&Self; 4], values: [&Self; 4], offset: &Self) -> Self {
        // Same as `affine_combination3`, extended to 4 operands for homogeneous
        // matrix-vector kernels once the translation term is split out as an
        // offset.
        // Keep this as one linear stage plus one affine-offset addition to avoid
        // changing coefficient/value operation order.
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "op",
            "affine-combination4-specialized"
        );
        Self(Self::linear_combination4(coeffs, values).0 + &offset.0)
    }

    #[inline]
    fn signed_product_sum2<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self {
        if let Some(sum) = hyperreal::Real::exact_rational_signed_product_sum(
            positive_terms,
            terms.map(|term| [&term[0].0, &term[1].0]),
        ) {
            crate::trace_dispatch!(
                "realistic_blas_hyperreal_backend",
                "op",
                "signed-product-sum2-exact-rational"
            );
            return Self(sum);
        }

        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "op",
            "signed-product-sum2-generic"
        );
        let mut total: Option<Self> = None;
        for i in 0..TERMS {
            let product = terms[i][0].clone().mul_ref(terms[i][1]);
            total = Some(match total.take() {
                Some(total) if positive_terms[i] => total.add_ref(&product),
                Some(total) => total.sub_ref(&product),
                None if positive_terms[i] => product,
                None => -product,
            });
        }
        total.unwrap_or_else(Self::zero)
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

    fn is_exact_dyadic_rational(&self) -> bool {
        crate::trace_dispatch!(
            "realistic_blas_hyperreal_backend",
            "query",
            "exact-dyadic-rational"
        );
        self.0.is_exact_dyadic_rational()
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
