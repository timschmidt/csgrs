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
    // Matrix inverse/division applies one determinant reciprocal across every
    // cofactor. Borrowing that shared factor avoids cloning exact/symbolic
    // hyperreal state for each lane; compact approximate backends intentionally
    // leave the default off because their direct owned multiply benchmarks
    // faster.
    const BORROW_SHARED_SCALE_FACTOR: bool = true;
    // Exact rational cofactors are short signed product sums. Fusing them in
    // hyperreal lets Rational share one denominator across the whole minor;
    // approximate backends keep the direct arithmetic expression instead.
    const FUSE_SIGNED_PRODUCT_SUM: bool = true;

    type Repr = BackendScalar;
}

impl BackendScalarTrait for BackendScalar {
    fn zero() -> Self {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "constructor", "zero");
        Self(hyperreal::Real::zero())
    }

    fn one() -> Self {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "constructor", "one");
        Self(hyperreal::Real::one())
    }

    fn e() -> Self {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "constructor", "e");
        Self(hyperreal::Real::e())
    }

    fn pi() -> Self {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "constructor", "pi");
        Self(hyperreal::Real::pi())
    }

    fn tau() -> Self {
        // Use hyperreal's cached internal `tau` representation instead of
        // rebuilding `2 * pi` through public scalar multiplication.
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "constructor", "tau");
        Self(hyperreal::Real::tau())
    }

    fn inverse(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "method", "inverse-owned");
        self.0.inverse().map(Self).map_err(Problem::from)
    }

    #[inline]
    fn inverse_ref(&self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "method", "inverse-ref");
        self.0.inverse_ref().map(Self).map_err(Problem::from)
    }

    fn pow(self, exponent: Self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "method", "pow");
        self.0.pow(exponent.0).map(Self).map_err(Problem::from)
    }

    #[inline]
    fn add_ref(self, rhs: &Self) -> Self {
        crate::trace_dispatch!(
            "hyperlattice_hyperreal_backend",
            "op",
            "add-owned-method-ref"
        );
        Self(&self.0 + &rhs.0)
    }

    #[inline]
    fn add_owned_ref(left: Self, right: &Self) -> Self {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "op", "add-owned-ref");
        Self(left.0 + &right.0)
    }

    #[inline]
    fn add_ref_owned(left: &Self, right: Self) -> Self {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "op", "add-ref-owned");
        Self(&left.0 + right.0)
    }

    #[inline]
    fn add_refs(left: &Self, right: &Self) -> Self {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "op", "add-ref-ref");
        Self(&left.0 + &right.0)
    }

    #[inline]
    fn sub_ref(self, rhs: &Self) -> Self {
        crate::trace_dispatch!(
            "hyperlattice_hyperreal_backend",
            "op",
            "sub-owned-method-ref"
        );
        Self(&self.0 - &rhs.0)
    }

    #[inline]
    fn sub_owned_ref(left: Self, right: &Self) -> Self {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "op", "sub-owned-ref");
        Self(left.0 - &right.0)
    }

    #[inline]
    fn sub_ref_owned(left: &Self, right: Self) -> Self {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "op", "sub-ref-owned");
        Self(&left.0 - right.0)
    }

    #[inline]
    fn sub_refs(left: &Self, right: &Self) -> Self {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "op", "sub-ref-ref");
        Self(&left.0 - &right.0)
    }

    #[inline]
    fn mul_ref(self, rhs: &Self) -> Self {
        crate::trace_dispatch!(
            "hyperlattice_hyperreal_backend",
            "op",
            "mul-owned-method-ref"
        );
        Self(&self.0 * &rhs.0)
    }

    #[inline]
    fn mul_owned_ref(left: Self, right: &Self) -> Self {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "op", "mul-owned-ref");
        Self(left.0 * &right.0)
    }

    #[inline]
    fn mul_ref_owned(left: &Self, right: Self) -> Self {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "op", "mul-ref-owned");
        Self(&left.0 * right.0)
    }

    #[inline]
    fn mul_refs(left: &Self, right: &Self) -> Self {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "op", "mul-ref-ref");
        Self(&left.0 * &right.0)
    }

    #[inline]
    fn div_ref(self, rhs: &Self) -> BlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_hyperreal_backend",
            "op",
            "div-owned-method-ref"
        );
        (&self.0 / &rhs.0).map(Self).map_err(Problem::from)
    }

    #[inline]
    fn div_owned_ref(left: Self, right: &Self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "op", "div-owned-ref");
        (left.0 / &right.0).map(Self).map_err(Problem::from)
    }

    #[inline]
    fn div_ref_owned(left: &Self, right: Self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "op", "div-ref-owned");
        (&left.0 / right.0).map(Self).map_err(Problem::from)
    }

    #[inline]
    fn div_refs(left: &Self, right: &Self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "op", "div-ref-ref");
        (&left.0 / &right.0).map(Self).map_err(Problem::from)
    }

    #[inline]
    fn dot3(left: [&Self; 3], right: [&Self; 3]) -> Self {
        // Delegate to hyperreal so exact-rational lanes can delay denominator
        // canonicalization until the final dot-product result. Non-rational
        // lanes still use the previous product/tree shape inside hyperreal.
        // 2026-05 benches: borrowed mat3 mul refs moved from roughly 4.99 us
        // to 2.29 us and vec3 dot from roughly 695 ns to 253 ns.
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "op", "dot3-specialized");
        Self(hyperreal::Real::dot3_refs(
            [&left[0].0, &left[1].0, &left[2].0],
            [&right[0].0, &right[1].0, &right[2].0],
        ))
    }

    #[inline]
    fn active_dot3(left: [&Self; 3], right: [&Self; 3]) -> Self {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "op", "active-dot3");
        Self(hyperreal::Real::active_dot3_refs(
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
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "op", "dot4-specialized");
        Self(hyperreal::Real::dot4_refs(
            [&left[0].0, &left[1].0, &left[2].0, &left[3].0],
            [&right[0].0, &right[1].0, &right[2].0, &right[3].0],
        ))
    }

    #[inline]
    fn active_dot4(left: [&Self; 4], right: [&Self; 4]) -> Self {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "op", "active-dot4");
        Self(hyperreal::Real::active_dot4_refs(
            [&left[0].0, &left[1].0, &left[2].0, &left[3].0],
            [&right[0].0, &right[1].0, &right[2].0, &right[3].0],
        ))
    }

    #[inline]
    fn linear_combination3(coeffs: [&Self; 3], values: [&Self; 3]) -> Self {
        // Route through `Real`'s fixed-arity form to let affine-structure
        // opportunities in symbolic form live longer than immediate expansion.
        crate::trace_dispatch!(
            "hyperlattice_hyperreal_backend",
            "op",
            "linear-combination3-specialized"
        );
        Self(hyperreal::Real::linear_combination3_refs(
            [&coeffs[0].0, &coeffs[1].0, &coeffs[2].0],
            [&values[0].0, &values[1].0, &values[2].0],
        ))
    }

    #[inline]
    fn active_linear_combination3(coeffs: [&Self; 3], values: [&Self; 3]) -> Self {
        crate::trace_dispatch!(
            "hyperlattice_hyperreal_backend",
            "op",
            "active-linear-combination3"
        );
        Self(hyperreal::Real::active_linear_combination3_refs(
            [&coeffs[0].0, &coeffs[1].0, &coeffs[2].0],
            [&values[0].0, &values[1].0, &values[2].0],
        ))
    }

    #[inline]
    fn linear_combination4(coeffs: [&Self; 4], values: [&Self; 4]) -> Self {
        // Same rationale as `dot4`; a dedicated 4-ary linear form can keep exact
        // rational denominator factorizations aligned with the matrix row.
        crate::trace_dispatch!(
            "hyperlattice_hyperreal_backend",
            "op",
            "linear-combination4-specialized"
        );
        Self(hyperreal::Real::linear_combination4_refs(
            [&coeffs[0].0, &coeffs[1].0, &coeffs[2].0, &coeffs[3].0],
            [&values[0].0, &values[1].0, &values[2].0, &values[3].0],
        ))
    }

    #[inline]
    fn active_linear_combination4(coeffs: [&Self; 4], values: [&Self; 4]) -> Self {
        crate::trace_dispatch!(
            "hyperlattice_hyperreal_backend",
            "op",
            "active-linear-combination4"
        );
        Self(hyperreal::Real::active_linear_combination4_refs(
            [&coeffs[0].0, &coeffs[1].0, &coeffs[2].0, &coeffs[3].0],
            [&values[0].0, &values[1].0, &values[2].0, &values[3].0],
        ))
    }

    #[inline]
    fn affine_combination4(coeffs: [&Self; 4], values: [&Self; 4], offset: &Self) -> Self {
        // Same as `affine_combination3`, extended to 4 operands for homogeneous
        // matrix-vector kernels once the translation term is split out as an
        // offset.
        let zero0 = coeffs[0].definitely_zero() || values[0].definitely_zero();
        let zero1 = coeffs[1].definitely_zero() || values[1].definitely_zero();
        let zero2 = coeffs[2].definitely_zero() || values[2].definitely_zero();
        let zero3 = coeffs[3].definitely_zero() || values[3].definitely_zero();
        if zero0 && zero1 && zero2 && zero3 {
            if offset.definitely_zero() {
                crate::trace_dispatch!(
                    "hyperlattice_hyperreal_backend",
                    "op",
                    "affine-combination4-all-zero"
                );
                return Self::zero();
            }
            crate::trace_dispatch!(
                "hyperlattice_hyperreal_backend",
                "op",
                "affine-combination4-all-zero-offset"
            );
            return offset.clone();
        }

        if offset.definitely_zero() {
            // Same zero-offset shortcut as the 3-ary helper.
            crate::trace_dispatch!(
                "hyperlattice_hyperreal_backend",
                "op",
                "affine-combination4-offset-zero"
            );
            return Self::linear_combination4(coeffs, values);
        }

        // Keep this as one linear stage plus one affine-offset addition to avoid
        // changing coefficient/value operation order.
        crate::trace_dispatch!(
            "hyperlattice_hyperreal_backend",
            "op",
            "affine-combination4-specialized"
        );
        let linear = hyperreal::Real::linear_combination4_refs(
            [&coeffs[0].0, &coeffs[1].0, &coeffs[2].0, &coeffs[3].0],
            [&values[0].0, &values[1].0, &values[2].0, &values[3].0],
        );
        Self(linear + &offset.0)
    }

    #[inline]
    fn active_signed_product_sum2<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self {
        if let Some(sum) = hyperreal::Real::exact_rational_signed_product_sum(
            positive_terms,
            terms.map(|term| [&term[0].0, &term[1].0]),
        ) {
            crate::trace_dispatch!(
                "hyperlattice_hyperreal_backend",
                "op",
                "active-signed-product-sum2-exact-rational"
            );
            return Self(sum);
        }

        crate::trace_dispatch!(
            "hyperlattice_hyperreal_backend",
            "op",
            "active-signed-product-sum2-generic"
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

    #[inline]
    fn signed_product_sum2<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self {
        // Keep exact-rational accumulation shape sparse-aware before building any
        // shared-denominator form in the backend. Dense exact-rational cases
        // then delegate to hyperreal's fused polynomial reducer, which keeps
        // determinant/cofactor terms uncanonicalized until the final sum; this
        // follows the fraction-delay idea in Bareiss elimination
        // (https://doi.org/10.2307/2004533) without changing the public
        // fixed-size cofactor formulas.
        let mut first_term: Option<([&Self; 2], bool)> = None;
        let mut second_term: Option<([&Self; 2], bool)> = None;
        let mut nonzero_count = 0usize;

        for i in 0..TERMS {
            if terms[i][0].definitely_zero() || terms[i][1].definitely_zero() {
                continue;
            }

            let term = (terms[i], positive_terms[i]);
            nonzero_count += 1;
            if nonzero_count == 1 {
                first_term = Some(term);
            } else if nonzero_count == 2 {
                second_term = Some(term);
            } else if nonzero_count > 2 {
                break;
            }
        }

        match nonzero_count {
            0 => {
                crate::trace_dispatch!(
                    "hyperlattice_hyperreal_backend",
                    "op",
                    "signed-product-sum2-all-zero"
                );
                return Self::zero();
            }
            1 => {
                let (term, positive) = first_term.expect("single non-zero term tracked");
                let product = term[0].clone().mul_ref(term[1]);
                crate::trace_dispatch!(
                    "hyperlattice_hyperreal_backend",
                    "op",
                    "signed-product-sum2-single-term"
                );
                return if positive { product } else { -product };
            }
            2 => {
                let (left_term, left_positive) = first_term.expect("first non-zero term tracked");
                let (right_term, right_positive) =
                    second_term.expect("second non-zero term tracked");
                if let Some(sum) = hyperreal::Real::exact_rational_signed_product_sum(
                    [left_positive, right_positive],
                    [
                        [&left_term[0].0, &left_term[1].0],
                        [&right_term[0].0, &right_term[1].0],
                    ],
                ) {
                    // Sparse cofactors still benefit from exact-rational
                    // denominator sharing. This is the same delayed
                    // canonicalization principle as the dense path below and
                    // Bareiss-style fraction-free elimination (Bareiss, Math.
                    // Comp. 22(103), 1968, https://doi.org/10.2307/2004533),
                    // but bounded to the two surviving products already found
                    // by the structural zero scan.
                    crate::trace_dispatch!(
                        "hyperlattice_hyperreal_backend",
                        "op",
                        "signed-product-sum2-sparse-two-exact-rational"
                    );
                    return Self(sum);
                }
                let left_product = left_term[0].clone().mul_ref(left_term[1]);
                let right_product = right_term[0].clone().mul_ref(right_term[1]);
                crate::trace_dispatch!(
                    "hyperlattice_hyperreal_backend",
                    "op",
                    "signed-product-sum2-sparse-two"
                );
                return match (left_positive, right_positive) {
                    (true, true) => left_product + right_product,
                    (true, false) => left_product - right_product,
                    (false, true) => -left_product + right_product,
                    (false, false) => -(left_product + right_product),
                };
            }
            _ => {}
        }

        if let Some(sum) = hyperreal::Real::exact_rational_signed_product_sum(
            positive_terms,
            terms.map(|term| [&term[0].0, &term[1].0]),
        ) {
            crate::trace_dispatch!(
                "hyperlattice_hyperreal_backend",
                "op",
                "signed-product-sum2-exact-rational"
            );
            return Self(sum);
        }

        crate::trace_dispatch!(
            "hyperlattice_hyperreal_backend",
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
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "method", "exp");
        self.0.exp().map(Self).map_err(Problem::from)
    }

    fn ln(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "method", "ln");
        self.0.ln().map(Self).map_err(Problem::from)
    }

    fn log10(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "method", "log10");
        self.0.log10().map(Self).map_err(Problem::from)
    }

    fn sqrt(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "method", "sqrt");
        self.0.sqrt().map(Self).map_err(Problem::from)
    }

    fn sin(self) -> Self {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "method", "sin");
        Self(self.0.sin())
    }

    fn cos(self) -> Self {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "method", "cos");
        Self(self.0.cos())
    }

    fn tan(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "method", "tan");
        self.0.tan().map(Self).map_err(Problem::from)
    }

    fn asin(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "method", "asin");
        self.0.asin().map(Self).map_err(Problem::from)
    }

    fn acos(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "method", "acos");
        self.0.acos().map(Self).map_err(Problem::from)
    }

    fn atan(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "method", "atan");
        self.0.atan().map(Self).map_err(Problem::from)
    }

    fn asinh(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "method", "asinh");
        self.0.asinh().map(Self).map_err(Problem::from)
    }

    fn acosh(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "method", "acosh");
        self.0.acosh().map(Self).map_err(Problem::from)
    }

    fn atanh(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "method", "atanh");
        self.0.atanh().map(Self).map_err(Problem::from)
    }

    fn div(self, rhs: Self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "op", "div-owned-owned");
        (self.0 / rhs.0).map(Self).map_err(Problem::from)
    }

    #[inline(always)]
    fn definitely_zero(&self) -> bool {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "query", "definitely-zero");
        self.0.definitely_zero()
    }

    #[inline(always)]
    fn definitely_one(&self) -> bool {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "query", "definitely-one");
        self.0
            .exact_rational_ref()
            .is_some_and(|exact_rational| exact_rational == &hyperreal::Rational::from(1_i8))
    }

    #[inline(always)]
    fn zero_or_one(&self) -> Option<bool> {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "query", "zero-or-one");
        // Transform kernels use this to distinguish direction vectors (`w=0`)
        // from points (`w=1`). Preserve the exact-rational fast path first:
        // moving a broader structural-zero probe ahead of it regressed symbolic
        // transform rows. Only fall back to `Real::definitely_zero` after the
        // exact identity check so scaled symbolic zeros are still classified
        // without approximation. This follows the exact-real guideline of
        // resolving algebraic/domain facts before numerical refinement; see
        // Boehm, Cartwright, Riggle, and O'Donnell, "Exact Real Arithmetic: A
        // Case Study in Higher Order Programming", LFP 1986.
        self.0
            .exact_rational_ref()
            .and_then(|exact_rational| {
                if exact_rational.is_zero() {
                    Some(false)
                } else if exact_rational.is_one() {
                    Some(true)
                } else {
                    None
                }
            })
            .or_else(|| {
                if self.0.definitely_zero() {
                    Some(false)
                } else {
                    None
                }
            })
    }

    #[inline(always)]
    fn zero_status(&self) -> ZeroStatus {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "query", "zero-status");
        map_zero(self.0.zero_status())
    }

    #[inline(always)]
    fn structural_facts(&self) -> ScalarFacts {
        crate::trace_dispatch!(
            "hyperlattice_hyperreal_backend",
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

    #[inline(always)]
    fn is_exact_rational(&self) -> bool {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "query", "exact-rational");
        self.0.exact_rational_ref().is_some()
    }

    #[inline(always)]
    fn is_exact_dyadic_rational(&self) -> bool {
        crate::trace_dispatch!(
            "hyperlattice_hyperreal_backend",
            "query",
            "exact-dyadic-rational"
        );
        self.0.is_exact_dyadic_rational()
    }

    #[inline(always)]
    fn refine_sign_until(&self, min_precision: i32) -> Option<ScalarSign> {
        crate::trace_dispatch!(
            "hyperlattice_hyperreal_backend",
            "query",
            "refine-sign-until"
        );
        self.0.refine_sign_until(min_precision).map(map_sign)
    }

    fn abort(&mut self, signal: AbortSignal) {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "query", "attach-abort");
        self.0.abort(signal);
    }

    #[inline(always)]
    fn into_f64(self) -> f64 {
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "conversion", "into-f64");
        f64::from(self.0)
    }

    #[inline(always)]
    fn to_f64_approx(&self) -> Option<f64> {
        crate::trace_dispatch!(
            "hyperlattice_hyperreal_backend",
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
            "hyperlattice_hyperreal_backend",
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
            "hyperlattice_hyperreal_backend",
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
            "hyperlattice_hyperreal_backend",
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
            "hyperlattice_hyperreal_backend",
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
        crate::trace_dispatch!("hyperlattice_hyperreal_backend", "trait_op", "neg-owned");
        Self(-self.0)
    }
}

impl Mul for BackendScalar {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: Self) -> Self::Output {
        crate::trace_dispatch!(
            "hyperlattice_hyperreal_backend",
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
            "hyperlattice_hyperreal_backend",
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
            "hyperlattice_hyperreal_backend",
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
            "hyperlattice_hyperreal_backend",
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
            "hyperlattice_hyperreal_backend",
            "trait_op",
            "div-owned-ref"
        );
        (&self.0 / &rhs.0).map(Self).map_err(Problem::from)
    }
}
