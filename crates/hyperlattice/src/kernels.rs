//! Private lattice adapters to scalar arithmetic and short pairwise-product schedules.
//!
//! Hyperreal owns representation, arithmetic, and scalar facts. This module selects
//! local zero/one/two-term schedules or delegates to its dot/linear-combination
//! constructors; it does not implement a second predicate kernel. Fixed arity is not
//! a bound on coefficient growth, expression work, memory, or cancellation latency.
//!
//! The cached method names describe a borrowed reusable operand, not a lattice cache
//! lookup or a guaranteed cache hit. Active means the caller retained a term for
//! arithmetic; it does not prove that term nonzero. Known-exact/known-dyadic methods
//! require valid current scalar representation evidence and bypass local validation.
//! No method supplies a policy receipt, typed resource budget, or geometric decision.

use hyperreal::{Real, RealExactSetFacts};

/// Representation scheduling tag for recognized rational/dyadic Real values.
///
/// This describes available scalar storage, not a complete mathematical rationality
/// decision or a proof bound to a particular coordinate. Callers must keep tags aligned
/// with the values from which they were obtained.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[allow(clippy::enum_variant_names)]
pub(crate) enum ExactRationalKind {
    /// No borrowed exact-rational representation was recognized; this does not prove irrationality.
    NonRational,
    /// A rational representation was recognized with a denominator that is not a power of two.
    ExactRational,
    /// A rational representation has a power-of-two denominator, including integers and zero.
    ExactDyadicRational,
}

/// Forward the borrowed iterator to Real::exact_set_facts without adding a lattice cache, normalization, or proof validation.
///
/// Order, empty-set eligibility, and fact semantics are owned by the scalar scan.
pub(crate) fn exact_real_set_facts<'a, I>(values: I) -> RealExactSetFacts
where
    I: IntoIterator<Item = &'a Real>,
{
    Real::exact_set_facts(values)
}

/// Crate-private arithmetic adapters implemented for Real, preserving scalar authority.
///
/// Methods expose borrowed reuse and fixed-shape product schedules to lattice
/// consumers. They construct exact scalar results rather than certifying topology;
/// unchecked known-representation routes require caller-established evidence.
pub(crate) trait RealKernelExt: Sized {
    /// Consume the left Real and multiply through borrowed references, leaving the reusable factor unconsumed.
    ///
    /// Adds a lattice dispatch label, not a local cache. Underlying scalar arithmetic
    /// may reuse its own structures/caches; no independent cache isolation is promised.
    fn mul_cached(self, factor: &Self) -> Self;

    /// Consume the left Real and add the borrowed right operand through borrowed scalar arithmetic; records a lattice label but maintains no local cache.
    fn add_cached(self, rhs: &Self) -> Self;

    /// Consume the left Real and subtract the borrowed right operand through borrowed scalar arithmetic; records a lattice label but maintains no local cache.
    fn sub_cached(self, rhs: &Self) -> Self;

    /// Inspect exact_rational_ref, then test a recognized rational denominator for dyadic form.
    ///
    /// A missing rational representation returns NonRational, which is absence of this
    /// representation proof rather than proof of mathematical irrationality. No refinement
    /// or numeric approximation is initiated to resolve an unrecognized representation.
    fn exact_rational_kind(&self) -> ExactRationalKind;

    /// Delegate the ordered three-lane dot expression to Real::dot3_refs.
    ///
    /// Exact-rational aggregation, zero handling, and any retained self-dot reuse are
    /// scalar-owned. No additional lattice sparsity scan or geometric decision occurs.
    fn dot3(left: [&Self; 3], right: [&Self; 3]) -> Self;

    /// Delegate the ordered four-lane dot expression to Real::dot4_refs, leaving exact-rational aggregation and symbolic fallback to the scalar owner.
    fn dot4(left: [&Self; 4], right: [&Self; 4]) -> Self;

    /// Pass the same three references as both dot operands, preserving identity for any scalar-owned self-dot reuse; does not itself cache or guarantee reuse.
    fn dot3_same(values: [&Self; 3]) -> Self;

    /// Pass the same four references as both dot operands, preserving identity for any scalar-owned self-dot reuse; does not itself cache or guarantee reuse.
    fn dot4_same(values: [&Self; 4]) -> Self;

    /// Delegate the three products `coefficients[i]*values[i]` to Real::linear_combination3_refs.
    ///
    /// Coefficients are not normalized, constrained positive, or required to sum to one;
    /// this is a linear sum rather than a validated affine/barycentric construction.
    fn linear_combination3(coefficients: [&Self; 3], values: [&Self; 3]) -> Self;

    /// Delegate three retained coefficient/value pairs to Real::active_linear_combination3_refs without an extra lattice zero scan.
    ///
    /// Active records a scheduling choice, not a proof of nonzero factors. The scalar
    /// implementation owns exact-rational selection and symbolic expression construction.
    fn active_linear_combination3(coefficients: [&Self; 3], values: [&Self; 3]) -> Self;

    /// Delegate four coefficient/value products to Real::linear_combination4_refs without weight normalization or a lattice sparsity scan.
    fn linear_combination4(coefficients: [&Self; 4], values: [&Self; 4]) -> Self;

    /// Construct an ordered signed sum after scanning factors with definitely_zero.
    ///
    /// For each term the left factor is queried first, and a definite zero skips the
    /// right query. All terms are scanned; retained means not definitely zero, including
    /// unknown or otherwise unrecognized mathematical zeros. Zero retained terms return
    /// Real::zero; one uses borrowed multiplication and its sign; two call the active
    /// reducer with just those two terms. Three or more pass the original array, including
    /// skipped zero terms, to the active reducer. Empty input returns exact zero.
    ///
    /// This path does not consume zero_status fact packets or invoke the scalar generic
    /// signed_product_sum identity recognizers. Predicate-call and term order matter for
    /// proof/cache/trace behavior and are not interchangeable with a structural fact scan.
    fn signed_product_sum2<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self;

    /// Try the scalar exact-rational aggregate first, then accumulate products in original order.
    ///
    /// The generic route multiplies every supplied pair, uses the first signed product
    /// as its initial total, and adds/subtracts subsequent products. It performs no local
    /// zero pruning or symbolic identity search; scalar operations can still simplify.
    /// Empty input yields zero. Active does not require a proof of nonzero values; no
    /// policy receipt or checked-input result is returned.
    fn active_signed_product_sum2<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self;

    /// Delegate to Real::exact_rational_signed_product_sum_known_exact with no factor-class checks.
    ///
    /// Every factor must currently be represented as an exact rational. The scalar route
    /// reads stored rational components directly; a stale/incorrect proof can change the
    /// represented value rather than produce a typed error. The caller must re-establish
    /// or validly retain representation evidence after mutation. Zeros remain admissible.
    fn active_signed_product_sum2_known_exact_rational<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self;

    /// Delegate to Real::exact_rational_signed_product_sum_known_dyadic without local guards.
    ///
    /// Every factor must currently be an exact rational with power-of-two denominator.
    /// The scalar route assumes that contract; this adapter neither verifies it nor
    /// returns a typed mismatch. Retained evidence must still apply to the actual factors;
    /// active does not additionally require nonzero values.
    fn active_signed_product_sum2_known_dyadic<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self;
}

impl RealKernelExt for Real {
    /// Consume the left Real and multiply through borrowed references, leaving the reusable factor unconsumed.
    ///
    /// Adds a lattice dispatch label, not a local cache. Underlying scalar arithmetic
    /// may reuse its own structures/caches; no independent cache isolation is promised.
    #[inline]
    fn mul_cached(self, factor: &Self) -> Self {
        // Hot elementwise kernels often reuse one scalar factor across an
        // entire vector or matrix. Keeping the factor borrowed avoids cloning
        // hyperreal expression graphs for every lane.
        crate::trace_dispatch!("hyperlattice", "real_kernel", "mul-cached");
        &self * factor
    }

    /// Consume the left Real and add the borrowed right operand through borrowed scalar arithmetic; records a lattice label but maintains no local cache.
    #[inline]
    fn add_cached(self, rhs: &Self) -> Self {
        crate::trace_dispatch!("hyperlattice", "real_kernel", "add-cached");
        &self + rhs
    }

    /// Consume the left Real and subtract the borrowed right operand through borrowed scalar arithmetic; records a lattice label but maintains no local cache.
    #[inline]
    fn sub_cached(self, rhs: &Self) -> Self {
        crate::trace_dispatch!("hyperlattice", "real_kernel", "sub-cached");
        &self - rhs
    }

    /// Inspect exact_rational_ref, then test a recognized rational denominator for dyadic form.
    ///
    /// A missing rational representation returns NonRational, which is absence of this
    /// representation proof rather than proof of mathematical irrationality. No refinement
    /// or numeric approximation is initiated to resolve an unrecognized representation.
    #[inline]
    fn exact_rational_kind(&self) -> ExactRationalKind {
        match self.exact_rational_ref() {
            Some(rational) if rational.is_dyadic() => ExactRationalKind::ExactDyadicRational,
            Some(_) => ExactRationalKind::ExactRational,
            None => ExactRationalKind::NonRational,
        }
    }

    /// Delegate the ordered three-lane dot expression to Real::dot3_refs.
    ///
    /// Exact-rational aggregation, zero handling, and any retained self-dot reuse are
    /// scalar-owned. No additional lattice sparsity scan or geometric decision occurs.
    #[inline]
    fn dot3(left: [&Self; 3], right: [&Self; 3]) -> Self {
        // Structural-dispatch note: vector and matrix callers can carry
        // row/column sparsity, orthonormality, affine-basis, and common
        // rational-grid facts so this layer can route directly to sparse,
        // fused, or dyadic exact reducers.
        Real::dot3_refs(left, right)
    }

    /// Delegate the ordered four-lane dot expression to Real::dot4_refs, leaving exact-rational aggregation and symbolic fallback to the scalar owner.
    #[inline]
    fn dot4(left: [&Self; 4], right: [&Self; 4]) -> Self {
        Real::dot4_refs(left, right)
    }

    /// Pass the same three references as both dot operands, preserving identity for any scalar-owned self-dot reuse; does not itself cache or guarantee reuse.
    #[inline]
    fn dot3_same(values: [&Self; 3]) -> Self {
        Real::dot3_refs(values, values)
    }

    /// Pass the same four references as both dot operands, preserving identity for any scalar-owned self-dot reuse; does not itself cache or guarantee reuse.
    #[inline]
    fn dot4_same(values: [&Self; 4]) -> Self {
        Real::dot4_refs(values, values)
    }

    /// Delegate the three products `coefficients[i]*values[i]` to Real::linear_combination3_refs.
    ///
    /// Coefficients are not normalized, constrained positive, or required to sum to one;
    /// this is a linear sum rather than a validated affine/barycentric construction.
    #[inline]
    fn linear_combination3(coefficients: [&Self; 3], values: [&Self; 3]) -> Self {
        Real::linear_combination3_refs(coefficients, values)
    }

    /// Delegate three retained coefficient/value pairs to Real::active_linear_combination3_refs without an extra lattice zero scan.
    ///
    /// Active records a scheduling choice, not a proof of nonzero factors. The scalar
    /// implementation owns exact-rational selection and symbolic expression construction.
    #[inline]
    fn active_linear_combination3(coefficients: [&Self; 3], values: [&Self; 3]) -> Self {
        Real::active_linear_combination3_refs(coefficients, values)
    }

    /// Delegate four coefficient/value products to Real::linear_combination4_refs without weight normalization or a lattice sparsity scan.
    #[inline]
    fn linear_combination4(coefficients: [&Self; 4], values: [&Self; 4]) -> Self {
        Real::linear_combination4_refs(coefficients, values)
    }

    /// Construct an ordered signed sum after scanning factors with definitely_zero.
    ///
    /// For each term the left factor is queried first, and a definite zero skips the
    /// right query. All terms are scanned; retained means not definitely zero, including
    /// unknown or otherwise unrecognized mathematical zeros. Zero retained terms return
    /// Real::zero; one uses borrowed multiplication and its sign; two call the active
    /// reducer with just those two terms. Three or more pass the original array, including
    /// skipped zero terms, to the active reducer. Empty input returns exact zero.
    ///
    /// This path does not consume zero_status fact packets or invoke the scalar generic
    /// signed_product_sum identity recognizers. Predicate-call and term order matter for
    /// proof/cache/trace behavior and are not interchangeable with a structural fact scan.
    #[inline]
    fn signed_product_sum2<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self {
        // Fixed determinant and cofactor formulas are short signed sums of
        // products. Prune zero terms before the exact-rational fused path so
        // sparse cases avoid building a shared-denominator accumulator at all.
        // Dense exact-rational cases preserve delayed normalization.
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
            }
        }

        match nonzero_count {
            0 => return Real::zero(),
            1 => {
                let (term, positive) = first_term.expect("single non-zero term tracked");
                let product = term[0] * term[1];
                return if positive { product } else { -product };
            }
            2 => {
                let (left, left_positive) = first_term.expect("first non-zero term tracked");
                let (right, right_positive) = second_term.expect("second non-zero term tracked");
                return Real::active_signed_product_sum2(
                    [left_positive, right_positive],
                    [left, right],
                );
            }
            _ => {}
        }

        Real::active_signed_product_sum2(positive_terms, terms)
    }

    /// Try the scalar exact-rational aggregate first, then accumulate products in original order.
    ///
    /// The generic route multiplies every supplied pair, uses the first signed product
    /// as its initial total, and adds/subtracts subsequent products. It performs no local
    /// zero pruning or symbolic identity search; scalar operations can still simplify.
    /// Empty input yields zero. Active does not require a proof of nonzero values; no
    /// policy receipt or checked-input result is returned.
    #[inline]
    fn active_signed_product_sum2<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self {
        if let Some(sum) = Real::exact_rational_signed_product_sum(positive_terms, terms) {
            crate::trace_dispatch!(
                "hyperlattice",
                "real_kernel",
                "signed-product-sum-exact-rational"
            );
            return sum;
        }

        crate::trace_dispatch!("hyperlattice", "real_kernel", "signed-product-sum-generic");
        let mut total: Option<Real> = None;
        for i in 0..TERMS {
            let product = terms[i][0] * terms[i][1];
            total = Some(match total.take() {
                Some(total) if positive_terms[i] => total + product,
                Some(total) => total - product,
                None if positive_terms[i] => product,
                None => -product,
            });
        }
        total.unwrap_or_else(Real::zero)
    }

    /// Delegate to Real::exact_rational_signed_product_sum_known_exact with no factor-class checks.
    ///
    /// Every factor must currently be represented as an exact rational. The scalar route
    /// reads stored rational components directly; a stale/incorrect proof can change the
    /// represented value rather than produce a typed error. The caller must re-establish
    /// or validly retain representation evidence after mutation. Zeros remain admissible.
    #[inline]
    fn active_signed_product_sum2_known_exact_rational<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self {
        Real::exact_rational_signed_product_sum_known_exact(positive_terms, terms)
    }

    /// Delegate to Real::exact_rational_signed_product_sum_known_dyadic without local guards.
    ///
    /// Every factor must currently be an exact rational with power-of-two denominator.
    /// The scalar route assumes that contract; this adapter neither verifies it nor
    /// returns a typed mismatch. Retained evidence must still apply to the actual factors;
    /// active does not additionally require nonzero values.
    #[inline]
    fn active_signed_product_sum2_known_dyadic<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self {
        Real::exact_rational_signed_product_sum_known_dyadic(positive_terms, terms)
    }
}
