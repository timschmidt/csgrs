//! Fixed-arity scalar kernels for [`Real`](hyperreal::Real).
//!
//! These helpers are intentionally crate-private. `hyperreal::Real` owns the
//! arithmetic representation; `hyperlattice` owns the small-vector and
//! small-matrix scheduling decisions that choose when to use sparse or fused
//! product sums.

use hyperreal::Real;

/// Exact-rational structure shared by a fixed set of [`Real`] values.
///
/// This is a compatibility alias for the scalar-layer fact type. Keeping the
/// public `hyperlattice` name avoids churn for vector/matrix callers while
/// making `hyperreal` the semantic owner of rational representation facts.
pub type ExactRealSetFacts = hyperreal::RealExactSetFacts;

/// Already-known exact-rational representation class for a real value.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactRationalKind {
    /// The scalar is not structurally known to be exact rational.
    NonRational,
    /// The scalar is exact rational, but not known to be dyadic.
    ExactRational,
    /// The scalar is exact rational with a power-of-two denominator.
    ExactDyadicRational,
}

pub(crate) fn exact_real_set_facts<'a, I>(values: I) -> ExactRealSetFacts
where
    I: IntoIterator<Item = &'a Real>,
{
    Real::exact_set_facts(values)
}

/// Crate-private lattice kernels layered on top of [`Real`].
pub(crate) trait RealKernelExt: Sized {
    /// Multiply an owned value by a shared borrowed factor.
    fn mul_cached(self, factor: &Self) -> Self;

    /// Add a shared borrowed value to an owned value.
    fn add_cached(self, rhs: &Self) -> Self;

    /// Subtract a shared borrowed value from an owned value.
    fn sub_cached(self, rhs: &Self) -> Self;

    /// Classify the exact-rational representation currently carried by the value.
    fn exact_rational_kind(&self) -> ExactRationalKind;

    /// Three-lane dot product with lattice-level sparse pruning.
    fn dot3(left: [&Self; 3], right: [&Self; 3]) -> Self;

    /// Four-lane dot product with lattice-level sparse pruning.
    fn dot4(left: [&Self; 4], right: [&Self; 4]) -> Self;

    /// Three-lane self dot product.
    fn dot3_same(values: [&Self; 3]) -> Self;

    /// Four-lane self dot product.
    fn dot4_same(values: [&Self; 4]) -> Self;

    /// Three-term linear combination.
    fn linear_combination3(coefficients: [&Self; 3], values: [&Self; 3]) -> Self;

    /// Three-term linear combination whose lanes were already classified active.
    fn active_linear_combination3(coefficients: [&Self; 3], values: [&Self; 3]) -> Self;

    /// Four-term linear combination.
    fn linear_combination4(coefficients: [&Self; 4], values: [&Self; 4]) -> Self;

    /// Signed sum of pairwise products with zero-term pruning.
    fn signed_product_sum2<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self;

    /// Signed sum of pairwise products whose terms are already considered active.
    fn active_signed_product_sum2<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self;

    /// Signed sum of exact-rational pairwise products after the caller cached that fact.
    fn active_signed_product_sum2_known_exact_rational<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self;
}

impl RealKernelExt for Real {
    #[inline]
    fn mul_cached(self, factor: &Self) -> Self {
        // Hot elementwise kernels often reuse one scalar factor across an
        // entire vector or matrix. Keeping the factor borrowed avoids cloning
        // hyperreal expression graphs for every lane.
        crate::trace_dispatch!("hyperlattice", "real_kernel", "mul-cached");
        &self * factor
    }

    #[inline]
    fn add_cached(self, rhs: &Self) -> Self {
        crate::trace_dispatch!("hyperlattice", "real_kernel", "add-cached");
        &self + rhs
    }

    #[inline]
    fn sub_cached(self, rhs: &Self) -> Self {
        crate::trace_dispatch!("hyperlattice", "real_kernel", "sub-cached");
        &self - rhs
    }

    #[inline]
    fn exact_rational_kind(&self) -> ExactRationalKind {
        match self.exact_rational_ref() {
            Some(rational) if rational.is_dyadic() => ExactRationalKind::ExactDyadicRational,
            Some(_) => ExactRationalKind::ExactRational,
            None => ExactRationalKind::NonRational,
        }
    }

    #[inline]
    fn dot3(left: [&Self; 3], right: [&Self; 3]) -> Self {
        // Structural-dispatch note: vector and matrix callers can carry
        // row/column sparsity, orthonormality, affine-basis, and common
        // rational-grid facts so this layer can route directly to sparse,
        // fused, or dyadic exact reducers. The sparse fixed-kernel scheduling
        // follows Gustavson's sparse-product idea; see Gustavson, "Two Fast
        // Algorithms for Sparse Matrices: Multiplication and Permuted
        // Transposition," ACM TOMS 4.3 (1978).
        Real::dot3_refs(left, right)
    }

    #[inline]
    fn dot4(left: [&Self; 4], right: [&Self; 4]) -> Self {
        Real::dot4_refs(left, right)
    }

    #[inline]
    fn dot3_same(values: [&Self; 3]) -> Self {
        Real::dot3_refs(values, values)
    }

    #[inline]
    fn dot4_same(values: [&Self; 4]) -> Self {
        Real::dot4_refs(values, values)
    }

    #[inline]
    fn linear_combination3(coefficients: [&Self; 3], values: [&Self; 3]) -> Self {
        Real::linear_combination3_refs(coefficients, values)
    }

    #[inline]
    fn active_linear_combination3(coefficients: [&Self; 3], values: [&Self; 3]) -> Self {
        Real::active_linear_combination3_refs(coefficients, values)
    }

    #[inline]
    fn linear_combination4(coefficients: [&Self; 4], values: [&Self; 4]) -> Self {
        Real::linear_combination4_refs(coefficients, values)
    }

    #[inline]
    fn signed_product_sum2<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self {
        // Fixed determinant and cofactor formulas are short signed sums of
        // products. Prune zero terms before the exact-rational fused path so
        // sparse cases avoid building a shared-denominator accumulator at all.
        // Dense exact-rational cases preserve the delayed-normalization
        // principle of Bareiss fraction-free elimination; see Bareiss,
        // "Sylvester's Identity and Multistep Integer-Preserving Gaussian
        // Elimination," Mathematics of Computation 22.103 (1968).
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

    #[inline]
    fn active_signed_product_sum2_known_exact_rational<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self {
        Real::exact_rational_signed_product_sum_known_exact(positive_terms, terms)
    }
}
