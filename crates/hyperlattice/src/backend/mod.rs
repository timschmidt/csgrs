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

    /// Whether repeated scaling by one shared scalar should borrow the factor.
    ///
    /// Borrowing the factor is a performance feature for symbolic/exact
    /// backends because it avoids cloning expression graphs or exact rational
    /// denominator state for every lane. Compact approximate scalars benchmark
    /// faster through the ordinary owned multiply expression that LLVM can
    /// inline and scalarize aggressively.
    const BORROW_SHARED_SCALE_FACTOR: bool = false;

    /// Whether fixed determinant/cofactor helpers should route short signed
    /// product sums through the backend.
    ///
    /// This is deliberately opt-in. Compact approximate scalars benchmark
    /// faster with the direct expression shape that LLVM already optimizes,
    /// while hyperreal exact rationals can use this hook to delay BigInt
    /// denominator canonicalization until the final cofactor row. 2026-05-09
    /// guarded Criterion after adding the opt-in gate: approximate mat3/mat4
    /// reciprocal, inverse, division, negative-powi, and borrowed division had
    /// no regressions outside noise; owned rows were faster in the clean run.
    const FUSE_SIGNED_PRODUCT_SUM: bool = false;

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
    /// Constructs tau.
    ///
    /// Backends can override this when they have a cached or symbolic `2*pi`.
    /// The default keeps compact approximate backends simple.
    fn tau() -> Self {
        crate::trace_dispatch!("realistic_blas_backend_trait", "constructor", "tau-default");
        Self::from(2_i8) * Self::pi()
    }
    /// Returns the multiplicative inverse.
    fn inverse(self) -> BlasResult<Self>;
    /// Returns the multiplicative inverse of a borrowed value.
    #[inline]
    fn inverse_ref(&self) -> BlasResult<Self> {
        crate::trace_dispatch!(
            "realistic_blas_backend_trait",
            "method",
            "inverse-ref-default"
        );
        self.clone().inverse()
    }
    /// Raises this value to a scalar exponent.
    fn pow(self, exponent: Self) -> BlasResult<Self>;
    /// Adds a borrowed right-hand operand.
    #[inline]
    fn add_ref(self, rhs: &Self) -> Self {
        crate::trace_dispatch!(
            "realistic_blas_backend_trait",
            "op",
            "add-owned-ref-default"
        );
        self + rhs.clone()
    }
    /// Adds an owned left-hand operand and a borrowed right-hand operand.
    #[inline]
    fn add_owned_ref(left: Self, right: &Self) -> Self {
        crate::trace_dispatch!(
            "realistic_blas_backend_trait",
            "op",
            "add-owned-ref-default"
        );
        left.add_ref(right)
    }
    /// Adds a borrowed left-hand operand and an owned right-hand operand.
    #[inline]
    fn add_ref_owned(left: &Self, right: Self) -> Self {
        crate::trace_dispatch!(
            "realistic_blas_backend_trait",
            "op",
            "add-ref-owned-default"
        );
        Self::add_refs(left, &right)
    }
    /// Adds two borrowed operands.
    #[inline]
    fn add_refs(left: &Self, right: &Self) -> Self {
        crate::trace_dispatch!("realistic_blas_backend_trait", "op", "add-ref-ref-default");
        left.clone().add_ref(right)
    }
    /// Subtracts a borrowed right-hand operand.
    #[inline]
    fn sub_ref(self, rhs: &Self) -> Self {
        crate::trace_dispatch!(
            "realistic_blas_backend_trait",
            "op",
            "sub-owned-ref-default"
        );
        self - rhs.clone()
    }
    /// Subtracts a borrowed right-hand operand from an owned left-hand operand.
    #[inline]
    fn sub_owned_ref(left: Self, right: &Self) -> Self {
        crate::trace_dispatch!(
            "realistic_blas_backend_trait",
            "op",
            "sub-owned-ref-default"
        );
        left.sub_ref(right)
    }
    /// Subtracts an owned right-hand operand from a borrowed left-hand operand.
    #[inline]
    fn sub_ref_owned(left: &Self, right: Self) -> Self {
        crate::trace_dispatch!(
            "realistic_blas_backend_trait",
            "op",
            "sub-ref-owned-default"
        );
        Self::sub_refs(left, &right)
    }
    /// Subtracts two borrowed operands.
    #[inline]
    fn sub_refs(left: &Self, right: &Self) -> Self {
        crate::trace_dispatch!("realistic_blas_backend_trait", "op", "sub-ref-ref-default");
        left.clone().sub_ref(right)
    }
    /// Multiplies by a borrowed right-hand operand.
    #[inline]
    fn mul_ref(self, rhs: &Self) -> Self {
        crate::trace_dispatch!(
            "realistic_blas_backend_trait",
            "op",
            "mul-owned-ref-default"
        );
        self * rhs.clone()
    }
    /// Multiplies an owned left-hand operand by a borrowed right-hand operand.
    #[inline]
    fn mul_owned_ref(left: Self, right: &Self) -> Self {
        crate::trace_dispatch!(
            "realistic_blas_backend_trait",
            "op",
            "mul-owned-ref-default"
        );
        left.mul_ref(right)
    }
    /// Multiplies a borrowed left-hand operand by an owned right-hand operand.
    #[inline]
    fn mul_ref_owned(left: &Self, right: Self) -> Self {
        crate::trace_dispatch!(
            "realistic_blas_backend_trait",
            "op",
            "mul-ref-owned-default"
        );
        Self::mul_refs(left, &right)
    }
    /// Multiplies two borrowed operands.
    #[inline]
    fn mul_refs(left: &Self, right: &Self) -> Self {
        crate::trace_dispatch!("realistic_blas_backend_trait", "op", "mul-ref-ref-default");
        left.clone().mul_ref(right)
    }
    /// Divides by a borrowed right-hand operand.
    #[inline]
    fn div_ref(self, rhs: &Self) -> BlasResult<Self> {
        crate::trace_dispatch!(
            "realistic_blas_backend_trait",
            "op",
            "div-owned-ref-default"
        );
        self.div(rhs.clone())
    }
    /// Divides an owned left-hand operand by a borrowed right-hand operand.
    #[inline]
    fn div_owned_ref(left: Self, right: &Self) -> BlasResult<Self> {
        crate::trace_dispatch!(
            "realistic_blas_backend_trait",
            "op",
            "div-owned-ref-default"
        );
        left.div_ref(right)
    }
    /// Divides a borrowed left-hand operand by an owned right-hand operand.
    #[inline]
    fn div_ref_owned(left: &Self, right: Self) -> BlasResult<Self> {
        crate::trace_dispatch!(
            "realistic_blas_backend_trait",
            "op",
            "div-ref-owned-default"
        );
        Self::div_refs(left, &right)
    }
    /// Divides two borrowed operands.
    #[inline]
    fn div_refs(left: &Self, right: &Self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_backend_trait", "op", "div-ref-ref-default");
        left.clone().div_ref(right)
    }
    /// Returns the three-lane dot product.
    #[inline]
    fn dot3(left: [&Self; 3], right: [&Self; 3]) -> Self {
        crate::trace_dispatch!("realistic_blas_backend_trait", "op", "dot3-default");
        let p0 = left[0].clone().mul_ref(right[0]);
        let p1 = left[1].clone().mul_ref(right[1]);
        let p2 = left[2].clone().mul_ref(right[2]);
        p0.add_ref(&p1).add_ref(&p2)
    }
    /// Returns the four-lane dot product.
    #[inline]
    fn dot4(left: [&Self; 4], right: [&Self; 4]) -> Self {
        crate::trace_dispatch!("realistic_blas_backend_trait", "op", "dot4-default");
        let p0 = left[0].clone().mul_ref(right[0]);
        let p1 = left[1].clone().mul_ref(right[1]);
        let p2 = left[2].clone().mul_ref(right[2]);
        let p3 = left[3].clone().mul_ref(right[3]);
        p0.add_ref(&p1).add_ref(&p2.add_ref(&p3))
    }
    /// Returns the three-lane linear combination `c0 * x0 + c1 * x1 + c2 * x2`.
    ///
    /// This default keeps existing dot-product behavior while giving matrix
    /// transform kernels a named hook to preserve richer forms in future
    /// backends.
    #[inline]
    fn linear_combination3(coeffs: [&Self; 3], values: [&Self; 3]) -> Self {
        crate::trace_dispatch!(
            "realistic_blas_backend_trait",
            "op",
            "linear-combination3-default"
        );
        Self::dot3(coeffs, values)
    }
    /// Returns the four-lane linear combination `c0 * x0 + c1 * x1 + c2 * x2 + c3 * x3`.
    ///
    /// This default keeps existing dot-product behavior while keeping the
    /// transform kernel interface consistent for fixed-size affine upgrades.
    #[inline]
    fn linear_combination4(coeffs: [&Self; 4], values: [&Self; 4]) -> Self {
        crate::trace_dispatch!(
            "realistic_blas_backend_trait",
            "op",
            "linear-combination4-default"
        );
        Self::dot4(coeffs, values)
    }
    /// Returns the four-lane affine combination `offset + c0 * x0 + c1 * x1 + c2 * x2 + c3 * x3`.
    ///
    /// The default lowers to the linear combination plus one addition.
    #[inline]
    fn affine_combination4(coeffs: [&Self; 4], values: [&Self; 4], offset: &Self) -> Self {
        crate::trace_dispatch!(
            "realistic_blas_backend_trait",
            "op",
            "affine-combination4-default"
        );
        Self::linear_combination4(coeffs, values).add_ref(offset)
    }
    /// Returns a fused signed sum of two-factor products.
    ///
    /// The default deliberately preserves the existing scalar operation order.
    /// Expensive exact backends can override this to delay canonicalization
    /// across determinant and cofactor polynomials. The performance model is
    /// the same one behind fraction-free exact linear algebra: delay expensive
    /// denominator work until a whole short polynomial has been assembled
    /// (Bareiss, Math. Comp. 22(103), 1968, <https://doi.org/10.2307/2004533>).
    #[inline]
    fn signed_product_sum2<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self {
        // Keep dense exact-rational paths delayed: eliminate zero factors before
        // combining to avoid constructing products that can never contribute.
        let mut first_term: Option<([&Self; 2], bool)> = None;
        let mut second_term: Option<([&Self; 2], bool)> = None;
        let mut total: Option<Self> = None;
        let mut nonzero_count = 0usize;

        for i in 0..TERMS {
            if terms[i][0].definitely_zero() || terms[i][1].definitely_zero() {
                continue;
            }

            let term = (terms[i], positive_terms[i]);
            nonzero_count += 1;

            match nonzero_count {
                1 => {
                    first_term = Some(term);
                }
                2 => {
                    second_term = Some(term);
                }
                _ => {
                    if nonzero_count == 3 {
                        let (left_term, left_positive) = first_term.expect(
                            "first non-zero term should have been recorded before dense accumulation",
                        );
                        let (right_term, right_positive) = second_term.expect(
                            "second non-zero term should have been recorded before dense accumulation",
                        );
                        let left_product = left_term[0].clone().mul_ref(left_term[1]);
                        let right_product = right_term[0].clone().mul_ref(right_term[1]);

                        total = Some(match (left_positive, right_positive) {
                            (true, true) => left_product + right_product,
                            (true, false) => left_product - right_product,
                            (false, true) => -left_product + right_product,
                            (false, false) => -(left_product + right_product),
                        });
                    }

                    let product = term.0[0].clone().mul_ref(term.0[1]);
                    total = Some(match total.take() {
                        Some(total) if term.1 => total.add_ref(&product),
                        Some(total) => total.sub_ref(&product),
                        None => {
                            if term.1 {
                                product
                            } else {
                                -product
                            }
                        }
                    });
                }
            }
        }

        match nonzero_count {
            0 => {
                crate::trace_dispatch!(
                    "realistic_blas_backend_trait",
                    "op",
                    "signed-product-sum2-all-zero"
                );
                Self::zero()
            }
            1 => {
                let (term, positive) = first_term.expect("single non-zero term tracked");
                crate::trace_dispatch!(
                    "realistic_blas_backend_trait",
                    "op",
                    "signed-product-sum2-single-term"
                );
                let product = term[0].clone().mul_ref(term[1]);
                if positive { product } else { -product }
            }
            2 => {
                let (left_term, left_positive) = first_term.expect("first non-zero term tracked");
                let (right_term, right_positive) =
                    second_term.expect("second non-zero term tracked");
                crate::trace_dispatch!(
                    "realistic_blas_backend_trait",
                    "op",
                    "signed-product-sum2-sparse-two"
                );
                let left_product = left_term[0].clone().mul_ref(left_term[1]);
                let right_product = right_term[0].clone().mul_ref(right_term[1]);
                match (left_positive, right_positive) {
                    (true, true) => left_product + right_product,
                    (true, false) => left_product - right_product,
                    (false, true) => -left_product + right_product,
                    (false, false) => -(left_product + right_product),
                }
            }
            _ => {
                crate::trace_dispatch!(
                    "realistic_blas_backend_trait",
                    "op",
                    "signed-product-sum2-dense"
                );
                total.expect("dense signed-product-sum2 should accumulate at least one value")
            }
        }
    }
    /// Returns a fused signed sum of already-active two-factor products.
    ///
    /// Callers use this only after they have already classified zero lanes.
    /// Unlike [`BackendScalar::signed_product_sum2`], the default deliberately
    /// does not issue fresh zero probes.
    #[inline]
    fn active_signed_product_sum2<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self {
        crate::trace_dispatch!(
            "realistic_blas_backend_trait",
            "op",
            "active-signed-product-sum2"
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
    /// Returns whether this value is definitely one.
    ///
    /// The default conservative check is `false`; exact backends should
    /// override this when they can prove a constant one exactly.
    #[inline]
    fn definitely_one(&self) -> bool {
        crate::trace_dispatch!("realistic_blas_backend_trait", "query", "definitely-one");
        false
    }
    /// Returns whether this value is definitely one or zero.
    ///
    /// `None` means the value is neither definitely zero nor definitely one.
    /// This specialization keeps point/direction branching to one query in
    /// vector transforms where both tests are otherwise needed.
    #[inline]
    fn zero_or_one(&self) -> Option<bool> {
        crate::trace_dispatch!("realistic_blas_backend_trait", "query", "zero-or-one");
        if self.definitely_zero() {
            Some(false)
        } else if self.definitely_one() {
            Some(true)
        } else {
            None
        }
    }
    /// Classifies whether this value is zero.
    fn zero_status(&self) -> ZeroStatus;
    /// Returns conservative structural facts about this value.
    #[inline]
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
    /// Returns true when the value is exactly rational and dyadic.
    ///
    /// Matrix right-division uses this as a representation heuristic: dyadic
    /// hyperreal rationals reduce with shifts, so shared-adjugate kernels can
    /// win there, while decimal rationals usually favor Gauss-Jordan solves
    /// with fewer intermediate products.
    fn is_exact_dyadic_rational(&self) -> bool {
        false
    }
    /// Tries to prove the sign without refining beyond the requested precision.
    #[inline]
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
