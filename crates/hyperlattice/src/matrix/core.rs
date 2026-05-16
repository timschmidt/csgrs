//! Fixed-size row-major matrices over [`Scalar`](crate::Scalar).
//!
//! Implementation map:
//! - type layout and generic array helpers
//! - powers, right-division, and fixed-size multiply kernels
//! - matrix-vector transforms and reusable transform handles
//! - determinant, adjugate, and inverse kernels
//! - public Matrix3/Matrix4 methods and operator impls

use std::array::from_fn;
use std::fmt;
use std::mem;
use std::ops::{Add, BitXor, Div, Index, IndexMut, Mul, Neg, Sub};

use crate::backend::{BackendScalar, ExactRationalKind};
use crate::require_known_nonzero_with_abort;
use crate::scalar::{
    ZeroStatus, clone_with_abort, reject_definite_zero, require_known_nonzero, with_abort,
    zero_status, zero_status_with_abort,
};
use crate::vector::{Vector3, Vector4, Vector4GeometricFacts, Vector4HomogeneousKind};
use crate::{AbortSignal, Backend, BlasResult, CheckedBlasResult, DefaultBackend, Problem, Scalar};

fn identity_array<B: Backend, const N: usize>() -> [[Scalar<B>; N]; N] {
    from_fn(|row| {
        from_fn(|col| {
            if row == col {
                Scalar::one()
            } else {
                Scalar::zero()
            }
        })
    })
}

fn transpose_array3<B: Backend>(matrix: [[Scalar<B>; 3]; 3]) -> [[Scalar<B>; 3]; 3] {
    // Right-division is implemented as a solve on transposes. Fixed-size
    // transposes keep that wrapper from paying generic `Option::take` and
    // `from_fn` overhead around the actual Gauss-Jordan work. 2026-05
    // targeted Criterion, 200 samples/8s: approx mat3 div_matrix_checked
    // improved ~40%, approx mat3 div_matrix improved ~22%, borrowed approx
    // mat3 div improved ~18%, and hyperreal-rational mat3 div rows stayed
    // neutral-to-slightly-better inside the 2% noise threshold.
    let [[m00, m01, m02], [m10, m11, m12], [m20, m21, m22]] = matrix;
    [[m00, m10, m20], [m01, m11, m21], [m02, m12, m22]]
}

fn transpose_array3_ref<B: Backend>(matrix: &[[Scalar<B>; 3]; 3]) -> [[Scalar<B>; 3]; 3] {
    [
        [
            matrix[0][0].clone(),
            matrix[1][0].clone(),
            matrix[2][0].clone(),
        ],
        [
            matrix[0][1].clone(),
            matrix[1][1].clone(),
            matrix[2][1].clone(),
        ],
        [
            matrix[0][2].clone(),
            matrix[1][2].clone(),
            matrix[2][2].clone(),
        ],
    ]
}

fn transpose_array4<B: Backend>(matrix: [[Scalar<B>; 4]; 4]) -> [[Scalar<B>; 4]; 4] {
    // Hand-written 4x4 transpose avoids the generic `Option::take` owned
    // transpose overhead in the right-division wrapper. 2026-05 targeted
    // Criterion, 200 samples/8s: approx mat4 div_matrix improved ~20%,
    // hyperreal-rational mat4 div_matrix improved ~2% within noise, borrowed
    // hyperreal-rational mat4 div was neutral, and borrowed approx mat4 div
    // moved +1.25% inside the 2% noise threshold.
    let [
        [m00, m01, m02, m03],
        [m10, m11, m12, m13],
        [m20, m21, m22, m23],
        [m30, m31, m32, m33],
    ] = matrix;
    [
        [m00, m10, m20, m30],
        [m01, m11, m21, m31],
        [m02, m12, m22, m32],
        [m03, m13, m23, m33],
    ]
}

fn transpose_array4_ref<B: Backend>(matrix: &[[Scalar<B>; 4]; 4]) -> [[Scalar<B>; 4]; 4] {
    // Same as `transpose_array_ref`, but fully unrolled because the 4x4
    // borrowed division benchmark is sensitive to generic array construction.
    [
        [
            matrix[0][0].clone(),
            matrix[1][0].clone(),
            matrix[2][0].clone(),
            matrix[3][0].clone(),
        ],
        [
            matrix[0][1].clone(),
            matrix[1][1].clone(),
            matrix[2][1].clone(),
            matrix[3][1].clone(),
        ],
        [
            matrix[0][2].clone(),
            matrix[1][2].clone(),
            matrix[2][2].clone(),
            matrix[3][2].clone(),
        ],
        [
            matrix[0][3].clone(),
            matrix[1][3].clone(),
            matrix[2][3].clone(),
            matrix[3][3].clone(),
        ],
    ]
}

/// Three-by-three row-major matrix.
#[derive(Clone, Debug, PartialEq)]
pub struct Matrix3<B: Backend = DefaultBackend>(
    /// Matrix entries in row-major order.
    pub [[Scalar<B>; 3]; 3],
);

/// Four-by-four row-major matrix.
#[derive(Clone, Debug, PartialEq)]
pub struct Matrix4<B: Backend = DefaultBackend>(
    /// Matrix entries in row-major order.
    pub [[Scalar<B>; 4]; 4],
);

/// Cached structural and exact-structure metadata for repeated right-division
/// by the same 3×3 divisor.
///
/// This type intentionally stores structural facts once and can lazily cache
/// the divisor adjugate, determinant, and determinant inverse. Yap's exact
/// geometric computation notes emphasize avoiding repeated expensive recomputation
/// when object-level structure is stable ("Towards Exact Geometric Computation",
/// 1997). The structure is tuned to keep fast-path checks cheap and defer
// shared-scale inversion until the first call that benefits.
#[derive(Debug, Clone)]
pub struct PreparedRightDivisor3<'a, B: Backend = DefaultBackend> {
    divisor: &'a Matrix3<B>,
    facts: Matrix3Facts,
    right_exact_rational_kind: ExactRationalKind,
    is_definitely_dense_for_inverse: bool,
    adjugate: Option<[[Scalar<B>; 3]; 3]>,
    determinant: Option<Scalar<B>>,
    reciprocal_determinant: Option<Scalar<B>>,
    inverse: Option<Matrix3<B>>,
}

/// Cached structural and exact-structure metadata for repeated right-division
/// by the same 4×4 divisor.
///
/// This mirrors `PreparedRightDivisor3` and additionally retains the fixed-
/// minor factors used by the existing cofactor inverse schedule. The extra
/// cache lets repeated divisions skip recomputing `(s, c)` and the shared
/// determinant path when it is beneficial.
#[derive(Debug, Clone)]
pub struct PreparedRightDivisor4<'a, B: Backend = DefaultBackend> {
    divisor: &'a Matrix4<B>,
    facts: Matrix4Facts,
    right_exact_rational_kind: ExactRationalKind,
    is_definitely_dense_for_inverse: bool,
    factors: Option<([Scalar<B>; 6], [Scalar<B>; 6])>,
    adjugate: Option<[[Scalar<B>; 4]; 4]>,
    determinant: Option<Scalar<B>>,
    reciprocal_determinant: Option<Scalar<B>>,
    inverse: Option<Matrix4<B>>,
}

#[derive(Clone, Copy, Debug)]
struct Matrix3Facts {
    is_identity: bool,
    is_diagonal: bool,
    // Triangular structure is used to select O(n²) triangular inverse kernels
    // before heavier affine/cofactor paths. This follows standard triangular
    // solve scheduling in Golub & Van Loan, *Matrix Computations*.
    is_upper_triangular: bool,
    is_lower_triangular: bool,
    // Cached off-diagonal signal for the affine 2×2 block; used to skip full
    // affine inversion/division schedules when the linear block is axis-aligned.
    // Retaining this cheap structural fact follows the exact-geometric strategy
    // of reducing symbolic/geometric structure before arithmetic; see Yap,
    // "Towards Exact Geometric Computation", 1997.
    linear_is_diagonal: bool,
    is_affine: bool,
    is_affine_translation: bool,
}

#[inline]
fn combine_exact_rational_kind(
    left: ExactRationalKind,
    right: ExactRationalKind,
) -> ExactRationalKind {
    use ExactRationalKind::{ExactDyadicRational, ExactRational, NonRational};
    match (left, right) {
        (NonRational, _) | (_, NonRational) => NonRational,
        (ExactRational, _) | (_, ExactRational) => ExactRational,
        (ExactDyadicRational, ExactDyadicRational) => ExactDyadicRational,
    }
}

#[inline]
fn matrix3_exact_rational_kind<B: Backend>(matrix: &[[Scalar<B>; 3]; 3]) -> ExactRationalKind {
    let mut kind = ExactRationalKind::ExactDyadicRational;
    for row in matrix {
        for value in row {
            kind = combine_exact_rational_kind(kind, value.exact_rational_kind());
            if kind == ExactRationalKind::NonRational {
                return kind;
            }
        }
    }
    kind
}

#[inline]
fn matrix4_exact_rational_kind<B: Backend>(matrix: &[[Scalar<B>; 4]; 4]) -> ExactRationalKind {
    let mut kind = ExactRationalKind::ExactDyadicRational;
    for row in matrix {
        for value in row {
            kind = combine_exact_rational_kind(kind, value.exact_rational_kind());
            if kind == ExactRationalKind::NonRational {
                return kind;
            }
        }
    }
    kind
}

#[inline]
fn matrix_exact_rational_kind<B: Backend, const N: usize>(
    matrix: &[[Scalar<B>; N]; N],
) -> ExactRationalKind {
    let mut kind = ExactRationalKind::ExactDyadicRational;
    for row in matrix {
        for value in row {
            kind = combine_exact_rational_kind(kind, value.exact_rational_kind());
            if kind == ExactRationalKind::NonRational {
                return kind;
            }
        }
    }
    kind
}

impl<'a, B: Backend> PreparedRightDivisor3<'a, B> {
    /// Build a reusable cache for repeated right-division against this divisor.
    pub fn new(divisor: &'a Matrix3<B>) -> Self {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor3-new"
        );
        let facts = matrix3_facts(&divisor.0);
        let right_exact_rational_kind = matrix3_exact_rational_kind(&divisor.0);
        Self {
            divisor,
            facts,
            right_exact_rational_kind,
            is_definitely_dense_for_inverse: B::FUSE_SIGNED_PRODUCT_SUM
                && matrix3_is_definitely_dense_for_inverse(&divisor.0),
            adjugate: None,
            determinant: None,
            reciprocal_determinant: None,
            inverse: None,
        }
    }

    /// Borrow the cached right-divisor matrix itself.
    ///
    /// The division path keeps structural facts colocated with this pointer so that
    /// kernels can avoid redundant structural recomputation.
    pub fn divisor(&self) -> &Matrix3<B> {
        self.divisor
    }

    #[inline]
    fn can_use_shared_adjugate(&self, left: &[[Scalar<B>; 3]; 3]) -> bool {
        match self.right_exact_rational_kind {
            ExactRationalKind::ExactDyadicRational => {
                matrix3_exact_rational_kind(left) == ExactRationalKind::ExactDyadicRational
            }
            ExactRationalKind::ExactRational => {
                matches!(
                    matrix3_exact_rational_kind(left),
                    ExactRationalKind::ExactDyadicRational | ExactRationalKind::ExactRational
                )
            }
            ExactRationalKind::NonRational => false,
        }
    }

    fn prepare_shared_adjugate(&mut self) -> BlasResult<&[[Scalar<B>; 3]; 3]> {
        if self.adjugate.is_none() {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "prepared-right-divisor3-cache-shared-adjugate"
            );
            let known_rational = self.right_exact_rational_kind == ExactRationalKind::ExactRational;
            let (adjugate, determinant) = if self.is_definitely_dense_for_inverse && known_rational
            {
                matrix3_adjugate_and_determinant_dense_exact_known_rational(&self.divisor.0)
            } else if self.is_definitely_dense_for_inverse {
                matrix3_adjugate_and_determinant_dense_exact(&self.divisor.0)
            } else {
                matrix3_adjugate_and_determinant(&self.divisor.0)
            };
            let reciprocal_determinant = determinant.inverse_ref()?;
            self.adjugate = Some(adjugate);
            self.determinant = Some(determinant);
            self.reciprocal_determinant = Some(reciprocal_determinant);
        }
        Ok(self
            .adjugate
            .as_ref()
            .expect("adjugate cache must be present"))
    }

    fn prepare_shared_adjugate_checked(&mut self) -> CheckedBlasResult<&[[Scalar<B>; 3]; 3]> {
        if self.adjugate.is_none() {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "prepared-right-divisor3-cache-shared-adjugate-checked"
            );
            let known_rational = self.right_exact_rational_kind == ExactRationalKind::ExactRational;
            let (adjugate, determinant) = if self.is_definitely_dense_for_inverse && known_rational
            {
                matrix3_adjugate_and_determinant_dense_exact_known_rational(&self.divisor.0)
            } else if self.is_definitely_dense_for_inverse {
                matrix3_adjugate_and_determinant_dense_exact(&self.divisor.0)
            } else {
                matrix3_adjugate_and_determinant(&self.divisor.0)
            };
            require_known_nonzero(&determinant)?;
            let reciprocal_determinant = determinant.inverse_ref()?;
            self.adjugate = Some(adjugate);
            self.determinant = Some(determinant);
            self.reciprocal_determinant = Some(reciprocal_determinant);
        }
        Ok(self
            .adjugate
            .as_ref()
            .expect("adjugate cache must be present"))
    }

    fn prepare_shared_adjugate_checked_with_abort(
        &mut self,
        signal: &AbortSignal,
    ) -> CheckedBlasResult<&[[Scalar<B>; 3]; 3]> {
        if self.adjugate.is_none() {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "prepared-right-divisor3-cache-shared-adjugate-checked-abort"
            );
            let known_rational = self.right_exact_rational_kind == ExactRationalKind::ExactRational;
            let (adjugate, determinant) = if self.is_definitely_dense_for_inverse && known_rational
            {
                matrix3_adjugate_and_determinant_dense_exact_known_rational(&self.divisor.0)
            } else if self.is_definitely_dense_for_inverse {
                matrix3_adjugate_and_determinant_dense_exact(&self.divisor.0)
            } else {
                matrix3_adjugate_and_determinant(&self.divisor.0)
            };
            let determinant = with_abort(determinant, signal);
            require_known_nonzero(&determinant)?;
            let reciprocal_determinant = determinant.inverse_ref()?;
            self.adjugate = Some(adjugate);
            // Keep the shared determinant cache exact for future abort-aware or
            // checked calls; this aligns with Yap's repeated-object reuse
            // guidance by avoiding unnecessary recomputation on subsequent
            // prepared-path uses.
            self.determinant = Some(determinant);
            self.reciprocal_determinant = Some(reciprocal_determinant);
        }
        Ok(self
            .adjugate
            .as_ref()
            .expect("adjugate cache must be present"))
    }

    /// Divides a left operand using cached divisor facts and cached shared
    /// adjugate/canonicalized determinant information.
    ///
    /// Reusing the right-side structural facts mirrors the exact GEOMETRIC
    /// approach in Yap, "Towards Exact Geometric Computation", 1997: expensive
    /// structure and factor derivations should be hoisted out of repeated
    /// calls when the object is reused.
    pub fn divide(&mut self, left: [[Scalar<B>; 3]; 3]) -> BlasResult<[[Scalar<B>; 3]; 3]> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor3-divide"
        );
        right_divide_matrix3_prepared(left, self)
    }

    /// Divides with checked zero-determinant behavior using cached factors.
    ///
    /// The checked variant still enforces a known-nonzero determinant check
    /// before reciprocation, matching existing `/` checked semantics while
    /// avoiding recomputation for repeated calls.
    pub fn divide_checked(
        &mut self,
        left: [[Scalar<B>; 3]; 3],
    ) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor3-divide-checked"
        );
        right_divide_matrix3_prepared_checked(left, self)
    }

    /// Divides with checked abort-aware semantics using cached factors.
    ///
    /// Abort-aware checks remain a thin layer here: first select the cached
    /// specialization and only then propagate the signal through the required
    /// determinant checks.
    pub fn divide_checked_with_abort(
        &mut self,
        left: [[Scalar<B>; 3]; 3],
        signal: &AbortSignal,
    ) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor3-divide-checked-abort"
        );
        right_divide_matrix3_prepared_checked_with_abort(left, self, signal)
    }

    /// Returns the inverse of the prepared divisor using its cached adjugate
    /// and reciprocal determinant.
    pub fn inverse(&mut self) -> BlasResult<Matrix3<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor3-inverse"
        );
        if let Some(inverse) = &self.inverse {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "prepared-right-divisor3-inverse-cache-hit"
            );
            return Ok(inverse.clone());
        }
        let _ = self.prepare_shared_adjugate()?;
        let inv_det = self
            .reciprocal_determinant
            .as_ref()
            .expect("reciprocal determinant cache should be present");
        let adjugate = self
            .adjugate
            .as_ref()
            .expect("adjugate cache should be present")
            .clone();
        let inverse = Matrix3(scale_matrix3(adjugate, inv_det));
        self.inverse = Some(inverse.clone());
        Ok(inverse)
    }

    /// Checked inverse of the prepared divisor using cached factors.
    pub fn inverse_checked(&mut self) -> CheckedBlasResult<Matrix3<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor3-inverse-checked"
        );
        let _ = self.prepare_shared_adjugate_checked()?;
        if let Some(inverse) = &self.inverse {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "prepared-right-divisor3-inverse-checked-cache-hit"
            );
            return Ok(inverse.clone());
        }
        let inv_det = self
            .reciprocal_determinant
            .as_ref()
            .expect("reciprocal determinant cache should be present");
        let adjugate = self
            .adjugate
            .as_ref()
            .expect("adjugate cache should be present")
            .clone();
        let inverse = Matrix3(scale_matrix3(adjugate, inv_det));
        self.inverse = Some(inverse.clone());
        Ok(inverse)
    }

    /// Abort-aware checked inverse of the prepared divisor using cached factors.
    pub fn inverse_checked_with_abort(
        &mut self,
        signal: &AbortSignal,
    ) -> CheckedBlasResult<Matrix3<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor3-inverse-checked-abort"
        );
        let _ = self.prepare_shared_adjugate_checked_with_abort(signal)?;
        if let Some(inverse) = &self.inverse {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "prepared-right-divisor3-inverse-checked-abort-cache-hit"
            );
            return Ok(inverse.clone());
        }
        let inv_det = self
            .reciprocal_determinant
            .as_ref()
            .expect("reciprocal determinant cache should be present");
        let adjugate = self
            .adjugate
            .as_ref()
            .expect("adjugate cache should be present")
            .clone();
        let inverse = Matrix3(scale_matrix3(adjugate, inv_det));
        self.inverse = Some(inverse.clone());
        Ok(inverse)
    }

    /// Returns the reciprocal matrix of the prepared divisor.
    pub fn reciprocal(&mut self) -> BlasResult<Matrix3<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor3-reciprocal"
        );
        self.inverse()
    }

    /// Checked reciprocal matrix of the prepared divisor.
    pub fn reciprocal_checked(&mut self) -> CheckedBlasResult<Matrix3<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor3-reciprocal-checked"
        );
        self.inverse_checked()
    }

    /// Abort-aware checked reciprocal matrix of the prepared divisor.
    pub fn reciprocal_checked_with_abort(
        &mut self,
        signal: &AbortSignal,
    ) -> CheckedBlasResult<Matrix3<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor3-reciprocal-checked-abort"
        );
        self.inverse_checked_with_abort(signal)
    }
}

impl<'a, B: Backend> PreparedRightDivisor4<'a, B> {
    /// Build a reusable cache for repeated right-division against this divisor.
    pub fn new(divisor: &'a Matrix4<B>) -> Self {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor4-new"
        );
        let facts = matrix4_facts(&divisor.0);
        let right_exact_rational_kind = matrix4_exact_rational_kind(&divisor.0);
        Self {
            divisor,
            facts,
            right_exact_rational_kind,
            is_definitely_dense_for_inverse: B::FUSE_SIGNED_PRODUCT_SUM
                && facts.is_definitely_dense_for_inverse,
            factors: None,
            adjugate: None,
            determinant: None,
            reciprocal_determinant: None,
            inverse: None,
        }
    }

    /// Borrow the cached right-divisor matrix itself.
    ///
    /// The division path keeps structural facts colocated with this pointer so that
    /// kernels can avoid redundant structural recomputation.
    pub fn divisor(&self) -> &Matrix4<B> {
        self.divisor
    }

    #[inline]
    fn can_use_shared_adjugate(&self, left: &[[Scalar<B>; 4]; 4]) -> bool {
        if B::FUSE_SIGNED_PRODUCT_SUM
            && self.right_exact_rational_kind != ExactRationalKind::NonRational
        {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "prepared-right-divisor4-exact-right-skip-left-kind"
            );
            return true;
        }
        match self.right_exact_rational_kind {
            ExactRationalKind::ExactDyadicRational => {
                let left_kind = matrix4_exact_rational_kind(left);
                left_kind == ExactRationalKind::ExactDyadicRational
                    || (B::FUSE_SIGNED_PRODUCT_SUM && left_kind == ExactRationalKind::ExactRational)
            }
            ExactRationalKind::ExactRational => {
                matches!(
                    matrix4_exact_rational_kind(left),
                    ExactRationalKind::ExactDyadicRational | ExactRationalKind::ExactRational
                )
            }
            ExactRationalKind::NonRational => false,
        }
    }

    fn prepare_shared_adjugate(&mut self) -> BlasResult<&[[Scalar<B>; 4]; 4]> {
        if self.adjugate.is_none() {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "prepared-right-divisor4-cache-shared-adjugate"
            );
            let dense_exact = self.is_definitely_dense_for_inverse;
            let known_rational = self.right_exact_rational_kind != ExactRationalKind::NonRational;
            let factors = self.factors.get_or_insert_with(|| {
                // Cache the six fixed 4×4 minors once for all prepared
                // applications. Reusing the same cache lets repeated
                // right-divisions avoid re-materializing minors while still
                // delaying scalar canonicalization until the final shared
                // reciprocal scale.
                if dense_exact && known_rational {
                    matrix4_factors_dense_exact_known_rational(&self.divisor.0)
                } else if dense_exact {
                    matrix4_factors_dense_exact(&self.divisor.0)
                } else {
                    matrix4_factors(&self.divisor.0)
                }
            });
            let determinant = if dense_exact && known_rational {
                determinant4_from_factors_known_rational(&factors.0, &factors.1)
            } else {
                determinant4_from_factors(&factors.0, &factors.1)
            };
            let reciprocal_determinant = determinant.inverse_ref()?;
            let adjugate = if dense_exact && known_rational {
                matrix4_adjugate_from_factors_dense_exact_known_rational(
                    &self.divisor.0,
                    &factors.0,
                    &factors.1,
                )
            } else if dense_exact {
                matrix4_adjugate_from_factors_dense_exact(&self.divisor.0, &factors.0, &factors.1)
            } else {
                matrix4_adjugate_from_factors(&self.divisor.0, &factors.0, &factors.1)
            };
            self.adjugate = Some(adjugate);
            self.determinant = Some(determinant);
            self.reciprocal_determinant = Some(reciprocal_determinant);
        }
        Ok(self
            .adjugate
            .as_ref()
            .expect("adjugate cache must be present"))
    }

    fn prepare_shared_adjugate_checked(&mut self) -> CheckedBlasResult<&[[Scalar<B>; 4]; 4]> {
        if self.adjugate.is_none() {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "prepared-right-divisor4-cache-shared-adjugate-checked"
            );
            let dense_exact = self.is_definitely_dense_for_inverse;
            let known_rational = self.right_exact_rational_kind != ExactRationalKind::NonRational;
            let factors = self.factors.get_or_insert_with(|| {
                // Keep the cached factors in the prepared handle so checked and
                // abort-aware division variants can share the exact same
                // structural work in each pass. This is a direct application
                // of Yap-style object-level reuse for repeated geometry
                // kernels (Yap, 1997).
                if dense_exact && known_rational {
                    matrix4_factors_dense_exact_known_rational(&self.divisor.0)
                } else if dense_exact {
                    matrix4_factors_dense_exact(&self.divisor.0)
                } else {
                    matrix4_factors(&self.divisor.0)
                }
            });
            let determinant = if dense_exact && known_rational {
                determinant4_from_factors_known_rational(&factors.0, &factors.1)
            } else {
                determinant4_from_factors(&factors.0, &factors.1)
            };
            require_known_nonzero(&determinant)?;
            let reciprocal_determinant = determinant.inverse_ref()?;
            let adjugate = if dense_exact && known_rational {
                matrix4_adjugate_from_factors_dense_exact_known_rational(
                    &self.divisor.0,
                    &factors.0,
                    &factors.1,
                )
            } else if dense_exact {
                matrix4_adjugate_from_factors_dense_exact(&self.divisor.0, &factors.0, &factors.1)
            } else {
                matrix4_adjugate_from_factors(&self.divisor.0, &factors.0, &factors.1)
            };
            self.adjugate = Some(adjugate);
            self.determinant = Some(determinant);
            self.reciprocal_determinant = Some(reciprocal_determinant);
        }
        Ok(self
            .adjugate
            .as_ref()
            .expect("adjugate cache must be present"))
    }

    fn prepare_shared_adjugate_checked_with_abort(
        &mut self,
        signal: &AbortSignal,
    ) -> CheckedBlasResult<&[[Scalar<B>; 4]; 4]> {
        if self.adjugate.is_none() {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "prepared-right-divisor4-cache-shared-adjugate-checked-abort"
            );
            let dense_exact = self.is_definitely_dense_for_inverse;
            let known_rational = self.right_exact_rational_kind != ExactRationalKind::NonRational;
            let factors = self.factors.get_or_insert_with(|| {
                // Keep this branch aligned with the checked variant so only one
                // factorization runs for a prepared divisor, then every abort-aware
                // caller shares that cache.
                if dense_exact && known_rational {
                    matrix4_factors_dense_exact_known_rational(&self.divisor.0)
                } else if dense_exact {
                    matrix4_factors_dense_exact(&self.divisor.0)
                } else {
                    matrix4_factors(&self.divisor.0)
                }
            });
            let determinant = if dense_exact && known_rational {
                with_abort(
                    determinant4_from_factors_known_rational(&factors.0, &factors.1),
                    signal,
                )
            } else {
                with_abort(determinant4_from_factors(&factors.0, &factors.1), signal)
            };
            require_known_nonzero(&determinant)?;
            let reciprocal_determinant = determinant.inverse_ref()?;
            let adjugate = if dense_exact && known_rational {
                matrix4_adjugate_from_factors_dense_exact_known_rational(
                    &self.divisor.0,
                    &factors.0,
                    &factors.1,
                )
            } else if dense_exact {
                matrix4_adjugate_from_factors_dense_exact(&self.divisor.0, &factors.0, &factors.1)
            } else {
                matrix4_adjugate_from_factors(&self.divisor.0, &factors.0, &factors.1)
            };
            self.adjugate = Some(adjugate);
            self.determinant = Some(determinant);
            self.reciprocal_determinant = Some(reciprocal_determinant);
        }
        Ok(self
            .adjugate
            .as_ref()
            .expect("adjugate cache must be present"))
    }

    /// Divides a left operand using cached divisor facts and cached shared
    /// adjugate/canonicalized determinant information.
    ///
    /// Reusing the right-side structural facts mirrors the exact GEOMETRIC
    /// approach in Yap, "Towards Exact Geometric Computation", 1997: expensive
    /// structure and factor derivations should be hoisted out of repeated
    /// calls when the object is reused.
    pub fn divide(&mut self, left: [[Scalar<B>; 4]; 4]) -> BlasResult<[[Scalar<B>; 4]; 4]> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor4-divide"
        );
        right_divide_matrix4_prepared(left, self)
    }

    /// Divides a caller-certified exact-rational left operand by this prepared
    /// divisor.
    ///
    /// This is intentionally an explicit API: generic callers still use
    /// [`PreparedRightDivisor4::divide`], while solver/geometry code that
    /// already carries an exact-rational matrix certificate can avoid rescanning
    /// the left matrix before selecting the known-rational multiply schedule.
    pub fn divide_exact_rational_left(
        &mut self,
        left: [[Scalar<B>; 4]; 4],
    ) -> BlasResult<[[Scalar<B>; 4]; 4]> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor4-divide-exact-rational-left"
        );
        right_divide_matrix4_prepared_exact_rational_left(left, self)
    }

    /// Divides with checked zero-determinant behavior using cached factors.
    ///
    /// The checked variant still enforces a known-nonzero determinant check
    /// before reciprocation, matching existing `/` checked semantics while
    /// avoiding recomputation for repeated calls.
    pub fn divide_checked(
        &mut self,
        left: [[Scalar<B>; 4]; 4],
    ) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor4-divide-checked"
        );
        right_divide_matrix4_prepared_checked(left, self)
    }

    /// Divides with checked abort-aware semantics using cached factors.
    ///
    /// Abort-aware checks remain a thin layer here: first select the cached
    /// specialization and only then propagate the signal through the required
    /// determinant checks.
    pub fn divide_checked_with_abort(
        &mut self,
        left: [[Scalar<B>; 4]; 4],
        signal: &AbortSignal,
    ) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor4-divide-checked-abort"
        );
        right_divide_matrix4_prepared_checked_with_abort(left, self, signal)
    }

    /// Returns the inverse of the prepared divisor using its cached adjugate
    /// and reciprocal determinant.
    ///
    /// This exposes the same object-level cache used by prepared right-division
    /// to callers that repeatedly need the inverse matrix itself.
    pub fn inverse(&mut self) -> BlasResult<Matrix4<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor4-inverse"
        );
        if let Some(inverse) = &self.inverse {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "prepared-right-divisor4-inverse-cache-hit"
            );
            return Ok(inverse.clone());
        }
        let _ = self.prepare_shared_adjugate()?;
        let inv_det = self
            .reciprocal_determinant
            .as_ref()
            .expect("reciprocal determinant cache should be present");
        let adjugate = self
            .adjugate
            .as_ref()
            .expect("adjugate cache should be present")
            .clone();
        let inverse = Matrix4(scale_matrix4(adjugate, inv_det));
        self.inverse = Some(inverse.clone());
        Ok(inverse)
    }

    /// Checked inverse of the prepared divisor using cached factors.
    pub fn inverse_checked(&mut self) -> CheckedBlasResult<Matrix4<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor4-inverse-checked"
        );
        let _ = self.prepare_shared_adjugate_checked()?;
        if let Some(inverse) = &self.inverse {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "prepared-right-divisor4-inverse-checked-cache-hit"
            );
            return Ok(inverse.clone());
        }
        let inv_det = self
            .reciprocal_determinant
            .as_ref()
            .expect("reciprocal determinant cache should be present");
        let adjugate = self
            .adjugate
            .as_ref()
            .expect("adjugate cache should be present")
            .clone();
        let inverse = Matrix4(scale_matrix4(adjugate, inv_det));
        self.inverse = Some(inverse.clone());
        Ok(inverse)
    }

    /// Abort-aware checked inverse of the prepared divisor using cached factors.
    pub fn inverse_checked_with_abort(
        &mut self,
        signal: &AbortSignal,
    ) -> CheckedBlasResult<Matrix4<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor4-inverse-checked-abort"
        );
        let _ = self.prepare_shared_adjugate_checked_with_abort(signal)?;
        if let Some(inverse) = &self.inverse {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "prepared-right-divisor4-inverse-checked-abort-cache-hit"
            );
            return Ok(inverse.clone());
        }
        let inv_det = self
            .reciprocal_determinant
            .as_ref()
            .expect("reciprocal determinant cache should be present");
        let adjugate = self
            .adjugate
            .as_ref()
            .expect("adjugate cache should be present")
            .clone();
        let inverse = Matrix4(scale_matrix4(adjugate, inv_det));
        self.inverse = Some(inverse.clone());
        Ok(inverse)
    }

    /// Returns the reciprocal matrix of the prepared divisor.
    ///
    /// This is an explicit reciprocal-family spelling for callers that have
    /// prepared the divisor object. It reuses the same cached scaled inverse as
    /// [`PreparedRightDivisor4::inverse`] instead of falling back to generic
    /// `Matrix4::reciprocal`.
    pub fn reciprocal(&mut self) -> BlasResult<Matrix4<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor4-reciprocal"
        );
        self.inverse()
    }

    /// Checked reciprocal matrix of the prepared divisor.
    pub fn reciprocal_checked(&mut self) -> CheckedBlasResult<Matrix4<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor4-reciprocal-checked"
        );
        self.inverse_checked()
    }

    /// Abort-aware checked reciprocal matrix of the prepared divisor.
    pub fn reciprocal_checked_with_abort(
        &mut self,
        signal: &AbortSignal,
    ) -> CheckedBlasResult<Matrix4<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor4-reciprocal-checked-abort"
        );
        self.inverse_checked_with_abort(signal)
    }

    /// Raises the prepared divisor to an integer power.
    ///
    /// Negative powers reuse the cached prepared inverse, so repeated
    /// `A^-k` workloads pay determinant/cofactor setup once at this object
    /// boundary instead of once per power call.
    pub fn powi(&mut self, exponent: i32) -> BlasResult<Matrix4<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor4-powi"
        );
        if exponent == -1 {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "powi",
                "prepared-negative-one-inverse"
            );
            return self.inverse();
        }
        let base = if exponent < 0 {
            self.inverse()?.0
        } else {
            self.divisor.0.clone()
        };
        let power = if self.right_exact_rational_kind != ExactRationalKind::NonRational {
            matrix_power4_known_rational(base, exponent.unsigned_abs())
        } else {
            matrix_power4(base, exponent.unsigned_abs())
        };
        Ok(Matrix4(power))
    }

    /// Checked integer power of the prepared divisor.
    pub fn powi_checked(&mut self, exponent: i32) -> CheckedBlasResult<Matrix4<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor4-powi-checked"
        );
        if exponent == -1 {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "powi",
                "prepared-negative-one-inverse-checked"
            );
            return self.inverse_checked();
        }
        let base = if exponent < 0 {
            self.inverse_checked()?.0
        } else {
            self.divisor.0.clone()
        };
        let power = if self.right_exact_rational_kind != ExactRationalKind::NonRational {
            matrix_power4_known_rational(base, exponent.unsigned_abs())
        } else {
            matrix_power4(base, exponent.unsigned_abs())
        };
        Ok(Matrix4(power))
    }

    /// Abort-aware checked integer power of the prepared divisor.
    pub fn powi_checked_with_abort(
        &mut self,
        exponent: i32,
        signal: &AbortSignal,
    ) -> CheckedBlasResult<Matrix4<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "prepared-right-divisor4-powi-checked-abort"
        );
        if exponent == -1 {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "powi",
                "prepared-negative-one-inverse-checked-abort"
            );
            return self.inverse_checked_with_abort(signal);
        }
        let base = if exponent < 0 {
            self.inverse_checked_with_abort(signal)?.0
        } else {
            self.divisor.0.clone()
        };
        let power = if self.right_exact_rational_kind != ExactRationalKind::NonRational {
            matrix_power4_known_rational(base, exponent.unsigned_abs())
        } else {
            matrix_power4(base, exponent.unsigned_abs())
        };
        Ok(Matrix4(power))
    }
}

#[derive(Clone, Copy, Debug)]
struct Matrix4Facts {
    is_identity: bool,
    is_diagonal: bool,
    is_upper_triangular: bool,
    is_lower_triangular: bool,
    // Cached off-diagonal signal for the affine 3×3 linear block.
    // Enables direct diagonal right-inverse formulas for scale-only affine
    // updates common in transform stacks. As with the 3×3 case, retaining this
    // cheap geometric fact lets later kernels reduce structure before
    // arithmetic; see Yap, "Towards Exact Geometric Computation", 1997.
    linear_is_diagonal: bool,
    // Direction transforms ignore the translation column because w = 0, so this
    // fact deliberately tracks only the 3x3 linear block plus the bottom-row
    // cross terms. Yap's exact-geometric-computation split between points and
    // directions is what makes the cheaper predicate valid.
    direction_linear_is_diagonal: bool,
    // Affine-linear diagonal blocks frequently appear in transform stacks; if the
    // three diagonal scale terms are already known to be nonzero, checked
    // diagonal paths can avoid re-running per-call zero guards.
    affine_linear_diagonal_is_definitely_nonzero: bool,
    is_definitely_dense_for_inverse: bool,
    // Matrix4 transform handles need per-row translation-column zero facts for
    // point/unknown kernels. The top three facts are already computed while
    // classifying diagonal structure, so retain them here instead of probing
    // `m03/m13/m23` again during handle construction. This is the same
    // "classify cheaply, reuse before arithmetic" principle used in exact
    // geometric computation; see Yap, "Towards Exact Geometric Computation",
    // 1997.
    translation_xyz_zero: [bool; 3],
    is_affine: bool,
    is_affine_translation: bool,
}

#[inline]
fn matrix3_facts<B: Backend>(matrix: &[[Scalar<B>; 3]; 3]) -> Matrix3Facts {
    // Collapse 3×3 structural predicates into one cheap scan so downstream
    // dispatch can avoid repeated definite checks in inverse/division hot loops.
    // This is the same retained structure principle in a fixed-size form:
    // compute once, reuse many times.
    let m00_one = matrix[0][0].definitely_one();
    let m01_zero = matrix[0][1].definitely_zero();
    let m02_zero = matrix[0][2].definitely_zero();
    let m10_zero = matrix[1][0].definitely_zero();
    let m11_one = matrix[1][1].definitely_one();
    let m12_zero = matrix[1][2].definitely_zero();
    let m20_zero = matrix[2][0].definitely_zero();
    let m21_zero = matrix[2][1].definitely_zero();
    let m22_one = matrix[2][2].definitely_one();

    let linear_is_diagonal = m01_zero && m10_zero;
    let is_diagonal = m01_zero && m02_zero && m10_zero && m12_zero && m20_zero && m21_zero;
    let is_identity = is_diagonal && m00_one && m11_one && m22_one;
    let is_affine = m20_zero && m21_zero && m22_one;
    // Recompute triangular predicates from the same local structural scan to
    // avoid extra scalar `definitely_zero` probes. For 3×3 matrices, these
    // are just fixed index checks and fall naturally out of the retained
    // local facts.
    // Golub & Van Loan, *Matrix Computations*, formalizes this as cheap
    // factored structure detection for fixed-size triangular kernels.
    let is_upper_triangular = m10_zero && m20_zero && m21_zero;
    let is_lower_triangular = m01_zero && m02_zero && m12_zero;
    // Reuse the retained linear diagonal fact instead of probing m01/m10 a
    // second time. The predicate is identical, but affine 2D transform handles
    // and inverse/division dispatch stay flatter by carrying the cheap
    // structural fact forward.
    let is_affine_translation = is_affine && m00_one && m11_one && linear_is_diagonal;

    Matrix3Facts {
        is_identity,
        is_diagonal,
        is_upper_triangular,
        is_lower_triangular,
        linear_is_diagonal,
        is_affine,
        is_affine_translation,
    }
}

#[inline]
fn matrix4_facts<B: Backend>(matrix: &[[Scalar<B>; 4]; 4]) -> Matrix4Facts {
    // Collapse 4×4 structural predicates plus homogeneous-column facts into one
    // scan. The returned struct is designed for cheap cloning along handle and
    // divide kernels where the same structural facts are queried repeatedly.
    let m00_one = matrix[0][0].definitely_one();
    let m01_zero = matrix[0][1].definitely_zero();
    let m02_zero = matrix[0][2].definitely_zero();
    let m03_zero = matrix[0][3].definitely_zero();
    let m10_zero = matrix[1][0].definitely_zero();
    let m11_one = matrix[1][1].definitely_one();
    let m12_zero = matrix[1][2].definitely_zero();
    let m13_zero = matrix[1][3].definitely_zero();
    let m20_zero = matrix[2][0].definitely_zero();
    let m21_zero = matrix[2][1].definitely_zero();
    let m22_one = matrix[2][2].definitely_one();
    let m23_zero = matrix[2][3].definitely_zero();
    let m30_zero = matrix[3][0].definitely_zero();
    let m31_zero = matrix[3][1].definitely_zero();
    let m32_zero = matrix[3][2].definitely_zero();
    let m33_one = matrix[3][3].definitely_one();
    let is_definitely_dense_for_inverse = matches!(matrix[1][0].zero_status(), ZeroStatus::NonZero)
        && matches!(matrix[0][1].zero_status(), ZeroStatus::NonZero)
        && matches!(matrix[3][0].zero_status(), ZeroStatus::NonZero);

    let linear_is_diagonal = m01_zero && m02_zero && m10_zero && m12_zero && m20_zero && m21_zero;
    let direction_linear_is_diagonal = linear_is_diagonal && m30_zero && m31_zero && m32_zero;
    let is_diagonal = m01_zero
        && m02_zero
        && m03_zero
        && m10_zero
        && m12_zero
        && m13_zero
        && m20_zero
        && m21_zero
        && m23_zero
        && m30_zero
        && m31_zero
        && m32_zero;
    // Recompute triangular predicates from this single structural pass.
    // Re-querying the same zero comparisons inside tiny helper functions adds
    // branchy call overhead with no extra information for fixed-size 4×4
    // schedules.
    // Golub & Van Loan, *Matrix Computations* (4th ed.), §3.6, recommends
    // this retained-facts style before entering O(n³) fallback kernels.
    let is_upper_triangular = m10_zero && m20_zero && m30_zero && m21_zero && m31_zero && m32_zero;
    let is_lower_triangular = m01_zero && m02_zero && m03_zero && m12_zero && m13_zero && m23_zero;
    let is_identity = is_diagonal && m00_one && m11_one && m22_one && m33_one;
    let is_affine = m30_zero && m31_zero && m32_zero && m33_one;
    let is_affine_translation = is_affine && m00_one && m11_one && m22_one && linear_is_diagonal;
    let affine_linear_diagonal_is_definitely_nonzero = if B::FUSE_SIGNED_PRODUCT_SUM {
        matches!(matrix[0][0].zero_status(), ZeroStatus::NonZero)
            && matches!(matrix[1][1].zero_status(), ZeroStatus::NonZero)
            && matches!(matrix[2][2].zero_status(), ZeroStatus::NonZero)
    } else {
        false
    };

    Matrix4Facts {
        is_identity,
        is_diagonal,
        is_upper_triangular,
        is_lower_triangular,
        linear_is_diagonal,
        direction_linear_is_diagonal,
        is_definitely_dense_for_inverse,
        translation_xyz_zero: [m03_zero, m13_zero, m23_zero],
        is_affine,
        is_affine_translation,
        affine_linear_diagonal_is_definitely_nonzero,
    }
}

#[inline]
fn matrix3_facts_assuming_const3<B: Backend, const N: usize>(
    matrix: &[[Scalar<B>; N]; N],
) -> Matrix3Facts {
    // `transform_vector_rhs_ref` is const-generic, so Rust cannot narrow `N`
    // from the surrounding `N == 3` branch enough to call `matrix3_facts`.
    // Keep this bridge local to that wrapper and mirror the fixed-size fact
    // scan without adding an allocation or a temporary matrix copy.
    debug_assert_eq!(N, 3);
    let m00_one = matrix[0][0].definitely_one();
    let m01_zero = matrix[0][1].definitely_zero();
    let m02_zero = matrix[0][2].definitely_zero();
    let m10_zero = matrix[1][0].definitely_zero();
    let m11_one = matrix[1][1].definitely_one();
    let m12_zero = matrix[1][2].definitely_zero();
    let m20_zero = matrix[2][0].definitely_zero();
    let m21_zero = matrix[2][1].definitely_zero();
    let m22_one = matrix[2][2].definitely_one();

    let linear_is_diagonal = m01_zero && m10_zero;
    let is_diagonal = m01_zero && m02_zero && m10_zero && m12_zero && m20_zero && m21_zero;
    let is_identity = is_diagonal && m00_one && m11_one && m22_one;
    let is_affine = m20_zero && m21_zero && m22_one;
    let is_upper_triangular = m10_zero && m20_zero && m21_zero;
    let is_lower_triangular = m01_zero && m02_zero && m12_zero;
    let is_affine_translation = is_affine && m00_one && m11_one && linear_is_diagonal;

    Matrix3Facts {
        is_identity,
        is_diagonal,
        is_upper_triangular,
        is_lower_triangular,
        linear_is_diagonal,
        is_affine,
        is_affine_translation,
    }
}

#[inline]
fn matrix4_facts_assuming_const4<B: Backend, const N: usize>(
    matrix: &[[Scalar<B>; N]; N],
) -> Matrix4Facts {
    // Same const-generic bridge as the 3x3 version. It preserves the one-scan
    // transform dispatch shape without forcing a heap allocation or copying
    // into a fixed-size temporary just to satisfy the type checker.
    debug_assert_eq!(N, 4);
    let m00_one = matrix[0][0].definitely_one();
    let m01_zero = matrix[0][1].definitely_zero();
    let m02_zero = matrix[0][2].definitely_zero();
    let m03_zero = matrix[0][3].definitely_zero();
    let m10_zero = matrix[1][0].definitely_zero();
    let m11_one = matrix[1][1].definitely_one();
    let m12_zero = matrix[1][2].definitely_zero();
    let m13_zero = matrix[1][3].definitely_zero();
    let m20_zero = matrix[2][0].definitely_zero();
    let m21_zero = matrix[2][1].definitely_zero();
    let m22_one = matrix[2][2].definitely_one();
    let m23_zero = matrix[2][3].definitely_zero();
    let m30_zero = matrix[3][0].definitely_zero();
    let m31_zero = matrix[3][1].definitely_zero();
    let m32_zero = matrix[3][2].definitely_zero();
    let m33_one = matrix[3][3].definitely_one();
    let is_definitely_dense_for_inverse = matches!(matrix[1][0].zero_status(), ZeroStatus::NonZero)
        && matches!(matrix[0][1].zero_status(), ZeroStatus::NonZero)
        && matches!(matrix[3][0].zero_status(), ZeroStatus::NonZero);

    let linear_is_diagonal = m01_zero && m02_zero && m10_zero && m12_zero && m20_zero && m21_zero;
    let direction_linear_is_diagonal = linear_is_diagonal && m30_zero && m31_zero && m32_zero;
    let is_diagonal = m01_zero
        && m02_zero
        && m03_zero
        && m10_zero
        && m12_zero
        && m13_zero
        && m20_zero
        && m21_zero
        && m23_zero
        && m30_zero
        && m31_zero
        && m32_zero;
    let is_upper_triangular = m10_zero && m20_zero && m30_zero && m21_zero && m31_zero && m32_zero;
    let is_lower_triangular = m01_zero && m02_zero && m03_zero && m12_zero && m13_zero && m23_zero;
    let is_identity = is_diagonal && m00_one && m11_one && m22_one && m33_one;
    let is_affine = m30_zero && m31_zero && m32_zero && m33_one;
    let is_affine_translation = is_affine && m00_one && m11_one && m22_one && linear_is_diagonal;
    let affine_linear_diagonal_is_definitely_nonzero = if B::FUSE_SIGNED_PRODUCT_SUM {
        matches!(matrix[0][0].zero_status(), ZeroStatus::NonZero)
            && matches!(matrix[1][1].zero_status(), ZeroStatus::NonZero)
            && matches!(matrix[2][2].zero_status(), ZeroStatus::NonZero)
    } else {
        false
    };

    Matrix4Facts {
        is_identity,
        is_diagonal,
        is_upper_triangular,
        is_lower_triangular,
        linear_is_diagonal,
        direction_linear_is_diagonal,
        is_definitely_dense_for_inverse,
        translation_xyz_zero: [m03_zero, m13_zero, m23_zero],
        is_affine,
        is_affine_translation,
        affine_linear_diagonal_is_definitely_nonzero,
    }
}

fn map_array2<B: Backend, const N: usize, F>(
    left: [Scalar<B>; N],
    right: [Scalar<B>; N],
    mut op: F,
) -> [Scalar<B>; N]
where
    F: FnMut(Scalar<B>, Scalar<B>) -> Scalar<B>,
{
    let mut right = right.into_iter();
    left.map(|lhs| op(lhs, right.next().expect("arrays have equal length")))
}

fn map_array_ref<B: Backend, const N: usize, F>(
    left: [Scalar<B>; N],
    right: &[Scalar<B>; N],
    mut op: F,
) -> [Scalar<B>; N]
where
    F: FnMut(Scalar<B>, &Scalar<B>) -> Scalar<B>,
{
    let mut right = right.iter();
    left.map(|lhs| op(lhs, right.next().expect("arrays have equal length")))
}

fn map_matrix2<B: Backend, const N: usize, F>(
    left: [[Scalar<B>; N]; N],
    right: [[Scalar<B>; N]; N],
    mut op: F,
) -> [[Scalar<B>; N]; N]
where
    F: FnMut(Scalar<B>, Scalar<B>) -> Scalar<B>,
{
    let mut right = right.into_iter();
    left.map(|lhs_row| {
        map_array2(
            lhs_row,
            right.next().expect("matrices have equal row counts"),
            &mut op,
        )
    })
}

fn map_matrix_ref<B: Backend, const N: usize, F>(
    left: [[Scalar<B>; N]; N],
    right: &[[Scalar<B>; N]; N],
    mut op: F,
) -> [[Scalar<B>; N]; N]
where
    F: FnMut(Scalar<B>, &Scalar<B>) -> Scalar<B>,
{
    let mut right = right.iter();
    left.map(|lhs_row| {
        map_array_ref(
            lhs_row,
            right.next().expect("matrices have equal row counts"),
            &mut op,
        )
    })
}

fn map_matrix_left_ref<B: Backend, const N: usize, F>(
    left: &[[Scalar<B>; N]; N],
    right: [[Scalar<B>; N]; N],
    mut op: F,
) -> [[Scalar<B>; N]; N]
where
    F: FnMut(&Scalar<B>, Scalar<B>) -> Scalar<B>,
{
    let mut left = left.iter();
    right.map(|rhs_row| {
        let mut left_row = left.next().expect("matrices have equal row counts").iter();
        rhs_row.map(|rhs| op(left_row.next().expect("arrays have equal length"), rhs))
    })
}

#[inline]
fn matrix_power_with<B: Backend, const N: usize, F>(
    base: [[Scalar<B>; N]; N],
    exponent: u32,
    mut multiply: F,
) -> [[Scalar<B>; N]; N]
where
    F: FnMut([[Scalar<B>; N]; N], [[Scalar<B>; N]; N]) -> [[Scalar<B>; N]; N],
{
    // Alternative researched paths for fixed 3x3/4x4 powers included
    // Cayley-Hamilton with Faddeev-LeVerrier characteristic coefficients
    // (Hou, SIAM Review 40(3), 1998, https://doi.org/10.1137/S003614459732076X)
    // and Berkowitz-style division-free characteristic polynomials
    // (Berkowitz, Information Processing Letters 18(3), 1984,
    // https://doi.org/10.1016/0020-0190(84)90018-8). For the small exponents
    // that dominate this crate's matrix benches, those approaches introduce
    // trace/determinant reductions before they can save a multiply. Keep powers
    // on repeated squaring and put the optimization budget into the fixed-size
    // multiply kernels below. 2026-05 targeted Criterion: approx mat3/mat4
    // powi moved from ~144.6/240.8 ns to ~94.5/175.4 ns; hyperreal-from-f64 mat3/mat4
    // powi moved from ~6.30/11.39 us to ~5.98/10.71 us. Hyperreal-rational
    // powi stayed within the normal Criterion noise band, so this keeps
    // hyperreal's per-cell exact-rational denominator schedule.
    //
    // Keep this helper and the fixed multiply helpers inline for downstream
    // benchmark crates. A post-full-suite 200-sample/8s pass found approximate
    // and symbolica borrowed matrix multiply neutral, while hyperreal mat3/mat4
    // borrowed multiply improved by ~4.98%/~4.54% after inlining the helper
    // layers.
    match exponent {
        0 => return identity_array(),
        1 => return base,
        // Low exponents dominate transform/matrix helper use. Unrolling them
        // avoids the generic squaring loop's extra clones and branch work.
        2 => return multiply(base.clone(), base),
        3 => {
            let square = multiply(base.clone(), base.clone());
            return multiply(square, base);
        }
        4 => {
            let square = multiply(base.clone(), base);
            return multiply(square.clone(), square);
        }
        _ => {}
    }

    let mut exp = exponent;
    let mut result = None;
    let mut factor = base;

    while exp > 0 {
        if exp & 1 == 1 {
            result = Some(match result {
                Some(result) => multiply(result, factor.clone()),
                None => factor.clone(),
            });
        }
        exp >>= 1;
        if exp > 0 {
            factor = multiply(factor.clone(), factor);
        }
    }

    result.expect("positive exponent sets at least one result bit")
}

#[inline]
fn matrix_power3<B: Backend>(base: [[Scalar<B>; 3]; 3], exponent: u32) -> [[Scalar<B>; 3]; 3] {
    crate::trace_dispatch!("hyperlattice_matrix", "helper", "matrix-power3-fixed-mul");
    // The hot small positive powers can square the existing base by reference,
    // then consume only the fresh square. This keeps exact matrix powers on
    // object-level reuse instead of cloning the base into hot multiply lanes.
    if exponent == 2 {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "matrix-power3-borrowed-square"
        );
        if B::FUSE_SIGNED_PRODUCT_SUM && matrix3_has_dense_multiply_certificate(&base) {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "matrix-power3-dense-certified-square"
            );
            return multiply_arrays3_dense_ref(&base, &base);
        }
        return multiply_arrays3_ref(&base, &base);
    }
    if exponent == 3 {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "matrix-power3-borrowed-cube"
        );
        if B::FUSE_SIGNED_PRODUCT_SUM && matrix3_has_dense_multiply_certificate(&base) {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "matrix-power3-dense-certified-cube"
            );
            let square = multiply_arrays3_dense_ref(&base, &base);
            return multiply_arrays3_rhs_ref_with_exact_dense_certificate(square, &base);
        }
        let square = multiply_arrays3_ref(&base, &base);
        return multiply_arrays3_rhs_ref(square, &base);
    }
    matrix_power_with(base, exponent, multiply_arrays3::<B>)
}

#[inline]
fn matrix_power4<B: Backend>(base: [[Scalar<B>; 4]; 4], exponent: u32) -> [[Scalar<B>; 4]; 4] {
    crate::trace_dispatch!("hyperlattice_matrix", "helper", "matrix-power4-fixed-mul");
    // Same borrowed square/cube schedule as 3x3; the mat4 powi benchmark is
    // particularly sensitive to avoiding owned base duplication before the
    // fixed multiply kernel has a chance to reuse structural facts.
    if exponent == 2 {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "matrix-power4-borrowed-square"
        );
        if B::FUSE_SIGNED_PRODUCT_SUM && matrix4_has_dense_multiply_certificate(&base) {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "matrix-power4-dense-certified-square"
            );
            return multiply_arrays4_dense_ref(&base, &base);
        }
        return multiply_arrays4_ref(&base, &base);
    }
    if exponent == 3 {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "matrix-power4-borrowed-cube"
        );
        if B::FUSE_SIGNED_PRODUCT_SUM && matrix4_has_dense_multiply_certificate(&base) {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "matrix-power4-dense-certified-cube"
            );
            let square = multiply_arrays4_dense_ref(&base, &base);
            return multiply_arrays4_rhs_ref_with_dense_certificate(square, &base);
        }
        let square = multiply_arrays4_ref(&base, &base);
        return multiply_arrays4_rhs_ref(square, &base);
    }
    matrix_power_with(base, exponent, multiply_arrays4::<B>)
}

#[inline]
fn matrix_power4_known_rational<B: Backend>(
    base: [[Scalar<B>; 4]; 4],
    exponent: u32,
) -> [[Scalar<B>; 4]; 4] {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "matrix-power4-known-rational"
    );
    if B::FUSE_SIGNED_PRODUCT_SUM {
        if exponent == 2 {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "matrix-power4-known-rational-square"
            );
            return multiply_arrays4_dense_known_rational_ref(&base, &base);
        }
        if exponent == 3 {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "matrix-power4-known-rational-cube"
            );
            let square = multiply_arrays4_dense_known_rational_ref(&base, &base);
            return multiply_arrays4_dense_known_rational_ref(&square, &base);
        }
    }
    matrix_power4(base, exponent)
}

fn ordinary_pivot<B: Backend, const N: usize>(
    left: &[[Scalar<B>; N]; N],
    col: usize,
) -> Option<usize> {
    let mut unknown = None;
    match zero_status(&left[col][col]) {
        ZeroStatus::NonZero => return Some(col),
        ZeroStatus::Unknown => unknown = Some(col),
        ZeroStatus::Zero => {}
    }

    for (row, values) in left.iter().enumerate().skip(col + 1) {
        match zero_status(&values[col]) {
            ZeroStatus::NonZero => return Some(row),
            ZeroStatus::Unknown if unknown.is_none() => unknown = Some(row),
            ZeroStatus::Zero | ZeroStatus::Unknown => {}
        }
    }

    unknown
}

fn checked_pivot<B: Backend, const N: usize, F>(
    left: &[[Scalar<B>; N]; N],
    col: usize,
    mut classify: F,
) -> CheckedBlasResult<usize>
where
    F: FnMut(&Scalar<B>) -> ZeroStatus,
{
    let mut has_unknown = false;
    for (row, values) in left.iter().enumerate().skip(col) {
        match classify(&values[col]) {
            ZeroStatus::NonZero => return Ok(row),
            ZeroStatus::Unknown => has_unknown = true,
            ZeroStatus::Zero => {}
        }
    }

    if has_unknown {
        Err(Problem::UnknownZero)
    } else {
        Err(Problem::DivideByZero)
    }
}

fn scale_entry_in_place<B: Backend>(value: &mut Scalar<B>, factor: &Scalar<B>) {
    let current = mem::replace(value, Scalar::zero());
    *value = current.mul_cached(factor);
}

fn subtract_scaled_entry_in_place<B: Backend>(
    value: &mut Scalar<B>,
    pivot: &Scalar<B>,
    factor: &Scalar<B>,
) {
    let current = mem::replace(value, Scalar::zero());
    // Keep both `pivot` and `factor` borrowed. The old form cloned `pivot`
    // before multiplying, which is expensive for hyperreal-backed matrices.
    *value = current - pivot * factor;
}

macro_rules! impl_solve_left_system_fixed {
    (
        $solve_fn:ident,
        $solve_checked_fn:ident,
        $solve_abort_fn:ident,
        $n:expr
    ) => {
        fn $solve_fn<B: Backend>(
            coefficients: [[Scalar<B>; $n]; $n],
            rhs: [[Scalar<B>; $n]; $n],
        ) -> BlasResult<[[Scalar<B>; $n]; $n]> {
            let mut left = coefficients;
            let mut right = rhs;

            for col in 0..$n {
                let Some(pivot) = ordinary_pivot(&left, col) else {
                    return Err(Problem::DivideByZero);
                };
                if pivot != col {
                    left.swap(col, pivot);
                    right.swap(col, pivot);
                }

                // Move the pivot out once so the same matrix slot is already
                // zeroed for row-elimination and we avoid an extra clone for
                // the inverse path. A structural unit-pivot bypass was tested
                // here and reverted: the extra `definitely_one` query regressed
                // mat3/mat4 right-division rows more than it saved in skipped
                // inverses. Keep the straight-line normalization schedule.
                let pivot = mem::replace(&mut left[col][col], Scalar::one());
                let inv_pivot = pivot.inverse()?;
                for i in 0..$n {
                    scale_entry_in_place(&mut right[col][i], &inv_pivot);
                }
                for i in (col + 1)..$n {
                    scale_entry_in_place(&mut left[col][i], &inv_pivot);
                }
                let pivot_left = left[col].clone();
                let pivot_right = right[col].clone();

                for row in 0..$n {
                    if row == col {
                        continue;
                    }
                    // Single precheck keeps one predicate per row; moved-out
                    // factors avoid a redundant zero write for 3x3 and 4x4.
                    if left[row][col].definitely_zero() {
                        continue;
                    }
                    let factor = mem::replace(&mut left[row][col], Scalar::zero());
                    for i in (col + 1)..$n {
                        subtract_scaled_entry_in_place(&mut left[row][i], &pivot_left[i], &factor);
                    }
                    for i in 0..$n {
                        subtract_scaled_entry_in_place(
                            &mut right[row][i],
                            &pivot_right[i],
                            &factor,
                        );
                    }
                }
            }

            Ok(right)
        }

        fn $solve_checked_fn<B: Backend>(
            coefficients: [[Scalar<B>; $n]; $n],
            rhs: [[Scalar<B>; $n]; $n],
        ) -> CheckedBlasResult<[[Scalar<B>; $n]; $n]> {
            let mut left = coefficients;
            let mut right = rhs;

            for col in 0..$n {
                let pivot = checked_pivot(&left, col, zero_status)?;
                if pivot != col {
                    left.swap(col, pivot);
                    right.swap(col, pivot);
                }

                // Keep checked solve on the same move-based pivot schedule as the
                // non-checked variant so checked kernels don’t pay extra slot
                // churn. A failing checked inverse still returns before mutation
                // of result rows beyond the local copy.
                let pivot = mem::replace(&mut left[col][col], Scalar::one());
                let inv_pivot = pivot.inverse()?;
                for i in 0..$n {
                    scale_entry_in_place(&mut right[col][i], &inv_pivot);
                }
                for i in (col + 1)..$n {
                    scale_entry_in_place(&mut left[col][i], &inv_pivot);
                }
                let pivot_left = left[col].clone();
                let pivot_right = right[col].clone();

                for row in 0..$n {
                    if row == col {
                        continue;
                    }
                    // Shared pivot-factor handling avoids the 4x4 branch.
                    if left[row][col].definitely_zero() {
                        continue;
                    }
                    let factor = mem::replace(&mut left[row][col], Scalar::zero());
                    for i in (col + 1)..$n {
                        subtract_scaled_entry_in_place(&mut left[row][i], &pivot_left[i], &factor);
                    }
                    for i in 0..$n {
                        subtract_scaled_entry_in_place(
                            &mut right[row][i],
                            &pivot_right[i],
                            &factor,
                        );
                    }
                }
            }

            Ok(right)
        }

        fn $solve_abort_fn<B: Backend>(
            coefficients: [[Scalar<B>; $n]; $n],
            rhs: [[Scalar<B>; $n]; $n],
            signal: &AbortSignal,
        ) -> CheckedBlasResult<[[Scalar<B>; $n]; $n]> {
            let mut left = coefficients;
            let mut right = rhs;

            for col in 0..$n {
                let pivot =
                    checked_pivot(&left, col, |value| zero_status_with_abort(value, signal))?;
                if pivot != col {
                    left.swap(col, pivot);
                    right.swap(col, pivot);
                }

                let pivot = mem::replace(&mut left[col][col], Scalar::one());
                let inv_pivot = clone_with_abort(&pivot, signal).inverse()?;
                for i in 0..$n {
                    scale_entry_in_place(&mut right[col][i], &inv_pivot);
                }
                for i in (col + 1)..$n {
                    scale_entry_in_place(&mut left[col][i], &inv_pivot);
                }
                let pivot_left = left[col].clone();
                let pivot_right = right[col].clone();

                for row in 0..$n {
                    if row == col {
                        continue;
                    }
                    // Abort-aware path keeps the same precheck/move policy as
                    // ordinary solve. The pivot factor is moved out only when a
                    // full elimination update is needed.
                    if left[row][col].definitely_zero() {
                        continue;
                    }
                    let factor = mem::replace(&mut left[row][col], Scalar::zero());
                    for i in (col + 1)..$n {
                        subtract_scaled_entry_in_place(&mut left[row][i], &pivot_left[i], &factor);
                    }
                    for i in 0..$n {
                        subtract_scaled_entry_in_place(
                            &mut right[row][i],
                            &pivot_right[i],
                            &factor,
                        );
                    }
                }
            }

            Ok(right)
        }
    };
}

impl_solve_left_system_fixed!(
    solve_left_system3,
    solve_left_system3_checked,
    solve_left_system3_checked_with_abort,
    3
);
impl_solve_left_system_fixed!(
    solve_left_system4,
    solve_left_system4_checked,
    solve_left_system4_checked_with_abort,
    4
);

fn prefer_shared_adjugate_right_division<B: Backend, const N: usize>(
    left: &[[Scalar<B>; N]; N],
    right: &[[Scalar<B>; N]; N],
) -> bool {
    // Shared adjugate division trades fewer inverses for more products. That
    // wins for dyadic hyperreal inputs because reduction is shift-only. Modern
    // hyperreal exact-rational reducers also handle some non-dyadic matrix
    // forms with one shared denominator, so keep dyadic as the hot first
    // predicate and isolate the broader exact-rational fallback below.
    // This is the same "delay the common scale" idea as fraction-free exact
    // linear algebra (Bareiss, Math. Comp. 22(103), 1968,
    // https://doi.org/10.2307/2004533), but applied only when traces show the
    // extra products are cheaper than repeated inverses.
    // Check the divisor first. The shared-adjugate branch is only useful when
    // `det(right)` and all adjugate cofactors stay dyadic; decimal divisors can
    // reject the path before scanning the dividend. This preserves the exact
    // same predicate but moves the cheapest likely rejection earlier.
    let right_kind = matrix_exact_rational_kind(right);
    if right_kind == ExactRationalKind::NonRational {
        return false;
    }
    if B::FUSE_SIGNED_PRODUCT_SUM && N == 4 {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-exact-right-skip-left-kind"
        );
        return true;
    }
    if B::FUSE_SIGNED_PRODUCT_SUM && N == 3 && right_kind == ExactRationalKind::ExactRational {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-exact-right-skip-left-kind"
        );
        return true;
    }
    let left_kind = matrix_exact_rational_kind(left);
    matches!(
        combine_exact_rational_kind(left_kind, right_kind),
        ExactRationalKind::ExactDyadicRational | ExactRationalKind::ExactRational
    )
}

fn prefer_shared_adjugate_right_division_ref3<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> bool {
    let right_kind = matrix3_exact_rational_kind(right);
    if right_kind == ExactRationalKind::NonRational {
        return false;
    }
    if B::FUSE_SIGNED_PRODUCT_SUM && right_kind == ExactRationalKind::ExactRational {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-ref-exact-right-skip-left-kind"
        );
        return true;
    }
    let left_kind = matrix3_exact_rational_kind(left);
    matches!(
        combine_exact_rational_kind(left_kind, right_kind),
        ExactRationalKind::ExactDyadicRational | ExactRationalKind::ExactRational
    )
}

#[inline]
fn matrix4_direction_linear_is_diagonal<B: Backend>(matrix: &[[Scalar<B>; 4]; 4]) -> bool {
    // Direction vectors have w = 0, so the translation column cannot contribute
    // to the result. This retained geometric fact lets translated diagonal affine
    // transforms use the same component-wise scale path as true diagonal
    // matrices without changing point or unknown-w behavior. This is the
    // projective point/direction split used in exact geometric computation; see
    // Yap, "Towards Exact Geometric Computation", 1997.
    matrix[0][1].definitely_zero()
        && matrix[0][2].definitely_zero()
        && matrix[1][0].definitely_zero()
        && matrix[1][2].definitely_zero()
        && matrix[2][0].definitely_zero()
        && matrix[2][1].definitely_zero()
        && matrix[3][0].definitely_zero()
        && matrix[3][1].definitely_zero()
        && matrix[3][2].definitely_zero()
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Matrix4DirectionLinearKind {
    Identity,
    Diagonal,
    General,
}

#[inline]
fn matrix4_direction_linear_kind<B: Backend>(
    matrix: &[[Scalar<B>; 4]; 4],
) -> Matrix4DirectionLinearKind {
    // Narrow one-shot direction predicate: translations do not affect
    // homogeneous directions. Classify the one-shot public direction path once
    // and feed the retained result into the transform helper; this avoids the
    // rejected pattern of asking the same zero questions again when the matrix
    // is diagonal but not identity.
    if !matrix4_direction_linear_is_diagonal(matrix) {
        return Matrix4DirectionLinearKind::General;
    }
    if matrix[0][0].definitely_one()
        && matrix[1][1].definitely_one()
        && matrix[2][2].definitely_one()
    {
        Matrix4DirectionLinearKind::Identity
    } else {
        Matrix4DirectionLinearKind::Diagonal
    }
}

#[inline]
fn matrix4_affine_linear_is_diagonal<B: Backend>(matrix: &[[Scalar<B>; 4]; 4]) -> bool {
    // Narrow one-shot point predicate: this is cheaper than `matrix4_facts`
    // when the caller only needs the affine-linear-diagonal fast path. Keep it
    // out of prepared paths, where retained `Matrix4Facts` are already
    // available. Targeted sentinel runs showed the public point transform
    // regressed after broad fact collection, while prepared handles stayed flat.
    // See Yap, "Towards Exact Geometric Computation", 1997.
    matrix[0][1].definitely_zero()
        && matrix[0][2].definitely_zero()
        && matrix[1][0].definitely_zero()
        && matrix[1][2].definitely_zero()
        && matrix[2][0].definitely_zero()
        && matrix[2][1].definitely_zero()
        && matrix[3][0].definitely_zero()
        && matrix[3][1].definitely_zero()
        && matrix[3][2].definitely_zero()
        && matrix[3][3].definitely_one()
}

#[inline]
fn matrix3_is_definitely_dense_for_inverse<B: Backend>(matrix: &[[Scalar<B>; 3]; 3]) -> bool {
    // Dense inverse benchmarks regressed after broad retained-fact scans were
    // added for sparse/affine wins. These three nonzero certificates are a
    // deliberately conservative escape hatch: `m10 != 0` rules out upper
    // triangular, `m01 != 0` rules out lower triangular, and `m20 != 0` rules
    // out affine form; together they also rule out diagonal/identity. When any
    // certificate is unknown, fall back to the full fact scan so exact geometry
    // paths keep their structural reductions. This preserves Yap's object-level
    // structure principle ("Towards Exact Geometric Computation", 1997) while
    // keeping dense cofactor kernels thin.
    matches!(matrix[1][0].zero_status(), ZeroStatus::NonZero)
        && matches!(matrix[0][1].zero_status(), ZeroStatus::NonZero)
        && matches!(matrix[2][0].zero_status(), ZeroStatus::NonZero)
}

#[inline]
fn matrix4_is_definitely_dense_for_inverse<B: Backend>(matrix: &[[Scalar<B>; 4]; 4]) -> bool {
    // Same dense-first guard as the 3x3 path. `m10 != 0` rejects upper
    // triangular, `m01 != 0` rejects lower triangular, and `m30 != 0` rejects
    // affine/homogeneous structure; diagonal and identity are subsets of the
    // triangular/affine structures already ruled out. The guard uses only three
    // cheap structural facts and never approximates, matching the exact
    // geometric-computation rule of exploiting structure only when it is known.
    // See Yap, "Towards Exact Geometric Computation", 1997.
    matches!(matrix[1][0].zero_status(), ZeroStatus::NonZero)
        && matches!(matrix[0][1].zero_status(), ZeroStatus::NonZero)
        && matches!(matrix[3][0].zero_status(), ZeroStatus::NonZero)
}

#[inline]
fn matrix3_has_dense_multiply_certificate<B: Backend>(matrix: &[[Scalar<B>; 3]; 3]) -> bool {
    // A dense cofactor inverse followed by `powi(-2)` is already known at the
    // object level to be on the dense route. Reuse the same three nonzero
    // certificates as the inverse dense guard to select a direct fixed multiply
    // and avoid a full per-lane zero scan. If any certificate is absent, fall
    // back to the sparse-aware multiply.
    matches!(matrix[1][0].zero_status(), ZeroStatus::NonZero)
        && matches!(matrix[0][1].zero_status(), ZeroStatus::NonZero)
        && matches!(matrix[2][0].zero_status(), ZeroStatus::NonZero)
}

#[inline]
fn matrix4_has_dense_multiply_certificate<B: Backend>(matrix: &[[Scalar<B>; 4]; 4]) -> bool {
    // Same narrow certificate as `matrix4_is_definitely_dense_for_inverse`.
    // It is intentionally not a proof that every product lane is nonzero; it is
    // a cheap signal that sparse probing is unlikely to pay for the exact
    // backend's dense inverse square.
    matches!(matrix[1][0].zero_status(), ZeroStatus::NonZero)
        && matches!(matrix[0][1].zero_status(), ZeroStatus::NonZero)
        && matches!(matrix[3][0].zero_status(), ZeroStatus::NonZero)
}

#[inline]
fn multiply_arrays4_ref_with_dense_certificate<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> [[Scalar<B>; 4]; 4] {
    if B::FUSE_SIGNED_PRODUCT_SUM
        && matrix4_has_dense_multiply_certificate(left)
        && matrix4_has_dense_multiply_certificate(right)
    {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply4-dense-certified-ref"
        );
        return multiply_arrays4_dense_ref(left, right);
    }
    multiply_arrays4_ref(left, right)
}

#[inline]
fn multiply_arrays4_rhs_ref_with_dense_certificate<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> [[Scalar<B>; 4]; 4] {
    if B::FUSE_SIGNED_PRODUCT_SUM
        && matrix4_has_dense_multiply_certificate(&left)
        && matrix4_has_dense_multiply_certificate(right)
    {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply4-dense-certified-owned-ref"
        );
        return multiply_arrays4_dense_ref(&left, right);
    }
    multiply_arrays4_rhs_ref(left, right)
}

#[inline]
fn invert_matrix4_affine_linear_diagonal<B: Backend>(
    matrix: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    // For diagonal affine linear blocks, invert is three scalar reciprocals and
    // three affine correction multiplies. This is the 3D axis-aligned case of the
    // block affine inverse used in `Matrix4::` transforms.
    // Inexact numeric code can treat this as a per-axis rescaling; exact code
    // avoids the full 3×3 determinant path and keeps reciprocal scheduling flat.
    // Golub and Van Loan (1977) show that triangular and axis-aligned block
    // inverses reduce to independent diagonal solves before translation.
    let inv00 = matrix[0][0].clone().inverse()?;
    let inv11 = matrix[1][1].clone().inverse()?;
    let inv22 = matrix[2][2].clone().inverse()?;
    let inv_tx = Scalar::zero() - (&matrix[0][3] * &inv00);
    let inv_ty = Scalar::zero() - (&matrix[1][3] * &inv11);
    let inv_tz = Scalar::zero() - (&matrix[2][3] * &inv22);

    Ok([
        [inv00, Scalar::zero(), Scalar::zero(), inv_tx],
        [Scalar::zero(), inv11, Scalar::zero(), inv_ty],
        [Scalar::zero(), Scalar::zero(), inv22, inv_tz],
        [
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
            Scalar::one(),
        ],
    ])
}

#[inline]
fn invert_matrix4_affine_linear_diagonal_checked<B: Backend>(
    matrix: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    require_known_nonzero(&matrix[0][0])?;
    require_known_nonzero(&matrix[1][1])?;
    require_known_nonzero(&matrix[2][2])?;
    invert_matrix4_affine_linear_diagonal(matrix)
}

#[inline]
fn invert_matrix4_affine_linear_diagonal_checked_with_abort<B: Backend>(
    matrix: &[[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    require_known_nonzero_with_abort(&matrix[0][0], signal)?;
    require_known_nonzero_with_abort(&matrix[1][1], signal)?;
    require_known_nonzero_with_abort(&matrix[2][2], signal)?;
    invert_matrix4_affine_linear_diagonal(matrix)
}

#[inline]
fn invert_matrix4_affine<B: Backend>(
    matrix: &[[Scalar<B>; 4]; 4],
    linear_is_diagonal: bool,
    is_affine_translation: bool,
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    // `linear_is_diagonal` and `is_affine_translation` are retained from
    // `Matrix4Facts`; do not re-probe them here. The helper is only entered
    // after the caller proves affine form.
    if linear_is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-affine-linear-diagonal"
        );
        return invert_matrix4_affine_linear_diagonal(matrix);
    }
    if is_affine_translation {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-affine-translation"
        );
        return Ok([
            [
                matrix[0][0].clone(),
                matrix[0][1].clone(),
                matrix[0][2].clone(),
                Scalar::zero() - &matrix[0][3],
            ],
            [
                matrix[1][0].clone(),
                matrix[1][1].clone(),
                matrix[1][2].clone(),
                Scalar::zero() - &matrix[1][3],
            ],
            [
                matrix[2][0].clone(),
                matrix[2][1].clone(),
                matrix[2][2].clone(),
                Scalar::zero() - &matrix[2][3],
            ],
            [
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
                Scalar::one(),
            ],
        ]);
    }
    invert_matrix4_affine_without_translation(matrix)
}

#[inline]
fn invert_matrix4_affine_without_translation<B: Backend>(
    matrix: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    // For affine 4×4 transforms, use the block identity:
    // [R t; 0 1]⁻¹ = [R⁻¹ -R⁻¹ t; 0 1].
    // This keeps the 3×3 linear inverse and one matrix-vector multiply separate from
    // the full 4×4 adjugate schedule and is typically faster for rigid/affine
    // workloads with dense 3×3 structure.
    let linear = [
        [
            matrix[0][0].clone(),
            matrix[0][1].clone(),
            matrix[0][2].clone(),
        ],
        [
            matrix[1][0].clone(),
            matrix[1][1].clone(),
            matrix[1][2].clone(),
        ],
        [
            matrix[2][0].clone(),
            matrix[2][1].clone(),
            matrix[2][2].clone(),
        ],
    ];
    let translation = [
        matrix[0][3].clone(),
        matrix[1][3].clone(),
        matrix[2][3].clone(),
    ];
    let inverse_linear = invert_matrix3(linear)?;

    let inverse_translation: [Scalar<B>; 3] = from_fn(|row| {
        let shifted = affine_translation_dot3(
            [
                &inverse_linear[row][0],
                &inverse_linear[row][1],
                &inverse_linear[row][2],
            ],
            [&translation[0], &translation[1], &translation[2]],
        );
        Scalar::zero() - shifted
    });

    Ok([
        [
            inverse_linear[0][0].clone(),
            inverse_linear[0][1].clone(),
            inverse_linear[0][2].clone(),
            inverse_translation[0].clone(),
        ],
        [
            inverse_linear[1][0].clone(),
            inverse_linear[1][1].clone(),
            inverse_linear[1][2].clone(),
            inverse_translation[1].clone(),
        ],
        [
            inverse_linear[2][0].clone(),
            inverse_linear[2][1].clone(),
            inverse_linear[2][2].clone(),
            inverse_translation[2].clone(),
        ],
        [
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
            Scalar::one(),
        ],
    ])
}

#[inline]
fn invert_matrix4_affine_checked<B: Backend>(
    matrix: &[[Scalar<B>; 4]; 4],
    linear_is_diagonal: bool,
    is_affine_translation: bool,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    if linear_is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-checked-affine-linear-diagonal"
        );
        return invert_matrix4_affine_linear_diagonal_checked(matrix);
    }
    if is_affine_translation {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-checked-affine-translation"
        );
        return Ok([
            [
                matrix[0][0].clone(),
                matrix[0][1].clone(),
                matrix[0][2].clone(),
                Scalar::zero() - &matrix[0][3],
            ],
            [
                matrix[1][0].clone(),
                matrix[1][1].clone(),
                matrix[1][2].clone(),
                Scalar::zero() - &matrix[1][3],
            ],
            [
                matrix[2][0].clone(),
                matrix[2][1].clone(),
                matrix[2][2].clone(),
                Scalar::zero() - &matrix[2][3],
            ],
            [
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
                Scalar::one(),
            ],
        ]);
    }
    invert_matrix4_affine_without_translation_checked(matrix)
}

#[inline]
fn invert_matrix4_affine_without_translation_checked<B: Backend>(
    matrix: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    // Called only when caller already established non-translation affine.
    // Avoids re-running an expensive structural predicate in every hot path.
    // For affine 4×4 transforms, use the block identity:
    // [R t; 0 1]⁻¹ = [R⁻¹ -R⁻¹ t; 0 1].
    // This keeps the 3×3 linear inverse and one matrix-vector multiply separate from
    // the full 4×4 adjugate schedule and is typically faster for rigid/affine
    // workloads with dense 3×3 structure.
    let linear = [
        [
            matrix[0][0].clone(),
            matrix[0][1].clone(),
            matrix[0][2].clone(),
        ],
        [
            matrix[1][0].clone(),
            matrix[1][1].clone(),
            matrix[1][2].clone(),
        ],
        [
            matrix[2][0].clone(),
            matrix[2][1].clone(),
            matrix[2][2].clone(),
        ],
    ];
    let translation = [
        matrix[0][3].clone(),
        matrix[1][3].clone(),
        matrix[2][3].clone(),
    ];
    let inverse_linear = invert_matrix3_checked(linear)?;

    let inverse_translation: [Scalar<B>; 3] = from_fn(|row| {
        let shifted = affine_translation_dot3(
            [
                &inverse_linear[row][0],
                &inverse_linear[row][1],
                &inverse_linear[row][2],
            ],
            [&translation[0], &translation[1], &translation[2]],
        );
        Scalar::zero() - shifted
    });

    Ok([
        [
            inverse_linear[0][0].clone(),
            inverse_linear[0][1].clone(),
            inverse_linear[0][2].clone(),
            inverse_translation[0].clone(),
        ],
        [
            inverse_linear[1][0].clone(),
            inverse_linear[1][1].clone(),
            inverse_linear[1][2].clone(),
            inverse_translation[1].clone(),
        ],
        [
            inverse_linear[2][0].clone(),
            inverse_linear[2][1].clone(),
            inverse_linear[2][2].clone(),
            inverse_translation[2].clone(),
        ],
        [
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
            Scalar::one(),
        ],
    ])
}

#[inline]
fn invert_matrix4_affine_checked_with_abort<B: Backend>(
    matrix: &[[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
    linear_is_diagonal: bool,
    is_affine_translation: bool,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    if linear_is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-checked-with-abort-affine-linear-diagonal"
        );
        return invert_matrix4_affine_linear_diagonal_checked_with_abort(matrix, signal);
    }
    if is_affine_translation {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-checked-affine-translation"
        );
        return Ok([
            [
                matrix[0][0].clone(),
                matrix[0][1].clone(),
                matrix[0][2].clone(),
                Scalar::zero() - &matrix[0][3],
            ],
            [
                matrix[1][0].clone(),
                matrix[1][1].clone(),
                matrix[1][2].clone(),
                Scalar::zero() - &matrix[1][3],
            ],
            [
                matrix[2][0].clone(),
                matrix[2][1].clone(),
                matrix[2][2].clone(),
                Scalar::zero() - &matrix[2][3],
            ],
            [
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
                Scalar::one(),
            ],
        ]);
    }
    invert_matrix4_affine_without_translation_checked_with_abort(matrix, signal)
}

#[inline]
fn invert_matrix4_affine_without_translation_checked_with_abort<B: Backend>(
    matrix: &[[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    // Called only when caller already established non-translation affine.
    // Avoids re-running an expensive structural predicate in every hot path.
    // For affine 4×4 transforms, use the block identity:
    // [R t; 0 1]⁻¹ = [R⁻¹ -R⁻¹ t; 0 1].
    // This keeps the 3×3 linear inverse and one matrix-vector multiply separate from
    // the full 4×4 adjugate schedule and is typically faster for rigid/affine
    // workloads with dense 3×3 structure.
    let linear = [
        [
            matrix[0][0].clone(),
            matrix[0][1].clone(),
            matrix[0][2].clone(),
        ],
        [
            matrix[1][0].clone(),
            matrix[1][1].clone(),
            matrix[1][2].clone(),
        ],
        [
            matrix[2][0].clone(),
            matrix[2][1].clone(),
            matrix[2][2].clone(),
        ],
    ];
    let translation = [
        matrix[0][3].clone(),
        matrix[1][3].clone(),
        matrix[2][3].clone(),
    ];
    let inverse_linear = invert_matrix3_checked_with_abort(linear, signal)?;

    let inverse_translation: [Scalar<B>; 3] = from_fn(|row| {
        let shifted = affine_translation_dot3(
            [
                &inverse_linear[row][0],
                &inverse_linear[row][1],
                &inverse_linear[row][2],
            ],
            [&translation[0], &translation[1], &translation[2]],
        );
        Scalar::zero() - shifted
    });

    Ok([
        [
            inverse_linear[0][0].clone(),
            inverse_linear[0][1].clone(),
            inverse_linear[0][2].clone(),
            inverse_translation[0].clone(),
        ],
        [
            inverse_linear[1][0].clone(),
            inverse_linear[1][1].clone(),
            inverse_linear[1][2].clone(),
            inverse_translation[1].clone(),
        ],
        [
            inverse_linear[2][0].clone(),
            inverse_linear[2][1].clone(),
            inverse_linear[2][2].clone(),
            inverse_translation[2].clone(),
        ],
        [
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
            Scalar::one(),
        ],
    ])
}

#[inline]
fn divide_matrix4_by_affine_linear_diagonal<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    // For affine with a diagonal 3×3 linear block, right-division is diagonal
    // scaling in each axis plus three correction terms for translation. This is
    // the same row-wise specialization used by affine point transforms and avoids
    // the generic 3×3 inversion inside the hot mat4 divide path.
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    let inv_a22 = right[2][2].clone().inverse()?;
    let inv_tx = Scalar::zero() - (&right[0][3] * &inv_a00);
    let inv_ty = Scalar::zero() - (&right[1][3] * &inv_a11);
    let inv_tz = Scalar::zero() - (&right[2][3] * &inv_a22);
    Ok([
        [
            {
                let row = left[0][0].clone();
                let row = row * &inv_a00;
                row
            },
            {
                let row = left[0][1].clone();
                let row = row * &inv_a11;
                row
            },
            {
                let row = left[0][2].clone();
                let row = row * &inv_a22;
                row
            },
            {
                let x = left[0][0].clone();
                let y = left[0][1].clone();
                let z = left[0][2].clone();
                left[0][3].clone() + (&(x * &inv_tx) + &((y * &inv_ty) + &(z * &inv_tz)))
            },
        ],
        [
            {
                let row = left[1][0].clone();
                let row = row * &inv_a00;
                row
            },
            {
                let row = left[1][1].clone();
                let row = row * &inv_a11;
                row
            },
            {
                let row = left[1][2].clone();
                let row = row * &inv_a22;
                row
            },
            {
                let x = left[1][0].clone();
                let y = left[1][1].clone();
                let z = left[1][2].clone();
                left[1][3].clone() + (&(x * &inv_tx) + &((y * &inv_ty) + &(z * &inv_tz)))
            },
        ],
        [
            {
                let row = left[2][0].clone();
                let row = row * &inv_a00;
                row
            },
            {
                let row = left[2][1].clone();
                let row = row * &inv_a11;
                row
            },
            {
                let row = left[2][2].clone();
                let row = row * &inv_a22;
                row
            },
            {
                let x = left[2][0].clone();
                let y = left[2][1].clone();
                let z = left[2][2].clone();
                left[2][3].clone() + (&(x * &inv_tx) + &((y * &inv_ty) + &(z * &inv_tz)))
            },
        ],
        [
            {
                let row = left[3][0].clone();
                let row = row * &inv_a00;
                row
            },
            {
                let row = left[3][1].clone();
                let row = row * &inv_a11;
                row
            },
            {
                let row = left[3][2].clone();
                let row = row * &inv_a22;
                row
            },
            {
                let x = left[3][0].clone();
                let y = left[3][1].clone();
                let z = left[3][2].clone();
                left[3][3].clone() + (&(x * &inv_tx) + &((y * &inv_ty) + &(z * &inv_tz)))
            },
        ],
    ])
}

#[inline]
fn divide_matrix4_by_affine_linear_diagonal_checked<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    require_known_nonzero(&right[2][2])?;
    divide_matrix4_by_affine_linear_diagonal(left, right)
}

#[inline]
fn divide_matrix4_by_affine_linear_diagonal_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    require_known_nonzero_with_abort(&right[0][0], signal)?;
    require_known_nonzero_with_abort(&right[1][1], signal)?;
    require_known_nonzero_with_abort(&right[2][2], signal)?;
    divide_matrix4_by_affine_linear_diagonal(left, right)
}

#[inline]
fn divide_matrix4_by_affine_linear_diagonal_ref<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    let inv_a22 = right[2][2].clone().inverse()?;
    let inv_tx = Scalar::zero() - (&right[0][3] * &inv_a00);
    let inv_ty = Scalar::zero() - (&right[1][3] * &inv_a11);
    let inv_tz = Scalar::zero() - (&right[2][3] * &inv_a22);
    Ok([
        [
            {
                let row = left[0][0].clone();
                let row = row * &inv_a00;
                row
            },
            {
                let row = left[0][1].clone();
                let row = row * &inv_a11;
                row
            },
            {
                let row = left[0][2].clone();
                let row = row * &inv_a22;
                row
            },
            {
                let x = left[0][0].clone();
                let y = left[0][1].clone();
                let z = left[0][2].clone();
                left[0][3].clone() + (&(x * &inv_tx) + &((y * &inv_ty) + &(z * &inv_tz)))
            },
        ],
        [
            {
                let row = left[1][0].clone();
                let row = row * &inv_a00;
                row
            },
            {
                let row = left[1][1].clone();
                let row = row * &inv_a11;
                row
            },
            {
                let row = left[1][2].clone();
                let row = row * &inv_a22;
                row
            },
            {
                let x = left[1][0].clone();
                let y = left[1][1].clone();
                let z = left[1][2].clone();
                left[1][3].clone() + (&(x * &inv_tx) + &((y * &inv_ty) + &(z * &inv_tz)))
            },
        ],
        [
            {
                let row = left[2][0].clone();
                let row = row * &inv_a00;
                row
            },
            {
                let row = left[2][1].clone();
                let row = row * &inv_a11;
                row
            },
            {
                let row = left[2][2].clone();
                let row = row * &inv_a22;
                row
            },
            {
                let x = left[2][0].clone();
                let y = left[2][1].clone();
                let z = left[2][2].clone();
                left[2][3].clone() + (&(x * &inv_tx) + &((y * &inv_ty) + &(z * &inv_tz)))
            },
        ],
        [
            {
                let row = left[3][0].clone();
                let row = row * &inv_a00;
                row
            },
            {
                let row = left[3][1].clone();
                let row = row * &inv_a11;
                row
            },
            {
                let row = left[3][2].clone();
                let row = row * &inv_a22;
                row
            },
            {
                let x = left[3][0].clone();
                let y = left[3][1].clone();
                let z = left[3][2].clone();
                left[3][3].clone() + (&(x * &inv_tx) + &((y * &inv_ty) + &(z * &inv_tz)))
            },
        ],
    ])
}

fn divide_matrix4_affine_by_affine_linear_diagonal<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    // For affine-by-affine, the same diagonal affine formula keeps the bottom
    // homogeneous row exact while collapsing the core to three inverse scalars.
    divide_matrix4_by_affine_linear_diagonal(left, right)
}

#[inline]
fn divide_matrix4_affine_by_affine_ref_linear_diagonal<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    divide_matrix4_by_affine_linear_diagonal_ref(left, right)
}

#[inline]
fn divide_matrix4_affine_by_affine_linear_diagonal_checked<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    divide_matrix4_by_affine_linear_diagonal_checked(left, right)
}

#[inline]
fn divide_matrix4_affine_by_affine_linear_diagonal_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    divide_matrix4_by_affine_linear_diagonal_checked_with_abort(left, right, signal)
}

#[inline]
fn affine_translation_column_update<B: Backend>(
    row: &[Scalar<B>; 4],
    inverse_translation: &[Scalar<B>; 3],
) -> Scalar<B> {
    // Right division by a translation-only affine matrix updates only the
    // homogeneous column. Route the 3-term dot through `linear_combination3`
    // rather than spelling out three multiplies and two adds so exact backends
    // can delay denominator/canonicalization work inside the short polynomial.
    // This is the fixed-size form of fraction-free/delayed-normalization
    // arithmetic (Bareiss, Math. Comp. 22(103), 1968,
    // <https://doi.org/10.2307/2004533>).
    let matrix_terms = [&row[0], &row[1], &row[2]];
    let translation_terms = [
        &inverse_translation[0],
        &inverse_translation[1],
        &inverse_translation[2],
    ];
    row[3].clone() + Scalar::linear_combination3(matrix_terms, translation_terms)
}

#[inline]
fn affine_translation_column_subtract_update<B: Backend>(
    row: &[Scalar<B>; 4],
    translation: [&Scalar<B>; 3],
) -> Scalar<B> {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "affine-translation-column-subtract"
    );
    let shifted = affine_translation_dot3([&row[0], &row[1], &row[2]], translation);
    row[3].clone() - shifted
}

#[inline]
fn affine_translation_dot3<B: Backend>(
    coefficients: [&Scalar<B>; 3],
    values: [&Scalar<B>; 3],
) -> Scalar<B> {
    if B::FUSE_SIGNED_PRODUCT_SUM {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "affine-translation-dot3-active-exact"
        );
        Scalar::active_linear_combination3(coefficients, values)
    } else {
        (coefficients[0] * values[0]) + &(coefficients[1] * values[1] + coefficients[2] * values[2])
    }
}

#[inline]
fn divide_matrix4_by_affine_no_translation<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    let inverse = invert_matrix4_affine_without_translation(right)?;
    Ok(multiply_arrays4(left, inverse))
}

#[inline]
fn divide_matrix4_affine_by_affine_no_translation<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    // For affine-by-affine right-division, keep the linear 3×3 block explicit and
    // avoid building the full 4×4 inverse product. The composition
    // [Rₗ tₗ; 0 1] / [Rᵣ tᵣ; 0 1] = [Rₗ Rᵣ⁻¹  tₗ + Rₗ(-Rᵣ⁻¹ tᵣ); 0 1].
    // This avoids extra multiplies in the homogeneous row and tends to be cheaper
    // for scene transforms than a full 4×4 multiply.
    let left_linear = [
        [left[0][0].clone(), left[0][1].clone(), left[0][2].clone()],
        [left[1][0].clone(), left[1][1].clone(), left[1][2].clone()],
        [left[2][0].clone(), left[2][1].clone(), left[2][2].clone()],
    ];
    let right_linear = [
        [
            right[0][0].clone(),
            right[0][1].clone(),
            right[0][2].clone(),
        ],
        [
            right[1][0].clone(),
            right[1][1].clone(),
            right[1][2].clone(),
        ],
        [
            right[2][0].clone(),
            right[2][1].clone(),
            right[2][2].clone(),
        ],
    ];
    let right_translation = [
        right[0][3].clone(),
        right[1][3].clone(),
        right[2][3].clone(),
    ];
    let right_inverse_linear = invert_matrix3(right_linear)?;
    let right_inverse_translation: [Scalar<B>; 3] = from_fn(|row| {
        let shifted = affine_translation_dot3(
            [
                &right_inverse_linear[row][0],
                &right_inverse_linear[row][1],
                &right_inverse_linear[row][2],
            ],
            [
                &right_translation[0],
                &right_translation[1],
                &right_translation[2],
            ],
        );
        Scalar::zero() - shifted
    });
    let linear = multiply_arrays3_affine_linear_with_exact_dense_certificate(
        left_linear,
        right_inverse_linear,
    );
    let translation: [Scalar<B>; 3] = from_fn(|row| {
        let shifted = affine_translation_dot3(
            [&left[row][0], &left[row][1], &left[row][2]],
            [
                &right_inverse_translation[0],
                &right_inverse_translation[1],
                &right_inverse_translation[2],
            ],
        );
        left[row][3].clone() + shifted
    });
    Ok([
        [
            linear[0][0].clone(),
            linear[0][1].clone(),
            linear[0][2].clone(),
            translation[0].clone(),
        ],
        [
            linear[1][0].clone(),
            linear[1][1].clone(),
            linear[1][2].clone(),
            translation[1].clone(),
        ],
        [
            linear[2][0].clone(),
            linear[2][1].clone(),
            linear[2][2].clone(),
            translation[2].clone(),
        ],
        [
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
            Scalar::one(),
        ],
    ])
}

#[inline]
fn divide_matrix4_by_affine_translation<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    // Right-dividing by translation-only affine uses homogeneous column update only:
    // M·[I t;0 1]⁻¹ = M·[I -t;0 1].
    if B::FUSE_SIGNED_PRODUCT_SUM {
        let translation = [&right[0][3], &right[1][3], &right[2][3]];
        return Ok([
            [
                left[0][0].clone(),
                left[0][1].clone(),
                left[0][2].clone(),
                affine_translation_column_subtract_update(&left[0], translation),
            ],
            [
                left[1][0].clone(),
                left[1][1].clone(),
                left[1][2].clone(),
                affine_translation_column_subtract_update(&left[1], translation),
            ],
            [
                left[2][0].clone(),
                left[2][1].clone(),
                left[2][2].clone(),
                affine_translation_column_subtract_update(&left[2], translation),
            ],
            [
                left[3][0].clone(),
                left[3][1].clone(),
                left[3][2].clone(),
                affine_translation_column_subtract_update(&left[3], translation),
            ],
        ]);
    }

    let inverse_translation = [
        Scalar::zero() - &right[0][3],
        Scalar::zero() - &right[1][3],
        Scalar::zero() - &right[2][3],
    ];
    Ok([
        [
            left[0][0].clone(),
            left[0][1].clone(),
            left[0][2].clone(),
            affine_translation_column_update(&left[0], &inverse_translation),
        ],
        [
            left[1][0].clone(),
            left[1][1].clone(),
            left[1][2].clone(),
            affine_translation_column_update(&left[1], &inverse_translation),
        ],
        [
            left[2][0].clone(),
            left[2][1].clone(),
            left[2][2].clone(),
            affine_translation_column_update(&left[2], &inverse_translation),
        ],
        [
            left[3][0].clone(),
            left[3][1].clone(),
            left[3][2].clone(),
            affine_translation_column_update(&left[3], &inverse_translation),
        ],
    ])
}

#[inline]
fn multiply_arrays3_affine_linear_with_exact_dense_certificate<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: [[Scalar<B>; 3]; 3],
) -> [[Scalar<B>; 3]; 3] {
    if B::FUSE_SIGNED_PRODUCT_SUM
        && matrix3_has_dense_multiply_certificate(&left)
        && matrix3_has_dense_multiply_certificate(&right)
    {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply3-affine-linear-dense-certified-exact"
        );
        return multiply_arrays3_dense_ref(&left, &right);
    }
    multiply_arrays3(left, right)
}

#[inline]
fn divide_matrix4_affine_by_affine_translation<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    // For affine-by-affine with translation-only divisor, the linear basis is unchanged.
    if B::FUSE_SIGNED_PRODUCT_SUM {
        let translation = [&right[0][3], &right[1][3], &right[2][3]];
        return Ok([
            [
                left[0][0].clone(),
                left[0][1].clone(),
                left[0][2].clone(),
                affine_translation_column_subtract_update(&left[0], translation),
            ],
            [
                left[1][0].clone(),
                left[1][1].clone(),
                left[1][2].clone(),
                affine_translation_column_subtract_update(&left[1], translation),
            ],
            [
                left[2][0].clone(),
                left[2][1].clone(),
                left[2][2].clone(),
                affine_translation_column_subtract_update(&left[2], translation),
            ],
            [
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
                Scalar::one(),
            ],
        ]);
    }

    let inverse_translation = [
        Scalar::zero() - &right[0][3],
        Scalar::zero() - &right[1][3],
        Scalar::zero() - &right[2][3],
    ];
    Ok([
        [
            left[0][0].clone(),
            left[0][1].clone(),
            left[0][2].clone(),
            affine_translation_column_update(&left[0], &inverse_translation),
        ],
        [
            left[1][0].clone(),
            left[1][1].clone(),
            left[1][2].clone(),
            affine_translation_column_update(&left[1], &inverse_translation),
        ],
        [
            left[2][0].clone(),
            left[2][1].clone(),
            left[2][2].clone(),
            affine_translation_column_update(&left[2], &inverse_translation),
        ],
        [
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
            Scalar::one(),
        ],
    ])
}

#[inline]
fn divide_matrix4_by_affine_checked<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    divide_matrix4_by_affine_no_translation_checked(left, right)
}

#[inline]
fn divide_matrix4_by_affine_checked_assumed_affine_translation<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    // Structural fact is prevalidated by the caller; avoid re-testing `right`.
    divide_matrix4_by_affine_checked_assuming_affine_translation(left, right)
}

#[inline]
fn divide_matrix4_by_affine_checked_assuming_affine_translation<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    // Structural fact is already established by caller to avoid duplicate checks in
    // this checked hot path.
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "right-divide4-checked-by-affine-translation"
    );
    Ok(divide_matrix4_by_affine_translation(left, right)?)
}

#[inline]
fn divide_matrix4_by_affine_no_translation_checked<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    let inverse = invert_matrix4_affine_without_translation_checked(right)?;
    Ok(multiply_arrays4(left, inverse))
}

#[inline]
fn divide_matrix4_affine_by_affine_checked<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    divide_matrix4_affine_by_affine_no_translation_checked(left, right)
}

#[inline]
fn divide_matrix4_affine_by_affine_checked_assumed_affine_translation<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    // Caller already proved the right divisor is translation-only affine.
    divide_matrix4_affine_by_affine_checked_assuming_affine_translation(left, right)
}

fn divide_matrix4_affine_by_affine_checked_assuming_affine_translation<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    // Right divisor is known translation-only affine; skip repeated structural checks.
    // This follows standard affine composition algebra for translation-only linear maps.
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "right-divide4-checked-affine-by-affine-translation"
    );
    Ok(divide_matrix4_affine_by_affine_translation(left, right)?)
}

#[inline]
fn divide_matrix4_affine_by_affine_no_translation_checked<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    let left_linear = [
        [left[0][0].clone(), left[0][1].clone(), left[0][2].clone()],
        [left[1][0].clone(), left[1][1].clone(), left[1][2].clone()],
        [left[2][0].clone(), left[2][1].clone(), left[2][2].clone()],
    ];
    let right_linear = [
        [
            right[0][0].clone(),
            right[0][1].clone(),
            right[0][2].clone(),
        ],
        [
            right[1][0].clone(),
            right[1][1].clone(),
            right[1][2].clone(),
        ],
        [
            right[2][0].clone(),
            right[2][1].clone(),
            right[2][2].clone(),
        ],
    ];
    let right_translation = [
        right[0][3].clone(),
        right[1][3].clone(),
        right[2][3].clone(),
    ];
    let right_inverse_linear = invert_matrix3_checked(right_linear)?;
    let right_inverse_translation: [Scalar<B>; 3] = from_fn(|row| {
        let shifted = affine_translation_dot3(
            [
                &right_inverse_linear[row][0],
                &right_inverse_linear[row][1],
                &right_inverse_linear[row][2],
            ],
            [
                &right_translation[0],
                &right_translation[1],
                &right_translation[2],
            ],
        );
        Scalar::zero() - shifted
    });
    let linear = multiply_arrays3_affine_linear_with_exact_dense_certificate(
        left_linear,
        right_inverse_linear,
    );
    let translation: [Scalar<B>; 3] = from_fn(|row| {
        let shifted = affine_translation_dot3(
            [&left[row][0], &left[row][1], &left[row][2]],
            [
                &right_inverse_translation[0],
                &right_inverse_translation[1],
                &right_inverse_translation[2],
            ],
        );
        left[row][3].clone() + shifted
    });
    Ok([
        [
            linear[0][0].clone(),
            linear[0][1].clone(),
            linear[0][2].clone(),
            translation[0].clone(),
        ],
        [
            linear[1][0].clone(),
            linear[1][1].clone(),
            linear[1][2].clone(),
            translation[1].clone(),
        ],
        [
            linear[2][0].clone(),
            linear[2][1].clone(),
            linear[2][2].clone(),
            translation[2].clone(),
        ],
        [
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
            Scalar::one(),
        ],
    ])
}

#[inline]
fn divide_matrix4_by_affine_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    divide_matrix4_by_affine_no_translation_checked_with_abort(left, right, signal)
}

#[inline]
fn divide_matrix4_by_affine_checked_with_abort_assumed_affine_translation<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    // Caller already established translation-only affine divisor; skip recompute.
    divide_matrix4_by_affine_checked_with_abort_assuming_affine_translation(left, right, signal)
}

#[inline]
fn divide_matrix4_by_affine_checked_with_abort_assuming_affine_translation<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
    _signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    // Abort signal is unused because translation-only affine divisors are
    // guaranteed nonsingular (determinant = 1), so early abort is never needed.
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "right-divide4-checked-abort-by-affine-translation"
    );
    Ok(divide_matrix4_by_affine_translation(left, right)?)
}

#[inline]
fn divide_matrix4_by_affine_no_translation_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    let inverse = invert_matrix4_affine_without_translation_checked_with_abort(right, signal)?;
    Ok(multiply_arrays4(left, inverse))
}

#[inline]
fn divide_matrix4_affine_by_affine_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    divide_matrix4_affine_by_affine_no_translation_checked_with_abort(left, right, signal)
}

#[inline]
fn divide_matrix4_affine_by_affine_checked_with_abort_assumed_affine_translation<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    // Affine-by-affine translation-only divisor fact is caller-proven.
    divide_matrix4_affine_by_affine_checked_with_abort_assuming_affine_translation(
        left, right, signal,
    )
}

#[inline]
fn divide_matrix4_affine_by_affine_checked_with_abort_assuming_affine_translation<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
    _signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    // Same structural optimization as above; no abort checks are needed when the
    // right affine divisor is guaranteed to be translation-only.
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "right-divide4-checked-abort-affine-by-affine-translation"
    );
    Ok(divide_matrix4_affine_by_affine_translation(left, right)?)
}

#[inline]
fn divide_matrix4_affine_by_affine_no_translation_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    let left_linear = [
        [left[0][0].clone(), left[0][1].clone(), left[0][2].clone()],
        [left[1][0].clone(), left[1][1].clone(), left[1][2].clone()],
        [left[2][0].clone(), left[2][1].clone(), left[2][2].clone()],
    ];
    let right_linear = [
        [
            right[0][0].clone(),
            right[0][1].clone(),
            right[0][2].clone(),
        ],
        [
            right[1][0].clone(),
            right[1][1].clone(),
            right[1][2].clone(),
        ],
        [
            right[2][0].clone(),
            right[2][1].clone(),
            right[2][2].clone(),
        ],
    ];
    let right_translation = [
        right[0][3].clone(),
        right[1][3].clone(),
        right[2][3].clone(),
    ];
    let right_inverse_linear = invert_matrix3_checked_with_abort(right_linear, signal)?;
    let right_inverse_translation: [Scalar<B>; 3] = from_fn(|row| {
        let shifted = affine_translation_dot3(
            [
                &right_inverse_linear[row][0],
                &right_inverse_linear[row][1],
                &right_inverse_linear[row][2],
            ],
            [
                &right_translation[0],
                &right_translation[1],
                &right_translation[2],
            ],
        );
        Scalar::zero() - shifted
    });
    let linear = multiply_arrays3_affine_linear_with_exact_dense_certificate(
        left_linear,
        right_inverse_linear,
    );
    let translation: [Scalar<B>; 3] = from_fn(|row| {
        let shifted = affine_translation_dot3(
            [&left[row][0], &left[row][1], &left[row][2]],
            [
                &right_inverse_translation[0],
                &right_inverse_translation[1],
                &right_inverse_translation[2],
            ],
        );
        left[row][3].clone() + shifted
    });
    Ok([
        [
            linear[0][0].clone(),
            linear[0][1].clone(),
            linear[0][2].clone(),
            translation[0].clone(),
        ],
        [
            linear[1][0].clone(),
            linear[1][1].clone(),
            linear[1][2].clone(),
            translation[1].clone(),
        ],
        [
            linear[2][0].clone(),
            linear[2][1].clone(),
            linear[2][2].clone(),
            translation[2].clone(),
        ],
        [
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
            Scalar::one(),
        ],
    ])
}

fn divide_matrix4_by_affine_ref_assumed_affine_translation<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    // Caller has already proven translation-only affine structure.
    divide_matrix4_by_affine_ref_translation(left, right)
}

fn divide_matrix4_by_affine_ref_translation<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    // Borrowed special-case for right-division by translation-only affine.
    // Using the prevalidated structural fact avoids rebuilding the full
    // translation column for the non-affine path and keeps this helper on the
    // same arithmetic schedule as the owned version.
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "right-divide4-ref-by-affine-translation"
    );
    let inverse_translation = [
        Scalar::zero() - &right[0][3],
        Scalar::zero() - &right[1][3],
        Scalar::zero() - &right[2][3],
    ];
    Ok([
        [
            left[0][0].clone(),
            left[0][1].clone(),
            left[0][2].clone(),
            affine_translation_column_update(&left[0], &inverse_translation),
        ],
        [
            left[1][0].clone(),
            left[1][1].clone(),
            left[1][2].clone(),
            affine_translation_column_update(&left[1], &inverse_translation),
        ],
        [
            left[2][0].clone(),
            left[2][1].clone(),
            left[2][2].clone(),
            affine_translation_column_update(&left[2], &inverse_translation),
        ],
        [
            left[3][0].clone(),
            left[3][1].clone(),
            left[3][2].clone(),
            affine_translation_column_update(&left[3], &inverse_translation),
        ],
    ])
}

#[inline]
fn divide_matrix4_by_affine_ref_no_translation<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    let inverse = invert_matrix4_affine_without_translation(right)?;
    Ok(multiply_arrays4_ref(left, &inverse))
}

#[inline]
fn divide_matrix4_affine_by_affine_ref_no_translation<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    // Borrowed affine-by-affine fast path with the right translation fact already
    // known false (non-translation). This keeps multiplication on the linear 3×3
    // block and avoids constructing a full owned copy of `left` up front.
    let left_linear = [
        [left[0][0].clone(), left[0][1].clone(), left[0][2].clone()],
        [left[1][0].clone(), left[1][1].clone(), left[1][2].clone()],
        [left[2][0].clone(), left[2][1].clone(), left[2][2].clone()],
    ];
    let right_linear = [
        [
            right[0][0].clone(),
            right[0][1].clone(),
            right[0][2].clone(),
        ],
        [
            right[1][0].clone(),
            right[1][1].clone(),
            right[1][2].clone(),
        ],
        [
            right[2][0].clone(),
            right[2][1].clone(),
            right[2][2].clone(),
        ],
    ];
    let right_translation = [
        right[0][3].clone(),
        right[1][3].clone(),
        right[2][3].clone(),
    ];
    let right_inverse_linear = invert_matrix3(right_linear)?;
    let right_inverse_translation: [Scalar<B>; 3] = from_fn(|row| {
        let shifted = affine_translation_dot3(
            [
                &right_inverse_linear[row][0],
                &right_inverse_linear[row][1],
                &right_inverse_linear[row][2],
            ],
            [
                &right_translation[0],
                &right_translation[1],
                &right_translation[2],
            ],
        );
        Scalar::zero() - shifted
    });
    let linear = multiply_arrays3_affine_linear_with_exact_dense_certificate(
        left_linear,
        right_inverse_linear,
    );
    let translation: [Scalar<B>; 3] = from_fn(|row| {
        let shifted = affine_translation_dot3(
            [&left[row][0], &left[row][1], &left[row][2]],
            [
                &right_inverse_translation[0],
                &right_inverse_translation[1],
                &right_inverse_translation[2],
            ],
        );
        left[row][3].clone() + shifted
    });
    Ok([
        [
            linear[0][0].clone(),
            linear[0][1].clone(),
            linear[0][2].clone(),
            translation[0].clone(),
        ],
        [
            linear[1][0].clone(),
            linear[1][1].clone(),
            linear[1][2].clone(),
            translation[1].clone(),
        ],
        [
            linear[2][0].clone(),
            linear[2][1].clone(),
            linear[2][2].clone(),
            translation[2].clone(),
        ],
        [
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
            Scalar::one(),
        ],
    ])
}

#[inline]
fn divide_matrix4_affine_by_affine_ref_translation<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    // Borrowed translation-only affine-by-affine special case avoids re-checking
    // right translation when caller already asserted it once.
    let inverse_translation = [
        Scalar::zero() - &right[0][3],
        Scalar::zero() - &right[1][3],
        Scalar::zero() - &right[2][3],
    ];
    Ok([
        [
            left[0][0].clone(),
            left[0][1].clone(),
            left[0][2].clone(),
            affine_translation_column_update(&left[0], &inverse_translation),
        ],
        [
            left[1][0].clone(),
            left[1][1].clone(),
            left[1][2].clone(),
            affine_translation_column_update(&left[1], &inverse_translation),
        ],
        [
            left[2][0].clone(),
            left[2][1].clone(),
            left[2][2].clone(),
            affine_translation_column_update(&left[2], &inverse_translation),
        ],
        [
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
            Scalar::one(),
        ],
    ])
}

fn invert_matrix3_by_diagonal<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    // For true diagonal matrices, inversion is n scalar inverses with no extra
    // multiply-add schedule; this avoids the division-heavy elimination and
    // cofactor work while preserving exact division semantics.
    if matrix[0][0] == matrix[1][1] && matrix[0][0] == matrix[2][2] {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix3-diagonal-uniform-scale"
        );
        let inv = matrix[0][0].clone().inverse()?;
        return Ok([
            [inv.clone(), Scalar::zero(), Scalar::zero()],
            [Scalar::zero(), inv.clone(), Scalar::zero()],
            [Scalar::zero(), Scalar::zero(), inv],
        ]);
    }
    let inv00 = matrix[0][0].clone().inverse()?;
    let inv11 = matrix[1][1].clone().inverse()?;
    let inv22 = matrix[2][2].clone().inverse()?;
    if B::MOVE_ELEMENTWISE {
        // Hyperreal-style backends prefer direct fixed-array construction here:
        // the structural diagonal fact already selected this kernel, so
        // per-cell branch dispatch only re-proves known sparsity. Approx-style
        // compact backends benchmark faster with the old `from_fn` shape after
        // LLVM scalarization, so this is deliberately backend-gated. This is
        // the fixed-size version of exploiting matrix structure before
        // arithmetic described by Golub and Van Loan, *Matrix Computations*,
        // and by Yap, "Towards Exact Geometric Computation", 1997.
        Ok([
            [inv00, Scalar::zero(), Scalar::zero()],
            [Scalar::zero(), inv11, Scalar::zero()],
            [Scalar::zero(), Scalar::zero(), inv22],
        ])
    } else {
        Ok(from_fn(|row| {
            from_fn(|col| {
                if row == 0 && col == 0 {
                    inv00.clone()
                } else if row == 1 && col == 1 {
                    inv11.clone()
                } else if row == 2 && col == 2 {
                    inv22.clone()
                } else {
                    Scalar::zero()
                }
            })
        }))
    }
}

#[inline]
fn invert_matrix3_by_diagonal_checked<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    require_known_nonzero(&matrix[0][0])?;
    require_known_nonzero(&matrix[1][1])?;
    require_known_nonzero(&matrix[2][2])?;
    invert_matrix3_by_diagonal(matrix)
}

#[inline]
fn invert_matrix3_by_diagonal_checked_with_abort<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    require_known_nonzero_with_abort(&matrix[0][0], signal)?;
    require_known_nonzero_with_abort(&matrix[1][1], signal)?;
    require_known_nonzero_with_abort(&matrix[2][2], signal)?;
    invert_matrix3_by_diagonal(matrix)
}

#[inline]
fn invert_matrix3_upper_triangular<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    // Upper-triangular inversion uses three pivot inverses plus short substitution:
    // exactly the arithmetic savings expected from specialized triangular kernels
    // in exact linear algebra. Avoiding minors here aligns with Bareiss-style
    // fraction-free goals by minimizing intermediate determinant scaling.
    let inv_a00 = matrix[0][0].clone().inverse()?;
    let inv_a11 = matrix[1][1].clone().inverse()?;
    let inv_a22 = matrix[2][2].clone().inverse()?;

    let inv_a01 = scale_by_shared_factor(Scalar::zero() - &matrix[0][1], &inv_a11);
    let inv_a01 = scale_by_shared_factor(inv_a01, &inv_a00);
    let inv_a12 = scale_by_shared_factor(Scalar::zero() - &matrix[1][2], &inv_a11);
    let inv_a12 = scale_by_shared_factor(inv_a12, &inv_a22);
    let inv_a02 = Scalar::zero() - ((&matrix[0][1] * &inv_a12) + (&matrix[0][2] * &inv_a22));
    let inv_a02 = scale_by_shared_factor(inv_a02, &inv_a00);

    Ok([
        [inv_a00, inv_a01, inv_a02],
        [Scalar::zero(), inv_a11, inv_a12],
        [Scalar::zero(), Scalar::zero(), inv_a22],
    ])
}

#[inline]
fn invert_matrix3_upper_triangular_checked<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    require_known_nonzero(&matrix[0][0])?;
    require_known_nonzero(&matrix[1][1])?;
    require_known_nonzero(&matrix[2][2])?;
    invert_matrix3_upper_triangular(matrix)
}

#[inline]
fn invert_matrix3_upper_triangular_checked_with_abort<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    require_known_nonzero_with_abort(&matrix[0][0], signal)?;
    require_known_nonzero_with_abort(&matrix[1][1], signal)?;
    require_known_nonzero_with_abort(&matrix[2][2], signal)?;
    invert_matrix3_upper_triangular(matrix)
}

#[inline]
fn invert_matrix3_lower_triangular<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    // Lower-triangular inversion is the dual of upper-triangular back-substitution.
    // Selecting this path preserves the same O(n²) schedule and avoids expensive
    // cofactor materialization for triangular right-divisor families.
    let inv_a00 = matrix[0][0].clone().inverse()?;
    let inv_a11 = matrix[1][1].clone().inverse()?;
    let inv_a22 = matrix[2][2].clone().inverse()?;

    let inv_a10 = scale_by_shared_factor(Scalar::zero() - &matrix[1][0], &inv_a00);
    let inv_a10 = scale_by_shared_factor(inv_a10, &inv_a11);
    let inv_a20 = Scalar::zero() - ((&matrix[2][0] * &inv_a00) + (&matrix[2][1] * &inv_a10));
    let inv_a20 = scale_by_shared_factor(inv_a20, &inv_a22);
    let inv_a21 = scale_by_shared_factor(Scalar::zero() - &matrix[2][1], &inv_a11);
    let inv_a21 = scale_by_shared_factor(inv_a21, &inv_a22);

    Ok([
        [inv_a00, Scalar::zero(), Scalar::zero()],
        [inv_a10, inv_a11, Scalar::zero()],
        [inv_a20, inv_a21, inv_a22],
    ])
}

#[inline]
fn invert_matrix3_lower_triangular_checked<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    require_known_nonzero(&matrix[0][0])?;
    require_known_nonzero(&matrix[1][1])?;
    require_known_nonzero(&matrix[2][2])?;
    invert_matrix3_lower_triangular(matrix)
}

#[inline]
fn invert_matrix3_lower_triangular_checked_with_abort<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    require_known_nonzero_with_abort(&matrix[0][0], signal)?;
    require_known_nonzero_with_abort(&matrix[1][1], signal)?;
    require_known_nonzero_with_abort(&matrix[2][2], signal)?;
    invert_matrix3_lower_triangular(matrix)
}

#[inline]
fn invert_matrix3_affine<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
    linear_is_diagonal: bool,
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    // See Golub and Van Loan, *Matrix Computations*: affine composition
    // in homogeneous coordinates is handled by a 2×2 block inverse plus one
    // rank-one translation correction, which is substantially cheaper than a full
    // adjugate for repeated geometric kernels.
    // The caller supplies `linear_is_diagonal` from `Matrix3Facts`, avoiding a
    // second probe of the same off-diagonal entries after affine dispatch.
    // This follows the retained-object-fact strategy in Yap, "Towards Exact
    // Geometric Computation", 1997.
    if linear_is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix3-affine-linear-diagonal"
        );
        return invert_matrix3_affine_linear_diagonal(matrix);
    }

    let a = matrix[0][0].clone();
    let b = matrix[0][1].clone();
    let c = matrix[1][0].clone();
    let d = matrix[1][1].clone();
    let tx = matrix[0][2].clone();
    let ty = matrix[1][2].clone();

    let det = (&a * &d) - (&b * &c);
    let inv_det = det.clone().inverse()?;
    let inv_a00 = scale_by_shared_factor(d, &inv_det);
    let inv_a01 = scale_by_shared_factor(Scalar::zero() - &b, &inv_det);
    let inv_a10 = scale_by_shared_factor(Scalar::zero() - &c, &inv_det);
    let inv_a11 = scale_by_shared_factor(a, &inv_det);
    let inv_tx = Scalar::zero() - ((&inv_a00 * &tx) + (&inv_a01 * &ty));
    let inv_ty = Scalar::zero() - ((&inv_a10 * &tx) + (&inv_a11 * &ty));

    Ok([
        [inv_a00, inv_a01, inv_tx],
        [inv_a10, inv_a11, inv_ty],
        [Scalar::zero(), Scalar::zero(), Scalar::one()],
    ])
}

#[inline]
fn invert_matrix3_affine_linear_diagonal<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    // When the affine 2×2 block is diagonal, inversion is two scalar
    // reciprocals and two multiply-adds for translation.
    // The same block-triangular structure is emphasized in LAPACK/ScaLAPACK notes
    // and in Golub & Van Loan's block matrix treatment.
    let inv_a00 = matrix[0][0].clone().inverse()?;
    let inv_a11 = matrix[1][1].clone().inverse()?;
    let inv_tx = Scalar::zero() - (matrix[0][2].clone() * &inv_a00);
    let inv_ty = Scalar::zero() - (matrix[1][2].clone() * &inv_a11);

    Ok([
        [inv_a00, Scalar::zero(), inv_tx],
        [Scalar::zero(), inv_a11, inv_ty],
        [Scalar::zero(), Scalar::zero(), Scalar::one()],
    ])
}

#[inline]
fn invert_matrix3_affine_linear_diagonal_checked<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    require_known_nonzero(&matrix[0][0])?;
    require_known_nonzero(&matrix[1][1])?;

    let inv_a00 = matrix[0][0].clone().inverse()?;
    let inv_a11 = matrix[1][1].clone().inverse()?;
    let inv_tx = Scalar::zero() - (matrix[0][2].clone() * &inv_a00);
    let inv_ty = Scalar::zero() - (matrix[1][2].clone() * &inv_a11);

    Ok([
        [inv_a00, Scalar::zero(), inv_tx],
        [Scalar::zero(), inv_a11, inv_ty],
        [Scalar::zero(), Scalar::zero(), Scalar::one()],
    ])
}

#[inline]
fn invert_matrix3_affine_linear_diagonal_checked_with_abort<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    let a00 = with_abort(matrix[0][0].clone(), signal);
    let a11 = with_abort(matrix[1][1].clone(), signal);
    let inv_a00 = a00;
    let inv_a11 = a11;
    require_known_nonzero_with_abort(&inv_a00, signal)?;
    require_known_nonzero_with_abort(&inv_a11, signal)?;
    let inv_a00 = inv_a00.inverse()?;
    let inv_a11 = inv_a11.inverse()?;
    let inv_tx = Scalar::zero() - (matrix[0][2].clone() * &inv_a00);
    let inv_ty = Scalar::zero() - (matrix[1][2].clone() * &inv_a11);

    Ok([
        [inv_a00, Scalar::zero(), inv_tx],
        [Scalar::zero(), inv_a11, inv_ty],
        [Scalar::zero(), Scalar::zero(), Scalar::one()],
    ])
}

#[inline]
fn divide_matrix3_by_affine_linear_diagonal<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    // The 2×2 linear diagonal branch is effectively three independent scale
    // factors plus a column correction; this avoids a 2×2 determinant and two
    // multiplications from the generic affine formula.
    // This is the 2D analogue of axis-aligned 4D affine division, and matches
    // the structural savings described in Golub and Van Loan, *Matrix
    // Computations*.
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    let inv_tx = Scalar::zero() - (&right[0][2] * &inv_a00);
    let inv_ty = Scalar::zero() - (&right[1][2] * &inv_a11);

    Ok([
        [
            {
                let row = left[0][0].clone();
                let row = row * &inv_a00;
                row
            },
            {
                let row = left[0][1].clone();
                let row = row * &inv_a11;
                row
            },
            {
                let x = left[0][0].clone();
                let y = left[0][1].clone();
                left[0][2].clone() + &(x * &inv_tx) + &(y * &inv_ty)
            },
        ],
        [
            {
                let row = left[1][0].clone();
                let row = row * &inv_a00;
                row
            },
            {
                let row = left[1][1].clone();
                let row = row * &inv_a11;
                row
            },
            {
                let x = left[1][0].clone();
                let y = left[1][1].clone();
                left[1][2].clone() + &(x * &inv_tx) + &(y * &inv_ty)
            },
        ],
        [
            {
                let row = left[2][0].clone();
                let row = row * &inv_a00;
                row
            },
            {
                let row = left[2][1].clone();
                let row = row * &inv_a11;
                row
            },
            {
                let x = left[2][0].clone();
                let y = left[2][1].clone();
                left[2][2].clone() + &(x * &inv_tx) + &(y * &inv_ty)
            },
        ],
    ])
}

#[inline]
fn divide_matrix3_by_affine_linear_diagonal_checked<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    divide_matrix3_by_affine_linear_diagonal(left, right)
}

#[inline]
fn divide_matrix3_by_affine_linear_diagonal_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    require_known_nonzero_with_abort(&right[0][0], signal)?;
    require_known_nonzero_with_abort(&right[1][1], signal)?;
    divide_matrix3_by_affine_linear_diagonal(left, right)
}

#[inline]
fn divide_matrix3_by_affine_ref_linear_diagonal<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    let inv_tx = Scalar::zero() - (&right[0][2] * &inv_a00);
    let inv_ty = Scalar::zero() - (&right[1][2] * &inv_a11);

    Ok([
        [
            {
                let row = left[0][0].clone();
                let row = row * &inv_a00;
                row
            },
            {
                let row = left[0][1].clone();
                let row = row * &inv_a11;
                row
            },
            {
                let x = left[0][0].clone();
                let y = left[0][1].clone();
                left[0][2].clone() + &(x * &inv_tx) + &(y * &inv_ty)
            },
        ],
        [
            {
                let row = left[1][0].clone();
                let row = row * &inv_a00;
                row
            },
            {
                let row = left[1][1].clone();
                let row = row * &inv_a11;
                row
            },
            {
                let x = left[1][0].clone();
                let y = left[1][1].clone();
                left[1][2].clone() + &(x * &inv_tx) + &(y * &inv_ty)
            },
        ],
        [
            {
                let row = left[2][0].clone();
                let row = row * &inv_a00;
                row
            },
            {
                let row = left[2][1].clone();
                let row = row * &inv_a11;
                row
            },
            {
                let x = left[2][0].clone();
                let y = left[2][1].clone();
                left[2][2].clone() + &(x * &inv_tx) + &(y * &inv_ty)
            },
        ],
    ])
}

#[inline]
fn divide_matrix3_affine_by_affine_linear_diagonal<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    divide_matrix3_by_affine_linear_diagonal(left, right)
}

#[inline]
fn divide_matrix3_affine_by_affine_ref_linear_diagonal<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    divide_matrix3_by_affine_ref_linear_diagonal(left, right)
}

#[inline]
fn divide_matrix3_affine_by_affine_linear_diagonal_checked<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    divide_matrix3_by_affine_linear_diagonal_checked(left, right)
}

#[inline]
fn divide_matrix3_affine_by_affine_linear_diagonal_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    divide_matrix3_by_affine_linear_diagonal_checked_with_abort(left, right, signal)
}

#[inline]
fn invert_matrix3_affine_checked<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
    linear_is_diagonal: bool,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    if linear_is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix3-checked-affine-linear-diagonal"
        );
        return invert_matrix3_affine_linear_diagonal_checked(matrix);
    }

    let a = matrix[0][0].clone();
    let b = matrix[0][1].clone();
    let c = matrix[1][0].clone();
    let d = matrix[1][1].clone();
    let tx = matrix[0][2].clone();
    let ty = matrix[1][2].clone();

    let det = (&a * &d) - (&b * &c);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    let inv_a00 = scale_by_shared_factor(d, &inv_det);
    let inv_a01 = scale_by_shared_factor(Scalar::zero() - &b, &inv_det);
    let inv_a10 = scale_by_shared_factor(Scalar::zero() - &c, &inv_det);
    let inv_a11 = scale_by_shared_factor(a, &inv_det);
    let inv_tx = Scalar::zero() - ((&inv_a00 * &tx) + (&inv_a01 * &ty));
    let inv_ty = Scalar::zero() - ((&inv_a10 * &tx) + (&inv_a11 * &ty));

    Ok([
        [inv_a00, inv_a01, inv_tx],
        [inv_a10, inv_a11, inv_ty],
        [Scalar::zero(), Scalar::zero(), Scalar::one()],
    ])
}

#[inline]
fn invert_matrix3_affine_checked_with_abort<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
    signal: &AbortSignal,
    linear_is_diagonal: bool,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    if linear_is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix3-checked-with-abort-affine-linear-diagonal"
        );
        return invert_matrix3_affine_linear_diagonal_checked_with_abort(matrix, signal);
    }

    let a = matrix[0][0].clone();
    let b = matrix[0][1].clone();
    let c = matrix[1][0].clone();
    let d = matrix[1][1].clone();
    let tx = matrix[0][2].clone();
    let ty = matrix[1][2].clone();

    let det = with_abort((&a * &d) - (&b * &c), signal);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    let inv_a00 = scale_by_shared_factor(d, &inv_det);
    let inv_a01 = scale_by_shared_factor(Scalar::zero() - &b, &inv_det);
    let inv_a10 = scale_by_shared_factor(Scalar::zero() - &c, &inv_det);
    let inv_a11 = scale_by_shared_factor(a, &inv_det);
    let inv_tx = Scalar::zero() - ((&inv_a00 * &tx) + (&inv_a01 * &ty));
    let inv_ty = Scalar::zero() - ((&inv_a10 * &tx) + (&inv_a11 * &ty));

    Ok([
        [inv_a00, inv_a01, inv_tx],
        [inv_a10, inv_a11, inv_ty],
        [Scalar::zero(), Scalar::zero(), Scalar::one()],
    ])
}

#[inline]
fn invert_matrix4_by_diagonal<B: Backend>(
    matrix: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    // The same diagonal path is the exact analog in 4x4: invert only diagonal
    // entries when structural zeros certify no couplings.
    if matrix[0][0] == matrix[1][1] && matrix[0][0] == matrix[2][2] && matrix[0][0] == matrix[3][3]
    {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-diagonal-uniform-scale"
        );
        let inv = matrix[0][0].clone().inverse()?;
        return Ok([
            [inv.clone(), Scalar::zero(), Scalar::zero(), Scalar::zero()],
            [Scalar::zero(), inv.clone(), Scalar::zero(), Scalar::zero()],
            [Scalar::zero(), Scalar::zero(), inv.clone(), Scalar::zero()],
            [Scalar::zero(), Scalar::zero(), Scalar::zero(), inv],
        ]);
    }
    let inv00 = matrix[0][0].clone().inverse()?;
    let inv11 = matrix[1][1].clone().inverse()?;
    let inv22 = matrix[2][2].clone().inverse()?;
    let inv33 = matrix[3][3].clone().inverse()?;
    if B::MOVE_ELEMENTWISE {
        // Hyperreal benefits from emitting the matrix directly: once
        // `Matrix4Facts::is_diagonal` chose this helper, all off-diagonal zeros
        // are certified object-level facts. Avoiding a second sparsity decision
        // keeps the symbolic/exact path thinner. The approx backend regressed
        // on this shape, so it keeps the `from_fn` builder below for flatter
        // compact-scalar timings. This follows the structure-first principle in
        // Golub and Van Loan, *Matrix Computations*, and Yap, "Towards Exact
        // Geometric Computation", 1997.
        Ok([
            [inv00, Scalar::zero(), Scalar::zero(), Scalar::zero()],
            [Scalar::zero(), inv11, Scalar::zero(), Scalar::zero()],
            [Scalar::zero(), Scalar::zero(), inv22, Scalar::zero()],
            [Scalar::zero(), Scalar::zero(), Scalar::zero(), inv33],
        ])
    } else {
        Ok(from_fn(|row| {
            from_fn(|col| {
                if row == 0 && col == 0 {
                    inv00.clone()
                } else if row == 1 && col == 1 {
                    inv11.clone()
                } else if row == 2 && col == 2 {
                    inv22.clone()
                } else if row == 3 && col == 3 {
                    inv33.clone()
                } else {
                    Scalar::zero()
                }
            })
        }))
    }
}

#[inline]
fn invert_matrix4_by_upper_triangular<B: Backend>(
    matrix: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    // Invert upper-triangular matrices via explicit fixed-size triangular
    // back-substitution. The inverse is upper-triangular with row-local support
    // `col >= row`, so avoid touching zero columns and zero RHS entries that are
    // guaranteed by the identity structure. This keeps the inversion path in O(n²)
    // while reducing inner-loop work versus generic right-division shape.
    // Golub & Van Loan, *Matrix Computations* (4th ed.), §3.6.
    let inv_a00 = matrix[0][0].clone().inverse()?;
    let inv_a11 = matrix[1][1].clone().inverse()?;
    let inv_a22 = matrix[2][2].clone().inverse()?;
    let inv_a33 = matrix[3][3].clone().inverse()?;
    let inv_diagonal = [inv_a00, inv_a11, inv_a22, inv_a33];
    let mut result = [
        [
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
        ],
        [
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
        ],
        [
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
        ],
        [
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
        ],
    ];

    for row in 0..4 {
        for col in row..4 {
            let mut value = if row == col {
                Scalar::one()
            } else {
                Scalar::zero()
            };
            for k in row..col {
                value = value - (&result[row][k] * &matrix[k][col]);
            }
            result[row][col] = value.mul_cached(&inv_diagonal[col]);
        }
    }
    Ok(result)
}

#[inline]
fn invert_matrix4_by_lower_triangular<B: Backend>(
    matrix: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    // Lower-triangular inverse is the mirrored O(n²) triangular solve used by the
    // upper branch, using the identity support `col <= row` directly.
    // This avoids both the general right-division row span and unnecessary zero
    // updates when inverting known lower-triangular divisors. See
    // Golub & Van Loan, *Matrix Computations* (4th ed.), §3.6.
    let inv_a00 = matrix[0][0].clone().inverse()?;
    let inv_a11 = matrix[1][1].clone().inverse()?;
    let inv_a22 = matrix[2][2].clone().inverse()?;
    let inv_a33 = matrix[3][3].clone().inverse()?;
    let inv_diagonal = [inv_a00, inv_a11, inv_a22, inv_a33];
    let mut result = [
        [
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
        ],
        [
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
        ],
        [
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
        ],
        [
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
        ],
    ];

    for row in 0..4 {
        for col in (0..=row).rev() {
            let mut value = if row == col {
                Scalar::one()
            } else {
                Scalar::zero()
            };
            for k in (col + 1)..=row {
                value = value - (&result[row][k] * &matrix[k][col]);
            }
            result[row][col] = value.mul_cached(&inv_diagonal[col]);
        }
    }
    Ok(result)
}

#[inline]
fn invert_matrix4_by_upper_triangular_checked<B: Backend>(
    matrix: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    require_known_nonzero(&matrix[0][0])?;
    require_known_nonzero(&matrix[1][1])?;
    require_known_nonzero(&matrix[2][2])?;
    require_known_nonzero(&matrix[3][3])?;
    invert_matrix4_by_upper_triangular(matrix)
}

#[inline]
fn invert_matrix4_by_lower_triangular_checked<B: Backend>(
    matrix: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    require_known_nonzero(&matrix[0][0])?;
    require_known_nonzero(&matrix[1][1])?;
    require_known_nonzero(&matrix[2][2])?;
    require_known_nonzero(&matrix[3][3])?;
    invert_matrix4_by_lower_triangular(matrix)
}

#[inline]
fn invert_matrix4_by_upper_triangular_checked_with_abort<B: Backend>(
    matrix: &[[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    require_known_nonzero_with_abort(&matrix[0][0], signal)?;
    require_known_nonzero_with_abort(&matrix[1][1], signal)?;
    require_known_nonzero_with_abort(&matrix[2][2], signal)?;
    require_known_nonzero_with_abort(&matrix[3][3], signal)?;
    invert_matrix4_by_upper_triangular(matrix)
}

#[inline]
fn invert_matrix4_by_lower_triangular_checked_with_abort<B: Backend>(
    matrix: &[[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    require_known_nonzero_with_abort(&matrix[0][0], signal)?;
    require_known_nonzero_with_abort(&matrix[1][1], signal)?;
    require_known_nonzero_with_abort(&matrix[2][2], signal)?;
    require_known_nonzero_with_abort(&matrix[3][3], signal)?;
    invert_matrix4_by_lower_triangular(matrix)
}

#[inline]
fn invert_matrix4_by_diagonal_checked<B: Backend>(
    matrix: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    require_known_nonzero(&matrix[0][0])?;
    require_known_nonzero(&matrix[1][1])?;
    require_known_nonzero(&matrix[2][2])?;
    require_known_nonzero(&matrix[3][3])?;
    invert_matrix4_by_diagonal(matrix)
}

#[inline]
fn invert_matrix4_by_diagonal_checked_with_abort<B: Backend>(
    matrix: &[[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    require_known_nonzero_with_abort(&matrix[0][0], signal)?;
    require_known_nonzero_with_abort(&matrix[1][1], signal)?;
    require_known_nonzero_with_abort(&matrix[2][2], signal)?;
    require_known_nonzero_with_abort(&matrix[3][3], signal)?;
    invert_matrix4_by_diagonal(matrix)
}

#[inline]
fn multiply_matrix3_by_left_diagonal<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> [[Scalar<B>; 3]; 3] {
    // Left multiplication by a diagonal matrix is equivalent to row-wise scaling
    // by diagonal pivots. For fixed-size kernels this is a one-pass map over
    // nine arithmetic groups and no dot-product schedule.
    let inv00 = left[0][0].clone();
    let inv11 = left[1][1].clone();
    let inv22 = left[2][2].clone();
    let [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]] = right.clone();

    [
        [
            r00.mul_cached(&inv00),
            r01.mul_cached(&inv00),
            r02.mul_cached(&inv00),
        ],
        [
            r10.mul_cached(&inv11),
            r11.mul_cached(&inv11),
            r12.mul_cached(&inv11),
        ],
        [
            r20.mul_cached(&inv22),
            r21.mul_cached(&inv22),
            r22.mul_cached(&inv22),
        ],
    ]
}

#[inline]
fn multiply_matrix3_by_right_diagonal<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> [[Scalar<B>; 3]; 3] {
    // Right multiplication by diagonal is column-wise scaling and preserves the
    // symbolic row structure used by exact rational kernels.
    let inv00 = right[0][0].clone();
    let inv11 = right[1][1].clone();
    let inv22 = right[2][2].clone();
    let [[l00, l01, l02], [l10, l11, l12], [l20, l21, l22]] = left.clone();

    [
        [
            l00.mul_cached(&inv00),
            l01.mul_cached(&inv11),
            l02.mul_cached(&inv22),
        ],
        [
            l10.mul_cached(&inv00),
            l11.mul_cached(&inv11),
            l12.mul_cached(&inv22),
        ],
        [
            l20.mul_cached(&inv00),
            l21.mul_cached(&inv11),
            l22.mul_cached(&inv22),
        ],
    ]
}

#[inline]
fn multiply_matrix4_by_left_diagonal<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> [[Scalar<B>; 4]; 4] {
    // Left multiplication by a diagonal matrix is row-wise scaling for all lanes.
    let inv00 = left[0][0].clone();
    let inv11 = left[1][1].clone();
    let inv22 = left[2][2].clone();
    let inv33 = left[3][3].clone();
    let [
        [r00, r01, r02, r03],
        [r10, r11, r12, r13],
        [r20, r21, r22, r23],
        [r30, r31, r32, r33],
    ] = right.clone();

    [
        [
            r00.mul_cached(&inv00),
            r01.mul_cached(&inv00),
            r02.mul_cached(&inv00),
            r03.mul_cached(&inv00),
        ],
        [
            r10.mul_cached(&inv11),
            r11.mul_cached(&inv11),
            r12.mul_cached(&inv11),
            r13.mul_cached(&inv11),
        ],
        [
            r20.mul_cached(&inv22),
            r21.mul_cached(&inv22),
            r22.mul_cached(&inv22),
            r23.mul_cached(&inv22),
        ],
        [
            r30.mul_cached(&inv33),
            r31.mul_cached(&inv33),
            r32.mul_cached(&inv33),
            r33.mul_cached(&inv33),
        ],
    ]
}

#[inline]
fn multiply_matrix4_by_right_diagonal<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> [[Scalar<B>; 4]; 4] {
    // Right multiplication by diagonal scales each column independently and keeps
    // symbolic sparsity checks in the generic multiply paths unchanged.
    let inv00 = right[0][0].clone();
    let inv11 = right[1][1].clone();
    let inv22 = right[2][2].clone();
    let inv33 = right[3][3].clone();
    let [
        [l00, l01, l02, l03],
        [l10, l11, l12, l13],
        [l20, l21, l22, l23],
        [l30, l31, l32, l33],
    ] = left.clone();

    [
        [
            l00.mul_cached(&inv00),
            l01.mul_cached(&inv11),
            l02.mul_cached(&inv22),
            l03.mul_cached(&inv33),
        ],
        [
            l10.mul_cached(&inv00),
            l11.mul_cached(&inv11),
            l12.mul_cached(&inv22),
            l13.mul_cached(&inv33),
        ],
        [
            l20.mul_cached(&inv00),
            l21.mul_cached(&inv11),
            l22.mul_cached(&inv22),
            l23.mul_cached(&inv33),
        ],
        [
            l30.mul_cached(&inv00),
            l31.mul_cached(&inv11),
            l32.mul_cached(&inv22),
            l33.mul_cached(&inv33),
        ],
    ]
}

#[inline]
fn divide_matrix3_by_diagonal<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    let inv00 = right[0][0].clone().inverse()?;
    let inv11 = right[1][1].clone().inverse()?;
    let inv22 = right[2][2].clone().inverse()?;
    let mut result = left;
    for row in &mut result {
        row[0] = row[0].clone().mul_cached(&inv00);
        row[1] = row[1].clone().mul_cached(&inv11);
        row[2] = row[2].clone().mul_cached(&inv22);
    }
    Ok(result)
}

#[inline]
fn divide_matrix3_by_upper_triangular<B: Backend>(
    mut left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    // Upper-triangular right-division is a fixed-size triangular solve on each row:
    // each row is independent, so we avoid building an explicit inverse and one adjugate.
    // This is the classic O(n^2) back-substitution path described in Golub & Van Loan,
    // *Matrix Computations*, when triangular structure is known.
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    let inv_a22 = right[2][2].clone().inverse()?;
    if B::FUSE_SIGNED_PRODUCT_SUM {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "divide3-upper-triangular-fused-exact"
        );
        let one = Scalar::one();
        for row in &mut left {
            let x0 = row[0].clone().mul_cached(&inv_a00);
            let x1 = (row[1].clone() - (&x0 * &right[0][1])).mul_cached(&inv_a11);
            let x2 = Scalar::active_signed_product_sum2(
                [true, false, false],
                [[&row[2], &one], [&x0, &right[0][2]], [&x1, &right[1][2]]],
            )
            .mul_cached(&inv_a22);
            *row = [x0, x1, x2];
        }
        return Ok(left);
    }

    let row0_0 = left[0][0].clone().mul_cached(&inv_a00);
    let row0_1 = (left[0][1].clone() - (row0_0.clone() * &right[0][1])).mul_cached(&inv_a11);
    let row0_2 =
        (left[0][2].clone() - (row0_0.clone() * &right[0][2]) - (row0_1.clone() * &right[1][2]))
            .mul_cached(&inv_a22);

    let row1_0 = left[1][0].clone().mul_cached(&inv_a00);
    let row1_1 = (left[1][1].clone() - (row1_0.clone() * &right[0][1])).mul_cached(&inv_a11);
    let row1_2 =
        (left[1][2].clone() - (row1_0.clone() * &right[0][2]) - (row1_1.clone() * &right[1][2]))
            .mul_cached(&inv_a22);

    let row2_0 = left[2][0].clone().mul_cached(&inv_a00);
    let row2_1 = (left[2][1].clone() - (row2_0.clone() * &right[0][1])).mul_cached(&inv_a11);
    let row2_2 =
        (left[2][2].clone() - (row2_0.clone() * &right[0][2]) - (row2_1.clone() * &right[1][2]))
            .mul_cached(&inv_a22);

    left[0] = [row0_0, row0_1, row0_2];
    left[1] = [row1_0, row1_1, row1_2];
    left[2] = [row2_0, row2_1, row2_2];
    Ok(left)
}

#[inline]
fn divide_matrix3_affine_upper_row<B: Backend>(
    row: &[Scalar<B>; 3],
    right: &[[Scalar<B>; 3]; 3],
    inv_a00: &Scalar<B>,
    inv_a11: &Scalar<B>,
    one: &Scalar<B>,
) -> [Scalar<B>; 3] {
    let x0 = row[0].clone().mul_cached(inv_a00);
    let x1 = (row[1].clone() - (&x0 * &right[0][1])).mul_cached(inv_a11);
    let x2 = mul_sub_add(&row[2], one, &x0, &right[0][2], &x1, &right[1][2]);
    [x0, x1, x2]
}

#[inline]
fn divide_matrix3_by_affine_upper_triangular<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    // A 2D affine upper-triangular divisor has an already-known homogeneous
    // diagonal of one. Reuse the retained affine/triangular facts and solve
    // only the two nontrivial diagonal lanes, avoiding a fresh inverse query
    // and multiply-by-one in the hot translation lane.
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    if B::FUSE_SIGNED_PRODUCT_SUM {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "divide3-affine-upper-triangular-fused-exact"
        );
    }
    let one = Scalar::one();
    Ok([
        divide_matrix3_affine_upper_row(&left[0], right, &inv_a00, &inv_a11, &one),
        divide_matrix3_affine_upper_row(&left[1], right, &inv_a00, &inv_a11, &one),
        divide_matrix3_affine_upper_row(&left[2], right, &inv_a00, &inv_a11, &one),
    ])
}

#[inline]
fn divide_matrix3_affine_by_affine_upper_triangular<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    if B::FUSE_SIGNED_PRODUCT_SUM {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "divide3-affine-left-affine-upper-triangular-fused-exact"
        );
    }
    let one = Scalar::one();
    Ok([
        divide_matrix3_affine_upper_row(&left[0], right, &inv_a00, &inv_a11, &one),
        divide_matrix3_affine_upper_row(&left[1], right, &inv_a00, &inv_a11, &one),
        [Scalar::zero(), Scalar::zero(), one],
    ])
}

#[inline]
fn divide_matrix3_by_affine_upper_triangular_checked<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    divide_matrix3_by_affine_upper_triangular(left, right)
}

#[inline]
fn divide_matrix3_affine_by_affine_upper_triangular_checked<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    divide_matrix3_affine_by_affine_upper_triangular(left, right)
}

#[inline]
fn divide_matrix3_by_affine_upper_triangular_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    require_known_nonzero_with_abort(&right[0][0], signal)?;
    require_known_nonzero_with_abort(&right[1][1], signal)?;
    divide_matrix3_by_affine_upper_triangular(left, right)
}

#[inline]
fn divide_matrix3_affine_by_affine_upper_triangular_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    require_known_nonzero_with_abort(&right[0][0], signal)?;
    require_known_nonzero_with_abort(&right[1][1], signal)?;
    divide_matrix3_affine_by_affine_upper_triangular(left, right)
}

#[inline]
fn divide_matrix3_by_upper_triangular_checked<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    require_known_nonzero(&right[2][2])?;
    divide_matrix3_by_upper_triangular(left, right)
}

#[inline]
fn divide_matrix3_by_upper_triangular_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    require_known_nonzero_with_abort(&right[0][0], signal)?;
    require_known_nonzero_with_abort(&right[1][1], signal)?;
    require_known_nonzero_with_abort(&right[2][2], signal)?;
    divide_matrix3_by_upper_triangular(left, right)
}

#[inline]
fn divide_matrix3_by_lower_triangular<B: Backend>(
    mut left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    // Lower-triangular right-division mirrors the transpose scheduling of the upper form.
    // Solving each row with forward substitution is O(n^2) and avoids the cubic adjugate path
    // when only triangular predicates are known.
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    let inv_a22 = right[2][2].clone().inverse()?;
    if B::FUSE_SIGNED_PRODUCT_SUM {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "divide3-lower-triangular-fused-exact"
        );
        let one = Scalar::one();
        for row in &mut left {
            let x2 = row[2].clone().mul_cached(&inv_a22);
            let x1 = (row[1].clone() - (&x2 * &right[2][1])).mul_cached(&inv_a11);
            let x0 = Scalar::active_signed_product_sum2(
                [true, false, false],
                [[&row[0], &one], [&x1, &right[1][0]], [&x2, &right[2][0]]],
            )
            .mul_cached(&inv_a00);
            *row = [x0, x1, x2];
        }
        return Ok(left);
    }

    let row0_2 = left[0][2].clone().mul_cached(&inv_a22);
    let row0_1 = (left[0][1].clone() - (row0_2.clone() * &right[2][1])).mul_cached(&inv_a11);
    let row0_0 =
        (left[0][0].clone() - (row0_1.clone() * &right[1][0]) - (row0_2.clone() * &right[2][0]))
            .mul_cached(&inv_a00);

    let row1_2 = left[1][2].clone().mul_cached(&inv_a22);
    let row1_1 = (left[1][1].clone() - (row1_2.clone() * &right[2][1])).mul_cached(&inv_a11);
    let row1_0 =
        (left[1][0].clone() - (row1_1.clone() * &right[1][0]) - (row1_2.clone() * &right[2][0]))
            .mul_cached(&inv_a00);

    let row2_2 = left[2][2].clone().mul_cached(&inv_a22);
    let row2_1 = (left[2][1].clone() - (row2_2.clone() * &right[2][1])).mul_cached(&inv_a11);
    let row2_0 =
        (left[2][0].clone() - (row2_1.clone() * &right[1][0]) - (row2_2.clone() * &right[2][0]))
            .mul_cached(&inv_a00);

    left[0] = [row0_0, row0_1, row0_2];
    left[1] = [row1_0, row1_1, row1_2];
    left[2] = [row2_0, row2_1, row2_2];
    Ok(left)
}

#[inline]
fn divide_matrix3_by_lower_triangular_checked<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    require_known_nonzero(&right[2][2])?;
    divide_matrix3_by_lower_triangular(left, right)
}

#[inline]
fn divide_matrix3_by_lower_triangular_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    require_known_nonzero_with_abort(&right[0][0], signal)?;
    require_known_nonzero_with_abort(&right[1][1], signal)?;
    require_known_nonzero_with_abort(&right[2][2], signal)?;
    divide_matrix3_by_lower_triangular(left, right)
}

#[inline]
fn affine_inverse_translation2<B: Backend>(
    linear: &[[Scalar<B>; 2]; 2],
    tx: &Scalar<B>,
    ty: &Scalar<B>,
) -> [Scalar<B>; 2] {
    [
        Scalar::zero() - &mul_add(&linear[0][0], tx, &linear[0][1], ty),
        Scalar::zero() - &mul_add(&linear[1][0], tx, &linear[1][1], ty),
    ]
}

#[inline]
fn affine_linear_dot2<B: Backend>(
    left_a: &Scalar<B>,
    right_a: &Scalar<B>,
    left_b: &Scalar<B>,
    right_b: &Scalar<B>,
) -> Scalar<B> {
    mul_add(left_a, right_a, left_b, right_b)
}

#[inline]
fn affine_translation_column_update_from_inverse2<B: Backend>(
    row: &[Scalar<B>; 3],
    translation: &[Scalar<B>; 2],
) -> Scalar<B> {
    row[2].clone() + mul_add(&row[0], &translation[0], &row[1], &translation[1])
}

#[inline]
fn affine_translation_column_subtract_update2<B: Backend>(
    row: &[Scalar<B>; 3],
    translation: [&Scalar<B>; 2],
) -> Scalar<B> {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "affine-translation-column-subtract2"
    );
    let shifted = mul_add(&row[0], translation[0], &row[1], translation[1]);
    row[2].clone() - shifted
}

#[inline]
fn divide_matrix3_by_affine_translation<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    // Right-dividing by translation-only affine only updates the offset column.
    let translation = [&right[0][2], &right[1][2]];
    Ok([
        [
            left[0][0].clone(),
            left[0][1].clone(),
            affine_translation_column_subtract_update2(&left[0], translation),
        ],
        [
            left[1][0].clone(),
            left[1][1].clone(),
            affine_translation_column_subtract_update2(&left[1], translation),
        ],
        [
            left[2][0].clone(),
            left[2][1].clone(),
            affine_translation_column_subtract_update2(&left[2], translation),
        ],
    ])
}

#[inline]
fn divide_matrix3_affine_by_affine_translation<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    // Affine-left by affine translation keeps the 2×2 linear block untouched.
    let translation = [&right[0][2], &right[1][2]];
    Ok([
        [
            left[0][0].clone(),
            left[0][1].clone(),
            affine_translation_column_subtract_update2(&left[0], translation),
        ],
        [
            left[1][0].clone(),
            left[1][1].clone(),
            affine_translation_column_subtract_update2(&left[1], translation),
        ],
        [Scalar::zero(), Scalar::zero(), Scalar::one()],
    ])
}

#[inline]
fn divide_matrix3_by_affine_ref_translation<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    // Borrowed affine-translation fast-path: avoids cloning the entire left
    // matrix just to call the owned helper while still touching only the
    // translation column.
    let translation = [&right[0][2], &right[1][2]];
    Ok([
        [
            left[0][0].clone(),
            left[0][1].clone(),
            affine_translation_column_subtract_update2(&left[0], translation),
        ],
        [
            left[1][0].clone(),
            left[1][1].clone(),
            affine_translation_column_subtract_update2(&left[1], translation),
        ],
        [
            left[2][0].clone(),
            left[2][1].clone(),
            affine_translation_column_subtract_update2(&left[2], translation),
        ],
    ])
}

#[inline]
fn divide_matrix3_affine_by_affine_ref_translation<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    // Borrowed affine-by-affine translation update. As with the translated-only
    // branch, this keeps all linear components unchanged and updates only the
    // third row/col terms touched by the translation.
    let translation = [&right[0][2], &right[1][2]];
    Ok([
        [
            left[0][0].clone(),
            left[0][1].clone(),
            affine_translation_column_subtract_update2(&left[0], translation),
        ],
        [
            left[1][0].clone(),
            left[1][1].clone(),
            affine_translation_column_subtract_update2(&left[1], translation),
        ],
        [Scalar::zero(), Scalar::zero(), Scalar::one()],
    ])
}

#[inline]
fn divide_matrix3_by_affine_ref_no_translation<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    // Borrowed version of affine-no-translation division for general affine
    // divisors. This keeps the right divisor in borrowed form and avoids
    // materializing an owned left clone before factor extraction.
    let a = right[0][0].clone();
    let b = right[0][1].clone();
    let c = right[1][0].clone();
    let d = right[1][1].clone();
    let tx = right[0][2].clone();
    let ty = right[1][2].clone();

    let right_det = (&a * &d) - (&b * &c);
    let right_inv_det = right_det.inverse()?;
    let right_inverse_linear = [
        [
            scale_by_shared_factor(d, &right_inv_det),
            scale_by_shared_factor(Scalar::zero() - &b, &right_inv_det),
        ],
        [
            scale_by_shared_factor(Scalar::zero() - &c, &right_inv_det),
            scale_by_shared_factor(a, &right_inv_det),
        ],
    ];
    let right_inverse_translation = affine_inverse_translation2(&right_inverse_linear, &tx, &ty);

    Ok([
        [
            affine_linear_dot2(
                &left[0][0],
                &right_inverse_linear[0][0],
                &left[0][1],
                &right_inverse_linear[1][0],
            ),
            affine_linear_dot2(
                &left[0][0],
                &right_inverse_linear[0][1],
                &left[0][1],
                &right_inverse_linear[1][1],
            ),
            affine_translation_column_update_from_inverse2(&left[0], &right_inverse_translation),
        ],
        [
            affine_linear_dot2(
                &left[1][0],
                &right_inverse_linear[0][0],
                &left[1][1],
                &right_inverse_linear[1][0],
            ),
            affine_linear_dot2(
                &left[1][0],
                &right_inverse_linear[0][1],
                &left[1][1],
                &right_inverse_linear[1][1],
            ),
            affine_translation_column_update_from_inverse2(&left[1], &right_inverse_translation),
        ],
        [
            affine_linear_dot2(
                &left[2][0],
                &right_inverse_linear[0][0],
                &left[2][1],
                &right_inverse_linear[1][0],
            ),
            affine_linear_dot2(
                &left[2][0],
                &right_inverse_linear[0][1],
                &left[2][1],
                &right_inverse_linear[1][1],
            ),
            affine_translation_column_update_from_inverse2(&left[2], &right_inverse_translation),
        ],
    ])
}

#[inline]
fn divide_matrix3_affine_by_affine_ref_no_translation<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    // Borrowed affine-by-affine no-translation helper. This avoids constructing
    // a temporary owned left matrix for the common affine-by-affine case.
    let a = right[0][0].clone();
    let b = right[0][1].clone();
    let c = right[1][0].clone();
    let d = right[1][1].clone();
    let tx = right[0][2].clone();
    let ty = right[1][2].clone();

    let right_det = (&a * &d) - (&b * &c);
    let right_inv_det = right_det.inverse()?;
    let right_inverse_linear = [
        [
            scale_by_shared_factor(d, &right_inv_det),
            scale_by_shared_factor(Scalar::zero() - &b, &right_inv_det),
        ],
        [
            scale_by_shared_factor(Scalar::zero() - &c, &right_inv_det),
            scale_by_shared_factor(a, &right_inv_det),
        ],
    ];
    let right_inverse_translation = affine_inverse_translation2(&right_inverse_linear, &tx, &ty);

    Ok([
        [
            affine_linear_dot2(
                &left[0][0],
                &right_inverse_linear[0][0],
                &left[0][1],
                &right_inverse_linear[1][0],
            ),
            affine_linear_dot2(
                &left[0][0],
                &right_inverse_linear[0][1],
                &left[0][1],
                &right_inverse_linear[1][1],
            ),
            affine_translation_column_update_from_inverse2(&left[0], &right_inverse_translation),
        ],
        [
            affine_linear_dot2(
                &left[1][0],
                &right_inverse_linear[0][0],
                &left[1][1],
                &right_inverse_linear[1][0],
            ),
            affine_linear_dot2(
                &left[1][0],
                &right_inverse_linear[0][1],
                &left[1][1],
                &right_inverse_linear[1][1],
            ),
            affine_translation_column_update_from_inverse2(&left[1], &right_inverse_translation),
        ],
        [Scalar::zero(), Scalar::zero(), Scalar::one()],
    ])
}

#[inline]
fn divide_matrix3_by_affine<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    let a = right[0][0].clone();
    let b = right[0][1].clone();
    let c = right[1][0].clone();
    let d = right[1][1].clone();
    let tx = right[0][2].clone();
    let ty = right[1][2].clone();

    let right_det = (&a * &d) - (&b * &c);
    let right_inv_det = right_det.inverse()?;
    let right_inverse_linear = [
        [
            scale_by_shared_factor(d, &right_inv_det),
            scale_by_shared_factor(Scalar::zero() - &b, &right_inv_det),
        ],
        [
            scale_by_shared_factor(Scalar::zero() - &c, &right_inv_det),
            scale_by_shared_factor(a, &right_inv_det),
        ],
    ];
    let right_inverse_translation = affine_inverse_translation2(&right_inverse_linear, &tx, &ty);

    Ok([
        [
            affine_linear_dot2(
                &left[0][0],
                &right_inverse_linear[0][0],
                &left[0][1],
                &right_inverse_linear[1][0],
            ),
            affine_linear_dot2(
                &left[0][0],
                &right_inverse_linear[0][1],
                &left[0][1],
                &right_inverse_linear[1][1],
            ),
            affine_translation_column_update_from_inverse2(&left[0], &right_inverse_translation),
        ],
        [
            affine_linear_dot2(
                &left[1][0],
                &right_inverse_linear[0][0],
                &left[1][1],
                &right_inverse_linear[1][0],
            ),
            affine_linear_dot2(
                &left[1][0],
                &right_inverse_linear[0][1],
                &left[1][1],
                &right_inverse_linear[1][1],
            ),
            affine_translation_column_update_from_inverse2(&left[1], &right_inverse_translation),
        ],
        [
            affine_linear_dot2(
                &left[2][0],
                &right_inverse_linear[0][0],
                &left[2][1],
                &right_inverse_linear[1][0],
            ),
            affine_linear_dot2(
                &left[2][0],
                &right_inverse_linear[0][1],
                &left[2][1],
                &right_inverse_linear[1][1],
            ),
            affine_translation_column_update_from_inverse2(&left[2], &right_inverse_translation),
        ],
    ])
}

#[inline]
fn divide_matrix3_affine_by_affine<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    let a = right[0][0].clone();
    let b = right[0][1].clone();
    let c = right[1][0].clone();
    let d = right[1][1].clone();
    let tx = right[0][2].clone();
    let ty = right[1][2].clone();

    let right_det = (&a * &d) - (&b * &c);
    let right_inv_det = right_det.inverse()?;
    let right_inverse_linear = [
        [
            scale_by_shared_factor(d, &right_inv_det),
            scale_by_shared_factor(Scalar::zero() - &b, &right_inv_det),
        ],
        [
            scale_by_shared_factor(Scalar::zero() - &c, &right_inv_det),
            scale_by_shared_factor(a, &right_inv_det),
        ],
    ];
    let right_inverse_translation = affine_inverse_translation2(&right_inverse_linear, &tx, &ty);

    Ok([
        [
            affine_linear_dot2(
                &left[0][0],
                &right_inverse_linear[0][0],
                &left[0][1],
                &right_inverse_linear[1][0],
            ),
            affine_linear_dot2(
                &left[0][0],
                &right_inverse_linear[0][1],
                &left[0][1],
                &right_inverse_linear[1][1],
            ),
            affine_translation_column_update_from_inverse2(&left[0], &right_inverse_translation),
        ],
        [
            affine_linear_dot2(
                &left[1][0],
                &right_inverse_linear[0][0],
                &left[1][1],
                &right_inverse_linear[1][0],
            ),
            affine_linear_dot2(
                &left[1][0],
                &right_inverse_linear[0][1],
                &left[1][1],
                &right_inverse_linear[1][1],
            ),
            affine_translation_column_update_from_inverse2(&left[1], &right_inverse_translation),
        ],
        [Scalar::zero(), Scalar::zero(), Scalar::one()],
    ])
}

#[inline]
fn divide_matrix3_by_diagonal_checked<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    require_known_nonzero(&right[2][2])?;
    divide_matrix3_by_diagonal(left, right)
}

#[inline]
fn divide_matrix3_by_diagonal_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    require_known_nonzero_with_abort(&right[0][0], signal)?;
    require_known_nonzero_with_abort(&right[1][1], signal)?;
    require_known_nonzero_with_abort(&right[2][2], signal)?;
    divide_matrix3_by_diagonal(left, right)
}

#[inline]
fn divide_matrix3_by_affine_checked<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    let a = right[0][0].clone();
    let b = right[0][1].clone();
    let c = right[1][0].clone();
    let d = right[1][1].clone();
    let tx = right[0][2].clone();
    let ty = right[1][2].clone();
    let right_det = (&a * &d) - (&b * &c);
    require_known_nonzero(&right_det)?;
    let right_inv_det = right_det.inverse()?;
    let right_inverse_linear = [
        [
            scale_by_shared_factor(d, &right_inv_det),
            scale_by_shared_factor(Scalar::zero() - &b, &right_inv_det),
        ],
        [
            scale_by_shared_factor(Scalar::zero() - &c, &right_inv_det),
            scale_by_shared_factor(a, &right_inv_det),
        ],
    ];
    let right_inverse_translation = affine_inverse_translation2(&right_inverse_linear, &tx, &ty);

    Ok([
        [
            affine_linear_dot2(
                &left[0][0],
                &right_inverse_linear[0][0],
                &left[0][1],
                &right_inverse_linear[1][0],
            ),
            affine_linear_dot2(
                &left[0][0],
                &right_inverse_linear[0][1],
                &left[0][1],
                &right_inverse_linear[1][1],
            ),
            affine_translation_column_update_from_inverse2(&left[0], &right_inverse_translation),
        ],
        [
            affine_linear_dot2(
                &left[1][0],
                &right_inverse_linear[0][0],
                &left[1][1],
                &right_inverse_linear[1][0],
            ),
            affine_linear_dot2(
                &left[1][0],
                &right_inverse_linear[0][1],
                &left[1][1],
                &right_inverse_linear[1][1],
            ),
            affine_translation_column_update_from_inverse2(&left[1], &right_inverse_translation),
        ],
        [
            affine_linear_dot2(
                &left[2][0],
                &right_inverse_linear[0][0],
                &left[2][1],
                &right_inverse_linear[1][0],
            ),
            affine_linear_dot2(
                &left[2][0],
                &right_inverse_linear[0][1],
                &left[2][1],
                &right_inverse_linear[1][1],
            ),
            affine_translation_column_update_from_inverse2(&left[2], &right_inverse_translation),
        ],
    ])
}

#[inline]
fn divide_matrix3_by_affine_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    let a = right[0][0].clone();
    let b = right[0][1].clone();
    let c = right[1][0].clone();
    let d = right[1][1].clone();
    let tx = right[0][2].clone();
    let ty = right[1][2].clone();
    let right_det = with_abort((&a * &d) - (&b * &c), signal);
    require_known_nonzero(&right_det)?;
    let right_inv_det = right_det.inverse()?;
    let right_inverse_linear = [
        [
            scale_by_shared_factor(d, &right_inv_det),
            scale_by_shared_factor(Scalar::zero() - &b, &right_inv_det),
        ],
        [
            scale_by_shared_factor(Scalar::zero() - &c, &right_inv_det),
            scale_by_shared_factor(a, &right_inv_det),
        ],
    ];
    let right_inverse_translation = affine_inverse_translation2(&right_inverse_linear, &tx, &ty);

    Ok([
        [
            affine_linear_dot2(
                &left[0][0],
                &right_inverse_linear[0][0],
                &left[0][1],
                &right_inverse_linear[1][0],
            ),
            affine_linear_dot2(
                &left[0][0],
                &right_inverse_linear[0][1],
                &left[0][1],
                &right_inverse_linear[1][1],
            ),
            affine_translation_column_update_from_inverse2(&left[0], &right_inverse_translation),
        ],
        [
            affine_linear_dot2(
                &left[1][0],
                &right_inverse_linear[0][0],
                &left[1][1],
                &right_inverse_linear[1][0],
            ),
            affine_linear_dot2(
                &left[1][0],
                &right_inverse_linear[0][1],
                &left[1][1],
                &right_inverse_linear[1][1],
            ),
            affine_translation_column_update_from_inverse2(&left[1], &right_inverse_translation),
        ],
        [
            affine_linear_dot2(
                &left[2][0],
                &right_inverse_linear[0][0],
                &left[2][1],
                &right_inverse_linear[1][0],
            ),
            affine_linear_dot2(
                &left[2][0],
                &right_inverse_linear[0][1],
                &left[2][1],
                &right_inverse_linear[1][1],
            ),
            affine_translation_column_update_from_inverse2(&left[2], &right_inverse_translation),
        ],
    ])
}

#[inline]
fn divide_matrix3_affine_by_affine_checked<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    let a = right[0][0].clone();
    let b = right[0][1].clone();
    let c = right[1][0].clone();
    let d = right[1][1].clone();
    let tx = right[0][2].clone();
    let ty = right[1][2].clone();
    let right_det = (&a * &d) - (&b * &c);
    require_known_nonzero(&right_det)?;
    let right_inv_det = right_det.inverse()?;
    let right_inverse_linear = [
        [
            scale_by_shared_factor(d, &right_inv_det),
            scale_by_shared_factor(Scalar::zero() - &b, &right_inv_det),
        ],
        [
            scale_by_shared_factor(Scalar::zero() - &c, &right_inv_det),
            scale_by_shared_factor(a, &right_inv_det),
        ],
    ];
    let right_inverse_translation = affine_inverse_translation2(&right_inverse_linear, &tx, &ty);

    Ok([
        [
            affine_linear_dot2(
                &left[0][0],
                &right_inverse_linear[0][0],
                &left[0][1],
                &right_inverse_linear[1][0],
            ),
            affine_linear_dot2(
                &left[0][0],
                &right_inverse_linear[0][1],
                &left[0][1],
                &right_inverse_linear[1][1],
            ),
            affine_translation_column_update_from_inverse2(&left[0], &right_inverse_translation),
        ],
        [
            affine_linear_dot2(
                &left[1][0],
                &right_inverse_linear[0][0],
                &left[1][1],
                &right_inverse_linear[1][0],
            ),
            affine_linear_dot2(
                &left[1][0],
                &right_inverse_linear[0][1],
                &left[1][1],
                &right_inverse_linear[1][1],
            ),
            affine_translation_column_update_from_inverse2(&left[1], &right_inverse_translation),
        ],
        [Scalar::zero(), Scalar::zero(), Scalar::one()],
    ])
}

#[inline]
fn divide_matrix3_affine_by_affine_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    let a = right[0][0].clone();
    let b = right[0][1].clone();
    let c = right[1][0].clone();
    let d = right[1][1].clone();
    let tx = right[0][2].clone();
    let ty = right[1][2].clone();
    let right_det = with_abort((&a * &d) - (&b * &c), signal);
    require_known_nonzero(&right_det)?;
    let right_inv_det = right_det.inverse()?;
    let right_inverse_linear = [
        [
            scale_by_shared_factor(d, &right_inv_det),
            scale_by_shared_factor(Scalar::zero() - &b, &right_inv_det),
        ],
        [
            scale_by_shared_factor(Scalar::zero() - &c, &right_inv_det),
            scale_by_shared_factor(a, &right_inv_det),
        ],
    ];
    let right_inverse_translation = affine_inverse_translation2(&right_inverse_linear, &tx, &ty);

    Ok([
        [
            affine_linear_dot2(
                &left[0][0],
                &right_inverse_linear[0][0],
                &left[0][1],
                &right_inverse_linear[1][0],
            ),
            affine_linear_dot2(
                &left[0][0],
                &right_inverse_linear[0][1],
                &left[0][1],
                &right_inverse_linear[1][1],
            ),
            affine_translation_column_update_from_inverse2(&left[0], &right_inverse_translation),
        ],
        [
            affine_linear_dot2(
                &left[1][0],
                &right_inverse_linear[0][0],
                &left[1][1],
                &right_inverse_linear[1][0],
            ),
            affine_linear_dot2(
                &left[1][0],
                &right_inverse_linear[0][1],
                &left[1][1],
                &right_inverse_linear[1][1],
            ),
            affine_translation_column_update_from_inverse2(&left[1], &right_inverse_translation),
        ],
        [Scalar::zero(), Scalar::zero(), Scalar::one()],
    ])
}

#[inline]
fn divide_matrix4_by_diagonal<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    let inv00 = right[0][0].clone().inverse()?;
    let inv11 = right[1][1].clone().inverse()?;
    let inv22 = right[2][2].clone().inverse()?;
    let inv33 = right[3][3].clone().inverse()?;
    let mut result = left;
    for row in &mut result {
        row[0] = row[0].clone().mul_cached(&inv00);
        row[1] = row[1].clone().mul_cached(&inv11);
        row[2] = row[2].clone().mul_cached(&inv22);
        row[3] = row[3].clone().mul_cached(&inv33);
    }
    Ok(result)
}

#[inline]
fn divide_matrix4_by_diagonal_checked<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    require_known_nonzero(&right[2][2])?;
    require_known_nonzero(&right[3][3])?;
    divide_matrix4_by_diagonal(left, right)
}

#[inline]
fn divide_matrix4_by_diagonal_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    require_known_nonzero_with_abort(&right[0][0], signal)?;
    require_known_nonzero_with_abort(&right[1][1], signal)?;
    require_known_nonzero_with_abort(&right[2][2], signal)?;
    require_known_nonzero_with_abort(&right[3][3], signal)?;
    divide_matrix4_by_diagonal(left, right)
}

#[inline]
fn divide_matrix4_by_upper_triangular<B: Backend>(
    mut left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    // Fixed-size upper-triangular right-division is row-wise back-substitution.
    // Each row is independent and needs one diagonal inversion plus at most
    // three fused updates per column element, so this is O(n²) versus O(n³)
    // cofactor scheduling. Each column solves a scalar recurrence that reuses
    // already-computed lower rows (Golub & Van Loan, *Matrix Computations*,
    // 4th ed., §3.6).
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    let inv_a22 = right[2][2].clone().inverse()?;
    let inv_a33 = right[3][3].clone().inverse()?;
    if B::FUSE_SIGNED_PRODUCT_SUM {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "divide4-upper-triangular-fused-exact"
        );
        // Exact backends can keep each row solve as short signed product sums:
        // b_j - x_0 u_0j - ... . This follows the fraction-delay guidance
        // used by Bareiss/common-factor exact matrix methods while avoiding
        // scalar zero probes inside the hot triangular lanes. Approx keeps the
        // compact loop below because prior nanosecond-scale rows were sensitive
        // to exact-backend abstractions.
        let one = Scalar::one();
        for row in &mut left {
            let x0 = row[0].clone().mul_cached(&inv_a00);
            let x1 = (row[1].clone() - (&x0 * &right[0][1])).mul_cached(&inv_a11);
            let x2 = Scalar::active_signed_product_sum2(
                [true, false, false],
                [[&row[2], &one], [&x0, &right[0][2]], [&x1, &right[1][2]]],
            )
            .mul_cached(&inv_a22);
            let x3 = Scalar::active_signed_product_sum2(
                [true, false, false, false],
                [
                    [&row[3], &one],
                    [&x0, &right[0][3]],
                    [&x1, &right[1][3]],
                    [&x2, &right[2][3]],
                ],
            )
            .mul_cached(&inv_a33);
            *row = [x0, x1, x2, x3];
        }
        return Ok(left);
    }
    let inv_diagonal = [inv_a00, inv_a11, inv_a22, inv_a33];

    for row in 0..4 {
        for col in 0..4 {
            let mut value = left[row][col].clone();
            for k in 0..col {
                value = value - (&left[row][k] * &right[k][col]);
            }
            left[row][col] = value.mul_cached(&inv_diagonal[col]);
        }
    }
    Ok(left)
}

#[inline]
fn divide_matrix4_affine_upper_row<B: Backend>(
    row: &[Scalar<B>; 4],
    right: &[[Scalar<B>; 4]; 4],
    inv_a00: &Scalar<B>,
    inv_a11: &Scalar<B>,
    inv_a22: &Scalar<B>,
    one: &Scalar<B>,
) -> [Scalar<B>; 4] {
    let x0 = row[0].clone().mul_cached(inv_a00);
    let x1 = (row[1].clone() - (&x0 * &right[0][1])).mul_cached(inv_a11);
    let x2 = if B::FUSE_SIGNED_PRODUCT_SUM {
        Scalar::active_signed_product_sum2(
            [true, false, false],
            [[&row[2], one], [&x0, &right[0][2]], [&x1, &right[1][2]]],
        )
        .mul_cached(inv_a22)
    } else {
        (row[2].clone() - (&x0 * &right[0][2]) - (&x1 * &right[1][2])).mul_cached(inv_a22)
    };
    let x3 = if B::FUSE_SIGNED_PRODUCT_SUM {
        Scalar::active_signed_product_sum2(
            [true, false, false, false],
            [
                [&row[3], one],
                [&x0, &right[0][3]],
                [&x1, &right[1][3]],
                [&x2, &right[2][3]],
            ],
        )
    } else {
        row[3].clone() - (&x0 * &right[0][3]) - (&x1 * &right[1][3]) - (&x2 * &right[2][3])
    };
    [x0, x1, x2, x3]
}

#[inline]
fn divide_matrix4_by_affine_upper_triangular<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    // A 3D affine upper-triangular divisor has homogeneous diagonal one and
    // zero bottom-row support. Solve only the three linear diagonal lanes and
    // leave the translation column as a fused affine update.
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    let inv_a22 = right[2][2].clone().inverse()?;
    if B::FUSE_SIGNED_PRODUCT_SUM {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "divide4-affine-upper-triangular-fused-exact"
        );
    }
    let one = Scalar::one();
    Ok([
        divide_matrix4_affine_upper_row(&left[0], right, &inv_a00, &inv_a11, &inv_a22, &one),
        divide_matrix4_affine_upper_row(&left[1], right, &inv_a00, &inv_a11, &inv_a22, &one),
        divide_matrix4_affine_upper_row(&left[2], right, &inv_a00, &inv_a11, &inv_a22, &one),
        divide_matrix4_affine_upper_row(&left[3], right, &inv_a00, &inv_a11, &inv_a22, &one),
    ])
}

#[inline]
fn divide_matrix4_affine_by_affine_upper_triangular<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    let inv_a22 = right[2][2].clone().inverse()?;
    if B::FUSE_SIGNED_PRODUCT_SUM {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "divide4-affine-left-affine-upper-triangular-fused-exact"
        );
    }
    let one = Scalar::one();
    Ok([
        divide_matrix4_affine_upper_row(&left[0], right, &inv_a00, &inv_a11, &inv_a22, &one),
        divide_matrix4_affine_upper_row(&left[1], right, &inv_a00, &inv_a11, &inv_a22, &one),
        divide_matrix4_affine_upper_row(&left[2], right, &inv_a00, &inv_a11, &inv_a22, &one),
        [Scalar::zero(), Scalar::zero(), Scalar::zero(), one],
    ])
}

#[inline]
fn divide_matrix4_by_affine_upper_triangular_checked<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    require_known_nonzero(&right[2][2])?;
    divide_matrix4_by_affine_upper_triangular(left, right)
}

#[inline]
fn divide_matrix4_affine_by_affine_upper_triangular_checked<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    require_known_nonzero(&right[2][2])?;
    divide_matrix4_affine_by_affine_upper_triangular(left, right)
}

#[inline]
fn divide_matrix4_by_affine_upper_triangular_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    require_known_nonzero_with_abort(&right[0][0], signal)?;
    require_known_nonzero_with_abort(&right[1][1], signal)?;
    require_known_nonzero_with_abort(&right[2][2], signal)?;
    divide_matrix4_by_affine_upper_triangular(left, right)
}

#[inline]
fn divide_matrix4_affine_by_affine_upper_triangular_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    require_known_nonzero_with_abort(&right[0][0], signal)?;
    require_known_nonzero_with_abort(&right[1][1], signal)?;
    require_known_nonzero_with_abort(&right[2][2], signal)?;
    divide_matrix4_affine_by_affine_upper_triangular(left, right)
}

#[inline]
fn divide_matrix4_by_upper_triangular_checked<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    require_known_nonzero(&right[2][2])?;
    require_known_nonzero(&right[3][3])?;
    divide_matrix4_by_upper_triangular(left, right)
}

#[inline]
fn divide_matrix4_by_upper_triangular_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    require_known_nonzero_with_abort(&right[0][0], signal)?;
    require_known_nonzero_with_abort(&right[1][1], signal)?;
    require_known_nonzero_with_abort(&right[2][2], signal)?;
    require_known_nonzero_with_abort(&right[3][3], signal)?;
    divide_matrix4_by_upper_triangular(left, right)
}

#[inline]
fn divide_matrix4_by_lower_triangular<B: Backend>(
    mut left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    // Fixed-size lower-triangular right-division is row-wise forward substitution.
    // The same structural complexity win used for upper triangular applies
    // symmetrically. Solve each column with the strict row-order recurrence.
    // Golub & Van Loan, *Matrix Computations* (4th ed.), §3.6.
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    let inv_a22 = right[2][2].clone().inverse()?;
    let inv_a33 = right[3][3].clone().inverse()?;
    if B::FUSE_SIGNED_PRODUCT_SUM {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "divide4-lower-triangular-fused-exact"
        );
        let one = Scalar::one();
        for row in &mut left {
            let x3 = row[3].clone().mul_cached(&inv_a33);
            let x2 = (row[2].clone() - (&x3 * &right[3][2])).mul_cached(&inv_a22);
            let x1 = Scalar::active_signed_product_sum2(
                [true, false, false],
                [[&row[1], &one], [&x2, &right[2][1]], [&x3, &right[3][1]]],
            )
            .mul_cached(&inv_a11);
            let x0 = Scalar::active_signed_product_sum2(
                [true, false, false, false],
                [
                    [&row[0], &one],
                    [&x1, &right[1][0]],
                    [&x2, &right[2][0]],
                    [&x3, &right[3][0]],
                ],
            )
            .mul_cached(&inv_a00);
            *row = [x0, x1, x2, x3];
        }
        return Ok(left);
    }
    let inv_diagonal = [inv_a00, inv_a11, inv_a22, inv_a33];

    for row in 0..4 {
        for col in (0..4).rev() {
            let mut value = left[row][col].clone();
            for k in (col + 1)..4 {
                value = value - (&left[row][k] * &right[k][col]);
            }
            left[row][col] = value.mul_cached(&inv_diagonal[col]);
        }
    }
    Ok(left)
}

#[inline]
fn divide_matrix4_by_lower_triangular_checked<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    require_known_nonzero(&right[2][2])?;
    require_known_nonzero(&right[3][3])?;
    divide_matrix4_by_lower_triangular(left, right)
}

#[inline]
fn divide_matrix4_by_lower_triangular_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    require_known_nonzero_with_abort(&right[0][0], signal)?;
    require_known_nonzero_with_abort(&right[1][1], signal)?;
    require_known_nonzero_with_abort(&right[2][2], signal)?;
    require_known_nonzero_with_abort(&right[3][3], signal)?;
    divide_matrix4_by_lower_triangular(left, right)
}

fn right_divide_matrix3<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: [[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    let right_facts = matrix3_facts(&right);

    if right_facts.is_identity {
        crate::trace_dispatch!("hyperlattice_matrix", "helper", "right-divide3-identity");
        return Ok(left);
    }
    if right_facts.is_diagonal {
        crate::trace_dispatch!("hyperlattice_matrix", "helper", "right-divide3-diagonal");
        return divide_matrix3_by_diagonal(left, &right);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix3_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-affine-left-affine-translation"
            );
            return divide_matrix3_affine_by_affine_translation(left, &right);
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-affine-by-translation"
        );
        return divide_matrix3_by_affine_translation(left, &right);
    }
    if right_facts.is_affine && right_facts.is_upper_triangular {
        let left_facts = matrix3_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-affine-left-affine-upper-triangular"
            );
            return divide_matrix3_affine_by_affine_upper_triangular(left, &right);
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-affine-upper-triangular"
        );
        return divide_matrix3_by_affine_upper_triangular(left, &right);
    }
    if right_facts.is_upper_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-upper-triangular"
        );
        // Exact triangular dispatch is cheaper than generic cofactor/Gauss-Jordan
        // for structurally triangular divisors. This is the same structural-first
        // principle used by triangular solve kernels in direct methods.
        return divide_matrix3_by_upper_triangular(left, &right);
    }
    if right_facts.is_lower_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-lower-triangular"
        );
        return divide_matrix3_by_lower_triangular(left, &right);
    }
    if right_facts.is_affine {
        // Left-side structural facts are only needed for affine dispatch, so
        // delay collecting them until after non-affine branches have been
        // eliminated.
        // This preserves structural short-circuiting in common dense matrix
        // workloads and aligns with the "defer expensive queries" policy in
        // exact geometric computation (Yap, 1997).
        let left_facts = matrix3_facts(&left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        let right_is_affine_translation = right_facts.is_affine_translation;
        crate::trace_dispatch!("hyperlattice_matrix", "helper", "right-divide3-affine");
        // Reuse the known structural fact for both left- and right-signed
        // branches to avoid rescanning the same affine predicate.
        if left_is_affine {
            if right_is_affine_translation {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide3-affine-left-affine-translation"
                );
                return divide_matrix3_affine_by_affine_translation(left, &right);
            }
            if right_linear_is_diagonal {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide3-affine-left-affine-linear-diagonal"
                );
                return divide_matrix3_affine_by_affine_linear_diagonal(left, &right);
            }
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-affine-left-affine"
            );
            return divide_matrix3_affine_by_affine(left, &right);
        }
        if right_is_affine_translation {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-affine-by-translation"
            );
            return divide_matrix3_by_affine_translation(left, &right);
        }
        if right_linear_is_diagonal {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-affine-linear-diagonal"
            );
            return divide_matrix3_by_affine_linear_diagonal(left, &right);
        }
        return divide_matrix3_by_affine(left, &right);
    }
    if !prefer_shared_adjugate_right_division(&left, &right) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-gauss-jordan"
        );
        return Ok(transpose_array3(solve_left_system3(
            transpose_array3(right),
            transpose_array3(left),
        )?));
    }

    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "right-divide3-shared-adjugate"
    );
    // Shared-scale prototype: compute `left * adj(right)` and distribute
    // `1/det(right)` only after the matrix product. Exact backends pay heavily
    // for each pivot inverse in Gauss-Jordan division, so this branch compares
    // one shared scalar inverse plus more multiplies against repeated pivot
    // normalization. Keep it only while matrix profile traces and Criterion
    // timings show wins.
    let (adjugate, det) = matrix3_adjugate_and_determinant(&right);
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(
        multiply_arrays3_with_exact_dense_certificate(left, adjugate),
        &inv_det,
    ))
}

fn right_divide_matrix3_ref<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    let right_facts = matrix3_facts(right);

    if right_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-ref-identity"
        );
        return Ok(left.clone());
    }
    if right_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-ref-diagonal"
        );
        return divide_matrix3_by_diagonal(left.clone(), right);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix3_facts(left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-ref-affine-left-affine-translation"
            );
            return divide_matrix3_affine_by_affine_ref_translation(left, right);
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-ref-affine-by-translation"
        );
        return divide_matrix3_by_affine_ref_translation(left, right);
    }
    if right_facts.is_affine && right_facts.is_upper_triangular {
        let left_facts = matrix3_facts(left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-ref-affine-left-affine-upper-triangular"
            );
            return divide_matrix3_affine_by_affine_upper_triangular(left.clone(), right);
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-ref-affine-upper-triangular"
        );
        return divide_matrix3_by_affine_upper_triangular(left.clone(), right);
    }
    if right_facts.is_upper_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-ref-upper-triangular"
        );
        return divide_matrix3_by_upper_triangular(left.clone(), right);
    }
    if right_facts.is_lower_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-ref-lower-triangular"
        );
        return divide_matrix3_by_lower_triangular(left.clone(), right);
    }
    if right_facts.is_affine {
        // Borrowed forms keep the same lazy policy as owned division so
        // non-affine right divisors avoid unnecessary structure probes.
        let left_facts = matrix3_facts(left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        let right_is_affine_translation = right_facts.is_affine_translation;
        crate::trace_dispatch!("hyperlattice_matrix", "helper", "right-divide3-ref-affine");
        // Same affine-flag reuse as owned division, preserving borrowed
        // dispatch shapes while avoiding duplicate `matrix3_is_affine` scans.
        if left_is_affine {
            if right_is_affine_translation {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide3-ref-affine-left-affine-translation"
                );
                return divide_matrix3_affine_by_affine_ref_translation(left, right);
            }
            if right_linear_is_diagonal {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide3-ref-affine-left-affine-linear-diagonal"
                );
                return divide_matrix3_affine_by_affine_ref_linear_diagonal(left, right);
            }
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-ref-affine-left-affine"
            );
            return divide_matrix3_affine_by_affine_ref_no_translation(left, right);
        }
        if right_is_affine_translation {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-ref-affine-by-translation"
            );
            return divide_matrix3_by_affine_ref_translation(left, right);
        }
        if right_linear_is_diagonal {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-ref-affine-linear-diagonal"
            );
            return divide_matrix3_by_affine_ref_linear_diagonal(left, right);
        }
        return divide_matrix3_by_affine_ref_no_translation(left, right);
    }
    if !prefer_shared_adjugate_right_division_ref3(left, right) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-ref-gauss-jordan"
        );
        // Borrowed right-division is implemented as a left solve on transposes.
        // Clone directly into transposed working storage instead of cloning both
        // matrices and dispatching through the owned `/` implementation.
        return Ok(transpose_array3(solve_left_system3(
            transpose_array3_ref(right),
            transpose_array3_ref(left),
        )?));
    }

    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "right-divide3-ref-shared-adjugate"
    );
    // Borrowed division keeps the left matrix borrowed through the product and
    // materializes only the divisor adjugate. This is the same shared-scale
    // experiment as the owned path, but avoids cloning both inputs before
    // transposed Gauss-Jordan elimination.
    let (adjugate, det) = matrix3_adjugate_and_determinant(right);
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(
        multiply_arrays3_ref_with_exact_dense_certificate(left, &adjugate),
        &inv_det,
    ))
}

fn right_divide_matrix3_checked<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: [[Scalar<B>; 3]; 3],
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    let right_facts = matrix3_facts(&right);

    if right_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-checked-identity"
        );
        return Ok(left);
    }
    if right_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-checked-diagonal"
        );
        return divide_matrix3_by_diagonal_checked(left, &right);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix3_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-checked-affine-left-affine-translation"
            );
            return divide_matrix3_affine_by_affine_translation(left, &right);
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-checked-affine-by-translation"
        );
        return divide_matrix3_by_affine_translation(left, &right);
    }
    if right_facts.is_affine && right_facts.is_upper_triangular {
        let left_facts = matrix3_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-checked-affine-left-affine-upper-triangular"
            );
            return divide_matrix3_affine_by_affine_upper_triangular_checked(left, &right);
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-checked-affine-upper-triangular"
        );
        return divide_matrix3_by_affine_upper_triangular_checked(left, &right);
    }
    if right_facts.is_upper_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-checked-upper-triangular"
        );
        return divide_matrix3_by_upper_triangular_checked(left, &right);
    }
    if right_facts.is_lower_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-checked-lower-triangular"
        );
        return divide_matrix3_by_lower_triangular_checked(left, &right);
    }
    if right_facts.is_affine {
        // Delay left-side structural facts until needed by affine right-divisor
        // handling so strict non-affine checked rows stay on a single fact-scan.
        // This structural laziness is consistent with deferred simplification
        // and sparse-path guidance in exact geometric computing literature.
        let left_facts = matrix3_facts(&left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-checked-affine"
        );
        // Keep branch classification shared across checked and checked-abort paths;
        // this is the cheapest way to reduce redundant structural queries on
        // large affine-matrix workloads.
        if left_is_affine {
            if right_linear_is_diagonal {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide3-checked-affine-left-affine-linear-diagonal"
                );
                return divide_matrix3_affine_by_affine_linear_diagonal_checked(left, &right);
            }
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-checked-affine-left-affine"
            );
            return divide_matrix3_affine_by_affine_checked(left, &right);
        }
        if right_linear_is_diagonal {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-checked-affine-linear-diagonal"
            );
            return divide_matrix3_by_affine_linear_diagonal_checked(left, &right);
        }
        return divide_matrix3_by_affine_checked(left, &right);
    }
    if !prefer_shared_adjugate_right_division(&left, &right) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-checked-gauss-jordan"
        );
        return Ok(transpose_array3(solve_left_system3_checked(
            transpose_array3(right),
            transpose_array3(left),
        )?));
    }

    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "right-divide3-checked-shared-adjugate"
    );
    let (adjugate, det) = matrix3_adjugate_and_determinant(&right);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(
        multiply_arrays3_with_exact_dense_certificate(left, adjugate),
        &inv_det,
    ))
}

fn right_divide_matrix3_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: [[Scalar<B>; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    let right_facts = matrix3_facts(&right);

    if right_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-checked-abort-identity"
        );
        return Ok(left);
    }
    if right_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-checked-abort-diagonal"
        );
        return divide_matrix3_by_diagonal_checked_with_abort(left, &right, signal);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix3_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-checked-abort-affine-left-affine-translation"
            );
            return divide_matrix3_affine_by_affine_translation(left, &right);
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-checked-abort-affine-by-translation"
        );
        return divide_matrix3_by_affine_translation(left, &right);
    }
    if right_facts.is_affine && right_facts.is_upper_triangular {
        let left_facts = matrix3_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-checked-abort-affine-left-affine-upper-triangular"
            );
            return divide_matrix3_affine_by_affine_upper_triangular_checked_with_abort(
                left, &right, signal,
            );
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-checked-abort-affine-upper-triangular"
        );
        return divide_matrix3_by_affine_upper_triangular_checked_with_abort(left, &right, signal);
    }
    if right_facts.is_upper_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-checked-abort-upper-triangular"
        );
        return divide_matrix3_by_upper_triangular_checked_with_abort(left, &right, signal);
    }
    if right_facts.is_lower_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-checked-abort-lower-triangular"
        );
        return divide_matrix3_by_lower_triangular_checked_with_abort(left, &right, signal);
    }
    if right_facts.is_affine {
        // Keep abort-aware checked code on the same fact-on-demand fast path as
        // its non-abort counterpart.
        let left_facts = matrix3_facts(&left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-checked-abort-affine"
        );
        // Cache the left structural fact once, because this branch remains hot in
        // iterative symbolic constraint pipelines.
        if left_is_affine {
            if right_linear_is_diagonal {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide3-checked-abort-affine-left-affine-linear-diagonal"
                );
                return divide_matrix3_affine_by_affine_linear_diagonal_checked_with_abort(
                    left, &right, signal,
                );
            }
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-checked-abort-affine-left-affine"
            );
            return divide_matrix3_affine_by_affine_checked_with_abort(left, &right, signal);
        }
        if right_linear_is_diagonal {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-checked-abort-affine-linear-diagonal"
            );
            return divide_matrix3_by_affine_linear_diagonal_checked_with_abort(
                left, &right, signal,
            );
        }
        return divide_matrix3_by_affine_checked_with_abort(left, &right, signal);
    }
    if !prefer_shared_adjugate_right_division(&left, &right) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-checked-abort-gauss-jordan"
        );
        return Ok(transpose_array3(solve_left_system3_checked_with_abort(
            transpose_array3(right),
            transpose_array3(left),
            signal,
        )?));
    }

    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "right-divide3-checked-abort-shared-adjugate"
    );
    let (adjugate, det) = matrix3_adjugate_and_determinant(&right);
    let det = with_abort(det, signal);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(
        multiply_arrays3_with_exact_dense_certificate(left, adjugate),
        &inv_det,
    ))
}

fn right_divide_matrix3_prepared<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    prepared: &mut PreparedRightDivisor3<B>,
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    let right_facts = prepared.facts;

    if right_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-identity"
        );
        return Ok(left);
    }
    if right_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-diagonal"
        );
        return divide_matrix3_by_diagonal(left, &prepared.divisor.0);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix3_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-prepared-affine-left-affine-translation"
            );
            return divide_matrix3_affine_by_affine_translation(left, &prepared.divisor.0);
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-affine-by-translation"
        );
        return divide_matrix3_by_affine_translation(left, &prepared.divisor.0);
    }
    if right_facts.is_affine && right_facts.is_upper_triangular {
        let left_facts = matrix3_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-prepared-affine-left-affine-upper-triangular"
            );
            return divide_matrix3_affine_by_affine_upper_triangular(left, &prepared.divisor.0);
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-affine-upper-triangular"
        );
        return divide_matrix3_by_affine_upper_triangular(left, &prepared.divisor.0);
    }
    if right_facts.is_upper_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-upper-triangular"
        );
        return divide_matrix3_by_upper_triangular(left, &prepared.divisor.0);
    }
    if right_facts.is_lower_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-lower-triangular"
        );
        return divide_matrix3_by_lower_triangular(left, &prepared.divisor.0);
    }
    if right_facts.is_affine {
        // Prepared handles already cache right-side facts, so we only compute
        // left-side facts when affine dispatch is selected.
        let left_facts = matrix3_facts(&left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        let right_is_affine_translation = right_facts.is_affine_translation;
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-affine"
        );
        if left_is_affine {
            if right_is_affine_translation {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide3-prepared-affine-left-affine-translation"
                );
                return divide_matrix3_affine_by_affine_translation(left, &prepared.divisor.0);
            }
            if right_linear_is_diagonal {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide3-prepared-affine-left-affine-linear-diagonal"
                );
                return divide_matrix3_affine_by_affine_linear_diagonal(left, &prepared.divisor.0);
            }
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-prepared-affine-left-affine"
            );
            return divide_matrix3_affine_by_affine(left, &prepared.divisor.0);
        }
        if right_is_affine_translation {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-prepared-affine-by-translation"
            );
            return divide_matrix3_by_affine_translation(left, &prepared.divisor.0);
        }
        if right_linear_is_diagonal {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-prepared-affine-linear-diagonal"
            );
            return divide_matrix3_by_affine_linear_diagonal(left, &prepared.divisor.0);
        }
        return divide_matrix3_by_affine(left, &prepared.divisor.0);
    }
    if !prepared.can_use_shared_adjugate(&left) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-gauss-jordan"
        );
        return Ok(transpose_array3(solve_left_system3(
            transpose_array3_ref(&prepared.divisor.0),
            transpose_array3(left),
        )?));
    }

    // Shared-adjugate is the intended win for repeated right-division by one divisor:
    // exact factor extraction is lifted out once, then each divisor application uses
    // one reciprocal and fixed-size matrix products. This is the same "delay common
    // scale" idea used in fraction-free elimination (Bareiss, 1968).
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "right-divide3-prepared-shared-adjugate"
    );
    let _ = prepared.prepare_shared_adjugate()?;
    let inv_det = prepared
        .reciprocal_determinant
        .as_ref()
        .expect("reciprocal determinant cache should be present");
    let adjugate = prepared
        .adjugate
        .as_ref()
        .expect("adjugate cache should be present");
    Ok(scale_matrix3(
        multiply_arrays3_rhs_ref_with_exact_dense_certificate(left, adjugate),
        inv_det,
    ))
}

fn right_divide_matrix3_prepared_checked<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    prepared: &mut PreparedRightDivisor3<B>,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    let right_facts = prepared.facts;

    if right_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-checked-identity"
        );
        return Ok(left);
    }
    if right_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-checked-diagonal"
        );
        return divide_matrix3_by_diagonal_checked(left, &prepared.divisor.0);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix3_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-prepared-checked-affine-left-affine-translation"
            );
            return divide_matrix3_affine_by_affine_translation(left, &prepared.divisor.0);
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-checked-affine-by-translation"
        );
        return divide_matrix3_by_affine_translation(left, &prepared.divisor.0);
    }
    if right_facts.is_affine && right_facts.is_upper_triangular {
        let left_facts = matrix3_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-prepared-checked-affine-left-affine-upper-triangular"
            );
            return divide_matrix3_affine_by_affine_upper_triangular_checked(
                left,
                &prepared.divisor.0,
            );
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-checked-affine-upper-triangular"
        );
        return divide_matrix3_by_affine_upper_triangular_checked(left, &prepared.divisor.0);
    }
    if right_facts.is_upper_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-checked-upper-triangular"
        );
        return divide_matrix3_by_upper_triangular_checked(left, &prepared.divisor.0);
    }
    if right_facts.is_lower_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-checked-lower-triangular"
        );
        return divide_matrix3_by_lower_triangular_checked(left, &prepared.divisor.0);
    }
    if right_facts.is_affine {
        // Keep checked prepared dispatch deterministic by computing left
        // structure only when required for affine fast paths.
        let left_facts = matrix3_facts(&left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-checked-affine"
        );
        if left_is_affine {
            if right_linear_is_diagonal {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide3-prepared-checked-affine-left-affine-linear-diagonal"
                );
                return divide_matrix3_affine_by_affine_linear_diagonal_checked(
                    left,
                    &prepared.divisor.0,
                );
            }
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-prepared-checked-affine-left-affine"
            );
            return divide_matrix3_affine_by_affine_checked(left, &prepared.divisor.0);
        }
        if right_linear_is_diagonal {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-prepared-checked-affine-linear-diagonal"
            );
            return divide_matrix3_by_affine_linear_diagonal_checked(left, &prepared.divisor.0);
        }
        return divide_matrix3_by_affine_checked(left, &prepared.divisor.0);
    }
    if !prepared.can_use_shared_adjugate(&left) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-checked-gauss-jordan"
        );
        return Ok(transpose_array3(solve_left_system3_checked(
            transpose_array3_ref(&prepared.divisor.0),
            transpose_array3(left),
        )?));
    }

    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "right-divide3-prepared-checked-shared-adjugate"
    );
    let _ = prepared.prepare_shared_adjugate_checked()?;
    let inv_det = prepared
        .reciprocal_determinant
        .as_ref()
        .expect("reciprocal determinant cache should be present");
    let adjugate = prepared
        .adjugate
        .as_ref()
        .expect("adjugate cache should be present");
    Ok(scale_matrix3(
        multiply_arrays3_rhs_ref_with_exact_dense_certificate(left, adjugate),
        inv_det,
    ))
}

fn right_divide_matrix3_prepared_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    prepared: &mut PreparedRightDivisor3<B>,
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    let right_facts = prepared.facts;

    if right_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-checked-abort-identity"
        );
        return Ok(left);
    }
    if right_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-checked-abort-diagonal"
        );
        return divide_matrix3_by_diagonal_checked_with_abort(left, &prepared.divisor.0, signal);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix3_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-prepared-checked-abort-affine-left-affine-translation"
            );
            return divide_matrix3_affine_by_affine_translation(left, &prepared.divisor.0);
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-checked-abort-affine-by-translation"
        );
        return divide_matrix3_by_affine_translation(left, &prepared.divisor.0);
    }
    if B::FUSE_SIGNED_PRODUCT_SUM && right_facts.is_affine && right_facts.is_upper_triangular {
        let left_facts = matrix3_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-prepared-checked-abort-affine-left-affine-upper-triangular"
            );
            return divide_matrix3_affine_by_affine_upper_triangular_checked_with_abort(
                left,
                &prepared.divisor.0,
                signal,
            );
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-checked-abort-affine-upper-triangular"
        );
        return divide_matrix3_by_affine_upper_triangular_checked_with_abort(
            left,
            &prepared.divisor.0,
            signal,
        );
    }
    if right_facts.is_upper_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-checked-abort-upper-triangular"
        );
        return divide_matrix3_by_upper_triangular_checked_with_abort(
            left,
            &prepared.divisor.0,
            signal,
        );
    }
    if right_facts.is_lower_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-checked-abort-lower-triangular"
        );
        return divide_matrix3_by_lower_triangular_checked_with_abort(
            left,
            &prepared.divisor.0,
            signal,
        );
    }
    if right_facts.is_affine {
        // Abort-aware prepared dispatch also reuses the same deferred probe
        // policy to avoid wasting structural queries for rows that already
        // match non-affine branches.
        let left_facts = matrix3_facts(&left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-checked-abort-affine"
        );
        if left_is_affine {
            if right_linear_is_diagonal {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide3-prepared-checked-abort-affine-left-affine-linear-diagonal"
                );
                return divide_matrix3_affine_by_affine_linear_diagonal_checked_with_abort(
                    left,
                    &prepared.divisor.0,
                    signal,
                );
            }
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-prepared-checked-abort-affine-left-affine"
            );
            return divide_matrix3_affine_by_affine_checked_with_abort(
                left,
                &prepared.divisor.0,
                signal,
            );
        }
        if right_linear_is_diagonal {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide3-prepared-checked-abort-affine-linear-diagonal"
            );
            return divide_matrix3_by_affine_linear_diagonal_checked_with_abort(
                left,
                &prepared.divisor.0,
                signal,
            );
        }
        return divide_matrix3_by_affine_checked_with_abort(left, &prepared.divisor.0, signal);
    }
    if !prepared.can_use_shared_adjugate(&left) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide3-prepared-checked-abort-gauss-jordan"
        );
        return Ok(transpose_array3(solve_left_system3_checked_with_abort(
            transpose_array3_ref(&prepared.divisor.0),
            transpose_array3(left),
            signal,
        )?));
    }

    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "right-divide3-prepared-checked-abort-shared-adjugate"
    );
    let _ = prepared.prepare_shared_adjugate_checked_with_abort(signal)?;
    let inv_det = prepared
        .reciprocal_determinant
        .as_ref()
        .expect("reciprocal determinant cache should be present");
    let adjugate = prepared
        .adjugate
        .as_ref()
        .expect("adjugate cache should be present");
    Ok(scale_matrix3(
        multiply_arrays3_rhs_ref_with_exact_dense_certificate(left, adjugate),
        inv_det,
    ))
}

fn right_divide_matrix4<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: [[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    if can_use_dense_exact_shared_adjugate4(&right) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-dense-exact-shared-adjugate"
        );
        return right_divide_matrix4_dense_exact_shared(&left, &right);
    }
    let right_facts = matrix4_facts(&right);
    if right_facts.is_identity {
        crate::trace_dispatch!("hyperlattice_matrix", "helper", "right-divide4-identity");
        return Ok(left);
    }
    if right_facts.is_diagonal {
        crate::trace_dispatch!("hyperlattice_matrix", "helper", "right-divide4-diagonal");
        return divide_matrix4_by_diagonal(left, &right);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix4_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-affine-left-affine-translation"
            );
            return divide_matrix4_affine_by_affine_translation(left, &right);
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-affine-by-translation"
        );
        return divide_matrix4_by_affine_translation(left, &right);
    }
    if B::FUSE_SIGNED_PRODUCT_SUM && right_facts.is_affine && right_facts.is_upper_triangular {
        let left_facts = matrix4_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-affine-left-affine-upper-triangular"
            );
            return divide_matrix4_affine_by_affine_upper_triangular(left, &right);
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-affine-upper-triangular"
        );
        return divide_matrix4_by_affine_upper_triangular(left, &right);
    }
    if right_facts.is_upper_triangular {
        // Right-dividing by an upper-triangular matrix is a collection of
        // triangular solves, with O(n²) complexity versus O(n³) for
        // adjugate-based cofactor routes. This is the exact same dispatch
        // policy used in triangular dense linear algebra kernels
        // (Golub & Van Loan, *Matrix Computations*, 4th ed., §4.2).
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-upper-triangular"
        );
        return divide_matrix4_by_upper_triangular(left, &right);
    }
    if right_facts.is_lower_triangular {
        // The lower-triangular branch is symmetric to the upper case and keeps
        // one-pass recurrence structure with cached diagonal reciprocals.
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-lower-triangular"
        );
        return divide_matrix4_by_lower_triangular(left, &right);
    }
    if right_facts.is_affine {
        // Left-side facts are only required for affine dispatch; keeping the
        // non-affine path down to a single right-fact scan avoids unnecessary
        // structural work and mirrors the deferred-symbolic strategy promoted in
        // Yap's exact geometric computation model.
        let left_facts = matrix4_facts(&left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        let right_is_affine_translation = right_facts.is_affine_translation;
        crate::trace_dispatch!("hyperlattice_matrix", "helper", "right-divide4-affine");
        // Reusing both affine flags cuts duplicate structural scans in mixed
        // geometric workloads. This follows the standard dispatcher pattern:
        // preserve expensive checks for once and reuse them across nearby
        // specializations (Golub & Van Loan, *Matrix Computations*).
        if left_is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-affine-left-affine"
            );
            if right_is_affine_translation {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide4-affine-left-affine-translation"
                );
                return divide_matrix4_affine_by_affine_translation(left, &right);
            }
            if right_linear_is_diagonal {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide4-affine-left-affine-linear-diagonal"
                );
                return divide_matrix4_affine_by_affine_linear_diagonal(left, &right);
            }
            return divide_matrix4_affine_by_affine_no_translation(left, &right);
        }
        if right_is_affine_translation {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-affine-by-translation"
            );
            return divide_matrix4_by_affine_translation(left, &right);
        }
        if right_linear_is_diagonal {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-affine-linear-diagonal"
            );
            if right_facts.affine_linear_diagonal_is_definitely_nonzero {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide4-affine-linear-diagonal-known-nonzero"
                );
                return divide_matrix4_by_affine_linear_diagonal(left, &right);
            }
            return divide_matrix4_by_affine_linear_diagonal_checked(left, &right);
        }
        return divide_matrix4_by_affine_no_translation(left, &right);
    }
    if !prefer_shared_adjugate_right_division(&left, &right) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-gauss-jordan"
        );
        return Ok(transpose_array4(solve_left_system4(
            transpose_array4(right),
            transpose_array4(left),
        )?));
    }

    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "right-divide4-shared-adjugate"
    );
    let dense_exact = B::FUSE_SIGNED_PRODUCT_SUM && right_facts.is_definitely_dense_for_inverse;
    let (s, c) = if dense_exact {
        matrix4_factors_dense_exact(&right)
    } else {
        matrix4_factors(&right)
    };
    let det = determinant4_from_factors(&s, &c);
    let inv_det = det.inverse()?;
    let adjugate = if dense_exact {
        matrix4_adjugate_from_factors_dense_exact(&right, &s, &c)
    } else {
        matrix4_adjugate_from_factors(&right, &s, &c)
    };
    Ok(scale_matrix4(
        multiply_arrays4_ref_with_dense_certificate(&left, &adjugate),
        &inv_det,
    ))
}

#[inline]
fn can_use_dense_exact_shared_adjugate4<B: Backend>(right: &[[Scalar<B>; 4]; 4]) -> bool {
    B::FUSE_SIGNED_PRODUCT_SUM
        && matrix4_is_definitely_dense_for_inverse(right)
        && matrix4_exact_rational_kind(right) != ExactRationalKind::NonRational
}

#[inline]
fn right_divide_matrix4_dense_exact_shared<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    let (s, c) = matrix4_factors_dense_exact_known_rational(right);
    let det = determinant4_from_factors_known_rational(&s, &c);
    let inv_det = det.inverse()?;
    let adjugate = matrix4_adjugate_from_factors_dense_exact_known_rational(right, &s, &c);
    let product = if matrix4_exact_rational_kind(left) != ExactRationalKind::NonRational {
        multiply_arrays4_dense_known_rational_ref(left, &adjugate)
    } else {
        multiply_arrays4_ref_with_dense_certificate(left, &adjugate)
    };
    Ok(scale_matrix4(product, &inv_det))
}

#[inline]
fn right_divide_matrix4_dense_exact_shared_checked<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    let (s, c) = matrix4_factors_dense_exact_known_rational(right);
    let det = determinant4_from_factors_known_rational(&s, &c);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    let adjugate = matrix4_adjugate_from_factors_dense_exact_known_rational(right, &s, &c);
    let product = if matrix4_exact_rational_kind(left) != ExactRationalKind::NonRational {
        multiply_arrays4_dense_known_rational_ref(left, &adjugate)
    } else {
        multiply_arrays4_ref_with_dense_certificate(left, &adjugate)
    };
    Ok(scale_matrix4(product, &inv_det))
}

#[inline]
fn right_divide_matrix4_dense_exact_shared_checked_with_abort<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    let (s, c) = matrix4_factors_dense_exact_known_rational(right);
    let det = with_abort(determinant4_from_factors_known_rational(&s, &c), signal);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    let adjugate = matrix4_adjugate_from_factors_dense_exact_known_rational(right, &s, &c);
    let product = if matrix4_exact_rational_kind(left) != ExactRationalKind::NonRational {
        multiply_arrays4_dense_known_rational_ref(left, &adjugate)
    } else {
        multiply_arrays4_ref_with_dense_certificate(left, &adjugate)
    };
    Ok(scale_matrix4(product, &inv_det))
}

fn right_divide_matrix4_prepared<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    prepared: &mut PreparedRightDivisor4<B>,
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    let right_facts = prepared.facts;

    if right_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-identity"
        );
        return Ok(left);
    }
    if right_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-diagonal"
        );
        return divide_matrix4_by_diagonal(left, &prepared.divisor.0);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix4_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-prepared-affine-left-affine-translation"
            );
            return divide_matrix4_affine_by_affine_translation(left, &prepared.divisor.0);
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-affine-by-translation"
        );
        return divide_matrix4_by_affine_translation(left, &prepared.divisor.0);
    }
    if B::FUSE_SIGNED_PRODUCT_SUM && right_facts.is_affine && right_facts.is_upper_triangular {
        let left_facts = matrix4_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-prepared-affine-left-affine-upper-triangular"
            );
            return divide_matrix4_affine_by_affine_upper_triangular(left, &prepared.divisor.0);
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-affine-upper-triangular"
        );
        return divide_matrix4_by_affine_upper_triangular(left, &prepared.divisor.0);
    }
    if right_facts.is_upper_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-upper-triangular"
        );
        return divide_matrix4_by_upper_triangular(left, &prepared.divisor.0);
    }
    if right_facts.is_lower_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-lower-triangular"
        );
        return divide_matrix4_by_lower_triangular(left, &prepared.divisor.0);
    }
    if right_facts.is_affine {
        // Prepared right facts already include all divisor structure, so defer
        // left-structure extraction until this branch.
        let left_facts = matrix4_facts(&left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        let right_is_affine_translation = right_facts.is_affine_translation;
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-affine"
        );
        if left_is_affine {
            if right_is_affine_translation {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide4-prepared-affine-left-affine-translation"
                );
                return divide_matrix4_affine_by_affine_translation(left, &prepared.divisor.0);
            }
            if right_linear_is_diagonal {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide4-prepared-affine-left-affine-linear-diagonal"
                );
                return divide_matrix4_affine_by_affine_linear_diagonal(left, &prepared.divisor.0);
            }
            return divide_matrix4_affine_by_affine_no_translation(left, &prepared.divisor.0);
        }
        if right_is_affine_translation {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-prepared-affine-by-translation"
            );
            return divide_matrix4_by_affine_translation(left, &prepared.divisor.0);
        }
        if right_linear_is_diagonal {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-prepared-affine-linear-diagonal"
            );
            if right_facts.affine_linear_diagonal_is_definitely_nonzero {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide4-prepared-affine-linear-diagonal-known-nonzero"
                );
                return divide_matrix4_by_affine_linear_diagonal(left, &prepared.divisor.0);
            }
            return divide_matrix4_by_affine_linear_diagonal_checked(left, &prepared.divisor.0);
        }
        return divide_matrix4_by_affine_no_translation(left, &prepared.divisor.0);
    }
    if !prepared.can_use_shared_adjugate(&left) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-gauss-jordan"
        );
        return Ok(transpose_array4(solve_left_system4(
            transpose_array4_ref(&prepared.divisor.0),
            transpose_array4(left),
        )?));
    }

    // The shared-adjugate route can keep exact dyadic workloads flat by reusing
    // one factorization and one scalar reciprocal across repeated right
    // divisons against the same 4×4 divisor. See Bareiss, "Sylvester's
    // identity and fraction-free Gaussian elimination" lineage, and Yap 1997.
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "right-divide4-prepared-shared-adjugate"
    );
    let _ = prepared.prepare_shared_adjugate()?;
    let inv_det = prepared
        .reciprocal_determinant
        .as_ref()
        .expect("reciprocal determinant cache should be present");
    let adjugate = prepared
        .adjugate
        .as_ref()
        .expect("adjugate cache should be present");
    let product = if prepared.right_exact_rational_kind != ExactRationalKind::NonRational
        && matrix4_exact_rational_kind(&left) != ExactRationalKind::NonRational
    {
        multiply_arrays4_dense_known_rational_ref(&left, adjugate)
    } else {
        multiply_arrays4_rhs_ref_with_dense_certificate(left, adjugate)
    };
    Ok(scale_matrix4(product, inv_det))
}

fn right_divide_matrix4_prepared_exact_rational_left<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    prepared: &mut PreparedRightDivisor4<B>,
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    if B::FUSE_SIGNED_PRODUCT_SUM
        && prepared.right_exact_rational_kind != ExactRationalKind::NonRational
    {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-certified-left-exact-shared-adjugate"
        );
        let _ = prepared.prepare_shared_adjugate()?;
        let inv_det = prepared
            .reciprocal_determinant
            .as_ref()
            .expect("reciprocal determinant cache should be present");
        let adjugate = prepared
            .adjugate
            .as_ref()
            .expect("adjugate cache should be present");
        let product = multiply_arrays4_dense_known_rational_ref(&left, adjugate);
        return Ok(scale_matrix4(product, inv_det));
    }

    right_divide_matrix4_prepared(left, prepared)
}

fn right_divide_matrix4_prepared_checked<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    prepared: &mut PreparedRightDivisor4<B>,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    let right_facts = prepared.facts;

    if right_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-checked-identity"
        );
        return Ok(left);
    }
    if right_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-checked-diagonal"
        );
        return divide_matrix4_by_diagonal_checked(left, &prepared.divisor.0);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix4_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-prepared-checked-affine-left-affine-translation"
            );
            return divide_matrix4_affine_by_affine_checked_assumed_affine_translation(
                left,
                &prepared.divisor.0,
            );
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-checked-affine-by-translation"
        );
        return divide_matrix4_by_affine_checked_assumed_affine_translation(
            left,
            &prepared.divisor.0,
        );
    }
    if B::FUSE_SIGNED_PRODUCT_SUM && right_facts.is_affine && right_facts.is_upper_triangular {
        let left_facts = matrix4_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-prepared-checked-affine-left-affine-upper-triangular"
            );
            return divide_matrix4_affine_by_affine_upper_triangular_checked(
                left,
                &prepared.divisor.0,
            );
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-checked-affine-upper-triangular"
        );
        return divide_matrix4_by_affine_upper_triangular_checked(left, &prepared.divisor.0);
    }
    if right_facts.is_upper_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-checked-upper-triangular"
        );
        return divide_matrix4_by_upper_triangular_checked(left, &prepared.divisor.0);
    }
    if right_facts.is_lower_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-checked-lower-triangular"
        );
        return divide_matrix4_by_lower_triangular_checked(left, &prepared.divisor.0);
    }
    if right_facts.is_affine {
        // Keep checked prepared division factored, probing left-side affine
        // structure only where it changes dispatch.
        let left_facts = matrix4_facts(&left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        let right_is_affine_translation = right_facts.is_affine_translation;
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-checked-affine"
        );
        if left_is_affine {
            if right_is_affine_translation {
                return divide_matrix4_affine_by_affine_checked_assumed_affine_translation(
                    left,
                    &prepared.divisor.0,
                );
            }
            if right_linear_is_diagonal {
                if right_facts.affine_linear_diagonal_is_definitely_nonzero {
                    crate::trace_dispatch!(
                        "hyperlattice_matrix",
                        "helper",
                        "right-divide4-prepared-checked-affine-left-affine-linear-diagonal-known-nonzero"
                    );
                    return divide_matrix4_affine_by_affine_linear_diagonal(
                        left,
                        &prepared.divisor.0,
                    );
                }
                return divide_matrix4_affine_by_affine_linear_diagonal_checked(
                    left,
                    &prepared.divisor.0,
                );
            }
            return divide_matrix4_affine_by_affine_checked(left, &prepared.divisor.0);
        }
        if right_is_affine_translation {
            return divide_matrix4_by_affine_checked_assumed_affine_translation(
                left,
                &prepared.divisor.0,
            );
        }
        if right_linear_is_diagonal {
            if right_facts.affine_linear_diagonal_is_definitely_nonzero {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide4-prepared-checked-affine-linear-diagonal-known-nonzero"
                );
                return divide_matrix4_by_affine_linear_diagonal(left, &prepared.divisor.0);
            }
            return divide_matrix4_by_affine_linear_diagonal_checked(left, &prepared.divisor.0);
        }
        return divide_matrix4_by_affine_checked(left, &prepared.divisor.0);
    }
    if !prepared.can_use_shared_adjugate(&left) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-checked-gauss-jordan"
        );
        return Ok(transpose_array4(solve_left_system4_checked(
            transpose_array4_ref(&prepared.divisor.0),
            transpose_array4(left),
        )?));
    }

    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "right-divide4-prepared-checked-shared-adjugate"
    );
    let _ = prepared.prepare_shared_adjugate_checked()?;
    let inv_det = prepared
        .reciprocal_determinant
        .as_ref()
        .expect("reciprocal determinant cache should be present");
    let adjugate = prepared
        .adjugate
        .as_ref()
        .expect("adjugate cache should be present");
    let product = if prepared.right_exact_rational_kind != ExactRationalKind::NonRational
        && matrix4_exact_rational_kind(&left) != ExactRationalKind::NonRational
    {
        multiply_arrays4_dense_known_rational_ref(&left, adjugate)
    } else {
        multiply_arrays4_rhs_ref_with_dense_certificate(left, adjugate)
    };
    Ok(scale_matrix4(product, inv_det))
}

fn right_divide_matrix4_prepared_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    prepared: &mut PreparedRightDivisor4<B>,
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    let right_facts = prepared.facts;

    if right_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-checked-abort-identity"
        );
        return Ok(left);
    }
    if right_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-checked-abort-diagonal"
        );
        return divide_matrix4_by_diagonal_checked_with_abort(left, &prepared.divisor.0, signal);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix4_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-prepared-checked-abort-affine-left-affine-translation"
            );
            return divide_matrix4_affine_by_affine_checked_with_abort_assumed_affine_translation(
                left,
                &prepared.divisor.0,
                signal,
            );
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-checked-abort-affine-by-translation"
        );
        return divide_matrix4_by_affine_checked_with_abort_assumed_affine_translation(
            left,
            &prepared.divisor.0,
            signal,
        );
    }
    if B::FUSE_SIGNED_PRODUCT_SUM && right_facts.is_affine && right_facts.is_upper_triangular {
        let left_facts = matrix4_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-prepared-checked-abort-affine-left-affine-upper-triangular"
            );
            return divide_matrix4_affine_by_affine_upper_triangular_checked_with_abort(
                left,
                &prepared.divisor.0,
                signal,
            );
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-checked-abort-affine-upper-triangular"
        );
        return divide_matrix4_by_affine_upper_triangular_checked_with_abort(
            left,
            &prepared.divisor.0,
            signal,
        );
    }
    if right_facts.is_upper_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-checked-abort-upper-triangular"
        );
        return divide_matrix4_by_upper_triangular_checked_with_abort(
            left,
            &prepared.divisor.0,
            signal,
        );
    }
    if right_facts.is_lower_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-checked-abort-lower-triangular"
        );
        return divide_matrix4_by_lower_triangular_checked_with_abort(
            left,
            &prepared.divisor.0,
            signal,
        );
    }
    if right_facts.is_affine {
        // Abort-aware prepared paths preserve the same fact-on-demand dispatch
        // policy as unchecked prepared division.
        let left_facts = matrix4_facts(&left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        let right_is_affine_translation = right_facts.is_affine_translation;
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-checked-abort-affine"
        );
        if left_is_affine {
            if right_is_affine_translation {
                return divide_matrix4_affine_by_affine_checked_with_abort_assumed_affine_translation(
                    left,
                    &prepared.divisor.0,
                    signal,
                );
            }
            if right_linear_is_diagonal {
                if right_facts.affine_linear_diagonal_is_definitely_nonzero {
                    crate::trace_dispatch!(
                        "hyperlattice_matrix",
                        "helper",
                        "right-divide4-prepared-checked-abort-affine-left-affine-linear-diagonal-known-nonzero"
                    );
                    return divide_matrix4_affine_by_affine_linear_diagonal(
                        left,
                        &prepared.divisor.0,
                    );
                }
                return divide_matrix4_affine_by_affine_linear_diagonal_checked_with_abort(
                    left,
                    &prepared.divisor.0,
                    signal,
                );
            }
            return divide_matrix4_affine_by_affine_checked_with_abort(
                left,
                &prepared.divisor.0,
                signal,
            );
        }
        if right_is_affine_translation {
            return divide_matrix4_by_affine_checked_with_abort_assumed_affine_translation(
                left,
                &prepared.divisor.0,
                signal,
            );
        }
        if right_linear_is_diagonal {
            if right_facts.affine_linear_diagonal_is_definitely_nonzero {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide4-prepared-checked-abort-affine-linear-diagonal-known-nonzero"
                );
                return divide_matrix4_by_affine_linear_diagonal(left, &prepared.divisor.0);
            }
            return divide_matrix4_by_affine_linear_diagonal_checked_with_abort(
                left,
                &prepared.divisor.0,
                signal,
            );
        }
        return divide_matrix4_by_affine_checked_with_abort(left, &prepared.divisor.0, signal);
    }
    if !prepared.can_use_shared_adjugate(&left) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-prepared-checked-abort-gauss-jordan"
        );
        return Ok(transpose_array4(solve_left_system4_checked_with_abort(
            transpose_array4_ref(&prepared.divisor.0),
            transpose_array4(left),
            signal,
        )?));
    }

    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "right-divide4-prepared-checked-abort-shared-adjugate"
    );
    let _ = prepared.prepare_shared_adjugate_checked_with_abort(signal)?;
    let inv_det = prepared
        .reciprocal_determinant
        .as_ref()
        .expect("reciprocal determinant cache should be present");
    let adjugate = prepared
        .adjugate
        .as_ref()
        .expect("adjugate cache should be present");
    let product = if prepared.right_exact_rational_kind != ExactRationalKind::NonRational
        && matrix4_exact_rational_kind(&left) != ExactRationalKind::NonRational
    {
        multiply_arrays4_dense_known_rational_ref(&left, adjugate)
    } else {
        multiply_arrays4_rhs_ref_with_dense_certificate(left, adjugate)
    };
    Ok(scale_matrix4(product, inv_det))
}

fn right_divide_matrix4_ref<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    if can_use_dense_exact_shared_adjugate4(right) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-ref-dense-exact-shared-adjugate"
        );
        return right_divide_matrix4_dense_exact_shared(left, right);
    }
    let right_facts = matrix4_facts(right);
    if right_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-ref-identity"
        );
        return Ok(left.clone());
    }
    if right_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-ref-diagonal"
        );
        return divide_matrix4_by_diagonal(left.clone(), right);
    }
    if B::FUSE_SIGNED_PRODUCT_SUM && right_facts.is_affine && right_facts.is_upper_triangular {
        let left_facts = matrix4_facts(left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-ref-affine-left-affine-upper-triangular"
            );
            return divide_matrix4_affine_by_affine_upper_triangular(left.clone(), right);
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-ref-affine-upper-triangular"
        );
        return divide_matrix4_by_affine_upper_triangular(left.clone(), right);
    }
    if right_facts.is_upper_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-ref-upper-triangular"
        );
        return divide_matrix4_by_upper_triangular(left.clone(), right);
    }
    if right_facts.is_lower_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-ref-lower-triangular"
        );
        return divide_matrix4_by_lower_triangular(left.clone(), right);
    }
    if right_facts.is_affine {
        // Defer structural inspection of `left` to keep non-affine fast paths free of an
        // extra matrix scan; this avoids unnecessary work and follows standard "avoid wasted
        // work for structurally-typed dispatch" guidance (Golub and Van Loan, Matrix
        // Computations, 2013).
        let left_facts = matrix4_facts(left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        let right_is_affine_translation = right_facts.is_affine_translation;
        crate::trace_dispatch!("hyperlattice_matrix", "helper", "right-divide4-ref-affine");
        if left_is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-ref-affine-left-affine"
            );
            if right_is_affine_translation {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide4-ref-affine-by-affine-translation"
                );
                return divide_matrix4_affine_by_affine_ref_translation(left, right);
            }
            if right_linear_is_diagonal {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide4-ref-affine-left-affine-linear-diagonal"
                );
                return divide_matrix4_affine_by_affine_ref_linear_diagonal(left, right);
            }
            return divide_matrix4_affine_by_affine_ref_no_translation(left, right);
        }
        if right_is_affine_translation {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-ref-by-affine-translation"
            );
            return divide_matrix4_by_affine_ref_assumed_affine_translation(left, right);
        }
        if right_linear_is_diagonal {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-ref-affine-linear-diagonal"
            );
            return divide_matrix4_by_affine_linear_diagonal_ref(left, right);
        }
        return divide_matrix4_by_affine_ref_no_translation(left, right);
    }
    if !prefer_shared_adjugate_right_division(left, right) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-ref-gauss-jordan"
        );
        // Same borrowed right-division shortcut as 3x3, with unrolled 4x4
        // transposes. The adjugate route is kept only for dyadic inputs.
        return Ok(transpose_array4(solve_left_system4(
            transpose_array4_ref(right),
            transpose_array4_ref(left),
        )?));
    }

    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "right-divide4-ref-shared-adjugate"
    );
    // The 4x4 cofactor route does substantially more scalar multiplication
    // than Gauss-Jordan, but it carries one shared determinant inverse. This
    // branch is intentionally isolated so trace rows can decide whether exact
    // rational normalization or scalar op count dominates.
    let dense_exact = B::FUSE_SIGNED_PRODUCT_SUM && right_facts.is_definitely_dense_for_inverse;
    let (s, c) = if dense_exact {
        matrix4_factors_dense_exact(right)
    } else {
        matrix4_factors(right)
    };
    let det = determinant4_from_factors(&s, &c);
    let inv_det = det.inverse()?;
    let adjugate = if dense_exact {
        matrix4_adjugate_from_factors_dense_exact(right, &s, &c)
    } else {
        matrix4_adjugate_from_factors(right, &s, &c)
    };
    Ok(scale_matrix4(
        multiply_arrays4_ref_with_dense_certificate(left, &adjugate),
        &inv_det,
    ))
}

fn right_divide_matrix4_checked<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: [[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    if can_use_dense_exact_shared_adjugate4(&right) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-checked-dense-exact-shared-adjugate"
        );
        return right_divide_matrix4_dense_exact_shared_checked(&left, &right);
    }
    let right_facts = matrix4_facts(&right);
    if right_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-checked-identity"
        );
        return Ok(left);
    }
    if right_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-checked-diagonal"
        );
        return divide_matrix4_by_diagonal_checked(left, &right);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix4_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-checked-affine-left-affine-translation"
            );
            return divide_matrix4_affine_by_affine_checked_assumed_affine_translation(
                left, &right,
            );
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-checked-affine-by-translation"
        );
        return divide_matrix4_by_affine_checked_assumed_affine_translation(left, &right);
    }
    if B::FUSE_SIGNED_PRODUCT_SUM && right_facts.is_affine && right_facts.is_upper_triangular {
        let left_facts = matrix4_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-checked-affine-left-affine-upper-triangular"
            );
            return divide_matrix4_affine_by_affine_upper_triangular_checked(left, &right);
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-checked-affine-upper-triangular"
        );
        return divide_matrix4_by_affine_upper_triangular_checked(left, &right);
    }
    if right_facts.is_upper_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-checked-upper-triangular"
        );
        return divide_matrix4_by_upper_triangular_checked(left, &right);
    }
    if right_facts.is_lower_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-checked-lower-triangular"
        );
        return divide_matrix4_by_lower_triangular_checked(left, &right);
    }
    if right_facts.is_affine {
        // Defer left-side structural probe until the affine branch to preserve Yap-style fast
        // pathing and avoid materializing facts for matrix divisions that dispatch through
        // cheaper triangular/cofactor routes.
        // See: Yap, "Toward an API for Real Number Types", 1994.
        let left_facts = matrix4_facts(&left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        let right_is_affine_translation = right_facts.is_affine_translation;
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-checked-affine"
        );
        if left_is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-checked-affine-left-affine"
            );
            if right_is_affine_translation {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide4-checked-affine-by-affine-translation"
                );
                return divide_matrix4_affine_by_affine_checked_assumed_affine_translation(
                    left, &right,
                );
            }
            if right_linear_is_diagonal {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide4-checked-affine-left-affine-linear-diagonal"
                );
                if right_facts.affine_linear_diagonal_is_definitely_nonzero {
                    crate::trace_dispatch!(
                        "hyperlattice_matrix",
                        "helper",
                        "right-divide4-checked-affine-left-affine-linear-diagonal-known-nonzero"
                    );
                    return divide_matrix4_affine_by_affine_linear_diagonal(left, &right);
                }
                return divide_matrix4_affine_by_affine_linear_diagonal_checked(left, &right);
            }
            return divide_matrix4_affine_by_affine_checked(left, &right);
        }
        if right_is_affine_translation {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-checked-by-affine-translation"
            );
            return divide_matrix4_by_affine_checked_assumed_affine_translation(left, &right);
        }
        if right_linear_is_diagonal {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-checked-affine-linear-diagonal"
            );
            if right_facts.affine_linear_diagonal_is_definitely_nonzero {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide4-checked-affine-linear-diagonal-known-nonzero"
                );
                return divide_matrix4_by_affine_linear_diagonal(left, &right);
            }
            return divide_matrix4_by_affine_linear_diagonal_checked(left, &right);
        }
        return divide_matrix4_by_affine_checked(left, &right);
    }
    if !prefer_shared_adjugate_right_division(&left, &right) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-checked-gauss-jordan"
        );
        return Ok(transpose_array4(solve_left_system4_checked(
            transpose_array4(right),
            transpose_array4(left),
        )?));
    }

    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "right-divide4-checked-shared-adjugate"
    );
    let dense_exact = B::FUSE_SIGNED_PRODUCT_SUM && right_facts.is_definitely_dense_for_inverse;
    let (s, c) = if dense_exact {
        matrix4_factors_dense_exact(&right)
    } else {
        matrix4_factors(&right)
    };
    let det = determinant4_from_factors(&s, &c);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    let adjugate = if dense_exact {
        matrix4_adjugate_from_factors_dense_exact(&right, &s, &c)
    } else {
        matrix4_adjugate_from_factors(&right, &s, &c)
    };
    Ok(scale_matrix4(
        multiply_arrays4_ref_with_dense_certificate(&left, &adjugate),
        &inv_det,
    ))
}

fn right_divide_matrix4_checked_with_abort<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: [[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    if can_use_dense_exact_shared_adjugate4(&right) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-checked-abort-dense-exact-shared-adjugate"
        );
        return right_divide_matrix4_dense_exact_shared_checked_with_abort(&left, &right, signal);
    }
    let right_facts = matrix4_facts(&right);
    if right_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-checked-abort-identity"
        );
        return Ok(left);
    }
    if right_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-checked-abort-diagonal"
        );
        return divide_matrix4_by_diagonal_checked_with_abort(left, &right, signal);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix4_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-checked-abort-affine-left-affine-translation"
            );
            return divide_matrix4_affine_by_affine_checked_with_abort_assumed_affine_translation(
                left, &right, signal,
            );
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-checked-abort-affine-by-translation"
        );
        return divide_matrix4_by_affine_checked_with_abort_assumed_affine_translation(
            left, &right, signal,
        );
    }
    if B::FUSE_SIGNED_PRODUCT_SUM && right_facts.is_affine && right_facts.is_upper_triangular {
        let left_facts = matrix4_facts(&left);
        if left_facts.is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-checked-abort-affine-left-affine-upper-triangular"
            );
            return divide_matrix4_affine_by_affine_upper_triangular_checked_with_abort(
                left, &right, signal,
            );
        }
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-checked-abort-affine-upper-triangular"
        );
        return divide_matrix4_by_affine_upper_triangular_checked_with_abort(left, &right, signal);
    }
    if right_facts.is_upper_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-checked-abort-upper-triangular"
        );
        return divide_matrix4_by_upper_triangular_checked_with_abort(left, &right, signal);
    }
    if right_facts.is_lower_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-checked-abort-lower-triangular"
        );
        return divide_matrix4_by_lower_triangular_checked_with_abort(left, &right, signal);
    }
    if right_facts.is_affine {
        // Defer left-structure extraction until affine handling is required so short-circuit
        // branches preserve exactness-cost predictability and skip unnecessary probes.
        // See: Golub and Van Loan, Matrix Computations, 4th ed.
        let left_facts = matrix4_facts(&left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        let right_is_affine_translation = right_facts.is_affine_translation;
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-checked-abort-affine"
        );
        if left_is_affine {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-checked-abort-affine-left-affine"
            );
            if right_is_affine_translation {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide4-checked-abort-affine-by-affine-translation"
                );
                return divide_matrix4_affine_by_affine_checked_with_abort_assumed_affine_translation(
                    left,
                    &right,
                    signal,
                );
            }
            if right_linear_is_diagonal {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide4-checked-abort-affine-left-affine-linear-diagonal"
                );
                if right_facts.affine_linear_diagonal_is_definitely_nonzero {
                    crate::trace_dispatch!(
                        "hyperlattice_matrix",
                        "helper",
                        "right-divide4-checked-abort-affine-left-affine-linear-diagonal-known-nonzero"
                    );
                    return divide_matrix4_affine_by_affine_linear_diagonal(left, &right);
                }
                return divide_matrix4_affine_by_affine_linear_diagonal_checked_with_abort(
                    left, &right, signal,
                );
            }
            return divide_matrix4_affine_by_affine_checked_with_abort(left, &right, signal);
        }
        if right_is_affine_translation {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-checked-abort-by-affine-translation"
            );
            return divide_matrix4_by_affine_checked_with_abort_assumed_affine_translation(
                left, &right, signal,
            );
        }
        if right_linear_is_diagonal {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "right-divide4-checked-abort-affine-linear-diagonal"
            );
            if right_facts.affine_linear_diagonal_is_definitely_nonzero {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "right-divide4-checked-abort-affine-linear-diagonal-known-nonzero"
                );
                return divide_matrix4_by_affine_linear_diagonal(left, &right);
            }
            return divide_matrix4_by_affine_linear_diagonal_checked_with_abort(
                left, &right, signal,
            );
        }
        return divide_matrix4_by_affine_checked_with_abort(left, &right, signal);
    }
    if !prefer_shared_adjugate_right_division(&left, &right) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "right-divide4-checked-abort-gauss-jordan"
        );
        return Ok(transpose_array4(solve_left_system4_checked_with_abort(
            transpose_array4(right),
            transpose_array4(left),
            signal,
        )?));
    }

    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "right-divide4-checked-abort-shared-adjugate"
    );
    let dense_exact = B::FUSE_SIGNED_PRODUCT_SUM && right_facts.is_definitely_dense_for_inverse;
    let (s, c) = if dense_exact {
        matrix4_factors_dense_exact(&right)
    } else {
        matrix4_factors(&right)
    };
    let det = determinant4_from_factors(&s, &c);
    let det = with_abort(det, signal);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    let adjugate = if dense_exact {
        matrix4_adjugate_from_factors_dense_exact(&right, &s, &c)
    } else {
        matrix4_adjugate_from_factors(&right, &s, &c)
    };
    Ok(scale_matrix4(
        multiply_arrays4_ref_with_dense_certificate(&left, &adjugate),
        &inv_det,
    ))
}

#[inline]
fn multiply_arrays3_borrowed<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> [[Scalar<B>; 3]; 3] {
    let left_nonzero = [
        [
            !left[0][0].definitely_zero(),
            !left[0][1].definitely_zero(),
            !left[0][2].definitely_zero(),
        ],
        [
            !left[1][0].definitely_zero(),
            !left[1][1].definitely_zero(),
            !left[1][2].definitely_zero(),
        ],
        [
            !left[2][0].definitely_zero(),
            !left[2][1].definitely_zero(),
            !left[2][2].definitely_zero(),
        ],
    ];
    let right_nonzero = [
        [
            !right[0][0].definitely_zero(),
            !right[0][1].definitely_zero(),
            !right[0][2].definitely_zero(),
        ],
        [
            !right[1][0].definitely_zero(),
            !right[1][1].definitely_zero(),
            !right[1][2].definitely_zero(),
        ],
        [
            !right[2][0].definitely_zero(),
            !right[2][1].definitely_zero(),
            !right[2][2].definitely_zero(),
        ],
    ];

    let left_all_nonzero = left_nonzero[0][0]
        && left_nonzero[0][1]
        && left_nonzero[0][2]
        && left_nonzero[1][0]
        && left_nonzero[1][1]
        && left_nonzero[1][2]
        && left_nonzero[2][0]
        && left_nonzero[2][1]
        && left_nonzero[2][2];
    let right_all_nonzero = right_nonzero[0][0]
        && right_nonzero[0][1]
        && right_nonzero[0][2]
        && right_nonzero[1][0]
        && right_nonzero[1][1]
        && right_nonzero[1][2]
        && right_nonzero[2][0]
        && right_nonzero[2][1]
        && right_nonzero[2][2];

    if left_all_nonzero && right_all_nonzero {
        crate::trace_dispatch!("hyperlattice_matrix", "helper", "multiply3-borrowed-dense");

        let cell = |row: usize, col: usize| {
            Scalar(B::Repr::dot3(
                [&left[row][0].0, &left[row][1].0, &left[row][2].0],
                [&right[0][col].0, &right[1][col].0, &right[2][col].0],
            ))
        };

        return [
            [cell(0, 0), cell(0, 1), cell(0, 2)],
            [cell(1, 0), cell(1, 1), cell(1, 2)],
            [cell(2, 0), cell(2, 1), cell(2, 2)],
        ];
    }

    crate::trace_dispatch!("hyperlattice_matrix", "helper", "multiply3-borrowed-sparse");

    let cell = |row: usize, col: usize| {
        let l0 = &left[row][0];
        let l1 = &left[row][1];
        let l2 = &left[row][2];
        let r0 = &right[0][col];
        let r1 = &right[1][col];
        let r2 = &right[2][col];
        let p0 = left_nonzero[row][0] && right_nonzero[0][col];
        let p1 = left_nonzero[row][1] && right_nonzero[1][col];
        let p2 = left_nonzero[row][2] && right_nonzero[2][col];
        let nonzero_count = usize::from(p0) + usize::from(p1) + usize::from(p2);

        match nonzero_count {
            0 => Scalar::zero(),
            1 => {
                if p0 {
                    l0 * r0
                } else if p1 {
                    l1 * r1
                } else {
                    l2 * r2
                }
            }
            2 => {
                if !p0 {
                    Scalar::active_signed_product_sum2([true, true], [[l1, r1], [l2, r2]])
                } else if !p1 {
                    Scalar::active_signed_product_sum2([true, true], [[l0, r0], [l2, r2]])
                } else {
                    Scalar::active_signed_product_sum2([true, true], [[l0, r0], [l1, r1]])
                }
            }
            _ => Scalar(B::Repr::dot3([&l0.0, &l1.0, &l2.0], [&r0.0, &r1.0, &r2.0])),
        }
    };

    [
        [cell(0, 0), cell(0, 1), cell(0, 2)],
        [cell(1, 0), cell(1, 1), cell(1, 2)],
        [cell(2, 0), cell(2, 1), cell(2, 2)],
    ]
}

#[inline]
fn multiply_arrays3_dense_ref<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> [[Scalar<B>; 3]; 3] {
    crate::trace_dispatch!("hyperlattice_matrix", "helper", "multiply3-dense-ref");
    let cell = |row: usize, col: usize| {
        Scalar(B::Repr::dot3(
            [&left[row][0].0, &left[row][1].0, &left[row][2].0],
            [&right[0][col].0, &right[1][col].0, &right[2][col].0],
        ))
    };

    [
        [cell(0, 0), cell(0, 1), cell(0, 2)],
        [cell(1, 0), cell(1, 1), cell(1, 2)],
        [cell(2, 0), cell(2, 1), cell(2, 2)],
    ]
}

#[inline]
fn multiply_arrays3_with_exact_dense_certificate<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: [[Scalar<B>; 3]; 3],
) -> [[Scalar<B>; 3]; 3] {
    if B::FUSE_SIGNED_PRODUCT_SUM
        && matrix3_has_dense_multiply_certificate(&left)
        && matrix3_has_dense_multiply_certificate(&right)
    {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply3-owned-owned-dense-certified-exact"
        );
        return multiply_arrays3_dense_ref(&left, &right);
    }
    multiply_arrays3(left, right)
}

#[inline]
fn multiply_arrays3_rhs_ref_with_exact_dense_certificate<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> [[Scalar<B>; 3]; 3] {
    if B::FUSE_SIGNED_PRODUCT_SUM
        && matrix3_has_dense_multiply_certificate(&left)
        && matrix3_has_dense_multiply_certificate(right)
    {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply3-owned-ref-dense-certified-exact"
        );
        return multiply_arrays3_dense_ref(&left, right);
    }
    multiply_arrays3_rhs_ref(left, right)
}

#[inline]
fn multiply_arrays3_ref_with_exact_dense_certificate<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> [[Scalar<B>; 3]; 3] {
    if B::FUSE_SIGNED_PRODUCT_SUM
        && matrix3_has_dense_multiply_certificate(left)
        && matrix3_has_dense_multiply_certificate(right)
    {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply3-ref-ref-dense-certified-exact"
        );
        return multiply_arrays3_dense_ref(left, right);
    }
    multiply_arrays3_ref(left, right)
}

#[inline]
fn multiply_arrays4_borrowed<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> [[Scalar<B>; 4]; 4] {
    let left_nonzero = [
        [
            !left[0][0].definitely_zero(),
            !left[0][1].definitely_zero(),
            !left[0][2].definitely_zero(),
            !left[0][3].definitely_zero(),
        ],
        [
            !left[1][0].definitely_zero(),
            !left[1][1].definitely_zero(),
            !left[1][2].definitely_zero(),
            !left[1][3].definitely_zero(),
        ],
        [
            !left[2][0].definitely_zero(),
            !left[2][1].definitely_zero(),
            !left[2][2].definitely_zero(),
            !left[2][3].definitely_zero(),
        ],
        [
            !left[3][0].definitely_zero(),
            !left[3][1].definitely_zero(),
            !left[3][2].definitely_zero(),
            !left[3][3].definitely_zero(),
        ],
    ];
    let right_nonzero = [
        [
            !right[0][0].definitely_zero(),
            !right[0][1].definitely_zero(),
            !right[0][2].definitely_zero(),
            !right[0][3].definitely_zero(),
        ],
        [
            !right[1][0].definitely_zero(),
            !right[1][1].definitely_zero(),
            !right[1][2].definitely_zero(),
            !right[1][3].definitely_zero(),
        ],
        [
            !right[2][0].definitely_zero(),
            !right[2][1].definitely_zero(),
            !right[2][2].definitely_zero(),
            !right[2][3].definitely_zero(),
        ],
        [
            !right[3][0].definitely_zero(),
            !right[3][1].definitely_zero(),
            !right[3][2].definitely_zero(),
            !right[3][3].definitely_zero(),
        ],
    ];

    let left_all_nonzero = left_nonzero[0][0]
        && left_nonzero[0][1]
        && left_nonzero[0][2]
        && left_nonzero[0][3]
        && left_nonzero[1][0]
        && left_nonzero[1][1]
        && left_nonzero[1][2]
        && left_nonzero[1][3]
        && left_nonzero[2][0]
        && left_nonzero[2][1]
        && left_nonzero[2][2]
        && left_nonzero[2][3]
        && left_nonzero[3][0]
        && left_nonzero[3][1]
        && left_nonzero[3][2]
        && left_nonzero[3][3];
    let right_all_nonzero = right_nonzero[0][0]
        && right_nonzero[0][1]
        && right_nonzero[0][2]
        && right_nonzero[0][3]
        && right_nonzero[1][0]
        && right_nonzero[1][1]
        && right_nonzero[1][2]
        && right_nonzero[1][3]
        && right_nonzero[2][0]
        && right_nonzero[2][1]
        && right_nonzero[2][2]
        && right_nonzero[2][3]
        && right_nonzero[3][0]
        && right_nonzero[3][1]
        && right_nonzero[3][2]
        && right_nonzero[3][3];

    if left_all_nonzero && right_all_nonzero {
        crate::trace_dispatch!("hyperlattice_matrix", "helper", "multiply4-borrowed-dense");
        let cell = |row: usize, col: usize| {
            let l0 = &left[row][0];
            let l1 = &left[row][1];
            let l2 = &left[row][2];
            let l3 = &left[row][3];
            let r0 = &right[0][col];
            let r1 = &right[1][col];
            let r2 = &right[2][col];
            let r3 = &right[3][col];
            if B::FUSE_SIGNED_PRODUCT_SUM {
                Scalar(B::Repr::active_dot4(
                    [&l0.0, &l1.0, &l2.0, &l3.0],
                    [&r0.0, &r1.0, &r2.0, &r3.0],
                ))
            } else {
                Scalar(B::Repr::dot4(
                    [&l0.0, &l1.0, &l2.0, &l3.0],
                    [&r0.0, &r1.0, &r2.0, &r3.0],
                ))
            }
        };

        return [
            [cell(0, 0), cell(0, 1), cell(0, 2), cell(0, 3)],
            [cell(1, 0), cell(1, 1), cell(1, 2), cell(1, 3)],
            [cell(2, 0), cell(2, 1), cell(2, 2), cell(2, 3)],
            [cell(3, 0), cell(3, 1), cell(3, 2), cell(3, 3)],
        ];
    }

    crate::trace_dispatch!("hyperlattice_matrix", "helper", "multiply4-borrowed-sparse");

    let cell = |row: usize, col: usize| {
        let l0 = &left[row][0];
        let l1 = &left[row][1];
        let l2 = &left[row][2];
        let l3 = &left[row][3];
        let r0 = &right[0][col];
        let r1 = &right[1][col];
        let r2 = &right[2][col];
        let r3 = &right[3][col];

        let left_row = left_nonzero[row];
        let p0 = left_row[0] && right_nonzero[0][col];
        let p1 = left_row[1] && right_nonzero[1][col];
        let p2 = left_row[2] && right_nonzero[2][col];
        let p3 = left_row[3] && right_nonzero[3][col];
        let nonzero_count = usize::from(p0) + usize::from(p1) + usize::from(p2) + usize::from(p3);

        match nonzero_count {
            0 => Scalar::zero(),
            1 => {
                if p0 {
                    l0 * r0
                } else if p1 {
                    l1 * r1
                } else if p2 {
                    l2 * r2
                } else {
                    l3 * r3
                }
            }
            2 => {
                if p0 {
                    if p1 {
                        // Sparse mat4 multiply is performance-sensitive for
                        // affine and inverse kernels because exact backends
                        // avoid constructing zero products. Keep each active
                        // lane explicit: a previous hand-unrolled branch used
                        // lane 3 for the `p0 && p1` case, which broke
                        // upper-triangular inverse products while preserving
                        // most dense benchmark rows.
                        Scalar::active_signed_product_sum2([true, true], [[l0, r0], [l1, r1]])
                    } else if p2 {
                        Scalar::active_signed_product_sum2([true, true], [[l0, r0], [l2, r2]])
                    } else {
                        Scalar::active_signed_product_sum2([true, true], [[l0, r0], [l3, r3]])
                    }
                } else if p1 {
                    if p2 {
                        Scalar::active_signed_product_sum2([true, true], [[l1, r1], [l2, r2]])
                    } else {
                        Scalar::active_signed_product_sum2([true, true], [[l1, r1], [l3, r3]])
                    }
                } else if p2 {
                    Scalar::active_signed_product_sum2([true, true], [[l2, r2], [l3, r3]])
                } else {
                    unreachable!("matrix multiply sparse branch expects exactly two active terms")
                }
            }
            3 => {
                if !p0 {
                    Scalar::active_signed_product_sum2(
                        [true, true, true],
                        [[l1, r1], [l2, r2], [l3, r3]],
                    )
                } else if !p1 {
                    Scalar::active_signed_product_sum2(
                        [true, true, true],
                        [[l0, r0], [l2, r2], [l3, r3]],
                    )
                } else if !p2 {
                    Scalar::active_signed_product_sum2(
                        [true, true, true],
                        [[l0, r0], [l1, r1], [l3, r3]],
                    )
                } else {
                    Scalar::active_signed_product_sum2(
                        [true, true, true],
                        [[l0, r0], [l1, r1], [l2, r2]],
                    )
                }
            }
            _ if B::FUSE_SIGNED_PRODUCT_SUM => Scalar(B::Repr::active_dot4(
                [&l0.0, &l1.0, &l2.0, &l3.0],
                [&r0.0, &r1.0, &r2.0, &r3.0],
            )),
            _ => Scalar(B::Repr::dot4(
                [&l0.0, &l1.0, &l2.0, &l3.0],
                [&r0.0, &r1.0, &r2.0, &r3.0],
            )),
        }
    };

    [
        [cell(0, 0), cell(0, 1), cell(0, 2), cell(0, 3)],
        [cell(1, 0), cell(1, 1), cell(1, 2), cell(1, 3)],
        [cell(2, 0), cell(2, 1), cell(2, 2), cell(2, 3)],
        [cell(3, 0), cell(3, 1), cell(3, 2), cell(3, 3)],
    ]
}

#[inline]
fn multiply_arrays4_dense_ref<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> [[Scalar<B>; 4]; 4] {
    crate::trace_dispatch!("hyperlattice_matrix", "helper", "multiply4-dense-ref");
    let cell = |row: usize, col: usize| {
        let l0 = &left[row][0];
        let l1 = &left[row][1];
        let l2 = &left[row][2];
        let l3 = &left[row][3];
        let r0 = &right[0][col];
        let r1 = &right[1][col];
        let r2 = &right[2][col];
        let r3 = &right[3][col];
        if B::FUSE_SIGNED_PRODUCT_SUM {
            Scalar(B::Repr::active_dot4(
                [&l0.0, &l1.0, &l2.0, &l3.0],
                [&r0.0, &r1.0, &r2.0, &r3.0],
            ))
        } else {
            Scalar(B::Repr::dot4(
                [&l0.0, &l1.0, &l2.0, &l3.0],
                [&r0.0, &r1.0, &r2.0, &r3.0],
            ))
        }
    };

    [
        [cell(0, 0), cell(0, 1), cell(0, 2), cell(0, 3)],
        [cell(1, 0), cell(1, 1), cell(1, 2), cell(1, 3)],
        [cell(2, 0), cell(2, 1), cell(2, 2), cell(2, 3)],
        [cell(3, 0), cell(3, 1), cell(3, 2), cell(3, 3)],
    ]
}

#[inline]
fn multiply_arrays4_dense_known_rational_ref<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> [[Scalar<B>; 4]; 4] {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "multiply4-dense-known-rational-ref"
    );
    let cell = |row: usize, col: usize| {
        let l0 = &left[row][0];
        let l1 = &left[row][1];
        let l2 = &left[row][2];
        let l3 = &left[row][3];
        let r0 = &right[0][col];
        let r1 = &right[1][col];
        let r2 = &right[2][col];
        let r3 = &right[3][col];
        Scalar(B::Repr::active_dot4_known_exact_rational(
            [&l0.0, &l1.0, &l2.0, &l3.0],
            [&r0.0, &r1.0, &r2.0, &r3.0],
        ))
    };

    [
        [cell(0, 0), cell(0, 1), cell(0, 2), cell(0, 3)],
        [cell(1, 0), cell(1, 1), cell(1, 2), cell(1, 3)],
        [cell(2, 0), cell(2, 1), cell(2, 2), cell(2, 3)],
        [cell(3, 0), cell(3, 1), cell(3, 2), cell(3, 3)],
    ]
}

#[inline]
fn multiply_arrays3<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: [[Scalar<B>; 3]; 3],
) -> [[Scalar<B>; 3]; 3] {
    // Compute structural facts once per operand so identity and diagonal
    // dispatch share the same zero/one probes. This keeps multiply aligned with
    // inverse/division fact reuse and avoids rechecking off-diagonal zeros in
    // dense fallback cases.
    let left_facts = matrix3_facts(&left);
    let right_facts = matrix3_facts(&right);
    if left_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply3-owned-owned-identity-left"
        );
        return right;
    }
    if right_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply3-owned-owned-identity-right"
        );
        return left;
    }
    if left_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply3-owned-owned-diagonal-left"
        );
        return multiply_matrix3_by_left_diagonal(&left, &right);
    }
    if right_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply3-owned-owned-diagonal-right"
        );
        return multiply_matrix3_by_right_diagonal(&left, &right);
    }

    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "multiply3-owned-owned-specialized"
    );
    multiply_arrays3_borrowed(&left, &right)
}

#[inline]
fn multiply_arrays4<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: [[Scalar<B>; 4]; 4],
) -> [[Scalar<B>; 4]; 4] {
    let left_facts = matrix4_facts(&left);
    let right_facts = matrix4_facts(&right);
    if left_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply4-owned-owned-identity-left"
        );
        return right;
    }
    if right_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply4-owned-owned-identity-right"
        );
        return left;
    }
    if left_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply4-owned-owned-diagonal-left"
        );
        return multiply_matrix4_by_left_diagonal(&left, &right);
    }
    if right_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply4-owned-owned-diagonal-right"
        );
        return multiply_matrix4_by_right_diagonal(&left, &right);
    }

    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "multiply4-owned-owned-specialized"
    );
    multiply_arrays4_borrowed(&left, &right)
}

#[inline]
fn multiply_arrays3_rhs_ref<B: Backend>(
    left: [[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> [[Scalar<B>; 3]; 3] {
    let left_facts = matrix3_facts(&left);
    let right_facts = matrix3_facts(right);
    if left_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply3-owned-ref-identity-left"
        );
        return right.clone();
    }
    if right_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply3-owned-ref-identity-right"
        );
        return left;
    }
    if left_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply3-owned-ref-diagonal-left"
        );
        return multiply_matrix3_by_left_diagonal(&left, right);
    }
    if right_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply3-owned-ref-diagonal-right"
        );
        return multiply_matrix3_by_right_diagonal(&left, right);
    }

    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "multiply3-owned-ref-specialized"
    );
    multiply_arrays3_borrowed(&left, right)
}

#[inline]
fn multiply_arrays4_rhs_ref<B: Backend>(
    left: [[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> [[Scalar<B>; 4]; 4] {
    let left_facts = matrix4_facts(&left);
    let right_facts = matrix4_facts(right);
    if left_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply4-owned-ref-identity-left"
        );
        return right.clone();
    }
    if right_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply4-owned-ref-identity-right"
        );
        return left;
    }
    if left_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply4-owned-ref-diagonal-left"
        );
        return multiply_matrix4_by_left_diagonal(&left, right);
    }
    if right_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply4-owned-ref-diagonal-right"
        );
        return multiply_matrix4_by_right_diagonal(&left, right);
    }

    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "multiply4-owned-ref-specialized"
    );
    multiply_arrays4_borrowed(&left, right)
}

#[inline]
fn multiply_arrays3_ref<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[[Scalar<B>; 3]; 3],
) -> [[Scalar<B>; 3]; 3] {
    // Fixed 3x3 multiply avoids the const-generic helper's per-cell "is there
    // a fourth lane?" branch and intermediate tiny arrays. A row-dot prototype
    // was traced and rejected because it regressed exact-rational powi despite
    // fewer reduction events; keep the proven per-cell dot schedule here.
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "multiply3-ref-ref-specialized"
    );

    let left_facts = matrix3_facts(left);
    let right_facts = matrix3_facts(right);
    if left_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply3-ref-ref-identity-left"
        );
        return right.clone();
    }
    if right_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply3-ref-ref-identity-right"
        );
        return left.clone();
    }
    if left_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply3-ref-ref-diagonal-left"
        );
        return multiply_matrix3_by_left_diagonal(left, right);
    }
    if right_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply3-ref-ref-diagonal-right"
        );
        return multiply_matrix3_by_right_diagonal(left, right);
    }

    multiply_arrays3_borrowed(left, right)
}

#[inline]
fn multiply_arrays4_ref<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[[Scalar<B>; 4]; 4],
) -> [[Scalar<B>; 4]; 4] {
    // Fixed 4x4 borrowed multiply is similarly unrolled. This is deliberately
    // duplicated from the generic path because the branchless version wins in
    // borrowed mat4 multiply benchmarks while keeping per-cell exact-rational
    // denominator schedules.
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "multiply4-ref-ref-specialized"
    );

    let left_facts = matrix4_facts(left);
    let right_facts = matrix4_facts(right);
    if left_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply4-ref-ref-identity-left"
        );
        return right.clone();
    }
    if right_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply4-ref-ref-identity-right"
        );
        return left.clone();
    }
    if left_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply4-ref-ref-diagonal-left"
        );
        return multiply_matrix4_by_left_diagonal(left, right);
    }
    if right_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "multiply4-ref-ref-diagonal-right"
        );
        return multiply_matrix4_by_right_diagonal(left, right);
    }

    multiply_arrays4_borrowed(left, right)
}

fn transform_vector_rhs_ref<B: Backend, const N: usize>(
    left: &[[Scalar<B>; N]; N],
    right: &[Scalar<B>; N],
) -> [Scalar<B>; N] {
    if N == 4 {
        // For the N==4 case, reuse one structural scan for identity, diagonal,
        // and direction-fast-path predicates. This keeps 4x4 transform kernels
        // on the same fact-on-demand policy used by division/inverse dispatch.
        // See Golub and Van Loan, *Matrix Computations* (4th ed.), and Yap,
        // "Towards Exact Geometric Computation", 1997.
        let left_facts = matrix4_facts_assuming_const4(left);
        if left_facts.is_identity {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "transform-vector4-identity"
            );
            return right.clone();
        }

        if left_facts.is_diagonal {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "transform-vector4-diagonal"
            );
            return from_fn(|row| right[row].clone().mul_cached(&left[row][row]));
        }

        // Classify the homogeneous coordinate once; zero/one specializations
        // cover direction/point transforms and are left unchanged for unknown
        // homogeneous entries.
        match right[3].zero_or_one() {
            Some(false) => {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "transform-vector-direction"
                );
                if left_facts.direction_linear_is_diagonal {
                    crate::trace_dispatch!(
                        "hyperlattice_matrix",
                        "helper",
                        "transform-vector-direction-diagonal"
                    );
                    return from_fn(|row| {
                        if row == 3 {
                            Scalar::zero()
                        } else {
                            right[row].clone().mul_cached(&left[row][row])
                        }
                    });
                }
                let vector_terms = [&right[0], &right[1], &right[2]];
                return from_fn(|row| {
                    let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
                    Scalar::linear_combination3(matrix_terms, vector_terms)
                });
            }
            Some(true) => {
                crate::trace_dispatch!("hyperlattice_matrix", "helper", "transform-vector-point");
                if left_facts.is_affine && left_facts.linear_is_diagonal {
                    crate::trace_dispatch!(
                        "hyperlattice_matrix",
                        "helper",
                        "transform-vector-point-affine-linear-diagonal"
                    );
                    return from_fn(|row| {
                        if row == 3 {
                            Scalar::one()
                        } else {
                            right[row].clone().mul_cached(&left[row][row]) + &left[row][3]
                        }
                    });
                }
                // Reuse translation-column zero facts collected by
                // `matrix4_facts_assuming_const4`; only m33 is not part of the
                // retained xyz translation facts. This avoids re-querying the
                // top three translation entries on point paths.
                let translation_is_zero: [bool; N] = from_fn(|row| {
                    if row < 3 {
                        left_facts.translation_xyz_zero[row]
                    } else {
                        left[row][3].definitely_zero()
                    }
                });
                let vector_terms = [&right[0], &right[1], &right[2]];
                return from_fn(|row| {
                    let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
                    // Point transforms preserve homogeneous offsets as affine sums
                    // to avoid forcing extra zero-like terms into a four-term
                    // form.
                    let mapped = Scalar::linear_combination3(matrix_terms, vector_terms);
                    if translation_is_zero[row] {
                        mapped
                    } else {
                        mapped + &left[row][3]
                    }
                });
            }
            None => {}
        }

        // Cache per-row translation entries once for non-direction/non-point rows to
        // avoid repeated fact probing inside the hot map loop.
        let translation_is_zero: [bool; N] = from_fn(|row| {
            if row < 3 {
                left_facts.translation_xyz_zero[row]
            } else {
                left[row][3].definitely_zero()
            }
        });
        let vector_terms = [&right[0], &right[1], &right[2]];
        crate::trace_dispatch!("hyperlattice_matrix", "helper", "transform-vector-full");
        from_fn(|row| {
            if translation_is_zero[row] {
                let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
                Scalar::linear_combination3(matrix_terms, vector_terms)
            } else {
                let matrix_terms = [&left[row][0], &left[row][1], &left[row][2], &left[row][3]];
                let vector_terms = [&right[0], &right[1], &right[2], &right[3]];
                // `Matrix4` transforms already encode translation in `left[row][3]`,
                // so this branch is a pure 4-term linear form. Keeping it on the
                // linear path avoids a redundant offset check and construction.
                Scalar::linear_combination4(matrix_terms, vector_terms)
            }
        })
    } else {
        // Reuse the retained 3×3 structural facts for the smaller transform
        // branch as well; this keeps the probe count aligned with other fixed-size
        // kernels and avoids duplicated definite-zero checks in this hot path.
        // As with 4×4 transforms, we prioritize fact-on-demand structural
        // classification before arithmetic.
        // Reference: Golub and Van Loan, *Matrix Computations* (4th ed.).
        let left_facts = matrix3_facts_assuming_const3(left);
        if left_facts.is_identity {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "transform-vector3-identity"
            );
            return right.clone();
        }

        if left_facts.is_diagonal {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "transform-vector3-diagonal"
            );
            return from_fn(|row| right[row].clone().mul_cached(&left[row][row]));
        }

        let vector_terms = [&right[0], &right[1], &right[2]];
        from_fn(|row| {
            let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
            // `N != 4` for current matrix-vector callers means 3-lane
            // geometry, so only the pure linear form is valid here.
            Scalar::linear_combination3(matrix_terms, vector_terms)
        })
    }
}

#[inline]
fn transform_vector3_rhs_ref_cached<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[Scalar<B>; 3],
) -> [Scalar<B>; 3] {
    // Matrix3 transforms never use a homogeneous column, so every output lane is
    // a fixed 3-term linear combination. The structural guards remain in this
    // shared helper because targeted sentinels showed the branchy reused helper
    // benchmarks faster than a separate dense-only helper for current
    // hyperreal-backed workloads.
    // Use the canonical `Matrix3Facts` scan here. It avoids duplicated
    // structural probes and keeps identity classification consistent with
    // inverse/division dispatch; importantly, it includes every off-diagonal
    // zero fact exactly once.
    let matrix_facts = matrix3_facts(left);
    if matrix_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "transform-vector3-identity"
        );
        return right.clone();
    }

    if matrix_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "transform-vector3-diagonal"
        );
        return from_fn(|row| right[row].clone().mul_cached(&left[row][row]));
    }

    crate::trace_dispatch!("hyperlattice_matrix", "helper", "transform-vector3-dense");
    transform_vector3_rhs_dense_ref(left, right)
}

#[inline]
fn transform_vector3_rhs_dense_ref<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[Scalar<B>; 3],
) -> [Scalar<B>; 3] {
    let vector_terms = [&right[0], &right[1], &right[2]];
    from_fn(|row| {
        let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
        Scalar::linear_combination3(matrix_terms, vector_terms)
    })
}

#[inline]
fn transform_vector3_rhs_dense_active_ref<B: Backend>(
    left: &[[Scalar<B>; 3]; 3],
    right: &[Scalar<B>; 3],
) -> [Scalar<B>; 3] {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "transform-vector3-dense-active"
    );
    let vector_terms = [&right[0], &right[1], &right[2]];
    from_fn(|row| {
        let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
        Scalar::active_linear_combination3(matrix_terms, vector_terms)
    })
}

#[inline]
fn transform_vector4_rhs_ref_cached_with_matrix_facts<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[Scalar<B>; 4],
    translation_is_zero: &[bool; 4],
    matrix_facts: Matrix4Facts,
) -> [Scalar<B>; 4] {
    if matrix_facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "transform-vector4-identity"
        );
        return right.clone();
    }

    if matrix_facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "transform-vector4-diagonal"
        );
        return from_fn(|row| right[row].clone().mul_cached(&left[row][row]));
    }

    // Batch transforms usually share one matrix; caching the translation column
    // zero checks here removes repeated fact probes per-row for every vector in
    // the batch while keeping branch behavior identical to scalar paths.
    // Direction/point checks are merged into one classifier to avoid doing two
    // separate predicate trips for the common unknown-`w` path.
    let vector_terms = [&right[0], &right[1], &right[2]];
    match right[3].zero_or_one() {
        Some(false) => {
            // A direction vector keeps the row-local 3-term linear form.
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "transform-vector4-direction"
            );
            return transform_vector4_rhs_direction_ref_cached(
                left,
                right,
                matrix_facts.direction_linear_is_diagonal,
            );
        }
        Some(true) => {
            // Point vectors can reuse exact translation offsets as an explicit
            // addition after the shared 3-term linear body.
            crate::trace_dispatch!("hyperlattice_matrix", "helper", "transform-vector4-point");
            return transform_vector4_rhs_point_ref_cached(left, right, translation_is_zero);
        }
        None => {}
    }

    crate::trace_dispatch!("hyperlattice_matrix", "helper", "transform-vector4-full");
    from_fn(|row| {
        if translation_is_zero[row] {
            let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
            Scalar::linear_combination3(matrix_terms, vector_terms)
        } else {
            let matrix_terms = [&left[row][0], &left[row][1], &left[row][2], &left[row][3]];
            let vector_terms = [&right[0], &right[1], &right[2], &right[3]];
            // Keep cached batch transforms aligned with the non-cached path:
            // all homogeneous translation is already part of the 4-term linear
            // form, so no extra offset term is required.
            Scalar::linear_combination4(matrix_terms, vector_terms)
        }
    })
}

#[inline]
fn transform_vector4_rhs_ref_with_facts<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[Scalar<B>; 4],
    translation_is_zero: &[bool; 4],
    all_translation_zero: bool,
    all_translation_nonzero: bool,
    direction_is_diagonal: bool,
    matrix_facts: Option<Matrix4Facts>,
    facts: Vector4GeometricFacts,
) -> [Scalar<B>; 4] {
    // Retained homogeneous classification lets us choose affine-specialized
    // kernels before re-running scalar structure probes. This mirrors Yap’s
    // retained-structure thesis for exact geometry, where cheap geometric
    // facts gate fast paths early and postpone canonicalization.
    match facts.homogeneous {
        Vector4HomogeneousKind::Direction => {
            // Keeping directions on the 3-term linear form also avoids touching
            // translation entries entirely in affine rows.
            transform_vector4_rhs_direction_ref_cached(left, right, direction_is_diagonal)
        }
        Vector4HomogeneousKind::Point => {
            if all_translation_zero {
                transform_vector4_rhs_full_no_translation_ref_cached(left, right)
            } else if all_translation_nonzero {
                transform_vector4_rhs_point_all_nonzero_ref_cached(left, right)
            } else {
                transform_vector4_rhs_point_ref_cached(left, right, translation_is_zero)
            }
        }
        Vector4HomogeneousKind::Unknown => {
            let matrix_facts = matrix_facts.unwrap_or_else(|| matrix4_facts(left));
            transform_vector4_rhs_ref_cached_with_matrix_facts(
                left,
                right,
                translation_is_zero,
                matrix_facts,
            )
        }
    }
}

#[inline]
fn transform_vector4_rhs_direction_ref_cached<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[Scalar<B>; 4],
    direction_is_diagonal: bool,
) -> [Scalar<B>; 4] {
    if direction_is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "transform-vector4-direction-diagonal-facts"
        );
        return [
            right[0].clone().mul_cached(&left[0][0]),
            right[1].clone().mul_cached(&left[1][1]),
            right[2].clone().mul_cached(&left[2][2]),
            Scalar::zero(),
        ];
    }

    // `direction_is_diagonal` is an exact retained fact from the matrix scan.
    // If it is false, identity/diagonal direction cases are already ruled out,
    // so the remaining valid fast form is the three-term linear combination.
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "transform-vector4-batch-direction"
    );
    let vector_terms = [&right[0], &right[1], &right[2]];
    from_fn(|row| {
        let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
        Scalar::linear_combination3(matrix_terms, vector_terms)
    })
}

fn transform_vector4_direction_batch_assumed_ref<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    rhs: &[Vector4<B>],
    direction_is_diagonal: bool,
) -> Vec<Vector4<B>> {
    let mut transformed = Vec::with_capacity(rhs.len());
    if direction_is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "transform-vector4-direction-batch-diagonal-assumed"
        );
        for vector in rhs {
            transformed.push(Vector4([
                vector.0[0].clone().mul_cached(&left[0][0]),
                vector.0[1].clone().mul_cached(&left[1][1]),
                vector.0[2].clone().mul_cached(&left[2][2]),
                Scalar::zero(),
            ]));
        }
        return transformed;
    }

    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "transform-vector4-direction-batch-linear-assumed"
    );
    for vector in rhs {
        // Directions have `w = 0`, so the translation column cannot contribute.
        // Keep the row computation as a three-term linear form to preserve
        // hyperreal's delayed product-sum reduction instead of constructing a
        // generic four-term expression with a structural zero. This is the
        // projective point/direction split used by exact geometric kernels; see
        // Yap, "Towards Exact Geometric Computation", 1997.
        let vector_terms = [&vector.0[0], &vector.0[1], &vector.0[2]];
        transformed.push(Vector4(from_fn(|row| {
            let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
            Scalar::linear_combination3(matrix_terms, vector_terms)
        })));
    }
    transformed
}

#[inline]
fn transform_vector4_rhs_point_affine_linear_diagonal_ref_cached<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[Scalar<B>; 4],
) -> [Scalar<B>; 4] {
    // For affine matrices with diagonal linear blocks and point vectors (w = 1),
    // each spatial lane is one cached scale plus one translation add. This avoids
    // building three-term linear combinations whose off-diagonal terms are known
    // structural zeros. The specialization follows the projective point/direction
    // split used in exact geometric computation; see Yap, "Towards Exact
    // Geometric Computation", 1997.
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "transform-vector4-point-affine-linear-diagonal"
    );
    [
        right[0].clone().mul_cached(&left[0][0]) + &left[0][3],
        right[1].clone().mul_cached(&left[1][1]) + &left[1][3],
        right[2].clone().mul_cached(&left[2][2]) + &left[2][3],
        // The caller either explicitly assumed a point or arrived here after
        // a retained homogeneous fact proved `w == 1`. Reusing the existing
        // lane avoids constructing a fresh scalar and preserves any cached
        // exact/symbolic representation already carried by the point. This is
        // the same object-level information preservation advocated for exact
        // geometric computation by Yap, "Towards Exact Geometric Computation",
        // 1997.
        right[3].clone(),
    ]
}

#[inline]
fn transform_vector4_rhs_point_with_scaled_w_ref_cached<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[Scalar<B>; 4],
    translation_is_zero: &[bool; 4],
    all_translation_zero: bool,
    all_translation_nonzero: bool,
    w_scale_is_one: bool,
    w_scale: &Scalar<B>,
) -> [Scalar<B>; 4] {
    // For known point vectors scaled by `w'`, the 4-term point transform
    // can be written as a 3-term spatial product plus `w'`-scaled
    // translation terms. Keeping this as an affine-only specialization
    // preserves inexpensive structural dispatch while avoiding full homogeneous
    // matrix multiplication when only one structural coefficient changed
    // (Yap, "Towards Exact Geometric Computation", 1997).
    let vector_terms = [&right[0], &right[1], &right[2]];
    // Precomputed translation flags allow this helper to avoid rescanning `w`-column
    // structural zeros after its caller already inspected it. That keeps this
    // short path branch-flat for known all-zero/all-nonzero affine offsets.
    if all_translation_zero {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "transform-vector4-point-scaled-w-full-no-translation"
        );
        return from_fn(|row| {
            let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
            Scalar::linear_combination3(matrix_terms, vector_terms)
        });
    }

    if all_translation_nonzero {
        // All rows have non-zero translation coefficients, so we can avoid
        // per-row branches on homogeneous offset activity and apply one
        // multiplied offset term per lane.
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "transform-vector4-point-scaled-w-full-nonzero"
        );
        let translation: [Scalar<B>; 4] = if w_scale_is_one {
            from_fn(|row| left[row][3].clone())
        } else {
            from_fn(|row| left[row][3].clone().mul_cached(w_scale))
        };
        if B::FUSE_SIGNED_PRODUCT_SUM {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "transform-vector4-point-scaled-w-full-nonzero-active"
            );
            return from_fn(|row| {
                let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
                Scalar::active_linear_combination3(matrix_terms, vector_terms) + &translation[row]
            });
        }
        return from_fn(|row| {
            let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
            Scalar::linear_combination3(matrix_terms, vector_terms) + &translation[row]
        });
    }

    // General projected-point fast path: use the 3-term spatial form and apply
    // `w'`-scaled affine offsets only where necessary.
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "transform-vector4-point-scaled-w-partial"
    );
    from_fn(|row| {
        let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
        let mapped = Scalar::linear_combination3(matrix_terms, vector_terms);
        if translation_is_zero[row] {
            mapped
        } else {
            if w_scale_is_one {
                mapped + &left[row][3]
            } else {
                mapped + &left[row][3].clone().mul_cached(w_scale)
            }
        }
    })
}

#[inline]
fn transform_vector4_rhs_point_ref_cached<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[Scalar<B>; 4],
    translation_is_zero: &[bool; 4],
) -> [Scalar<B>; 4] {
    // Keep point transforms on the 3-term linear form and only add offsets
    // when needed according to cached structural translation facts. Callers
    // enter this helper after retained matrix facts have already ruled out
    // identity and diagonal transforms; rechecking them here made prepared
    // point and mixed-batch paths pay a second full matrix probe.
    // See Yap, "Towards Exact Geometric Computation", 1997.
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "transform-vector4-batch-point"
    );
    let vector_terms = [&right[0], &right[1], &right[2]];
    from_fn(|row| {
        let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
        let mapped = Scalar::linear_combination3(matrix_terms, vector_terms);
        if translation_is_zero[row] {
            mapped
        } else {
            mapped + &left[row][3]
        }
    })
}

#[inline]
fn transform_vector4_rhs_point_all_nonzero_ref_cached<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[Scalar<B>; 4],
) -> [Scalar<B>; 4] {
    // Point transforms with guaranteed non-zero translation entries in every row
    // use a compact 3-term affine core and explicit offset for the same reason.
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "transform-vector4-point-all-nonzero"
    );
    let vector_terms = [&right[0], &right[1], &right[2]];
    from_fn(|row| {
        let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
        Scalar::linear_combination3(matrix_terms, vector_terms) + &left[row][3]
    })
}

#[inline]
fn transform_vector4_rhs_full_no_translation_ref_cached<B: Backend>(
    left: &[[Scalar<B>; 4]; 4],
    right: &[Scalar<B>; 4],
) -> [Scalar<B>; 4] {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "transform-vector4-batch-full-no-translation"
    );
    let vector_terms = [&right[0], &right[1], &right[2]];
    from_fn(|row| {
        let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
        Scalar::linear_combination3(matrix_terms, vector_terms)
    })
}

#[derive(Clone, Copy, Debug)]
pub struct TransformedMatrix3<'a, B: Backend = DefaultBackend> {
    matrix: &'a Matrix3<B>,
    facts: Matrix3Facts,
}

impl<'a, B: Backend> TransformedMatrix3<'a, B> {
    fn new(matrix: &'a Matrix3<B>) -> Self {
        let facts = matrix3_facts(&matrix.0);
        Self { matrix, facts }
    }

    pub fn transform_vector(&self, rhs: &Vector3<B>) -> Vector3<B> {
        if self.facts.is_identity {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "transform-vector3-identity"
            );
            return rhs.clone();
        }
        if self.facts.is_diagonal {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "transform-vector3-diagonal"
            );
            return Vector3(from_fn(|row| {
                rhs.0[row].clone().mul_cached(&self.matrix.0[row][row])
            }));
        }
        crate::trace_dispatch!("hyperlattice_matrix", "helper", "transform-vector3-dense");
        Vector3(transform_vector3_rhs_dense_ref(&self.matrix.0, &rhs.0))
    }

    pub fn vector(&self, rhs: &'a Vector3<B>) -> TransformedVector3<'a, B> {
        TransformedVector3 {
            matrix: self.matrix,
            facts: self.facts,
            vector: rhs,
        }
    }

    pub fn transform_vector_batch(&self, rhs: &[Vector3<B>]) -> Vec<Vector3<B>> {
        if self.facts.is_identity {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "transform-vector3-batch-identity"
            );
            return rhs.to_vec();
        }
        if self.facts.is_diagonal {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "transform-vector3-batch-diagonal"
            );
            return rhs
                .iter()
                .map(|vector| {
                    Vector3(from_fn(|row| {
                        vector.0[row].clone().mul_cached(&self.matrix.0[row][row])
                    }))
                })
                .collect();
        }
        let mut transformed = Vec::with_capacity(rhs.len());
        for vector in rhs {
            transformed.push(self.transform_vector(vector));
        }
        transformed
    }
}

#[derive(Clone, Copy, Debug)]
pub struct TransformedMatrix4<'a, B: Backend = DefaultBackend> {
    matrix: &'a Matrix4<B>,
    facts: Matrix4Facts,
    translation_is_zero: [bool; 4],
    all_translation_zero: bool,
    all_translation_nonzero: bool,
    direction_is_diagonal: bool,
}

impl<'a, B: Backend> TransformedMatrix4<'a, B> {
    #[inline]
    fn transform_vector_with_facts(
        &self,
        rhs: &Vector4<B>,
        vector_facts: Vector4GeometricFacts,
    ) -> Vector4<B> {
        if self.facts.is_identity {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "transform-vector4-identity"
            );
            return rhs.clone();
        }
        if self.facts.is_diagonal {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "transform-vector4-diagonal"
            );
            if matches!(vector_facts.homogeneous, Vector4HomogeneousKind::Direction) {
                return Vector4([
                    rhs.0[0].clone().mul_cached(&self.matrix.0[0][0]),
                    rhs.0[1].clone().mul_cached(&self.matrix.0[1][1]),
                    rhs.0[2].clone().mul_cached(&self.matrix.0[2][2]),
                    Scalar::zero(),
                ]);
            }
            if matches!(vector_facts.homogeneous, Vector4HomogeneousKind::Point)
                && self.facts.is_affine
            {
                // Affine diagonal point transforms preserve homogeneous w = 1.
                // When the point fact is already known, returning structural one
                // saves the otherwise redundant `1 * m33` multiply and keeps the
                // exact projective point/direction invariant visible to later
                // kernels. This follows the homogeneous-coordinate split used by
                // exact geometric computation; see Yap, "Towards Exact Geometric
                // Computation", 1997.
                return Vector4(
                    transform_vector4_rhs_point_affine_linear_diagonal_ref_cached(
                        &self.matrix.0,
                        &rhs.0,
                    ),
                );
            }
            return Vector4(from_fn(|row| {
                rhs.0[row].clone().mul_cached(&self.matrix.0[row][row])
            }));
        }
        if matches!(vector_facts.homogeneous, Vector4HomogeneousKind::Point)
            && self.facts.is_affine
            && self.facts.linear_is_diagonal
        {
            return Vector4(
                transform_vector4_rhs_point_affine_linear_diagonal_ref_cached(
                    &self.matrix.0,
                    &rhs.0,
                ),
            );
        }

        Vector4(transform_vector4_rhs_ref_with_facts(
            &self.matrix.0,
            &rhs.0,
            &self.translation_is_zero,
            self.all_translation_zero,
            self.all_translation_nonzero,
            self.direction_is_diagonal,
            Some(self.facts),
            vector_facts,
        ))
    }

    fn new(matrix: &'a Matrix4<B>) -> Self {
        let facts = matrix4_facts(&matrix.0);
        Self::new_with_facts(matrix, facts)
    }

    fn new_with_facts(matrix: &'a Matrix4<B>, facts: Matrix4Facts) -> Self {
        // Cache the per-row homogeneous-column definitely-zero facts once; this
        // keeps batch direction/path selection on the fast linear form when the
        // translation coefficient is structurally impossible to be non-zero.
        // The first three values are retained from `matrix4_facts`; only m33
        // needs a fresh zero query here. Keeping those existing structural facts
        // avoids duplicate probes in every transform handle while not adding
        // any new work to inverse/division fact scans.
        let translation_is_zero = [
            facts.translation_xyz_zero[0],
            facts.translation_xyz_zero[1],
            facts.translation_xyz_zero[2],
            matrix[3][3].definitely_zero(),
        ];
        let all_translation_zero = translation_is_zero.iter().all(|value| *value);
        let all_translation_nonzero = translation_is_zero.iter().all(|value| !*value);
        // Precompute the direction-linear diagonal matrix structure once so
        // all-direction batches can stay on shared scalar multiply without
        // per-vector helper branch probes. Translation is intentionally ignored:
        // homogeneous directions have w = 0, so the translation column cannot
        // contribute to the result.
        let direction_is_diagonal = facts.direction_linear_is_diagonal;
        Self {
            matrix,
            facts,
            translation_is_zero,
            all_translation_zero,
            all_translation_nonzero,
            direction_is_diagonal,
        }
    }

    pub fn transform_vector(&self, rhs: &Vector4<B>) -> Vector4<B> {
        self.transform_vector_with_facts(rhs, rhs.geometric_facts())
    }

    #[inline]
    pub fn transform_direction_vector(&self, rhs: &Vector4<B>) -> Vector4<B> {
        self.transform_vector_with_facts(
            rhs,
            Vector4GeometricFacts {
                homogeneous: Vector4HomogeneousKind::Direction,
            },
        )
    }

    #[inline]
    pub fn transform_point_vector(&self, rhs: &Vector4<B>) -> Vector4<B> {
        if self.facts.is_identity {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "transform-vector4-point-identity"
            );
            // Check retained identity before the affine-diagonal point kernel.
            // The handle already paid to classify the matrix, so cloning the
            // point preserves all exact/symbolic scalar structure and avoids
            // three identity multiplies plus three zero translations. This is
            // Yap's object-package principle applied at the prepared-kernel
            // boundary: use geometric facts before scalar arithmetic (Yap,
            // "Towards Exact Geometric Computation", 1997).
            return rhs.clone();
        }
        if self.facts.is_affine && self.facts.linear_is_diagonal {
            return Vector4(
                transform_vector4_rhs_point_affine_linear_diagonal_ref_cached(
                    &self.matrix.0,
                    &rhs.0,
                ),
            );
        }
        self.transform_vector_with_facts(
            rhs,
            Vector4GeometricFacts {
                homogeneous: Vector4HomogeneousKind::Point,
            },
        )
    }

    pub fn vector(&self, rhs: &'a Vector4<B>) -> TransformedVector4<'a, B> {
        TransformedVector4 {
            matrix: self.matrix,
            facts: self.facts,
            translation_is_zero: self.translation_is_zero,
            all_translation_zero: self.all_translation_zero,
            all_translation_nonzero: self.all_translation_nonzero,
            direction_is_diagonal: self.direction_is_diagonal,
            // Defer homogeneous classification until materialization.
            // Identity transforms can return the input without knowing whether
            // it is a point, direction, or unknown, so eager `w` classification
            // is wasted on that exact-object fast path. This keeps deferred
            // kernels aligned with Yap's recommendation to exploit geometric
            // object structure before lower-level number facts (Yap, "Towards
            // Exact Geometric Computation", 1997).
            vector_facts: None,
            vector: rhs,
        }
    }

    pub fn transform_vector_batch(&self, rhs: &[Vector4<B>]) -> Vec<Vector4<B>> {
        if self.facts.is_identity {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "transform-vector4-batch-identity"
            );
            return rhs.to_vec();
        }
        if self.facts.is_diagonal {
            let mut transformed = Vec::with_capacity(rhs.len());
            if let Some(first) = rhs.first() {
                match first.0[3].zero_or_one() {
                    Some(false) => {
                        if rhs
                            .iter()
                            .skip(1)
                            .all(|vector| vector.0[3].definitely_zero())
                        {
                            crate::trace_dispatch!(
                                "hyperlattice_matrix",
                                "helper",
                                "transform-vector4-batch-diagonal-direction"
                            );
                            // After the first vector classifies the batch candidate,
                            // the remaining direction scan only needs `w == 0`.
                            // This avoids asking whether every later direction is a
                            // point while keeping unknown first vectors on a single
                            // `zero_or_one` fallback probe. See Yap, "Towards Exact
                            // Geometric Computation", 1997.
                            for vector in rhs {
                                transformed.push(Vector4([
                                    vector.0[0].clone().mul_cached(&self.matrix.0[0][0]),
                                    vector.0[1].clone().mul_cached(&self.matrix.0[1][1]),
                                    vector.0[2].clone().mul_cached(&self.matrix.0[2][2]),
                                    Scalar::zero(),
                                ]));
                            }
                            return transformed;
                        }
                    }
                    Some(true)
                        if self.facts.is_affine
                            && rhs
                                .iter()
                                .skip(1)
                                .all(|vector| vector.0[3].definitely_one()) =>
                    {
                        crate::trace_dispatch!(
                            "hyperlattice_matrix",
                            "helper",
                            "transform-vector4-batch-diagonal-point"
                        );
                        // After the first vector classifies the batch candidate,
                        // uniform affine point batches only need `w == 1` for the
                        // remaining vectors. This preserves the exact homogeneous
                        // invariant without paying full point/direction
                        // classification per lane.
                        for vector in rhs {
                            transformed.push(Vector4([
                                vector.0[0].clone().mul_cached(&self.matrix.0[0][0]),
                                vector.0[1].clone().mul_cached(&self.matrix.0[1][1]),
                                vector.0[2].clone().mul_cached(&self.matrix.0[2][2]),
                                Scalar::one(),
                            ]));
                        }
                        return transformed;
                    }
                    _ => {}
                }
            }
            for vector in rhs {
                transformed.push(Vector4(from_fn(|row| {
                    vector.0[row].clone().mul_cached(&self.matrix.0[row][row])
                })));
            }
            return transformed;
        }
        let mut transformed = Vec::with_capacity(rhs.len());

        // Classify batch shape with one cheap pass and no per-vector storage.
        // This keeps all-regular batches allocation-free and lets unknown/point
        // direction specialization stay branch-free until needed.
        let mut has_direction = false;
        let mut has_point = false;
        let mut has_unknown = false;
        for vector in rhs {
            match vector.geometric_facts().homogeneous {
                Vector4HomogeneousKind::Direction => has_direction = true,
                Vector4HomogeneousKind::Point => has_point = true,
                Vector4HomogeneousKind::Unknown => has_unknown = true,
            }

            // If all three kinds appear, a mixed batch is certain and we stop
            // early; we still classify again below only when mixed is true.
            if (has_direction && has_point)
                || (has_direction && has_unknown)
                || (has_point && has_unknown)
            {
                break;
            }
        }

        // Common batch shapes (all directions, all points, all unknown) avoid
        // per-vector classification and fact-vector allocation.
        if has_direction && !has_point && !has_unknown {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "transform-vector4-batch-direction"
            );
            if self.direction_is_diagonal {
                for vector in rhs {
                    transformed.push(Vector4([
                        vector.0[0].clone().mul_cached(&self.matrix.0[0][0]),
                        vector.0[1].clone().mul_cached(&self.matrix.0[1][1]),
                        vector.0[2].clone().mul_cached(&self.matrix.0[2][2]),
                        Scalar::zero(),
                    ]));
                }
            } else {
                for vector in rhs {
                    transformed.push(Vector4(transform_vector4_rhs_direction_ref_cached(
                        &self.matrix.0,
                        &vector.0,
                        self.direction_is_diagonal,
                    )));
                }
            }
            return transformed;
        }

        if has_point && !has_direction && !has_unknown {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "transform-vector4-batch-point"
            );
            if self.facts.is_affine && self.facts.linear_is_diagonal {
                for vector in rhs {
                    transformed.push(Vector4(
                        transform_vector4_rhs_point_affine_linear_diagonal_ref_cached(
                            &self.matrix.0,
                            &vector.0,
                        ),
                    ));
                }
            } else if self.all_translation_nonzero {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "transform-vector4-batch-point-all-nonzero"
                );
                for vector in rhs {
                    transformed.push(Vector4(transform_vector4_rhs_point_all_nonzero_ref_cached(
                        &self.matrix.0,
                        &vector.0,
                    )));
                }
            } else if self.all_translation_zero {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "transform-vector4-batch-full-no-translation"
                );
                for vector in rhs {
                    transformed.push(Vector4(
                        transform_vector4_rhs_full_no_translation_ref_cached(
                            &self.matrix.0,
                            &vector.0,
                        ),
                    ));
                }
            } else {
                for vector in rhs {
                    transformed.push(Vector4(transform_vector4_rhs_point_ref_cached(
                        &self.matrix.0,
                        &vector.0,
                        &self.translation_is_zero,
                    )));
                }
            }
            return transformed;
        }

        if has_unknown && !has_direction && !has_point {
            // All unknown homogeneous vectors can use the generic point/pointish
            // kernel directly without materializing fact structs.
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "transform-vector4-batch-unknown"
            );
            for vector in rhs {
                transformed.push(Vector4(transform_vector4_rhs_ref_cached_with_matrix_facts(
                    &self.matrix.0,
                    &vector.0,
                    &self.translation_is_zero,
                    self.facts,
                )));
            }
            return transformed;
        }

        // Mixed shapes (direction/point/unknown combos) need per-vector facts.
        // Classify once in the second pass and keep the chosen fast kernels.
        let mut vector_facts = Vec::with_capacity(rhs.len());
        for vector in rhs {
            vector_facts.push(vector.geometric_facts());
        }

        if has_direction && has_point && !has_unknown {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "transform-vector4-batch-mixed"
            );
            for (vector, facts) in rhs.iter().zip(vector_facts.iter()) {
                transformed.push(self.transform_vector_with_facts(vector, *facts));
            }
            return transformed;
        }

        if has_unknown && (has_direction || has_point) {
            // Fallback: per-vector facts handles direction/unknown or
            // point/unknown mixtures safely.
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "transform-vector4-batch-mixed"
            );
            for (vector, facts) in rhs.iter().zip(vector_facts.iter()) {
                transformed.push(self.transform_vector_with_facts(vector, *facts));
            }
            return transformed;
        }

        // Degenerate safety net for empty batches or impossible classification
        // states; should be equivalent to mixed dispatch.
        for (vector, facts) in rhs.iter().zip(vector_facts.iter()) {
            transformed.push(self.transform_vector_with_facts(vector, *facts));
        }
        transformed
    }

    /// Transforms a batch whose inputs are known homogeneous directions.
    ///
    /// This is the static prepared-kernel form of the generic batch transform:
    /// callers that already know `w = 0` can avoid the batch classification pass
    /// and keep dispatch deterministic across every lane. The optimization is
    /// intentionally handle-scoped rather than stored on every vector or matrix,
    /// following Yap's recommendation to exploit geometric-object structure
    /// above the BigNumber layer without making each scalar operation heavier
    /// (Yap, "Towards Exact Geometric Computation", 1997).
    pub fn transform_direction_batch(&self, rhs: &[Vector4<B>]) -> Vec<Vector4<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "transform-vector4-direction-batch-assumed"
        );
        if self.facts.is_identity {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "transform-vector4-direction-batch-identity-assumed"
            );
            // Prepared handles already retain the exact identity fact. Use it
            // before the diagonal direction kernel so known directions under an
            // identity transform are cloned rather than multiplied by three
            // structural ones. This is the cheapest object-level reduction in
            // Yap's exact-geometric-computation sense: preserve the geometric
            // object fact and avoid scalar arithmetic entirely (Yap, "Towards
            // Exact Geometric Computation", 1997).
            return rhs.to_vec();
        }
        transform_vector4_direction_batch_assumed_ref(
            &self.matrix.0,
            rhs,
            self.direction_is_diagonal,
        )
    }

    /// Transforms a batch whose inputs are known homogeneous points.
    ///
    /// This skips the generic point/direction/unknown classification pass and
    /// keeps all lanes on the same prepared point schedule. The method is
    /// deliberately opt-in: ordinary `transform_vector_batch` remains thin, and
    /// only geometry code with object-level point facts pays for the specialized
    /// API surface.
    pub fn transform_point_batch(&self, rhs: &[Vector4<B>]) -> Vec<Vector4<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "transform-vector4-point-batch-assumed"
        );
        if self.facts.is_identity {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "transform-vector4-point-batch-identity-assumed"
            );
            // Same retained-object reduction as directions: for identity
            // transforms the mathematically exact result is the input batch, so
            // cloning preserves all scalar structure and avoids unnecessary
            // approximation, canonicalization, and additive identity work. See
            // Yap, "Towards Exact Geometric Computation", 1997.
            return rhs.to_vec();
        }
        let mut transformed = Vec::with_capacity(rhs.len());
        if self.facts.is_affine && self.facts.linear_is_diagonal {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "transform-vector4-point-batch-affine-linear-diagonal-assumed"
            );
            // Keep this hot affine point-batch loop local to the prepared
            // handle. Extracting it to a shared helper was tested in 2026-05 and
            // regressed approx by ~5-6% even with `#[inline]`, likely by
            // disturbing LLVM's scalarization of the small fixed loop. If a
            // future exact-backend run shows a stable hyperreal win from a
            // different shape, gate it behind a backend capability instead of
            // making compact interval lanes pay for it. This follows the
            // thin/static-kernel rule from the Yap backlog: exploit retained
            // point facts, but do not introduce abstraction cost into a
            // nanosecond-scale kernel.
            for vector in rhs {
                transformed.push(Vector4([
                    vector.0[0].clone().mul_cached(&self.matrix.0[0][0]) + &self.matrix.0[0][3],
                    vector.0[1].clone().mul_cached(&self.matrix.0[1][1]) + &self.matrix.0[1][3],
                    vector.0[2].clone().mul_cached(&self.matrix.0[2][2]) + &self.matrix.0[2][3],
                    // Keep the canonical point lane as a freshly constructed
                    // `1` in this batch kernel. Cloning the input `w` lane was
                    // tested because the API assumes a point, but it regressed
                    // both approx and hyperreal-family rows in this hot loop.
                    Scalar::one(),
                ]));
            }
            return transformed;
        }

        if self.all_translation_nonzero {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "transform-vector4-point-batch-all-nonzero-assumed"
            );
            for vector in rhs {
                transformed.push(Vector4(transform_vector4_rhs_point_all_nonzero_ref_cached(
                    &self.matrix.0,
                    &vector.0,
                )));
            }
            return transformed;
        }

        if self.all_translation_zero {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "transform-vector4-point-batch-no-translation-assumed"
            );
            for vector in rhs {
                transformed.push(Vector4(
                    transform_vector4_rhs_full_no_translation_ref_cached(&self.matrix.0, &vector.0),
                ));
            }
            return transformed;
        }

        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "transform-vector4-point-batch-partial-translation-assumed"
        );
        for vector in rhs {
            transformed.push(Vector4(transform_vector4_rhs_point_ref_cached(
                &self.matrix.0,
                &vector.0,
                &self.translation_is_zero,
            )));
        }
        transformed
    }
}

#[derive(Clone, Copy, Debug)]
pub struct TransformedVector3<'a, B: Backend = DefaultBackend> {
    matrix: &'a Matrix3<B>,
    facts: Matrix3Facts,
    vector: &'a Vector3<B>,
}

impl<'a, B: Backend> TransformedVector3<'a, B> {
    #[inline]
    pub fn materialize(self) -> Vector3<B> {
        if self.facts.is_identity {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "materialize-vector3-identity"
            );
            return self.vector.clone();
        }
        if self.facts.is_diagonal {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "materialize-vector3-diagonal"
            );
            return Vector3(from_fn(|row| {
                self.vector.0[row]
                    .clone()
                    .mul_cached(&self.matrix.0[row][row])
            }));
        }
        crate::trace_dispatch!("hyperlattice_matrix", "helper", "transform-vector3-dense");
        Vector3(transform_vector3_rhs_dense_ref(
            &self.matrix.0,
            &self.vector.0,
        ))
    }
}

#[derive(Clone, Copy, Debug)]
pub struct TransformedVector4<'a, B: Backend = DefaultBackend> {
    matrix: &'a Matrix4<B>,
    facts: Matrix4Facts,
    translation_is_zero: [bool; 4],
    all_translation_zero: bool,
    all_translation_nonzero: bool,
    direction_is_diagonal: bool,
    vector_facts: Option<Vector4GeometricFacts>,
    vector: &'a Vector4<B>,
}

impl<'a, B: Backend> TransformedVector4<'a, B> {
    #[inline]
    pub fn materialize(self) -> Vector4<B> {
        if self.facts.is_identity {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "materialize-vector4-identity"
            );
            return self.vector.clone();
        }
        let vector_facts = self
            .vector_facts
            .unwrap_or_else(|| self.vector.geometric_facts());
        if self.facts.is_diagonal {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "method",
                "materialize-vector4-diagonal"
            );
            if matches!(vector_facts.homogeneous, Vector4HomogeneousKind::Direction) {
                return Vector4([
                    self.vector.0[0].clone().mul_cached(&self.matrix.0[0][0]),
                    self.vector.0[1].clone().mul_cached(&self.matrix.0[1][1]),
                    self.vector.0[2].clone().mul_cached(&self.matrix.0[2][2]),
                    Scalar::zero(),
                ]);
            }
            if matches!(vector_facts.homogeneous, Vector4HomogeneousKind::Point)
                && self.facts.is_affine
            {
                // Preserve the retained point fact through deferred
                // materialization instead of flattening it into a fourth cached
                // multiply. The same projective invariant is used by the eager
                // transform path above; see Yap, "Towards Exact Geometric
                // Computation", 1997.
                return Vector4(
                    transform_vector4_rhs_point_affine_linear_diagonal_ref_cached(
                        &self.matrix.0,
                        &self.vector.0,
                    ),
                );
            }
            return Vector4(from_fn(|row| {
                self.vector.0[row]
                    .clone()
                    .mul_cached(&self.matrix.0[row][row])
            }));
        }
        if matches!(vector_facts.homogeneous, Vector4HomogeneousKind::Point)
            && self.facts.is_affine
            && self.facts.linear_is_diagonal
        {
            return Vector4(
                transform_vector4_rhs_point_affine_linear_diagonal_ref_cached(
                    &self.matrix.0,
                    &self.vector.0,
                ),
            );
        }
        Vector4(transform_vector4_rhs_ref_with_facts(
            &self.matrix.0,
            &self.vector.0,
            &self.translation_is_zero,
            self.all_translation_zero,
            self.all_translation_nonzero,
            self.direction_is_diagonal,
            Some(self.facts),
            vector_facts,
        ))
    }
}

#[inline]
fn scale_by_shared_factor<B: Backend>(value: Scalar<B>, factor: &Scalar<B>) -> Scalar<B> {
    // The determinant reciprocal is a common scale applied to every cofactor.
    // Hyperreal opts into borrowing that scale so exact/symbolic state is not
    // cloned per lane; compact approximate backends stay on owned multiply
    // because their two-f64 representation benchmarks faster after LLVM
    // scalarization. This is the fixed-size analogue of delaying the common
    // denominator in fraction-free elimination:
    // Bareiss, Math. Comp. 22(103), 1968, https://doi.org/10.2307/2004533.
    if B::BORROW_SHARED_SCALE_FACTOR {
        value.mul_cached(factor)
    } else {
        value * factor.clone()
    }
}

fn scale_matrix3<B: Backend>(
    matrix: [[Scalar<B>; 3]; 3],
    factor: &Scalar<B>,
) -> [[Scalar<B>; 3]; 3] {
    // Keep the shared determinant inverse borrowed and unroll the fixed 3x3
    // scale. The cofactor inverse/division kernels follow the fraction-free
    // principle of delaying the common denominator until the last pass
    // (Bareiss, Math. Comp. 22(103), 1968, https://doi.org/10.2307/2004533);
    // spelling out the final pass avoids nested `array::map` closure layout for
    // hyperreal reciprocal/div_matrix rows while preserving that single shared
    // inverse. `scale_by_shared_factor` deliberately keeps compact approximate
    // backends on owned multiplication; their scalar is two f64s, so clone
    // avoidance loses to the simpler optimized expression.
    let [[m00, m01, m02], [m10, m11, m12], [m20, m21, m22]] = matrix;
    [
        [
            scale_by_shared_factor(m00, factor),
            scale_by_shared_factor(m01, factor),
            scale_by_shared_factor(m02, factor),
        ],
        [
            scale_by_shared_factor(m10, factor),
            scale_by_shared_factor(m11, factor),
            scale_by_shared_factor(m12, factor),
        ],
        [
            scale_by_shared_factor(m20, factor),
            scale_by_shared_factor(m21, factor),
            scale_by_shared_factor(m22, factor),
        ],
    ]
}

fn scale_matrix4<B: Backend>(
    matrix: [[Scalar<B>; 4]; 4],
    factor: &Scalar<B>,
) -> [[Scalar<B>; 4]; 4] {
    // Same shared-scale rationale as `scale_matrix3`, but for right-division's
    // unscaled 4x4 adjugate. `invert_matrix4` has its own fused cofactor-scale
    // schedule, so this helper stays focused on matrix division.
    let [
        [m00, m01, m02, m03],
        [m10, m11, m12, m13],
        [m20, m21, m22, m23],
        [m30, m31, m32, m33],
    ] = matrix;
    [
        [
            scale_by_shared_factor(m00, factor),
            scale_by_shared_factor(m01, factor),
            scale_by_shared_factor(m02, factor),
            scale_by_shared_factor(m03, factor),
        ],
        [
            scale_by_shared_factor(m10, factor),
            scale_by_shared_factor(m11, factor),
            scale_by_shared_factor(m12, factor),
            scale_by_shared_factor(m13, factor),
        ],
        [
            scale_by_shared_factor(m20, factor),
            scale_by_shared_factor(m21, factor),
            scale_by_shared_factor(m22, factor),
            scale_by_shared_factor(m23, factor),
        ],
        [
            scale_by_shared_factor(m30, factor),
            scale_by_shared_factor(m31, factor),
            scale_by_shared_factor(m32, factor),
            scale_by_shared_factor(m33, factor),
        ],
    ]
}

#[inline]
fn mul_sub<B: Backend>(
    left_a: &Scalar<B>,
    right_a: &Scalar<B>,
    left_b: &Scalar<B>,
    right_b: &Scalar<B>,
) -> Scalar<B> {
    if B::FUSE_SIGNED_PRODUCT_SUM {
        // Structural zero pruning is intentionally done before forming exact
        // products. In hyperreal this avoids allocating symbolic/rational terms
        // that would later canonicalize to zero; in approximate backends this
        // helper is bypassed to keep LLVM's compact direct expression shape.
        // The sparse-kernel idea follows Gustavson's observation that skipping
        // known-zero products is the central win in sparse matrix arithmetic:
        // Gustavson, ACM TOMS 4(3), 1978, https://doi.org/10.1145/355791.355796.
        let first_zero = left_a.definitely_zero() || right_a.definitely_zero();
        let second_zero = left_b.definitely_zero() || right_b.definitely_zero();

        if first_zero || second_zero {
            crate::trace_dispatch!("hyperlattice_matrix", "helper", "mul-sub-pruned");
            if first_zero && second_zero {
                return Scalar::zero();
            }
            if first_zero {
                return -(left_b * right_b);
            }
            return left_a * right_a;
        }
        Scalar::active_signed_product_sum2([true, false], [[left_a, right_a], [left_b, right_b]])
    } else {
        left_a * right_a - left_b * right_b
    }
}

#[inline]
fn mul_sub_dense_exact<B: Backend>(
    left_a: &Scalar<B>,
    right_a: &Scalar<B>,
    left_b: &Scalar<B>,
    right_b: &Scalar<B>,
) -> Scalar<B> {
    crate::trace_dispatch!("hyperlattice_matrix", "helper", "mul-sub-dense-exact");
    Scalar::active_signed_product_sum2([true, false], [[left_a, right_a], [left_b, right_b]])
}

#[inline]
fn mul_sub_dense_exact_known_rational<B: Backend>(
    left_a: &Scalar<B>,
    right_a: &Scalar<B>,
    left_b: &Scalar<B>,
    right_b: &Scalar<B>,
) -> Scalar<B> {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "mul-sub-dense-exact-known-rational"
    );
    Scalar::active_signed_product_sum2_known_exact_rational(
        [true, false],
        [[left_a, right_a], [left_b, right_b]],
    )
}

fn mul_add<B: Backend>(
    left_a: &Scalar<B>,
    right_a: &Scalar<B>,
    left_b: &Scalar<B>,
    right_b: &Scalar<B>,
) -> Scalar<B> {
    if B::FUSE_SIGNED_PRODUCT_SUM {
        // Same structural-zero gate as `mul_sub`: delay exact product
        // construction until after cheap zero facts decide which lanes can
        // contribute. The surviving nonzero lanes are then passed to the
        // backend fused product-sum path so exact rationals can share one
        // denominator, mirroring Bareiss's delayed-canonicalization principle
        // (Math. Comp. 22(103), 1968, https://doi.org/10.2307/2004533).
        let first_zero = left_a.definitely_zero() || right_a.definitely_zero();
        let second_zero = left_b.definitely_zero() || right_b.definitely_zero();

        if first_zero || second_zero {
            crate::trace_dispatch!("hyperlattice_matrix", "helper", "mul-add-pruned");
            if first_zero && second_zero {
                return Scalar::zero();
            }
            if first_zero {
                return left_b * right_b;
            }
            return left_a * right_a;
        }
        Scalar::active_signed_product_sum2([true, true], [[left_a, right_a], [left_b, right_b]])
    } else {
        left_a * right_a + left_b * right_b
    }
}

#[inline]
fn mul_add_sub<B: Backend>(
    left_a: &Scalar<B>,
    right_a: &Scalar<B>,
    left_b: &Scalar<B>,
    right_b: &Scalar<B>,
    left_c: &Scalar<B>,
    right_c: &Scalar<B>,
) -> Scalar<B> {
    if B::FUSE_SIGNED_PRODUCT_SUM {
        // Three-term cofactors are the hottest inverse path. Check inexpensive
        // structural zero facts before building any products so sparse minors
        // collapse without approximation or BigInt gcd work. Dense minors still
        // use the fused exact-rational product-sum path to defer denominator
        // canonicalization until the backend sees all signed terms together.
        let first_zero = left_a.definitely_zero() || right_a.definitely_zero();
        let second_zero = left_b.definitely_zero() || right_b.definitely_zero();
        let third_zero = left_c.definitely_zero() || right_c.definitely_zero();
        let nonzero_count = (!first_zero) as u8 + (!second_zero) as u8 + (!third_zero) as u8;

        if nonzero_count <= 2 {
            crate::trace_dispatch!("hyperlattice_matrix", "helper", "mul-add-sub-pruned");
            return match nonzero_count {
                0 => Scalar::zero(),
                1 => {
                    if !first_zero {
                        left_a * right_a
                    } else if !second_zero {
                        left_b * right_b
                    } else {
                        -(left_c * right_c)
                    }
                }
                2 => {
                    if first_zero {
                        Scalar::active_signed_product_sum2(
                            [true, false],
                            [[left_b, right_b], [left_c, right_c]],
                        )
                    } else if second_zero {
                        Scalar::active_signed_product_sum2(
                            [true, false],
                            [[left_a, right_a], [left_c, right_c]],
                        )
                    } else {
                        Scalar::active_signed_product_sum2(
                            [true, true],
                            [[left_a, right_a], [left_b, right_b]],
                        )
                    }
                }
                _ => unreachable!(),
            };
        }
        Scalar::active_signed_product_sum2(
            [true, true, false],
            [[left_a, right_a], [left_b, right_b], [left_c, right_c]],
        )
    } else {
        mul_add(left_a, right_a, left_b, right_b) - left_c * right_c
    }
}

#[inline]
fn mul_add_sub_dense_exact<B: Backend>(
    left_a: &Scalar<B>,
    right_a: &Scalar<B>,
    left_b: &Scalar<B>,
    right_b: &Scalar<B>,
    left_c: &Scalar<B>,
    right_c: &Scalar<B>,
) -> Scalar<B> {
    crate::trace_dispatch!("hyperlattice_matrix", "helper", "mul-add-sub-dense-exact");
    Scalar::active_signed_product_sum2(
        [true, true, false],
        [[left_a, right_a], [left_b, right_b], [left_c, right_c]],
    )
}

#[inline]
fn mul_add_sub_dense_exact_known_rational<B: Backend>(
    left_a: &Scalar<B>,
    right_a: &Scalar<B>,
    left_b: &Scalar<B>,
    right_b: &Scalar<B>,
    left_c: &Scalar<B>,
    right_c: &Scalar<B>,
) -> Scalar<B> {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "mul-add-sub-dense-exact-known-rational"
    );
    Scalar::active_signed_product_sum2_known_exact_rational(
        [true, true, false],
        [[left_a, right_a], [left_b, right_b], [left_c, right_c]],
    )
}

fn mul_sub_add<B: Backend>(
    left_a: &Scalar<B>,
    right_a: &Scalar<B>,
    left_b: &Scalar<B>,
    right_b: &Scalar<B>,
    left_c: &Scalar<B>,
    right_c: &Scalar<B>,
) -> Scalar<B> {
    if B::FUSE_SIGNED_PRODUCT_SUM {
        // Keep the sign pattern separate from the zero-pruning decision. This
        // lets structural facts remove zero lanes before the exact backend sees
        // the signed product sum, reducing unnecessary symbolic nodes while
        // preserving the same determinant/cofactor polynomial.
        let first_zero = left_a.definitely_zero() || right_a.definitely_zero();
        let second_zero = left_b.definitely_zero() || right_b.definitely_zero();
        let third_zero = left_c.definitely_zero() || right_c.definitely_zero();
        let nonzero_count = (!first_zero) as u8 + (!second_zero) as u8 + (!third_zero) as u8;

        if nonzero_count <= 2 {
            crate::trace_dispatch!("hyperlattice_matrix", "helper", "mul-sub-add-pruned");
            return match nonzero_count {
                0 => Scalar::zero(),
                1 => {
                    if !first_zero {
                        left_a * right_a
                    } else if !second_zero {
                        -(left_b * right_b)
                    } else {
                        -(left_c * right_c)
                    }
                }
                2 => {
                    if first_zero {
                        Scalar::active_signed_product_sum2(
                            [false, false],
                            [[left_b, right_b], [left_c, right_c]],
                        )
                    } else if second_zero {
                        Scalar::active_signed_product_sum2(
                            [true, false],
                            [[left_a, right_a], [left_c, right_c]],
                        )
                    } else {
                        Scalar::active_signed_product_sum2(
                            [true, false],
                            [[left_a, right_a], [left_b, right_b]],
                        )
                    }
                }
                _ => unreachable!(),
            };
        }
        Scalar::active_signed_product_sum2(
            [true, false, false],
            [[left_a, right_a], [left_b, right_b], [left_c, right_c]],
        )
    } else {
        left_a * right_a - mul_add(left_b, right_b, left_c, right_c)
    }
}

#[inline]
fn mul_sub_add_dense_exact<B: Backend>(
    left_a: &Scalar<B>,
    right_a: &Scalar<B>,
    left_b: &Scalar<B>,
    right_b: &Scalar<B>,
    left_c: &Scalar<B>,
    right_c: &Scalar<B>,
) -> Scalar<B> {
    crate::trace_dispatch!("hyperlattice_matrix", "helper", "mul-sub-add-dense-exact");
    Scalar::active_signed_product_sum2(
        [true, false, false],
        [[left_a, right_a], [left_b, right_b], [left_c, right_c]],
    )
}

#[inline]
fn mul_sub_add_dense_exact_known_rational<B: Backend>(
    left_a: &Scalar<B>,
    right_a: &Scalar<B>,
    left_b: &Scalar<B>,
    right_b: &Scalar<B>,
    left_c: &Scalar<B>,
    right_c: &Scalar<B>,
) -> Scalar<B> {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "mul-sub-add-dense-exact-known-rational"
    );
    Scalar::active_signed_product_sum2_known_exact_rational(
        [true, false, false],
        [[left_a, right_a], [left_b, right_b], [left_c, right_c]],
    )
}

#[inline]
fn determinant3<B: Backend>(m: &[[Scalar<B>; 3]; 3]) -> Scalar<B> {
    crate::trace_dispatch!("hyperlattice_matrix", "helper", "determinant3");
    // Keep determinant infallible and division-free. A Bareiss prototype would
    // need pivot divisions and a fallback for singular or unknown-zero pivots,
    // which does not match the public determinant contract and adds exact
    // rational normalization work to the common 3x3 case. The algorithm was
    // checked against Bareiss's integer-preserving elimination paper
    // (https://www.ams.org/mcom/1968-22-103/S0025-5718-1968-0226829-0/S0025-5718-1968-0226829-0.pdf);
    // for these fixed sizes, keeping cofactors division-free plus delaying dot
    // canonicalization in hyperreal gave the measured wins without changing
    // determinant semantics.
    let c00 = mul_sub(&m[1][1], &m[2][2], &m[1][2], &m[2][1]);
    let c10 = mul_sub(&m[1][2], &m[2][0], &m[1][0], &m[2][2]);
    let c20 = mul_sub(&m[1][0], &m[2][1], &m[1][1], &m[2][0]);
    Scalar::dot3([&m[0][0], &m[0][1], &m[0][2]], [&c00, &c10, &c20])
}

#[inline]
fn matrix3_adjugate_and_determinant<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
) -> ([[Scalar<B>; 3]; 3], Scalar<B>) {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "matrix3-adjugate-and-determinant"
    );
    let m = &matrix;
    let c00 = mul_sub(&m[1][1], &m[2][2], &m[1][2], &m[2][1]);
    let c01 = mul_sub(&m[0][2], &m[2][1], &m[0][1], &m[2][2]);
    let c02 = mul_sub(&m[0][1], &m[1][2], &m[0][2], &m[1][1]);
    let c10 = mul_sub(&m[1][2], &m[2][0], &m[1][0], &m[2][2]);
    let c11 = mul_sub(&m[0][0], &m[2][2], &m[0][2], &m[2][0]);
    let c12 = mul_sub(&m[0][2], &m[1][0], &m[0][0], &m[1][2]);
    let c20 = mul_sub(&m[1][0], &m[2][1], &m[1][1], &m[2][0]);
    let c21 = mul_sub(&m[0][1], &m[2][0], &m[0][0], &m[2][1]);
    let c22 = mul_sub(&m[0][0], &m[1][1], &m[0][1], &m[1][0]);
    let det = Scalar::dot3([&m[0][0], &m[0][1], &m[0][2]], [&c00, &c10, &c20]);
    ([[c00, c01, c02], [c10, c11, c12], [c20, c21, c22]], det)
}

#[inline(never)]
fn matrix3_adjugate_and_determinant_dense_exact<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
) -> ([[Scalar<B>; 3]; 3], Scalar<B>) {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "matrix3-adjugate-and-determinant-dense-exact"
    );
    let m = &matrix;
    let c00 = mul_sub_dense_exact(&m[1][1], &m[2][2], &m[1][2], &m[2][1]);
    let c01 = mul_sub_dense_exact(&m[0][2], &m[2][1], &m[0][1], &m[2][2]);
    let c02 = mul_sub_dense_exact(&m[0][1], &m[1][2], &m[0][2], &m[1][1]);
    let c10 = mul_sub_dense_exact(&m[1][2], &m[2][0], &m[1][0], &m[2][2]);
    let c11 = mul_sub_dense_exact(&m[0][0], &m[2][2], &m[0][2], &m[2][0]);
    let c12 = mul_sub_dense_exact(&m[0][2], &m[1][0], &m[0][0], &m[1][2]);
    let c20 = mul_sub_dense_exact(&m[1][0], &m[2][1], &m[1][1], &m[2][0]);
    let c21 = mul_sub_dense_exact(&m[0][1], &m[2][0], &m[0][0], &m[2][1]);
    let c22 = mul_sub_dense_exact(&m[0][0], &m[1][1], &m[0][1], &m[1][0]);
    let det =
        Scalar::active_linear_combination3([&m[0][0], &m[0][1], &m[0][2]], [&c00, &c10, &c20]);
    ([[c00, c01, c02], [c10, c11, c12], [c20, c21, c22]], det)
}

#[inline(never)]
fn matrix3_adjugate_and_determinant_dense_exact_known_rational<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
) -> ([[Scalar<B>; 3]; 3], Scalar<B>) {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "matrix3-adjugate-and-determinant-dense-exact-known-rational"
    );
    let m = &matrix;
    let c00 = mul_sub_dense_exact_known_rational(&m[1][1], &m[2][2], &m[1][2], &m[2][1]);
    let c01 = mul_sub_dense_exact_known_rational(&m[0][2], &m[2][1], &m[0][1], &m[2][2]);
    let c02 = mul_sub_dense_exact_known_rational(&m[0][1], &m[1][2], &m[0][2], &m[1][1]);
    let c10 = mul_sub_dense_exact_known_rational(&m[1][2], &m[2][0], &m[1][0], &m[2][2]);
    let c11 = mul_sub_dense_exact_known_rational(&m[0][0], &m[2][2], &m[0][2], &m[2][0]);
    let c12 = mul_sub_dense_exact_known_rational(&m[0][2], &m[1][0], &m[0][0], &m[1][2]);
    let c20 = mul_sub_dense_exact_known_rational(&m[1][0], &m[2][1], &m[1][1], &m[2][0]);
    let c21 = mul_sub_dense_exact_known_rational(&m[0][1], &m[2][0], &m[0][0], &m[2][1]);
    let c22 = mul_sub_dense_exact_known_rational(&m[0][0], &m[1][1], &m[0][1], &m[1][0]);
    let det = Scalar::active_signed_product_sum2_known_exact_rational(
        [true, true, true],
        [[&m[0][0], &c00], [&m[0][1], &c10], [&m[0][2], &c20]],
    );
    ([[c00, c01, c02], [c10, c11, c12], [c20, c21, c22]], det)
}

#[inline]
fn matrix3_scaled_adjugate<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    crate::trace_dispatch!("hyperlattice_matrix", "helper", "matrix3-scaled-adjugate");
    let m = &matrix;
    let c00 = mul_sub(&m[1][1], &m[2][2], &m[1][2], &m[2][1]);
    let c01 = mul_sub(&m[0][2], &m[2][1], &m[0][1], &m[2][2]);
    let c02 = mul_sub(&m[0][1], &m[1][2], &m[0][2], &m[1][1]);
    let c10 = mul_sub(&m[1][2], &m[2][0], &m[1][0], &m[2][2]);
    let c11 = mul_sub(&m[0][0], &m[2][2], &m[0][2], &m[2][0]);
    let c12 = mul_sub(&m[0][2], &m[1][0], &m[0][0], &m[1][2]);
    let c20 = mul_sub(&m[1][0], &m[2][1], &m[1][1], &m[2][0]);
    let c21 = mul_sub(&m[0][1], &m[2][0], &m[0][0], &m[2][1]);
    let c22 = mul_sub(&m[0][0], &m[1][1], &m[0][1], &m[1][0]);
    let det = Scalar::dot3([&m[0][0], &m[0][1], &m[0][2]], [&c00, &c10, &c20]);
    let inv_det = det.inverse()?;
    // Mat3 reciprocal is hot enough to keep a scaled-cofactor schedule separate
    // from right-division's unscaled-adjugate path. This avoids constructing an
    // intermediate matrix only to immediately rescale it, while preserving one
    // shared determinant reciprocal. The delayed common-scale principle follows
    // Bareiss's fraction-free exact linear algebra work, Math. Comp. 22(103),
    // 1968, https://doi.org/10.2307/2004533.
    Ok([
        [
            scale_by_shared_factor(c00, &inv_det),
            scale_by_shared_factor(c01, &inv_det),
            scale_by_shared_factor(c02, &inv_det),
        ],
        [
            scale_by_shared_factor(c10, &inv_det),
            scale_by_shared_factor(c11, &inv_det),
            scale_by_shared_factor(c12, &inv_det),
        ],
        [
            scale_by_shared_factor(c20, &inv_det),
            scale_by_shared_factor(c21, &inv_det),
            scale_by_shared_factor(c22, &inv_det),
        ],
    ])
}

#[inline(never)]
fn matrix3_scaled_adjugate_dense_exact<B: Backend>(
    matrix: &[[Scalar<B>; 3]; 3],
) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "matrix3-scaled-adjugate-dense-exact"
    );
    let m = &matrix;
    let c00 = mul_sub_dense_exact(&m[1][1], &m[2][2], &m[1][2], &m[2][1]);
    let c01 = mul_sub_dense_exact(&m[0][2], &m[2][1], &m[0][1], &m[2][2]);
    let c02 = mul_sub_dense_exact(&m[0][1], &m[1][2], &m[0][2], &m[1][1]);
    let c10 = mul_sub_dense_exact(&m[1][2], &m[2][0], &m[1][0], &m[2][2]);
    let c11 = mul_sub_dense_exact(&m[0][0], &m[2][2], &m[0][2], &m[2][0]);
    let c12 = mul_sub_dense_exact(&m[0][2], &m[1][0], &m[0][0], &m[1][2]);
    let c20 = mul_sub_dense_exact(&m[1][0], &m[2][1], &m[1][1], &m[2][0]);
    let c21 = mul_sub_dense_exact(&m[0][1], &m[2][0], &m[0][0], &m[2][1]);
    let c22 = mul_sub_dense_exact(&m[0][0], &m[1][1], &m[0][1], &m[1][0]);
    let det =
        Scalar::active_linear_combination3([&m[0][0], &m[0][1], &m[0][2]], [&c00, &c10, &c20]);
    let inv_det = det.inverse()?;
    Ok([
        [
            scale_by_shared_factor(c00, &inv_det),
            scale_by_shared_factor(c01, &inv_det),
            scale_by_shared_factor(c02, &inv_det),
        ],
        [
            scale_by_shared_factor(c10, &inv_det),
            scale_by_shared_factor(c11, &inv_det),
            scale_by_shared_factor(c12, &inv_det),
        ],
        [
            scale_by_shared_factor(c20, &inv_det),
            scale_by_shared_factor(c21, &inv_det),
            scale_by_shared_factor(c22, &inv_det),
        ],
    ])
}

#[inline]
fn invert_matrix3<B: Backend>(matrix: [[Scalar<B>; 3]; 3]) -> BlasResult<[[Scalar<B>; 3]; 3]> {
    crate::trace_dispatch!("hyperlattice_matrix", "helper", "invert-matrix3");
    if matrix3_is_definitely_dense_for_inverse(&matrix) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix3-dense-cofactor"
        );
        if B::FUSE_SIGNED_PRODUCT_SUM {
            return matrix3_scaled_adjugate_dense_exact(&matrix);
        }
        return matrix3_scaled_adjugate(&matrix);
    }
    let facts = matrix3_facts(&matrix);
    if facts.is_identity {
        crate::trace_dispatch!("hyperlattice_matrix", "helper", "invert-matrix3-identity");
        return Ok(matrix);
    }
    if facts.is_diagonal {
        crate::trace_dispatch!("hyperlattice_matrix", "helper", "invert-matrix3-diagonal");
        return invert_matrix3_by_diagonal(&matrix);
    }
    if facts.is_upper_triangular {
        // Triangular kernels beat general affine/cofactor methods when this fact
        // holds, because each row/column has one structural dependency chain.
        // This is a small, explicit specialization for exact-geometric workloads.
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix3-upper-triangular"
        );
        return invert_matrix3_upper_triangular(&matrix);
    }
    if facts.is_lower_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix3-lower-triangular"
        );
        return invert_matrix3_lower_triangular(&matrix);
    }
    if facts.is_affine {
        crate::trace_dispatch!("hyperlattice_matrix", "helper", "invert-matrix3-affine");
        return invert_matrix3_affine(&matrix, facts.linear_is_diagonal);
    }
    // Cofactor inversion is intentionally kept for 3x3 reciprocal/inverse.
    // A Gauss-Jordan solve against the identity was benchmarked on the matrix
    // suite and was much slower because it pays one pivot inverse per column.
    matrix3_scaled_adjugate(&matrix)
}

#[inline]
fn invert_matrix3_checked<B: Backend>(
    matrix: [[Scalar<B>; 3]; 3],
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    crate::trace_dispatch!("hyperlattice_matrix", "helper", "invert-matrix3-checked");
    if matrix3_is_definitely_dense_for_inverse(&matrix) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix3-checked-dense-cofactor"
        );
        let (adjugate, det) = if B::FUSE_SIGNED_PRODUCT_SUM {
            matrix3_adjugate_and_determinant_dense_exact(&matrix)
        } else {
            matrix3_adjugate_and_determinant(&matrix)
        };
        require_known_nonzero(&det)?;
        let inv_det = det.inverse()?;
        return Ok(scale_matrix3(adjugate, &inv_det));
    }
    let facts = matrix3_facts(&matrix);
    if facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix3-checked-identity"
        );
        return Ok(matrix);
    }
    if facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix3-checked-diagonal"
        );
        return invert_matrix3_by_diagonal_checked(&matrix);
    }
    if facts.is_upper_triangular {
        // Checked fast path preserves the same dispatch preference as ordinary
        // inverse but with an explicit nonzero guarantee on diagonal pivots.
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix3-checked-upper-triangular"
        );
        return invert_matrix3_upper_triangular_checked(&matrix);
    }
    if facts.is_lower_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix3-checked-lower-triangular"
        );
        return invert_matrix3_lower_triangular_checked(&matrix);
    }
    if facts.is_affine {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix3-checked-affine"
        );
        return invert_matrix3_affine_checked(&matrix, facts.linear_is_diagonal);
    }
    let (adjugate, det) = matrix3_adjugate_and_determinant(&matrix);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(adjugate, &inv_det))
}

#[inline]
fn invert_matrix3_checked_with_abort<B: Backend>(
    matrix: [[Scalar<B>; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 3]; 3]> {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "invert-matrix3-checked-with-abort"
    );
    if matrix3_is_definitely_dense_for_inverse(&matrix) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix3-checked-with-abort-dense-cofactor"
        );
        let (adjugate, det) = if B::FUSE_SIGNED_PRODUCT_SUM {
            matrix3_adjugate_and_determinant_dense_exact(&matrix)
        } else {
            matrix3_adjugate_and_determinant(&matrix)
        };
        let det = with_abort(det, signal);
        require_known_nonzero(&det)?;
        let inv_det = det.inverse()?;
        return Ok(scale_matrix3(adjugate, &inv_det));
    }
    let facts = matrix3_facts(&matrix);
    if facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix3-checked-with-abort-identity"
        );
        return Ok(matrix);
    }
    if facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix3-checked-with-abort-diagonal"
        );
        return invert_matrix3_by_diagonal_checked_with_abort(&matrix, signal);
    }
    if facts.is_upper_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix3-checked-with-abort-upper-triangular"
        );
        return invert_matrix3_upper_triangular_checked_with_abort(&matrix, signal);
    }
    if facts.is_lower_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix3-checked-with-abort-lower-triangular"
        );
        return invert_matrix3_lower_triangular_checked_with_abort(&matrix, signal);
    }
    if facts.is_affine {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix3-checked-with-abort-affine"
        );
        return invert_matrix3_affine_checked_with_abort(&matrix, signal, facts.linear_is_diagonal);
    }
    let (adjugate, det) = matrix3_adjugate_and_determinant(&matrix);
    let det = with_abort(det, signal);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(adjugate, &inv_det))
}

#[inline]
fn matrix4_factors<B: Backend>(m: &[[Scalar<B>; 4]; 4]) -> ([Scalar<B>; 6], [Scalar<B>; 6]) {
    crate::trace_dispatch!("hyperlattice_matrix", "helper", "matrix4-factors");
    // Keep the cofactor inverse helpers inline across crate boundaries. The
    // full suite exposed a mat4 reciprocal layout regression; after inlining
    // the fixed inverse/cofactor layers, 200-sample/8s targeted reruns improved
    // approximate mat4 reciprocal by ~2.79% and hyperreal mat4 reciprocal by
    // ~3.99%, with astro128/numerica128 reciprocal staying inside noise.
    let s = [
        mul_sub(&m[0][0], &m[1][1], &m[1][0], &m[0][1]),
        mul_sub(&m[0][0], &m[1][2], &m[1][0], &m[0][2]),
        mul_sub(&m[0][0], &m[1][3], &m[1][0], &m[0][3]),
        mul_sub(&m[0][1], &m[1][2], &m[1][1], &m[0][2]),
        mul_sub(&m[0][1], &m[1][3], &m[1][1], &m[0][3]),
        mul_sub(&m[0][2], &m[1][3], &m[1][2], &m[0][3]),
    ];
    let c = [
        mul_sub(&m[2][0], &m[3][1], &m[3][0], &m[2][1]),
        mul_sub(&m[2][0], &m[3][2], &m[3][0], &m[2][2]),
        mul_sub(&m[2][0], &m[3][3], &m[3][0], &m[2][3]),
        mul_sub(&m[2][1], &m[3][2], &m[3][1], &m[2][2]),
        mul_sub(&m[2][1], &m[3][3], &m[3][1], &m[2][3]),
        mul_sub(&m[2][2], &m[3][3], &m[3][2], &m[2][3]),
    ];
    (s, c)
}

#[inline(never)]
fn matrix4_factors_dense_exact<B: Backend>(
    m: &[[Scalar<B>; 4]; 4],
) -> ([Scalar<B>; 6], [Scalar<B>; 6]) {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "matrix4-factors-dense-exact"
    );
    let s = [
        mul_sub_dense_exact(&m[0][0], &m[1][1], &m[1][0], &m[0][1]),
        mul_sub_dense_exact(&m[0][0], &m[1][2], &m[1][0], &m[0][2]),
        mul_sub_dense_exact(&m[0][0], &m[1][3], &m[1][0], &m[0][3]),
        mul_sub_dense_exact(&m[0][1], &m[1][2], &m[1][1], &m[0][2]),
        mul_sub_dense_exact(&m[0][1], &m[1][3], &m[1][1], &m[0][3]),
        mul_sub_dense_exact(&m[0][2], &m[1][3], &m[1][2], &m[0][3]),
    ];
    let c = [
        mul_sub_dense_exact(&m[2][0], &m[3][1], &m[3][0], &m[2][1]),
        mul_sub_dense_exact(&m[2][0], &m[3][2], &m[3][0], &m[2][2]),
        mul_sub_dense_exact(&m[2][0], &m[3][3], &m[3][0], &m[2][3]),
        mul_sub_dense_exact(&m[2][1], &m[3][2], &m[3][1], &m[2][2]),
        mul_sub_dense_exact(&m[2][1], &m[3][3], &m[3][1], &m[2][3]),
        mul_sub_dense_exact(&m[2][2], &m[3][3], &m[3][2], &m[2][3]),
    ];
    (s, c)
}

#[inline(never)]
fn matrix4_factors_dense_exact_known_rational<B: Backend>(
    m: &[[Scalar<B>; 4]; 4],
) -> ([Scalar<B>; 6], [Scalar<B>; 6]) {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "matrix4-factors-dense-exact-known-rational"
    );
    let s = [
        mul_sub_dense_exact_known_rational(&m[0][0], &m[1][1], &m[1][0], &m[0][1]),
        mul_sub_dense_exact_known_rational(&m[0][0], &m[1][2], &m[1][0], &m[0][2]),
        mul_sub_dense_exact_known_rational(&m[0][0], &m[1][3], &m[1][0], &m[0][3]),
        mul_sub_dense_exact_known_rational(&m[0][1], &m[1][2], &m[1][1], &m[0][2]),
        mul_sub_dense_exact_known_rational(&m[0][1], &m[1][3], &m[1][1], &m[0][3]),
        mul_sub_dense_exact_known_rational(&m[0][2], &m[1][3], &m[1][2], &m[0][3]),
    ];
    let c = [
        mul_sub_dense_exact_known_rational(&m[2][0], &m[3][1], &m[3][0], &m[2][1]),
        mul_sub_dense_exact_known_rational(&m[2][0], &m[3][2], &m[3][0], &m[2][2]),
        mul_sub_dense_exact_known_rational(&m[2][0], &m[3][3], &m[3][0], &m[2][3]),
        mul_sub_dense_exact_known_rational(&m[2][1], &m[3][2], &m[3][1], &m[2][2]),
        mul_sub_dense_exact_known_rational(&m[2][1], &m[3][3], &m[3][1], &m[2][3]),
        mul_sub_dense_exact_known_rational(&m[2][2], &m[3][3], &m[3][2], &m[2][3]),
    ];
    (s, c)
}

fn determinant4_from_factors<B: Backend>(s: &[Scalar<B>; 6], c: &[Scalar<B>; 6]) -> Scalar<B> {
    crate::trace_dispatch!("hyperlattice_matrix", "helper", "determinant4-from-factors");
    // This is the fixed six-minor determinant polynomial
    //   s0*c5 - s1*c4 + s2*c3 + s3*c2 - s4*c1 + s5*c0.
    // Route it as one signed product sum so hyperreal exact rationals can
    // share a final denominator instead of reducing a dot product plus two
    // extra products and a subtraction. Backends that do not opt in keep the
    // direct expression below, preserving the approximate scalar expression
    // shape measured in the regression guard benchmarks.
    if B::FUSE_SIGNED_PRODUCT_SUM {
        Scalar::signed_product_sum2(
            [true, false, true, true, false, true],
            [
                [&s[0], &c[5]],
                [&s[1], &c[4]],
                [&s[2], &c[3]],
                [&s[3], &c[2]],
                [&s[4], &c[1]],
                [&s[5], &c[0]],
            ],
        )
    } else {
        let positive = Scalar::dot3([&s[0], &s[2], &s[3]], [&c[5], &c[3], &c[2]]) + &s[5] * &c[0];
        let negative = &s[1] * &c[4] + &s[4] * &c[1];
        positive - negative
    }
}

fn determinant4_from_factors_known_rational<B: Backend>(
    s: &[Scalar<B>; 6],
    c: &[Scalar<B>; 6],
) -> Scalar<B> {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "determinant4-from-factors-known-rational"
    );
    Scalar::active_signed_product_sum2_known_exact_rational(
        [true, false, true, true, false, true],
        [
            [&s[0], &c[5]],
            [&s[1], &c[4]],
            [&s[2], &c[3]],
            [&s[3], &c[2]],
            [&s[4], &c[1]],
            [&s[5], &c[0]],
        ],
    )
}

#[inline]
fn determinant4<B: Backend>(m: &[[Scalar<B>; 4]; 4]) -> Scalar<B> {
    crate::trace_dispatch!("hyperlattice_matrix", "helper", "determinant4");
    // The six-minor formula shares the same division-free rationale as 3x3.
    // It is also reused by the cofactor inverse path, so determinant and
    // inverse stay aligned with the trace counters used for regression checks.
    // Bareiss/Gauss-Jordan alternatives remain useful for larger or purely
    // integer systems, but on this 4x4 public API the traced bottleneck was
    // rational canonicalization inside dot products, not the minor schedule.
    let (s, c) = matrix4_factors(m);
    determinant4_from_factors(&s, &c)
}

#[inline]
fn matrix4_scaled_adjugate_from_factors<B: Backend>(
    m: &[[Scalar<B>; 4]; 4],
    s: &[Scalar<B>; 6],
    c: &[Scalar<B>; 6],
    inv_det: &Scalar<B>,
) -> [[Scalar<B>; 4]; 4] {
    [
        [
            scale_by_shared_factor(
                mul_add_sub(&m[1][1], &c[5], &m[1][3], &c[3], &m[1][2], &c[4]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_sub_add(&m[0][2], &c[4], &m[0][1], &c[5], &m[0][3], &c[3]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_add_sub(&m[3][1], &s[5], &m[3][3], &s[3], &m[3][2], &s[4]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_sub_add(&m[2][2], &s[4], &m[2][1], &s[5], &m[2][3], &s[3]),
                inv_det,
            ),
        ],
        [
            scale_by_shared_factor(
                mul_sub_add(&m[1][2], &c[2], &m[1][0], &c[5], &m[1][3], &c[1]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_add_sub(&m[0][0], &c[5], &m[0][3], &c[1], &m[0][2], &c[2]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_sub_add(&m[3][2], &s[2], &m[3][0], &s[5], &m[3][3], &s[1]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_add_sub(&m[2][0], &s[5], &m[2][3], &s[1], &m[2][2], &s[2]),
                inv_det,
            ),
        ],
        [
            scale_by_shared_factor(
                mul_add_sub(&m[1][0], &c[4], &m[1][3], &c[0], &m[1][1], &c[2]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_sub_add(&m[0][1], &c[2], &m[0][0], &c[4], &m[0][3], &c[0]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_add_sub(&m[3][0], &s[4], &m[3][3], &s[0], &m[3][1], &s[2]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_sub_add(&m[2][1], &s[2], &m[2][0], &s[4], &m[2][3], &s[0]),
                inv_det,
            ),
        ],
        [
            scale_by_shared_factor(
                mul_sub_add(&m[1][1], &c[1], &m[1][0], &c[3], &m[1][2], &c[0]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_add_sub(&m[0][0], &c[3], &m[0][2], &c[0], &m[0][1], &c[1]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_sub_add(&m[3][1], &s[1], &m[3][0], &s[3], &m[3][2], &s[0]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_add_sub(&m[2][0], &s[3], &m[2][2], &s[0], &m[2][1], &s[1]),
                inv_det,
            ),
        ],
    ]
}

#[inline(never)]
fn matrix4_scaled_adjugate_from_factors_dense_exact<B: Backend>(
    m: &[[Scalar<B>; 4]; 4],
    s: &[Scalar<B>; 6],
    c: &[Scalar<B>; 6],
    inv_det: &Scalar<B>,
) -> [[Scalar<B>; 4]; 4] {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "matrix4-scaled-adjugate-dense-exact"
    );
    [
        [
            scale_by_shared_factor(
                mul_add_sub_dense_exact(&m[1][1], &c[5], &m[1][3], &c[3], &m[1][2], &c[4]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_sub_add_dense_exact(&m[0][2], &c[4], &m[0][1], &c[5], &m[0][3], &c[3]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_add_sub_dense_exact(&m[3][1], &s[5], &m[3][3], &s[3], &m[3][2], &s[4]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_sub_add_dense_exact(&m[2][2], &s[4], &m[2][1], &s[5], &m[2][3], &s[3]),
                inv_det,
            ),
        ],
        [
            scale_by_shared_factor(
                mul_sub_add_dense_exact(&m[1][2], &c[2], &m[1][0], &c[5], &m[1][3], &c[1]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_add_sub_dense_exact(&m[0][0], &c[5], &m[0][3], &c[1], &m[0][2], &c[2]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_sub_add_dense_exact(&m[3][2], &s[2], &m[3][0], &s[5], &m[3][3], &s[1]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_add_sub_dense_exact(&m[2][0], &s[5], &m[2][3], &s[1], &m[2][2], &s[2]),
                inv_det,
            ),
        ],
        [
            scale_by_shared_factor(
                mul_add_sub_dense_exact(&m[1][0], &c[4], &m[1][3], &c[0], &m[1][1], &c[2]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_sub_add_dense_exact(&m[0][1], &c[2], &m[0][0], &c[4], &m[0][3], &c[0]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_add_sub_dense_exact(&m[3][0], &s[4], &m[3][3], &s[0], &m[3][1], &s[2]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_sub_add_dense_exact(&m[2][1], &s[2], &m[2][0], &s[4], &m[2][3], &s[0]),
                inv_det,
            ),
        ],
        [
            scale_by_shared_factor(
                mul_sub_add_dense_exact(&m[1][1], &c[1], &m[1][0], &c[3], &m[1][2], &c[0]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_add_sub_dense_exact(&m[0][0], &c[3], &m[0][2], &c[0], &m[0][1], &c[1]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_sub_add_dense_exact(&m[3][1], &s[1], &m[3][0], &s[3], &m[3][2], &s[0]),
                inv_det,
            ),
            scale_by_shared_factor(
                mul_add_sub_dense_exact(&m[2][0], &s[3], &m[2][2], &s[0], &m[2][1], &s[1]),
                inv_det,
            ),
        ],
    ]
}

#[inline]
fn invert_matrix4<B: Backend>(matrix: [[Scalar<B>; 4]; 4]) -> BlasResult<[[Scalar<B>; 4]; 4]> {
    if matrix4_is_definitely_dense_for_inverse(&matrix) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-dense-cofactor"
        );
        let (s, c) = if B::FUSE_SIGNED_PRODUCT_SUM {
            matrix4_factors_dense_exact(&matrix)
        } else {
            matrix4_factors(&matrix)
        };
        let det = determinant4_from_factors(&s, &c);
        let inv_det = det.inverse()?;
        if B::FUSE_SIGNED_PRODUCT_SUM {
            return Ok(matrix4_scaled_adjugate_from_factors_dense_exact(
                &matrix, &s, &c, &inv_det,
            ));
        }
        return Ok(matrix4_scaled_adjugate_from_factors(
            &matrix, &s, &c, &inv_det,
        ));
    }
    let facts = matrix4_facts(&matrix);
    if facts.is_identity {
        crate::trace_dispatch!("hyperlattice_matrix", "helper", "invert-matrix4-identity");
        return Ok(matrix);
    }
    if facts.is_diagonal {
        crate::trace_dispatch!("hyperlattice_matrix", "helper", "invert-matrix4-diagonal");
        return invert_matrix4_by_diagonal(&matrix);
    }
    if facts.is_upper_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-upper-triangular"
        );
        return invert_matrix4_by_upper_triangular(&matrix);
    }
    if facts.is_lower_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-lower-triangular"
        );
        return invert_matrix4_by_lower_triangular(&matrix);
    }
    if facts.is_affine {
        crate::trace_dispatch!("hyperlattice_matrix", "helper", "invert-matrix4-affine");
        return invert_matrix4_affine(
            &matrix,
            facts.linear_is_diagonal,
            facts.is_affine_translation,
        );
    }
    // The fixed cofactor formula also wins for 4x4 inverse despite doing more
    // arithmetic than elimination. It creates one shared determinant inverse,
    // while the solve prototype repeatedly normalized pivot rows and regressed
    // both dyadic and decimal-rational benchmark rows.
    let (s, c) = matrix4_factors(&matrix);
    let det = determinant4_from_factors(&s, &c);
    let inv_det = det.inverse()?;
    Ok(matrix4_scaled_adjugate_from_factors(
        &matrix, &s, &c, &inv_det,
    ))
}

#[inline]
fn invert_matrix4_checked<B: Backend>(
    matrix: [[Scalar<B>; 4]; 4],
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    if matrix4_is_definitely_dense_for_inverse(&matrix) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-checked-dense-cofactor"
        );
        let (s, c) = if B::FUSE_SIGNED_PRODUCT_SUM {
            matrix4_factors_dense_exact(&matrix)
        } else {
            matrix4_factors(&matrix)
        };
        let det = determinant4_from_factors(&s, &c);
        require_known_nonzero(&det)?;
        let inv_det = det.inverse()?;
        if B::FUSE_SIGNED_PRODUCT_SUM {
            return Ok(matrix4_scaled_adjugate_from_factors_dense_exact(
                &matrix, &s, &c, &inv_det,
            ));
        }
        return Ok(matrix4_scaled_adjugate_from_factors(
            &matrix, &s, &c, &inv_det,
        ));
    }
    let facts = matrix4_facts(&matrix);
    if facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-checked-identity"
        );
        return Ok(matrix);
    }
    if facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-checked-diagonal"
        );
        return invert_matrix4_by_diagonal_checked(&matrix);
    }
    if facts.is_upper_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-checked-upper-triangular"
        );
        return invert_matrix4_by_upper_triangular_checked(&matrix);
    }
    if facts.is_lower_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-checked-lower-triangular"
        );
        return invert_matrix4_by_lower_triangular_checked(&matrix);
    }
    if facts.is_affine {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-checked-affine"
        );
        return invert_matrix4_affine_checked(
            &matrix,
            facts.linear_is_diagonal,
            facts.is_affine_translation,
        );
    }
    let (s, c) = matrix4_factors(&matrix);
    let det = determinant4_from_factors(&s, &c);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    Ok(matrix4_scaled_adjugate_from_factors(
        &matrix, &s, &c, &inv_det,
    ))
}

#[inline]
fn invert_matrix4_checked_with_abort<B: Backend>(
    matrix: [[Scalar<B>; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Scalar<B>; 4]; 4]> {
    if matrix4_is_definitely_dense_for_inverse(&matrix) {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-checked-with-abort-dense-cofactor"
        );
        let (s, c) = if B::FUSE_SIGNED_PRODUCT_SUM {
            matrix4_factors_dense_exact(&matrix)
        } else {
            matrix4_factors(&matrix)
        };
        let det = determinant4_from_factors(&s, &c);
        let det = with_abort(det, signal);
        require_known_nonzero(&det)?;
        let inv_det = det.inverse()?;
        if B::FUSE_SIGNED_PRODUCT_SUM {
            return Ok(matrix4_scaled_adjugate_from_factors_dense_exact(
                &matrix, &s, &c, &inv_det,
            ));
        }
        return Ok(matrix4_scaled_adjugate_from_factors(
            &matrix, &s, &c, &inv_det,
        ));
    }
    let facts = matrix4_facts(&matrix);
    if facts.is_identity {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-checked-with-abort-identity"
        );
        return Ok(matrix);
    }
    if facts.is_diagonal {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-checked-with-abort-diagonal"
        );
        return invert_matrix4_by_diagonal_checked_with_abort(&matrix, signal);
    }
    if facts.is_upper_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-checked-with-abort-upper-triangular"
        );
        return invert_matrix4_by_upper_triangular_checked_with_abort(&matrix, signal);
    }
    if facts.is_lower_triangular {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-checked-with-abort-lower-triangular"
        );
        return invert_matrix4_by_lower_triangular_checked_with_abort(&matrix, signal);
    }
    if facts.is_affine {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "helper",
            "invert-matrix4-checked-with-abort-affine"
        );
        return invert_matrix4_affine_checked_with_abort(
            &matrix,
            signal,
            facts.linear_is_diagonal,
            facts.is_affine_translation,
        );
    }
    let (s, c) = matrix4_factors(&matrix);
    let det = determinant4_from_factors(&s, &c);
    let det = with_abort(det, signal);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    Ok(matrix4_scaled_adjugate_from_factors(
        &matrix, &s, &c, &inv_det,
    ))
}

#[inline]
fn matrix4_adjugate_from_factors<B: Backend>(
    m: &[[Scalar<B>; 4]; 4],
    s: &[Scalar<B>; 6],
    c: &[Scalar<B>; 6],
) -> [[Scalar<B>; 4]; 4] {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "matrix4-unscaled-adjugate-from-factors"
    );
    // Shared-scale division needs the 4x4 adjugate without multiplying each
    // cofactor by `1/det`. This deliberately duplicates the scaled inverse
    // formula above: refactoring the hot inverse path through an unscaled
    // temporary would add an extra matrix pass and previously made these rows
    // sensitive to code layout. Keep the duplicate only while right-division
    // benchmarks prove that delaying the common scalar is worthwhile.
    [
        [
            mul_add_sub(&m[1][1], &c[5], &m[1][3], &c[3], &m[1][2], &c[4]),
            mul_sub_add(&m[0][2], &c[4], &m[0][1], &c[5], &m[0][3], &c[3]),
            mul_add_sub(&m[3][1], &s[5], &m[3][3], &s[3], &m[3][2], &s[4]),
            mul_sub_add(&m[2][2], &s[4], &m[2][1], &s[5], &m[2][3], &s[3]),
        ],
        [
            mul_sub_add(&m[1][2], &c[2], &m[1][0], &c[5], &m[1][3], &c[1]),
            mul_add_sub(&m[0][0], &c[5], &m[0][3], &c[1], &m[0][2], &c[2]),
            mul_sub_add(&m[3][2], &s[2], &m[3][0], &s[5], &m[3][3], &s[1]),
            mul_add_sub(&m[2][0], &s[5], &m[2][3], &s[1], &m[2][2], &s[2]),
        ],
        [
            mul_add_sub(&m[1][0], &c[4], &m[1][3], &c[0], &m[1][1], &c[2]),
            mul_sub_add(&m[0][1], &c[2], &m[0][0], &c[4], &m[0][3], &c[0]),
            mul_add_sub(&m[3][0], &s[4], &m[3][3], &s[0], &m[3][1], &s[2]),
            mul_sub_add(&m[2][1], &s[2], &m[2][0], &s[4], &m[2][3], &s[0]),
        ],
        [
            mul_sub_add(&m[1][1], &c[1], &m[1][0], &c[3], &m[1][2], &c[0]),
            mul_add_sub(&m[0][0], &c[3], &m[0][2], &c[0], &m[0][1], &c[1]),
            mul_sub_add(&m[3][1], &s[1], &m[3][0], &s[3], &m[3][2], &s[0]),
            mul_add_sub(&m[2][0], &s[3], &m[2][2], &s[0], &m[2][1], &s[1]),
        ],
    ]
}

#[inline(never)]
fn matrix4_adjugate_from_factors_dense_exact<B: Backend>(
    m: &[[Scalar<B>; 4]; 4],
    s: &[Scalar<B>; 6],
    c: &[Scalar<B>; 6],
) -> [[Scalar<B>; 4]; 4] {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "matrix4-unscaled-adjugate-dense-exact"
    );
    [
        [
            mul_add_sub_dense_exact(&m[1][1], &c[5], &m[1][3], &c[3], &m[1][2], &c[4]),
            mul_sub_add_dense_exact(&m[0][2], &c[4], &m[0][1], &c[5], &m[0][3], &c[3]),
            mul_add_sub_dense_exact(&m[3][1], &s[5], &m[3][3], &s[3], &m[3][2], &s[4]),
            mul_sub_add_dense_exact(&m[2][2], &s[4], &m[2][1], &s[5], &m[2][3], &s[3]),
        ],
        [
            mul_sub_add_dense_exact(&m[1][2], &c[2], &m[1][0], &c[5], &m[1][3], &c[1]),
            mul_add_sub_dense_exact(&m[0][0], &c[5], &m[0][3], &c[1], &m[0][2], &c[2]),
            mul_sub_add_dense_exact(&m[3][2], &s[2], &m[3][0], &s[5], &m[3][3], &s[1]),
            mul_add_sub_dense_exact(&m[2][0], &s[5], &m[2][3], &s[1], &m[2][2], &s[2]),
        ],
        [
            mul_add_sub_dense_exact(&m[1][0], &c[4], &m[1][3], &c[0], &m[1][1], &c[2]),
            mul_sub_add_dense_exact(&m[0][1], &c[2], &m[0][0], &c[4], &m[0][3], &c[0]),
            mul_add_sub_dense_exact(&m[3][0], &s[4], &m[3][3], &s[0], &m[3][1], &s[2]),
            mul_sub_add_dense_exact(&m[2][1], &s[2], &m[2][0], &s[4], &m[2][3], &s[0]),
        ],
        [
            mul_sub_add_dense_exact(&m[1][1], &c[1], &m[1][0], &c[3], &m[1][2], &c[0]),
            mul_add_sub_dense_exact(&m[0][0], &c[3], &m[0][2], &c[0], &m[0][1], &c[1]),
            mul_sub_add_dense_exact(&m[3][1], &s[1], &m[3][0], &s[3], &m[3][2], &s[0]),
            mul_add_sub_dense_exact(&m[2][0], &s[3], &m[2][2], &s[0], &m[2][1], &s[1]),
        ],
    ]
}

#[inline(never)]
fn matrix4_adjugate_from_factors_dense_exact_known_rational<B: Backend>(
    m: &[[Scalar<B>; 4]; 4],
    s: &[Scalar<B>; 6],
    c: &[Scalar<B>; 6],
) -> [[Scalar<B>; 4]; 4] {
    crate::trace_dispatch!(
        "hyperlattice_matrix",
        "helper",
        "matrix4-unscaled-adjugate-dense-exact-known-rational"
    );
    [
        [
            mul_add_sub_dense_exact_known_rational(
                &m[1][1], &c[5], &m[1][3], &c[3], &m[1][2], &c[4],
            ),
            mul_sub_add_dense_exact_known_rational(
                &m[0][2], &c[4], &m[0][1], &c[5], &m[0][3], &c[3],
            ),
            mul_add_sub_dense_exact_known_rational(
                &m[3][1], &s[5], &m[3][3], &s[3], &m[3][2], &s[4],
            ),
            mul_sub_add_dense_exact_known_rational(
                &m[2][2], &s[4], &m[2][1], &s[5], &m[2][3], &s[3],
            ),
        ],
        [
            mul_sub_add_dense_exact_known_rational(
                &m[1][2], &c[2], &m[1][0], &c[5], &m[1][3], &c[1],
            ),
            mul_add_sub_dense_exact_known_rational(
                &m[0][0], &c[5], &m[0][3], &c[1], &m[0][2], &c[2],
            ),
            mul_sub_add_dense_exact_known_rational(
                &m[3][2], &s[2], &m[3][0], &s[5], &m[3][3], &s[1],
            ),
            mul_add_sub_dense_exact_known_rational(
                &m[2][0], &s[5], &m[2][3], &s[1], &m[2][2], &s[2],
            ),
        ],
        [
            mul_add_sub_dense_exact_known_rational(
                &m[1][0], &c[4], &m[1][3], &c[0], &m[1][1], &c[2],
            ),
            mul_sub_add_dense_exact_known_rational(
                &m[0][1], &c[2], &m[0][0], &c[4], &m[0][3], &c[0],
            ),
            mul_add_sub_dense_exact_known_rational(
                &m[3][0], &s[4], &m[3][3], &s[0], &m[3][1], &s[2],
            ),
            mul_sub_add_dense_exact_known_rational(
                &m[2][1], &s[2], &m[2][0], &s[4], &m[2][3], &s[0],
            ),
        ],
        [
            mul_sub_add_dense_exact_known_rational(
                &m[1][1], &c[1], &m[1][0], &c[3], &m[1][2], &c[0],
            ),
            mul_add_sub_dense_exact_known_rational(
                &m[0][0], &c[3], &m[0][2], &c[0], &m[0][1], &c[1],
            ),
            mul_sub_add_dense_exact_known_rational(
                &m[3][1], &s[1], &m[3][0], &s[3], &m[3][2], &s[0],
            ),
            mul_add_sub_dense_exact_known_rational(
                &m[2][0], &s[3], &m[2][2], &s[0], &m[2][1], &s[1],
            ),
        ],
    ]
}

macro_rules! impl_matrix {
    (
        $name:ident,
        $vector:ident,
        $n:expr,
        $div_fn:ident,
        $div_ref_fn:ident,
        $power_fn:ident,
        $mul_owned_fn:ident,
        $mul_rhs_ref_fn:ident,
        $mul_ref_fn:ident,
        $div_checked_fn:ident,
        $div_checked_abort_fn:ident
    ) => {
        impl<B: Backend> $name<B> {
            /// Constructs a matrix from row-major entries.
            pub fn new(values: [[Scalar<B>; $n]; $n]) -> Self {
                crate::trace_dispatch!("hyperlattice_matrix", "constructor", "new");
                Self(values)
            }

            /// Returns the zero matrix.
            pub fn zero() -> Self {
                crate::trace_dispatch!("hyperlattice_matrix", "constructor", "zero");
                Self(from_fn(|_| from_fn(|_| Scalar::zero())))
            }

            /// Returns the identity matrix.
            pub fn identity() -> Self {
                crate::trace_dispatch!("hyperlattice_matrix", "constructor", "identity");
                Self(from_fn(|row| {
                    from_fn(|col| {
                        if row == col {
                            Scalar::one()
                        } else {
                            Scalar::zero()
                        }
                    })
                }))
            }

            /// Returns the transpose.
            pub fn transpose(&self) -> Self {
                crate::trace_dispatch!("hyperlattice_matrix", "method", "transpose");
                Self(from_fn(|row| from_fn(|col| self.0[col][row].clone())))
            }

            /// Returns the matrix inverse.
            ///
            /// This is equivalent to [`inverse`](Self::inverse).
            pub fn reciprocal(self) -> BlasResult<Self> {
                crate::trace_dispatch!("hyperlattice_matrix", "method", "reciprocal");
                self.inverse()
            }

            /// Returns the checked matrix inverse.
            ///
            /// This is equivalent to [`inverse_checked`](Self::inverse_checked).
            pub fn reciprocal_checked(self) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!("hyperlattice_matrix", "method", "reciprocal-checked");
                self.inverse_checked()
            }

            /// Raises the matrix to an integer power.
            ///
            /// Negative exponents invert the matrix first.
            pub fn powi(self, exponent: i32) -> BlasResult<Self> {
                crate::trace_dispatch!("hyperlattice_matrix", "method", "powi");
                if exponent == -1 {
                    crate::trace_dispatch!("hyperlattice_matrix", "powi", "negative-one-inverse");
                    return self.inverse();
                }
                // Negative powers deliberately materialize A^-1 before
                // repeated squaring. A delayed-scale prototype using
                // A^-k = adj(A)^k * det(A)^-k looked structurally attractive,
                // but 2026-05 targeted Criterion showed it regressed
                // hyperreal-rational mat3/mat4 powi(-2) by roughly 6%/12%.
                // The larger unscaled cofactors outweighed saving the common
                // determinant scale, so keep the inverse-first schedule.
                let base = if exponent < 0 {
                    self.inverse()?.0
                } else {
                    self.0
                };
                Ok(Self($power_fn(base, exponent.unsigned_abs())))
            }

            /// Raises the matrix to an integer power using checked inversion.
            pub fn powi_checked(self, exponent: i32) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!("hyperlattice_matrix", "method", "powi-checked");
                if exponent == -1 {
                    crate::trace_dispatch!(
                        "hyperlattice_matrix",
                        "powi",
                        "negative-one-inverse-checked"
                    );
                    return self.inverse_checked();
                }
                let base = if exponent < 0 {
                    self.inverse_checked()?.0
                } else {
                    self.0
                };
                Ok(Self($power_fn(base, exponent.unsigned_abs())))
            }

            /// Raises the matrix to an integer power after attaching an abort signal.
            pub fn powi_checked_with_abort(
                self,
                exponent: i32,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!("hyperlattice_matrix", "method", "powi-checked-with-abort");
                if exponent == -1 {
                    crate::trace_dispatch!(
                        "hyperlattice_matrix",
                        "powi",
                        "negative-one-inverse-checked-with-abort"
                    );
                    return self.inverse_checked_with_abort(signal);
                }
                let base = if exponent < 0 {
                    self.inverse_checked_with_abort(signal)?.0
                } else {
                    self.0
                };
                Ok(Self($power_fn(base, exponent.unsigned_abs())))
            }

            /// Divides every entry by `rhs` after rejecting unknown-zero divisors.
            pub fn div_scalar_checked(self, rhs: Scalar<B>) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!("hyperlattice_matrix", "method", "div-scalar-checked");
                require_known_nonzero(&rhs)?;
                let inv_rhs = rhs.inverse()?;
                if B::MOVE_ELEMENTWISE {
                    Ok(Self(
                        self.0
                            .map(|row| row.map(|value| value.mul_cached(&inv_rhs))),
                    ))
                } else {
                    let mut values = self.0;
                    for row in &mut values {
                        for value in row {
                            *value = value.clone().mul_cached(&inv_rhs);
                        }
                    }
                    Ok(Self(values))
                }
            }

            /// Divides every entry by `rhs` after attaching an abort signal.
            pub fn div_scalar_checked_with_abort(
                self,
                rhs: Scalar<B>,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "method",
                    "div-scalar-checked-with-abort"
                );
                let rhs = with_abort(rhs, signal);
                require_known_nonzero(&rhs)?;
                let inv_rhs = rhs.inverse()?;
                if B::MOVE_ELEMENTWISE {
                    Ok(Self(
                        self.0
                            .map(|row| row.map(|value| value.mul_cached(&inv_rhs))),
                    ))
                } else {
                    let mut values = self.0;
                    for row in &mut values {
                        for value in row {
                            *value = value.clone().mul_cached(&inv_rhs);
                        }
                    }
                    Ok(Self(values))
                }
            }

            /// Divides by another matrix using checked inversion of the divisor.
            pub fn div_matrix_checked(self, rhs: Self) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!("hyperlattice_matrix", "method", "div-matrix-checked");
                Ok(Self($div_checked_fn(self.0, rhs.0)?))
            }

            /// Divides by another matrix using abort-aware checked inversion.
            pub fn div_matrix_checked_with_abort(
                self,
                rhs: Self,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "method",
                    "div-matrix-checked-with-abort"
                );
                Ok(Self($div_checked_abort_fn(self.0, rhs.0, signal)?))
            }
        }

        impl<B: Backend> Index<usize> for $name<B> {
            type Output = [Scalar<B>; $n];

            fn index(&self, index: usize) -> &Self::Output {
                &self.0[index]
            }
        }

        impl<B: Backend> IndexMut<usize> for $name<B> {
            fn index_mut(&mut self, index: usize) -> &mut Self::Output {
                &mut self.0[index]
            }
        }

        impl<B: Backend> fmt::Display for $name<B> {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str("[")?;
                for row in 0..$n {
                    if row > 0 {
                        f.write_str(", ")?;
                    }
                    f.write_str("[")?;
                    for col in 0..$n {
                        if col > 0 {
                            f.write_str(", ")?;
                        }
                        if f.alternate() {
                            write!(f, "{:#}", self.0[row][col])?;
                        } else {
                            write!(f, "{}", self.0[row][col])?;
                        }
                    }
                    f.write_str("]")?;
                }
                f.write_str("]")
            }
        }

        impl<B: Backend> Add for $name<B> {
            type Output = Self;

            fn add(self, rhs: Self) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "add-owned-owned");
                if B::MOVE_ELEMENTWISE {
                    Self(map_matrix2(self.0, rhs.0, |lhs, rhs| lhs + rhs))
                } else {
                    Self(from_fn(|row| {
                        from_fn(|col| self.0[row][col].clone() + rhs.0[row][col].clone())
                    }))
                }
            }
        }

        impl<B: Backend> Add<&$name<B>> for $name<B> {
            type Output = Self;

            fn add(self, rhs: &$name<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "add-owned-ref");
                Self(map_matrix_ref(self.0, &rhs.0, Scalar::add_cached))
            }
        }

        impl<B: Backend> Add<$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn add(self, rhs: $name<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "add-ref-owned");
                $name(map_matrix_left_ref(&self.0, rhs.0, |lhs, rhs| lhs + rhs))
            }
        }

        impl<B: Backend> Add<&$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn add(self, rhs: &$name<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "add-ref-ref");
                $name(from_fn(|row| {
                    from_fn(|col| &self.0[row][col] + &rhs.0[row][col])
                }))
            }
        }

        impl<B: Backend> Add<Scalar<B>> for $name<B> {
            type Output = Self;

            fn add(self, rhs: Scalar<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "add-scalar-owned");
                let rhs = &rhs;
                if B::MOVE_ELEMENTWISE {
                    Self(self.0.map(|row| row.map(|value| value.add_cached(rhs))))
                } else {
                    let mut values = self.0;
                    for row in &mut values {
                        for value in row {
                            *value = value.clone().add_cached(rhs);
                        }
                    }
                    Self(values)
                }
            }
        }

        impl<B: Backend> Add<&Scalar<B>> for $name<B> {
            type Output = Self;

            fn add(self, rhs: &Scalar<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "add-scalar-ref");
                Self(self.0.map(|row| row.map(|value| value.add_cached(rhs))))
            }
        }

        impl<B: Backend> Sub for $name<B> {
            type Output = Self;

            fn sub(self, rhs: Self) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "sub-owned-owned");
                if B::MOVE_ELEMENTWISE {
                    Self(map_matrix2(self.0, rhs.0, |lhs, rhs| lhs - rhs))
                } else {
                    Self(from_fn(|row| {
                        from_fn(|col| self.0[row][col].clone() - rhs.0[row][col].clone())
                    }))
                }
            }
        }

        impl<B: Backend> Sub<&$name<B>> for $name<B> {
            type Output = Self;

            fn sub(self, rhs: &$name<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "sub-owned-ref");
                Self(map_matrix_ref(self.0, &rhs.0, Scalar::sub_cached))
            }
        }

        impl<B: Backend> Sub<$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn sub(self, rhs: $name<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "sub-ref-owned");
                $name(map_matrix_left_ref(&self.0, rhs.0, |lhs, rhs| lhs - rhs))
            }
        }

        impl<B: Backend> Sub<&$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn sub(self, rhs: &$name<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "sub-ref-ref");
                $name(from_fn(|row| {
                    from_fn(|col| &self.0[row][col] - &rhs.0[row][col])
                }))
            }
        }

        impl<B: Backend> Sub<Scalar<B>> for $name<B> {
            type Output = Self;

            fn sub(self, rhs: Scalar<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "sub-scalar-owned");
                let rhs = -rhs;
                let rhs = &rhs;
                if B::MOVE_ELEMENTWISE {
                    Self(self.0.map(|row| row.map(|value| value.add_cached(rhs))))
                } else {
                    let mut values = self.0;
                    for row in &mut values {
                        for value in row {
                            *value = value.clone().add_cached(rhs);
                        }
                    }
                    Self(values)
                }
            }
        }

        impl<B: Backend> Sub<&Scalar<B>> for $name<B> {
            type Output = Self;

            fn sub(self, rhs: &Scalar<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "sub-scalar-ref");
                let rhs = -rhs.clone();
                Self(self.0.map(|row| row.map(|value| value.add_cached(&rhs))))
            }
        }

        impl<B: Backend> Neg for $name<B> {
            type Output = Self;

            fn neg(self) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "neg-owned");
                if B::MOVE_ELEMENTWISE {
                    Self(self.0.map(|row| row.map(|value| -value)))
                } else {
                    Self(from_fn(|row| from_fn(|col| -self.0[row][col].clone())))
                }
            }
        }

        impl<B: Backend> Neg for &$name<B> {
            type Output = $name<B>;

            fn neg(self) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "neg-ref");
                $name(from_fn(|row| from_fn(|col| -self.0[row][col].clone())))
            }
        }

        impl<B: Backend> Mul<Scalar<B>> for $name<B> {
            type Output = Self;

            fn mul(self, rhs: Scalar<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "mul-scalar-owned");
                let rhs = &rhs;
                if B::MOVE_ELEMENTWISE {
                    Self(self.0.map(|row| row.map(|value| value.mul_cached(rhs))))
                } else {
                    let mut values = self.0;
                    for row in &mut values {
                        for value in row {
                            *value = value.clone().mul_cached(rhs);
                        }
                    }
                    Self(values)
                }
            }
        }

        impl<B: Backend> Mul<&Scalar<B>> for $name<B> {
            type Output = Self;

            fn mul(self, rhs: &Scalar<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "mul-scalar-ref");
                Self(self.0.map(|row| row.map(|value| value.mul_cached(rhs))))
            }
        }

        impl<B: Backend> Div<Scalar<B>> for $name<B> {
            type Output = BlasResult<Self>;

            fn div(self, rhs: Scalar<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "div-scalar-owned");
                reject_definite_zero(&rhs)?;
                let inv_rhs = rhs.inverse()?;
                if B::MOVE_ELEMENTWISE && $n == 3 {
                    Ok(Self(self.0.map(|row| row.map(|value| &value * &inv_rhs))))
                } else if B::MOVE_ELEMENTWISE {
                    Ok(Self(
                        self.0
                            .map(|row| row.map(|value| value.mul_cached(&inv_rhs))),
                    ))
                } else {
                    let mut values = self.0;
                    for row in &mut values {
                        for value in row {
                            *value = value.clone().mul_cached(&inv_rhs);
                        }
                    }
                    Ok(Self(values))
                }
            }
        }

        impl<B: Backend> Div<&Scalar<B>> for $name<B> {
            type Output = BlasResult<Self>;

            fn div(self, rhs: &Scalar<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "div-scalar-ref");
                reject_definite_zero(rhs)?;
                let inv_rhs = rhs.inverse_ref()?;
                if B::MOVE_ELEMENTWISE && $n == 3 {
                    Ok(Self(self.0.map(|row| row.map(|value| &value * &inv_rhs))))
                } else if B::MOVE_ELEMENTWISE {
                    Ok(Self(
                        self.0
                            .map(|row| row.map(|value| value.mul_cached(&inv_rhs))),
                    ))
                } else {
                    let mut values = self.0;
                    for row in &mut values {
                        for value in row {
                            *value = value.clone().mul_cached(&inv_rhs);
                        }
                    }
                    Ok(Self(values))
                }
            }
        }

        impl<B: Backend> Mul for $name<B> {
            type Output = Self;

            fn mul(self, rhs: Self) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "mul-owned-owned");
                Self($mul_owned_fn(self.0, rhs.0))
            }
        }

        impl<B: Backend> Mul<&$name<B>> for $name<B> {
            type Output = Self;

            fn mul(self, rhs: &$name<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "mul-owned-ref");
                Self($mul_rhs_ref_fn(self.0, &rhs.0))
            }
        }

        impl<B: Backend> Mul<$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn mul(self, rhs: $name<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "mul-ref-owned");
                $name($mul_ref_fn(&self.0, &rhs.0))
            }
        }

        impl<B: Backend> Mul<&$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn mul(self, rhs: &$name<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "mul-ref-ref");
                $name($mul_ref_fn(&self.0, &rhs.0))
            }
        }

        impl<B: Backend> Div for $name<B> {
            type Output = BlasResult<Self>;

            fn div(self, rhs: Self) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "div-owned-owned");
                Ok(Self($div_fn(self.0, rhs.0)?))
            }
        }

        impl<B: Backend> Div<&$name<B>> for $name<B> {
            type Output = BlasResult<Self>;

            fn div(self, rhs: &$name<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "div-owned-ref");
                self / rhs.clone()
            }
        }

        impl<B: Backend> Div<$name<B>> for &$name<B> {
            type Output = BlasResult<$name<B>>;

            fn div(self, rhs: $name<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "div-ref-owned");
                self.clone() / rhs
            }
        }

        impl<B: Backend> Div<&$name<B>> for &$name<B> {
            type Output = BlasResult<$name<B>>;

            fn div(self, rhs: &$name<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "div-ref-ref");
                Ok($name($div_ref_fn(&self.0, &rhs.0)?))
            }
        }

        impl<B: Backend> Mul<$vector<B>> for $name<B> {
            type Output = $vector<B>;

            fn mul(self, rhs: $vector<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "transform-vector-owned-owned");
                $vector(transform_vector_rhs_ref(&self.0, &rhs.0))
            }
        }

        impl<B: Backend> Mul<&$vector<B>> for $name<B> {
            type Output = $vector<B>;

            fn mul(self, rhs: &$vector<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "transform-vector-owned-ref");
                $vector(transform_vector_rhs_ref(&self.0, &rhs.0))
            }
        }

        impl<B: Backend> Mul<$vector<B>> for &$name<B> {
            type Output = $vector<B>;

            fn mul(self, rhs: $vector<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "transform-vector-ref-owned");
                $vector(transform_vector_rhs_ref(&self.0, &rhs.0))
            }
        }

        impl<B: Backend> Mul<&$vector<B>> for &$name<B> {
            type Output = $vector<B>;

            fn mul(self, rhs: &$vector<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "transform-vector-ref-ref");
                $vector(transform_vector_rhs_ref(&self.0, &rhs.0))
            }
        }

        impl<B: Backend> BitXor<i32> for $name<B> {
            type Output = BlasResult<Self>;

            fn bitxor(self, rhs: i32) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_matrix", "op", "bitxor-powi");
                self.powi(rhs)
            }
        }
    };
}

impl_matrix!(
    Matrix3,
    Vector3,
    3,
    right_divide_matrix3,
    right_divide_matrix3_ref,
    matrix_power3,
    multiply_arrays3,
    multiply_arrays3_rhs_ref,
    multiply_arrays3_ref,
    right_divide_matrix3_checked,
    right_divide_matrix3_checked_with_abort
);
impl_matrix!(
    Matrix4,
    Vector4,
    4,
    right_divide_matrix4,
    right_divide_matrix4_ref,
    matrix_power4,
    multiply_arrays4,
    multiply_arrays4_rhs_ref,
    multiply_arrays4_ref,
    right_divide_matrix4_checked,
    right_divide_matrix4_checked_with_abort
);

impl<B: Backend> Matrix3<B> {
    /// Constructs a 3x3 diagonal matrix from known diagonal entries.
    pub fn diagonal(diagonal: [Scalar<B>; 3]) -> Self {
        crate::trace_dispatch!("hyperlattice_matrix", "constructor", "diagonal3");
        let [d0, d1, d2] = diagonal;
        Self([
            [d0, Scalar::zero(), Scalar::zero()],
            [Scalar::zero(), d1, Scalar::zero()],
            [Scalar::zero(), Scalar::zero(), d2],
        ])
    }

    /// Constructs the inverse of a known 3x3 diagonal matrix.
    ///
    /// This opt-in constructor carries the diagonal object fact from the caller
    /// instead of rediscovering it with structural probes inside
    /// [`Matrix3::reciprocal`]. That keeps ordinary inverse/division paths flat
    /// while preserving the exact diagonal solve `D^-1 = diag(1/d_i)` when the
    /// geometry layer already knows the matrix shape. The choice follows Yap's
    /// object-package guidance for exact geometric computation ("Towards Exact
    /// Geometric Computation", 1997) and the diagonal-system specialization in
    /// Golub and Van Loan, *Matrix Computations*.
    pub fn diagonal_inverse(diagonal: [Scalar<B>; 3]) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_matrix", "constructor", "diagonal3-inverse");
        let [d0, d1, d2] = diagonal;
        Ok(Self::diagonal([
            d0.inverse()?,
            d1.inverse()?,
            d2.inverse()?,
        ]))
    }

    /// Divides this matrix on the right by a known 3x3 diagonal matrix.
    ///
    /// Right division by `D = diag(d0,d1,d2)` is column scaling:
    /// `A / D = A * diag(1/d0,1/d1,1/d2)`. This explicit path avoids building
    /// a diagonal inverse matrix and avoids generic matrix multiplication when
    /// a caller already retained the diagonal object fact. Keeping the route
    /// opt-in preserves deterministic performance for ordinary matrix division
    /// while exploiting geometric-object structure as recommended by Yap,
    /// "Towards Exact Geometric Computation", 1997. The algebra is the standard
    /// diagonal linear-system specialization described by Golub and Van Loan,
    /// *Matrix Computations*.
    pub fn div_diagonal(self, diagonal: [Scalar<B>; 3]) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "div-diagonal3");
        let [[a00, a01, a02], [a10, a11, a12], [a20, a21, a22]] = self.0;
        let [d0, d1, d2] = diagonal;
        let inv0 = d0.inverse()?;
        let inv1 = d1.inverse()?;
        let inv2 = d2.inverse()?;
        Ok(Self([
            [
                a00.mul_cached(&inv0),
                a01.mul_cached(&inv1),
                a02.mul_cached(&inv2),
            ],
            [
                a10.mul_cached(&inv0),
                a11.mul_cached(&inv1),
                a12.mul_cached(&inv2),
            ],
            [
                a20.mul_cached(&inv0),
                a21.mul_cached(&inv1),
                a22.mul_cached(&inv2),
            ],
        ]))
    }

    /// Inverts a caller-certified upper-triangular 3x3 matrix.
    ///
    /// This skips the generic structural classifier and enters the fixed-size
    /// triangular substitution kernel directly. Use it only when the object
    /// layer already knows the matrix is upper triangular.
    pub fn upper_triangular_inverse(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "upper-triangular3-inverse");
        Ok(Self(invert_matrix3_upper_triangular(&self.0)?))
    }

    /// Checked variant of [`Matrix3::upper_triangular_inverse`].
    pub fn upper_triangular_inverse_checked(self) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "upper-triangular3-inverse-checked"
        );
        Ok(Self(invert_matrix3_upper_triangular_checked(&self.0)?))
    }

    /// Abort-aware checked variant of [`Matrix3::upper_triangular_inverse`].
    pub fn upper_triangular_inverse_checked_with_abort(
        self,
        signal: &AbortSignal,
    ) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "upper-triangular3-inverse-checked-with-abort"
        );
        Ok(Self(invert_matrix3_upper_triangular_checked_with_abort(
            &self.0, signal,
        )?))
    }

    /// Inverts a caller-certified lower-triangular 3x3 matrix.
    pub fn lower_triangular_inverse(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "lower-triangular3-inverse");
        Ok(Self(invert_matrix3_lower_triangular(&self.0)?))
    }

    /// Checked variant of [`Matrix3::lower_triangular_inverse`].
    pub fn lower_triangular_inverse_checked(self) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "lower-triangular3-inverse-checked"
        );
        Ok(Self(invert_matrix3_lower_triangular_checked(&self.0)?))
    }

    /// Abort-aware checked variant of [`Matrix3::lower_triangular_inverse`].
    pub fn lower_triangular_inverse_checked_with_abort(
        self,
        signal: &AbortSignal,
    ) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "lower-triangular3-inverse-checked-with-abort"
        );
        Ok(Self(invert_matrix3_lower_triangular_checked_with_abort(
            &self.0, signal,
        )?))
    }

    /// Right-divides by a caller-certified upper-triangular 3x3 matrix.
    pub fn div_upper_triangular(self, divisor: Self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "div-upper-triangular3");
        Ok(Self(divide_matrix3_by_upper_triangular(
            self.0, &divisor.0,
        )?))
    }

    /// Checked variant of [`Matrix3::div_upper_triangular`].
    pub fn div_upper_triangular_checked(self, divisor: Self) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "div-upper-triangular3-checked"
        );
        Ok(Self(divide_matrix3_by_upper_triangular_checked(
            self.0, &divisor.0,
        )?))
    }

    /// Abort-aware checked variant of [`Matrix3::div_upper_triangular`].
    pub fn div_upper_triangular_checked_with_abort(
        self,
        divisor: Self,
        signal: &AbortSignal,
    ) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "div-upper-triangular3-checked-with-abort"
        );
        Ok(Self(divide_matrix3_by_upper_triangular_checked_with_abort(
            self.0, &divisor.0, signal,
        )?))
    }

    /// Right-divides by a caller-certified lower-triangular 3x3 matrix.
    pub fn div_lower_triangular(self, divisor: Self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "div-lower-triangular3");
        Ok(Self(divide_matrix3_by_lower_triangular(
            self.0, &divisor.0,
        )?))
    }

    /// Checked variant of [`Matrix3::div_lower_triangular`].
    pub fn div_lower_triangular_checked(self, divisor: Self) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "div-lower-triangular3-checked"
        );
        Ok(Self(divide_matrix3_by_lower_triangular_checked(
            self.0, &divisor.0,
        )?))
    }

    /// Abort-aware checked variant of [`Matrix3::div_lower_triangular`].
    pub fn div_lower_triangular_checked_with_abort(
        self,
        divisor: Self,
        signal: &AbortSignal,
    ) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "div-lower-triangular3-checked-with-abort"
        );
        Ok(Self(divide_matrix3_by_lower_triangular_checked_with_abort(
            self.0, &divisor.0, signal,
        )?))
    }

    /// Divides `self` by a known 3x3 diagonal matrix and applies the result to
    /// a single vector.
    ///
    /// For `D = diag(d0,d1,d2)`, matrix-vector application follows:
    /// `(A / D) * x = A * (D^{-1} x)`. Scaling `x` first by the reciprocal
    /// diagonal then using the normal matrix-vector kernel preserves the exact
    /// structure while avoiding construction of an intermediate matrix.
    ///
    /// This path is an opt-in structural fast path aligned with
    /// "Towards Exact Geometric Computation", 1997 (Yap), and the diagonal
    /// specialization strategy in Golub and Van Loan's *Matrix Computations*.
    pub fn div_diagonal_vector(
        &self,
        diagonal: [Scalar<B>; 3],
        rhs: &Vector3<B>,
    ) -> BlasResult<Vector3<B>> {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "div-diagonal3-vector");
        let [d0, d1, d2] = diagonal;
        let (inv0, inv1, inv2) = if d0 == d1 && d0 == d2 {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "div-diagonal3-vector-uniform-scale"
            );
            let inv = d0.inverse()?;
            (inv.clone(), inv.clone(), inv)
        } else {
            (d0.inverse()?, d1.inverse()?, d2.inverse()?)
        };

        let rhs_div = [
            rhs.0[0].clone().mul_cached(&inv0),
            rhs.0[1].clone().mul_cached(&inv1),
            rhs.0[2].clone().mul_cached(&inv2),
        ];
        let mapped = if B::FUSE_SIGNED_PRODUCT_SUM {
            transform_vector3_rhs_dense_active_ref(&self.0, &rhs_div)
        } else {
            transform_vector3_rhs_ref_cached(&self.0, &rhs_div)
        };
        Ok(Vector3(mapped))
    }

    /// Returns a prepared right-divisor handle for repeated division by this matrix.
    ///
    /// This avoids re-deriving structural facts, cofactors, and determinant
    /// inverses for hot geometric pipelines where the same divisor is reused.
    /// The optimization follows Yap's "Towards Exact Geometric Computation", 1997,
    /// which advises moving expensive object-level preprocessing to stable object
    /// boundaries.
    pub fn prepare_right_divisor(&self) -> PreparedRightDivisor3<'_, B> {
        PreparedRightDivisor3::new(self)
    }

    /// Divides this matrix by a prepared right divisor.
    ///
    /// This is the same exact result as `self / divisor` but keeps repeated right-side
    /// preprocessing materialized in `divisor`, matching the object-level cache
    /// strategy in exact geometry engines.
    pub fn div_matrix_with_prepared(
        self,
        divisor: &mut PreparedRightDivisor3<'_, B>,
    ) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "div-matrix-with-prepared");
        Ok(Self(divisor.divide(self.0)?))
    }

    /// Divides by a prepared right divisor with checked determinant validation.
    ///
    /// The divisor cache is reused, but the known-nonzero requirement is still
    /// enforced at every call site before any reciprocal of the cached determinant.
    pub fn div_matrix_checked_with_prepared(
        self,
        divisor: &mut PreparedRightDivisor3<'_, B>,
    ) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "div-matrix-checked-with-prepared"
        );
        Ok(Self(divisor.divide_checked(self.0)?))
    }

    /// Divides by a prepared right divisor with abort-aware checked semantics.
    pub fn div_matrix_checked_with_prepared_with_abort(
        self,
        divisor: &mut PreparedRightDivisor3<'_, B>,
        signal: &AbortSignal,
    ) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "div-matrix-checked-with-prepared-with-abort"
        );
        Ok(Self(divisor.divide_checked_with_abort(self.0, signal)?))
    }

    /// Constructs a scalar multiple of the 3x3 identity matrix.
    pub fn uniform_scale(scale: Scalar<B>) -> Self {
        crate::trace_dispatch!("hyperlattice_matrix", "constructor", "uniform-scale3");
        Self([
            [scale.clone(), Scalar::zero(), Scalar::zero()],
            [Scalar::zero(), scale.clone(), Scalar::zero()],
            [Scalar::zero(), Scalar::zero(), scale],
        ])
    }

    /// Constructs the inverse of a known scalar multiple of the 3x3 identity.
    ///
    /// This mirrors [`Matrix4::uniform_scale_inverse`] for 2D homogeneous
    /// geometry. It is intentionally explicit: previous hidden uniform-scale
    /// detection regressed adjacent diagonal reciprocal paths because equality
    /// checks taxed every diagonal matrix. When a caller already knows `A = sI`,
    /// one scalar inverse and two clones are sufficient. This follows Yap's
    /// object-layer specialization principle ("Towards Exact Geometric
    /// Computation", 1997) and the diagonal solve specialization in Golub and
    /// Van Loan, *Matrix Computations*.
    pub fn uniform_scale_inverse(scale: Scalar<B>) -> BlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "constructor",
            "uniform-scale3-inverse"
        );
        let inv = scale.inverse()?;
        Ok(Self::uniform_scale(inv))
    }

    /// Transforms all vectors in `rhs` using the same matrix.
    ///
    /// This is a convenience batch helper for repeated transformations with
    /// shared matrix state in caller-owned loops. If the same matrix is reused
    /// across multiple batches, prefer [`Matrix3::transform_vec3_handle`]:
    /// targeted hyperreal sentinels show the prebuilt handle avoids repeated
    /// structural fact construction in dense workloads while preserving the
    /// same arithmetic path.
    pub fn transform_vec3_batch(&self, rhs: &[Vector3<B>]) -> Vec<Vector3<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "transform-vector-vec3-batch"
        );
        self.transform_vec3_handle().transform_vector_batch(rhs)
    }

    /// Returns a lightweight transform handle for a shared matrix.
    ///
    /// The handle retains matrix structural facts once and reuses them across
    /// single-vector, deferred-vector, and batch transforms. This follows the
    /// same "classify before arithmetic" strategy used by exact geometric
    /// computation; see Yap, "Towards Exact Geometric Computation", 1997.
    pub fn transform_vec3_handle(&self) -> TransformedMatrix3<'_, B> {
        TransformedMatrix3::new(self)
    }

    /// Returns a handle for a single vector transformation under this matrix.
    ///
    /// Reusing the matrix handle keeps the zero-translation and direction facts
    /// in the same precomputed form used by batch transforms.
    pub fn transform_vec3_with<'a>(&'a self, rhs: &'a Vector3<B>) -> TransformedVector3<'a, B> {
        self.transform_vec3_handle().vector(rhs)
    }

    /// Returns the matrix inverse using the adjugate and determinant.
    ///
    /// The ordinary path rejects a definite-zero determinant and otherwise
    /// propagates scalar arithmetic errors from the selected backend.
    pub fn inverse(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "matrix3-inverse");
        Ok(Self(invert_matrix3(self.0)?))
    }

    /// Returns the matrix inverse after rejecting unknown-zero determinants.
    pub fn inverse_checked(self) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "matrix3-inverse-checked");
        Ok(Self(invert_matrix3_checked(self.0)?))
    }

    /// Returns the checked matrix inverse after attaching an abort signal.
    pub fn inverse_checked_with_abort(self, signal: &AbortSignal) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "matrix3-inverse-checked-with-abort"
        );
        Ok(Self(invert_matrix3_checked_with_abort(self.0, signal)?))
    }

    /// Returns the determinant.
    pub fn determinant(&self) -> Scalar<B> {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "matrix3-determinant");
        determinant3(&self.0)
    }
}

/// A signed 4D basis axis used by caller-certified signed-permutation matrices.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SignedAxis4 {
    /// Positive X axis.
    PosX,
    /// Negative X axis.
    NegX,
    /// Positive Y axis.
    PosY,
    /// Negative Y axis.
    NegY,
    /// Positive Z axis.
    PosZ,
    /// Negative Z axis.
    NegZ,
    /// Positive W axis.
    PosW,
    /// Negative W axis.
    NegW,
}

impl SignedAxis4 {
    #[inline]
    fn index(self) -> usize {
        match self {
            Self::PosX | Self::NegX => 0,
            Self::PosY | Self::NegY => 1,
            Self::PosZ | Self::NegZ => 2,
            Self::PosW | Self::NegW => 3,
        }
    }

    #[inline]
    fn is_negative(self) -> bool {
        matches!(self, Self::NegX | Self::NegY | Self::NegZ | Self::NegW)
    }
}

#[inline]
fn signed_axis4_scalar<B: Backend>(axis: SignedAxis4) -> Scalar<B> {
    if axis.is_negative() {
        -Scalar::one()
    } else {
        Scalar::one()
    }
}

#[inline]
fn signed_axis4_apply<B: Backend>(value: Scalar<B>, axis: SignedAxis4) -> Scalar<B> {
    if axis.is_negative() { -value } else { value }
}

impl<B: Backend> Matrix4<B> {
    /// Constructs a 4x4 affine translation matrix from known x/y/z offsets.
    pub fn affine_translation(translation: [Scalar<B>; 3]) -> Self {
        crate::trace_dispatch!("hyperlattice_matrix", "constructor", "affine-translation");
        let [tx, ty, tz] = translation;
        Self([
            [Scalar::one(), Scalar::zero(), Scalar::zero(), tx],
            [Scalar::zero(), Scalar::one(), Scalar::zero(), ty],
            [Scalar::zero(), Scalar::zero(), Scalar::one(), tz],
            [
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
                Scalar::one(),
            ],
        ])
    }

    /// Constructs the inverse of a caller-certified affine translation.
    ///
    /// Translation inverse is exact negation of the offset vector. Keeping this
    /// as a known-object API avoids the generic affine fact scan and does not
    /// enter determinant/cofactor arithmetic.
    pub fn affine_translation_inverse(translation: [Scalar<B>; 3]) -> Self {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "constructor",
            "affine-translation-inverse"
        );
        let [tx, ty, tz] = translation;
        Self::affine_translation([-tx, -ty, -tz])
    }

    /// Right-divides this matrix by a caller-certified affine translation.
    ///
    /// For a translation `T(t)`, `A / T(t) = A * T(-t)`, so only the
    /// homogeneous translation column changes. This exposes the same
    /// object-level fast path used by generic affine dispatch without making
    /// callers rediscover translation structure numerically.
    pub fn div_affine_translation(self, translation: [Scalar<B>; 3]) -> Self {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "div-affine-translation4");
        let [row0, row1, row2, row3] = self.0;
        let [tx, ty, tz] = translation;
        let terms = [&tx, &ty, &tz];
        let t0 = affine_translation_column_subtract_update(&row0, terms);
        let t1 = affine_translation_column_subtract_update(&row1, terms);
        let t2 = affine_translation_column_subtract_update(&row2, terms);
        let t3 = affine_translation_column_subtract_update(&row3, terms);
        let [a00, a01, a02, _] = row0;
        let [a10, a11, a12, _] = row1;
        let [a20, a21, a22, _] = row2;
        let [a30, a31, a32, _] = row3;
        Self([
            [a00, a01, a02, t0],
            [a10, a11, a12, t1],
            [a20, a21, a22, t2],
            [a30, a31, a32, t3],
        ])
    }

    /// Constructs a caller-certified affine orthonormal transform.
    ///
    /// The caller supplies the object fact that the 3x3 linear block is
    /// orthonormal. This constructor does not validate that fact; it preserves
    /// it for known-object inverse and right-division paths.
    pub fn affine_orthonormal(linear: [[Scalar<B>; 3]; 3], translation: [Scalar<B>; 3]) -> Self {
        crate::trace_dispatch!("hyperlattice_matrix", "constructor", "affine-orthonormal");
        let [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]] = linear;
        let [tx, ty, tz] = translation;
        Self([
            [r00, r01, r02, tx],
            [r10, r11, r12, ty],
            [r20, r21, r22, tz],
            [
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
                Scalar::one(),
            ],
        ])
    }

    /// Constructs the inverse of a caller-certified affine orthonormal transform.
    ///
    /// For `M = [R t; 0 1]` with orthonormal `R`, `M^-1 = [R^T -R^T t; 0 1]`.
    /// This bypasses generic affine inversion and avoids determinant/cofactor
    /// arithmetic for rigid transform stacks.
    pub fn affine_orthonormal_inverse(
        linear: [[Scalar<B>; 3]; 3],
        translation: [Scalar<B>; 3],
    ) -> Self {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "constructor",
            "affine-orthonormal-inverse"
        );
        let [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]] = linear;
        let [tx, ty, tz] = translation;
        let it0 = Scalar::zero() - affine_translation_dot3([&r00, &r10, &r20], [&tx, &ty, &tz]);
        let it1 = Scalar::zero() - affine_translation_dot3([&r01, &r11, &r21], [&tx, &ty, &tz]);
        let it2 = Scalar::zero() - affine_translation_dot3([&r02, &r12, &r22], [&tx, &ty, &tz]);
        Self([
            [r00, r10, r20, it0],
            [r01, r11, r21, it1],
            [r02, r12, r22, it2],
            [
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
                Scalar::one(),
            ],
        ])
    }

    /// Right-divides this matrix by a caller-certified affine orthonormal transform.
    pub fn div_affine_orthonormal(
        self,
        linear: [[Scalar<B>; 3]; 3],
        translation: [Scalar<B>; 3],
    ) -> Self {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "div-affine-orthonormal4");
        let [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]] = linear;
        let [tx, ty, tz] = translation;
        let it0 = Scalar::zero() - affine_translation_dot3([&r00, &r10, &r20], [&tx, &ty, &tz]);
        let it1 = Scalar::zero() - affine_translation_dot3([&r01, &r11, &r21], [&tx, &ty, &tz]);
        let it2 = Scalar::zero() - affine_translation_dot3([&r02, &r12, &r22], [&tx, &ty, &tz]);
        let inv_translation = [&it0, &it1, &it2];
        let result = self.0.map(|row| {
            let [a0, a1, a2, a3] = row;
            let c0 = Scalar::active_linear_combination3([&a0, &a1, &a2], [&r00, &r01, &r02]);
            let c1 = Scalar::active_linear_combination3([&a0, &a1, &a2], [&r10, &r11, &r12]);
            let c2 = Scalar::active_linear_combination3([&a0, &a1, &a2], [&r20, &r21, &r22]);
            let c3 = affine_translation_dot3([&a0, &a1, &a2], inv_translation) + a3;
            [c0, c1, c2, c3]
        });
        Self(result)
    }

    /// Constructs a caller-certified signed-permutation matrix.
    ///
    /// Each row names the signed input axis selected by that output row. This
    /// does not validate uniqueness; callers use it when the construction
    /// provenance already proves signed-permutation structure.
    pub fn signed_permutation(rows: [SignedAxis4; 4]) -> Self {
        crate::trace_dispatch!("hyperlattice_matrix", "constructor", "signed-permutation4");
        let [r0, r1, r2, r3] = rows;
        let mut matrix = [
            [
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
            ],
            [
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
            ],
            [
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
            ],
            [
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
            ],
        ];
        matrix[0][r0.index()] = signed_axis4_scalar(r0);
        matrix[1][r1.index()] = signed_axis4_scalar(r1);
        matrix[2][r2.index()] = signed_axis4_scalar(r2);
        matrix[3][r3.index()] = signed_axis4_scalar(r3);
        Self(matrix)
    }

    /// Constructs the inverse of a caller-certified signed-permutation matrix.
    pub fn signed_permutation_inverse(rows: [SignedAxis4; 4]) -> Self {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "constructor",
            "signed-permutation4-inverse"
        );
        let [r0, r1, r2, r3] = rows;
        let mut matrix = [
            [
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
            ],
            [
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
            ],
            [
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
            ],
            [
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
            ],
        ];
        matrix[r0.index()][0] = signed_axis4_scalar(r0);
        matrix[r1.index()][1] = signed_axis4_scalar(r1);
        matrix[r2.index()][2] = signed_axis4_scalar(r2);
        matrix[r3.index()][3] = signed_axis4_scalar(r3);
        Self(matrix)
    }

    /// Right-divides this matrix by a caller-certified signed-permutation matrix.
    pub fn div_signed_permutation(self, rows: [SignedAxis4; 4]) -> Self {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "div-signed-permutation4");
        let [r0, r1, r2, r3] = rows;
        Self(self.0.map(|row| {
            [
                signed_axis4_apply(row[r0.index()].clone(), r0),
                signed_axis4_apply(row[r1.index()].clone(), r1),
                signed_axis4_apply(row[r2.index()].clone(), r2),
                signed_axis4_apply(row[r3.index()].clone(), r3),
            ]
        }))
    }

    /// Applies a caller-certified signed-permutation transform to a vector.
    pub fn transform_signed_permutation_vector(
        rows: [SignedAxis4; 4],
        rhs: &Vector4<B>,
    ) -> Vector4<B> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "transform-signed-permutation4-vector"
        );
        let [r0, r1, r2, r3] = rows;
        Vector4([
            signed_axis4_apply(rhs.0[r0.index()].clone(), r0),
            signed_axis4_apply(rhs.0[r1.index()].clone(), r1),
            signed_axis4_apply(rhs.0[r2.index()].clone(), r2),
            signed_axis4_apply(rhs.0[r3.index()].clone(), r3),
        ])
    }

    /// Applies a caller-certified signed-permutation transform to a vector batch.
    pub fn transform_signed_permutation_batch(
        rows: [SignedAxis4; 4],
        rhs: &[Vector4<B>],
    ) -> Vec<Vector4<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "transform-signed-permutation4-batch"
        );
        let [r0, r1, r2, r3] = rows;
        rhs.iter()
            .map(|vector| {
                Vector4([
                    signed_axis4_apply(vector.0[r0.index()].clone(), r0),
                    signed_axis4_apply(vector.0[r1.index()].clone(), r1),
                    signed_axis4_apply(vector.0[r2.index()].clone(), r2),
                    signed_axis4_apply(vector.0[r3.index()].clone(), r3),
                ])
            })
            .collect()
    }

    /// Constructs a 4x4 diagonal matrix from known diagonal entries.
    pub fn diagonal(diagonal: [Scalar<B>; 4]) -> Self {
        crate::trace_dispatch!("hyperlattice_matrix", "constructor", "diagonal");
        let [d0, d1, d2, d3] = diagonal;
        Self([
            [d0, Scalar::zero(), Scalar::zero(), Scalar::zero()],
            [Scalar::zero(), d1, Scalar::zero(), Scalar::zero()],
            [Scalar::zero(), Scalar::zero(), d2, Scalar::zero()],
            [Scalar::zero(), Scalar::zero(), Scalar::zero(), d3],
        ])
    }

    /// Constructs the inverse of a known 4x4 diagonal matrix.
    ///
    /// Keep this as an explicit known-structure API rather than another hidden
    /// [`Matrix4::reciprocal`] branch. Prior diagonal/uniform-scale experiments
    /// showed that adding dynamic probes to the general inverse path made
    /// adjacent cases less deterministic. When callers retain the object-level
    /// fact that `D` is diagonal, the exact inverse is just four independent
    /// scalar reciprocals and certified off-diagonal zeros. This mirrors Yap's
    /// recommendation to exploit geometric-object structure before arithmetic
    /// ("Towards Exact Geometric Computation", 1997) and the diagonal solve
    /// treatment in Golub and Van Loan, *Matrix Computations*.
    pub fn diagonal_inverse(diagonal: [Scalar<B>; 4]) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_matrix", "constructor", "diagonal-inverse");
        let [d0, d1, d2, d3] = diagonal;
        Ok(Self::diagonal([
            d0.inverse()?,
            d1.inverse()?,
            d2.inverse()?,
            d3.inverse()?,
        ]))
    }

    /// Divides this matrix on the right by a known 4x4 diagonal matrix.
    ///
    /// For `D = diag(d0,d1,d2,d3)`, right division scales each column of `A` by
    /// the matching reciprocal. This is deliberately separate from generic
    /// [`Matrix4::div_matrix_checked`] and `/` dispatch: previous dynamic
    /// structure-detection experiments showed probe costs can make related
    /// division paths less flat. When a higher geometry layer already knows the
    /// divisor is diagonal, this path uses four scalar reciprocals and sixteen
    /// cached multiplies with no determinant/cofactor work. This follows Yap's
    /// object-level exact geometric computation guidance ("Towards Exact
    /// Geometric Computation", 1997) and the diagonal solve specialization in
    /// Golub and Van Loan, *Matrix Computations*.
    pub fn div_diagonal(self, diagonal: [Scalar<B>; 4]) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "div-diagonal");
        let [
            [a00, a01, a02, a03],
            [a10, a11, a12, a13],
            [a20, a21, a22, a23],
            [a30, a31, a32, a33],
        ] = self.0;
        let [d0, d1, d2, d3] = diagonal;
        let inv0 = d0.inverse()?;
        let inv1 = d1.inverse()?;
        let inv2 = d2.inverse()?;
        let inv3 = d3.inverse()?;
        Ok(Self([
            [
                a00.mul_cached(&inv0),
                a01.mul_cached(&inv1),
                a02.mul_cached(&inv2),
                a03.mul_cached(&inv3),
            ],
            [
                a10.mul_cached(&inv0),
                a11.mul_cached(&inv1),
                a12.mul_cached(&inv2),
                a13.mul_cached(&inv3),
            ],
            [
                a20.mul_cached(&inv0),
                a21.mul_cached(&inv1),
                a22.mul_cached(&inv2),
                a23.mul_cached(&inv3),
            ],
            [
                a30.mul_cached(&inv0),
                a31.mul_cached(&inv1),
                a32.mul_cached(&inv2),
                a33.mul_cached(&inv3),
            ],
        ]))
    }

    /// Inverts a caller-certified upper-triangular 4x4 matrix.
    pub fn upper_triangular_inverse(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "upper-triangular4-inverse");
        Ok(Self(invert_matrix4_by_upper_triangular(&self.0)?))
    }

    /// Checked variant of [`Matrix4::upper_triangular_inverse`].
    pub fn upper_triangular_inverse_checked(self) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "upper-triangular4-inverse-checked"
        );
        Ok(Self(invert_matrix4_by_upper_triangular_checked(&self.0)?))
    }

    /// Abort-aware checked variant of [`Matrix4::upper_triangular_inverse`].
    pub fn upper_triangular_inverse_checked_with_abort(
        self,
        signal: &AbortSignal,
    ) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "upper-triangular4-inverse-checked-with-abort"
        );
        Ok(Self(invert_matrix4_by_upper_triangular_checked_with_abort(
            &self.0, signal,
        )?))
    }

    /// Inverts a caller-certified lower-triangular 4x4 matrix.
    pub fn lower_triangular_inverse(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "lower-triangular4-inverse");
        Ok(Self(invert_matrix4_by_lower_triangular(&self.0)?))
    }

    /// Checked variant of [`Matrix4::lower_triangular_inverse`].
    pub fn lower_triangular_inverse_checked(self) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "lower-triangular4-inverse-checked"
        );
        Ok(Self(invert_matrix4_by_lower_triangular_checked(&self.0)?))
    }

    /// Abort-aware checked variant of [`Matrix4::lower_triangular_inverse`].
    pub fn lower_triangular_inverse_checked_with_abort(
        self,
        signal: &AbortSignal,
    ) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "lower-triangular4-inverse-checked-with-abort"
        );
        Ok(Self(invert_matrix4_by_lower_triangular_checked_with_abort(
            &self.0, signal,
        )?))
    }

    /// Right-divides by a caller-certified upper-triangular 4x4 matrix.
    pub fn div_upper_triangular(self, divisor: Self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "div-upper-triangular4");
        Ok(Self(divide_matrix4_by_upper_triangular(
            self.0, &divisor.0,
        )?))
    }

    /// Checked variant of [`Matrix4::div_upper_triangular`].
    pub fn div_upper_triangular_checked(self, divisor: Self) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "div-upper-triangular4-checked"
        );
        Ok(Self(divide_matrix4_by_upper_triangular_checked(
            self.0, &divisor.0,
        )?))
    }

    /// Abort-aware checked variant of [`Matrix4::div_upper_triangular`].
    pub fn div_upper_triangular_checked_with_abort(
        self,
        divisor: Self,
        signal: &AbortSignal,
    ) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "div-upper-triangular4-checked-with-abort"
        );
        Ok(Self(divide_matrix4_by_upper_triangular_checked_with_abort(
            self.0, &divisor.0, signal,
        )?))
    }

    /// Right-divides by a caller-certified lower-triangular 4x4 matrix.
    pub fn div_lower_triangular(self, divisor: Self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "div-lower-triangular4");
        Ok(Self(divide_matrix4_by_lower_triangular(
            self.0, &divisor.0,
        )?))
    }

    /// Checked variant of [`Matrix4::div_lower_triangular`].
    pub fn div_lower_triangular_checked(self, divisor: Self) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "div-lower-triangular4-checked"
        );
        Ok(Self(divide_matrix4_by_lower_triangular_checked(
            self.0, &divisor.0,
        )?))
    }

    /// Abort-aware checked variant of [`Matrix4::div_lower_triangular`].
    pub fn div_lower_triangular_checked_with_abort(
        self,
        divisor: Self,
        signal: &AbortSignal,
    ) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "div-lower-triangular4-checked-with-abort"
        );
        Ok(Self(divide_matrix4_by_lower_triangular_checked_with_abort(
            self.0, &divisor.0, signal,
        )?))
    }

    /// Divides `self` by a known 4x4 diagonal matrix and applies the result to
    /// a single homogeneous vector.
    ///
    /// Using `D = diag(d0,d1,d2,d3)`, the vector multiply obeys
    /// `(A / D) * x = A * (D^{-1} x)`. Pre-scaling `x` by reciprocal
    /// diagonal factors is substantially cheaper than materializing `A / D`
    /// before the transform and keeps homogeneous direction/point structure in
    /// the matrix-vector helper where one existing structural branch can still
    /// run.
    ///
    /// The same geometric-structure rationale from Yap's "Towards Exact
    /// Geometric Computation" (1997) applies here: retain object facts, defer
    /// expensive algebra, and reduce to a cheaper exact kernel when the divisor
    /// structure is known. See Golub and Van Loan, *Matrix Computations* for
    /// the diagonal solve perspective.
    pub fn div_diagonal_vector(
        &self,
        diagonal: [Scalar<B>; 4],
        rhs: &Vector4<B>,
    ) -> BlasResult<Vector4<B>> {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "div-diagonal4-vector");
        let [d0, d1, d2, d3] = diagonal;
        let vector_facts = rhs.geometric_facts();
        if matches!(vector_facts.homogeneous, Vector4HomogeneousKind::Direction) {
            // Direction vectors are guaranteed `w == 0`; avoiding `d3` work keeps
            // this branch aligned with specialized direction kernels and avoids
            // unnecessary reciprocal work when only three linear scales are ever used.
            // This follows Yap's geometric-object split between points and
            // directions in "Towards Exact Geometric Computation", 1997.
            let (inv0, inv1, inv2) = if d0 == d1 && d0 == d2 {
                crate::trace_dispatch!(
                    "hyperlattice_matrix",
                    "helper",
                    "div-diagonal4-vector-direction-uniform-scale"
                );
                let inv = d0.inverse()?;
                (inv.clone(), inv.clone(), inv)
            } else {
                (d0.inverse()?, d1.inverse()?, d2.inverse()?)
            };
            let rhs_div = [
                rhs.0[0].clone().mul_cached(&inv0),
                rhs.0[1].clone().mul_cached(&inv1),
                rhs.0[2].clone().mul_cached(&inv2),
                Scalar::zero(),
            ];
            return Ok(Vector4(transform_vector4_rhs_direction_ref_cached(
                &self.0,
                &rhs_div,
                matrix4_direction_linear_is_diagonal(&self.0),
            )));
        }

        let linear_uniform_scale = d0 == d1 && d0 == d2;
        let (inv0, inv1, inv2) = if linear_uniform_scale {
            crate::trace_dispatch!(
                "hyperlattice_matrix",
                "helper",
                "div-diagonal4-vector-linear-uniform-scale"
            );
            let inv = d0.inverse()?;
            (inv.clone(), inv.clone(), inv)
        } else {
            (d0.inverse()?, d1.inverse()?, d2.inverse()?)
        };
        // Keep the common affine case `d3 == 1` from paying a needless
        // reciprocal and downstream one-multiplies. Structural affine factors
        // are exact where supplied, so this branch is safe and preserves
        // exact symbolic structure for downstream transforms.
        let inv3_is_one = d3.definitely_one();
        let inv3 = if inv3_is_one {
            Scalar::one()
        } else {
            d3.inverse()?
        };
        let rhs_div_3_scale = if inv3_is_one {
            rhs.0[3].clone()
        } else if rhs.0[3].definitely_one() {
            inv3.clone()
        } else {
            rhs.0[3].clone().mul_cached(&inv3)
        };

        let rhs_div_x = rhs.0[0].clone().mul_cached(&inv0);
        let rhs_div_y = rhs.0[1].clone().mul_cached(&inv1);
        let rhs_div_z = rhs.0[2].clone().mul_cached(&inv2);

        match vector_facts.homogeneous {
            Vector4HomogeneousKind::Point => {
                let translation_is_zero = [
                    self.0[0][3].definitely_zero(),
                    self.0[1][3].definitely_zero(),
                    self.0[2][3].definitely_zero(),
                    self.0[3][3].definitely_zero(),
                ];
                let all_translation_zero = translation_is_zero.iter().all(|value| *value);
                let all_translation_nonzero = translation_is_zero.iter().all(|value| !*value);
                // Preserve known point structure and avoid generic
                // point/affine ambiguity by keeping `w` as a retained factor.
                // After pre-scaling, the point transform is:
                // `(A / D) * p = A * (D^{-1} p)` with `p.w' = p.w * d3^{-1}`.
                // This avoids rebuilding a full four-term dot and keeps the
                // special-point row scheduling aligned with projective geometry
                // routines (Yap, "Towards Exact Geometric Computation", 1997).
                let rhs_div = [rhs_div_x, rhs_div_y, rhs_div_z, rhs_div_3_scale];
                return Ok(Vector4(
                    transform_vector4_rhs_point_with_scaled_w_ref_cached(
                        &self.0,
                        &rhs_div,
                        &translation_is_zero,
                        all_translation_zero,
                        all_translation_nonzero,
                        inv3_is_one,
                        &inv3,
                    ),
                ));
            }
            Vector4HomogeneousKind::Direction => {
                let rhs_div = [rhs_div_x, rhs_div_y, rhs_div_z, Scalar::zero()];
                let direction_is_diagonal = matrix4_direction_linear_is_diagonal(&self.0);
                return Ok(Vector4(transform_vector4_rhs_direction_ref_cached(
                    &self.0,
                    &rhs_div,
                    direction_is_diagonal,
                )));
            }
            Vector4HomogeneousKind::Unknown => {
                let matrix_facts = matrix4_facts(&self.0);
                let translation_is_zero = [
                    matrix_facts.translation_xyz_zero[0],
                    matrix_facts.translation_xyz_zero[1],
                    matrix_facts.translation_xyz_zero[2],
                    self.0[3][3].definitely_zero(),
                ];
                let all_translation_zero = translation_is_zero.iter().all(|value| *value);
                let all_translation_nonzero = translation_is_zero.iter().all(|value| !*value);
                let rhs_div = [rhs_div_x, rhs_div_y, rhs_div_z, rhs_div_3_scale];
                return Ok(Vector4(transform_vector4_rhs_ref_with_facts(
                    &self.0,
                    &rhs_div,
                    &translation_is_zero,
                    all_translation_zero,
                    all_translation_nonzero,
                    matrix_facts.direction_linear_is_diagonal,
                    Some(matrix_facts),
                    vector_facts,
                )));
            }
        };
    }

    /// Divides a matrix by a known 4x4 diagonal divisor and applies the result
    /// to a direction vector.
    ///
    /// Direction vectors have `w == 0`, so the fourth diagonal divisor entry is
    /// provably irrelevant. Avoiding its inversion and W-scaling is an exact
    /// structural optimization that follows Yap's geometric-object viewpoint in
    /// "Towards Exact Geometric Computation" (1997): preserve and exploit
    /// projective-direction facts so arithmetic follows the minimal necessary
    /// path.
    #[inline]
    pub fn div_diagonal_direction_vector(
        &self,
        diagonal: [Scalar<B>; 4],
        rhs: &Vector4<B>,
    ) -> BlasResult<Vector4<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "div-diagonal4-vector-direction-only"
        );
        let [d0, d1, d2, _d3] = diagonal;
        let inv0 = d0.inverse()?;
        let inv1 = d1.inverse()?;
        let inv2 = d2.inverse()?;
        let rhs_div = [
            rhs.0[0].clone().mul_cached(&inv0),
            rhs.0[1].clone().mul_cached(&inv1),
            rhs.0[2].clone().mul_cached(&inv2),
            Scalar::zero(),
        ];
        Ok(Vector4(transform_vector4_rhs_direction_ref_cached(
            &self.0,
            &rhs_div,
            matrix4_direction_linear_is_diagonal(&self.0),
        )))
    }

    /// Returns a prepared right-divisor handle for repeated division by this matrix.
    ///
    /// This avoids re-deriving structural facts, cofactors, and determinant
    /// inverses for hot geometric pipelines where the same divisor is reused.
    /// The optimization follows Yap's "Towards Exact Geometric Computation", 1997,
    /// which advises moving expensive object-level preprocessing to stable object
    /// boundaries.
    pub fn prepare_right_divisor(&self) -> PreparedRightDivisor4<'_, B> {
        PreparedRightDivisor4::new(self)
    }

    /// Divides this matrix by a prepared right divisor.
    ///
    /// This is the same exact result as `self / divisor` but keeps repeated right-side
    /// preprocessing materialized in `divisor`, matching the object-level cache
    /// strategy in exact geometry engines.
    pub fn div_matrix_with_prepared(
        self,
        divisor: &mut PreparedRightDivisor4<'_, B>,
    ) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "div-matrix-with-prepared");
        Ok(Self(divisor.divide(self.0)?))
    }

    /// Divides this caller-certified exact-rational matrix by a prepared right
    /// divisor.
    ///
    /// Use this only when the caller already retains the exact-rational fact for
    /// `self`. It exposes the prepared/shared-adjugate fast path without paying a
    /// second 16-lane exact-rational-kind scan in the hot loop.
    pub fn div_exact_rational_matrix_with_prepared(
        self,
        divisor: &mut PreparedRightDivisor4<'_, B>,
    ) -> BlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "div-exact-rational-matrix-with-prepared"
        );
        Ok(Self(divisor.divide_exact_rational_left(self.0)?))
    }

    /// Divides by a prepared right divisor with checked determinant validation.
    ///
    /// The divisor cache is reused, but the known-nonzero requirement is still
    /// enforced at every call site before any reciprocal of the cached determinant.
    pub fn div_matrix_checked_with_prepared(
        self,
        divisor: &mut PreparedRightDivisor4<'_, B>,
    ) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "div-matrix-checked-with-prepared"
        );
        Ok(Self(divisor.divide_checked(self.0)?))
    }

    /// Divides by a prepared right divisor with abort-aware checked semantics.
    pub fn div_matrix_checked_with_prepared_with_abort(
        self,
        divisor: &mut PreparedRightDivisor4<'_, B>,
        signal: &AbortSignal,
    ) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "div-matrix-checked-with-prepared-with-abort"
        );
        Ok(Self(divisor.divide_checked_with_abort(self.0, signal)?))
    }

    /// Constructs a scalar multiple of the 4x4 identity matrix.
    pub fn uniform_scale(scale: Scalar<B>) -> Self {
        crate::trace_dispatch!("hyperlattice_matrix", "constructor", "uniform-scale");
        Self([
            [
                scale.clone(),
                Scalar::zero(),
                Scalar::zero(),
                Scalar::zero(),
            ],
            [
                Scalar::zero(),
                scale.clone(),
                Scalar::zero(),
                Scalar::zero(),
            ],
            [
                Scalar::zero(),
                Scalar::zero(),
                scale.clone(),
                Scalar::zero(),
            ],
            [Scalar::zero(), Scalar::zero(), Scalar::zero(), scale],
        ])
    }

    /// Constructs the inverse of a known scalar multiple of the 4x4 identity.
    ///
    /// This is intentionally an opt-in API rather than an automatic
    /// `Matrix4::reciprocal` dispatch branch: prior targeted benches showed
    /// that dynamic equality checks for uniform scale made adjacent diagonal
    /// inverse paths less flat. When the caller already owns the object-level
    /// fact `A = sI`, one scalar inverse is sufficient and reusing it preserves
    /// hyperreal's exact/symbolic node cache. This is the explicit object-layer
    /// specialization recommended by Yap, "Towards Exact Geometric
    /// Computation", 1997, and the diagonal solve specialization in Golub and
    /// Van Loan, *Matrix Computations*.
    pub fn uniform_scale_inverse(scale: Scalar<B>) -> BlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "constructor",
            "uniform-scale-inverse"
        );
        let inv = scale.inverse()?;
        Ok(Self::uniform_scale(inv))
    }

    /// Returns a lightweight transform handle for repeated matrix-vector transforms.
    ///
    /// The handle caches affine, diagonal, translation-column, and
    /// point/direction-relevant facts once so repeated transforms can avoid
    /// rebuilding those structural classifications. This is a deliberate
    /// retained-geometry fast path in the spirit of exact geometric
    /// computation; see Yap, "Towards Exact Geometric Computation", 1997.
    pub fn transform_vec4_handle(&self) -> TransformedMatrix4<'_, B> {
        TransformedMatrix4::new(self)
    }

    /// Returns a handle for a single vector transformation under this matrix.
    ///
    /// Reusing the same handle path keeps matrix-wide facts aligned with the
    /// shared-batch transform path and avoids duplicating translation probes.
    pub fn transform_vec4_with<'a>(&'a self, rhs: &'a Vector4<B>) -> TransformedVector4<'a, B> {
        self.transform_vec4_handle().vector(rhs)
    }

    /// Transforms a point vector assuming `rhs[3] == 1`, which keeps a single
    /// guaranteed affine helper shape and avoids probing point/direction
    /// predicates.
    pub fn transform_vec4_point(&self, rhs: &Vector4<B>) -> Vector4<B> {
        if matrix4_affine_linear_is_diagonal(&self.0) {
            return Vector4(
                transform_vector4_rhs_point_affine_linear_diagonal_ref_cached(&self.0, &rhs.0),
            );
        }
        let facts = matrix4_facts(&self.0);
        // Reuse precomputed structural facts for the fallback path by
        // constructing the handle with `new_with_facts` instead of recomputing
        // in `transform_vec4_handle`. This keeps object-level structure on the
        // handle, matching Yap's "geometric package" philosophy.
        // See Yap, "Towards Exact Geometric Computation", 1997.
        TransformedMatrix4::new_with_facts(self, facts).transform_point_vector(rhs)
    }

    /// Transforms a direction vector assuming `rhs[3] == 0`, keeping the fast
    /// 3-term affine-less form.
    pub fn transform_vec4_direction(&self, rhs: &Vector4<B>) -> Vector4<B> {
        if B::FUSE_SIGNED_PRODUCT_SUM {
            match matrix4_direction_linear_kind(&self.0) {
                Matrix4DirectionLinearKind::Identity => {
                    crate::trace_dispatch!(
                        "hyperlattice_matrix",
                        "method",
                        "transform-vector-vec4-direction-linear-identity"
                    );
                    return rhs.clone();
                }
                Matrix4DirectionLinearKind::Diagonal => {
                    return Vector4(transform_vector4_rhs_direction_ref_cached(
                        &self.0, &rhs.0, true,
                    ));
                }
                Matrix4DirectionLinearKind::General => {
                    return Vector4(transform_vector4_rhs_direction_ref_cached(
                        &self.0, &rhs.0, false,
                    ));
                }
            }
        }
        Vector4(transform_vector4_rhs_direction_ref_cached(
            &self.0,
            &rhs.0,
            matrix4_direction_linear_is_diagonal(&self.0),
        ))
    }

    /// Transforms a batch of homogeneous directions, assuming every input has `w = 0`.
    ///
    /// Use this when geometry/object-level facts already classify the whole
    /// batch as directions. It avoids the generic batch classifier and keeps
    /// the translation column out of the arithmetic schedule.
    pub fn transform_vec4_direction_batch(&self, rhs: &[Vector4<B>]) -> Vec<Vector4<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "transform-vector-vec4-direction-batch"
        );
        // One-shot direction batches do not need the full Matrix4 handle fact
        // scan. The only matrix fact required for the fastest direction kernel
        // is whether the 3x3 linear block is diagonal while the bottom spatial
        // row is zero. That keeps this convenience API thinner than a prepared
        // handle, while repeated callers can still build the handle once. The
        // distinction follows Yap's exact-geometry package advice: retain and
        // reuse object facts when they exist, but do not make isolated arithmetic
        // calls pay for unrelated geometric metadata.
        transform_vector4_direction_batch_assumed_ref(
            &self.0,
            rhs,
            matrix4_direction_linear_is_diagonal(&self.0),
        )
    }

    /// Transforms a batch of homogeneous points, assuming every input has `w = 1`.
    ///
    /// Use this when geometry/object-level facts already classify the whole
    /// batch as points. It preserves the exact point invariant and avoids the
    /// generic point/direction/unknown classification pass.
    pub fn transform_vec4_point_batch(&self, rhs: &[Vector4<B>]) -> Vec<Vector4<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "transform-vector-vec4-point-batch"
        );
        // Unlike direction batches, point batches did not benefit from a
        // thinner one-shot public route: the public affine-diagonal helper was
        // trace-clean but regressed the approx backend and did not show a stable
        // exact-backend win. If that changes, prefer a backend-gated split over
        // letting approx loop shape block hyperreal performance.
        self.transform_vec4_handle().transform_point_batch(rhs)
    }

    /// Transforms all vectors in `rhs` using the same matrix.
    ///
    /// For `4x4`, per-row homogeneous-coordinate facts are cached once so they
    /// are reused for every vector in the batch without changing observable
    /// affine structure. If the same matrix is reused across multiple batches,
    /// prefer [`Matrix4::transform_vec4_handle`]: targeted translated-diagonal
    /// and dense batch sentinels show the prebuilt handle avoids repeated
    /// structural fact construction and keeps point/direction dispatch flat.
    pub fn transform_vec4_batch(&self, rhs: &[Vector4<B>]) -> Vec<Vector4<B>> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "transform-vector-vec4-batch"
        );
        self.transform_vec4_handle().transform_vector_batch(rhs)
    }

    /// Returns the matrix inverse using a fixed-size cofactor expansion.
    ///
    /// The ordinary path rejects a definite-zero determinant and propagates
    /// scalar arithmetic errors from the selected backend.
    pub fn inverse(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "matrix4-inverse");
        Ok(Self(invert_matrix4(self.0)?))
    }

    /// Returns the matrix inverse after rejecting unknown-zero determinants.
    pub fn inverse_checked(self) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "matrix4-inverse-checked");
        Ok(Self(invert_matrix4_checked(self.0)?))
    }

    /// Returns the checked matrix inverse after attaching an abort signal.
    pub fn inverse_checked_with_abort(self, signal: &AbortSignal) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!(
            "hyperlattice_matrix",
            "method",
            "matrix4-inverse-checked-with-abort"
        );
        Ok(Self(invert_matrix4_checked_with_abort(self.0, signal)?))
    }

    /// Returns the determinant.
    pub fn determinant(&self) -> Scalar<B> {
        crate::trace_dispatch!("hyperlattice_matrix", "method", "matrix4-determinant");
        determinant4(&self.0)
    }
}
