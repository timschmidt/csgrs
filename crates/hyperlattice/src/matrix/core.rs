//! Fixed-size row-major matrices over [`crate::Real`].
//!
//! Implementation map:
//! - type layout and generic array helpers
//! - powers, right-division, and fixed-size multiply kernels
//! - immediate matrix-vector and batch transforms
//! - determinant, adjugate, and inverse kernels
//! - public Matrix3/Matrix4 methods and operator impls

use std::array::from_fn;
use std::fmt;
use std::mem;
use std::ops::{Add, BitXor, Div, Index, IndexMut, Mul, Neg, Sub};

use crate::point::Point3;
use crate::scalar::{clone_with_abort, reject_definite_zero, require_known_nonzero, with_abort};
use crate::vector::{Vector3, Vector4, Vector4GeometricFacts, Vector4HomogeneousKind};
use crate::{
    AbortSignal, BlasResult, CheckedBlasResult, ExactRationalKind, Problem, Real,
    RealExactSetFacts, RealKernelExt, RealSign, SymbolicDependencyMask, ZeroKnowledge,
    ZeroOneMinusOneStatus,
};

/// Records a matrix dispatch path under the `hyperlattice_matrix` layer.
/// It forwards the same triple as `trace_dispatch!`, so trace counts are
/// unchanged; it only keeps each call on one line.
macro_rules! trace_matrix {
    ($operation:expr, $path:expr) => {
        crate::trace_dispatch!("hyperlattice_matrix", $operation, $path)
    };
}

/// Constructs the row-major identity with exact zero and one scalars.
///
/// Nested array construction visits rows then columns; dimension zero gives an
/// empty array without scalar construction.
fn identity_array<const N: usize>() -> [[Real; N]; N] {
    from_fn(|row| {
        from_fn(|col| {
            if row == col {
                Real::one()
            } else {
                Real::zero()
            }
        })
    })
}

/// Transposes an owned 3×3 array by moving scalar handles.
///
/// Right-division uses this layout change to express a solve on transposes; no
/// scalar arithmetic, refinement, or cloning is performed.
fn transpose_array3(matrix: [[Real; 3]; 3]) -> [[Real; 3]; 3] {
    // Right-division is implemented as a solve on transposes. Fixed-size
    // transposes keep that wrapper from paying generic `Option::take` and
    // `from_fn` overhead around the actual Gauss-Jordan work. Structural note:
    // keep this path local to matrix algebra; future exact-grid or sparse-row
    // facts should choose faster solve kernels here without leaking predicate
    // or triangulation semantics into `hyperlattice`.
    let [[m00, m01, m02], [m10, m11, m12], [m20, m21, m22]] = matrix;
    [[m00, m10, m20], [m01, m11, m21], [m02, m12, m22]]
}

/// Transposes a borrowed 3×3 array, cloning handles in output row-major order.
///
/// Each input column becomes an output row. Scalar cloning preserves its shared
/// representation; this helper performs no arithmetic.
fn transpose_array3_ref(matrix: &[[Real; 3]; 3]) -> [[Real; 3]; 3] {
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

/// Transposes an owned 4×4 array by moving scalar handles.
///
/// This is the fixed-size layout change used by right-division wrappers, without
/// scalar arithmetic, refinement, or cloning.
fn transpose_array4(matrix: [[Real; 4]; 4]) -> [[Real; 4]; 4] {
    // Hand-written 4x4 transpose avoids the generic `Option::take` owned
    // transpose overhead in the right-division wrapper. Retaining this as a
    // fixed-size kernel also gives future structural matrix facts a clean
    // dispatch point for exact hyperreal-backed solves.
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

/// Transposes a borrowed 4×4 array, cloning handles in output row-major order.
///
/// Each input column becomes an output row, preserving scalar sharing without
/// arithmetic.
fn transpose_array4_ref(matrix: &[[Real; 4]; 4]) -> [[Real; 4]; 4] {
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

/// Returns the low `N * N` bits for a row-major matrix entry mask.
///
/// The caller must keep `N * N <= 16`; dimensions three and four are used here.
/// The full-width case returns `u16::MAX` without a shift by 16. Zero gives zero.
#[inline]
fn matrix_mask<const N: usize>() -> u16 {
    debug_assert!(N * N <= u16::BITS as usize);
    if N * N == u16::BITS as usize {
        u16::MAX
    } else {
        (1_u16 << (N * N)) - 1
    }
}

/// Encodes one entry as bit `row * N + column`.
///
/// Callers must supply valid indices with the resulting bit index below 16; this
/// helper does not validate either the matrix bounds or the shift width.
#[inline]
fn matrix_entry_bit<const N: usize>(row: usize, column: usize) -> u16 {
    1_u16 << (row * N + column)
}

/// Reads a row-major entry bit, returning `None` for either out-of-range index.
///
/// The matrix dimension must fit the 16-bit entry mask. `Some(false)` means the
/// bit is absent; for a knowledge mask it does not prove the opposite fact.
#[inline]
fn matrix_entry_mask_value<const N: usize>(mask: u16, row: usize, column: usize) -> Option<bool> {
    if row < N && column < N {
        Some((mask & matrix_entry_bit::<N>(row, column)) != 0)
    } else {
        None
    }
}

/// Counts only the low `N` known-zero bits of a row or column mask.
///
/// Higher bits are ignored. The shift requires `N < 8` (current matrix callers
/// use three or four); the debug assertion alone also admits eight and does not
/// enforce this complete precondition. Dimension zero counts no bits.
#[inline]
fn matrix_lane_known_zero_count<const N: usize>(mask: u8) -> u32 {
    debug_assert!(N <= u8::BITS as usize);
    (mask & ((1_u8 << N) - 1)).count_ones()
}

/// Tests whether at most one lane entry lacks a retained zero fact.
///
/// An entirely known-zero lane also qualifies. Uses the low `N` bits and the
/// dimension precondition of [`matrix_lane_known_zero_count`].
#[inline]
fn matrix_lane_has_sparse_support<const N: usize>(mask: u8) -> bool {
    matrix_lane_known_zero_count::<N>(mask) >= (N as u32).saturating_sub(1)
}

/// Tests whether all low `N` lane bits certify zero.
///
/// Uses the dimension precondition of [`matrix_lane_known_zero_count`]; the
/// zero-length lane qualifies vacuously.
#[inline]
fn matrix_lane_is_known_zero<const N: usize>(mask: u8) -> bool {
    matrix_lane_known_zero_count::<N>(mask) == N as u32
}

/// Scans lane masks in order and stops at the first entirely known-zero lane.
///
/// The low-bit and dimension rules are those of [`matrix_lane_is_known_zero`].
/// An empty collection has no zero lane.
#[inline]
fn matrix_has_zero_lane<const N: usize>(masks: [u8; N]) -> bool {
    masks.into_iter().any(matrix_lane_is_known_zero::<N>)
}

/// Chooses a determinant schedule from supplied facts without rechecking entries.
///
/// Precedence is zero row, zero column, diagonal, triangular, sparse rows or
/// columns, shared denominator, dyadic, exact rational, then generic fallback.
/// Each successful earlier branch skips later checks. Masks use the lane-width
/// precondition of [`matrix_lane_known_zero_count`]; flags and scalar facts must
/// describe the same unchanged matrix to carry mathematical meaning.
#[inline]
fn matrix_determinant_schedule_hint<const N: usize>(
    exact: RealExactSetFacts,
    row_zero_masks: [u8; N],
    column_zero_masks: [u8; N],
    is_diagonal: bool,
    is_upper_triangular: bool,
    is_lower_triangular: bool,
) -> MatrixDeterminantScheduleHint {
    if matrix_has_zero_lane::<N>(row_zero_masks) || matrix_has_zero_lane::<N>(column_zero_masks) {
        return MatrixDeterminantScheduleHint::StructurallyZero;
    }
    if is_diagonal {
        return MatrixDeterminantScheduleHint::Diagonal;
    }
    if is_upper_triangular || is_lower_triangular {
        return MatrixDeterminantScheduleHint::Triangular;
    }
    if row_zero_masks
        .into_iter()
        .all(matrix_lane_has_sparse_support::<N>)
        || column_zero_masks
            .into_iter()
            .all(matrix_lane_has_sparse_support::<N>)
    {
        return MatrixDeterminantScheduleHint::SparseSupport;
    }
    if exact.has_shared_denominator_schedule() {
        return MatrixDeterminantScheduleHint::SharedDenominator;
    }
    if exact.has_dyadic_schedule() {
        return MatrixDeterminantScheduleHint::Dyadic;
    }
    if exact.is_nonempty_exact_rational() {
        return MatrixDeterminantScheduleHint::ExactRational;
    }
    MatrixDeterminantScheduleHint::GenericRealFallback
}

/// Collects definite zeros in one row-major scalar scan.
///
/// Returns the entry mask, per-row column masks, and per-column row masks.
/// Only `Real::definitely_zero` is consulted; absent bits are not nonzero proofs.
/// The dimension must fit the 16-bit entry mask and eight-bit lane masks.
#[inline]
fn matrix_zero_masks<const N: usize>(matrix: &[[Real; N]; N]) -> (u16, [u8; N], [u8; N]) {
    let mut entry_mask = 0_u16;
    let mut row_masks = [0_u8; N];
    let mut column_masks = [0_u8; N];
    for (row, values) in matrix.iter().enumerate() {
        for (column, value) in values.iter().enumerate() {
            if value.definitely_zero() {
                entry_mask |= matrix_entry_bit::<N>(row, column);
                row_masks[row] |= 1_u8 << column;
                column_masks[column] |= 1_u8 << row;
            }
        }
    }
    (entry_mask, row_masks, column_masks)
}

/// Collects definite ones into a row-major entry mask.
///
/// Each entry is queried once with `Real::definitely_one`; missing bits carry no
/// inequality claim. The dimension must fit the 16-bit entry mask.
#[inline]
fn matrix_one_mask<const N: usize>(matrix: &[[Real; N]; N]) -> u16 {
    let mut mask = 0_u16;
    for (row, values) in matrix.iter().enumerate() {
        for (column, value) in values.iter().enumerate() {
            if value.definitely_one() {
                mask |= matrix_entry_bit::<N>(row, column);
            }
        }
    }
    mask
}

/// Unions retained symbolic dependency families in row-major entry order.
///
/// Reads each scalar’s detailed facts, starting from the empty dependency mask.
/// This summarizes recognized families rather than deciding scalar equality.
#[inline]
fn matrix_symbolic_dependency_mask<const N: usize>(
    matrix: &[[Real; N]; N],
) -> SymbolicDependencyMask {
    matrix
        .iter()
        .flat_map(|row| row.iter())
        .fold(SymbolicDependencyMask::NONE, |mask, value| {
            mask.union(value.detailed_facts().symbolic.dependencies)
        })
}

/// Recognizes one signed unit per row and column from retained scalar statuses.
///
/// Rows are recognized in order, stopping at the first failure; column uniqueness
/// is checked only after all four rows succeed. `None` means recognition failed,
/// not that an unrecognized symbolic matrix is proved unequal to every permutation.
#[inline]
fn matrix4_signed_permutation_rows(matrix: &[[Real; 4]; 4]) -> Option<[SignedAxis4; 4]> {
    // A signed permutation matrix has one exact signed unit in each row and
    // column, with every other entry exactly zero. Retaining this fact at the
    // matrix boundary lets transform and division code choose signed-axis
    // schedules without re-probing every scalar lane. The row/column sparsity
    // pattern is the fixed-size analogue of sparse matrix scheduling.
    let rows = [
        matrix4_signed_axis_row(&matrix[0])?,
        matrix4_signed_axis_row(&matrix[1])?,
        matrix4_signed_axis_row(&matrix[2])?,
        matrix4_signed_axis_row(&matrix[3])?,
    ];
    let mut used_columns = 0_u8;
    for axis in rows {
        let bit = 1_u8 << axis.index();
        if used_columns & bit != 0 {
            return None;
        }
        used_columns |= bit;
    }
    (used_columns == 0b1111).then_some(rows)
}

/// Recognizes an exact-rational scalar multiple of the 3×3 identity.
///
/// Trusts the supplied diagonal flag, then compares exposed rational diagonals;
/// zero scale is allowed and symbolic equality is not requested.
#[inline]
fn matrix3_exact_rational_uniform_scale(matrix: &[[Real; 3]; 3], is_diagonal: bool) -> bool {
    if !is_diagonal {
        return false;
    }
    exact_rational_diagonal_entries_equal([&matrix[0][0], &matrix[1][1], &matrix[2][2]])
}

/// Recognizes an exact-rational scalar multiple of the full 4×4 identity.
///
/// Trusts the supplied diagonal flag, then compares all four rational diagonals,
/// including the homogeneous coordinate. Zero scale is allowed.
#[inline]
fn matrix4_exact_rational_uniform_scale(matrix: &[[Real; 4]; 4], is_diagonal: bool) -> bool {
    if !is_diagonal {
        return false;
    }
    exact_rational_diagonal_entries_equal([
        &matrix[0][0],
        &matrix[1][1],
        &matrix[2][2],
        &matrix[3][3],
    ])
}

/// Compares exposed exact-rational diagonal values without symbolic equality.
///
/// Requires a nonempty array: the first element is indexed directly. Every value
/// must expose the same rational; failure stops the scan. An absent rational view
/// is a recognition failure, not a proof that the values differ.
#[inline]
fn exact_rational_diagonal_entries_equal<const N: usize>(entries: [&Real; N]) -> bool {
    // Conservative uniform-scale detection is deliberately exact-rational only:
    // it never asks symbolic `Real` values to prove equality and therefore
    // cannot force approximation or deep graph walks from a structural-facts
    // query. This keeps the object fact cheap, in the spirit of the
    // the exact object-structure policy: missing a uniform-scale fact is a performance miss, while a
    // false fact would be a correctness bug.
    let Some(first) = entries[0].exact_rational_ref() else {
        return false;
    };
    entries[1..]
        .iter()
        .all(|entry| entry.exact_rational_ref() == Some(first))
}

/// Borrows four row entries for the signed-unit recognition scan.
///
/// The result identifies the sole recognized unit’s column and sign, or `None`
/// when [`matrix4_signed_axis_row_refs`] cannot certify that row.
#[inline]
fn matrix4_signed_axis_row(row: &[Real; 4]) -> Option<SignedAxis4> {
    matrix4_signed_axis_row_refs([&row[0], &row[1], &row[2], &row[3]])
}

/// Recognizes exactly one signed unit and three zeros, in column order.
///
/// A second unit, other value, or unknown status rejects the row immediately; an
/// all-zero row also returns `None`. No approximate comparison is used.
#[inline]
fn matrix4_signed_axis_row_refs(row: [&Real; 4]) -> Option<SignedAxis4> {
    let mut axis = None;
    for (index, value) in row.into_iter().enumerate() {
        let status = value.zero_one_or_minus_one();
        match status {
            ZeroOneMinusOneStatus::Zero => {}
            ZeroOneMinusOneStatus::One | ZeroOneMinusOneStatus::MinusOne if axis.is_none() => {
                axis = Some(signed_axis4_from_index(
                    index,
                    matches!(status, ZeroOneMinusOneStatus::MinusOne),
                )?);
            }
            _ => return None,
        }
    }
    axis
}

/// Maps column indices zero through three and a sign to X, Y, Z, or W.
///
/// Returns `None` for any other index.
#[inline]
fn signed_axis4_from_index(index: usize, negative: bool) -> Option<SignedAxis4> {
    match (index, negative) {
        (0, false) => Some(SignedAxis4::PosX),
        (0, true) => Some(SignedAxis4::NegX),
        (1, false) => Some(SignedAxis4::PosY),
        (1, true) => Some(SignedAxis4::NegY),
        (2, false) => Some(SignedAxis4::PosZ),
        (2, true) => Some(SignedAxis4::NegZ),
        (3, false) => Some(SignedAxis4::PosW),
        (3, true) => Some(SignedAxis4::NegW),
        _ => None,
    }
}

/// Three-by-three row-major matrix over exact scalar representations.
///
/// Entries are publicly editable. `PartialEq` delegates to scalar structural
/// equality; it does not decide equality of all represented real values.
/// Changing entries does not update previously returned structural facts.
#[derive(Debug, PartialEq)]
pub struct Matrix3(
    /// Matrix entries in row-major order.
    pub [[Real; 3]; 3],
);

/// Four-by-four row-major matrix over exact scalar representations.
///
/// Entries are publicly editable. `PartialEq` delegates to scalar structural
/// equality; it does not decide equality of all represented real values.
/// Changing entries does not update previously returned structural facts.
#[derive(Debug, PartialEq)]
pub struct Matrix4(
    /// Matrix entries in row-major order.
    pub [[Real; 4]; 4],
);

impl Clone for Matrix3 {
    /// Clones all nine scalar handles in row-major order, preserving scalar sharing.
    #[inline]
    fn clone(&self) -> Self {
        Self([
            [
                self.0[0][0].clone(),
                self.0[0][1].clone(),
                self.0[0][2].clone(),
            ],
            [
                self.0[1][0].clone(),
                self.0[1][1].clone(),
                self.0[1][2].clone(),
            ],
            [
                self.0[2][0].clone(),
                self.0[2][1].clone(),
                self.0[2][2].clone(),
            ],
        ])
    }
}

impl Clone for Matrix4 {
    /// Clones all sixteen scalar handles in row-major order, preserving scalar sharing.
    #[inline]
    fn clone(&self) -> Self {
        Self([
            [
                self.0[0][0].clone(),
                self.0[0][1].clone(),
                self.0[0][2].clone(),
                self.0[0][3].clone(),
            ],
            [
                self.0[1][0].clone(),
                self.0[1][1].clone(),
                self.0[1][2].clone(),
                self.0[1][3].clone(),
            ],
            [
                self.0[2][0].clone(),
                self.0[2][1].clone(),
                self.0[2][2].clone(),
                self.0[2][3].clone(),
            ],
            [
                self.0[3][0].clone(),
                self.0[3][1].clone(),
                self.0[3][2].clone(),
                self.0[3][3].clone(),
            ],
        ])
    }
}

/// Advisory determinant schedule selected from retained matrix facts.
///
/// The hint is intentionally not a determinant value and not an invertibility
/// certificate. It tells callers which fixed-size exact arithmetic package is
/// worth trying before generic determinant expansion. This keeps matrix
/// structure in `hyperlattice` while predicates and topology stay in
/// `hyperlimit`, keeping geometric object packages separate from arithmetic
/// packages. Sparse and triangular schedule names describe the retained matrix
/// structure directly.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MatrixDeterminantScheduleHint {
    /// The determinant is structurally zero because at least one row or column
    /// is entirely known zero.
    StructurallyZero,
    /// The matrix is structurally diagonal.
    Diagonal,
    /// The matrix is structurally triangular but not diagonal.
    Triangular,
    /// Every row or every column has certified sparse support.
    SparseSupport,
    /// Every entry has one shared reduced denominator.
    SharedDenominator,
    /// Every entry is dyadic.
    Dyadic,
    /// Every entry is exact rational, but no stronger retained schedule applies.
    ExactRational,
    /// No exact-rational matrix schedule is certified by retained facts.
    GenericRealFallback,
}

impl MatrixDeterminantScheduleHint {
    /// Returns whether this hint is selected entirely by retained matrix shape.
    ///
    /// Shape-driven hints can be chosen without inspecting scalar denominator
    /// schedules. The first four enum variants are shape-driven; the remaining
    /// variants depend on scalar representation facts or select the fallback.
    pub fn is_shape_driven(self) -> bool {
        matches!(
            self,
            Self::StructurallyZero | Self::Diagonal | Self::Triangular | Self::SparseSupport
        )
    }

    /// Returns whether this hint is selected by exact-rational scalar facts.
    ///
    /// These routes are still exact, but they depend on `hyperreal`'s retained
    /// arithmetic facts rather than solely on matrix shape. Downstream kernels
    /// can use this to try fraction-free or common-denominator schedules before
    /// generic `Real` expansion.
    pub fn is_exact_rational_driven(self) -> bool {
        matches!(
            self,
            Self::SharedDenominator | Self::Dyadic | Self::ExactRational
        )
    }

    /// Returns whether no retained exact matrix schedule was certified.
    pub fn requires_generic_real_fallback(self) -> bool {
        matches!(self, Self::GenericRealFallback)
    }
}

/// Conservative homogeneous 2D transform kind for a [`Matrix3`].
///
/// The kind is a scheduling fact, not a proof of geometric intent. It is
/// derived from exact structural facts already collected during matrix
/// classification. Object packages expose cheap structure, while topology and
/// scalar algebra remain separate decisions.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Matrix3TransformKind {
    /// Structurally the identity transform.
    Identity,
    /// Homogeneous affine translation with unit diagonal linear block.
    AffineTranslation,
    /// Affine transform whose 2x2 linear block is structurally diagonal.
    AffineDiagonalLinear,
    /// Affine transform with bottom row `[0, 0, 1]`.
    Affine,
    /// No affine transform structure was certified.
    Projective,
}

/// Conservative homogeneous 3D transform kind for a [`Matrix4`].
///
/// This groups the public affine, diagonal-linear, translation, and
/// signed-permutation facts into one stable dispatch key. It intentionally
/// remains advisory: exact determinant signs, incidence predicates, and
/// topology decisions still belong to exact scalar/predicate kernels.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Matrix4TransformKind {
    /// Structurally the identity transform.
    Identity,
    /// Structurally a signed permutation of homogeneous coordinates.
    SignedPermutation,
    /// Homogeneous affine translation with unit diagonal linear block.
    AffineTranslation,
    /// Affine transform whose 3x3 linear block is structurally diagonal.
    AffineDiagonalLinear,
    /// Affine transform with bottom row `[0, 0, 0, 1]`.
    Affine,
    /// No affine or signed-permutation transform structure was certified.
    Projective,
}

/// Conservative structural snapshot for a [`Matrix3`].
///
/// Matrix collection combines shape checks with separate rational-set, entry-mask,
/// and symbolic-dependency passes. Retaining this packet lets callers select
/// schedules without repeating those queries. A missing fact is not a proof of
/// its opposite, and uniform-scale recognition is restricted to exposed rationals.
///
/// Public fields can be edited independently. Queries trust their stored values;
/// they neither validate consistency nor retain a link to the source matrix.
/// Mathematical certificates require an unmodified packet for the same unchanged
/// matrix. Recollect facts after changing matrix entries.
///
/// Entry bit `row * 3 + column` uses row-major order. Row-mask bits index columns;
/// column-mask bits index rows. Lane queries ignore bits above the low 3;
/// `is_zero` instead compares the whole entry mask with its expected full mask.
/// Indexed queries return `None` out of bounds. Sparse support means at most one
/// entry lacks a zero fact, including an entirely known-zero lane.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Matrix3StructuralFacts {
    /// Exact-rational representation facts for all entries.
    pub exact: RealExactSetFacts,
    /// Union of scalar symbolic dependency families across all entries.
    ///
    /// Records recognized scalar families for scheduling without exposing scalar
    /// storage. It does not decide equality, determinant sign, or topology.
    pub symbolic_dependencies: SymbolicDependencyMask,
    /// Bit mask of entries known to be exactly zero, in row-major order.
    pub zero_mask: u16,
    /// Bit mask of entries known to be exactly one, in row-major order.
    pub one_mask: u16,
    /// Per-row masks of entries known to be exactly zero.
    pub row_zero_masks: [u8; 3],
    /// Per-column masks of entries known to be exactly zero.
    pub column_zero_masks: [u8; 3],
    /// Whether the matrix is structurally the identity.
    pub is_identity: bool,
    /// Whether all off-diagonal entries are structurally zero.
    pub is_diagonal: bool,
    /// Whether the matrix is a diagonal exact-rational scalar multiple of identity.
    ///
    /// This is conservative: symbolic equal diagonal entries are not reported
    /// here unless they are exposed as exact rationals by `hyperreal`.
    pub is_exact_rational_uniform_scale: bool,
    /// Whether all entries below the diagonal are structurally zero.
    pub is_upper_triangular: bool,
    /// Whether all entries above the diagonal are structurally zero.
    pub is_lower_triangular: bool,
    /// Whether the 2D homogeneous affine row is structurally `[0, 0, 1]`.
    pub is_affine: bool,
    /// Whether the affine linear block is structurally diagonal and unit-scaled.
    pub is_affine_translation: bool,
    /// Conservative transform kind derived from retained homogeneous facts.
    pub transform_kind: Matrix3TransformKind,
}

impl Matrix3StructuralFacts {
    /// Returns true when every matrix entry is known to be exactly zero.
    pub fn is_zero(self) -> bool {
        self.zero_mask == matrix_mask::<3>()
    }

    /// Returns whether one entry is structurally known to be exactly zero.
    ///
    /// `None` means the row or column is out of bounds. `Some(false)` is not a
    /// nonzero certificate; it only means the cheap structural scan did not
    /// prove zero. Preserving that distinction follows the exact geometric
    /// computation rule that conservative object facts may select algorithms,
    /// but absent facts must not decide mathematics.
    pub fn entry_known_zero(self, row: usize, column: usize) -> Option<bool> {
        matrix_entry_mask_value::<3>(self.zero_mask, row, column)
    }

    /// Returns whether one entry has a retained exact-one bit.
    ///
    /// `None` means an out-of-bounds index; `Some(false)` is not an inequality
    /// proof. No scalar is queried to validate the stored bit.
    pub fn entry_known_one(self, row: usize, column: usize) -> Option<bool> {
        matrix_entry_mask_value::<3>(self.one_mask, row, column)
    }

    /// Returns the zero mask for one row.
    pub fn row_zero_mask(self, row: usize) -> Option<u8> {
        self.row_zero_masks.get(row).copied()
    }

    /// Returns the zero mask for one column.
    pub fn column_zero_mask(self, column: usize) -> Option<u8> {
        self.column_zero_masks.get(column).copied()
    }

    /// Counts entries in one row that are structurally known zero.
    pub fn row_known_zero_count(self, row: usize) -> Option<u32> {
        self.row_zero_mask(row)
            .map(matrix_lane_known_zero_count::<3>)
    }

    /// Counts entries in one column that are structurally known zero.
    pub fn column_known_zero_count(self, column: usize) -> Option<u32> {
        self.column_zero_mask(column)
            .map(matrix_lane_known_zero_count::<3>)
    }

    /// Returns whether one row is structurally known to be entirely zero.
    ///
    /// This is the determinant-relevant version of [`Self::row_zero_mask`]:
    /// callers get a semantic certificate instead of depending on the mask
    /// layout. The fact is conservative and may return `Some(false)` for a row
    /// that is mathematically zero but not cheaply certified by scalar facts.
    pub fn row_is_known_zero(self, row: usize) -> Option<bool> {
        self.row_zero_mask(row).map(matrix_lane_is_known_zero::<3>)
    }

    /// Returns whether one column is structurally known to be entirely zero.
    pub fn column_is_known_zero(self, column: usize) -> Option<bool> {
        self.column_zero_mask(column)
            .map(matrix_lane_is_known_zero::<3>)
    }

    /// Returns whether any row has a retained all-zero certificate.
    pub fn has_known_zero_row(self) -> bool {
        self.row_zero_masks
            .into_iter()
            .any(matrix_lane_is_known_zero::<3>)
    }

    /// Returns whether any column has a retained all-zero certificate.
    pub fn has_known_zero_column(self) -> bool {
        self.column_zero_masks
            .into_iter()
            .any(matrix_lane_is_known_zero::<3>)
    }

    /// Returns whether any row or column is certified entirely zero.
    pub fn has_known_zero_lane(self) -> bool {
        self.has_known_zero_row() || self.has_known_zero_column()
    }

    /// Returns whether one row has certified sparse support.
    ///
    /// A row is sparse here when all but at most one entry are known zero. This
    /// exposes the sparse-kernel sparse-row scheduling signal without forcing
    /// callers to interpret mask layouts directly.
    pub fn row_has_sparse_support(self, row: usize) -> Option<bool> {
        self.row_zero_mask(row)
            .map(matrix_lane_has_sparse_support::<3>)
    }

    /// Returns whether one column has certified sparse support.
    pub fn column_has_sparse_support(self, column: usize) -> Option<bool> {
        self.column_zero_mask(column)
            .map(matrix_lane_has_sparse_support::<3>)
    }

    /// Returns whether every row has certified sparse support.
    pub fn all_rows_have_sparse_support(self) -> bool {
        self.row_zero_masks
            .into_iter()
            .all(matrix_lane_has_sparse_support::<3>)
    }

    /// Returns whether every column has certified sparse support.
    pub fn all_columns_have_sparse_support(self) -> bool {
        self.column_zero_masks
            .into_iter()
            .all(matrix_lane_has_sparse_support::<3>)
    }

    /// Select an advisory determinant schedule from retained matrix facts.
    ///
    /// Precedence is zero row or column, diagonal, triangular, sparse rows or
    /// columns, shared denominator, dyadic, exact rational, then generic fallback.
    /// In particular, shared denominator wins when entries are also dyadic.
    /// Missing facts do not decide invertibility. A valid zero-lane fact does
    /// prove singularity; queries do not validate edited packets.
    pub fn determinant_schedule_hint(self) -> MatrixDeterminantScheduleHint {
        matrix_determinant_schedule_hint::<3>(
            self.exact,
            self.row_zero_masks,
            self.column_zero_masks,
            self.is_diagonal,
            self.is_upper_triangular,
            self.is_lower_triangular,
        )
    }
}

/// Conservative structural snapshot for a [`Matrix4`].
///
/// Matrix collection combines shape checks with separate rational-set, entry-mask,
/// and symbolic-dependency passes. Retaining this packet lets callers select
/// schedules without repeating those queries. A missing fact is not a proof of
/// its opposite, and uniform-scale recognition is restricted to exposed rationals.
///
/// Public fields can be edited independently. Queries trust their stored values;
/// they neither validate consistency nor retain a link to the source matrix.
/// Mathematical certificates require an unmodified packet for the same unchanged
/// matrix. Recollect facts after changing matrix entries.
///
/// Entry bit `row * 4 + column` uses row-major order. Row-mask bits index columns;
/// column-mask bits index rows. Lane queries ignore bits above the low 4;
/// `is_zero` instead compares the whole entry mask with its expected full mask.
/// Indexed queries return `None` out of bounds. Sparse support means at most one
/// entry lacks a zero fact, including an entirely known-zero lane.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Matrix4StructuralFacts {
    /// Exact-rational representation facts for all entries.
    pub exact: RealExactSetFacts,
    /// Union of scalar symbolic dependency families across all entries.
    ///
    /// Transform stacks can retain this alongside affine and sparse facts to
    /// select exact or symbolic-aware algebra routes without peeking into
    /// scalar storage. It is not a determinant, invertibility, or topology
    /// certificate.
    pub symbolic_dependencies: SymbolicDependencyMask,
    /// Bit mask of entries known to be exactly zero, in row-major order.
    pub zero_mask: u16,
    /// Bit mask of entries known to be exactly one, in row-major order.
    pub one_mask: u16,
    /// Per-row masks of entries known to be exactly zero.
    pub row_zero_masks: [u8; 4],
    /// Per-column masks of entries known to be exactly zero.
    pub column_zero_masks: [u8; 4],
    /// Whether the matrix is structurally the identity.
    pub is_identity: bool,
    /// Whether all off-diagonal entries are structurally zero.
    pub is_diagonal: bool,
    /// Whether the matrix is a diagonal exact-rational scalar multiple of identity.
    ///
    /// Compares all four exposed rational diagonal values, including the
    /// homogeneous entry. Zero scale qualifies. Equal symbolic values without
    /// rational views do not qualify; no general equality decision is attempted.
    pub is_exact_rational_uniform_scale: bool,
    /// Whether all entries below the diagonal are structurally zero.
    pub is_upper_triangular: bool,
    /// Whether all entries above the diagonal are structurally zero.
    pub is_lower_triangular: bool,
    /// Whether the homogeneous affine row is structurally `[0, 0, 0, 1]`.
    pub is_affine: bool,
    /// Whether the affine linear block is structurally diagonal and unit-scaled.
    pub is_affine_translation: bool,
    /// Whether the top-left 3×3 block has structurally zero off-diagonal entries.
    /// This flag alone does not require an affine bottom row.
    pub linear_is_diagonal: bool,
    /// Whether the top-left 3×3 block is diagonal and the bottom row's xyz
    /// entries are zero. Translation and the bottom-right entry are unrestricted;
    /// a direction with homogeneous weight zero ignores them.
    pub direction_linear_is_diagonal: bool,
    /// Row-wise signed-axis certificate for a signed-permutation matrix.
    ///
    /// `Some(rows)` means every row contains exactly one signed unit, every
    /// other entry is exactly zero, and every column is used once. This is a
    /// scheduling fact for exact transform and division kernels; determinant
    /// sign and topology remain separate decisions.
    pub signed_permutation_rows: Option<[SignedAxis4; 4]>,
    /// Whether the translation column's xyz entries are structurally zero.
    pub translation_xyz_zero: [bool; 3],
    /// Conservative transform kind derived from retained homogeneous facts.
    pub transform_kind: Matrix4TransformKind,
}

impl Matrix4StructuralFacts {
    /// Returns true when every matrix entry is known to be exactly zero.
    pub fn is_zero(self) -> bool {
        self.zero_mask == matrix_mask::<4>()
    }

    /// Returns whether this matrix has a certified signed-permutation shape.
    pub fn is_signed_permutation(self) -> bool {
        self.signed_permutation_rows.is_some()
    }

    /// Returns whether one entry is structurally known to be exactly zero.
    ///
    /// `Some(false)` deliberately does not certify nonzero. It means only that
    /// the retained object facts did not prove zero cheaply, preserving the
    /// conservative exact-computation boundary described by the exactness policy.
    pub fn entry_known_zero(self, row: usize, column: usize) -> Option<bool> {
        matrix_entry_mask_value::<4>(self.zero_mask, row, column)
    }

    /// Returns whether one entry has a retained exact-one bit.
    ///
    /// `None` means an out-of-bounds index; `Some(false)` is not an inequality
    /// proof. No scalar is queried to validate the stored bit.
    pub fn entry_known_one(self, row: usize, column: usize) -> Option<bool> {
        matrix_entry_mask_value::<4>(self.one_mask, row, column)
    }

    /// Returns the zero mask for one row.
    pub fn row_zero_mask(self, row: usize) -> Option<u8> {
        self.row_zero_masks.get(row).copied()
    }

    /// Returns the zero mask for one column.
    pub fn column_zero_mask(self, column: usize) -> Option<u8> {
        self.column_zero_masks.get(column).copied()
    }

    /// Counts entries in one row that are structurally known zero.
    pub fn row_known_zero_count(self, row: usize) -> Option<u32> {
        self.row_zero_mask(row)
            .map(matrix_lane_known_zero_count::<4>)
    }

    /// Counts entries in one column that are structurally known zero.
    pub fn column_known_zero_count(self, column: usize) -> Option<u32> {
        self.column_zero_mask(column)
            .map(matrix_lane_known_zero_count::<4>)
    }

    /// Returns whether one row is structurally known to be entirely zero.
    ///
    /// This exposes the structural-zero determinant certificate as a semantic
    /// query. Matrix users should prefer this helper over decoding raw masks,
    /// keeping the mask representation local to `hyperlattice`.
    pub fn row_is_known_zero(self, row: usize) -> Option<bool> {
        self.row_zero_mask(row).map(matrix_lane_is_known_zero::<4>)
    }

    /// Returns whether one column is structurally known to be entirely zero.
    pub fn column_is_known_zero(self, column: usize) -> Option<bool> {
        self.column_zero_mask(column)
            .map(matrix_lane_is_known_zero::<4>)
    }

    /// Returns whether any row has a retained all-zero certificate.
    pub fn has_known_zero_row(self) -> bool {
        self.row_zero_masks
            .into_iter()
            .any(matrix_lane_is_known_zero::<4>)
    }

    /// Returns whether any column has a retained all-zero certificate.
    pub fn has_known_zero_column(self) -> bool {
        self.column_zero_masks
            .into_iter()
            .any(matrix_lane_is_known_zero::<4>)
    }

    /// Returns whether any row or column is certified entirely zero.
    pub fn has_known_zero_lane(self) -> bool {
        self.has_known_zero_row() || self.has_known_zero_column()
    }

    /// Returns whether at most one row entry lacks a retained zero fact.
    ///
    /// Includes all-zero rows; `None` means the row index is out of bounds.
    pub fn row_has_sparse_support(self, row: usize) -> Option<bool> {
        self.row_zero_mask(row)
            .map(matrix_lane_has_sparse_support::<4>)
    }

    /// Returns whether one column has certified sparse support.
    pub fn column_has_sparse_support(self, column: usize) -> Option<bool> {
        self.column_zero_mask(column)
            .map(matrix_lane_has_sparse_support::<4>)
    }

    /// Returns whether every row has certified sparse support.
    pub fn all_rows_have_sparse_support(self) -> bool {
        self.row_zero_masks
            .into_iter()
            .all(matrix_lane_has_sparse_support::<4>)
    }

    /// Returns whether every column has certified sparse support.
    pub fn all_columns_have_sparse_support(self) -> bool {
        self.column_zero_masks
            .into_iter()
            .all(matrix_lane_has_sparse_support::<4>)
    }

    /// Select an advisory determinant schedule from retained matrix facts.
    ///
    /// Uses the precedence of [`Matrix3StructuralFacts::determinant_schedule_hint`].
    /// A valid zero-lane fact proves singularity; other schedule names alone do
    /// not decide invertibility. Edited packets are not independently validated.
    pub fn determinant_schedule_hint(self) -> MatrixDeterminantScheduleHint {
        matrix_determinant_schedule_hint::<4>(
            self.exact,
            self.row_zero_masks,
            self.column_zero_masks,
            self.is_diagonal,
            self.is_upper_triangular,
            self.is_lower_triangular,
        )
    }
}

/// Internal 3×3 dispatch snapshot with directly accessible shape flags.
///
/// Collected by [`matrix3_facts`]; it carries no reference or mutation tracking.
#[derive(Clone, Copy, Debug)]
struct Matrix3Facts {
    /// Public snapshot returned by structural-fact queries.
    public: Matrix3StructuralFacts,
    /// Retained unit-diagonal and zero-off-diagonal fact.
    is_identity: bool,
    /// Retained zero-off-diagonal fact for the full matrix.
    is_diagonal: bool,
    /// Known zeros below the diagonal, used by triangular schedules.
    is_upper_triangular: bool,
    /// Known zeros above the diagonal, used by triangular schedules.
    is_lower_triangular: bool,
    /// Known zeros off the diagonal of the top-left homogeneous linear block.
    linear_is_diagonal: bool,
    /// Retained canonical homogeneous bottom-row fact.
    is_affine: bool,
    /// Affine shape with an identity linear block; translation is unrestricted.
    is_affine_translation: bool,
}

/// Joins two representation classes for an aggregate arithmetic schedule.
///
/// `NonRational` dominates `ExactRational`, which dominates `ExactDyadicRational`.
/// These are representation facts, not a proof of mathematical irrationality.
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

/// Classifies a 3×3 matrix’s exposed rational representations in row-major order.
///
/// Starts with the dyadic class and stops at the first `NonRational` entry;
/// remaining entries need not be inspected.
#[inline]
fn matrix3_exact_rational_kind(matrix: &[[Real; 3]; 3]) -> ExactRationalKind {
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

/// Classifies a 4×4 matrix’s exposed rational representations in row-major order.
///
/// Starts with the dyadic class and stops at the first `NonRational` entry.
fn matrix4_exact_rational_kind(matrix: &[[Real; 4]; 4]) -> ExactRationalKind {
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

/// Tests for exposed exact-rational, nonzero entries throughout a 4×4 matrix.
///
/// Visits row-major order, requiring a rational view before testing definite zero,
/// and stops at the first failure. Dense support does not imply invertibility.
#[inline]
fn matrix4_is_dense_exact_rational(matrix: &[[Real; 4]; 4]) -> bool {
    matrix
        .iter()
        .flatten()
        .all(|value| value.exact_rational_ref().is_some() && !value.definitely_zero())
}

/// Classifies rational representations of a square array in row-major order.
///
/// Joins entry classes until `NonRational` appears. The empty array retains the
/// initial dyadic class vacuously; this does not certify a nonempty rational set.
#[inline]
fn matrix_exact_rational_kind<const N: usize>(matrix: &[[Real; N]; N]) -> ExactRationalKind {
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

/// Internal 4×4 dispatch snapshot with directly accessible shape flags.
///
/// Collected by [`matrix4_facts`]; it carries no reference or mutation tracking.
#[derive(Clone, Copy, Debug)]
struct Matrix4Facts {
    /// Public snapshot returned by structural-fact queries.
    public: Matrix4StructuralFacts,
    /// Retained unit-diagonal and zero-off-diagonal fact.
    is_identity: bool,
    /// Retained zero-off-diagonal fact for the full matrix.
    is_diagonal: bool,
    /// Known zeros below the diagonal, used by triangular schedules.
    is_upper_triangular: bool,
    /// Known zeros above the diagonal, used by triangular schedules.
    is_lower_triangular: bool,
    /// Known zeros off the diagonal of the top-left homogeneous linear block.
    linear_is_diagonal: bool,
    /// Diagonal top-left 3×3 block and zero bottom-row xyz cross terms.
    direction_linear_is_diagonal: bool,
    /// Whether all first three diagonal scalars have `ZeroKnowledge::NonZero`.
    /// This does not by itself certify affine or diagonal matrix shape.
    affine_linear_diagonal_is_definitely_nonzero: bool,
    /// Definite-zero flags for entries (0,3), (1,3), and (2,3).
    translation_xyz_zero: [bool; 3],
    /// Retained canonical homogeneous bottom-row fact.
    is_affine: bool,
    /// Affine shape with an identity linear block; translation is unrestricted.
    is_affine_translation: bool,
}

/// Prioritizes identity, translation, diagonal affine, affine, then fallback.
///
/// Consumes supplied structural flags without scalar queries. `Projective` is the
/// fallback label and does not itself certify non-affinity or invertibility.
#[inline]
fn matrix3_transform_kind(
    is_identity: bool,
    is_affine: bool,
    is_affine_translation: bool,
    linear_is_diagonal: bool,
) -> Matrix3TransformKind {
    if is_identity {
        Matrix3TransformKind::Identity
    } else if is_affine_translation {
        Matrix3TransformKind::AffineTranslation
    } else if is_affine && linear_is_diagonal {
        Matrix3TransformKind::AffineDiagonalLinear
    } else if is_affine {
        Matrix3TransformKind::Affine
    } else {
        Matrix3TransformKind::Projective
    }
}

/// Prioritizes identity, signed permutation, translation, diagonal affine, affine.
///
/// Returns `Projective` if none qualifies. Supplied flags are trusted; this label
/// is not a nonsingularity proof or a proof that affine structure is impossible.
#[inline]
fn matrix4_transform_kind(
    is_identity: bool,
    is_affine: bool,
    is_affine_translation: bool,
    linear_is_diagonal: bool,
    signed_permutation_rows: Option<[SignedAxis4; 4]>,
) -> Matrix4TransformKind {
    if is_identity {
        Matrix4TransformKind::Identity
    } else if signed_permutation_rows.is_some() {
        Matrix4TransformKind::SignedPermutation
    } else if is_affine_translation {
        Matrix4TransformKind::AffineTranslation
    } else if is_affine && linear_is_diagonal {
        Matrix4TransformKind::AffineDiagonalLinear
    } else if is_affine {
        Matrix4TransformKind::Affine
    } else {
        Matrix4TransformKind::Projective
    }
}

/// Collects a 3×3 structural snapshot for matrix dispatch and public inspection.
///
/// A fixed entry scan supplies diagonal and homogeneous shape flags. Separate
/// passes collect rational-set facts, zero masks, one masks, and symbolic
/// dependencies; rational diagonal comparison supplies uniform-scale recognition.
/// These are conservative retained facts, not a general equality decision. The
/// returned packet is not bound to future mutations of the input.
#[inline]
fn matrix3_facts(matrix: &[[Real; 3]; 3]) -> Matrix3Facts {
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
    // Derive triangular flags from the same local shape facts without adding
    // scalar probes here; the complete snapshot uses separate passes below.
    let is_upper_triangular = m10_zero && m20_zero && m21_zero;
    let is_lower_triangular = m01_zero && m02_zero && m12_zero;
    // Reuse the retained linear diagonal fact instead of probing m01/m10 a
    // second time. The predicate is identical, but affine 2D batch transforms
    // and inverse/division dispatch stay flatter by carrying the cheap
    // structural fact forward.
    let is_affine_translation = is_affine && m00_one && m11_one && linear_is_diagonal;
    let exact = crate::kernels::exact_real_set_facts(matrix.iter().flat_map(|row| row.iter()));
    let (zero_mask, row_zero_masks, column_zero_masks) = matrix_zero_masks(matrix);
    let one_mask = matrix_one_mask(matrix);
    let transform_kind = matrix3_transform_kind(
        is_identity,
        is_affine,
        is_affine_translation,
        linear_is_diagonal,
    );
    let public = Matrix3StructuralFacts {
        exact,
        symbolic_dependencies: matrix_symbolic_dependency_mask(matrix),
        zero_mask,
        one_mask,
        row_zero_masks,
        column_zero_masks,
        is_identity,
        is_diagonal,
        is_exact_rational_uniform_scale: matrix3_exact_rational_uniform_scale(matrix, is_diagonal),
        is_upper_triangular,
        is_lower_triangular,
        is_affine,
        is_affine_translation,
        transform_kind,
    };

    Matrix3Facts {
        public,
        is_identity,
        is_diagonal,
        is_upper_triangular,
        is_lower_triangular,
        linear_is_diagonal,
        is_affine,
        is_affine_translation,
    }
}

/// Collects a 4×4 structural snapshot for matrix dispatch and public inspection.
///
/// The initial fixed scan supplies shape and translation flags. A short-circuit
/// `zero_status` scan separately checks the first three diagonal entries for known
/// nonzero values, even if affine or diagonal shape is absent. Further passes
/// collect rational-set facts, entry masks, signed-permutation rows, symbolic
/// dependencies, and rational uniform scale. No single-pass or mutation-tracking
/// guarantee is made.
#[inline]
fn matrix4_facts(matrix: &[[Real; 4]; 4]) -> Matrix4Facts {
    // Scan entries once for the shape flags below. The complete snapshot also
    // includes the separate scalar-set, mask, and signed-axis passes below.
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
    // Derive triangular flags from the initial scan; later mask and scalar-set
    // collection still makes separate passes over the entries.
    let is_upper_triangular = m10_zero && m20_zero && m30_zero && m21_zero && m31_zero && m32_zero;
    let is_lower_triangular = m01_zero && m02_zero && m03_zero && m12_zero && m13_zero && m23_zero;
    let is_identity = is_diagonal && m00_one && m11_one && m22_one && m33_one;
    let is_affine = m30_zero && m31_zero && m32_zero && m33_one;
    let is_affine_translation = is_affine && m00_one && m11_one && m22_one && linear_is_diagonal;
    let affine_linear_diagonal_is_definitely_nonzero =
        matches!(matrix[0][0].zero_status(), ZeroKnowledge::NonZero)
            && matches!(matrix[1][1].zero_status(), ZeroKnowledge::NonZero)
            && matches!(matrix[2][2].zero_status(), ZeroKnowledge::NonZero);
    let exact = crate::kernels::exact_real_set_facts(matrix.iter().flat_map(|row| row.iter()));
    let (zero_mask, row_zero_masks, column_zero_masks) = matrix_zero_masks(matrix);
    let one_mask = matrix_one_mask(matrix);
    let translation_xyz_zero = [m03_zero, m13_zero, m23_zero];
    let signed_permutation_rows = matrix4_signed_permutation_rows(matrix);
    let transform_kind = matrix4_transform_kind(
        is_identity,
        is_affine,
        is_affine_translation,
        linear_is_diagonal,
        signed_permutation_rows,
    );
    let public = Matrix4StructuralFacts {
        exact,
        symbolic_dependencies: matrix_symbolic_dependency_mask(matrix),
        zero_mask,
        one_mask,
        row_zero_masks,
        column_zero_masks,
        is_identity,
        is_diagonal,
        is_exact_rational_uniform_scale: matrix4_exact_rational_uniform_scale(matrix, is_diagonal),
        is_upper_triangular,
        is_lower_triangular,
        is_affine,
        is_affine_translation,
        linear_is_diagonal,
        direction_linear_is_diagonal,
        signed_permutation_rows,
        translation_xyz_zero,
        transform_kind,
    };

    Matrix4Facts {
        public,
        is_identity,
        is_diagonal,
        is_upper_triangular,
        is_lower_triangular,
        linear_is_diagonal,
        direction_linear_is_diagonal,
        translation_xyz_zero,
        is_affine,
        is_affine_translation,
        affine_linear_diagonal_is_definitely_nonzero,
    }
}

/// Minimal retained shape packet for an immediate 4D matrix-vector product.
///
/// Carries only facts consumed by that dispatch, with no matrix reference,
/// mutation tracking, rational-set facts, or signed-permutation recognition.
#[derive(Clone, Copy)]
struct Matrix4TransformDispatchFacts {
    /// Whether all off-diagonals are definite zeros and all diagonal entries ones.
    is_identity: bool,
    /// Whether the entire 4×4 off-diagonal is definitely zero.
    is_diagonal: bool,
    /// Whether the bottom row is definitely `[0, 0, 0, 1]`.
    is_affine: bool,
    /// Whether the top-left 3×3 off-diagonal is definitely zero, regardless of bottom row.
    linear_is_diagonal: bool,
    /// Whether the linear block is diagonal and bottom-row xyz entries vanish.
    direction_linear_is_diagonal: bool,
    /// Definite-zero flags for the top three entries of the final column.
    translation_xyz_zero: [bool; 3],
}

/// Collects only the shape facts used by immediate homogeneous-vector dispatch.
///
/// Requires `N == 4`; the debug assertion is not a typed dimension check. Probes
/// linear off-diagonals, bottom-row xyz, then all three translation entries, with
/// short-circuiting within the first two groups. Identity and affine tests then
/// query diagonal ones as needed. No rational-set or dependency summary is built.
/// False flags mean missing structural knowledge, not proofs of the opposite.
#[inline]
fn matrix4_transform_dispatch_facts<const N: usize>(
    matrix: &[[Real; N]; N],
) -> Matrix4TransformDispatchFacts {
    debug_assert_eq!(N, 4);
    let linear_is_diagonal = matrix[0][1].definitely_zero()
        && matrix[0][2].definitely_zero()
        && matrix[1][0].definitely_zero()
        && matrix[1][2].definitely_zero()
        && matrix[2][0].definitely_zero()
        && matrix[2][1].definitely_zero();
    let bottom_xyz_zero = matrix[3][0].definitely_zero()
        && matrix[3][1].definitely_zero()
        && matrix[3][2].definitely_zero();
    let translation_xyz_zero = [
        matrix[0][3].definitely_zero(),
        matrix[1][3].definitely_zero(),
        matrix[2][3].definitely_zero(),
    ];
    let is_diagonal =
        linear_is_diagonal && bottom_xyz_zero && translation_xyz_zero.iter().all(|zero| *zero);
    let is_identity = is_diagonal
        && matrix[0][0].definitely_one()
        && matrix[1][1].definitely_one()
        && matrix[2][2].definitely_one()
        && matrix[3][3].definitely_one();
    let is_affine = bottom_xyz_zero && matrix[3][3].definitely_one();
    Matrix4TransformDispatchFacts {
        is_identity,
        is_diagonal,
        is_affine,
        linear_is_diagonal,
        direction_linear_is_diagonal: linear_is_diagonal && bottom_xyz_zero,
        translation_xyz_zero,
    }
}

/// Applies an owned-left/borrowed-right scalar operation in ascending lane order.
///
/// Equal lengths follow from the array types, justifying the iterator `expect`.
/// The callback runs exactly `N` times, with no calls for an empty array. This
/// helper adds no cloning, arithmetic, or validation beyond the supplied callback.
fn map_array_ref<const N: usize, F>(left: [Real; N], right: &[Real; N], mut op: F) -> [Real; N]
where
    F: FnMut(Real, &Real) -> Real,
{
    let mut right = right.iter();
    left.map(|lhs| op(lhs, right.next().expect("arrays have equal length")))
}

/// Maps corresponding entries in row-major order, consuming only the left matrix.
///
/// The same mutable callback is reused across rows and receives each right entry
/// by reference. Const-generic array sizes guarantee equal row and lane counts;
/// empty matrices make no callback calls.
fn map_matrix_ref<const N: usize, F>(
    left: [[Real; N]; N],
    right: &[[Real; N]; N],
    mut op: F,
) -> [[Real; N]; N]
where
    F: FnMut(Real, &Real) -> Real,
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

/// Maps corresponding entries in row-major order, consuming only the right matrix.
///
/// Preserves operand order for noncommutative callbacks such as subtraction.
/// Matching array sizes justify the iterator `expect` calls; no entries are cloned
/// by this helper and an empty matrix makes no callback calls.
fn map_matrix_left_ref<const N: usize, F>(
    left: &[[Real; N]; N],
    right: [[Real; N]; N],
    mut op: F,
) -> [[Real; N]; N]
where
    F: FnMut(&Real, Real) -> Real,
{
    let mut left = left.iter();
    right.map(|rhs_row| {
        let mut left_row = left.next().expect("matrices have equal row counts").iter();
        rhs_row.map(|rhs| op(left_row.next().expect("arrays have equal length"), rhs))
    })
}

/// Fixed-size dispatch hook for entrywise negation of owned matrix arrays.
///
/// Keeps public matrix operators on the explicit 3×3/4×4 row schedules.
trait MatrixNegOwned: Sized {
    /// Negates entries in row-major order, consuming the input matrix.
    fn neg_owned(self) -> Self;
}

/// Fixed-size dispatch hook for entrywise negation of borrowed matrix arrays.
///
/// Keeps public matrix operators on the explicit 3×3/4×4 row schedules.
trait MatrixNegRefs: Sized {
    /// Negates entries in row-major order, borrowing the input matrix.
    fn neg_refs(&self) -> Self;
}

/// Negates three owned scalars from left to right, moving each into scalar negation.
#[inline]
fn neg_row3_owned(row: [Real; 3]) -> [Real; 3] {
    let [x, y, z] = row;
    [-x, -y, -z]
}

/// Negates three borrowed scalars from left to right through borrowed scalar negation.
#[inline]
fn neg_row3_refs(row: &[Real; 3]) -> [Real; 3] {
    [-&row[0], -&row[1], -&row[2]]
}

/// Negates four owned scalars from left to right, moving each into scalar negation.
#[inline]
fn neg_row4_owned(row: [Real; 4]) -> [Real; 4] {
    let [x, y, z, w] = row;
    [-x, -y, -z, -w]
}

/// Negates four borrowed scalars from left to right through borrowed scalar negation.
#[inline]
fn neg_row4_refs(row: &[Real; 4]) -> [Real; 4] {
    [-&row[0], -&row[1], -&row[2], -&row[3]]
}

impl MatrixNegOwned for [[Real; 3]; 3] {
    #[inline]
    /// Negates entries in row-major order, consuming the input matrix.
    fn neg_owned(self) -> Self {
        let [x, y, z] = self;
        [neg_row3_owned(x), neg_row3_owned(y), neg_row3_owned(z)]
    }
}

impl MatrixNegRefs for [[Real; 3]; 3] {
    #[inline]
    /// Negates entries in row-major order, borrowing the input matrix.
    fn neg_refs(&self) -> Self {
        [
            neg_row3_refs(&self[0]),
            neg_row3_refs(&self[1]),
            neg_row3_refs(&self[2]),
        ]
    }
}

impl MatrixNegOwned for [[Real; 4]; 4] {
    #[inline]
    /// Negates entries in row-major order, consuming the input matrix.
    fn neg_owned(self) -> Self {
        let [x, y, z, w] = self;
        [
            neg_row4_owned(x),
            neg_row4_owned(y),
            neg_row4_owned(z),
            neg_row4_owned(w),
        ]
    }
}

impl MatrixNegRefs for [[Real; 4]; 4] {
    #[inline]
    /// Negates entries in row-major order, borrowing the input matrix.
    fn neg_refs(&self) -> Self {
        [
            neg_row4_refs(&self[0]),
            neg_row4_refs(&self[1]),
            neg_row4_refs(&self[2]),
            neg_row4_refs(&self[3]),
        ]
    }
}

/// Raises a square array to a nonnegative power using the supplied multiplication.
///
/// The callback must implement associative matrix multiplication compatible with
/// the exact identity array. Exponents zero and one return identity and the base
/// without multiplying; four squares twice. Other exponents use binary squaring,
/// seeding the accumulator at the first set bit instead of multiplying by identity.
/// For positive `e`, callback count is `floor(log2(e)) + popcount(e) - 1`.
///
/// The positive-exponent invariant justifies the final `expect`. Scalar handles
/// are cloned to reuse factors; no inversion, cancellation polling, or scalar-work
/// budget is supplied here. A logarithmic callback count is not a total-work bound.
#[inline]
fn matrix_power_with<const N: usize, F>(
    base: [[Real; N]; N],
    exponent: u32,
    mut multiply: F,
) -> [[Real; N]; N]
where
    F: FnMut([[Real; N]; N], [[Real; N]; N]) -> [[Real; N]; N],
{
    // Alternative researched paths for fixed 3x3/4x4 powers included
    // Cayley-Hamilton with Faddeev-LeVerrier coefficients and Berkowitz-style
    // division-free characteristic polynomials. For the small exponents
    // that dominate this crate's matrix benches, those approaches introduce
    // trace/determinant reductions before they can save a multiply. Keep powers
    // on repeated squaring and put the optimization budget into the fixed-size
    // multiply kernels below. 2026-05 targeted Criterion:
    // hyperreal-from-f64 mat3/mat4 powi moved from ~6.30/11.39 us to
    // ~5.98/10.71 us. Hyperreal-rational powi stayed within the normal
    // Criterion noise band, so this keeps hyperreal's per-cell exact-rational
    // denominator schedule.
    //
    // Keep this helper and the fixed multiply helpers inline for downstream
    // benchmark crates. A post-full-suite 200-sample/8s pass found hyperreal
    // mat3/mat4 borrowed multiply improved by ~4.98%/~4.54% after inlining the
    // helper layers.
    match exponent {
        0 => return identity_array(),
        1 => return base,
        // Low exponents dominate transform/matrix helper use. The fixed-size
        // callers handle 2 and 3 before entering this generic loop.
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

/// Dispatches nonnegative 3×3 powers to borrowed square/cube or binary squaring.
///
/// For two and three, checks every base entry for known nonzero status before
/// choosing a dense dot-product kernel. A cube rechecks the newly computed square
/// before its final multiplication: a dense base can have a sparse or zero square.
/// Other exponents use [`matrix_power_with`] with the ordinary owned kernel.
/// Dense recognition does not require rational entries or imply invertibility.
#[inline]
fn matrix_power3(base: [[Real; 3]; 3], exponent: u32) -> [[Real; 3]; 3] {
    trace_matrix!("helper", "matrix-power3-fixed-mul");
    // The hot small positive powers can square the existing base by reference,
    // then consume only the fresh square. This keeps exact matrix powers on
    // object-level reuse instead of cloning the base into hot multiply lanes.
    if exponent == 2 {
        trace_matrix!("helper", "matrix-power3-borrowed-square");
        if matrix3_has_dense_multiply_certificate(&base) {
            trace_matrix!("helper", "matrix-power3-dense-certified-square");
            return multiply_arrays3_dense_ref(&base, &base);
        }
        return multiply_arrays3_ref(&base, &base);
    }
    if exponent == 3 {
        trace_matrix!("helper", "matrix-power3-borrowed-cube");
        if matrix3_has_dense_multiply_certificate(&base) {
            trace_matrix!("helper", "matrix-power3-dense-certified-cube");
            let square = multiply_arrays3_dense_ref(&base, &base);
            return multiply_arrays3_rhs_ref_with_exact_dense_certificate(square, &base);
        }
        let square = multiply_arrays3_ref(&base, &base);
        return multiply_arrays3_rhs_ref(square, &base);
    }
    matrix_power_with(base, exponent, multiply_arrays3)
}

/// Dispatches nonnegative 4×4 powers to borrowed square/cube or binary squaring.
///
/// Known-nonzero entries select the dense square path. Cubing rechecks both operands
/// after squaring instead of inheriting density from the base. Other exponents use
/// [`matrix_power_with`]; negative powers must be inverted by the caller first.
#[inline]
fn matrix_power4(base: [[Real; 4]; 4], exponent: u32) -> [[Real; 4]; 4] {
    trace_matrix!("helper", "matrix-power4-fixed-mul");
    // Same borrowed square/cube schedule as 3x3; the mat4 powi benchmark is
    // particularly sensitive to avoiding owned base duplication before the
    // fixed multiply kernel has a chance to reuse structural facts.
    if exponent == 2 {
        trace_matrix!("helper", "matrix-power4-borrowed-square");
        if matrix4_has_dense_multiply_certificate(&base) {
            trace_matrix!("helper", "matrix-power4-dense-certified-square");
            return multiply_arrays4_dense_ref(&base, &base);
        }
        return multiply_arrays4_ref(&base, &base);
    }
    if exponent == 3 {
        trace_matrix!("helper", "matrix-power4-borrowed-cube");
        if matrix4_has_dense_multiply_certificate(&base) {
            trace_matrix!("helper", "matrix-power4-dense-certified-cube");
            let square = multiply_arrays4_dense_ref(&base, &base);
            return multiply_arrays4_rhs_ref_with_dense_certificate(square, &base);
        }
        let square = multiply_arrays4_ref(&base, &base);
        return multiply_arrays4_rhs_ref(square, &base);
    }
    matrix_power_with(base, exponent, multiply_arrays4)
}

/// Selects a pivot from rows `col..N`, preferring known nonzero over unknown.
///
/// The caller must supply `col < N`. The diagonal is queried first, then lower
/// rows in order. Returns immediately on the first known nonzero; otherwise keeps
/// the first unknown candidate and returns `None` only when all candidates are
/// known zero. Selection itself neither refines a sign nor proves an unknown
/// pivot safe to invert; the subsequent scalar inverse retains that responsibility.
fn ordinary_pivot<const N: usize>(left: &[[Real; N]; N], col: usize) -> Option<usize> {
    let mut unknown = None;
    match Real::zero_status(&left[col][col]) {
        ZeroKnowledge::NonZero => return Some(col),
        ZeroKnowledge::Unknown => unknown = Some(col),
        ZeroKnowledge::Zero => {}
    }

    for (row, values) in left.iter().enumerate().skip(col + 1) {
        match Real::zero_status(&values[col]) {
            ZeroKnowledge::NonZero => return Some(row),
            ZeroKnowledge::Unknown if unknown.is_none() => unknown = Some(row),
            ZeroKnowledge::Zero | ZeroKnowledge::Unknown => {}
        }
    }

    unknown
}

/// Selects the first known-nonzero entry in column `col`, scanning rows `col..N`.
///
/// An earlier unknown does not prevent a later known-nonzero pivot. If none is
/// known nonzero, returns `UnknownZero` when any candidate is unknown, otherwise
/// `DivideByZero`. This helper performs no refinement or cancellation polling.
/// Solve callers use `col < N`; an out-of-range column has no candidate rows and
/// also returns `DivideByZero`, rather than a separate index-validation error.
fn checked_pivot<const N: usize>(left: &[[Real; N]; N], col: usize) -> CheckedBlasResult<usize> {
    let mut has_unknown = false;
    for (row, values) in left.iter().enumerate().skip(col) {
        match values[col].zero_status() {
            ZeroKnowledge::NonZero => return Ok(row),
            ZeroKnowledge::Unknown => has_unknown = true,
            ZeroKnowledge::Zero => {}
        }
    }

    if has_unknown {
        Err(Problem::UnknownZero)
    } else {
        Err(Problem::DivideByZero)
    }
}

/// Moves an entry into cached scalar multiplication by a borrowed factor.
///
/// Temporarily replaces the slot with exact zero, then stores the product; this
/// is an ownership operation, not a proof that the entry is zero. It supplies no
/// rollback guarantee if scalar arithmetic panics.
fn scale_entry_in_place(value: &mut Real, factor: &Real) {
    let current = mem::replace(value, Real::zero());
    *value = current.mul_cached(factor);
}

/// Updates an entry as `value - pivot * factor`, borrowing both multiplier inputs.
///
/// Moves the old value out through a temporary exact-zero slot, then stores the
/// scalar result. Elimination owns these buffers; this helper is not transactional
/// and introduces no separate scalar error or cancellation boundary.
fn subtract_scaled_entry_in_place(value: &mut Real, pivot: &Real, factor: &Real) {
    let current = mem::replace(value, Real::zero());
    // Keep both `pivot` and `factor` borrowed. The old form cloned `pivot`
    // before multiplying, which is expensive for hyperreal-backed matrices.
    *value = current - pivot * factor;
}

/// Generates fixed-size Gauss–Jordan solvers for `coefficients * X = rhs`.
///
/// Instantiated for three and four right-hand sides in dimensions three and four.
/// Each step chooses a pivot row, swaps both buffers, normalizes the pivot row,
/// then eliminates that column from every other row unless its factor is a
/// definite zero. Only the right buffer is returned; errors publish no partial
/// solution. All row mutations are local to the consumed arrays.
///
/// Ordinary selection may attempt an unknown pivot; checked selection requires
/// known nonzero status. The abort-aware checked variant attaches the supplied
/// signal to each pivot reciprocal, without polling at loop boundaries. Scalar
/// work and cancellation remain delegated to `Real`; fixed loop sizes do not
/// bound coefficient growth, expression size, refinement, or total work.
macro_rules! impl_solve_left_system_fixed {
    (
        $solve_fn:ident,
        $solve_checked_fn:ident,
        $solve_abort_fn:ident,
        $n:expr
    ) => {
        /// Solves `coefficients * X = rhs` with ordinary pivot selection.
        ///
        /// Prefers the first known-nonzero pivot, otherwise tries the first unknown
        /// candidate. An all-known-zero column returns `DivideByZero`; scalar inverse
        /// errors propagate. Unknown pivots are not silently treated as zeros.
        fn $solve_fn(
            coefficients: [[Real; $n]; $n],
            rhs: [[Real; $n]; $n],
        ) -> BlasResult<[[Real; $n]; $n]> {
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

                // Move the pivot out once and install the normalized unit
                // diagonal before row elimination, avoiding an extra clone for
                // the inverse path. A structural unit-pivot bypass was tested
                // here and reverted: the extra `definitely_one` query regressed
                // mat3/mat4 right-division rows more than it saved in skipped
                // inverses. Keep the straight-line normalization schedule.
                let pivot = mem::replace(&mut left[col][col], Real::one());
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
                    let factor = mem::replace(&mut left[row][col], Real::zero());
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

        /// Solves `coefficients * X = rhs`, requiring known-nonzero pivots.
        ///
        /// Without a known pivot, an unknown candidate returns `UnknownZero`; an
        /// all-known-zero column returns `DivideByZero`. Scalar inverse errors also
        /// propagate, and only a completed right-hand-side buffer is returned.
        fn $solve_checked_fn(
            coefficients: [[Real; $n]; $n],
            rhs: [[Real; $n]; $n],
        ) -> CheckedBlasResult<[[Real; $n]; $n]> {
            let mut left = coefficients;
            let mut right = rhs;

            for col in 0..$n {
                let pivot = checked_pivot(&left, col)?;
                if pivot != col {
                    left.swap(col, pivot);
                    right.swap(col, pivot);
                }

                // Keep checked solve on the same move-based pivot schedule as the
                // non-checked variant so checked kernels don’t pay extra slot
                // churn. A failing checked inverse still returns before mutation
                // of result rows beyond the local copy.
                let pivot = mem::replace(&mut left[col][col], Real::one());
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
                    let factor = mem::replace(&mut left[row][col], Real::zero());
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

        /// Solves `coefficients * X = rhs` with checked pivots and reciprocal signal attachment.
        ///
        /// Pivot selection occurs before cloning and attaching the signal to that
        /// pivot for inversion. The helper does not explicitly poll the flag per row
        /// or attach it to every input/output entry. Exact shortcuts may finish
        /// without consulting the flag; this is cooperative cancellation, not a
        /// deadline or a guarantee that an already-set flag prevents success.
        fn $solve_abort_fn(
            coefficients: [[Real; $n]; $n],
            rhs: [[Real; $n]; $n],
            signal: &AbortSignal,
        ) -> CheckedBlasResult<[[Real; $n]; $n]> {
            let mut left = coefficients;
            let mut right = rhs;

            for col in 0..$n {
                let pivot = checked_pivot(&left, col)?;
                if pivot != col {
                    left.swap(col, pivot);
                    right.swap(col, pivot);
                }

                let pivot = mem::replace(&mut left[col][col], Real::one());
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
                    let factor = mem::replace(&mut left[row][col], Real::zero());
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

/// Selects shared-adjugate arithmetic from exposed rational representation classes.
///
/// Classifies the divisor first and rejects `NonRational`. For 4×4, any rational
/// divisor qualifies without inspecting the dividend. For 3×3, a non-dyadic rational
/// divisor also skips the dividend scan; a dyadic divisor requires rational entries
/// in both operands. Other dimensions use the combined-class rule.
///
/// This is a schedule heuristic, not an invertibility or common-denominator proof.
/// The selected arithmetic path must still handle a zero or unresolved determinant.
fn prefer_shared_adjugate_right_division<const N: usize>(
    left: &[[Real; N]; N],
    right: &[[Real; N]; N],
) -> bool {
    // Inspect the divisor first. Exact-rational divisors can select this
    // schedule without requiring a rational dividend on the branches below.
    // Determinant validation remains the selected arithmetic path's job.
    let right_kind = matrix_exact_rational_kind(right);
    if right_kind == ExactRationalKind::NonRational {
        return false;
    }
    if N == 4 {
        trace_matrix!("helper", "right-divide4-exact-right-skip-left-kind");
        return true;
    }
    if N == 3 && right_kind == ExactRationalKind::ExactRational {
        trace_matrix!("helper", "right-divide3-exact-right-skip-left-kind");
        return true;
    }
    let left_kind = matrix_exact_rational_kind(left);
    matches!(
        combine_exact_rational_kind(left_kind, right_kind),
        ExactRationalKind::ExactDyadicRational | ExactRationalKind::ExactRational
    )
}

/// Selects the borrowed 3×3 shared-adjugate schedule, inspecting the divisor first.
///
/// A non-rational representation rejects the schedule; a non-dyadic rational
/// divisor accepts without scanning the dividend. Dyadic divisors require a rational
/// dividend too. No determinant or singularity decision is made here.
fn prefer_shared_adjugate_right_division_ref3(
    left: &[[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> bool {
    let right_kind = matrix3_exact_rational_kind(right);
    if right_kind == ExactRationalKind::NonRational {
        return false;
    }
    if right_kind == ExactRationalKind::ExactRational {
        trace_matrix!("helper", "right-divide3-ref-exact-right-skip-left-kind");
        return true;
    }
    let left_kind = matrix3_exact_rational_kind(left);
    matches!(
        combine_exact_rational_kind(left_kind, right_kind),
        ExactRationalKind::ExactDyadicRational | ExactRationalKind::ExactRational
    )
}

/// Recognizes diagonal action on vectors with homogeneous weight zero.
///
/// Checks the six off-diagonal entries of the top-left 3×3 block, then the three
/// bottom-row xyz entries, stopping on the first missing definite-zero fact.
/// The translation column and bottom-right value are unrestricted because their
/// contributions are multiplied by zero; affine shape is not required.
#[inline]
fn matrix4_direction_linear_is_diagonal(matrix: &[[Real; 4]; 4]) -> bool {
    // Direction vectors have w = 0, so the translation column cannot contribute
    // to the result. This retained geometric fact lets translated diagonal affine
    // transforms use the same component-wise scale path as true diagonal
    // matrices without changing point or unknown-w behavior. This is the
    // projective point/direction split used in exact geometric computation; see
    // the exact object-structure policy
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

/// Conservative schedule for the action on homogeneous vectors with weight zero.
///
/// Translation and the bottom-right scalar do not participate in this classification.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Matrix4DirectionLinearKind {
    /// Known unit top-left diagonal, zero cross terms, and zero bottom-row xyz.
    Identity,
    /// Known diagonal direction action, without a complete unit-diagonal certificate.
    Diagonal,
    /// Direction-diagonal shape was not certified; use the general action.
    General,
}

/// Classifies the action on homogeneous directions without inspecting translation.
///
/// Checks direction-diagonal shape first, then the first three diagonal entries
/// for definite ones. Unknown shape selects `General`; missing unit facts select
/// `Diagonal` once the zero pattern is known. No non-affinity or invertibility
/// claim follows from these fallback labels.
#[inline]
fn matrix4_direction_linear_kind(matrix: &[[Real; 4]; 4]) -> Matrix4DirectionLinearKind {
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

/// Recognizes an affine matrix with a diagonal top-left 3×3 linear block.
///
/// Checks six linear off-diagonals and bottom-row xyz for definite zeros, then
/// requires a definite one at (3,3). Translation is unrestricted and the linear
/// diagonal may contain zeros, so this fact does not establish invertibility.
#[inline]
fn matrix4_affine_linear_is_diagonal(matrix: &[[Real; 4]; 4]) -> bool {
    // Narrow one-shot point predicate: this is cheaper than `matrix4_facts`
    // when the caller only needs the affine-linear-diagonal fast path. Keep it
    // out of cached paths, where retained `Matrix4Facts` are already
    // available. Targeted sentinel runs showed the public point transform
    // regressed after broad fact collection, while retained batch facts stayed flat.
    // the exact object-structure policy
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

/// Recognizes three nonzero witnesses that exclude the 3×3 shape shortcuts.
///
/// Queries (1,0), (0,1), then (2,0), stopping unless each has known nonzero status.
/// Together they exclude upper/lower triangular and affine shapes, hence identity
/// and diagonal shapes too. Despite the name, not every entry is tested for density,
/// and determinant nonzero is not established. Unknown falls back to shape scans.
#[inline]
fn matrix3_is_definitely_dense_for_inverse(matrix: &[[Real; 3]; 3]) -> bool {
    // Dense inverse benchmarks regressed after broad retained-fact scans were
    // added for sparse/affine wins. These three nonzero certificates are a
    // deliberately conservative escape hatch: `m10 != 0` rules out upper
    // triangular, `m01 != 0` rules out lower triangular, and `m20 != 0` rules
    // out affine form; together they also rule out diagonal/identity. When any
    // certificate is unknown, fall back to the full fact scan so exact geometry
    // paths keep their structural reductions. This preserves the object-level
    // structure principle the exact object-structure policy while
    // keeping dense cofactor kernels thin.
    matches!(matrix[1][0].zero_status(), ZeroKnowledge::NonZero)
        && matches!(matrix[0][1].zero_status(), ZeroKnowledge::NonZero)
        && matches!(matrix[2][0].zero_status(), ZeroKnowledge::NonZero)
}

/// Recognizes three nonzero witnesses that exclude the 4×4 shape shortcuts.
///
/// Queries (1,0), (0,1), then (3,0) in that order. These exclude upper/lower
/// triangular and affine shapes, but do not prove full entry density or
/// invertibility. A missing nonzero fact rejects this shortcut conservatively.
#[inline]
fn matrix4_is_definitely_dense_for_inverse(matrix: &[[Real; 4]; 4]) -> bool {
    // Same dense-first guard as the 3x3 path. `m10 != 0` rejects upper
    // triangular, `m01 != 0` rejects lower triangular, and `m30 != 0` rejects
    // affine/homogeneous structure; diagonal and identity are subsets of the
    // triangular/affine structures already ruled out. The guard uses only three
    // cheap structural facts and never approximates, matching the exact
    // geometric-computation rule of exploiting structure only when it is known.
    // the exact object-structure policy
    matches!(matrix[1][0].zero_status(), ZeroKnowledge::NonZero)
        && matches!(matrix[0][1].zero_status(), ZeroKnowledge::NonZero)
        && matches!(matrix[3][0].zero_status(), ZeroKnowledge::NonZero)
}

/// Checks every 3×3 entry for known nonzero status in row-major order.
///
/// Stops at the first zero or unknown. This is full support certification for the
/// dense multiplication kernel, independent of rationality or invertibility; it
/// must be re-established for products that can acquire exact zeros.
#[inline]
fn matrix3_has_dense_multiply_certificate(matrix: &[[Real; 3]; 3]) -> bool {
    // The direct reducer is valid only for matrices that are actually dense.
    // Checking every lane here replaces the sparse-aware multiply's identical
    // zero scan; a few nonzero samples are not enough because exact zeros need
    // the zero-pruned reducer to remain schedule-independent.
    matrix
        .iter()
        .flatten()
        .all(|value| matches!(value.zero_status(), ZeroKnowledge::NonZero))
}

/// Checks every 4×4 entry for known nonzero status in row-major order.
///
/// Stops at the first zero or unknown. No rationality, determinant, or density of
/// a later product is implied by a successful scan.
#[inline]
fn matrix4_has_dense_multiply_certificate(matrix: &[[Real; 4]; 4]) -> bool {
    matrix
        .iter()
        .flatten()
        .all(|value| matches!(value.zero_status(), ZeroKnowledge::NonZero))
}

/// Multiplies borrowed matrices after checking left, then right, for dense support.
///
/// Both scans must succeed to use the dense dot-product kernel. Otherwise the
/// ordinary borrowed kernel retains its structural and zero-pruning shortcuts.
/// The right scan is skipped if the left fails.
#[inline]
fn multiply_arrays4_ref_with_dense_certificate(
    left: &[[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> [[Real; 4]; 4] {
    if matrix4_has_dense_multiply_certificate(left) && matrix4_has_dense_multiply_certificate(right)
    {
        trace_matrix!("helper", "multiply4-dense-certified-ref");
        return multiply_arrays4_dense_ref(left, right);
    }
    multiply_arrays4_ref(left, right)
}

/// Multiplies an owned left and borrowed right matrix with fresh density checks.
///
/// Checks the left operand first; only if it succeeds is the right checked. Dense
/// inputs are borrowed for dot products; failure falls back to the ordinary
/// owned-left/borrowed-right kernel. A prior dense base does not certify its square.
#[inline]
fn multiply_arrays4_rhs_ref_with_dense_certificate(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> [[Real; 4]; 4] {
    if matrix4_has_dense_multiply_certificate(&left)
        && matrix4_has_dense_multiply_certificate(right)
    {
        trace_matrix!("helper", "multiply4-dense-certified-owned-ref");
        return multiply_arrays4_dense_ref(&left, right);
    }
    multiply_arrays4_rhs_ref(left, right)
}

/// Inverts a caller-established affine matrix `[D t; 0 1]` with diagonal `D`.
///
/// Computes the three diagonal reciprocals in axis order, then returns
/// `[D⁻¹ -D⁻¹ t; 0 1]`. Shape is not revalidated; off-diagonal and bottom-row
/// entries are replaced by the required canonical values. Scalar inverse errors
/// propagate before any result is returned. Unknown-zero diagonals are left to
/// the ordinary scalar inverse rather than rejected by a separate checked guard.
#[inline]
fn invert_matrix4_affine_linear_diagonal(matrix: &[[Real; 4]; 4]) -> BlasResult<[[Real; 4]; 4]> {
    // For diagonal affine linear blocks, invert is three scalar reciprocals and
    // three affine correction multiplies. This is the 3D axis-aligned case of the
    // block affine inverse used in `Matrix4::` transforms.
    // Inexact numeric code can treat this as a per-axis rescaling; exact code
    // avoids the full 3×3 determinant path and keeps reciprocal scheduling flat.
    let inv00 = matrix[0][0].clone().inverse()?;
    let inv11 = matrix[1][1].clone().inverse()?;
    let inv22 = matrix[2][2].clone().inverse()?;
    let inv_tx = Real::zero() - (&matrix[0][3] * &inv00);
    let inv_ty = Real::zero() - (&matrix[1][3] * &inv11);
    let inv_tz = Real::zero() - (&matrix[2][3] * &inv22);

    Ok([
        [inv00, Real::zero(), Real::zero(), inv_tx],
        [Real::zero(), inv11, Real::zero(), inv_ty],
        [Real::zero(), Real::zero(), inv22, inv_tz],
        [Real::zero(), Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Checks diagonal nonzero knowledge before the diagonal-affine inverse formula.
///
/// Requires caller-established `[D t; 0 1]` shape. Guards entries (0,0), (1,1),
/// then (2,2), stopping at the first `DivideByZero` or `UnknownZero`; all three
/// guards precede reciprocal construction. Then delegates to
/// [`invert_matrix4_affine_linear_diagonal`] and propagates scalar errors.
#[inline]
fn invert_matrix4_affine_linear_diagonal_checked(
    matrix: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    require_known_nonzero(&matrix[0][0])?;
    require_known_nonzero(&matrix[1][1])?;
    require_known_nonzero(&matrix[2][2])?;
    invert_matrix4_affine_linear_diagonal(matrix)
}

/// Inverts an affine matrix using retained diagonal/translation shape flags.
///
/// The caller must establish bottom row `[0, 0, 0, 1]` and the truth of supplied
/// positive shape flags. Diagonal non-translation selects three reciprocals;
/// translation-only copies the identity linear block and negates translation;
/// otherwise the general block inverse is used. No shape predicates are repeated
/// here, and missing flags do not prove the corresponding structure absent.
#[inline]
fn invert_matrix4_affine(
    matrix: &[[Real; 4]; 4],
    linear_is_diagonal: bool,
    is_affine_translation: bool,
) -> BlasResult<[[Real; 4]; 4]> {
    // `linear_is_diagonal` and `is_affine_translation` are retained from
    // `Matrix4Facts`; do not re-probe them here. The helper is only entered
    // after the caller proves affine form.
    if linear_is_diagonal && !is_affine_translation {
        trace_matrix!("helper", "invert-matrix4-affine-linear-diagonal");
        return invert_matrix4_affine_linear_diagonal(matrix);
    }
    if is_affine_translation {
        trace_matrix!("helper", "invert-matrix4-affine-translation");
        return Ok([
            [
                matrix[0][0].clone(),
                matrix[0][1].clone(),
                matrix[0][2].clone(),
                Real::zero() - &matrix[0][3],
            ],
            [
                matrix[1][0].clone(),
                matrix[1][1].clone(),
                matrix[1][2].clone(),
                Real::zero() - &matrix[1][3],
            ],
            [
                matrix[2][0].clone(),
                matrix[2][1].clone(),
                matrix[2][2].clone(),
                Real::zero() - &matrix[2][3],
            ],
            [Real::zero(), Real::zero(), Real::zero(), Real::one()],
        ]);
    }
    invert_matrix4_affine_without_translation(matrix)
}

/// Computes the general affine block inverse `[R⁻¹ -R⁻¹ t; 0 1]`.
///
/// The caller must establish affine bottom-row shape. The name refers to bypassing
/// the translation-only shortcut: translation `t` is still used and need not be
/// zero. Clones the 3×3 block and translation, inverts the block, then forms three
/// translation dot products in row order. Scalar inverse errors propagate; the
/// canonical bottom row is constructed without inspecting the input bottom row.
#[inline]
fn invert_matrix4_affine_without_translation(
    matrix: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
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

    let inverse_translation: [Real; 3] = from_fn(|row| {
        let shifted = affine_translation_dot3(
            [
                &inverse_linear[row][0],
                &inverse_linear[row][1],
                &inverse_linear[row][2],
            ],
            [&translation[0], &translation[1], &translation[2]],
        );
        Real::zero() - shifted
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
        [Real::zero(), Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Selects checked diagonal, translation-only, or general affine inversion.
///
/// Requires caller-established affine shape and valid retained flags, as in
/// [`invert_matrix4_affine`]. Translation-only needs no reciprocal guard because
/// the supplied identity-linear fact implies determinant one. Other branches
/// propagate checked scalar/block inversion errors without publishing partial output.
#[inline]
fn invert_matrix4_affine_checked(
    matrix: &[[Real; 4]; 4],
    linear_is_diagonal: bool,
    is_affine_translation: bool,
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    if linear_is_diagonal && !is_affine_translation {
        trace_matrix!("helper", "invert-matrix4-checked-affine-linear-diagonal");
        return invert_matrix4_affine_linear_diagonal_checked(matrix);
    }
    if is_affine_translation {
        trace_matrix!("helper", "invert-matrix4-checked-affine-translation");
        return Ok([
            [
                matrix[0][0].clone(),
                matrix[0][1].clone(),
                matrix[0][2].clone(),
                Real::zero() - &matrix[0][3],
            ],
            [
                matrix[1][0].clone(),
                matrix[1][1].clone(),
                matrix[1][2].clone(),
                Real::zero() - &matrix[1][3],
            ],
            [
                matrix[2][0].clone(),
                matrix[2][1].clone(),
                matrix[2][2].clone(),
                Real::zero() - &matrix[2][3],
            ],
            [Real::zero(), Real::zero(), Real::zero(), Real::one()],
        ]);
    }
    invert_matrix4_affine_without_translation_checked(matrix)
}

/// Computes the general affine block inverse using checked 3×3 inversion.
///
/// Requires bottom row `[0, 0, 0, 1]`; translation is retained despite the name.
/// The 3×3 checked inverse completes before `-R⁻¹ t` is formed. Zero/unknown
/// failures propagate from that inverse; this helper does not validate affine
/// shape or start an independent decision procedure.
#[inline]
fn invert_matrix4_affine_without_translation_checked(
    matrix: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
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

    let inverse_translation: [Real; 3] = from_fn(|row| {
        let shifted = affine_translation_dot3(
            [
                &inverse_linear[row][0],
                &inverse_linear[row][1],
                &inverse_linear[row][2],
            ],
            [&translation[0], &translation[1], &translation[2]],
        );
        Real::zero() - shifted
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
        [Real::zero(), Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Selects affine inverse branches while forwarding the signal only to the general block path.
///
/// Affine shape and positive flags are caller-established. The diagonal-linear
/// branch uses the ordinary checked helper, and the translation-only branch
/// negates translation: neither branch attaches or polls `signal`. The remaining
/// branch delegates to abort-aware 3×3 inversion. This is not a deadline or an
/// unconditional cancellation check; exact shortcuts may succeed with a set flag.
#[inline]
fn invert_matrix4_affine_checked_with_abort(
    matrix: &[[Real; 4]; 4],
    signal: &AbortSignal,
    linear_is_diagonal: bool,
    is_affine_translation: bool,
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    if linear_is_diagonal && !is_affine_translation {
        trace_matrix!(
            "helper",
            "invert-matrix4-checked-with-abort-affine-linear-diagonal"
        );
        return invert_matrix4_affine_linear_diagonal_checked(matrix);
    }
    if is_affine_translation {
        trace_matrix!("helper", "invert-matrix4-checked-affine-translation");
        return Ok([
            [
                matrix[0][0].clone(),
                matrix[0][1].clone(),
                matrix[0][2].clone(),
                Real::zero() - &matrix[0][3],
            ],
            [
                matrix[1][0].clone(),
                matrix[1][1].clone(),
                matrix[1][2].clone(),
                Real::zero() - &matrix[1][3],
            ],
            [
                matrix[2][0].clone(),
                matrix[2][1].clone(),
                matrix[2][2].clone(),
                Real::zero() - &matrix[2][3],
            ],
            [Real::zero(), Real::zero(), Real::zero(), Real::one()],
        ]);
    }
    invert_matrix4_affine_without_translation_checked_with_abort(matrix, signal)
}

/// Forms `[R⁻¹ -R⁻¹ t; 0 1]` after abort-aware checked inversion of `R`.
///
/// Requires affine shape; translation need not vanish. Passes `signal` to the 3×3
/// inverse, then forms translation products without new attachment or explicit
/// polling. Scalar propagation and exact shortcuts determine which later work
/// observes the flag. Inversion errors return before constructing the output.
#[inline]
fn invert_matrix4_affine_without_translation_checked_with_abort(
    matrix: &[[Real; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Real; 4]; 4]> {
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

    let inverse_translation: [Real; 3] = from_fn(|row| {
        let shifted = affine_translation_dot3(
            [
                &inverse_linear[row][0],
                &inverse_linear[row][1],
                &inverse_linear[row][2],
            ],
            [&translation[0], &translation[1], &translation[2]],
        );
        Real::zero() - shifted
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
        [Real::zero(), Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Right-divides any 4×4 dividend by a caller-established `[D t; 0 1]`.
///
/// Computes the three reciprocals of diagonal `D`, then `u = -D⁻¹ t`. Each output
/// row has entries `left[i,j] / D[j,j]` for j<3 and
/// `left[i,3] + left[i,0]*u[0] + (left[i,1]*u[1] + left[i,2]*u[2])`.
/// All four rows are updated; the dividend need not be affine. The right shape
/// is trusted, scalar inverse errors propagate, and owned dividend entries are
/// still cloned for this fixed schedule.
#[inline]
fn divide_matrix4_by_affine_linear_diagonal(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
    // For affine with a diagonal 3×3 linear block, right-division is diagonal
    // scaling in each axis plus three correction terms for translation. This is
    // the same row-wise specialization used by affine point transforms and avoids
    // the generic 3×3 inversion inside the hot mat4 divide path.
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    let inv_a22 = right[2][2].clone().inverse()?;
    let inv_tx = Real::zero() - (&right[0][3] * &inv_a00);
    let inv_ty = Real::zero() - (&right[1][3] * &inv_a11);
    let inv_tz = Real::zero() - (&right[2][3] * &inv_a22);
    Ok([
        [
            {
                let row = left[0][0].clone();

                row * &inv_a00
            },
            {
                let row = left[0][1].clone();

                row * &inv_a11
            },
            {
                let row = left[0][2].clone();

                row * &inv_a22
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

                row * &inv_a00
            },
            {
                let row = left[1][1].clone();

                row * &inv_a11
            },
            {
                let row = left[1][2].clone();

                row * &inv_a22
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

                row * &inv_a00
            },
            {
                let row = left[2][1].clone();

                row * &inv_a11
            },
            {
                let row = left[2][2].clone();

                row * &inv_a22
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

                row * &inv_a00
            },
            {
                let row = left[3][1].clone();

                row * &inv_a11
            },
            {
                let row = left[3][2].clone();

                row * &inv_a22
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

/// Guards the three diagonal divisors before diagonal-affine right division.
///
/// The caller establishes the right shape; the left matrix is unrestricted.
/// Queries (0,0), (1,1), (2,2) in order, returning the first zero/unknown error
/// before any reciprocal, then delegates to the ordinary formula.
#[inline]
fn divide_matrix4_by_affine_linear_diagonal_checked(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    require_known_nonzero(&right[2][2])?;
    divide_matrix4_by_affine_linear_diagonal(left, right)
}

/// Borrows both operands for diagonal-affine right division of all four rows.
///
/// Uses the reciprocal and translation-correction schedule of
/// [`divide_matrix4_by_affine_linear_diagonal`], cloning needed entries. The
/// right operand must already have `[D t; 0 1]` shape; the left is unrestricted.
/// Scalar inverse errors propagate without an additional known-nonzero guard.
#[inline]
fn divide_matrix4_by_affine_linear_diagonal_ref(
    left: &[[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    let inv_a22 = right[2][2].clone().inverse()?;
    let inv_tx = Real::zero() - (&right[0][3] * &inv_a00);
    let inv_ty = Real::zero() - (&right[1][3] * &inv_a11);
    let inv_tz = Real::zero() - (&right[2][3] * &inv_a22);
    Ok([
        [
            {
                let row = left[0][0].clone();

                row * &inv_a00
            },
            {
                let row = left[0][1].clone();

                row * &inv_a11
            },
            {
                let row = left[0][2].clone();

                row * &inv_a22
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

                row * &inv_a00
            },
            {
                let row = left[1][1].clone();

                row * &inv_a11
            },
            {
                let row = left[1][2].clone();

                row * &inv_a22
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

                row * &inv_a00
            },
            {
                let row = left[2][1].clone();

                row * &inv_a11
            },
            {
                let row = left[2][2].clone();

                row * &inv_a22
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

                row * &inv_a00
            },
            {
                let row = left[3][1].clone();

                row * &inv_a11
            },
            {
                let row = left[3][2].clone();

                row * &inv_a22
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

/// Routes an affine dividend and diagonal-affine divisor to the all-row formula.
///
/// Callers establish both affine shapes and the divisor’s diagonal linear block.
/// The delegated kernel computes all four rows; it does not hard-code the dividend
/// bottom row or repeat a shape check. Scalar errors propagate unchanged.
fn divide_matrix4_affine_by_affine_linear_diagonal(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
    // For affine-by-affine, the same diagonal affine formula keeps the bottom
    // homogeneous row exact while collapsing the core to three inverse scalars.
    divide_matrix4_by_affine_linear_diagonal(left, right)
}

/// Borrowed affine/diagonal-affine dispatch adapter to the all-row division formula.
///
/// Uses [`divide_matrix4_by_affine_linear_diagonal_ref`] without new predicates;
/// callers retain the affine and diagonal shape evidence for these operands.
#[inline]
fn divide_matrix4_affine_by_affine_ref_linear_diagonal(
    left: &[[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
    divide_matrix4_by_affine_linear_diagonal_ref(left, right)
}

/// Checked affine/diagonal-affine adapter with no additional shape validation.
///
/// Forwards to [`divide_matrix4_by_affine_linear_diagonal_checked`], retaining
/// its diagonal guard order and exact failure result.
#[inline]
fn divide_matrix4_affine_by_affine_linear_diagonal_checked(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    divide_matrix4_by_affine_linear_diagonal_checked(left, right)
}

/// Adds a supplied inverse-translation dot product to one row’s final entry.
///
/// Computes `row[3] + dot(row[0..3], inverse_translation)` through the ordinary
/// three-term linear-combination adapter. The caller supplies the signed inverse
/// translation; this helper neither negates it nor validates an affine matrix.
#[inline]
fn affine_translation_column_update(row: &[Real; 4], inverse_translation: &[Real; 3]) -> Real {
    // Right division by a translation-only affine matrix updates only the
    // homogeneous column. Route the 3-term dot through `linear_combination3`
    // rather than spelling out three multiplies and two adds so exact Real kernels
    // can delay denominator/canonicalization work inside the short polynomial.
    let matrix_terms = [&row[0], &row[1], &row[2]];
    let translation_terms = [
        &inverse_translation[0],
        &inverse_translation[1],
        &inverse_translation[2],
    ];
    row[3].clone() + Real::linear_combination3(matrix_terms, translation_terms)
}

/// Subtracts the original translation’s dot product from one row’s final entry.
///
/// Computes `row[3] - dot(row[0..3], translation)` through
/// [`affine_translation_dot3`]. Unlike the addition helper, it receives the
/// unnegated translation and uses the active-term scalar schedule.
#[inline]
fn affine_translation_column_subtract_update(row: &[Real; 4], translation: [&Real; 3]) -> Real {
    trace_matrix!("helper", "affine-translation-column-subtract");
    let shifted = affine_translation_dot3([&row[0], &row[1], &row[2]], translation);
    row[3].clone() - shifted
}

/// Forwards three ordered products to the active scalar linear-combination kernel.
///
/// This helper adds a trace label but no zero scan, rationality check, or local
/// normalization. “Active” means retained terms, not known nonzero values. The
/// trace label’s “exact” wording does not restrict the inputs to rational scalars.
#[inline]
fn affine_translation_dot3(coefficients: [&Real; 3], values: [&Real; 3]) -> Real {
    trace_matrix!("helper", "affine-translation-dot3-active-exact");
    Real::active_linear_combination3(coefficients, values)
}

/// Right-divides an unrestricted dividend by a caller-established affine divisor.
///
/// Builds the general affine inverse, including its translation, then multiplies
/// the full 4×4 matrices. The name excludes the translation-only shortcut, not
/// the divisor’s translation column. Inversion errors propagate before multiplication.
#[inline]
fn divide_matrix4_by_affine_no_translation(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
    let inverse = invert_matrix4_affine_without_translation(right)?;
    Ok(multiply_arrays4(left, inverse))
}

/// Right-divides two caller-established affine matrices using their 3×3 blocks.
///
/// For `[L a; 0 1] / [R b; 0 1]`, computes `R⁻¹`, then `u = -R⁻¹ b`,
/// `L R⁻¹`, and `a + L u` in that order. Reconstructs bottom row `[0,0,0,1]`
/// without reading either input bottom row. Translation is retained; the name
/// refers to the general path beyond the translation-only shortcut. Inverse
/// errors propagate, and the new linear operands are checked for dense support.
#[inline]
fn divide_matrix4_affine_by_affine_no_translation(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
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
    let right_inverse_translation: [Real; 3] = from_fn(|row| {
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
        Real::zero() - shifted
    });
    let linear = multiply_arrays3_affine_linear_with_exact_dense_certificate(
        left_linear,
        right_inverse_linear,
    );
    let translation: [Real; 3] = from_fn(|row| {
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
        [Real::zero(), Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Right-divides any matrix by a caller-established translation `[I t; 0 1]`.
///
/// Copies the first three columns and computes the fourth as `left[i,3] - L_i·t`
/// for all four rows. Returns `Ok` without a reciprocal or domain guard: the
/// identity-linear precondition implies determinant one. Shape is not validated,
/// and this algebraic fact supplies no bound on scalar expression work.
#[inline]
fn divide_matrix4_by_affine_translation(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
    // Right-dividing by translation-only affine uses homogeneous column update only:
    // M·[I t;0 1]⁻¹ = M·[I -t;0 1].
    let translation = [&right[0][3], &right[1][3], &right[2][3]];
    Ok([
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
    ])
}

/// Multiplies affine linear blocks after checking both for known dense support.
///
/// Checks the left block before the right; both must have every entry known
/// nonzero to select the dense dot kernel. Otherwise uses ordinary multiplication.
/// The name does not impose a rational-input requirement, and previous density
/// of a source matrix does not certify its inverse or a product.
#[inline]
fn multiply_arrays3_affine_linear_with_exact_dense_certificate(
    left: [[Real; 3]; 3],
    right: [[Real; 3]; 3],
) -> [[Real; 3]; 3] {
    if matrix3_has_dense_multiply_certificate(&left)
        && matrix3_has_dense_multiply_certificate(&right)
    {
        trace_matrix!("helper", "multiply3-affine-linear-dense-certified-exact");
        return multiply_arrays3_dense_ref(&left, &right);
    }
    multiply_arrays3(left, right)
}

/// Right-divides an affine matrix by a caller-established pure translation.
///
/// Copies the top-left linear block, subtracts its action on the divisor’s
/// translation from the top three offsets, and constructs `[0,0,0,1]` as the
/// bottom row. Both shape preconditions are trusted. No reciprocal is needed;
/// this differs from the unrestricted helper, which computes all four rows.
#[inline]
fn divide_matrix4_affine_by_affine_translation(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
    // For affine-by-affine with translation-only divisor, the linear basis is unchanged.
    let translation = [&right[0][3], &right[1][3], &right[2][3]];
    Ok([
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
        [Real::zero(), Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Dispatch adapter for checked right division by a known affine divisor.
///
/// Forwards the unrestricted dividend to the general affine inverse/product
/// path, without recognizing translation-only shape here.
#[inline]
fn divide_matrix4_by_affine_checked(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    divide_matrix4_by_affine_no_translation_checked(left, right)
}

/// Forwards retained translation-only shape to the checked translation dispatcher.
///
/// The caller must establish `[I t; 0 1]` for the divisor. No shape or scalar
/// guard is added by this adapter; the dividend may be any 4×4 matrix.
#[inline]
fn divide_matrix4_by_affine_checked_assumed_affine_translation(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    // Structural fact is prevalidated by the caller; avoid re-testing `right`.
    divide_matrix4_by_affine_checked_assuming_affine_translation(left, right)
}

/// Records the checked translation branch and performs all-row translation division.
///
/// Trusts the divisor’s identity-linear affine shape. Since no reciprocal is
/// required, this delegates directly to the ordinary translation formula without
/// known-nonzero queries; “checked” does not validate the supplied shape.
#[inline]
fn divide_matrix4_by_affine_checked_assuming_affine_translation(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    // Structural fact is already established by caller to avoid duplicate checks in
    // this checked hot path.
    trace_matrix!("helper", "right-divide4-checked-by-affine-translation");
    divide_matrix4_by_affine_translation(left, right)
}

/// Right-divides any matrix using a checked general affine inverse of the divisor.
///
/// Requires the divisor’s affine bottom row. Inversion errors propagate before
/// the full 4×4 product. Translation is included despite the helper’s name.
#[inline]
fn divide_matrix4_by_affine_no_translation_checked(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    let inverse = invert_matrix4_affine_without_translation_checked(right)?;
    Ok(multiply_arrays4(left, inverse))
}

/// Routes two known affine operands to checked general block right division.
///
/// No shape checks or translation-only recognition are performed by this adapter.
#[inline]
fn divide_matrix4_affine_by_affine_checked(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    divide_matrix4_affine_by_affine_no_translation_checked(left, right)
}

/// Forwards known affine/translation shapes to the checked block dispatcher.
///
/// The left must be affine and the right translation-only; this adapter adds no
/// validation or arithmetic to the delegated schedule.
#[inline]
fn divide_matrix4_affine_by_affine_checked_assumed_affine_translation(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    // Caller already proved the right divisor is translation-only affine.
    divide_matrix4_affine_by_affine_checked_assuming_affine_translation(left, right)
}

/// Records checked affine-by-translation dispatch and uses the ordinary block formula.
///
/// Trusts both shape preconditions, uses no reciprocal, and reconstructs a
/// canonical affine bottom row. A `CheckedBlasResult` does not certify unvalidated
/// caller-supplied shape flags.
fn divide_matrix4_affine_by_affine_checked_assuming_affine_translation(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    // Right divisor is known translation-only affine; skip repeated structural checks.
    // This follows standard affine composition algebra for translation-only linear maps.
    trace_matrix!(
        "helper",
        "right-divide4-checked-affine-by-affine-translation"
    );
    divide_matrix4_affine_by_affine_translation(left, right)
}

/// Computes checked affine right division as `[L R⁻¹, a - L R⁻¹ b; 0, 1]`.
///
/// Both operands must already be affine. Checked inversion of `R` completes
/// before inverse-translation, linear-product, and final-translation work.
/// The bottom row is reconstructed, not checked; translations need not vanish.
#[inline]
fn divide_matrix4_affine_by_affine_no_translation_checked(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
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
    let right_inverse_translation: [Real; 3] = from_fn(|row| {
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
        Real::zero() - shifted
    });
    let linear = multiply_arrays3_affine_linear_with_exact_dense_certificate(
        left_linear,
        right_inverse_linear,
    );
    let translation: [Real; 3] = from_fn(|row| {
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
        [Real::zero(), Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Routes an unrestricted dividend to abort-aware checked general affine division.
///
/// Requires a known affine divisor; the signal is forwarded to its block inverse,
/// with no local polling or additional shape recognition.
#[inline]
fn divide_matrix4_by_affine_checked_with_abort(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    divide_matrix4_by_affine_no_translation_checked_with_abort(left, right, signal)
}

/// Forwards a known translation divisor to the signal-accepting translation adapter.
///
/// The delegated translation-only path ignores `signal`; accepting this argument
/// does not provide a cancellation check or a scalar-work bound.
#[inline]
fn divide_matrix4_by_affine_checked_with_abort_assumed_affine_translation(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    // Caller already established translation-only affine divisor; skip recompute.
    divide_matrix4_by_affine_checked_with_abort_assuming_affine_translation(left, right, signal)
}

/// Performs checked-dispatch translation division while explicitly ignoring the signal.
///
/// Requires a caller-established translation-only divisor and updates all four
/// dividend rows. Determinant one removes the reciprocal, but does not bound
/// translation expression work or justify a general cancellation guarantee.
/// Neither flag polling nor signal attachment occurs here.
#[inline]
fn divide_matrix4_by_affine_checked_with_abort_assuming_affine_translation(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
    _signal: &AbortSignal,
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    // This shortcut ignores the supplied signal. Determinant one avoids
    // reciprocal work but does not bound translation-expression work.
    trace_matrix!(
        "helper",
        "right-divide4-checked-abort-by-affine-translation"
    );
    divide_matrix4_by_affine_translation(left, right)
}

/// Right-divides any matrix after abort-aware checked affine inversion.
///
/// Requires affine divisor shape; translation is included. Forwards `signal` to
/// the inverse and then multiplies without further attachment or explicit polling.
/// Scalar work inherits whatever signal dependencies the inverse retained.
#[inline]
fn divide_matrix4_by_affine_no_translation_checked_with_abort(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    let inverse = invert_matrix4_affine_without_translation_checked_with_abort(right, signal)?;
    Ok(multiply_arrays4(left, inverse))
}

/// Routes two known affine operands to abort-aware checked block right division.
///
/// No local polling or shape checks are added; the signal is forwarded to the
/// right linear-block inverse by the delegated helper.
#[inline]
fn divide_matrix4_affine_by_affine_checked_with_abort(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    divide_matrix4_affine_by_affine_no_translation_checked_with_abort(left, right, signal)
}

/// Forwards known affine/translation shapes to the signal-accepting block shortcut.
///
/// The delegated translation path ignores the supplied signal. Both shape facts
/// must be valid for these unchanged operands; no validation is added here.
#[inline]
fn divide_matrix4_affine_by_affine_checked_with_abort_assumed_affine_translation(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    // Affine-by-affine translation-only divisor fact is caller-proven.
    divide_matrix4_affine_by_affine_checked_with_abort_assuming_affine_translation(
        left, right, signal,
    )
}

/// Uses affine-by-translation arithmetic without polling or attaching the signal.
///
/// Requires affine left and translation-only right operands. The determinant-one
/// fact removes reciprocal work but does not bound scalar expressions. Constructs
/// the canonical affine bottom row and returns the ordinary shortcut’s result.
#[inline]
fn divide_matrix4_affine_by_affine_checked_with_abort_assuming_affine_translation(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
    _signal: &AbortSignal,
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    // This translation shortcut does not attach or poll the supplied signal;
    // nonsingularity alone is not a cancellation or work-bound guarantee.
    trace_matrix!(
        "helper",
        "right-divide4-checked-abort-affine-by-affine-translation"
    );
    divide_matrix4_affine_by_affine_translation(left, right)
}

/// Computes affine block right division with signal forwarding to `R⁻¹`.
///
/// Requires both affine bottom rows. Abort-aware checked 3×3 inversion precedes
/// `-R⁻¹ b`, `L R⁻¹`, and `a - L R⁻¹ b`. Later products and sums add no separate
/// attachment or polling; scalar propagation determines cancellation behavior.
/// Translations are retained and the result’s bottom row is reconstructed.
#[inline]
fn divide_matrix4_affine_by_affine_no_translation_checked_with_abort(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Real; 4]; 4]> {
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
    let right_inverse_translation: [Real; 3] = from_fn(|row| {
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
        Real::zero() - shifted
    });
    let linear = multiply_arrays3_affine_linear_with_exact_dense_certificate(
        left_linear,
        right_inverse_linear,
    );
    let translation: [Real; 3] = from_fn(|row| {
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
        [Real::zero(), Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Borrowed dispatch adapter for an already established translation-only divisor.
///
/// Adds no shape checks; delegates all four dividend rows to the borrowed
/// translation-column update schedule.
fn divide_matrix4_by_affine_ref_assumed_affine_translation(
    left: &[[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
    // Caller has already proven translation-only affine structure.
    divide_matrix4_by_affine_ref_translation(left, right)
}

/// Right-divides borrowed matrices by a known translation-only divisor.
///
/// Negates the three translation scalars once, then adds an ordinary three-term
/// linear combination to each row’s final entry. The owned path instead subtracts
/// an active-term combination of the original translation. Both express the same
/// right product, but their scalar schedules and traces are distinct. The dividend
/// is unrestricted; divisor shape is trusted and no reciprocal is constructed.
fn divide_matrix4_by_affine_ref_translation(
    left: &[[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
    // Borrowed special-case for right-division by translation-only affine.
    // Using the prevalidated structural fact avoids rebuilding the full
    // translation column for the non-affine path and keeps this helper on the
    // same right-product formula, using a different scalar schedule than the owned version.
    trace_matrix!("helper", "right-divide4-ref-by-affine-translation");
    let inverse_translation = [
        Real::zero() - &right[0][3],
        Real::zero() - &right[1][3],
        Real::zero() - &right[2][3],
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

/// Right-divides a borrowed arbitrary dividend by a known affine divisor.
///
/// Constructs the general affine inverse, including translation, then uses
/// borrowed full-matrix multiplication. Inverse failures propagate before the
/// product; shape is not revalidated.
#[inline]
fn divide_matrix4_by_affine_ref_no_translation(
    left: &[[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
    let inverse = invert_matrix4_affine_without_translation(right)?;
    Ok(multiply_arrays4_ref(left, &inverse))
}

/// Borrows two known affine matrices for general block right division.
///
/// Clones the linear blocks and right translation, then computes `R⁻¹`,
/// `-R⁻¹ b`, `L R⁻¹`, and `a - L R⁻¹ b`. Constructs a canonical affine bottom
/// row. The name excludes only the translation shortcut; translation is retained.
/// Errors from ordinary block inversion propagate without a partial result.
#[inline]
fn divide_matrix4_affine_by_affine_ref_no_translation(
    left: &[[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
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
    let right_inverse_translation: [Real; 3] = from_fn(|row| {
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
        Real::zero() - shifted
    });
    let linear = multiply_arrays3_affine_linear_with_exact_dense_certificate(
        left_linear,
        right_inverse_linear,
    );
    let translation: [Real; 3] = from_fn(|row| {
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
        [Real::zero(), Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Borrows affine/translation operands for the three-row translation shortcut.
///
/// Negates the divisor translation once, then uses ordinary linear combinations
/// to add its inverse contribution to the three affine offsets. Copies the linear
/// block and constructs `[0,0,0,1]`. Requires both shape facts; unlike the owned
/// shortcut, this does not subtract an active-term combination of the original t.
#[inline]
fn divide_matrix4_affine_by_affine_ref_translation(
    left: &[[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
    // Borrowed translation-only affine-by-affine special case avoids re-checking
    // right translation when caller already asserted it once.
    let inverse_translation = [
        Real::zero() - &right[0][3],
        Real::zero() - &right[1][3],
        Real::zero() - &right[2][3],
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
        [Real::zero(), Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Inverts a caller-certified diagonal 3×3 matrix, reusing a uniform reciprocal when recognized.
///
/// Compares diagonal entries with structural `Real` equality, first (0,0) with
/// (1,1), then (2,2). Equal symbolic representations can qualify; this is not the
/// rational-only public uniform-scale fact. A match constructs one inverse and
/// clones it; otherwise constructs three inverses in index order. Scalar errors
/// propagate. Off-diagonal shape is trusted and the result’s zeros are constructed.
fn invert_matrix3_by_diagonal(matrix: &[[Real; 3]; 3]) -> BlasResult<[[Real; 3]; 3]> {
    // For true diagonal matrices, inversion is n scalar inverses with no extra
    // multiply-add schedule; this avoids the division-heavy elimination and
    // cofactor work while preserving exact division semantics.
    if matrix[0][0] == matrix[1][1] && matrix[0][0] == matrix[2][2] {
        trace_matrix!("helper", "invert-matrix3-diagonal-uniform-scale");
        let inv = matrix[0][0].clone().inverse()?;
        return Ok([
            [inv.clone(), Real::zero(), Real::zero()],
            [Real::zero(), inv.clone(), Real::zero()],
            [Real::zero(), Real::zero(), inv],
        ]);
    }
    let inv00 = matrix[0][0].clone().inverse()?;
    let inv11 = matrix[1][1].clone().inverse()?;
    let inv22 = matrix[2][2].clone().inverse()?;
    // Hyperreal-style Real kernels prefer direct fixed-array construction here:
    // the structural diagonal fact already selected this kernel, so
    // per-cell branch dispatch only re-proves known sparsity. This is the
    // fixed-size version of exploiting matrix structure before arithmetic.
    Ok([
        [inv00, Real::zero(), Real::zero()],
        [Real::zero(), inv11, Real::zero()],
        [Real::zero(), Real::zero(), inv22],
    ])
}

/// Guards all three diagonal entries before ordinary diagonal inversion.
///
/// Requires caller-certified diagonal shape. Known-nonzero guards run in index
/// order, stopping at the first zero/unknown error, before uniform-scale equality
/// checks or any reciprocal construction.
#[inline]
fn invert_matrix3_by_diagonal_checked(
    matrix: &[[Real; 3]; 3],
) -> CheckedBlasResult<[[Real; 3]; 3]> {
    require_known_nonzero(&matrix[0][0])?;
    require_known_nonzero(&matrix[1][1])?;
    require_known_nonzero(&matrix[2][2])?;
    invert_matrix3_by_diagonal(matrix)
}

/// Inverts a caller-certified upper-triangular 3×3 matrix by explicit substitution.
///
/// Constructs diagonal reciprocals in index order, then inverse entries (0,1),
/// (1,2), and (0,2). With `B = A⁻¹`, the corner is
/// `B[0,2] = -(A[0,1]*B[1,2] + A[0,2]*B[2,2])/A[0,0]`.
/// The lower triangle is constructed as zero without validating input shape.
/// Ordinary scalar inverse errors propagate; no separate known-nonzero guard runs.
#[inline]
fn invert_matrix3_upper_triangular(matrix: &[[Real; 3]; 3]) -> BlasResult<[[Real; 3]; 3]> {
    // Three pivot reciprocals feed the explicit off-diagonal substitutions.
    // This path does not construct a general cofactor matrix.
    let inv_a00 = matrix[0][0].clone().inverse()?;
    let inv_a11 = matrix[1][1].clone().inverse()?;
    let inv_a22 = matrix[2][2].clone().inverse()?;

    let inv_a01 = scale_by_shared_factor(Real::zero() - &matrix[0][1], &inv_a11);
    let inv_a01 = scale_by_shared_factor(inv_a01, &inv_a00);
    let inv_a12 = scale_by_shared_factor(Real::zero() - &matrix[1][2], &inv_a11);
    let inv_a12 = scale_by_shared_factor(inv_a12, &inv_a22);
    let inv_a02 = Real::zero() - ((&matrix[0][1] * &inv_a12) + (&matrix[0][2] * &inv_a22));
    let inv_a02 = scale_by_shared_factor(inv_a02, &inv_a00);

    Ok([
        [inv_a00, inv_a01, inv_a02],
        [Real::zero(), inv_a11, inv_a12],
        [Real::zero(), Real::zero(), inv_a22],
    ])
}

/// Checks the three diagonal pivots before upper-triangular 3×3 inversion.
///
/// The caller establishes triangular shape. Guards run from index zero to two
/// and stop at the first zero/unknown result; all precede reciprocal construction.
#[inline]
fn invert_matrix3_upper_triangular_checked(
    matrix: &[[Real; 3]; 3],
) -> CheckedBlasResult<[[Real; 3]; 3]> {
    require_known_nonzero(&matrix[0][0])?;
    require_known_nonzero(&matrix[1][1])?;
    require_known_nonzero(&matrix[2][2])?;
    invert_matrix3_upper_triangular(matrix)
}

/// Inverts a caller-certified lower-triangular 3×3 matrix by explicit substitution.
///
/// Constructs diagonal reciprocals, then entries (1,0), (2,0), and (2,1), in that
/// order. The corner uses `B[2,0] = -(A[2,0]*B[0,0] + A[2,1]*B[1,0])/A[2,2]`.
/// Upper entries are constructed as zero. Shape is not checked; scalar inverse
/// errors propagate without a separate known-nonzero guard.
#[inline]
fn invert_matrix3_lower_triangular(matrix: &[[Real; 3]; 3]) -> BlasResult<[[Real; 3]; 3]> {
    // Lower-triangular inversion is the dual of upper-triangular back-substitution.
    // This explicit fixed-size schedule avoids cofactor materialization for
    // known lower-triangular matrices.
    let inv_a00 = matrix[0][0].clone().inverse()?;
    let inv_a11 = matrix[1][1].clone().inverse()?;
    let inv_a22 = matrix[2][2].clone().inverse()?;

    let inv_a10 = scale_by_shared_factor(Real::zero() - &matrix[1][0], &inv_a00);
    let inv_a10 = scale_by_shared_factor(inv_a10, &inv_a11);
    let inv_a20 = Real::zero() - ((&matrix[2][0] * &inv_a00) + (&matrix[2][1] * &inv_a10));
    let inv_a20 = scale_by_shared_factor(inv_a20, &inv_a22);
    let inv_a21 = scale_by_shared_factor(Real::zero() - &matrix[2][1], &inv_a11);
    let inv_a21 = scale_by_shared_factor(inv_a21, &inv_a22);

    Ok([
        [inv_a00, Real::zero(), Real::zero()],
        [inv_a10, inv_a11, Real::zero()],
        [inv_a20, inv_a21, inv_a22],
    ])
}

/// Checks the three diagonal pivots before lower-triangular 3×3 inversion.
///
/// Requires caller-established lower-triangular shape. Guards run in ascending
/// index order and complete before delegating to the ordinary formula.
#[inline]
fn invert_matrix3_lower_triangular_checked(
    matrix: &[[Real; 3]; 3],
) -> CheckedBlasResult<[[Real; 3]; 3]> {
    require_known_nonzero(&matrix[0][0])?;
    require_known_nonzero(&matrix[1][1])?;
    require_known_nonzero(&matrix[2][2])?;
    invert_matrix3_lower_triangular(matrix)
}

/// Inverts a caller-established 2D affine matrix `[R t; 0 1]`.
///
/// A retained diagonal-linear flag selects the two-reciprocal helper. Otherwise
/// forms the 2×2 determinant `ad-bc`, takes one reciprocal, scales the adjugate,
/// and computes `-R⁻¹ t`. The bottom row is reconstructed as `[0,0,1]`.
/// Shape and a positive diagonal flag must be valid; they are not rechecked.
/// The ordinary determinant inverse supplies any scalar failure.
#[inline]
fn invert_matrix3_affine(
    matrix: &[[Real; 3]; 3],
    linear_is_diagonal: bool,
) -> BlasResult<[[Real; 3]; 3]> {
    // See Golub and Van Loan, *Matrix Computations*: affine composition
    // in homogeneous coordinates is handled by a 2×2 block inverse plus one
    // translation correction -R⁻¹t. The full matrix bottom row is known.
    // The caller supplies `linear_is_diagonal` from `Matrix3Facts`, avoiding a
    // second probe of the same off-diagonal entries after affine dispatch.
    // Reuse the retained object facts.
    if linear_is_diagonal {
        trace_matrix!("helper", "invert-matrix3-affine-linear-diagonal");
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
    let inv_a01 = scale_by_shared_factor(Real::zero() - &b, &inv_det);
    let inv_a10 = scale_by_shared_factor(Real::zero() - &c, &inv_det);
    let inv_a11 = scale_by_shared_factor(a, &inv_det);
    let inv_tx = Real::zero() - ((&inv_a00 * &tx) + (&inv_a01 * &ty));
    let inv_ty = Real::zero() - ((&inv_a10 * &tx) + (&inv_a11 * &ty));

    Ok([
        [inv_a00, inv_a01, inv_tx],
        [inv_a10, inv_a11, inv_ty],
        [Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Computes `[D⁻¹ -D⁻¹ t; 0 1]` for a caller-certified diagonal 2D affine block.
///
/// Uses two scalar reciprocals, first x then y, and two translation corrections.
/// The off-diagonal and bottom-row shape is trusted, not validated; ordinary
/// scalar inverse failures propagate before publishing a result.
#[inline]
fn invert_matrix3_affine_linear_diagonal(matrix: &[[Real; 3]; 3]) -> BlasResult<[[Real; 3]; 3]> {
    // The diagonal 2×2 block needs two reciprocals. Each translation correction
    // is the corresponding offset multiplied by its reciprocal and negated.
    let inv_a00 = matrix[0][0].clone().inverse()?;
    let inv_a11 = matrix[1][1].clone().inverse()?;
    let inv_tx = Real::zero() - (matrix[0][2].clone() * &inv_a00);
    let inv_ty = Real::zero() - (matrix[1][2].clone() * &inv_a11);

    Ok([
        [inv_a00, Real::zero(), inv_tx],
        [Real::zero(), inv_a11, inv_ty],
        [Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Guards x then y before computing a diagonal 2D affine inverse.
///
/// Requires known affine/diagonal shape. Both known-nonzero guards precede the
/// two reciprocals and translation corrections. Returns the first DivideByZero
/// or UnknownZero encountered without independently resolving the unknown.
#[inline]
fn invert_matrix3_affine_linear_diagonal_checked(
    matrix: &[[Real; 3]; 3],
) -> CheckedBlasResult<[[Real; 3]; 3]> {
    require_known_nonzero(&matrix[0][0])?;
    require_known_nonzero(&matrix[1][1])?;

    let inv_a00 = matrix[0][0].clone().inverse()?;
    let inv_a11 = matrix[1][1].clone().inverse()?;
    let inv_tx = Real::zero() - (matrix[0][2].clone() * &inv_a00);
    let inv_ty = Real::zero() - (matrix[1][2].clone() * &inv_a11);

    Ok([
        [inv_a00, Real::zero(), inv_tx],
        [Real::zero(), inv_a11, inv_ty],
        [Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Attaches a signal to both diagonal clones before checked 2D affine inversion.
///
/// Requires known affine/diagonal shape. Attachment occurs for x and y before
/// either known-nonzero guard, so both are attached even if the first guard fails.
/// Guards and reciprocals then run x before y. Translation scalars receive no
/// separate attachment, and this helper does not explicitly poll the flag.
/// Signal propagation and cancellation behavior remain scalar-owned.
#[inline]
fn invert_matrix3_affine_linear_diagonal_checked_with_abort(
    matrix: &[[Real; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Real; 3]; 3]> {
    let a00 = with_abort(matrix[0][0].clone(), signal);
    let a11 = with_abort(matrix[1][1].clone(), signal);
    let inv_a00 = a00;
    let inv_a11 = a11;
    require_known_nonzero(&inv_a00)?;
    require_known_nonzero(&inv_a11)?;
    let inv_a00 = inv_a00.inverse()?;
    let inv_a11 = inv_a11.inverse()?;
    let inv_tx = Real::zero() - (matrix[0][2].clone() * &inv_a00);
    let inv_ty = Real::zero() - (matrix[1][2].clone() * &inv_a11);

    Ok([
        [inv_a00, Real::zero(), inv_tx],
        [Real::zero(), inv_a11, inv_ty],
        [Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Right-divides any 3×3 dividend by a known diagonal-linear affine divisor.
///
/// For divisor `[D t;0 1]`, computes two reciprocals and `u = -D⁻¹ t`, then
/// returns each row as `[left[i,0]/D[0,0], left[i,1]/D[1,1],
/// (left[i,2] + left[i,0]*u[0]) + left[i,1]*u[1]]`. All three rows are updated;
/// the dividend need not be affine. Shape is trusted and scalar inverse errors
/// propagate. The owned input’s entries are cloned by this fixed schedule.
#[inline]
fn divide_matrix3_by_affine_linear_diagonal(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> BlasResult<[[Real; 3]; 3]> {
    // The 2×2 linear diagonal branch uses two reciprocals and a translation
    // column correction, avoiding formation of a 2×2 determinant.
    // This is the 2D analogue of axis-aligned 4D affine division, and matches
    // the structural savings described in Golub and Van Loan, *Matrix
    // Computations*.
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    let inv_tx = Real::zero() - (&right[0][2] * &inv_a00);
    let inv_ty = Real::zero() - (&right[1][2] * &inv_a11);

    Ok([
        [
            {
                let row = left[0][0].clone();

                row * &inv_a00
            },
            {
                let row = left[0][1].clone();

                row * &inv_a11
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

                row * &inv_a00
            },
            {
                let row = left[1][1].clone();

                row * &inv_a11
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

                row * &inv_a00
            },
            {
                let row = left[2][1].clone();

                row * &inv_a11
            },
            {
                let x = left[2][0].clone();
                let y = left[2][1].clone();
                left[2][2].clone() + &(x * &inv_tx) + &(y * &inv_ty)
            },
        ],
    ])
}

/// Guards the two nontrivial diagonal divisors before all-row affine division.
///
/// Requires the right affine/diagonal shape; the dividend is unrestricted. Checks
/// x then y, returning the first zero/unknown error before reciprocal work, then
/// delegates to [`divide_matrix3_by_affine_linear_diagonal`].
#[inline]
fn divide_matrix3_by_affine_linear_diagonal_checked(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> CheckedBlasResult<[[Real; 3]; 3]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    divide_matrix3_by_affine_linear_diagonal(left, right)
}

/// Borrows both operands for the two-reciprocal diagonal-affine division formula.
///
/// Requires a known diagonal-linear affine divisor; updates all three dividend
/// rows with the same association as the owned helper. Clones required entries
/// and propagates ordinary inverse errors without an additional zero-status guard.
#[inline]
fn divide_matrix3_by_affine_ref_linear_diagonal(
    left: &[[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> BlasResult<[[Real; 3]; 3]> {
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    let inv_tx = Real::zero() - (&right[0][2] * &inv_a00);
    let inv_ty = Real::zero() - (&right[1][2] * &inv_a11);

    Ok([
        [
            {
                let row = left[0][0].clone();

                row * &inv_a00
            },
            {
                let row = left[0][1].clone();

                row * &inv_a11
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

                row * &inv_a00
            },
            {
                let row = left[1][1].clone();

                row * &inv_a11
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

                row * &inv_a00
            },
            {
                let row = left[2][1].clone();

                row * &inv_a11
            },
            {
                let x = left[2][0].clone();
                let y = left[2][1].clone();
                left[2][2].clone() + &(x * &inv_tx) + &(y * &inv_ty)
            },
        ],
    ])
}

/// Routes known affine/diagonal-affine operands to the unrestricted-row formula.
///
/// Adds no shape validation or arithmetic; the delegated helper computes all
/// three rows rather than replacing the affine bottom row.
#[inline]
fn divide_matrix3_affine_by_affine_linear_diagonal(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> BlasResult<[[Real; 3]; 3]> {
    divide_matrix3_by_affine_linear_diagonal(left, right)
}

/// Borrowed dispatch adapter for known affine/diagonal-affine operands.
///
/// Delegates to [`divide_matrix3_by_affine_ref_linear_diagonal`] without new
/// shape predicates, preserving its scalar order and errors.
#[inline]
fn divide_matrix3_affine_by_affine_ref_linear_diagonal(
    left: &[[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> BlasResult<[[Real; 3]; 3]> {
    divide_matrix3_by_affine_ref_linear_diagonal(left, right)
}

/// Checked adapter for known affine/diagonal-affine operands.
///
/// Delegates to the all-row helper with x-then-y known-nonzero guards. Checked
/// scalar status does not validate the caller’s shape evidence.
#[inline]
fn divide_matrix3_affine_by_affine_linear_diagonal_checked(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> CheckedBlasResult<[[Real; 3]; 3]> {
    divide_matrix3_by_affine_linear_diagonal_checked(left, right)
}

/// Inverts a known 2D affine matrix using diagonal guards or a determinant guard.
///
/// A valid retained diagonal flag selects the two-pivot checked helper. Otherwise
/// constructs `ad-bc`, requires its known-nonzero status, then constructs the
/// reciprocal, adjugate scale and `-R⁻¹ t`. Reconstructs `[0,0,1]` without
/// validating the input bottom row; zero/unknown errors propagate.
#[inline]
fn invert_matrix3_affine_checked(
    matrix: &[[Real; 3]; 3],
    linear_is_diagonal: bool,
) -> CheckedBlasResult<[[Real; 3]; 3]> {
    if linear_is_diagonal {
        trace_matrix!("helper", "invert-matrix3-checked-affine-linear-diagonal");
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
    let inv_a01 = scale_by_shared_factor(Real::zero() - &b, &inv_det);
    let inv_a10 = scale_by_shared_factor(Real::zero() - &c, &inv_det);
    let inv_a11 = scale_by_shared_factor(a, &inv_det);
    let inv_tx = Real::zero() - ((&inv_a00 * &tx) + (&inv_a01 * &ty));
    let inv_ty = Real::zero() - ((&inv_a10 * &tx) + (&inv_a11 * &ty));

    Ok([
        [inv_a00, inv_a01, inv_tx],
        [inv_a10, inv_a11, inv_ty],
        [Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Selects signal-attached checked inversion for a known 2D affine matrix.
///
/// The diagonal branch attaches both pivot clones before their guards. Otherwise
/// forms `ad-bc`, attaches the signal to that determinant, then guards and inverts
/// it. Later scaling and translation corrections add no local attachment or flag
/// polling. Affine shape and positive flags remain caller-established facts.
#[inline]
fn invert_matrix3_affine_checked_with_abort(
    matrix: &[[Real; 3]; 3],
    signal: &AbortSignal,
    linear_is_diagonal: bool,
) -> CheckedBlasResult<[[Real; 3]; 3]> {
    if linear_is_diagonal {
        trace_matrix!(
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
    let inv_a01 = scale_by_shared_factor(Real::zero() - &b, &inv_det);
    let inv_a10 = scale_by_shared_factor(Real::zero() - &c, &inv_det);
    let inv_a11 = scale_by_shared_factor(a, &inv_det);
    let inv_tx = Real::zero() - ((&inv_a00 * &tx) + (&inv_a01 * &ty));
    let inv_ty = Real::zero() - ((&inv_a10 * &tx) + (&inv_a11 * &ty));

    Ok([
        [inv_a00, inv_a01, inv_tx],
        [inv_a10, inv_a11, inv_ty],
        [Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Inverts a caller-certified diagonal 4×4 matrix with structural uniform-scale reuse.
///
/// Compares entry (0,0) with (1,1), (2,2), then (3,3), short-circuiting on a
/// structural mismatch. A match, including equal symbolic representations, uses
/// one reciprocal and clones it; otherwise takes four reciprocals in order.
/// Scalar errors propagate. This equality check is distinct from the public
/// rational-only uniform-scale fact and does not validate off-diagonal zeros.
#[inline]
fn invert_matrix4_by_diagonal(matrix: &[[Real; 4]; 4]) -> BlasResult<[[Real; 4]; 4]> {
    // The same diagonal path is the exact analog in 4x4: invert only diagonal
    // entries when structural zeros certify no couplings.
    if matrix[0][0] == matrix[1][1] && matrix[0][0] == matrix[2][2] && matrix[0][0] == matrix[3][3]
    {
        trace_matrix!("helper", "invert-matrix4-diagonal-uniform-scale");
        let inv = matrix[0][0].clone().inverse()?;
        return Ok([
            [inv.clone(), Real::zero(), Real::zero(), Real::zero()],
            [Real::zero(), inv.clone(), Real::zero(), Real::zero()],
            [Real::zero(), Real::zero(), inv.clone(), Real::zero()],
            [Real::zero(), Real::zero(), Real::zero(), inv],
        ]);
    }
    let inv00 = matrix[0][0].clone().inverse()?;
    let inv11 = matrix[1][1].clone().inverse()?;
    let inv22 = matrix[2][2].clone().inverse()?;
    let inv33 = matrix[3][3].clone().inverse()?;
    // Hyperreal benefits from emitting the matrix directly: once
    // `Matrix4Facts::is_diagonal` chose this helper, all off-diagonal zeros
    // are certified object-level facts. No second sparsity decision is needed
    // when constructing the canonical off-diagonal zeros.
    Ok([
        [inv00, Real::zero(), Real::zero(), Real::zero()],
        [Real::zero(), inv11, Real::zero(), Real::zero()],
        [Real::zero(), Real::zero(), inv22, Real::zero()],
        [Real::zero(), Real::zero(), Real::zero(), inv33],
    ])
}

/// Inverts a known upper-triangular 4×4 matrix by solving rows of `X A = I`.
///
/// Constructs four diagonal reciprocals first. Rows increase from zero to three;
/// within each row, columns increase from the diagonal, and earlier columns are
/// subtracted in increasing k order before scaling by `1/A[col,col]`. Lower
/// entries remain zero. There are ten result scalings and ten product/subtraction
/// updates, with no per-term zero pruning in these loops.
///
/// The caller establishes shape; ordinary inverse errors propagate. Generalizing
/// these three nested indices to dimension n gives cubic scalar-operation work,
/// not a quadratic full-inverse algorithm. Scalar complexity is not bounded here.
#[inline]
#[allow(clippy::needless_range_loop)]
fn invert_matrix4_by_upper_triangular(matrix: &[[Real; 4]; 4]) -> BlasResult<[[Real; 4]; 4]> {
    // Invert upper-triangular matrices via explicit fixed-size triangular
    // back-substitution. The inverse is upper-triangular with row-local support
    // `col >= row`, so avoid touching zero columns and zero RHS entries that are
    // guaranteed by the identity structure. The three-index schedule would be
    // cubic in a variable dimension; here its four-dimensional work is explicit.
    // Golub & Van Loan, *Matrix Computations* (4th ed.), §3.6.
    let inv_a00 = matrix[0][0].clone().inverse()?;
    let inv_a11 = matrix[1][1].clone().inverse()?;
    let inv_a22 = matrix[2][2].clone().inverse()?;
    let inv_a33 = matrix[3][3].clone().inverse()?;
    let inv_diagonal = [inv_a00, inv_a11, inv_a22, inv_a33];
    let mut result = [
        [Real::zero(), Real::zero(), Real::zero(), Real::zero()],
        [Real::zero(), Real::zero(), Real::zero(), Real::zero()],
        [Real::zero(), Real::zero(), Real::zero(), Real::zero()],
        [Real::zero(), Real::zero(), Real::zero(), Real::zero()],
    ];

    for row in 0..4 {
        for col in row..4 {
            let mut value = if row == col {
                Real::one()
            } else {
                Real::zero()
            };
            for k in row..col {
                value -= &result[row][k] * &matrix[k][col];
            }
            result[row][col] = value.mul_cached(&inv_diagonal[col]);
        }
    }
    Ok(result)
}

/// Inverts a known lower-triangular 4×4 matrix by solving rows of `X A = I`.
///
/// Constructs four reciprocals first. Rows increase, columns decrease from each
/// diagonal, and k increases from col+1 through row for each subtraction. Upper
/// entries remain zero. Ten result scalings and ten product/subtraction updates
/// are performed. Shape is trusted; scalar inverse errors propagate.
///
/// The variable-dimension form of this three-index schedule is cubic in scalar
/// operations; fixed dimension does not bound scalar expression or precision work.
#[inline]
#[allow(clippy::needless_range_loop)]
fn invert_matrix4_by_lower_triangular(matrix: &[[Real; 4]; 4]) -> BlasResult<[[Real; 4]; 4]> {
    // Lower-triangular inversion uses the mirrored three-index schedule of the
    // upper branch, with identity support `col <= row` and decreasing columns.
    // This avoids both the general right-division row span and unnecessary zero
    // updates when inverting known lower-triangular divisors. See
    // Golub & Van Loan, *Matrix Computations* (4th ed.), §3.6.
    let inv_a00 = matrix[0][0].clone().inverse()?;
    let inv_a11 = matrix[1][1].clone().inverse()?;
    let inv_a22 = matrix[2][2].clone().inverse()?;
    let inv_a33 = matrix[3][3].clone().inverse()?;
    let inv_diagonal = [inv_a00, inv_a11, inv_a22, inv_a33];
    let mut result = [
        [Real::zero(), Real::zero(), Real::zero(), Real::zero()],
        [Real::zero(), Real::zero(), Real::zero(), Real::zero()],
        [Real::zero(), Real::zero(), Real::zero(), Real::zero()],
        [Real::zero(), Real::zero(), Real::zero(), Real::zero()],
    ];

    for row in 0..4 {
        for col in (0..=row).rev() {
            let mut value = if row == col {
                Real::one()
            } else {
                Real::zero()
            };
            for k in (col + 1)..=row {
                value -= &result[row][k] * &matrix[k][col];
            }
            result[row][col] = value.mul_cached(&inv_diagonal[col]);
        }
    }
    Ok(result)
}

/// Checks four diagonal pivots before known upper-triangular 4×4 inversion.
///
/// All guards run in ascending index order before the first reciprocal. The first
/// zero/unknown error is returned; triangular shape is a caller precondition.
#[inline]
fn invert_matrix4_by_upper_triangular_checked(
    matrix: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    require_known_nonzero(&matrix[0][0])?;
    require_known_nonzero(&matrix[1][1])?;
    require_known_nonzero(&matrix[2][2])?;
    require_known_nonzero(&matrix[3][3])?;
    invert_matrix4_by_upper_triangular(matrix)
}

/// Checks four diagonal pivots before known lower-triangular 4×4 inversion.
///
/// Uses ascending guard order even though substitution later visits columns in
/// reverse order. Shape is trusted, and all guards precede reciprocal construction.
#[inline]
fn invert_matrix4_by_lower_triangular_checked(
    matrix: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    require_known_nonzero(&matrix[0][0])?;
    require_known_nonzero(&matrix[1][1])?;
    require_known_nonzero(&matrix[2][2])?;
    require_known_nonzero(&matrix[3][3])?;
    invert_matrix4_by_lower_triangular(matrix)
}

/// Checks four diagonal pivots before diagonal 4×4 inversion and uniform-scale reuse.
///
/// Requires known diagonal shape; guards run in ascending index order and stop
/// at the first zero/unknown. Structural equality and reciprocals occur only
/// after all four guards succeed.
#[inline]
fn invert_matrix4_by_diagonal_checked(
    matrix: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    require_known_nonzero(&matrix[0][0])?;
    require_known_nonzero(&matrix[1][1])?;
    require_known_nonzero(&matrix[2][2])?;
    require_known_nonzero(&matrix[3][3])?;
    invert_matrix4_by_diagonal(matrix)
}

/// Multiplies by a known diagonal left matrix through row scaling.
///
/// Clones its three diagonal factors and the right entries, then performs nine
/// cached scalar multiplications in row-major order. The `inv` local names hold
/// factors, not reciprocals. No zero pruning or shape validation is performed.
#[inline]
fn multiply_matrix3_by_left_diagonal(
    left: &[[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> [[Real; 3]; 3] {
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

/// Multiplies by a known diagonal right matrix through column scaling.
///
/// Clones the three factors and left entries, then performs nine row-major
/// scalar multiplications. Factors are not inverted, and nonzero status is not
/// required; the caller must establish the right operand’s diagonal shape.
#[inline]
fn multiply_matrix3_by_right_diagonal(
    left: &[[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> [[Real; 3]; 3] {
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

/// Multiplies by a known diagonal left matrix through four-row scaling.
///
/// Clones diagonal factors and right entries, then performs sixteen row-major
/// cached multiplications without dot products or per-entry zero pruning. The
/// factors may be zero; diagonal shape is trusted and no inversion occurs.
#[inline]
fn multiply_matrix4_by_left_diagonal(
    left: &[[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> [[Real; 4]; 4] {
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

/// Multiplies by a known diagonal right matrix through four-column scaling.
///
/// Clones factors and left entries, then performs sixteen row-major cached scalar
/// multiplications. No reciprocal, nonzero guard, or shape validation occurs.
#[inline]
fn multiply_matrix4_by_right_diagonal(
    left: &[[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> [[Real; 4]; 4] {
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

/// Right-divides by a known diagonal matrix through reciprocal column scaling.
///
/// Constructs three reciprocals in index order before updating any row. Mutates
/// the consumed dividend buffer, cloning each entry for its cached multiplication.
/// Right shape is trusted; scalar inverse errors return before a result is published.
#[inline]
fn divide_matrix3_by_diagonal(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> BlasResult<[[Real; 3]; 3]> {
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

/// Solves each row `x * U = left_row` for a known upper-triangular divisor.
///
/// Computes three shared diagonal reciprocals, then x0, x1, x2 in that order for
/// each row. The last numerator is an active signed sum `left2 - x0*U02 - x1*U12`;
/// it does not add a local zero-pruning pass. Shape is trusted and ordinary scalar
/// inverse failures propagate. Each row solve is quadratic in a variable dimension;
/// solving all n rows would be cubic, with scalar complexity accounted separately.
#[inline]
fn divide_matrix3_by_upper_triangular(
    mut left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> BlasResult<[[Real; 3]; 3]> {
    // Upper-triangular right-division is a fixed-size triangular solve on each row:
    // each row is independent, so we avoid building an explicit inverse and one adjugate.
    // Each row solves increasing columns. Quadratic work per row becomes cubic
    // for a variable-size square matrix with all rows as right-hand sides.
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    let inv_a22 = right[2][2].clone().inverse()?;
    trace_matrix!("helper", "divide3-upper-triangular-fused-exact");
    let one = Real::one();
    for row in &mut left {
        let x0 = row[0].clone().mul_cached(&inv_a00);
        let x1 = (row[1].clone() - (&x0 * &right[0][1])).mul_cached(&inv_a11);
        let x2 = Real::active_signed_product_sum2(
            [true, false, false],
            [[&row[2], &one], [&x0, &right[0][2]], [&x1, &right[1][2]]],
        )
        .mul_cached(&inv_a22);
        *row = [x0, x1, x2];
    }
    Ok(left)
}

/// Solves one row against a known upper-triangular 2D affine divisor.
///
/// The caller supplies exact reciprocals of `right[0,0]` and `right[1,1]`, plus an
/// exact scalar one. Computes x0 then x1; the last coordinate is
/// `row2 - x0*right02 - x1*right12`, using the zero-pruning `mul_sub_add` helper.
/// No final reciprocal is needed because the homogeneous diagonal is known one.
/// The supplied shape, reciprocal relations, and constant are not validated.
#[inline]
fn divide_matrix3_affine_upper_row(
    row: &[Real; 3],
    right: &[[Real; 3]; 3],
    inv_a00: &Real,
    inv_a11: &Real,
    one: &Real,
) -> [Real; 3] {
    let x0 = row[0].clone().mul_cached(inv_a00);
    let x1 = (row[1].clone() - (&x0 * &right[0][1])).mul_cached(inv_a11);
    let x2 = mul_sub_add(&row[2], one, &x0, &right[0][2], &x1, &right[1][2]);
    [x0, x1, x2]
}

/// Right-divides an unrestricted dividend by a known upper-triangular affine matrix.
///
/// Constructs two shared diagonal reciprocals and a scalar one, then solves all
/// three rows through [`divide_matrix3_affine_upper_row`]. The divisor must have
/// bottom row `[0,0,1]`; ordinary inverse errors propagate without a separate guard.
#[inline]
fn divide_matrix3_by_affine_upper_triangular(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> BlasResult<[[Real; 3]; 3]> {
    // A 2D affine upper-triangular divisor has an already-known homogeneous
    // diagonal of one. Reuse the retained affine/triangular facts and solve
    // only the two nontrivial diagonal lanes, avoiding a fresh inverse query
    // and multiply-by-one in the hot translation lane.
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    trace_matrix!("helper", "divide3-affine-upper-triangular-fused-exact");
    let one = Real::one();
    Ok([
        divide_matrix3_affine_upper_row(&left[0], right, &inv_a00, &inv_a11, &one),
        divide_matrix3_affine_upper_row(&left[1], right, &inv_a00, &inv_a11, &one),
        divide_matrix3_affine_upper_row(&left[2], right, &inv_a00, &inv_a11, &one),
    ])
}

/// Solves only the two nontrivial rows for known affine/upper-affine operands.
///
/// Constructs two reciprocals, solves the top rows with the shared row helper,
/// and supplies canonical bottom row `[0,0,1]`. Both shape facts are caller-owned
/// preconditions; neither is revalidated before reconstruction.
#[inline]
fn divide_matrix3_affine_by_affine_upper_triangular(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> BlasResult<[[Real; 3]; 3]> {
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    trace_matrix!(
        "helper",
        "divide3-affine-left-affine-upper-triangular-fused-exact"
    );
    let one = Real::one();
    Ok([
        divide_matrix3_affine_upper_row(&left[0], right, &inv_a00, &inv_a11, &one),
        divide_matrix3_affine_upper_row(&left[1], right, &inv_a00, &inv_a11, &one),
        [Real::zero(), Real::zero(), one],
    ])
}

/// Guards x and y pivots before all-row upper-affine right division.
///
/// Checks both nontrivial diagonal entries before any reciprocal. The homogeneous
/// one and triangular shape are caller-established, not separately validated.
#[inline]
fn divide_matrix3_by_affine_upper_triangular_checked(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> CheckedBlasResult<[[Real; 3]; 3]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    divide_matrix3_by_affine_upper_triangular(left, right)
}

/// Guards x and y before affine-by-upper-affine right division.
///
/// Checks in ascending order, then delegates to the two-row reconstruction path.
/// Requires valid affine shape on both operands and upper-triangular right shape.
#[inline]
fn divide_matrix3_affine_by_affine_upper_triangular_checked(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> CheckedBlasResult<[[Real; 3]; 3]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    divide_matrix3_affine_by_affine_upper_triangular(left, right)
}

/// Checks all three diagonal pivots before upper-triangular row solves.
///
/// Ascending guards finish before reciprocal construction. The first zero or
/// unknown result propagates; the divisor’s triangular shape remains a precondition.
#[inline]
fn divide_matrix3_by_upper_triangular_checked(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> CheckedBlasResult<[[Real; 3]; 3]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    require_known_nonzero(&right[2][2])?;
    divide_matrix3_by_upper_triangular(left, right)
}

/// Solves each row `x * L = left_row` for a known lower-triangular divisor.
///
/// Constructs three reciprocals in ascending index order, then solves x2, x1, x0.
/// The final numerator uses the active signed sum `left0 - x1*L10 - x2*L20`.
/// Rows are replaced only after their three coordinates are formed. Shape is
/// trusted and inverse errors propagate. Per-row variable-dimension work is
/// quadratic; all n rows require cubic scalar-operation work, not a total-work bound.
#[inline]
fn divide_matrix3_by_lower_triangular(
    mut left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> BlasResult<[[Real; 3]; 3]> {
    // Lower-triangular right-division mirrors the transpose scheduling of the upper form.
    // Each row solves decreasing columns; the quadratic per-row schedule is
    // repeated for all rows. No explicit inverse is constructed.
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    let inv_a22 = right[2][2].clone().inverse()?;
    trace_matrix!("helper", "divide3-lower-triangular-fused-exact");
    let one = Real::one();
    for row in &mut left {
        let x2 = row[2].clone().mul_cached(&inv_a22);
        let x1 = (row[1].clone() - (&x2 * &right[2][1])).mul_cached(&inv_a11);
        let x0 = Real::active_signed_product_sum2(
            [true, false, false],
            [[&row[0], &one], [&x1, &right[1][0]], [&x2, &right[2][0]]],
        )
        .mul_cached(&inv_a00);
        *row = [x0, x1, x2];
    }
    Ok(left)
}

/// Guards three diagonal pivots before lower-triangular row solves.
///
/// Guards ascend from zero to two, although coordinates are solved in reverse
/// order. All guards precede reciprocals; shape is trusted and zero/unknown
/// errors propagate without a partial quotient.
#[inline]
fn divide_matrix3_by_lower_triangular_checked(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> CheckedBlasResult<[[Real; 3]; 3]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    require_known_nonzero(&right[2][2])?;
    divide_matrix3_by_lower_triangular(left, right)
}

/// Computes the negated action of a supplied 2×2 block on a translation vector.
///
/// Returns `-linear * [tx, ty]` in row order through the zero-pruning `mul_add`
/// helper. For an affine inverse the caller supplies `R⁻¹`, not the original R;
/// this helper neither inverts the block nor validates that relation.
#[inline]
fn affine_inverse_translation2(linear: &[[Real; 2]; 2], tx: &Real, ty: &Real) -> [Real; 2] {
    [
        Real::zero() - &mul_add(&linear[0][0], tx, &linear[0][1], ty),
        Real::zero() - &mul_add(&linear[1][0], tx, &linear[1][1], ty),
    ]
}

/// Forms the ordered two-product sum `left_a*right_a + left_b*right_b`.
///
/// Delegates to `mul_add`, whose structural-zero gates can omit products before
/// selecting scalar arithmetic. Missing zero knowledge retains a term; no
/// independent affine, rationality, or nonzero certificate is established here.
#[inline]
fn affine_linear_dot2(left_a: &Real, right_a: &Real, left_b: &Real, right_b: &Real) -> Real {
    mul_add(left_a, right_a, left_b, right_b)
}

/// Adds the contribution of a supplied inverse translation to a homogeneous row.
///
/// Computes `row[2] + row[0]*translation[0] + row[1]*translation[1]`, with the
/// two products grouped through `mul_add`. For division by `[R t;0 1]`, the caller
/// supplies `translation = -R⁻¹t`. No inverse or relation validation occurs here.
#[inline]
fn affine_translation_column_update_from_inverse2(
    row: &[Real; 3],
    translation: &[Real; 2],
) -> Real {
    row[2].clone() + mul_add(&row[0], &translation[0], &row[1], &translation[1])
}

/// Subtracts a pure translation’s action from a row’s final coordinate.
///
/// Computes `row[2] - (row[0]*t[0] + row[1]*t[1])` through the zero-pruning
/// two-product helper. Receives the original translation, not its negation;
/// records one column-update event without validating a surrounding matrix.
#[inline]
fn affine_translation_column_subtract_update2(row: &[Real; 3], translation: [&Real; 2]) -> Real {
    trace_matrix!("helper", "affine-translation-column-subtract2");
    let shifted = mul_add(&row[0], translation[0], &row[1], translation[1]);
    row[2].clone() - shifted
}

/// Right-divides an arbitrary 3×3 matrix by a known pure translation `[I t;0 1]`.
///
/// Clones the first two columns and subtracts their action on t from each of
/// three final-column entries. Returns `Ok` without a reciprocal or scalar guard,
/// relying on the caller’s translation-only shape proof. The determinant-one
/// fact supplies no bound on translation-expression work.
#[inline]
fn divide_matrix3_by_affine_translation(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> BlasResult<[[Real; 3]; 3]> {
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

/// Right-divides a known affine matrix by a known pure translation.
///
/// Copies the 2×2 linear block and subtracts its action on the original translation
/// from the two offsets. Reconstructs bottom row `[0,0,1]` rather than inspecting
/// it. Both shape facts are preconditions; no reciprocal or shape check runs.
#[inline]
fn divide_matrix3_affine_by_affine_translation(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> BlasResult<[[Real; 3]; 3]> {
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
        [Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Borrows both operands for all-row division by a known pure translation.
///
/// Clones the first two columns and applies the same subtract-update helper as
/// the owned 3×3 path to all three rows. The dividend is unrestricted; the right
/// operand’s `[I t;0 1]` shape is trusted. No reciprocal is constructed.
#[inline]
fn divide_matrix3_by_affine_ref_translation(
    left: &[[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> BlasResult<[[Real; 3]; 3]> {
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

/// Borrows known affine/translation operands for the two-offset division shortcut.
///
/// Updates only the top two entries of the final column, copies the linear block,
/// and constructs `[0,0,1]`. Uses the same original-translation subtraction
/// schedule as the owned 3×3 helper, without revalidating either shape.
#[inline]
fn divide_matrix3_affine_by_affine_ref_translation(
    left: &[[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> BlasResult<[[Real; 3]; 3]> {
    // Borrowed affine-by-affine translation update: preserve the linear block,
    // update the two offsets in the final column, and reconstruct the bottom row.
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
        [Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Right-divides a borrowed arbitrary matrix by a caller-established affine divisor.
///
/// For `[R t;0 1]`, forms `det(R)=ad-bc`, shares one reciprocal across the 2×2
/// adjugate, then computes `u=-R⁻¹t`. Each output row is `[left_xy*R⁻¹,
/// left_last + left_xy*u]`. All rows are computed. The name excludes the
/// translation-only shortcut, not translation itself. Scalar inverse errors
/// propagate before result construction; affine shape is trusted.
#[inline]
fn divide_matrix3_by_affine_ref_no_translation(
    left: &[[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> BlasResult<[[Real; 3]; 3]> {
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
            scale_by_shared_factor(Real::zero() - &b, &right_inv_det),
        ],
        [
            scale_by_shared_factor(Real::zero() - &c, &right_inv_det),
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

/// Right-divides two borrowed known affine matrices through a 2×2 inverse.
///
/// Computes `[L R⁻¹, a - L R⁻¹t;0 1]`, sharing one ordinary determinant reciprocal.
/// Only the top two rows are evaluated and the bottom row is reconstructed; both
/// input shapes must already be established. Translation is retained despite
/// the name, and scalar inverse errors return without a partial quotient.
#[inline]
fn divide_matrix3_affine_by_affine_ref_no_translation(
    left: &[[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> BlasResult<[[Real; 3]; 3]> {
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
            scale_by_shared_factor(Real::zero() - &b, &right_inv_det),
        ],
        [
            scale_by_shared_factor(Real::zero() - &c, &right_inv_det),
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
        [Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Right-divides an unrestricted owned dividend by a known 2D affine divisor.
///
/// Forms the divisor’s 2×2 determinant, takes one reciprocal, scales the adjugate,
/// and computes inverse translation before constructing all three result rows.
/// Uses two-term linear combinations for columns and offsets rather than a full
/// 3×3 product. The right bottom row is trusted; ordinary inverse errors propagate.
#[inline]
fn divide_matrix3_by_affine(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> BlasResult<[[Real; 3]; 3]> {
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
            scale_by_shared_factor(Real::zero() - &b, &right_inv_det),
        ],
        [
            scale_by_shared_factor(Real::zero() - &c, &right_inv_det),
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

/// Right-divides known 2D affine operands with one shared determinant reciprocal.
///
/// Returns `[L R⁻¹, a - L R⁻¹t;0 1]`, using the explicit 2×2 adjugate and
/// zero-pruning two-term sums. Computes only the top two rows and constructs the
/// canonical bottom row. Consuming the dividend does not avoid per-entry clones.
/// Both affine shapes are preconditions; scalar inverse errors propagate.
#[inline]
fn divide_matrix3_affine_by_affine(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> BlasResult<[[Real; 3]; 3]> {
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
            scale_by_shared_factor(Real::zero() - &b, &right_inv_det),
        ],
        [
            scale_by_shared_factor(Real::zero() - &c, &right_inv_det),
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
        [Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Checks three diagonal divisors before reciprocal column scaling.
///
/// The caller establishes diagonal shape. Ascending known-nonzero guards stop
/// at the first zero/unknown error and all precede reciprocal construction;
/// then [`divide_matrix3_by_diagonal`] updates the consumed dividend.
#[inline]
fn divide_matrix3_by_diagonal_checked(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> CheckedBlasResult<[[Real; 3]; 3]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    require_known_nonzero(&right[2][2])?;
    divide_matrix3_by_diagonal(left, right)
}

/// Checks the 2×2 determinant before right-dividing by a known affine divisor.
///
/// The dividend is unrestricted. Forms ad-bc and requires known nonzero status
/// before its reciprocal, adjugate scaling and inverse-translation products.
/// All three result rows are computed. Shape is not validated and zero/unknown
/// errors propagate without starting a separate refinement procedure.
#[inline]
fn divide_matrix3_by_affine_checked(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> CheckedBlasResult<[[Real; 3]; 3]> {
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
            scale_by_shared_factor(Real::zero() - &b, &right_inv_det),
        ],
        [
            scale_by_shared_factor(Real::zero() - &c, &right_inv_det),
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

/// Attaches the signal to the affine divisor’s determinant before checked division.
///
/// Requires known right affine shape, with an unrestricted dividend. Computes
/// ad-bc, attaches, guards, then inverts that determinant. Later adjugate, offset
/// and row products add no local attachment or explicit polling. Scalar signal
/// propagation determines cancellation; no deadline or total-work bound is supplied.
#[inline]
fn divide_matrix3_by_affine_checked_with_abort(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Real; 3]; 3]> {
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
            scale_by_shared_factor(Real::zero() - &b, &right_inv_det),
        ],
        [
            scale_by_shared_factor(Real::zero() - &c, &right_inv_det),
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

/// Checks the right linear determinant before known affine-by-affine division.
///
/// After ad-bc is known nonzero, computes the inverse linear block and translation,
/// then only the two nontrivial result rows. Reconstructs `[0,0,1]` under the
/// caller’s shape evidence; checked scalar status does not validate either shape.
#[inline]
fn divide_matrix3_affine_by_affine_checked(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> CheckedBlasResult<[[Real; 3]; 3]> {
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
            scale_by_shared_factor(Real::zero() - &b, &right_inv_det),
        ],
        [
            scale_by_shared_factor(Real::zero() - &c, &right_inv_det),
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
        [Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Divides known affine operands after attaching the signal to the right determinant.
///
/// Attachment precedes the known-nonzero guard and inverse. The two-row block
/// formula and canonical bottom row then follow without new local polling or
/// attachment. Shape remains caller-established; scalar errors propagate before
/// a completed result, and signal acceptance is not a preemptive work bound.
#[inline]
fn divide_matrix3_affine_by_affine_checked_with_abort(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Real; 3]; 3]> {
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
            scale_by_shared_factor(Real::zero() - &b, &right_inv_det),
        ],
        [
            scale_by_shared_factor(Real::zero() - &c, &right_inv_det),
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
        [Real::zero(), Real::zero(), Real::one()],
    ])
}

/// Right-divides by a known diagonal 4×4 matrix through reciprocal column scaling.
///
/// Constructs four reciprocals in index order before mutating any row. Reuses
/// them across sixteen cached multiplications in row-major order, cloning each
/// old entry into the operation. Shape is trusted; inverse errors return before
/// a completed quotient, and the dividend need not be invertible or affine.
#[inline]
fn divide_matrix4_by_diagonal(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
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

/// Guards four diagonal values before 4×4 reciprocal column scaling.
///
/// Ascending guards all precede reciprocal construction and stop at the first
/// zero/unknown error. Diagonal shape is a caller precondition, not revalidated.
#[inline]
fn divide_matrix4_by_diagonal_checked(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    require_known_nonzero(&right[2][2])?;
    require_known_nonzero(&right[3][3])?;
    divide_matrix4_by_diagonal(left, right)
}

/// Solves each row `x*U=left_row` for a known upper-triangular 4×4 divisor.
///
/// Constructs four diagonal reciprocals once, then visits rows in order and solves
/// x0, x1, x2, x3. The final two numerators use active signed sums with three and
/// four terms, without local zero pruning. Replaces a row after all four values
/// are formed. Sixteen result scalings share the reciprocals; ordinary inverse
/// errors propagate and shape is trusted.
///
/// For a variable dimension, one row is quadratic in scalar operations and all
/// n rows are cubic. This fixed-size schedule supplies no scalar-work bound.
#[inline]
#[allow(clippy::needless_range_loop)]
fn divide_matrix4_by_upper_triangular(
    mut left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
    // Each independent row solves increasing coordinates of x*U=b. The four
    // diagonal reciprocals are constructed once and shared across every row.
    // Variable-size work is quadratic per row and cubic for all n rows.
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    let inv_a22 = right[2][2].clone().inverse()?;
    let inv_a33 = right[3][3].clone().inverse()?;
    trace_matrix!("helper", "divide4-upper-triangular-fused-exact");
    // Exact Real kernels can keep each row solve as short signed product sums:
    // b_j - x_0 u_0j - ... . This follows the fraction-delay guidance
    // used by fraction-free/common-factor exact matrix methods while avoiding
    // scalar zero probes inside the hot triangular lanes.
    let one = Real::one();
    for row in &mut left {
        let x0 = row[0].clone().mul_cached(&inv_a00);
        let x1 = (row[1].clone() - (&x0 * &right[0][1])).mul_cached(&inv_a11);
        let x2 = Real::active_signed_product_sum2(
            [true, false, false],
            [[&row[2], &one], [&x0, &right[0][2]], [&x1, &right[1][2]]],
        )
        .mul_cached(&inv_a22);
        let x3 = Real::active_signed_product_sum2(
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
    Ok(left)
}

/// Solves one row against a known upper-triangular 3D affine divisor.
///
/// The caller supplies exact reciprocals of the first three diagonal entries and
/// a scalar one. Solves x0, x1, x2 with three cached scalings; x3 is the four-term
/// active numerator with no scaling because the homogeneous diagonal is one.
/// No local zero-pruning pass or validation of shape, reciprocals, or the constant
/// is performed. The row itself need not be an affine bottom row or a point.
#[inline]
fn divide_matrix4_affine_upper_row(
    row: &[Real; 4],
    right: &[[Real; 4]; 4],
    inv_a00: &Real,
    inv_a11: &Real,
    inv_a22: &Real,
    one: &Real,
) -> [Real; 4] {
    let x0 = row[0].clone().mul_cached(inv_a00);
    let x1 = (row[1].clone() - (&x0 * &right[0][1])).mul_cached(inv_a11);
    let x2 = Real::active_signed_product_sum2(
        [true, false, false],
        [[&row[2], one], [&x0, &right[0][2]], [&x1, &right[1][2]]],
    )
    .mul_cached(inv_a22);
    let x3 = Real::active_signed_product_sum2(
        [true, false, false, false],
        [
            [&row[3], one],
            [&x0, &right[0][3]],
            [&x1, &right[1][3]],
            [&x2, &right[2][3]],
        ],
    );
    [x0, x1, x2, x3]
}

/// Right-divides any matrix by a known upper-triangular 3D affine divisor.
///
/// Constructs three shared reciprocals, then applies the affine row helper to
/// all four rows: twelve result scalings and four unscaled offset updates. The
/// caller establishes the unit homogeneous diagonal and bottom-row zeros.
/// Ordinary scalar inverse errors propagate before publishing the result.
#[inline]
fn divide_matrix4_by_affine_upper_triangular(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
    // A 3D affine upper-triangular divisor has homogeneous diagonal one and
    // zero bottom-row support. Solve only the three linear diagonal lanes and
    // leave the translation column as a fused affine update.
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    let inv_a22 = right[2][2].clone().inverse()?;
    trace_matrix!("helper", "divide4-affine-upper-triangular-fused-exact");
    let one = Real::one();
    Ok([
        divide_matrix4_affine_upper_row(&left[0], right, &inv_a00, &inv_a11, &inv_a22, &one),
        divide_matrix4_affine_upper_row(&left[1], right, &inv_a00, &inv_a11, &inv_a22, &one),
        divide_matrix4_affine_upper_row(&left[2], right, &inv_a00, &inv_a11, &inv_a22, &one),
        divide_matrix4_affine_upper_row(&left[3], right, &inv_a00, &inv_a11, &inv_a22, &one),
    ])
}

/// Solves the top three rows for known affine/upper-affine operands.
///
/// Shares three reciprocal pivots across nine result scalings and three offset
/// updates, then constructs bottom row `[0,0,0,1]`. Both affine shapes and the
/// right triangular structure are caller preconditions, not rechecked here.
#[inline]
fn divide_matrix4_affine_by_affine_upper_triangular(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    let inv_a22 = right[2][2].clone().inverse()?;
    trace_matrix!(
        "helper",
        "divide4-affine-left-affine-upper-triangular-fused-exact"
    );
    let one = Real::one();
    Ok([
        divide_matrix4_affine_upper_row(&left[0], right, &inv_a00, &inv_a11, &inv_a22, &one),
        divide_matrix4_affine_upper_row(&left[1], right, &inv_a00, &inv_a11, &inv_a22, &one),
        divide_matrix4_affine_upper_row(&left[2], right, &inv_a00, &inv_a11, &inv_a22, &one),
        [Real::zero(), Real::zero(), Real::zero(), one],
    ])
}

/// Guards three nontrivial pivots before all-row upper-affine 4×4 division.
///
/// Checks indices zero through two before any reciprocal. The unit homogeneous
/// diagonal and triangular/affine shape are trusted, with zero/unknown failures
/// propagated from the guards.
#[inline]
fn divide_matrix4_by_affine_upper_triangular_checked(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    require_known_nonzero(&right[2][2])?;
    divide_matrix4_by_affine_upper_triangular(left, right)
}

/// Guards three pivots before the affine-by-upper-affine reconstruction path.
///
/// Ascending guards precede reciprocal construction. Both affine shapes and the
/// right triangular pattern must be valid before calling this helper.
#[inline]
fn divide_matrix4_affine_by_affine_upper_triangular_checked(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    require_known_nonzero(&right[2][2])?;
    divide_matrix4_affine_by_affine_upper_triangular(left, right)
}

/// Checks all four diagonal pivots before upper-triangular row division.
///
/// Guards run in ascending order, stopping at the first zero/unknown, then
/// delegate without extra predicates. The right triangular shape is trusted.
#[inline]
fn divide_matrix4_by_upper_triangular_checked(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    require_known_nonzero(&right[2][2])?;
    require_known_nonzero(&right[3][3])?;
    divide_matrix4_by_upper_triangular(left, right)
}

/// Solves each row `x*L=left_row` for a known lower-triangular 4×4 divisor.
///
/// Constructs four reciprocals in ascending index order; coordinates are then
/// solved x3, x2, x1, x0. The final two numerators use active signed sums with
/// three and four terms. Replaces each row after all values are formed, using
/// sixteen result scalings overall. No shape validation or local zero pruning
/// is added. Ordinary inverse errors propagate.
///
/// Quadratic scalar-operation work per variable-size row becomes cubic for all
/// n rows; neither count bounds scalar expression growth or cancellation latency.
#[inline]
#[allow(clippy::needless_range_loop)]
fn divide_matrix4_by_lower_triangular(
    mut left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
    // Each independent row solves decreasing coordinates of x*L=b, sharing
    // four reciprocals across all rows. No other dividend row is consulted.
    let inv_a00 = right[0][0].clone().inverse()?;
    let inv_a11 = right[1][1].clone().inverse()?;
    let inv_a22 = right[2][2].clone().inverse()?;
    let inv_a33 = right[3][3].clone().inverse()?;
    trace_matrix!("helper", "divide4-lower-triangular-fused-exact");
    let one = Real::one();
    for row in &mut left {
        let x3 = row[3].clone().mul_cached(&inv_a33);
        let x2 = (row[2].clone() - (&x3 * &right[3][2])).mul_cached(&inv_a22);
        let x1 = Real::active_signed_product_sum2(
            [true, false, false],
            [[&row[1], &one], [&x2, &right[2][1]], [&x3, &right[3][1]]],
        )
        .mul_cached(&inv_a11);
        let x0 = Real::active_signed_product_sum2(
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
    Ok(left)
}

/// Guards four diagonal pivots before lower-triangular row division.
///
/// Guards ascend from zero to three even though coordinates are later solved
/// in reverse order. All precede reciprocals; shape is a caller precondition
/// and zero/unknown errors propagate without a partial quotient.
#[inline]
fn divide_matrix4_by_lower_triangular_checked(
    left: [[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    require_known_nonzero(&right[0][0])?;
    require_known_nonzero(&right[1][1])?;
    require_known_nonzero(&right[2][2])?;
    require_known_nonzero(&right[3][3])?;
    divide_matrix4_by_lower_triangular(left, right)
}

/// Computes the right quotient X satisfying `X * right = left` by structural dispatch.
///
/// Collects right facts first, then tries identity, diagonal, pure translation,
/// upper-affine with a nondiagonal linear block, other upper triangular, lower
/// triangular, and remaining affine shapes, in that order. Upper-triangular
/// handling excludes affine divisors with diagonal linear blocks so their
/// translation-aware specialization remains reachable. Left shape facts are
/// collected only inside affine cases, where an affine dividend permits bottom-row
/// reconstruction; the dividend need not be invertible.
///
/// The remaining schedule uses a shared adjugate for a non-dyadic rational divisor,
/// or a dyadic divisor with a rational dividend. Otherwise it solves
/// `rightᵀ * Xᵀ = leftᵀ`. The adjugate route takes one determinant reciprocal before
/// forming and scaling `left * adj(right)`. Ordinary scalar errors propagate;
/// structural recognition and rational-class selection do not prove nonsingularity.
fn right_divide_matrix3(left: [[Real; 3]; 3], right: [[Real; 3]; 3]) -> BlasResult<[[Real; 3]; 3]> {
    let right_facts = matrix3_facts(&right);

    if right_facts.is_identity {
        trace_matrix!("helper", "right-divide3-identity");
        return Ok(left);
    }
    if right_facts.is_diagonal {
        trace_matrix!("helper", "right-divide3-diagonal");
        return divide_matrix3_by_diagonal(left, &right);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix3_facts(&left);
        if left_facts.is_affine {
            trace_matrix!("helper", "right-divide3-affine-left-affine-translation");
            return divide_matrix3_affine_by_affine_translation(left, &right);
        }
        trace_matrix!("helper", "right-divide3-affine-by-translation");
        return divide_matrix3_by_affine_translation(left, &right);
    }
    if right_facts.is_affine && right_facts.is_upper_triangular && !right_facts.linear_is_diagonal {
        let left_facts = matrix3_facts(&left);
        if left_facts.is_affine {
            trace_matrix!(
                "helper",
                "right-divide3-affine-left-affine-upper-triangular"
            );
            return divide_matrix3_affine_by_affine_upper_triangular(left, &right);
        }
        trace_matrix!("helper", "right-divide3-affine-upper-triangular");
        return divide_matrix3_by_affine_upper_triangular(left, &right);
    }
    if right_facts.is_upper_triangular && !(right_facts.is_affine && right_facts.linear_is_diagonal)
    {
        trace_matrix!("helper", "right-divide3-upper-triangular");
        // Solve rows against the known triangular divisor with shared reciprocals.
        return divide_matrix3_by_upper_triangular(left, &right);
    }
    if right_facts.is_lower_triangular {
        trace_matrix!("helper", "right-divide3-lower-triangular");
        return divide_matrix3_by_lower_triangular(left, &right);
    }
    if right_facts.is_affine {
        // This remaining affine case needs left shape facts for bottom-row reconstruction.
        let left_facts = matrix3_facts(&left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        trace_matrix!("helper", "right-divide3-affine");
        // Reuse the collected left-affine and right-linear-diagonal flags.
        if left_is_affine {
            if right_linear_is_diagonal {
                trace_matrix!("helper", "right-divide3-affine-left-affine-linear-diagonal");
                return divide_matrix3_affine_by_affine_linear_diagonal(left, &right);
            }
            trace_matrix!("helper", "right-divide3-affine-left-affine");
            return divide_matrix3_affine_by_affine(left, &right);
        }
        if right_linear_is_diagonal {
            trace_matrix!("helper", "right-divide3-affine-linear-diagonal");
            return divide_matrix3_by_affine_linear_diagonal(left, &right);
        }
        return divide_matrix3_by_affine(left, &right);
    }
    if !prefer_shared_adjugate_right_division(&left, &right) {
        trace_matrix!("helper", "right-divide3-gauss-jordan");
        return Ok(transpose_array3(solve_left_system3(
            transpose_array3(right),
            transpose_array3(left),
        )?));
    }

    trace_matrix!("helper", "right-divide3-shared-adjugate");
    // Construct one determinant reciprocal, then scale the adjugate product last.
    let (adjugate, det) = matrix3_adjugate_and_determinant(&right);
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(
        multiply_arrays3_with_exact_dense_certificate(left, adjugate),
        &inv_det,
    ))
}

/// Borrows both matrices for right division with the owned path's shape priority.
///
/// Uses the ordered structural cases of [`right_divide_matrix3`], cloning the
/// identity result and working storage required by diagonal/triangular helpers.
/// Borrowed affine helpers retain the inputs through row construction. The
/// rational-class selector checks the divisor first: non-dyadic rational data
/// qualifies directly; a dyadic divisor also requires a rational dividend.
///
/// The fallback clones directly into transposed solve buffers. The shared-adjugate
/// route builds the divisor's adjugate, takes its determinant reciprocal and
/// borrows the dividend through multiplication. It does not eliminate all scalar
/// cloning, impose a scalar-work bound or require the dividend to be nonsingular.
fn right_divide_matrix3_ref(
    left: &[[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> BlasResult<[[Real; 3]; 3]> {
    let right_facts = matrix3_facts(right);

    if right_facts.is_identity {
        trace_matrix!("helper", "right-divide3-ref-identity");
        return Ok(left.clone());
    }
    if right_facts.is_diagonal {
        trace_matrix!("helper", "right-divide3-ref-diagonal");
        return divide_matrix3_by_diagonal(left.clone(), right);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix3_facts(left);
        if left_facts.is_affine {
            trace_matrix!("helper", "right-divide3-ref-affine-left-affine-translation");
            return divide_matrix3_affine_by_affine_ref_translation(left, right);
        }
        trace_matrix!("helper", "right-divide3-ref-affine-by-translation");
        return divide_matrix3_by_affine_ref_translation(left, right);
    }
    if right_facts.is_affine && right_facts.is_upper_triangular && !right_facts.linear_is_diagonal {
        let left_facts = matrix3_facts(left);
        if left_facts.is_affine {
            trace_matrix!(
                "helper",
                "right-divide3-ref-affine-left-affine-upper-triangular"
            );
            return divide_matrix3_affine_by_affine_upper_triangular(left.clone(), right);
        }
        trace_matrix!("helper", "right-divide3-ref-affine-upper-triangular");
        return divide_matrix3_by_affine_upper_triangular(left.clone(), right);
    }
    if right_facts.is_upper_triangular && !(right_facts.is_affine && right_facts.linear_is_diagonal)
    {
        trace_matrix!("helper", "right-divide3-ref-upper-triangular");
        return divide_matrix3_by_upper_triangular(left.clone(), right);
    }
    if right_facts.is_lower_triangular {
        trace_matrix!("helper", "right-divide3-ref-lower-triangular");
        return divide_matrix3_by_lower_triangular(left.clone(), right);
    }
    if right_facts.is_affine {
        // Borrowed forms keep the same lazy policy as owned division so
        // non-affine right divisors avoid unnecessary structure probes.
        let left_facts = matrix3_facts(left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        trace_matrix!("helper", "right-divide3-ref-affine");
        // Same affine-flag reuse as owned division, preserving borrowed
        // dispatch shapes while avoiding duplicate `matrix3_is_affine` scans.
        if left_is_affine {
            if right_linear_is_diagonal {
                trace_matrix!(
                    "helper",
                    "right-divide3-ref-affine-left-affine-linear-diagonal"
                );
                return divide_matrix3_affine_by_affine_ref_linear_diagonal(left, right);
            }
            trace_matrix!("helper", "right-divide3-ref-affine-left-affine");
            return divide_matrix3_affine_by_affine_ref_no_translation(left, right);
        }
        if right_linear_is_diagonal {
            trace_matrix!("helper", "right-divide3-ref-affine-linear-diagonal");
            return divide_matrix3_by_affine_ref_linear_diagonal(left, right);
        }
        return divide_matrix3_by_affine_ref_no_translation(left, right);
    }
    if !prefer_shared_adjugate_right_division_ref3(left, right) {
        trace_matrix!("helper", "right-divide3-ref-gauss-jordan");
        // Borrowed right-division is implemented as a left solve on transposes.
        // Clone directly into transposed working storage instead of cloning both
        // matrices and dispatching through the owned `/` implementation.
        return Ok(transpose_array3(solve_left_system3(
            transpose_array3_ref(right),
            transpose_array3_ref(left),
        )?));
    }

    trace_matrix!("helper", "right-divide3-ref-shared-adjugate");
    // Keep the dividend borrowed through multiplication by the constructed adjugate.
    let (adjugate, det) = matrix3_adjugate_and_determinant(right);
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(
        multiply_arrays3_ref_with_exact_dense_certificate(left, &adjugate),
        &inv_det,
    ))
}

/// Right-divides with known-nonzero guards on the selected reciprocal schedule.
///
/// Preserves [`right_divide_matrix3`]'s structural and rational-class dispatch.
/// Identity returns the dividend and translation needs no reciprocal. Diagonal,
/// triangular and affine cases use their checked guards; other divisors use checked
/// transposed Gauss–Jordan or a shared adjugate whose determinant is guarded before
/// inversion. The first zero/unknown guard error propagates without a partial
/// quotient. Checked scalar knowledge is not a universal singularity decision or
/// a separate refinement policy.
fn right_divide_matrix3_checked(
    left: [[Real; 3]; 3],
    right: [[Real; 3]; 3],
) -> CheckedBlasResult<[[Real; 3]; 3]> {
    let right_facts = matrix3_facts(&right);

    if right_facts.is_identity {
        trace_matrix!("helper", "right-divide3-checked-identity");
        return Ok(left);
    }
    if right_facts.is_diagonal {
        trace_matrix!("helper", "right-divide3-checked-diagonal");
        return divide_matrix3_by_diagonal_checked(left, &right);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix3_facts(&left);
        if left_facts.is_affine {
            trace_matrix!(
                "helper",
                "right-divide3-checked-affine-left-affine-translation"
            );
            return divide_matrix3_affine_by_affine_translation(left, &right);
        }
        trace_matrix!("helper", "right-divide3-checked-affine-by-translation");
        return divide_matrix3_by_affine_translation(left, &right);
    }
    if right_facts.is_affine && right_facts.is_upper_triangular && !right_facts.linear_is_diagonal {
        let left_facts = matrix3_facts(&left);
        if left_facts.is_affine {
            trace_matrix!(
                "helper",
                "right-divide3-checked-affine-left-affine-upper-triangular"
            );
            return divide_matrix3_affine_by_affine_upper_triangular_checked(left, &right);
        }
        trace_matrix!("helper", "right-divide3-checked-affine-upper-triangular");
        return divide_matrix3_by_affine_upper_triangular_checked(left, &right);
    }
    if right_facts.is_upper_triangular && !(right_facts.is_affine && right_facts.linear_is_diagonal)
    {
        trace_matrix!("helper", "right-divide3-checked-upper-triangular");
        return divide_matrix3_by_upper_triangular_checked(left, &right);
    }
    if right_facts.is_lower_triangular {
        trace_matrix!("helper", "right-divide3-checked-lower-triangular");
        return divide_matrix3_by_lower_triangular_checked(left, &right);
    }
    if right_facts.is_affine {
        // Inspect left shape only when choosing this affine specialization.
        let left_facts = matrix3_facts(&left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        trace_matrix!("helper", "right-divide3-checked-affine");
        // Reuse these flags without repeating affine predicates in the subbranches.
        if left_is_affine {
            if right_linear_is_diagonal {
                trace_matrix!(
                    "helper",
                    "right-divide3-checked-affine-left-affine-linear-diagonal"
                );
                return divide_matrix3_affine_by_affine_linear_diagonal_checked(left, &right);
            }
            trace_matrix!("helper", "right-divide3-checked-affine-left-affine");
            return divide_matrix3_affine_by_affine_checked(left, &right);
        }
        if right_linear_is_diagonal {
            trace_matrix!("helper", "right-divide3-checked-affine-linear-diagonal");
            return divide_matrix3_by_affine_linear_diagonal_checked(left, &right);
        }
        return divide_matrix3_by_affine_checked(left, &right);
    }
    if !prefer_shared_adjugate_right_division(&left, &right) {
        trace_matrix!("helper", "right-divide3-checked-gauss-jordan");
        return Ok(transpose_array3(solve_left_system3_checked(
            transpose_array3(right),
            transpose_array3(left),
        )?));
    }

    trace_matrix!("helper", "right-divide3-checked-shared-adjugate");
    let (adjugate, det) = matrix3_adjugate_and_determinant(&right);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(
        multiply_arrays3_with_exact_dense_certificate(left, adjugate),
        &inv_det,
    ))
}

/// Dispatches checked right division with branch-dependent signal attachment.
///
/// Uses the same branch order as [`right_divide_matrix3_checked`]. Identity,
/// diagonal, translation, triangular and diagonal-linear affine cases neither
/// attach nor poll this signal. General affine division attaches the 2×2
/// determinant before guarding it; the shared-adjugate route does likewise after
/// forming the adjugate and determinant. The transposed solve forwards the flag
/// to its checked pivot routine.
///
/// Later scalar work inherits the selected routine's signal dependencies. An
/// accepted signal is not an unconditional cancellation check, deadline or bound
/// on work performed before attachment. Zero/unknown guards retain their order.
fn right_divide_matrix3_checked_with_abort(
    left: [[Real; 3]; 3],
    right: [[Real; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Real; 3]; 3]> {
    let right_facts = matrix3_facts(&right);

    if right_facts.is_identity {
        trace_matrix!("helper", "right-divide3-checked-abort-identity");
        return Ok(left);
    }
    if right_facts.is_diagonal {
        trace_matrix!("helper", "right-divide3-checked-abort-diagonal");
        return divide_matrix3_by_diagonal_checked(left, &right);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix3_facts(&left);
        if left_facts.is_affine {
            trace_matrix!(
                "helper",
                "right-divide3-checked-abort-affine-left-affine-translation"
            );
            return divide_matrix3_affine_by_affine_translation(left, &right);
        }
        trace_matrix!(
            "helper",
            "right-divide3-checked-abort-affine-by-translation"
        );
        return divide_matrix3_by_affine_translation(left, &right);
    }
    if right_facts.is_affine && right_facts.is_upper_triangular && !right_facts.linear_is_diagonal {
        let left_facts = matrix3_facts(&left);
        if left_facts.is_affine {
            trace_matrix!(
                "helper",
                "right-divide3-checked-abort-affine-left-affine-upper-triangular"
            );
            return divide_matrix3_affine_by_affine_upper_triangular_checked(left, &right);
        }
        trace_matrix!(
            "helper",
            "right-divide3-checked-abort-affine-upper-triangular"
        );
        return divide_matrix3_by_affine_upper_triangular_checked(left, &right);
    }
    if right_facts.is_upper_triangular && !(right_facts.is_affine && right_facts.linear_is_diagonal)
    {
        trace_matrix!("helper", "right-divide3-checked-abort-upper-triangular");
        return divide_matrix3_by_upper_triangular_checked(left, &right);
    }
    if right_facts.is_lower_triangular {
        trace_matrix!("helper", "right-divide3-checked-abort-lower-triangular");
        return divide_matrix3_by_lower_triangular_checked(left, &right);
    }
    if right_facts.is_affine {
        // Keep abort-aware checked code on the same fact-on-demand fast path as
        // its non-abort counterpart.
        let left_facts = matrix3_facts(&left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        trace_matrix!("helper", "right-divide3-checked-abort-affine");
        // Reuse the left-affine flag for the two possible linear-block shapes.
        if left_is_affine {
            if right_linear_is_diagonal {
                trace_matrix!(
                    "helper",
                    "right-divide3-checked-abort-affine-left-affine-linear-diagonal"
                );
                return divide_matrix3_affine_by_affine_linear_diagonal_checked(left, &right);
            }
            trace_matrix!("helper", "right-divide3-checked-abort-affine-left-affine");
            return divide_matrix3_affine_by_affine_checked_with_abort(left, &right, signal);
        }
        if right_linear_is_diagonal {
            trace_matrix!(
                "helper",
                "right-divide3-checked-abort-affine-linear-diagonal"
            );
            return divide_matrix3_by_affine_linear_diagonal_checked(left, &right);
        }
        return divide_matrix3_by_affine_checked_with_abort(left, &right, signal);
    }
    if !prefer_shared_adjugate_right_division(&left, &right) {
        trace_matrix!("helper", "right-divide3-checked-abort-gauss-jordan");
        return Ok(transpose_array3(solve_left_system3_checked_with_abort(
            transpose_array3(right),
            transpose_array3(left),
            signal,
        )?));
    }

    trace_matrix!("helper", "right-divide3-checked-abort-shared-adjugate");
    let (adjugate, det) = matrix3_adjugate_and_determinant(&right);
    let det = with_abort(det, signal);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(
        multiply_arrays3_with_exact_dense_certificate(left, adjugate),
        &inv_det,
    ))
}

/// Computes `left * right⁻¹`, preferring an early rational cofactor schedule.
///
/// Before collecting right shape facts, tests three nonzero witnesses and exposed
/// rational representation via [`can_use_dense_exact_shared_adjugate4`]. Otherwise
/// tries identity, diagonal, translation, nondiagonal-linear upper affine, other
/// upper triangular, lower triangular and remaining affine cases, in that order.
/// Affine divisors with diagonal linear blocks bypass the general upper-triangular
/// case. Left shape facts are collected only for affine cases.
///
/// For a non-affine dividend and diagonal-linear affine divisor, known nonzero
/// linear diagonals select ordinary division; missing knowledge selects checked
/// guards even on this ordinary API. The affine-dividend branch instead calls
/// its ordinary specialized helper. The final selector uses shared adjugate
/// arithmetic for any rational divisor, without requiring a rational dividend;
/// otherwise it solves the transposed left system. The ordinary shared route
/// inverts the determinant before constructing the adjugate and product. No route
/// requires an invertible dividend or treats a zero dividend as a singularity bypass.
fn right_divide_matrix4(left: [[Real; 4]; 4], right: [[Real; 4]; 4]) -> BlasResult<[[Real; 4]; 4]> {
    if can_use_dense_exact_shared_adjugate4(&right) {
        trace_matrix!("helper", "right-divide4-dense-exact-shared-adjugate");
        return right_divide_matrix4_dense_exact_shared(&left, &right);
    }
    let right_facts = matrix4_facts(&right);
    if right_facts.is_identity {
        trace_matrix!("helper", "right-divide4-identity");
        return Ok(left);
    }
    if right_facts.is_diagonal {
        trace_matrix!("helper", "right-divide4-diagonal");
        return divide_matrix4_by_diagonal(left, &right);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix4_facts(&left);
        if left_facts.is_affine {
            trace_matrix!("helper", "right-divide4-affine-left-affine-translation");
            return divide_matrix4_affine_by_affine_translation(left, &right);
        }
        trace_matrix!("helper", "right-divide4-affine-by-translation");
        return divide_matrix4_by_affine_translation(left, &right);
    }
    if right_facts.is_affine && right_facts.is_upper_triangular && !right_facts.linear_is_diagonal {
        let left_facts = matrix4_facts(&left);
        if left_facts.is_affine {
            trace_matrix!(
                "helper",
                "right-divide4-affine-left-affine-upper-triangular"
            );
            return divide_matrix4_affine_by_affine_upper_triangular(left, &right);
        }
        trace_matrix!("helper", "right-divide4-affine-upper-triangular");
        return divide_matrix4_by_affine_upper_triangular(left, &right);
    }
    if right_facts.is_upper_triangular && !(right_facts.is_affine && right_facts.linear_is_diagonal)
    {
        // Each row solves against the triangular divisor independently. Variable-size
        // scalar-operation work is quadratic per row, cubic for all n rows.
        trace_matrix!("helper", "right-divide4-upper-triangular");
        return divide_matrix4_by_upper_triangular(left, &right);
    }
    if right_facts.is_lower_triangular {
        // The lower-triangular branch is symmetric to the upper case and keeps
        // one-pass recurrence structure with cached diagonal reciprocals.
        trace_matrix!("helper", "right-divide4-lower-triangular");
        return divide_matrix4_by_lower_triangular(left, &right);
    }
    if right_facts.is_affine {
        // Only affine specialization needs the dividend's affine shape flag.
        let left_facts = matrix4_facts(&left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        trace_matrix!("helper", "right-divide4-affine");
        // Reuse both shape flags across the affine subbranches.
        if left_is_affine {
            trace_matrix!("helper", "right-divide4-affine-left-affine");
            if right_linear_is_diagonal {
                trace_matrix!("helper", "right-divide4-affine-left-affine-linear-diagonal");
                return divide_matrix4_affine_by_affine_linear_diagonal(left, &right);
            }
            return divide_matrix4_affine_by_affine_no_translation(left, &right);
        }
        if right_linear_is_diagonal {
            trace_matrix!("helper", "right-divide4-affine-linear-diagonal");
            if right_facts.affine_linear_diagonal_is_definitely_nonzero {
                trace_matrix!(
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
        trace_matrix!("helper", "right-divide4-gauss-jordan");
        return Ok(transpose_array4(solve_left_system4(
            transpose_array4(right),
            transpose_array4(left),
        )?));
    }

    trace_matrix!("helper", "right-divide4-shared-adjugate");
    let (s, c) = matrix4_factors(&right);
    let det = determinant4_from_factors(&s, &c);
    let inv_det = det.inverse()?;
    let adjugate = matrix4_adjugate_from_factors(&right, &s, &c);
    Ok(scale_matrix4(
        multiply_arrays4_ref_with_dense_certificate(&left, &adjugate),
        &inv_det,
    ))
}

/// Recognizes the early rational cofactor route without collecting full matrix facts.
///
/// First requires known nonzero entries (1,0), (0,1), (3,0), in that order, through
/// [`matrix4_is_definitely_dense_for_inverse`]. Only then classifies all exposed
/// scalar representations as rational. The three witnesses exclude triangular
/// and affine shortcuts; they do not certify full entry density or invertibility.
/// A missing witness or rational representation returns false, leaving later
/// structural dispatch to choose a schedule.
#[inline]
fn can_use_dense_exact_shared_adjugate4(right: &[[Real; 4]; 4]) -> bool {
    matrix4_is_definitely_dense_for_inverse(right)
        && matrix4_exact_rational_kind(right) != ExactRationalKind::NonRational
}

/// Right-divides borrowed operands using rational minors and one determinant reciprocal.
///
/// Called after the early rational-divisor guard; the known-rational scalar
/// kernels require rational right entries, including any zeros. Forms two sets
/// of six 2×2 minors, their determinant, its reciprocal, then the adjugate. A
/// rational dividend selects the known-rational product kernel; other dividends
/// use fresh density checks and the ordinary multiplication fallback as needed.
/// Scales the product last. Ordinary inverse errors propagate before the adjugate
/// and product, and neither operand is assumed fully dense merely by this name.
#[inline]
fn right_divide_matrix4_dense_exact_shared(
    left: &[[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
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

/// Guards the determinant in the early known-rational 4×4 cofactor schedule.
///
/// Requires the same rational-divisor precondition as
/// [`right_divide_matrix4_dense_exact_shared`]. Constructs minors and determinant,
/// requires known nonzero status, then inverts, constructs the adjugate, selects
/// the product kernel from dividend rationality and scales. A guard failure stops
/// before inverse/product work; the early nonzero-entry witnesses alone do not
/// establish this determinant condition.
#[inline]
fn right_divide_matrix4_dense_exact_shared_checked(
    left: &[[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
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

/// Attaches a signal to the rational determinant before checked shared division.
///
/// Requires the early route's rational-divisor precondition. Minor and determinant
/// construction precedes attachment; the known-nonzero guard and inverse follow
/// it. Adjugate construction, dividend classification, product and final scaling
/// add no explicit polling or attachment. Scalar propagation determines subsequent
/// cancellation, so this does not bound the pre-attachment or total work.
#[inline]
fn right_divide_matrix4_dense_exact_shared_checked_with_abort(
    left: &[[Real; 4]; 4],
    right: &[[Real; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Real; 4]; 4]> {
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

/// Borrows both operands for 4×4 right division, retaining the dense-first priority.
///
/// Uses the early rational-divisor guard and ordered shape cases of
/// [`right_divide_matrix4`]. Identity clones the dividend; diagonal/triangular
/// cases clone required working storage, while borrowed affine kernels construct
/// the output from references. The unrestricted diagonal-linear affine branch
/// uses ordinary scalar reciprocals, without the owned dispatcher's extra
/// known-nonzero guard fallback. Borrowing therefore does not imply identical
/// predicate or failure schedules. Scalar work can strengthen cached knowledge;
/// a later checked call may then succeed where a cold call returned unknown.
///
/// The final shared-adjugate selector admits every exposed rational divisor,
/// including non-dyadic values and symbolic dividends. Other cases clone directly
/// into transposed Gauss–Jordan buffers. Shared division takes one determinant
/// reciprocal before adjugate construction, product and final scaling.
fn right_divide_matrix4_ref(
    left: &[[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> BlasResult<[[Real; 4]; 4]> {
    if can_use_dense_exact_shared_adjugate4(right) {
        trace_matrix!("helper", "right-divide4-ref-dense-exact-shared-adjugate");
        return right_divide_matrix4_dense_exact_shared(left, right);
    }
    let right_facts = matrix4_facts(right);
    if right_facts.is_identity {
        trace_matrix!("helper", "right-divide4-ref-identity");
        return Ok(left.clone());
    }
    if right_facts.is_diagonal {
        trace_matrix!("helper", "right-divide4-ref-diagonal");
        return divide_matrix4_by_diagonal(left.clone(), right);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix4_facts(left);
        if left_facts.is_affine {
            trace_matrix!("helper", "right-divide4-ref-affine-left-affine-translation");
            return divide_matrix4_affine_by_affine_ref_translation(left, right);
        }
        trace_matrix!("helper", "right-divide4-ref-affine-by-translation");
        return divide_matrix4_by_affine_ref_assumed_affine_translation(left, right);
    }
    if right_facts.is_affine && right_facts.is_upper_triangular && !right_facts.linear_is_diagonal {
        let left_facts = matrix4_facts(left);
        if left_facts.is_affine {
            trace_matrix!(
                "helper",
                "right-divide4-ref-affine-left-affine-upper-triangular"
            );
            return divide_matrix4_affine_by_affine_upper_triangular(left.clone(), right);
        }
        trace_matrix!("helper", "right-divide4-ref-affine-upper-triangular");
        return divide_matrix4_by_affine_upper_triangular(left.clone(), right);
    }
    if right_facts.is_upper_triangular && !(right_facts.is_affine && right_facts.linear_is_diagonal)
    {
        trace_matrix!("helper", "right-divide4-ref-upper-triangular");
        return divide_matrix4_by_upper_triangular(left.clone(), right);
    }
    if right_facts.is_lower_triangular {
        trace_matrix!("helper", "right-divide4-ref-lower-triangular");
        return divide_matrix4_by_lower_triangular(left.clone(), right);
    }
    if right_facts.is_affine {
        // Collect dividend shape facts only for this affine specialization.
        let left_facts = matrix4_facts(left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        trace_matrix!("helper", "right-divide4-ref-affine");
        if left_is_affine {
            trace_matrix!("helper", "right-divide4-ref-affine-left-affine");
            if right_linear_is_diagonal {
                trace_matrix!(
                    "helper",
                    "right-divide4-ref-affine-left-affine-linear-diagonal"
                );
                return divide_matrix4_affine_by_affine_ref_linear_diagonal(left, right);
            }
            return divide_matrix4_affine_by_affine_ref_no_translation(left, right);
        }
        if right_linear_is_diagonal {
            trace_matrix!("helper", "right-divide4-ref-affine-linear-diagonal");
            return divide_matrix4_by_affine_linear_diagonal_ref(left, right);
        }
        return divide_matrix4_by_affine_ref_no_translation(left, right);
    }
    if !prefer_shared_adjugate_right_division(left, right) {
        trace_matrix!("helper", "right-divide4-ref-gauss-jordan");
        // Clone directly into transposed solve buffers. The rejected shared route
        // admits any rational divisor, not just dyadic values.
        return Ok(transpose_array4(solve_left_system4(
            transpose_array4_ref(right),
            transpose_array4_ref(left),
        )?));
    }

    trace_matrix!("helper", "right-divide4-ref-shared-adjugate");
    // Share the determinant reciprocal across the final product scaling.
    let (s, c) = matrix4_factors(right);
    let det = determinant4_from_factors(&s, &c);
    let inv_det = det.inverse()?;
    let adjugate = matrix4_adjugate_from_factors(right, &s, &c);
    Ok(scale_matrix4(
        multiply_arrays4_ref_with_dense_certificate(left, &adjugate),
        &inv_det,
    ))
}

/// Applies checked right division with early rational and structural specializations.
///
/// The early cofactor route guards its determinant. Remaining shape priority
/// matches [`right_divide_matrix4`]; identity and translation need no reciprocal.
/// Diagonal and triangular paths guard their pivots. Within diagonal-linear affine
/// cases, retained known-nonzero facts permit ordinary helpers; otherwise ascending
/// checked guards run. General affine cases use checked block inversion.
///
/// Other divisors use checked transposed Gauss–Jordan or a shared adjugate with
/// its determinant guarded before inversion and adjugate construction. Failure
/// propagates without a partial quotient. These local guards neither prove all
/// symbolic singularities nor supply a total-resource limit.
fn right_divide_matrix4_checked(
    left: [[Real; 4]; 4],
    right: [[Real; 4]; 4],
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    if can_use_dense_exact_shared_adjugate4(&right) {
        trace_matrix!(
            "helper",
            "right-divide4-checked-dense-exact-shared-adjugate"
        );
        return right_divide_matrix4_dense_exact_shared_checked(&left, &right);
    }
    let right_facts = matrix4_facts(&right);
    if right_facts.is_identity {
        trace_matrix!("helper", "right-divide4-checked-identity");
        return Ok(left);
    }
    if right_facts.is_diagonal {
        trace_matrix!("helper", "right-divide4-checked-diagonal");
        return divide_matrix4_by_diagonal_checked(left, &right);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix4_facts(&left);
        if left_facts.is_affine {
            trace_matrix!(
                "helper",
                "right-divide4-checked-affine-left-affine-translation"
            );
            return divide_matrix4_affine_by_affine_checked_assumed_affine_translation(
                left, &right,
            );
        }
        trace_matrix!("helper", "right-divide4-checked-affine-by-translation");
        return divide_matrix4_by_affine_checked_assumed_affine_translation(left, &right);
    }
    if right_facts.is_affine && right_facts.is_upper_triangular && !right_facts.linear_is_diagonal {
        let left_facts = matrix4_facts(&left);
        if left_facts.is_affine {
            trace_matrix!(
                "helper",
                "right-divide4-checked-affine-left-affine-upper-triangular"
            );
            return divide_matrix4_affine_by_affine_upper_triangular_checked(left, &right);
        }
        trace_matrix!("helper", "right-divide4-checked-affine-upper-triangular");
        return divide_matrix4_by_affine_upper_triangular_checked(left, &right);
    }
    if right_facts.is_upper_triangular && !(right_facts.is_affine && right_facts.linear_is_diagonal)
    {
        trace_matrix!("helper", "right-divide4-checked-upper-triangular");
        return divide_matrix4_by_upper_triangular_checked(left, &right);
    }
    if right_facts.is_lower_triangular {
        trace_matrix!("helper", "right-divide4-checked-lower-triangular");
        return divide_matrix4_by_lower_triangular_checked(left, &right);
    }
    if right_facts.is_affine {
        // Collect dividend shape facts for affine bottom-row reconstruction.
        let left_facts = matrix4_facts(&left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        trace_matrix!("helper", "right-divide4-checked-affine");
        if left_is_affine {
            trace_matrix!("helper", "right-divide4-checked-affine-left-affine");
            if right_linear_is_diagonal {
                trace_matrix!(
                    "helper",
                    "right-divide4-checked-affine-left-affine-linear-diagonal"
                );
                if right_facts.affine_linear_diagonal_is_definitely_nonzero {
                    trace_matrix!(
                        "helper",
                        "right-divide4-checked-affine-left-affine-linear-diagonal-known-nonzero"
                    );
                    return divide_matrix4_affine_by_affine_linear_diagonal(left, &right);
                }
                return divide_matrix4_affine_by_affine_linear_diagonal_checked(left, &right);
            }
            return divide_matrix4_affine_by_affine_checked(left, &right);
        }
        if right_linear_is_diagonal {
            trace_matrix!("helper", "right-divide4-checked-affine-linear-diagonal");
            if right_facts.affine_linear_diagonal_is_definitely_nonzero {
                trace_matrix!(
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
        trace_matrix!("helper", "right-divide4-checked-gauss-jordan");
        return Ok(transpose_array4(solve_left_system4_checked(
            transpose_array4(right),
            transpose_array4(left),
        )?));
    }

    trace_matrix!("helper", "right-divide4-checked-shared-adjugate");
    let (s, c) = matrix4_factors(&right);
    let det = determinant4_from_factors(&s, &c);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    let adjugate = matrix4_adjugate_from_factors(&right, &s, &c);
    Ok(scale_matrix4(
        multiply_arrays4_ref_with_dense_certificate(&left, &adjugate),
        &inv_det,
    ))
}

/// Routes checked 4×4 right division with the selected kernel's signal behavior.
///
/// Preserves the early rational route and shape ordering of
/// [`right_divide_matrix4_checked`]. Early and final shared-adjugate routes attach
/// the determinant after its construction and before its guard/inverse. General
/// affine and transposed-solve branches forward the flag to their inverse/pivot
/// routines. Their internal specializations determine actual attachments.
///
/// Identity, diagonal, triangular and diagonal-linear affine cases do not consume
/// the signal; translation adapters accept it but ignore it. Thus an already-set
/// flag need not reject the operation. No dispatcher-level polling, deadline or
/// work bound is added, and all checked scalar failures propagate normally.
fn right_divide_matrix4_checked_with_abort(
    left: [[Real; 4]; 4],
    right: [[Real; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    if can_use_dense_exact_shared_adjugate4(&right) {
        trace_matrix!(
            "helper",
            "right-divide4-checked-abort-dense-exact-shared-adjugate"
        );
        return right_divide_matrix4_dense_exact_shared_checked_with_abort(&left, &right, signal);
    }
    let right_facts = matrix4_facts(&right);
    if right_facts.is_identity {
        trace_matrix!("helper", "right-divide4-checked-abort-identity");
        return Ok(left);
    }
    if right_facts.is_diagonal {
        trace_matrix!("helper", "right-divide4-checked-abort-diagonal");
        return divide_matrix4_by_diagonal_checked(left, &right);
    }
    if right_facts.is_affine_translation {
        let left_facts = matrix4_facts(&left);
        if left_facts.is_affine {
            trace_matrix!(
                "helper",
                "right-divide4-checked-abort-affine-left-affine-translation"
            );
            return divide_matrix4_affine_by_affine_checked_with_abort_assumed_affine_translation(
                left, &right, signal,
            );
        }
        trace_matrix!(
            "helper",
            "right-divide4-checked-abort-affine-by-translation"
        );
        return divide_matrix4_by_affine_checked_with_abort_assumed_affine_translation(
            left, &right, signal,
        );
    }
    if right_facts.is_affine && right_facts.is_upper_triangular && !right_facts.linear_is_diagonal {
        let left_facts = matrix4_facts(&left);
        if left_facts.is_affine {
            trace_matrix!(
                "helper",
                "right-divide4-checked-abort-affine-left-affine-upper-triangular"
            );
            return divide_matrix4_affine_by_affine_upper_triangular_checked(left, &right);
        }
        trace_matrix!(
            "helper",
            "right-divide4-checked-abort-affine-upper-triangular"
        );
        return divide_matrix4_by_affine_upper_triangular_checked(left, &right);
    }
    if right_facts.is_upper_triangular && !(right_facts.is_affine && right_facts.linear_is_diagonal)
    {
        trace_matrix!("helper", "right-divide4-checked-abort-upper-triangular");
        return divide_matrix4_by_upper_triangular_checked(left, &right);
    }
    if right_facts.is_lower_triangular {
        trace_matrix!("helper", "right-divide4-checked-abort-lower-triangular");
        return divide_matrix4_by_lower_triangular_checked(left, &right);
    }
    if right_facts.is_affine {
        // Inspect dividend shape only when this affine branch needs it.
        let left_facts = matrix4_facts(&left);
        let left_is_affine = left_facts.is_affine;
        let right_linear_is_diagonal = right_facts.linear_is_diagonal;
        trace_matrix!("helper", "right-divide4-checked-abort-affine");
        if left_is_affine {
            trace_matrix!("helper", "right-divide4-checked-abort-affine-left-affine");
            if right_linear_is_diagonal {
                trace_matrix!(
                    "helper",
                    "right-divide4-checked-abort-affine-left-affine-linear-diagonal"
                );
                if right_facts.affine_linear_diagonal_is_definitely_nonzero {
                    trace_matrix!(
                        "helper",
                        "right-divide4-checked-abort-affine-left-affine-linear-diagonal-known-nonzero"
                    );
                    return divide_matrix4_affine_by_affine_linear_diagonal(left, &right);
                }
                return divide_matrix4_affine_by_affine_linear_diagonal_checked(left, &right);
            }
            return divide_matrix4_affine_by_affine_checked_with_abort(left, &right, signal);
        }
        if right_linear_is_diagonal {
            trace_matrix!(
                "helper",
                "right-divide4-checked-abort-affine-linear-diagonal"
            );
            if right_facts.affine_linear_diagonal_is_definitely_nonzero {
                trace_matrix!(
                    "helper",
                    "right-divide4-checked-abort-affine-linear-diagonal-known-nonzero"
                );
                return divide_matrix4_by_affine_linear_diagonal(left, &right);
            }
            return divide_matrix4_by_affine_linear_diagonal_checked(left, &right);
        }
        return divide_matrix4_by_affine_checked_with_abort(left, &right, signal);
    }
    if !prefer_shared_adjugate_right_division(&left, &right) {
        trace_matrix!("helper", "right-divide4-checked-abort-gauss-jordan");
        return Ok(transpose_array4(solve_left_system4_checked_with_abort(
            transpose_array4(right),
            transpose_array4(left),
            signal,
        )?));
    }

    trace_matrix!("helper", "right-divide4-checked-abort-shared-adjugate");
    let (s, c) = matrix4_factors(&right);
    let det = determinant4_from_factors(&s, &c);
    let det = with_abort(det, signal);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    let adjugate = matrix4_adjugate_from_factors(&right, &s, &c);
    Ok(scale_matrix4(
        multiply_arrays4_ref_with_dense_certificate(&left, &adjugate),
        &inv_det,
    ))
}

/// Multiplies borrowed 3×3 matrices while pruning products with definitely-zero factors.
///
/// Scans all left entries, then all right entries, in row-major order using
/// `!definitely_zero()`. The local nonzero flags mean retained lanes, not proved
/// nonzero values: unknown lanes remain active. If every lane is retained, each
/// result cell uses `Real::dot3`; this dense dispatch is weaker than a known-nonzero
/// certificate and does not imply an invertible or dense result.
///
/// Otherwise each cell intersects its row/column flags. Zero active products yield
/// exact zero, one uses ordinary scalar multiplication, two use an ordered active
/// signed sum, and three use `dot3`. Selected lanes stay in increasing index order;
/// result cells are built row-major. Scalar reducers own rational aggregation and
/// symbolic behavior. No approximation, signal attachment or total-work limit is
/// introduced by this matrix schedule.
#[inline]
fn multiply_arrays3_borrowed(left: &[[Real; 3]; 3], right: &[[Real; 3]; 3]) -> [[Real; 3]; 3] {
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
        trace_matrix!("helper", "multiply3-borrowed-dense");

        let cell = |row: usize, col: usize| {
            Real::dot3(
                [&left[row][0], &left[row][1], &left[row][2]],
                [&right[0][col], &right[1][col], &right[2][col]],
            )
        };

        return [
            [cell(0, 0), cell(0, 1), cell(0, 2)],
            [cell(1, 0), cell(1, 1), cell(1, 2)],
            [cell(2, 0), cell(2, 1), cell(2, 2)],
        ];
    }

    trace_matrix!("helper", "multiply3-borrowed-sparse");

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
            0 => Real::zero(),
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
                    Real::active_signed_product_sum2([true, true], [[l1, r1], [l2, r2]])
                } else if !p1 {
                    Real::active_signed_product_sum2([true, true], [[l0, r0], [l2, r2]])
                } else {
                    Real::active_signed_product_sum2([true, true], [[l0, r0], [l1, r1]])
                }
            }
            _ => Real::dot3([l0, l1, l2], [r0, r1, r2]),
        }
    };

    [
        [cell(0, 0), cell(0, 1), cell(0, 2)],
        [cell(1, 0), cell(1, 1), cell(1, 2)],
        [cell(2, 0), cell(2, 1), cell(2, 2)],
    ]
}

/// Builds nine row-major three-lane dot products without a matrix-level sparsity scan.
///
/// Each cell pairs the left row and right column in index order through `Real::dot3`.
/// The helper does not itself certify density or rationality; scalar dot handling
/// still applies to zeros and symbolic entries. Callers using a density shortcut
/// must establish their own scheduling evidence, and the result may contain zeros.
#[inline]
fn multiply_arrays3_dense_ref(left: &[[Real; 3]; 3], right: &[[Real; 3]; 3]) -> [[Real; 3]; 3] {
    trace_matrix!("helper", "multiply3-dense-ref");
    let cell = |row: usize, col: usize| {
        Real::dot3(
            [&left[row][0], &left[row][1], &left[row][2]],
            [&right[0][col], &right[1][col], &right[2][col]],
        )
    };

    [
        [cell(0, 0), cell(0, 1), cell(0, 2)],
        [cell(1, 0), cell(1, 1), cell(1, 2)],
        [cell(2, 0), cell(2, 1), cell(2, 2)],
    ]
}

/// Checks current dense-support evidence before multiplying two owned 3×3 matrices.
///
/// Scans the left matrix for known nonzero status, then the right only if the first
/// scan succeeds. Both must pass to borrow them into [`multiply_arrays3_dense_ref`];
/// otherwise uses ordinary owned multiplication. Despite the name, this requires
/// no rational-representation proof. Unknown rejects the shortcut, not the product.
/// An input certificate does not certify the product's support for later operations.
#[inline]
fn multiply_arrays3_with_exact_dense_certificate(
    left: [[Real; 3]; 3],
    right: [[Real; 3]; 3],
) -> [[Real; 3]; 3] {
    if matrix3_has_dense_multiply_certificate(&left)
        && matrix3_has_dense_multiply_certificate(&right)
    {
        trace_matrix!("helper", "multiply3-owned-owned-dense-certified-exact");
        return multiply_arrays3_dense_ref(&left, &right);
    }
    multiply_arrays3(left, right)
}

/// Checks left-then-right nonzero evidence for an owned/borrowed 3×3 product.
///
/// On full support certificates, borrows both operands for the dense dot kernel;
/// otherwise preserves the ordinary owned-left/borrowed-right dispatch. These are
/// current known-nonzero checks, not rationality checks. Used after an intermediate
/// square in cubing, where even a dense source can have become zero; stale source
/// support must not substitute for inspecting that intermediate.
#[inline]
fn multiply_arrays3_rhs_ref_with_exact_dense_certificate(
    left: [[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> [[Real; 3]; 3] {
    if matrix3_has_dense_multiply_certificate(&left)
        && matrix3_has_dense_multiply_certificate(right)
    {
        trace_matrix!("helper", "multiply3-owned-ref-dense-certified-exact");
        return multiply_arrays3_dense_ref(&left, right);
    }
    multiply_arrays3_rhs_ref(left, right)
}

/// Multiplies two borrowed 3×3 matrices after fresh dense-support checks.
///
/// The right scan runs only after every left entry is known nonzero. If both pass,
/// uses the direct dot kernel; a zero or unknown falls back to ordinary borrowed
/// multiplication, preserving its identity/diagonal/sparse paths. Does not establish
/// rationality, invertibility, result density or a scalar-resource bound.
#[inline]
fn multiply_arrays3_ref_with_exact_dense_certificate(
    left: &[[Real; 3]; 3],
    right: &[[Real; 3]; 3],
) -> [[Real; 3]; 3] {
    if matrix3_has_dense_multiply_certificate(left) && matrix3_has_dense_multiply_certificate(right)
    {
        trace_matrix!("helper", "multiply3-ref-ref-dense-certified-exact");
        return multiply_arrays3_dense_ref(left, right);
    }
    multiply_arrays3_ref(left, right)
}

/// Multiplies borrowed 4×4 matrices using ordered active-product subsets per cell.
///
/// Scans all sixteen left entries, then all sixteen right entries, row-major with
/// `!definitely_zero()`. A retained lane may be unknown. When all are retained,
/// constructs sixteen four-term active signed sums; the dense label is not a
/// known-nonzero or rational certificate.
///
/// Otherwise each cell intersects row/column flags, returning exact zero for no
/// products, ordinary multiplication for one, or active signed sums for two,
/// three or four. Every subset preserves increasing lane order. In particular the
/// first pair is lanes zero and one, never zero and three. The two-term arm's
/// unreachable case is excluded by the counted four booleans, not by scalar sign
/// knowledge. Cells are constructed row-major. Scalar arithmetic may simplify or
/// aggregate further; this helper adds no refinement policy or resource limit.
#[inline]
fn multiply_arrays4_borrowed(left: &[[Real; 4]; 4], right: &[[Real; 4]; 4]) -> [[Real; 4]; 4] {
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
        trace_matrix!("helper", "multiply4-borrowed-dense");
        let cell = |row: usize, col: usize| {
            let l0 = &left[row][0];
            let l1 = &left[row][1];
            let l2 = &left[row][2];
            let l3 = &left[row][3];
            let r0 = &right[0][col];
            let r1 = &right[1][col];
            let r2 = &right[2][col];
            let r3 = &right[3][col];
            Real::active_signed_product_sum2(
                [true, true, true, true],
                [[l0, r0], [l1, r1], [l2, r2], [l3, r3]],
            )
        };

        return [
            [cell(0, 0), cell(0, 1), cell(0, 2), cell(0, 3)],
            [cell(1, 0), cell(1, 1), cell(1, 2), cell(1, 3)],
            [cell(2, 0), cell(2, 1), cell(2, 2), cell(2, 3)],
            [cell(3, 0), cell(3, 1), cell(3, 2), cell(3, 3)],
        ];
    }

    trace_matrix!("helper", "multiply4-borrowed-sparse");

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
            0 => Real::zero(),
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
                        // Exactly lanes zero and one are active here. Substituting
                        // lane three would change sparse products, including those
                        // used to check triangular inverses.
                        Real::active_signed_product_sum2([true, true], [[l0, r0], [l1, r1]])
                    } else if p2 {
                        Real::active_signed_product_sum2([true, true], [[l0, r0], [l2, r2]])
                    } else {
                        Real::active_signed_product_sum2([true, true], [[l0, r0], [l3, r3]])
                    }
                } else if p1 {
                    if p2 {
                        Real::active_signed_product_sum2([true, true], [[l1, r1], [l2, r2]])
                    } else {
                        Real::active_signed_product_sum2([true, true], [[l1, r1], [l3, r3]])
                    }
                } else if p2 {
                    Real::active_signed_product_sum2([true, true], [[l2, r2], [l3, r3]])
                } else {
                    unreachable!("matrix multiply sparse branch expects exactly two active terms")
                }
            }
            3 => {
                if !p0 {
                    Real::active_signed_product_sum2(
                        [true, true, true],
                        [[l1, r1], [l2, r2], [l3, r3]],
                    )
                } else if !p1 {
                    Real::active_signed_product_sum2(
                        [true, true, true],
                        [[l0, r0], [l2, r2], [l3, r3]],
                    )
                } else if !p2 {
                    Real::active_signed_product_sum2(
                        [true, true, true],
                        [[l0, r0], [l1, r1], [l3, r3]],
                    )
                } else {
                    Real::active_signed_product_sum2(
                        [true, true, true],
                        [[l0, r0], [l1, r1], [l2, r2]],
                    )
                }
            }
            _ => Real::active_signed_product_sum2(
                [true, true, true, true],
                [[l0, r0], [l1, r1], [l2, r2], [l3, r3]],
            ),
        }
    };

    [
        [cell(0, 0), cell(0, 1), cell(0, 2), cell(0, 3)],
        [cell(1, 0), cell(1, 1), cell(1, 2), cell(1, 3)],
        [cell(2, 0), cell(2, 1), cell(2, 2), cell(2, 3)],
        [cell(3, 0), cell(3, 1), cell(3, 2), cell(3, 3)],
    ]
}

/// Constructs sixteen ordered four-term sums without matrix-level zero pruning.
///
/// Borrows each left row/right column and passes all four positive products to the
/// active scalar reducer in lane order, constructing cells row-major. Does not
/// validate a dense-support certificate, require rational entries, or prove result
/// density. Any scalar-level aggregation or simplification remains scalar-owned.
#[inline]
fn multiply_arrays4_dense_ref(left: &[[Real; 4]; 4], right: &[[Real; 4]; 4]) -> [[Real; 4]; 4] {
    trace_matrix!("helper", "multiply4-dense-ref");
    let cell = |row: usize, col: usize| {
        let l0 = &left[row][0];
        let l1 = &left[row][1];
        let l2 = &left[row][2];
        let l3 = &left[row][3];
        let r0 = &right[0][col];
        let r1 = &right[1][col];
        let r2 = &right[2][col];
        let r3 = &right[3][col];
        Real::active_signed_product_sum2(
            [true, true, true, true],
            [[l0, r0], [l1, r1], [l2, r2], [l3, r3]],
        )
    };

    [
        [cell(0, 0), cell(0, 1), cell(0, 2), cell(0, 3)],
        [cell(1, 0), cell(1, 1), cell(1, 2), cell(1, 3)],
        [cell(2, 0), cell(2, 1), cell(2, 2), cell(2, 3)],
        [cell(3, 0), cell(3, 1), cell(3, 2), cell(3, 3)],
    ]
}

/// Constructs a 4×4 product using caller-established exact-rational representations.
///
/// Both operands' current entries must expose exact rational values. Each row-major
/// cell passes all four lane-ordered positive products to the unchecked
/// known-rational reducer. Zeros are admissible; full support is not required.
///
/// No factor-class check or typed mismatch is provided. The scalar reducer reads
/// stored rational components directly, so using symbolic factors or stale evidence
/// can change the represented value. Callers must establish or validly transport
/// the rational-representation proof for these exact operands.
#[inline]
fn multiply_arrays4_dense_known_rational_ref(
    left: &[[Real; 4]; 4],
    right: &[[Real; 4]; 4],
) -> [[Real; 4]; 4] {
    trace_matrix!("helper", "multiply4-dense-known-rational-ref");
    let cell = |row: usize, col: usize| {
        let l0 = &left[row][0];
        let l1 = &left[row][1];
        let l2 = &left[row][2];
        let l3 = &left[row][3];
        let r0 = &right[0][col];
        let r1 = &right[1][col];
        let r2 = &right[2][col];
        let r3 = &right[3][col];
        Real::active_signed_product_sum2_known_exact_rational(
            [true, true, true, true],
            [[l0, r0], [l1, r1], [l2, r2], [l3, r3]],
        )
    };

    [
        [cell(0, 0), cell(0, 1), cell(0, 2), cell(0, 3)],
        [cell(1, 0), cell(1, 1), cell(1, 2), cell(1, 3)],
        [cell(2, 0), cell(2, 1), cell(2, 2), cell(2, 3)],
        [cell(3, 0), cell(3, 1), cell(3, 2), cell(3, 3)],
    ]
}

/// Ephemeral structural facts needed by ordinary matrix multiplication.
///
/// Omits the full matrix shape/mask report to collect only the predicates consumed
/// by identity and diagonal shortcuts. Positive fields justify their respective
/// shortcuts for the inspected matrix state; false fields need not prove the
/// opposite geometry. Copies are unbound snapshots, not mutation-aware caches.
#[derive(Clone, Copy)]
struct MatrixIdentityDiagonalFacts {
    /// All off-diagonal entries passed definite-zero checks and diagonal entries passed definite-one checks.
    is_identity: bool,
    /// All off-diagonal entries passed definite-zero checks; diagonal values are unrestricted.
    is_diagonal: bool,
}

/// Collects only the structural identity/diagonal facts used by one-shot products.
///
/// Checks off-diagonal entries row-major for definite zero, short-circuiting at the
/// first failure and skipping diagonal entries. Only when that succeeds does it
/// check diagonal entries for definite one in index order. A false result means
/// this structural test failed, not a proof of mathematical inequality. For `N=0`,
/// both universal tests are vacuously true; for `N=1`, diagonal is always true.
/// The returned snapshot is not tied to later mutations and must be recomputed.
#[inline]
fn matrix_identity_diagonal_facts<const N: usize>(
    matrix: &[[Real; N]; N],
) -> MatrixIdentityDiagonalFacts {
    let is_diagonal = (0..N)
        .all(|row| (0..N).all(|column| row == column || matrix[row][column].definitely_zero()));
    let is_identity = is_diagonal && (0..N).all(|index| matrix[index][index].definitely_one());
    MatrixIdentityDiagonalFacts {
        is_identity,
        is_diagonal,
    }
}

/// Consumes two 3×3 operands and selects identity, diagonal or active-cell multiplication.
///
/// Collects left identity/diagonal facts and then right facts before any branch.
/// Priority is left identity, right identity, left diagonal, right diagonal, then
/// [`multiply_arrays3_borrowed`]. Identity returns the opposite owned array;
/// diagonal paths scale rows or columns. Only the final path records the
/// owned-owned specialized event. Shape knowledge does not require invertibility,
/// and zero or unknown entries remain valid multiplication inputs.
#[inline]
fn multiply_arrays3(left: [[Real; 3]; 3], right: [[Real; 3]; 3]) -> [[Real; 3]; 3] {
    // One-shot multiplication consumes only identity and diagonal facts. Full
    // structural reports remain at explicit query/cached boundaries.
    let left_facts = matrix_identity_diagonal_facts(&left);
    let right_facts = matrix_identity_diagonal_facts(&right);
    if left_facts.is_identity {
        trace_matrix!("helper", "multiply3-owned-owned-identity-left");
        return right;
    }
    if right_facts.is_identity {
        trace_matrix!("helper", "multiply3-owned-owned-identity-right");
        return left;
    }
    if left_facts.is_diagonal {
        trace_matrix!("helper", "multiply3-owned-owned-diagonal-left");
        return multiply_matrix3_by_left_diagonal(&left, &right);
    }
    if right_facts.is_diagonal {
        trace_matrix!("helper", "multiply3-owned-owned-diagonal-right");
        return multiply_matrix3_by_right_diagonal(&left, &right);
    }

    trace_matrix!("helper", "multiply3-owned-owned-specialized");
    multiply_arrays3_borrowed(&left, &right)
}

/// Consumes two 4×4 matrices with left-first identity/diagonal shortcut priority.
///
/// Collects both fact snapshots, left then right, before returning either identity
/// operand's counterpart. Next tries left-diagonal row scaling, right-diagonal
/// column scaling, and finally [`multiply_arrays4_borrowed`]. The specialized trace
/// is emitted only on that fallback. Products retain exact scalar semantics but
/// no input shape or density claim automatically applies to their results.
#[inline]
fn multiply_arrays4(left: [[Real; 4]; 4], right: [[Real; 4]; 4]) -> [[Real; 4]; 4] {
    let left_facts = matrix_identity_diagonal_facts(&left);
    let right_facts = matrix_identity_diagonal_facts(&right);
    if left_facts.is_identity {
        trace_matrix!("helper", "multiply4-owned-owned-identity-left");
        return right;
    }
    if right_facts.is_identity {
        trace_matrix!("helper", "multiply4-owned-owned-identity-right");
        return left;
    }
    if left_facts.is_diagonal {
        trace_matrix!("helper", "multiply4-owned-owned-diagonal-left");
        return multiply_matrix4_by_left_diagonal(&left, &right);
    }
    if right_facts.is_diagonal {
        trace_matrix!("helper", "multiply4-owned-owned-diagonal-right");
        return multiply_matrix4_by_right_diagonal(&left, &right);
    }

    trace_matrix!("helper", "multiply4-owned-owned-specialized");
    multiply_arrays4_borrowed(&left, &right)
}

/// Multiplies an owned left 3×3 matrix by a borrowed right matrix.
///
/// Scans left and right identity/diagonal facts before branching, in that order.
/// Left identity clones the right array; right identity returns the owned left.
/// Then left/right diagonal scaling precedes borrowed active-cell multiplication.
/// Only the fallback records the owned-ref specialized event. Borrowing the right
/// operand does not remove scalar cloning performed by the selected kernel.
#[inline]
fn multiply_arrays3_rhs_ref(left: [[Real; 3]; 3], right: &[[Real; 3]; 3]) -> [[Real; 3]; 3] {
    let left_facts = matrix_identity_diagonal_facts(&left);
    let right_facts = matrix_identity_diagonal_facts(right);
    if left_facts.is_identity {
        trace_matrix!("helper", "multiply3-owned-ref-identity-left");
        return right.clone();
    }
    if right_facts.is_identity {
        trace_matrix!("helper", "multiply3-owned-ref-identity-right");
        return left;
    }
    if left_facts.is_diagonal {
        trace_matrix!("helper", "multiply3-owned-ref-diagonal-left");
        return multiply_matrix3_by_left_diagonal(&left, right);
    }
    if right_facts.is_diagonal {
        trace_matrix!("helper", "multiply3-owned-ref-diagonal-right");
        return multiply_matrix3_by_right_diagonal(&left, right);
    }

    trace_matrix!("helper", "multiply3-owned-ref-specialized");
    multiply_arrays3_borrowed(&left, right)
}

/// Multiplies owned/borrowed 4×4 operands using identity and diagonal facts.
///
/// Collects both snapshots before choosing left identity, right identity,
/// left-diagonal row scaling, right-diagonal column scaling or the borrowed
/// active-cell kernel. Left identity clones the right array; right identity
/// returns the left. The owned-ref specialized event belongs only to the fallback;
/// unknown lanes are retained by the selected scalar schedule.
#[inline]
fn multiply_arrays4_rhs_ref(left: [[Real; 4]; 4], right: &[[Real; 4]; 4]) -> [[Real; 4]; 4] {
    let left_facts = matrix_identity_diagonal_facts(&left);
    let right_facts = matrix_identity_diagonal_facts(right);
    if left_facts.is_identity {
        trace_matrix!("helper", "multiply4-owned-ref-identity-left");
        return right.clone();
    }
    if right_facts.is_identity {
        trace_matrix!("helper", "multiply4-owned-ref-identity-right");
        return left;
    }
    if left_facts.is_diagonal {
        trace_matrix!("helper", "multiply4-owned-ref-diagonal-left");
        return multiply_matrix4_by_left_diagonal(&left, right);
    }
    if right_facts.is_diagonal {
        trace_matrix!("helper", "multiply4-owned-ref-diagonal-right");
        return multiply_matrix4_by_right_diagonal(&left, right);
    }

    trace_matrix!("helper", "multiply4-owned-ref-specialized");
    multiply_arrays4_borrowed(&left, right)
}

/// Multiplies borrowed 3×3 operands, including borrowed-left/owned-right operator calls.
///
/// Records the ref-ref specialized event on entry, even when a shortcut follows.
/// Then collects left and right identity/diagonal snapshots before choosing left
/// identity, right identity, left diagonal or right diagonal in order. Identity
/// clones the opposite array. Otherwise delegates to the borrowed active-cell
/// kernel, retaining its lane and per-cell scalar reduction order.
#[inline]
fn multiply_arrays3_ref(left: &[[Real; 3]; 3], right: &[[Real; 3]; 3]) -> [[Real; 3]; 3] {
    // Preserve the fixed three-lane, per-cell reduction schedule and entry trace.
    trace_matrix!("helper", "multiply3-ref-ref-specialized");

    let left_facts = matrix_identity_diagonal_facts(left);
    let right_facts = matrix_identity_diagonal_facts(right);
    if left_facts.is_identity {
        trace_matrix!("helper", "multiply3-ref-ref-identity-left");
        return right.clone();
    }
    if right_facts.is_identity {
        trace_matrix!("helper", "multiply3-ref-ref-identity-right");
        return left.clone();
    }
    if left_facts.is_diagonal {
        trace_matrix!("helper", "multiply3-ref-ref-diagonal-left");
        return multiply_matrix3_by_left_diagonal(left, right);
    }
    if right_facts.is_diagonal {
        trace_matrix!("helper", "multiply3-ref-ref-diagonal-right");
        return multiply_matrix3_by_right_diagonal(left, right);
    }

    multiply_arrays3_borrowed(left, right)
}

/// Multiplies borrowed 4×4 operands through structural shortcuts and active-cell sums.
///
/// The ref-ref specialized trace precedes both fact scans and all shortcuts, so it
/// also occurs for identity/diagonal products. After left then right snapshots,
/// priority is left identity, right identity, left diagonal, right diagonal, then
/// [`multiply_arrays4_borrowed`]. Identity clones the opposite array. The public
/// borrowed-left/owned-right operator also uses this path; its owned right input
/// does not select the owned-owned dispatch.
#[inline]
fn multiply_arrays4_ref(left: &[[Real; 4]; 4], right: &[[Real; 4]; 4]) -> [[Real; 4]; 4] {
    // Preserve the fixed four-lane, per-cell reduction schedule and entry trace.
    trace_matrix!("helper", "multiply4-ref-ref-specialized");

    let left_facts = matrix_identity_diagonal_facts(left);
    let right_facts = matrix_identity_diagonal_facts(right);
    if left_facts.is_identity {
        trace_matrix!("helper", "multiply4-ref-ref-identity-left");
        return right.clone();
    }
    if right_facts.is_identity {
        trace_matrix!("helper", "multiply4-ref-ref-identity-right");
        return left.clone();
    }
    if left_facts.is_diagonal {
        trace_matrix!("helper", "multiply4-ref-ref-diagonal-left");
        return multiply_matrix4_by_left_diagonal(left, right);
    }
    if right_facts.is_diagonal {
        trace_matrix!("helper", "multiply4-ref-ref-diagonal-right");
        return multiply_matrix4_by_right_diagonal(left, right);
    }

    multiply_arrays4_borrowed(left, right)
}

/// Applies a matrix to a borrowed vector for the admitted dimensions three and four.
///
/// For `N=4`, collects minimal matrix facts, tries identity then diagonal, and
/// classifies w once. Directions omit the final column; points add that column
/// as an offset, with a diagonal-affine shortcut. Minus one and other/unknown
/// weights stay on the full homogeneous route, using three terms only where the
/// final-column coefficient is definitely zero. All four output rows are computed;
/// a projective matrix can give a direction a nonzero output weight.
///
/// For `N=3`, identity and diagonal shortcuts precede three-term row combinations.
/// The const-generic signature is implementation sharing, not general-N support:
/// callers must supply dimension three or four. No normalization to a Cartesian
/// point, checked nonzero test, signal attachment or total-work bound occurs here.
fn transform_vector_rhs_ref<const N: usize>(left: &[[Real; N]; N], right: &[Real; N]) -> [Real; N] {
    if N == 4 {
        // Probe only the identity, diagonal, affine, and translation facts read
        // below. Full public matrix reports are intentionally left to explicit
        // `structural_facts` and internal batch-dispatch boundaries.
        let left_facts = matrix4_transform_dispatch_facts(left);
        if left_facts.is_identity {
            trace_matrix!("helper", "transform-vector4-identity");
            return right.clone();
        }

        if left_facts.is_diagonal {
            trace_matrix!("helper", "transform-vector4-diagonal");
            return from_fn(|row| right[row].clone().mul_cached(&left[row][row]));
        }

        // Classify the homogeneous coordinate once. The scalar-owned combined
        // zero/one/minus-one fact gives direction/point transforms a single
        // query and keeps signed-unit projective coordinates, such as `w = -1`,
        // on the generic homogeneous path. That preserves the object-fact
        // boundary used by the generic homogeneous path.
        match right[3].zero_one_or_minus_one() {
            ZeroOneMinusOneStatus::Zero => {
                trace_matrix!("helper", "transform-vector-direction");
                if left_facts.direction_linear_is_diagonal {
                    trace_matrix!("helper", "transform-vector-direction-diagonal");
                    return from_fn(|row| {
                        if row == 3 {
                            Real::zero()
                        } else {
                            right[row].clone().mul_cached(&left[row][row])
                        }
                    });
                }
                let vector_terms = [&right[0], &right[1], &right[2]];
                return from_fn(|row| {
                    let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
                    Real::linear_combination3(matrix_terms, vector_terms)
                });
            }
            ZeroOneMinusOneStatus::One => {
                trace_matrix!("helper", "transform-vector-point");
                if left_facts.is_affine && left_facts.linear_is_diagonal {
                    trace_matrix!("helper", "transform-vector-point-affine-linear-diagonal");
                    return from_fn(|row| {
                        if row == 3 {
                            Real::one()
                        } else {
                            right[row].clone().mul_cached(&left[row][row]) + &left[row][3]
                        }
                    });
                }
                // Reuse translation-column zero facts collected by
                // `matrix4_transform_dispatch_facts`; only m33 is not part of the
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
                    let mapped = Real::linear_combination3(matrix_terms, vector_terms);
                    if translation_is_zero[row] {
                        mapped
                    } else {
                        mapped + &left[row][3]
                    }
                });
            }
            ZeroOneMinusOneStatus::MinusOne | ZeroOneMinusOneStatus::NeitherOrUnknown => {}
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
        trace_matrix!("helper", "transform-vector-full");
        from_fn(|row| {
            if translation_is_zero[row] {
                let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
                Real::linear_combination3(matrix_terms, vector_terms)
            } else {
                let matrix_terms = [&left[row][0], &left[row][1], &left[row][2], &left[row][3]];
                let vector_terms = [&right[0], &right[1], &right[2], &right[3]];
                // `Matrix4` transforms already encode translation in `left[row][3]`,
                // so this branch is a pure 4-term linear form. Keeping it on the
                // linear path avoids a redundant offset check and construction.
                Real::linear_combination4(matrix_terms, vector_terms)
            }
        })
    } else {
        // Current non-4D callers are 3D; only identity/diagonal facts are consumed.
        let left_facts = matrix_identity_diagonal_facts(left);
        if left_facts.is_identity {
            trace_matrix!("helper", "transform-vector3-identity");
            return right.clone();
        }

        if left_facts.is_diagonal {
            trace_matrix!("helper", "transform-vector3-diagonal");
            return from_fn(|row| right[row].clone().mul_cached(&left[row][row]));
        }

        let vector_terms = [&right[0], &right[1], &right[2]];
        from_fn(|row| {
            let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
            // `N != 4` for current matrix-vector callers means 3-lane
            // geometry, so only the pure linear form is valid here.
            Real::linear_combination3(matrix_terms, vector_terms)
        })
    }
}

/// Computes three row-ordered linear combinations without matrix shape dispatch.
///
/// Pairs each matrix row with all three vector coordinates. The dense name means
/// the caller bypassed identity/diagonal shortcuts; zero and symbolic entries remain
/// valid, and scalar linear-combination handling owns pruning and aggregation.
#[inline]
fn transform_vector3_rhs_dense_ref(left: &[[Real; 3]; 3], right: &[Real; 3]) -> [Real; 3] {
    let vector_terms = [&right[0], &right[1], &right[2]];
    from_fn(|row| {
        let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
        Real::linear_combination3(matrix_terms, vector_terms)
    })
}

/// Applies three retained three-term linear forms without another lattice zero scan.
///
/// Uses the active scalar constructor for each row, preserving lane order. Active
/// means scheduled for arithmetic, not proved nonzero or rational. Used after vector
/// prescaling for diagonal division; the helper neither inverts nor validates those
/// scales and adds no independent matrix-shape check.
#[inline]
fn transform_vector3_rhs_dense_active_ref(left: &[[Real; 3]; 3], right: &[Real; 3]) -> [Real; 3] {
    trace_matrix!("helper", "transform-vector3-dense-active");
    let vector_terms = [&right[0], &right[1], &right[2]];
    from_fn(|row| {
        let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
        Real::active_linear_combination3(matrix_terms, vector_terms)
    })
}

/// Applies a full homogeneous transform using supplied matrix and column facts.
///
/// Identity clones the vector; diagonal scales all four lanes. Otherwise each row
/// uses its first three columns, adding the fourth product unless the corresponding
/// column flag certifies zero. Callers route general-weight vectors here, including
/// known minus one; no point/direction classifier runs locally.
///
/// All facts must apply to this matrix state. A true zero flag authorizes omission;
/// a false flag retains the product without proving it nonzero. Does not normalize
/// output w or establish projective validity.
#[inline]
fn transform_vector4_rhs_ref_cached_with_matrix_facts(
    left: &[[Real; 4]; 4],
    right: &[Real; 4],
    translation_is_zero: &[bool; 4],
    matrix_facts: Matrix4Facts,
) -> [Real; 4] {
    if matrix_facts.is_identity {
        trace_matrix!("helper", "transform-vector4-identity");
        return right.clone();
    }

    if matrix_facts.is_diagonal {
        trace_matrix!("helper", "transform-vector4-diagonal");
        return from_fn(|row| right[row].clone().mul_cached(&left[row][row]));
    }

    // Callers enter this helper only after retained vector facts classify `w`
    // as unknown (including minus one). Known directions and points already use
    // their dedicated three-term kernels, so avoid repeating those impossible
    // classifications here.
    let vector_terms = [&right[0], &right[1], &right[2]];
    trace_matrix!("helper", "transform-vector4-full");
    from_fn(|row| {
        if translation_is_zero[row] {
            let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
            Real::linear_combination3(matrix_terms, vector_terms)
        } else {
            let matrix_terms = [&left[row][0], &left[row][1], &left[row][2], &left[row][3]];
            let vector_terms = [&right[0], &right[1], &right[2], &right[3]];
            // Keep cached batch transforms aligned with the non-cached path:
            // all homogeneous translation is already part of the 4-term linear
            // form, so no extra offset term is required.
            Real::linear_combination4(matrix_terms, vector_terms)
        }
    })
}

/// Chooses a homogeneous transform from caller-supplied vector and matrix evidence.
///
/// Directions use the three-term direction helper. Points choose all-zero column,
/// all-retained column, or per-row offset handling, in that order. Unknown means
/// not classified as w=0 or w=1, including known other weights; only that case
/// consumes the optional matrix facts or collects them lazily when absent.
///
/// The vector kind and all positive shape/zero flags must describe these operands.
/// The all-nonzero name denotes no definitely-zero column entries, not positive
/// nonzero proofs. This dispatcher does not revalidate supplied facts or attach
/// a signal; misclassified w can change the transform.
#[inline]
#[allow(clippy::too_many_arguments)]
fn transform_vector4_rhs_ref_with_facts(
    left: &[[Real; 4]; 4],
    right: &[Real; 4],
    translation_is_zero: &[bool; 4],
    all_translation_zero: bool,
    all_translation_nonzero: bool,
    direction_is_diagonal: bool,
    matrix_facts: Option<Matrix4Facts>,
    facts: Vector4GeometricFacts,
) -> [Real; 4] {
    // Trust the supplied homogeneous classification instead of probing w again.
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

/// Transforms an assumed w=0 vector while omitting the matrix's final column.
///
/// A true diagonal-direction fact must certify a diagonal top-left block and zero
/// bottom-row xyz entries. It selects three cached scales and exact output w=0.
/// Otherwise computes three-term combinations for all four rows, so a projective
/// bottom row can produce nonzero w. False need only mean missing shortcut evidence.
/// The input w and retained matrix fact are not checked here.
#[inline]
fn transform_vector4_rhs_direction_ref_cached(
    left: &[[Real; 4]; 4],
    right: &[Real; 4],
    direction_is_diagonal: bool,
) -> [Real; 4] {
    if direction_is_diagonal {
        trace_matrix!("helper", "transform-vector4-direction-diagonal-facts");
        return [
            right[0].clone().mul_cached(&left[0][0]),
            right[1].clone().mul_cached(&left[1][1]),
            right[2].clone().mul_cached(&left[2][2]),
            Real::zero(),
        ];
    }

    // No diagonal-direction shortcut was supplied. The general three-term form
    // is valid even when that structure exists but was not recognized.
    trace_matrix!("helper", "transform-vector4-batch-direction");
    let vector_terms = [&right[0], &right[1], &right[2]];
    from_fn(|row| {
        let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
        Real::linear_combination3(matrix_terms, vector_terms)
    })
}

/// Transforms an ordered batch whose vectors are already known to have w=0.
///
/// Allocates output capacity for the input length and selects the matrix schedule
/// once. A valid diagonal-direction fact gives three scales and zero w per vector;
/// otherwise four three-term row forms omit translation. No per-vector homogeneous
/// classification occurs. Empty input yields an empty vector; output allocation and
/// scalar work have no typed resource budget here.
fn transform_vector4_direction_batch_assumed_ref(
    left: &[[Real; 4]; 4],
    rhs: &[Vector4],
    direction_is_diagonal: bool,
) -> Vec<Vector4> {
    let mut transformed = Vec::with_capacity(rhs.len());
    if direction_is_diagonal {
        trace_matrix!(
            "helper",
            "transform-vector4-direction-batch-diagonal-assumed"
        );
        for vector in rhs {
            transformed.push(Vector4([
                vector.0[0].clone().mul_cached(&left[0][0]),
                vector.0[1].clone().mul_cached(&left[1][1]),
                vector.0[2].clone().mul_cached(&left[2][2]),
                Real::zero(),
            ]));
        }
        return transformed;
    }

    trace_matrix!("helper", "transform-vector4-direction-batch-linear-assumed");
    for vector in rhs {
        // Assumed w=0 removes the final-column contribution in every row.
        let vector_terms = [&vector.0[0], &vector.0[1], &vector.0[2]];
        transformed.push(Vector4(from_fn(|row| {
            let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
            Real::linear_combination3(matrix_terms, vector_terms)
        })));
    }
    transformed
}

/// Transforms an assumed point through a known affine matrix with diagonal linear block.
///
/// Computes each xyz coordinate as its cached diagonal scale plus translation,
/// then clones the input w lane. Requires w=1 and the caller-established matrix
/// shape; neither is checked. Cloning preserves the existing scalar representation,
/// whereas the corresponding batch loop constructs a fresh one. No perspective
/// division is needed under the stated affine precondition.
#[inline]
fn transform_vector4_rhs_point_affine_linear_diagonal_ref_cached(
    left: &[[Real; 4]; 4],
    right: &[Real; 4],
) -> [Real; 4] {
    // The caller establishes a diagonal affine linear block and w=1.
    trace_matrix!("helper", "transform-vector4-point-affine-linear-diagonal");
    [
        right[0].clone().mul_cached(&left[0][0]) + &left[0][3],
        right[1].clone().mul_cached(&left[1][1]) + &left[1][3],
        right[2].clone().mul_cached(&left[2][2]) + &left[2][3],
        // Preserve the existing point-weight scalar rather than constructing one.
        right[3].clone(),
    ]
}

/// Applies a pre-scaled point using a separately supplied effective homogeneous weight.
///
/// Computes each of four rows as `linear_xyz * right_xyz + last_column * w_scale`.
/// The matrix may be projective. The fourth right lane is not read: the caller must
/// keep the separate scale and its one flag consistent with the intended vector,
/// as in diagonal-divisor prescaling of an original w=1 point.
///
/// All-zero column evidence omits offsets. With all column entries retained,
/// precomputes four offsets (clones for scale one, otherwise cached products), then
/// uses active three-term forms plus those offsets. The partial case uses ordinary
/// forms and adds only retained offsets. All-nonzero means no definite-zero flags;
/// it does not certify any coefficient nonzero. No facts are validated locally.
#[inline]
fn transform_vector4_rhs_point_with_scaled_w_ref_cached(
    left: &[[Real; 4]; 4],
    right: &[Real; 4],
    translation_is_zero: &[bool; 4],
    all_translation_zero: bool,
    all_translation_nonzero: bool,
    w_scale_is_one: bool,
    w_scale: &Real,
) -> [Real; 4] {
    // A separately retained weight scales the final column in all four rows;
    // this formula also applies to projective matrices.
    let vector_terms = [&right[0], &right[1], &right[2]];
    // Precomputed translation flags allow this helper to avoid rescanning `w`-column
    // structural zeros after its caller already inspected it. That keeps this
    // all-zero/all-retained offset schedules independent of per-row zero probes.
    if all_translation_zero {
        trace_matrix!(
            "helper",
            "transform-vector4-point-scaled-w-full-no-translation"
        );
        return from_fn(|row| {
            let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
            Real::linear_combination3(matrix_terms, vector_terms)
        });
    }

    if all_translation_nonzero {
        // No final-column coefficient was definitely zero; retain all offsets.
        // Unknown coefficients do not need a nonzero proof for this sum.
        trace_matrix!("helper", "transform-vector4-point-scaled-w-full-nonzero");
        let translation: [Real; 4] = if w_scale_is_one {
            from_fn(|row| left[row][3].clone())
        } else {
            from_fn(|row| left[row][3].clone().mul_cached(w_scale))
        };
        trace_matrix!(
            "helper",
            "transform-vector4-point-scaled-w-full-nonzero-active"
        );
        return from_fn(|row| {
            let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
            Real::active_linear_combination3(matrix_terms, vector_terms) + &translation[row]
        });
    }

    // General projected-point fast path: use the 3-term spatial form and apply
    // `w'`-scaled affine offsets only where necessary.
    trace_matrix!("helper", "transform-vector4-point-scaled-w-partial");
    from_fn(|row| {
        let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
        let mapped = Real::linear_combination3(matrix_terms, vector_terms);
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

/// Applies an assumed w=1 point using per-row final-column zero evidence.
///
/// Computes four three-term xyz forms, adding the corresponding final-column entry
/// only when its flag is false. True flags must certify zero for this matrix; false
/// flags may retain unknowns. No identity/diagonal rescan or homogeneous classifier
/// runs, and a projective output need not have w=1.
#[inline]
fn transform_vector4_rhs_point_ref_cached(
    left: &[[Real; 4]; 4],
    right: &[Real; 4],
    translation_is_zero: &[bool; 4],
) -> [Real; 4] {
    // Reuse final-column zero flags without repeating matrix-shape classification.
    // Non-affine diagonal matrices can also reach this point schedule.
    trace_matrix!("helper", "transform-vector4-batch-point");
    let vector_terms = [&right[0], &right[1], &right[2]];
    from_fn(|row| {
        let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
        let mapped = Real::linear_combination3(matrix_terms, vector_terms);
        if translation_is_zero[row] {
            mapped
        } else {
            mapped + &left[row][3]
        }
    })
}

/// Adds every final-column offset to the three-term transform of an assumed point.
///
/// Computes all four rows with ordinary scalar linear combinations. The selected
/// schedule retains every offset; the caller's all-nonzero flag only means none was
/// definitely zero, so unknown offsets are valid. The input must have w=1. No local
/// zero test, affine-shape assertion or perspective normalization is performed.
#[inline]
fn transform_vector4_rhs_point_all_nonzero_ref_cached(
    left: &[[Real; 4]; 4],
    right: &[Real; 4],
) -> [Real; 4] {
    // Every final-column offset is retained, including coefficients of unknown status.
    trace_matrix!("helper", "transform-vector4-point-all-nonzero");
    let vector_terms = [&right[0], &right[1], &right[2]];
    from_fn(|row| {
        let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
        Real::linear_combination3(matrix_terms, vector_terms) + &left[row][3]
    })
}

/// Applies four three-term row forms when the entire final column is known zero.
///
/// The caller certifies all four zero coefficients, including the bottom-right
/// entry. The input w is irrelevant and is not read. This is valid for a general
/// homogeneous matrix; it does not imply affine shape or output w=1.
#[inline]
fn transform_vector4_rhs_full_no_translation_ref_cached(
    left: &[[Real; 4]; 4],
    right: &[Real; 4],
) -> [Real; 4] {
    trace_matrix!("helper", "transform-vector4-batch-full-no-translation");
    let vector_terms = [&right[0], &right[1], &right[2]];
    from_fn(|row| {
        let matrix_terms = [&left[row][0], &left[row][1], &left[row][2]];
        Real::linear_combination3(matrix_terms, vector_terms)
    })
}

/// Converts homogeneous coordinates to a Cartesian point by sharing one w reciprocal.
///
/// If w is definitely one, moves xyz directly into the point. Otherwise takes the
/// ordinary scalar inverse and scales x, y, z in order. A known zero weight returns
/// the inverse error; unknown status does not trigger a checked nonzero guard and
/// may construct a symbolic reciprocal. Success is not a separate nonzero certificate
/// or completed numerical evaluation. No epsilon, absolute-value normalization or
/// special point-at-infinity carrier is introduced.
fn point3_from_homogeneous(transformed: Vector4) -> BlasResult<Point3> {
    let [x, y, z, w] = transformed.0;
    if w.definitely_one() {
        return Ok(Point3::new(x, y, z));
    }
    let inv_w = w.inverse()?;
    Ok(Point3::new(
        x.mul_cached(&inv_w),
        y.mul_cached(&inv_w),
        z.mul_cached(&inv_w),
    ))
}

/// Borrowed matrix and minimal shape facts for one immediate 3D vector batch.
///
/// The shared reference prevents ordinary mutation of the matrix while this handle
/// exists. Construction collects identity/diagonal facts once; this is not a public
/// persistent cache or a bound on output allocation or scalar work.
#[derive(Clone, Copy)]
struct BatchTransform3<'a> {
    /// Matrix borrowed for the lifetime of this immediate batch handle.
    matrix: &'a Matrix3,
    /// Identity/diagonal snapshot collected from this matrix by the constructor.
    facts: MatrixIdentityDiagonalFacts,
}

impl<'a> BatchTransform3<'a> {
    /// Borrows the matrix and immediately collects its minimal identity/diagonal facts.
    fn new(matrix: &'a Matrix3) -> Self {
        Self {
            matrix,
            facts: matrix_identity_diagonal_facts(&matrix.0),
        }
    }

    /// Applies the three-term kernel to one vector after batch shortcut selection.
    ///
    /// The batch method handles identity/diagonal cases before calling this method.
    /// Its dense trace label does not certify nonzero entries or dense output.
    fn transform_vector(&self, rhs: &Vector3) -> Vector3 {
        // The batch entry point handles recognized identity/diagonal matrices first.
        // Other matrices need row combinations, but need not have dense support.
        trace_matrix!("helper", "transform-vector3-dense");
        Vector3(transform_vector3_rhs_dense_ref(&self.matrix.0, &rhs.0))
    }

    /// Transforms vectors in input order using the cached matrix shape once.
    ///
    /// Identity clones the slice; diagonal maps three scales per vector; the remaining
    /// route allocates capacity for the input length and uses row combinations. Empty
    /// input returns empty output. No per-vector shape report is allocated.
    fn transform_vector_batch(&self, rhs: &[Vector3]) -> Vec<Vector3> {
        if self.facts.is_identity {
            trace_matrix!("method", "transform-vector3-batch-identity");
            return rhs.to_vec();
        }
        if self.facts.is_diagonal {
            trace_matrix!("method", "transform-vector3-batch-diagonal");
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

/// Borrowed homogeneous transform with matrix and final-column facts for immediate batches.
///
/// The matrix remains shared-borrowed while the handle exists. Supplied facts must
/// belong to that same matrix; the alternate constructor does not validate their
/// origin. Cached flags select arithmetic without converting unknowns to false
/// geometry claims. This handle supplies no persistent cache, cancellation token
/// or typed allocation/work budget.
#[derive(Clone, Copy, Debug)]
struct BatchTransform4<'a> {
    /// Matrix whose entries the cached facts must describe.
    matrix: &'a Matrix4,
    /// Caller-valid or freshly collected shape report for this matrix.
    facts: Matrix4Facts,
    /// Definite-zero evidence for all four final-column entries, including m33.
    translation_is_zero: [bool; 4],
    /// All four final-column zero flags are true, permitting complete omission.
    all_translation_zero: bool,
    /// All four zero flags are false; entries are retained, possibly unknown.
    all_translation_nonzero: bool,
    /// Diagonal top-left block and zero bottom xyz row, sufficient for three direction scales.
    direction_is_diagonal: bool,
}

impl<'a> BatchTransform4<'a> {
    /// Transforms one vector with caller-valid homogeneous classification.
    ///
    /// Identity is normally intercepted by public/batch callers. Diagonal direction
    /// cases use three scales and zero w; affine diagonal points use the point helper,
    /// which clones w. Other diagonal vectors scale all four lanes. A remaining affine
    /// matrix with diagonal linear block also gets the point shortcut. All other
    /// cases forward the retained column/matrix/vector facts without reclassification.
    #[inline]
    fn transform_vector_with_facts(
        &self,
        rhs: &Vector4,
        vector_facts: Vector4GeometricFacts,
    ) -> Vector4 {
        // Identity matrices are handled by both public callers before this
        // retained-facts helper is entered.
        if self.facts.is_diagonal {
            trace_matrix!("method", "transform-vector4-diagonal");
            if matches!(vector_facts.homogeneous, Vector4HomogeneousKind::Direction) {
                return Vector4([
                    rhs.0[0].clone().mul_cached(&self.matrix.0[0][0]),
                    rhs.0[1].clone().mul_cached(&self.matrix.0[1][1]),
                    rhs.0[2].clone().mul_cached(&self.matrix.0[2][2]),
                    Real::zero(),
                ]);
            }
            if matches!(vector_facts.homogeneous, Vector4HomogeneousKind::Point)
                && self.facts.is_affine
            {
                // The affine diagonal point helper clones the supplied w=1 lane.
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

    /// Collects matrix facts once, then derives the retained final-column flags.
    fn new(matrix: &'a Matrix4) -> Self {
        let facts = matrix4_facts(&matrix.0);
        Self::new_with_facts(matrix, facts)
    }

    /// Borrows the matrix with an existing, caller-valid shape report.
    ///
    /// Reuses the three xyz column-zero flags, queries only m33 for the fourth flag,
    /// then computes all-zero/all-retained summaries and reuses diagonal-direction
    /// evidence. The supplied report is not checked against the matrix. All-retained
    /// does not establish known nonzero status for any column entry.
    fn new_with_facts(matrix: &'a Matrix4, facts: Matrix4Facts) -> Self {
        // Cache the per-row homogeneous-column definitely-zero facts once; this
        // keeps batch direction/path selection on the fast linear form when the
        // translation coefficient is structurally impossible to be non-zero.
        // The first three values are retained from `matrix4_facts`; only m33
        // needs a fresh zero query here. Keeping those existing structural facts
        // avoids duplicate probes in each batch lane while not adding any new
        // work to inverse/division fact scans.
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

    /// Injects the assumed w=1 classification into retained single-vector dispatch.
    ///
    /// Does not inspect input w. Public one-shot callers have already intercepted the
    /// affine diagonal-linear case; other private use still relies on a true point
    /// precondition and valid cached matrix facts.
    #[inline]
    fn transform_point_vector(&self, rhs: &Vector4) -> Vector4 {
        // The public one-shot point entry point intercepts every affine matrix
        // with a diagonal linear block (including identity) before constructing
        // this cached handle. The remaining point cases all use retained facts.
        self.transform_vector_with_facts(
            rhs,
            Vector4GeometricFacts {
                homogeneous: Vector4HomogeneousKind::Point,
            },
        )
    }

    /// Transforms a batch using uniform-kind shortcuts or a second per-vector fact pass.
    ///
    /// Identity clones the slice. Diagonal matrices classify the first w and test only
    /// the matching zero/one property on later vectors for uniform direction/affine-point
    /// shortcuts; other diagonal batches scale all four lanes. Remaining matrices scan
    /// until two homogeneous kinds are seen. This early stop establishes mixture, not
    /// a complete inventory of kinds later in the slice.
    ///
    /// Uniform direction, point and general-weight batches reuse one schedule without
    /// a vector of per-input facts. Mixed batches allocate that vector and classify
    /// every input again, so a third kind later in the slice still gets correct
    /// per-vector dispatch. All paths preserve order and allocate owned output as
    /// needed; uniform does not mean allocation-free. Empty input yields empty output.
    /// Neither output length nor fixed row arity bounds scalar-expression work.
    fn transform_vector_batch(&self, rhs: &[Vector4]) -> Vec<Vector4> {
        if self.facts.is_identity {
            trace_matrix!("method", "transform-vector4-batch-identity");
            return rhs.to_vec();
        }
        if self.facts.is_diagonal {
            let mut transformed = Vec::with_capacity(rhs.len());
            if let Some(first) = rhs.first() {
                match first.0[3].zero_one_or_minus_one() {
                    ZeroOneMinusOneStatus::Zero => {
                        if rhs
                            .iter()
                            .skip(1)
                            .all(|vector| vector.0[3].definitely_zero())
                        {
                            trace_matrix!("helper", "transform-vector4-batch-diagonal-direction");
                            // After the first vector classifies the batch candidate,
                            // the remaining direction scan only needs `w == 0`.
                            // This avoids asking whether every later direction is a
                            // point while keeping unknown first vectors on a single
                            // combined signed-unit classifier.
                            for vector in rhs {
                                transformed.push(Vector4([
                                    vector.0[0].clone().mul_cached(&self.matrix.0[0][0]),
                                    vector.0[1].clone().mul_cached(&self.matrix.0[1][1]),
                                    vector.0[2].clone().mul_cached(&self.matrix.0[2][2]),
                                    Real::zero(),
                                ]));
                            }
                            return transformed;
                        }
                    }
                    ZeroOneMinusOneStatus::One
                        if self.facts.is_affine
                            && rhs
                                .iter()
                                .skip(1)
                                .all(|vector| vector.0[3].definitely_one()) =>
                    {
                        trace_matrix!("helper", "transform-vector4-batch-diagonal-point");
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
                                Real::one(),
                            ]));
                        }
                        return transformed;
                    }
                    ZeroOneMinusOneStatus::MinusOne
                    | ZeroOneMinusOneStatus::One
                    | ZeroOneMinusOneStatus::NeitherOrUnknown => {}
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

        // Classify batch kind without a per-vector fact array. Output storage was
        // already reserved; uniform batches only avoid the later fact-vector allocation.
        let mut has_direction = false;
        let mut has_point = false;
        let mut has_unknown = false;
        for vector in rhs {
            match vector.geometric_facts().homogeneous {
                Vector4HomogeneousKind::Direction => has_direction = true,
                Vector4HomogeneousKind::Point => has_point = true,
                Vector4HomogeneousKind::Unknown => has_unknown = true,
            }

            // If any two kinds appear, a mixed batch is certain and we stop
            // early; we still classify again below only when mixed is true.
            if (has_direction && (has_point || has_unknown)) || (has_point && has_unknown) {
                break;
            }
        }

        // Common batch shapes (all directions, all points, all unknown) avoid
        // per-vector classification and fact-vector allocation.
        if has_direction && !has_point && !has_unknown {
            trace_matrix!("helper", "transform-vector4-batch-direction");
            if self.direction_is_diagonal {
                for vector in rhs {
                    transformed.push(Vector4([
                        vector.0[0].clone().mul_cached(&self.matrix.0[0][0]),
                        vector.0[1].clone().mul_cached(&self.matrix.0[1][1]),
                        vector.0[2].clone().mul_cached(&self.matrix.0[2][2]),
                        Real::zero(),
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
            trace_matrix!("helper", "transform-vector4-batch-point");
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
                trace_matrix!("helper", "transform-vector4-batch-point-all-nonzero");
                for vector in rhs {
                    transformed.push(Vector4(transform_vector4_rhs_point_all_nonzero_ref_cached(
                        &self.matrix.0,
                        &vector.0,
                    )));
                }
            } else if self.all_translation_zero {
                trace_matrix!("helper", "transform-vector4-batch-full-no-translation");
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
            // All general-weight vectors can use the full homogeneous kernel
            // without a vector of per-input classification records.
            trace_matrix!("helper", "transform-vector4-batch-unknown");
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
            trace_matrix!("method", "transform-vector4-batch-mixed");
            for (vector, facts) in rhs.iter().zip(vector_facts.iter()) {
                transformed.push(self.transform_vector_with_facts(vector, *facts));
            }
            return transformed;
        }

        if has_unknown && (has_direction || has_point) {
            // Fallback: per-vector facts handles direction/unknown or
            // point/unknown mixtures safely.
            trace_matrix!("method", "transform-vector4-batch-mixed");
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

    /// Transforms a batch whose inputs are assumed to have w=1, without classifying them.
    ///
    /// Identity clones the inputs. Affine diagonal-linear matrices use three scales
    /// plus offsets and construct fresh output ones. Otherwise chooses all-retained
    /// column, all-zero column, then per-row offset handling. All-retained includes
    /// unknown coefficients and supplies no nonzero proof. The precondition is not
    /// validated, and invalid weights can behave differently across these shortcuts.
    /// Allocates output in input order; empty input remains empty.
    fn transform_point_batch(&self, rhs: &[Vector4]) -> Vec<Vector4> {
        trace_matrix!("method", "transform-vector4-point-batch-assumed");
        if self.facts.is_identity {
            trace_matrix!("helper", "transform-vector4-point-batch-identity-assumed");
            // Identity preserves the supplied scalar representations by cloning.
            return rhs.to_vec();
        }
        let mut transformed = Vec::with_capacity(rhs.len());
        if self.facts.is_affine && self.facts.linear_is_diagonal {
            trace_matrix!(
                "helper",
                "transform-vector4-point-batch-affine-linear-diagonal-assumed"
            );
            // Apply the caller-established point fact uniformly across this batch.
            for vector in rhs {
                transformed.push(Vector4([
                    vector.0[0].clone().mul_cached(&self.matrix.0[0][0]) + &self.matrix.0[0][3],
                    vector.0[1].clone().mul_cached(&self.matrix.0[1][1]) + &self.matrix.0[1][3],
                    vector.0[2].clone().mul_cached(&self.matrix.0[2][2]) + &self.matrix.0[2][3],
                    // This batch kernel constructs a fresh canonical point lane.
                    Real::one(),
                ]));
            }
            return transformed;
        }

        if self.all_translation_nonzero {
            trace_matrix!(
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
            trace_matrix!(
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

        trace_matrix!(
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

/// Multiplies an owned scalar by a borrowed reusable factor through `mul_cached`.
///
/// Used for shared determinant reciprocals, but the factor can be any Real,
/// including zero. This helper neither inverts nor checks it and owns no cache;
/// scalar arithmetic controls expression sharing and simplification.
fn scale_by_shared_factor(value: Real, factor: &Real) -> Real {
    // Borrow the reusable factor; the scalar layer owns any cache/structure reuse.
    value.mul_cached(factor)
}

/// Scales nine consumed matrix entries row-major by the same borrowed factor.
///
/// Unrolls the final scale pass used after adjugate/product construction. Calls
/// [`scale_by_shared_factor`] nine times without cloning the factor here. Does
/// not prove that the factor is a determinant reciprocal, require nonzero entries,
/// or perform fraction-free elimination.
fn scale_matrix3(matrix: [[Real; 3]; 3], factor: &Real) -> [[Real; 3]; 3] {
    // Apply the borrowed common factor in one unrolled row-major pass.
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

/// Scales sixteen consumed matrix entries row-major by one borrowed factor.
///
/// Used to apply a shared determinant reciprocal to an unscaled right-division
/// product. Performs sixteen shared-factor calls without an additional matrix
/// classification, reciprocal or zero guard. This is elementwise scaling, not
/// an invertibility proof or a scalar-work bound.
fn scale_matrix4(matrix: [[Real; 4]; 4], factor: &Real) -> [[Real; 4]; 4] {
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

/// Computes the first product minus the second, pruning definitely-zero factors.
///
/// Tests the first pair then the second, left factor before right within each
/// short-circuiting pair. Both pair statuses precede arithmetic. Unknown factors
/// remain active. Zero/one retained products give zero, a negated second product,
/// or the first product; two use an ordered active sum with signs `[+, -]`.
/// No sign refinement or independent nonzero certificate is established.
#[inline]
fn mul_sub(left_a: &Real, right_a: &Real, left_b: &Real, right_b: &Real) -> Real {
    // Test pairwise zero evidence before constructing products; retain unknowns.
    let first_zero = left_a.definitely_zero() || right_a.definitely_zero();
    let second_zero = left_b.definitely_zero() || right_b.definitely_zero();

    if first_zero || second_zero {
        trace_matrix!("helper", "mul-sub-pruned");
        if first_zero && second_zero {
            return Real::zero();
        }
        if first_zero {
            return -(left_b * right_b);
        }
        return left_a * right_a;
    }
    Real::active_signed_product_sum2([true, false], [[left_a, right_a], [left_b, right_b]])
}

/// Computes the ordered two-product difference without local zero pruning.
///
/// Uses active signs `[+, -]`. Dense/exact names do not require rational storage
/// or prove nonzero factors; the scalar reducer selects exact-rational aggregation
/// or its symbolic fallback. No factor-class precondition is bypassed here.
#[inline]
fn mul_sub_dense_exact(left_a: &Real, right_a: &Real, left_b: &Real, right_b: &Real) -> Real {
    trace_matrix!("helper", "mul-sub-dense-exact");
    Real::active_signed_product_sum2([true, false], [[left_a, right_a], [left_b, right_b]])
}

/// Computes a two-product difference assuming all four factors are exact rationals.
///
/// Passes signs `[+, -]` to the unchecked known-rational reducer. Zeros are valid,
/// but every current factor representation must satisfy the caller's proof. No
/// class check or typed mismatch is returned; stale evidence can change the value.
#[inline]
fn mul_sub_dense_exact_known_rational(
    left_a: &Real,
    right_a: &Real,
    left_b: &Real,
    right_b: &Real,
) -> Real {
    trace_matrix!("helper", "mul-sub-dense-exact-known-rational");
    Real::active_signed_product_sum2_known_exact_rational(
        [true, false],
        [[left_a, right_a], [left_b, right_b]],
    )
}

/// Computes a two-product difference under an exact-dyadic representation precondition.
///
/// All four factors must currently expose rational values with power-of-two
/// denominators, including zero/integer cases. Uses signs `[+, -]` without local
/// zero or representation checks. The caller must preserve that proof; the helper
/// does not turn a violated precondition into a typed error.
#[inline]
fn mul_sub_dense_exact_known_dyadic(
    left_a: &Real,
    right_a: &Real,
    left_b: &Real,
    right_b: &Real,
) -> Real {
    trace_matrix!("helper", "mul-sub-dense-exact-known-dyadic");
    Real::active_signed_product_sum2_known_dyadic(
        [true, false],
        [[left_a, right_a], [left_b, right_b]],
    )
}

/// Adds two products after ordered definite-zero pruning.
///
/// Tests the first factor pair, then the second, short-circuiting each pair left
/// before right. Unknown factors are retained. Zero/one retained products return
/// zero or the surviving product; two use active signs `[+, +]`. Scalar reducers
/// own rational denominator aggregation and symbolic construction.
fn mul_add(left_a: &Real, right_a: &Real, left_b: &Real, right_b: &Real) -> Real {
    // Share the ordered zero-gate schedule with mul_sub, using positive signs.
    let first_zero = left_a.definitely_zero() || right_a.definitely_zero();
    let second_zero = left_b.definitely_zero() || right_b.definitely_zero();

    if first_zero || second_zero {
        trace_matrix!("helper", "mul-add-pruned");
        if first_zero && second_zero {
            return Real::zero();
        }
        if first_zero {
            return left_b * right_b;
        }
        return left_a * right_a;
    }
    Real::active_signed_product_sum2([true, true], [[left_a, right_a], [left_b, right_b]])
}

/// Computes `a_left*a_right + b_left*b_right - c_left*c_right` with zero pruning.
///
/// Tests all three pairs in a,b,c order, left before right within each pair.
/// Retains unknowns. Zero active pairs yield zero; one preserves its sign; two
/// use the corresponding ordered signed subset; three use `[+, +, -]`.
/// The pruned match's unreachable arm follows from counting three booleans and
/// entering only at count at most two, not from a scalar sign proof.
#[inline]
fn mul_add_sub(
    left_a: &Real,
    right_a: &Real,
    left_b: &Real,
    right_b: &Real,
    left_c: &Real,
    right_c: &Real,
) -> Real {
    // Prune certified zero products, preserving the signs of every retained pair.
    let first_zero = left_a.definitely_zero() || right_a.definitely_zero();
    let second_zero = left_b.definitely_zero() || right_b.definitely_zero();
    let third_zero = left_c.definitely_zero() || right_c.definitely_zero();
    let nonzero_count = (!first_zero) as u8 + (!second_zero) as u8 + (!third_zero) as u8;

    if nonzero_count <= 2 {
        trace_matrix!("helper", "mul-add-sub-pruned");
        return match nonzero_count {
            0 => Real::zero(),
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
                    Real::active_signed_product_sum2(
                        [true, false],
                        [[left_b, right_b], [left_c, right_c]],
                    )
                } else if second_zero {
                    Real::active_signed_product_sum2(
                        [true, false],
                        [[left_a, right_a], [left_c, right_c]],
                    )
                } else {
                    Real::active_signed_product_sum2(
                        [true, true],
                        [[left_a, right_a], [left_b, right_b]],
                    )
                }
            }
            _ => unreachable!(),
        };
    }
    Real::active_signed_product_sum2(
        [true, true, false],
        [[left_a, right_a], [left_b, right_b], [left_c, right_c]],
    )
}

/// Computes three ordered products with signs `[+, +, -]` and no local zero scan.
///
/// The active reducer retains all supplied terms and owns rational/symbolic
/// selection. The name supplies neither rationality nor nonzero certificates.
#[inline]
fn mul_add_sub_dense_exact(
    left_a: &Real,
    right_a: &Real,
    left_b: &Real,
    right_b: &Real,
    left_c: &Real,
    right_c: &Real,
) -> Real {
    trace_matrix!("helper", "mul-add-sub-dense-exact");
    Real::active_signed_product_sum2(
        [true, true, false],
        [[left_a, right_a], [left_b, right_b], [left_c, right_c]],
    )
}

/// Computes signs `[+, +, -]` after the caller proves all six factors exact rational.
///
/// Delegates directly to the known-rational reducer, accepting zeros and omitting
/// class/zero checks. Representation evidence must still describe these factors;
/// this helper supplies no typed response to stale or invalid evidence.
#[inline]
fn mul_add_sub_dense_exact_known_rational(
    left_a: &Real,
    right_a: &Real,
    left_b: &Real,
    right_b: &Real,
    left_c: &Real,
    right_c: &Real,
) -> Real {
    trace_matrix!("helper", "mul-add-sub-dense-exact-known-rational");
    Real::active_signed_product_sum2_known_exact_rational(
        [true, true, false],
        [[left_a, right_a], [left_b, right_b], [left_c, right_c]],
    )
}

/// Computes the first product minus both remaining products, despite the name.
///
/// The actual polynomial is `a_left*a_right - b_left*b_right - c_left*c_right`.
/// Tests pairs in a,b,c order, left before right, and retains unknown factors.
/// Pruned subsets preserve signs `[+, -, -]` and lane order, including a negative
/// third-only product and a two-negative-product sum when a vanishes. The zero
/// count bounds the pruned match, making its unreachable arm an integer invariant.
fn mul_sub_add(
    left_a: &Real,
    right_a: &Real,
    left_b: &Real,
    right_b: &Real,
    left_c: &Real,
    right_c: &Real,
) -> Real {
    // Keep the sign pattern separate from the zero-pruning decision. This
    // lets structural facts remove zero lanes before the exact Real kernel sees
    // the signed product sum, reducing unnecessary symbolic nodes while
    // preserving the same determinant/cofactor polynomial.
    let first_zero = left_a.definitely_zero() || right_a.definitely_zero();
    let second_zero = left_b.definitely_zero() || right_b.definitely_zero();
    let third_zero = left_c.definitely_zero() || right_c.definitely_zero();
    let nonzero_count = (!first_zero) as u8 + (!second_zero) as u8 + (!third_zero) as u8;

    if nonzero_count <= 2 {
        trace_matrix!("helper", "mul-sub-add-pruned");
        return match nonzero_count {
            0 => Real::zero(),
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
                    Real::active_signed_product_sum2(
                        [false, false],
                        [[left_b, right_b], [left_c, right_c]],
                    )
                } else if second_zero {
                    Real::active_signed_product_sum2(
                        [true, false],
                        [[left_a, right_a], [left_c, right_c]],
                    )
                } else {
                    Real::active_signed_product_sum2(
                        [true, false],
                        [[left_a, right_a], [left_b, right_b]],
                    )
                }
            }
            _ => unreachable!(),
        };
    }
    Real::active_signed_product_sum2(
        [true, false, false],
        [[left_a, right_a], [left_b, right_b], [left_c, right_c]],
    )
}

/// Computes signs `[+, -, -]` without local zero pruning; the third product is subtracted.
///
/// Matches [`mul_sub_add`]'s polynomial, retaining all terms for the active scalar
/// reducer. Dense/exact does not independently establish rationality or nonzero values.
#[inline]
fn mul_sub_add_dense_exact(
    left_a: &Real,
    right_a: &Real,
    left_b: &Real,
    right_b: &Real,
    left_c: &Real,
    right_c: &Real,
) -> Real {
    trace_matrix!("helper", "mul-sub-add-dense-exact");
    Real::active_signed_product_sum2(
        [true, false, false],
        [[left_a, right_a], [left_b, right_b], [left_c, right_c]],
    )
}

/// Computes signs `[+, -, -]` assuming all six current factors are exact rationals.
///
/// The third product is subtracted, matching [`mul_sub_add`] rather than a literal
/// reading of its name. No representation or zero checks are added. Zeros are
/// admissible; invalid caller evidence need not produce a typed mismatch.
#[inline]
fn mul_sub_add_dense_exact_known_rational(
    left_a: &Real,
    right_a: &Real,
    left_b: &Real,
    right_b: &Real,
    left_c: &Real,
    right_c: &Real,
) -> Real {
    trace_matrix!("helper", "mul-sub-add-dense-exact-known-rational");
    Real::active_signed_product_sum2_known_exact_rational(
        [true, false, false],
        [[left_a, right_a], [left_b, right_b], [left_c, right_c]],
    )
}

/// Evaluates a 3×3 determinant by first-row cofactor expansion without division.
///
/// Forms the three signed minors from rows one and two in column order, then
/// uses `dot3` with row zero. The minor helpers prune definite-zero factors;
/// unknown and symbolic factors remain valid. Returns a Real directly, including
/// zero for singular rational cases, without pivot selection or a nonzero guard.
/// Division-free arithmetic does not imply bounded scalar expression work.
#[inline]
fn determinant3(m: &[[Real; 3]; 3]) -> Real {
    trace_matrix!("helper", "determinant3");
    // First-row expansion is defined without selecting or dividing by pivots.
    let c00 = mul_sub(&m[1][1], &m[2][2], &m[1][2], &m[2][1]);
    let c10 = mul_sub(&m[1][2], &m[2][0], &m[1][0], &m[2][2]);
    let c20 = mul_sub(&m[1][0], &m[2][1], &m[1][1], &m[2][0]);
    Real::dot3([&m[0][0], &m[0][1], &m[0][2]], [&c00, &c10, &c20])
}

/// Constructs the classical adjugate and determinant, sharing three expansion minors.
///
/// Builds all nine transposed cofactors in adjugate row-major order using `mul_sub`.
/// Then dots the source's first row with the adjugate's first column to obtain the
/// determinant. No reciprocal, invertibility check or density assumption is used;
/// singular matrices still have a well-defined adjugate and determinant.
#[inline]
fn matrix3_adjugate_and_determinant(matrix: &[[Real; 3]; 3]) -> ([[Real; 3]; 3], Real) {
    trace_matrix!("helper", "matrix3-adjugate-and-determinant");
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
    let det = Real::dot3([&m[0][0], &m[0][1], &m[0][2]], [&c00, &c10, &c20]);
    ([[c00, c01, c02], [c10, c11, c12], [c20, c21, c22]], det)
}

/// Constructs nine unpruned adjugate entries and their shared determinant expression.
///
/// With `known_dyadic=true`, every source entry must currently be exact dyadic;
/// minor differences and the final three-product determinant bypass representation
/// checks. With false, ordinary active reducers admit rational or symbolic inputs.
/// The boolean is not validated and does not prove nonzero determinant or density.
///
/// Entries are constructed adjugate row-major, then the source first row is paired
/// with the adjugate first column. The returned pair remains defined for singular
/// inputs; no reciprocal is taken here.
#[inline(never)]
fn matrix3_adjugate_and_determinant_dense_exact(
    matrix: &[[Real; 3]; 3],
    known_dyadic: bool,
) -> ([[Real; 3]; 3], Real) {
    trace_matrix!("helper", "matrix3-adjugate-and-determinant-dense-exact");
    let m = &matrix;
    let difference: fn(&Real, &Real, &Real, &Real) -> Real = if known_dyadic {
        mul_sub_dense_exact_known_dyadic
    } else {
        mul_sub_dense_exact
    };
    let c00 = difference(&m[1][1], &m[2][2], &m[1][2], &m[2][1]);
    let c01 = difference(&m[0][2], &m[2][1], &m[0][1], &m[2][2]);
    let c02 = difference(&m[0][1], &m[1][2], &m[0][2], &m[1][1]);
    let c10 = difference(&m[1][2], &m[2][0], &m[1][0], &m[2][2]);
    let c11 = difference(&m[0][0], &m[2][2], &m[0][2], &m[2][0]);
    let c12 = difference(&m[0][2], &m[1][0], &m[0][0], &m[1][2]);
    let c20 = difference(&m[1][0], &m[2][1], &m[1][1], &m[2][0]);
    let c21 = difference(&m[0][1], &m[2][0], &m[0][0], &m[2][1]);
    let c22 = difference(&m[0][0], &m[1][1], &m[0][1], &m[1][0]);
    let determinant_terms = [[&m[0][0], &c00], [&m[0][1], &c10], [&m[0][2], &c20]];
    let det = if known_dyadic {
        Real::active_signed_product_sum2_known_dyadic([true; 3], determinant_terms)
    } else {
        Real::active_linear_combination3([&m[0][0], &m[0][1], &m[0][2]], [&c00, &c10, &c20])
    };
    ([[c00, c01, c02], [c10, c11, c12], [c20, c21, c22]], det)
}

/// Forms a general 3×3 inverse through nine pruned cofactors and one determinant reciprocal.
///
/// Builds the adjugate entries row-major, obtains the determinant from the source
/// first row and adjugate first column, then takes the ordinary scalar inverse.
/// Only afterward scales the nine entries into the result. A reciprocal error
/// propagates without a completed inverse; unknown determinant status is not
/// rejected by a checked guard here. No pivoting or universal singularity decision
/// is performed.
#[inline]
fn matrix3_scaled_adjugate(matrix: &[[Real; 3]; 3]) -> BlasResult<[[Real; 3]; 3]> {
    trace_matrix!("helper", "matrix3-scaled-adjugate");
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
    let det = Real::dot3([&m[0][0], &m[0][1], &m[0][2]], [&c00, &c10, &c20]);
    let inv_det = det.inverse()?;
    // Scale individual cofactors while constructing the output, sharing inv_det.
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

/// Inverts through an aggregate rational path or an unpruned cofactor schedule.
///
/// Scans representations row-major, stopping at the first missing rational view.
/// Encountering a non-dyadic rational does not stop the scan: only an entirely
/// rational matrix with at least one non-dyadic entry selects the scalar aggregate.
/// Otherwise a fully dyadic scan authorizes unchecked dyadic minors/determinant;
/// a missing rational view selects ordinary active reducers for all cofactors.
///
/// The cofactor routes form nine adjugate entries and the determinant, take one
/// ordinary reciprocal, then scale the result row-major. Scalar failures propagate.
/// This helper establishes its own representation choice; its dense name does not
/// require every entry nonzero. The ordinary inverse dispatcher already tries a
/// rational aggregate before reaching this fallback.
#[inline(never)]
fn matrix3_scaled_adjugate_dense_exact(matrix: &[[Real; 3]; 3]) -> BlasResult<[[Real; 3]; 3]> {
    trace_matrix!("helper", "matrix3-scaled-adjugate-dense-exact");
    let exact_rational_is_dyadic = matrix.iter().flatten().try_fold(true, |all_dyadic, value| {
        Some(all_dyadic & value.exact_rational_ref()?.is_dyadic())
    });
    // Only a completed rational scan with a non-dyadic entry selects this aggregate.
    if matches!(exact_rational_is_dyadic, Some(false)) {
        trace_matrix!(
            "helper",
            "matrix3-scaled-adjugate-dense-exact-rational-aggregate"
        );
        return Real::exact_rational_matrix3_inverse_known_exact([
            [&matrix[0][0], &matrix[0][1], &matrix[0][2]],
            [&matrix[1][0], &matrix[1][1], &matrix[1][2]],
            [&matrix[2][0], &matrix[2][1], &matrix[2][2]],
        ]);
    }
    let known_dyadic = matches!(exact_rational_is_dyadic, Some(true));
    let m = &matrix;
    let difference: fn(&Real, &Real, &Real, &Real) -> Real = if known_dyadic {
        mul_sub_dense_exact_known_dyadic
    } else {
        mul_sub_dense_exact
    };
    let c00 = difference(&m[1][1], &m[2][2], &m[1][2], &m[2][1]);
    let c01 = difference(&m[0][2], &m[2][1], &m[0][1], &m[2][2]);
    let c02 = difference(&m[0][1], &m[1][2], &m[0][2], &m[1][1]);
    let c10 = difference(&m[1][2], &m[2][0], &m[1][0], &m[2][2]);
    let c11 = difference(&m[0][0], &m[2][2], &m[0][2], &m[2][0]);
    let c12 = difference(&m[0][2], &m[1][0], &m[0][0], &m[1][2]);
    let c20 = difference(&m[1][0], &m[2][1], &m[1][1], &m[2][0]);
    let c21 = difference(&m[0][1], &m[2][0], &m[0][0], &m[2][1]);
    let c22 = difference(&m[0][0], &m[1][1], &m[0][1], &m[1][0]);
    let determinant_terms = [[&m[0][0], &c00], [&m[0][1], &c10], [&m[0][2], &c20]];
    let det = if known_dyadic {
        Real::active_signed_product_sum2_known_dyadic([true; 3], determinant_terms)
    } else {
        Real::active_linear_combination3([&m[0][0], &m[0][1], &m[0][2]], [&c00, &c10, &c20])
    };
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

/// Dispatches ordinary 3×3 inversion from structural and representation evidence.
///
/// First tests the three nonzero witnesses excluding triangular/affine shortcuts;
/// they do not prove full density or invertibility. If admitted, tries the rational
/// aggregate (whose error is final) and otherwise the dense cofactor helper.
/// Remaining cases collect matrix facts and try identity, diagonal, upper triangular,
/// lower triangular, then affine, in order. Upper handling excludes affine matrices
/// with diagonal linear blocks so their affine specialization remains reachable.
///
/// The fallback is a scaled adjugate, not Gauss–Jordan. Ordinary scalar inverse
/// errors propagate; unknown determinant status can still construct a symbolic
/// result. No global nonzero proof, signal polling or total-work limit is added.
#[inline]
fn invert_matrix3(matrix: [[Real; 3]; 3]) -> BlasResult<[[Real; 3]; 3]> {
    trace_matrix!("helper", "invert-matrix3");
    if matrix3_is_definitely_dense_for_inverse(&matrix) {
        trace_matrix!("helper", "invert-matrix3-dense-cofactor");
        if let Some(inverse) = matrix3_dense_exact_rational_inverse(&matrix) {
            return inverse;
        }
        return matrix3_scaled_adjugate_dense_exact(&matrix);
    }
    let facts = matrix3_facts(&matrix);
    if facts.is_identity {
        trace_matrix!("helper", "invert-matrix3-identity");
        return Ok(matrix);
    }
    if facts.is_diagonal {
        trace_matrix!("helper", "invert-matrix3-diagonal");
        return invert_matrix3_by_diagonal(&matrix);
    }
    if facts.is_upper_triangular && !(facts.is_affine && facts.linear_is_diagonal) {
        // Retain the triangular-before-affine ordering for this admitted shape.
        trace_matrix!("helper", "invert-matrix3-upper-triangular");
        return invert_matrix3_upper_triangular(&matrix);
    }
    if facts.is_lower_triangular {
        trace_matrix!("helper", "invert-matrix3-lower-triangular");
        return invert_matrix3_lower_triangular(&matrix);
    }
    if facts.is_affine {
        trace_matrix!("helper", "invert-matrix3-affine");
        return invert_matrix3_affine(&matrix, facts.linear_is_diagonal);
    }
    // The general ordinary inverse uses a determinant reciprocal, not a pivot solve.
    matrix3_scaled_adjugate(&matrix)
}

/// Inverts with checked reciprocal prerequisites while preserving structural priority.
///
/// The early three-entry witness admits a rational aggregate when available;
/// its singular error returns directly. Otherwise an all-dyadic scan chooses the
/// dense adjugate schedule and the determinant must be known nonzero before inversion.
/// Identity, diagonal, upper/lower triangular and affine cases follow ordinary
/// priority, with the appropriate checked helpers. The remaining general adjugate
/// also guards its determinant before inversion and scaling.
///
/// Zero/unknown guard errors propagate without a partial result. Checked knowledge
/// is local to the chosen schedule, not a universal decision for symbolic singularity.
#[inline]
fn invert_matrix3_checked(matrix: [[Real; 3]; 3]) -> CheckedBlasResult<[[Real; 3]; 3]> {
    trace_matrix!("helper", "invert-matrix3-checked");
    if matrix3_is_definitely_dense_for_inverse(&matrix) {
        trace_matrix!("helper", "invert-matrix3-checked-dense-cofactor");
        if let Some(inverse) = matrix3_dense_exact_rational_inverse(&matrix) {
            return inverse;
        }
        let known_dyadic = matrix.iter().flatten().all(Real::is_exact_dyadic_rational);
        let (adjugate, det) = matrix3_adjugate_and_determinant_dense_exact(&matrix, known_dyadic);
        require_known_nonzero(&det)?;
        let inv_det = det.inverse()?;
        return Ok(scale_matrix3(adjugate, &inv_det));
    }
    let facts = matrix3_facts(&matrix);
    if facts.is_identity {
        trace_matrix!("helper", "invert-matrix3-checked-identity");
        return Ok(matrix);
    }
    if facts.is_diagonal {
        trace_matrix!("helper", "invert-matrix3-checked-diagonal");
        return invert_matrix3_by_diagonal_checked(&matrix);
    }
    if facts.is_upper_triangular && !(facts.is_affine && facts.linear_is_diagonal) {
        // Checked fast path preserves the same dispatch preference as ordinary
        // inverse but with an explicit nonzero guarantee on diagonal pivots.
        trace_matrix!("helper", "invert-matrix3-checked-upper-triangular");
        return invert_matrix3_upper_triangular_checked(&matrix);
    }
    if facts.is_lower_triangular {
        trace_matrix!("helper", "invert-matrix3-checked-lower-triangular");
        return invert_matrix3_lower_triangular_checked(&matrix);
    }
    if facts.is_affine {
        trace_matrix!("helper", "invert-matrix3-checked-affine");
        return invert_matrix3_affine_checked(&matrix, facts.linear_is_diagonal);
    }
    let (adjugate, det) = matrix3_adjugate_and_determinant(&matrix);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(adjugate, &inv_det))
}

/// Routes checked inversion with determinant/pivot signal behavior specific to each branch.
///
/// The early three-entry witness goes directly to a dense adjugate after testing
/// all-dyadic representation; unlike the other two dispatchers, it does not call
/// the rational aggregate. Attaches the signal to the constructed determinant before
/// its known-nonzero guard, inverse and final scaling. The general adjugate fallback
/// uses the same attachment order.
///
/// Identity, diagonal and triangular shortcuts do not consume the signal. Affine
/// handling forwards it to that helper's determinant/diagonal schedule. No local
/// polling or preemptive work bound is added; construction before attachment can
/// already perform scalar work. A supplied flag need not reject every shortcut.
#[inline]
fn invert_matrix3_checked_with_abort(
    matrix: [[Real; 3]; 3],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Real; 3]; 3]> {
    trace_matrix!("helper", "invert-matrix3-checked-with-abort");
    if matrix3_is_definitely_dense_for_inverse(&matrix) {
        trace_matrix!("helper", "invert-matrix3-checked-with-abort-dense-cofactor");
        let known_dyadic = matrix.iter().flatten().all(Real::is_exact_dyadic_rational);
        let (adjugate, det) = matrix3_adjugate_and_determinant_dense_exact(&matrix, known_dyadic);
        let det = with_abort(det, signal);
        require_known_nonzero(&det)?;
        let inv_det = det.inverse()?;
        return Ok(scale_matrix3(adjugate, &inv_det));
    }
    let facts = matrix3_facts(&matrix);
    if facts.is_identity {
        trace_matrix!("helper", "invert-matrix3-checked-with-abort-identity");
        return Ok(matrix);
    }
    if facts.is_diagonal {
        trace_matrix!("helper", "invert-matrix3-checked-with-abort-diagonal");
        return invert_matrix3_by_diagonal_checked(&matrix);
    }
    if facts.is_upper_triangular && !(facts.is_affine && facts.linear_is_diagonal) {
        trace_matrix!(
            "helper",
            "invert-matrix3-checked-with-abort-upper-triangular"
        );
        return invert_matrix3_upper_triangular_checked(&matrix);
    }
    if facts.is_lower_triangular {
        trace_matrix!(
            "helper",
            "invert-matrix3-checked-with-abort-lower-triangular"
        );
        return invert_matrix3_lower_triangular_checked(&matrix);
    }
    if facts.is_affine {
        trace_matrix!("helper", "invert-matrix3-checked-with-abort-affine");
        return invert_matrix3_affine_checked_with_abort(&matrix, signal, facts.linear_is_diagonal);
    }
    let (adjugate, det) = matrix3_adjugate_and_determinant(&matrix);
    let det = with_abort(det, signal);
    require_known_nonzero(&det)?;
    let inv_det = det.inverse()?;
    Ok(scale_matrix3(adjugate, &inv_det))
}

/// Constructs the twelve 2×2 minors shared by the 4×4 determinant and adjugate.
///
/// Returns `(s, c)` for row pairs (0,1) and (2,3), respectively. Each array uses
/// column pairs (0,1), (0,2), (0,3), (1,2), (1,3), (2,3), in that order, with
/// minor `m[r][i] * m[r+1][j] - m[r+1][i] * m[r][j]`. Builds all of `s` before
/// `c`, using [`mul_sub`] to prune definitely-zero products while retaining
/// unknown factors. No division, rational representation or nonsingularity is
/// required. These arrays are values, not certificates bound to the source;
/// callers must keep them consistent with any matrix used for an adjugate.
#[inline]
fn matrix4_factors(m: &[[Real; 4]; 4]) -> ([Real; 6], [Real; 6]) {
    trace_matrix!("helper", "matrix4-factors");
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

/// Builds the same ordered minors as [`matrix4_factors`] without local zero pruning.
///
/// Each two-product difference uses the active scalar reducer, which selects
/// rational aggregation when available and otherwise uses ordinary Real
/// arithmetic. Despite the name, symbolic entries and zeros are admissible;
/// neither entry density nor invertibility is assumed.
#[inline(never)]
fn matrix4_factors_dense_exact(m: &[[Real; 4]; 4]) -> ([Real; 6], [Real; 6]) {
    trace_matrix!("helper", "matrix4-factors-dense-exact");
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

/// Builds the ordered 4×4 minors with unchecked known-rational scalar reducers.
///
/// Uses the layout of [`matrix4_factors`]. Every entry must currently expose an
/// exact-rational representation; zeros and singular matrices remain valid.
/// The caller supplies this evidence: the helper neither rechecks it nor
/// returns a typed mismatch. Invalid or stale evidence can cause scalar
/// reducers to use only stored rational components and change the value.
#[inline(never)]
fn matrix4_factors_dense_exact_known_rational(m: &[[Real; 4]; 4]) -> ([Real; 6], [Real; 6]) {
    trace_matrix!("helper", "matrix4-factors-dense-exact-known-rational");
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

/// Evaluates `s0*c5 - s1*c4 + s2*c3 + s3*c2 - s4*c1 + s5*c0`.
///
/// This is the Laplace expansion along the first two rows when `s` and `c`
/// have the layout of [`matrix4_factors`] for the same matrix. Otherwise it
/// still evaluates that polynomial; no source consistency is checked. Uses
/// one signed-product-sum adapter, whose definite-zero pruning retains unknown
/// factors and whose active reducer can aggregate rational terms. No reciprocal
/// or nonzero test on the determinant is needed.
fn determinant4_from_factors(s: &[Real; 6], c: &[Real; 6]) -> Real {
    trace_matrix!("helper", "determinant4-from-factors");
    // This is the fixed six-minor determinant polynomial
    //   s0*c5 - s1*c4 + s2*c3 + s3*c2 - s4*c1 + s5*c0.
    // Route it as one signed product sum so hyperreal exact rationals can
    // share a final denominator instead of reducing a dot product plus two
    // extra products and a subtraction.
    Real::signed_product_sum2(
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

/// Evaluates the six-minor determinant polynomial with a known-rational aggregate.
///
/// Uses the signs and pairing of [`determinant4_from_factors`] without local
/// pruning or representation checks. All twelve supplied values must currently
/// be exact rationals, including any zeros; a determinant interpretation also
/// requires consistent source minors. Incorrect representation evidence is not
/// guaranteed to produce a typed error.
fn determinant4_from_factors_known_rational(s: &[Real; 6], c: &[Real; 6]) -> Real {
    trace_matrix!("helper", "determinant4-from-factors-known-rational");
    Real::active_signed_product_sum2_known_exact_rational(
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

/// Computes a 4×4 determinant from twelve 2×2 minors without division or pivots.
///
/// First scans for exposed nonzero rational entries throughout the matrix.
/// Only that fully dense rational case uses the known-rational minor and
/// determinant reducers. Any zero or missing rational view selects the
/// ordinary pruning helpers. Singular matrices remain valid determinant
/// inputs; this function neither proves invertibility nor refines a sign.
#[inline]
fn determinant4(m: &[[Real; 4]; 4]) -> Real {
    trace_matrix!("helper", "determinant4");
    if matrix4_is_dense_exact_rational(m) {
        let (s, c) = matrix4_factors_dense_exact_known_rational(m);
        determinant4_from_factors_known_rational(&s, &c)
    } else {
        let (s, c) = matrix4_factors(m);
        determinant4_from_factors(&s, &c)
    }
}

/// Constructs the transposed cofactors of `m`, scaling each by `inv_det`.
///
/// Requires `s` and `c` to be the current minors of `m` in [`matrix4_factors`]
/// order. Visits the sixteen output entries row-major, expanding each cofactor
/// into three products and immediately applying [`scale_by_shared_factor`].
/// Cofactor helpers prune definitely-zero products and retain unknown factors.
/// Any scale is accepted, including zero; this is an inverse only when the
/// caller has supplied `1/det(m)` for a nonzero determinant. No reciprocal,
/// determinant guard or source-consistency check occurs here.
#[inline]
fn matrix4_scaled_adjugate_from_factors(
    m: &[[Real; 4]; 4],
    s: &[Real; 6],
    c: &[Real; 6],
    inv_det: &Real,
) -> [[Real; 4]; 4] {
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

/// Scales the 4×4 adjugate using active three-product reducers without local pruning.
///
/// Has the source-minor and scale contract of
/// [`matrix4_scaled_adjugate_from_factors`], including sixteen row-major scaling
/// calls. Active reducers still select their scalar arithmetic from the inputs;
/// symbolic values and zeros are supported without a rationality or full-density
/// precondition. This helper does not construct or validate the reciprocal.
#[inline(never)]
fn matrix4_scaled_adjugate_from_factors_dense_exact(
    m: &[[Real; 4]; 4],
    s: &[Real; 6],
    c: &[Real; 6],
    inv_det: &Real,
) -> [[Real; 4]; 4] {
    trace_matrix!("helper", "matrix4-scaled-adjugate-dense-exact");
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

/// Attempts a 3×3 inverse entirely through the scalar rational aggregate.
///
/// Classifies current representations row-major, stopping at the first missing
/// rational view. That case returns `None`, allowing a symbolic fallback; it
/// does not prove irrationality. Otherwise selects the dyadic aggregate only
/// if every denominator is a power of two, and the general rational aggregate
/// for other rational matrices. Returns `Some(result)`, including
/// `Some(Err(Problem::DivideByZero))` for a singular matrix, which callers must
/// not mistake for a request to fall back. Zero entries are valid: full density
/// is not required. No cancellation flag is accepted or attached here.
#[inline]
fn matrix3_dense_exact_rational_inverse(
    matrix: &[[Real; 3]; 3],
) -> Option<BlasResult<[[Real; 3]; 3]>> {
    let exact_kind = matrix3_exact_rational_kind(matrix);
    (exact_kind != ExactRationalKind::NonRational).then(|| {
        trace_matrix!("helper", "invert-matrix3-dense-exact-rational-aggregate");
        let values = [
            [&matrix[0][0], &matrix[0][1], &matrix[0][2]],
            [&matrix[1][0], &matrix[1][1], &matrix[1][2]],
            [&matrix[2][0], &matrix[2][1], &matrix[2][2]],
        ];
        if exact_kind == ExactRationalKind::ExactDyadicRational {
            Real::exact_rational_matrix3_inverse_known_dyadic(values)
        } else {
            Real::exact_rational_matrix3_inverse_known_exact(values)
        }
    })
}

/// Attempts a 4×4 inverse through the scalar rational or dyadic aggregate.
///
/// Applies the representation and `Option<Result<...>>` contract of
/// [`matrix3_dense_exact_rational_inverse`] to sixteen entries. Classification
/// checks current storage before calling unchecked scalar entry points;
/// symbolic storage returns `None`, while rational singularity returns
/// `Some(Err(Problem::DivideByZero))`. Zeros are admissible, and no signal is
/// attached even when a signal-aware inverse dispatcher calls this helper.
#[inline]
fn matrix4_dense_exact_rational_inverse(
    matrix: &[[Real; 4]; 4],
) -> Option<BlasResult<[[Real; 4]; 4]>> {
    let exact_kind = matrix4_exact_rational_kind(matrix);
    (exact_kind != ExactRationalKind::NonRational).then(|| {
        trace_matrix!("helper", "invert-matrix4-dense-exact-rational-aggregate");
        let values = [
            [&matrix[0][0], &matrix[0][1], &matrix[0][2], &matrix[0][3]],
            [&matrix[1][0], &matrix[1][1], &matrix[1][2], &matrix[1][3]],
            [&matrix[2][0], &matrix[2][1], &matrix[2][2], &matrix[2][3]],
            [&matrix[3][0], &matrix[3][1], &matrix[3][2], &matrix[3][3]],
        ];
        if exact_kind == ExactRationalKind::ExactDyadicRational {
            Real::exact_rational_matrix4_inverse_known_dyadic(values)
        } else {
            Real::exact_rational_matrix4_inverse_known_exact(values)
        }
    })
}

/// Inverts an owned 4×4 matrix using structure before a shared cofactor reciprocal.
///
/// First tries the three nonzero witnesses excluding shape shortcuts, not a
/// full-density or nonsingularity certificate. That route tries the rational
/// aggregate, returning its success or error directly; symbolic inputs use
/// unpruned active minors and scaled cofactors. Otherwise collects shape facts
/// and tries identity, diagonal, upper triangular, lower triangular, then
/// affine, in order. Upper handling excludes affine diagonal-linear matrices.
/// The remaining general path uses zero-pruned minors and cofactors.
///
/// Both cofactor paths construct one ordinary determinant reciprocal before
/// assembling the scaled adjugate. Scalar errors propagate; unknown determinant
/// status alone is not rejected by an explicit checked guard. This schedule
/// performs no Gauss–Jordan pivot search.
#[inline]
fn invert_matrix4(matrix: [[Real; 4]; 4]) -> BlasResult<[[Real; 4]; 4]> {
    if matrix4_is_definitely_dense_for_inverse(&matrix) {
        trace_matrix!("helper", "invert-matrix4-dense-cofactor");
        if let Some(inverse) = matrix4_dense_exact_rational_inverse(&matrix) {
            return inverse;
        }
        let (s, c) = matrix4_factors_dense_exact(&matrix);
        let det = determinant4_from_factors(&s, &c);
        let inv_det = det.inverse()?;
        return Ok(matrix4_scaled_adjugate_from_factors_dense_exact(
            &matrix, &s, &c, &inv_det,
        ));
    }
    let facts = matrix4_facts(&matrix);
    if facts.is_identity {
        trace_matrix!("helper", "invert-matrix4-identity");
        return Ok(matrix);
    }
    if facts.is_diagonal {
        trace_matrix!("helper", "invert-matrix4-diagonal");
        return invert_matrix4_by_diagonal(&matrix);
    }
    if facts.is_upper_triangular && !(facts.is_affine && facts.linear_is_diagonal) {
        trace_matrix!("helper", "invert-matrix4-upper-triangular");
        return invert_matrix4_by_upper_triangular(&matrix);
    }
    if facts.is_lower_triangular {
        trace_matrix!("helper", "invert-matrix4-lower-triangular");
        return invert_matrix4_by_lower_triangular(&matrix);
    }
    if facts.is_affine {
        trace_matrix!("helper", "invert-matrix4-affine");
        return invert_matrix4_affine(
            &matrix,
            facts.linear_is_diagonal,
            facts.is_affine_translation,
        );
    }
    let (s, c) = matrix4_factors(&matrix);
    let det = determinant4_from_factors(&s, &c);
    let inv_det = det.inverse()?;
    Ok(matrix4_scaled_adjugate_from_factors(
        &matrix, &s, &c, &inv_det,
    ))
}

/// Uses the 4×4 inverse dispatch order with known-nonzero divisor guards.
///
/// Shares the early witnesses, rational aggregate and shape priority of
/// [`invert_matrix4`]. Rational aggregate results, including singular errors,
/// are final. Other cofactor paths require known nonzero determinant status
/// before constructing its reciprocal and scaled adjugate: zero returns
/// `DivideByZero`, unknown returns `UnknownZero`, without refinement by this
/// guard. Identity returns unchanged; remaining shape shortcuts delegate to
/// their checked divisor schedules. No signal is attached here.
#[inline]
fn invert_matrix4_checked(matrix: [[Real; 4]; 4]) -> CheckedBlasResult<[[Real; 4]; 4]> {
    if matrix4_is_definitely_dense_for_inverse(&matrix) {
        trace_matrix!("helper", "invert-matrix4-checked-dense-cofactor");
        if let Some(inverse) = matrix4_dense_exact_rational_inverse(&matrix) {
            return inverse;
        }
        let (s, c) = matrix4_factors_dense_exact(&matrix);
        let det = determinant4_from_factors(&s, &c);
        require_known_nonzero(&det)?;
        let inv_det = det.inverse()?;
        return Ok(matrix4_scaled_adjugate_from_factors_dense_exact(
            &matrix, &s, &c, &inv_det,
        ));
    }
    let facts = matrix4_facts(&matrix);
    if facts.is_identity {
        trace_matrix!("helper", "invert-matrix4-checked-identity");
        return Ok(matrix);
    }
    if facts.is_diagonal {
        trace_matrix!("helper", "invert-matrix4-checked-diagonal");
        return invert_matrix4_by_diagonal_checked(&matrix);
    }
    if facts.is_upper_triangular && !(facts.is_affine && facts.linear_is_diagonal) {
        trace_matrix!("helper", "invert-matrix4-checked-upper-triangular");
        return invert_matrix4_by_upper_triangular_checked(&matrix);
    }
    if facts.is_lower_triangular {
        trace_matrix!("helper", "invert-matrix4-checked-lower-triangular");
        return invert_matrix4_by_lower_triangular_checked(&matrix);
    }
    if facts.is_affine {
        trace_matrix!("helper", "invert-matrix4-checked-affine");
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

/// Performs checked 4×4 inversion with branch-specific cooperative signal attachment.
///
/// Retains the order of [`invert_matrix4_checked`]. The early rational aggregate
/// returns directly without consuming `signal`, even if it is already set.
/// For the symbolic dense fallback and general cofactor path, constructs minors
/// and determinant first, attaches the signal to that determinant, then guards
/// it, takes its reciprocal and constructs scaled cofactors. Attachment alone
/// does not poll the flag or bound work performed before it.
///
/// Identity, diagonal and triangular shortcuts do not use the signal. Affine
/// handling forwards it to its own dispatcher, whose diagonal-linear and
/// translation shortcuts also bypass attachment. General affine handling
/// delegates to the signal-aware 3×3 linear inverse. Scalar propagation governs
/// later cancellation; this is not a uniform preemptive cancellation contract.
#[inline]
fn invert_matrix4_checked_with_abort(
    matrix: [[Real; 4]; 4],
    signal: &AbortSignal,
) -> CheckedBlasResult<[[Real; 4]; 4]> {
    if matrix4_is_definitely_dense_for_inverse(&matrix) {
        trace_matrix!("helper", "invert-matrix4-checked-with-abort-dense-cofactor");
        if let Some(inverse) = matrix4_dense_exact_rational_inverse(&matrix) {
            return inverse;
        }
        let (s, c) = matrix4_factors_dense_exact(&matrix);
        let det = determinant4_from_factors(&s, &c);
        let det = with_abort(det, signal);
        require_known_nonzero(&det)?;
        let inv_det = det.inverse()?;
        return Ok(matrix4_scaled_adjugate_from_factors_dense_exact(
            &matrix, &s, &c, &inv_det,
        ));
    }
    let facts = matrix4_facts(&matrix);
    if facts.is_identity {
        trace_matrix!("helper", "invert-matrix4-checked-with-abort-identity");
        return Ok(matrix);
    }
    if facts.is_diagonal {
        trace_matrix!("helper", "invert-matrix4-checked-with-abort-diagonal");
        return invert_matrix4_by_diagonal_checked(&matrix);
    }
    if facts.is_upper_triangular && !(facts.is_affine && facts.linear_is_diagonal) {
        trace_matrix!(
            "helper",
            "invert-matrix4-checked-with-abort-upper-triangular"
        );
        return invert_matrix4_by_upper_triangular_checked(&matrix);
    }
    if facts.is_lower_triangular {
        trace_matrix!(
            "helper",
            "invert-matrix4-checked-with-abort-lower-triangular"
        );
        return invert_matrix4_by_lower_triangular_checked(&matrix);
    }
    if facts.is_affine {
        trace_matrix!("helper", "invert-matrix4-checked-with-abort-affine");
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

/// Builds the unscaled 4×4 adjugate for division that delays a shared reciprocal.
///
/// Requires current source minors in [`matrix4_factors`] order. Constructs all
/// sixteen transposed cofactors row-major, using definite-zero pruning while
/// retaining unknown factors. Singular matrices still have an adjugate; this
/// helper neither inverts nor tests the determinant. Right-division callers
/// can multiply their dividend by this matrix before applying the common scale.
/// Source/minor consistency is a caller invariant, not checked here.
#[inline]
fn matrix4_adjugate_from_factors(
    m: &[[Real; 4]; 4],
    s: &[Real; 6],
    c: &[Real; 6],
) -> [[Real; 4]; 4] {
    trace_matrix!("helper", "matrix4-unscaled-adjugate-from-factors");
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

/// Builds the unscaled 4×4 adjugate with unchecked known-rational reducers.
///
/// Requires the same source/minor consistency as [`matrix4_adjugate_from_factors`],
/// plus current exact-rational representations for every matrix entry and minor.
/// Uses unpruned three-product reducers for the same row-major transposed
/// cofactors. Zeros and singular matrices are valid; no reciprocal is taken.
/// Incorrect representation evidence is not guaranteed to yield a typed error,
/// since the scalar reducers read stored rational components directly.
#[inline(never)]
fn matrix4_adjugate_from_factors_dense_exact_known_rational(
    m: &[[Real; 4]; 4],
    s: &[Real; 6],
    c: &[Real; 6],
) -> [[Real; 4]; 4] {
    trace_matrix!(
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

/// Generates the common Matrix3/Matrix4 constructors, arithmetic and formatting API.
///
/// The named tuple carrier must hold a row-major square Real array; the vector
/// carrier has the matching dimension. The supplied hooks implement ordinary
/// and checked right division, unsigned powers and ownership-specific products.
/// Current instantiations are dimensions three and four; the shared vector
/// transform helper is specialized to those dimensions, not arbitrary sizes.
///
/// Matrix products act on column vectors; scalar addition/subtraction broadcast
/// to every entry. Division returns a typed result, and checked variants retain
/// zero-versus-unknown errors. This macro supplies no independent predicate
/// policy, certainty receipt, arithmetic cache or total-work budget.
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
        /// Shared constructors, scalar operations and fallible matrix operations.
        impl $name {
            /// Moves row-major Real entries into the matrix without validation or conversion.
            ///
            /// The array type fixes the dimensions. Singular and symbolic entries are accepted;
            /// this constructor does not establish invertibility or a geometric transform kind.
            pub fn new(values: [[Real; $n]; $n]) -> Self {
                trace_matrix!("constructor", "new");
                Self(values)
            }

            /// Constructs an array of exact scalar zeros in row-major order.
            ///
            /// This is the additive identity, and is singular for both supported dimensions.
            pub fn zero() -> Self {
                trace_matrix!("constructor", "zero");
                Self(from_fn(|_| from_fn(|_| Real::zero())))
            }

            /// Constructs the multiplicative identity with exact ones on the diagonal.
            ///
            /// Off-diagonal entries are exact zeros; no arithmetic or shape recognition is needed.
            pub fn identity() -> Self {
                trace_matrix!("constructor", "identity");
                Self(from_fn(|row| {
                    from_fn(|col| {
                        if row == col {
                            Real::one()
                        } else {
                            Real::zero()
                        }
                    })
                }))
            }

            /// Returns the transpose by cloning scalar handles in output row-major order.
            ///
            /// Entry `(row, col)` comes from `(col, row)`. No arithmetic or conjugation occurs;
            /// scalar sharing can remain, so this is not independent proof/cache storage.
            pub fn transpose(&self) -> Self {
                trace_matrix!("method", "transpose");
                Self(from_fn(|row| from_fn(|col| self.0[col][row].clone())))
            }

            /// Consumes the matrix and forwards to [`Self::inverse`].
            ///
            /// This compatibility spelling adds its own dispatch event before the inverse's
            /// schedule. It is a matrix inverse, not componentwise reciprocation, and preserves
            /// the ordinary inverse's domain errors and handling of unknown status.
            pub fn reciprocal(self) -> BlasResult<Self> {
                trace_matrix!("method", "reciprocal");
                self.inverse()
            }

            /// Consumes the matrix and forwards to [`Self::inverse_checked`].
            ///
            /// Adds its own dispatch event, preserving the checked inverse's divisor guards
            /// and typed errors. This is a matrix inverse, not componentwise reciprocation.
            pub fn reciprocal_checked(self) -> CheckedBlasResult<Self> {
                trace_matrix!("method", "reciprocal-checked");
                self.inverse_checked()
            }

            /// Raises the matrix to an integer power, inverting first for negative exponents.
            ///
            /// Zero returns identity even for a singular base; one returns the base.
            /// Minus one returns its inverse directly. Other magnitudes use borrowed
            /// square/cube specializations or binary squaring. `i32::MIN` is handled
            /// through its unsigned magnitude, without signed negation overflow.
            /// Scalar inversion errors propagate; no general scalar-work bound is supplied.
            pub fn powi(self, exponent: i32) -> BlasResult<Self> {
                trace_matrix!("method", "powi");
                if exponent == -1 {
                    trace_matrix!("powi", "negative-one-inverse");
                    return self.inverse();
                }
                let base = if exponent < 0 {
                    self.inverse()?.0
                } else {
                    self.0
                };
                Ok(Self($power_fn(base, exponent.unsigned_abs())))
            }

            /// Raises the matrix to an integer power using checked inversion when negative.
            ///
            /// Nonnegative exponents use the same multiplication paths as [`Self::powi`]
            /// without an invertibility guard. Negative exponents first require the
            /// checked inverse; zero/unknown pivot or determinant errors propagate.
            pub fn powi_checked(self, exponent: i32) -> CheckedBlasResult<Self> {
                trace_matrix!("method", "powi-checked");
                if exponent == -1 {
                    trace_matrix!("powi", "negative-one-inverse-checked");
                    return self.inverse_checked();
                }
                let base = if exponent < 0 {
                    self.inverse_checked()?.0
                } else {
                    self.0
                };
                Ok(Self($power_fn(base, exponent.unsigned_abs())))
            }

            /// Raises the matrix to an integer power, using abort-aware inversion when negative.
            ///
            /// For nonnegative exponents this method neither attaches nor polls `signal`;
            /// it follows the ordinary power path. Negative exponents first invoke
            /// [`Self::inverse_checked_with_abort`], then multiply the returned entries.
            /// Minus one returns that inverse directly. No new signal attachment or
            /// polling occurs in the subsequent exponentiation loop, and exact paths
            /// need not stop merely because the flag is set.
            pub fn powi_checked_with_abort(
                self,
                exponent: i32,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                trace_matrix!("method", "powi-checked-with-abort");
                if exponent == -1 {
                    trace_matrix!("powi", "negative-one-inverse-checked-with-abort");
                    return self.inverse_checked_with_abort(signal);
                }
                let base = if exponent < 0 {
                    self.inverse_checked_with_abort(signal)?.0
                } else {
                    self.0
                };
                Ok(Self($power_fn(base, exponent.unsigned_abs())))
            }

            /// Divides every entry by one shared scalar reciprocal after a nonzero guard.
            ///
            /// Queries the divisor before any scaling: known zero returns `DivideByZero`,
            /// unknown status returns `UnknownZero`, without refinement by this guard.
            /// Then constructs one reciprocal and multiplies entries row-major through the
            /// borrowed-factor adapter. Even an all-zero matrix must pass the divisor guard;
            /// other scalar reciprocal errors propagate before output construction.
            pub fn div_scalar_checked(self, rhs: Real) -> CheckedBlasResult<Self> {
                trace_matrix!("method", "div-scalar-checked");
                require_known_nonzero(&rhs)?;
                let inv_rhs = rhs.inverse()?;
                Ok(Self(
                    self.0
                        .map(|row| row.map(|value| value.mul_cached(&inv_rhs))),
                ))
            }

            /// Attaches the signal to the divisor before checked componentwise division.
            ///
            /// Attachment precedes the known-nonzero guard, even on zero/unknown failure.
            /// On success, constructs one reciprocal and scales entries row-major as in
            /// [`Self::div_scalar_checked`]. Attachment alone does not poll the flag; scalar
            /// propagation governs later cancellation. This is not a preemptive work bound.
            pub fn div_scalar_checked_with_abort(
                self,
                rhs: Real,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                trace_matrix!("method", "div-scalar-checked-with-abort");
                let rhs = with_abort(rhs, signal);
                require_known_nonzero(&rhs)?;
                let inv_rhs = rhs.inverse()?;
                Ok(Self(
                    self.0
                        .map(|row| row.map(|value| value.mul_cached(&inv_rhs))),
                ))
            }

            /// Computes right division `X * rhs = self` using checked divisor schedules.
            ///
            /// For invertible `rhs`, the result is `self * rhs⁻¹`, with operand order preserved.
            /// Dispatch may use structural formulas, shared adjugates or a transposed solve;
            /// it need not materialize the inverse. Required zero/unknown pivots or divisors
            /// return `DivideByZero`/`UnknownZero`; scalar errors also propagate. A zero dividend
            /// does not authorize bypassing a singular divisor. Only completed results return.
            pub fn div_matrix_checked(self, rhs: Self) -> CheckedBlasResult<Self> {
                trace_matrix!("method", "div-matrix-checked");
                Ok(Self($div_checked_fn(self.0, rhs.0)?))
            }

            /// Computes checked right division with branch-specific cooperative signal use.
            ///
            /// Has the equation and failure contract of [`Self::div_matrix_checked`]. Forwards
            /// `signal` to the dimension-specific dispatcher; structural shortcuts may bypass
            /// attachment, while cofactor or solve paths attach to their selected divisors.
            /// There is no unconditional polling here, and a set flag need not prevent an
            /// exact shortcut from completing. No uniform cancellation latency is promised.
            pub fn div_matrix_checked_with_abort(
                self,
                rhs: Self,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                trace_matrix!("method", "div-matrix-checked-with-abort");
                Ok(Self($div_checked_abort_fn(self.0, rhs.0, signal)?))
            }
        }

        /// Borrows rows through zero-based array indexing.
        impl Index<usize> for $name {
            /// One complete row of scalar entries.
            type Output = [Real; $n];

            /// Borrows the selected row without copying entries.
            ///
            /// # Panics
            /// Panics when the row index is outside the matrix dimension.
            fn index(&self, index: usize) -> &Self::Output {
                &self.0[index]
            }
        }

        /// Exposes mutable row storage without preserving previously collected matrix facts.
        impl IndexMut<usize> for $name {
            /// Mutably borrows the selected row, allowing any Real entries to replace it.
            ///
            /// Callers must re-establish retained facts affected by mutation.
            ///
            /// # Panics
            /// Panics when the row index is outside the matrix dimension.
            fn index_mut(&mut self, index: usize) -> &mut Self::Output {
                &mut self.0[index]
            }
        }

        /// Formats nested row-major brackets for human-readable matrix diagnostics.
        impl fmt::Display for $name {
            /// Writes rows and columns with comma-space separators.
            ///
            /// Ordinary formatting uses each Real display. Alternate formatting (`{:#}`)
            /// uses `diagnostic_decimal(-256)` with its default 32 decimal places and
            /// trailing-zero removal; scalar conversion failures appear as nonnumeric
            /// markers. Outer precision, width and alignment are not forwarded to entries
            /// or applied to the whole matrix. Writer errors propagate as `fmt::Error`.
            /// This display is not an exact serialization or a geometric certificate.
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
                            write!(f, "{}", self.0[row][col].diagnostic_decimal(-256))?;
                        } else {
                            write!(f, "{}", self.0[row][col])?;
                        }
                    }
                    f.write_str("]")?;
                }
                f.write_str("]")
            }
        }

        /// Entrywise matrix addition, consuming both matrices.
        impl Add for $name {
            /// The owned matrix of entrywise results.
            type Output = Self;

            /// Forms each output entry as `left[row][col] + right[row][col]`.
            ///
            /// Updates the owned left entries row-major through scalar assignment. Operand order is
            /// preserved.
            fn add(mut self, rhs: Self) -> Self::Output {
                trace_matrix!("op", "add-owned-owned");
                for row in 0..$n {
                    for col in 0..$n {
                        self.0[row][col] += &rhs.0[row][col];
                    }
                }
                self
            }
        }

        /// Entrywise matrix addition, consuming the left matrix and borrowing the right.
        impl Add<&$name> for $name {
            /// The owned matrix of entrywise results.
            type Output = Self;

            /// Forms each output entry as `left[row][col] + right[row][col]`.
            ///
            /// Maps entries row-major through the borrowed-right scalar adapter. Operand order is
            /// preserved.
            fn add(self, rhs: &$name) -> Self::Output {
                trace_matrix!("op", "add-owned-ref");
                Self(map_matrix_ref(self.0, &rhs.0, Real::add_cached))
            }
        }

        /// Entrywise matrix addition, borrowing the left matrix and consuming the right.
        impl Add<$name> for &$name {
            /// The owned matrix of entrywise results.
            type Output = $name;

            /// Forms each output entry as `left[row][col] + right[row][col]`.
            ///
            /// Maps entries row-major with borrowed left and owned right scalars. Operand order is
            /// preserved.
            fn add(self, rhs: $name) -> Self::Output {
                trace_matrix!("op", "add-ref-owned");
                $name(map_matrix_left_ref(&self.0, rhs.0, |lhs, rhs| lhs + rhs))
            }
        }

        /// Entrywise matrix addition, borrowing both matrices.
        impl Add<&$name> for &$name {
            /// The owned matrix of entrywise results.
            type Output = $name;

            /// Forms each output entry as `left[row][col] + right[row][col]`.
            ///
            /// Constructs output entries row-major through borrowed scalar arithmetic. Operand
            /// order is preserved.
            fn add(self, rhs: &$name) -> Self::Output {
                trace_matrix!("op", "add-ref-ref");
                $name(from_fn(|row| {
                    from_fn(|col| &self.0[row][col] + &rhs.0[row][col])
                }))
            }
        }

        /// Broadcasts scalar addition across an owned matrix.
        impl Add<Real> for $name {
            /// The owned matrix after scalar broadcasting.
            type Output = Self;

            /// Applies `+ rhs` to every entry, including off-diagonal entries.
            ///
            /// Uses a borrowed reusable scalar in each row-major addition. This is not addition of
            /// a scalar multiple of identity.
            fn add(self, rhs: Real) -> Self::Output {
                trace_matrix!("op", "add-scalar-owned");
                let rhs = &rhs;
                Self(self.0.map(|row| row.map(|value| value.add_cached(rhs))))
            }
        }

        /// Broadcasts scalar addition across an owned matrix.
        impl Add<&Real> for $name {
            /// The owned matrix after scalar broadcasting.
            type Output = Self;

            /// Applies `+ rhs` to every entry, including off-diagonal entries.
            ///
            /// Uses a borrowed reusable scalar in each row-major addition. This is not addition of
            /// a scalar multiple of identity.
            fn add(self, rhs: &Real) -> Self::Output {
                trace_matrix!("op", "add-scalar-ref");
                Self(self.0.map(|row| row.map(|value| value.add_cached(rhs))))
            }
        }

        /// Entrywise matrix subtraction, consuming both matrices.
        impl Sub for $name {
            /// The owned matrix of entrywise results.
            type Output = Self;

            /// Forms each output entry as `left[row][col] - right[row][col]`.
            ///
            /// Updates the owned left entries row-major through scalar assignment. Operand order is
            /// preserved.
            fn sub(mut self, rhs: Self) -> Self::Output {
                trace_matrix!("op", "sub-owned-owned");
                for row in 0..$n {
                    for col in 0..$n {
                        self.0[row][col] -= &rhs.0[row][col];
                    }
                }
                self
            }
        }

        /// Entrywise matrix subtraction, consuming the left matrix and borrowing the right.
        impl Sub<&$name> for $name {
            /// The owned matrix of entrywise results.
            type Output = Self;

            /// Forms each output entry as `left[row][col] - right[row][col]`.
            ///
            /// Maps entries row-major through the borrowed-right scalar adapter. Operand order is
            /// preserved.
            fn sub(self, rhs: &$name) -> Self::Output {
                trace_matrix!("op", "sub-owned-ref");
                Self(map_matrix_ref(self.0, &rhs.0, Real::sub_cached))
            }
        }

        /// Entrywise matrix subtraction, borrowing the left matrix and consuming the right.
        impl Sub<$name> for &$name {
            /// The owned matrix of entrywise results.
            type Output = $name;

            /// Forms each output entry as `left[row][col] - right[row][col]`.
            ///
            /// Maps entries row-major with borrowed left and owned right scalars. Operand order is
            /// preserved.
            fn sub(self, rhs: $name) -> Self::Output {
                trace_matrix!("op", "sub-ref-owned");
                $name(map_matrix_left_ref(&self.0, rhs.0, |lhs, rhs| lhs - rhs))
            }
        }

        /// Entrywise matrix subtraction, borrowing both matrices.
        impl Sub<&$name> for &$name {
            /// The owned matrix of entrywise results.
            type Output = $name;

            /// Forms each output entry as `left[row][col] - right[row][col]`.
            ///
            /// Constructs output entries row-major through borrowed scalar arithmetic. Operand
            /// order is preserved.
            fn sub(self, rhs: &$name) -> Self::Output {
                trace_matrix!("op", "sub-ref-ref");
                $name(from_fn(|row| {
                    from_fn(|col| &self.0[row][col] - &rhs.0[row][col])
                }))
            }
        }

        /// Broadcasts scalar subtraction across an owned matrix.
        impl Sub<Real> for $name {
            /// The owned matrix after scalar broadcasting.
            type Output = Self;

            /// Applies `- rhs` to every entry, including off-diagonal entries.
            ///
            /// Negates the owned scalar once, then uses borrowed addition. This is not addition of
            /// a scalar multiple of identity.
            fn sub(self, rhs: Real) -> Self::Output {
                trace_matrix!("op", "sub-scalar-owned");
                let rhs = -rhs;
                let rhs = &rhs;
                Self(self.0.map(|row| row.map(|value| value.add_cached(rhs))))
            }
        }

        /// Broadcasts scalar subtraction across an owned matrix.
        impl Sub<&Real> for $name {
            /// The owned matrix after scalar broadcasting.
            type Output = Self;

            /// Applies `- rhs` to every entry, including off-diagonal entries.
            ///
            /// Clones and negates the borrowed scalar once, then uses borrowed addition. This is
            /// not addition of a scalar multiple of identity.
            fn sub(self, rhs: &Real) -> Self::Output {
                trace_matrix!("op", "sub-scalar-ref");
                let rhs = -rhs.clone();
                Self(self.0.map(|row| row.map(|value| value.add_cached(&rhs))))
            }
        }

        /// Entrywise negation of a consumed matrix.
        impl Neg for $name {
            /// The owned additive inverse matrix.
            type Output = Self;

            /// Negates each scalar through the fixed-size row-major owned schedule.
            fn neg(self) -> Self::Output {
                trace_matrix!("op", "neg-owned");
                Self(<[[Real; $n]; $n] as MatrixNegOwned>::neg_owned(self.0))
            }
        }

        /// Entrywise negation of a borrowed matrix.
        impl Neg for &$name {
            /// The owned additive inverse matrix.
            type Output = $name;

            /// Negates each scalar through the fixed-size row-major borrowed schedule.
            fn neg(self) -> Self::Output {
                trace_matrix!("op", "neg-ref");
                $name(<[[Real; $n]; $n] as MatrixNegRefs>::neg_refs(&self.0))
            }
        }

        /// Scales every entry of an owned matrix by a consumed scalar.
        impl Mul<Real> for $name {
            /// The owned scaled matrix.
            type Output = Self;

            /// Multiplies entries row-major in place using scalar multiplication assignment.
            ///
            /// Zero and symbolic factors are accepted; no invertibility condition is required.
            fn mul(mut self, rhs: Real) -> Self::Output {
                trace_matrix!("op", "mul-scalar-owned");
                for row in &mut self.0 {
                    for value in row {
                        *value *= &rhs;
                    }
                }
                self
            }
        }

        /// Scales every entry of an owned matrix by a borrowed scalar.
        impl Mul<&Real> for $name {
            /// The owned scaled matrix.
            type Output = Self;

            /// Multiplies entries row-major through the borrowed-factor adapter.
            ///
            /// Zero and symbolic factors are accepted; no invertibility condition is required.
            fn mul(self, rhs: &Real) -> Self::Output {
                trace_matrix!("op", "mul-scalar-ref");
                Self(self.0.map(|row| row.map(|value| value.mul_cached(rhs))))
            }
        }

        // The shared reciprocal is computed once, then applied to every lane.
        #[allow(clippy::suspicious_arithmetic_impl)]
        /// Fallible componentwise division by a consumed scalar.
        impl Div<Real> for $name {
            /// The scaled matrix or a scalar division error.
            type Output = BlasResult<Self>;

            /// Rejects a definitely-zero divisor, then constructs one shared reciprocal.
            ///
            /// Unknown zero status is not rejected by this ordinary guard; scalar inverse
            /// errors propagate. Scales all entries row-major only after reciprocal success.
            /// Consumes the divisor for its reciprocal, then scales entries in place. Even a zero
            /// dividend does not bypass the divisor.
            fn div(mut self, rhs: Real) -> Self::Output {
                trace_matrix!("op", "div-scalar-owned");
                reject_definite_zero(&rhs)?;
                let inv_rhs = rhs.inverse()?;
                for row in &mut self.0 {
                    for value in row {
                        *value *= &inv_rhs;
                    }
                }
                Ok(self)
            }
        }

        #[allow(clippy::suspicious_arithmetic_impl)]
        /// Fallible componentwise division by a borrowed scalar.
        impl Div<&Real> for $name {
            /// The scaled matrix or a scalar division error.
            type Output = BlasResult<Self>;

            /// Rejects a definitely-zero divisor, then constructs one shared reciprocal.
            ///
            /// Unknown zero status is not rejected by this ordinary guard; scalar inverse
            /// errors propagate. Scales all entries row-major only after reciprocal success.
            /// Uses a borrowed reciprocal; Matrix3 multiplies through borrowed scalars,
            /// while Matrix4 uses the borrowed-factor adapter. Even a zero dividend does not bypass
            /// the divisor.
            fn div(self, rhs: &Real) -> Self::Output {
                trace_matrix!("op", "div-scalar-ref");
                reject_definite_zero(rhs)?;
                let inv_rhs = rhs.inverse_ref()?;
                if $n == 3 {
                    Ok(Self(self.0.map(|row| row.map(|value| &value * &inv_rhs))))
                } else {
                    Ok(Self(
                        self.0
                            .map(|row| row.map(|value| value.mul_cached(&inv_rhs))),
                    ))
                }
            }
        }

        /// Matrix composition, consuming both matrices.
        impl Mul for $name {
            /// The owned matrix product.
            type Output = Self;

            /// Forms `left * right`, with entry `(i,j)` equal to the sum of
            /// `left[i][k]*right[k][j]`.
            ///
            /// On column vectors the right transformation acts first. Delegates to the owned
            /// product dispatcher.
            fn mul(self, rhs: Self) -> Self::Output {
                trace_matrix!("op", "mul-owned-owned");
                Self($mul_owned_fn(self.0, rhs.0))
            }
        }

        /// Matrix composition, consuming the left matrix and borrowing the right.
        impl Mul<&$name> for $name {
            /// The owned matrix product.
            type Output = Self;

            /// Forms `left * right`, with entry `(i,j)` equal to the sum of
            /// `left[i][k]*right[k][j]`.
            ///
            /// On column vectors the right transformation acts first. Delegates to the
            /// owned-left/borrowed-right product dispatcher.
            fn mul(self, rhs: &$name) -> Self::Output {
                trace_matrix!("op", "mul-owned-ref");
                Self($mul_rhs_ref_fn(self.0, &rhs.0))
            }
        }

        /// Matrix composition, borrowing the left matrix and consuming the right.
        impl Mul<$name> for &$name {
            /// The owned matrix product.
            type Output = $name;

            /// Forms `left * right`, with entry `(i,j)` equal to the sum of
            /// `left[i][k]*right[k][j]`.
            ///
            /// On column vectors the right transformation acts first. Uses the fully borrowed
            /// product dispatcher before dropping the owned right matrix.
            fn mul(self, rhs: $name) -> Self::Output {
                trace_matrix!("op", "mul-ref-owned");
                $name($mul_ref_fn(&self.0, &rhs.0))
            }
        }

        /// Matrix composition, borrowing both matrices.
        impl Mul<&$name> for &$name {
            /// The owned matrix product.
            type Output = $name;

            /// Forms `left * right`, with entry `(i,j)` equal to the sum of
            /// `left[i][k]*right[k][j]`.
            ///
            /// On column vectors the right transformation acts first. Delegates to the fully
            /// borrowed product dispatcher.
            fn mul(self, rhs: &$name) -> Self::Output {
                trace_matrix!("op", "mul-ref-ref");
                $name($mul_ref_fn(&self.0, &rhs.0))
            }
        }

        /// Fallible right division, consuming both matrices.
        impl Div for $name {
            /// The owned right quotient or a propagated division error.
            type Output = BlasResult<Self>;

            /// Solves `X * right = left`, mathematically `left * right⁻¹` for invertible right.
            ///
            /// Delegates to the ordinary owned right-division dispatcher. The dispatcher selects
            /// structural, cofactor or transposed-solve
            /// schedules; inverse materialization is not required. Scalar domain errors
            /// propagate. Ordinary paths do not uniformly certify unknown divisor status;
            /// use the checked method when known-nonzero guards are required.
            fn div(self, rhs: Self) -> Self::Output {
                trace_matrix!("op", "div-owned-owned");
                Ok(Self($div_fn(self.0, rhs.0)?))
            }
        }

        /// Fallible right division, consuming the left matrix and borrowing the right.
        impl Div<&$name> for $name {
            /// The owned right quotient or a propagated division error.
            type Output = BlasResult<Self>;

            /// Solves `X * right = left`, mathematically `left * right⁻¹` for invertible right.
            ///
            /// Clones the right matrix handles and invokes owned/owned division. The dispatcher
            /// selects structural, cofactor or transposed-solve
            /// schedules; inverse materialization is not required. Scalar domain errors
            /// propagate. Ordinary paths do not uniformly certify unknown divisor status;
            /// use the checked method when known-nonzero guards are required.
            fn div(self, rhs: &$name) -> Self::Output {
                trace_matrix!("op", "div-owned-ref");
                self / rhs.clone()
            }
        }

        /// Fallible right division, borrowing the left matrix and consuming the right.
        impl Div<$name> for &$name {
            /// The owned right quotient or a propagated division error.
            type Output = BlasResult<$name>;

            /// Solves `X * right = left`, mathematically `left * right⁻¹` for invertible right.
            ///
            /// Clones the left matrix handles and invokes owned/owned division. The dispatcher
            /// selects structural, cofactor or transposed-solve
            /// schedules; inverse materialization is not required. Scalar domain errors
            /// propagate. Ordinary paths do not uniformly certify unknown divisor status;
            /// use the checked method when known-nonzero guards are required.
            fn div(self, rhs: $name) -> Self::Output {
                trace_matrix!("op", "div-ref-owned");
                self.clone() / rhs
            }
        }

        /// Fallible right division, borrowing both matrices.
        impl Div<&$name> for &$name {
            /// The owned right quotient or a propagated division error.
            type Output = BlasResult<$name>;

            /// Solves `X * right = left`, mathematically `left * right⁻¹` for invertible right.
            ///
            /// Delegates directly to the fully borrowed right-division dispatcher. The dispatcher
            /// selects structural, cofactor or transposed-solve
            /// schedules; inverse materialization is not required. Scalar domain errors
            /// propagate. Ordinary paths do not uniformly certify unknown divisor status;
            /// use the checked method when known-nonzero guards are required.
            fn div(self, rhs: &$name) -> Self::Output {
                trace_matrix!("op", "div-ref-ref");
                Ok($name($div_ref_fn(&self.0, &rhs.0)?))
            }
        }

        /// Column-vector transformation, consuming the matrix and consuming the vector.
        impl Mul<$vector> for $name {
            /// The owned transformed vector with the original dimension.
            type Output = $vector;

            /// Forms one row-dot-vector result per output coordinate through the shared
            /// borrowed-input dispatcher. Owned operands remain available to that call.
            ///
            /// Matrix4 uses the supplied homogeneous weight, including nonunit values;
            /// this operation does not divide by the output weight or convert to a point.
            fn mul(self, rhs: $vector) -> Self::Output {
                trace_matrix!("op", "transform-vector-owned-owned");
                $vector(transform_vector_rhs_ref(&self.0, &rhs.0))
            }
        }

        /// Column-vector transformation, consuming the matrix and borrowing the vector.
        impl Mul<&$vector> for $name {
            /// The owned transformed vector with the original dimension.
            type Output = $vector;

            /// Forms one row-dot-vector result per output coordinate through the shared
            /// borrowed-input dispatcher. Owned operands remain available to that call.
            ///
            /// Matrix4 uses the supplied homogeneous weight, including nonunit values;
            /// this operation does not divide by the output weight or convert to a point.
            fn mul(self, rhs: &$vector) -> Self::Output {
                trace_matrix!("op", "transform-vector-owned-ref");
                $vector(transform_vector_rhs_ref(&self.0, &rhs.0))
            }
        }

        /// Column-vector transformation, borrowing the matrix and consuming the vector.
        impl Mul<$vector> for &$name {
            /// The owned transformed vector with the original dimension.
            type Output = $vector;

            /// Forms one row-dot-vector result per output coordinate through the shared
            /// borrowed-input dispatcher. Owned operands remain available to that call.
            ///
            /// Matrix4 uses the supplied homogeneous weight, including nonunit values;
            /// this operation does not divide by the output weight or convert to a point.
            fn mul(self, rhs: $vector) -> Self::Output {
                trace_matrix!("op", "transform-vector-ref-owned");
                $vector(transform_vector_rhs_ref(&self.0, &rhs.0))
            }
        }

        /// Column-vector transformation, borrowing the matrix and borrowing the vector.
        impl Mul<&$vector> for &$name {
            /// The owned transformed vector with the original dimension.
            type Output = $vector;

            /// Forms one row-dot-vector result per output coordinate through the shared
            /// borrowed-input dispatcher. Owned operands remain available to that call.
            ///
            /// Matrix4 uses the supplied homogeneous weight, including nonunit values;
            /// this operation does not divide by the output weight or convert to a point.
            fn mul(self, rhs: &$vector) -> Self::Output {
                trace_matrix!("op", "transform-vector-ref-ref");
                $vector(transform_vector_rhs_ref(&self.0, &rhs.0))
            }
        }

        /// Uses the caret operator as fallible integer matrix exponentiation.
        impl BitXor<i32> for $name {
            /// The powered matrix or an inverse error.
            type Output = BlasResult<Self>;

            /// Forwards the exponent to [`Self::powi`], including negative inverse powers.
            ///
            /// This is arithmetic matrix power, not entrywise power or bitwise XOR.
            fn bitxor(self, rhs: i32) -> Self::Output {
                trace_matrix!("op", "bitxor-powi");
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

impl Matrix3 {
    /// Returns exact-rational representation facts for all matrix entries.
    ///
    /// This exposes the matrix-level common-scale signal without exposing
    /// rational storage. Callers that retain this fact can select dyadic or
    /// shared-denominator exact schedules before entering determinant, inverse,
    /// or predicate preparation code. Object structure stays visible long enough
    /// to choose the exact arithmetic package.
    pub fn exact_facts(&self) -> RealExactSetFacts {
        trace_matrix!("query", "matrix3-exact-facts");
        matrix3_facts(&self.0).public.exact
    }

    /// Returns structural and exact-rational facts for this matrix.
    ///
    /// This is the object-level fact boundary for matrix callers: zero/one
    /// masks, triangular/affine shape, and exact coordinate-set facts are
    /// gathered together without exposing scalar storage internals. Keeping the
    /// summary at the matrix layer matches the recommendation to select exact
    /// geometric algorithms from retained object structure before scalar
    /// expansion.
    pub fn structural_facts(&self) -> Matrix3StructuralFacts {
        trace_matrix!("query", "matrix3-structural-facts");
        matrix3_facts(&self.0).public
    }

    /// Constructs a 3x3 diagonal matrix from known diagonal entries.
    pub fn diagonal(diagonal: [Real; 3]) -> Self {
        trace_matrix!("constructor", "diagonal3");
        let [d0, d1, d2] = diagonal;
        Self([
            [d0, Real::zero(), Real::zero()],
            [Real::zero(), d1, Real::zero()],
            [Real::zero(), Real::zero(), d2],
        ])
    }

    /// Constructs the inverse of a known 3x3 diagonal matrix.
    ///
    /// This opt-in constructor carries the diagonal object fact from the caller
    /// instead of rediscovering it with structural probes inside
    /// [`Matrix3::reciprocal`]. That keeps ordinary inverse/division paths flat
    /// while preserving the exact diagonal solve `D^-1 = diag(1/d_i)` when the
    /// geometry layer already knows the matrix shape. The choice follows the
    /// object-package guidance for exact geometric computation the exact object-structure policy and the diagonal-system specialization in
    /// Golub and Van Loan, *Matrix Computations*.
    pub fn diagonal_inverse(diagonal: [Real; 3]) -> BlasResult<Self> {
        trace_matrix!("constructor", "diagonal3-inverse");
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
    /// while exploiting geometric-object structure. The algebra is the standard
    /// diagonal linear-system specialization described by Golub and Van Loan,
    /// *Matrix Computations*.
    pub fn div_diagonal(self, diagonal: [Real; 3]) -> BlasResult<Self> {
        trace_matrix!("method", "div-diagonal3");
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
        trace_matrix!("method", "upper-triangular3-inverse");
        Ok(Self(invert_matrix3_upper_triangular(&self.0)?))
    }

    /// Checked variant of [`Matrix3::upper_triangular_inverse`].
    pub fn upper_triangular_inverse_checked(self) -> CheckedBlasResult<Self> {
        trace_matrix!("method", "upper-triangular3-inverse-checked");
        Ok(Self(invert_matrix3_upper_triangular_checked(&self.0)?))
    }

    /// Inverts a caller-certified lower-triangular 3x3 matrix.
    pub fn lower_triangular_inverse(self) -> BlasResult<Self> {
        trace_matrix!("method", "lower-triangular3-inverse");
        Ok(Self(invert_matrix3_lower_triangular(&self.0)?))
    }

    /// Checked variant of [`Matrix3::lower_triangular_inverse`].
    pub fn lower_triangular_inverse_checked(self) -> CheckedBlasResult<Self> {
        trace_matrix!("method", "lower-triangular3-inverse-checked");
        Ok(Self(invert_matrix3_lower_triangular_checked(&self.0)?))
    }

    /// Right-divides by a caller-certified upper-triangular 3x3 matrix.
    pub fn div_upper_triangular(self, divisor: Self) -> BlasResult<Self> {
        trace_matrix!("method", "div-upper-triangular3");
        Ok(Self(divide_matrix3_by_upper_triangular(
            self.0, &divisor.0,
        )?))
    }

    /// Checked variant of [`Matrix3::div_upper_triangular`].
    pub fn div_upper_triangular_checked(self, divisor: Self) -> CheckedBlasResult<Self> {
        trace_matrix!("method", "div-upper-triangular3-checked");
        Ok(Self(divide_matrix3_by_upper_triangular_checked(
            self.0, &divisor.0,
        )?))
    }

    /// Right-divides by a caller-certified lower-triangular 3x3 matrix.
    pub fn div_lower_triangular(self, divisor: Self) -> BlasResult<Self> {
        trace_matrix!("method", "div-lower-triangular3");
        Ok(Self(divide_matrix3_by_lower_triangular(
            self.0, &divisor.0,
        )?))
    }

    /// Checked variant of [`Matrix3::div_lower_triangular`].
    pub fn div_lower_triangular_checked(self, divisor: Self) -> CheckedBlasResult<Self> {
        trace_matrix!("method", "div-lower-triangular3-checked");
        Ok(Self(divide_matrix3_by_lower_triangular_checked(
            self.0, &divisor.0,
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
    /// This path is an opt-in structural fast path for a known diagonal divisor.
    pub fn div_diagonal_vector(&self, diagonal: [Real; 3], rhs: &Vector3) -> BlasResult<Vector3> {
        trace_matrix!("method", "div-diagonal3-vector");
        let [d0, d1, d2] = diagonal;
        let (inv0, inv1, inv2) = if d0 == d1 && d0 == d2 {
            trace_matrix!("helper", "div-diagonal3-vector-uniform-scale");
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
        let mapped = transform_vector3_rhs_dense_active_ref(&self.0, &rhs_div);
        Ok(Vector3(mapped))
    }

    /// Constructs a scalar multiple of the 3x3 identity matrix.
    pub fn uniform_scale(scale: Real) -> Self {
        trace_matrix!("constructor", "uniform-scale3");
        Self([
            [scale.clone(), Real::zero(), Real::zero()],
            [Real::zero(), scale.clone(), Real::zero()],
            [Real::zero(), Real::zero(), scale],
        ])
    }

    /// Constructs the inverse of a known scalar multiple of the 3x3 identity.
    ///
    /// This mirrors [`Matrix4::uniform_scale_inverse`] for 2D homogeneous
    /// geometry. It is intentionally explicit: previous hidden uniform-scale
    /// detection regressed adjacent diagonal reciprocal paths because equality
    /// checks taxed every diagonal matrix. When a caller already knows `A = sI`,
    /// one scalar inverse and two clones are sufficient.
    pub fn uniform_scale_inverse(scale: Real) -> BlasResult<Self> {
        trace_matrix!("constructor", "uniform-scale3-inverse");
        let inv = scale.inverse()?;
        Ok(Self::uniform_scale(inv))
    }

    /// Transforms all vectors in `rhs` using the same matrix.
    ///
    /// Matrix facts are computed once per call and reused for every vector.
    pub fn transform_vec3_batch(&self, rhs: &[Vector3]) -> Vec<Vector3> {
        trace_matrix!("method", "transform-vector-vec3-batch");
        BatchTransform3::new(self).transform_vector_batch(rhs)
    }

    /// Transforms one vector immediately.
    pub fn transform_vec3(&self, rhs: &Vector3) -> Vector3 {
        trace_matrix!("method", "transform-vector-vec3");
        self * rhs
    }

    /// Returns the matrix inverse using the adjugate and determinant.
    ///
    /// The ordinary path rejects a definite-zero determinant and otherwise
    /// propagates scalar arithmetic errors from the selected Real kernel.
    pub fn inverse(self) -> BlasResult<Self> {
        trace_matrix!("method", "matrix3-inverse");
        Ok(Self(invert_matrix3(self.0)?))
    }

    /// Returns the matrix inverse after rejecting unknown-zero determinants.
    pub fn inverse_checked(self) -> CheckedBlasResult<Self> {
        trace_matrix!("method", "matrix3-inverse-checked");
        Ok(Self(invert_matrix3_checked(self.0)?))
    }

    /// Returns the checked matrix inverse after attaching an abort signal.
    pub fn inverse_checked_with_abort(self, signal: &AbortSignal) -> CheckedBlasResult<Self> {
        trace_matrix!("method", "matrix3-inverse-checked-with-abort");
        Ok(Self(invert_matrix3_checked_with_abort(self.0, signal)?))
    }

    /// Returns the determinant.
    pub fn determinant(&self) -> Real {
        trace_matrix!("method", "matrix3-determinant");
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
fn signed_axis4_scalar(axis: SignedAxis4) -> Real {
    if axis.is_negative() {
        -Real::one()
    } else {
        Real::one()
    }
}

#[inline]
fn signed_axis4_apply(value: Real, axis: SignedAxis4) -> Real {
    if axis.is_negative() { -value } else { value }
}

impl Matrix4 {
    /// Constructs a matrix from row-major entries.
    pub fn from_row_major(values: [Real; 16]) -> Self {
        let [
            m00,
            m01,
            m02,
            m03,
            m10,
            m11,
            m12,
            m13,
            m20,
            m21,
            m22,
            m23,
            m30,
            m31,
            m32,
            m33,
        ] = values;
        Self::new([
            [m00, m01, m02, m03],
            [m10, m11, m12, m13],
            [m20, m21, m22, m23],
            [m30, m31, m32, m33],
        ])
    }

    /// Constructs a 4x4 matrix from a row-major slice of exactly 16 entries.
    pub fn from_row_slice(values: &[Real]) -> Option<Self> {
        if values.len() != 16 {
            return None;
        }
        let values = std::array::from_fn(|index| values[index].clone());
        Some(Self::from_row_major(values))
    }

    /// Returns exact-rational representation facts for all matrix entries.
    ///
    /// Transform stacks often preserve a common rational grid. Carrying this
    /// coarse summary lets higher-level code prepare exact kernels without
    /// re-querying every scalar entry, while `hyperreal` remains responsible
    /// for denominator storage and reduction.
    pub fn exact_facts(&self) -> RealExactSetFacts {
        trace_matrix!("query", "matrix4-exact-facts");
        matrix4_facts(&self.0).public.exact
    }

    /// Returns structural and exact-rational facts for this matrix.
    ///
    /// The affine and homogeneous-direction flags are reported alongside entry
    /// masks so transform stacks can dispatch to translation, diagonal, sparse,
    /// or generic schedules from one retained summary.
    pub fn structural_facts(&self) -> Matrix4StructuralFacts {
        trace_matrix!("query", "matrix4-structural-facts");
        matrix4_facts(&self.0).public
    }

    /// Constructs a 4x4 affine translation matrix from known x/y/z offsets.
    pub fn affine_translation(translation: [Real; 3]) -> Self {
        trace_matrix!("constructor", "affine-translation");
        let [tx, ty, tz] = translation;
        Self([
            [Real::one(), Real::zero(), Real::zero(), tx],
            [Real::zero(), Real::one(), Real::zero(), ty],
            [Real::zero(), Real::zero(), Real::one(), tz],
            [Real::zero(), Real::zero(), Real::zero(), Real::one()],
        ])
    }

    /// Constructs an affine non-uniform scale matrix.
    pub fn affine_nonuniform_scale(scale: [Real; 3]) -> Self {
        let [sx, sy, sz] = scale;
        Self([
            [sx, Real::zero(), Real::zero(), Real::zero()],
            [Real::zero(), sy, Real::zero(), Real::zero()],
            [Real::zero(), Real::zero(), sz, Real::zero()],
            [Real::zero(), Real::zero(), Real::zero(), Real::one()],
        ])
    }

    /// Constructs an affine x-axis rotation matrix.
    pub fn rotation_x(angle: Real) -> Self {
        let sin = angle.clone().sin();
        let cos = angle.cos();
        Self::from_row_major([
            Real::one(),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            cos.clone(),
            -sin.clone(),
            Real::zero(),
            Real::zero(),
            sin,
            cos,
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::one(),
        ])
    }

    /// Constructs an affine y-axis rotation matrix.
    pub fn rotation_y(angle: Real) -> Self {
        let sin = angle.clone().sin();
        let cos = angle.cos();
        Self::from_row_major([
            cos.clone(),
            Real::zero(),
            sin.clone(),
            Real::zero(),
            Real::zero(),
            Real::one(),
            Real::zero(),
            Real::zero(),
            -sin,
            Real::zero(),
            cos,
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::one(),
        ])
    }

    /// Constructs an affine z-axis rotation matrix.
    pub fn rotation_z(angle: Real) -> Self {
        let sin = angle.clone().sin();
        let cos = angle.cos();
        Self::from_row_major([
            cos.clone(),
            -sin.clone(),
            Real::zero(),
            Real::zero(),
            sin,
            cos,
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::one(),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::one(),
        ])
    }

    /// Constructs an affine rotation matrix from a checked axis and angle.
    pub fn rotation_axis_angle(axis: &Vector3, angle: Real) -> CheckedBlasResult<Self> {
        // Preserve the named coordinate-axis construction when the axis is
        // exactly sparse. Besides avoiding an unnecessary normalization, this
        // retains the structural zeros and the shared sine/cosine objects that
        // exact downstream predicates use as rotation certificates.
        for coordinate in 0..3 {
            if (0..3)
                .filter(|&candidate| candidate != coordinate)
                .all(|candidate| axis.0[candidate].definitely_zero())
            {
                let signed_angle = match axis.0[coordinate].refine_sign_until(128) {
                    Some(RealSign::Positive) => angle,
                    Some(RealSign::Negative) => -angle,
                    Some(RealSign::Zero) | None => break,
                };
                return Ok(match coordinate {
                    0 => Self::rotation_x(signed_angle),
                    1 => Self::rotation_y(signed_angle),
                    2 => Self::rotation_z(signed_angle),
                    _ => unreachable!(),
                });
            }
        }
        let axis = axis.normalize_checked()?;
        let sin = angle.clone().sin();
        let cos = angle.cos();
        let one_minus_cos = Real::one() - cos.clone();
        let [x, y, z] = axis.0;
        let zero = Real::zero();
        let one = Real::one();
        Ok(Self::from_row_major([
            cos.clone() + x.clone() * x.clone() * one_minus_cos.clone(),
            x.clone() * y.clone() * one_minus_cos.clone() - z.clone() * sin.clone(),
            x.clone() * z.clone() * one_minus_cos.clone() + y.clone() * sin.clone(),
            zero.clone(),
            y.clone() * x.clone() * one_minus_cos.clone() + z.clone() * sin.clone(),
            cos.clone() + y.clone() * y.clone() * one_minus_cos.clone(),
            y.clone() * z.clone() * one_minus_cos.clone() - x.clone() * sin.clone(),
            zero.clone(),
            z.clone() * x.clone() * one_minus_cos.clone() - y.clone() * sin.clone(),
            z.clone() * y.clone() * one_minus_cos.clone() + x.clone() * sin,
            cos + z.clone() * z * one_minus_cos,
            zero.clone(),
            zero.clone(),
            zero.clone(),
            zero,
            one,
        ]))
    }

    /// Constructs an affine rotation matrix that maps `from` onto `to`.
    ///
    /// Both inputs are normalized with structural zero checks. A pair whose
    /// antipodal relation bounded refinement proves receives a half turn
    /// about a fixed seed axis; every other pair receives the minimal
    /// rotation of [`Self::rotation_between_unit_vectors`], including its
    /// typed failure when `1 + from·to` cannot be certified nonzero.
    pub fn rotation_between_vectors(from: &Vector3, to: &Vector3) -> CheckedBlasResult<Self> {
        let from = from.normalize_checked()?;
        let to = to.normalize_checked()?;
        let dot = from.dot(&to);

        if let Some(RealSign::Zero) = (dot.clone() - Real::one()).refine_sign_until(128) {
            return Ok(Self::identity());
        }

        if let Some(RealSign::Zero) = (dot.clone() + Real::one()).refine_sign_until(128) {
            let threshold = (Real::from(9_u8) / Real::from(10_u8))?;
            let seed = if matches!(
                (from.0[0].clone().abs() - threshold).refine_sign_until(128),
                Some(RealSign::Negative)
            ) {
                Vector3::x()
            } else {
                Vector3::y()
            };
            return Self::rotation_axis_angle(&from.unit_cross_checked(&seed)?, Real::pi());
        }

        Self::minimal_rotation_between_unit_vectors(&from, &to, dot)
    }

    /// Constructs the minimal rotation that maps caller-certified unit vector
    /// `from` onto unit vector `to`.
    ///
    /// The inputs are not normalized or validated as unit vectors. When the
    /// cross product has a structural nonzero certificate, the axis-angle
    /// construction keeps its sparse-axis and shared sine/cosine structure.
    /// Otherwise the same rotation is built algebraically as
    /// `c I + [k]x + k kᵀ / (1 + c)`, with `c = from·to` and
    /// `k = from × to`. That form needs no axis normalization, inverse cosine
    /// or trigonometric evaluation; its only divisor is `1 + c`, which is
    /// nonzero exactly when the vectors are not antipodal.
    ///
    /// Antipodal inputs have no minimal rotation and return
    /// [`Problem::DivideByZero`]. A divisor whose nonzero sign cannot be
    /// certified returns the scalar layer's typed problem, such as
    /// [`Problem::UnknownZero`]; it is never treated as nonzero.
    pub fn rotation_between_unit_vectors(from: &Vector3, to: &Vector3) -> CheckedBlasResult<Self> {
        trace_matrix!("constructor", "rotation-between-unit-vectors");
        let dot = from.dot(to);
        if let Some(RealSign::Zero) = (dot.clone() - Real::one()).refine_sign_until(128) {
            return Ok(Self::identity());
        }
        Self::minimal_rotation_between_unit_vectors(from, to, dot)
    }

    /// Shared tail of the vector-alignment constructors after the identity
    /// shortcut; `dot` is `from·to`.
    fn minimal_rotation_between_unit_vectors(
        from: &Vector3,
        to: &Vector3,
        dot: Real,
    ) -> CheckedBlasResult<Self> {
        let cross = from.cross(to);
        match cross
            .normalize_checked()
            .and_then(|axis| Ok((axis, from.angle_to(to)?)))
        {
            Ok((axis, angle)) => return Self::rotation_axis_angle(&axis, angle),
            // Missing structural evidence for the axis, or a structurally
            // parallel pair, is decided by the algebraic form below.
            Err(Problem::UnknownZero | Problem::DivideByZero) => {}
            Err(problem) => return Err(problem),
        }
        trace_matrix!("rotation-between-unit-vectors", "algebraic-rodrigues");
        let scale = (Real::one() + dot.clone()).inverse()?;
        let [x, y, z] = cross.0;
        let scaled = |left: &Real, right: &Real| left * right * &scale;
        let zero = Real::zero();
        Ok(Self::from_row_major([
            dot.clone() + scaled(&x, &x),
            scaled(&x, &y) - z.clone(),
            scaled(&x, &z) + y.clone(),
            zero.clone(),
            scaled(&y, &x) + z.clone(),
            dot.clone() + scaled(&y, &y),
            scaled(&y, &z) - x.clone(),
            zero.clone(),
            scaled(&z, &x) - y.clone(),
            scaled(&z, &y) + x.clone(),
            dot + scaled(&z, &z),
            zero.clone(),
            zero.clone(),
            zero.clone(),
            zero,
            Real::one(),
        ]))
    }

    /// Constructs the inverse of a caller-certified affine translation.
    ///
    /// Translation inverse is exact negation of the offset vector. Keeping this
    /// as a known-object API avoids the generic affine fact scan and does not
    /// enter determinant/cofactor arithmetic.
    pub fn affine_translation_inverse(translation: [Real; 3]) -> Self {
        trace_matrix!("constructor", "affine-translation-inverse");
        let [tx, ty, tz] = translation;
        Self::affine_translation([-tx, -ty, -tz])
    }

    /// Right-divides this matrix by a caller-certified affine translation.
    ///
    /// For a translation `T(t)`, `A / T(t) = A * T(-t)`, so only the
    /// homogeneous translation column changes. This exposes the same
    /// object-level fast path used by generic affine dispatch without making
    /// callers rediscover translation structure numerically.
    pub fn div_affine_translation(self, translation: [Real; 3]) -> Self {
        trace_matrix!("method", "div-affine-translation4");
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
    pub fn affine_orthonormal(linear: [[Real; 3]; 3], translation: [Real; 3]) -> Self {
        trace_matrix!("constructor", "affine-orthonormal");
        let [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]] = linear;
        let [tx, ty, tz] = translation;
        Self([
            [r00, r01, r02, tx],
            [r10, r11, r12, ty],
            [r20, r21, r22, tz],
            [Real::zero(), Real::zero(), Real::zero(), Real::one()],
        ])
    }

    /// Constructs the inverse of a caller-certified affine orthonormal transform.
    ///
    /// For `M = [R t; 0 1]` with orthonormal `R`, `M^-1 = [R^T -R^T t; 0 1]`.
    /// This bypasses generic affine inversion and avoids determinant/cofactor
    /// arithmetic for rigid transform stacks.
    pub fn affine_orthonormal_inverse(linear: [[Real; 3]; 3], translation: [Real; 3]) -> Self {
        trace_matrix!("constructor", "affine-orthonormal-inverse");
        let [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]] = linear;
        let [tx, ty, tz] = translation;
        let it0 = Real::zero() - affine_translation_dot3([&r00, &r10, &r20], [&tx, &ty, &tz]);
        let it1 = Real::zero() - affine_translation_dot3([&r01, &r11, &r21], [&tx, &ty, &tz]);
        let it2 = Real::zero() - affine_translation_dot3([&r02, &r12, &r22], [&tx, &ty, &tz]);
        Self([
            [r00, r10, r20, it0],
            [r01, r11, r21, it1],
            [r02, r12, r22, it2],
            [Real::zero(), Real::zero(), Real::zero(), Real::one()],
        ])
    }

    /// Right-divides this matrix by a caller-certified affine orthonormal transform.
    pub fn div_affine_orthonormal(self, linear: [[Real; 3]; 3], translation: [Real; 3]) -> Self {
        trace_matrix!("method", "div-affine-orthonormal4");
        let [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]] = linear;
        let [tx, ty, tz] = translation;
        let it0 = Real::zero() - affine_translation_dot3([&r00, &r10, &r20], [&tx, &ty, &tz]);
        let it1 = Real::zero() - affine_translation_dot3([&r01, &r11, &r21], [&tx, &ty, &tz]);
        let it2 = Real::zero() - affine_translation_dot3([&r02, &r12, &r22], [&tx, &ty, &tz]);
        let inv_translation = [&it0, &it1, &it2];
        let result = self.0.map(|row| {
            let [a0, a1, a2, a3] = row;
            let c0 = Real::active_linear_combination3([&a0, &a1, &a2], [&r00, &r01, &r02]);
            let c1 = Real::active_linear_combination3([&a0, &a1, &a2], [&r10, &r11, &r12]);
            let c2 = Real::active_linear_combination3([&a0, &a1, &a2], [&r20, &r21, &r22]);
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
        trace_matrix!("constructor", "signed-permutation4");
        let [r0, r1, r2, r3] = rows;
        let mut matrix = [
            [Real::zero(), Real::zero(), Real::zero(), Real::zero()],
            [Real::zero(), Real::zero(), Real::zero(), Real::zero()],
            [Real::zero(), Real::zero(), Real::zero(), Real::zero()],
            [Real::zero(), Real::zero(), Real::zero(), Real::zero()],
        ];
        matrix[0][r0.index()] = signed_axis4_scalar(r0);
        matrix[1][r1.index()] = signed_axis4_scalar(r1);
        matrix[2][r2.index()] = signed_axis4_scalar(r2);
        matrix[3][r3.index()] = signed_axis4_scalar(r3);
        Self(matrix)
    }

    /// Constructs the inverse of a caller-certified signed-permutation matrix.
    pub fn signed_permutation_inverse(rows: [SignedAxis4; 4]) -> Self {
        trace_matrix!("constructor", "signed-permutation4-inverse");
        let [r0, r1, r2, r3] = rows;
        let mut matrix = [
            [Real::zero(), Real::zero(), Real::zero(), Real::zero()],
            [Real::zero(), Real::zero(), Real::zero(), Real::zero()],
            [Real::zero(), Real::zero(), Real::zero(), Real::zero()],
            [Real::zero(), Real::zero(), Real::zero(), Real::zero()],
        ];
        matrix[r0.index()][0] = signed_axis4_scalar(r0);
        matrix[r1.index()][1] = signed_axis4_scalar(r1);
        matrix[r2.index()][2] = signed_axis4_scalar(r2);
        matrix[r3.index()][3] = signed_axis4_scalar(r3);
        Self(matrix)
    }

    /// Right-divides this matrix by a caller-certified signed-permutation matrix.
    pub fn div_signed_permutation(self, rows: [SignedAxis4; 4]) -> Self {
        trace_matrix!("method", "div-signed-permutation4");
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
    pub fn transform_signed_permutation_vector(rows: [SignedAxis4; 4], rhs: &Vector4) -> Vector4 {
        trace_matrix!("method", "transform-signed-permutation4-vector");
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
        rhs: &[Vector4],
    ) -> Vec<Vector4> {
        trace_matrix!("method", "transform-signed-permutation4-batch");
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
    pub fn diagonal(diagonal: [Real; 4]) -> Self {
        trace_matrix!("constructor", "diagonal");
        let [d0, d1, d2, d3] = diagonal;
        Self([
            [d0, Real::zero(), Real::zero(), Real::zero()],
            [Real::zero(), d1, Real::zero(), Real::zero()],
            [Real::zero(), Real::zero(), d2, Real::zero()],
            [Real::zero(), Real::zero(), Real::zero(), d3],
        ])
    }

    /// Constructs the inverse of a known 4x4 diagonal matrix.
    ///
    /// Keep this as an explicit known-structure API rather than another hidden
    /// [`Matrix4::reciprocal`] branch. Prior diagonal/uniform-scale experiments
    /// showed that adding dynamic probes to the general inverse path made
    /// adjacent cases less deterministic. When callers retain the object-level
    /// fact that `D` is diagonal, the exact inverse is just four independent
    /// scalar reciprocals and certified off-diagonal zeros. This mirrors the
    /// recommendation to exploit geometric-object structure before arithmetic
    /// the exact object-structure policy and the diagonal solve
    /// treatment in Golub and Van Loan, *Matrix Computations*.
    pub fn diagonal_inverse(diagonal: [Real; 4]) -> BlasResult<Self> {
        trace_matrix!("constructor", "diagonal-inverse");
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
    /// cached multiplies with no determinant/cofactor work. This follows the
    /// object-level exact geometric computation guidance the exact object-structure policy and the diagonal solve specialization in
    /// Golub and Van Loan, *Matrix Computations*.
    pub fn div_diagonal(self, diagonal: [Real; 4]) -> BlasResult<Self> {
        trace_matrix!("method", "div-diagonal");
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
        trace_matrix!("method", "upper-triangular4-inverse");
        Ok(Self(invert_matrix4_by_upper_triangular(&self.0)?))
    }

    /// Checked variant of [`Matrix4::upper_triangular_inverse`].
    pub fn upper_triangular_inverse_checked(self) -> CheckedBlasResult<Self> {
        trace_matrix!("method", "upper-triangular4-inverse-checked");
        Ok(Self(invert_matrix4_by_upper_triangular_checked(&self.0)?))
    }

    /// Inverts a caller-certified lower-triangular 4x4 matrix.
    pub fn lower_triangular_inverse(self) -> BlasResult<Self> {
        trace_matrix!("method", "lower-triangular4-inverse");
        Ok(Self(invert_matrix4_by_lower_triangular(&self.0)?))
    }

    /// Checked variant of [`Matrix4::lower_triangular_inverse`].
    pub fn lower_triangular_inverse_checked(self) -> CheckedBlasResult<Self> {
        trace_matrix!("method", "lower-triangular4-inverse-checked");
        Ok(Self(invert_matrix4_by_lower_triangular_checked(&self.0)?))
    }

    /// Right-divides by a caller-certified upper-triangular 4x4 matrix.
    pub fn div_upper_triangular(self, divisor: Self) -> BlasResult<Self> {
        trace_matrix!("method", "div-upper-triangular4");
        Ok(Self(divide_matrix4_by_upper_triangular(
            self.0, &divisor.0,
        )?))
    }

    /// Checked variant of [`Matrix4::div_upper_triangular`].
    pub fn div_upper_triangular_checked(self, divisor: Self) -> CheckedBlasResult<Self> {
        trace_matrix!("method", "div-upper-triangular4-checked");
        Ok(Self(divide_matrix4_by_upper_triangular_checked(
            self.0, &divisor.0,
        )?))
    }

    /// Right-divides by a caller-certified lower-triangular 4x4 matrix.
    pub fn div_lower_triangular(self, divisor: Self) -> BlasResult<Self> {
        trace_matrix!("method", "div-lower-triangular4");
        Ok(Self(divide_matrix4_by_lower_triangular(
            self.0, &divisor.0,
        )?))
    }

    /// Checked variant of [`Matrix4::div_lower_triangular`].
    pub fn div_lower_triangular_checked(self, divisor: Self) -> CheckedBlasResult<Self> {
        trace_matrix!("method", "div-lower-triangular4-checked");
        Ok(Self(divide_matrix4_by_lower_triangular_checked(
            self.0, &divisor.0,
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
    /// Retain object facts, defer expensive algebra, and reduce to a cheaper
    /// exact kernel when the divisor structure is known.
    pub fn div_diagonal_vector(&self, diagonal: [Real; 4], rhs: &Vector4) -> BlasResult<Vector4> {
        trace_matrix!("method", "div-diagonal4-vector");
        let [d0, d1, d2, d3] = diagonal;
        let vector_facts = rhs.geometric_facts();
        if matches!(vector_facts.homogeneous, Vector4HomogeneousKind::Direction) {
            // Direction vectors are guaranteed `w == 0`; avoiding `d3` work keeps
            // this branch aligned with specialized direction kernels and avoids
            // unnecessary reciprocal work when only three linear scales are ever used.
            // This follows the geometric-object split between points and
            // directions in the exact object-structure policy.
            let (inv0, inv1, inv2) = if d0 == d1 && d0 == d2 {
                trace_matrix!("helper", "div-diagonal4-vector-direction-uniform-scale");
                let inv = d0.inverse()?;
                (inv.clone(), inv.clone(), inv)
            } else {
                (d0.inverse()?, d1.inverse()?, d2.inverse()?)
            };
            let rhs_div = [
                rhs.0[0].clone().mul_cached(&inv0),
                rhs.0[1].clone().mul_cached(&inv1),
                rhs.0[2].clone().mul_cached(&inv2),
                Real::zero(),
            ];
            return Ok(Vector4(transform_vector4_rhs_direction_ref_cached(
                &self.0,
                &rhs_div,
                matrix4_direction_linear_is_diagonal(&self.0),
            )));
        }

        let linear_uniform_scale = d0 == d1 && d0 == d2;
        let (inv0, inv1, inv2) = if linear_uniform_scale {
            trace_matrix!("helper", "div-diagonal4-vector-linear-uniform-scale");
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
            Real::one()
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
                // special-point row scheduling aligned with projective geometry.
                let rhs_div = [rhs_div_x, rhs_div_y, rhs_div_z, rhs_div_3_scale];
                Ok(Vector4(
                    transform_vector4_rhs_point_with_scaled_w_ref_cached(
                        &self.0,
                        &rhs_div,
                        &translation_is_zero,
                        all_translation_zero,
                        all_translation_nonzero,
                        inv3_is_one,
                        &inv3,
                    ),
                ))
            }
            Vector4HomogeneousKind::Direction => {
                let rhs_div = [rhs_div_x, rhs_div_y, rhs_div_z, Real::zero()];
                let direction_is_diagonal = matrix4_direction_linear_is_diagonal(&self.0);
                Ok(Vector4(transform_vector4_rhs_direction_ref_cached(
                    &self.0,
                    &rhs_div,
                    direction_is_diagonal,
                )))
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
                Ok(Vector4(transform_vector4_rhs_ref_with_facts(
                    &self.0,
                    &rhs_div,
                    &translation_is_zero,
                    all_translation_zero,
                    all_translation_nonzero,
                    matrix_facts.direction_linear_is_diagonal,
                    Some(matrix_facts),
                    vector_facts,
                )))
            }
        }
    }

    /// Divides a matrix by a known 4x4 diagonal divisor and applies the result
    /// to a direction vector.
    ///
    /// Direction vectors have `w == 0`, so the fourth diagonal divisor entry is
    /// provably irrelevant. Avoiding its inversion and W-scaling is an exact
    /// structural optimization: preserve and exploit projective-direction facts
    /// so arithmetic follows the minimal necessary path.
    #[inline]
    pub fn div_diagonal_direction_vector(
        &self,
        diagonal: [Real; 4],
        rhs: &Vector4,
    ) -> BlasResult<Vector4> {
        trace_matrix!("method", "div-diagonal4-vector-direction-only");
        let [d0, d1, d2, _d3] = diagonal;
        let inv0 = d0.inverse()?;
        let inv1 = d1.inverse()?;
        let inv2 = d2.inverse()?;
        let rhs_div = [
            rhs.0[0].clone().mul_cached(&inv0),
            rhs.0[1].clone().mul_cached(&inv1),
            rhs.0[2].clone().mul_cached(&inv2),
            Real::zero(),
        ];
        Ok(Vector4(transform_vector4_rhs_direction_ref_cached(
            &self.0,
            &rhs_div,
            matrix4_direction_linear_is_diagonal(&self.0),
        )))
    }

    /// Constructs a scalar multiple of the 4x4 identity matrix.
    pub fn uniform_scale(scale: Real) -> Self {
        trace_matrix!("constructor", "uniform-scale");
        Self([
            [scale.clone(), Real::zero(), Real::zero(), Real::zero()],
            [Real::zero(), scale.clone(), Real::zero(), Real::zero()],
            [Real::zero(), Real::zero(), scale.clone(), Real::zero()],
            [Real::zero(), Real::zero(), Real::zero(), scale],
        ])
    }

    /// Constructs the inverse of a known scalar multiple of the 4x4 identity.
    ///
    /// This is intentionally an opt-in API rather than an automatic
    /// `Matrix4::reciprocal` dispatch branch: prior targeted benches showed
    /// that dynamic equality checks for uniform scale made adjacent diagonal
    /// inverse paths less flat. When the caller already owns the object-level
    /// fact `A = sI`, one scalar inverse is sufficient and reusing it preserves
    /// hyperreal's exact/symbolic node cache. This is an explicit object-layer
    /// specialization of the diagonal solve.
    pub fn uniform_scale_inverse(scale: Real) -> BlasResult<Self> {
        trace_matrix!("constructor", "uniform-scale-inverse");
        let inv = scale.inverse()?;
        Ok(Self::uniform_scale(inv))
    }

    /// Transforms a point vector assuming `rhs[3] == 1`, which keeps a single
    /// guaranteed affine helper shape and avoids probing point/direction
    /// predicates.
    pub fn transform_vec4_point(&self, rhs: &Vector4) -> Vector4 {
        if matrix4_affine_linear_is_diagonal(&self.0) {
            return Vector4(
                transform_vector4_rhs_point_affine_linear_diagonal_ref_cached(&self.0, &rhs.0),
            );
        }
        let facts = matrix4_facts(&self.0);
        // Reuse precomputed structural facts for the immediate fallback path
        // instead of recomputing them. This keeps object-level structure inside
        // the geometric package.
        BatchTransform4::new_with_facts(self, facts).transform_point_vector(rhs)
    }

    /// Transforms a 3D point using homogeneous coordinates.
    pub fn transform_point3(&self, point: &Point3) -> BlasResult<Point3> {
        let transformed = self.transform_vec4_point(&Vector4::new([
            point.x.clone(),
            point.y.clone(),
            point.z.clone(),
            Real::one(),
        ]));
        point3_from_homogeneous(transformed)
    }

    /// Transforms 3D points in one immediate batch.
    ///
    /// Matrix facts are computed once for the operation; no caller-managed
    /// transform state escapes the call.
    pub fn transform_point3_batch(&self, points: &[Point3]) -> BlasResult<Vec<Point3>> {
        let homogeneous = points
            .iter()
            .map(|point| {
                Vector4::new([
                    point.x.clone(),
                    point.y.clone(),
                    point.z.clone(),
                    Real::one(),
                ])
            })
            .collect::<Vec<_>>();
        self.transform_vec4_point_batch(&homogeneous)
            .into_iter()
            .map(point3_from_homogeneous)
            .collect()
    }

    /// Transforms a 3D direction using homogeneous coordinates.
    pub fn transform_direction3(&self, direction: &Vector3) -> Vector3 {
        let transformed = self.transform_vec4_direction(&Vector4::new([
            direction.0[0].clone(),
            direction.0[1].clone(),
            direction.0[2].clone(),
            Real::zero(),
        ]));
        let [x, y, z, _w] = transformed.0;
        Vector3::new([x, y, z])
    }

    /// Transforms 3D directions in one immediate batch.
    pub fn transform_direction3_batch(&self, directions: &[Vector3]) -> Vec<Vector3> {
        let homogeneous = directions
            .iter()
            .map(|direction| {
                Vector4::new([
                    direction.0[0].clone(),
                    direction.0[1].clone(),
                    direction.0[2].clone(),
                    Real::zero(),
                ])
            })
            .collect::<Vec<_>>();
        self.transform_vec4_direction_batch(&homogeneous)
            .into_iter()
            .map(|vector| {
                let [x, y, z, _w] = vector.0;
                Vector3::new([x, y, z])
            })
            .collect()
    }

    /// Transforms a direction vector assuming `rhs[3] == 0`, keeping the fast
    /// 3-term affine-less form.
    pub fn transform_vec4_direction(&self, rhs: &Vector4) -> Vector4 {
        match matrix4_direction_linear_kind(&self.0) {
            Matrix4DirectionLinearKind::Identity => {
                trace_matrix!("method", "transform-vector-vec4-direction-linear-identity");
                rhs.clone()
            }
            Matrix4DirectionLinearKind::Diagonal => Vector4(
                transform_vector4_rhs_direction_ref_cached(&self.0, &rhs.0, true),
            ),
            Matrix4DirectionLinearKind::General => Vector4(
                transform_vector4_rhs_direction_ref_cached(&self.0, &rhs.0, false),
            ),
        }
    }

    /// Transforms a batch of homogeneous directions, assuming every input has `w = 0`.
    ///
    /// Use this when geometry/object-level facts already classify the whole
    /// batch as directions. It avoids the generic batch classifier and keeps
    /// the translation column out of the arithmetic schedule.
    pub fn transform_vec4_direction_batch(&self, rhs: &[Vector4]) -> Vec<Vector4> {
        trace_matrix!("method", "transform-vector-vec4-direction-batch");
        // Direction batches do not need the full Matrix4 fact
        // scan. The only matrix fact required for the fastest direction kernel
        // is whether the 3x3 linear block is diagonal while the bottom spatial
        // row is zero. This keeps the immediate API from paying for unrelated
        // geometric metadata.
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
    pub fn transform_vec4_point_batch(&self, rhs: &[Vector4]) -> Vec<Vector4> {
        trace_matrix!("method", "transform-vector-vec4-point-batch");
        // Unlike direction batches, point batches did not benefit from a
        // thinner one-shot public route: the public affine-diagonal helper was
        // trace-clean but did not show a stable exact-Real kernel win. If that
        // changes, prefer a Real kernel-gated split keyed by retained structural
        // facts.
        BatchTransform4::new(self).transform_point_batch(rhs)
    }

    /// Transforms all vectors in `rhs` using the same matrix.
    ///
    /// Per-row homogeneous-coordinate facts are computed once per call and
    /// reused for every vector in the batch.
    pub fn transform_vec4_batch(&self, rhs: &[Vector4]) -> Vec<Vector4> {
        trace_matrix!("method", "transform-vector-vec4-batch");
        BatchTransform4::new(self).transform_vector_batch(rhs)
    }

    /// Transforms one homogeneous vector immediately.
    pub fn transform_vec4(&self, rhs: &Vector4) -> Vector4 {
        trace_matrix!("method", "transform-vector-vec4");
        self * rhs
    }

    /// Returns the matrix inverse using a fixed-size cofactor expansion.
    ///
    /// The ordinary path rejects a definite-zero determinant and propagates
    /// scalar arithmetic errors from the selected Real kernel.
    pub fn inverse(self) -> BlasResult<Self> {
        trace_matrix!("method", "matrix4-inverse");
        Ok(Self(invert_matrix4(self.0)?))
    }

    /// Returns the matrix inverse after rejecting unknown-zero determinants.
    pub fn inverse_checked(self) -> CheckedBlasResult<Self> {
        trace_matrix!("method", "matrix4-inverse-checked");
        Ok(Self(invert_matrix4_checked(self.0)?))
    }

    /// Returns the checked matrix inverse after attaching an abort signal.
    pub fn inverse_checked_with_abort(self, signal: &AbortSignal) -> CheckedBlasResult<Self> {
        trace_matrix!("method", "matrix4-inverse-checked-with-abort");
        Ok(Self(invert_matrix4_checked_with_abort(self.0, signal)?))
    }

    /// Returns the determinant.
    pub fn determinant(&self) -> Real {
        trace_matrix!("method", "matrix4-determinant");
        determinant4(&self.0)
    }
}

#[cfg(test)]
mod fact_layout_tests {
    use super::{Matrix3Facts, Matrix3StructuralFacts, Matrix4Facts, Matrix4StructuralFacts};
    use std::mem::size_of;

    #[test]
    fn immediate_matrix_facts_retain_one_exact_set_summary() {
        // Private dispatch packets add only compact booleans and masks to the
        // public structural packet. A second RealExactSetFacts is 96 bytes on
        // the validation target, so this budget catches accidental duplicate
        // retention while leaving room for small scheduling facts.
        const INTERNAL_SCHEDULING_FACT_BUDGET: usize = 32;

        assert!(
            size_of::<Matrix3Facts>()
                <= size_of::<Matrix3StructuralFacts>() + INTERNAL_SCHEDULING_FACT_BUDGET
        );
        assert!(
            size_of::<Matrix4Facts>()
                <= size_of::<Matrix4StructuralFacts>() + INTERNAL_SCHEDULING_FACT_BUDGET
        );
    }
}
