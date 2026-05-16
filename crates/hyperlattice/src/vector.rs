//! Fixed-size vectors over [`Real`](crate::Real).

use std::array::from_fn;
use std::fmt;
use std::ops::{Add, Div, Index, IndexMut, Mul, Neg, Sub};
use std::sync::atomic::Ordering;

use crate::scalar::{clone_with_abort, reject_definite_zero, require_known_nonzero, with_abort};
use crate::{
    AbortSignal, BlasResult, CheckedBlasResult, ExactRealSetFacts, Problem, Real, RealKernelExt,
    RealSymbolicDependencyMask, RealZeroOneMinusOneStatus, ZeroStatus,
};

/// Coordinate axis in a 2D vector.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Axis2 {
    /// The x axis.
    X,
    /// The y axis.
    Y,
}

/// Signed 2D basis axis certified by vector structural facts.
///
/// This is a compact object-level certificate for vectors whose coordinates
/// are exactly one signed unit and one exact zero. It lets sparse linear
/// algebra, curve, and predicate preparation code choose signed-axis schedules
/// without repeating scalar identity probes. The separation between this
/// structural fact and geometric decisions follows Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SignedAxis2 {
    /// Positive X axis.
    PosX,
    /// Negative X axis.
    NegX,
    /// Positive Y axis.
    PosY,
    /// Negative Y axis.
    NegY,
}

impl SignedAxis2 {
    /// Returns the unsigned coordinate axis.
    pub const fn axis(self) -> Axis2 {
        match self {
            Self::PosX | Self::NegX => Axis2::X,
            Self::PosY | Self::NegY => Axis2::Y,
        }
    }

    /// Returns whether this signed axis has negative orientation.
    pub const fn is_negative(self) -> bool {
        matches!(self, Self::NegX | Self::NegY)
    }

    /// Returns `1` for positive axes and `-1` for negative axes.
    pub fn sign_real(self) -> Real {
        if self.is_negative() {
            -Real::one()
        } else {
            Real::one()
        }
    }
}

impl Axis2 {
    /// Returns the component index for this axis.
    pub const fn index(self) -> usize {
        match self {
            Self::X => 0,
            Self::Y => 1,
        }
    }

    /// Returns a one-bit mask for this axis.
    pub const fn bit(self) -> u8 {
        1 << self.index()
    }
}

/// Borrowed view of vector coordinates that share an exact rational scale.
///
/// The view keeps references to the original [`Real`] coordinates and exposes
/// only conservative object facts: exact-set schedule eligibility,
/// denominator kind, coarse rational storage class, plus zero/nonzero masks. It
/// intentionally does not expose numerators or the common denominator;
/// `hyperreal::Rational` remains responsible for scalar storage and reduction.
/// This is the first borrowed common-scale object in the vector layer,
/// following Yap's guidance that exact geometric computation should preserve
/// rational object structure before scalar expansion; see Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug)]
pub struct VectorSharedScaleView<'a, const N: usize> {
    components: [&'a Real; N],
    /// Exact-rational representation facts for all borrowed coordinates.
    pub exact: ExactRealSetFacts,
    /// Bit mask of coordinates known to be exactly zero.
    pub known_zero_mask: u128,
    /// Bit mask of coordinates known to be nonzero.
    pub known_nonzero_mask: u128,
    /// Bit mask of coordinates whose zero status is unknown.
    pub unknown_zero_mask: u128,
}

impl<'a, const N: usize> VectorSharedScaleView<'a, N> {
    /// Attempts to build a borrowed shared-scale view from coordinate refs.
    ///
    /// This returns `None` unless every coordinate is an exact rational and all
    /// reduced denominators match. Empty coordinate sets are rejected because no
    /// concrete exact shared-scale schedule can be selected from them.
    pub fn from_components(components: [&'a Real; N]) -> Option<Self> {
        crate::trace_dispatch!(
            "hyperlattice_vector",
            "query",
            "shared-scale-view-from-components"
        );
        let exact = crate::kernels::exact_real_set_facts(components.iter().copied());
        if !exact.has_shared_denominator_schedule() {
            return None;
        }
        let (known_zero_mask, known_nonzero_mask, unknown_zero_mask) =
            vector_zero_status_masks(components);
        Some(Self {
            components,
            exact,
            known_zero_mask,
            known_nonzero_mask,
            unknown_zero_mask,
        })
    }

    /// Returns the borrowed coordinates.
    pub fn components(self) -> [&'a Real; N] {
        self.components
    }

    /// Returns the number of coordinates in this view.
    pub const fn len(self) -> usize {
        N
    }

    /// Returns whether the view has no coordinates.
    pub const fn is_empty(self) -> bool {
        N == 0
    }

    /// Returns true when every coordinate is known to be exactly zero.
    pub fn is_known_zero(self) -> bool {
        self.known_zero_mask == vector_mask::<N>()
    }

    /// Returns true when every coordinate is known to be nonzero.
    pub fn is_known_dense(self) -> bool {
        self.known_nonzero_mask == vector_mask::<N>()
    }

    /// Counts coordinates known to be exactly zero.
    ///
    /// Callers should prefer this helper over reading the bit-mask layout
    /// directly when choosing sparse exact kernels. This keeps mask encoding
    /// local to `hyperlattice` while preserving the object-level structural
    /// facts that Yap recommends using before scalar expansion; see Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    /// (1997).
    pub fn known_zero_count(self) -> u32 {
        self.known_zero_mask.count_ones()
    }

    /// Counts coordinates known to be nonzero.
    pub fn known_nonzero_count(self) -> u32 {
        self.known_nonzero_mask.count_ones()
    }

    /// Counts coordinates whose zero status is not structurally certified.
    pub fn unknown_zero_count(self) -> u32 {
        self.unknown_zero_mask.count_ones()
    }

    /// Returns the dot product with another shared-scale view.
    ///
    /// Both views certify that every lane is already an exact rational, so this
    /// method jumps directly to the known-exact fused product-sum route instead
    /// of asking every factor to prove exactness again. The reducer itself
    /// remains in `hyperreal`, preserving the scalar abstraction boundary and
    /// Yap's object-structure-first exact-computation model; see Yap, "Towards
    /// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997).
    /// The fused reduction follows the delayed-normalization idea used by
    /// Bareiss, "Sylvester's Identity and Multistep Integer-Preserving Gaussian
    /// Elimination," *Mathematics of Computation* 22.103 (1968).
    pub fn dot(self, rhs: Self) -> Real {
        crate::trace_dispatch!(
            "hyperlattice_vector",
            "method",
            "shared-scale-view-dot-known-exact"
        );
        Real::exact_rational_signed_product_sum_known_exact(
            [true; N],
            from_fn(|index| [self.components[index], rhs.components[index]]),
        )
    }

    /// Returns the squared Euclidean norm using the known-exact dot route.
    ///
    /// This is an algebraic value, not a geometric predicate. Callers that use
    /// the sign of the result for topology must still go through `hyperlimit`.
    pub fn squared_norm(self) -> Real {
        self.dot(self)
    }
}

impl<'a> VectorSharedScaleView<'a, 2> {
    /// Returns the 2D exterior product `self.x * rhs.y - self.y * rhs.x`.
    ///
    /// This is the algebraic determinant behind planar orientation, not an
    /// orientation predicate. The method consumes the retained exact-rational
    /// certificate and dispatches directly to the known-exact product-sum
    /// reducer, preserving the object-level common-scale information that Yap
    /// recommends retaining before scalar expansion. See Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997). The
    /// fused signed sum follows Bareiss-style delayed normalization; see
    /// Bareiss, "Sylvester's Identity and Multistep Integer-Preserving Gaussian
    /// Elimination," *Mathematics of Computation* 22.103 (1968).
    pub fn wedge(self, rhs: Self) -> Real {
        crate::trace_dispatch!(
            "hyperlattice_vector",
            "method",
            "shared-scale-view-wedge-known-exact"
        );
        Real::exact_rational_signed_product_sum_known_exact(
            [true, false],
            [
                [self.components[0], rhs.components[1]],
                [self.components[1], rhs.components[0]],
            ],
        )
    }
}

impl<'a> VectorSharedScaleView<'a, 3> {
    /// Returns the 3D cross product as an ordinary [`Vector3`].
    ///
    /// Each component is a two-term determinant, so this method uses the
    /// retained exact-rational certificate to call the known-exact fused
    /// product-sum reducer for each lane. The result is not wrapped in
    /// [`SharedScaleVec`] because reduced output coordinates may lose a common
    /// reduced denominator after cancellation or zero components. This keeps
    /// the current abstraction honest while still following Yap's guidance to
    /// preserve object-level exact structure for the arithmetic package
    /// selection; see Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997). The short determinant reductions
    /// follow Bareiss-style delayed normalization; see Bareiss, "Sylvester's
    /// Identity and Multistep Integer-Preserving Gaussian Elimination,"
    /// *Mathematics of Computation* 22.103 (1968).
    pub fn cross(self, rhs: Self) -> Vector3 {
        crate::trace_dispatch!(
            "hyperlattice_vector",
            "method",
            "shared-scale-view-cross-known-exact"
        );
        let determinant = |a: &Real, b: &Real, c: &Real, d: &Real| {
            Real::exact_rational_signed_product_sum_known_exact([true, false], [[a, b], [c, d]])
        };
        Vector3::new([
            determinant(
                self.components[1],
                rhs.components[2],
                self.components[2],
                rhs.components[1],
            ),
            determinant(
                self.components[2],
                rhs.components[0],
                self.components[0],
                rhs.components[2],
            ),
            determinant(
                self.components[0],
                rhs.components[1],
                self.components[1],
                rhs.components[0],
            ),
        ])
    }
}

/// Owned vector coordinates certified to share one exact rational scale.
///
/// `SharedScaleVec` is the first owning common-scale carrier in
/// `hyperlattice`. It stores ordinary [`Real`] coordinates and cached
/// storage-free exact facts, rather than exposing rational numerators or the
/// common denominator. That keeps scalar representation and reduction in
/// `hyperreal`, while letting matrix, predicate, and triangulation code retain
/// Yap-style object structure across API boundaries. See Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Debug, PartialEq)]
pub struct SharedScaleVec<const N: usize> {
    components: [Real; N],
    /// Exact-rational representation facts for all owned coordinates.
    pub exact: ExactRealSetFacts,
}

impl<const N: usize> SharedScaleVec<N> {
    /// Attempts to construct an owned shared-scale vector.
    ///
    /// This returns `None` unless every coordinate is an exact rational and all
    /// reduced denominators match. The object is a semantic carrier for future
    /// common-denominator kernels; it deliberately does not expose the shared
    /// denominator.
    pub fn from_components(components: [Real; N]) -> Option<Self> {
        crate::trace_dispatch!(
            "hyperlattice_vector",
            "constructor",
            "shared-scale-vec-from-components"
        );
        let exact = crate::kernels::exact_real_set_facts(components.iter());
        if !exact.has_shared_denominator_schedule() {
            return None;
        }
        Some(Self { components, exact })
    }

    /// Returns the owned coordinates by reference.
    pub fn components(&self) -> &[Real; N] {
        &self.components
    }

    /// Consumes the object and returns the owned coordinates.
    pub fn into_components(self) -> [Real; N] {
        self.components
    }

    /// Returns a borrowed shared-scale view over the owned coordinates.
    ///
    /// The view recomputes conservative masks from the retained coordinates but
    /// preserves the same scalar abstraction boundary as direct vector views.
    pub fn as_view(&self) -> VectorSharedScaleView<'_, N> {
        VectorSharedScaleView::from_components(from_fn(|index| &self.components[index]))
            .expect("SharedScaleVec invariant guarantees shared denominator")
    }

    /// Returns the number of coordinates.
    pub const fn len(&self) -> usize {
        N
    }

    /// Returns whether this vector contains no coordinates.
    pub const fn is_empty(&self) -> bool {
        N == 0
    }

    /// Count coordinates known to be exactly zero.
    ///
    /// This forwards the borrowed view's mask-derived count without exposing
    /// the mask layout. Keeping count queries stable gives callers a cheap
    /// sparse-kernel dispatch signal while preserving the common-scale storage
    /// boundary described by Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997).
    pub fn known_zero_count(&self) -> u32 {
        self.as_view().known_zero_count()
    }

    /// Count coordinates known to be nonzero.
    pub fn known_nonzero_count(&self) -> u32 {
        self.as_view().known_nonzero_count()
    }

    /// Count coordinates whose zero status is unknown.
    pub fn unknown_zero_count(&self) -> u32 {
        self.as_view().unknown_zero_count()
    }

    /// Returns the dot product with another owned shared-scale vector.
    ///
    /// The owned vectors retain their common-scale certificate across
    /// lifetimes. This method consumes that certificate through borrowed views
    /// and uses the known-exact scalar product-sum path without exposing
    /// numerators or denominators.
    pub fn dot(&self, rhs: &Self) -> Real {
        self.as_view().dot(rhs.as_view())
    }

    /// Returns the squared Euclidean norm using the retained exact certificate.
    pub fn squared_norm(&self) -> Real {
        self.as_view().squared_norm()
    }
}

impl SharedScaleVec<2> {
    /// Returns the 2D exterior product with another owned shared-scale vector.
    ///
    /// The result is an exact algebraic scalar expression. Topological
    /// orientation decisions remain in `hyperlimit`; this method only preserves
    /// and consumes the common-scale arithmetic certificate.
    pub fn wedge(&self, rhs: &Self) -> Real {
        self.as_view().wedge(rhs.as_view())
    }
}

impl SharedScaleVec<3> {
    /// Returns the 3D cross product with another owned shared-scale vector.
    ///
    /// The result is an ordinary [`Vector3`] because reduction may erase the
    /// common-scale shape in some lanes. Use [`Vector3::into_shared_scale`] if
    /// the caller wants to recover that certificate opportunistically.
    pub fn cross(&self, rhs: &Self) -> Vector3 {
        self.as_view().cross(rhs.as_view())
    }
}

#[inline]
fn vector_mask<const N: usize>() -> u128 {
    debug_assert!(N <= u128::BITS as usize);
    if N == u128::BITS as usize {
        u128::MAX
    } else {
        (1_u128 << N) - 1
    }
}

#[inline]
fn vector_zero_status_masks<const N: usize>(components: [&Real; N]) -> (u128, u128, u128) {
    let mut known_zero_mask = 0_u128;
    let mut known_nonzero_mask = 0_u128;
    let mut unknown_zero_mask = 0_u128;
    for (index, component) in components.into_iter().enumerate() {
        let bit = 1_u128 << index;
        match component.zero_status() {
            ZeroStatus::Zero => known_zero_mask |= bit,
            ZeroStatus::NonZero => known_nonzero_mask |= bit,
            ZeroStatus::Unknown => unknown_zero_mask |= bit,
        }
    }
    (known_zero_mask, known_nonzero_mask, unknown_zero_mask)
}

#[inline]
fn vector_one_mask<const N: usize>(components: [&Real; N]) -> u128 {
    let mut mask = 0_u128;
    for (index, component) in components.into_iter().enumerate() {
        if component.definitely_one() {
            mask |= 1_u128 << index;
        }
    }
    mask
}

#[inline]
fn single_bit_index(mask: u128) -> Option<usize> {
    if mask.count_ones() == 1 {
        Some(mask.trailing_zeros() as usize)
    } else {
        None
    }
}

#[inline]
fn signed_axis2_from_components(values: &[Real; 2]) -> Option<SignedAxis2> {
    // The combined signed-unit query is scalar-owned, so the vector layer can
    // carry a signed-axis certificate without depending on rational storage.
    // This is Yap's object-fact rule in miniature: retain cheap shape before
    // expanding into algebra, but leave predicate decisions to `hyperlimit`.
    // See Yap, "Towards Exact Geometric Computation," Computational Geometry
    // 7.1-2 (1997).
    match (
        values[0].zero_one_or_minus_one(),
        values[1].zero_one_or_minus_one(),
    ) {
        (RealZeroOneMinusOneStatus::One, RealZeroOneMinusOneStatus::Zero) => {
            Some(SignedAxis2::PosX)
        }
        (RealZeroOneMinusOneStatus::MinusOne, RealZeroOneMinusOneStatus::Zero) => {
            Some(SignedAxis2::NegX)
        }
        (RealZeroOneMinusOneStatus::Zero, RealZeroOneMinusOneStatus::One) => {
            Some(SignedAxis2::PosY)
        }
        (RealZeroOneMinusOneStatus::Zero, RealZeroOneMinusOneStatus::MinusOne) => {
            Some(SignedAxis2::NegY)
        }
        _ => None,
    }
}

fn vector_symbolic_dependency_mask<const N: usize>(
    values: [&Real; N],
) -> RealSymbolicDependencyMask {
    values
        .into_iter()
        .fold(RealSymbolicDependencyMask::NONE, |mask, value| {
            mask.union(value.detailed_facts().symbolic.dependencies)
        })
}

/// Cheap structural facts known for a [`Vector2`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Vector2Facts {
    /// Zero status for `[x, y]` components.
    pub component_zero: [ZeroStatus; 2],
    /// Exact-rational representation facts for the coordinate set.
    ///
    /// This gives predicate, triangulation, and transform callers a stable way
    /// to select dyadic or shared-denominator exact kernels without peeking
    /// into `hyperreal::Rational` internals. It is structural metadata only;
    /// topological decisions still belong in `hyperlimit`.
    pub exact: ExactRealSetFacts,
    /// Union of scalar symbolic dependency families across all components.
    ///
    /// This summary lets transform, predicate-preparation, and future solver
    /// code schedule constant-family or opaque-expression paths without
    /// inspecting `Real` internals. It is deliberately a dependency-family
    /// certificate, not a CAS expression graph. The boundary follows Yap's
    /// exact-computation separation between expression structure and geometric
    /// decisions; see Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997).
    pub symbolic_dependencies: RealSymbolicDependencyMask,
    /// Axis occupied by a known nonzero component when the other component is
    /// known zero.
    pub known_axis: Option<Axis2>,
    /// Signed unit axis when one component is exactly `1` or `-1` and the other
    /// component is exactly zero.
    ///
    /// This is a conservative structural certificate only. It is useful for
    /// sparse and signed-permutation schedules, but orientation and incidence
    /// decisions still belong in `hyperlimit`.
    pub known_signed_axis: Option<SignedAxis2>,
    /// Whether both components are known zero.
    pub known_zero: bool,
}

impl Vector2Facts {
    /// Return the zero status for one component.
    pub fn component_zero(self, axis: Axis2) -> ZeroStatus {
        self.component_zero[axis.index()]
    }

    /// Return a bit mask of components known to be exactly zero.
    ///
    /// Bit 0 is x and bit 1 is y. This is structural metadata for choosing
    /// sparse exact kernels; it is not a geometric degeneracy predicate by
    /// itself.
    pub fn known_zero_mask(self) -> u8 {
        let mut mask = 0;
        if matches!(self.component_zero[0], ZeroStatus::Zero) {
            mask |= Axis2::X.bit();
        }
        if matches!(self.component_zero[1], ZeroStatus::Zero) {
            mask |= Axis2::Y.bit();
        }
        mask
    }

    /// Return a bit mask of components known to be nonzero.
    ///
    /// Carrying this mask lets callers avoid repeated scalar fact probes when
    /// choosing sparse product-sum or axis-aligned exact paths. The caller must
    /// still route sidedness and incidence decisions through `hyperlimit`.
    pub fn known_nonzero_mask(self) -> u8 {
        let mut mask = 0;
        if matches!(self.component_zero[0], ZeroStatus::NonZero) {
            mask |= Axis2::X.bit();
        }
        if matches!(self.component_zero[1], ZeroStatus::NonZero) {
            mask |= Axis2::Y.bit();
        }
        mask
    }

    /// Return a bit mask of components whose zero status is unknown.
    pub fn unknown_zero_mask(self) -> u8 {
        let mut mask = 0;
        if matches!(self.component_zero[0], ZeroStatus::Unknown) {
            mask |= Axis2::X.bit();
        }
        if matches!(self.component_zero[1], ZeroStatus::Unknown) {
            mask |= Axis2::Y.bit();
        }
        mask
    }

    /// Returns whether any component has unknown zero status.
    pub fn has_unknown_zero(self) -> bool {
        self.unknown_zero_mask() != 0
    }

    /// Count components known to be exactly zero.
    pub fn known_zero_count(self) -> u32 {
        self.known_zero_mask().count_ones()
    }

    /// Count components known to be nonzero.
    pub fn known_nonzero_count(self) -> u32 {
        self.known_nonzero_mask().count_ones()
    }

    /// Count components with unknown zero status.
    ///
    /// Structural-dispatch note: count helpers keep higher crates from
    /// reinterpreting the mask layout. This lets future `hyperlattice` versions
    /// grow richer vector facts, while callers still select sparse exact
    /// product-sum paths from stable public metadata. The sparse-kernel
    /// motivation follows Gustavson, "Two Fast Algorithms for Sparse Matrices:
    /// Multiplication and Permuted Transposition," *ACM Transactions on
    /// Mathematical Software* 4.3 (1978).
    pub fn unknown_zero_count(self) -> u32 {
        self.unknown_zero_mask().count_ones()
    }

    /// Return the zero status of the squared Euclidean norm.
    ///
    /// This is a structural certificate about `x*x + y*y`, not the norm value
    /// itself. If any coordinate is known nonzero, the squared norm is known
    /// nonzero over the exact ordered `Real` field; if all coordinates are
    /// known zero it is zero; otherwise it remains unknown. Carrying this fact
    /// at the vector boundary follows Yap's object-structure-first exact
    /// computation model and lets normalization or distance code reject
    /// zero/unknown cases before building a product-sum.
    pub fn squared_norm_zero_status(self) -> ZeroStatus {
        squared_norm_zero_status_from_counts(self.known_nonzero_count(), self.unknown_zero_count())
    }

    /// Returns whether this vector is a certified signed unit axis.
    pub fn is_signed_unit_axis(self) -> bool {
        self.known_signed_axis.is_some()
    }
}

/// Cheap structural facts known for a [`Vector3`].
///
/// The masks are conservative object metadata for exact-kernel dispatch. They
/// intentionally do not decide collinearity, orientation, incidence, or
/// containment; those combinatorial questions belong in `hyperlimit`. Carrying
/// the masks at the vector boundary follows Yap's exact-geometric-computation
/// guidance to preserve object structure before scalar expansion; see Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997). Sparse mask scheduling follows the same motivation as Gustavson,
/// "Two Fast Algorithms for Sparse Matrices: Multiplication and Permuted
/// Transposition," *ACM Transactions on Mathematical Software* 4.3 (1978).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Vector3Facts {
    /// Zero status for `[x, y, z]` components.
    pub component_zero: [ZeroStatus; 3],
    /// Exact-rational representation facts for the coordinate set.
    pub exact: ExactRealSetFacts,
    /// Union of scalar symbolic dependency families across all components.
    ///
    /// This is structural metadata for algorithm selection only. Incidence,
    /// orientation, and containment still belong in `hyperlimit`.
    pub symbolic_dependencies: RealSymbolicDependencyMask,
    /// Bit mask of components known to be exactly zero.
    pub known_zero_mask: u8,
    /// Bit mask of components known to be nonzero.
    pub known_nonzero_mask: u8,
    /// Bit mask of components whose zero status is unknown.
    pub unknown_zero_mask: u8,
    /// Bit mask of components known to be exactly one.
    pub one_mask: u8,
    /// Component index of a known one-hot vector, when exactly one component is
    /// nonzero and all other components are known zero.
    pub known_axis_index: Option<usize>,
    /// Whether all components are known zero.
    pub known_zero: bool,
}

impl Vector3Facts {
    /// Count components known to be exactly zero.
    pub fn known_zero_count(self) -> u32 {
        self.known_zero_mask.count_ones()
    }

    /// Count components known to be nonzero.
    pub fn known_nonzero_count(self) -> u32 {
        self.known_nonzero_mask.count_ones()
    }

    /// Count components with unknown zero status.
    pub fn unknown_zero_count(self) -> u32 {
        self.unknown_zero_mask.count_ones()
    }

    /// Return the zero status of the squared Euclidean norm.
    ///
    /// This structural certificate lets callers avoid constructing a self-dot
    /// expression just to discover definite zero or definite nonzero norm
    /// status. It is conservative metadata only; exact distance comparisons
    /// still belong in predicate code.
    pub fn squared_norm_zero_status(self) -> ZeroStatus {
        squared_norm_zero_status_from_counts(self.known_nonzero_count(), self.unknown_zero_count())
    }
}

/// Cheap structural facts known for a [`Vector4`].
///
/// In addition to sparse masks, this carries the homogeneous point/direction
/// classification used by projective transform pipelines. The classification is
/// exact structural metadata only: `w = 0` is a direction, `w = 1` is a point,
/// and every other or unknown weight remains [`Vector4HomogeneousKind::Unknown`]
/// so generic projective algebra is preserved.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Vector4Facts {
    /// Zero status for `[x, y, z, w]` components.
    pub component_zero: [ZeroStatus; 4],
    /// Exact-rational representation facts for the coordinate set.
    pub exact: ExactRealSetFacts,
    /// Union of scalar symbolic dependency families across all components.
    ///
    /// Homogeneous transform pipelines can use this to keep symbolic constants
    /// and opaque computable lanes visible at the vector-object boundary
    /// without exposing scalar representation details.
    pub symbolic_dependencies: RealSymbolicDependencyMask,
    /// Bit mask of components known to be exactly zero.
    pub known_zero_mask: u8,
    /// Bit mask of components known to be nonzero.
    pub known_nonzero_mask: u8,
    /// Bit mask of components whose zero status is unknown.
    pub unknown_zero_mask: u8,
    /// Bit mask of components known to be exactly one.
    pub one_mask: u8,
    /// Component index of a known one-hot vector, when exactly one component is
    /// nonzero and all other components are known zero.
    pub known_axis_index: Option<usize>,
    /// Whether all components are known zero.
    pub known_zero: bool,
    /// Homogeneous point/direction classification for the `w` coordinate.
    pub homogeneous: Vector4HomogeneousKind,
}

impl Vector4Facts {
    /// Count components known to be exactly zero.
    pub fn known_zero_count(self) -> u32 {
        self.known_zero_mask.count_ones()
    }

    /// Count components known to be nonzero.
    pub fn known_nonzero_count(self) -> u32 {
        self.known_nonzero_mask.count_ones()
    }

    /// Count components with unknown zero status.
    pub fn unknown_zero_count(self) -> u32 {
        self.unknown_zero_mask.count_ones()
    }

    /// Return the zero status of the squared Euclidean norm.
    ///
    /// For homogeneous vectors this is still only an algebraic norm fact over
    /// the four stored components. It does not classify projective points or
    /// directions topologically.
    pub fn squared_norm_zero_status(self) -> ZeroStatus {
        squared_norm_zero_status_from_counts(self.known_nonzero_count(), self.unknown_zero_count())
    }
}

/// Two-dimensional vector.
#[derive(Clone, Debug, PartialEq)]
pub struct Vector2(
    /// Components stored in `[x, y]` order.
    pub [Real; 2],
);

/// Three-dimensional vector.
#[derive(Clone, Debug, PartialEq)]
pub struct Vector3(
    /// Components stored in `[x, y, z]` order.
    pub [Real; 3],
);

/// Four-dimensional vector.
#[derive(Clone, Debug, PartialEq)]
pub struct Vector4(
    /// Components stored in `[x, y, z, w]` order.
    pub [Real; 4],
);

/// Exact structural classification of a 4D homogeneous coordinate.
///
/// This is an object fact for projective transform dispatch: `w = 0` is a
/// direction, `w = 1` is an affine point, and every other or undecidable value
/// remains unknown so generic homogeneous algebra is preserved.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Vector4HomogeneousKind {
    /// A vector lies on the implicit vector subspace in homogeneous coordinates.
    Direction,
    /// A point has unit homogeneous coordinate in projective form.
    Point,
    /// `w` is neither provably zero nor one from structural facts.
    Unknown,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct Vector4GeometricFacts {
    /// Whether this 4-vector is structurally a point, direction, or unknown.
    pub(crate) homogeneous: Vector4HomogeneousKind,
}

#[inline]
fn vector4_geometric_facts(values: &[Real; 4]) -> Vector4GeometricFacts {
    // Homogeneous geometry kernels in exact computation benefit from keeping
    // `w`-classification as retained structure; this is the projective split
    // used throughout 3D affine pipelines, and mirrors the direction/point
    // specialization logic that precedes exact reductions in robust kernels
    // (Yap, “Towards Exact Geometric Computation”, 1997).
    let homogeneous = match values[3].zero_one_or_minus_one() {
        RealZeroOneMinusOneStatus::Zero => Vector4HomogeneousKind::Direction,
        RealZeroOneMinusOneStatus::One => Vector4HomogeneousKind::Point,
        RealZeroOneMinusOneStatus::MinusOne | RealZeroOneMinusOneStatus::NeitherOrUnknown => {
            Vector4HomogeneousKind::Unknown
        }
    };
    Vector4GeometricFacts { homogeneous }
}

#[inline(always)]
fn squared_norm_zero_status_from_counts(known_nonzero: u32, unknown_zero: u32) -> ZeroStatus {
    if known_nonzero > 0 {
        ZeroStatus::NonZero
    } else if unknown_zero > 0 {
        ZeroStatus::Unknown
    } else {
        ZeroStatus::Zero
    }
}

#[inline(always)]
fn require_known_nonzero_status(status: ZeroStatus) -> CheckedBlasResult<()> {
    match status {
        ZeroStatus::Zero => {
            crate::trace_dispatch!("hyperlattice_vector", "norm-facts", "checked-zero-rejected");
            Err(Problem::DivideByZero)
        }
        ZeroStatus::NonZero => {
            crate::trace_dispatch!("hyperlattice_vector", "norm-facts", "checked-nonzero");
            Ok(())
        }
        ZeroStatus::Unknown => {
            crate::trace_dispatch!(
                "hyperlattice_vector",
                "norm-facts",
                "checked-unknown-rejected"
            );
            Err(Problem::UnknownZero)
        }
    }
}

trait VectorSelfDot {
    fn self_dot(values: &Self) -> Real;
}

impl VectorSelfDot for [Real; 2] {
    #[inline]
    fn self_dot(values: &Self) -> Real {
        Real::signed_product_sum2(
            [true, true],
            [[&values[0], &values[0]], [&values[1], &values[1]]],
        )
    }
}

impl VectorSelfDot for [Real; 3] {
    #[inline]
    fn self_dot(values: &Self) -> Real {
        Real::dot3_same([&values[0], &values[1], &values[2]])
    }
}

impl VectorSelfDot for [Real; 4] {
    #[inline]
    fn self_dot(values: &Self) -> Real {
        Real::dot4_same([&values[0], &values[1], &values[2], &values[3]])
    }
}

fn map_array2<const N: usize, F>(left: [Real; N], right: [Real; N], mut op: F) -> [Real; N]
where
    F: FnMut(Real, Real) -> Real,
{
    let mut right = right.into_iter();
    left.map(|lhs| op(lhs, right.next().expect("arrays have equal length")))
}

fn map_array_ref<const N: usize, F>(left: [Real; N], right: &[Real; N], mut op: F) -> [Real; N]
where
    F: FnMut(Real, &Real) -> Real,
{
    let mut right = right.iter();
    left.map(|lhs| op(lhs, right.next().expect("arrays have equal length")))
}

macro_rules! impl_vector {
    ($name:ident, $n:expr) => {
        impl $name {
            /// Constructs a vector from its component array.
            pub fn new(values: [Real; $n]) -> Self {
                crate::trace_dispatch!("hyperlattice_vector", "constructor", "new");
                Self(values)
            }

            /// Returns the zero vector.
            pub fn zero() -> Self {
                crate::trace_dispatch!("hyperlattice_vector", "constructor", "zero");
                Self(from_fn(|_| Real::zero()))
            }

            /// Returns the Euclidean magnitude.
            pub fn magnitude(&self) -> BlasResult<Real> {
                crate::trace_dispatch!("hyperlattice_vector", "method", "magnitude");
                self.magnitude_squared_fast().sqrt()
            }

            /// Returns the Euclidean magnitude after attaching an abort signal.
            pub fn magnitude_with_abort(&self, signal: &AbortSignal) -> BlasResult<Real> {
                crate::trace_dispatch!("hyperlattice_vector", "method", "magnitude-with-abort");
                with_abort(self.dot_with_abort(self, signal), signal).sqrt()
            }

            #[inline]
            fn magnitude_squared_fast(&self) -> Real {
                // Magnitude is a self-dot, so each structural zero fact is
                // shared by both multiplicands. Use the dedicated self-dot
                // kernels to avoid redundant fact probes while keeping dense
                // inputs on the Real dot hook; that preserves hyperreal's
                // deferred exact-rational reduction strategy.
                <[Real; $n] as VectorSelfDot>::self_dot(&self.0)
            }

            /// Returns a unit vector in the same direction.
            ///
            /// This rejects definite zero magnitudes before division. If the
            /// Real arithmetic rejects a divisor for another reason, that
            /// [`Problem`](crate::Problem) is propagated.
            pub fn normalize(&self) -> BlasResult<Self> {
                crate::trace_dispatch!("hyperlattice_vector", "method", "normalize");
                let mag = self.magnitude()?;
                let inv_mag = mag.inverse()?;
                // Keep the borrowed `Mul` form here. A `mul_cached` prototype
                // reused the reciprocal magnitude like matrix inverse scaling,
                // but Criterion regressed hyperreal and hyperreal-rational
                // normalize rows. For 3/4 component vectors,
                // preserving the Real kernel's direct borrowed multiply is cheaper
                // than forcing the matrix shared-scale path.
                Ok(Self(from_fn(|i| &self.0[i] * &inv_mag)))
            }

            /// Returns a unit vector after rejecting zero and unknown-zero magnitudes.
            pub fn normalize_checked(&self) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!("hyperlattice_vector", "method", "normalize-checked");
                let norm_status = self.structural_facts().squared_norm_zero_status();
                require_known_nonzero_status(norm_status)?;
                let mag_squared = self.magnitude_squared_fast();
                let mag = mag_squared.sqrt()?;
                let inv_mag = mag.inverse()?;
                // See `normalize`: direct borrowed multiply benchmarks faster
                // than forcing the matrix shared-scale helper for vectors.
                Ok(Self(from_fn(|i| &self.0[i] * &inv_mag)))
            }

            /// Returns a checked unit vector after attaching an abort signal.
            pub fn normalize_checked_with_abort(
                &self,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!(
                    "hyperlattice_vector",
                    "method",
                    "normalize-checked-with-abort"
                );
                let norm_status = self.structural_facts().squared_norm_zero_status();
                require_known_nonzero_status(norm_status)?;
                let mag_squared = with_abort(self.dot_with_abort(self, signal), signal);
                let mag = mag_squared.sqrt()?;
                let inv_mag = mag.inverse()?;
                // See `normalize`: direct borrowed multiply keeps this vector
                // path faster after abort-aware magnitude construction.
                Ok(Self(from_fn(|i| &self.0[i] * &inv_mag)))
            }

            /// Divides every component by `rhs` after rejecting unknown-zero divisors.
            pub fn div_scalar_checked(self, rhs: Real) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!("hyperlattice_vector", "method", "div-scalar-checked");
                require_known_nonzero(&rhs)?;
                let inv_rhs = rhs.inverse()?;
                if true && $n == 3 {
                    // Keep this vec3 path aligned with normalize: for three
                    // lanes, borrowed multiply can be cheaper than
                    // forcing the shared-scale helper used by matrices.
                    Ok(Self(self.0.map(|value| &value * &inv_rhs)))
                } else if true {
                    // Vec4 still benchmarks faster through the cached scalar
                    // multiply path because it amortizes the shared scale over
                    // one extra lane.
                    Ok(Self(self.0.map(|value| value.mul_cached(&inv_rhs))))
                } else {
                    let mut values = self.0;
                    for value in &mut values {
                        *value = value.clone().mul_cached(&inv_rhs);
                    }
                    Ok(Self(values))
                }
            }

            /// Divides every component by `rhs` after attaching an abort signal.
            pub fn div_scalar_checked_with_abort(
                self,
                rhs: Real,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!(
                    "hyperlattice_vector",
                    "method",
                    "div-scalar-checked-with-abort"
                );
                let rhs = with_abort(rhs, signal);
                require_known_nonzero(&rhs)?;
                let inv_rhs = rhs.inverse()?;
                if true && $n == 3 {
                    Ok(Self(self.0.map(|value| &value * &inv_rhs)))
                } else if true {
                    Ok(Self(self.0.map(|value| value.mul_cached(&inv_rhs))))
                } else {
                    let mut values = self.0;
                    for value in &mut values {
                        *value = value.clone().mul_cached(&inv_rhs);
                    }
                    Ok(Self(values))
                }
            }
        }

        impl Index<usize> for $name {
            type Output = Real;

            fn index(&self, index: usize) -> &Self::Output {
                &self.0[index]
            }
        }

        impl IndexMut<usize> for $name {
            fn index_mut(&mut self, index: usize) -> &mut Self::Output {
                &mut self.0[index]
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str("[")?;
                for i in 0..$n {
                    if i > 0 {
                        f.write_str(", ")?;
                    }
                    if f.alternate() {
                        write!(f, "{:#}", self.0[i])?;
                    } else {
                        write!(f, "{}", self.0[i])?;
                    }
                }
                f.write_str("]")
            }
        }

        impl Add for $name {
            type Output = Self;

            fn add(self, rhs: Self) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "add-owned-owned");
                if true {
                    Self(map_array2(self.0, rhs.0, |lhs, rhs| lhs + rhs))
                } else {
                    Self(from_fn(|i| self.0[i].clone() + rhs.0[i].clone()))
                }
            }
        }

        impl Add<&$name> for $name {
            type Output = Self;

            fn add(self, rhs: &$name) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "add-owned-ref");
                Self(map_array_ref(self.0, &rhs.0, Real::add_cached))
            }
        }

        impl Add<$name> for &$name {
            type Output = $name;

            fn add(self, rhs: $name) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "add-ref-owned");
                let mut left = self.0.iter();
                $name(
                    rhs.0
                        .map(|rhs| left.next().expect("vectors have equal length") + rhs),
                )
            }
        }

        impl Add<&$name> for &$name {
            type Output = $name;

            fn add(self, rhs: &$name) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "add-ref-ref");
                $name(from_fn(|i| &self.0[i] + &rhs.0[i]))
            }
        }

        impl Add<Real> for $name {
            type Output = Self;

            fn add(self, rhs: Real) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "add-scalar-owned");
                let rhs = &rhs;
                if true {
                    Self(self.0.map(|value| value.add_cached(rhs)))
                } else {
                    let mut values = self.0;
                    for value in &mut values {
                        *value = value.clone().add_cached(rhs);
                    }
                    Self(values)
                }
            }
        }

        impl Add<&Real> for $name {
            type Output = Self;

            fn add(self, rhs: &Real) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "add-scalar-ref");
                Self(self.0.map(|value| value.add_cached(rhs)))
            }
        }

        impl Sub for $name {
            type Output = Self;

            fn sub(self, rhs: Self) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "sub-owned-owned");
                if true {
                    Self(map_array2(self.0, rhs.0, |lhs, rhs| lhs - rhs))
                } else {
                    Self(from_fn(|i| self.0[i].clone() - rhs.0[i].clone()))
                }
            }
        }

        impl Sub<&$name> for $name {
            type Output = Self;

            fn sub(self, rhs: &$name) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "sub-owned-ref");
                Self(map_array_ref(self.0, &rhs.0, Real::sub_cached))
            }
        }

        impl Sub<$name> for &$name {
            type Output = $name;

            fn sub(self, rhs: $name) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "sub-ref-owned");
                let mut left = self.0.iter();
                $name(
                    rhs.0
                        .map(|rhs| left.next().expect("vectors have equal length") - rhs),
                )
            }
        }

        impl Sub<&$name> for &$name {
            type Output = $name;

            fn sub(self, rhs: &$name) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "sub-ref-ref");
                $name(from_fn(|i| &self.0[i] - &rhs.0[i]))
            }
        }

        impl Sub<Real> for $name {
            type Output = Self;

            fn sub(self, rhs: Real) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "sub-scalar-owned");
                let rhs = -rhs;
                let rhs = &rhs;
                if true {
                    Self(self.0.map(|value| value.add_cached(rhs)))
                } else {
                    let mut values = self.0;
                    for value in &mut values {
                        *value = value.clone().add_cached(rhs);
                    }
                    Self(values)
                }
            }
        }

        impl Sub<&Real> for $name {
            type Output = Self;

            fn sub(self, rhs: &Real) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "sub-scalar-ref");
                let rhs = -rhs.clone();
                Self(self.0.map(|value| value.add_cached(&rhs)))
            }
        }

        impl Neg for $name {
            type Output = Self;

            fn neg(self) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "neg-owned");
                if true {
                    Self(self.0.map(|value| -value))
                } else {
                    Self(from_fn(|i| -self.0[i].clone()))
                }
            }
        }

        impl Neg for &$name {
            type Output = $name;

            fn neg(self) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "neg-ref");
                $name(from_fn(|i| -self.0[i].clone()))
            }
        }

        impl Mul<Real> for $name {
            type Output = Self;

            fn mul(self, rhs: Real) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "mul-scalar-owned");
                let rhs = &rhs;
                if true {
                    Self(self.0.map(|value| value.mul_cached(rhs)))
                } else {
                    let mut values = self.0;
                    for value in &mut values {
                        *value = value.clone().mul_cached(rhs);
                    }
                    Self(values)
                }
            }
        }

        impl Mul<&Real> for $name {
            type Output = Self;

            fn mul(self, rhs: &Real) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "mul-scalar-ref");
                Self(self.0.map(|value| value.mul_cached(rhs)))
            }
        }

        impl Div<Real> for $name {
            type Output = BlasResult<Self>;

            fn div(self, rhs: Real) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "div-scalar-owned");
                reject_definite_zero(&rhs)?;
                let inv_rhs = rhs.inverse()?;
                if true && $n == 3 {
                    Ok(Self(self.0.map(|value| &value * &inv_rhs)))
                } else if true {
                    Ok(Self(self.0.map(|value| value.mul_cached(&inv_rhs))))
                } else {
                    let mut values = self.0;
                    for value in &mut values {
                        *value = value.clone().mul_cached(&inv_rhs);
                    }
                    Ok(Self(values))
                }
            }
        }

        impl Div<&Real> for $name {
            type Output = BlasResult<Self>;

            fn div(self, rhs: &Real) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "div-scalar-ref");
                reject_definite_zero(rhs)?;
                let inv_rhs = rhs.inverse_ref()?;
                if true && $n == 3 {
                    Ok(Self(self.0.map(|value| &value * &inv_rhs)))
                } else if true {
                    Ok(Self(self.0.map(|value| value.mul_cached(&inv_rhs))))
                } else {
                    let mut values = self.0;
                    for value in &mut values {
                        *value = value.clone().mul_cached(&inv_rhs);
                    }
                    Ok(Self(values))
                }
            }
        }
    };
}

impl_vector!(Vector2, 2);
impl_vector!(Vector3, 3);
impl_vector!(Vector4, 4);

impl Vector4 {
    #[inline]
    pub(crate) fn geometric_facts(&self) -> Vector4GeometricFacts {
        // Keep geometric classification on the vector object itself so matrix
        // handles can avoid re-reading `zero_or_one` for mixed batches.
        vector4_geometric_facts(&self.0)
    }
}

impl Vector2 {
    /// Converts this vector into an owned shared-scale carrier when possible.
    ///
    /// This preserves the common-denominator certificate across ownership
    /// boundaries without exposing scalar numerator or denominator storage.
    pub fn into_shared_scale(self) -> Option<SharedScaleVec<2>> {
        SharedScaleVec::from_components(self.0)
    }

    /// Returns a borrowed shared-scale view when both coordinates are exact
    /// rationals with one reduced denominator.
    ///
    /// The returned view lets predicate and geometry layers carry the
    /// common-scale fact as object metadata without exposing numerator or
    /// denominator storage. Use `hyperlimit` for topology decisions; this view
    /// is only an exact-kernel scheduling hint.
    pub fn shared_scale_view(&self) -> Option<VectorSharedScaleView<'_, 2>> {
        crate::trace_dispatch!("hyperlattice_vector", "query", "vector2-shared-scale-view");
        VectorSharedScaleView::from_components([&self.0[0], &self.0[1]])
    }

    /// Returns cheap structural facts for this vector.
    ///
    /// This method preserves coordinate zero masks at the vector boundary so
    /// higher crates can select sparse exact kernels without re-probing every
    /// scalar lane. It remains algebraic metadata, not a topology predicate:
    /// orientation, incidence, and containment decisions still belong in
    /// `hyperlimit`. This follows Yap's recommendation to exploit geometric
    /// object structure before lower-level arithmetic in exact geometric
    /// computation; see Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997).
    pub fn structural_facts(&self) -> Vector2Facts {
        crate::trace_dispatch!("hyperlattice_vector", "query", "vector2-structural-facts");
        let component_zero = [self.0[0].zero_status(), self.0[1].zero_status()];
        let known_zero = matches!(component_zero, [ZeroStatus::Zero, ZeroStatus::Zero]);
        let known_axis = match component_zero {
            [ZeroStatus::NonZero, ZeroStatus::Zero] => Some(Axis2::X),
            [ZeroStatus::Zero, ZeroStatus::NonZero] => Some(Axis2::Y),
            _ => None,
        };

        Vector2Facts {
            component_zero,
            exact: crate::kernels::exact_real_set_facts(self.0.iter()),
            symbolic_dependencies: vector_symbolic_dependency_mask([&self.0[0], &self.0[1]]),
            known_axis,
            known_signed_axis: signed_axis2_from_components(&self.0),
            known_zero,
        }
    }

    /// Returns the dot product with `rhs`.
    pub fn dot(&self, rhs: &Self) -> Real {
        crate::trace_dispatch!("hyperlattice_vector", "method", "dot2");
        crate::dot2([&self.0[0], &self.0[1]], [&rhs.0[0], &rhs.0[1]])
    }

    /// Returns the dot product after attaching an abort signal to operands.
    pub fn dot_with_abort(&self, rhs: &Self, signal: &AbortSignal) -> Real {
        crate::trace_dispatch!("hyperlattice_vector", "method", "dot2-with-abort");
        if !signal.load(Ordering::Relaxed) {
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot2-inactive-signal");
            return self.dot(rhs);
        }

        let has0 = !self.0[0].definitely_zero() && !rhs.0[0].definitely_zero();
        let has1 = !self.0[1].definitely_zero() && !rhs.0[1].definitely_zero();

        if !has0 && !has1 {
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot2-sparse-all-zero");
            return Real::zero();
        }

        let product = |lhs: &Real, rhs: &Real, signal: &AbortSignal| {
            clone_with_abort(lhs, signal) * clone_with_abort(rhs, signal)
        };

        if has0 && has1 {
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot2-sparse-two-nonzero");
            let lhs0 = clone_with_abort(&self.0[0], signal);
            let rhs0 = clone_with_abort(&rhs.0[0], signal);
            let lhs1 = clone_with_abort(&self.0[1], signal);
            let rhs1 = clone_with_abort(&rhs.0[1], signal);
            return Real::active_signed_product_sum2(
                [true, true],
                [[&lhs0, &rhs0], [&lhs1, &rhs1]],
            );
        }

        crate::trace_dispatch!("hyperlattice_vector", "abort", "dot2-sparse-single");
        if has0 {
            product(&self.0[0], &rhs.0[0], signal)
        } else {
            product(&self.0[1], &rhs.0[1], signal)
        }
    }

    /// Returns the 2D exterior product with `rhs`.
    ///
    /// This is an exact scalar expression, not an orientation predicate. Use
    /// `hyperlimit` to decide its sign when topology is affected.
    pub fn wedge(&self, rhs: &Self) -> Real {
        crate::trace_dispatch!("hyperlattice_vector", "method", "wedge2");
        crate::wedge2([&self.0[0], &self.0[1]], [&rhs.0[0], &rhs.0[1]])
    }

    /// Returns the squared distance to `rhs`.
    pub fn squared_distance(&self, rhs: &Self) -> Real {
        crate::trace_dispatch!("hyperlattice_vector", "method", "squared-distance2");
        crate::squared_distance2([&self.0[0], &self.0[1]], [&rhs.0[0], &rhs.0[1]])
    }
}

impl Vector3 {
    /// Converts this vector into an owned shared-scale carrier when possible.
    ///
    /// The resulting object stores the original [`Real`] coordinates plus
    /// cached exact-set facts, giving downstream exact kernels a stable
    /// common-scale signal without a dependency on rational internals.
    pub fn into_shared_scale(self) -> Option<SharedScaleVec<3>> {
        SharedScaleVec::from_components(self.0)
    }

    /// Returns a borrowed shared-scale view when all coordinates are exact
    /// rationals with one reduced denominator.
    ///
    /// This is an opt-in bridge toward common-scale geometry kernels. It keeps
    /// vector provenance at the object layer, as recommended by Yap, while
    /// leaving scalar storage and reduction in `hyperreal`.
    pub fn shared_scale_view(&self) -> Option<VectorSharedScaleView<'_, 3>> {
        crate::trace_dispatch!("hyperlattice_vector", "query", "vector3-shared-scale-view");
        VectorSharedScaleView::from_components([&self.0[0], &self.0[1], &self.0[2]])
    }

    /// Returns cheap structural facts for this vector.
    ///
    /// This preserves zero, one, one-hot, and exact-set facts on the vector
    /// object. Callers can select sparse or shared-denominator algebra kernels
    /// from these facts, but geometric predicates remain the responsibility of
    /// `hyperlimit`.
    pub fn structural_facts(&self) -> Vector3Facts {
        crate::trace_dispatch!("hyperlattice_vector", "query", "vector3-structural-facts");
        let component_zero = [
            self.0[0].zero_status(),
            self.0[1].zero_status(),
            self.0[2].zero_status(),
        ];
        let (known_zero_mask, known_nonzero_mask, unknown_zero_mask) =
            vector_zero_status_masks([&self.0[0], &self.0[1], &self.0[2]]);
        Vector3Facts {
            component_zero,
            exact: crate::kernels::exact_real_set_facts(self.0.iter()),
            symbolic_dependencies: vector_symbolic_dependency_mask([
                &self.0[0], &self.0[1], &self.0[2],
            ]),
            known_zero_mask: known_zero_mask as u8,
            known_nonzero_mask: known_nonzero_mask as u8,
            unknown_zero_mask: unknown_zero_mask as u8,
            one_mask: vector_one_mask([&self.0[0], &self.0[1], &self.0[2]]) as u8,
            known_axis_index: if known_zero_mask.count_ones() == 2
                && known_nonzero_mask.count_ones() == 1
                && unknown_zero_mask == 0
            {
                single_bit_index(known_nonzero_mask)
            } else {
                None
            },
            known_zero: known_zero_mask == vector_mask::<3>(),
        }
    }

    /// Returns exact-rational representation facts for the three coordinates.
    ///
    /// The facts are intentionally coarse and storage-free. They let higher
    /// crates carry the common-scale signal described by Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997), while
    /// keeping exact determinant and predicate decisions routed through the
    /// appropriate kernel layer.
    pub fn exact_facts(&self) -> ExactRealSetFacts {
        crate::trace_dispatch!("hyperlattice_vector", "query", "vector3-exact-facts");
        crate::kernels::exact_real_set_facts(self.0.iter())
    }

    /// Returns the 3D cross product with `rhs`.
    ///
    /// This is an exact algebraic vector expression, not a topology predicate.
    /// Each lane is a short determinant routed through the fixed product-sum
    /// reducer so exact rational inputs can delay normalization. See Bareiss,
    /// "Sylvester's Identity and Multistep Integer-Preserving Gaussian
    /// Elimination," *Mathematics of Computation* 22.103 (1968). Future
    /// callers that already carry common-scale certificates should prefer
    /// [`SharedScaleVec<3>::cross`] to bypass repeated scalar exactness probes,
    /// preserving the object-level structure emphasized by Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997).
    pub fn cross(&self, rhs: &Self) -> Self {
        crate::trace_dispatch!("hyperlattice_vector", "method", "cross3");
        Self::new([
            Real::signed_product_sum2(
                [true, false],
                [[&self.0[1], &rhs.0[2]], [&self.0[2], &rhs.0[1]]],
            ),
            Real::signed_product_sum2(
                [true, false],
                [[&self.0[2], &rhs.0[0]], [&self.0[0], &rhs.0[2]]],
            ),
            Real::signed_product_sum2(
                [true, false],
                [[&self.0[0], &rhs.0[1]], [&self.0[1], &rhs.0[0]]],
            ),
        ])
    }

    /// Returns the dot product with `rhs`.
    pub fn dot(&self, rhs: &Self) -> Real {
        crate::trace_dispatch!("hyperlattice_vector", "method", "dot3");
        Real::dot3(
            [&self.0[0], &self.0[1], &self.0[2]],
            [&rhs.0[0], &rhs.0[1], &rhs.0[2]],
        )
    }

    /// Returns the dot product after attaching an abort signal to operands.
    pub fn dot_with_abort(&self, rhs: &Self, signal: &AbortSignal) -> Real {
        crate::trace_dispatch!("hyperlattice_vector", "method", "dot3-with-abort");
        if !signal.load(Ordering::Relaxed) {
            // 2026-05 trace-guided shortcut: inactive abort signals are the
            // common predicate benchmark case, and clone-and-attach bypasses
            // hyperreal's shared-denominator exact-rational dot path. Reuse
            // the ordinary dot unless a cancellation request is already set;
            // the active path below preserves abort-aware operand attachment.
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot3-inactive-signal");
            return self.dot(rhs);
        }
        let has0 = !self.0[0].definitely_zero() && !rhs.0[0].definitely_zero();
        let has1 = !self.0[1].definitely_zero() && !rhs.0[1].definitely_zero();
        let has2 = !self.0[2].definitely_zero() && !rhs.0[2].definitely_zero();

        if !has0 && !has1 && !has2 {
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot3-sparse-all-zero");
            return Real::zero();
        }

        let product = |lhs: &Real, rhs: &Real, signal: &AbortSignal| {
            clone_with_abort(lhs, signal) * clone_with_abort(rhs, signal)
        };
        let product_sum2 =
            |lhs0: &Real, rhs0: &Real, lhs1: &Real, rhs1: &Real, signal: &AbortSignal| {
                // The structural scan above has already reduced this active-abort
                // dot to exactly two contributing lanes. Attach the abort signal
                // once per surviving operand, then keep the pair as a product-sum
                // so exact Real kernels can reuse denominator/canonicalization work
                // rather than materializing two independent products plus an add.
                // This is the same delayed-normalization principle as Bareiss,
                // Math. Comp. 22(103), 1968, <https://doi.org/10.2307/2004533>.
                let lhs0 = clone_with_abort(lhs0, signal);
                let rhs0 = clone_with_abort(rhs0, signal);
                let lhs1 = clone_with_abort(lhs1, signal);
                let rhs1 = clone_with_abort(rhs1, signal);
                Real::active_signed_product_sum2([true, true], [[&lhs0, &rhs0], [&lhs1, &rhs1]])
            };
        let product_sum3 = |lhs0: &Real,
                            rhs0: &Real,
                            lhs1: &Real,
                            rhs1: &Real,
                            lhs2: &Real,
                            rhs2: &Real,
                            signal: &AbortSignal| {
            // All three lanes survived cheap structural-zero pruning. Keep the
            // active-abort dot as one exact product polynomial after operand
            // attachment instead of reducing three independent products. This
            // mirrors the vec4 sparse-three fast path and follows Bareiss-style
            // delayed normalization for short exact sums:
            // Bareiss, Math. Comp. 22(103), 1968,
            // <https://doi.org/10.2307/2004533>.
            let lhs0 = clone_with_abort(lhs0, signal);
            let rhs0 = clone_with_abort(rhs0, signal);
            let lhs1 = clone_with_abort(lhs1, signal);
            let rhs1 = clone_with_abort(rhs1, signal);
            let lhs2 = clone_with_abort(lhs2, signal);
            let rhs2 = clone_with_abort(rhs2, signal);
            Real::active_signed_product_sum2(
                [true, true, true],
                [[&lhs0, &rhs0], [&lhs1, &rhs1], [&lhs2, &rhs2]],
            )
        };
        if has0 as u8 + has1 as u8 + has2 as u8 == 1 {
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot3-sparse-single");
            return if has0 {
                product(&self.0[0], &rhs.0[0], signal)
            } else if has1 {
                product(&self.0[1], &rhs.0[1], signal)
            } else {
                product(&self.0[2], &rhs.0[2], signal)
            };
        }

        if has0 as u8 + has1 as u8 + has2 as u8 == 2 {
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot3-sparse-two");
            return if has0 && has1 {
                product_sum2(&self.0[0], &rhs.0[0], &self.0[1], &rhs.0[1], signal)
            } else if has0 && has2 {
                product_sum2(&self.0[0], &rhs.0[0], &self.0[2], &rhs.0[2], signal)
            } else {
                product_sum2(&self.0[1], &rhs.0[1], &self.0[2], &rhs.0[2], signal)
            };
        }

        crate::trace_dispatch!("hyperlattice_vector", "abort", "dot3-sparse-three-nonzero");
        product_sum3(
            &self.0[0], &rhs.0[0], &self.0[1], &rhs.0[1], &self.0[2], &rhs.0[2], signal,
        )
    }
}

impl Vector4 {
    /// Converts this vector into an owned shared-scale carrier when possible.
    ///
    /// This is useful for homogeneous transform paths that need to carry an
    /// object-level denominator schedule beyond a borrowed vector lifetime.
    pub fn into_shared_scale(self) -> Option<SharedScaleVec<4>> {
        SharedScaleVec::from_components(self.0)
    }

    /// Returns a borrowed shared-scale view when all coordinates are exact
    /// rationals with one reduced denominator.
    ///
    /// Homogeneous transform and predicate code can carry this view to select
    /// shared-denominator exact schedules without rescanning every coordinate
    /// or depending on `Rational` internals.
    pub fn shared_scale_view(&self) -> Option<VectorSharedScaleView<'_, 4>> {
        crate::trace_dispatch!("hyperlattice_vector", "query", "vector4-shared-scale-view");
        VectorSharedScaleView::from_components([&self.0[0], &self.0[1], &self.0[2], &self.0[3]])
    }

    /// Returns cheap structural facts for this homogeneous vector.
    ///
    /// The returned facts combine sparse coordinate masks, exact-set facts, and
    /// the projective point/direction split used by matrix transform kernels.
    /// This follows Yap's object-fact discipline while keeping all scalar
    /// representation details in `hyperreal`.
    pub fn structural_facts(&self) -> Vector4Facts {
        crate::trace_dispatch!("hyperlattice_vector", "query", "vector4-structural-facts");
        let component_zero = [
            self.0[0].zero_status(),
            self.0[1].zero_status(),
            self.0[2].zero_status(),
            self.0[3].zero_status(),
        ];
        let (known_zero_mask, known_nonzero_mask, unknown_zero_mask) =
            vector_zero_status_masks([&self.0[0], &self.0[1], &self.0[2], &self.0[3]]);
        Vector4Facts {
            component_zero,
            exact: crate::kernels::exact_real_set_facts(self.0.iter()),
            symbolic_dependencies: vector_symbolic_dependency_mask([
                &self.0[0], &self.0[1], &self.0[2], &self.0[3],
            ]),
            known_zero_mask: known_zero_mask as u8,
            known_nonzero_mask: known_nonzero_mask as u8,
            unknown_zero_mask: unknown_zero_mask as u8,
            one_mask: vector_one_mask([&self.0[0], &self.0[1], &self.0[2], &self.0[3]]) as u8,
            known_axis_index: if known_zero_mask.count_ones() == 3
                && known_nonzero_mask.count_ones() == 1
                && unknown_zero_mask == 0
            {
                single_bit_index(known_nonzero_mask)
            } else {
                None
            },
            known_zero: known_zero_mask == vector_mask::<4>(),
            homogeneous: self.geometric_facts().homogeneous,
        }
    }

    /// Returns exact-rational representation facts for the four coordinates.
    ///
    /// This is a retained structural fact for projective and affine pipelines:
    /// callers can detect dyadic grids or shared reduced denominators before
    /// choosing a fixed exact schedule, without exposing scalar storage.
    pub fn exact_facts(&self) -> ExactRealSetFacts {
        crate::trace_dispatch!("hyperlattice_vector", "query", "vector4-exact-facts");
        crate::kernels::exact_real_set_facts(self.0.iter())
    }

    /// Returns the dot product with `rhs`.
    pub fn dot(&self, rhs: &Self) -> Real {
        crate::trace_dispatch!("hyperlattice_vector", "method", "dot4");
        Real::dot4(
            [&self.0[0], &self.0[1], &self.0[2], &self.0[3]],
            [&rhs.0[0], &rhs.0[1], &rhs.0[2], &rhs.0[3]],
        )
    }

    /// Returns the dot product after attaching an abort signal to operands.
    pub fn dot_with_abort(&self, rhs: &Self, signal: &AbortSignal) -> Real {
        crate::trace_dispatch!("hyperlattice_vector", "method", "dot4-with-abort");
        if !signal.load(Ordering::Relaxed) {
            // Same inactive-abort policy as `Vector3`: keep matrix/vector
            // benches on the Real dot specialization unless cancellation is
            // already requested.
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot4-inactive-signal");
            return self.dot(rhs);
        }
        let has0 = !self.0[0].definitely_zero() && !rhs.0[0].definitely_zero();
        let has1 = !self.0[1].definitely_zero() && !rhs.0[1].definitely_zero();
        let has2 = !self.0[2].definitely_zero() && !rhs.0[2].definitely_zero();
        let has3 = !self.0[3].definitely_zero() && !rhs.0[3].definitely_zero();
        let nonzero = has0 as u8 + has1 as u8 + has2 as u8 + has3 as u8;

        if nonzero == 0 {
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot4-sparse-all-zero");
            return Real::zero();
        }

        let product = |lhs: &Real, rhs: &Real, signal: &AbortSignal| {
            clone_with_abort(lhs, signal) * clone_with_abort(rhs, signal)
        };
        let product_sum2 =
            |lhs0: &Real, rhs0: &Real, lhs1: &Real, rhs1: &Real, signal: &AbortSignal| {
                // Keep the two active-abort lanes as one product-sum after operand
                // attachment. This mirrors the non-abort sparse-dot fast path and
                // preserves exact-rational sharing where abort wrappers still allow
                // the Real kernel to see through to exact structure, following the
                // delayed-normalization principle in Bareiss, Math. Comp. 22(103),
                // 1968, <https://doi.org/10.2307/2004533>.
                let lhs0 = clone_with_abort(lhs0, signal);
                let rhs0 = clone_with_abort(rhs0, signal);
                let lhs1 = clone_with_abort(lhs1, signal);
                let rhs1 = clone_with_abort(rhs1, signal);
                Real::active_signed_product_sum2([true, true], [[&lhs0, &rhs0], [&lhs1, &rhs1]])
            };
        let product_sum3 = |lhs0: &Real,
                            rhs0: &Real,
                            lhs1: &Real,
                            rhs1: &Real,
                            lhs2: &Real,
                            rhs2: &Real,
                            signal: &AbortSignal| {
            // Three active sparse lanes are still a short exact polynomial, so
            // attach abort guards once per operand and keep the sum in the
            // Real kernel's fixed-product reducer instead of materializing three
            // products plus two adds. This follows the same delayed
            // normalization rationale as Bareiss fraction-free elimination:
            // keep exact products grouped until the last responsible moment.
            // Bareiss, Math. Comp. 22(103), 1968,
            // <https://doi.org/10.2307/2004533>.
            let lhs0 = clone_with_abort(lhs0, signal);
            let rhs0 = clone_with_abort(rhs0, signal);
            let lhs1 = clone_with_abort(lhs1, signal);
            let rhs1 = clone_with_abort(rhs1, signal);
            let lhs2 = clone_with_abort(lhs2, signal);
            let rhs2 = clone_with_abort(rhs2, signal);
            Real::active_signed_product_sum2(
                [true, true, true],
                [[&lhs0, &rhs0], [&lhs1, &rhs1], [&lhs2, &rhs2]],
            )
        };
        let product_sum4 = |lhs0: &Real,
                            rhs0: &Real,
                            lhs1: &Real,
                            rhs1: &Real,
                            lhs2: &Real,
                            rhs2: &Real,
                            lhs3: &Real,
                            rhs3: &Real,
                            signal: &AbortSignal| {
            // The dense active-abort vec4 dot is a four-term exact polynomial.
            // Keeping all terms in the Real reducer avoids four separate
            // guarded products and three immediate additions, preserving shared
            // denominator/canonicalization work until the fused sum. This is the
            // same fraction-free/delayed-normalization idea used by Bareiss,
            // Math. Comp. 22(103), 1968,
            // <https://doi.org/10.2307/2004533>.
            let lhs0 = clone_with_abort(lhs0, signal);
            let rhs0 = clone_with_abort(rhs0, signal);
            let lhs1 = clone_with_abort(lhs1, signal);
            let rhs1 = clone_with_abort(rhs1, signal);
            let lhs2 = clone_with_abort(lhs2, signal);
            let rhs2 = clone_with_abort(rhs2, signal);
            let lhs3 = clone_with_abort(lhs3, signal);
            let rhs3 = clone_with_abort(rhs3, signal);
            Real::active_signed_product_sum2(
                [true, true, true, true],
                [
                    [&lhs0, &rhs0],
                    [&lhs1, &rhs1],
                    [&lhs2, &rhs2],
                    [&lhs3, &rhs3],
                ],
            )
        };

        if nonzero == 1 {
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot4-sparse-single");
            return if has0 {
                product(&self.0[0], &rhs.0[0], signal)
            } else if has1 {
                product(&self.0[1], &rhs.0[1], signal)
            } else if has2 {
                product(&self.0[2], &rhs.0[2], signal)
            } else {
                product(&self.0[3], &rhs.0[3], signal)
            };
        }

        if nonzero == 2 {
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot4-sparse-two");
            return if has0 && has1 {
                product_sum2(&self.0[0], &rhs.0[0], &self.0[1], &rhs.0[1], signal)
            } else if has0 && has2 {
                product_sum2(&self.0[0], &rhs.0[0], &self.0[2], &rhs.0[2], signal)
            } else if has0 && has3 {
                product_sum2(&self.0[0], &rhs.0[0], &self.0[3], &rhs.0[3], signal)
            } else if has1 && has2 {
                product_sum2(&self.0[1], &rhs.0[1], &self.0[2], &rhs.0[2], signal)
            } else if has1 && has3 {
                product_sum2(&self.0[1], &rhs.0[1], &self.0[3], &rhs.0[3], signal)
            } else {
                product_sum2(&self.0[2], &rhs.0[2], &self.0[3], &rhs.0[3], signal)
            };
        }

        if nonzero == 3 {
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot4-sparse-three");
            return if !has0 {
                product_sum3(
                    &self.0[1], &rhs.0[1], &self.0[2], &rhs.0[2], &self.0[3], &rhs.0[3], signal,
                )
            } else if !has1 {
                product_sum3(
                    &self.0[0], &rhs.0[0], &self.0[2], &rhs.0[2], &self.0[3], &rhs.0[3], signal,
                )
            } else if !has2 {
                product_sum3(
                    &self.0[0], &rhs.0[0], &self.0[1], &rhs.0[1], &self.0[3], &rhs.0[3], signal,
                )
            } else {
                product_sum3(
                    &self.0[0], &rhs.0[0], &self.0[1], &rhs.0[1], &self.0[2], &rhs.0[2], signal,
                )
            };
        }

        crate::trace_dispatch!("hyperlattice_vector", "abort", "dot4-sparse-four");
        product_sum4(
            &self.0[0], &rhs.0[0], &self.0[1], &rhs.0[1], &self.0[2], &rhs.0[2], &self.0[3],
            &rhs.0[3], signal,
        )
    }
}
