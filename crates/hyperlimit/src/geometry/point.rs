use crate::RealSymbolicDependencyMask;
use hyperreal::{Real, RealExactSetFacts, ZeroKnowledge};

/// Borrowed view of point coordinates that share one exact rational scale.
///
/// The view carries only conservative facts and coordinate references:
/// denominator kind, coarse rational storage class, and zero/nonzero masks. It
/// does not expose numerators or denominators. That keeps scalar representation
/// ownership in `hyperreal` while allowing predicate objects to retain the
/// common-scale signal Yap identifies as important for exact geometric
/// computation. See Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug)]
pub struct PointSharedScaleView<'a, const N: usize> {
    coordinates: [&'a Real; N],
    /// Exact-rational facts for the borrowed coordinates.
    pub exact: RealExactSetFacts,
    /// Bit mask of coordinates known to be exactly zero.
    pub known_zero_mask: u128,
    /// Bit mask of coordinates known to be nonzero.
    pub known_nonzero_mask: u128,
    /// Bit mask of coordinates whose zero status is unknown.
    pub unknown_zero_mask: u128,
}

impl<'a, const N: usize> PointSharedScaleView<'a, N> {
    /// Attempts to build a borrowed shared-scale coordinate view.
    ///
    /// Returns `None` unless every coordinate is an exact rational and all
    /// reduced denominators match. Empty coordinate lists are rejected by
    /// [`RealExactSetFacts::has_shared_denominator_schedule`], since there is
    /// no concrete denominator schedule to carry.
    pub fn from_coordinates(coordinates: [&'a Real; N]) -> Option<Self> {
        crate::trace_dispatch!(
            "hyperlimit",
            "geometry",
            "point-shared-scale-view-from-coordinates"
        );
        let exact = Real::exact_set_facts(coordinates.iter().copied());
        if !exact.has_shared_denominator_schedule() {
            return None;
        }
        let (known_zero_mask, known_nonzero_mask, unknown_zero_mask) =
            coordinate_zero_status_masks(coordinates);
        Some(Self {
            coordinates,
            exact,
            known_zero_mask,
            known_nonzero_mask,
            unknown_zero_mask,
        })
    }

    /// Returns the borrowed coordinates.
    pub fn coordinates(self) -> [&'a Real; N] {
        self.coordinates
    }

    /// Returns the number of coordinates.
    pub const fn len(self) -> usize {
        N
    }

    /// Returns whether this view contains no coordinates.
    pub const fn is_empty(self) -> bool {
        N == 0
    }

    /// Returns true when every coordinate is structurally known zero.
    pub fn is_known_zero(self) -> bool {
        self.known_zero_mask == coordinate_mask::<N>()
    }

    /// Returns true when every coordinate is structurally known nonzero.
    pub fn is_known_dense(self) -> bool {
        self.known_nonzero_mask == coordinate_mask::<N>()
    }

    /// Counts coordinates known to be exactly zero.
    ///
    /// This helper keeps callers from depending on the mask layout while still
    /// allowing sparse predicate and triangulation kernels to consume retained
    /// coordinate structure. The design follows Yap's exact-geometric-
    /// computation guidance: preserve inexpensive object facts for arithmetic
    /// package selection, but keep geometric decisions in certified predicates.
    /// See Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7.1-2 (1997).
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
}

#[inline]
fn coordinate_mask<const N: usize>() -> u128 {
    debug_assert!(N <= u128::BITS as usize);
    if N == u128::BITS as usize {
        u128::MAX
    } else {
        (1_u128 << N) - 1
    }
}

#[inline]
fn coordinate_zero_status_masks<const N: usize>(coordinates: [&Real; N]) -> (u128, u128, u128) {
    let mut known_zero_mask = 0_u128;
    let mut known_nonzero_mask = 0_u128;
    let mut unknown_zero_mask = 0_u128;
    for (index, coordinate) in coordinates.into_iter().enumerate() {
        let bit = 1_u128 << index;
        match coordinate.structural_facts().zero {
            ZeroKnowledge::Zero => known_zero_mask |= bit,
            ZeroKnowledge::NonZero => known_nonzero_mask |= bit,
            ZeroKnowledge::Unknown => unknown_zero_mask |= bit,
        }
    }
    (known_zero_mask, known_nonzero_mask, unknown_zero_mask)
}

#[inline]
fn coordinate_one_mask<const N: usize>(coordinates: [&Real; N]) -> u128 {
    let mut mask = 0_u128;
    for (index, coordinate) in coordinates.into_iter().enumerate() {
        if coordinate.definitely_one() {
            mask |= 1_u128 << index;
        }
    }
    mask
}

#[inline]
fn coordinate_symbolic_dependency_mask<const N: usize>(
    coordinates: [&Real; N],
) -> RealSymbolicDependencyMask {
    coordinates
        .into_iter()
        .fold(RealSymbolicDependencyMask::NONE, |mask, coordinate| {
            mask.union(coordinate.detailed_facts().symbolic.dependencies)
        })
}

#[inline]
fn single_bit_index(mask: u128) -> Option<usize> {
    if mask.count_ones() == 1 {
        Some(mask.trailing_zeros() as usize)
    } else {
        None
    }
}

/// Cheap structural facts known for a [`Point2`].
///
/// These facts live in `hyperlimit` because predicate preparation needs point
/// metadata without depending on `hyperlattice` vector types. They are
/// conservative coordinate facts only: they help choose exact arithmetic
/// packages, sparse determinant schedules, and future common-scale kernels, but
/// they do not decide orientation, incidence, or containment by themselves.
/// This follows Yap's exact-geometric-computation discipline to preserve
/// object structure before scalar expansion; see Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Point2Facts {
    /// Exact-rational representation facts for the coordinate set.
    pub exact: RealExactSetFacts,
    /// Union of scalar symbolic dependency families across all coordinates.
    ///
    /// Predicate preparation can use this storage-free summary to choose
    /// symbolic-aware exact arithmetic routes without depending on `Real`'s
    /// private class representation. It is not an orientation, incidence, or
    /// containment certificate. That separation follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997).
    pub symbolic_dependencies: RealSymbolicDependencyMask,
    /// Bit mask of coordinates known to be exactly zero.
    pub known_zero_mask: u8,
    /// Bit mask of coordinates known to be nonzero.
    pub known_nonzero_mask: u8,
    /// Bit mask of coordinates whose zero status is unknown.
    pub unknown_zero_mask: u8,
    /// Bit mask of coordinates known to be exactly one.
    pub one_mask: u8,
    /// Coordinate index of a known one-hot point, when exactly one coordinate is
    /// nonzero and the other coordinate is known zero.
    pub known_axis_index: Option<usize>,
    /// Whether all coordinates are known zero.
    pub known_zero: bool,
}

impl Point2Facts {
    /// Counts coordinates known to be exactly zero.
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

    /// Returns whether any coordinate has unknown zero status.
    ///
    /// Unknown-zero coordinates must block sparse determinant schedules that
    /// require certified support. Keeping that uncertainty explicit is part of
    /// Yap's exact-geometric-computation boundary: facts can select exact
    /// arithmetic packages, but they must not silently decide topology. See
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7.1-2 (1997).
    pub fn has_unknown_zero(self) -> bool {
        self.unknown_zero_mask != 0
    }

    /// Returns whether this point is structurally one-hot.
    ///
    /// One-hot means exactly one coordinate is known nonzero and every other
    /// coordinate is known zero. The nonzero coordinate need not be `1`.
    pub fn is_one_hot(self) -> bool {
        self.known_axis_index.is_some()
    }

    /// Returns whether this point has certified sparse coordinate support.
    ///
    /// The origin and one-hot points are useful sparse determinant candidates.
    /// This is only a scheduling fact; orientation and incidence still require
    /// certified predicates.
    pub fn has_sparse_support(self) -> bool {
        self.known_zero || self.is_one_hot()
    }
}

/// Cheap structural facts known for a [`Point3`].
///
/// The 3D form mirrors [`Point2Facts`] and stays storage-free: scalar
/// denominator and numerator details remain owned by `hyperreal`, while
/// predicate and plane preparation can carry coordinate masks and exact-set
/// summaries. The retained-object strategy follows Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Point3Facts {
    /// Exact-rational representation facts for the coordinate set.
    pub exact: RealExactSetFacts,
    /// Union of scalar symbolic dependency families across all coordinates.
    ///
    /// Plane, sphere, and batch predicate preparation can retain this point
    /// fact while keeping scalar expression ownership in `hyperreal`.
    pub symbolic_dependencies: RealSymbolicDependencyMask,
    /// Bit mask of coordinates known to be exactly zero.
    pub known_zero_mask: u8,
    /// Bit mask of coordinates known to be nonzero.
    pub known_nonzero_mask: u8,
    /// Bit mask of coordinates whose zero status is unknown.
    pub unknown_zero_mask: u8,
    /// Bit mask of coordinates known to be exactly one.
    pub one_mask: u8,
    /// Coordinate index of a known one-hot point, when exactly one coordinate is
    /// nonzero and every other coordinate is known zero.
    pub known_axis_index: Option<usize>,
    /// Whether all coordinates are known zero.
    pub known_zero: bool,
}

impl Point3Facts {
    /// Counts coordinates known to be exactly zero.
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

    /// Returns whether any coordinate has unknown zero status.
    ///
    /// This conservative query lets prepared 3D predicates avoid sparse
    /// determinant schedules when support is incomplete. The fact/query split
    /// follows Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7.1-2 (1997).
    pub fn has_unknown_zero(self) -> bool {
        self.unknown_zero_mask != 0
    }

    /// Returns whether this point is structurally one-hot.
    ///
    /// One-hot means exactly one coordinate is known nonzero and every other
    /// coordinate is known zero. The nonzero coordinate may have any exact or
    /// symbolic nonzero value.
    pub fn is_one_hot(self) -> bool {
        self.known_axis_index.is_some()
    }

    /// Returns whether this point has certified sparse coordinate support.
    ///
    /// The origin and one-hot points are useful candidates for future sparse
    /// plane, sphere, and determinant kernels, but this remains only an
    /// arithmetic scheduling fact.
    pub fn has_sparse_support(self) -> bool {
        self.known_zero || self.is_one_hot()
    }
}

/// 2D point with Real coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct Point2 {
    /// X coordinate.
    pub x: Real,
    /// Y coordinate.
    pub y: Real,
}

impl Point2 {
    /// Construct a 2D point from coordinates.
    pub const fn new(x: Real, y: Real) -> Self {
        Self { x, y }
    }

    /// Returns a borrowed shared-scale view of the point coordinates.
    ///
    /// Predicate preparation can carry this view to select future
    /// shared-denominator determinant schedules without exposing scalar
    /// rational storage. The view is only a scheduling fact; orientation and
    /// incidence decisions still go through certified predicates.
    pub fn shared_scale_view(&self) -> Option<PointSharedScaleView<'_, 2>> {
        PointSharedScaleView::from_coordinates([&self.x, &self.y])
    }

    /// Returns cheap structural facts for this point.
    ///
    /// Predicate code can use these facts to select exact sparse or
    /// shared-denominator schedules without re-reading every scalar coordinate.
    /// The result is not a topology predicate; callers must still use certified
    /// predicate functions for orientation, incidence, and containment.
    pub fn structural_facts(&self) -> Point2Facts {
        crate::trace_dispatch!("hyperlimit", "geometry", "point2-structural-facts");
        let coordinates = [&self.x, &self.y];
        let (known_zero_mask, known_nonzero_mask, unknown_zero_mask) =
            coordinate_zero_status_masks(coordinates);
        Point2Facts {
            exact: Real::exact_set_facts(coordinates),
            symbolic_dependencies: coordinate_symbolic_dependency_mask(coordinates),
            known_zero_mask: known_zero_mask as u8,
            known_nonzero_mask: known_nonzero_mask as u8,
            unknown_zero_mask: unknown_zero_mask as u8,
            one_mask: coordinate_one_mask(coordinates) as u8,
            known_axis_index: if known_zero_mask.count_ones() == 1
                && known_nonzero_mask.count_ones() == 1
                && unknown_zero_mask == 0
            {
                single_bit_index(known_nonzero_mask)
            } else {
                None
            },
            known_zero: known_zero_mask == coordinate_mask::<2>(),
        }
    }
}

/// 3D point with Real coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct Point3 {
    /// X coordinate.
    pub x: Real,
    /// Y coordinate.
    pub y: Real,
    /// Z coordinate.
    pub z: Real,
}

impl Point3 {
    /// Construct a 3D point from coordinates.
    pub const fn new(x: Real, y: Real, z: Real) -> Self {
        Self { x, y, z }
    }

    /// Returns a borrowed shared-scale view of the point coordinates.
    ///
    /// This is the 3D coordinate analogue of
    /// [`Point2::shared_scale_view`], preserving common-scale metadata at the
    /// predicate geometry boundary while keeping exact arithmetic ownership in
    /// `hyperreal`.
    pub fn shared_scale_view(&self) -> Option<PointSharedScaleView<'_, 3>> {
        PointSharedScaleView::from_coordinates([&self.x, &self.y, &self.z])
    }

    /// Returns cheap structural facts for this point.
    ///
    /// This method keeps point-coordinate masks at the predicate geometry
    /// boundary. It gives plane, sphere, and batch predicate preparation code a
    /// stable fact package while leaving all scalar representation details in
    /// `hyperreal`.
    pub fn structural_facts(&self) -> Point3Facts {
        crate::trace_dispatch!("hyperlimit", "geometry", "point3-structural-facts");
        let coordinates = [&self.x, &self.y, &self.z];
        let (known_zero_mask, known_nonzero_mask, unknown_zero_mask) =
            coordinate_zero_status_masks(coordinates);
        Point3Facts {
            exact: Real::exact_set_facts(coordinates),
            symbolic_dependencies: coordinate_symbolic_dependency_mask(coordinates),
            known_zero_mask: known_zero_mask as u8,
            known_nonzero_mask: known_nonzero_mask as u8,
            unknown_zero_mask: unknown_zero_mask as u8,
            one_mask: coordinate_one_mask(coordinates) as u8,
            known_axis_index: if known_zero_mask.count_ones() == 2
                && known_nonzero_mask.count_ones() == 1
                && unknown_zero_mask == 0
            {
                single_bit_index(known_nonzero_mask)
            } else {
                None
            },
            known_zero: known_zero_mask == coordinate_mask::<3>(),
        }
    }
}
