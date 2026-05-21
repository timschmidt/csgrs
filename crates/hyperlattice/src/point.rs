//! Exact point carriers and point-level structural facts.
//!
//! Points are deliberately separate from vectors: both store [`Real`]
//! coordinates, but points carry geometric-object facts such as common-scale
//! coordinate views and one-hot support. Those facts are algebraic scheduling
//! metadata for downstream predicates, not topology decisions.

use crate::{ExactRealSetFacts, Real, RealSymbolicDependencyMask, Vector2, Vector3, ZeroStatus};

/// Borrowed view of point coordinates that share one exact rational scale.
#[derive(Clone, Copy, Debug)]
pub struct PointSharedScaleView<'a, const N: usize> {
    coordinates: [&'a Real; N],
    /// Exact-rational facts for the borrowed coordinates.
    pub exact: ExactRealSetFacts,
    /// Bit mask of coordinates known to be exactly zero.
    pub known_zero_mask: u128,
    /// Bit mask of coordinates known to be nonzero.
    pub known_nonzero_mask: u128,
    /// Bit mask of coordinates whose zero status is unknown.
    pub unknown_zero_mask: u128,
}

impl<'a, const N: usize> PointSharedScaleView<'a, N> {
    /// Attempts to build a borrowed shared-scale coordinate view.
    pub fn from_coordinates(coordinates: [&'a Real; N]) -> Option<Self> {
        crate::trace_dispatch!(
            "hyperlattice_point",
            "query",
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

    /// Returns the retained common-scale fact packet for this view.
    pub fn facts(self) -> PointSharedScaleFacts<N> {
        PointSharedScaleFacts {
            exact: self.exact,
            known_zero_mask: self.known_zero_mask,
            known_nonzero_mask: self.known_nonzero_mask,
            unknown_zero_mask: self.unknown_zero_mask,
        }
    }

    /// Returns true when every coordinate is structurally known zero.
    pub fn is_known_zero(self) -> bool {
        self.facts().is_known_zero()
    }

    /// Returns true when every coordinate is structurally known nonzero.
    pub fn is_known_dense(self) -> bool {
        self.facts().is_known_dense()
    }

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
}

/// Conservative fact packet for a point whose coordinates share one scale.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PointSharedScaleFacts<const N: usize> {
    /// Exact-rational facts for the coordinates.
    pub exact: ExactRealSetFacts,
    /// Bit mask of coordinates known to be exactly zero.
    pub known_zero_mask: u128,
    /// Bit mask of coordinates known to be nonzero.
    pub known_nonzero_mask: u128,
    /// Bit mask of coordinates whose zero status is unknown.
    pub unknown_zero_mask: u128,
}

impl<const N: usize> PointSharedScaleFacts<N> {
    /// Returns true when every coordinate is structurally known zero.
    pub fn is_known_zero(self) -> bool {
        self.known_zero_mask == coordinate_mask::<N>()
    }

    /// Returns true when every coordinate is structurally known nonzero.
    pub fn is_known_dense(self) -> bool {
        self.known_nonzero_mask == coordinate_mask::<N>()
    }

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
}

/// Cheap structural facts known for a [`Point2`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Point2Facts {
    /// Exact-rational representation facts for the coordinate set.
    pub exact: ExactRealSetFacts,
    /// Union of scalar symbolic dependency families across all coordinates.
    pub symbolic_dependencies: RealSymbolicDependencyMask,
    /// Bit mask of coordinates known to be exactly zero.
    pub known_zero_mask: u8,
    /// Bit mask of coordinates known to be nonzero.
    pub known_nonzero_mask: u8,
    /// Bit mask of coordinates whose zero status is unknown.
    pub unknown_zero_mask: u8,
    /// Bit mask of coordinates known to be exactly one.
    pub one_mask: u8,
    /// Coordinate index of a known one-hot point.
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
    pub fn has_unknown_zero(self) -> bool {
        self.unknown_zero_mask != 0
    }

    /// Returns whether this point is structurally one-hot.
    pub fn is_one_hot(self) -> bool {
        self.known_axis_index.is_some()
    }

    /// Returns whether this point has certified sparse coordinate support.
    pub fn has_sparse_support(self) -> bool {
        self.known_zero || self.is_one_hot()
    }
}

/// Cheap structural facts known for a [`Point3`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Point3Facts {
    /// Exact-rational representation facts for the coordinate set.
    pub exact: ExactRealSetFacts,
    /// Union of scalar symbolic dependency families across all coordinates.
    pub symbolic_dependencies: RealSymbolicDependencyMask,
    /// Bit mask of coordinates known to be exactly zero.
    pub known_zero_mask: u8,
    /// Bit mask of coordinates known to be nonzero.
    pub known_nonzero_mask: u8,
    /// Bit mask of coordinates whose zero status is unknown.
    pub unknown_zero_mask: u8,
    /// Bit mask of coordinates known to be exactly one.
    pub one_mask: u8,
    /// Coordinate index of a known one-hot point.
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
    pub fn has_unknown_zero(self) -> bool {
        self.unknown_zero_mask != 0
    }

    /// Returns whether this point is structurally one-hot.
    pub fn is_one_hot(self) -> bool {
        self.known_axis_index.is_some()
    }

    /// Returns whether this point has certified sparse coordinate support.
    pub fn has_sparse_support(self) -> bool {
        self.known_zero || self.is_one_hot()
    }
}

/// 2D point with exact [`Real`] coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct Point2 {
    /// X coordinate.
    pub x: Real,
    /// Y coordinate.
    pub y: Real,
}

impl Point2 {
    /// Constructs a 2D point from coordinates.
    pub const fn new(x: Real, y: Real) -> Self {
        Self { x, y }
    }

    /// Returns this point as a vector with the same coordinate values.
    pub fn to_vector(&self) -> Vector2 {
        Vector2::new([self.x.clone(), self.y.clone()])
    }

    /// Consumes this point into a vector with the same coordinate values.
    pub fn into_vector(self) -> Vector2 {
        Vector2::new([self.x, self.y])
    }

    /// Returns a borrowed shared-scale view of the point coordinates.
    pub fn shared_scale_view(&self) -> Option<PointSharedScaleView<'_, 2>> {
        PointSharedScaleView::from_coordinates([&self.x, &self.y])
    }

    /// Returns cheap structural facts for this point.
    pub fn structural_facts(&self) -> Point2Facts {
        crate::trace_dispatch!("hyperlattice_point", "query", "point2-structural-facts");
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

impl From<Vector2> for Point2 {
    fn from(value: Vector2) -> Self {
        Self::new(value.0[0].clone(), value.0[1].clone())
    }
}

impl From<Point2> for Vector2 {
    fn from(value: Point2) -> Self {
        value.into_vector()
    }
}

/// 3D point with exact [`Real`] coordinates.
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
    /// Constructs a 3D point from coordinates.
    pub const fn new(x: Real, y: Real, z: Real) -> Self {
        Self { x, y, z }
    }

    /// Returns this point as a vector with the same coordinate values.
    pub fn to_vector(&self) -> Vector3 {
        Vector3::new([self.x.clone(), self.y.clone(), self.z.clone()])
    }

    /// Consumes this point into a vector with the same coordinate values.
    pub fn into_vector(self) -> Vector3 {
        Vector3::new([self.x, self.y, self.z])
    }

    /// Returns a borrowed shared-scale view of the point coordinates.
    pub fn shared_scale_view(&self) -> Option<PointSharedScaleView<'_, 3>> {
        PointSharedScaleView::from_coordinates([&self.x, &self.y, &self.z])
    }

    /// Returns cheap structural facts for this point.
    pub fn structural_facts(&self) -> Point3Facts {
        crate::trace_dispatch!("hyperlattice_point", "query", "point3-structural-facts");
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

impl From<Vector3> for Point3 {
    fn from(value: Vector3) -> Self {
        Self::new(value.0[0].clone(), value.0[1].clone(), value.0[2].clone())
    }
}

impl From<Point3> for Vector3 {
    fn from(value: Point3) -> Self {
        value.into_vector()
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
        match coordinate.zero_status() {
            ZeroStatus::Zero => known_zero_mask |= bit,
            ZeroStatus::NonZero => known_nonzero_mask |= bit,
            ZeroStatus::Unknown => unknown_zero_mask |= bit,
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
