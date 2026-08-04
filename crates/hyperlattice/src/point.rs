//! Exact point carriers and point-level structural facts.
//!
//! Points are deliberately separate from vectors: both store [`Real`]
//! coordinates, but points carry geometric-object facts such as common-scale
//! coordinate views and one-hot support. Those facts are algebraic scheduling
//! metadata for downstream predicates, not topology decisions.

use crate::{
    BlasResult, Real, RealExactSetFacts, RealSymbolicDependencyMask, Vector2, Vector3, ZeroStatus,
};
use std::ops::{Add, Sub};

/// Borrowed view of point coordinates that share one exact rational scale.
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
    pub exact: RealExactSetFacts,
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
    pub exact: RealExactSetFacts,
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
    pub exact: RealExactSetFacts,
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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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

    /// Returns the origin point.
    pub fn origin() -> Self {
        Self::new(Real::zero(), Real::zero())
    }

    /// Lifts a finite `f64` coordinate array into a hyperreal-backed point.
    pub fn try_from_f64_array(values: [f64; 2]) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_point", "constructor", "point2-from-f64-array");
        Ok(Self::new(
            Real::try_from(values[0])?,
            Real::try_from(values[1])?,
        ))
    }

    /// Lifts a finite `f32` coordinate array into a hyperreal-backed point.
    pub fn try_from_f32_array(values: [f32; 2]) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_point", "constructor", "point2-from-f32-array");
        Ok(Self::new(
            Real::try_from(values[0])?,
            Real::try_from(values[1])?,
        ))
    }

    /// Lossily exports this point to finite `f64` coordinates.
    pub fn to_f64_array_lossy(&self) -> Option<[f64; 2]> {
        crate::trace_dispatch!("hyperlattice_point", "export", "point2-to-f64-array-lossy");
        Some([
            self.x.to_f64_lossy().filter(|value| value.is_finite())?,
            self.y.to_f64_lossy().filter(|value| value.is_finite())?,
        ])
    }

    /// Lossily exports this point to finite `f32` coordinates.
    pub fn to_f32_array_lossy(&self) -> Option<[f32; 2]> {
        crate::trace_dispatch!("hyperlattice_point", "export", "point2-to-f32-array-lossy");
        Some([
            self.x.to_f32_lossy().filter(|value| value.is_finite())?,
            self.y.to_f32_lossy().filter(|value| value.is_finite())?,
        ])
    }

    /// Returns this point as a vector with the same coordinate values.
    pub fn to_vector(&self) -> Vector2 {
        Vector2::new([self.x.clone(), self.y.clone()])
    }

    /// Consumes this point into a vector with the same coordinate values.
    pub fn into_vector(self) -> Vector2 {
        Vector2::new([self.x, self.y])
    }

    /// Interpolates between two points as `self + t * (to - self)`.
    pub fn lerp(&self, to: &Self, t: &Real) -> Self {
        crate::trace_dispatch!("hyperlattice_point", "method", "point2-lerp");
        Vector2::from(self.clone())
            .lerp(&Vector2::from(to.clone()), t)
            .into()
    }

    /// Returns the centroid of a non-empty point slice.
    pub fn centroid(points: &[Self]) -> Option<Self> {
        crate::trace_dispatch!("hyperlattice_point", "method", "point2-centroid");
        let vectors = points
            .iter()
            .cloned()
            .map(Vector2::from)
            .collect::<Vec<_>>();
        Vector2::mean(&vectors).map(Self::from)
    }

    /// Returns a weighted point sum.
    ///
    /// Weight normalization belongs to the caller.
    pub fn weighted_sum(points: &[Self], weights: &[Real]) -> Option<Self> {
        crate::trace_dispatch!("hyperlattice_point", "method", "point2-weighted-sum");
        let vectors = points
            .iter()
            .cloned()
            .map(Vector2::from)
            .collect::<Vec<_>>();
        Vector2::weighted_sum(&vectors, weights).map(Self::from)
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

    /// Returns the origin point.
    pub fn origin() -> Self {
        Self::new(Real::zero(), Real::zero(), Real::zero())
    }

    /// Lifts a finite `f64` coordinate array into a hyperreal-backed point.
    pub fn try_from_f64_array(values: [f64; 3]) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_point", "constructor", "point3-from-f64-array");
        Ok(Self::new(
            Real::try_from(values[0])?,
            Real::try_from(values[1])?,
            Real::try_from(values[2])?,
        ))
    }

    /// Lifts a finite `f32` coordinate array into a hyperreal-backed point.
    pub fn try_from_f32_array(values: [f32; 3]) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_point", "constructor", "point3-from-f32-array");
        Ok(Self::new(
            Real::try_from(values[0])?,
            Real::try_from(values[1])?,
            Real::try_from(values[2])?,
        ))
    }

    /// Lossily exports this point to finite `f64` coordinates.
    pub fn to_f64_array_lossy(&self) -> Option<[f64; 3]> {
        crate::trace_dispatch!("hyperlattice_point", "export", "point3-to-f64-array-lossy");
        Some([
            self.x.to_f64_lossy().filter(|value| value.is_finite())?,
            self.y.to_f64_lossy().filter(|value| value.is_finite())?,
            self.z.to_f64_lossy().filter(|value| value.is_finite())?,
        ])
    }

    /// Lossily exports this point to finite `f32` coordinates.
    pub fn to_f32_array_lossy(&self) -> Option<[f32; 3]> {
        crate::trace_dispatch!("hyperlattice_point", "export", "point3-to-f32-array-lossy");
        Some([
            self.x.to_f32_lossy().filter(|value| value.is_finite())?,
            self.y.to_f32_lossy().filter(|value| value.is_finite())?,
            self.z.to_f32_lossy().filter(|value| value.is_finite())?,
        ])
    }

    /// Returns this point as a vector with the same coordinate values.
    pub fn to_vector(&self) -> Vector3 {
        Vector3::new([self.x.clone(), self.y.clone(), self.z.clone()])
    }

    /// Consumes this point into a vector with the same coordinate values.
    pub fn into_vector(self) -> Vector3 {
        Vector3::new([self.x, self.y, self.z])
    }

    /// Interpolates between two points as `self + t * (to - self)`.
    pub fn lerp(&self, to: &Self, t: &Real) -> Self {
        crate::trace_dispatch!("hyperlattice_point", "method", "point3-lerp");
        Vector3::from(self.clone())
            .lerp(&Vector3::from(to.clone()), t)
            .into()
    }

    /// Returns the centroid of a non-empty point slice.
    pub fn centroid(points: &[Self]) -> Option<Self> {
        crate::trace_dispatch!("hyperlattice_point", "method", "point3-centroid");
        let vectors = points
            .iter()
            .cloned()
            .map(Vector3::from)
            .collect::<Vec<_>>();
        Vector3::mean(&vectors).map(Self::from)
    }

    /// Returns a weighted point sum.
    ///
    /// Weight normalization belongs to the caller.
    pub fn weighted_sum(points: &[Self], weights: &[Real]) -> Option<Self> {
        crate::trace_dispatch!("hyperlattice_point", "method", "point3-weighted-sum");
        let vectors = points
            .iter()
            .cloned()
            .map(Vector3::from)
            .collect::<Vec<_>>();
        Vector3::weighted_sum(&vectors, weights).map(Self::from)
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

impl Add<Vector2> for Point2 {
    type Output = Self;

    fn add(self, rhs: Vector2) -> Self::Output {
        let [x, y] = rhs.0;
        Self::new(self.x + x, self.y + y)
    }
}

impl Add<&Vector2> for Point2 {
    type Output = Self;

    fn add(self, rhs: &Vector2) -> Self::Output {
        Self::new(self.x + rhs.0[0].clone(), self.y + rhs.0[1].clone())
    }
}

impl Sub<Vector2> for Point2 {
    type Output = Self;

    fn sub(self, rhs: Vector2) -> Self::Output {
        let [x, y] = rhs.0;
        Self::new(self.x - x, self.y - y)
    }
}

impl Sub<&Vector2> for Point2 {
    type Output = Self;

    fn sub(self, rhs: &Vector2) -> Self::Output {
        Self::new(self.x - rhs.0[0].clone(), self.y - rhs.0[1].clone())
    }
}

impl Sub for Point2 {
    type Output = Vector2;

    fn sub(self, rhs: Self) -> Self::Output {
        Vector2::new([self.x - rhs.x, self.y - rhs.y])
    }
}

impl Sub<&Point2> for Point2 {
    type Output = Vector2;

    fn sub(self, rhs: &Point2) -> Self::Output {
        Vector2::new([self.x - rhs.x.clone(), self.y - rhs.y.clone()])
    }
}

impl Sub<Point2> for &Point2 {
    type Output = Vector2;

    fn sub(self, rhs: Point2) -> Self::Output {
        Vector2::new([self.x.clone() - rhs.x, self.y.clone() - rhs.y])
    }
}

impl Sub<&Point2> for &Point2 {
    type Output = Vector2;

    fn sub(self, rhs: &Point2) -> Self::Output {
        Vector2::new([&self.x - &rhs.x, &self.y - &rhs.y])
    }
}

impl Add<Vector3> for Point3 {
    type Output = Self;

    fn add(self, rhs: Vector3) -> Self::Output {
        let [x, y, z] = rhs.0;
        Self::new(self.x + x, self.y + y, self.z + z)
    }
}

impl Add<&Vector3> for Point3 {
    type Output = Self;

    fn add(self, rhs: &Vector3) -> Self::Output {
        Self::new(
            self.x + rhs.0[0].clone(),
            self.y + rhs.0[1].clone(),
            self.z + rhs.0[2].clone(),
        )
    }
}

impl Sub<Vector3> for Point3 {
    type Output = Self;

    fn sub(self, rhs: Vector3) -> Self::Output {
        let [x, y, z] = rhs.0;
        Self::new(self.x - x, self.y - y, self.z - z)
    }
}

impl Sub<&Vector3> for Point3 {
    type Output = Self;

    fn sub(self, rhs: &Vector3) -> Self::Output {
        Self::new(
            self.x - rhs.0[0].clone(),
            self.y - rhs.0[1].clone(),
            self.z - rhs.0[2].clone(),
        )
    }
}

impl Sub for Point3 {
    type Output = Vector3;

    fn sub(self, rhs: Self) -> Self::Output {
        Vector3::new([self.x - rhs.x, self.y - rhs.y, self.z - rhs.z])
    }
}

impl Sub<&Point3> for Point3 {
    type Output = Vector3;

    fn sub(self, rhs: &Point3) -> Self::Output {
        Vector3::new([
            self.x - rhs.x.clone(),
            self.y - rhs.y.clone(),
            self.z - rhs.z.clone(),
        ])
    }
}

impl Sub<Point3> for &Point3 {
    type Output = Vector3;

    fn sub(self, rhs: Point3) -> Self::Output {
        Vector3::new([
            self.x.clone() - rhs.x,
            self.y.clone() - rhs.y,
            self.z.clone() - rhs.z,
        ])
    }
}

impl Sub<&Point3> for &Point3 {
    type Output = Vector3;

    fn sub(self, rhs: &Point3) -> Self::Output {
        Vector3::new([&self.x - &rhs.x, &self.y - &rhs.y, &self.z - &rhs.z])
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

#[cfg(test)]
mod tests {
    use super::*;

    fn real(value: i32) -> Real {
        Real::from(value)
    }

    #[test]
    fn point_boundary_arrays_reject_nonfinite_values() {
        let point = Point3::try_from_f64_array([1.0, 2.0, 3.0]).unwrap();
        assert_eq!(point.to_f64_array_lossy(), Some([1.0, 2.0, 3.0]));
        assert!(Point3::try_from_f64_array([1.0, f64::NAN, 3.0]).is_err());

        let point = Point2::try_from_f32_array([1.5, 2.5]).unwrap();
        assert_eq!(point.to_f32_array_lossy(), Some([1.5, 2.5]));
    }

    #[test]
    fn point_centroid_weighted_sum_and_lerp_stay_in_real_coordinates() {
        let a = Point3::new(real(0), real(0), real(0));
        let b = Point3::new(real(4), real(8), real(12));
        let half = (Real::from(1_u32) / Real::from(2_u32)).unwrap();
        let mid = a.lerp(&b, &half);

        assert_eq!(mid.to_f64_array_lossy(), Some([2.0, 4.0, 6.0]));
        assert_eq!(
            Point3::centroid(&[a.clone(), b.clone()])
                .unwrap()
                .to_f64_array_lossy(),
            Some([2.0, 4.0, 6.0])
        );
        assert_eq!(
            Point3::weighted_sum(&[a, b], &[Real::from(1_u32), Real::from(2_u32)])
                .unwrap()
                .to_f64_array_lossy(),
            Some([8.0, 16.0, 24.0])
        );
    }
}
