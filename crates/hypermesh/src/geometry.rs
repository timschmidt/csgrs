//! Hyperreal planes, AABBs, and vector helpers.

use std::cmp::Ordering;

use hyperlattice::{Plane3Coefficients, Point3, ProjectivePlane3, Real};

use crate::error::HypermeshResult;
pub use crate::predicate::{
    Classification, classify_point, classify_projective_point, compare_real,
};
pub(crate) use crate::predicate::{PreparedPoint3, classify_real};

/// Exact plane `normal . point + offset = 0`.
#[derive(Clone, Debug, PartialEq)]
pub struct Plane {
    /// Plane normal coefficients.
    pub normal: Point3,
    /// Constant offset.
    pub offset: Real,
}

impl Plane {
    /// Constructs a plane from exact coefficients.
    pub const fn new(normal: Point3, offset: Real) -> Self {
        Self { normal, offset }
    }

    /// Constructs a plane from scalar coefficients.
    pub fn from_coefficients(a: Real, b: Real, c: Real, d: Real) -> Self {
        Self::new(Point3::new(a, b, c), d)
    }

    /// Constructs an axis-aligned plane `point[axis] - value = 0`.
    pub fn axis_aligned(axis: usize, value: Real) -> Self {
        let zero = Real::zero();
        let one = Real::one();
        let normal = match axis {
            0 => Point3::new(one, zero.clone(), zero),
            1 => Point3::new(zero.clone(), one, zero),
            2 => Point3::new(zero.clone(), zero, one),
            _ => panic!("axis must be 0, 1, or 2"),
        };
        Self::new(normal, -value)
    }

    /// Constructs the oriented plane through three affine points.
    pub fn from_points(p0: &Point3, p1: &Point3, p2: &Point3) -> Self {
        let u = sub_points(p1, p0);
        let v = sub_points(p2, p0);
        let normal = cross_arrays(&u, &v);
        let offset = -dot_point(&normal, p0);
        Self::new(normal, offset)
    }

    /// Returns this plane with all coefficients negated.
    pub fn inverted(&self) -> Self {
        Self::new(
            Point3::new(
                -self.normal.x.clone(),
                -self.normal.y.clone(),
                -self.normal.z.clone(),
            ),
            -self.offset.clone(),
        )
    }

    /// Returns the exact expression `normal . point + offset`.
    pub fn expression_at_point(&self, point: &Point3) -> Real {
        Real::signed_product_sum(
            [true, true, true, true],
            [
                [&self.normal.x, &point.x],
                [&self.normal.y, &point.y],
                [&self.normal.z, &point.z],
                [&self.offset, &Real::one()],
            ],
        )
    }

    /// Returns true when the normal is structurally known non-zero.
    pub fn is_valid(&self) -> bool {
        !(self.normal.x.definitely_zero()
            && self.normal.y.definitely_zero()
            && self.normal.z.definitely_zero())
    }

    /// Converts to hyperlattice's projective plane carrier.
    pub fn as_projective(&self) -> ProjectivePlane3 {
        ProjectivePlane3::new(self.normal.clone(), self.offset.clone())
    }

    /// Returns `(axis, value)` for planes of form `normal[axis] * x + d = 0`.
    pub fn axis_split_value(&self) -> Option<(usize, Real)> {
        let zero = Real::zero();
        for axis in 0..3 {
            let components = [&self.normal.x, &self.normal.y, &self.normal.z];
            if components
                .iter()
                .enumerate()
                .all(|(i, value)| i == axis || value.definitely_zero())
                && !components[axis].definitely_zero()
            {
                let value = ((&zero - &self.offset) / components[axis]).ok()?;
                return Some((axis, value));
            }
        }
        None
    }
}

impl Plane3Coefficients for Plane {
    fn normal(&self) -> &Point3 {
        &self.normal
    }

    fn offset(&self) -> &Real {
        &self.offset
    }
}

/// Hyperreal axis-aligned bounding box.
#[derive(Clone, Debug, PartialEq)]
pub struct Aabb {
    /// Minimum coordinate.
    pub min: Point3,
    /// Maximum coordinate.
    pub max: Point3,
}

impl Aabb {
    /// Constructs an AABB from exact endpoints.
    pub const fn new(min: Point3, max: Point3) -> Self {
        Self { min, max }
    }

    /// Returns the extent along one axis.
    pub fn extent(&self, axis: usize) -> Real {
        axis_ref(&self.max, axis) - axis_ref(&self.min, axis)
    }

    /// Returns the longest axis when exact comparisons can certify an order.
    pub fn longest_axis(&self) -> HypermeshResult<usize> {
        let ex = self.extent(0);
        let ey = self.extent(1);
        let ez = self.extent(2);
        if compare_real(&ex, &ey)? != Ordering::Less && compare_real(&ex, &ez)? != Ordering::Less {
            Ok(0)
        } else if compare_real(&ey, &ez)? != Ordering::Less {
            Ok(1)
        } else {
            Ok(2)
        }
    }

    /// Returns the midpoint along one axis.
    pub fn midpoint(&self, axis: usize) -> Real {
        ((axis_ref(&self.min, axis) + axis_ref(&self.max, axis)) / Real::from(2))
            .expect("division by literal two is always valid")
    }

    /// Creates a splitting plane at the midpoint of the selected axis.
    pub fn splitting_plane(&self, axis: usize) -> Plane {
        Plane::axis_aligned(axis, self.midpoint(axis))
    }

    /// Returns true when `point` lies inside the closed AABB.
    pub fn contains_point(&self, point: &Point3) -> HypermeshResult<bool> {
        for axis in 0..3 {
            if compare_real(axis_ref(point, axis), axis_ref(&self.min, axis))?.is_lt()
                || compare_real(axis_ref(point, axis), axis_ref(&self.max, axis))?.is_gt()
            {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// Returns the left half, with `max[axis]` clamped to `value`.
    pub fn left_half(&self, axis: usize, value: Real) -> Self {
        let mut max = self.max.clone();
        *axis_mut(&mut max, axis) = value;
        Self::new(self.min.clone(), max)
    }

    /// Returns the right half, with `min[axis]` clamped to `value`.
    pub fn right_half(&self, axis: usize, value: Real) -> Self {
        let mut min = self.min.clone();
        *axis_mut(&mut min, axis) = value;
        Self::new(min, self.max.clone())
    }
}

pub(crate) fn axis_ref(point: &Point3, axis: usize) -> &Real {
    match axis {
        0 => &point.x,
        1 => &point.y,
        2 => &point.z,
        _ => panic!("axis must be 0, 1, or 2"),
    }
}

pub(crate) fn axis_mut(point: &mut Point3, axis: usize) -> &mut Real {
    match axis {
        0 => &mut point.x,
        1 => &mut point.y,
        2 => &mut point.z,
        _ => panic!("axis must be 0, 1, or 2"),
    }
}

pub(crate) fn dot_point(left: &Point3, right: &Point3) -> Real {
    Real::signed_product_sum(
        [true, true, true],
        [
            [&left.x, &right.x],
            [&left.y, &right.y],
            [&left.z, &right.z],
        ],
    )
}

pub(crate) fn sub_points(left: &Point3, right: &Point3) -> [Real; 3] {
    [&left.x - &right.x, &left.y - &right.y, &left.z - &right.z]
}

pub(crate) fn cross_arrays(left: &[Real; 3], right: &[Real; 3]) -> Point3 {
    Point3::new(
        (&left[1] * &right[2]) - (&left[2] * &right[1]),
        (&left[2] * &right[0]) - (&left[0] * &right[2]),
        (&left[0] * &right[1]) - (&left[1] * &right[0]),
    )
}
