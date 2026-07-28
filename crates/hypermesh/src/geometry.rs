//! Hyperreal planes, AABBs, and vector helpers.

use std::cmp::Ordering;

use hyperlattice::{Plane3Coefficients, Point3, ProjectivePlane3, Rational, Real, RealSign};

use crate::error::HypermeshResult;
pub use crate::predicate::{
    Classification, classify_point, classify_projective_point, compare_real,
};
pub(crate) use crate::predicate::{Point3PredicateEvidence, classify_real};

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
        if let [
            Some(x0),
            Some(y0),
            Some(z0),
            Some(x1),
            Some(y1),
            Some(z1),
            Some(x2),
            Some(y2),
            Some(z2),
        ] = [
            &p0.x, &p0.y, &p0.z, &p1.x, &p1.y, &p1.z, &p2.x, &p2.y, &p2.z,
        ]
        .map(Real::exact_rational_ref)
        {
            if let Some([x, y, z, offset]) = Rational::affine_plane3_coefficients_known_dyadic([
                [x0, y0, z0],
                [x1, y1, z1],
                [x2, y2, z2],
            ]) {
                return Self::new(
                    Point3::new(Real::from(x), Real::from(y), Real::from(z)),
                    Real::from(offset),
                );
            }
            let exact_dyadic = [x0, y0, z0, x1, y1, z1, x2, y2, z2]
                .into_iter()
                .all(Rational::is_dyadic);
            let signs = [true, false, false, false, true, true];
            let x_terms = [[y1, z2], [y1, z0], [y0, z2], [z1, y2], [z1, y0], [z0, y2]];
            let y_terms = [[z1, x2], [z1, x0], [z0, x2], [x1, z2], [x1, z0], [x0, z2]];
            let z_terms = [[x1, y2], [x1, y0], [x0, y2], [y1, x2], [y1, x0], [y0, x2]];
            let x = if exact_dyadic {
                Rational::signed_product_sum_known_dyadic(signs, x_terms)
            } else {
                Rational::signed_product_sum(signs, x_terms)
            };
            let y = if exact_dyadic {
                Rational::signed_product_sum_known_dyadic(signs, y_terms)
            } else {
                Rational::signed_product_sum(signs, y_terms)
            };
            let z = if exact_dyadic {
                Rational::signed_product_sum_known_dyadic(signs, z_terms)
            } else {
                Rational::signed_product_sum(signs, z_terms)
            };
            let offset_terms = [[&x, x0], [&y, y0], [&z, z0]];
            let offset = if exact_dyadic {
                Rational::signed_product_sum_known_dyadic([false; 3], offset_terms)
            } else {
                Rational::signed_product_sum([false; 3], offset_terms)
            };
            return Self::new(
                Point3::new(Real::from(x), Real::from(y), Real::from(z)),
                Real::from(offset),
            );
        }
        let u = sub_points(p1, p0);
        let v = sub_points(p2, p0);
        let normal = cross_arrays(&u, &v);
        let offset = -dot_point(&normal, p0);
        Self::new(normal, offset)
    }

    /// Returns whether three affine points structurally define a valid plane.
    ///
    /// This is the allocation-reduced validation counterpart of
    /// `Plane::from_points(...).is_valid()`: it evaluates cross-product
    /// components only until one is not structurally zero and does not build
    /// the unused plane offset.
    pub fn points_are_nondegenerate(p0: &Point3, p1: &Point3, p2: &Point3) -> bool {
        let coordinates = [
            [&p0.x, &p0.y, &p0.z],
            [&p1.x, &p1.y, &p1.z],
            [&p2.x, &p2.y, &p2.z],
        ];
        // A successful primitive filter is a certificate of the exact sign;
        // inconclusive projections continue to exact rational or symbolic
        // evaluation below.
        for [u, v] in [[1, 2], [2, 0], [0, 1]] {
            if matches!(
                Real::certified_affine_det2_sign(
                    [coordinates[0][u], coordinates[0][v]],
                    [coordinates[1][u], coordinates[1][v]],
                    [coordinates[2][u], coordinates[2][v]],
                ),
                Some(RealSign::Negative | RealSign::Positive)
            ) {
                return true;
            }
        }
        if let [
            [Some(x0), Some(y0), Some(z0)],
            [Some(x1), Some(y1), Some(z1)],
            [Some(x2), Some(y2), Some(z2)],
        ] = coordinates.map(|point| point.map(Real::exact_rational_ref))
        {
            let uy = y1 - y0;
            let uz = z1 - z0;
            let vy = y2 - y0;
            let vz = z2 - z0;
            if rational_difference_cross_is_nonzero(&uy, &vz, &uz, &vy) {
                return true;
            }
            let ux = x1 - x0;
            let vx = x2 - x0;
            return rational_difference_cross_is_nonzero(&uz, &vx, &ux, &vz)
                || rational_difference_cross_is_nonzero(&ux, &vy, &uy, &vx);
        }
        let uy = &p1.y - &p0.y;
        let uz = &p1.z - &p0.z;
        let vy = &p2.y - &p0.y;
        let vz = &p2.z - &p0.z;
        if difference_cross_is_nonzero(&uy, &vz, &uz, &vy) {
            return true;
        }
        let ux = &p1.x - &p0.x;
        let vx = &p2.x - &p0.x;
        difference_cross_is_nonzero(&uz, &vx, &ux, &vz)
            || difference_cross_is_nonzero(&ux, &vy, &uy, &vx)
    }

    pub(crate) fn points_are_collinear_on_support(
        &self,
        a: &Point3,
        b: &Point3,
        c: &Point3,
    ) -> bool {
        let normal = [&self.normal.x, &self.normal.y, &self.normal.z];
        let coordinates = [[&a.x, &a.y, &a.z], [&b.x, &b.y, &b.z], [&c.x, &c.y, &c.z]];
        for (axis, coefficient) in normal.into_iter().enumerate() {
            let Some(component) = coefficient.exact_rational_ref() else {
                continue;
            };
            if component.is_zero() {
                continue;
            }
            let u = (axis + 1) % 3;
            let v = (axis + 2) % 3;
            let [Some(au), Some(bu), Some(cu)] =
                coordinates.map(|point| point[u].exact_rational_ref())
            else {
                break;
            };
            let [Some(av), Some(bv), Some(cv)] =
                coordinates.map(|point| point[v].exact_rational_ref())
            else {
                break;
            };
            if (au == bu && bu == cu) || (av == bv && bv == cv) {
                return true;
            }
            return Rational::signed_product_sum_ordering(
                [true, true, true, false, false, false],
                [[au, bv], [bu, cv], [cu, av], [au, cv], [bu, av], [cu, bv]],
            ) == std::cmp::Ordering::Equal;
        }
        !Self::points_are_nondegenerate(a, b, c)
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
        for axis in 0..3 {
            let components = [&self.normal.x, &self.normal.y, &self.normal.z];
            if components
                .iter()
                .enumerate()
                .all(|(i, value)| i == axis || value.definitely_zero())
                && !components[axis].definitely_zero()
            {
                let value = -((&self.offset / components[axis]).ok()?);
                return Some((axis, value));
            }
        }
        None
    }
}

#[inline]
fn difference_cross_is_nonzero(
    left_a: &Real,
    left_b: &Real,
    right_a: &Real,
    right_b: &Real,
) -> bool {
    if let [Some(left_a), Some(left_b), Some(right_a), Some(right_b)] =
        [left_a, left_b, right_a, right_b].map(Real::exact_rational_ref)
    {
        return !Rational::signed_product_sum_ordering(
            [true, false],
            [[left_a, left_b], [right_a, right_b]],
        )
        .is_eq();
    }
    !Real::diff_of_products(left_a, left_b, right_a, right_b).definitely_zero()
}

#[inline]
fn rational_difference_cross_is_nonzero(
    left_a: &Rational,
    left_b: &Rational,
    right_a: &Rational,
    right_b: &Rational,
) -> bool {
    !Rational::signed_product_sum_ordering([true, false], [[left_a, left_b], [right_a, right_b]])
        .is_eq()
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
        Real::diff_of_products(&left[1], &right[2], &left[2], &right[1]),
        Real::diff_of_products(&left[2], &right[0], &left[0], &right[2]),
        Real::diff_of_products(&left[0], &right[1], &left[1], &right[0]),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn previous_points_are_nondegenerate(p0: &Point3, p1: &Point3, p2: &Point3) -> bool {
        let u = sub_points(p1, p0);
        let v = sub_points(p2, p0);
        [[1, 2, 2, 1], [2, 0, 0, 2], [0, 1, 1, 0]]
            .into_iter()
            .any(|[ua, vb, ub, va]| {
                !Real::diff_of_products(&u[ua], &v[vb], &u[ub], &v[va]).definitely_zero()
            })
    }

    #[test]
    fn axis_split_value_handles_non_unit_normal_exactly() {
        let plane =
            Plane::from_coefficients(Real::from(0), Real::from(2), Real::from(0), Real::from(-6));

        assert_eq!(plane.axis_split_value(), Some((1, Real::from(3))));
    }

    #[test]
    fn point_nondegeneracy_matches_materialized_plane_validation() {
        let point = |x, y, z| Point3::new(Real::from(x), Real::from(y), Real::from(z));
        let cases = [
            [point(0, 0, 0), point(2, 0, 0), point(0, 3, 0)],
            [point(1, 1, 1), point(2, 2, 2), point(3, 3, 3)],
        ];

        for [a, b, c] in cases {
            assert_eq!(
                Plane::points_are_nondegenerate(&a, &b, &c),
                Plane::from_points(&a, &b, &c).is_valid()
            );
        }
    }

    proptest! {
        #[test]
        fn optimized_point_nondegeneracy_matches_previous_exact_rational_path(
            coordinates in proptest::array::uniform9((-50_i32..=50, 1_u32..=11))
        ) {
            let coordinates = coordinates.map(|(numerator, denominator)| {
                (Real::from(numerator) / Real::from(denominator))
                    .expect("the generated denominator is positive")
            });
            let [x0, y0, z0, x1, y1, z1, x2, y2, z2] = coordinates;
            let p0 = Point3::new(x0, y0, z0);
            let p1 = Point3::new(x1, y1, z1);
            let p2 = Point3::new(x2, y2, z2);

            prop_assert_eq!(
                Plane::points_are_nondegenerate(&p0, &p1, &p2),
                previous_points_are_nondegenerate(&p0, &p1, &p2)
            );
        }
    }
}
