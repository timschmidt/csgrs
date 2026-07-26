//! Projective and homogeneous 3D object algebra.
//!
//! This module keeps exact plane intersections as structured homogeneous
//! objects. Predicate crates can decide incidence and sidedness later without
//! forcing affine division or expanding every coordinate independently.

use crate::{BlasResult, Point3, Real};

/// Read-only plane coefficients for projective 3D constructions.
pub trait Plane3Coefficients {
    /// Returns the exact normal coefficients.
    fn normal(&self) -> &Point3;

    /// Returns the exact offset in `normal . point + offset = 0`.
    fn offset(&self) -> &Real;
}

/// Minimal exact plane coefficient carrier for projective algebra.
#[derive(Clone, Debug, PartialEq)]
pub struct ProjectivePlane3 {
    /// Plane normal vector.
    pub normal: Point3,
    /// Constant offset in `normal . point + offset = 0`.
    pub offset: Real,
}

impl ProjectivePlane3 {
    /// Constructs a plane coefficient carrier.
    pub const fn new(normal: Point3, offset: Real) -> Self {
        Self { normal, offset }
    }
}

impl Plane3Coefficients for ProjectivePlane3 {
    fn normal(&self) -> &Point3 {
        &self.normal
    }

    fn offset(&self) -> &Real {
        &self.offset
    }
}

/// Homogeneous 3D point `(x : y : z : w)`.
#[derive(Clone, Debug, PartialEq)]
pub struct HomogeneousPoint3 {
    /// Homogeneous x numerator.
    pub x: Real,
    /// Homogeneous y numerator.
    pub y: Real,
    /// Homogeneous z numerator.
    pub z: Real,
    /// Homogeneous scale.
    pub w: Real,
}

impl HomogeneousPoint3 {
    /// Creates a homogeneous point from explicit coordinates.
    pub const fn new(x: Real, y: Real, z: Real, w: Real) -> Self {
        Self { x, y, z, w }
    }

    /// Returns borrowed coordinates in `(x, y, z, w)` order.
    pub fn coordinates(&self) -> [&Real; 4] {
        [&self.x, &self.y, &self.z, &self.w]
    }

    /// Returns exact-rational structure facts for the homogeneous tuple.
    pub fn coordinate_facts(&self) -> crate::ExactRealSetFacts {
        Real::exact_set_facts(self.coordinates())
    }

    /// Converts this finite homogeneous point to affine coordinates.
    pub fn to_affine_point(&self) -> BlasResult<Point3> {
        if let [Some(x), Some(y), Some(z), Some(weight)] =
            self.coordinates().map(Real::exact_rational_ref)
        {
            if weight.is_one() {
                crate::trace_dispatch!(
                    "hyperlattice_projective",
                    "affine-point",
                    "exact-unit-weight"
                );
                return Ok(Point3::new(self.x.clone(), self.y.clone(), self.z.clone()));
            }
            if [x, y, z, weight].into_iter().all(|value| value.is_dyadic()) {
                crate::trace_dispatch!(
                    "hyperlattice_projective",
                    "affine-point",
                    "exact-dyadic-quotients"
                );
                return Ok(Point3::new(
                    Real::exact_rational_quotient_known_dyadic(&self.x, &self.w)?,
                    Real::exact_rational_quotient_known_dyadic(&self.y, &self.w)?,
                    Real::exact_rational_quotient_known_dyadic(&self.z, &self.w)?,
                ));
            }
        }
        crate::trace_dispatch!("hyperlattice_projective", "affine-point", "general");
        Ok(Point3::new(
            (&self.x / &self.w)?,
            (&self.y / &self.w)?,
            (&self.z / &self.w)?,
        ))
    }

    /// Returns the exact homogeneous plane expression
    /// `a*x + b*y + c*z + d*w`.
    pub fn plane_expression<P>(&self, plane: &P) -> Real
    where
        P: Plane3Coefficients + ?Sized,
    {
        homogeneous_point_plane_expression(self, plane)
    }
}

/// Homogeneous 3D line in Pluecker form.
#[derive(Clone, Debug, PartialEq)]
pub struct HomogeneousLine3 {
    /// Direction vector.
    pub direction: Point3,
    /// Moment vector `p x direction`.
    pub moment: Point3,
}

impl HomogeneousLine3 {
    /// Creates a homogeneous Pluecker line.
    pub const fn new(direction: Point3, moment: Point3) -> Self {
        Self { direction, moment }
    }

    /// Intersects this line with a plane and returns a homogeneous point.
    pub fn intersect_plane<P>(&self, plane: &P) -> HomogeneousPoint3
    where
        P: Plane3Coefficients + ?Sized,
    {
        intersect_homogeneous_line_plane(self, plane)
    }

    /// Returns exact-rational structure facts for all Pluecker coordinates.
    pub fn coordinate_facts(&self) -> crate::ExactRealSetFacts {
        Real::exact_set_facts([
            &self.direction.x,
            &self.direction.y,
            &self.direction.z,
            &self.moment.x,
            &self.moment.y,
            &self.moment.z,
        ])
    }
}

/// Constructs the homogeneous intersection point of three planes.
pub fn intersect_three_planes<P>(first: &P, second: &P, third: &P) -> HomogeneousPoint3
where
    P: Plane3Coefficients + ?Sized,
{
    let n1 = first.normal();
    let n2 = second.normal();
    let n3 = third.normal();

    HomogeneousPoint3::new(
        neg_det3(
            first.offset(),
            &n1.y,
            &n1.z,
            second.offset(),
            &n2.y,
            &n2.z,
            third.offset(),
            &n3.y,
            &n3.z,
        ),
        neg_det3(
            &n1.x,
            first.offset(),
            &n1.z,
            &n2.x,
            second.offset(),
            &n2.z,
            &n3.x,
            third.offset(),
            &n3.z,
        ),
        neg_det3(
            &n1.x,
            &n1.y,
            first.offset(),
            &n2.x,
            &n2.y,
            second.offset(),
            &n3.x,
            &n3.y,
            third.offset(),
        ),
        det3(
            &n1.x, &n1.y, &n1.z, &n2.x, &n2.y, &n2.z, &n3.x, &n3.y, &n3.z,
        ),
    )
}

/// Constructs the homogeneous Pluecker line where two planes intersect.
pub fn intersect_two_planes<P>(first: &P, second: &P) -> HomogeneousLine3
where
    P: Plane3Coefficients + ?Sized,
{
    let direction = cross(first.normal(), second.normal());
    let moment = Point3::new(
        sub(
            &mul(first.offset(), &second.normal().x),
            &mul(second.offset(), &first.normal().x),
        ),
        sub(
            &mul(first.offset(), &second.normal().y),
            &mul(second.offset(), &first.normal().y),
        ),
        sub(
            &mul(first.offset(), &second.normal().z),
            &mul(second.offset(), &first.normal().z),
        ),
    );
    HomogeneousLine3::new(direction, moment)
}

/// Intersects a homogeneous Pluecker line and a plane as a homogeneous point.
pub fn intersect_homogeneous_line_plane<P>(line: &HomogeneousLine3, plane: &P) -> HomogeneousPoint3
where
    P: Plane3Coefficients + ?Sized,
{
    let n_cross_m = cross(plane.normal(), &line.moment);
    let d_u = Point3::new(
        mul(plane.offset(), &line.direction.x),
        mul(plane.offset(), &line.direction.y),
        mul(plane.offset(), &line.direction.z),
    );
    HomogeneousPoint3::new(
        sub(&n_cross_m.x, &d_u.x),
        sub(&n_cross_m.y, &d_u.y),
        sub(&n_cross_m.z, &d_u.z),
        dot(plane.normal(), &line.direction),
    )
}

/// Returns the exact homogeneous plane expression `a*x + b*y + c*z + d*w`.
pub fn homogeneous_point_plane_expression<P>(point: &HomogeneousPoint3, plane: &P) -> Real
where
    P: Plane3Coefficients + ?Sized,
{
    Real::signed_product_sum(
        [true; 4],
        [
            [&plane.normal().x, &point.x],
            [&plane.normal().y, &point.y],
            [&plane.normal().z, &point.z],
            [plane.offset(), &point.w],
        ],
    )
}

fn cross(left: &Point3, right: &Point3) -> Point3 {
    Point3::new(
        sub(&mul(&left.y, &right.z), &mul(&left.z, &right.y)),
        sub(&mul(&left.z, &right.x), &mul(&left.x, &right.z)),
        sub(&mul(&left.x, &right.y), &mul(&left.y, &right.x)),
    )
}

fn dot(left: &Point3, right: &Point3) -> Real {
    Real::signed_product_sum(
        [true; 3],
        [
            [&left.x, &right.x],
            [&left.y, &right.y],
            [&left.z, &right.z],
        ],
    )
}

#[allow(clippy::too_many_arguments)]
fn det3(
    a: &Real,
    b: &Real,
    c: &Real,
    d: &Real,
    e: &Real,
    f: &Real,
    g: &Real,
    h: &Real,
    i: &Real,
) -> Real {
    Real::signed_product_sum(
        [true, true, true, false, false, false],
        [
            [a, e, i],
            [b, f, g],
            [c, d, h],
            [c, e, g],
            [b, d, i],
            [a, f, h],
        ],
    )
}

#[allow(clippy::too_many_arguments)]
fn neg_det3(
    a: &Real,
    b: &Real,
    c: &Real,
    d: &Real,
    e: &Real,
    f: &Real,
    g: &Real,
    h: &Real,
    i: &Real,
) -> Real {
    Real::signed_product_sum(
        [false, false, false, true, true, true],
        [
            [a, e, i],
            [b, f, g],
            [c, d, h],
            [c, e, g],
            [b, d, i],
            [a, f, h],
        ],
    )
}

fn sub(left: &Real, right: &Real) -> Real {
    left - right
}

fn mul(left: &Real, right: &Real) -> Real {
    left * right
}

#[cfg(test)]
mod tests {
    use super::*;

    fn r(value: i32) -> Real {
        value.into()
    }

    fn p3(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    fn plane(a: i32, b: i32, c: i32, d: i32) -> ProjectivePlane3 {
        ProjectivePlane3::new(p3(a, b, c), r(d))
    }

    #[test]
    fn three_plane_intersection_preserves_homogeneous_point() {
        let x_eq_1 = plane(1, 0, 0, -1);
        let y_eq_2 = plane(0, 1, 0, -2);
        let z_eq_3 = plane(0, 0, 1, -3);

        let point = intersect_three_planes(&x_eq_1, &y_eq_2, &z_eq_3);
        assert_eq!(point, HomogeneousPoint3::new(r(1), r(2), r(3), r(1)));
        assert_eq!(point.to_affine_point().unwrap(), p3(1, 2, 3));
    }

    #[test]
    fn wide_dyadic_affine_point_matches_independent_exact_division() {
        let value = |text: &str| text.parse::<Real>().unwrap();
        let point = HomogeneousPoint3::new(
            value("680564733841876926926749214863536422913/128"),
            value("-340282366920938463463374607431768211507/32"),
            value("5/256"),
            value("3/8"),
        );
        let expected = Point3::new(
            (&point.x / &point.w).unwrap(),
            (&point.y / &point.w).unwrap(),
            (&point.z / &point.w).unwrap(),
        );

        assert_eq!(point.to_affine_point().unwrap(), expected);
    }

    #[test]
    fn two_plane_line_and_line_plane_match_three_plane_intersection() {
        let x_eq_1 = plane(1, 0, 0, -1);
        let y_eq_2 = plane(0, 1, 0, -2);
        let z_eq_3 = plane(0, 0, 1, -3);

        let line = intersect_two_planes(&x_eq_1, &y_eq_2);
        assert_eq!(line.direction, p3(0, 0, 1));
        assert_eq!(line.moment, p3(2, -1, 0));

        let point = line.intersect_plane(&z_eq_3);
        assert_eq!(point, intersect_three_planes(&x_eq_1, &y_eq_2, &z_eq_3));
        assert_eq!(point.to_affine_point().unwrap(), p3(1, 2, 3));
    }

    #[test]
    fn parallel_planes_produce_point_at_infinity_without_affine_division() {
        let x_eq_1 = plane(1, 0, 0, -1);
        let x_eq_2 = plane(1, 0, 0, -2);
        let z_eq_0 = plane(0, 0, 1, 0);

        let point = intersect_three_planes(&x_eq_1, &x_eq_2, &z_eq_0);
        assert_eq!(point.w, r(0));
        assert!(point.to_affine_point().is_err());
    }
}
