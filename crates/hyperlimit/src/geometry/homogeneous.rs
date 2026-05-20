//! Homogeneous 3D constructions for exact plane intersections.
//!
//! Yap calls out geometric object packages, especially points and
//! hyperplanes, as a layer above raw BigNumber arithmetic and gives
//! homogeneous/common representations of rational vectors as the motivating
//! example. These constructors keep plane intersections as homogeneous
//! `Real` objects so callers can test incidence or hand the construction to a
//! later exact predicate without immediately dividing into independent scalar
//! coordinates. See Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997), Section 6.3.
//!
//! The plane-triple and plane-line formulas also match the integer
//! plane-based construction strategy used by Nehring-Wirxel, Kern, Trettner,
//! and Kobbelt, "Exact and Efficient Mesh-Kernel Generation," *Computer
//! Graphics Forum* 44.5 (2025), where exact plane intersections are retained
//! as structured objects before downstream classification.

use hyperreal::Real;

use crate::error::{PredicateError, Result};
use crate::geometry::{Plane3, Point3};
use crate::predicate::{PredicateOutcome, PredicatePolicy, RefinementNeed, Sign};
use crate::real::{mul_ref, sub_ref};
use crate::resolve::{map_outcome, resolve_real_sign, signed_term_filter};

/// Homogeneous 3D point `(x : y : z : w)`.
///
/// The represented affine point is `(x / w, y / w, z / w)` when `w` is
/// nonzero. A zero `w` is not a malformed value; it is a point at infinity.
/// Keeping the homogeneous value intact follows Yap's exact-geometric-
/// computation guidance to preserve vector/common-scale structure before
/// scalar expansion. See Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), Section 6.3.
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

    /// Returns exact-rational structure facts for the homogeneous coordinate
    /// tuple.
    pub fn coordinate_facts(&self) -> hyperreal::RealExactSetFacts {
        Real::exact_set_facts(self.coordinates())
    }

    /// Converts this finite homogeneous point to affine coordinates.
    ///
    /// Division is intentionally explicit. Callers that only need incidence or
    /// sidedness should keep the homogeneous point and use
    /// [`classify_homogeneous_point_plane`] instead of forcing three scalar
    /// quotients. That is the common-representation tactic Yap recommends for
    /// rational vectors in Section 6.3 of "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997).
    pub fn to_affine_point(&self) -> Result<Point3> {
        Ok(Point3::new(
            (&self.x / &self.w).map_err(|_| PredicateError::Real("homogeneous x/w"))?,
            (&self.y / &self.w).map_err(|_| PredicateError::Real("homogeneous y/w"))?,
            (&self.z / &self.w).map_err(|_| PredicateError::Real("homogeneous z/w"))?,
        ))
    }

    /// Returns the exact homogeneous plane expression
    /// `a*x + b*y + c*z + d*w`.
    pub fn plane_expression(&self, plane: &Plane3) -> Real {
        homogeneous_point_plane_expression(self, plane)
    }

    /// Classifies whether this homogeneous point is on a plane.
    pub fn classify_plane_incidence(&self, plane: &Plane3) -> PredicateOutcome<bool> {
        classify_homogeneous_point_plane(self, plane)
    }
}

/// Homogeneous 3D line in Pluecker form.
///
/// `direction` is the line direction and `moment` is `p x direction` for any
/// finite point `p` on the line. The representation is homogeneous: scaling
/// both vectors by the same nonzero factor leaves the represented line
/// unchanged. This keeps the intersection of two exact planes as a geometric
/// object, following Yap's object-package layer and the plane-based mesh-kernel
/// construction strategy.
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
    pub fn intersect_plane(&self, plane: &Plane3) -> HomogeneousPoint3 {
        intersect_homogeneous_line_plane(self, plane)
    }

    /// Returns exact-rational structure facts for all Pluecker coordinates.
    pub fn coordinate_facts(&self) -> hyperreal::RealExactSetFacts {
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
///
/// For planes `a*x + b*y + c*z + d = 0`, this uses Cramer's rule and returns
/// `(X : Y : Z : W)` with `W = det([a b c])`. It does not divide by `W`, so
/// parallel or otherwise singular plane triples remain explicit homogeneous
/// constructions. This is the exact common-vector representation advocated by
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997), Section 6.3.
pub fn intersect_three_planes(
    first: &Plane3,
    second: &Plane3,
    third: &Plane3,
) -> HomogeneousPoint3 {
    let n1 = &first.normal;
    let n2 = &second.normal;
    let n3 = &third.normal;
    let minus_d1 = neg(&first.offset);
    let minus_d2 = neg(&second.offset);
    let minus_d3 = neg(&third.offset);

    HomogeneousPoint3::new(
        det3(
            &minus_d1, &n1.y, &n1.z, &minus_d2, &n2.y, &n2.z, &minus_d3, &n3.y, &n3.z,
        ),
        det3(
            &n1.x, &minus_d1, &n1.z, &n2.x, &minus_d2, &n2.z, &n3.x, &minus_d3, &n3.z,
        ),
        det3(
            &n1.x, &n1.y, &minus_d1, &n2.x, &n2.y, &minus_d2, &n3.x, &n3.y, &minus_d3,
        ),
        det3(
            &n1.x, &n1.y, &n1.z, &n2.x, &n2.y, &n2.z, &n3.x, &n3.y, &n3.z,
        ),
    )
}

/// Constructs the homogeneous Pluecker line where two planes intersect.
///
/// The direction is `n1 x n2`; the moment is `d1*n2 - d2*n1`, matching
/// `p x direction` for any finite point `p` on both planes. Keeping this as a
/// line object rather than immediately choosing a point mirrors the geometric
/// object package Yap argues for and the plane-based mesh-kernel workflow.
pub fn intersect_two_planes(first: &Plane3, second: &Plane3) -> HomogeneousLine3 {
    let direction = cross(&first.normal, &second.normal);
    let moment = Point3::new(
        sub(
            &mul(&first.offset, &second.normal.x),
            &mul(&second.offset, &first.normal.x),
        ),
        sub(
            &mul(&first.offset, &second.normal.y),
            &mul(&second.offset, &first.normal.y),
        ),
        sub(
            &mul(&first.offset, &second.normal.z),
            &mul(&second.offset, &first.normal.z),
        ),
    );
    HomogeneousLine3::new(direction, moment)
}

/// Intersects a homogeneous Pluecker line and a plane as a homogeneous point.
///
/// With line direction `u`, moment `v = p x u`, and plane `(n, d)`, the
/// returned point is `(n x v - d*u : n.u)`. The formula avoids selecting an
/// arbitrary affine point on the line before the plane test, preserving the
/// exact object structure emphasized by Yap and by plane-based mesh kernels.
pub fn intersect_homogeneous_line_plane(
    line: &HomogeneousLine3,
    plane: &Plane3,
) -> HomogeneousPoint3 {
    let n_cross_m = cross(&plane.normal, &line.moment);
    let d_u = Point3::new(
        mul(&plane.offset, &line.direction.x),
        mul(&plane.offset, &line.direction.y),
        mul(&plane.offset, &line.direction.z),
    );
    HomogeneousPoint3::new(
        sub(&n_cross_m.x, &d_u.x),
        sub(&n_cross_m.y, &d_u.y),
        sub(&n_cross_m.z, &d_u.z),
        dot(&plane.normal, &line.direction),
    )
}

/// Classifies whether a homogeneous point lies on a plane.
///
/// This evaluates `a*x + b*y + c*z + d*w` directly, avoiding affine division.
/// The predicate is therefore valid for both finite points and points at
/// infinity. Incidence decisions stay in the same exact sign-resolution
/// pipeline as other `hyperlimit` predicates, following Yap's requirement that
/// combinatorial decisions be error-free.
pub fn classify_homogeneous_point_plane(
    point: &HomogeneousPoint3,
    plane: &Plane3,
) -> PredicateOutcome<bool> {
    classify_homogeneous_point_plane_with_policy(point, plane, PredicatePolicy::default())
}

/// Classifies homogeneous point/plane incidence with an explicit predicate
/// policy.
pub fn classify_homogeneous_point_plane_with_policy(
    point: &HomogeneousPoint3,
    plane: &Plane3,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    let expression = homogeneous_point_plane_expression(point, plane);
    map_outcome(
        resolve_real_sign(
            &expression,
            policy,
            || {
                let x_term = mul(&plane.normal.x, &point.x);
                let y_term = mul(&plane.normal.y, &point.y);
                let z_term = mul(&plane.normal.z, &point.z);
                let w_term = mul(&plane.offset, &point.w);
                signed_term_filter(&[
                    (&x_term, Sign::Positive),
                    (&y_term, Sign::Positive),
                    (&z_term, Sign::Positive),
                    (&w_term, Sign::Positive),
                ])
            },
            || None,
            RefinementNeed::RealRefinement,
        ),
        |sign| sign == Sign::Zero,
    )
}

fn homogeneous_point_plane_expression(point: &HomogeneousPoint3, plane: &Plane3) -> Real {
    Real::signed_product_sum(
        [true; 4],
        [
            [&plane.normal.x, &point.x],
            [&plane.normal.y, &point.y],
            [&plane.normal.z, &point.z],
            [&plane.offset, &point.w],
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

fn neg(value: &Real) -> Real {
    sub(&Real::from(0), value)
}

fn sub(left: &Real, right: &Real) -> Real {
    sub_ref(left, right)
}

fn mul(left: &Real, right: &Real) -> Real {
    mul_ref(left, right)
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

    fn plane(a: i32, b: i32, c: i32, d: i32) -> Plane3 {
        Plane3::new(p3(a, b, c), r(d))
    }

    #[test]
    fn three_plane_intersection_preserves_homogeneous_point() {
        let x_eq_1 = plane(1, 0, 0, -1);
        let y_eq_2 = plane(0, 1, 0, -2);
        let z_eq_3 = plane(0, 0, 1, -3);

        let point = intersect_three_planes(&x_eq_1, &y_eq_2, &z_eq_3);
        assert_eq!(point, HomogeneousPoint3::new(r(1), r(2), r(3), r(1)));
        assert_eq!(point.to_affine_point().unwrap(), p3(1, 2, 3));

        for plane in [&x_eq_1, &y_eq_2, &z_eq_3] {
            assert_eq!(point.classify_plane_incidence(plane).value(), Some(true));
        }
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

    #[test]
    fn homogeneous_incidence_handles_points_at_infinity() {
        let x_eq_1 = plane(1, 0, 0, -1);
        let y_eq_2 = plane(0, 1, 0, -2);
        let x_eq_0 = plane(1, 0, 0, 0);
        let line = intersect_two_planes(&x_eq_1, &y_eq_2);
        let point_at_infinity = line.intersect_plane(&x_eq_0);

        assert_eq!(point_at_infinity.w, r(0));
        assert_eq!(
            classify_homogeneous_point_plane(&point_at_infinity, &x_eq_1).value(),
            Some(true)
        );
        assert_eq!(
            classify_homogeneous_point_plane(&point_at_infinity, &y_eq_2).value(),
            Some(true)
        );
    }
}
