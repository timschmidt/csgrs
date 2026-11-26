//! Predicate wrappers for homogeneous 3D constructions.
//!
//! The homogeneous point and Pluecker line carriers live in `hyperlattice`.
//! This module keeps incidence classification in `hyperlimit`, where predicate
//! policy, provenance, and exact/unknown outcomes are owned.
//!
//! Construction wrappers retain division-free homogeneous coordinates, not an
//! affine existence/uniqueness certificate. Incidence checks a linear expression
//! without validating the carriers. Preserve its local certainty; no source
//! construction history is attached by this module.

use crate::predicate::PredicatePolicy;
pub use hyperlattice::{HomogeneousLine3, HomogeneousPoint3};
use hyperlattice::{Plane3Coefficients, homogeneous_point_plane_expression};

use crate::geometry::Plane3;
use crate::predicate::{PredicateOutcome, RefinementNeed, Sign};
use crate::real::mul_ref;
use crate::resolve::{map_outcome, resolve_real_sign, signed_term_filter};
use core::cmp::Ordering;
use hyperreal::Rational;

/// Constructs the homogeneous intersection point of three planes.
///
/// Delegates to Hyperlattice's signed 3-by-3 minors of the three coefficient
/// rows, retaining a common determinant as `w`. For a unique finite intersection,
/// dividing `(x, y, z)` by nonzero `w` gives the affine point. A nonzero tuple
/// with `w = 0` represents a projective point at infinity; dependent rows can
/// produce the all-zero tuple, which is not a projective point.
///
/// No rank, plane validity, uniqueness or affine-conversion check is performed.
/// The returned carrier is not normalized, and its coordinate equality is not
/// equality up to a nonzero homogeneous scale.
pub fn intersect_three_planes(
    first: &Plane3,
    second: &Plane3,
    third: &Plane3,
) -> HomogeneousPoint3 {
    hyperlattice::intersect_three_planes(first, second, third)
}

/// Constructs the homogeneous Pluecker line where two planes intersect.
///
/// For normals `n1`, `n2` and offsets `d1`, `d2`, the shared construction returns
/// direction `n1 x n2` and moment `d1*n2 - d2*n1`. A nonzero direction defines
/// the usual affine intersection line. Parallel or coincident planes can give
/// zero direction or an all-zero carrier; this wrapper neither rejects those
/// cases nor claims a unique affine line. No division or normalization occurs.
pub fn intersect_two_planes(first: &Plane3, second: &Plane3) -> HomogeneousLine3 {
    hyperlattice::intersect_two_planes(first, second)
}

/// Intersects a homogeneous Pluecker line and a plane as a homogeneous point.
///
/// For line direction `u`, moment `m`, and plane coefficients `(n, d)`, delegates
/// the exact tuple `(n x m - d*u, n . u)`. The line's Pluecker relation and
/// nondegeneracy are assumed, not checked. A parallel line may give `w = 0`;
/// a contained line may give the all-zero tuple rather than a selected point.
/// Convert to affine coordinates separately when a finite point is required.
pub fn intersect_homogeneous_line_plane(
    line: &HomogeneousLine3,
    plane: &Plane3,
) -> HomogeneousPoint3 {
    hyperlattice::intersect_homogeneous_line_plane(line, plane)
}

/// Classifies homogeneous point/plane incidence with an explicit predicate
/// policy.
///
/// Tests whether `normal.x*x + normal.y*y + normal.z*z + offset*w` is zero,
/// without dividing by `w`, so valid points at infinity can be queried. This
/// is incidence, not affine sidedness: the sign of a homogeneous expression
/// also depends on the representative's scale.
///
/// Directly exposed rational coefficients and coordinates use an exact product
/// sum; other values enter the shared sign resolver with a structural term
/// filter. The resulting Boolean keeps that resolver's certainty and stage.
/// Approximate true may reflect a terminal zero classification rather than
/// proven incidence. No carrier validity check is made: an all-zero point or
/// plane expression is accepted as zero, not diagnosed as invalid geometry.
pub fn classify_homogeneous_point_plane_with_policy(
    point: &HomogeneousPoint3,
    plane: &Plane3,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    if let Some(outcome) = classify_exact_rational_homogeneous_point_plane(point, plane) {
        return outcome;
    }

    let expression = homogeneous_point_plane_expression(point, plane);
    map_outcome(
        resolve_real_sign(
            &expression,
            policy,
            || {
                let x_term = mul_ref(&plane.normal.x, &point.x);
                let y_term = mul_ref(&plane.normal.y, &point.y);
                let z_term = mul_ref(&plane.normal.z, &point.z);
                let w_term = mul_ref(&plane.offset, &point.w);
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

/// Decide the four-term incidence form only when all eight scalars expose rational references.
/// A missing rational representation declines without asserting non-incidence;
/// an available exact form returns Exact certainty and stage, independently of policy.
#[inline]
fn classify_exact_rational_homogeneous_point_plane(
    point: &HomogeneousPoint3,
    plane: &Plane3,
) -> Option<PredicateOutcome<bool>> {
    let [Some(a), Some(b), Some(c), Some(d)] = [
        &plane.normal.x,
        &plane.normal.y,
        &plane.normal.z,
        &plane.offset,
    ]
    .map(hyperreal::Real::exact_rational_ref) else {
        return None;
    };
    let [Some(x), Some(y), Some(z), Some(w)] =
        [&point.x, &point.y, &point.z, &point.w].map(hyperreal::Real::exact_rational_ref)
    else {
        return None;
    };

    let ordering =
        Rational::signed_product_sum_ordering([true; 4], [[a, x], [b, y], [c, z], [d, w]]);
    crate::trace_dispatch!(
        "hyperlimit",
        "classify_homogeneous_point_plane",
        "exact-rational-linear-form"
    );
    Some(PredicateOutcome::decided(
        ordering == Ordering::Equal,
        crate::Certainty::Exact,
        crate::Escalation::Exact,
    ))
}

impl Plane3Coefficients for Plane3 {
    /// Borrow the native normal coefficients without cloning or normalizing them.
    fn normal(&self) -> &hyperlattice::Point3 {
        &self.normal
    }

    /// Borrow the additive coefficient in the native plane equation.
    fn offset(&self) -> &hyperreal::Real {
        &self.offset
    }
}

#[cfg(test)]
/// Rational and symbolic construction/incidence witnesses, including affine rejection at infinity.
mod tests {
    use super::*;
    use crate::{Point3, Real};

    /// Permit terminal decisions; these fixtures inspect values, not complete certainty receipts.
    const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

    /// Construct an exact integer scalar for geometric expectations.
    fn r(value: i32) -> Real {
        value.into()
    }

    /// Construct a spatial point or coefficient vector from exact integers.
    fn p3(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    /// Construct integer coefficients of `a*x + b*y + c*z + d = 0` without normalization.
    fn plane(a: i32, b: i32, c: i32, d: i32) -> Plane3 {
        Plane3::new(p3(a, b, c), r(d))
    }

    #[test]
    /// Check the independent (1,2,3,1) solution and incidence with all three axis planes.
    fn three_plane_intersection_preserves_homogeneous_point() {
        let x_eq_1 = plane(1, 0, 0, -1);
        let y_eq_2 = plane(0, 1, 0, -2);
        let z_eq_3 = plane(0, 0, 1, -3);

        let point = intersect_three_planes(&x_eq_1, &y_eq_2, &z_eq_3);
        assert_eq!(point, HomogeneousPoint3::new(r(1), r(2), r(3), r(1)));
        assert_eq!(point.to_affine_point().unwrap(), p3(1, 2, 3));

        for plane in [&x_eq_1, &y_eq_2, &z_eq_3] {
            assert_eq!(
                crate::classify_homogeneous_point_plane(&point, plane, APPROX).value(),
                Some(true)
            );
        }
    }

    #[test]
    /// Check exact direction/moment expectations and compose line-plane with affine conversion.
    fn two_plane_line_and_line_plane_match_three_plane_intersection() {
        let x_eq_1 = plane(1, 0, 0, -1);
        let y_eq_2 = plane(0, 1, 0, -2);
        let z_eq_3 = plane(0, 0, 1, -3);

        let line = intersect_two_planes(&x_eq_1, &y_eq_2);
        assert_eq!(line.direction, p3(0, 0, 1));
        assert_eq!(line.moment, p3(2, -1, 0));

        let point = intersect_homogeneous_line_plane(&line, &z_eq_3);
        assert_eq!(point, intersect_three_planes(&x_eq_1, &y_eq_2, &z_eq_3));
        assert_eq!(point.to_affine_point().unwrap(), p3(1, 2, 3));
    }

    #[test]
    /// Exercise symbolic point/plane inputs and an erf expression with known nonzero expectation.
    /// Value assertions alone do not establish which generic resolver stage was used.
    fn symbolic_homogeneous_incidence_uses_the_generic_real_pipeline() {
        let pi = Real::pi();
        let point = HomogeneousPoint3::new(pi.clone(), r(0), r(0), r(1));
        let x_eq_pi = Plane3::new(p3(1, 0, 0), -pi);
        let x_eq_zero = plane(1, 0, 0, 0);

        let incident = crate::classify_homogeneous_point_plane(&point, &x_eq_pi, APPROX);
        assert_eq!(incident.value(), Some(true));
        assert_eq!(
            crate::classify_homogeneous_point_plane(&point, &x_eq_zero, APPROX).value(),
            Some(false)
        );

        let rational_point = HomogeneousPoint3::new(r(1), r(0), r(0), r(1));
        assert_eq!(
            crate::classify_homogeneous_point_plane(&rational_point, &x_eq_pi, APPROX).value(),
            Some(false)
        );

        let opaque = Real::from(1).erf() - "4/5".parse::<Real>().unwrap();
        let opaque_point = HomogeneousPoint3::new(opaque, r(0), r(0), r(1));
        assert_eq!(
            crate::classify_homogeneous_point_plane(&opaque_point, &x_eq_zero, APPROX).value(),
            Some(false)
        );
    }

    #[test]
    /// Check zero homogeneous weight and failed affine conversion for parallel x constraints.
    fn parallel_planes_produce_point_at_infinity_without_affine_division() {
        let x_eq_1 = plane(1, 0, 0, -1);
        let x_eq_2 = plane(1, 0, 0, -2);
        let z_eq_0 = plane(0, 0, 1, 0);

        let point = intersect_three_planes(&x_eq_1, &x_eq_2, &z_eq_0);
        assert_eq!(point.w, r(0));
        assert!(point.to_affine_point().is_err());
    }

    #[test]
    /// Query both generating planes at a line's projective intersection with a parallel plane.
    fn homogeneous_incidence_handles_points_at_infinity() {
        let x_eq_1 = plane(1, 0, 0, -1);
        let y_eq_2 = plane(0, 1, 0, -2);
        let x_eq_0 = plane(1, 0, 0, 0);
        let line = intersect_two_planes(&x_eq_1, &y_eq_2);
        let point_at_infinity = line.intersect_plane(&x_eq_0);

        assert_eq!(point_at_infinity.w, r(0));
        assert_eq!(
            crate::classify_homogeneous_point_plane(&point_at_infinity, &x_eq_1, APPROX).value(),
            Some(true)
        );
        assert_eq!(
            crate::classify_homogeneous_point_plane(&point_at_infinity, &y_eq_2, APPROX).value(),
            Some(true)
        );
    }
}
