//! Rational distance regressions for a 3D box-shell bound and a collapsed triangle.
//!
//! The first fixture analytically separates squared thresholds two and three;
//! the second places the query in a collinear triangle's closed segment hull.
//! These call distance predicates directly, not a planner, and extract only
//! ordering values under the approximation-permitted policy.

use core::cmp::Ordering;

use hyperlimit::{Point3, PredicatePolicy, compare_point_triangle3_distance_squared};
use hyperreal::Real;

/// Approximation-permitted policy for rational distance controls; receipts are not inspected.
const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

/// Construct a rational scalar by exact division, requiring a nonzero denominator.
fn r(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).expect("nonzero denominator")
}

/// Construct a point from three signed numerator/denominator pairs.
fn p3(x: (i32, i32), y: (i32, i32), z: (i32, i32)) -> Point3 {
    Point3::new(r(x.0, x.1), r(y.0, y.1), r(z.0, z.1))
}

/// Compare the origin/triangle squared distance against two and three.
///
/// All vertices lie on `x+y+z=5/2`; their centroid `(5/6,5/6,5/6)` is the origin's
/// orthogonal projection and lies in the triangle. Its squared norm is `25/12`.
/// The assertions require Greater than two and Less than three, not equality to
/// 25/12 or behavior of any subdivision planner.
#[test]
fn three_dimensional_box_shell_uses_the_full_diagonal_radius() {
    // Compare the squared 2D and 3D corner radii (2 and 3) of boxes with
    // unit half-widths. The exact squared distance 25/12 lies between them.
    let a = p3((17, 6), (-7, 6), (5, 6));
    let b = p3((5, 6), (17, 6), (-7, 6));
    let c = p3((-7, 6), (5, 6), (17, 6));
    let origin = p3((0, 1), (0, 1), (0, 1));

    assert_eq!(
        compare_point_triangle3_distance_squared(&origin, &a, &b, &c, &Real::from(2), APPROX)
            .value(),
        Some(Ordering::Greater)
    );
    assert_eq!(
        compare_point_triangle3_distance_squared(&origin, &a, &b, &c, &Real::from(3), APPROX)
            .value(),
        Some(Ordering::Less)
    );
}

/// Require zero squared distance when the origin lies in a collinear triangle's hull.
///
/// The x-axis vertices are -10, 10, and 20; this is one segment-degenerate control,
/// not coverage of all repeated-vertex or singular configurations.
#[test]
fn degenerate_triangle_reduces_to_its_closed_segment_hull() {
    let origin = p3((0, 1), (0, 1), (0, 1));
    let a = p3((-10, 1), (0, 1), (0, 1));
    let b = p3((10, 1), (0, 1), (0, 1));
    let c = p3((20, 1), (0, 1), (0, 1));

    assert_eq!(
        compare_point_triangle3_distance_squared(&origin, &a, &b, &c, &Real::from(0), APPROX)
            .value(),
        Some(Ordering::Equal)
    );
}
