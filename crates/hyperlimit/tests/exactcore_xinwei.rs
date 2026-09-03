use core::cmp::Ordering;

use hyperlimit::{Point3, PredicatePolicy, compare_point_triangle3_distance_squared};
use hyperreal::Real;

const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

fn r(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).expect("nonzero denominator")
}

fn p3(x: (i32, i32), y: (i32, i32), z: (i32, i32)) -> Point3 {
    Point3::new(r(x.0, x.1), r(y.0, y.1), r(z.0, z.1))
}

#[test]
fn three_dimensional_box_shell_uses_the_full_diagonal_radius() {
    // The projection of the origin is the centroid (5/6, 5/6, 5/6), and
    // its exact squared distance to this triangle is 25/12: greater than the
    // legacy 3D planner's squared sqrt(2) shell but less than the sound squared
    // sqrt(3) shell for a unit-radius subdivision box.
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
