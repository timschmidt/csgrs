//! Native ring torus sections by off-axis spheres, offset tori and cones,
//! through an exact rational atlas of the torus.
//!
//! Every expected point below lies on an atlas seam of the torus
//! (x, y, z) = ((3 + cos φ) cos θ, (3 + cos φ) sin θ, sin φ) at θ = 0, θ = π
//! or φ = 0, and is derived independently from the two defining equations.
//! Seam points are matched with exact linear forms against represented values.

use hyperbrep::*;
use std::cmp::Ordering;

fn q(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn root(n: i64) -> Real {
    Real::from(n).sqrt().unwrap()
}

fn pi(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}

/// R = 3, r = 1 about the z axis through `center`.
fn torus(center: Point3) -> Surface {
    Surface::torus(
        center,
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        3.into(),
        1.into(),
    )
    .unwrap()
}

fn ends(result: SurfaceSurfaceIntersection) -> Vec<GeometricPoint3> {
    let pieces = match result {
        SurfaceSurfaceIntersection::Curves(curves) => curves,
        SurfaceSurfaceIntersection::Curve(curve) => vec![*curve],
        other => panic!("curve pieces required, found {other:?}"),
    };
    for piece in &pieces {
        assert_eq!(piece.curve().kind(), Curve3Kind::TensorImage);
        assert!(piece.first_pcurve().materialize().is_err());
        assert!(piece.second_pcurve().materialize().is_err());
    }
    pieces
        .iter()
        .flat_map(|piece| {
            let curve = piece.curve();
            [
                curve.point_at(curve.domain().start()).unwrap(),
                curve.point_at(curve.domain().end()).unwrap(),
            ]
        })
        .collect()
}

/// Number of piece ends at the exact point (x, y, z).
fn count(ends: &[GeometricPoint3], x: &Real, y: &Real, z: &Real) -> usize {
    ends.iter()
        .filter(|point| {
            [(Vector3::x(), x), (Vector3::y(), y), (Vector3::z(), z)]
                .into_iter()
                .all(|(axis, value)| {
                    point.compare_linear_form(&axis, value).unwrap() == Ordering::Equal
                })
        })
        .count()
}

/// Each seam crossing joins exactly two pieces from adjacent patches.
fn assert_junction(ends: &[GeometricPoint3], x: Real, y: Real, z: Real) {
    assert_eq!(count(ends, &x, &y, &z), 2);
}

/// The unit sphere about (4, 0, 0): on θ = 0, (cos φ - 1)^2 + sin^2 φ = 1
/// gives cos φ = 1/2, the points (7/2, 0, ±√3/2); on φ = 0,
/// |4(cos θ, sin θ) - (4, 0)|^2 = 1 gives cos θ = 31/32, (31/8, ±√63/8, 0).
#[test]
fn off_axis_sphere_crosses_known_torus_seams() {
    let sphere = Surface::sphere(
        pi(4, 0, 0),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        1.into(),
    )
    .unwrap();
    let ends = ends(torus(Point3::origin()).intersect_surface(&sphere).unwrap());
    let z = (root(3) / Real::from(2)).unwrap();
    assert_junction(&ends, q(7, 2), Real::zero(), z.clone());
    assert_junction(&ends, q(7, 2), Real::zero(), -z);
    let y = (root(63) / Real::from(8)).unwrap();
    assert_junction(&ends, q(31, 8), y.clone(), Real::zero());
    assert_junction(&ends, q(31, 8), -y, Real::zero());
    // Both operand orders chart the torus.
    let swapped = ends_of_swapped(&sphere);
    assert_eq!(swapped.len(), ends.len());
}

fn ends_of_swapped(sphere: &Surface) -> Vec<GeometricPoint3> {
    ends(sphere.intersect_surface(&torus(Point3::origin())).unwrap())
}

/// Non-coaxial torus pairs remain a typed limitation: a measured overlapping
/// pair took minutes through the atlas, so the route is not admitted.
#[test]
fn non_coaxial_torus_pairs_remain_typed() {
    let small = Surface::torus(
        pi(3, 0, 1),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        1.into(),
        q(1, 2),
    )
    .unwrap();
    assert!(matches!(
        torus(Point3::origin()).intersect_surface(&small),
        Err(GeometryError::UnsupportedIntersection)
    ));
}

/// The 45-degree cone (x - 1)^2 + y^2 = (4 - z)^2 opening down from
/// (1, 0, 4), off the torus axis. On θ = π, x = -(3 + c) and z = s give
/// 4 + c = 4 - s, so c = -s: the points (-3 ∓ √2/2, 0, ∓√2/2).
#[test]
fn off_axis_downward_cone_crosses_known_torus_seams() {
    let cone = Surface::cone(
        pi(1, 0, 4),
        Vector3::x(),
        -Vector3::y(),
        -Vector3::z(),
        (Real::pi() / Real::from(4)).unwrap(),
    )
    .unwrap();
    let ends = ends(torus(Point3::origin()).intersect_surface(&cone).unwrap());
    let half = (root(2) / Real::from(2)).unwrap();
    assert_junction(&ends, Real::from(-3) - &half, Real::zero(), -half.clone());
    assert_junction(&ends, Real::from(-3) + &half, Real::zero(), half);
}

#[test]
fn distant_sphere_gives_a_complete_empty_result() {
    let far = Surface::sphere(
        pi(9, 0, 0),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        1.into(),
    )
    .unwrap();
    assert!(matches!(
        torus(Point3::origin()).intersect_surface(&far).unwrap(),
        SurfaceSurfaceIntersection::None
    ));
}
