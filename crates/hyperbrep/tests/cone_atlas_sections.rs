//! Native cone sections by spheres centred off the cone axis, through an exact
//! rational atlas of the cone's bounded relevant portion.
//!
//! Fixture: the 45-degree cone x^2 + y^2 = z^2, z >= 0, from the origin. On
//! its atlas seam over the direction (1, 0) the cone is (h, 0, h).

use hyperbrep::*;
use std::cmp::Ordering;

fn pi(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}

fn cone() -> Surface {
    Surface::cone(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        (Real::pi() / Real::from(4)).unwrap(),
    )
    .unwrap()
}

fn sphere(center: Point3, radius: i64) -> Surface {
    Surface::sphere(
        center,
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        radius.into(),
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

fn count(ends: &[GeometricPoint3], point: &Point3) -> usize {
    ends.iter()
        .filter(|end| {
            [
                (Vector3::x(), &point.x),
                (Vector3::y(), &point.y),
                (Vector3::z(), &point.z),
            ]
            .into_iter()
            .all(|(axis, value)| end.compare_linear_form(&axis, value).unwrap() == Ordering::Equal)
        })
        .count()
}

/// The unit sphere about (1, 0, 2) meets the seam (h, 0, h) where
/// (h - 1)^2 + (h - 2)^2 = 1, so h = 1 or 2: each point joins two pieces.
#[test]
fn off_axis_sphere_crosses_the_cone_seam_at_known_points() {
    let center = pi(1, 0, 2);
    for result in [
        cone()
            .intersect_surface(&sphere(center.clone(), 1))
            .unwrap(),
        sphere(center.clone(), 1)
            .intersect_surface(&cone())
            .unwrap(),
    ] {
        let ends = ends(result);
        assert_eq!(count(&ends, &pi(1, 0, 1)), 2);
        assert_eq!(count(&ends, &pi(2, 0, 2)), 2);
    }
}

/// A sphere through the apex is refused typed; a sphere entirely below the
/// apex plane is a complete empty result.
#[test]
fn apex_and_opposite_side_cases_are_explicit() {
    assert!(matches!(
        cone().intersect_surface(&sphere(pi(1, 0, 0), 1)),
        Err(GeometryError::UnsupportedIntersection)
    ));
    assert!(matches!(
        cone().intersect_surface(&sphere(pi(3, 0, -5), 1)).unwrap(),
        SurfaceSurfaceIntersection::None
    ));
}
