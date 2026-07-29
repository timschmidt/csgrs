use csgrs::{Real, solid};
use hyperlattice::{Point3, Vector3};

#[test]
fn sphere_diameter_ray_reports_only_surface_hits() {
    let mesh = solid::sphere(Real::from(10_u8), 32, 16);
    let origin = Point3::new(Real::from(-20_i8), Real::zero(), Real::zero());
    let hits = solid::ray_intersections(&mesh, &origin, &Vector3::x()).unwrap();

    assert_eq!(hits.len(), 2);
    assert!((hits[0].1.to_f64_lossy().expect("finite parameter") - 10.0).abs() < f64::EPSILON);
    assert!((hits[1].1.to_f64_lossy().expect("finite parameter") - 30.0).abs() < 1.0e-12);
    assert_eq!(
        solid::ray_intersections(&mesh, &origin, &Vector3::x()).unwrap(),
        hits,
        "the retained native query must preserve the exact result"
    );
}

#[test]
fn disjoint_distributions_preserve_every_native_copy() {
    let cube = solid::cube(Real::one());
    let linear = solid::distribute_linear(&cube, 8, Vector3::x(), Real::from(2_u8));
    let grid = solid::distribute_grid(&cube, 4, 4, Real::from(2_u8), Real::from(2_u8));
    let arc = solid::distribute_arc(
        &cube,
        12,
        Real::from(10_u8),
        Real::zero(),
        Real::from(330_u16),
    );

    assert_eq!(linear.triangles.len(), 8 * cube.triangles.len());
    assert_eq!(grid.triangles.len(), 16 * cube.triangles.len());
    assert_eq!(arc.triangles.len(), 12 * cube.triangles.len());
    assert!(linear.is_closed_manifold());
    assert!(grid.is_closed_manifold());
    assert!(arc.is_closed_manifold());
}
