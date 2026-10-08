use csgrs::{GeometryCertainty, GeometryContext, Real, solid};
use hyperlattice::{Point3, Vector3};
use hyperlimit::PredicatePolicy;

#[test]
fn sphere_diameter_ray_reports_only_surface_hits() {
    // The diameter passes exactly through two sampled-sphere vertices. Each
    // vertex contact is certified from the input coordinates and reported as
    // the vertex itself, so every incident triangle reports the same exact hit.
    let mesh = solid::sphere(Real::from(10_u8), 32, 16);
    let origin = Point3::new(Real::from(-20_i8), Real::zero(), Real::zero());
    let outcome =
        solid::ray_intersections(&mesh, &origin, &Vector3::x(), &GeometryContext::STRICT)
            .unwrap();
    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
    let hits = outcome.value;

    assert_eq!(hits.len(), 2);
    for (hit, parameter) in hits.iter().zip([10_u8, 30]) {
        assert_eq!(
            hyperlimit::compare_reals(&hit.1, &Real::from(parameter), PredicatePolicy::STRICT)
                .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }
    assert_eq!(
        solid::ray_intersections(&mesh, &origin, &Vector3::x(), &GeometryContext::STRICT)
            .unwrap()
            .value,
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

#[test]
fn sampled_sphere_at_fifteen_degree_steps_has_a_certified_convex_hull() {
    // Each latitude-band quad is an exactly planar trapezoid. With exact
    // 15-degree trigonometry its coplanarity is a certified multiquadratic
    // zero, so the hull is decided under the strict policy.
    let sphere = solid::sphere(Real::from(8_u8), 24, 4);
    let hull = solid::convex_hull(&sphere).expect("certified hull");
    // 24 * 3 ring vertices plus two poles, all extreme: 2V - 4 triangles.
    assert_eq!(hull.triangles.len(), 2 * (24 * 3 + 2) - 4);
}
