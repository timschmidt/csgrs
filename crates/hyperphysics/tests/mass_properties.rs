use hyperphysics::{
    BodyId, BodyKind, ClosedTriangleMesh3, ExactBody3, ExactFixture3, ExactMaterial, FixtureId,
    MaterialId, PhysicsError, PhysicsShape3, Real, Triangle3, Vector3,
};
use hyperreal::Rational;
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).unwrap())
}

fn v(x: i32, y: i32, z: i32) -> Vector3 {
    Vector3::new([r(x), r(y), r(z)])
}

fn tetra_mesh(scale: i32) -> ClosedTriangleMesh3 {
    let o = v(0, 0, 0);
    let x = v(scale, 0, 0);
    let y = v(0, scale, 0);
    let z = v(0, 0, scale);
    ClosedTriangleMesh3::new(vec![
        Triangle3::new([o.clone(), z.clone(), y.clone()]),
        Triangle3::new([o.clone(), x.clone(), z.clone()]),
        Triangle3::new([o, y.clone(), x.clone()]),
        Triangle3::new([x, y, z]),
    ])
    .unwrap()
}

#[test]
fn tetrahedron_mass_properties_are_exact_rationals() {
    let report = tetra_mesh(1).uniform_density_mass_properties(r(1)).unwrap();

    assert_eq!(report.volume, q(1, 6));
    assert_eq!(report.mass, q(1, 6));
    assert_eq!(
        report.center_of_mass,
        Vector3::new([q(1, 4), q(1, 4), q(1, 4)])
    );
    assert_eq!(report.inertia_about_origin.xx, q(1, 30));
    assert_eq!(report.inertia_about_origin.yy, q(1, 30));
    assert_eq!(report.inertia_about_origin.zz, q(1, 30));
    assert_eq!(report.inertia_about_origin.xy, q(-1, 120));
    assert_eq!(report.inertia_about_center_of_mass.xx, q(1, 80));
    assert_eq!(report.inertia_about_center_of_mass.xy, q(1, 480));
    assert_eq!(report.certificate.triangle_count, 4);
    assert!(!report.certificate.orientation_was_negative);
}

#[test]
fn inward_orientation_is_reported_but_volume_stays_physical() {
    let o = v(0, 0, 0);
    let x = v(1, 0, 0);
    let y = v(0, 1, 0);
    let z = v(0, 0, 1);
    let inward = ClosedTriangleMesh3::new(vec![
        Triangle3::new([o.clone(), y.clone(), z.clone()]),
        Triangle3::new([o.clone(), z.clone(), x.clone()]),
        Triangle3::new([o, x.clone(), y.clone()]),
        Triangle3::new([x, z, y]),
    ])
    .unwrap();

    let report = inward.uniform_density_mass_properties(r(2)).unwrap();

    assert_eq!(report.signed_volume, q(-1, 6));
    assert_eq!(report.volume, q(1, 6));
    assert_eq!(report.mass, q(1, 3));
    assert_eq!(
        report.center_of_mass,
        Vector3::new([q(1, 4), q(1, 4), q(1, 4)])
    );
    assert!(report.certificate.orientation_was_negative);
}

#[test]
fn invalid_density_and_zero_volume_are_rejected() {
    assert_eq!(
        ExactMaterial::new(MaterialId::new("bad").unwrap(), "bad", r(0)).unwrap_err(),
        PhysicsError::NonPositiveDensity
    );

    let flat = ClosedTriangleMesh3::new(vec![Triangle3::new([v(0, 0, 0), v(1, 0, 0), v(0, 1, 0)])])
        .unwrap();
    assert_eq!(
        flat.uniform_density_mass_properties(r(1)).unwrap_err(),
        PhysicsError::ZeroVolume
    );
}

#[test]
fn body_fixture_boundary_keeps_shape_and_material_without_engine_adapter() {
    let material = ExactMaterial::new(MaterialId::new("steel").unwrap(), "Steel", r(7850)).unwrap();
    let shape = PhysicsShape3::closed_triangle_mesh(tetra_mesh(1));
    let fixture = ExactFixture3::new(FixtureId::new("tetra").unwrap(), shape, material.clone());
    let body = ExactBody3::new(
        BodyId::new("body").unwrap(),
        BodyKind::Dynamic,
        vec![fixture],
    );

    assert_eq!(body.kind(), BodyKind::Dynamic);
    assert_eq!(body.fixtures()[0].material(), &material);
    match body.fixtures()[0].shape() {
        PhysicsShape3::ClosedTriangleMesh(mesh) => assert_eq!(mesh.triangle_count(), 4),
        other => panic!("expected closed triangle mesh fixture, got {other:?}"),
    }
}

proptest! {
    #[test]
    fn scaled_axis_tetrahedra_have_exact_cubic_volume_and_linear_centroid(scale in 1_i32..16) {
        let report = tetra_mesh(scale).uniform_density_mass_properties(r(1)).unwrap();
        let cube = i64::from(scale).pow(3);

        prop_assert_eq!(report.volume, q(cube, 6));
        prop_assert_eq!(
            report.center_of_mass,
            Vector3::new([q(i64::from(scale), 4), q(i64::from(scale), 4), q(i64::from(scale), 4)])
        );
        prop_assert_eq!(report.certificate.triangle_count, 4);
    }
}
