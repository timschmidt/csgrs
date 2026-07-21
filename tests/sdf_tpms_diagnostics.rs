use csgrs::mesh::Mesh;
use hyperlattice::{Point3, Real};

fn r(value: f64) -> Real {
    Real::try_from(value).expect("test values must be finite")
}

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

#[test]
fn sdf_boundary_converts_only_at_sampling_boundary() {
    let mesh: Mesh<()> = Mesh::sdf(
        |p| p.to_vector().norm() - r(0.6),
        (8, 8, 8),
        p3(-1.0, -1.0, -1.0),
        p3(1.0, 1.0, 1.0),
        r(0.0),
        (),
    );

    assert!(!mesh.triangles().is_empty());
}

#[test]
fn tpms_solid_accepts_positive_hyperreal_period_and_thickness() {
    let volume = Mesh::<()>::cube(r(2.0), ());
    let gyroid = volume.gyroid_solid(16, r(2.0), r(0.0), r(0.15), ());

    assert!(!gyroid.triangles().is_empty());
}
