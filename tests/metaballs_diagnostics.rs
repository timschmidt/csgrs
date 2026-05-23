use csgrs::mesh::{Mesh, metaballs::MetaBall};
use hyperlattice::{Point3, Real};

fn r(value: f64) -> Real {
    Real::try_from(value).expect("test values must be finite")
}

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

#[test]
fn metaballs_generate_mesh_from_hyperreal_centers() {
    let balls = [
        MetaBall::new(p3(-0.35, 0.0, 0.0), r(0.6)),
        MetaBall::new(p3(0.35, 0.0, 0.0), r(0.6)),
    ];
    let mesh: Mesh<()> = Mesh::metaballs(&balls, (10, 10, 10), r(0.35), r(0.2), ());

    assert!(!mesh.polygons.is_empty());
}

#[test]
fn metaball_influence_at_center_is_finite_hyperreal_value() {
    let ball = MetaBall::new(p3(0.0, 0.0, 0.0), r(1.0));
    let influence = ball.influence(&p3(0.0, 0.0, 0.0));

    assert!(influence > r(0.0));
}
