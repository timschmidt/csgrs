use csgrs::{csg::CSG, polygon::Polygon, sketch::Profile, vertex::Vertex};
use hyperlattice::{Point3, Real, Vector3};

fn r(value: f64) -> Real {
    Real::try_from(value).expect("test values must be finite")
}

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

#[test]
fn loft_revolve_and_sweep_accept_hyperreal_geometry() {
    let square = Profile::square(r(1.0));
    let revolved = square
        .translate(r(1.0), r(0.0), r(0.0))
        .revolve(r(270.0), 24, ());
    assert!(revolved.expect("revolve").polygons.len() > 0);

    let path = vec![p3(0.0, 0.0, 0.0), p3(0.0, 0.0, 0.9)];
    let swept = Profile::square(r(0.08)).sweep(&path, ());
    assert!(!swept.polygons.is_empty());

    let bottom = Polygon::new(
        vec![
            Vertex::new(p3(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(p3(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(p3(0.0, 1.0, 0.0), Vector3::z()),
        ],
        (),
    );
    let top = Polygon::new(
        vec![
            Vertex::new(p3(0.0, 0.0, 0.5), Vector3::z()),
            Vertex::new(p3(0.8, 0.0, 0.5), Vector3::z()),
            Vertex::new(p3(0.0, 0.8, 0.5), Vector3::z()),
        ],
        (),
    );
    assert!(
        !Profile::loft(&bottom, &top, true)
            .expect("loft")
            .polygons
            .is_empty()
    );
}
