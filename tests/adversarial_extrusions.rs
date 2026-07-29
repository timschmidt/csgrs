use csgrs::{curve, solid};
use hyperlattice::{Point3, Real};

fn r(value: f64) -> Real {
    Real::try_from(value).expect("test values must be finite")
}

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

#[test]
fn loft_revolve_and_sweep_accept_native_hyperreal_geometry() {
    let revolved = curve::revolve(&curve::square(r(1.0)), r(270.0), 24).expect("revolve");
    assert!(!revolved.triangles.is_empty());
    assert!(revolved.has_unique_nondegenerate_triangles());
    assert!(revolved.is_closed_manifold_geometry());

    let path = vec![p3(0.0, 0.0, 0.0), p3(0.0, 0.0, 0.9)];
    let swept = curve::sweep(&curve::square(r(0.08)), &path);
    assert!(!swept.triangles.is_empty());
    assert!(swept.has_unique_nondegenerate_triangles());
    assert!(swept.is_closed_manifold_geometry());

    let bottom = vec![p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0), p3(0.0, 1.0, 0.0)];
    let top = vec![p3(0.0, 0.0, 0.5), p3(0.8, 0.0, 0.5), p3(0.0, 0.8, 0.5)];
    let lofted = solid::loft(&[bottom, top]).expect("loft");
    assert!(!lofted.triangles.is_empty());
    assert!(lofted.has_unique_nondegenerate_triangles());
    assert!(lofted.is_closed_manifold_geometry());
}
