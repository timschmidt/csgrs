use csgrs::{GeometryContext, curve, solid};
use hyperlattice::{Point3, Real};
use hyperlimit::PredicatePolicy;
use hypermesh::MeshContext;

const CONTEXT: MeshContext = MeshContext::new(PredicatePolicy::STRICT);

fn is_exact_solid(mesh: &hypermesh::TriangleMesh) -> bool {
    mesh.has_unique_nondegenerate_triangles(&CONTEXT)
        .expect("strict mesh predicate")
        .into_value()
        && mesh
            .is_closed_manifold_geometry(&CONTEXT)
            .expect("strict mesh predicate")
            .into_value()
}

fn r(value: f64) -> Real {
    Real::try_from(value).expect("test values must be finite")
}

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

#[test]
fn loft_revolve_and_sweep_accept_native_hyperreal_geometry() {
    let revolved =
        curve::revolve(&curve::square(r(1.0)), r(270.0), 24, &GeometryContext::STRICT)
            .expect("revolve")
            .into_value();
    assert!(!revolved.triangles.is_empty());
    assert!(is_exact_solid(&revolved));

    let path = vec![p3(0.0, 0.0, 0.0), p3(0.0, 0.0, 0.9)];
    let swept = curve::try_sweep(&curve::square(r(0.08)), &path, &GeometryContext::STRICT)
        .expect("sweep")
        .into_value();
    assert!(!swept.triangles.is_empty());
    assert!(is_exact_solid(&swept));

    let bottom = vec![p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0), p3(0.0, 1.0, 0.0)];
    let top = vec![p3(0.0, 0.0, 0.5), p3(0.8, 0.0, 0.5), p3(0.0, 0.8, 0.5)];
    let lofted = solid::loft(&[bottom, top]).expect("loft");
    assert!(!lofted.triangles.is_empty());
    assert!(is_exact_solid(&lofted));
}
