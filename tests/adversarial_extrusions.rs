//! Fixed segmented-revolution, straight-sweep and triangular-loft mesh witnesses.
//! Native face and edge checks concern the resulting meshes, not analytic-source fidelity.

use csgrs::{GeometryContext, curve, solid};
use hyperlattice::{Point3, Real};
use hyperlimit::PredicatePolicy;
use hypermesh::MeshContext;

/// Strict policy shared by the post-construction face and edge predicates.
const CONTEXT: MeshContext = MeshContext::new(PredicatePolicy::STRICT);

/// Requires unique nondegenerate faces and one oppositely directed pair per geometric edge.
/// Short-circuits on the first false predicate and panics on undecided/errors.
/// Despite its name, this does not inspect vertex links, self-intersection or source fidelity.
fn is_exact_solid(mesh: &hypermesh::TriangleMesh) -> bool {
    mesh.has_unique_nondegenerate_triangles(&CONTEXT)
        .expect("strict mesh predicate")
        .into_value()
        && mesh
            .is_closed_manifold_geometry(&CONTEXT)
            .expect("strict mesh predicate")
            .into_value()
}

/// Imports the exact binary value of a finite fixture float, panicking otherwise.
fn r(value: f64) -> Real {
    Real::try_from(value).expect("test values must be finite")
}

/// Builds a native point from three independently exact finite-float imports.
fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

#[test]
/// Checks nonempty validated meshes from a 270-degree/24-step square revolution,
/// a two-point straight square sweep and a loft between two corresponding triangles.
/// Local construction receipts are extracted without separate certainty assertions.
fn loft_revolve_and_sweep_accept_native_hyperreal_geometry() {
    let revolved = curve::revolve(
        &curve::square(r(1.0)).expect("valid planar fixture"),
        r(270.0),
        24,
        GeometryContext::STRICT,
    )
    .expect("revolve")
    .into_value();
    assert!(!revolved.triangles.is_empty());
    assert!(is_exact_solid(&revolved));

    let path = vec![p3(0.0, 0.0, 0.0), p3(0.0, 0.0, 0.9)];
    let swept = curve::try_sweep(
        &curve::square(r(0.08)).expect("valid planar fixture"),
        &path,
        GeometryContext::STRICT,
    )
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
