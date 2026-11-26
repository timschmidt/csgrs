//! Shared assertions for materialized native meshes in root fuzz targets.
//! These checks inspect the returned mesh, not its fidelity to an authored source
//! or completeness of an operation that may have failed before this helper ran.

use hyperlimit::PredicatePolicy;
use hypermesh::{MeshContext, TriangleMesh};

/// Fixed strict policy for geometric uniqueness and optional edge-pairing checks.
const MESH_CONTEXT: MeshContext = MeshContext::new(PredicatePolicy::STRICT);

/// Asserts valid indices, finite exact coordinates and unique nondegenerate triangles.
/// With `require_closed_solid`, additionally checks exact geometric edge pairing
/// for nonempty triangle lists; empty output skips that check. Native errors,
/// including undecided validation, panic through `expect`. Receipts are not retained.
/// These checks do not establish intended volume, global non-self-intersection,
/// analytic fidelity or the full regularized-set semantics of a Boolean result.
pub fn validate_triangle_mesh(mesh: &TriangleMesh, require_closed_solid: bool) {
    assert!(
        mesh.triangles.iter().all(|triangle| {
            triangle
                .indices()
                .into_iter()
                .all(|index| index < mesh.positions.len())
        }),
        "triangle index escaped the exact position buffer"
    );
    assert!(
        mesh.positions.iter().all(|position| {
            position.x.is_finite() && position.y.is_finite() && position.z.is_finite()
        }),
        "mesh materialized a non-finite exact position"
    );
    let unique = mesh
        .has_unique_nondegenerate_triangles(&MESH_CONTEXT)
        .expect("strict exact triangle validation must decide");
    assert!(
        unique.value,
        "mesh materialized duplicate or degenerate exact triangle geometry"
    );
    if require_closed_solid && !mesh.triangles.is_empty() {
        let closed = mesh
            .is_closed_manifold_geometry(&MESH_CONTEXT)
            .expect("strict exact manifold validation must decide");
        assert!(
            closed.value,
            "solid constructor materialized an open, inconsistent, or geometrically non-manifold boundary"
        );
    }
}
