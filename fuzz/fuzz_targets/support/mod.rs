use hypermesh::TriangleMesh;

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
    assert!(
        mesh.has_unique_nondegenerate_triangles(),
        "mesh materialized duplicate or degenerate exact triangle geometry"
    );
    if require_closed_solid && !mesh.triangles.is_empty() {
        assert!(
            mesh.is_closed_manifold_geometry(),
            "solid constructor materialized an open, inconsistent, or geometrically non-manifold boundary"
        );
    }
}
