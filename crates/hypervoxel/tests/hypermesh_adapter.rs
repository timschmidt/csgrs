#![cfg(feature = "hypermesh-adapter")]

use hypermesh::exact::{ExactMesh, ValidationPolicy};
use hypervoxel::{
    GridFrame, GridSource, HypermeshTriangleSolidAdapterBlocker, MaterialRegionId,
    PreparedExactTriangleSolidMesh, VoxelizationPolicy, adapt_hypermesh_exact_solid,
    voxelize_prepared_exact_triangle_solid_mesh,
};

fn tetrahedron_i64() -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, //
            2, 0, 0, //
            0, 2, 0, //
            0, 0, 2,
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap()
}

#[test]
fn hypermesh_exact_solid_adapts_to_prepared_triangle_voxelization() {
    let mesh = tetrahedron_i64();
    let source = GridSource::new("hypermesh:tetrahedron", 1);
    let adapter = adapt_hypermesh_exact_solid(&mesh, None, Some(source)).unwrap();

    assert!(adapter.report.exact_triangle_solid_ready);
    assert!(adapter.report.blockers.is_empty());
    assert_eq!(adapter.report.source_vertex_count, 4);
    assert_eq!(adapter.report.source_triangle_count, 4);
    assert_eq!(adapter.report.emitted_triangle_count, 4);

    let solid = adapter.solid.unwrap();
    assert!(solid.report().exact_solid_source_ready);
    let prepared = PreparedExactTriangleSolidMesh::prepare(solid).unwrap();
    assert!(prepared.report().exact_prepared_solid_ready);
    assert_eq!(prepared.report().prepared_triangle_count, 4);

    let frame = GridFrame::builder()
        .depth(2)
        .source(GridSource::new("voxel:tetrahedron", 1))
        .build()
        .unwrap();
    let (_, report, schedule) = voxelize_prepared_exact_triangle_solid_mesh(
        frame,
        &prepared,
        MaterialRegionId(9),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    assert!(report.predicate_certificates.is_fully_certified());
    assert!(schedule.boundary_aabb_rejections > 0);
}

#[test]
fn hypermesh_adapter_rejects_lossy_or_stale_solid_evidence() {
    let (pos, idx) = (
        vec![
            0.0, 0.0, 0.0, //
            1.0, 0.0, 0.0, //
            0.0, 1.0, 0.0, //
            0.0, 0.0, 1.0,
        ],
        vec![0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    );
    let lossy = ExactMesh::from_f64_triangles(&pos, &idx).unwrap();
    let lossy_adapter = adapt_hypermesh_exact_solid(&lossy, None, None).unwrap();
    assert!(!lossy_adapter.report.exact_triangle_solid_ready);
    assert!(lossy_adapter.solid.is_none());
    assert!(
        lossy_adapter
            .report
            .blockers
            .contains(&HypermeshTriangleSolidAdapterBlocker::SourceNotExact)
    );

    let exact = tetrahedron_i64();
    let mut stale = exact.solid_handoff().unwrap();
    stale.retained_face_planes += 1;
    let stale_adapter = adapt_hypermesh_exact_solid(&exact, Some(&stale), None).unwrap();
    assert!(!stale_adapter.report.exact_triangle_solid_ready);
    assert!(stale_adapter.solid.is_none());
    assert!(
        stale_adapter
            .report
            .blockers
            .contains(&HypermeshTriangleSolidAdapterBlocker::StaleSolidHandoff)
    );

    let open = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_adapter = adapt_hypermesh_exact_solid(&open, None, None).unwrap();
    assert!(
        open_adapter
            .report
            .blockers
            .contains(&HypermeshTriangleSolidAdapterBlocker::SolidHandoffNotReady)
    );
}
