#![cfg(feature = "hypermesh-adapter")]

use hypermesh::{InputMesh, Point3, Real, Triangle};
use hypervoxel::{
    GridFrame, GridSource, HypermeshTriangleSolidAdapterBlocker, MaterialRegionId,
    PreparedExactTriangleSolidMesh, VoxelizationPolicy, adapt_hypermesh_exact_solid,
    voxelize_prepared_exact_triangle_solid_mesh,
};

fn tetrahedron_i64() -> InputMesh {
    InputMesh::new(
        vec![
            point(0, 0, 0),
            point(2, 0, 0),
            point(0, 2, 0),
            point(0, 0, 2),
        ],
        vec![
            Triangle::new(0, 2, 1),
            Triangle::new(0, 1, 3),
            Triangle::new(1, 2, 3),
            Triangle::new(2, 0, 3),
        ],
    )
}

fn point(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

#[test]
fn hypermesh_exact_solid_adapts_to_prepared_triangle_voxelization() {
    let mesh = tetrahedron_i64();
    let source = GridSource::new("hypermesh:tetrahedron", 1);
    let adapter = adapt_hypermesh_exact_solid(&mesh, Some(source)).unwrap();

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
fn hypermesh_adapter_rejects_open_solid_evidence() {
    let open = InputMesh::new(
        vec![point(0, 0, 0), point(1, 0, 0), point(0, 1, 0)],
        vec![Triangle::new(0, 1, 2)],
    );
    let open_adapter = adapt_hypermesh_exact_solid(&open, None).unwrap();
    assert!(
        open_adapter
            .report
            .blockers
            .contains(&HypermeshTriangleSolidAdapterBlocker::SolidHandoffNotReady)
    );
}
