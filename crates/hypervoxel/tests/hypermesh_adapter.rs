#![cfg(feature = "hypermesh-adapter")]

use hyperlimit::PredicatePolicy;
use hypermesh::{MeshCertainty, MeshContext, Point3, Real, Triangle, TriangleMesh};
use hypervoxel::{
    ExactTriangleSolid, GridFrame, MaterialRegionId, VoxelizationPolicy,
    adapt_hypermesh_exact_solid, voxelize_exact_triangle_solid,
};

fn tetrahedron_i64() -> TriangleMesh {
    TriangleMesh::new(
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
fn hypermesh_exact_solid_adapts_to_scheduled_triangle_voxelization() {
    let mesh = tetrahedron_i64();
    let outcome =
        adapt_hypermesh_exact_solid(&MeshContext::new(PredicatePolicy::STRICT), &mesh).unwrap();
    assert_eq!(outcome.certainty, MeshCertainty::Certified);
    let solid = outcome.into_value();
    assert!(solid.report().exact_solid_source_ready);
    let solid = ExactTriangleSolid::new(solid).unwrap();
    assert_eq!(solid.triangle_count(), 4);

    let frame = GridFrame::unit(2).unwrap();
    let (_, report, schedule) = voxelize_exact_triangle_solid(
        frame,
        &solid,
        MaterialRegionId(9),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    assert!(report.predicate_certificates.is_fully_certified());
    assert!(schedule.boundary_aabb_rejections > 0);
}

#[test]
fn hypermesh_adapter_rejects_open_solid_evidence() {
    let open = TriangleMesh::new(
        vec![point(0, 0, 0), point(1, 0, 0), point(0, 1, 0)],
        vec![Triangle::new(0, 1, 2)],
    );
    assert!(
        adapt_hypermesh_exact_solid(&MeshContext::new(PredicatePolicy::STRICT), &open).is_err()
    );
}
