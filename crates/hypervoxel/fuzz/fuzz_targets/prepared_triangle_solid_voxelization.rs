#![no_main]

use hyperreal::Real;
use hypervoxel::{
    ExactTriangle3, ExactTriangleSolidMesh, ExactTriangleSurfaceMesh, GridFrame, MaterialRegionId,
    PreparedExactTriangleSolidMesh, VoxelizationPolicy, voxelize_exact_triangle_solid_mesh,
    voxelize_prepared_exact_triangle_solid_mesh,
    voxelize_prepared_exact_triangle_solid_mesh_by_adaptive_local_component_consensus,
    voxelize_prepared_exact_triangle_solid_mesh_by_axis_sweeps,
    voxelize_prepared_exact_triangle_solid_mesh_by_component_consensus,
};
use libfuzzer_sys::fuzz_target;

fn r(value: u64) -> Real {
    Real::from(value)
}

fn tri(vertices: [[Real; 3]; 3]) -> ExactTriangle3 {
    ExactTriangle3::new(vertices, Some(0))
}

fuzz_target!(|data: (u8, u8, u8, bool)| {
    let (depth_raw, lo_raw, span_raw, closed_solid) = data;
    let depth = (depth_raw % 3) + 2;
    let frame = GridFrame::builder().depth(depth).build().unwrap();
    let cells = 1_u64 << depth;
    let lo = 1 + (u64::from(lo_raw) % (cells - 1));
    let hi = (lo + 1 + (u64::from(span_raw) % (cells - lo))).min(cells);
    let p = |x, y, z| [r(x), r(y), r(z)];
    let surface = ExactTriangleSurfaceMesh::new(vec![
        tri([p(lo, lo, lo), p(lo, hi, hi), p(lo, hi, lo)]),
        tri([p(lo, lo, lo), p(lo, lo, hi), p(lo, hi, hi)]),
        tri([p(hi, lo, lo), p(hi, hi, lo), p(hi, lo, hi)]),
        tri([p(hi, hi, lo), p(hi, hi, hi), p(hi, lo, hi)]),
        tri([p(lo, lo, lo), p(hi, lo, lo), p(lo, lo, hi)]),
        tri([p(hi, lo, lo), p(hi, lo, hi), p(lo, lo, hi)]),
        tri([p(lo, hi, lo), p(lo, hi, hi), p(hi, hi, lo)]),
        tri([p(hi, hi, lo), p(lo, hi, hi), p(hi, hi, hi)]),
        tri([p(lo, lo, lo), p(lo, hi, lo), p(hi, lo, lo)]),
        tri([p(hi, lo, lo), p(lo, hi, lo), p(hi, hi, lo)]),
        tri([p(lo, lo, hi), p(hi, lo, hi), p(lo, hi, hi)]),
        tri([p(hi, lo, hi), p(hi, hi, hi), p(lo, hi, hi)]),
    ]);
    let solid = ExactTriangleSolidMesh::new(surface, closed_solid);
    let prepared = PreparedExactTriangleSolidMesh::prepare(solid.clone());
    if !closed_solid {
        assert!(prepared.is_err());
        return;
    }
    let prepared = prepared.unwrap();
    let policy = VoxelizationPolicy::conservative_cover();

    let (ordinary, ordinary_report) = voxelize_exact_triangle_solid_mesh(
        frame.clone(),
        &solid,
        MaterialRegionId(1),
        policy.clone(),
    )
    .unwrap();
    let (direct, direct_report, _) = voxelize_prepared_exact_triangle_solid_mesh(
        frame.clone(),
        &prepared,
        MaterialRegionId(1),
        policy.clone(),
    )
    .unwrap();
    let (sweep, sweep_report, _) = voxelize_prepared_exact_triangle_solid_mesh_by_axis_sweeps(
        frame.clone(),
        &prepared,
        MaterialRegionId(1),
        policy.clone(),
    )
    .unwrap();
    let (components, component_report, _) =
        voxelize_prepared_exact_triangle_solid_mesh_by_component_consensus(
            frame.clone(),
            &prepared,
            MaterialRegionId(1),
            policy.clone(),
        )
        .unwrap();
    let (adaptive, adaptive_report, _) =
        voxelize_prepared_exact_triangle_solid_mesh_by_adaptive_local_component_consensus(
            frame,
            &prepared,
            MaterialRegionId(1),
            policy,
        )
        .unwrap();

    assert_eq!(direct, ordinary);
    assert_eq!(sweep, ordinary);
    assert_eq!(components, ordinary);
    assert_eq!(adaptive, ordinary);
    for report in [
        ordinary_report,
        direct_report,
        sweep_report,
        component_report,
        adaptive_report,
    ] {
        assert_eq!(report.unknown_cells, 0);
        assert!(report.predicate_certificates.is_fully_certified());
    }
});
