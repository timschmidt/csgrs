#![no_main]

use hyperreal::Real;
use hypervoxel::{
    ExactTriangle3, ExactTriangleSolidMesh, ExactTriangleSurfaceMesh, GridFrame, GridSource,
    MaterialRegionId, PreparedExactTriangleSolidMesh, VoxelizationPolicy,
    voxelize_exact_triangle_solid_mesh, voxelize_prepared_exact_triangle_solid_mesh,
    voxelize_prepared_exact_triangle_solid_mesh_by_axis_sweeps,
    voxelize_prepared_exact_triangle_solid_mesh_by_components,
    voxelize_prepared_exact_triangle_solid_mesh_by_verified_components,
};
use libfuzzer_sys::fuzz_target;

fn r(value: u64) -> Real {
    Real::from(value)
}

fn tri(vertices: [[Real; 3]; 3]) -> ExactTriangle3 {
    ExactTriangle3::new(vertices, Some(0))
}

fuzz_target!(|data: (u8, u8, u8, bool)| {
    let (depth_raw, lo_raw, span_raw, closed_replay) = data;
    let depth = (depth_raw % 3) + 2;
    let frame = GridFrame::builder()
        .depth(depth)
        .source(GridSource::new("fuzz:prepared-triangle-solid", 1))
        .build()
        .unwrap();
    let cells = 1_u64 << depth;
    let lo = 1 + (u64::from(lo_raw) % (cells - 1));
    let hi = (lo + 1 + (u64::from(span_raw) % (cells - lo))).min(cells);
    let p = |x, y, z| [r(x), r(y), r(z)];
    let surface = ExactTriangleSurfaceMesh::new(
        vec![
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
        ],
        frame.source().cloned(),
        true,
    );
    let solid = ExactTriangleSolidMesh::new(surface, closed_replay);
    let prepared = PreparedExactTriangleSolidMesh::prepare(solid.clone());
    if !closed_replay {
        assert!(prepared.is_err());
        return;
    }

    let prepared = prepared.unwrap();
    assert!(prepared.report().exact_prepared_solid_ready);
    let (_, ordinary_report) = voxelize_exact_triangle_solid_mesh(
        frame.clone(),
        &solid,
        MaterialRegionId(1),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    let (_, prepared_report, schedule) = voxelize_prepared_exact_triangle_solid_mesh(
        frame.clone(),
        &prepared,
        MaterialRegionId(1),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    let (_, component_report, components) = voxelize_prepared_exact_triangle_solid_mesh_by_components(
        frame.clone(),
        &prepared,
        MaterialRegionId(1),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    let (_, verified_report, verified_components) =
        voxelize_prepared_exact_triangle_solid_mesh_by_verified_components(
            frame.clone(),
            &prepared,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (_, sweep_report, sweep) = voxelize_prepared_exact_triangle_solid_mesh_by_axis_sweeps(
        frame.clone(),
        &prepared,
        MaterialRegionId(1),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    assert_eq!(
        prepared_report.predicate_certificates,
        ordinary_report.predicate_certificates
    );
    assert_eq!(
        component_report.predicate_certificates,
        ordinary_report.predicate_certificates
    );
    assert_eq!(
        verified_report.predicate_certificates,
        ordinary_report.predicate_certificates
    );
    assert_eq!(
        sweep_report.predicate_certificates,
        ordinary_report.predicate_certificates
    );
    assert_eq!(prepared_report.unknown_cells, ordinary_report.unknown_cells);
    assert_eq!(component_report.unknown_cells, ordinary_report.unknown_cells);
    assert_eq!(verified_report.unknown_cells, ordinary_report.unknown_cells);
    assert_eq!(sweep_report.unknown_cells, ordinary_report.unknown_cells);
    assert!(schedule.boundary_aabb_rejections > 0);
    assert!(schedule.ray_aabb_rejections > 0);
    assert!(schedule.ray_triangle_tests < schedule.ray_attempts * 12);
    assert!(components.boundary_aabb_rejections > 0);
    assert!(components.component_ray_aabb_rejections <= schedule.ray_aabb_rejections);
    assert!(components.component_ray_triangle_tests <= schedule.ray_triangle_tests);
    assert_eq!(verified_components.arrangement_conflicting_cells, 0);
    assert_eq!(verified_components.arrangement_unknown_cells, 0);
    assert_eq!(verified_components.arrangement_boundary_regression_cells, 0);
    assert_eq!(sweep.sweep_classified_cells + sweep.fallback_cells, sweep.open_cells);
    assert_eq!(sweep.fallback_unknown_cells, 0);
    assert_eq!(sweep.fallback_boundary_regression_cells, 0);
    assert!(sweep.exact_axis_sweep_ready);
});
