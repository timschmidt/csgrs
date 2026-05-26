#![no_main]

use hyperreal::Real;
use hypervoxel::{
    ExactTriangle3, ExactTriangleSolidMesh, ExactTriangleSurfaceMesh, GridFrame, GridSource,
    MaterialRegionId, PreparedExactTriangleSolidMesh, VoxelizationPolicy,
    voxelize_exact_triangle_solid_mesh, voxelize_prepared_exact_triangle_solid_mesh,
    voxelize_prepared_exact_triangle_solid_mesh_by_adaptive_local_component_consensus,
    voxelize_prepared_exact_triangle_solid_mesh_by_axis_sweeps,
    voxelize_prepared_exact_triangle_solid_mesh_by_component_consensus,
    voxelize_prepared_exact_triangle_solid_mesh_by_components,
    voxelize_prepared_exact_triangle_solid_mesh_by_consensus_axis_sweeps,
    voxelize_prepared_exact_triangle_solid_mesh_by_local_component_consensus,
    voxelize_prepared_exact_triangle_solid_mesh_by_verified_adaptive_local_component_consensus,
    voxelize_prepared_exact_triangle_solid_mesh_by_verified_component_consensus,
    voxelize_prepared_exact_triangle_solid_mesh_by_verified_consensus_axis_sweeps,
    voxelize_prepared_exact_triangle_solid_mesh_by_verified_components,
    voxelize_prepared_exact_triangle_solid_mesh_by_verified_local_component_consensus,
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
    let (_, consensus_report, consensus) =
        voxelize_prepared_exact_triangle_solid_mesh_by_consensus_axis_sweeps(
            frame.clone(),
            &prepared,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (_, verified_consensus_report, verified_consensus) =
        voxelize_prepared_exact_triangle_solid_mesh_by_verified_consensus_axis_sweeps(
            frame.clone(),
            &prepared,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (_, component_consensus_report, component_consensus) =
        voxelize_prepared_exact_triangle_solid_mesh_by_component_consensus(
            frame.clone(),
            &prepared,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (_, verified_component_report, verified_component) =
        voxelize_prepared_exact_triangle_solid_mesh_by_verified_component_consensus(
            frame.clone(),
            &prepared,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (_, local_component_report, local_component) =
        voxelize_prepared_exact_triangle_solid_mesh_by_local_component_consensus(
            frame.clone(),
            &prepared,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (_, verified_local_report, verified_local) =
        voxelize_prepared_exact_triangle_solid_mesh_by_verified_local_component_consensus(
            frame.clone(),
            &prepared,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (_, adaptive_local_report, adaptive_local) =
        voxelize_prepared_exact_triangle_solid_mesh_by_adaptive_local_component_consensus(
            frame.clone(),
            &prepared,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (_, verified_adaptive_local_report, verified_adaptive_local) =
        voxelize_prepared_exact_triangle_solid_mesh_by_verified_adaptive_local_component_consensus(
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
    assert_eq!(
        consensus_report.predicate_certificates,
        ordinary_report.predicate_certificates
    );
    assert_eq!(
        verified_consensus_report.predicate_certificates,
        ordinary_report.predicate_certificates
    );
    assert_eq!(
        component_consensus_report.predicate_certificates,
        ordinary_report.predicate_certificates
    );
    assert_eq!(
        verified_component_report.predicate_certificates,
        ordinary_report.predicate_certificates
    );
    assert_eq!(
        local_component_report.predicate_certificates,
        ordinary_report.predicate_certificates
    );
    assert_eq!(
        verified_local_report.predicate_certificates,
        ordinary_report.predicate_certificates
    );
    assert_eq!(
        adaptive_local_report.predicate_certificates,
        ordinary_report.predicate_certificates
    );
    assert_eq!(
        verified_adaptive_local_report.predicate_certificates,
        ordinary_report.predicate_certificates
    );
    assert_eq!(prepared_report.unknown_cells, ordinary_report.unknown_cells);
    assert_eq!(component_report.unknown_cells, ordinary_report.unknown_cells);
    assert_eq!(verified_report.unknown_cells, ordinary_report.unknown_cells);
    assert_eq!(sweep_report.unknown_cells, ordinary_report.unknown_cells);
    assert_eq!(consensus_report.unknown_cells, ordinary_report.unknown_cells);
    assert_eq!(
        verified_consensus_report.unknown_cells,
        ordinary_report.unknown_cells
    );
    assert_eq!(
        component_consensus_report.unknown_cells,
        ordinary_report.unknown_cells
    );
    assert_eq!(
        verified_component_report.unknown_cells,
        ordinary_report.unknown_cells
    );
    assert_eq!(
        local_component_report.unknown_cells,
        ordinary_report.unknown_cells
    );
    assert_eq!(verified_local_report.unknown_cells, ordinary_report.unknown_cells);
    assert_eq!(adaptive_local_report.unknown_cells, ordinary_report.unknown_cells);
    assert_eq!(
        verified_adaptive_local_report.unknown_cells,
        ordinary_report.unknown_cells
    );
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
    assert_eq!(
        consensus.consensus_classified_cells + consensus.fallback_cells,
        consensus.open_cells
    );
    assert_eq!(consensus.conflicting_vote_cells, 0);
    assert_eq!(consensus.fallback_unknown_cells, 0);
    assert_eq!(consensus.fallback_boundary_regression_cells, 0);
    assert!(consensus.exact_consensus_axis_sweep_ready);
    assert_eq!(verified_consensus.grid_mismatch_cells, 0);
    assert!(verified_consensus.predicate_certificates_match);
    assert!(verified_consensus.boundary_counts_match);
    assert!(verified_consensus.unknown_counts_match);
    assert!(verified_consensus.aggregate_matches);
    assert!(verified_consensus.exact_verified_consensus_axis_sweep_ready);
    assert_eq!(
        component_consensus.consensus_cells
            + component_consensus.exterior_cells
            + component_consensus.retry_consensus_cells
            + component_consensus.fallback_cells,
        component_consensus.open_cells
    );
    assert_eq!(component_consensus.fallback_unknown_cells, 0);
    assert_eq!(component_consensus.fallback_boundary_regression_cells, 0);
    assert_eq!(verified_component.grid_mismatch_cells, 0);
    assert!(verified_component.predicate_certificates_match);
    assert!(verified_component.boundary_counts_match);
    assert!(verified_component.unknown_counts_match);
    assert!(verified_component.aggregate_matches);
    assert!(verified_component
        .component_audit
        .exact_component_consensus_audit_ready);
    assert_eq!(
        local_component.consensus_cells
            + local_component.exterior_cells
            + local_component.retry_consensus_cells
            + local_component.fallback_cells,
        local_component.open_cells
    );
    assert_eq!(local_component.fallback_unknown_cells, 0);
    assert_eq!(local_component.fallback_boundary_regression_cells, 0);
    let local_attempted_rows = local_component.axis_sweep_rows.iter().sum::<usize>();
    assert_eq!(
        local_component.row_cache_lookups,
        local_attempted_rows
    );
    assert_eq!(
        local_component.row_cache_misses,
        local_component.row_candidate_scheduled_rows
    );
    assert_eq!(
        local_component.row_candidate_scheduled_rows + local_component.row_cache_hits,
        local_attempted_rows
    );
    assert_eq!(
        local_component.row_cache_hits + local_component.row_cache_misses,
        local_component.row_cache_lookups
    );
    assert_eq!(
        local_component.row_window_scheduled_rows,
        local_component.row_candidate_scheduled_rows
    );
    assert!(local_component.row_window_aabb_rejections <= local_component.row_candidate_aabb_rejections);
    assert!(local_component.row_cache_broadened_misses <= local_component.row_cache_misses);
    assert_eq!(
        local_component.row_candidate_aabb_rejections,
        local_component.row_ray_aabb_rejections
    );
    assert_eq!(verified_local.grid_mismatch_cells, 0);
    assert!(verified_local.predicate_certificates_match);
    assert!(verified_local.boundary_counts_match);
    assert!(verified_local.unknown_counts_match);
    assert!(verified_local.aggregate_matches);
    assert!(verified_local
        .component_audit
        .exact_component_consensus_audit_ready);
    assert!(verified_local.component_audit.row_cache_accounting_matches);
    assert!(verified_local.component_audit.row_window_accounting_matches);
    assert_eq!(
        adaptive_local.consensus_cells
            + adaptive_local.exterior_cells
            + adaptive_local.retry_consensus_cells
            + adaptive_local.fallback_cells,
        adaptive_local.open_cells
    );
    assert_eq!(adaptive_local.fallback_unknown_cells, 0);
    assert_eq!(adaptive_local.fallback_boundary_regression_cells, 0);
    let adaptive_attempted_rows = adaptive_local.axis_sweep_rows.iter().sum::<usize>();
    assert_eq!(
        adaptive_local.row_cache_lookups,
        adaptive_attempted_rows
    );
    assert_eq!(
        adaptive_local.row_cache_misses,
        adaptive_local.row_candidate_scheduled_rows
    );
    assert_eq!(
        adaptive_local.row_candidate_scheduled_rows + adaptive_local.row_cache_hits,
        adaptive_attempted_rows
    );
    assert_eq!(
        adaptive_local.row_cache_hits + adaptive_local.row_cache_misses,
        adaptive_local.row_cache_lookups
    );
    assert_eq!(
        adaptive_local.row_window_scheduled_rows,
        adaptive_local.row_candidate_scheduled_rows
    );
    assert!(
        adaptive_local.row_window_aabb_rejections
            <= adaptive_local.row_candidate_aabb_rejections
    );
    assert!(adaptive_local.row_cache_broadened_misses <= adaptive_local.row_cache_misses);
    assert_eq!(
        adaptive_local.row_candidate_aabb_rejections,
        adaptive_local.row_ray_aabb_rejections
    );
    assert_eq!(verified_adaptive_local.grid_mismatch_cells, 0);
    assert!(verified_adaptive_local.predicate_certificates_match);
    assert!(verified_adaptive_local.boundary_counts_match);
    assert!(verified_adaptive_local.unknown_counts_match);
    assert!(verified_adaptive_local.aggregate_matches);
    assert!(verified_adaptive_local
        .component_audit
        .exact_component_consensus_audit_ready);
    assert!(verified_adaptive_local
        .component_audit
        .row_cache_accounting_matches);
    assert!(verified_adaptive_local
        .component_audit
        .row_window_accounting_matches);
});
