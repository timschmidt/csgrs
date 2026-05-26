use hyperreal::{Rational, Real};
use hypervoxel::{
    BoundaryPolicy, ExactTriangle3, ExactTriangleSolidMesh, ExactTriangleSurfaceMesh, GridFrame,
    GridSource, HypervoxelError, MaterialRegionId, OccupancyState, PreparedExactTriangleSolidMesh,
    QuantizationPolicy, VoxelAddress, VoxelTriangleMeshClassifier, VoxelTriangleSolidClassifier,
    VoxelizationPolicy, classify_cell_against_prepared_triangle_solid_mesh,
    classify_cell_against_triangle_solid_mesh, classify_cell_against_triangle_surface_mesh,
    voxelize_exact_triangle_solid_mesh, voxelize_exact_triangle_surface_mesh,
    voxelize_prepared_exact_triangle_solid_mesh,
    voxelize_prepared_exact_triangle_solid_mesh_by_adaptive_axis_sweeps,
    voxelize_prepared_exact_triangle_solid_mesh_by_adaptive_local_component_consensus,
    voxelize_prepared_exact_triangle_solid_mesh_by_axis_sweeps,
    voxelize_prepared_exact_triangle_solid_mesh_by_component_consensus,
    voxelize_prepared_exact_triangle_solid_mesh_by_components,
    voxelize_prepared_exact_triangle_solid_mesh_by_consensus_axis_sweeps,
    voxelize_prepared_exact_triangle_solid_mesh_by_local_component_consensus,
    voxelize_prepared_exact_triangle_solid_mesh_by_verified_adaptive_axis_sweeps,
    voxelize_prepared_exact_triangle_solid_mesh_by_verified_adaptive_local_component_consensus,
    voxelize_prepared_exact_triangle_solid_mesh_by_verified_component_consensus,
    voxelize_prepared_exact_triangle_solid_mesh_by_verified_components,
    voxelize_prepared_exact_triangle_solid_mesh_by_verified_consensus_axis_sweeps,
    voxelize_prepared_exact_triangle_solid_mesh_by_verified_local_component_consensus,
};
use proptest::prelude::*;

fn r(value: i64) -> Real {
    value.into()
}

fn rf(n: i64, d: u64) -> Real {
    Rational::fraction(n, d).unwrap().into()
}

fn frame(depth: u8) -> GridFrame {
    GridFrame::builder()
        .depth(depth)
        .source(GridSource::new("mesh:triangle-surface", 1))
        .build()
        .unwrap()
}

fn tri(vertices: [[Real; 3]; 3]) -> ExactTriangle3 {
    ExactTriangle3::new(vertices, Some(0))
}

fn cube_surface(min: i64, max: i64, frame: &GridFrame) -> ExactTriangleSurfaceMesh {
    let triangles = rectangular_box_triangles([min, min, min], [max, max, max]);
    ExactTriangleSurfaceMesh::new(triangles, frame.source().cloned(), true)
}

fn rectangular_box_triangles(min: [i64; 3], max: [i64; 3]) -> Vec<ExactTriangle3> {
    let p = |x, y, z| [r(x), r(y), r(z)];
    vec![
        tri([
            p(min[0], min[1], min[2]),
            p(min[0], max[1], max[2]),
            p(min[0], max[1], min[2]),
        ]),
        tri([
            p(min[0], min[1], min[2]),
            p(min[0], min[1], max[2]),
            p(min[0], max[1], max[2]),
        ]),
        tri([
            p(max[0], min[1], min[2]),
            p(max[0], max[1], min[2]),
            p(max[0], min[1], max[2]),
        ]),
        tri([
            p(max[0], max[1], min[2]),
            p(max[0], max[1], max[2]),
            p(max[0], min[1], max[2]),
        ]),
        tri([
            p(min[0], min[1], min[2]),
            p(max[0], min[1], min[2]),
            p(min[0], min[1], max[2]),
        ]),
        tri([
            p(max[0], min[1], min[2]),
            p(max[0], min[1], max[2]),
            p(min[0], min[1], max[2]),
        ]),
        tri([
            p(min[0], max[1], min[2]),
            p(min[0], max[1], max[2]),
            p(max[0], max[1], min[2]),
        ]),
        tri([
            p(max[0], max[1], min[2]),
            p(min[0], max[1], max[2]),
            p(max[0], max[1], max[2]),
        ]),
        tri([
            p(min[0], min[1], min[2]),
            p(min[0], max[1], min[2]),
            p(max[0], min[1], min[2]),
        ]),
        tri([
            p(max[0], min[1], min[2]),
            p(min[0], max[1], min[2]),
            p(max[0], max[1], min[2]),
        ]),
        tri([
            p(min[0], min[1], max[2]),
            p(max[0], min[1], max[2]),
            p(min[0], max[1], max[2]),
        ]),
        tri([
            p(max[0], min[1], max[2]),
            p(max[0], max[1], max[2]),
            p(min[0], max[1], max[2]),
        ]),
    ]
}

fn sheared_box_triangles(min: i64, max: i64, offset: i64) -> Vec<ExactTriangle3> {
    let p = |x: i64, y: i64, z: i64| [r(x + offset) + rf(z, 2), r(y), r(z)];
    vec![
        tri([p(min, min, min), p(min, max, max), p(min, max, min)]),
        tri([p(min, min, min), p(min, min, max), p(min, max, max)]),
        tri([p(max, min, min), p(max, max, min), p(max, min, max)]),
        tri([p(max, max, min), p(max, max, max), p(max, min, max)]),
        tri([p(min, min, min), p(max, min, min), p(min, min, max)]),
        tri([p(max, min, min), p(max, min, max), p(min, min, max)]),
        tri([p(min, max, min), p(min, max, max), p(max, max, min)]),
        tri([p(max, max, min), p(min, max, max), p(max, max, max)]),
        tri([p(min, min, min), p(min, max, min), p(max, min, min)]),
        tri([p(max, min, min), p(min, max, min), p(max, max, min)]),
        tri([p(min, min, max), p(max, min, max), p(min, max, max)]),
        tri([p(max, min, max), p(max, max, max), p(min, max, max)]),
    ]
}

fn two_sheared_aligned_box_surface(frame: &GridFrame) -> ExactTriangleSurfaceMesh {
    let mut triangles = sheared_box_triangles(1, 5, 0);
    triangles.extend(sheared_box_triangles(1, 5, 8));
    ExactTriangleSurfaceMesh::new(triangles, frame.source().cloned(), true)
}

fn assert_component_row_cache_accounting(
    report: &hypervoxel::PreparedTriangleSolidComponentConsensusVoxelizationReport,
) {
    let attempted_rows = report.axis_sweep_rows.iter().sum::<usize>();
    assert_eq!(report.row_cache_lookups, attempted_rows);
    assert_eq!(report.row_cache_misses, report.row_candidate_scheduled_rows);
    assert_eq!(
        report.row_candidate_scheduled_rows + report.row_cache_hits,
        attempted_rows
    );
    assert_eq!(
        report.row_cache_hits + report.row_cache_misses,
        report.row_cache_lookups
    );
}

fn sheared_box_surface(min: i64, max: i64, frame: &GridFrame) -> ExactTriangleSurfaceMesh {
    let triangles = sheared_box_triangles(min, max, 0);
    ExactTriangleSurfaceMesh::new(triangles, frame.source().cloned(), true)
}

#[test]
fn exact_triangle_surface_voxelizes_plane_cover_as_boundary_cells() {
    let frame = frame(1);
    let source = frame.source().cloned();
    let mesh = ExactTriangleSurfaceMesh::new(
        vec![
            tri([[r(0), r(0), r(1)], [r(2), r(0), r(1)], [r(0), r(2), r(1)]]),
            tri([[r(2), r(0), r(1)], [r(2), r(2), r(1)], [r(0), r(2), r(1)]]),
        ],
        source,
        true,
    );

    let (grid, report) = voxelize_exact_triangle_surface_mesh(
        frame,
        &mesh,
        MaterialRegionId(3),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();

    assert_eq!(grid.len(), 8);
    assert_eq!(report.boundary_cells, 8);
    assert_eq!(report.unknown_cells, 0);
    assert_eq!(report.predicate_certificates.boundary_cells, 8);
    assert!(report.exact_topology_ready());
    assert!(report.source_replay_ready());
    assert!(
        grid.iter()
            .all(|(_, cell)| cell.occupancy == OccupancyState::Boundary)
    );
}

#[test]
fn exact_triangle_surface_classifies_tiny_triangle_against_one_cell() {
    let frame = frame(2);
    let mesh = ExactTriangleSurfaceMesh::new(
        vec![tri([
            [rf(1, 4), rf(1, 4), rf(1, 4)],
            [rf(3, 4), rf(1, 4), rf(1, 4)],
            [rf(1, 4), rf(3, 4), rf(1, 4)],
        ])],
        frame.source().cloned(),
        true,
    );

    let hit = classify_cell_against_triangle_surface_mesh(
        VoxelAddress::new(2, [0, 0, 0]).unwrap(),
        &frame,
        &mesh,
    )
    .unwrap();
    let miss = classify_cell_against_triangle_surface_mesh(
        VoxelAddress::new(2, [3, 3, 3]).unwrap(),
        &frame,
        &mesh,
    )
    .unwrap();

    assert_eq!(hit, VoxelTriangleMeshClassifier::Boundary);
    assert_eq!(miss, VoxelTriangleMeshClassifier::Outside);
}

#[test]
fn exact_triangle_surface_rejects_empty_and_degenerate_sources() {
    let frame = frame(1);
    let empty = ExactTriangleSurfaceMesh::new(vec![], frame.source().cloned(), true);
    assert_eq!(
        voxelize_exact_triangle_surface_mesh(
            frame.clone(),
            &empty,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        ),
        Err(HypervoxelError::InvalidSourceGeometry {
            reason: "triangle surface mesh has no triangles"
        })
    );

    let degenerate = ExactTriangleSurfaceMesh::new(
        vec![tri([
            [r(0), r(0), r(0)],
            [r(1), r(1), r(1)],
            [r(2), r(2), r(2)],
        ])],
        frame.source().cloned(),
        true,
    );
    assert!(degenerate.report().degenerate_triangle_count > 0);
    assert_eq!(
        voxelize_exact_triangle_surface_mesh(
            frame,
            &degenerate,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        ),
        Err(HypervoxelError::InvalidSourceGeometry {
            reason: "triangle surface mesh contains degenerate triangles"
        })
    );
}

#[test]
fn exact_triangle_surface_requires_producer_source_replay() {
    let frame = frame(1);
    let mesh = ExactTriangleSurfaceMesh::new(
        vec![tri([
            [r(0), r(0), r(1)],
            [r(2), r(0), r(1)],
            [r(0), r(2), r(1)],
        ])],
        frame.source().cloned(),
        false,
    );

    assert!(!mesh.report().exact_triangle_source_ready);
    assert_eq!(
        voxelize_exact_triangle_surface_mesh(
            frame,
            &mesh,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        ),
        Err(HypervoxelError::InvalidSourceGeometry {
            reason: "triangle surface mesh lacks exact source replay"
        })
    );
}

#[test]
fn triangle_surface_boundary_policy_keeps_unknown_and_lossy_edges_named() {
    let frame = frame(1);
    let mesh = ExactTriangleSurfaceMesh::new(
        vec![tri([
            [r(0), r(0), r(1)],
            [r(2), r(0), r(1)],
            [r(0), r(2), r(1)],
        ])],
        frame.source().cloned(),
        true,
    );
    let unknown_policy = VoxelizationPolicy {
        quantization: QuantizationPolicy::ConservativeCover,
        boundary: BoundaryPolicy::BoundaryAsUnknown,
    };
    let lossy_policy = VoxelizationPolicy {
        quantization: QuantizationPolicy::ConservativeCover,
        boundary: BoundaryPolicy::LossySideChoice,
    };

    let (_, unknown_report) = voxelize_exact_triangle_surface_mesh(
        frame.clone(),
        &mesh,
        MaterialRegionId(2),
        unknown_policy,
    )
    .unwrap();
    assert!(unknown_report.unknown_cells > 0);
    assert!(!unknown_report.exact_topology_ready());

    let (lossy_grid, lossy_report) =
        voxelize_exact_triangle_surface_mesh(frame, &mesh, MaterialRegionId(2), lossy_policy)
            .unwrap();
    assert!(
        lossy_grid
            .iter()
            .all(|(_, cell)| cell.occupancy == OccupancyState::LossyAdapterValue)
    );
    assert!(!lossy_report.exact_topology_ready());
}

#[test]
fn exact_triangle_solid_voxelizes_closed_cube_boundary_and_interior() {
    let frame = frame(3);
    let solid = ExactTriangleSolidMesh::new(cube_surface(2, 6, &frame), true);

    let center = classify_cell_against_triangle_solid_mesh(
        VoxelAddress::new(3, [3, 3, 3]).unwrap(),
        &frame,
        &solid,
    )
    .unwrap();
    let outside = classify_cell_against_triangle_solid_mesh(
        VoxelAddress::new(3, [0, 0, 0]).unwrap(),
        &frame,
        &solid,
    )
    .unwrap();
    let boundary = classify_cell_against_triangle_solid_mesh(
        VoxelAddress::new(3, [2, 3, 3]).unwrap(),
        &frame,
        &solid,
    )
    .unwrap();

    assert_eq!(center, VoxelTriangleSolidClassifier::Inside);
    assert_eq!(outside, VoxelTriangleSolidClassifier::Outside);
    assert_eq!(boundary, VoxelTriangleSolidClassifier::Boundary);

    let (grid, report) = voxelize_exact_triangle_solid_mesh(
        frame,
        &solid,
        MaterialRegionId(4),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    assert_eq!(report.unknown_cells, 0);
    assert_eq!(report.predicate_certificates.inside_cells, 8);
    // The conservative cover uses closed cell AABBs. A cube face lying on a
    // grid plane is therefore boundary evidence for cells on both incident
    // sides of that plane, leaving only the 2x2x2 strict interior as filled.
    assert_eq!(report.predicate_certificates.boundary_cells, 208);
    assert_eq!(grid.len(), 216);
    assert!(report.exact_topology_ready());
}

#[test]
fn exact_triangle_solid_rejects_missing_closed_solid_replay() {
    let frame = frame(2);
    let solid = ExactTriangleSolidMesh::new(cube_surface(1, 3, &frame), false);

    assert!(!solid.report().exact_solid_source_ready);
    assert_eq!(
        voxelize_exact_triangle_solid_mesh(
            frame,
            &solid,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        ),
        Err(HypervoxelError::InvalidSourceGeometry {
            reason: "triangle solid mesh lacks exact closed-solid replay"
        })
    );
}

#[test]
fn prepared_triangle_solid_replays_cube_with_exact_schedule_report() {
    let frame = frame(3);
    let solid = ExactTriangleSolidMesh::new(cube_surface(2, 6, &frame), true);
    let prepared = PreparedExactTriangleSolidMesh::prepare(solid.clone()).unwrap();

    assert!(prepared.report().exact_prepared_solid_ready);
    assert_eq!(prepared.report().prepared_triangle_count, 12);
    assert_eq!(prepared.report().unknown_bound_count, 0);

    let interior = classify_cell_against_prepared_triangle_solid_mesh(
        VoxelAddress::new(3, [3, 3, 3]).unwrap(),
        &frame,
        &prepared,
    )
    .unwrap();
    assert_eq!(interior.classifier, VoxelTriangleSolidClassifier::Inside);
    assert_eq!(interior.boundary_aabb_rejections, 12);
    assert_eq!(interior.boundary_triangle_tests, 0);
    assert_eq!(interior.ray_attempts.len(), 1);
    assert!(interior.ray_attempts[0].certified);
    assert_eq!(interior.ray_attempts[0].ray_aabb_rejections, 10);
    assert_eq!(interior.ray_attempts[0].triangle_tests, 2);

    let boundary = classify_cell_against_prepared_triangle_solid_mesh(
        VoxelAddress::new(3, [2, 3, 3]).unwrap(),
        &frame,
        &prepared,
    )
    .unwrap();
    assert_eq!(boundary.classifier, VoxelTriangleSolidClassifier::Boundary);
    assert!(boundary.boundary_triangle_tests > 0);
    assert!(boundary.ray_attempts.is_empty());

    let (ordinary_grid, ordinary_report) = voxelize_exact_triangle_solid_mesh(
        frame.clone(),
        &solid,
        MaterialRegionId(4),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    let (prepared_grid, prepared_report, schedule) = voxelize_prepared_exact_triangle_solid_mesh(
        frame,
        &prepared,
        MaterialRegionId(4),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();

    assert_eq!(prepared_grid.len(), ordinary_grid.len());
    assert_eq!(prepared_report.unknown_cells, ordinary_report.unknown_cells);
    assert_eq!(
        prepared_report.predicate_certificates,
        ordinary_report.predicate_certificates
    );
    assert_eq!(schedule.classified_cells, 512);
    assert!(schedule.boundary_aabb_rejections > schedule.boundary_triangle_tests);
    assert!(schedule.ray_aabb_rejections > 0);
    assert!(schedule.ray_triangle_tests < schedule.ray_attempts * 12);
    assert!(schedule.ambiguous_ray_attempts > 0);
    assert!(schedule.ambiguous_ray_attempts < schedule.ray_attempts);
    assert!(prepared_report.exact_topology_ready());
}

#[test]
fn component_prepared_triangle_solid_classifies_components_with_fewer_rays() {
    let frame = frame(3);
    let solid = ExactTriangleSolidMesh::new(cube_surface(2, 6, &frame), true);
    let prepared = PreparedExactTriangleSolidMesh::prepare(solid).unwrap();
    let (per_cell_grid, per_cell_report, per_cell_schedule) =
        voxelize_prepared_exact_triangle_solid_mesh(
            frame.clone(),
            &prepared,
            MaterialRegionId(4),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (component_grid, component_report, components) =
        voxelize_prepared_exact_triangle_solid_mesh_by_components(
            frame.clone(),
            &prepared,
            MaterialRegionId(4),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (verified_grid, verified_report, verified_components) =
        voxelize_prepared_exact_triangle_solid_mesh_by_verified_components(
            frame,
            &prepared,
            MaterialRegionId(4),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();

    assert_eq!(component_grid.len(), per_cell_grid.len());
    assert_eq!(verified_grid.len(), per_cell_grid.len());
    assert_eq!(
        component_report.predicate_certificates,
        per_cell_report.predicate_certificates
    );
    assert_eq!(
        verified_report.predicate_certificates,
        per_cell_report.predicate_certificates
    );
    assert_eq!(component_report.unknown_cells, 0);
    assert_eq!(verified_report.unknown_cells, 0);
    assert_eq!(components.classified_cells, 512);
    assert_eq!(components.boundary_cells, 208);
    assert_eq!(components.open_cells, 304);
    assert_eq!(components.components, 2);
    assert_eq!(components.exterior_components, 1);
    assert_eq!(components.ray_classified_components, 1);
    assert_eq!(components.inside_components, 1);
    assert_eq!(components.outside_components, 1);
    assert_eq!(components.unknown_components, 0);
    assert!(components.component_ray_aabb_rejections > 0);
    assert!(components.component_ray_triangle_tests < per_cell_schedule.ray_triangle_tests);
    assert!(component_report.exact_topology_ready());
    assert_eq!(verified_components.arrangement_verified_components, 1);
    assert_eq!(verified_components.arrangement_verified_cells, 7);
    assert_eq!(verified_components.arrangement_conflicting_cells, 0);
    assert_eq!(verified_components.arrangement_unknown_cells, 0);
    assert_eq!(verified_components.arrangement_boundary_regression_cells, 0);
    assert!(verified_components.arrangement_ray_attempts > 0);
    assert!(verified_components.arrangement_ray_aabb_rejections > 0);
    assert!(verified_report.exact_topology_ready());
}

#[test]
fn axis_sweep_prepared_triangle_solid_batches_exact_row_parity() {
    let frame = frame(3);
    let solid = ExactTriangleSolidMesh::new(cube_surface(2, 6, &frame), true);
    let prepared = PreparedExactTriangleSolidMesh::prepare(solid).unwrap();
    let (per_cell_grid, per_cell_report, per_cell_schedule) =
        voxelize_prepared_exact_triangle_solid_mesh(
            frame.clone(),
            &prepared,
            MaterialRegionId(4),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (sweep_grid, sweep_report, sweep) =
        voxelize_prepared_exact_triangle_solid_mesh_by_axis_sweeps(
            frame,
            &prepared,
            MaterialRegionId(4),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();

    assert_eq!(sweep_grid, per_cell_grid);
    assert_eq!(
        sweep_report.predicate_certificates,
        per_cell_report.predicate_certificates
    );
    assert_eq!(sweep_report.unknown_cells, 0);
    assert_eq!(sweep.classified_cells, 512);
    assert_eq!(sweep.boundary_cells, 208);
    assert_eq!(sweep.open_cells, 304);
    assert_eq!(
        sweep.sweep_classified_cells + sweep.fallback_cells,
        sweep.open_cells
    );
    assert!(sweep.sweep_rows > 0);
    assert!(sweep.certified_sweep_rows > 0);
    assert!(sweep.row_ray_triangle_tests < per_cell_schedule.ray_triangle_tests);
    assert_eq!(sweep.fallback_unknown_cells, 0);
    assert_eq!(sweep.fallback_boundary_regression_cells, 0);
    assert!(sweep.exact_axis_sweep_ready);
    assert!(sweep_report.exact_topology_ready());

    let lossy_policy = VoxelizationPolicy {
        quantization: QuantizationPolicy::ConservativeCover,
        boundary: BoundaryPolicy::LossySideChoice,
    };
    let (_, lossy_report, lossy_sweep) =
        voxelize_prepared_exact_triangle_solid_mesh_by_axis_sweeps(
            GridFrame::builder()
                .depth(3)
                .source(GridSource::new("mesh:triangle-surface", 1))
                .build()
                .unwrap(),
            &prepared,
            MaterialRegionId(4),
            lossy_policy,
        )
        .unwrap();
    assert_eq!(lossy_report.boundary_cells, sweep_report.boundary_cells);
    assert!(lossy_report.aggregate.has_lossy);
    assert!(!lossy_report.exact_topology_ready());
    assert!(lossy_sweep.exact_axis_sweep_ready);
}

#[test]
fn adaptive_axis_sweep_tries_three_exact_arrangement_axes_before_fallback() {
    let frame = frame(3);
    let solid = ExactTriangleSolidMesh::new(cube_surface(2, 6, &frame), true);
    let prepared = PreparedExactTriangleSolidMesh::prepare(solid).unwrap();
    let (per_cell_grid, per_cell_report, per_cell_schedule) =
        voxelize_prepared_exact_triangle_solid_mesh(
            frame.clone(),
            &prepared,
            MaterialRegionId(4),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (x_sweep_grid, _, x_sweep) = voxelize_prepared_exact_triangle_solid_mesh_by_axis_sweeps(
        frame.clone(),
        &prepared,
        MaterialRegionId(4),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    let (adaptive_grid, adaptive_report, adaptive) =
        voxelize_prepared_exact_triangle_solid_mesh_by_adaptive_axis_sweeps(
            frame.clone(),
            &prepared,
            MaterialRegionId(4),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (verified_grid, verified_report, verified) =
        voxelize_prepared_exact_triangle_solid_mesh_by_verified_adaptive_axis_sweeps(
            frame.clone(),
            &prepared,
            MaterialRegionId(4),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();

    assert_eq!(adaptive_grid, per_cell_grid);
    assert_eq!(adaptive_grid, x_sweep_grid);
    assert_eq!(verified_grid, per_cell_grid);
    assert_eq!(
        adaptive_report.predicate_certificates,
        per_cell_report.predicate_certificates
    );
    assert_eq!(
        verified_report.predicate_certificates,
        per_cell_report.predicate_certificates
    );
    assert_eq!(adaptive.boundary_cells, x_sweep.boundary_cells);
    assert_eq!(adaptive.open_cells, x_sweep.open_cells);
    assert_eq!(
        adaptive.sweep_classified_cells + adaptive.fallback_cells,
        adaptive.open_cells
    );
    assert!(adaptive.axis_sweep_rows.iter().all(|rows| *rows > 0));
    assert!(adaptive.axis_certified_sweep_rows[0] > 0);
    assert!(adaptive.axis_certified_sweep_rows.iter().sum::<usize>() > 0);
    assert!(adaptive.fallback_cells <= x_sweep.fallback_cells);
    assert!(adaptive.row_ray_triangle_tests <= per_cell_schedule.ray_triangle_tests);
    assert_eq!(adaptive.fallback_unknown_cells, 0);
    assert_eq!(adaptive.fallback_boundary_regression_cells, 0);
    assert_eq!(adaptive.row_parameter_order_unknowns, 0);
    assert!(adaptive.exact_adaptive_axis_sweep_ready);
    assert!(adaptive_report.exact_topology_ready());
    assert_eq!(verified.compared_cells, 512);
    assert_eq!(verified.grid_mismatch_cells, 0);
    assert!(verified.predicate_certificates_match);
    assert!(verified.boundary_counts_match);
    assert!(verified.unknown_counts_match);
    assert!(verified.aggregate_matches);
    assert!(verified.verifier_exact_topology_ready);
    assert_eq!(verified.adaptive, adaptive);
    assert_eq!(verified.verifier, per_cell_schedule);
    assert!(verified.exact_verified_adaptive_axis_sweep_ready);

    let lossy_policy = VoxelizationPolicy {
        quantization: QuantizationPolicy::ConservativeCover,
        boundary: BoundaryPolicy::LossySideChoice,
    };
    let (_, lossy_report, lossy_verified) =
        voxelize_prepared_exact_triangle_solid_mesh_by_verified_adaptive_axis_sweeps(
            frame,
            &prepared,
            MaterialRegionId(4),
            lossy_policy,
        )
        .unwrap();
    assert!(lossy_report.aggregate.has_lossy);
    assert_eq!(lossy_verified.grid_mismatch_cells, 0);
    assert!(lossy_verified.predicate_certificates_match);
    assert!(!lossy_verified.verifier_exact_topology_ready);
    assert!(!lossy_verified.exact_verified_adaptive_axis_sweep_ready);
}

#[test]
fn consensus_axis_sweep_requires_multi_axis_winding_agreement() {
    let frame = frame(3);
    let solid = ExactTriangleSolidMesh::new(cube_surface(2, 6, &frame), true);
    let prepared = PreparedExactTriangleSolidMesh::prepare(solid).unwrap();
    let (per_cell_grid, per_cell_report, per_cell_schedule) =
        voxelize_prepared_exact_triangle_solid_mesh(
            frame.clone(),
            &prepared,
            MaterialRegionId(4),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (consensus_grid, consensus_report, consensus) =
        voxelize_prepared_exact_triangle_solid_mesh_by_consensus_axis_sweeps(
            frame.clone(),
            &prepared,
            MaterialRegionId(4),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (verified_grid, verified_report, verified) =
        voxelize_prepared_exact_triangle_solid_mesh_by_verified_consensus_axis_sweeps(
            frame.clone(),
            &prepared,
            MaterialRegionId(4),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();

    assert_eq!(consensus_grid, per_cell_grid);
    assert_eq!(verified_grid, per_cell_grid);
    assert_eq!(
        consensus_report.predicate_certificates,
        per_cell_report.predicate_certificates
    );
    assert_eq!(
        verified_report.predicate_certificates,
        per_cell_report.predicate_certificates
    );
    assert_eq!(consensus.boundary_cells, 208);
    assert_eq!(consensus.open_cells, 304);
    assert_eq!(
        consensus.consensus_classified_cells + consensus.fallback_cells,
        consensus.open_cells
    );
    assert!(consensus.axis_sweep_rows.iter().all(|rows| *rows > 0));
    assert!(
        consensus
            .axis_certified_sweep_rows
            .iter()
            .all(|rows| *rows > 0)
    );
    assert!(consensus.voted_cells > 0);
    assert!(consensus.consensus_votes >= consensus.voted_cells);
    assert_eq!(consensus.conflicting_vote_cells, 0);
    assert_eq!(consensus.fallback_unknown_cells, 0);
    assert_eq!(consensus.fallback_boundary_regression_cells, 0);
    assert_eq!(consensus.row_parameter_order_unknowns, 0);
    assert!(consensus.row_ray_triangle_tests <= per_cell_schedule.ray_triangle_tests);
    assert!(consensus.exact_consensus_axis_sweep_ready);
    assert!(consensus_report.exact_topology_ready());

    assert_eq!(verified.compared_cells, 512);
    assert_eq!(verified.grid_mismatch_cells, 0);
    assert!(verified.predicate_certificates_match);
    assert!(verified.boundary_counts_match);
    assert!(verified.unknown_counts_match);
    assert!(verified.aggregate_matches);
    assert!(verified.verifier_exact_topology_ready);
    assert_eq!(verified.consensus, consensus);
    assert_eq!(verified.verifier, per_cell_schedule);
    assert!(verified.exact_verified_consensus_axis_sweep_ready);
}

#[test]
fn component_consensus_handles_non_grid_aligned_solid_with_verifier_replay() {
    let frame = frame(3);
    let solid = ExactTriangleSolidMesh::new(sheared_box_surface(1, 5, &frame), true);
    let prepared = PreparedExactTriangleSolidMesh::prepare(solid).unwrap();
    let (per_cell_grid, per_cell_report, per_cell_schedule) =
        voxelize_prepared_exact_triangle_solid_mesh(
            frame.clone(),
            &prepared,
            MaterialRegionId(12),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (component_grid, component_report, component_consensus) =
        voxelize_prepared_exact_triangle_solid_mesh_by_component_consensus(
            frame.clone(),
            &prepared,
            MaterialRegionId(12),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (verified_grid, verified_report, verified) =
        voxelize_prepared_exact_triangle_solid_mesh_by_verified_component_consensus(
            frame.clone(),
            &prepared,
            MaterialRegionId(12),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();

    assert_eq!(component_grid, per_cell_grid);
    assert_eq!(verified_grid, per_cell_grid);
    assert_eq!(
        component_report.predicate_certificates,
        per_cell_report.predicate_certificates
    );
    assert_eq!(
        verified_report.predicate_certificates,
        per_cell_report.predicate_certificates
    );
    assert_eq!(
        component_consensus.consensus_cells
            + component_consensus.exterior_cells
            + component_consensus.retry_consensus_cells
            + component_consensus.fallback_cells,
        component_consensus.open_cells
    );
    assert!(component_consensus.components > 0);
    assert!(component_consensus.consensus_components > 0);
    assert!(component_consensus.consensus_cells > 0);
    assert_eq!(component_consensus.fallback_unknown_cells, 0);
    assert_eq!(component_consensus.fallback_boundary_regression_cells, 0);
    assert_eq!(component_consensus.row_parameter_order_unknowns, 0);
    assert!(component_consensus.row_votes > 0);
    assert!(component_consensus.exact_component_consensus_ready);

    assert_eq!(verified.compared_cells, 512);
    assert_eq!(verified.grid_mismatch_cells, 0);
    assert!(verified.predicate_certificates_match);
    assert!(verified.boundary_counts_match);
    assert!(verified.unknown_counts_match);
    assert!(verified.aggregate_matches);
    assert!(verified.verifier_exact_topology_ready);
    assert!(
        verified
            .component_audit
            .exact_component_consensus_audit_ready
    );
    assert!(verified.component_audit.open_cell_accounting_matches);
    assert_eq!(verified.component_consensus, component_consensus);
    assert_eq!(verified.verifier, per_cell_schedule);
    assert!(verified.exact_verified_component_consensus_ready);
}

#[test]
fn local_component_consensus_schedules_only_component_rows() {
    let frame = frame(3);
    let solid = ExactTriangleSolidMesh::new(sheared_box_surface(1, 5, &frame), true);
    let prepared = PreparedExactTriangleSolidMesh::prepare(solid).unwrap();
    let (per_cell_grid, per_cell_report, per_cell_schedule) =
        voxelize_prepared_exact_triangle_solid_mesh(
            frame.clone(),
            &prepared,
            MaterialRegionId(13),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (_, _, global_component) =
        voxelize_prepared_exact_triangle_solid_mesh_by_component_consensus(
            frame.clone(),
            &prepared,
            MaterialRegionId(13),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (local_grid, local_report, local_component) =
        voxelize_prepared_exact_triangle_solid_mesh_by_local_component_consensus(
            frame.clone(),
            &prepared,
            MaterialRegionId(13),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (verified_grid, verified_report, verified) =
        voxelize_prepared_exact_triangle_solid_mesh_by_verified_local_component_consensus(
            frame.clone(),
            &prepared,
            MaterialRegionId(13),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();

    assert_eq!(local_grid, per_cell_grid);
    assert_eq!(verified_grid, per_cell_grid);
    assert_eq!(
        local_report.predicate_certificates,
        per_cell_report.predicate_certificates
    );
    assert_eq!(
        verified_report.predicate_certificates,
        per_cell_report.predicate_certificates
    );
    assert_eq!(
        local_component.consensus_cells
            + local_component.exterior_cells
            + local_component.retry_consensus_cells
            + local_component.fallback_cells,
        local_component.open_cells
    );
    assert_eq!(local_component.components, global_component.components);
    assert_eq!(
        local_component.axis_sweep_rows.iter().sum::<usize>(),
        local_component
            .axis_certified_sweep_rows
            .iter()
            .sum::<usize>()
            + local_component
                .axis_ambiguous_sweep_rows
                .iter()
                .sum::<usize>()
    );
    assert_component_row_cache_accounting(&local_component);
    assert!(local_component.row_candidate_triangles > 0);
    assert!(local_component.row_candidate_aabb_rejections > 0);
    assert_eq!(
        local_component.row_candidate_aabb_rejections,
        local_component.row_ray_aabb_rejections
    );
    assert!(
        local_component.axis_sweep_rows.iter().sum::<usize>()
            <= global_component.axis_sweep_rows.iter().sum::<usize>()
    );
    assert!(local_component.row_votes > 0);
    assert_eq!(local_component.fallback_unknown_cells, 0);
    assert_eq!(local_component.fallback_boundary_regression_cells, 0);
    assert_eq!(local_component.row_parameter_order_unknowns, 0);
    assert!(local_component.exact_component_consensus_ready);

    assert_eq!(verified.grid_mismatch_cells, 0);
    assert!(verified.predicate_certificates_match);
    assert!(verified.boundary_counts_match);
    assert!(verified.unknown_counts_match);
    assert!(verified.aggregate_matches);
    assert!(verified.verifier_exact_topology_ready);
    assert!(
        verified
            .component_audit
            .exact_component_consensus_audit_ready
    );
    assert!(verified.component_audit.row_cache_accounting_matches);
    assert!(verified.component_audit.row_candidate_schedule_matches);
    assert!(verified.component_audit.row_candidate_rejections_match);
    assert_eq!(verified.component_consensus, local_component);
    assert_eq!(verified.verifier, per_cell_schedule);
    assert!(verified.exact_verified_component_consensus_ready);
}

#[test]
fn adaptive_local_component_consensus_stops_after_complete_component_proof() {
    let frame = frame(3);
    let solid = ExactTriangleSolidMesh::new(sheared_box_surface(1, 5, &frame), true);
    let prepared = PreparedExactTriangleSolidMesh::prepare(solid).unwrap();
    let (per_cell_grid, per_cell_report, per_cell_schedule) =
        voxelize_prepared_exact_triangle_solid_mesh(
            frame.clone(),
            &prepared,
            MaterialRegionId(14),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (_, _, local_component) =
        voxelize_prepared_exact_triangle_solid_mesh_by_local_component_consensus(
            frame.clone(),
            &prepared,
            MaterialRegionId(14),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (adaptive_grid, adaptive_report, adaptive_component) =
        voxelize_prepared_exact_triangle_solid_mesh_by_adaptive_local_component_consensus(
            frame.clone(),
            &prepared,
            MaterialRegionId(14),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (verified_grid, verified_report, verified) =
        voxelize_prepared_exact_triangle_solid_mesh_by_verified_adaptive_local_component_consensus(
            frame.clone(),
            &prepared,
            MaterialRegionId(14),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();

    assert_eq!(adaptive_grid, per_cell_grid);
    assert_eq!(verified_grid, per_cell_grid);
    assert_eq!(
        adaptive_report.predicate_certificates,
        per_cell_report.predicate_certificates
    );
    assert_eq!(
        verified_report.predicate_certificates,
        per_cell_report.predicate_certificates
    );
    assert_eq!(
        adaptive_component.consensus_cells
            + adaptive_component.exterior_cells
            + adaptive_component.retry_consensus_cells
            + adaptive_component.fallback_cells,
        adaptive_component.open_cells
    );
    assert_eq!(adaptive_component.components, local_component.components);
    assert_eq!(
        adaptive_component.consensus_components,
        local_component.consensus_components
    );
    assert!(
        adaptive_component.axis_sweep_rows.iter().sum::<usize>()
            <= local_component.axis_sweep_rows.iter().sum::<usize>()
    );
    assert!(adaptive_component.axis_sweep_rows[0] > 0);
    assert!(
        adaptive_component.axis_sweep_rows[1] < local_component.axis_sweep_rows[1]
            || adaptive_component.axis_sweep_rows[2] < local_component.axis_sweep_rows[2]
    );
    assert!(adaptive_component.retry_direction_attempts > 0);
    assert!(adaptive_component.retry_ray_attempts > 0);
    assert!(adaptive_component.retry_ray_aabb_rejections > 0);
    assert!(adaptive_component.retry_ray_triangle_tests > 0);
    assert!(adaptive_component.retry_consensus_components > 0);
    assert!(adaptive_component.retry_consensus_cells > 0);
    assert_component_row_cache_accounting(&adaptive_component);
    assert!(adaptive_component.row_candidate_triangles > 0);
    assert!(adaptive_component.row_candidate_aabb_rejections > 0);
    assert_eq!(
        adaptive_component.row_candidate_aabb_rejections,
        adaptive_component.row_ray_aabb_rejections
    );
    assert_eq!(adaptive_component.fallback_unknown_cells, 0);
    assert_eq!(adaptive_component.fallback_boundary_regression_cells, 0);
    assert_eq!(adaptive_component.row_parameter_order_unknowns, 0);
    assert!(adaptive_component.exact_component_consensus_ready);

    assert_eq!(verified.grid_mismatch_cells, 0);
    assert!(verified.predicate_certificates_match);
    assert!(verified.boundary_counts_match);
    assert!(verified.unknown_counts_match);
    assert!(verified.aggregate_matches);
    assert!(verified.verifier_exact_topology_ready);
    assert!(
        verified
            .component_audit
            .exact_component_consensus_audit_ready
    );
    assert!(verified.component_audit.row_cache_accounting_matches);
    assert!(verified.component_audit.component_accounting_matches);
    assert!(verified.component_audit.retry_subset_matches);
    assert_eq!(verified.component_consensus, adaptive_component);
    assert_eq!(verified.verifier, per_cell_schedule);
    assert!(verified.exact_verified_component_consensus_ready);
}

#[test]
fn local_component_consensus_reuses_exact_row_certificates_across_components() {
    let frame = frame(4);
    let solid = ExactTriangleSolidMesh::new(two_sheared_aligned_box_surface(&frame), true);
    let prepared = PreparedExactTriangleSolidMesh::prepare(solid).unwrap();
    let (per_cell_grid, per_cell_report, per_cell_schedule) =
        voxelize_prepared_exact_triangle_solid_mesh(
            frame.clone(),
            &prepared,
            MaterialRegionId(16),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (local_grid, local_report, local_component) =
        voxelize_prepared_exact_triangle_solid_mesh_by_local_component_consensus(
            frame.clone(),
            &prepared,
            MaterialRegionId(16),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (verified_grid, verified_report, verified) =
        voxelize_prepared_exact_triangle_solid_mesh_by_verified_local_component_consensus(
            frame,
            &prepared,
            MaterialRegionId(16),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();

    assert_eq!(local_grid, per_cell_grid);
    assert_eq!(verified_grid, per_cell_grid);
    assert_eq!(
        local_report.predicate_certificates,
        per_cell_report.predicate_certificates
    );
    assert_eq!(
        verified_report.predicate_certificates,
        per_cell_report.predicate_certificates
    );
    assert!(local_component.components >= 2);
    assert!(local_component.consensus_components >= 2);
    assert_component_row_cache_accounting(&local_component);
    assert!(
        local_component.row_cache_hits > 0,
        "aligned enclosed components should reuse exact row certificates"
    );
    assert!(local_component.row_cache_misses > 0);
    assert!(local_component.row_candidate_triangles > 0);
    assert!(local_component.row_votes > 0);
    assert_eq!(local_component.fallback_unknown_cells, 0);
    assert_eq!(local_component.fallback_boundary_regression_cells, 0);
    assert_eq!(local_component.row_parameter_order_unknowns, 0);
    assert!(local_component.exact_component_consensus_ready);

    assert_eq!(verified.grid_mismatch_cells, 0);
    assert!(verified.predicate_certificates_match);
    assert!(verified.boundary_counts_match);
    assert!(verified.unknown_counts_match);
    assert!(verified.aggregate_matches);
    assert!(verified.verifier_exact_topology_ready);
    assert_eq!(verified.component_consensus, local_component);
    assert_eq!(verified.verifier, per_cell_schedule);
    assert!(
        verified
            .component_audit
            .exact_component_consensus_audit_ready
    );
    assert!(verified.component_audit.row_cache_accounting_matches);
    assert!(verified.component_audit.row_candidate_schedule_matches);
    assert!(verified.component_audit.row_candidate_rejections_match);
    assert!(verified.exact_verified_component_consensus_ready);
}

#[test]
fn fuzz_regression_axis_aligned_component_audit_allows_zero_row_candidates() {
    let frame = frame(3);
    let solid = ExactTriangleSolidMesh::new(cube_surface(1, 5, &frame), true);
    let prepared = PreparedExactTriangleSolidMesh::prepare(solid).unwrap();
    let (_, _, verified) =
        voxelize_prepared_exact_triangle_solid_mesh_by_verified_component_consensus(
            frame,
            &prepared,
            MaterialRegionId(15),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();

    assert!(
        verified
            .component_audit
            .exact_component_consensus_audit_ready,
        "audit: {:#?}\ncomponent: {:#?}",
        verified.component_audit,
        verified.component_consensus
    );
    assert!(verified.exact_verified_component_consensus_ready);
}

#[test]
fn prepared_triangle_solid_rejects_non_solid_source_replay() {
    let frame = frame(2);
    let solid = ExactTriangleSolidMesh::new(cube_surface(1, 3, &frame), false);

    assert_eq!(
        PreparedExactTriangleSolidMesh::prepare(solid),
        Err(HypervoxelError::InvalidSourceGeometry {
            reason: "triangle solid mesh lacks exact closed-solid replay"
        })
    );
}

proptest! {
    #[test]
    fn generated_axis_aligned_triangles_have_certified_surface_reports(
        depth in 1_u8..4,
        x in 0_u64..8,
        y in 0_u64..8,
        z in 0_u64..8,
    ) {
        let frame = frame(depth);
        let cells = 1_u64 << depth;
        let x0 = x % cells;
        let y0 = y % cells;
        let z0 = z % cells;
        let mesh = ExactTriangleSurfaceMesh::new(
            vec![tri([
                [Real::from(x0), Real::from(y0), Real::from(z0)],
                [Real::from((x0 + 1).min(cells)), Real::from(y0), Real::from(z0)],
                [Real::from(x0), Real::from((y0 + 1).min(cells)), Real::from(z0)],
            ])],
            frame.source().cloned(),
            true,
        );

        let report = mesh.report();
        prop_assert_eq!(report.triangle_count, 1);
        if report.exact_triangle_source_ready {
            let (_, voxel_report) = voxelize_exact_triangle_surface_mesh(
                frame,
                &mesh,
                MaterialRegionId(9),
                VoxelizationPolicy::conservative_cover(),
            ).unwrap();
            prop_assert_eq!(voxel_report.unknown_cells, 0);
            prop_assert!(voxel_report.predicate_certificates.is_fully_certified());
        } else {
            prop_assert!(report.degenerate_triangle_count > 0);
        }
    }

    #[test]
    fn generated_cube_solids_match_consensus_winding_replay(
        depth in 2_u8..4,
        lo_raw in 0_u64..8,
        span_raw in 0_u64..8,
    ) {
        let frame = frame(depth);
        let cells = 1_u64 << depth;
        let lo = 1 + (lo_raw % (cells - 2));
        let hi = lo + 1 + (span_raw % (cells - lo - 1));
        let solid = ExactTriangleSolidMesh::new(cube_surface(lo as i64, hi as i64, &frame), true);
        let prepared = PreparedExactTriangleSolidMesh::prepare(solid).unwrap();

        let (per_cell_grid, per_cell_report, _) = voxelize_prepared_exact_triangle_solid_mesh(
            frame.clone(),
            &prepared,
            MaterialRegionId(11),
            VoxelizationPolicy::conservative_cover(),
        ).unwrap();
        let (consensus_grid, consensus_report, consensus) =
            voxelize_prepared_exact_triangle_solid_mesh_by_consensus_axis_sweeps(
                frame,
                &prepared,
                MaterialRegionId(11),
                VoxelizationPolicy::conservative_cover(),
            ).unwrap();

        prop_assert_eq!(consensus_grid, per_cell_grid);
        prop_assert_eq!(
            consensus_report.predicate_certificates,
            per_cell_report.predicate_certificates
        );
        prop_assert_eq!(consensus_report.unknown_cells, per_cell_report.unknown_cells);
        prop_assert_eq!(
            consensus.consensus_classified_cells + consensus.fallback_cells,
            consensus.open_cells
        );
        prop_assert_eq!(consensus.conflicting_vote_cells, 0);
        prop_assert_eq!(consensus.fallback_unknown_cells, 0);
        prop_assert_eq!(consensus.fallback_boundary_regression_cells, 0);
        prop_assert!(consensus.exact_consensus_axis_sweep_ready);
    }
}
