use hyperreal::{Rational, Real};
use hypervoxel::{
    BoundaryPolicy, ExactTriangle3, ExactTriangleSolidMesh, ExactTriangleSurfaceMesh, GridFrame,
    GridSource, HypervoxelError, MaterialRegionId, OccupancyState, PreparedExactTriangleSolidMesh,
    QuantizationPolicy, VoxelAddress, VoxelTriangleMeshClassifier, VoxelTriangleSolidClassifier,
    VoxelizationPolicy, classify_cell_against_prepared_triangle_solid_mesh,
    classify_cell_against_triangle_solid_mesh, classify_cell_against_triangle_surface_mesh,
    voxelize_exact_triangle_solid_mesh, voxelize_exact_triangle_surface_mesh,
    voxelize_prepared_exact_triangle_solid_mesh,
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
    let p = |x, y, z| [r(x), r(y), r(z)];
    let triangles = vec![
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
    ];
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
    assert_eq!(interior.ray_attempts[0].triangle_tests, 12);

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
    assert!(schedule.ambiguous_ray_attempts > 0);
    assert!(schedule.ambiguous_ray_attempts < schedule.ray_attempts);
    assert!(prepared_report.exact_topology_ready());
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
}
