use hyperreal::{Rational, Real};
use hypervoxel::{
    BoundaryPolicy, ExactTriangle3, ExactTriangleSurfaceMesh, GridFrame, GridSource,
    HypervoxelError, MaterialRegionId, OccupancyState, QuantizationPolicy, VoxelAddress,
    VoxelTriangleMeshClassifier, VoxelizationPolicy, classify_cell_against_triangle_surface_mesh,
    voxelize_exact_triangle_surface_mesh,
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
