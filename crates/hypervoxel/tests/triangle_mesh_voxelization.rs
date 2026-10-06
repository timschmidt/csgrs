use hyperreal::{Rational, Real};
use hypervoxel::{
    ExactTriangle3, ExactTriangleSolid, ExactTriangleSolidMesh, ExactTriangleSurfaceMesh,
    GridFrame, HypervoxelError, MaterialRegionId, OccupancyState, VoxelAddress,
    VoxelTriangleMeshClassification, VoxelTriangleSolidClassification, VoxelizationPolicy,
    classify_cell_against_exact_triangle_solid, classify_cell_against_triangle_solid_mesh,
    classify_cell_against_triangle_surface_mesh, voxelize_exact_triangle_solid,
    voxelize_exact_triangle_solid_by_adaptive_axis_sweeps,
    voxelize_exact_triangle_solid_by_adaptive_local_component_consensus,
    voxelize_exact_triangle_solid_by_axis_sweeps,
    voxelize_exact_triangle_solid_by_component_consensus,
    voxelize_exact_triangle_solid_by_components,
    voxelize_exact_triangle_solid_by_consensus_axis_sweeps,
    voxelize_exact_triangle_solid_by_local_component_consensus, voxelize_exact_triangle_solid_mesh,
    voxelize_exact_triangle_surface_mesh,
};
use proptest::prelude::*;

fn r(value: i64) -> Real {
    value.into()
}

fn rf(numerator: i64, denominator: u64) -> Real {
    Rational::fraction(numerator, denominator).unwrap().into()
}

fn frame(depth: u8) -> GridFrame {
    GridFrame::unit(depth).unwrap()
}

fn tri(vertices: [[Real; 3]; 3]) -> ExactTriangle3 {
    ExactTriangle3::new(vertices, Some(0))
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

fn cube_surface(min: i64, max: i64) -> ExactTriangleSurfaceMesh {
    ExactTriangleSurfaceMesh::new(rectangular_box_triangles([min, min, min], [max, max, max]))
}

fn sheared_box_surface(min: i64, max: i64) -> ExactTriangleSurfaceMesh {
    let p = |x: i64, y: i64, z: i64| [r(x) + rf(z, 2), r(y), r(z)];
    ExactTriangleSurfaceMesh::new(vec![
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
    ])
}

fn exact_cube(depth: u8, min: i64, max: i64) -> (GridFrame, ExactTriangleSolid) {
    let frame = frame(depth);
    let solid = ExactTriangleSolidMesh::new(cube_surface(min, max), true);
    let solid = ExactTriangleSolid::new(solid).unwrap();
    (frame, solid)
}

#[test]
fn exact_triangle_surface_classifies_boundary_without_float_tolerances() {
    let frame = frame(1);
    let mesh = ExactTriangleSurfaceMesh::new(vec![tri([
        [r(0), r(0), r(0)],
        [r(2), r(0), r(0)],
        [r(0), r(2), r(0)],
    ])]);
    let address = VoxelAddress::new(1, [0, 0, 0]).unwrap();
    assert_eq!(
        classify_cell_against_triangle_surface_mesh(address, &frame, &mesh).unwrap(),
        VoxelTriangleMeshClassification::Boundary
    );

    let (grid, report) = voxelize_exact_triangle_surface_mesh(
        frame,
        &mesh,
        MaterialRegionId(3),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    assert!(!grid.is_empty());
    assert_eq!(report.unknown_cells, 0);
}

#[test]
fn exact_triangle_surface_rejects_empty_and_degenerate_sources() {
    let frame = frame(1);
    let empty = ExactTriangleSurfaceMesh::new(Vec::new());
    assert!(
        voxelize_exact_triangle_surface_mesh(
            frame.clone(),
            &empty,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        )
        .is_err()
    );

    let degenerate = ExactTriangleSurfaceMesh::new(vec![tri([
        [r(0), r(0), r(0)],
        [r(1), r(1), r(1)],
        [r(2), r(2), r(2)],
    ])]);
    assert!(
        voxelize_exact_triangle_surface_mesh(
            frame,
            &degenerate,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        )
        .is_err()
    );
}

#[test]
fn exact_triangle_solid_classifies_boundary_inside_and_outside() {
    let frame = frame(3);
    let solid = ExactTriangleSolidMesh::new(cube_surface(2, 6), true);
    let boundary = VoxelAddress::new(3, [2, 3, 3]).unwrap();
    let inside = VoxelAddress::new(3, [3, 3, 3]).unwrap();
    let outside = VoxelAddress::new(3, [0, 0, 0]).unwrap();
    assert_eq!(
        classify_cell_against_triangle_solid_mesh(boundary, &frame, &solid).unwrap(),
        VoxelTriangleSolidClassification::Boundary
    );
    assert_eq!(
        classify_cell_against_triangle_solid_mesh(inside, &frame, &solid).unwrap(),
        VoxelTriangleSolidClassification::Inside
    );
    assert_eq!(
        classify_cell_against_triangle_solid_mesh(outside, &frame, &solid).unwrap(),
        VoxelTriangleSolidClassification::Outside
    );
}

#[test]
fn scheduled_and_direct_solid_voxelization_match() {
    let (frame, solid) = exact_cube(3, 2, 6);
    let (direct_grid, direct_report) = voxelize_exact_triangle_solid_mesh(
        frame.clone(),
        solid.mesh(),
        MaterialRegionId(4),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    let (scheduled_grid, scheduled_report, schedule) = voxelize_exact_triangle_solid(
        frame,
        &solid,
        MaterialRegionId(4),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    assert_eq!(scheduled_grid, direct_grid);
    assert_eq!(scheduled_report.aggregate, direct_report.aggregate);
    assert_eq!(scheduled_report.unknown_cells, 0);
    assert!(schedule.boundary_aabb_rejections > 0);
}

#[test]
fn component_and_sweep_accelerators_match_per_cell_classification() {
    let (frame, solid) = exact_cube(3, 2, 6);
    let policy = VoxelizationPolicy::conservative_cover();
    let material = MaterialRegionId(5);
    let (expected, _, _) =
        voxelize_exact_triangle_solid(frame.clone(), &solid, material, policy.clone()).unwrap();
    let (components, _, component_schedule) = voxelize_exact_triangle_solid_by_components(
        frame.clone(),
        &solid,
        material,
        policy.clone(),
    )
    .unwrap();
    let (axis, _, axis_schedule) = voxelize_exact_triangle_solid_by_axis_sweeps(
        frame.clone(),
        &solid,
        material,
        policy.clone(),
    )
    .unwrap();
    let (adaptive, _, adaptive_schedule) = voxelize_exact_triangle_solid_by_adaptive_axis_sweeps(
        frame.clone(),
        &solid,
        material,
        policy.clone(),
    )
    .unwrap();
    let (consensus, _, consensus_schedule) =
        voxelize_exact_triangle_solid_by_consensus_axis_sweeps(frame, &solid, material, policy)
            .unwrap();

    assert_eq!(components, expected);
    assert_eq!(axis, expected);
    assert_eq!(adaptive, expected);
    assert_eq!(consensus, expected);
    assert_eq!(component_schedule.unknown_components, 0);
    assert!(axis_schedule.exact_axis_sweep_ready);
    assert!(adaptive_schedule.exact_adaptive_axis_sweep_ready);
    assert!(consensus_schedule.exact_consensus_axis_sweep_ready);
}

#[test]
fn component_consensus_variants_match_per_cell_on_sheared_solid() {
    let frame = frame(3);
    let solid =
        ExactTriangleSolid::new(ExactTriangleSolidMesh::new(sheared_box_surface(1, 5), true))
            .unwrap();
    let policy = VoxelizationPolicy::conservative_cover();
    let material = MaterialRegionId(6);
    let (expected, _, _) =
        voxelize_exact_triangle_solid(frame.clone(), &solid, material, policy.clone()).unwrap();
    let (global, _, global_schedule) = voxelize_exact_triangle_solid_by_component_consensus(
        frame.clone(),
        &solid,
        material,
        policy.clone(),
    )
    .unwrap();
    let (local, _, local_schedule) = voxelize_exact_triangle_solid_by_local_component_consensus(
        frame.clone(),
        &solid,
        material,
        policy.clone(),
    )
    .unwrap();
    let (adaptive, _, adaptive_schedule) =
        voxelize_exact_triangle_solid_by_adaptive_local_component_consensus(
            frame, &solid, material, policy,
        )
        .unwrap();

    assert_eq!(global, expected);
    assert_eq!(local, expected);
    assert_eq!(adaptive, expected);
    assert!(global_schedule.exact_component_consensus_ready);
    assert!(local_schedule.exact_component_consensus_ready);
    assert!(adaptive_schedule.exact_component_consensus_ready);
}

#[test]
fn classifier_reuses_exact_schedule() {
    let (frame, solid) = exact_cube(3, 2, 6);
    let inside = VoxelAddress::new(3, [3, 3, 3]).unwrap();
    let report = classify_cell_against_exact_triangle_solid(inside, &frame, &solid).unwrap();
    assert_eq!(
        report.classification,
        VoxelTriangleSolidClassification::Inside
    );
    assert!(report.boundary_aabb_rejections > 0);
}

#[test]
fn exact_triangle_solid_rejects_open_source() {
    let solid = ExactTriangleSolidMesh::new(cube_surface(1, 3), false);
    assert_eq!(
        ExactTriangleSolid::new(solid),
        Err(HypervoxelError::InvalidSourceGeometry {
            reason: "triangle solid mesh lacks exact closed-solid replay"
        })
    );
}

proptest! {
    #[test]
    fn generated_cube_solids_match_direct_and_scheduled_paths(
        depth in 2_u8..4,
        min in 0_i64..2,
        extent in 1_i64..3,
    ) {
        let frame = frame(depth);
        let cells = 1_i64 << depth;
        let max = (min + extent).min(cells);
        prop_assume!(min < max);
        let mesh = ExactTriangleSolidMesh::new(cube_surface(min, max), true);
        let solid = ExactTriangleSolid::new(mesh.clone()).unwrap();
        let policy = VoxelizationPolicy::conservative_cover();
        let (direct, direct_report) = voxelize_exact_triangle_solid_mesh(
            frame.clone(), &mesh, MaterialRegionId(9), policy.clone(),
        ).unwrap();
        let (cached, cached_report, _) = voxelize_exact_triangle_solid(
            frame, &solid, MaterialRegionId(9), policy,
        ).unwrap();
        prop_assert_eq!(&cached, &direct);
        prop_assert_eq!(cached_report.aggregate, direct_report.aggregate);
        prop_assert_eq!(cached_report.unknown_cells, 0);
        prop_assert!(!cached.iter().any(|(_, cell)| cell.occupancy == OccupancyState::Unknown));
    }
}
