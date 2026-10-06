use hyperlimit::Aabb3Intersection;
use hyperreal::{Rational, Real};
use hypervoxel::{
    BoundaryPolicy, ExactAabb3, ExactBox, ExactConvexHalfSpaceSet, ExactHalfSpace, GridFrame,
    HypervoxelError, MaterialRegionId, OccupancyState, QuantizationPolicy, QueryRegion,
    SparseVoxelGrid, VoxelAddress, VoxelCell, VoxelPayload, VoxelizationPolicy,
    exact_voxel_surface_triangle_mesh_from_faces, extract_exposed_faces, greedy_face_patches,
    lossy_obj_from_quad_mesh, lossy_quad_mesh_from_faces, voxel_neighbors6, voxelize_exact_box,
    voxelize_exact_convex_halfspace_set, voxelize_exact_halfspace,
};

fn r(n: i32) -> Real {
    n.into()
}

fn rf(n: i64, d: u64) -> Real {
    Rational::fraction(n, d).unwrap().into()
}

fn frame() -> GridFrame {
    GridFrame::unit(2).unwrap()
}

#[test]
fn conservative_cover_preserves_boundary_cells_for_exact_box() {
    let exact_box = ExactBox::new(
        [rf(1, 2), rf(1, 2), rf(1, 2)],
        [rf(5, 2), rf(5, 2), rf(5, 2)],
    );
    let (grid, report) = voxelize_exact_box(
        frame(),
        &exact_box,
        MaterialRegionId(7),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();

    assert_eq!(grid.len(), 27);
    assert_eq!(report.unknown_cells, 0);
    assert_eq!(report.boundary_cells, 26);
    assert_eq!(report.predicate_certificates.inside_cells, 1);
    assert_eq!(report.predicate_certificates.outside_cells, 37);
    assert_eq!(report.predicate_certificates.certified_cells(), 64);
    assert!(report.exact_topology_ready());
    assert_eq!(report.aggregate.occupancy_interval.lower, rf(1, 64));
    assert_eq!(report.aggregate.occupancy_interval.upper, rf(27, 64));
    assert_eq!(
        grid.get(VoxelAddress::new(2, [1, 1, 1]).unwrap())
            .unwrap()
            .occupancy,
        OccupancyState::Filled
    );
}

#[test]
fn exact_primitives_reject_degenerate_geometry() {
    let inverted = ExactBox::new([r(2), r(0), r(0)], [r(1), r(1), r(1)]);
    assert_eq!(
        voxelize_exact_box(
            frame(),
            &inverted,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        ),
        Err(HypervoxelError::InvalidSourceGeometry {
            reason: "box minimum exceeds maximum",
        })
    );

    let zero_extent = ExactBox::new([r(1), r(0), r(0)], [r(1), r(1), r(1)]);
    assert!(
        voxelize_exact_box(
            frame(),
            &zero_extent,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        )
        .is_err()
    );

    let zero_normal = ExactHalfSpace::new([r(0), r(0), r(0)], r(1));
    assert!(
        voxelize_exact_halfspace(
            frame(),
            &zero_normal,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        )
        .is_err()
    );

    let empty_solid = ExactConvexHalfSpaceSet::new(Vec::new());
    assert!(
        voxelize_exact_convex_halfspace_set(
            frame(),
            &empty_solid,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        )
        .is_err()
    );
}

#[test]
fn exact_halfspace_and_convex_set_classify_without_float_tolerances() {
    let halfspace = ExactHalfSpace::new([r(1), r(0), r(0)], r(2));
    let (grid, report) = voxelize_exact_halfspace(
        frame(),
        &halfspace,
        MaterialRegionId(8),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    assert_eq!(report.boundary_cells, 16);
    assert_eq!(grid.len(), 48);

    let solid = ExactConvexHalfSpaceSet::new(vec![
        ExactHalfSpace::new([r(-1), r(0), r(0)], r(-1)),
        ExactHalfSpace::new([r(1), r(0), r(0)], r(3)),
        ExactHalfSpace::new([r(0), r(-1), r(0)], r(-1)),
        ExactHalfSpace::new([r(0), r(1), r(0)], r(3)),
        ExactHalfSpace::new([r(0), r(0), r(-1)], r(-1)),
        ExactHalfSpace::new([r(0), r(0), r(1)], r(3)),
    ]);
    let (grid, report) = voxelize_exact_convex_halfspace_set(
        frame(),
        &solid,
        MaterialRegionId(9),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    assert_eq!(grid.len(), 64);
    assert_eq!(report.boundary_cells, 56);
    assert_eq!(report.predicate_certificates.inside_cells, 8);
    assert_eq!(report.unknown_cells, 0);
    assert!(report.exact_topology_ready());
}

#[test]
fn boundary_as_unknown_keeps_uncertainty_explicit() {
    let exact_box = ExactBox::new(
        [rf(1, 2), rf(1, 2), rf(1, 2)],
        [rf(5, 2), rf(5, 2), rf(5, 2)],
    );
    let (grid, report) = voxelize_exact_box(
        frame(),
        &exact_box,
        MaterialRegionId(3),
        VoxelizationPolicy {
            quantization: QuantizationPolicy::ConservativeCover,
            boundary: BoundaryPolicy::BoundaryAsUnknown,
        },
    )
    .unwrap();

    assert_eq!(report.unknown_cells, 26);
    assert!(!report.exact_topology_ready());
    assert_eq!(grid.len(), 27);
    assert!(
        grid.iter()
            .any(|(_, cell)| cell.occupancy == OccupancyState::Unknown)
    );
}

fn voxelized_box() -> SparseVoxelGrid {
    let exact_box = ExactBox::new([r(1), r(1), r(1)], [r(3), r(3), r(3)]);
    let (grid, _) = voxelize_exact_box(
        frame(),
        &exact_box,
        MaterialRegionId(4),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    grid
}

#[test]
fn grid_queries_use_direct_evidence() {
    let grid = voxelized_box();
    let region = QueryRegion {
        min: [1, 1, 1],
        max: [2, 2, 2],
        depth: 2,
    };
    let aggregate = grid.query_region_aggregate(&region);
    assert_eq!(aggregate.child_count, 8);
    assert!(aggregate.all_filled);

    let broad_phase = grid
        .query_aabb_broad_phase(&ExactAabb3 {
            min: [r(0), r(0), r(0)],
            max: [r(1), r(1), r(1)],
        })
        .unwrap();
    assert_eq!(broad_phase.tested_cells, 8);
    assert_eq!(broad_phase.candidates.len(), 1);
    assert_eq!(broad_phase.rejected_addresses.len(), 7);
    assert_eq!(
        broad_phase.candidates[0].relation,
        Aabb3Intersection::Touching
    );
}

#[test]
fn exact_cell_center_and_corners_remain_rational() {
    let bounds = VoxelAddress::new(2, [1, 2, 3])
        .unwrap()
        .bounds(&frame())
        .unwrap();
    assert_eq!(bounds.center(), [rf(3, 2), rf(5, 2), rf(7, 2)]);
    assert_eq!(bounds.corners()[0], [r(1), r(2), r(3)]);
    assert_eq!(bounds.corners()[7], [r(2), r(3), r(4)]);
}

#[test]
fn exposed_faces_feed_direct_lossy_and_exact_mesh_paths() {
    let grid = voxelized_box();
    let faces = extract_exposed_faces(&grid).unwrap();
    assert_eq!(faces.len(), 24);
    assert!(faces.iter().all(|face| face.cell_bounds.extent(0) == r(1)));

    let preview = lossy_quad_mesh_from_faces(&faces).unwrap();
    assert_eq!(preview.vertices.len(), 96);
    assert_eq!(preview.triangles.len(), 48);
    let obj = lossy_obj_from_quad_mesh(&preview);
    assert_eq!(obj.vertex_records, preview.vertices.len());
    assert_eq!(obj.face_records, preview.triangles.len());

    let patches = greedy_face_patches(&faces);
    assert!(patches.len() < faces.len());

    let exact = exact_voxel_surface_triangle_mesh_from_faces(&faces).unwrap();
    assert_eq!(exact.triangles.len(), 48);
    assert!(!exact.vertices.is_empty());

    let mut uncertain = grid.clone();
    uncertain
        .set(
            VoxelAddress::new(2, [1, 1, 1]).unwrap(),
            VoxelCell::unknown(),
        )
        .unwrap();
    assert!(extract_exposed_faces(&uncertain).is_err());
}

#[test]
fn grid_connectivity_stays_in_integer_grid_space() {
    let grid = voxelized_box();
    let seed = VoxelAddress::new(2, [1, 1, 1]).unwrap();

    assert_eq!(
        voxel_neighbors6(VoxelAddress::new(2, [0, 0, 0]).unwrap()).len(),
        3
    );
    assert_eq!(grid.query_neighbors6(seed).neighbors.len(), 6);
    let component = grid.query_connected_component(seed).unwrap();
    assert_eq!(component.addresses.len(), 8);
    assert!(component.exact_component_ready);
    let band = grid.query_manhattan_band(seed, 1).unwrap();
    assert_eq!(band.distances.len(), 4);
    assert_eq!(band.distances[&seed], 0);
}

#[test]
fn sparse_empty_cells_are_absent() {
    let mut grid = SparseVoxelGrid::new(frame());
    let address = VoxelAddress::new(2, [1, 1, 1]).unwrap();
    grid.set(
        address,
        VoxelCell::boundary(VoxelPayload::MaterialRegion(MaterialRegionId(1))),
    )
    .unwrap();
    assert_eq!(grid.len(), 1);
    grid.set(address, VoxelCell::empty()).unwrap();
    assert!(grid.is_empty());
}
