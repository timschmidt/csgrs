use hyperreal::{Rational, Real};
use hypervoxel::{
    AggregateCertainty, ChunkAddress, ChunkPageSummary, ChunkPagedSparseGrid, ChunkShape,
    GridFrame, HypervoxelError, LengthUnit, MaterialRegionId, OccupancyState, QueryRegion,
    SparseVoxelGrid, VoxelAddress, VoxelAggregateFacts, VoxelCell, VoxelPayload,
    VoxelSpatialAggregateFacts, chunk_paged_greedy_face_patch_plan,
    extract_chunk_paged_exposed_faces, extract_exposed_faces,
};

use hypervoxel::SvoVoxelGrid;

fn r(n: i32) -> Real {
    n.into()
}

fn rf(n: i64, d: u64) -> Real {
    Rational::fraction(n, d).unwrap().into()
}

fn frame(depth: u8) -> GridFrame {
    GridFrame::builder()
        .origin([r(-1), r(2), r(3)])
        .pitch([
            Rational::fraction(1, 8).unwrap().into(),
            Rational::fraction(1, 4).unwrap().into(),
            Rational::fraction(1, 2).unwrap().into(),
        ])
        .depth(depth)
        .units(LengthUnit::Millimeter)
        .build()
        .unwrap()
}

#[test]
fn exact_cell_bounds_use_grid_depth_not_float_chunk_size() {
    let frame = frame(4);
    let address = VoxelAddress::new(2, [1, 2, 3]).unwrap();
    let bounds = address.bounds(&frame).unwrap();

    assert_eq!(bounds.min[0], rf(-8 + 4, 8));
    assert_eq!(bounds.max[0], rf(-8 + 8, 8));
    assert_eq!(bounds.min[1], rf(2 * 4 + 8, 4));
    assert_eq!(bounds.max[1], rf(2 * 4 + 12, 4));
    assert_eq!(bounds.extent(2), r(2));
}

#[test]
fn frame_rejects_non_positive_or_unknown_cell_axes() {
    let zero = GridFrame::builder()
        .pitch([r(1), r(0), r(1)])
        .build()
        .unwrap_err();
    assert_eq!(zero, HypervoxelError::NonPositiveCellAxis { axis: 1 });

    let negative = GridFrame::builder()
        .pitch([r(1), r(1), r(-1)])
        .build()
        .unwrap_err();
    assert_eq!(negative, HypervoxelError::NonPositiveCellAxis { axis: 2 });
}

#[test]
fn child_index_mapping_is_stable_and_checked() {
    let root = VoxelAddress::root();
    assert_eq!(root.child(0).unwrap().xyz, [0, 0, 0]);
    assert_eq!(root.child(5).unwrap().xyz, [1, 0, 1]);
    assert_eq!(root.child(7).unwrap().xyz, [1, 1, 1]);
    assert_eq!(root.child(8), Err(HypervoxelError::InvalidChildIndex(8)));
}

#[test]
fn morton_codes_and_child_paths_round_trip_exact_addresses() {
    let address = VoxelAddress::new(5, [17, 3, 29]).unwrap();
    assert_eq!(
        VoxelAddress::from_morton_code(address.depth, address.morton_code()).unwrap(),
        address
    );
    assert_eq!(
        VoxelAddress::from_child_path(&address.child_path()).unwrap(),
        address
    );
}

#[test]
fn chunk_page_summary_partitions_addresses_without_world_coordinates() {
    let shape = ChunkShape::new(2).unwrap();
    let addresses = [
        VoxelAddress::new(4, [0, 0, 0]).unwrap(),
        VoxelAddress::new(4, [3, 3, 3]).unwrap(),
        VoxelAddress::new(4, [4, 0, 0]).unwrap(),
    ];
    let summary = ChunkPageSummary::from_addresses(shape, addresses);

    assert_eq!(shape.cells_per_axis(), 4);
    assert_eq!(summary.stored_cells, 3);
    assert!(summary.has_stored_cells);
    assert_eq!(summary.page_count, 2);
    assert!(summary.exact_integer_partition);
    assert_eq!(summary.page_capacity_cells, 128);
    assert!(summary.exact_page_cover_ready);
    let empty = ChunkPageSummary::from_addresses(shape, std::iter::empty::<VoxelAddress>());
    assert_eq!(empty.stored_cells, 0);
    assert!(!empty.has_stored_cells);
    assert_eq!(empty.page_count, 0);
    assert_eq!(empty.page_capacity_cells, 0);
    assert!(!empty.exact_page_cover_ready);
    let split = ChunkAddress::split(VoxelAddress::new(4, [6, 7, 8]).unwrap(), shape);
    assert_eq!(split.chunk.xyz, [1, 1, 2]);
    assert_eq!(split.local_xyz, [2, 3, 0]);
    assert_eq!(split.local_extent, 4);
    assert!(split.local_in_bounds);
    assert!(split.exact_recompose_ready);

    let coarse_split = ChunkAddress::split(VoxelAddress::new(1, [1, 0, 1]).unwrap(), shape);
    assert_eq!(coarse_split.local_extent, 2);
    assert!(coarse_split.exact_recompose_ready);
    assert_eq!(
        ChunkShape::new(22).unwrap_err(),
        HypervoxelError::DepthTooLarge {
            depth: 22,
            max_supported: 21
        }
    );
}

#[test]
fn chunk_paged_sparse_storage_replays_exact_addresses_and_payload_blockers() {
    let frame = frame(4);
    let shape = ChunkShape::new(2).unwrap();
    let a = VoxelAddress::new(4, [0, 0, 0]).unwrap();
    let b = VoxelAddress::new(4, [3, 3, 3]).unwrap();
    let c = VoxelAddress::new(4, [4, 0, 0]).unwrap();
    let absent = VoxelAddress::new(4, [8, 8, 8]).unwrap();
    let mut grid = SparseVoxelGrid::new(frame.clone());
    grid.set(a, VoxelCell::material(MaterialRegionId(1)))
        .unwrap();
    grid.set(
        b,
        VoxelCell::boundary(VoxelPayload::MaterialRegion(MaterialRegionId(1))),
    )
    .unwrap();
    grid.set(c, VoxelCell::material(MaterialRegionId(2)))
        .unwrap();

    let paged = ChunkPagedSparseGrid::from_sparse_grid(&grid, shape).unwrap();
    let report = paged.report();
    assert_eq!(paged.len(), 3);
    assert_eq!(paged.page_count(), 2);
    assert_eq!(report.summary.stored_cells, 3);
    assert_eq!(report.summary.page_count, 2);
    assert_eq!(report.finest_depth_cells, 3);
    assert_eq!(report.non_finest_depth_cells, 0);
    assert!(report.exact_address_replay_ready);
    assert!(report.exact_payload_replay_ready);
    assert!(report.exact_chunk_storage_ready);
    assert_eq!(report.aggregate.child_count, 3);
    assert_eq!(paged.get(a).unwrap().occupancy, OccupancyState::Filled);
    assert_eq!(paged.get(absent).unwrap().occupancy, OccupancyState::Empty);

    let first_page = paged
        .page(ChunkAddress::containing(a, shape))
        .unwrap()
        .report(&frame);
    assert_eq!(first_page.stored_cells, 2);
    assert_eq!(first_page.finest_depth_cells, 2);
    assert!(first_page.local_addresses_in_bounds);
    assert!(first_page.exact_local_recompose_ready);
    assert!(first_page.exact_page_replay_ready);

    let region = QueryRegion {
        min: [0, 0, 0],
        max: [3, 3, 3],
        depth: 4,
    };
    let region_report = paged.query_region_aggregate(&region).unwrap();
    assert_eq!(region_report.tested_pages, 2);
    assert_eq!(region_report.rejected_pages, 1);
    assert_eq!(region_report.candidate_pages, 1);
    assert_eq!(region_report.cross_depth_candidate_pages, 0);
    assert_eq!(region_report.tested_cells, 2);
    assert_eq!(region_report.matched_cells, 2);
    assert!(region_report.exact_page_filter_ready);
    assert!(region_report.exact_region_query_ready);
    assert_eq!(region_report.aggregate.child_count, 2);

    let disjoint_region = QueryRegion {
        min: [12, 12, 12],
        max: [15, 15, 15],
        depth: 4,
    };
    let empty_region = paged.query_region_aggregate(&disjoint_region).unwrap();
    assert_eq!(empty_region.rejected_pages, 2);
    assert_eq!(empty_region.tested_cells, 0);
    assert_eq!(empty_region.matched_cells, 0);
    assert!(empty_region.exact_page_filter_ready);
    assert!(!empty_region.exact_region_query_ready);

    let mut blocked = grid.clone();
    blocked
        .set(
            VoxelAddress::new(4, [4, 1, 0]).unwrap(),
            VoxelCell::unknown(),
        )
        .unwrap();
    let blocked = ChunkPagedSparseGrid::from_sparse_grid(&blocked, shape).unwrap();
    assert!(blocked.report().has_unknown);
    assert!(!blocked.report().exact_payload_replay_ready);
    assert!(!blocked.report().exact_chunk_storage_ready);

    let mut coarse_grid = SparseVoxelGrid::new(frame.clone());
    coarse_grid
        .set(
            VoxelAddress::new(2, [1, 1, 1]).unwrap(),
            VoxelCell::material(MaterialRegionId(9)),
        )
        .unwrap();
    let coarse_paged = ChunkPagedSparseGrid::from_sparse_grid(&coarse_grid, shape).unwrap();
    let cross_depth = coarse_paged.query_region_aggregate(&region).unwrap();
    assert_eq!(cross_depth.cross_depth_candidate_pages, 1);
    assert!(!cross_depth.exact_page_filter_ready);
    assert!(!cross_depth.exact_region_query_ready);

    let broad_phase = paged
        .query_aabb_broad_phase(&hypervoxel::ExactAabb3 {
            min: [r(-1), r(2), r(3)],
            max: [rf(-5, 8), rf(12, 4), rf(10, 2)],
        })
        .unwrap();
    assert_eq!(broad_phase.tested_pages, 2);
    assert_eq!(broad_phase.rejected_pages, 1);
    assert_eq!(broad_phase.candidate_pages, 1);
    assert_eq!(broad_phase.unknown_pages, 0);
    assert_eq!(broad_phase.cells.tested_cells, 2);
    assert_eq!(broad_phase.cells.candidates.len(), 2);
    assert!(broad_phase.exact_page_filter_ready);
    assert!(broad_phase.exact_paged_broad_phase_ready);

    let miss = paged
        .query_aabb_broad_phase(&hypervoxel::ExactAabb3 {
            min: [r(10), r(10), r(10)],
            max: [r(11), r(11), r(11)],
        })
        .unwrap();
    assert_eq!(miss.rejected_pages, 2);
    assert_eq!(miss.cells.tested_cells, 0);
    assert!(!miss.cells.has_tested_cells);
    assert!(miss.exact_page_filter_ready);
    assert!(!miss.exact_paged_broad_phase_ready);

    let mut component_grid = SparseVoxelGrid::new(frame.clone());
    let s0 = VoxelAddress::new(4, [1, 1, 1]).unwrap();
    let s1 = VoxelAddress::new(4, [2, 1, 1]).unwrap();
    let s2 = VoxelAddress::new(4, [3, 1, 1]).unwrap();
    let isolated = VoxelAddress::new(4, [12, 12, 12]).unwrap();
    for address in [s0, s1, s2, isolated] {
        component_grid
            .set(address, VoxelCell::material(MaterialRegionId(7)))
            .unwrap();
    }
    let component_pages = ChunkPagedSparseGrid::from_sparse_grid(&component_grid, shape).unwrap();
    let component = component_pages.query_connected_component(s0).unwrap();
    assert_eq!(component.addresses, vec![s0, s1, s2]);
    assert!(component.has_reached_cells);
    assert!(component.exact_component_ready);
    assert!(component.page_hits > 0);
    assert!(component.page_misses > 0);
    assert!(component.cross_page_edges > 0);
    assert_eq!(component.aggregate.child_count, 3);
    let band = component_pages.query_manhattan_band(s0, 3).unwrap();
    assert_eq!(band.distances.len(), 3);
    assert_eq!(band.distances[&s0], 0);
    assert_eq!(band.distances[&s1], 1);
    assert_eq!(band.distances[&s2], 2);
    assert!(band.has_reached_cells);
    assert!(band.exact_distance_band_ready);
    assert!(band.page_hits > 0);
    assert!(band.page_misses > 0);
    assert!(band.cross_page_edges > 0);
    let paged_shell = extract_chunk_paged_exposed_faces(&component_pages).unwrap();
    let sparse_shell = extract_exposed_faces(&component_grid).unwrap();
    assert_eq!(paged_shell.len(), 20);
    assert_eq!(paged_shell, sparse_shell);
    let paged_patches = chunk_paged_greedy_face_patch_plan(&component_pages).unwrap();
    assert_eq!(paged_patches.exact_faces, paged_shell.len());
    assert!(paged_patches.patches.len() < paged_shell.len());

    let empty_component = component_pages
        .query_connected_component(VoxelAddress::new(4, [0, 0, 0]).unwrap())
        .unwrap();
    assert!(empty_component.addresses.is_empty());
    assert!(!empty_component.has_reached_cells);
    assert!(!empty_component.exact_component_ready);

    component_grid.set(s1, VoxelCell::unknown()).unwrap();
    let blocked_component_pages =
        ChunkPagedSparseGrid::from_sparse_grid(&component_grid, shape).unwrap();
    let blocked_component = blocked_component_pages
        .query_connected_component(s0)
        .unwrap();
    assert_eq!(blocked_component.addresses, vec![s0, s1, s2]);
    assert!(blocked_component.has_unknown);
    assert!(!blocked_component.exact_component_ready);
    let blocked_band = blocked_component_pages.query_manhattan_band(s0, 2).unwrap();
    assert_eq!(blocked_band.distances.len(), 3);
    assert!(blocked_band.has_unknown);
    assert!(!blocked_band.exact_distance_band_ready);
    assert!(extract_chunk_paged_exposed_faces(&blocked_component_pages).is_err());
    assert!(chunk_paged_greedy_face_patch_plan(&blocked_component_pages).is_err());
}

#[test]
fn aggregate_preserves_unknown_boundary_and_lossy_states() {
    let cells = [
        VoxelCell::material(MaterialRegionId(1)),
        VoxelCell::boundary(VoxelPayload::MaterialRegion(MaterialRegionId(1))),
        VoxelCell::unknown(),
    ];
    let facts = VoxelAggregateFacts::from_cells(&cells);
    assert!(facts.has_boundary);
    assert!(facts.has_unknown);
    assert_eq!(facts.certainty, AggregateCertainty::Unknown);
    assert_eq!(facts.conservative_occupancy(), OccupancyState::Unknown);
    assert_eq!(facts.occupancy_interval.total_cells, 3);
    assert_eq!(facts.occupancy_interval.definite_filled_cells, 1);
    assert_eq!(facts.occupancy_interval.possible_occupied_cells, 3);
    assert_eq!(facts.occupancy_interval.lower, rf(1, 3));
    assert_eq!(facts.occupancy_interval.upper, r(1));

    let lossy = [
        VoxelCell::material(MaterialRegionId(1)),
        VoxelCell {
            occupancy: OccupancyState::LossyAdapterValue,
            payload: VoxelPayload::LossyAdapterValue(42),
        },
    ];
    let facts = VoxelAggregateFacts::from_cells(&lossy);
    assert_eq!(facts.certainty, AggregateCertainty::Lossy);
    assert_eq!(
        facts.conservative_occupancy(),
        OccupancyState::LossyAdapterValue
    );
    assert_eq!(
        facts.occupancy_interval.certainty,
        AggregateCertainty::Lossy
    );
}

#[test]
fn voxel_cell_report_rejects_incoherent_payload_occupancy_pairs() {
    let exact = VoxelCell::material(MaterialRegionId(7)).report();
    assert!(exact.payload_matches_occupancy);
    assert!(exact.exact_cell_evidence_ready);
    assert!(!exact.has_unknown);
    assert!(!exact.has_lossy);

    let unknown = VoxelCell::unknown().report();
    assert!(unknown.payload_matches_occupancy);
    assert!(unknown.has_unknown);
    assert!(!unknown.exact_cell_evidence_ready);

    let lossy = VoxelCell::lossy_adapter_value(42).report();
    assert!(lossy.payload_matches_occupancy);
    assert!(lossy.has_lossy);
    assert!(!lossy.exact_cell_evidence_ready);

    let incoherent = VoxelCell {
        occupancy: OccupancyState::Empty,
        payload: VoxelPayload::MaterialRegion(MaterialRegionId(9)),
    }
    .report();
    assert!(!incoherent.payload_matches_occupancy);
    assert!(!incoherent.exact_cell_evidence_ready);
}

#[test]
fn occupancy_interval_distinguishes_exact_filled_from_certified_boundary_range() {
    let exact = VoxelAggregateFacts::from_cells([
        &VoxelCell::material(MaterialRegionId(1)),
        &VoxelCell::empty(),
    ]);
    assert!(exact.occupancy_interval.is_point_interval());
    assert_eq!(exact.occupancy_interval.lower, rf(1, 2));
    assert_eq!(
        exact.occupancy_interval.certainty,
        AggregateCertainty::Exact
    );

    let boundary = VoxelAggregateFacts::from_cells([
        &VoxelCell::material(MaterialRegionId(1)),
        &VoxelCell::boundary(VoxelPayload::MaterialRegion(MaterialRegionId(1))),
        &VoxelCell::empty(),
    ]);
    assert!(!boundary.occupancy_interval.is_point_interval());
    assert_eq!(boundary.occupancy_interval.lower, rf(1, 3));
    assert_eq!(boundary.occupancy_interval.upper, rf(2, 3));
    assert_eq!(
        boundary.occupancy_interval.certainty,
        AggregateCertainty::Certified
    );
}

#[test]
fn empty_aggregate_interval_stays_unknown_not_zero_occupancy() {
    let empty = VoxelAggregateFacts::from_cells(std::iter::empty::<&VoxelCell>());

    assert_eq!(empty.child_count, 0);
    assert_eq!(empty.certainty, AggregateCertainty::Unknown);
    assert_eq!(empty.conservative_occupancy(), OccupancyState::Unknown);
    assert_eq!(empty.occupancy_interval.total_cells, 0);
    assert_eq!(empty.occupancy_interval.definite_filled_cells, 0);
    assert_eq!(empty.occupancy_interval.possible_occupied_cells, 0);
    assert_eq!(empty.occupancy_interval.lower, r(0));
    assert_eq!(empty.occupancy_interval.upper, r(1));
    assert_eq!(
        empty.occupancy_interval.certainty,
        AggregateCertainty::Unknown
    );
    assert!(!empty.occupancy_interval.is_point_interval());

    let parent = VoxelAggregateFacts::from_aggregates(std::iter::empty::<&VoxelAggregateFacts>());
    assert_eq!(parent.child_count, 0);
    assert_eq!(parent.certainty, AggregateCertainty::Unknown);
    assert_eq!(parent.conservative_occupancy(), OccupancyState::Unknown);
    assert_eq!(parent.occupancy_interval.lower, r(0));
    assert_eq!(parent.occupancy_interval.upper, r(1));
    assert_eq!(
        parent.occupancy_interval.certainty,
        AggregateCertainty::Unknown
    );
}

#[test]
fn finite_frame_aggregate_counts_implied_empty_cells_and_rejects_overflow() {
    let cells = [VoxelCell::material(MaterialRegionId(1))];
    let aggregate = VoxelAggregateFacts::from_explicit_cells_in_frame(4, cells.iter()).unwrap();

    assert_eq!(aggregate.child_count, 4);
    assert_eq!(aggregate.certainty, AggregateCertainty::Exact);
    assert_eq!(aggregate.conservative_occupancy(), OccupancyState::Mixed);
    assert_eq!(aggregate.occupancy_interval.lower, rf(1, 4));
    assert_eq!(aggregate.occupancy_interval.upper, rf(1, 4));

    assert_eq!(
        VoxelAggregateFacts::from_explicit_cells_in_frame(0, cells.iter()).unwrap_err(),
        HypervoxelError::InvalidAggregateSummary {
            total_cells: 0,
            explicit_cells: 1
        }
    );
}

#[test]
fn sparse_grid_validates_frame_depth_and_empty_absence() {
    let frame = frame(3);
    let mut grid = SparseVoxelGrid::new(frame);
    let address = VoxelAddress::new(3, [3, 4, 5]).unwrap();
    assert_eq!(grid.get(address).unwrap(), VoxelCell::empty());

    grid.set(address, VoxelCell::material(MaterialRegionId(9)))
        .unwrap();
    assert_eq!(grid.len(), 1);
    assert_eq!(grid.get(address).unwrap().occupancy, OccupancyState::Filled);

    let outside = VoxelAddress::new(4, [0, 0, 0]).unwrap();
    assert_eq!(
        grid.get(outside),
        Err(HypervoxelError::DepthOutsideFrame {
            depth: 4,
            frame_depth: 3
        })
    );
}

#[test]
fn svo_dag_reuses_collapsed_subtrees_and_preserves_aggregates() {
    let mut grid = SvoVoxelGrid::new(frame(3));
    assert_eq!(grid.stats().nodes, 1);
    assert!(grid.aggregate().all_empty);
    assert_eq!(grid.aggregate().child_count, 512);

    let a = VoxelAddress::new(3, [0, 0, 0]).unwrap();
    let b = VoxelAddress::new(3, [7, 7, 7]).unwrap();
    grid.set(a, VoxelCell::material(MaterialRegionId(11)))
        .unwrap();
    grid.set(
        b,
        VoxelCell::boundary(VoxelPayload::MaterialRegion(MaterialRegionId(11))),
    )
    .unwrap();

    assert_eq!(grid.get(a).unwrap().occupancy, OccupancyState::Filled);
    assert_eq!(grid.get(b).unwrap().occupancy, OccupancyState::Boundary);
    assert!(grid.aggregate().has_boundary);
    assert_eq!(grid.aggregate().child_count, 512);
    assert!(grid.stats().nodes < 32);

    grid.set(
        VoxelAddress::new(3, [1, 1, 1]).unwrap(),
        VoxelCell::lossy_adapter_value(7),
    )
    .unwrap();
    assert!(grid.aggregate().has_lossy);
}

#[test]
fn svo_to_sparse_grid_expands_collapsed_non_empty_leaves() {
    let frame = frame(3);
    let mut grid = SvoVoxelGrid::new(frame.clone());
    let coarse = VoxelAddress::new(1, [0, 0, 0]).unwrap();
    let far = VoxelAddress::new(3, [7, 7, 7]).unwrap();
    grid.set(coarse, VoxelCell::material(MaterialRegionId(3)))
        .unwrap();
    grid.set(
        far,
        VoxelCell::boundary(VoxelPayload::MaterialRegion(MaterialRegionId(4))),
    )
    .unwrap();

    let sparse = grid.to_sparse_grid().unwrap();
    assert_eq!(sparse.len(), 65);
    assert_eq!(
        sparse
            .get(VoxelAddress::new(3, [3, 3, 3]).unwrap())
            .unwrap(),
        VoxelCell::material(MaterialRegionId(3))
    );
    assert_eq!(sparse.get(far).unwrap().occupancy, OccupancyState::Boundary);
    assert!(
        SvoVoxelGrid::new(frame)
            .to_sparse_grid()
            .unwrap()
            .is_empty()
    );
}

#[test]
fn sparse_to_svo_compaction_round_trips_finest_depth_cells() {
    let frame = frame(6);
    let mut sparse = SparseVoxelGrid::new(frame);
    for i in 0..64_u64 {
        sparse
            .set(
                VoxelAddress::new(6, [i, (i * 3) % 64, (i * 7) % 64]).unwrap(),
                VoxelCell::material(MaterialRegionId((i % 4) as u32)),
            )
            .unwrap();
    }

    let svo = SvoVoxelGrid::from_sparse_grid(&sparse).unwrap();
    assert_eq!(svo.to_sparse_grid().unwrap(), sparse);
    assert_eq!(svo.stats().nodes, 39);
    assert!(SvoVoxelGrid::node_storage_stride_bytes() < std::mem::size_of::<VoxelAggregateFacts>());
    assert_eq!(
        svo.node_storage_bytes(),
        svo.stats().nodes * SvoVoxelGrid::node_storage_stride_bytes()
    );
}

#[test]
fn uniform_child_collapse_preserves_parent_logical_depth() {
    let mut svo = SvoVoxelGrid::new(frame(2));
    let material = VoxelCell::material(MaterialRegionId(21));
    for child in 0..8_u8 {
        svo.set(VoxelAddress::root().child(child).unwrap(), material)
            .unwrap();
    }

    assert_eq!(
        svo.get(VoxelAddress::new(2, [3, 3, 3]).unwrap()).unwrap(),
        material
    );
    assert_eq!(svo.aggregate().child_count, 64);
    assert_eq!(svo.aggregate().occupancy_interval.total_cells, 64);
}

#[test]
fn svo_surface_and_triangle_mesh_are_direct_computations() {
    let frame = frame(2);
    let mut grid = SvoVoxelGrid::new(frame.clone());
    grid.set(
        VoxelAddress::root(),
        VoxelCell::material(MaterialRegionId(8)),
    )
    .unwrap();

    let sparse = grid.to_sparse_grid().unwrap();
    let sparse_faces = extract_exposed_faces(&sparse).unwrap();
    let faces = hypervoxel::extract_svo_exposed_faces(&grid).unwrap();
    assert_eq!(faces, sparse_faces);
    assert_eq!(faces.len(), 96);

    let mesh = hypervoxel::svo_exact_surface_triangle_mesh(&grid).unwrap();
    assert_eq!(mesh.triangles.len(), faces.len() * 2);
    assert!(!mesh.vertices.is_empty());

    let empty = SvoVoxelGrid::new(frame.clone());
    assert!(
        hypervoxel::extract_svo_exposed_faces(&empty)
            .unwrap()
            .is_empty()
    );
    assert!(hypervoxel::svo_exact_surface_triangle_mesh(&empty).is_err());

    let mut lossy = SvoVoxelGrid::new(frame);
    lossy
        .set(VoxelAddress::root(), VoxelCell::lossy_adapter_value(5))
        .unwrap();
    assert!(hypervoxel::extract_svo_exposed_faces(&lossy).is_err());
    assert!(hypervoxel::svo_exact_surface_triangle_mesh(&lossy).is_err());
}

#[test]
fn spatial_aggregate_reports_exact_bounds_and_child_mask() {
    let frame = frame(2);
    let mut grid = SparseVoxelGrid::new(frame.clone());
    let left = VoxelAddress::new(2, [0, 0, 0]).unwrap();
    let right = VoxelAddress::new(2, [3, 3, 3]).unwrap();
    grid.set(left, VoxelCell::material(MaterialRegionId(1)))
        .unwrap();
    grid.set(right, VoxelCell::material(MaterialRegionId(1)))
        .unwrap();
    let spatial = VoxelSpatialAggregateFacts::from_grid(&grid).unwrap();
    assert_eq!(spatial.stored_cells, 2);
    assert!(spatial.has_spatial_evidence);
    assert!(spatial.has_child(0));
    assert!(spatial.has_child(7));
    assert!(spatial.exact_bounds_ready);
    let bounds = spatial.exact_bounds.unwrap();
    assert_eq!(bounds.min, left.bounds(grid.frame()).unwrap().min);
    assert_eq!(bounds.max, right.bounds(grid.frame()).unwrap().max);

    let empty_frame = hypervoxel::GridFrame::builder().depth(2).build().unwrap();
    let empty = VoxelSpatialAggregateFacts::from_grid(&SparseVoxelGrid::new(empty_frame)).unwrap();
    assert_eq!(empty.stored_cells, 0);
    assert!(!empty.has_spatial_evidence);
    assert!(empty.exact_bounds.is_none());
    assert!(!empty.exact_bounds_ready);
}
