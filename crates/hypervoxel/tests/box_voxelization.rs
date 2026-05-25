use hyperlimit::Aabb3Intersection;
use hyperreal::{Rational, Real};
use hypervoxel::{
    AggregateCertainty, BoundaryPolicy, ChunkPagedSparseGrid, ChunkShape, CompressedStorageKind,
    CompressedStorageManifest, DeterministicSnapshot, ExactAabb3, ExactBox,
    ExactConvexHalfSpaceSet, ExactHalfSpace, FieldSampleId, FieldSampleRecord, FreshnessStatus,
    GridAabbHandoff, GridFrame, GridSource, HypervoxelError, LatticeAabbHandoff,
    LossyMeshExportReport, MaterialDisplayColor, MaterialDisplayPalette, MaterialRegionId,
    MaterialRegionRecord, OccupancyState, PreparedSparseVoxelGridExt, PreparedVoxelGrid,
    PreviewExportFormat, PreviewExportManifest, PreviewScalarPolicy, ProcessStateId,
    ProcessStateRecord, QuantizationPolicy, QueryRegion, SideTableLinkStatus, SnapshotFormat,
    SparseVoxelGrid, StorageReplayStatus, VoxelAddress, VoxelAggregateFacts, VoxelCell,
    VoxelHandoffDomain, VoxelHandoffManifest, VoxelMemoryBudgetManifest, VoxelSideTables,
    VoxelizationAudit, VoxelizationPolicy, audit_chunk_paged_material_regions,
    audit_chunk_paged_process_states, diff_chunk_paged_sparse_grids, diff_sparse_grids,
    extract_exposed_faces, extract_exposed_faces_with_report, greedy_face_patch_plan,
    lookup_material_display_colors, lossy_obj_from_quad_mesh, lossy_quad_mesh_from_faces,
    query_material_regions, report_material_region_metadata, sample_manhattan_distance_field,
    sample_signed_manhattan_distance_field, select_lod_cells, voxel_neighbors6, voxelize_exact_box,
    voxelize_exact_convex_halfspace_set, voxelize_exact_halfspace,
};

fn r(n: i32) -> Real {
    n.into()
}

fn rf(n: i64, d: u64) -> Real {
    Rational::fraction(n, d).unwrap().into()
}

fn frame() -> GridFrame {
    GridFrame::builder()
        .origin([r(0), r(0), r(0)])
        .pitch([r(1), r(1), r(1)])
        .depth(2)
        .source(GridSource::new("grid", 1))
        .build()
        .unwrap()
}

#[test]
fn conservative_cover_preserves_boundary_cells_for_exact_box() {
    let exact_box = ExactBox::new(
        [rf(1, 2), rf(1, 2), rf(1, 2)],
        [rf(5, 2), rf(5, 2), rf(5, 2)],
        Some(GridSource::new("box", 3)),
    );
    let source_report = exact_box.report();
    assert!(source_report.exact_box_ready);
    assert_eq!(source_report.ordered_axes, vec![0, 1, 2]);
    assert!(source_report.zero_extent_axes.is_empty());
    let (grid, report) = voxelize_exact_box(
        frame(),
        &exact_box,
        MaterialRegionId(7),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();

    assert_eq!(report.unknown_cells, 0);
    assert_eq!(report.boundary_cells, 26);
    assert_eq!(report.predicate_certificates.inside_cells, 1);
    assert_eq!(report.predicate_certificates.outside_cells, 37);
    assert_eq!(report.predicate_certificates.boundary_cells, 26);
    assert_eq!(report.predicate_certificates.certified_cells(), 64);
    assert!(report.predicate_certificates.is_fully_certified());
    assert!(report.exact_topology_ready());
    assert!(!report.source_replay_ready());
    assert_eq!(grid.len(), 27);
    assert_eq!(report.aggregate.child_count, 64);
    assert!(report.aggregate.has_boundary);
    assert_eq!(report.aggregate.occupancy_interval.total_cells, 64);
    assert_eq!(report.aggregate.occupancy_interval.lower, rf(1, 64));
    assert_eq!(report.aggregate.occupancy_interval.upper, rf(27, 64));
    let audit = VoxelizationAudit::from_grid_and_report(&grid, &report);
    assert_eq!(audit.total_frame_cells, 64);
    assert_eq!(audit.stored_cells, 27);
    assert_eq!(audit.boundary_cells, 26);
    assert_eq!(audit.implied_empty_cells, 37);
    assert_eq!(audit.predicate_certified_cells, 64);
    assert_eq!(audit.predicate_unknown_cells, 0);
    assert!(!audit.has_uncertainty());
    assert!(audit.exact_audit_ready);
    let mut lossy_report = report.clone();
    lossy_report.legacy_adapter = Some(hypervoxel::LegacyAdapterStatus::lossy(
        hypervoxel::LegacyAdapterKind::VoxelisObjVoxelize,
        "triangle epsilon fixture",
    ));
    let lossy_audit = VoxelizationAudit::from_grid_and_report(&grid, &lossy_report);
    assert!(!lossy_audit.exact_adapter_replay);
    assert!(!lossy_audit.exact_audit_ready);
    let mut blank_policy_report = report.clone();
    blank_policy_report.legacy_adapter = Some(hypervoxel::LegacyAdapterStatus::exact(
        hypervoxel::LegacyAdapterKind::VoxelisObjVoxelize,
        "\t",
    ));
    let blank_policy_audit = VoxelizationAudit::from_grid_and_report(&grid, &blank_policy_report);
    assert!(!blank_policy_audit.exact_adapter_replay);
    assert!(!blank_policy_audit.exact_audit_ready);
    assert_eq!(
        grid.get(hypervoxel::VoxelAddress::new(2, [1, 1, 1]).unwrap())
            .unwrap()
            .occupancy,
        OccupancyState::Filled
    );
}

#[test]
fn exact_source_geometry_reports_invalid_degenerate_inputs() {
    let inverted = ExactBox::new([r(2), r(0), r(0)], [r(1), r(1), r(1)], None);
    let box_report = inverted.report();
    assert!(!box_report.exact_box_ready);
    assert_eq!(box_report.invalid_axes, vec![0]);
    assert!(box_report.zero_extent_axes.is_empty());
    assert!(matches!(
        voxelize_exact_box(
            frame(),
            &inverted,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        ),
        Err(HypervoxelError::InvalidSourceGeometry { reason })
            if reason == "box minimum exceeds maximum"
    ));

    let zero_extent = ExactBox::new([r(1), r(0), r(0)], [r(1), r(1), r(1)], None);
    let zero_extent_report = zero_extent.report();
    assert!(!zero_extent_report.exact_box_ready);
    assert_eq!(zero_extent_report.zero_extent_axes, vec![0]);
    assert!(matches!(
        voxelize_exact_box(
            frame(),
            &zero_extent,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        ),
        Err(HypervoxelError::InvalidSourceGeometry { reason })
            if reason == "box has zero extent"
    ));

    let zero_normal = ExactHalfSpace::new([r(0), r(0), r(0)], r(1), None);
    let halfspace_report = zero_normal.report();
    assert!(halfspace_report.zero_normal_rejected);
    assert!(!halfspace_report.exact_halfspace_ready);
    assert_eq!(halfspace_report.known_zero_normal_axes, vec![0, 1, 2]);
    assert!(halfspace_report.unknown_normal_axes.is_empty());
    assert!(matches!(
        voxelize_exact_halfspace(
            frame(),
            &zero_normal,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        ),
        Err(HypervoxelError::InvalidSourceGeometry { reason })
            if reason == "half-space normal is zero"
    ));

    let empty_solid = ExactConvexHalfSpaceSet::new(Vec::new(), None);
    let solid_report = empty_solid.report();
    assert!(solid_report.empty_predicate_set);
    assert!(!solid_report.exact_solid_predicate_ready);
    assert!(matches!(
        voxelize_exact_convex_halfspace_set(
            frame(),
            &empty_solid,
            MaterialRegionId(1),
            VoxelizationPolicy::conservative_cover(),
        ),
        Err(HypervoxelError::InvalidSourceGeometry { reason })
            if reason == "convex half-space set has no predicates"
    ));
}

#[test]
fn conservative_interior_excludes_boundary_cells() {
    let exact_box = ExactBox::new(
        [rf(1, 2), rf(1, 2), rf(1, 2)],
        [rf(5, 2), rf(5, 2), rf(5, 2)],
        None,
    );
    let (grid, report) = voxelize_exact_box(
        frame(),
        &exact_box,
        MaterialRegionId(2),
        VoxelizationPolicy {
            quantization: QuantizationPolicy::ConservativeInterior,
            boundary: BoundaryPolicy::KeepBoundary,
        },
    )
    .unwrap();

    assert_eq!(report.boundary_cells, 26);
    assert_eq!(report.predicate_certificates.boundary_cells, 26);
    assert_eq!(report.predicate_certificates.certified_cells(), 64);
    assert_eq!(grid.len(), 1);
    assert_eq!(report.aggregate.child_count, 64);
    assert_eq!(
        report.aggregate.conservative_occupancy(),
        OccupancyState::Mixed
    );
    assert_eq!(report.aggregate.occupancy_interval.lower, rf(1, 64));
    assert_eq!(report.aggregate.occupancy_interval.upper, rf(1, 64));
    assert!(report.exact_topology_ready());
}

#[test]
fn finite_voxelization_report_preserves_implied_empty_cells_in_aggregate() {
    let exact_box = ExactBox::new([r(10), r(10), r(10)], [r(11), r(11), r(11)], None);
    let (grid, report) = voxelize_exact_box(
        frame(),
        &exact_box,
        MaterialRegionId(2),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();

    assert!(grid.is_empty());
    assert_eq!(report.predicate_certificates.outside_cells, 64);
    assert_eq!(report.aggregate.child_count, 64);
    assert!(report.aggregate.all_empty);
    assert_eq!(report.aggregate.certainty, AggregateCertainty::Exact);
    assert_eq!(
        report.aggregate.conservative_occupancy(),
        OccupancyState::Empty
    );
    assert!(report.aggregate.occupancy_interval.is_point_interval());
    assert_eq!(report.aggregate.occupancy_interval.lower, r(0));
    assert_eq!(report.aggregate.occupancy_interval.upper, r(0));
}

#[test]
fn exact_halfspace_voxelization_classifies_linear_boundary_cells() {
    let halfspace =
        ExactHalfSpace::new([r(1), r(0), r(0)], r(2), Some(GridSource::new("plane", 1)));
    let (grid, report) = voxelize_exact_halfspace(
        frame(),
        &halfspace,
        MaterialRegionId(8),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();

    assert_eq!(report.boundary_cells, 16);
    assert_eq!(report.predicate_certificates.inside_cells, 32);
    assert_eq!(report.predicate_certificates.boundary_cells, 16);
    assert_eq!(report.predicate_certificates.outside_cells, 16);
    assert_eq!(grid.len(), 48);
    assert_eq!(
        grid.get(VoxelAddress::new(2, [0, 0, 0]).unwrap())
            .unwrap()
            .occupancy,
        OccupancyState::Filled
    );
    assert_eq!(
        grid.get(VoxelAddress::new(2, [2, 0, 0]).unwrap())
            .unwrap()
            .occupancy,
        OccupancyState::Boundary
    );
    assert_eq!(report.freshness(), hypervoxel::FreshnessStatus::Stale);
}

#[test]
fn exact_convex_halfspace_set_voxelizes_closed_solid_without_triangle_epsilons() {
    let solid = ExactConvexHalfSpaceSet::new(
        vec![
            ExactHalfSpace::new([r(-1), r(0), r(0)], r(-1), None),
            ExactHalfSpace::new([r(1), r(0), r(0)], r(3), None),
            ExactHalfSpace::new([r(0), r(-1), r(0)], r(-1), None),
            ExactHalfSpace::new([r(0), r(1), r(0)], r(3), None),
            ExactHalfSpace::new([r(0), r(0), r(-1)], r(-1), None),
            ExactHalfSpace::new([r(0), r(0), r(1)], r(3), None),
        ],
        Some(GridSource::new("convex-cube", 1)),
    );
    let (grid, report) = voxelize_exact_convex_halfspace_set(
        frame(),
        &solid,
        MaterialRegionId(11),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();

    assert!(solid.has_predicates());
    assert_eq!(report.boundary_cells, 56);
    assert_eq!(report.unknown_cells, 0);
    assert_eq!(report.predicate_certificates.inside_cells, 8);
    assert_eq!(report.predicate_certificates.boundary_cells, 56);
    assert_eq!(report.predicate_certificates.classified_cells(), 64);
    assert_eq!(grid.len(), 64);
    assert!(report.aggregate.has_boundary);
    assert_eq!(
        grid.get(VoxelAddress::new(2, [2, 2, 2]).unwrap())
            .unwrap()
            .occupancy,
        OccupancyState::Filled
    );
}

#[test]
fn boundary_as_unknown_keeps_uncertainty_explicit() {
    let exact_box = ExactBox::new(
        [rf(1, 2), rf(1, 2), rf(1, 2)],
        [rf(5, 2), rf(5, 2), rf(5, 2)],
        None,
    );
    let (grid, report) = voxelize_exact_box(
        frame(),
        &exact_box,
        MaterialRegionId(2),
        VoxelizationPolicy {
            quantization: QuantizationPolicy::ConservativeCover,
            boundary: BoundaryPolicy::BoundaryAsUnknown,
        },
    )
    .unwrap();

    assert_eq!(report.boundary_cells, 26);
    assert_eq!(report.unknown_cells, 26);
    assert_eq!(report.predicate_certificates.unknown_cells, 0);
    assert_eq!(report.predicate_certificates.boundary_cells, 26);
    assert!(report.aggregate.has_unknown);
    assert!(!report.exact_topology_ready());
    assert_eq!(grid.len(), 27);
}

#[test]
fn prepared_query_region_aggregates_stored_cells() {
    let exact_box = ExactBox::new([r(1), r(1), r(1)], [r(3), r(3), r(3)], None);
    let (grid, report) = voxelize_exact_box(
        frame(),
        &exact_box,
        MaterialRegionId(4),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    let prepared = PreparedVoxelGrid::new(report.frame.clone(), grid, report.aggregate.clone())
        .with_report(report);

    let region = QueryRegion {
        min: [1, 1, 1],
        max: [2, 2, 2],
        depth: 2,
    };
    let aggregate = prepared.query_region_aggregate(&region).unwrap();
    assert_eq!(aggregate.child_count, 8);
    assert!(aggregate.all_filled);
    assert_eq!(prepared.stored_non_empty_addresses().len(), 8);
}

#[test]
fn prepared_aabb_broad_phase_reports_candidates_rejections_and_unknowns_separately() {
    let exact_box = ExactBox::new([r(1), r(1), r(1)], [r(3), r(3), r(3)], None);
    let (grid, report) = voxelize_exact_box(
        frame(),
        &exact_box,
        MaterialRegionId(4),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    let prepared = PreparedVoxelGrid::new(report.frame.clone(), grid, report.aggregate.clone())
        .with_report(report);

    let broad_phase = prepared
        .query_aabb_broad_phase(&ExactAabb3 {
            min: [r(0), r(0), r(0)],
            max: [r(1), r(1), r(1)],
        })
        .unwrap();
    assert!(broad_phase.is_fully_decided());
    assert!(broad_phase.certified_broad_phase_ready);
    assert_eq!(broad_phase.tested_cells, 8);
    assert!(broad_phase.has_tested_cells);
    assert_eq!(broad_phase.candidates.len(), 1);
    assert_eq!(broad_phase.rejected_addresses.len(), 7);
    assert_eq!(broad_phase.unknown_addresses.len(), 0);
    assert_eq!(
        broad_phase.candidates[0].address,
        VoxelAddress::new(2, [1, 1, 1]).unwrap()
    );
    assert_eq!(
        broad_phase.candidates[0].relation,
        Aabb3Intersection::Touching
    );

    let empty_prepared = PreparedVoxelGrid::new(
        prepared.frame.clone(),
        SparseVoxelGrid::new(prepared.frame.clone()),
        hypervoxel::VoxelAggregateFacts::from_cells(std::iter::empty::<&VoxelCell>()),
    );
    let empty_broad_phase = empty_prepared
        .query_aabb_broad_phase(&ExactAabb3 {
            min: [r(0), r(0), r(0)],
            max: [r(1), r(1), r(1)],
        })
        .unwrap();
    assert_eq!(empty_broad_phase.tested_cells, 0);
    assert!(!empty_broad_phase.has_tested_cells);
    assert!(!empty_broad_phase.certified_broad_phase_ready);
    assert!(!empty_broad_phase.is_fully_decided());
}

#[test]
fn deterministic_snapshot_is_stable_and_includes_side_tables() {
    let exact_box = ExactBox::new([r(1), r(1), r(1)], [r(2), r(2), r(2)], None);
    let (grid, _) = voxelize_exact_box(
        frame(),
        &exact_box,
        MaterialRegionId(9),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    let mut side_tables = VoxelSideTables::default();
    side_tables.insert_material(
        MaterialRegionId(9),
        MaterialRegionRecord {
            label: "calibrated resin".into(),
            density: Some(rf(117, 100)),
            provenance: "test fixture".into(),
        },
    );
    side_tables.insert_field_sample(
        FieldSampleId(3),
        FieldSampleRecord {
            label: "dose".into(),
            lower: Some(rf(1, 10)),
            upper: Some(rf(9, 10)),
            provenance: "fixture".into(),
        },
    );
    side_tables.insert_process_state(
        ProcessStateId(5),
        ProcessStateRecord {
            label: "gelled".into(),
            provenance: "fixture".into(),
        },
    );

    let left = DeterministicSnapshot::text_v1(&grid, &side_tables);
    let right = DeterministicSnapshot::text_v1(&grid, &side_tables);
    assert_eq!(left, right);
    let report = left.report();
    assert!(report.exact_snapshot_replay_ready);
    assert!(report.full_frame_metadata);
    assert!(report.side_table_records_included);
    assert!(report.serialized_cell_records > 0);
    assert!(report.has_cell_records);
    assert_eq!(report.byte_len, left.bytes.len());
    let text = String::from_utf8(left.bytes).unwrap();
    assert!(text.contains("hypervoxel-text-v1"));
    assert!(text.contains("material 9"));
    assert!(text.contains("field_sample 3"));
    assert!(text.contains("process_state 5"));
}

#[test]
fn chunk_paged_process_state_audit_preserves_side_table_boundaries() {
    let mut grid = SparseVoxelGrid::new(frame());
    grid.set(
        VoxelAddress::new(2, [1, 1, 1]).unwrap(),
        VoxelCell::process_state(ProcessStateId(5)),
    )
    .unwrap();
    grid.set(
        VoxelAddress::new(2, [2, 1, 1]).unwrap(),
        VoxelCell::process_state(ProcessStateId(6)),
    )
    .unwrap();
    let mut side_tables = VoxelSideTables::default();
    side_tables.insert_process_state(
        ProcessStateId(5),
        ProcessStateRecord {
            label: "gelled".into(),
            provenance: "fixture".into(),
        },
    );
    side_tables.insert_process_state(
        ProcessStateId(6),
        ProcessStateRecord {
            label: String::new(),
            provenance: String::new(),
        },
    );

    let paged = ChunkPagedSparseGrid::from_sparse_grid(&grid, ChunkShape::new(1).unwrap()).unwrap();
    let audit = audit_chunk_paged_process_states(&paged, &side_tables);
    assert_eq!(audit.tested_pages, 2);
    assert_eq!(audit.tested_cells, 2);
    assert_eq!(audit.process_payload_cells, 2);
    assert_eq!(audit.non_process_payload_cells, 0);
    assert_eq!(
        audit.referenced_states,
        [ProcessStateId(5), ProcessStateId(6)].into()
    );
    assert_eq!(audit.resolved_records, 2);
    assert_eq!(audit.empty_labels, [ProcessStateId(6)].into());
    assert_eq!(audit.empty_provenance, [ProcessStateId(6)].into());
    assert!(!audit.is_complete());
    assert!(!audit.exact_paged_process_audit_ready);

    grid.set(
        VoxelAddress::new(2, [3, 1, 1]).unwrap(),
        VoxelCell::process_state(ProcessStateId(7)),
    )
    .unwrap();
    grid.set(
        VoxelAddress::new(2, [0, 1, 1]).unwrap(),
        VoxelCell::unknown(),
    )
    .unwrap();
    grid.set(
        VoxelAddress::new(2, [0, 2, 1]).unwrap(),
        VoxelCell::lossy_adapter_value(12),
    )
    .unwrap();
    let blocked_paged =
        ChunkPagedSparseGrid::from_sparse_grid(&grid, ChunkShape::new(1).unwrap()).unwrap();
    let blocked = audit_chunk_paged_process_states(&blocked_paged, &side_tables);
    assert_eq!(blocked.process_payload_cells, 3);
    assert_eq!(blocked.non_process_payload_cells, 2);
    assert_eq!(blocked.missing_records, [ProcessStateId(7)].into());
    assert_eq!(blocked.unknown_cells, 1);
    assert_eq!(blocked.lossy_cells, 1);
    assert!(!blocked.exact_paged_process_audit_ready);

    side_tables.insert_process_state(
        ProcessStateId(6),
        ProcessStateRecord {
            label: "cured".into(),
            provenance: "fixture".into(),
        },
    );
    let ready_grid = {
        let mut ready = SparseVoxelGrid::new(frame());
        ready
            .set(
                VoxelAddress::new(2, [1, 1, 1]).unwrap(),
                VoxelCell::process_state(ProcessStateId(5)),
            )
            .unwrap();
        ready
            .set(
                VoxelAddress::new(2, [2, 1, 1]).unwrap(),
                VoxelCell::process_state(ProcessStateId(6)),
            )
            .unwrap();
        ready
    };
    let ready_paged =
        ChunkPagedSparseGrid::from_sparse_grid(&ready_grid, ChunkShape::new(1).unwrap()).unwrap();
    let ready = audit_chunk_paged_process_states(&ready_paged, &side_tables);
    assert!(ready.is_complete());
    assert!(ready.exact_paged_process_audit_ready);

    let empty_paged = ChunkPagedSparseGrid::from_sparse_grid(
        &SparseVoxelGrid::new(frame()),
        ChunkShape::new(1).unwrap(),
    )
    .unwrap();
    let empty = audit_chunk_paged_process_states(&empty_paged, &side_tables);
    assert_eq!(empty.tested_pages, 0);
    assert_eq!(empty.tested_cells, 0);
    assert!(!empty.has_process_states);
    assert!(!empty.is_complete());
    assert!(!empty.exact_paged_process_audit_ready);
}

#[test]
fn deterministic_binary_snapshot_is_stable_and_keeps_exact_scalars_as_bytes() {
    let exact_box = ExactBox::new([r(1), r(1), r(1)], [r(2), r(2), r(2)], None);
    let (grid, _) = voxelize_exact_box(
        frame(),
        &exact_box,
        MaterialRegionId(9),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    let side_tables = VoxelSideTables::default();

    let left = DeterministicSnapshot::binary_v1(&grid, &side_tables);
    let right = DeterministicSnapshot::binary_v1(&grid, &side_tables);
    assert_eq!(left, right);
    assert_eq!(left.format, SnapshotFormat::BinaryV1);
    assert!(left.report().exact_snapshot_replay_ready);
    assert!(left.report().exact_scalar_encoding);
    assert_eq!(left.report().serialized_cell_records, 1);
    assert!(left.report().has_cell_records);
    assert!(left.bytes.starts_with(b"HYPERVOXEL-BIN-V1\0"));
    assert!(!left.bytes.windows(3).any(|bytes| bytes == b"NaN"));

    let empty = DeterministicSnapshot::binary_v1(
        &SparseVoxelGrid::new(frame()),
        &VoxelSideTables::default(),
    );
    let empty_report = empty.report();
    assert_eq!(empty_report.serialized_cell_records, 0);
    assert!(!empty_report.has_cell_records);
    assert!(!empty_report.exact_snapshot_replay_ready);
}

#[test]
fn deterministic_run_length_snapshot_groups_identical_morton_runs() {
    let mut grid = hypervoxel::SparseVoxelGrid::new(frame());
    for x in 0..4 {
        grid.set(
            VoxelAddress::new(2, [x, 0, 0]).unwrap(),
            VoxelCell::material(MaterialRegionId(1)),
        )
        .unwrap();
    }

    let snapshot = DeterministicSnapshot::run_length_binary_v1(&grid);
    assert_eq!(snapshot.format, SnapshotFormat::RunLengthBinaryV1);
    let report = snapshot.report();
    assert!(report.exact_address_encoding);
    assert!(report.serialized_cell_records > 0);
    assert!(report.has_cell_records);
    assert!(!report.full_frame_metadata);
    assert!(!report.side_table_records_included);
    assert!(!report.exact_snapshot_replay_ready);
    assert!(snapshot.bytes.starts_with(b"HYPERVOXEL-RLE-V1\0"));
    assert!(
        snapshot.bytes.len()
            < DeterministicSnapshot::binary_v1(&grid, &VoxelSideTables::default())
                .bytes
                .len()
    );
}

#[test]
fn material_query_reports_missing_side_table_records_without_interpreting_material_laws() {
    let mut grid = hypervoxel::SparseVoxelGrid::new(frame());
    grid.set(
        VoxelAddress::new(2, [1, 1, 1]).unwrap(),
        VoxelCell::material(MaterialRegionId(2)),
    )
    .unwrap();
    grid.set(
        VoxelAddress::new(2, [2, 1, 1]).unwrap(),
        VoxelCell::material(MaterialRegionId(3)),
    )
    .unwrap();
    let mut side_tables = VoxelSideTables::default();
    side_tables.insert_material(
        MaterialRegionId(2),
        MaterialRegionRecord {
            label: "known".into(),
            density: None,
            provenance: "fixture".into(),
        },
    );

    let report = query_material_regions(&grid, &side_tables);
    assert_eq!(report.referenced.len(), 2);
    assert!(report.has_references());
    assert!(report.missing_records.contains(&MaterialRegionId(3)));
    assert!(!report.is_fully_resolved());
    let metadata = report_material_region_metadata(&report, &side_tables);
    assert_eq!(metadata.referenced_regions, 2);
    assert!(metadata.has_material_regions);
    assert_eq!(metadata.resolved_records, 1);
    assert!(metadata.missing_records.contains(&MaterialRegionId(3)));
    assert!(
        metadata
            .records_missing_density
            .contains(&MaterialRegionId(2))
    );
    assert!(!metadata.is_complete());
    assert_eq!(metadata.certainty, AggregateCertainty::Unknown);
    let paged = ChunkPagedSparseGrid::from_sparse_grid(&grid, ChunkShape::new(1).unwrap()).unwrap();
    let paged_material = audit_chunk_paged_material_regions(&paged, &side_tables);
    assert_eq!(paged_material.query, report);
    assert_eq!(paged_material.metadata, metadata);
    assert_eq!(paged_material.tested_pages, 2);
    assert_eq!(paged_material.tested_cells, 2);
    assert_eq!(paged_material.material_payload_cells, 2);
    assert_eq!(paged_material.non_material_payload_cells, 0);
    assert_eq!(paged_material.unknown_cells, 0);
    assert_eq!(paged_material.lossy_cells, 0);
    assert!(!paged_material.exact_paged_material_audit_ready);

    let mut blocked_grid = grid.clone();
    blocked_grid
        .set(
            VoxelAddress::new(2, [3, 1, 1]).unwrap(),
            VoxelCell::unknown(),
        )
        .unwrap();
    blocked_grid
        .set(
            VoxelAddress::new(2, [0, 1, 1]).unwrap(),
            VoxelCell::lossy_adapter_value(77),
        )
        .unwrap();
    let blocked_paged =
        ChunkPagedSparseGrid::from_sparse_grid(&blocked_grid, ChunkShape::new(1).unwrap()).unwrap();
    let blocked_material = audit_chunk_paged_material_regions(&blocked_paged, &side_tables);
    assert_eq!(blocked_material.query, report);
    assert_eq!(blocked_material.material_payload_cells, 2);
    assert_eq!(blocked_material.non_material_payload_cells, 2);
    assert_eq!(blocked_material.unknown_cells, 1);
    assert_eq!(blocked_material.lossy_cells, 1);
    assert!(!blocked_material.exact_paged_material_audit_ready);

    let mut palette = MaterialDisplayPalette::default();
    palette.insert(
        MaterialRegionId(2),
        MaterialDisplayColor {
            r: 255,
            g: 0,
            b: 0,
            a: 255,
        },
    );
    let colors = lookup_material_display_colors(&report, &palette);
    assert_eq!(colors.resolved_colors, 1);
    assert!(colors.has_material_regions);
    assert_eq!(colors.missing_colors, vec![MaterialRegionId(3)]);
    assert!(!colors.complete_display_palette_ready);
    assert!(!colors.adapter.exact_replay);

    palette.insert(
        MaterialRegionId(3),
        MaterialDisplayColor {
            r: 0,
            g: 0,
            b: 255,
            a: 255,
        },
    );
    let complete_colors = lookup_material_display_colors(&report, &palette);
    assert_eq!(complete_colors.resolved_colors, 2);
    assert!(complete_colors.has_material_regions);
    assert!(complete_colors.missing_colors.is_empty());
    assert!(complete_colors.complete_display_palette_ready);
    assert!(!complete_colors.adapter.exact_replay);

    let empty_query = query_material_regions(&SparseVoxelGrid::new(frame()), &side_tables);
    assert!(!empty_query.has_references());
    assert!(!empty_query.is_fully_resolved());
    let empty_metadata = report_material_region_metadata(&empty_query, &side_tables);
    assert_eq!(empty_metadata.referenced_regions, 0);
    assert!(!empty_metadata.has_material_regions);
    assert!(!empty_metadata.is_complete());
    assert_eq!(empty_metadata.certainty, AggregateCertainty::Unknown);
    let empty_colors = lookup_material_display_colors(&empty_query, &palette);
    assert!(!empty_colors.has_material_regions);
    assert!(!empty_colors.complete_display_palette_ready);
    let empty_paged = ChunkPagedSparseGrid::from_sparse_grid(
        &SparseVoxelGrid::new(frame()),
        ChunkShape::new(1).unwrap(),
    )
    .unwrap();
    let empty_material = audit_chunk_paged_material_regions(&empty_paged, &side_tables);
    assert_eq!(empty_material.tested_pages, 0);
    assert_eq!(empty_material.tested_cells, 0);
    assert!(!empty_material.query.has_references());
    assert!(!empty_material.exact_paged_material_audit_ready);
}

#[test]
fn material_metadata_report_accepts_only_explicit_density_and_provenance() {
    let mut grid = hypervoxel::SparseVoxelGrid::new(frame());
    grid.set(
        VoxelAddress::new(2, [1, 1, 1]).unwrap(),
        VoxelCell::material(MaterialRegionId(7)),
    )
    .unwrap();
    let mut side_tables = VoxelSideTables::default();
    side_tables.insert_material(
        MaterialRegionId(7),
        MaterialRegionRecord {
            label: "known-density".into(),
            density: Some(rf(123, 100)),
            provenance: "lot:alpha".into(),
        },
    );

    let query = query_material_regions(&grid, &side_tables);
    let metadata = report_material_region_metadata(&query, &side_tables);
    assert!(metadata.has_material_regions);
    assert!(metadata.is_complete());
    assert_eq!(metadata.records_with_density, [MaterialRegionId(7)].into());
    assert_eq!(metadata.certainty, AggregateCertainty::Exact);
    let paged = ChunkPagedSparseGrid::from_sparse_grid(&grid, ChunkShape::new(1).unwrap()).unwrap();
    let paged_material = audit_chunk_paged_material_regions(&paged, &side_tables);
    assert_eq!(paged_material.query, query);
    assert_eq!(paged_material.metadata, metadata);
    assert_eq!(
        paged_material.referenced_regions,
        [MaterialRegionId(7)].into()
    );
    assert!(paged_material.exact_paged_material_audit_ready);
}

#[test]
fn sparse_grid_diff_reports_semantic_storage_mismatches() {
    let address = VoxelAddress::new(2, [1, 1, 1]).unwrap();
    let mut left = hypervoxel::SparseVoxelGrid::new(frame());
    let mut right = hypervoxel::SparseVoxelGrid::new(frame());
    left.set(address, VoxelCell::material(MaterialRegionId(1)))
        .unwrap();
    right
        .set(address, VoxelCell::material(MaterialRegionId(2)))
        .unwrap();
    right
        .set(
            VoxelAddress::new(2, [2, 2, 2]).unwrap(),
            VoxelCell::material(MaterialRegionId(2)),
        )
        .unwrap();

    let diff = diff_sparse_grids(&left, &right);
    assert!(!diff.is_equal());
    assert!(!diff.semantic_equivalence_ready);
    assert!(diff.frame_matches);
    assert_eq!(diff.compared_addresses, 2);
    assert!(diff.has_compared_addresses);
    assert_eq!(diff.mismatch_count, 2);
    assert_eq!(diff.differing_cells, vec![address]);
    assert_eq!(diff.only_right.len(), 1);
    let paged_left =
        ChunkPagedSparseGrid::from_sparse_grid(&left, ChunkShape::new(1).unwrap()).unwrap();
    let paged_right =
        ChunkPagedSparseGrid::from_sparse_grid(&right, ChunkShape::new(1).unwrap()).unwrap();
    let paged_diff = diff_chunk_paged_sparse_grids(&paged_left, &paged_right);
    assert!(!paged_diff.is_equal());
    assert!(paged_diff.frame_matches);
    assert!(paged_diff.shape_matches);
    assert_eq!(paged_diff.left_pages, 1);
    assert_eq!(paged_diff.right_pages, 2);
    assert_eq!(paged_diff.shared_pages, 1);
    assert_eq!(paged_diff.only_right_pages.len(), 1);
    assert_eq!(paged_diff.compared_addresses, diff.compared_addresses);
    assert_eq!(paged_diff.only_left, diff.only_left);
    assert_eq!(paged_diff.only_right, diff.only_right);
    assert_eq!(paged_diff.differing_cells, diff.differing_cells);
    assert_eq!(paged_diff.mismatch_count, diff.mismatch_count);
    assert!(paged_diff.exact_page_diff_ready);
    assert!(!paged_diff.semantic_equivalence_ready);

    let equal = diff_sparse_grids(&left, &left);
    assert!(equal.frame_matches);
    assert!(equal.has_compared_addresses);
    assert!(equal.semantic_equivalence_ready);
    assert_eq!(equal.mismatch_count, 0);
    let paged_equal = diff_chunk_paged_sparse_grids(&paged_left, &paged_left);
    assert!(paged_equal.frame_matches);
    assert!(paged_equal.shape_matches);
    assert!(paged_equal.exact_page_diff_ready);
    assert!(paged_equal.semantic_equivalence_ready);
    assert_eq!(paged_equal.mismatch_count, 0);

    let differently_paged_left =
        ChunkPagedSparseGrid::from_sparse_grid(&left, ChunkShape::new(2).unwrap()).unwrap();
    let shape_diff = diff_chunk_paged_sparse_grids(&paged_left, &differently_paged_left);
    assert!(shape_diff.frame_matches);
    assert!(!shape_diff.shape_matches);
    assert_eq!(shape_diff.compared_addresses, 1);
    assert_eq!(shape_diff.mismatch_count, 1);
    assert!(!shape_diff.exact_page_diff_ready);
    assert!(!shape_diff.semantic_equivalence_ready);

    let empty_left = SparseVoxelGrid::new(frame());
    let empty_right = SparseVoxelGrid::new(frame());
    let empty_equal = diff_sparse_grids(&empty_left, &empty_right);
    assert!(empty_equal.frame_matches);
    assert_eq!(empty_equal.compared_addresses, 0);
    assert!(!empty_equal.has_compared_addresses);
    assert_eq!(empty_equal.mismatch_count, 0);
    assert!(!empty_equal.semantic_equivalence_ready);
    assert!(!empty_equal.is_equal());
    let empty_paged_left =
        ChunkPagedSparseGrid::from_sparse_grid(&empty_left, ChunkShape::new(1).unwrap()).unwrap();
    let empty_paged_right =
        ChunkPagedSparseGrid::from_sparse_grid(&empty_right, ChunkShape::new(1).unwrap()).unwrap();
    let empty_paged = diff_chunk_paged_sparse_grids(&empty_paged_left, &empty_paged_right);
    assert!(empty_paged.frame_matches);
    assert!(empty_paged.shape_matches);
    assert_eq!(empty_paged.left_pages, 0);
    assert_eq!(empty_paged.compared_addresses, 0);
    assert!(!empty_paged.has_compared_addresses);
    assert!(empty_paged.exact_page_diff_ready);
    assert!(!empty_paged.semantic_equivalence_ready);

    let shifted_frame = GridFrame::builder()
        .origin([r(1), r(0), r(0)])
        .pitch([r(1), r(1), r(1)])
        .depth(2)
        .build()
        .unwrap();
    let mut shifted = hypervoxel::SparseVoxelGrid::new(shifted_frame);
    shifted
        .set(address, VoxelCell::material(MaterialRegionId(1)))
        .unwrap();
    let frame_diff = diff_sparse_grids(&left, &shifted);
    assert!(!frame_diff.frame_matches);
    assert_eq!(frame_diff.compared_addresses, 1);
    assert_eq!(frame_diff.mismatch_count, 1);
    assert!(!frame_diff.semantic_equivalence_ready);
    let shifted_paged =
        ChunkPagedSparseGrid::from_sparse_grid(&shifted, ChunkShape::new(1).unwrap()).unwrap();
    let paged_frame_diff = diff_chunk_paged_sparse_grids(&paged_left, &shifted_paged);
    assert!(!paged_frame_diff.frame_matches);
    assert!(paged_frame_diff.shape_matches);
    assert_eq!(paged_frame_diff.compared_addresses, 1);
    assert_eq!(paged_frame_diff.mismatch_count, 1);
    assert!(!paged_frame_diff.exact_page_diff_ready);
    assert!(!paged_frame_diff.semantic_equivalence_ready);
}

#[test]
fn frame_facts_and_aabb_handoff_expose_exact_scheduling_without_float_export() {
    let frame = GridFrame::builder()
        .origin([rf(1, 4), rf(-1, 4), r(0)])
        .pitch([rf(1, 2), rf(1, 2), rf(1, 2)])
        .depth(3)
        .source(GridSource::new("exact-frame", 4))
        .build()
        .unwrap();
    let facts = frame.facts();
    assert!(facts.is_exact_rational_frame());
    assert!(facts.has_dyadic_schedule());
    assert_eq!(facts.cells_per_axis, 8);

    let handoff =
        GridAabbHandoff::from_address(&frame, VoxelAddress::new(3, [1, 2, 3]).unwrap()).unwrap();
    assert_eq!(handoff.source.unwrap().id, "exact-frame");
    assert_eq!(handoff.bounds.extent(0), rf(1, 2));
    assert_eq!(handoff.bounds.center()[2], rf(7, 4));

    let handoff =
        GridAabbHandoff::from_address(&frame, VoxelAddress::new(3, [1, 2, 3]).unwrap()).unwrap();
    let lattice = handoff.clone().into_lattice();
    assert_eq!(lattice.source, handoff.source);
    assert_eq!(lattice.min, handoff.bounds.min_vector());
    assert_eq!(lattice.max, handoff.bounds.max_vector());
    let (min_facts, max_facts) = lattice.vector_facts();
    assert!(min_facts.exact.all_exact_rational);
    assert!(max_facts.exact.all_exact_rational);

    let direct_lattice =
        LatticeAabbHandoff::from_address(&frame, VoxelAddress::new(3, [1, 2, 3]).unwrap()).unwrap();
    assert_eq!(direct_lattice.min, lattice.min);
}

#[test]
fn exact_cell_center_and_corners_remain_rational() {
    let bounds = VoxelAddress::new(2, [1, 2, 3])
        .unwrap()
        .bounds(&frame())
        .unwrap();
    assert_eq!(bounds.center(), [rf(3, 2), rf(5, 2), rf(7, 2)]);
    let corners = bounds.corners();
    assert_eq!(corners[0], [r(1), r(2), r(3)]);
    assert_eq!(corners[7], [r(2), r(3), r(4)]);
}

#[test]
fn exposed_faces_are_exact_before_lossy_mesh_export() {
    let exact_box = ExactBox::new([r(1), r(1), r(1)], [r(3), r(3), r(3)], None);
    let (grid, _) = voxelize_exact_box(
        frame(),
        &exact_box,
        MaterialRegionId(4),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();

    let shell = extract_exposed_faces_with_report(&grid).unwrap();
    assert_eq!(shell.exact_faces, 24);
    assert!(shell.has_exact_faces);
    assert_eq!(shell.skipped_unknown_cells, 0);
    assert_eq!(shell.skipped_lossy_cells, 0);
    assert_eq!(shell.unknown_neighbor_sides, 0);
    assert_eq!(shell.lossy_neighbor_sides, 0);
    assert!(shell.exact_shell_ready);
    let faces = extract_exposed_faces(&grid).unwrap();
    assert_eq!(faces.len(), 24);
    assert!(faces.iter().all(|face| face.cell_bounds.extent(0) == r(1)));
    assert!(
        faces
            .iter()
            .all(|face| face.side.integer_normal() != [0, 0, 0])
    );

    let report = LossyMeshExportReport::quad_faces(faces.len(), "quad-per-exact-face display");
    assert_eq!(report.exact_faces, 24);
    assert_eq!(report.display_triangles, 48);
    assert!(report.exact_face_identity_preserved);
    assert!(report.display_only);
    assert!(!report.exact_geometry_replay_ready);
    assert!(!report.adapter.exact_replay);

    let mesh = lossy_quad_mesh_from_faces(&faces, "quad-per-exact-face display").unwrap();
    assert_eq!(mesh.vertices.len(), 96);
    assert_eq!(mesh.triangles.len(), 48);
    assert_eq!(mesh.report, report);
    let obj = lossy_obj_from_quad_mesh(&mesh);
    assert!(obj.text.starts_with("# hypervoxel lossy obj preview"));
    assert!(obj.text.contains("\nf "));
    assert_eq!(obj.vertex_records, mesh.vertices.len());
    assert_eq!(obj.face_records, mesh.triangles.len());
    assert!(obj.preview_only);
    assert!(!obj.adapter.exact_replay);

    let patches = greedy_face_patch_plan(&faces, "test preview");
    assert_eq!(patches.exact_faces, 24);
    assert!(patches.patches.len() < faces.len());
    assert!(!patches.export_report.adapter.exact_replay);

    let mut uncertain = grid.clone();
    uncertain
        .set(
            VoxelAddress::new(2, [1, 1, 1]).unwrap(),
            VoxelCell::unknown(),
        )
        .unwrap();
    uncertain
        .set(
            VoxelAddress::new(2, [2, 1, 1]).unwrap(),
            VoxelCell::lossy_adapter_value(9),
        )
        .unwrap();
    let uncertain_shell = extract_exposed_faces_with_report(&uncertain).unwrap();
    assert_eq!(uncertain_shell.skipped_unknown_cells, 1);
    assert_eq!(uncertain_shell.skipped_lossy_cells, 1);
    assert!(uncertain_shell.unknown_neighbor_sides > 0);
    assert!(uncertain_shell.lossy_neighbor_sides > 0);
    assert!(!uncertain_shell.exact_shell_ready);

    let empty_shell = extract_exposed_faces_with_report(&SparseVoxelGrid::new(frame())).unwrap();
    assert_eq!(empty_shell.exact_faces, 0);
    assert!(!empty_shell.has_exact_faces);
    assert!(!empty_shell.exact_shell_ready);
}

#[test]
fn compression_export_and_handoff_reports_keep_adapter_status_explicit() {
    let exact_box = ExactBox::new([r(1), r(1), r(1)], [r(3), r(3), r(3)], None);
    let (grid, report) = voxelize_exact_box(
        frame(),
        &exact_box,
        MaterialRegionId(4),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();

    let storage_report = CompressedStorageManifest {
        kind: CompressedStorageKind::SparseVoxelDag,
        stored_cells: grid.len(),
        physical_records: 5,
        chunk_shape: None,
        preserves_aggregate_facts: true,
        preserves_payload_ids: true,
        preserves_side_table_links: true,
    }
    .report();
    assert_eq!(storage_report.replay_status, StorageReplayStatus::Exact);
    assert!(storage_report.has_stored_cells);
    assert!(storage_report.physical_layout_ready);
    assert!(storage_report.exact_storage_replay_ready);
    assert!(storage_report.certified_aggregate_replay_ready);
    assert_eq!(
        storage_report.aggregate_certainty,
        AggregateCertainty::Exact
    );
    assert!(storage_report.adapter.exact_replay);
    let certified_storage = CompressedStorageManifest {
        kind: CompressedStorageKind::RunLengthSnapshot,
        stored_cells: grid.len(),
        physical_records: grid.len(),
        chunk_shape: None,
        preserves_aggregate_facts: true,
        preserves_payload_ids: true,
        preserves_side_table_links: false,
    }
    .report();
    assert!(!certified_storage.exact_storage_replay_ready);
    assert!(certified_storage.physical_layout_ready);
    assert!(certified_storage.certified_aggregate_replay_ready);
    let empty_storage = CompressedStorageManifest {
        kind: CompressedStorageKind::SparseMap,
        stored_cells: 0,
        physical_records: 0,
        chunk_shape: None,
        preserves_aggregate_facts: true,
        preserves_payload_ids: true,
        preserves_side_table_links: true,
    }
    .report();
    assert!(empty_storage.physical_layout_ready);
    assert!(!empty_storage.has_stored_cells);
    assert!(!empty_storage.exact_storage_replay_ready);
    assert!(!empty_storage.certified_aggregate_replay_ready);
    let impossible_storage = CompressedStorageManifest {
        kind: CompressedStorageKind::SparseVoxelDag,
        stored_cells: grid.len(),
        physical_records: 0,
        chunk_shape: None,
        preserves_aggregate_facts: true,
        preserves_payload_ids: true,
        preserves_side_table_links: true,
    }
    .report();
    assert!(!impossible_storage.physical_layout_ready);
    assert_eq!(
        impossible_storage.replay_status,
        StorageReplayStatus::Unknown
    );
    assert!(!impossible_storage.exact_storage_replay_ready);
    assert!(!impossible_storage.certified_aggregate_replay_ready);
    let memory_report = VoxelMemoryBudgetManifest {
        kind: CompressedStorageKind::SparseVoxelDag,
        estimated_bytes: 4096,
        budget_bytes: 1024,
        preserves_exact_semantics_when_over_budget: true,
    }
    .report();
    assert!(!memory_report.within_budget);
    assert_eq!(memory_report.over_budget_bytes, 3072);
    assert!(memory_report.has_memory_evidence);
    assert!(memory_report.exact_semantics_preserved);
    assert!(memory_report.exact_memory_budget_ready);

    let lossy_memory_report = VoxelMemoryBudgetManifest {
        kind: CompressedStorageKind::RunLengthSnapshot,
        estimated_bytes: 4096,
        budget_bytes: 1024,
        preserves_exact_semantics_when_over_budget: false,
    }
    .report();
    assert!(!lossy_memory_report.exact_semantics_preserved);
    assert!(!lossy_memory_report.exact_memory_budget_ready);

    let vacuous_memory_report = VoxelMemoryBudgetManifest {
        kind: CompressedStorageKind::SparseMap,
        estimated_bytes: 0,
        budget_bytes: 1024,
        preserves_exact_semantics_when_over_budget: true,
    }
    .report();
    assert!(vacuous_memory_report.within_budget);
    assert!(!vacuous_memory_report.has_memory_evidence);
    assert!(vacuous_memory_report.exact_semantics_preserved);
    assert!(!vacuous_memory_report.exact_memory_budget_ready);

    let lossy_export = PreviewExportManifest {
        format: PreviewExportFormat::Gltf,
        exact_input_primitives: grid.len(),
        exported_primitives: 12,
        scalar_policy: PreviewScalarPolicy::PrimitiveFloat,
        preserves_grid_topology: true,
        has_explicit_labels: true,
    }
    .report();
    assert_eq!(lossy_export.freshness, FreshnessStatus::Unknown);
    assert!(lossy_export.has_input_primitives);
    assert!(lossy_export.has_exported_primitives);
    assert!(!lossy_export.exact_grid_topology_replay);
    assert!(!lossy_export.source_geometry_replay);
    assert!(!lossy_export.adapter.exact_replay);

    let sdf_preview = PreviewExportManifest {
        format: PreviewExportFormat::ContinuousSdfPreview,
        exact_input_primitives: grid.len(),
        exported_primitives: grid.len(),
        scalar_policy: PreviewScalarPolicy::ExactString,
        preserves_grid_topology: true,
        has_explicit_labels: true,
    }
    .report();
    assert_eq!(sdf_preview.freshness, FreshnessStatus::Unknown);
    assert!(!sdf_preview.exact_grid_topology_replay);
    assert!(!sdf_preview.source_geometry_replay);
    assert!(!sdf_preview.adapter.exact_replay);

    let empty_export = PreviewExportManifest {
        format: PreviewExportFormat::Vtm,
        exact_input_primitives: 0,
        exported_primitives: 0,
        scalar_policy: PreviewScalarPolicy::ExactString,
        preserves_grid_topology: true,
        has_explicit_labels: true,
    }
    .report();
    assert!(!empty_export.has_input_primitives);
    assert!(!empty_export.has_exported_primitives);
    assert!(!empty_export.exact_grid_topology_replay);
    assert!(!empty_export.adapter.exact_replay);

    let handoff = VoxelHandoffManifest {
        domain: VoxelHandoffDomain::Hyperphysics,
        source: Some(GridSource::new("voxel-artifact", 2)),
        expected_source: Some(GridSource::new("voxel-artifact", 2)),
        required_side_table_links: 1,
        supplied_side_table_links: 0,
        aggregate: report.aggregate,
    }
    .report();
    assert_eq!(handoff.freshness, FreshnessStatus::Current);
    assert_eq!(handoff.side_table_links, SideTableLinkStatus::Missing);
    assert_eq!(handoff.aggregate_certainty, AggregateCertainty::Exact);
    assert!(handoff.has_aggregate_evidence);
    assert!(!handoff.exact_handoff_ready);

    let empty_handoff = VoxelHandoffManifest {
        domain: VoxelHandoffDomain::Hyperphysics,
        source: Some(GridSource::new("voxel-artifact", 2)),
        expected_source: Some(GridSource::new("voxel-artifact", 2)),
        required_side_table_links: 0,
        supplied_side_table_links: 0,
        aggregate: VoxelAggregateFacts::from_cells(std::iter::empty::<&VoxelCell>()),
    }
    .report();
    assert!(!empty_handoff.has_aggregate_evidence);
    assert!(!empty_handoff.exact_handoff_ready);
}

#[test]
fn prepared_queries_keep_connectivity_in_integer_grid_space() {
    let exact_box = ExactBox::new([r(1), r(1), r(1)], [r(3), r(3), r(3)], None);
    let (grid, report) = voxelize_exact_box(
        frame(),
        &exact_box,
        MaterialRegionId(4),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    let prepared = PreparedVoxelGrid::new(report.frame.clone(), grid, report.aggregate.clone())
        .with_report(report);
    let seed = VoxelAddress::new(2, [1, 1, 1]).unwrap();

    assert_eq!(
        voxel_neighbors6(VoxelAddress::new(2, [0, 0, 0]).unwrap()).len(),
        3
    );
    let occupancy = prepared.query_occupancy(seed).unwrap();
    assert!(occupancy.exact_cell_evidence_ready);
    let neighbors = prepared.query_neighbors6(seed);
    assert_eq!(neighbors.neighbors.len(), 6);
    assert!(neighbors.exact_neighbors_ready);
    let component = prepared.query_connected_component(seed).unwrap();
    assert_eq!(component.addresses.len(), 8);
    assert!(component.has_reached_cells);
    assert!(component.exact_component_ready);

    let band = prepared.query_manhattan_band(seed, 1).unwrap();
    assert_eq!(band.distances.len(), 4);
    assert_eq!(band.distances[&seed], 0);
    assert!(band.has_reached_cells);
    assert!(band.exact_distance_band_ready);

    let empty_seed = VoxelAddress::new(2, [0, 0, 0]).unwrap();
    let empty_component = prepared.query_connected_component(empty_seed).unwrap();
    assert!(empty_component.addresses.is_empty());
    assert!(!empty_component.has_reached_cells);
    assert!(!empty_component.exact_component_ready);
    let empty_band = prepared.query_manhattan_band(empty_seed, 1).unwrap();
    assert!(empty_band.distances.is_empty());
    assert!(!empty_band.has_reached_cells);
    assert!(!empty_band.exact_distance_band_ready);
}

#[test]
fn lod_selection_and_distance_preview_are_integer_grid_queries() {
    let exact_box = ExactBox::new([r(1), r(1), r(1)], [r(3), r(3), r(3)], None);
    let (grid, _) = voxelize_exact_box(
        frame(),
        &exact_box,
        MaterialRegionId(4),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();

    let lod = select_lod_cells(&grid, 0).unwrap();
    assert_eq!(lod.target_depth, 0);
    assert_eq!(lod.selected_cells, 1);
    assert!(lod.has_selected_cells);
    assert_eq!(lod.cells.len(), 1);
    assert_eq!(lod.cells[0].address, VoxelAddress::root());
    assert!(lod.certified_lod_aggregate_ready);
    assert_eq!(
        lod.selected_cells,
        lod.exact_aggregate_cells + lod.certified_aggregate_cells
    );
    assert_eq!(lod.unknown_aggregate_cells, 0);
    assert_eq!(lod.lossy_aggregate_cells, 0);
    assert_eq!(
        lod.selected_aggregate().child_count,
        lod.cells[0].aggregate.child_count
    );

    let mut uncertain = SparseVoxelGrid::new(frame());
    uncertain
        .set(
            VoxelAddress::new(2, [0, 0, 0]).unwrap(),
            VoxelCell::unknown(),
        )
        .unwrap();
    uncertain
        .set(
            VoxelAddress::new(2, [3, 3, 3]).unwrap(),
            VoxelCell::lossy_adapter_value(7),
        )
        .unwrap();
    let uncertain_lod = select_lod_cells(&uncertain, 1).unwrap();
    assert_eq!(uncertain_lod.selected_cells, 2);
    assert!(uncertain_lod.has_selected_cells);
    assert_eq!(uncertain_lod.unknown_aggregate_cells, 1);
    assert_eq!(uncertain_lod.lossy_aggregate_cells, 1);
    assert!(!uncertain_lod.certified_lod_aggregate_ready);

    let empty_lod = select_lod_cells(&SparseVoxelGrid::new(frame()), 1).unwrap();
    assert_eq!(empty_lod.selected_cells, 0);
    assert!(!empty_lod.has_selected_cells);
    assert!(!empty_lod.certified_lod_aggregate_ready);
    assert_eq!(empty_lod.selected_aggregate().child_count, 0);

    let preview = sample_manhattan_distance_field(
        &grid,
        QueryRegion {
            min: [0, 0, 0],
            max: [1, 0, 0],
            depth: 2,
        },
    )
    .unwrap();
    assert_eq!(preview.samples.len(), 2);
    assert_eq!(preview.source_cells, grid.len());
    assert!(preview.has_distance_source);
    assert_eq!(preview.samples[0].manhattan_distance, Some(3));
    assert_eq!(preview.samples[1].manhattan_distance, Some(2));
    assert!(preview.exact_address_distance_ready);
    let signed = sample_signed_manhattan_distance_field(
        &grid,
        QueryRegion {
            min: [1, 1, 1],
            max: [1, 1, 1],
            depth: 2,
        },
    )
    .unwrap();
    assert_eq!(signed.source_cells, preview.source_cells);
    assert!(signed.has_distance_source);
    assert_eq!(signed.samples[0].signed_manhattan_distance, Some(0));
    assert!(signed.samples[0].occupied);
    assert!(signed.exact_address_distance_ready);
    assert!(!signed.continuous_sdf_ready);

    let uncertain_preview = sample_manhattan_distance_field(
        &uncertain,
        QueryRegion {
            min: [0, 0, 0],
            max: [0, 0, 0],
            depth: 2,
        },
    )
    .unwrap();
    assert!(uncertain_preview.has_distance_source);
    assert!(!uncertain_preview.exact_address_distance_ready);

    let empty_grid = SparseVoxelGrid::new(frame());
    let empty_preview = sample_manhattan_distance_field(
        &empty_grid,
        QueryRegion {
            min: [0, 0, 0],
            max: [0, 0, 0],
            depth: 2,
        },
    )
    .unwrap();
    assert_eq!(empty_preview.source_cells, 0);
    assert!(!empty_preview.has_distance_source);
    assert_eq!(empty_preview.samples[0].manhattan_distance, None);
    assert!(!empty_preview.exact_address_distance_ready);
}
