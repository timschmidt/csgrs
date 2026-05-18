use hyperreal::{Rational, Real};
use hypervoxel::{
    AggregateCertainty, BoundaryPolicy, CompressedStorageKind, CompressedStorageManifest,
    DeterministicSnapshot, ExactBox, ExactConvexHalfSpaceSet, ExactHalfSpace, FieldSampleId,
    FieldSampleRecord, FreshnessStatus, GridAabbHandoff, GridFrame, GridSource, LatticeAabbHandoff,
    LossyMeshExportReport, MaterialDisplayColor, MaterialDisplayPalette, MaterialRegionId,
    MaterialRegionRecord, OccupancyState, PreparedSparseVoxelGridExt, PreparedVoxelGrid,
    PreviewExportFormat, PreviewExportManifest, PreviewScalarPolicy, ProcessStateId,
    ProcessStateRecord, QuantizationPolicy, QueryRegion, SideTableLinkStatus, SnapshotFormat,
    StorageReplayStatus, VoxelAddress, VoxelCell, VoxelHandoffDomain, VoxelHandoffManifest,
    VoxelMemoryBudgetManifest, VoxelSideTables, VoxelizationAudit, VoxelizationPolicy,
    diff_sparse_grids, extract_exposed_faces, greedy_face_patch_plan,
    lookup_material_display_colors, lossy_obj_from_quad_mesh, lossy_quad_mesh_from_faces,
    query_material_regions, sample_manhattan_distance_field,
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
    assert_eq!(grid.len(), 27);
    assert!(report.aggregate.has_boundary);
    let audit = VoxelizationAudit::from_grid_and_report(&grid, &report);
    assert_eq!(audit.total_frame_cells, 64);
    assert_eq!(audit.stored_cells, 27);
    assert_eq!(audit.boundary_cells, 26);
    assert_eq!(audit.implied_empty_cells, 37);
    assert_eq!(audit.predicate_certified_cells, 64);
    assert_eq!(audit.predicate_unknown_cells, 0);
    assert!(!audit.has_uncertainty());
    assert_eq!(
        grid.get(hypervoxel::VoxelAddress::new(2, [1, 1, 1]).unwrap())
            .unwrap()
            .occupancy,
        OccupancyState::Filled
    );
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
    assert_eq!(
        report.aggregate.conservative_occupancy(),
        OccupancyState::Filled
    );
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
    let text = String::from_utf8(left.bytes).unwrap();
    assert!(text.contains("hypervoxel-text-v1"));
    assert!(text.contains("material 9"));
    assert!(text.contains("field_sample 3"));
    assert!(text.contains("process_state 5"));
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
    assert!(left.bytes.starts_with(b"HYPERVOXEL-BIN-V1\0"));
    assert!(!left.bytes.windows(3).any(|bytes| bytes == b"NaN"));
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
    assert!(report.missing_records.contains(&MaterialRegionId(3)));
    assert!(!report.is_fully_resolved());

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
    assert_eq!(colors.missing_colors, vec![MaterialRegionId(3)]);
    assert!(!colors.adapter.exact_replay);
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
    assert_eq!(diff.differing_cells, vec![address]);
    assert_eq!(diff.only_right.len(), 1);
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
    assert!(!report.adapter.exact_replay);

    let mesh = lossy_quad_mesh_from_faces(&faces, "quad-per-exact-face display").unwrap();
    assert_eq!(mesh.vertices.len(), 96);
    assert_eq!(mesh.triangles.len(), 48);
    assert_eq!(mesh.report, report);
    let obj = lossy_obj_from_quad_mesh(&mesh);
    assert!(obj.text.starts_with("# hypervoxel lossy obj preview"));
    assert!(obj.text.contains("\nf "));
    assert!(!obj.adapter.exact_replay);

    let patches = greedy_face_patch_plan(&faces, "test preview");
    assert_eq!(patches.exact_faces, 24);
    assert!(patches.patches.len() < faces.len());
    assert!(!patches.export_report.adapter.exact_replay);
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
    assert_eq!(
        storage_report.aggregate_certainty,
        AggregateCertainty::Exact
    );
    assert!(storage_report.adapter.exact_replay);
    let memory_report = VoxelMemoryBudgetManifest {
        kind: CompressedStorageKind::SparseVoxelDag,
        estimated_bytes: 4096,
        budget_bytes: 1024,
        preserves_exact_semantics_when_over_budget: true,
    }
    .report();
    assert!(!memory_report.within_budget);
    assert_eq!(memory_report.over_budget_bytes, 3072);
    assert!(memory_report.exact_semantics_preserved);

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
    assert!(!lossy_export.adapter.exact_replay);

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
    assert_eq!(prepared.query_neighbors6(seed).neighbors.len(), 6);
    let component = prepared.query_connected_component(seed).unwrap();
    assert_eq!(component.addresses.len(), 8);

    let band = prepared.query_manhattan_band(seed, 1).unwrap();
    assert_eq!(band.distances.len(), 4);
    assert_eq!(band.distances[&seed], 0);
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
    assert_eq!(lod.cells.len(), 1);
    assert_eq!(lod.cells[0].address, VoxelAddress::root());

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
    assert_eq!(preview.samples[0].manhattan_distance, Some(3));
    assert_eq!(preview.samples[1].manhattan_distance, Some(2));
    let signed = sample_signed_manhattan_distance_field(
        &grid,
        QueryRegion {
            min: [1, 1, 1],
            max: [1, 1, 1],
            depth: 2,
        },
    )
    .unwrap();
    assert_eq!(signed.samples[0].signed_manhattan_distance, Some(0));
    assert!(signed.samples[0].occupied);
}
