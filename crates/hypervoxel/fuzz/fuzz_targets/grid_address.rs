#![no_main]

use std::collections::BTreeSet;

use hyperreal::Real;
use hypervoxel::{
    AdapterNumericContract, AdapterToleranceStatus, AggregateCertainty, CertifiedFieldInterval,
    CertifiedVectorInterval,
    AddressRay, AxisPermutationTransform, ChunkAddress, ChunkPageSummary, ChunkPagedSparseGrid, ChunkShape,
    CompressedStorageKind, CompressedStorageManifest, DeterministicSnapshot, ExactAabb3,
    ExactAffineTransform, ExactBox, ExactConvexHalfSpaceSet, ExactHalfSpace,
    ExactTriangle3, ExactTriangleSolidMesh, ExactTriangleSurfaceMesh, ContinuousFieldVoxelCell,
    ContinuousFieldVoxelManifest, FieldAggregateFacts, FieldEnvelopeFacts, FieldSampleId,
    FieldSampleRecord, FreshnessStatus, GridAabbHandoff, GridBasis, GridCoordinateSystem,
    GridFrame, GridFrameManifest, GridHandedness, GridSource, ImageStackContainer,
    ImageStackManifest, LegacyAdapterKind, LegacyAdapterStatus, LengthUnit,
    MaterialDisplayPalette, MaterialRegionId, MaterialRegionRecord, PreparedExactTriangleSolidMesh,
    PreparedSparseVoxelGridExt, PreparedVoxelGrid, PreviewExportFormat, PreviewExportManifest, PreviewScalarPolicy, ProcessGridArtifact,
    ProcessGridRole, ProcessStateId, ProcessStateRecord, QuantizationPolicy, QueryRegion, SignedAxis, SupportDirection,
    SparseVoxelGrid, SvoVoxelGrid, SweptVolumeProvenance, VoxelAddress, VoxelArtifactId, VoxelArtifactManifest,
    VoxelArtifactRole, VoxelCandidateKind, VoxelCandidateManifest, VoxelCell, VoxelChannelMapping, VoxelEditBatch,
    VoxelFieldCouplingKind, VoxelFieldCouplingManifest, VoxelHandoffDomain, VoxelHandoffManifest,
    VoxelIndexConvention, VoxelIoCompression, VoxelIoMetadata, VoxelMemoryBudgetManifest,
    VoxelSideTables, VoxelSliceNaming, VoxelSliceOrdering, VoxelSpatialAggregateFacts,
    VoxelTraceDimension, VoxelTraceManifest, VoxelizationAudit, VoxelizationPolicy,
    audit_chunk_paged_field_samples, audit_chunk_paged_material_regions,
    audit_chunk_paged_process_states, audit_exact_surface_triangle_mesh_vocabulary,
    audit_exact_voxel_surface_topology, certify_chunk_paged_handoff, chunk_paged_binary_snapshot_v1,
    chunk_paged_exact_surface_triangle_mesh_with_report,
    chunk_paged_run_length_snapshot_v1, chunk_paged_greedy_face_patch_plan_with_report,
    classify_chunk_paged_support_mask, classify_support_mask, continuous_field_address,
    diff_chunk_paged_sparse_grids, diff_sparse_grids,
    extract_chunk_paged_exposed_faces_with_report, extract_exposed_faces,
    exact_voxel_surface_triangle_mesh_from_faces, extract_exposed_faces_with_report,
    extract_svo_exposed_faces_with_report, greedy_face_patch_plan,
    lookup_material_display_colors, lossy_obj_from_quad_mesh, lossy_quad_mesh_from_faces,
    materialize_legacy_voxelis_u8_chunk_paged_storage, query_field_samples,
    query_material_regions, report_material_region_metadata, sample_manhattan_distance_field,
    sample_signed_manhattan_distance_field, select_lod_cells, sweep_address_segment,
    trace_address_ray, voxel_neighbors6, voxelize_exact_box, voxelize_exact_convex_halfspace_set,
    voxelize_exact_halfspace, voxelize_prepared_exact_triangle_solid_mesh,
    voxelize_prepared_exact_triangle_solid_mesh_by_adaptive_axis_sweeps,
    voxelize_prepared_exact_triangle_solid_mesh_by_verified_adaptive_axis_sweeps,
    VoxelAggregateFacts,
};
use libfuzzer_sys::fuzz_target;
use voxelis::{
    MaxDepth, VoxInterner,
    spatial::{VoxOpsWrite, VoxTree},
};

fuzz_target!(|data: (u8, u64, u64, u64)| {
    let (depth_raw, x, y, z) = data;
    let depth = depth_raw % 10;
    let frame = GridFrame::builder().depth(depth).build().unwrap();
    let cells = 1_u64 << depth;
    let address = VoxelAddress::new(depth, [x % cells, y % cells, z % cells]).unwrap();
    let bounds = address.bounds(&frame).unwrap();
    let _ = GridAabbHandoff::from_address(&frame, address)
        .unwrap()
        .into_lattice()
        .vector_facts();
    let frame_manifest = GridFrameManifest {
        frame: frame.clone(),
        basis: if depth_raw & 1 == 0 {
            GridBasis::AxisAligned
        } else {
            GridBasis::Unknown
        },
        handedness: if depth_raw & 2 == 0 {
            GridHandedness::RightHanded
        } else {
            GridHandedness::Unknown
        },
        coordinate_system: if depth_raw & 4 == 0 {
            GridCoordinateSystem::HyperGrid
        } else {
            GridCoordinateSystem::Unknown
        },
        chunk_shape: ChunkShape::new(depth.min(3)).ok(),
    }
    .report();
    assert_eq!(frame_manifest.facts.depth, depth);

    for axis in 0..3 {
        let extent = bounds.extent(axis);
        assert_eq!(extent, Real::from(1));
    }

    if depth > 0 {
        let parent = address.parent().unwrap();
        assert_eq!(parent.depth, depth - 1);
        parent.bounds(&frame).unwrap();
    }
    let chunk_shape = ChunkShape::new(depth.min(3)).unwrap();
    let split = ChunkAddress::split(address, chunk_shape);
    assert!(split.local_in_bounds);
    assert!(split.exact_recompose_ready);
    assert!(split.local_extent <= chunk_shape.cells_per_axis());

    let exact_box = ExactBox::new(
        [Real::from(0), Real::from(0), Real::from(0)],
        [Real::from(cells.min(8)), Real::from(cells.min(8)), Real::from(cells.min(8))],
        None,
    );
    assert!(exact_box.report().exact_box_ready);
    let small_depth = depth.min(3);
    let small_frame = GridFrame::builder().depth(small_depth).build().unwrap();
    let (grid, report) = voxelize_exact_box(
        small_frame,
        &exact_box,
        MaterialRegionId(1),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    let small_cells = 1_u64 << small_depth;
    let covered_cells_per_axis = cells.min(8).min(small_cells);
    let covered_cell_count = covered_cells_per_axis.pow(3);
    assert_eq!(grid.len() as u64, covered_cell_count);
    assert_eq!(report.boundary_cells, 0);
    assert_eq!(
        report.predicate_certificates.inside_cells as u64,
        covered_cell_count
    );
    assert_eq!(report.predicate_certificates.boundary_cells, 0);
    let paged_grid = ChunkPagedSparseGrid::from_sparse_grid(&grid, ChunkShape::new(small_depth.min(2)).unwrap()).unwrap();
    assert_eq!(paged_grid.len() as u64, covered_cell_count);
    assert_eq!(paged_grid.report().summary.stored_cells as u64, covered_cell_count);
    assert!(paged_grid.report().exact_address_replay_ready);
    assert!(paged_grid.report().exact_payload_replay_ready);
    assert!(paged_grid.report().exact_chunk_storage_ready);
    let first_small_address = VoxelAddress::new(small_depth, [0, 0, 0]).unwrap();
    assert_eq!(
        paged_grid.get(first_small_address).unwrap().occupancy,
        grid.get(first_small_address).unwrap().occupancy
    );
    let paged_region = paged_grid
        .query_region_aggregate(&QueryRegion {
            min: [0, 0, 0],
            max: [small_cells - 1, small_cells - 1, small_cells - 1],
            depth: small_depth,
        })
        .unwrap();
    assert_eq!(paged_region.matched_cells as u64, covered_cell_count);
    assert_eq!(paged_region.aggregate.child_count as u64, covered_cell_count);
    assert!(paged_region.exact_page_filter_ready);
    assert!(paged_region.exact_region_query_ready);
    let paged_broad_phase = paged_grid
        .query_aabb_broad_phase(&ExactAabb3 {
            min: [Real::from(0), Real::from(0), Real::from(0)],
            max: [
                Real::from(small_cells),
                Real::from(small_cells),
                Real::from(small_cells),
            ],
        })
        .unwrap();
    assert_eq!(paged_broad_phase.cells.candidates.len() as u64, covered_cell_count);
    assert_eq!(paged_broad_phase.cells.unknown_addresses.len(), 0);
    assert_eq!(paged_broad_phase.unknown_pages, 0);
    assert!(paged_broad_phase.exact_page_filter_ready);
    assert_eq!(
        paged_broad_phase.exact_paged_broad_phase_ready,
        covered_cell_count > 0
    );
    let paged_component = paged_grid.query_connected_component(first_small_address).unwrap();
    assert_eq!(paged_component.addresses.len() as u64, covered_cell_count);
    assert_eq!(paged_component.has_reached_cells, covered_cell_count > 0);
    assert_eq!(
        paged_component.exact_component_ready,
        covered_cell_count > 0
    );
    assert_eq!(paged_component.aggregate.child_count as u64, covered_cell_count);
    let paged_band = paged_grid
        .query_manhattan_band(first_small_address, u32::from(small_depth) + 3)
        .unwrap();
    assert_eq!(paged_band.distances.len() as u64, covered_cell_count);
    assert_eq!(paged_band.has_reached_cells, covered_cell_count > 0);
    assert_eq!(
        paged_band.exact_distance_band_ready,
        covered_cell_count > 0
    );
    assert_eq!(paged_band.aggregate.child_count as u64, covered_cell_count);
    let halfspace =
        ExactHalfSpace::new([Real::from(1), Real::from(0), Real::from(0)], Real::from(1), None);
    assert!(halfspace.report().exact_halfspace_ready);
    voxelize_exact_halfspace(
        GridFrame::builder().depth(small_depth).build().unwrap(),
        &halfspace,
        MaterialRegionId(2),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    let convex = ExactConvexHalfSpaceSet::new(
        vec![
            ExactHalfSpace::new([Real::from(-1), Real::from(0), Real::from(0)], Real::from(0), None),
            ExactHalfSpace::new([Real::from(1), Real::from(0), Real::from(0)], Real::from(cells.min(3)), None),
        ],
        None,
    );
    assert!(convex.report().exact_solid_predicate_ready);
    voxelize_exact_convex_halfspace_set(
        GridFrame::builder().depth(small_depth).build().unwrap(),
        &convex,
        MaterialRegionId(3),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    assert_eq!(report.unknown_cells, 0);
    assert!(report.predicate_certificates.has_classified_cells());
    assert!(report.predicate_certificates.is_fully_certified());
    assert!(report.exact_topology_ready());
    assert!(!report.source_replay_ready());
    let triangle_frame = GridFrame::builder()
        .depth(2)
        .source(GridSource::new("fuzz:adaptive-axis-sweep", u64::from(depth_raw) + 17))
        .build()
        .unwrap();
    let p = |x, y, z| [Real::from(x), Real::from(y), Real::from(z)];
    let tri = |vertices| ExactTriangle3::new(vertices, Some(0));
    let triangle_surface = ExactTriangleSurfaceMesh::new(
        vec![
            tri([p(1, 1, 1), p(1, 3, 3), p(1, 3, 1)]),
            tri([p(1, 1, 1), p(1, 1, 3), p(1, 3, 3)]),
            tri([p(3, 1, 1), p(3, 3, 1), p(3, 1, 3)]),
            tri([p(3, 3, 1), p(3, 3, 3), p(3, 1, 3)]),
            tri([p(1, 1, 1), p(3, 1, 1), p(1, 1, 3)]),
            tri([p(3, 1, 1), p(3, 1, 3), p(1, 1, 3)]),
            tri([p(1, 3, 1), p(1, 3, 3), p(3, 3, 1)]),
            tri([p(3, 3, 1), p(1, 3, 3), p(3, 3, 3)]),
            tri([p(1, 1, 1), p(1, 3, 1), p(3, 1, 1)]),
            tri([p(3, 1, 1), p(1, 3, 1), p(3, 3, 1)]),
            tri([p(1, 1, 3), p(3, 1, 3), p(1, 3, 3)]),
            tri([p(3, 1, 3), p(3, 3, 3), p(1, 3, 3)]),
        ],
        triangle_frame.source().cloned(),
        true,
    );
    let triangle_solid = ExactTriangleSolidMesh::new(triangle_surface, true);
    let prepared_triangle = PreparedExactTriangleSolidMesh::prepare(triangle_solid).unwrap();
    let (triangle_per_cell, triangle_per_cell_report, _) =
        voxelize_prepared_exact_triangle_solid_mesh(
            triangle_frame.clone(),
            &prepared_triangle,
            MaterialRegionId(6),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (triangle_adaptive, triangle_adaptive_report, adaptive_sweep) =
        voxelize_prepared_exact_triangle_solid_mesh_by_adaptive_axis_sweeps(
            triangle_frame.clone(),
            &prepared_triangle,
            MaterialRegionId(6),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    let (triangle_verified, triangle_verified_report, verified_sweep) =
        voxelize_prepared_exact_triangle_solid_mesh_by_verified_adaptive_axis_sweeps(
            triangle_frame,
            &prepared_triangle,
            MaterialRegionId(6),
            VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
    assert_eq!(triangle_adaptive, triangle_per_cell);
    assert_eq!(triangle_verified, triangle_per_cell);
    assert_eq!(
        triangle_adaptive_report.predicate_certificates,
        triangle_per_cell_report.predicate_certificates
    );
    assert_eq!(
        triangle_verified_report.predicate_certificates,
        triangle_per_cell_report.predicate_certificates
    );
    assert_eq!(
        adaptive_sweep.sweep_classified_cells + adaptive_sweep.fallback_cells,
        adaptive_sweep.open_cells
    );
    assert_eq!(
        adaptive_sweep.exact_adaptive_axis_sweep_ready,
        adaptive_sweep.boundary_unknown_cells == 0
            && adaptive_sweep.fallback_unknown_cells == 0
            && adaptive_sweep.fallback_boundary_regression_cells == 0
            && adaptive_sweep.row_parameter_order_unknowns == 0
            && adaptive_sweep.classified_cells > 0
    );
    assert_eq!(verified_sweep.adaptive, adaptive_sweep);
    assert_eq!(verified_sweep.grid_mismatch_cells, 0);
    assert_eq!(
        verified_sweep.exact_verified_adaptive_axis_sweep_ready,
        verified_sweep.adaptive.exact_adaptive_axis_sweep_ready
            && verified_sweep.grid_mismatch_cells == 0
            && verified_sweep.predicate_certificates_match
            && verified_sweep.boundary_counts_match
            && verified_sweep.unknown_counts_match
            && verified_sweep.aggregate_matches
            && verified_sweep.verifier_exact_topology_ready
    );
    assert!(report.aggregate.occupancy_interval.lower <= report.aggregate.occupancy_interval.upper);
    assert_eq!(
        report.aggregate.occupancy_interval.total_cells,
        report.aggregate.child_count
    );
    assert_eq!(
        report.aggregate.child_count,
        usize::try_from(1_u64 << (3 * u32::from(small_depth))).unwrap()
    );
    let mut svo = SvoVoxelGrid::new(GridFrame::builder().depth(small_depth).build().unwrap());
    let svo_address = VoxelAddress::new(
        small_depth,
        [
            x % (1_u64 << small_depth),
            y % (1_u64 << small_depth),
            z % (1_u64 << small_depth),
        ],
    )
    .unwrap();
    let svo_edit = svo
        .set_with_report(svo_address, VoxelCell::material(MaterialRegionId(5)))
        .unwrap();
    assert!(svo_edit.exact_path_replay_ready);
    let svo_report = svo.report();
    assert!(svo_report.root_aggregate_covers_frame);
    assert!(svo_report.has_materialized_evidence);
    assert!(svo_report.exact_dag_replay_ready);
    let (svo_sparse, svo_replay) = svo.replay_sparse_grid_with_report().unwrap();
    assert_eq!(svo_sparse.get(svo_address).unwrap(), VoxelCell::material(MaterialRegionId(5)));
    let (compacted_svo, compaction) =
        SvoVoxelGrid::from_sparse_grid_with_report(&svo_sparse).unwrap();
    let (compacted_sparse, compacted_replay) =
        compacted_svo.replay_sparse_grid_with_report().unwrap();
    assert_eq!(compacted_sparse, svo_sparse);
    assert_eq!(compaction.sparse_replay, compacted_replay);
    assert_eq!(compaction.source_cells, svo_sparse.len());
    assert_eq!(compaction.finest_depth_cells, svo_sparse.len());
    assert_eq!(compaction.non_finest_depth_cells, 0);
    assert!(compaction.semantic_round_trip_matches_source);
    assert_eq!(
        compaction.exact_svo_compaction_ready,
        compaction.source_cells > 0
            && compaction.exact_payload_cells == compaction.source_cells
            && compaction.unknown_cells == 0
            && compaction.lossy_cells == 0
            && compacted_replay.exact_sparse_replay_ready
            && compaction.semantic_round_trip_matches_source
    );
    assert_eq!(svo_replay.logical_leaf_cells, svo_report.logical_leaf_cells);
    assert!(svo_replay.aggregate_replay_matches_root);
    assert_eq!(svo_replay.materialized_sparse_cells, svo_sparse.len());
    assert_eq!(
        svo_replay.exact_sparse_replay_ready,
        svo_report.exact_dag_replay_ready
            && svo_replay.aggregate_replay_matches_root
            && svo_replay.unknown_leaf_cells == 0
            && svo_replay.lossy_leaf_cells == 0
            && svo_replay.exact_payload_cells == svo_replay.expanded_non_empty_leaf_cells
    );
    let (svo_faces, svo_surface) = extract_svo_exposed_faces_with_report(&svo).unwrap();
    let sparse_shell = extract_exposed_faces_with_report(&svo_sparse).unwrap();
    assert_eq!(svo_faces, sparse_shell.faces);
    assert_eq!(svo_surface.shell.exact_faces, sparse_shell.exact_faces);
    assert_eq!(svo_surface.sparse_replay, svo_replay);
    assert_eq!(
        svo_surface.exact_svo_surface_replay_ready,
        svo_replay.exact_sparse_replay_ready
            && svo_surface.shell.exact_shell_ready
            && svo_surface.topology.exact_surface_topology_ready
            && svo_surface.exact_faces > 0
    );
    assert_eq!(
        svo_report.logical_leaf_cells,
        usize::try_from(1_u64 << (3 * u32::from(small_depth))).unwrap()
    );
    assert_eq!(
        svo_report.root_aggregate.occupancy_interval.total_cells,
        svo_report.logical_leaf_cells
    );
    let empty_aggregate = VoxelAggregateFacts::from_cells(std::iter::empty::<&VoxelCell>());
    assert_eq!(empty_aggregate.certainty, AggregateCertainty::Unknown);
    assert_eq!(
        empty_aggregate.occupancy_interval.certainty,
        AggregateCertainty::Unknown
    );
    assert!(empty_aggregate.occupancy_interval.lower < empty_aggregate.occupancy_interval.upper);
    assert_eq!(
        report.predicate_certificates.classified_cells(),
        usize::try_from(1_u64 << (3 * u32::from(small_depth))).unwrap()
    );
    let intake_frame = GridFrame::builder()
        .depth(1)
        .source(GridSource::new("fuzz:sdf", u64::from(depth_raw) + 1))
        .build()
        .unwrap();
    let intake_source = intake_frame.source().cloned();
    let mut intake_rows = Vec::new();
    for iz in 0..2 {
        for iy in 0..2 {
            for ix in 0..2 {
                intake_rows.push(ContinuousFieldVoxelCell::new(
                    continuous_field_address(&intake_frame, [ix, iy, iz]).unwrap(),
                    VoxelCell::material(MaterialRegionId(42)),
                ));
            }
        }
    }
    let intake = ContinuousFieldVoxelManifest {
        frame: intake_frame,
        source: intake_source.clone(),
        expected_source: intake_source,
        expected_cell_count: intake_rows.len(),
        cells: intake_rows,
    };
    assert!(intake.report().exact_materialization_ready);
    assert!(intake.materialize_exact_sparse_grid().is_ok());
    let cell_report = VoxelCell::material(MaterialRegionId(u32::from(depth_raw))).report();
    assert!(cell_report.payload_matches_occupancy);
    assert!(cell_report.exact_cell_evidence_ready);
    let lossy_cell_report = VoxelCell::lossy_adapter_value(u32::from(depth_raw)).report();
    assert!(lossy_cell_report.payload_matches_occupancy);
    assert!(lossy_cell_report.has_lossy);
    assert!(!lossy_cell_report.exact_cell_evidence_ready);
    let audit = VoxelizationAudit::from_grid_and_report(&grid, &report);
    if audit.exact_audit_ready {
        assert!(!audit.has_uncertainty());
        if report.legacy_adapter.is_some() {
            assert!(audit.exact_adapter_replay);
        }
        assert_eq!(audit.predicate_unknown_cells, 0);
        assert_eq!(audit.predicate_certified_cells as u64, audit.total_frame_cells);
    }
    let faces = extract_exposed_faces(&grid).unwrap();
    assert!(faces.len() <= grid.len() * 6);
    let shell = extract_exposed_faces_with_report(&grid).unwrap();
    assert_eq!(shell.exact_faces, shell.faces.len());
    assert_eq!(shell.has_exact_faces, shell.exact_faces > 0);
    assert_eq!(shell.faces, faces);
    assert!(shell.exact_shell_ready);
    let topology = audit_exact_voxel_surface_topology(&shell.faces);
    assert_eq!(topology.input_faces, shell.exact_faces);
    assert_eq!(
        topology.exact_surface_topology_ready,
        shell.exact_shell_ready
            && topology.input_faces > 0
            && topology.mixed_depth_faces == 0
            && topology.duplicate_faces.is_empty()
            && topology.degenerate_faces.is_empty()
            && topology.boundary_edges.is_empty()
            && topology.nonmanifold_edges.is_empty()
    );
    assert_eq!(topology.face_edge_records, topology.audited_faces * 4);
    let exact_surface_mesh = exact_voxel_surface_triangle_mesh_from_faces(&shell.faces);
    assert_eq!(exact_surface_mesh.report.topology, topology);
    assert_eq!(
        exact_surface_mesh.report.exact_triangle_surface_mesh_ready,
        topology.exact_surface_topology_ready
    );
    if exact_surface_mesh.report.exact_triangle_surface_mesh_ready {
        assert_eq!(exact_surface_mesh.triangles.len(), shell.exact_faces * 2);
        assert_eq!(
            exact_surface_mesh.report.face_triangle_records,
            exact_surface_mesh.triangles.len()
        );
        assert!(exact_surface_mesh.report.exact_face_identity_preserved);
    } else {
        assert!(exact_surface_mesh.triangles.is_empty());
    }
    let mesh_vocabulary = audit_exact_surface_triangle_mesh_vocabulary(&exact_surface_mesh);
    assert_eq!(
        mesh_vocabulary.exact_shared_mesh_vocabulary_ready,
        exact_surface_mesh
            .report
            .exact_triangle_surface_mesh_ready
            && mesh_vocabulary.out_of_bounds_indices.is_empty()
            && mesh_vocabulary.degenerate_triangles.is_empty()
            && mesh_vocabulary.invalid_split_triangles.is_empty()
            && mesh_vocabulary.duplicate_source_splits.is_empty()
            && mesh_vocabulary
                .source_faces_with_wrong_triangle_count
                .is_empty()
            && mesh_vocabulary.boundary_index_edges.is_empty()
            && mesh_vocabulary.nonmanifold_index_edges.is_empty()
    );
    let paged_shell = extract_chunk_paged_exposed_faces_with_report(&paged_grid).unwrap();
    assert_eq!(paged_shell.exact_faces, shell.exact_faces);
    assert_eq!(paged_shell.has_exact_faces, shell.has_exact_faces);
    assert_eq!(paged_shell.tested_cells, grid.len());
    assert_eq!(paged_shell.tested_sides, grid.len() * 6);
    assert_eq!(paged_shell.exact_paged_shell_ready, shell.exact_shell_ready);
    let shell_keys = shell
        .faces
        .iter()
        .map(|face| (face.address, face.side.integer_normal()))
        .collect::<BTreeSet<_>>();
    let paged_shell_keys = paged_shell
        .faces
        .iter()
        .map(|face| (face.address, face.side.integer_normal()))
        .collect::<BTreeSet<_>>();
    assert_eq!(paged_shell_keys, shell_keys);
    let paged_surface_mesh =
        chunk_paged_exact_surface_triangle_mesh_with_report(&paged_grid).unwrap();
    assert_eq!(paged_surface_mesh.shell, paged_shell);
    assert_eq!(paged_surface_mesh.mesh, exact_surface_mesh);
    assert_eq!(
        paged_surface_mesh.exact_paged_triangle_mesh_ready,
        paged_surface_mesh.shell.exact_paged_shell_ready
            && paged_surface_mesh
                .vocabulary
                .exact_shared_mesh_vocabulary_ready
    );
    let paged_patch_plan =
        chunk_paged_greedy_face_patch_plan_with_report(&paged_grid, "fuzz paged preview").unwrap();
    assert_eq!(paged_patch_plan.shell.exact_faces, shell.exact_faces);
    assert_eq!(paged_patch_plan.plan.exact_faces, shell.exact_faces);
    assert_eq!(paged_patch_plan.patch_area_faces, shell.exact_faces);
    assert_eq!(paged_patch_plan.duplicate_patch_faces, 0);
    assert_eq!(paged_patch_plan.missing_shell_faces, 0);
    assert_eq!(paged_patch_plan.extra_patch_faces, 0);
    assert_eq!(
        paged_patch_plan.exact_patch_cover_ready,
        paged_shell.exact_paged_shell_ready
    );
    let empty_shell = extract_exposed_faces_with_report(&SparseVoxelGrid::new(report.frame.clone())).unwrap();
    assert_eq!(empty_shell.exact_faces, 0);
    assert!(!empty_shell.has_exact_faces);
    assert!(!empty_shell.exact_shell_ready);
    let mesh = lossy_quad_mesh_from_faces(&faces, "fuzz preview").unwrap();
    assert!(mesh.report.exact_face_identity_preserved);
    assert!(mesh.report.display_only);
    assert!(!mesh.report.exact_geometry_replay_ready);
    let obj = lossy_obj_from_quad_mesh(&mesh);
    assert!(obj.preview_only);
    assert_eq!(obj.vertex_records, mesh.vertices.len());
    assert_eq!(obj.face_records, mesh.triangles.len());
    greedy_face_patch_plan(&faces, "fuzz preview");
    let binary_snapshot = DeterministicSnapshot::binary_v1(&grid, &VoxelSideTables::default());
    assert!(binary_snapshot.report().exact_snapshot_replay_ready);
    assert_eq!(binary_snapshot.report().serialized_cell_records, grid.len());
    assert!(binary_snapshot.report().has_cell_records);
    let paged_binary_snapshot =
        chunk_paged_binary_snapshot_v1(&paged_grid, &VoxelSideTables::default()).unwrap();
    assert_eq!(paged_binary_snapshot.snapshot, binary_snapshot);
    assert_eq!(paged_binary_snapshot.replayed_cells, grid.len());
    assert_eq!(paged_binary_snapshot.unknown_cells, 0);
    assert_eq!(paged_binary_snapshot.lossy_cells, 0);
    assert_eq!(
        paged_binary_snapshot.exact_paged_snapshot_ready,
        paged_grid.report().exact_chunk_storage_ready
            && binary_snapshot.report().exact_snapshot_replay_ready
    );
    let empty_snapshot = DeterministicSnapshot::binary_v1(
        &SparseVoxelGrid::new(report.frame.clone()),
        &VoxelSideTables::default(),
    );
    assert_eq!(empty_snapshot.report().serialized_cell_records, 0);
    assert!(!empty_snapshot.report().has_cell_records);
    assert!(!empty_snapshot.report().exact_snapshot_replay_ready);
    let empty_paged_snapshot = chunk_paged_binary_snapshot_v1(
        &ChunkPagedSparseGrid::from_sparse_grid(
            &SparseVoxelGrid::new(report.frame.clone()),
            ChunkShape::new(small_depth.min(2)).unwrap(),
        )
        .unwrap(),
        &VoxelSideTables::default(),
    )
    .unwrap();
    assert_eq!(empty_paged_snapshot.snapshot, empty_snapshot);
    assert_eq!(empty_paged_snapshot.replayed_cells, 0);
    assert!(!empty_paged_snapshot.exact_paged_snapshot_ready);
    let rle_snapshot = DeterministicSnapshot::run_length_binary_v1(&grid);
    assert!(rle_snapshot.report().exact_address_encoding);
    assert!(rle_snapshot.report().has_cell_records);
    assert!(!rle_snapshot.report().exact_snapshot_replay_ready);
    let paged_rle_snapshot = chunk_paged_run_length_snapshot_v1(&paged_grid).unwrap();
    assert_eq!(paged_rle_snapshot.snapshot, rle_snapshot);
    assert_eq!(paged_rle_snapshot.replayed_cells, grid.len());
    assert!(paged_rle_snapshot.exact_cell_count_replay);
    assert!(!paged_rle_snapshot.exact_paged_snapshot_ready);
    let diff_report = diff_sparse_grids(&grid, &grid);
    assert!(diff_report.semantic_equivalence_ready);
    assert!(diff_report.frame_matches);
    assert_eq!(diff_report.mismatch_count, 0);
    assert_eq!(diff_report.compared_addresses, grid.len());
    assert_eq!(diff_report.has_compared_addresses, diff_report.compared_addresses > 0);
    let paged_diff_report = diff_chunk_paged_sparse_grids(&paged_grid, &paged_grid);
    assert_eq!(
        paged_diff_report.semantic_equivalence_ready,
        diff_report.semantic_equivalence_ready && paged_grid.report().exact_chunk_storage_ready
    );
    assert_eq!(paged_diff_report.frame_matches, diff_report.frame_matches);
    assert!(paged_diff_report.shape_matches);
    assert_eq!(
        paged_diff_report.compared_addresses,
        diff_report.compared_addresses
    );

    let legacy_depth = (depth_raw % 3) + 1;
    let legacy_cells = 1_u64 << legacy_depth;
    let legacy_xyz = [x % legacy_cells, y % legacy_cells, z % legacy_cells];
    let mut legacy_interner = VoxInterner::<u8>::with_memory_budget(8192);
    let mut legacy_tree = VoxTree::<u8>::new(MaxDepth::new(legacy_depth));
    let legacy_value = depth_raw.wrapping_add(1);
    assert!(legacy_tree.set(
        &mut legacy_interner,
        glam::IVec3::new(
            legacy_xyz[0] as i32,
            legacy_xyz[1] as i32,
            legacy_xyz[2] as i32
        ),
        legacy_value,
    ));
    let (legacy_paged, legacy_report) = materialize_legacy_voxelis_u8_chunk_paged_storage(
        &legacy_tree,
        &legacy_interner,
        GridFrame::builder().depth(legacy_depth).build().unwrap(),
        ChunkShape::new(depth_raw % 3).unwrap(),
    )
    .unwrap();
    assert_eq!(legacy_report.scanned_cells, legacy_report.replayed_cells);
    assert_eq!(legacy_report.materialized_cells, 1);
    assert_eq!(legacy_report.paging_mismatch_cells, 0);
    assert!(legacy_report.exhaustive_chunk_port_ready);
    assert!(!legacy_report.exact_voxelization_ready);
    assert_eq!(
        legacy_paged
            .get(VoxelAddress::new(legacy_depth, legacy_xyz).unwrap())
            .unwrap(),
        VoxelCell::material(MaterialRegionId(u32::from(legacy_value)))
    );
    assert_eq!(paged_diff_report.mismatch_count, diff_report.mismatch_count);
    assert!(paged_diff_report.exact_page_diff_ready);
    let empty_diff = diff_sparse_grids(
        &SparseVoxelGrid::new(report.frame.clone()),
        &SparseVoxelGrid::new(report.frame.clone()),
    );
    assert_eq!(empty_diff.compared_addresses, 0);
    assert!(!empty_diff.has_compared_addresses);
    assert!(!empty_diff.semantic_equivalence_ready);
    let empty_paged_diff = diff_chunk_paged_sparse_grids(
        &ChunkPagedSparseGrid::from_sparse_grid(
            &SparseVoxelGrid::new(report.frame.clone()),
            ChunkShape::new(small_depth.min(2)).unwrap(),
        )
        .unwrap(),
        &ChunkPagedSparseGrid::from_sparse_grid(
            &SparseVoxelGrid::new(report.frame.clone()),
            ChunkShape::new(small_depth.min(2)).unwrap(),
        )
        .unwrap(),
    );
    assert_eq!(empty_paged_diff.compared_addresses, 0);
    assert!(!empty_paged_diff.has_compared_addresses);
    assert!(empty_paged_diff.exact_page_diff_ready);
    assert!(!empty_paged_diff.semantic_equivalence_ready);
    let chunk_summary = ChunkPageSummary::from_addresses(
        ChunkShape::new(small_depth.min(2)).unwrap(),
        grid.iter().map(|(address, _)| *address),
    );
    assert!(chunk_summary.exact_integer_partition);
    assert!(chunk_summary.exact_page_cover_ready);
    assert!(chunk_summary.has_stored_cells);
    assert!(chunk_summary.page_capacity_cells >= chunk_summary.stored_cells);
    let empty_chunk_summary = ChunkPageSummary::from_addresses(
        ChunkShape::new(small_depth.min(2)).unwrap(),
        std::iter::empty::<VoxelAddress>(),
    );
    assert_eq!(empty_chunk_summary.stored_cells, 0);
    assert!(!empty_chunk_summary.has_stored_cells);
    assert!(!empty_chunk_summary.exact_page_cover_ready);
    let swept_report = ProcessGridArtifact::new(
        ProcessGridRole::SweptVolumeCache,
        None,
        vec!["fuzz".into()],
        report.aggregate.clone(),
    )
    .with_swept_volume(SweptVolumeProvenance {
        source: Some(GridSource::new("fuzz-path", 1)),
        expected_source_version: Some(if depth_raw & 2 == 0 { 1 } else { 2 }),
        tool_or_beam: Some("fuzz-tool".into()),
        exact_source_replay_available: depth_raw & 1 == 0,
        broad_phase_only: true,
        quantization_policy: "fuzz conservative cover".into(),
    })
    .swept_volume
    .as_ref()
    .unwrap()
    .report();
    if swept_report.can_stand_in_for_exact_path {
        assert_eq!(swept_report.source_freshness, FreshnessStatus::Current);
        assert!(swept_report.exact_source_replay_available);
        assert!(swept_report.has_tool_or_beam);
        assert!(swept_report.has_quantization_policy);
        assert!(!swept_report.broad_phase_only);
    }
    let source_only_swept_report = SweptVolumeProvenance {
        source: Some(GridSource::new("fuzz-path", 1)),
        expected_source_version: Some(1),
        tool_or_beam: None,
        exact_source_replay_available: true,
        broad_phase_only: false,
        quantization_policy: String::new(),
    }
    .report();
    assert!(!source_only_swept_report.has_tool_or_beam);
    assert!(!source_only_swept_report.has_quantization_policy);
    assert!(!source_only_swept_report.can_stand_in_for_exact_path);
    let candidate_report = VoxelCandidateManifest {
        kind: VoxelCandidateKind::GridResolution,
        freshness: if depth_raw & 1 == 0 {
            FreshnessStatus::Current
        } else {
            FreshnessStatus::Stale
        },
        aggregate_certainty: if depth_raw & 2 == 0 {
            AggregateCertainty::Exact
        } else {
            AggregateCertainty::Unknown
        },
        unknown_count: usize::from(depth_raw & 2),
        lossy_count: usize::from(depth_raw & 4),
        exact_replay_available: depth_raw & 8 == 0,
        exact_evidence_count: usize::from(depth_raw & 16 == 0),
    }
    .report();
    assert_eq!(
        candidate_report.has_exact_evidence,
        candidate_report.exact_evidence_count > 0
    );
    if candidate_report.promotable_as_exact {
        assert!(candidate_report.has_exact_evidence);
        assert!(candidate_report.exact_replay_available);
        assert_eq!(candidate_report.freshness, FreshnessStatus::Current);
        assert_eq!(candidate_report.aggregate_certainty, AggregateCertainty::Exact);
        assert_eq!(candidate_report.unknown_count, 0);
        assert_eq!(candidate_report.lossy_count, 0);
    }
    let coupling_report = VoxelFieldCouplingManifest {
        kind: VoxelFieldCouplingKind::Photochemical,
        freshness: FreshnessStatus::Current,
        aggregate: report.aggregate.clone(),
        residual_replay_available: depth_raw & 1 == 0,
        adapter_error_bound: (depth_raw & 2 != 0).then_some(if depth_raw & 4 == 0 {
            Real::from(1)
        } else {
            Real::from(-1)
        }),
        missing_sample_records: usize::from(depth_raw & 4),
    }
    .report();
    assert_eq!(
        coupling_report.certified_adapter_error_bound_ready,
        coupling_report.has_adapter_error_bound
            && coupling_report.adapter_error_bound_non_negative
    );
    if coupling_report.requires_error_bounded_adapter {
        assert!(coupling_report.certified_adapter_error_bound_ready);
        assert!(!coupling_report.usable_as_exact_residual_evidence);
    }
    let artifact_report = VoxelArtifactManifest {
        id: VoxelArtifactId(format!("fuzz:{depth}")),
        role: VoxelArtifactRole::StorageSnapshot,
        freshness: FreshnessStatus::Current,
        aggregate: report.aggregate.clone(),
        storage_replay: hypervoxel::StorageReplayStatus::Exact,
        missing_side_table_links: usize::from(depth_raw & 1),
        intended_domains: vec![VoxelHandoffDomain::Hyperparts],
    }
    .report();
    assert!(artifact_report.role_supports_exact_indexing);
    assert!(artifact_report.stable_id_ready);
    assert!(artifact_report.intended_domain_ready);
    if artifact_report.indexable_as_exact {
        assert_eq!(artifact_report.missing_side_table_links, 0);
        assert!(artifact_report.stable_id_ready);
        assert!(artifact_report.intended_domain_ready);
        assert!(artifact_report.has_aggregate_evidence);
    }
    let empty_artifact_report = VoxelArtifactManifest {
        id: VoxelArtifactId(format!("fuzz-empty:{depth}")),
        role: VoxelArtifactRole::StorageSnapshot,
        freshness: FreshnessStatus::Current,
        aggregate: VoxelAggregateFacts::from_cells(std::iter::empty::<&VoxelCell>()),
        storage_replay: hypervoxel::StorageReplayStatus::Exact,
        missing_side_table_links: 0,
        intended_domains: vec![VoxelHandoffDomain::Hyperparts],
    }
    .report();
    assert!(!empty_artifact_report.has_aggregate_evidence);
    assert!(!empty_artifact_report.indexable_as_exact);
    let domainless_artifact_report = VoxelArtifactManifest {
        id: VoxelArtifactId(format!("fuzz-domainless:{depth}")),
        role: VoxelArtifactRole::StorageSnapshot,
        freshness: FreshnessStatus::Current,
        aggregate: report.aggregate.clone(),
        storage_replay: hypervoxel::StorageReplayStatus::Exact,
        missing_side_table_links: 0,
        intended_domains: Vec::new(),
    }
    .report();
    assert!(!domainless_artifact_report.intended_domain_ready);
    assert!(!domainless_artifact_report.indexable_as_exact);
    let unnamed_artifact_report = VoxelArtifactManifest {
        id: VoxelArtifactId(" ".into()),
        role: VoxelArtifactRole::StorageSnapshot,
        freshness: FreshnessStatus::Current,
        aggregate: report.aggregate.clone(),
        storage_replay: hypervoxel::StorageReplayStatus::Exact,
        missing_side_table_links: 0,
        intended_domains: vec![VoxelHandoffDomain::Hyperparts],
    }
    .report();
    assert!(!unnamed_artifact_report.stable_id_ready);
    assert!(!unnamed_artifact_report.indexable_as_exact);
    VoxelArtifactManifest {
        id: VoxelArtifactId(format!("fuzz-preview:{depth}")),
        role: VoxelArtifactRole::PreviewArtifact,
        freshness: FreshnessStatus::Current,
        aggregate: report.aggregate.clone(),
        storage_replay: hypervoxel::StorageReplayStatus::Exact,
        missing_side_table_links: 0,
        intended_domains: vec![VoxelHandoffDomain::Hyperparts],
    }
    .report();
    let policy = VoxelizationPolicy {
        quantization: match depth_raw & 3 {
            0 => QuantizationPolicy::UnsignedDistanceSampling,
            1 => QuantizationPolicy::SignedDistanceSampling,
            2 => QuantizationPolicy::MaterialRegionRasterization,
            _ => QuantizationPolicy::ProcessExposureGrid,
        },
        boundary: hypervoxel::BoundaryPolicy::KeepBoundary,
    };
    assert!(policy.is_exact_semantic_role());
    let io_report = ImageStackManifest {
        container: ImageStackContainer::ZstdQoi,
        slices: 1,
        channels: 1,
        bit_depth: 8,
        channel_mappings: vec![VoxelChannelMapping::OccupancyMask],
        metadata: VoxelIoMetadata {
            dimensions: Some([1, 1, 1]),
            axis_order: Some([0, 1, 2]),
            has_explicit_origin: true,
            has_explicit_spacing: true,
            units: Some(LengthUnit::Unitless),
            has_payload_mapping: true,
            has_label_mapping: false,
            has_missing_slice_policy: true,
            has_duplicate_slice_policy: true,
            slice_naming: if depth_raw & 1 == 0 {
                VoxelSliceNaming::ExplicitIndex
            } else {
                VoxelSliceNaming::Lexicographic
            },
            slice_ordering: if depth_raw & 2 == 0 {
                VoxelSliceOrdering::LowToHigh
            } else {
                VoxelSliceOrdering::HighToLow
            },
            index_convention: VoxelIndexConvention::CellCenter,
            compression: VoxelIoCompression::Zstd,
        },
        source: Some(GridSource::new("fuzz-stack", u64::from(depth_raw))),
        expected_source: Some(GridSource::new(
            "fuzz-stack",
            u64::from(depth_raw ^ (depth_raw & 1)),
        )),
        required_side_table_links: usize::from(depth_raw & 3),
        supplied_side_table_links: usize::from(depth_raw & 1),
    }
    .report();
    assert_eq!(io_report.declared_channels, Some(1));
    assert_eq!(io_report.mapped_channels, 1);
    assert_eq!(io_report.unmapped_channels, 0);
    assert_eq!(io_report.extra_channel_mappings, 0);
    assert_eq!(io_report.bit_depth, Some(8));
    assert_eq!(io_report.declared_sample_slots, Some(1));
    assert!(io_report.has_sample_evidence);
    assert_eq!(io_report.invalid_dimension_axes, 0);
    assert!(io_report.positive_dimensions_ready);
    assert!(io_report.has_missing_slice_policy);
    assert!(io_report.has_duplicate_slice_policy);
    assert_eq!(
        io_report.exact_sample_replay_ready,
        io_report.freshness == FreshnessStatus::Current
    );
    if io_report.exact_sample_replay_ready {
        assert!(io_report.certified_sample_replay_ready);
        assert!(io_report.adapter.exact_replay);
    }
    if io_report.freshness == FreshnessStatus::Current {
        assert_eq!(io_report.source_freshness, FreshnessStatus::Current);
    }
    let zero_dimension_io_report = ImageStackManifest {
        container: ImageStackContainer::ZippedPng,
        slices: 1,
        channels: 1,
        bit_depth: 8,
        channel_mappings: vec![VoxelChannelMapping::OccupancyMask],
        metadata: VoxelIoMetadata {
            dimensions: Some([1, u64::from(depth_raw & 1), 1]),
            axis_order: Some([0, 1, 2]),
            has_explicit_origin: true,
            has_explicit_spacing: true,
            units: Some(LengthUnit::Unitless),
            has_payload_mapping: true,
            has_label_mapping: true,
            has_missing_slice_policy: true,
            has_duplicate_slice_policy: true,
            slice_naming: VoxelSliceNaming::ExplicitIndex,
            slice_ordering: VoxelSliceOrdering::LowToHigh,
            index_convention: VoxelIndexConvention::CellCenter,
            compression: VoxelIoCompression::Zip,
        },
        source: Some(GridSource::new("fuzz-zero-dimension", 1)),
        expected_source: Some(GridSource::new("fuzz-zero-dimension", 1)),
        required_side_table_links: 0,
        supplied_side_table_links: 0,
    }
    .report();
    assert_eq!(
        zero_dimension_io_report.positive_dimensions_ready,
        zero_dimension_io_report.invalid_dimension_axes == 0
    );
    assert_eq!(
        zero_dimension_io_report.has_sample_evidence,
        zero_dimension_io_report
            .declared_sample_slots
            .is_some_and(|slots| slots > 0)
    );
    if zero_dimension_io_report.invalid_dimension_axes > 0 {
        assert!(!zero_dimension_io_report.has_sample_evidence);
        assert!(!zero_dimension_io_report.certified_sample_replay_ready);
        assert!(!zero_dimension_io_report.exact_sample_replay_ready);
    }
    let empty_sample_io_report = ImageStackManifest {
        container: ImageStackContainer::ZippedPng,
        slices: 0,
        channels: 1,
        bit_depth: 8,
        channel_mappings: vec![VoxelChannelMapping::OccupancyMask],
        metadata: VoxelIoMetadata {
            dimensions: Some([1, 1, 1]),
            axis_order: Some([0, 1, 2]),
            has_explicit_origin: true,
            has_explicit_spacing: true,
            units: Some(LengthUnit::Unitless),
            has_payload_mapping: true,
            has_label_mapping: true,
            has_missing_slice_policy: true,
            has_duplicate_slice_policy: true,
            slice_naming: VoxelSliceNaming::ExplicitIndex,
            slice_ordering: VoxelSliceOrdering::LowToHigh,
            index_convention: VoxelIndexConvention::CellCenter,
            compression: VoxelIoCompression::Zip,
        },
        source: Some(GridSource::new("fuzz-empty-sample", 1)),
        expected_source: Some(GridSource::new("fuzz-empty-sample", 1)),
        required_side_table_links: 0,
        supplied_side_table_links: 0,
    }
    .report();
    assert_eq!(empty_sample_io_report.declared_sample_slots, Some(0));
    assert!(!empty_sample_io_report.has_sample_evidence);
    assert!(!empty_sample_io_report.certified_sample_replay_ready);
    assert!(!empty_sample_io_report.exact_sample_replay_ready);
    let compression_report = CompressedStorageManifest {
        kind: CompressedStorageKind::RunLengthSnapshot,
        stored_cells: prepared_storage_len_for_fuzz(small_depth),
        physical_records: 1,
        chunk_shape: ChunkShape::new(small_depth.min(2)).ok(),
        preserves_aggregate_facts: true,
        preserves_payload_ids: true,
        preserves_side_table_links: false,
    }
    .report();
    assert!(compression_report.physical_layout_ready);
    assert!(compression_report.has_stored_cells);
    assert!(!compression_report.exact_storage_replay_ready);
    assert!(compression_report.certified_aggregate_replay_ready);
    let empty_compression_report = CompressedStorageManifest {
        kind: CompressedStorageKind::SparseMap,
        stored_cells: 0,
        physical_records: 0,
        chunk_shape: None,
        preserves_aggregate_facts: true,
        preserves_payload_ids: true,
        preserves_side_table_links: true,
    }
    .report();
    assert!(empty_compression_report.physical_layout_ready);
    assert!(!empty_compression_report.has_stored_cells);
    assert!(!empty_compression_report.exact_storage_replay_ready);
    assert!(!empty_compression_report.certified_aggregate_replay_ready);
    let impossible_compression_report = CompressedStorageManifest {
        kind: CompressedStorageKind::RunLengthSnapshot,
        stored_cells: 1,
        physical_records: 0,
        chunk_shape: ChunkShape::new(1).ok(),
        preserves_aggregate_facts: true,
        preserves_payload_ids: true,
        preserves_side_table_links: true,
    }
    .report();
    assert!(!impossible_compression_report.physical_layout_ready);
    assert!(!impossible_compression_report.exact_storage_replay_ready);
    assert!(!impossible_compression_report.certified_aggregate_replay_ready);
    let memory_report = VoxelMemoryBudgetManifest {
        kind: CompressedStorageKind::SparseVoxelDag,
        estimated_bytes: prepared_storage_len_for_fuzz(small_depth) * 64,
        budget_bytes: usize::from(depth_raw) + 1,
        preserves_exact_semantics_when_over_budget: depth_raw & 1 == 0,
    }
    .report();
    assert_eq!(
        memory_report.exact_memory_budget_ready,
        memory_report.has_memory_evidence && memory_report.exact_semantics_preserved
    );
    let empty_memory_report = VoxelMemoryBudgetManifest {
        kind: CompressedStorageKind::SparseMap,
        estimated_bytes: 0,
        budget_bytes: usize::from(depth_raw) + 1,
        preserves_exact_semantics_when_over_budget: true,
    }
    .report();
    assert!(!empty_memory_report.has_memory_evidence);
    assert!(!empty_memory_report.exact_memory_budget_ready);
    let preview_report = PreviewExportManifest {
        format: PreviewExportFormat::ContinuousSdfPreview,
        exact_input_primitives: 1,
        exported_primitives: 1,
        scalar_policy: PreviewScalarPolicy::PrimitiveFloat,
        preserves_grid_topology: false,
        has_explicit_labels: false,
    }
    .report();
    assert!(preview_report.has_input_primitives);
    assert!(preview_report.has_exported_primitives);
    assert!(!preview_report.exact_grid_topology_replay);
    assert!(!preview_report.source_geometry_replay);
    let empty_preview_report = PreviewExportManifest {
        format: PreviewExportFormat::Vtm,
        exact_input_primitives: 0,
        exported_primitives: 0,
        scalar_policy: PreviewScalarPolicy::ExactString,
        preserves_grid_topology: true,
        has_explicit_labels: true,
    }
    .report();
    assert!(!empty_preview_report.has_input_primitives);
    assert!(!empty_preview_report.has_exported_primitives);
    assert!(!empty_preview_report.exact_grid_topology_replay);
    let adapter_report = AdapterNumericContract::primitive_float(
        LegacyAdapterStatus::lossy(LegacyAdapterKind::PreviewRenderer, "fuzz display epsilon"),
        Some(Real::from(1)),
        (depth_raw & 1 == 0).then_some(Real::from(0)),
        (depth_raw & 2 == 0).then_some(Real::from(1)),
        if depth_raw & 4 == 0 {
            AdapterToleranceStatus::Explicit
        } else {
            AdapterToleranceStatus::LossyImplicit
        },
    )
    .report();
    if adapter_report.tolerance_status == AdapterToleranceStatus::Explicit {
        assert_eq!(
            adapter_report.tolerance_declaration_complete,
            adapter_report.has_explicit_error_bound
                && adapter_report.epsilon_is_non_negative
                && adapter_report.tolerance_is_non_negative
        );
    } else {
        assert!(!adapter_report.tolerance_declaration_complete);
    }
    assert!(!adapter_report.can_drive_exact_topology);
    let blank_policy_adapter = AdapterNumericContract::exact(
        LegacyAdapterStatus::exact(LegacyAdapterKind::ImportExport, " "),
        Real::from(1),
    )
    .report();
    assert!(blank_policy_adapter.adapter.exact_replay);
    assert!(!blank_policy_adapter.adapter_policy_ready);
    assert!(!blank_policy_adapter.can_drive_exact_topology);
    let handoff_report = VoxelHandoffManifest {
        domain: VoxelHandoffDomain::Hypercircuit,
        source: Some(GridSource::new("fuzz", 1)),
        expected_source: Some(GridSource::new("fuzz", 2)),
        required_side_table_links: 1,
        supplied_side_table_links: 0,
        aggregate: report.aggregate.clone(),
    }
    .report();
    assert_eq!(
        handoff_report.has_aggregate_evidence,
        handoff_report.aggregate.child_count > 0
    );
    assert!(!handoff_report.exact_handoff_ready);
    let trace_report = VoxelTraceManifest {
        operation: "fuzz".into(),
        dimensions: vec![
            VoxelTraceDimension::GridFrameConstruction,
            VoxelTraceDimension::ExactVoxelizationPredicateBatch,
            VoxelTraceDimension::ExactVoxelizationPredicateBatch,
            VoxelTraceDimension::DomainHandoffReport,
        ],
        exact_predicate_count: prepared_storage_len_for_fuzz(small_depth),
        lossy_adapter_count: usize::from(depth_raw & 1),
        unknown_count: usize::from(depth_raw & 2),
    }
    .report();
    assert_eq!(
        trace_report.exact_trace_evidence_ready,
        trace_report.has_operation_dimension
            && trace_report.has_exact_evidence
            && trace_report.lossy_adapter_count == 0
            && trace_report.unknown_count == 0
    );
    if trace_report.exact_trace_evidence_ready {
        assert!(!trace_report.has_lossy_adapter_work);
        assert!(!trace_report.has_unknowns);
    }
    let vacuous_trace = VoxelTraceManifest {
        operation: "vacuous fuzz trace".into(),
        dimensions: Vec::new(),
        exact_predicate_count: 0,
        lossy_adapter_count: 0,
        unknown_count: 0,
    }
    .report();
    assert!(!vacuous_trace.has_operation_dimension);
    assert!(!vacuous_trace.has_exact_evidence);
    assert!(!vacuous_trace.exact_trace_evidence_ready);

    let prepared = PreparedVoxelGrid::new(report.frame.clone(), grid, report.aggregate.clone());
    let prepared_query = prepared.prepared_query_report(depth_raw & 1 == 0).unwrap();
    assert_eq!(prepared_query.freshness, FreshnessStatus::Unknown);
    assert!(!prepared_query.report_frame_matches);
    assert_eq!(
        prepared_query.has_query_evidence,
        prepared_query.non_empty_cells > 0
    );
    assert!(!prepared_query.exact_query_evidence_ready);
    let broad_phase_report = prepared
        .query_aabb_broad_phase(&ExactAabb3 {
            min: [Real::from(0), Real::from(0), Real::from(0)],
            max: [
                Real::from(cells.min(2)),
                Real::from(cells.min(2)),
                Real::from(cells.min(2)),
            ],
        })
        .unwrap();
    assert_eq!(
        broad_phase_report.tested_cells,
        broad_phase_report.candidates.len()
            + broad_phase_report.rejected_addresses.len()
            + broad_phase_report.unknown_addresses.len()
    );
    assert_eq!(
        broad_phase_report.has_tested_cells,
        broad_phase_report.tested_cells > 0
    );
    if broad_phase_report.certified_broad_phase_ready {
        assert!(broad_phase_report.is_fully_decided());
        assert!(broad_phase_report.has_tested_cells);
        assert!(broad_phase_report.unknown_addresses.is_empty());
    }
    let empty_prepared = PreparedVoxelGrid::new(
        report.frame.clone(),
        SparseVoxelGrid::new(report.frame.clone()),
        VoxelAggregateFacts::from_cells(std::iter::empty::<&VoxelCell>()),
    );
    let empty_broad_phase = empty_prepared
        .query_aabb_broad_phase(&ExactAabb3 {
            min: [Real::from(0), Real::from(0), Real::from(0)],
            max: [Real::from(1), Real::from(1), Real::from(1)],
        })
        .unwrap();
    assert_eq!(empty_broad_phase.tested_cells, 0);
    assert!(!empty_broad_phase.has_tested_cells);
    assert!(!empty_broad_phase.certified_broad_phase_ready);
    let occupancy_query = prepared.query_occupancy(address).unwrap();
    assert_eq!(
        occupancy_query.exact_cell_evidence_ready,
        !matches!(
            occupancy_query.cell.occupancy,
            hypervoxel::OccupancyState::Unknown | hypervoxel::OccupancyState::LossyAdapterValue
        )
    );
    assert!(prepared.query_neighbors6(address).exact_neighbors_ready);
    let band = prepared.query_manhattan_band(address, 1).unwrap();
    assert_eq!(band.has_reached_cells, !band.distances.is_empty());
    if band.exact_distance_band_ready {
        assert!(band.has_reached_cells);
        for reached in band.distances.keys() {
            assert!(!matches!(
                prepared.query_occupancy(*reached).unwrap().cell.occupancy,
                hypervoxel::OccupancyState::Unknown
                    | hypervoxel::OccupancyState::LossyAdapterValue
            ));
        }
    }
    let spatial_report = VoxelSpatialAggregateFacts::from_grid(&prepared.storage, None).unwrap();
    assert_eq!(
        spatial_report.has_spatial_evidence,
        spatial_report.stored_cells > 0
    );
    assert_eq!(
        spatial_report.exact_bounds_ready,
        spatial_report.has_spatial_evidence && spatial_report.exact_bounds.is_some()
    );
    assert_eq!(spatial_report.source_replay_ready, false);
    assert_eq!(spatial_report.freshness, FreshnessStatus::Unknown);
    assert_eq!(spatial_report.exact_bounds.is_some(), spatial_report.stored_cells > 0);
    let support_report = classify_support_mask(
        &prepared.storage,
        &prepared.storage,
        SupportDirection::new((depth_raw as usize) % 3, if depth_raw & 1 == 0 { -1 } else { 1 })
            .unwrap(),
    )
    .unwrap();
    if support_report.exact_support_mask_ready {
        assert!(support_report.has_checked_cells);
        assert_eq!(support_report.unsupported_cells, 0);
        assert_eq!(support_report.unknown_cells, 0);
        assert_eq!(support_report.lossy_cells, 0);
    }
    let paged_prepared_storage = ChunkPagedSparseGrid::from_sparse_grid(
        &prepared.storage,
        ChunkShape::new(small_depth.min(2)).unwrap(),
    )
    .unwrap();
    let paged_support_report = classify_chunk_paged_support_mask(
        &paged_prepared_storage,
        &paged_prepared_storage,
        SupportDirection::new((depth_raw as usize) % 3, if depth_raw & 1 == 0 { -1 } else { 1 })
            .unwrap(),
    )
    .unwrap();
    assert_eq!(paged_support_report.support, support_report);
    assert_eq!(
        paged_support_report.exact_paged_support_ready,
        support_report.exact_support_mask_ready
            && paged_prepared_storage.report().exact_chunk_storage_ready
    );
    assert_eq!(
        paged_support_report.target_cells,
        support_report.checked_cells
    );
    let empty_support_report = classify_support_mask(
        &SparseVoxelGrid::new(report.frame.clone()),
        &prepared.storage,
        SupportDirection::new((depth_raw as usize) % 3, 1).unwrap(),
    )
    .unwrap();
    assert_eq!(empty_support_report.checked_cells, 0);
    assert!(!empty_support_report.has_checked_cells);
    assert!(!empty_support_report.exact_support_mask_ready);
    let empty_paged_support_report = classify_chunk_paged_support_mask(
        &ChunkPagedSparseGrid::from_sparse_grid(
            &SparseVoxelGrid::new(report.frame.clone()),
            ChunkShape::new(small_depth.min(2)).unwrap(),
        )
        .unwrap(),
        &paged_prepared_storage,
        SupportDirection::new((depth_raw as usize) % 3, 1).unwrap(),
    )
    .unwrap();
    assert_eq!(empty_paged_support_report.support, empty_support_report);
    assert_eq!(empty_paged_support_report.target_pages, 0);
    assert!(!empty_paged_support_report.exact_paged_support_ready);
    let neighbors = voxel_neighbors6(VoxelAddress::new(small_depth, [0, 0, 0]).unwrap());
    assert!(neighbors.len() <= 3);
    prepared
        .query_manhattan_band(VoxelAddress::new(small_depth, [0, 0, 0]).unwrap(), 2)
        .unwrap();
    let lod_report = select_lod_cells(&prepared.storage, 0).unwrap();
    assert_eq!(lod_report.selected_cells, lod_report.cells.len());
    assert_eq!(lod_report.has_selected_cells, lod_report.selected_cells > 0);
    assert_eq!(
        lod_report.selected_cells,
        lod_report.exact_aggregate_cells
            + lod_report.certified_aggregate_cells
            + lod_report.unknown_aggregate_cells
            + lod_report.lossy_aggregate_cells
    );
    if lod_report.certified_lod_aggregate_ready {
        assert!(lod_report.has_selected_cells);
        assert_eq!(lod_report.unknown_aggregate_cells, 0);
        assert_eq!(lod_report.lossy_aggregate_cells, 0);
    }
    let empty_lod_report = select_lod_cells(&SparseVoxelGrid::new(report.frame.clone()), 0).unwrap();
    assert_eq!(empty_lod_report.selected_cells, 0);
    assert!(!empty_lod_report.has_selected_cells);
    assert!(!empty_lod_report.certified_lod_aggregate_ready);
    let distance_preview = sample_manhattan_distance_field(
        &prepared.storage,
        QueryRegion {
            min: [0, 0, 0],
            max: [0, 0, 0],
            depth: small_depth,
        },
    )
    .unwrap();
    assert!(distance_preview.has_distance_source);
    assert!(distance_preview.source_cells > 0);
    assert!(distance_preview.exact_address_distance_ready);
    let signed_distance_preview = sample_signed_manhattan_distance_field(
        &prepared.storage,
        QueryRegion {
            min: [0, 0, 0],
            max: [0, 0, 0],
            depth: small_depth,
        },
    )
    .unwrap();
    assert_eq!(
        signed_distance_preview.exact_address_distance_ready,
        distance_preview.exact_address_distance_ready
    );
    assert_eq!(
        signed_distance_preview.has_distance_source,
        distance_preview.has_distance_source
    );
    assert!(!signed_distance_preview.continuous_sdf_ready);
    let empty_distance_preview = sample_manhattan_distance_field(
        &SparseVoxelGrid::new(report.frame.clone()),
        QueryRegion {
            min: [0, 0, 0],
            max: [0, 0, 0],
            depth: small_depth,
        },
    )
    .unwrap();
    assert_eq!(empty_distance_preview.source_cells, 0);
    assert!(!empty_distance_preview.has_distance_source);
    assert!(!empty_distance_preview.exact_address_distance_ready);
    let ray_trace = trace_address_ray(AddressRay {
        start: VoxelAddress::new(small_depth, [0, 0, 0]).unwrap(),
        axis: 0,
        direction: 1,
        max_steps: 2,
    })
    .unwrap();
    assert!(ray_trace.exact_address_trace_ready);
    assert!(ray_trace.stopped_at_boundary || ray_trace.stopped_at_step_limit);

    let mut edited = hypervoxel::SparseVoxelGrid::new(GridFrame::builder().depth(small_depth).build().unwrap());
    let mut batch = VoxelEditBatch::new();
    let sample_address = VoxelAddress::new(small_depth, [0, 0, 0]).unwrap();
    let material_address =
        VoxelAddress::new(small_depth, [0, 0, u64::from(small_depth > 0)]).unwrap();
    batch.push(sample_address, VoxelCell::field_sample(FieldSampleId(7)));
    batch.push(material_address, VoxelCell::material(MaterialRegionId(8)));
    let batch_report = batch.apply_with_report(&mut edited).unwrap();
    assert_eq!(batch_report.applied_edits, 2);
    assert!(batch_report.has_applied_edits);
    assert!(batch_report.exact_batch_replay_ready);
    assert_eq!(batch_report.non_exact_current_cells, 0);
    assert!(batch_report
        .edits
        .iter()
        .all(|edit| edit.exact_edit_replay_ready));
    assert_eq!(batch_report.stored_explicit_cells, 2);
    assert_eq!(batch_report.removed_explicit_cells, 0);
    let mut lossy_batch = VoxelEditBatch::new();
    lossy_batch.push(sample_address, VoxelCell::lossy_adapter_value(13));
    let lossy_batch_report = lossy_batch.apply_with_report(&mut edited).unwrap();
    assert_eq!(lossy_batch_report.non_exact_current_cells, 1);
    assert!(!lossy_batch_report.edits[0].exact_edit_replay_ready);
    assert!(!lossy_batch_report.exact_batch_replay_ready);
    let empty_batch_report = VoxelEditBatch::new()
        .apply_with_report(&mut edited)
        .unwrap();
    assert_eq!(empty_batch_report.applied_edits, 0);
    assert!(!empty_batch_report.has_applied_edits);
    assert!(!empty_batch_report.exact_batch_replay_ready);

    let mut side_tables = VoxelSideTables::default();
    side_tables.insert_field_sample(
        FieldSampleId(7),
        FieldSampleRecord {
            label: "fuzz".into(),
            lower: Some(Real::from(0)),
            upper: Some(Real::from(cells.min(8))),
            provenance: "fuzz".into(),
        },
    );
    side_tables.insert_material(
        MaterialRegionId(8),
        MaterialRegionRecord {
            label: "fuzz-material".into(),
            density: (depth_raw & 1 == 0).then_some(Real::from(1)),
            provenance: if depth_raw & 2 == 0 {
                "fuzz".into()
            } else {
                String::new()
            },
        },
    );
    side_tables.insert_process_state(
        ProcessStateId(9),
        ProcessStateRecord {
            label: if depth_raw & 4 == 0 {
                "fuzz-process".into()
            } else {
                String::new()
            },
            provenance: if depth_raw & 8 == 0 {
                "fuzz".into()
            } else {
                String::new()
            },
        },
    );
    let field_facts = FieldAggregateFacts::from_grid(&edited, &side_tables).unwrap();
    if field_facts.certified_field_bounds_ready {
        assert!(field_facts.has_field_samples);
        assert_eq!(field_facts.missing_records, 0);
        assert_eq!(field_facts.missing_bounds, 0);
    }
    let empty_field_facts =
        FieldAggregateFacts::from_grid(&SparseVoxelGrid::new(report.frame.clone()), &side_tables)
            .unwrap();
    assert_eq!(empty_field_facts.sample_cell_count, 0);
    assert!(!empty_field_facts.has_field_samples);
    assert!(!empty_field_facts.certified_field_bounds_ready);
    let field_query = query_field_samples(&edited, &side_tables);
    let paged_edited = ChunkPagedSparseGrid::from_sparse_grid(
        &edited,
        ChunkShape::new(small_depth.min(2)).unwrap(),
    )
    .unwrap();
    let paged_field = audit_chunk_paged_field_samples(&paged_edited, &side_tables).unwrap();
    assert_eq!(paged_field.query, field_query);
    assert_eq!(paged_field.aggregate, field_facts);
    assert_eq!(
        paged_field.exact_paged_field_audit_ready,
        paged_edited.report().exact_chunk_storage_ready
            && paged_field.tested_cells > 0
            && paged_field.unknown_cells == 0
            && paged_field.lossy_cells == 0
            && field_query.is_fully_resolved()
            && field_facts.certified_field_bounds_ready
    );
    let empty_field_query =
        query_field_samples(&SparseVoxelGrid::new(report.frame.clone()), &side_tables);
    let empty_paged_field = audit_chunk_paged_field_samples(
        &ChunkPagedSparseGrid::from_sparse_grid(
            &SparseVoxelGrid::new(report.frame.clone()),
            ChunkShape::new(small_depth.min(2)).unwrap(),
        )
        .unwrap(),
        &side_tables,
    )
    .unwrap();
    assert_eq!(empty_paged_field.query, empty_field_query);
    assert_eq!(empty_paged_field.aggregate, empty_field_facts);
    assert!(!empty_paged_field.exact_paged_field_audit_ready);
    let mut process_grid =
        SparseVoxelGrid::new(GridFrame::builder().depth(small_depth).build().unwrap());
    process_grid
        .set(
            VoxelAddress::new(small_depth, [0, 0, 0]).unwrap(),
            VoxelCell::process_state(ProcessStateId(9)),
        )
        .unwrap();
    let paged_process_grid = ChunkPagedSparseGrid::from_sparse_grid(
        &process_grid,
        ChunkShape::new(small_depth.min(2)).unwrap(),
    )
    .unwrap();
    let process_audit = audit_chunk_paged_process_states(&paged_process_grid, &side_tables);
    assert_eq!(process_audit.process_payload_cells, 1);
    assert_eq!(process_audit.non_process_payload_cells, 0);
    assert_eq!(process_audit.has_process_states, true);
    assert_eq!(
        process_audit.exact_paged_process_audit_ready,
        paged_process_grid.report().exact_chunk_storage_ready
            && process_audit.tested_cells > 0
            && process_audit.unknown_cells == 0
            && process_audit.lossy_cells == 0
            && process_audit.is_complete()
    );
    let empty_process_audit = audit_chunk_paged_process_states(
        &ChunkPagedSparseGrid::from_sparse_grid(
            &SparseVoxelGrid::new(report.frame.clone()),
            ChunkShape::new(small_depth.min(2)).unwrap(),
        )
        .unwrap(),
        &side_tables,
    );
    assert_eq!(empty_process_audit.tested_cells, 0);
    assert!(!empty_process_audit.has_process_states);
    assert!(!empty_process_audit.exact_paged_process_audit_ready);
    let vector_envelope = CertifiedVectorInterval {
        components: vec![CertifiedFieldInterval {
            lower: Real::from(0),
            upper: Real::from(cells.min(8)),
        }],
    };
    let envelope_facts = FieldEnvelopeFacts::from_envelopes(
        (depth_raw & 1 == 0).then_some(&vector_envelope),
        std::iter::empty(),
    )
    .unwrap();
    if envelope_facts.certified_envelope_ready {
        assert!(envelope_facts.has_envelopes);
        assert!(envelope_facts.envelope_count > 0);
        assert_eq!(envelope_facts.incompatible_shapes, 0);
    }
    let empty_envelope_facts = FieldEnvelopeFacts::from_envelopes(
        std::iter::empty::<&CertifiedVectorInterval>(),
        std::iter::empty(),
    )
    .unwrap();
    assert_eq!(empty_envelope_facts.envelope_count, 0);
    assert!(!empty_envelope_facts.has_envelopes);
    assert!(!empty_envelope_facts.certified_envelope_ready);
    query_field_samples(&edited, &side_tables);
    let material_query = query_material_regions(&edited, &side_tables);
    let material_metadata = report_material_region_metadata(&material_query, &side_tables);
    assert_eq!(
        material_metadata.has_material_regions,
        material_metadata.referenced_regions > 0
    );
    if material_metadata.is_complete() {
        assert!(material_metadata.has_material_regions);
    }
    let paged_material = audit_chunk_paged_material_regions(&paged_edited, &side_tables);
    assert_eq!(paged_material.query, material_query);
    assert_eq!(paged_material.metadata, material_metadata);
    assert_eq!(
        paged_material.exact_paged_material_audit_ready,
        paged_edited.report().exact_chunk_storage_ready
            && paged_material.tested_cells > 0
            && paged_material.unknown_cells == 0
            && paged_material.lossy_cells == 0
            && material_query.is_fully_resolved()
            && material_metadata.is_complete()
    );
    let handoff_source = GridSource::new("fuzz:paged-handoff", u64::from(depth_raw) + 1);
    let paged_handoff = certify_chunk_paged_handoff(
        &paged_edited,
        &side_tables,
        VoxelHandoffDomain::Hyperphysics,
        Some(handoff_source.clone()),
        Some(handoff_source.clone()),
    )
    .unwrap();
    assert_eq!(paged_handoff.freshness, FreshnessStatus::Current);
    assert!(
        paged_handoff.complete_side_table_links <= paged_handoff.supplied_side_table_links
    );
    assert!(
        paged_handoff.supplied_side_table_links <= paged_handoff.required_side_table_links
    );
    assert_eq!(
        paged_handoff.side_table_evidence_ready,
        paged_handoff.required_side_table_links == paged_handoff.complete_side_table_links
    );
    assert_eq!(
        paged_handoff.exact_paged_handoff_ready,
        paged_edited.report().exact_chunk_storage_ready
            && paged_handoff.snapshot.exact_paged_snapshot_ready
            && paged_handoff.side_table_evidence_ready
            && paged_handoff.domain_report.exact_handoff_ready
    );
    let stale_paged_handoff = certify_chunk_paged_handoff(
        &paged_edited,
        &side_tables,
        VoxelHandoffDomain::Hyperphysics,
        Some(GridSource::new("fuzz:paged-handoff", u64::from(depth_raw))),
        Some(handoff_source),
    )
    .unwrap();
    assert_eq!(stale_paged_handoff.freshness, FreshnessStatus::Stale);
    assert!(!stale_paged_handoff.domain_report.exact_handoff_ready);
    assert!(!stale_paged_handoff.exact_paged_handoff_ready);
    let color_report = lookup_material_display_colors(&material_query, &MaterialDisplayPalette::default());
    assert_eq!(
        color_report.complete_display_palette_ready,
        color_report.has_material_regions && color_report.missing_colors.is_empty()
    );
    let empty_material_query =
        query_material_regions(&SparseVoxelGrid::new(report.frame.clone()), &side_tables);
    assert!(!empty_material_query.has_references());
    assert!(!empty_material_query.is_fully_resolved());
    let empty_material_metadata =
        report_material_region_metadata(&empty_material_query, &side_tables);
    assert!(!empty_material_metadata.has_material_regions);
    assert!(!empty_material_metadata.is_complete());
    let empty_paged_material = audit_chunk_paged_material_regions(
        &ChunkPagedSparseGrid::from_sparse_grid(
            &SparseVoxelGrid::new(report.frame.clone()),
            ChunkShape::new(small_depth.min(2)).unwrap(),
        )
        .unwrap(),
        &side_tables,
    );
    assert_eq!(empty_paged_material.query, empty_material_query);
    assert_eq!(empty_paged_material.metadata, empty_material_metadata);
    assert!(!empty_paged_material.exact_paged_material_audit_ready);
    let empty_color_report =
        lookup_material_display_colors(&empty_material_query, &MaterialDisplayPalette::default());
    assert!(!empty_color_report.has_material_regions);
    assert!(!empty_color_report.complete_display_palette_ready);
    let sweep = sweep_address_segment(&prepared, sample_address, sample_address).unwrap();
    assert!(sweep.trace.exact_address_trace_ready);
    assert!(sweep.trace.reached_end);
    assert!(sweep.exact_sweep_samples_ready);
    let path = sample_address.child_path();
    assert_eq!(VoxelAddress::from_child_path(&path).unwrap(), sample_address);
    assert_eq!(
        VoxelAddress::from_morton_code(sample_address.depth, sample_address.morton_code()).unwrap(),
        sample_address
    );
    let transform = AxisPermutationTransform::new(
        [
            SignedAxis::new(1, 1).unwrap(),
            SignedAxis::new(0, -1).unwrap(),
            SignedAxis::new(2, 1).unwrap(),
        ],
        [Real::from(0), Real::from(cells as i32), Real::from(0)],
    )
    .unwrap();
    let bounds = sample_address.bounds(&prepared.frame).unwrap();
    transform.map_bounds(&bounds).unwrap();
    ExactAffineTransform::identity().map_bounds(&bounds).unwrap();
});

fn prepared_storage_len_for_fuzz(depth: u8) -> usize {
    usize::from(depth) + 1
}
