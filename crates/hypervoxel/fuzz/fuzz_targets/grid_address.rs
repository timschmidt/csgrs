#![no_main]

use hyperreal::Real;
use hypervoxel::{
    AdapterNumericContract, AdapterToleranceStatus, AggregateCertainty,
    AddressRay, AxisPermutationTransform, ChunkPageSummary, ChunkShape, CompressedStorageKind,
    CompressedStorageManifest, DeterministicSnapshot, ExactAffineTransform, ExactBox,
    ExactConvexHalfSpaceSet, ExactHalfSpace, FieldAggregateFacts, FieldSampleId,
    FieldSampleRecord, FreshnessStatus, GridAabbHandoff, GridBasis, GridCoordinateSystem,
    GridFrame, GridFrameManifest, GridHandedness, GridSource, ImageStackContainer,
    ImageStackManifest, LegacyAdapterKind, LegacyAdapterStatus, LengthUnit,
    MaterialDisplayPalette, MaterialRegionId, PreparedSparseVoxelGridExt, PreparedVoxelGrid,
    PreviewExportFormat, PreviewExportManifest, PreviewScalarPolicy, ProcessGridArtifact,
    ProcessGridRole, QuantizationPolicy, QueryRegion, SignedAxis, SupportDirection,
    SweptVolumeProvenance, VoxelAddress, VoxelArtifactId, VoxelArtifactManifest, VoxelArtifactRole,
    VoxelCandidateKind, VoxelCandidateManifest, VoxelCell, VoxelChannelMapping, VoxelEditBatch,
    VoxelFieldCouplingKind, VoxelFieldCouplingManifest, VoxelHandoffDomain, VoxelHandoffManifest,
    VoxelIndexConvention, VoxelIoCompression, VoxelIoMetadata, VoxelMemoryBudgetManifest,
    VoxelSideTables, VoxelSliceNaming, VoxelSliceOrdering, VoxelSpatialAggregateFacts,
    VoxelTraceDimension, VoxelTraceManifest, VoxelizationAudit, VoxelizationPolicy,
    classify_support_mask,
    diff_sparse_grids, extract_exposed_faces, greedy_face_patch_plan, lookup_material_display_colors,
    lossy_obj_from_quad_mesh, lossy_quad_mesh_from_faces, query_field_samples,
    query_material_regions, sample_manhattan_distance_field, sample_signed_manhattan_distance_field,
    select_lod_cells, sweep_address_segment, trace_address_ray, voxel_neighbors6,
    voxelize_exact_box, voxelize_exact_convex_halfspace_set, voxelize_exact_halfspace,
};
use libfuzzer_sys::fuzz_target;

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

    let exact_box = ExactBox::new(
        [Real::from(0), Real::from(0), Real::from(0)],
        [Real::from(cells.min(8)), Real::from(cells.min(8)), Real::from(cells.min(8))],
        None,
    );
    let small_depth = depth.min(3);
    let small_frame = GridFrame::builder().depth(small_depth).build().unwrap();
    let (grid, report) = voxelize_exact_box(
        small_frame,
        &exact_box,
        MaterialRegionId(1),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    let halfspace = ExactHalfSpace::new([Real::from(1), Real::from(0), Real::from(0)], Real::from(1), None);
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
    voxelize_exact_convex_halfspace_set(
        GridFrame::builder().depth(small_depth).build().unwrap(),
        &convex,
        MaterialRegionId(3),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    assert_eq!(report.unknown_cells, 0);
    assert_eq!(
        report.predicate_certificates.classified_cells(),
        usize::try_from(1_u64 << (3 * u32::from(small_depth))).unwrap()
    );
    VoxelizationAudit::from_grid_and_report(&grid, &report);
    let faces = extract_exposed_faces(&grid).unwrap();
    assert!(faces.len() <= grid.len() * 6);
    let mesh = lossy_quad_mesh_from_faces(&faces, "fuzz preview").unwrap();
    lossy_obj_from_quad_mesh(&mesh);
    greedy_face_patch_plan(&faces, "fuzz preview");
    DeterministicSnapshot::binary_v1(&grid, &VoxelSideTables::default());
    DeterministicSnapshot::run_length_binary_v1(&grid);
    diff_sparse_grids(&grid, &grid);
    ChunkPageSummary::from_addresses(
        ChunkShape::new(small_depth.min(2)).unwrap(),
        grid.iter().map(|(address, _)| *address),
    );
    ProcessGridArtifact::new(
        ProcessGridRole::SweptVolumeCache,
        None,
        vec!["fuzz".into()],
        report.aggregate.clone(),
    )
    .with_swept_volume(SweptVolumeProvenance {
        source: Some(GridSource::new("fuzz-path", 1)),
        tool_or_beam: Some("fuzz-tool".into()),
        exact_source_replay_available: depth_raw & 1 == 0,
        broad_phase_only: true,
        quantization_policy: "fuzz conservative cover".into(),
    })
    .swept_volume
    .as_ref()
    .unwrap()
    .report();
    VoxelCandidateManifest {
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
    }
    .report();
    VoxelFieldCouplingManifest {
        kind: VoxelFieldCouplingKind::Photochemical,
        freshness: FreshnessStatus::Current,
        aggregate: report.aggregate.clone(),
        residual_replay_available: depth_raw & 1 == 0,
        adapter_error_bound: (depth_raw & 2 != 0).then_some(Real::from(1)),
        missing_sample_records: usize::from(depth_raw & 4),
    }
    .report();
    VoxelArtifactManifest {
        id: VoxelArtifactId(format!("fuzz:{depth}")),
        role: VoxelArtifactRole::StorageSnapshot,
        freshness: FreshnessStatus::Current,
        aggregate: report.aggregate.clone(),
        storage_replay: hypervoxel::StorageReplayStatus::Exact,
        missing_side_table_links: usize::from(depth_raw & 1),
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
    ImageStackManifest {
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
    CompressedStorageManifest {
        kind: CompressedStorageKind::RunLengthSnapshot,
        stored_cells: prepared_storage_len_for_fuzz(small_depth),
        physical_records: 1,
        chunk_shape: ChunkShape::new(small_depth.min(2)).ok(),
        preserves_aggregate_facts: true,
        preserves_payload_ids: true,
        preserves_side_table_links: false,
    }
    .report();
    VoxelMemoryBudgetManifest {
        kind: CompressedStorageKind::SparseVoxelDag,
        estimated_bytes: prepared_storage_len_for_fuzz(small_depth) * 64,
        budget_bytes: usize::from(depth_raw) + 1,
        preserves_exact_semantics_when_over_budget: depth_raw & 1 == 0,
    }
    .report();
    PreviewExportManifest {
        format: PreviewExportFormat::ContinuousSdfPreview,
        exact_input_primitives: 1,
        exported_primitives: 1,
        scalar_policy: PreviewScalarPolicy::PrimitiveFloat,
        preserves_grid_topology: false,
        has_explicit_labels: false,
    }
    .report();
    AdapterNumericContract::primitive_float(
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
    VoxelHandoffManifest {
        domain: VoxelHandoffDomain::Hypercircuit,
        source: Some(GridSource::new("fuzz", 1)),
        expected_source: Some(GridSource::new("fuzz", 2)),
        required_side_table_links: 1,
        supplied_side_table_links: 0,
        aggregate: report.aggregate.clone(),
    }
    .report();
    VoxelTraceManifest {
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

    let prepared = PreparedVoxelGrid::new(report.frame.clone(), grid, report.aggregate.clone());
    prepared.prepared_query_report(depth_raw & 1 == 0).unwrap();
    VoxelSpatialAggregateFacts::from_grid(&prepared.storage, None).unwrap();
    classify_support_mask(
        &prepared.storage,
        &prepared.storage,
        SupportDirection::new((depth_raw as usize) % 3, if depth_raw & 1 == 0 { -1 } else { 1 })
            .unwrap(),
    )
    .unwrap();
    let neighbors = voxel_neighbors6(VoxelAddress::new(small_depth, [0, 0, 0]).unwrap());
    assert!(neighbors.len() <= 3);
    prepared
        .query_manhattan_band(VoxelAddress::new(small_depth, [0, 0, 0]).unwrap(), 2)
        .unwrap();
    select_lod_cells(&prepared.storage, 0).unwrap();
    sample_manhattan_distance_field(
        &prepared.storage,
        QueryRegion {
            min: [0, 0, 0],
            max: [0, 0, 0],
            depth: small_depth,
        },
    )
    .unwrap();
    sample_signed_manhattan_distance_field(
        &prepared.storage,
        QueryRegion {
            min: [0, 0, 0],
            max: [0, 0, 0],
            depth: small_depth,
        },
    )
    .unwrap();
    trace_address_ray(AddressRay {
        start: VoxelAddress::new(small_depth, [0, 0, 0]).unwrap(),
        axis: 0,
        direction: 1,
        max_steps: 2,
    })
    .unwrap();

    let mut edited = hypervoxel::SparseVoxelGrid::new(GridFrame::builder().depth(small_depth).build().unwrap());
    let mut batch = VoxelEditBatch::new();
    let sample_address = VoxelAddress::new(small_depth, [0, 0, 0]).unwrap();
    batch.push(sample_address, VoxelCell::field_sample(FieldSampleId(7)));
    batch.apply_to(&mut edited).unwrap();

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
    FieldAggregateFacts::from_grid(&edited, &side_tables).unwrap();
    query_field_samples(&edited, &side_tables);
    let material_query = query_material_regions(&edited, &side_tables);
    lookup_material_display_colors(&material_query, &MaterialDisplayPalette::default());
    sweep_address_segment(&prepared, sample_address, sample_address).unwrap();
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
