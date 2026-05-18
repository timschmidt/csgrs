use hyperreal::{Rational, Real};
use hypervoxel::{
    AdapterNumericContract, AdapterScalarPrecision, AdapterToleranceStatus, AggregateCertainty,
    BoundaryPolicy, ChunkPageSummary, ChunkShape, FreshnessStatus, GridBasis, GridCoordinateSystem,
    GridFrame, GridFrameManifest, GridHandedness, GridSource, HypervoxelError, ImageStackContainer,
    ImageStackManifest, LegacyAdapterKind, LegacyAdapterStatus, LengthUnit, MaterialRegionId,
    OccupancyState, PreparedSparseVoxelGridExt, PreparedVoxelGrid, QuantizationPolicy,
    SideTableLinkStatus, SparseVoxelGrid, StorageReplayStatus, VoxelAddress, VoxelAggregateFacts,
    VoxelArtifactId, VoxelArtifactManifest, VoxelArtifactRole, VoxelCell, VoxelChannelMapping,
    VoxelHandoffDomain, VoxelIndexConvention, VoxelInterchangeFormat, VoxelInterchangeManifest,
    VoxelIoCompression, VoxelIoMetadata, VoxelIoMetadataStatus, VoxelIoPaletteStatus,
    VoxelIoPayloadStatus, VoxelPayload, VoxelPredicateCertificateReport, VoxelSliceNaming,
    VoxelSliceOrdering, VoxelSpatialAggregateFacts, VoxelTraceDimension, VoxelTraceManifest,
    VoxelizationPolicy, VoxelizationReport,
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
        .source(GridSource::new("mesh:gear", 7))
        .build()
        .unwrap()
}

#[test]
fn exact_cell_bounds_use_grid_depth_not_float_chunk_size() {
    let frame = frame(4);
    let address = VoxelAddress::new(2, [1, 2, 3]).unwrap();
    let bounds = address.bounds(&frame).unwrap();

    assert_eq!(bounds.min[0], rf(-1 * 8 + 4, 8));
    assert_eq!(bounds.max[0], rf(-1 * 8 + 8, 8));
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
    assert_eq!(summary.page_count, 2);
    assert_eq!(
        ChunkShape::new(22).unwrap_err(),
        HypervoxelError::DepthTooLarge {
            depth: 22,
            max_supported: 21
        }
    );
}

#[test]
fn grid_frame_manifest_reports_basis_handedness_coordinate_system_and_chunk_shape() {
    let frame = frame(4);
    let complete = GridFrameManifest {
        frame: frame.clone(),
        basis: GridBasis::AxisAligned,
        handedness: GridHandedness::RightHanded,
        coordinate_system: GridCoordinateSystem::HyperGrid,
        chunk_shape: Some(ChunkShape::new(2).unwrap()),
    }
    .report();
    assert!(complete.complete);
    assert_eq!(complete.facts.depth, 4);
    assert_eq!(complete.facts.cells_per_axis, 16);
    assert_eq!(complete.basis, GridBasis::AxisAligned);
    assert_eq!(complete.handedness, GridHandedness::RightHanded);
    assert_eq!(complete.coordinate_system, GridCoordinateSystem::HyperGrid);
    assert_eq!(complete.chunk_shape.unwrap().cells_per_axis(), 4);

    let incomplete = GridFrameManifest {
        frame,
        basis: GridBasis::Unknown,
        handedness: GridHandedness::Unknown,
        coordinate_system: GridCoordinateSystem::Unknown,
        chunk_shape: None,
    }
    .report();
    assert!(!incomplete.complete);
}

#[test]
fn image_stack_and_interchange_manifests_report_unknown_metadata_explicitly() {
    let incomplete = ImageStackManifest {
        container: ImageStackContainer::ZippedPng,
        slices: 4,
        channels: 2,
        bit_depth: 8,
        channel_mappings: vec![VoxelChannelMapping::OccupancyMask],
        metadata: VoxelIoMetadata {
            dimensions: None,
            axis_order: None,
            has_explicit_origin: false,
            has_explicit_spacing: false,
            units: None,
            has_payload_mapping: true,
            has_label_mapping: false,
            has_missing_slice_policy: false,
            has_duplicate_slice_policy: false,
            slice_naming: VoxelSliceNaming::Unknown,
            slice_ordering: VoxelSliceOrdering::Unknown,
            index_convention: VoxelIndexConvention::Unknown,
            compression: VoxelIoCompression::Unknown,
        },
        source: Some(GridSource::new("scan:missing", 1)),
        expected_source: Some(GridSource::new("scan:missing", 2)),
        required_side_table_links: 2,
        supplied_side_table_links: 1,
    };
    let report = incomplete.report();
    assert_eq!(report.freshness, FreshnessStatus::Unknown);
    assert_eq!(report.unknown_metadata_fields, 11);
    assert_eq!(report.unmapped_channels, 1);
    assert_eq!(report.metadata_status, VoxelIoMetadataStatus::Unknown);
    assert_eq!(report.payload_status, VoxelIoPayloadStatus::Unknown);
    assert_eq!(report.palette_status, VoxelIoPaletteStatus::Lost);
    assert_eq!(report.source_freshness, FreshnessStatus::Stale);
    assert_eq!(report.side_table_links, SideTableLinkStatus::Missing);
    assert!(!report.adapter.exact_replay);

    let exact = VoxelInterchangeManifest {
        format: VoxelInterchangeFormat::Nrrd,
        payload_exact: true,
        certified_payload_mapping: false,
        lost_payload_information: false,
        metadata: VoxelIoMetadata {
            dimensions: Some([4, 4, 4]),
            axis_order: Some([0, 1, 2]),
            has_explicit_origin: true,
            has_explicit_spacing: true,
            units: Some(LengthUnit::Micrometer),
            has_payload_mapping: true,
            has_label_mapping: true,
            has_missing_slice_policy: true,
            has_duplicate_slice_policy: true,
            slice_naming: VoxelSliceNaming::ExplicitIndex,
            slice_ordering: VoxelSliceOrdering::LowToHigh,
            index_convention: VoxelIndexConvention::CellCenter,
            compression: VoxelIoCompression::None,
        },
        source: Some(GridSource::new("nrrd:exact", 7)),
        expected_source: Some(GridSource::new("nrrd:exact", 7)),
        required_side_table_links: 1,
        supplied_side_table_links: 1,
    };
    let report = exact.report();
    assert_eq!(report.freshness, FreshnessStatus::Current);
    assert_eq!(report.unknown_metadata_fields, 0);
    assert_eq!(report.payload_status, VoxelIoPayloadStatus::ExactReplay);
    assert_eq!(report.source_freshness, FreshnessStatus::Current);
    assert_eq!(report.side_table_links, SideTableLinkStatus::Complete);
    assert_eq!(report.index_convention, VoxelIndexConvention::CellCenter);
    assert_eq!(report.compression, VoxelIoCompression::None);
    assert!(report.adapter.exact_replay);

    let certified = VoxelInterchangeManifest {
        format: VoxelInterchangeFormat::Vdb,
        payload_exact: false,
        certified_payload_mapping: true,
        lost_payload_information: false,
        metadata: VoxelIoMetadata {
            dimensions: Some([16, 8, 4]),
            axis_order: Some([2, 1, 0]),
            has_explicit_origin: true,
            has_explicit_spacing: true,
            units: Some(LengthUnit::Meter),
            has_payload_mapping: true,
            has_label_mapping: false,
            has_missing_slice_policy: true,
            has_duplicate_slice_policy: true,
            slice_naming: VoxelSliceNaming::ExplicitIndex,
            slice_ordering: VoxelSliceOrdering::LowToHigh,
            index_convention: VoxelIndexConvention::CellCenter,
            compression: VoxelIoCompression::Native,
        },
        source: None,
        expected_source: None,
        required_side_table_links: 0,
        supplied_side_table_links: 0,
    }
    .report();
    assert_eq!(
        certified.payload_status,
        VoxelIoPayloadStatus::CertifiedMapping
    );
    assert_eq!(certified.palette_status, VoxelIoPaletteStatus::Lost);

    let invalid_axis_order = VoxelIoMetadata {
        dimensions: Some([4, 4, 4]),
        axis_order: Some([0, 0, 2]),
        has_explicit_origin: true,
        has_explicit_spacing: true,
        units: Some(LengthUnit::Millimeter),
        has_payload_mapping: true,
        has_label_mapping: true,
        has_missing_slice_policy: true,
        has_duplicate_slice_policy: true,
        slice_naming: VoxelSliceNaming::Lexicographic,
        slice_ordering: VoxelSliceOrdering::HighToLow,
        index_convention: VoxelIndexConvention::NodeCorner,
        compression: VoxelIoCompression::Zip,
    };
    assert!(!invalid_axis_order.axis_order_is_permutation());
    assert_eq!(
        VoxelInterchangeManifest {
            format: VoxelInterchangeFormat::RawWithSidecar,
            metadata: invalid_axis_order,
            payload_exact: true,
            certified_payload_mapping: false,
            lost_payload_information: false,
            source: Some(GridSource::new("raw:bad-axis", 1)),
            expected_source: Some(GridSource::new("raw:bad-axis", 1)),
            required_side_table_links: 0,
            supplied_side_table_links: 0,
        }
        .report()
        .metadata_status,
        VoxelIoMetadataStatus::Unknown
    );
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
}

#[test]
fn sparse_grid_validates_frame_depth_and_empty_absence() {
    let frame = frame(3);
    let mut grid = SparseVoxelGrid::new(frame);
    let address = VoxelAddress::new(3, [3, 4, 5]).unwrap();
    assert_eq!(grid.get(address).unwrap(), VoxelCell::empty());

    let report = grid
        .set(address, VoxelCell::material(MaterialRegionId(9)))
        .unwrap();
    assert_eq!(report.previous, None);
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
fn svo_dag_reuses_collapsed_empty_subtrees_and_preserves_aggregates() {
    let frame = frame(3);
    let mut grid = SvoVoxelGrid::new(frame);
    let empty_stats = grid.stats();
    assert_eq!(empty_stats.nodes, 1);
    assert!(grid.aggregate().all_empty);

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
    assert_eq!(
        grid.get(VoxelAddress::new(3, [1, 1, 1]).unwrap())
            .unwrap()
            .occupancy,
        OccupancyState::Empty
    );
    assert!(grid.stats().nodes < 32);
}

#[test]
fn voxelization_report_exposes_freshness_and_legacy_status() {
    let frame = frame(2);
    let aggregate = VoxelAggregateFacts::from_cells([&VoxelCell::material(MaterialRegionId(3))]);
    let report = VoxelizationReport {
        source: Some(GridSource::new("mesh:gear", 7)),
        frame: frame.clone(),
        policy: VoxelizationPolicy {
            quantization: QuantizationPolicy::ConservativeCover,
            boundary: BoundaryPolicy::KeepBoundary,
        },
        aggregate,
        unknown_cells: 0,
        boundary_cells: 0,
        predicate_certificates: VoxelPredicateCertificateReport::from_counts(1, 0, 0, 0),
        legacy_adapter: Some(LegacyAdapterStatus::lossy(
            LegacyAdapterKind::VoxelisObjVoxelize,
            "DVec3 epsilon triangle/cube fixture",
        )),
    };

    assert_eq!(report.freshness(), FreshnessStatus::Current);
    assert!(!report.legacy_adapter.as_ref().unwrap().exact_replay);

    let mut stale = report.clone();
    stale.source = Some(GridSource::new("mesh:gear", 8));
    assert_eq!(stale.freshness(), FreshnessStatus::Stale);
}

#[test]
fn adapter_numeric_contract_keeps_float_tolerance_out_of_exact_topology() {
    let exact = AdapterNumericContract::exact(
        LegacyAdapterStatus::exact(LegacyAdapterKind::ImportExport, "exact replayed fixture"),
        r(1),
    )
    .report();
    assert_eq!(exact.scalar_precision, AdapterScalarPrecision::Exact);
    assert!(exact.has_explicit_scale);
    assert!(exact.scale_is_positive);
    assert!(!exact.has_unbounded_tolerance);
    assert!(exact.can_contribute_certified_values);
    assert!(exact.can_drive_exact_topology);

    let missing_tolerance = AdapterNumericContract::primitive_float(
        LegacyAdapterStatus::lossy(LegacyAdapterKind::VoxelisObjVoxelize, "triangle epsilon"),
        Some(r(1)),
        None,
        None,
        AdapterToleranceStatus::Missing,
    )
    .report();
    assert_eq!(
        missing_tolerance.scalar_precision,
        AdapterScalarPrecision::PrimitiveFloat
    );
    assert!(missing_tolerance.has_unbounded_tolerance);
    assert!(!missing_tolerance.can_contribute_certified_values);
    assert!(!missing_tolerance.can_drive_exact_topology);

    let negative_epsilon = AdapterNumericContract::primitive_float(
        LegacyAdapterStatus::lossy(LegacyAdapterKind::PreviewRenderer, "bad display epsilon"),
        Some(r(1)),
        Some(r(-1)),
        Some(r(0)),
        AdapterToleranceStatus::Explicit,
    )
    .report();
    assert!(!negative_epsilon.epsilon_is_non_negative);
    assert!(!negative_epsilon.can_contribute_certified_values);
}

#[test]
fn voxelization_policy_names_non_occupancy_grid_roles() {
    let signed_distance = VoxelizationPolicy {
        quantization: QuantizationPolicy::SignedDistanceSampling,
        boundary: BoundaryPolicy::KeepBoundary,
    };
    assert!(signed_distance.is_exact_semantic_role());
    assert!(!signed_distance.is_occupancy_policy());

    let process_exposure = VoxelizationPolicy {
        quantization: QuantizationPolicy::ProcessExposureGrid,
        boundary: BoundaryPolicy::BoundaryAsUnknown,
    };
    assert!(process_exposure.is_exact_semantic_role());
    assert!(!process_exposure.is_occupancy_policy());

    let material_raster = VoxelizationPolicy {
        quantization: QuantizationPolicy::MaterialRegionRasterization,
        boundary: BoundaryPolicy::KeepBoundary,
    };
    assert!(material_raster.is_occupancy_policy());

    let lossy_preview = VoxelizationPolicy {
        quantization: QuantizationPolicy::LossyPreview,
        boundary: BoundaryPolicy::LossySideChoice,
    };
    assert!(!lossy_preview.is_exact_semantic_role());
    assert!(!lossy_preview.is_occupancy_policy());
}

#[test]
fn trace_manifest_deduplicates_dimensions_and_preserves_uncertainty_counts() {
    let report = VoxelTraceManifest {
        operation: "box voxelization preview".into(),
        dimensions: vec![
            VoxelTraceDimension::ExactVoxelizationPredicateBatch,
            VoxelTraceDimension::GridFrameConstruction,
            VoxelTraceDimension::ExactVoxelizationPredicateBatch,
            VoxelTraceDimension::LossyMeshExportLowering,
        ],
        exact_predicate_count: 512,
        lossy_adapter_count: 4,
        unknown_count: 2,
    }
    .report();

    assert_eq!(report.dimension_count, 3);
    assert_eq!(
        report.dimensions,
        vec![
            VoxelTraceDimension::GridFrameConstruction,
            VoxelTraceDimension::ExactVoxelizationPredicateBatch,
            VoxelTraceDimension::LossyMeshExportLowering,
        ]
    );
    assert!(report.has_lossy_adapter_work);
    assert!(report.has_unknowns);
}

#[test]
fn voxel_artifact_manifest_prevents_stale_or_incomplete_exact_indexing() {
    let aggregate = VoxelAggregateFacts::from_cells([&VoxelCell::material(MaterialRegionId(3))]);
    let exact = VoxelArtifactManifest {
        id: VoxelArtifactId("artifact:occupancy:gear".into()),
        role: VoxelArtifactRole::OccupancyCache,
        freshness: FreshnessStatus::Current,
        aggregate: aggregate.clone(),
        storage_replay: StorageReplayStatus::Exact,
        missing_side_table_links: 0,
        intended_domains: vec![VoxelHandoffDomain::Hyperphysics],
    }
    .report();
    assert!(exact.indexable_as_exact);

    let stale = VoxelArtifactManifest {
        id: VoxelArtifactId("artifact:field:thermal".into()),
        role: VoxelArtifactRole::FieldSampleGrid,
        freshness: FreshnessStatus::Stale,
        aggregate,
        storage_replay: StorageReplayStatus::Certified,
        missing_side_table_links: 1,
        intended_domains: vec![VoxelHandoffDomain::Hypercircuit],
    }
    .report();
    assert!(!stale.indexable_as_exact);
    assert_eq!(stale.missing_side_table_links, 1);
}

#[test]
fn spatial_aggregate_reports_exact_bounds_child_mask_and_freshness() {
    let frame = frame(2);
    let mut grid = SparseVoxelGrid::new(frame.clone());
    let left = VoxelAddress::new(2, [0, 0, 0]).unwrap();
    let right = VoxelAddress::new(2, [3, 3, 3]).unwrap();
    grid.set(left, VoxelCell::material(MaterialRegionId(1)))
        .unwrap();
    grid.set(right, VoxelCell::material(MaterialRegionId(1)))
        .unwrap();
    let report = VoxelizationReport {
        source: Some(GridSource::new("mesh:gear", 7)),
        frame,
        policy: VoxelizationPolicy::conservative_cover(),
        aggregate: grid.stored_aggregate(),
        unknown_cells: 0,
        boundary_cells: 0,
        predicate_certificates: VoxelPredicateCertificateReport::from_counts(2, 0, 0, 0),
        legacy_adapter: None,
    };

    let spatial = VoxelSpatialAggregateFacts::from_grid(&grid, Some(&report)).unwrap();
    assert_eq!(spatial.stored_cells, 2);
    assert!(spatial.has_child(0));
    assert!(spatial.has_child(7));
    assert_eq!(spatial.freshness, FreshnessStatus::Current);
    let bounds = spatial.exact_bounds.unwrap();
    assert_eq!(bounds.min, left.bounds(grid.frame()).unwrap().min);
    assert_eq!(bounds.max, right.bounds(grid.frame()).unwrap().max);
}

#[test]
fn prepared_query_report_exposes_exact_cache_evidence_and_payoff() {
    let frame = frame(3);
    let mut grid = SparseVoxelGrid::new(frame.clone());
    let first = VoxelAddress::new(3, [1, 2, 3]).unwrap();
    let second = VoxelAddress::new(3, [4, 5, 6]).unwrap();
    grid.set(first, VoxelCell::material(MaterialRegionId(1)))
        .unwrap();
    grid.set(second, VoxelCell::unknown()).unwrap();
    let aggregate = grid.stored_aggregate();
    let source_report = VoxelizationReport {
        source: Some(GridSource::new("mesh:gear", 7)),
        frame: frame.clone(),
        policy: VoxelizationPolicy::conservative_cover(),
        aggregate: aggregate.clone(),
        unknown_cells: 1,
        boundary_cells: 0,
        predicate_certificates: VoxelPredicateCertificateReport::from_counts(1, 0, 0, 1),
        legacy_adapter: None,
    };
    let prepared = PreparedVoxelGrid::new(frame, grid, aggregate).with_report(source_report);

    let report = prepared.prepared_query_report(true).unwrap();
    assert_eq!(report.stored_cells, 2);
    assert_eq!(report.non_empty_cells, 2);
    assert_eq!(report.freshness, FreshnessStatus::Current);
    assert!(report.predicate_replay_available);
    assert_eq!(report.aabb_handoffs.len(), 2);
    assert_eq!(report.aabb_handoffs[0].address, first);
    assert_eq!(report.cache_entries, 3);
    assert_eq!(report.estimated_saved_cell_reads, 3);
    assert_eq!(report.aggregate, prepared.aggregate);
}
