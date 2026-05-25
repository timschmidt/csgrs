use hyperreal::{Rational, Real};
use hypervoxel::{
    AdapterNumericContract, AdapterScalarPrecision, AdapterToleranceStatus, AggregateCertainty,
    BoundaryPolicy, ChunkAddress, ChunkPageSummary, ChunkPagedSparseGrid, ChunkShape,
    ContinuousFieldVoxelCell, ContinuousFieldVoxelInterchangeManifest,
    ContinuousFieldVoxelManifest, ContinuousFieldVoxelRowOrder, FreshnessStatus, GridBasis,
    GridCoordinateSystem, GridFrame, GridFrameManifest, GridHandedness, GridSource,
    HypervoxelError, ImageStackContainer, ImageStackManifest, LegacyAdapterKind,
    LegacyAdapterStatus, LengthUnit, MaterialRegionId, OccupancyState, PreparedSparseVoxelGridExt,
    PreparedVoxelGrid, QuantizationPolicy, SideTableLinkStatus, SparseVoxelGrid,
    StorageReplayStatus, VoxelAddress, VoxelAggregateFacts, VoxelArtifactId, VoxelArtifactManifest,
    VoxelArtifactRole, VoxelCell, VoxelChannelMapping, VoxelHandoffDomain, VoxelIndexConvention,
    VoxelInterchangeFormat, VoxelInterchangeManifest, VoxelIoCompression, VoxelIoMetadata,
    VoxelIoMetadataStatus, VoxelIoPaletteStatus, VoxelIoPayloadStatus, VoxelPayload,
    VoxelPredicateCertificateReport, VoxelSliceNaming, VoxelSliceOrdering,
    VoxelSpatialAggregateFacts, VoxelTraceDimension, VoxelTraceManifest, VoxelizationPolicy,
    VoxelizationReport, continuous_field_address,
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
fn continuous_field_voxel_manifest_accepts_exact_sdf_cell_rows() {
    let frame = frame(1);
    let cells = vec![
        ContinuousFieldVoxelCell::new(
            continuous_field_address(&frame, [0, 0, 0]).unwrap(),
            VoxelCell::material(MaterialRegionId(1)),
        ),
        ContinuousFieldVoxelCell::new(
            continuous_field_address(&frame, [1, 0, 0]).unwrap(),
            VoxelCell::boundary(VoxelPayload::MaterialRegion(MaterialRegionId(1))),
        ),
        ContinuousFieldVoxelCell::new(
            continuous_field_address(&frame, [0, 1, 0]).unwrap(),
            VoxelCell::empty(),
        ),
    ];
    let manifest = ContinuousFieldVoxelManifest {
        frame: frame.clone(),
        source: frame.source().cloned(),
        expected_source: frame.source().cloned(),
        expected_cell_count: cells.len(),
        cells,
    };
    let report = manifest.report();

    assert_eq!(report.freshness, FreshnessStatus::Current);
    assert_eq!(report.supplied_cell_count, 3);
    assert_eq!(report.duplicate_address_count, 0);
    assert_eq!(report.frame_validated_cell_count, 3);
    assert!(report.finest_depth_only);
    assert!(report.complete_expected_cover);
    assert!(!report.complete_frame_cover);
    assert!(report.exact_cell_evidence_ready);
    assert!(!report.exact_materialization_ready);
    assert_eq!(report.predicate_certificates.inside_cells, 1);
    assert_eq!(report.predicate_certificates.outside_cells, 1);
    assert_eq!(report.predicate_certificates.boundary_cells, 1);
    assert_eq!(report.predicate_certificates.unknown_cells, 0);
    assert!(manifest.materialize_exact_sparse_grid().is_err());

    let prepared = manifest.materialize_sparse_grid().unwrap();
    assert_eq!(
        prepared.report.as_ref().unwrap().predicate_certificates,
        report.predicate_certificates
    );
    assert!(prepared.report.as_ref().unwrap().exact_topology_ready());
    assert_eq!(prepared.storage.len(), 2);

    let mut full_cover = Vec::new();
    for z in 0..2 {
        for y in 0..2 {
            for x in 0..2 {
                full_cover.push(ContinuousFieldVoxelCell::new(
                    continuous_field_address(&frame, [x, y, z]).unwrap(),
                    VoxelCell::material(MaterialRegionId(1)),
                ));
            }
        }
    }
    let full_manifest = ContinuousFieldVoxelManifest {
        frame: frame.clone(),
        source: frame.source().cloned(),
        expected_source: frame.source().cloned(),
        expected_cell_count: full_cover.len(),
        cells: full_cover,
    };
    assert!(full_manifest.report().exact_materialization_ready);
    assert_eq!(
        full_manifest
            .materialize_exact_sparse_grid()
            .unwrap()
            .storage
            .len(),
        8
    );
}

#[test]
fn continuous_field_voxel_manifest_blocks_stale_duplicate_unknown_or_incomplete_rows() {
    let frame = frame(2);
    let address = continuous_field_address(&frame, [0, 0, 0]).unwrap();
    let stale_duplicate_unknown = ContinuousFieldVoxelManifest {
        frame: frame.clone(),
        source: Some(GridSource::new("mesh:gear", 6)),
        expected_source: frame.source().cloned(),
        expected_cell_count: 4,
        cells: vec![
            ContinuousFieldVoxelCell::new(address, VoxelCell::unknown()),
            ContinuousFieldVoxelCell::new(address, VoxelCell::lossy_adapter_value(7)),
        ],
    };
    let report = stale_duplicate_unknown.report();

    assert_eq!(report.freshness, FreshnessStatus::Stale);
    assert_eq!(report.duplicate_address_count, 1);
    assert_eq!(report.frame_validated_cell_count, 2);
    assert!(!report.complete_expected_cover);
    assert!(!report.exact_cell_evidence_ready);
    assert!(!report.exact_materialization_ready);
    assert_eq!(report.predicate_certificates.unknown_cells, 2);
    assert!(report.aggregate.has_unknown);
    assert!(report.aggregate.has_lossy);

    let prepared = stale_duplicate_unknown.materialize_sparse_grid().unwrap();
    assert!(!prepared.report.as_ref().unwrap().exact_topology_ready());
}

#[test]
fn continuous_field_interchange_report_validates_frame_and_row_contract() {
    let frame = frame(1);
    let rows = (0..2)
        .flat_map(|z| {
            let frame = frame.clone();
            (0..2).flat_map(move |y| {
                let frame = frame.clone();
                (0..2).map(move |x| {
                    ContinuousFieldVoxelCell::new(
                        continuous_field_address(&frame, [x, y, z]).unwrap(),
                        VoxelCell::material(MaterialRegionId(1)),
                    )
                })
            })
        })
        .collect::<Vec<_>>();
    let intake = ContinuousFieldVoxelManifest {
        frame: frame.clone(),
        source: frame.source().cloned(),
        expected_source: frame.source().cloned(),
        expected_cell_count: rows.len(),
        cells: rows,
    };
    let manifest = ContinuousFieldVoxelInterchangeManifest {
        source: frame.source().cloned(),
        expected_source: frame.source().cloned(),
        coordinate_system: GridCoordinateSystem::HyperGrid,
        row_order: ContinuousFieldVoxelRowOrder::ZMajorYThenXFast,
        declared_depth: frame.depth(),
        declared_dimensions: [2, 2, 2],
        declared_cell_count: 8,
    };
    let report = intake.interchange_report(&manifest);
    assert!(report.exact_interchange_ready);

    let bad_manifest = ContinuousFieldVoxelInterchangeManifest {
        coordinate_system: GridCoordinateSystem::Unknown,
        row_order: ContinuousFieldVoxelRowOrder::Unknown,
        declared_depth: frame.depth() + 1,
        declared_dimensions: [2, 2, 1],
        declared_cell_count: 7,
        ..manifest
    };
    let bad_report = intake.interchange_report(&bad_manifest);
    assert_eq!(bad_report.freshness, FreshnessStatus::Current);
    assert!(!bad_report.depth_matches_frame);
    assert!(!bad_report.dimensions_match_frame);
    assert!(!bad_report.cell_count_matches);
    assert!(!bad_report.coordinate_system_declared);
    assert!(!bad_report.row_order_declared);
    assert!(!bad_report.exact_interchange_ready);
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
    assert_eq!(report.invalid_dimension_axes, 0);
    assert!(!report.positive_dimensions_ready);
    assert_eq!(report.declared_channels, Some(2));
    assert_eq!(report.mapped_channels, 1);
    assert_eq!(report.unmapped_channels, 1);
    assert_eq!(report.extra_channel_mappings, 0);
    assert_eq!(report.bit_depth, Some(8));
    assert_eq!(report.declared_sample_slots, None);
    assert!(!report.has_sample_evidence);
    assert!(!report.has_missing_slice_policy);
    assert!(!report.has_duplicate_slice_policy);
    assert_eq!(report.metadata_status, VoxelIoMetadataStatus::Unknown);
    assert_eq!(report.payload_status, VoxelIoPayloadStatus::Unknown);
    assert_eq!(report.palette_status, VoxelIoPaletteStatus::Lost);
    assert_eq!(report.source_freshness, FreshnessStatus::Stale);
    assert_eq!(report.side_table_links, SideTableLinkStatus::Missing);
    assert!(!report.adapter.exact_replay);
    assert!(!report.certified_sample_replay_ready);
    assert!(!report.exact_sample_replay_ready);

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
    assert_eq!(report.invalid_dimension_axes, 0);
    assert!(report.positive_dimensions_ready);
    assert_eq!(report.declared_channels, None);
    assert_eq!(report.mapped_channels, 0);
    assert_eq!(report.extra_channel_mappings, 0);
    assert_eq!(report.bit_depth, None);
    assert_eq!(report.declared_sample_slots, Some(64));
    assert!(report.has_sample_evidence);
    assert!(report.has_missing_slice_policy);
    assert!(report.has_duplicate_slice_policy);
    assert_eq!(report.payload_status, VoxelIoPayloadStatus::ExactReplay);
    assert_eq!(report.source_freshness, FreshnessStatus::Current);
    assert_eq!(report.side_table_links, SideTableLinkStatus::Complete);
    assert_eq!(report.index_convention, VoxelIndexConvention::CellCenter);
    assert_eq!(report.compression, VoxelIoCompression::None);
    assert!(report.adapter.exact_replay);
    assert!(report.certified_sample_replay_ready);
    assert!(report.exact_sample_replay_ready);

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
    assert_eq!(certified.source_freshness, FreshnessStatus::Unknown);
    assert_eq!(certified.freshness, FreshnessStatus::Unknown);
    assert_eq!(certified.invalid_dimension_axes, 0);
    assert!(certified.positive_dimensions_ready);
    assert_eq!(certified.declared_sample_slots, Some(512));
    assert!(certified.has_sample_evidence);
    assert!(!certified.adapter.exact_replay);
    assert_eq!(certified.palette_status, VoxelIoPaletteStatus::Lost);
    assert!(certified.certified_sample_replay_ready);
    assert!(!certified.exact_sample_replay_ready);

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

    let overdeclared_channels = ImageStackManifest {
        container: ImageStackContainer::ZstdQoi,
        slices: 1,
        channels: 1,
        bit_depth: 16,
        channel_mappings: vec![
            VoxelChannelMapping::OccupancyMask,
            VoxelChannelMapping::MaterialRegion,
        ],
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
            compression: VoxelIoCompression::Zstd,
        },
        source: Some(GridSource::new("qoi:overdeclared", 1)),
        expected_source: Some(GridSource::new("qoi:overdeclared", 1)),
        required_side_table_links: 0,
        supplied_side_table_links: 0,
    }
    .report();
    assert_eq!(overdeclared_channels.mapped_channels, 1);
    assert_eq!(overdeclared_channels.extra_channel_mappings, 1);
    assert_eq!(overdeclared_channels.declared_sample_slots, Some(1));
    assert!(overdeclared_channels.has_sample_evidence);
    assert_eq!(
        overdeclared_channels.payload_status,
        VoxelIoPayloadStatus::Unknown
    );
    assert_eq!(overdeclared_channels.freshness, FreshnessStatus::Unknown);
    assert!(!overdeclared_channels.adapter.exact_replay);
    assert!(!overdeclared_channels.certified_sample_replay_ready);
    assert!(!overdeclared_channels.exact_sample_replay_ready);

    let zero_axis = VoxelInterchangeManifest {
        format: VoxelInterchangeFormat::RawWithSidecar,
        metadata: VoxelIoMetadata {
            dimensions: Some([4, 0, 2]),
            axis_order: Some([0, 1, 2]),
            has_explicit_origin: true,
            has_explicit_spacing: true,
            units: Some(LengthUnit::Millimeter),
            has_payload_mapping: true,
            has_label_mapping: true,
            has_missing_slice_policy: true,
            has_duplicate_slice_policy: true,
            slice_naming: VoxelSliceNaming::ExplicitIndex,
            slice_ordering: VoxelSliceOrdering::LowToHigh,
            index_convention: VoxelIndexConvention::CellCenter,
            compression: VoxelIoCompression::None,
        },
        payload_exact: true,
        certified_payload_mapping: false,
        lost_payload_information: false,
        source: Some(GridSource::new("raw:zero-axis", 1)),
        expected_source: Some(GridSource::new("raw:zero-axis", 1)),
        required_side_table_links: 0,
        supplied_side_table_links: 0,
    }
    .report();
    assert_eq!(zero_axis.unknown_metadata_fields, 0);
    assert_eq!(zero_axis.invalid_dimension_axes, 1);
    assert_eq!(zero_axis.declared_sample_slots, Some(0));
    assert!(!zero_axis.has_sample_evidence);
    assert!(!zero_axis.positive_dimensions_ready);
    assert_eq!(zero_axis.metadata_status, VoxelIoMetadataStatus::Unknown);
    assert!(!zero_axis.certified_sample_replay_ready);
    assert!(!zero_axis.exact_sample_replay_ready);

    let empty_stack = ImageStackManifest {
        container: ImageStackContainer::ZippedPng,
        slices: 0,
        channels: 1,
        bit_depth: 8,
        channel_mappings: vec![VoxelChannelMapping::OccupancyMask],
        metadata: VoxelIoMetadata {
            dimensions: Some([4, 4, 4]),
            axis_order: Some([0, 1, 2]),
            has_explicit_origin: true,
            has_explicit_spacing: true,
            units: Some(LengthUnit::Millimeter),
            has_payload_mapping: true,
            has_label_mapping: true,
            has_missing_slice_policy: true,
            has_duplicate_slice_policy: true,
            slice_naming: VoxelSliceNaming::ExplicitIndex,
            slice_ordering: VoxelSliceOrdering::LowToHigh,
            index_convention: VoxelIndexConvention::CellCenter,
            compression: VoxelIoCompression::Zip,
        },
        source: Some(GridSource::new("png:empty", 1)),
        expected_source: Some(GridSource::new("png:empty", 1)),
        required_side_table_links: 0,
        supplied_side_table_links: 0,
    }
    .report();
    assert_eq!(empty_stack.declared_sample_slots, Some(0));
    assert!(!empty_stack.has_sample_evidence);
    assert!(!empty_stack.certified_sample_replay_ready);
    assert!(!empty_stack.exact_sample_replay_ready);
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
    let empty_report = grid.report();
    assert_eq!(empty_report.logical_leaf_cells, 512);
    assert_eq!(empty_report.root_aggregate.child_count, 512);
    assert_eq!(
        empty_report.root_aggregate.occupancy_interval.total_cells,
        512
    );
    assert!(empty_report.root_aggregate_covers_frame);
    assert!(!empty_report.has_materialized_evidence);
    assert!(!empty_report.exact_dag_replay_ready);

    let a = VoxelAddress::new(3, [0, 0, 0]).unwrap();
    let b = VoxelAddress::new(3, [7, 7, 7]).unwrap();
    let edit = grid
        .set_with_report(a, VoxelCell::material(MaterialRegionId(11)))
        .unwrap();
    assert!(edit.root_changed);
    assert!(edit.exact_path_replay_ready);
    assert_eq!(edit.edit.previous.unwrap(), VoxelCell::empty());
    grid.set(
        b,
        VoxelCell::boundary(VoxelPayload::MaterialRegion(MaterialRegionId(11))),
    )
    .unwrap();

    assert_eq!(grid.get(a).unwrap().occupancy, OccupancyState::Filled);
    assert_eq!(grid.get(b).unwrap().occupancy, OccupancyState::Boundary);
    assert!(grid.aggregate().has_boundary);
    let report = grid.report();
    assert_eq!(report.logical_leaf_cells, 512);
    assert_eq!(report.root_aggregate.child_count, 512);
    assert_eq!(
        report.root_aggregate.occupancy_interval.total_cells,
        report.logical_leaf_cells
    );
    assert!(report.root_aggregate_covers_frame);
    assert!(report.has_materialized_evidence);
    assert!(report.exact_dag_replay_ready);
    assert_eq!(
        grid.get(VoxelAddress::new(3, [1, 1, 1]).unwrap())
            .unwrap()
            .occupancy,
        OccupancyState::Empty
    );
    assert!(grid.stats().nodes < 32);

    let lossy = grid
        .set_with_report(
            VoxelAddress::new(3, [1, 1, 1]).unwrap(),
            VoxelCell::lossy_adapter_value(7),
        )
        .unwrap();
    assert!(!lossy.exact_path_replay_ready);
    assert!(grid.report().root_aggregate.has_lossy);
    assert!(!grid.report().exact_dag_replay_ready);
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
    assert!(report.source_replay_ready());
    assert!(!report.legacy_adapter.as_ref().unwrap().exact_replay);
    assert!(!report.exact_topology_ready());

    let mut stale = report.clone();
    stale.source = Some(GridSource::new("mesh:gear", 8));
    assert_eq!(stale.freshness(), FreshnessStatus::Stale);
    assert!(!stale.source_replay_ready());

    let mut exact_adapter = report.clone();
    exact_adapter.legacy_adapter = Some(LegacyAdapterStatus::exact(
        LegacyAdapterKind::VoxelisObjVoxelize,
        "predicate replayed fixture",
    ));
    assert!(exact_adapter.exact_topology_ready());

    let mut blank_policy_adapter = exact_adapter.clone();
    blank_policy_adapter.legacy_adapter = Some(LegacyAdapterStatus::exact(
        LegacyAdapterKind::VoxelisObjVoxelize,
        "   ",
    ));
    let adapter = blank_policy_adapter.legacy_adapter.as_ref().unwrap();
    assert!(adapter.exact_replay);
    assert!(!adapter.has_policy());
    assert!(!adapter.exact_replay_ready());
    assert!(!blank_policy_adapter.exact_topology_ready());

    let mut empty_predicates = exact_adapter.clone();
    empty_predicates.predicate_certificates =
        VoxelPredicateCertificateReport::from_counts(0, 0, 0, 0);
    assert!(
        !empty_predicates
            .predicate_certificates
            .has_classified_cells()
    );
    assert!(!empty_predicates.predicate_certificates.is_fully_certified());
    assert!(!empty_predicates.exact_topology_ready());

    let mut unknown_predicate = exact_adapter.clone();
    unknown_predicate.predicate_certificates =
        VoxelPredicateCertificateReport::from_counts(0, 0, 0, 1);
    assert!(
        !unknown_predicate
            .predicate_certificates
            .is_fully_certified()
    );
    assert!(!unknown_predicate.exact_topology_ready());
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
    assert!(!exact.has_explicit_error_bound);
    assert!(exact.tolerance_declaration_complete);
    assert!(!exact.has_unbounded_tolerance);
    assert!(exact.adapter_policy_ready);
    assert!(exact.can_contribute_certified_values);
    assert!(exact.can_drive_exact_topology);

    let blank_policy_exact = AdapterNumericContract::exact(
        LegacyAdapterStatus::exact(LegacyAdapterKind::ImportExport, ""),
        r(1),
    )
    .report();
    assert!(blank_policy_exact.adapter.exact_replay);
    assert!(!blank_policy_exact.adapter_policy_ready);
    assert!(!blank_policy_exact.can_drive_exact_topology);

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
    assert!(!missing_tolerance.tolerance_declaration_complete);
    assert!(!missing_tolerance.can_contribute_certified_values);
    assert!(!missing_tolerance.can_drive_exact_topology);

    let explicit_but_unbounded = AdapterNumericContract::primitive_float(
        LegacyAdapterStatus::lossy(LegacyAdapterKind::PreviewRenderer, "empty explicit bounds"),
        Some(r(1)),
        None,
        None,
        AdapterToleranceStatus::Explicit,
    )
    .report();
    assert!(!explicit_but_unbounded.has_explicit_error_bound);
    assert!(!explicit_but_unbounded.tolerance_declaration_complete);
    assert!(!explicit_but_unbounded.can_contribute_certified_values);
    assert!(!explicit_but_unbounded.can_drive_exact_topology);

    let negative_epsilon = AdapterNumericContract::primitive_float(
        LegacyAdapterStatus::lossy(LegacyAdapterKind::PreviewRenderer, "bad display epsilon"),
        Some(r(1)),
        Some(r(-1)),
        Some(r(0)),
        AdapterToleranceStatus::Explicit,
    )
    .report();
    assert!(!negative_epsilon.epsilon_is_non_negative);
    assert!(negative_epsilon.has_explicit_error_bound);
    assert!(!negative_epsilon.tolerance_declaration_complete);
    assert!(!negative_epsilon.can_contribute_certified_values);

    let exact_with_stray_tolerance = AdapterNumericContract {
        adapter: LegacyAdapterStatus::exact(LegacyAdapterKind::ImportExport, "stray epsilon"),
        source_scale: Some(r(1)),
        scalar_precision: AdapterScalarPrecision::Exact,
        epsilon: Some(r(0)),
        tolerance: None,
        tolerance_status: AdapterToleranceStatus::NotApplicable,
    }
    .report();
    assert!(!exact_with_stray_tolerance.tolerance_declaration_complete);
    assert!(!exact_with_stray_tolerance.can_drive_exact_topology);
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
    assert!(report.has_operation_dimension);
    assert!(report.has_exact_evidence);
    assert!(!report.exact_trace_evidence_ready);

    let vacuous = VoxelTraceManifest {
        operation: "empty timing shell".into(),
        dimensions: Vec::new(),
        exact_predicate_count: 0,
        lossy_adapter_count: 0,
        unknown_count: 0,
    }
    .report();
    assert!(!vacuous.has_operation_dimension);
    assert!(!vacuous.has_exact_evidence);
    assert!(!vacuous.exact_trace_evidence_ready);

    let dimension_only = VoxelTraceManifest {
        operation: "dimension without exact evidence".into(),
        dimensions: vec![VoxelTraceDimension::PreparedQuery],
        exact_predicate_count: 0,
        lossy_adapter_count: 0,
        unknown_count: 0,
    }
    .report();
    assert!(dimension_only.has_operation_dimension);
    assert!(!dimension_only.has_exact_evidence);
    assert!(!dimension_only.exact_trace_evidence_ready);

    let exact_report = VoxelTraceManifest {
        operation: "exact prepared query".into(),
        dimensions: vec![
            VoxelTraceDimension::PreparedQuery,
            VoxelTraceDimension::DomainHandoffReport,
        ],
        exact_predicate_count: 12,
        lossy_adapter_count: 0,
        unknown_count: 0,
    }
    .report();
    assert!(!exact_report.has_lossy_adapter_work);
    assert!(!exact_report.has_unknowns);
    assert!(exact_report.has_operation_dimension);
    assert!(exact_report.has_exact_evidence);
    assert!(exact_report.exact_trace_evidence_ready);
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
    assert!(exact.role_supports_exact_indexing);
    assert!(exact.stable_id_ready);
    assert!(exact.intended_domain_ready);
    assert!(exact.has_aggregate_evidence);
    assert!(exact.indexable_as_exact);

    let empty_aggregate = VoxelAggregateFacts::from_cells(std::iter::empty::<&VoxelCell>());
    let empty = VoxelArtifactManifest {
        id: VoxelArtifactId("artifact:empty".into()),
        role: VoxelArtifactRole::OccupancyCache,
        freshness: FreshnessStatus::Current,
        aggregate: empty_aggregate,
        storage_replay: StorageReplayStatus::Exact,
        missing_side_table_links: 0,
        intended_domains: vec![VoxelHandoffDomain::Hyperphysics],
    }
    .report();
    assert!(!empty.has_aggregate_evidence);
    assert!(!empty.indexable_as_exact);

    let preview = VoxelArtifactManifest {
        id: VoxelArtifactId("artifact:preview:gear".into()),
        role: VoxelArtifactRole::PreviewArtifact,
        freshness: FreshnessStatus::Current,
        aggregate: aggregate.clone(),
        storage_replay: StorageReplayStatus::Exact,
        missing_side_table_links: 0,
        intended_domains: vec![VoxelHandoffDomain::Hyperparts],
    }
    .report();
    assert!(!preview.role_supports_exact_indexing);
    assert!(!preview.indexable_as_exact);

    let unnamed = VoxelArtifactManifest {
        id: VoxelArtifactId("   ".into()),
        role: VoxelArtifactRole::OccupancyCache,
        freshness: FreshnessStatus::Current,
        aggregate: aggregate.clone(),
        storage_replay: StorageReplayStatus::Exact,
        missing_side_table_links: 0,
        intended_domains: vec![VoxelHandoffDomain::Hyperphysics],
    }
    .report();
    assert!(!unnamed.stable_id_ready);
    assert!(unnamed.intended_domain_ready);
    assert!(!unnamed.indexable_as_exact);

    let domainless = VoxelArtifactManifest {
        id: VoxelArtifactId("artifact:occupancy:domainless".into()),
        role: VoxelArtifactRole::OccupancyCache,
        freshness: FreshnessStatus::Current,
        aggregate: aggregate.clone(),
        storage_replay: StorageReplayStatus::Exact,
        missing_side_table_links: 0,
        intended_domains: Vec::new(),
    }
    .report();
    assert!(domainless.stable_id_ready);
    assert!(!domainless.intended_domain_ready);
    assert!(!domainless.indexable_as_exact);

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
    assert!(spatial.has_spatial_evidence);
    assert!(spatial.has_child(0));
    assert!(spatial.has_child(7));
    assert_eq!(spatial.freshness, FreshnessStatus::Current);
    assert!(spatial.exact_bounds_ready);
    assert!(spatial.source_replay_ready);
    let bounds = spatial.exact_bounds.unwrap();
    assert_eq!(bounds.min, left.bounds(grid.frame()).unwrap().min);
    assert_eq!(bounds.max, right.bounds(grid.frame()).unwrap().max);

    let empty_frame = hypervoxel::GridFrame::builder().depth(2).build().unwrap();
    let empty =
        VoxelSpatialAggregateFacts::from_grid(&SparseVoxelGrid::new(empty_frame), None).unwrap();
    assert_eq!(empty.stored_cells, 0);
    assert!(!empty.has_spatial_evidence);
    assert!(empty.exact_bounds.is_none());
    assert!(!empty.exact_bounds_ready);
    assert_eq!(empty.freshness, FreshnessStatus::Unknown);
    assert!(!empty.source_replay_ready);
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
    assert!(report.has_query_evidence);
    assert_eq!(report.freshness, FreshnessStatus::Current);
    assert!(report.report_frame_matches);
    assert!(report.predicate_replay_available);
    assert_eq!(report.aabb_handoffs.len(), 2);
    assert_eq!(report.aabb_handoffs[0].address, first);
    assert_eq!(report.cache_entries, 3);
    assert_eq!(report.estimated_saved_cell_reads, 3);
    assert_eq!(report.aggregate, prepared.aggregate);
    assert!(!report.exact_query_evidence_ready);

    let mismatched_report = VoxelizationReport {
        source: Some(GridSource::new("mesh:gear", 7)),
        frame: GridFrame::builder().depth(2).build().unwrap(),
        policy: VoxelizationPolicy::conservative_cover(),
        aggregate: prepared.aggregate.clone(),
        unknown_cells: 0,
        boundary_cells: 0,
        predicate_certificates: VoxelPredicateCertificateReport::from_counts(1, 0, 0, 0),
        legacy_adapter: None,
    };
    let mismatched = PreparedVoxelGrid::new(
        prepared.frame.clone(),
        prepared.storage.clone(),
        prepared.aggregate.clone(),
    )
    .with_report(mismatched_report)
    .prepared_query_report(true)
    .unwrap();
    assert_eq!(mismatched.freshness, FreshnessStatus::Stale);
    assert!(!mismatched.report_frame_matches);
    assert!(!mismatched.exact_query_evidence_ready);

    let exact_frame = GridFrame::builder()
        .depth(3)
        .source(GridSource::new("mesh:gear", 7))
        .build()
        .unwrap();
    let mut exact_grid = SparseVoxelGrid::new(exact_frame.clone());
    exact_grid
        .set(first, VoxelCell::material(MaterialRegionId(1)))
        .unwrap();
    let exact_aggregate = exact_grid.stored_aggregate();
    let exact_report = VoxelizationReport {
        source: Some(GridSource::new("mesh:gear", 7)),
        frame: exact_frame.clone(),
        policy: VoxelizationPolicy::conservative_cover(),
        aggregate: exact_aggregate.clone(),
        unknown_cells: 0,
        boundary_cells: 0,
        predicate_certificates: VoxelPredicateCertificateReport::from_counts(1, 0, 0, 0),
        legacy_adapter: None,
    };
    let exact_prepared =
        PreparedVoxelGrid::new(exact_frame, exact_grid, exact_aggregate).with_report(exact_report);
    let exact_ready = exact_prepared.prepared_query_report(true).unwrap();
    assert!(exact_ready.has_query_evidence);
    assert!(exact_ready.exact_query_evidence_ready);

    let empty_frame = exact_prepared.frame.clone();
    let empty_grid = SparseVoxelGrid::new(empty_frame.clone());
    let empty_aggregate = empty_grid.stored_aggregate();
    let empty_report = VoxelizationReport {
        source: Some(GridSource::new("mesh:gear", 7)),
        frame: empty_frame.clone(),
        policy: VoxelizationPolicy::conservative_cover(),
        aggregate: empty_aggregate.clone(),
        unknown_cells: 0,
        boundary_cells: 0,
        predicate_certificates: VoxelPredicateCertificateReport::from_counts(0, 0, 0, 0),
        legacy_adapter: None,
    };
    let empty_prepared =
        PreparedVoxelGrid::new(empty_frame, empty_grid, empty_aggregate).with_report(empty_report);
    let empty_query = empty_prepared.prepared_query_report(true).unwrap();
    assert_eq!(empty_query.non_empty_cells, 0);
    assert!(!empty_query.has_query_evidence);
    assert!(!empty_query.exact_query_evidence_ready);
}
