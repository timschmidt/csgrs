use hyperreal::{Rational, Real};
use hypervoxel::{
    AddressRay, AggregateCertainty, AxisPermutationTransform, CellBounds, CertifiedFieldInterval,
    CertifiedTensorInterval, CertifiedVectorInterval, ChunkPagedSparseGrid, ChunkShape,
    ExactAffineTransform, FieldAggregateFacts, FieldEnvelopeFacts, FieldSampleId,
    FieldSampleRecord, GridFrame, HypervoxelError, MaterialRegionId, SignedAxis, SparseVoxelGrid,
    SupportCellStatus, SupportDirection, VoxelAddress, VoxelCell, VoxelEditBatch, VoxelSideTables,
    classify_chunk_paged_support_mask, classify_support_mask, query_field_samples,
    sweep_address_segment, trace_address_ray, trace_address_segment,
};

fn r(n: i32) -> Real {
    n.into()
}

fn rf(n: i64, d: u64) -> Real {
    Rational::fraction(n, d).unwrap().into()
}

fn frame() -> GridFrame {
    GridFrame::builder().depth(3).build().unwrap()
}

#[test]
fn field_aggregate_unions_certified_side_table_bounds() {
    let mut grid = SparseVoxelGrid::new(frame());
    grid.set(
        VoxelAddress::new(3, [1, 1, 1]).unwrap(),
        VoxelCell::field_sample(FieldSampleId(1)),
    )
    .unwrap();
    grid.set(
        VoxelAddress::new(3, [2, 1, 1]).unwrap(),
        VoxelCell::field_sample(FieldSampleId(2)),
    )
    .unwrap();

    let mut side_tables = VoxelSideTables::default();
    side_tables.insert_field_sample(
        FieldSampleId(1),
        FieldSampleRecord {
            label: "dose low".into(),
            lower: Some(rf(1, 4)),
            upper: Some(rf(3, 4)),
        },
    );
    side_tables.insert_field_sample(
        FieldSampleId(2),
        FieldSampleRecord {
            label: "dose high".into(),
            lower: Some(rf(1, 2)),
            upper: Some(rf(5, 4)),
        },
    );

    let facts = FieldAggregateFacts::from_grid(&grid, &side_tables).unwrap();
    assert_eq!(facts.sample_cell_count, 2);
    assert!(facts.has_field_samples);
    assert_eq!(facts.certainty, AggregateCertainty::Certified);
    assert!(facts.certified_field_bounds_ready);
    let interval = facts.interval.as_ref().unwrap();
    assert_eq!(interval.lower, rf(1, 4));
    let ball = interval.enclosing_ball();
    assert_eq!(ball.center, rf(3, 4));
    assert_eq!(ball.radius, rf(1, 2));
    assert_eq!(facts.missing_records, 0);
    let query = query_field_samples(&grid, &side_tables);
    assert!(query.is_fully_resolved());
}

#[test]
fn vector_and_tensor_envelopes_merge_component_intervals_conservatively() {
    let a = CertifiedFieldInterval {
        lower: r(0),
        upper: r(1),
    };
    let b = CertifiedFieldInterval {
        lower: r(-1),
        upper: r(2),
    };
    let vector_left = CertifiedVectorInterval {
        components: vec![a.clone(), a.clone()],
    };
    let vector_right = CertifiedVectorInterval {
        components: vec![b.clone(), a.clone()],
    };
    let tensor = CertifiedTensorInterval {
        rows: 1,
        cols: 2,
        components: vec![a.clone(), b.clone()],
    };

    let facts =
        FieldEnvelopeFacts::from_envelopes([&vector_left, &vector_right], [&tensor]).unwrap();
    assert_eq!(facts.vector_count, 2);
    assert_eq!(facts.tensor_count, 1);
    assert_eq!(facts.envelope_count, 3);
    assert!(facts.has_envelopes);
    assert_eq!(facts.certainty, AggregateCertainty::Certified);
    assert!(facts.certified_envelope_ready);
    assert_eq!(facts.vector_interval.unwrap().components[0].lower, r(-1));
    assert_eq!(facts.tensor_interval.unwrap().components[1].upper, r(2));

    let incompatible_vector = CertifiedVectorInterval {
        components: vec![a.clone(), b.clone(), a.clone()],
    };
    let incompatible = FieldEnvelopeFacts::from_envelopes(
        [&vector_left, &incompatible_vector],
        std::iter::empty::<&CertifiedTensorInterval>(),
    )
    .unwrap();
    assert_eq!(incompatible.incompatible_shapes, 1);
    assert!(incompatible.has_envelopes);
    assert!(!incompatible.certified_envelope_ready);
    assert_eq!(incompatible.certainty, AggregateCertainty::Unknown);

    let empty = FieldEnvelopeFacts::from_envelopes(
        std::iter::empty::<&CertifiedVectorInterval>(),
        std::iter::empty::<&CertifiedTensorInterval>(),
    )
    .unwrap();
    assert_eq!(empty.envelope_count, 0);
    assert!(!empty.has_envelopes);
    assert_eq!(empty.certainty, AggregateCertainty::Unknown);
    assert!(!empty.certified_envelope_ready);
}

#[test]
fn field_aggregate_reports_missing_records_and_bounds_as_unknown() {
    let mut grid = SparseVoxelGrid::new(frame());
    grid.set(
        VoxelAddress::new(3, [1, 1, 1]).unwrap(),
        VoxelCell::field_sample(FieldSampleId(1)),
    )
    .unwrap();
    grid.set(
        VoxelAddress::new(3, [2, 1, 1]).unwrap(),
        VoxelCell::field_sample(FieldSampleId(2)),
    )
    .unwrap();

    let mut side_tables = VoxelSideTables::default();
    side_tables.insert_field_sample(
        FieldSampleId(1),
        FieldSampleRecord {
            label: "unbounded".into(),
            lower: Some(r(0)),
            upper: None,
        },
    );

    let facts = FieldAggregateFacts::from_grid(&grid, &side_tables).unwrap();
    assert_eq!(facts.certainty, AggregateCertainty::Unknown);
    assert!(!facts.certified_field_bounds_ready);
    assert_eq!(facts.missing_bounds, 1);
    assert_eq!(facts.missing_records, 1);

    let empty_facts =
        FieldAggregateFacts::from_grid(&SparseVoxelGrid::new(frame()), &side_tables).unwrap();
    assert_eq!(empty_facts.sample_cell_count, 0);
    assert!(!empty_facts.has_field_samples);
    assert_eq!(empty_facts.certainty, AggregateCertainty::Unknown);
    assert!(!empty_facts.certified_field_bounds_ready);

    let query = query_field_samples(&grid, &side_tables);
    assert_eq!(query.referenced.len(), 2);
    assert!(query.missing_bounds.contains(&FieldSampleId(1)));
    assert!(query.missing_records.contains(&FieldSampleId(2)));
    assert!(!query.is_fully_resolved());
}

#[test]
fn inverted_field_interval_is_rejected_before_it_can_certify_grid_state() {
    let mut grid = SparseVoxelGrid::new(frame());
    grid.set(
        VoxelAddress::new(3, [1, 1, 1]).unwrap(),
        VoxelCell::field_sample(FieldSampleId(1)),
    )
    .unwrap();
    let mut side_tables = VoxelSideTables::default();
    side_tables.insert_field_sample(
        FieldSampleId(1),
        FieldSampleRecord {
            label: "bad".into(),
            lower: Some(r(2)),
            upper: Some(r(1)),
        },
    );

    assert!(matches!(
        FieldAggregateFacts::from_grid(&grid, &side_tables),
        Err(HypervoxelError::UnknownScalarOrdering {
            field: "inverted field interval"
        })
    ));
}

#[test]
fn edit_batches_apply_in_order_and_empty_cells_remove_explicit_storage() {
    let mut grid = SparseVoxelGrid::new(frame());
    let address = VoxelAddress::new(3, [1, 1, 1]).unwrap();
    let mut batch = VoxelEditBatch::new();
    batch.push(address, VoxelCell::material(MaterialRegionId(3)));
    batch.push(address, VoxelCell::empty());

    batch.apply_to(&mut grid).unwrap();
    assert!(grid.is_empty());
}

#[test]
fn address_segment_trace_rejects_mismatched_depths() {
    let left = VoxelAddress::new(2, [1, 1, 1]).unwrap();
    let right = VoxelAddress::new(3, [2, 2, 2]).unwrap();
    assert!(matches!(
        trace_address_segment(left, right),
        Err(HypervoxelError::MismatchedAddressDepth { left: 2, right: 3 })
    ));
}

#[test]
fn address_segment_sweep_samples_cells_and_conservative_aggregate() {
    let mut grid = SparseVoxelGrid::new(frame());
    for x in 1..=3 {
        grid.set(
            VoxelAddress::new(3, [x, 1, 1]).unwrap(),
            VoxelCell::material(MaterialRegionId(1)),
        )
        .unwrap();
    }
    let start = VoxelAddress::new(3, [1, 1, 1]).unwrap();
    let end = VoxelAddress::new(3, [3, 1, 1]).unwrap();

    let sweep = sweep_address_segment(&grid, start, end).unwrap();
    assert_eq!(sweep.trace.addresses.len(), 3);
    assert!(sweep.trace.exact_address_trace_ready);
    assert!(sweep.trace.reached_end);
    assert!(sweep.exact_sweep_samples_ready);
    assert!(sweep.aggregate.all_filled);
    assert_eq!(sweep.aggregate.material_regions.len(), 1);
}

#[test]
fn address_ray_trace_stops_at_grid_boundary_or_step_limit() {
    let ray = AddressRay {
        start: VoxelAddress::new(3, [2, 1, 1]).unwrap(),
        axis: 0,
        direction: -1,
        max_steps: 10,
    };
    let trace = trace_address_ray(ray).unwrap();
    assert_eq!(trace.addresses.len(), 3);
    assert_eq!(trace.addresses.last().unwrap().xyz, [0, 1, 1]);
    assert!(trace.exact_address_trace_ready);
    assert!(trace.stopped_at_boundary);
    assert!(!trace.stopped_at_step_limit);

    let limited = trace_address_ray(AddressRay {
        max_steps: 1,
        direction: 1,
        ..ray
    })
    .unwrap();
    assert_eq!(limited.addresses.len(), 2);
    assert!(limited.exact_address_trace_ready);
    assert!(!limited.stopped_at_boundary);
    assert!(limited.stopped_at_step_limit);

    assert_eq!(
        trace_address_ray(AddressRay {
            start: VoxelAddress {
                depth: 3,
                xyz: [8, 0, 0],
            },
            axis: 0,
            direction: 1,
            max_steps: 1,
        })
        .unwrap_err(),
        HypervoxelError::AddressOverflow
    );
}

#[test]
fn signed_axis_transform_maps_bounds_without_float_matrices() {
    let transform = AxisPermutationTransform::new(
        [
            SignedAxis::new(1, 1).unwrap(),
            SignedAxis::new(0, -1).unwrap(),
            SignedAxis::new(2, 1).unwrap(),
        ],
        [r(10), r(20), r(30)],
    )
    .unwrap();
    let bounds = VoxelAddress::new(3, [1, 2, 3])
        .unwrap()
        .bounds(&frame())
        .unwrap();

    let mapped = transform.map_bounds(&bounds).unwrap();
    assert_eq!(mapped.min, [r(12), r(18), r(33)]);
    assert_eq!(mapped.max, [r(13), r(19), r(34)]);

    let reversed = CellBounds {
        min: bounds.max,
        max: bounds.min,
    };
    assert_eq!(transform.map_bounds(&reversed).unwrap(), mapped);
}

#[test]
fn exact_affine_transform_maps_bounds_by_certified_term_intervals() {
    let affine = ExactAffineTransform::new(
        [
            [r(1), r(-2), r(0)],
            [r(0), r(-1), r(1)],
            [r(-3), r(0), r(2)],
        ],
        [r(10), r(20), r(30)],
    );
    let bounds = VoxelAddress::new(3, [1, 2, 3])
        .unwrap()
        .bounds(&frame())
        .unwrap();

    let mapped = affine.map_bounds(&bounds).unwrap();
    assert_eq!(mapped.min, [r(5), r(20), r(30)]);
    assert_eq!(mapped.max, [r(8), r(22), r(35)]);
}

#[test]
fn invalid_signed_axis_transform_rejects_duplicate_source_axes() {
    assert!(matches!(
        AxisPermutationTransform::new(
            [
                SignedAxis::new(0, 1).unwrap(),
                SignedAxis::new(0, -1).unwrap(),
                SignedAxis::new(2, 1).unwrap(),
            ],
            [r(0), r(0), r(0)],
        ),
        Err(HypervoxelError::InvalidAxisPermutation)
    ));
}

#[test]
fn support_mask_reports_unsupported_unknown_and_lossy_cells_explicitly() {
    let mut target = SparseVoxelGrid::new(frame());
    let mut support = SparseVoxelGrid::new(frame());
    let supported = VoxelAddress::new(3, [1, 1, 1]).unwrap();
    let unsupported = VoxelAddress::new(3, [3, 1, 1]).unwrap();
    let unknown = VoxelAddress::new(3, [5, 1, 1]).unwrap();
    let lossy = VoxelAddress::new(3, [7, 1, 1]).unwrap();

    for address in [supported, unsupported, unknown, lossy] {
        target
            .set(address, VoxelCell::material(MaterialRegionId(1)))
            .unwrap();
    }
    support
        .set(
            VoxelAddress::new(3, [1, 1, 0]).unwrap(),
            VoxelCell::material(MaterialRegionId(2)),
        )
        .unwrap();
    support
        .set(
            VoxelAddress::new(3, [5, 1, 0]).unwrap(),
            VoxelCell::unknown(),
        )
        .unwrap();
    support
        .set(
            VoxelAddress::new(3, [7, 1, 0]).unwrap(),
            VoxelCell::lossy_adapter_value(99),
        )
        .unwrap();

    let report =
        classify_support_mask(&target, &support, SupportDirection::new(2, -1).unwrap()).unwrap();
    assert_eq!(report.checked_cells, 4);
    assert!(report.has_checked_cells);
    assert_eq!(report.supported_cells, 1);
    assert_eq!(report.unsupported_cells, 1);
    assert_eq!(report.unknown_cells, 1);
    assert_eq!(report.lossy_cells, 1);
    assert!(!report.exact_support_mask_ready);
    assert!(!report.is_conservatively_supported());
    assert_eq!(report.cells[0].status, SupportCellStatus::Supported);
    assert_eq!(report.cells[1].status, SupportCellStatus::Unsupported);
    assert_eq!(report.cells[2].status, SupportCellStatus::Unknown);
    assert_eq!(report.cells[3].status, SupportCellStatus::Lossy);
    let paged_target =
        ChunkPagedSparseGrid::from_sparse_grid(&target, ChunkShape::new(0).unwrap()).unwrap();
    let paged_support =
        ChunkPagedSparseGrid::from_sparse_grid(&support, ChunkShape::new(0).unwrap()).unwrap();
    let paged_report = classify_chunk_paged_support_mask(
        &paged_target,
        &paged_support,
        SupportDirection::new(2, -1).unwrap(),
    )
    .unwrap();
    assert_eq!(paged_report.support, report);
    assert_eq!(paged_report.target_pages, 4);
    assert_eq!(paged_report.target_cells, 4);
    assert_eq!(paged_report.support_plane_probes, 0);
    assert_eq!(paged_report.support_page_hits, 3);
    assert_eq!(paged_report.support_page_misses, 1);
    assert_eq!(paged_report.cross_page_support_probes, 4);
    assert!(!paged_report.exact_paged_support_ready);
    assert!(matches!(
        SupportDirection::new(4, -1),
        Err(HypervoxelError::InvalidSupportDirection)
    ));

    let mut ready_support = SparseVoxelGrid::new(frame());
    ready_support
        .set(
            VoxelAddress::new(3, [1, 1, 0]).unwrap(),
            VoxelCell::material(MaterialRegionId(2)),
        )
        .unwrap();
    let mut ready_target = SparseVoxelGrid::new(frame());
    ready_target
        .set(
            VoxelAddress::new(3, [1, 1, 1]).unwrap(),
            VoxelCell::material(MaterialRegionId(1)),
        )
        .unwrap();
    let ready = classify_support_mask(
        &ready_target,
        &ready_support,
        SupportDirection::new(2, -1).unwrap(),
    )
    .unwrap();
    assert!(ready.has_checked_cells);
    assert!(ready.exact_support_mask_ready);
    assert!(ready.is_conservatively_supported());
    let ready_paged_target =
        ChunkPagedSparseGrid::from_sparse_grid(&ready_target, ChunkShape::new(0).unwrap()).unwrap();
    let ready_paged_support =
        ChunkPagedSparseGrid::from_sparse_grid(&ready_support, ChunkShape::new(0).unwrap())
            .unwrap();
    let ready_paged = classify_chunk_paged_support_mask(
        &ready_paged_target,
        &ready_paged_support,
        SupportDirection::new(2, -1).unwrap(),
    )
    .unwrap();
    assert_eq!(ready_paged.support, ready);
    assert_eq!(ready_paged.support_page_hits, 1);
    assert_eq!(ready_paged.support_page_misses, 0);
    assert!(ready_paged.exact_paged_support_ready);

    let mut plane_target = SparseVoxelGrid::new(frame());
    plane_target
        .set(
            VoxelAddress::new(3, [2, 2, 0]).unwrap(),
            VoxelCell::material(MaterialRegionId(3)),
        )
        .unwrap();
    let plane_paged_target =
        ChunkPagedSparseGrid::from_sparse_grid(&plane_target, ChunkShape::new(0).unwrap()).unwrap();
    let plane_paged = classify_chunk_paged_support_mask(
        &plane_paged_target,
        &ready_paged_support,
        SupportDirection::new(2, -1).unwrap(),
    )
    .unwrap();
    assert_eq!(plane_paged.support.support_plane_cells, 1);
    assert_eq!(plane_paged.support_plane_probes, 1);
    assert_eq!(plane_paged.support_page_hits, 0);
    assert!(plane_paged.exact_paged_support_ready);

    let empty = classify_support_mask(
        &SparseVoxelGrid::new(frame()),
        &ready_support,
        SupportDirection::new(2, -1).unwrap(),
    )
    .unwrap();
    assert_eq!(empty.checked_cells, 0);
    assert!(!empty.has_checked_cells);
    assert!(!empty.exact_support_mask_ready);
    assert!(!empty.is_conservatively_supported());
    let empty_paged_target = ChunkPagedSparseGrid::from_sparse_grid(
        &SparseVoxelGrid::new(frame()),
        ChunkShape::new(0).unwrap(),
    )
    .unwrap();
    let empty_paged = classify_chunk_paged_support_mask(
        &empty_paged_target,
        &ready_paged_support,
        SupportDirection::new(2, -1).unwrap(),
    )
    .unwrap();
    assert_eq!(empty_paged.support.checked_cells, 0);
    assert_eq!(empty_paged.target_pages, 0);
    assert!(!empty_paged.exact_paged_support_ready);

    let coarser_support = SparseVoxelGrid::new(GridFrame::builder().depth(2).build().unwrap());
    let coarser_support =
        ChunkPagedSparseGrid::from_sparse_grid(&coarser_support, ChunkShape::new(0).unwrap())
            .unwrap();
    assert_eq!(
        classify_chunk_paged_support_mask(
            &ready_paged_target,
            &coarser_support,
            SupportDirection::new(2, -1).unwrap(),
        )
        .unwrap_err(),
        HypervoxelError::MismatchedAddressDepth { left: 3, right: 2 }
    );
}
