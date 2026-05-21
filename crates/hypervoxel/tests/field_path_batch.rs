use hyperreal::{Rational, Real};
use hypervoxel::{
    AddressRay, AggregateCertainty, AxisPermutationTransform, CertifiedFieldInterval,
    CertifiedTensorInterval, CertifiedVectorInterval, ExactAffineTransform, FieldAggregateFacts,
    FieldEnvelopeFacts, FieldSampleId, FieldSampleRecord, FreshnessStatus, GridFrame, GridSource,
    HypervoxelError, MaterialRegionId, OccupancyState, PreparedVoxelGrid, ProcessGridArtifact,
    ProcessGridRole, SignedAxis, SparseVoxelGrid, SupportCellStatus, SupportDirection,
    SweptVolumeProvenance, VoxelAddress, VoxelCandidateKind, VoxelCandidateManifest, VoxelCell,
    VoxelEditBatch, VoxelFieldCouplingKind, VoxelFieldCouplingManifest, VoxelSideTables,
    classify_support_mask, query_field_samples, sweep_address_segment, trace_address_ray,
    trace_address_segment,
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
            provenance: "fixture".into(),
        },
    );
    side_tables.insert_field_sample(
        FieldSampleId(2),
        FieldSampleRecord {
            label: "dose high".into(),
            lower: Some(rf(1, 2)),
            upper: Some(rf(5, 4)),
            provenance: "fixture".into(),
        },
    );

    let facts = FieldAggregateFacts::from_grid(&grid, &side_tables).unwrap();
    assert_eq!(facts.sample_cell_count, 2);
    assert!(facts.has_field_samples);
    assert_eq!(facts.certainty, AggregateCertainty::Certified);
    assert!(facts.certified_field_bounds_ready);
    let interval = facts.interval.unwrap();
    assert_eq!(interval.lower, rf(1, 4));
    let ball = interval.enclosing_ball();
    assert_eq!(ball.center, rf(3, 4));
    assert_eq!(ball.radius, rf(1, 2));
    assert_eq!(facts.missing_records, 0);
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
            provenance: "fixture".into(),
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
            provenance: "antagonistic".into(),
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

    let reports = batch.apply_to(&mut grid).unwrap();
    assert_eq!(reports.len(), 2);
    assert!(reports.iter().all(|report| report.frame_validated));
    assert!(reports[0].stored_explicit_cell);
    assert!(reports[0].exact_edit_replay_ready);
    assert!(!reports[0].removed_explicit_cell);
    assert!(reports[1].removed_explicit_cell);
    assert!(reports[1].exact_edit_replay_ready);
    assert!(!reports[1].semantic_noop);
    assert_eq!(reports[0].current.occupancy, OccupancyState::Filled);
    assert_eq!(
        reports[1].previous.unwrap().occupancy,
        OccupancyState::Filled
    );
    assert!(grid.is_empty());

    let mut replay_grid = SparseVoxelGrid::new(frame());
    let summary = batch.apply_with_report(&mut replay_grid).unwrap();
    assert_eq!(summary.applied_edits, 2);
    assert!(summary.has_applied_edits);
    assert_eq!(summary.stored_explicit_cells, 1);
    assert_eq!(summary.removed_explicit_cells, 1);
    assert_eq!(summary.semantic_noops, 0);
    assert_eq!(summary.non_exact_current_cells, 0);
    assert!(summary.exact_batch_replay_ready);
    assert!(replay_grid.is_empty());

    let mut uncertain_grid = SparseVoxelGrid::new(frame());
    let mut uncertain_batch = VoxelEditBatch::new();
    uncertain_batch.push(address, VoxelCell::unknown());
    uncertain_batch.push(
        VoxelAddress::new(3, [2, 1, 1]).unwrap(),
        VoxelCell::lossy_adapter_value(11),
    );
    let uncertain_summary = uncertain_batch
        .apply_with_report(&mut uncertain_grid)
        .unwrap();
    assert_eq!(uncertain_summary.non_exact_current_cells, 2);
    assert!(
        uncertain_summary
            .edits
            .iter()
            .all(|edit| !edit.exact_edit_replay_ready)
    );
    assert!(!uncertain_summary.exact_batch_replay_ready);

    let empty_batch = VoxelEditBatch::new();
    let empty_summary = empty_batch
        .apply_with_report(&mut SparseVoxelGrid::new(frame()))
        .unwrap();
    assert_eq!(empty_summary.applied_edits, 0);
    assert!(!empty_summary.has_applied_edits);
    assert_eq!(empty_summary.non_exact_current_cells, 0);
    assert!(!empty_summary.exact_batch_replay_ready);
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
    let aggregate = grid.stored_aggregate();
    let prepared = PreparedVoxelGrid::new(frame(), grid, aggregate);
    let start = VoxelAddress::new(3, [1, 1, 1]).unwrap();
    let end = VoxelAddress::new(3, [3, 1, 1]).unwrap();

    let sweep = sweep_address_segment(&prepared, start, end).unwrap();
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
fn process_grid_artifact_carries_role_provenance_and_aggregate_without_domain_laws() {
    let aggregate = hypervoxel::VoxelAggregateFacts::from_cells([&VoxelCell::process_state(
        hypervoxel::ProcessStateId(4),
    )]);
    let artifact = ProcessGridArtifact::new(
        ProcessGridRole::PhotopolymerDose,
        Some(GridSource::new("exposure:path:17", 3)),
        vec!["resin-lot-a".into(), "405nm".into()],
        aggregate,
    );

    assert_eq!(artifact.role, ProcessGridRole::PhotopolymerDose);
    assert_eq!(artifact.source.unwrap().version, 3);
    assert_eq!(artifact.process_tags.len(), 2);
    assert!(artifact.aggregate.all_filled);
}

#[test]
fn swept_volume_provenance_rejects_broad_phase_cache_as_path_truth() {
    let aggregate = hypervoxel::VoxelAggregateFacts::from_cells([&VoxelCell::process_state(
        hypervoxel::ProcessStateId(4),
    )]);
    let artifact = ProcessGridArtifact::new(
        ProcessGridRole::SweptVolumeCache,
        Some(GridSource::new("toolpath:roughing", 9)),
        vec!["6mm endmill".into()],
        aggregate,
    )
    .with_swept_volume(SweptVolumeProvenance {
        source: Some(GridSource::new("toolpath:roughing", 9)),
        expected_source_version: Some(9),
        tool_or_beam: Some("6mm endmill".into()),
        exact_source_replay_available: true,
        broad_phase_only: true,
        quantization_policy: "conservative cover, keep boundary".into(),
    });

    let swept = artifact.swept_volume.as_ref().unwrap().report();
    assert_eq!(swept.source_freshness, FreshnessStatus::Current);
    assert!(swept.has_tool_or_beam);
    assert!(swept.has_quantization_policy);
    assert!(swept.exact_source_replay_available);
    assert!(swept.broad_phase_only);
    assert!(!swept.can_stand_in_for_exact_path);

    let stale = SweptVolumeProvenance {
        source: Some(GridSource::new("toolpath:roughing", 8)),
        expected_source_version: Some(9),
        tool_or_beam: Some("6mm endmill".into()),
        exact_source_replay_available: true,
        broad_phase_only: false,
        quantization_policy: "conservative cover, keep boundary".into(),
    }
    .report();
    assert_eq!(stale.source_freshness, FreshnessStatus::Stale);
    assert!(!stale.can_stand_in_for_exact_path);

    let exact_ready = SweptVolumeProvenance {
        source: Some(GridSource::new("toolpath:finishing", 2)),
        expected_source_version: Some(2),
        tool_or_beam: Some("1mm ball endmill".into()),
        exact_source_replay_available: true,
        broad_phase_only: false,
        quantization_policy: "exact source replay required".into(),
    }
    .report();
    assert!(exact_ready.can_stand_in_for_exact_path);

    let missing_tool = SweptVolumeProvenance {
        source: Some(GridSource::new("toolpath:finishing", 2)),
        expected_source_version: Some(2),
        tool_or_beam: Some(" ".into()),
        exact_source_replay_available: true,
        broad_phase_only: false,
        quantization_policy: "exact source replay required".into(),
    }
    .report();
    assert!(!missing_tool.has_tool_or_beam);
    assert!(!missing_tool.can_stand_in_for_exact_path);

    let missing_policy = SweptVolumeProvenance {
        source: Some(GridSource::new("toolpath:finishing", 2)),
        expected_source_version: Some(2),
        tool_or_beam: Some("1mm ball endmill".into()),
        exact_source_replay_available: true,
        broad_phase_only: false,
        quantization_policy: String::new(),
    }
    .report();
    assert!(!missing_policy.has_quantization_policy);
    assert!(!missing_policy.can_stand_in_for_exact_path);
}

#[test]
fn voxel_candidate_requires_fresh_exact_replay_before_promotion() {
    let exact = VoxelCandidateManifest {
        kind: VoxelCandidateKind::CompressionOrLodPolicy,
        freshness: FreshnessStatus::Current,
        aggregate_certainty: AggregateCertainty::Exact,
        unknown_count: 0,
        lossy_count: 0,
        exact_replay_available: true,
        exact_evidence_count: 3,
    }
    .report();
    assert!(exact.has_exact_evidence);
    assert!(exact.promotable_as_exact);

    let vacuous = VoxelCandidateManifest {
        kind: VoxelCandidateKind::CompressionOrLodPolicy,
        freshness: FreshnessStatus::Current,
        aggregate_certainty: AggregateCertainty::Exact,
        unknown_count: 0,
        lossy_count: 0,
        exact_replay_available: true,
        exact_evidence_count: 0,
    }
    .report();
    assert!(!vacuous.has_exact_evidence);
    assert!(!vacuous.promotable_as_exact);

    let stale_lossy = VoxelCandidateManifest {
        kind: VoxelCandidateKind::ExposureDoseSchedule,
        freshness: FreshnessStatus::Stale,
        aggregate_certainty: AggregateCertainty::Lossy,
        unknown_count: 1,
        lossy_count: 1,
        exact_replay_available: true,
        exact_evidence_count: 1,
    }
    .report();
    assert!(!stale_lossy.promotable_as_exact);
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
}

#[test]
fn field_coupling_requires_residual_replay_or_adapter_error_report() {
    let exact_aggregate =
        hypervoxel::VoxelAggregateFacts::from_cells([&VoxelCell::field_sample(FieldSampleId(1))]);
    let exact = VoxelFieldCouplingManifest {
        kind: VoxelFieldCouplingKind::Electromagnetic,
        freshness: FreshnessStatus::Current,
        aggregate: exact_aggregate,
        residual_replay_available: true,
        adapter_error_bound: None,
        missing_sample_records: 0,
    }
    .report();
    assert!(exact.usable_as_exact_residual_evidence);
    assert!(!exact.has_adapter_error_bound);
    assert!(!exact.certified_adapter_error_bound_ready);
    assert!(!exact.requires_error_bounded_adapter);

    let uncertain_aggregate = hypervoxel::VoxelAggregateFacts::from_cells([&VoxelCell::unknown()]);
    let adapter = VoxelFieldCouplingManifest {
        kind: VoxelFieldCouplingKind::Thermal,
        freshness: FreshnessStatus::Current,
        aggregate: uncertain_aggregate,
        residual_replay_available: false,
        adapter_error_bound: Some(rf(1, 1000)),
        missing_sample_records: 1,
    }
    .report();
    assert!(!adapter.usable_as_exact_residual_evidence);
    assert!(adapter.has_adapter_error_bound);
    assert!(adapter.adapter_error_bound_non_negative);
    assert!(adapter.certified_adapter_error_bound_ready);
    assert!(adapter.requires_error_bounded_adapter);

    let invalid_bound = VoxelFieldCouplingManifest {
        kind: VoxelFieldCouplingKind::Optical,
        freshness: FreshnessStatus::Current,
        aggregate: hypervoxel::VoxelAggregateFacts::from_cells([&VoxelCell::field_sample(
            FieldSampleId(2),
        )]),
        residual_replay_available: false,
        adapter_error_bound: Some(r(-1)),
        missing_sample_records: 0,
    }
    .report();
    assert!(invalid_bound.has_adapter_error_bound);
    assert!(!invalid_bound.adapter_error_bound_non_negative);
    assert!(!invalid_bound.certified_adapter_error_bound_ready);
    assert!(!invalid_bound.usable_as_exact_residual_evidence);
    assert!(!invalid_bound.requires_error_bounded_adapter);
}
