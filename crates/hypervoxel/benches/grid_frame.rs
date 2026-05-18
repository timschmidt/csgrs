use criterion::{Criterion, criterion_group, criterion_main};
use hyperreal::{Rational, Real};
use hypervoxel::{
    AdapterNumericContract, AdapterToleranceStatus, AddressRay, AggregateCertainty,
    AxisPermutationTransform, ChunkPageSummary, ChunkShape, CompressedStorageKind,
    CompressedStorageManifest, DeterministicSnapshot, ExactAffineTransform, ExactBox,
    ExactConvexHalfSpaceSet, ExactHalfSpace, FieldAggregateFacts, FieldSampleId, FieldSampleRecord,
    FreshnessStatus, GridAabbHandoff, GridBasis, GridCoordinateSystem, GridFrame,
    GridFrameManifest, GridHandedness, GridSource, ImageStackContainer, ImageStackManifest,
    LegacyAdapterKind, LegacyAdapterStatus, LengthUnit, MaterialDisplayPalette, MaterialRegionId,
    PreparedSparseVoxelGridExt, PreparedVoxelGrid, PreviewExportFormat, PreviewExportManifest,
    PreviewScalarPolicy, ProcessGridArtifact, ProcessGridRole, QuantizationPolicy, QueryRegion,
    SignedAxis, SparseVoxelGrid, SupportDirection, SvoVoxelGrid, SweptVolumeProvenance,
    VoxelAddress, VoxelArtifactId, VoxelArtifactManifest, VoxelArtifactRole, VoxelCandidateKind,
    VoxelCandidateManifest, VoxelCell, VoxelChannelMapping, VoxelEditBatch, VoxelFieldCouplingKind,
    VoxelFieldCouplingManifest, VoxelHandoffDomain, VoxelHandoffManifest, VoxelIndexConvention,
    VoxelIoCompression, VoxelIoMetadata, VoxelMemoryBudgetManifest, VoxelSideTables,
    VoxelSliceNaming, VoxelSliceOrdering, VoxelSpatialAggregateFacts, VoxelTraceDimension,
    VoxelTraceManifest, VoxelizationAudit, VoxelizationPolicy, classify_support_mask,
    diff_sparse_grids, extract_exposed_faces, greedy_face_patch_plan,
    lookup_material_display_colors, lossy_obj_from_quad_mesh, lossy_quad_mesh_from_faces,
    query_field_samples, query_material_regions, sample_manhattan_distance_field,
    sample_signed_manhattan_distance_field, select_lod_cells, sweep_address_segment,
    trace_address_ray, voxelize_exact_box, voxelize_exact_convex_halfspace_set,
    voxelize_exact_halfspace,
};

fn r(n: i32) -> Real {
    n.into()
}

fn frame() -> GridFrame {
    GridFrame::builder()
        .origin([r(0), r(0), r(0)])
        .pitch([
            Rational::fraction(1, 64).unwrap().into(),
            Rational::fraction(1, 64).unwrap().into(),
            Rational::fraction(1, 64).unwrap().into(),
        ])
        .depth(6)
        .build()
        .unwrap()
}

fn bench_cell_bounds(c: &mut Criterion) {
    let frame = frame();
    let addresses = (0..64)
        .map(|i| VoxelAddress::new(6, [i, 63 - i, (i * 17) % 64]).unwrap())
        .collect::<Vec<_>>();

    c.bench_function("exact_cell_bounds_prime_addresses", |b| {
        b.iter(|| {
            addresses
                .iter()
                .map(|address| address.bounds(&frame).unwrap())
                .collect::<Vec<_>>()
        })
    });
    let manifest = GridFrameManifest {
        frame,
        basis: GridBasis::AxisAligned,
        handedness: GridHandedness::RightHanded,
        coordinate_system: GridCoordinateSystem::HyperGrid,
        chunk_shape: Some(ChunkShape::new(3).unwrap()),
    };
    c.bench_function("grid_frame_manifest_report", |b| {
        b.iter(|| manifest.report())
    });
}

fn bench_sparse_edits(c: &mut Criterion) {
    let frame = frame();
    c.bench_function("semantic_sparse_grid_edits", |b| {
        b.iter(|| {
            let mut grid = SparseVoxelGrid::new(frame.clone());
            for i in 0..64 {
                let address = VoxelAddress::new(6, [i, i, i]).unwrap();
                grid.set(address, VoxelCell::material(MaterialRegionId(i as u32)))
                    .unwrap();
            }
            grid.stored_aggregate()
        })
    });
}

fn bench_exact_box_voxelization(c: &mut Criterion) {
    let frame = GridFrame::builder()
        .origin([r(0), r(0), r(0)])
        .pitch([r(1), r(1), r(1)])
        .depth(4)
        .build()
        .unwrap();
    let exact_box = ExactBox::new([r(3), r(3), r(3)], [r(11), r(11), r(11)], None);

    c.bench_function("exact_box_voxelization_conservative_cover", |b| {
        b.iter(|| {
            voxelize_exact_box(
                frame.clone(),
                &exact_box,
                MaterialRegionId(1),
                VoxelizationPolicy::conservative_cover(),
            )
            .unwrap()
        })
    });

    let halfspace = ExactHalfSpace::new([r(1), r(0), r(0)], r(8), None);
    c.bench_function("exact_halfspace_voxelization_conservative_cover", |b| {
        b.iter(|| {
            voxelize_exact_halfspace(
                frame.clone(),
                &halfspace,
                MaterialRegionId(1),
                VoxelizationPolicy::conservative_cover(),
            )
            .unwrap()
        })
    });

    let solid = ExactConvexHalfSpaceSet::new(
        vec![
            ExactHalfSpace::new([r(-1), r(0), r(0)], r(-3), None),
            ExactHalfSpace::new([r(1), r(0), r(0)], r(11), None),
            ExactHalfSpace::new([r(0), r(-1), r(0)], r(-3), None),
            ExactHalfSpace::new([r(0), r(1), r(0)], r(11), None),
            ExactHalfSpace::new([r(0), r(0), r(-1)], r(-3), None),
            ExactHalfSpace::new([r(0), r(0), r(1)], r(11), None),
        ],
        None,
    );
    c.bench_function("exact_convex_halfspace_set_voxelization", |b| {
        b.iter(|| {
            voxelize_exact_convex_halfspace_set(
                frame.clone(),
                &solid,
                MaterialRegionId(1),
                VoxelizationPolicy::conservative_cover(),
            )
            .unwrap()
        })
    });
}

fn bench_svo_path_copy_edits(c: &mut Criterion) {
    let frame = GridFrame::builder().depth(6).build().unwrap();
    c.bench_function("semantic_svo_path_copy_edits", |b| {
        b.iter(|| {
            let mut grid = SvoVoxelGrid::new(frame.clone());
            for i in 0..64 {
                let address = VoxelAddress::new(6, [i, (i * 3) % 64, (i * 7) % 64]).unwrap();
                grid.set(
                    address,
                    VoxelCell::material(MaterialRegionId((i % 4) as u32)),
                )
                .unwrap();
            }
            grid.stats()
        })
    });
}

fn bench_exposed_face_extraction(c: &mut Criterion) {
    let frame = GridFrame::builder().depth(4).build().unwrap();
    let exact_box = ExactBox::new([r(3), r(3), r(3)], [r(11), r(11), r(11)], None);
    let (grid, _) = voxelize_exact_box(
        frame,
        &exact_box,
        MaterialRegionId(1),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();

    c.bench_function("exact_exposed_face_extraction", |b| {
        b.iter(|| extract_exposed_faces(&grid).unwrap())
    });
}

fn bench_connectivity_and_export_adapters(c: &mut Criterion) {
    let frame = GridFrame::builder().depth(4).build().unwrap();
    let exact_box = ExactBox::new([r(3), r(3), r(3)], [r(11), r(11), r(11)], None);
    let (grid, report) = voxelize_exact_box(
        frame,
        &exact_box,
        MaterialRegionId(1),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    let prepared =
        PreparedVoxelGrid::new(report.frame.clone(), grid.clone(), report.aggregate.clone());
    let seed = VoxelAddress::new(4, [3, 3, 3]).unwrap();

    c.bench_function("semantic_connected_component", |b| {
        b.iter(|| prepared.query_connected_component(seed).unwrap())
    });
    c.bench_function("lod_selection", |b| {
        b.iter(|| select_lod_cells(&grid, 2).unwrap())
    });
    c.bench_function("manhattan_distance_preview", |b| {
        b.iter(|| {
            sample_manhattan_distance_field(
                &grid,
                QueryRegion {
                    min: [0, 0, 0],
                    max: [3, 3, 0],
                    depth: 4,
                },
            )
            .unwrap()
        })
    });
    c.bench_function("signed_manhattan_distance_preview", |b| {
        b.iter(|| {
            sample_signed_manhattan_distance_field(
                &grid,
                QueryRegion {
                    min: [0, 0, 0],
                    max: [3, 3, 0],
                    depth: 4,
                },
            )
            .unwrap()
        })
    });
    c.bench_function("address_ray_trace", |b| {
        b.iter(|| {
            trace_address_ray(AddressRay {
                start: seed,
                axis: 0,
                direction: 1,
                max_steps: 8,
            })
            .unwrap()
        })
    });

    let faces = extract_exposed_faces(&grid).unwrap();
    c.bench_function("lossy_quad_mesh_from_exact_faces", |b| {
        b.iter(|| lossy_quad_mesh_from_faces(&faces, "bench preview").unwrap())
    });
    let mesh = lossy_quad_mesh_from_faces(&faces, "bench preview").unwrap();
    c.bench_function("lossy_obj_from_quad_mesh", |b| {
        b.iter(|| lossy_obj_from_quad_mesh(&mesh))
    });
    c.bench_function("greedy_face_patch_plan", |b| {
        b.iter(|| greedy_face_patch_plan(&faces, "bench preview"))
    });

    let side_tables = VoxelSideTables::default();
    c.bench_function("deterministic_binary_snapshot", |b| {
        b.iter(|| DeterministicSnapshot::binary_v1(&grid, &side_tables))
    });
    c.bench_function("voxelization_audit", |b| {
        b.iter(|| VoxelizationAudit::from_grid_and_report(&grid, &report))
    });
    c.bench_function("voxel_predicate_certificate_summary", |b| {
        b.iter(|| {
            (
                report.predicate_certificates.certified_cells(),
                report.predicate_certificates.classified_cells(),
            )
        })
    });
    c.bench_function("deterministic_run_length_snapshot", |b| {
        b.iter(|| DeterministicSnapshot::run_length_binary_v1(&grid))
    });
    c.bench_function("semantic_sparse_grid_diff", |b| {
        b.iter(|| diff_sparse_grids(&grid, &grid))
    });
}

fn bench_batches_field_facts_and_sweeps(c: &mut Criterion) {
    let frame = frame();
    c.bench_function("deterministic_sparse_edit_batch", |b| {
        b.iter(|| {
            let mut grid = SparseVoxelGrid::new(frame.clone());
            let mut batch = VoxelEditBatch::new();
            for i in 0..64 {
                batch.push(
                    VoxelAddress::new(6, [i, i, (i * 5) % 64]).unwrap(),
                    VoxelCell::material(MaterialRegionId((i % 8) as u32)),
                );
            }
            batch.apply_to(&mut grid).unwrap()
        })
    });

    let mut grid = SparseVoxelGrid::new(frame.clone());
    let mut side_tables = VoxelSideTables::default();
    for i in 0..32 {
        let id = FieldSampleId(i);
        grid.set(
            VoxelAddress::new(6, [i as u64, 0, 0]).unwrap(),
            VoxelCell::field_sample(id),
        )
        .unwrap();
        side_tables.insert_field_sample(
            id,
            FieldSampleRecord {
                label: format!("sample-{i}"),
                lower: Some(Real::from(i as i32)),
                upper: Some(Real::from(i as i32 + 1)),
                provenance: "bench".into(),
            },
        );
    }
    c.bench_function("field_sample_interval_aggregate", |b| {
        b.iter(|| FieldAggregateFacts::from_grid(&grid, &side_tables).unwrap())
    });
    c.bench_function("field_sample_side_table_query", |b| {
        b.iter(|| query_field_samples(&grid, &side_tables))
    });
    c.bench_function("material_region_side_table_query", |b| {
        b.iter(|| query_material_regions(&grid, &side_tables))
    });
    c.bench_function("material_display_color_lookup", |b| {
        let palette = MaterialDisplayPalette::default();
        b.iter(|| {
            lookup_material_display_colors(&query_material_regions(&grid, &side_tables), &palette)
        })
    });
    c.bench_function("chunk_page_summary", |b| {
        let shape = ChunkShape::new(3).unwrap();
        b.iter(|| ChunkPageSummary::from_addresses(shape, grid.iter().map(|(address, _)| *address)))
    });
    c.bench_function("process_grid_artifact_report", |b| {
        b.iter(|| {
            ProcessGridArtifact::new(
                ProcessGridRole::SweptVolumeCache,
                None,
                vec!["bench".into()],
                grid.stored_aggregate(),
            )
            .with_swept_volume(SweptVolumeProvenance {
                source: Some(GridSource::new("bench-path", 1)),
                tool_or_beam: Some("fixture".into()),
                exact_source_replay_available: true,
                broad_phase_only: true,
                quantization_policy: "conservative cover".into(),
            })
        })
    });
    let candidate_manifest = VoxelCandidateManifest {
        kind: VoxelCandidateKind::SupportOrProcessMask,
        freshness: FreshnessStatus::Current,
        aggregate_certainty: AggregateCertainty::Exact,
        unknown_count: 0,
        lossy_count: 0,
        exact_replay_available: true,
    };
    c.bench_function("voxel_candidate_report", |b| {
        b.iter(|| candidate_manifest.report())
    });
    let coupling_manifest = VoxelFieldCouplingManifest {
        kind: VoxelFieldCouplingKind::Thermal,
        freshness: FreshnessStatus::Current,
        aggregate: grid.stored_aggregate(),
        residual_replay_available: true,
        adapter_error_bound: None,
        missing_sample_records: 0,
    };
    c.bench_function("voxel_field_coupling_report", |b| {
        b.iter(|| coupling_manifest.report())
    });
    c.bench_function("support_mask_report", |b| {
        b.iter(|| classify_support_mask(&grid, &grid, SupportDirection::new(2, -1).unwrap()))
    });
    let artifact_manifest = VoxelArtifactManifest {
        id: VoxelArtifactId("bench:artifact".into()),
        role: VoxelArtifactRole::ProcessGrid,
        freshness: FreshnessStatus::Current,
        aggregate: grid.stored_aggregate(),
        storage_replay: hypervoxel::StorageReplayStatus::Exact,
        missing_side_table_links: 0,
        intended_domains: vec![VoxelHandoffDomain::Hyperpath],
    };
    c.bench_function("voxel_artifact_report", |b| {
        b.iter(|| artifact_manifest.report())
    });
    c.bench_function("voxel_spatial_aggregate_facts", |b| {
        b.iter(|| VoxelSpatialAggregateFacts::from_grid(&grid, None).unwrap())
    });
    let storage_manifest = CompressedStorageManifest {
        kind: CompressedStorageKind::SparseVoxelDag,
        stored_cells: grid.len(),
        physical_records: 16,
        chunk_shape: Some(ChunkShape::new(3).unwrap()),
        preserves_aggregate_facts: true,
        preserves_payload_ids: true,
        preserves_side_table_links: false,
    };
    c.bench_function("compressed_storage_manifest_report", |b| {
        b.iter(|| storage_manifest.report())
    });
    let memory_manifest = VoxelMemoryBudgetManifest {
        kind: CompressedStorageKind::SparseVoxelDag,
        estimated_bytes: 4096,
        budget_bytes: 2048,
        preserves_exact_semantics_when_over_budget: true,
    };
    c.bench_function("voxel_memory_budget_report", |b| {
        b.iter(|| memory_manifest.report())
    });
    let preview_manifest = PreviewExportManifest {
        format: PreviewExportFormat::Vtm,
        exact_input_primitives: grid.len(),
        exported_primitives: grid.len(),
        scalar_policy: PreviewScalarPolicy::ExactString,
        preserves_grid_topology: true,
        has_explicit_labels: true,
    };
    c.bench_function("preview_export_manifest_report", |b| {
        b.iter(|| preview_manifest.report())
    });
    let adapter_contract = AdapterNumericContract::primitive_float(
        LegacyAdapterStatus::lossy(LegacyAdapterKind::PreviewRenderer, "bench display epsilon"),
        Some(r(1)),
        Some(Rational::fraction(1, 1024).unwrap().into()),
        Some(Rational::fraction(1, 512).unwrap().into()),
        AdapterToleranceStatus::Explicit,
    );
    c.bench_function("adapter_numeric_contract_report", |b| {
        b.iter(|| adapter_contract.report())
    });
    let handoff_manifest = VoxelHandoffManifest {
        domain: VoxelHandoffDomain::Hyperpath,
        source: Some(GridSource::new("bench-grid", 1)),
        expected_source: Some(GridSource::new("bench-grid", 1)),
        required_side_table_links: 1,
        supplied_side_table_links: 1,
        aggregate: grid.stored_aggregate(),
    };
    c.bench_function("voxel_domain_handoff_report", |b| {
        b.iter(|| handoff_manifest.report())
    });
    c.bench_function("lattice_aabb_handoff", |b| {
        b.iter(|| {
            GridAabbHandoff::from_address(grid.frame(), VoxelAddress::new(6, [1, 2, 3]).unwrap())
                .unwrap()
                .into_lattice()
                .vector_facts()
        })
    });
    let trace_manifest = VoxelTraceManifest {
        operation: "bench-grid".into(),
        dimensions: vec![
            VoxelTraceDimension::GridFrameConstruction,
            VoxelTraceDimension::OccupancyAggregatePropagation,
            VoxelTraceDimension::PreparedQuery,
            VoxelTraceDimension::DomainHandoffReport,
        ],
        exact_predicate_count: 64,
        lossy_adapter_count: 0,
        unknown_count: 0,
    };
    c.bench_function("voxel_trace_report", |b| b.iter(|| trace_manifest.report()));
    let role_policies = [
        VoxelizationPolicy {
            quantization: QuantizationPolicy::UnsignedDistanceSampling,
            boundary: hypervoxel::BoundaryPolicy::KeepBoundary,
        },
        VoxelizationPolicy {
            quantization: QuantizationPolicy::SignedDistanceSampling,
            boundary: hypervoxel::BoundaryPolicy::KeepBoundary,
        },
        VoxelizationPolicy {
            quantization: QuantizationPolicy::MaterialRegionRasterization,
            boundary: hypervoxel::BoundaryPolicy::KeepBoundary,
        },
        VoxelizationPolicy {
            quantization: QuantizationPolicy::ProcessExposureGrid,
            boundary: hypervoxel::BoundaryPolicy::BoundaryAsUnknown,
        },
    ];
    c.bench_function("voxelization_policy_role_checks", |b| {
        b.iter(|| {
            role_policies
                .iter()
                .map(|policy| {
                    (
                        policy.is_exact_semantic_role(),
                        policy.is_occupancy_policy(),
                    )
                })
                .collect::<Vec<_>>()
        })
    });
    let manifest = ImageStackManifest {
        container: ImageStackContainer::ZippedPng,
        slices: 32,
        channels: 1,
        bit_depth: 16,
        channel_mappings: vec![VoxelChannelMapping::FieldSample],
        metadata: VoxelIoMetadata {
            dimensions: Some([64, 64, 32]),
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
            compression: VoxelIoCompression::Zip,
        },
        source: Some(GridSource::new("bench-stack", 1)),
        expected_source: Some(GridSource::new("bench-stack", 1)),
        required_side_table_links: 1,
        supplied_side_table_links: 1,
    };
    c.bench_function("image_stack_manifest_report", |b| {
        b.iter(|| manifest.report())
    });

    let prepared = PreparedVoxelGrid::new(frame, grid.clone(), grid.stored_aggregate());
    let start = VoxelAddress::new(6, [0, 0, 0]).unwrap();
    let end = VoxelAddress::new(6, [31, 0, 0]).unwrap();
    c.bench_function("prepared_query_report", |b| {
        b.iter(|| prepared.prepared_query_report(true).unwrap())
    });
    c.bench_function("address_segment_sweep", |b| {
        b.iter(|| sweep_address_segment(&prepared, start, end).unwrap())
    });

    let transform = AxisPermutationTransform::new(
        [
            SignedAxis::new(1, 1).unwrap(),
            SignedAxis::new(0, -1).unwrap(),
            SignedAxis::new(2, 1).unwrap(),
        ],
        [r(10), r(20), r(30)],
    )
    .unwrap();
    let bounds = start.bounds(&prepared.frame).unwrap();
    c.bench_function("signed_axis_bounds_transform", |b| {
        b.iter(|| transform.map_bounds(&bounds).unwrap())
    });
    let affine = ExactAffineTransform::new(
        [[r(1), r(1), r(0)], [r(0), r(1), r(0)], [r(0), r(0), r(1)]],
        [r(10), r(20), r(30)],
    );
    c.bench_function("exact_affine_bounds_transform", |b| {
        b.iter(|| affine.map_bounds(&bounds).unwrap())
    });
}

criterion_group!(
    benches,
    bench_cell_bounds,
    bench_sparse_edits,
    bench_exact_box_voxelization,
    bench_svo_path_copy_edits,
    bench_exposed_face_extraction,
    bench_connectivity_and_export_adapters,
    bench_batches_field_facts_and_sweeps
);
criterion_main!(benches);
