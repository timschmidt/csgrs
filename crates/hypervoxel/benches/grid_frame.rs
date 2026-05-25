use criterion::{Criterion, criterion_group, criterion_main};
use hyperreal::{Rational, Real};
use hypervoxel::{
    AdapterNumericContract, AdapterToleranceStatus, AddressRay, AggregateCertainty,
    AxisPermutationTransform, CertifiedFieldInterval, CertifiedVectorInterval, ChunkAddress,
    ChunkPageSummary, ChunkShape, CompressedStorageKind, CompressedStorageManifest,
    ContinuousFieldVoxelCell, ContinuousFieldVoxelInterchangeManifest,
    ContinuousFieldVoxelManifest, ContinuousFieldVoxelRowOrder, DeterministicSnapshot, ExactAabb3,
    ExactAffineTransform, ExactBox, ExactConvexHalfSpaceSet, ExactHalfSpace, ExactTriangle3,
    ExactTriangleSurfaceMesh, FieldAggregateFacts, FieldEnvelopeFacts, FieldSampleId,
    FieldSampleRecord, FreshnessStatus, GridAabbHandoff, GridBasis, GridCoordinateSystem,
    GridFrame, GridFrameManifest, GridHandedness, GridSource, ImageStackContainer,
    ImageStackManifest, LegacyAdapterKind, LegacyAdapterStatus, LengthUnit, MaterialDisplayPalette,
    MaterialRegionId, MaterialRegionRecord, PreparedSparseVoxelGridExt, PreparedVoxelGrid,
    PreviewExportFormat, PreviewExportManifest, PreviewScalarPolicy, ProcessGridArtifact,
    ProcessGridRole, QuantizationPolicy, QueryRegion, SignedAxis, SparseVoxelGrid,
    SupportDirection, SvoVoxelGrid, SweptVolumeProvenance, VoxelAddress, VoxelArtifactId,
    VoxelArtifactManifest, VoxelArtifactRole, VoxelCandidateKind, VoxelCandidateManifest,
    VoxelCell, VoxelChannelMapping, VoxelEditBatch, VoxelFieldCouplingKind,
    VoxelFieldCouplingManifest, VoxelHandoffDomain, VoxelHandoffManifest, VoxelIndexConvention,
    VoxelIoCompression, VoxelIoMetadata, VoxelMemoryBudgetManifest, VoxelSideTables,
    VoxelSliceNaming, VoxelSliceOrdering, VoxelSpatialAggregateFacts, VoxelTraceDimension,
    VoxelTraceManifest, VoxelizationAudit, VoxelizationPolicy, classify_support_mask,
    continuous_field_address, diff_sparse_grids, extract_exposed_faces,
    extract_exposed_faces_with_report, greedy_face_patch_plan, lookup_material_display_colors,
    lossy_obj_from_quad_mesh, lossy_quad_mesh_from_faces, query_field_samples,
    query_material_regions, report_material_region_metadata, sample_manhattan_distance_field,
    sample_signed_manhattan_distance_field, select_lod_cells, sweep_address_segment,
    trace_address_ray, voxelize_exact_box, voxelize_exact_convex_halfspace_set,
    voxelize_exact_halfspace, voxelize_exact_triangle_surface_mesh,
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
    c.bench_function("occupancy_interval_from_cells", |b| {
        b.iter(|| {
            let cells = (0..64)
                .map(|i| {
                    if i % 3 == 0 {
                        VoxelCell::material(MaterialRegionId(1))
                    } else if i % 3 == 1 {
                        VoxelCell::boundary(hypervoxel::VoxelPayload::MaterialRegion(
                            MaterialRegionId(1),
                        ))
                    } else {
                        VoxelCell::empty()
                    }
                })
                .collect::<Vec<_>>();
            hypervoxel::VoxelAggregateFacts::from_cells(cells.iter()).occupancy_interval
        })
    });
    let cell = VoxelCell::material(MaterialRegionId(1));
    c.bench_function("voxel_cell_semantic_report", |b| b.iter(|| cell.report()));
    c.bench_function("finite_frame_aggregate_from_sparse_cells", |b| {
        b.iter(|| {
            let cells = (0..64)
                .map(|_| VoxelCell::material(MaterialRegionId(1)))
                .collect::<Vec<_>>();
            hypervoxel::VoxelAggregateFacts::from_explicit_cells_in_frame(4096, cells.iter())
                .unwrap()
                .occupancy_interval
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
    c.bench_function("exact_box_source_report", |b| b.iter(|| exact_box.report()));
    c.bench_function("exact_halfspace_source_report", |b| {
        b.iter(|| halfspace.report())
    });
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
    c.bench_function("exact_convex_halfspace_set_source_report", |b| {
        b.iter(|| solid.report())
    });
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
    let triangle_mesh = ExactTriangleSurfaceMesh::new(
        vec![
            ExactTriangle3::new(
                [[r(4), r(4), r(8)], [r(12), r(4), r(8)], [r(4), r(12), r(8)]],
                Some(0),
            ),
            ExactTriangle3::new(
                [
                    [r(12), r(4), r(8)],
                    [r(12), r(12), r(8)],
                    [r(4), r(12), r(8)],
                ],
                Some(1),
            ),
        ],
        frame.source().cloned(),
        true,
    );
    c.bench_function("exact_triangle_surface_mesh_voxelization", |b| {
        b.iter(|| {
            voxelize_exact_triangle_surface_mesh(
                frame.clone(),
                &triangle_mesh,
                MaterialRegionId(9),
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
    c.bench_function("semantic_svo_storage_report", |b| {
        b.iter(|| {
            let mut grid = SvoVoxelGrid::new(frame.clone());
            for i in 0..64 {
                let address = VoxelAddress::new(6, [i, (i * 3) % 64, (i * 7) % 64]).unwrap();
                grid.set_with_report(
                    address,
                    VoxelCell::material(MaterialRegionId((i % 4) as u32)),
                )
                .unwrap();
            }
            let report = grid.report();
            (
                report.has_materialized_evidence,
                report.root_aggregate_covers_frame,
                report.exact_dag_replay_ready,
                report.stats.nodes,
            )
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
    c.bench_function("exact_exposed_face_extraction_report", |b| {
        b.iter(|| {
            let report = extract_exposed_faces_with_report(&grid).unwrap();
            (
                report.exact_faces,
                report.has_exact_faces,
                report.exact_shell_ready,
            )
        })
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
        b.iter(|| {
            let query = prepared.query_connected_component(seed).unwrap();
            (
                query.addresses.len(),
                query.has_reached_cells,
                query.exact_component_ready,
            )
        })
    });
    c.bench_function("lod_selection", |b| {
        b.iter(|| {
            let report = select_lod_cells(&grid, 2).unwrap();
            (
                report.selected_cells,
                report.has_selected_cells,
                report.certified_lod_aggregate_ready,
            )
        })
    });
    let lod_report = select_lod_cells(&grid, 2).unwrap();
    c.bench_function("lod_selected_aggregate_report", |b| {
        b.iter(|| lod_report.selected_aggregate())
    });
    c.bench_function("manhattan_distance_preview", |b| {
        b.iter(|| {
            let preview = sample_manhattan_distance_field(
                &grid,
                QueryRegion {
                    min: [0, 0, 0],
                    max: [3, 3, 0],
                    depth: 4,
                },
            )
            .unwrap();
            (
                preview.samples.len(),
                preview.source_cells,
                preview.has_distance_source,
                preview.exact_address_distance_ready,
            )
        })
    });
    c.bench_function("signed_manhattan_distance_preview", |b| {
        b.iter(|| {
            let preview = sample_signed_manhattan_distance_field(
                &grid,
                QueryRegion {
                    min: [0, 0, 0],
                    max: [3, 3, 0],
                    depth: 4,
                },
            )
            .unwrap();
            (
                preview.samples.len(),
                preview.source_cells,
                preview.has_distance_source,
                preview.exact_address_distance_ready,
                preview.continuous_sdf_ready,
            )
        })
    });
    c.bench_function("address_ray_trace", |b| {
        b.iter(|| {
            let trace = trace_address_ray(AddressRay {
                start: seed,
                axis: 0,
                direction: 1,
                max_steps: 8,
            })
            .unwrap();
            (
                trace.exact_address_trace_ready,
                trace.stopped_at_boundary,
                trace.stopped_at_step_limit,
            )
        })
    });
    let query_aabb = ExactAabb3 {
        min: [r(0), r(0), r(0)],
        max: [r(4), r(4), r(4)],
    };
    c.bench_function("prepared_aabb_broad_phase", |b| {
        b.iter(|| {
            let report = prepared.query_aabb_broad_phase(&query_aabb).unwrap();
            (
                report.tested_cells,
                report.has_tested_cells,
                report.certified_broad_phase_ready,
                report.candidates.len(),
            )
        })
    });

    let faces = extract_exposed_faces(&grid).unwrap();
    c.bench_function("lossy_quad_mesh_from_exact_faces", |b| {
        b.iter(|| {
            let mesh = lossy_quad_mesh_from_faces(&faces, "bench preview").unwrap();
            (
                mesh.report.exact_face_identity_preserved,
                mesh.report.display_only,
                mesh.report.exact_geometry_replay_ready,
            )
        })
    });
    let mesh = lossy_quad_mesh_from_faces(&faces, "bench preview").unwrap();
    c.bench_function("lossy_obj_from_quad_mesh", |b| {
        b.iter(|| {
            let obj = lossy_obj_from_quad_mesh(&mesh);
            (obj.vertex_records, obj.face_records, obj.preview_only)
        })
    });
    c.bench_function("greedy_face_patch_plan", |b| {
        b.iter(|| greedy_face_patch_plan(&faces, "bench preview"))
    });

    let side_tables = VoxelSideTables::default();
    c.bench_function("deterministic_binary_snapshot", |b| {
        b.iter(|| {
            let snapshot = DeterministicSnapshot::binary_v1(&grid, &side_tables);
            (
                snapshot.report().byte_len,
                snapshot.report().serialized_cell_records,
                snapshot.report().has_cell_records,
                snapshot.report().exact_snapshot_replay_ready,
            )
        })
    });
    c.bench_function("voxelization_audit", |b| {
        b.iter(|| {
            let audit = VoxelizationAudit::from_grid_and_report(&grid, &report);
            (
                audit.exact_audit_ready,
                audit.exact_adapter_replay,
                audit.predicate_certified_cells,
                audit.predicate_unknown_cells,
            )
        })
    });
    c.bench_function("voxel_predicate_certificate_summary", |b| {
        b.iter(|| {
            (
                report.predicate_certificates.certified_cells(),
                report.predicate_certificates.classified_cells(),
                report.predicate_certificates.has_classified_cells(),
                report.predicate_certificates.is_fully_certified(),
                report.exact_topology_ready(),
                report.source_replay_ready(),
            )
        })
    });
    c.bench_function("deterministic_run_length_snapshot", |b| {
        b.iter(|| {
            let snapshot = DeterministicSnapshot::run_length_binary_v1(&grid);
            (
                snapshot.report().byte_len,
                snapshot.report().serialized_cell_records,
                snapshot.report().has_cell_records,
                snapshot.report().exact_snapshot_replay_ready,
            )
        })
    });
    c.bench_function("semantic_sparse_grid_diff", |b| {
        b.iter(|| {
            let report = diff_sparse_grids(&grid, &grid);
            (
                report.semantic_equivalence_ready,
                report.has_compared_addresses,
                report.frame_matches,
                report.mismatch_count,
                report.compared_addresses,
            )
        })
    });
    #[cfg(feature = "legacy-voxelis")]
    {
        use voxelis::{
            MaxDepth, VoxInterner,
            spatial::{VoxOpsWrite, VoxTree},
        };

        let mut interner = VoxInterner::<u8>::with_memory_budget(4096);
        let mut legacy_tree = VoxTree::<u8>::new(MaxDepth::new(4));
        let samples = [VoxelAddress::new(4, [1, 1, 1]).unwrap()];
        let _ = legacy_tree.set(&mut interner, glam::IVec3::new(1, 1, 1), 7);
        let mut expected = SparseVoxelGrid::new(GridFrame::builder().depth(4).build().unwrap());
        expected
            .set(samples[0], VoxelCell::material(MaterialRegionId(7)))
            .unwrap();
        c.bench_function("legacy_voxelis_u8_storage_diff", |b| {
            b.iter(|| {
                let report = hypervoxel::compare_legacy_voxelis_u8_samples(
                    &legacy_tree,
                    &interner,
                    &expected,
                    samples,
                )
                .unwrap();
                (
                    report.sampled_storage_equivalence_ready,
                    report.has_compared_addresses,
                    report.exact_voxelization_ready,
                    report.mismatch_count,
                )
            })
        });
    }
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
            let report = batch.apply_with_report(&mut grid).unwrap();
            (
                report.applied_edits,
                report.has_applied_edits,
                report.stored_explicit_cells,
                report.non_exact_current_cells,
                report.exact_batch_replay_ready,
            )
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
    grid.set(
        VoxelAddress::new(6, [0, 1, 0]).unwrap(),
        VoxelCell::material(MaterialRegionId(99)),
    )
    .unwrap();
    side_tables.insert_material(
        MaterialRegionId(99),
        MaterialRegionRecord {
            label: "bench material".into(),
            density: Some(Rational::fraction(117, 100).unwrap().into()),
            provenance: "bench".into(),
        },
    );
    c.bench_function("field_sample_interval_aggregate", |b| {
        b.iter(|| {
            let facts = FieldAggregateFacts::from_grid(&grid, &side_tables).unwrap();
            (
                facts.sample_cell_count,
                facts.has_field_samples,
                facts.certified_field_bounds_ready,
                facts.certainty,
            )
        })
    });
    let scalar_envelope = CertifiedFieldInterval {
        lower: Real::from(0),
        upper: Real::from(31),
    };
    let vector_envelope = CertifiedVectorInterval {
        components: vec![scalar_envelope.clone(), scalar_envelope.clone()],
    };
    c.bench_function("field_vector_envelope_facts", |b| {
        b.iter(|| {
            let facts = FieldEnvelopeFacts::from_envelopes(
                [&vector_envelope],
                std::iter::empty::<&hypervoxel::CertifiedTensorInterval>(),
            )
            .unwrap();
            (
                facts.envelope_count,
                facts.has_envelopes,
                facts.certified_envelope_ready,
                facts.certainty,
            )
        })
    });
    c.bench_function("field_sample_side_table_query", |b| {
        b.iter(|| query_field_samples(&grid, &side_tables))
    });
    c.bench_function("material_region_side_table_query", |b| {
        b.iter(|| query_material_regions(&grid, &side_tables))
    });
    c.bench_function("material_region_metadata_report", |b| {
        b.iter(|| {
            let query = query_material_regions(&grid, &side_tables);
            let report = report_material_region_metadata(&query, &side_tables);
            (
                report.has_material_regions,
                report.resolved_records,
                report.is_complete(),
                report.certainty,
            )
        })
    });
    c.bench_function("material_display_color_lookup", |b| {
        let palette = MaterialDisplayPalette::default();
        b.iter(|| {
            let report = lookup_material_display_colors(
                &query_material_regions(&grid, &side_tables),
                &palette,
            );
            (
                report.has_material_regions,
                report.resolved_colors,
                report.complete_display_palette_ready,
            )
        })
    });
    c.bench_function("chunk_page_summary", |b| {
        let shape = ChunkShape::new(3).unwrap();
        b.iter(|| {
            let report =
                ChunkPageSummary::from_addresses(shape, grid.iter().map(|(address, _)| *address));
            (
                report.exact_integer_partition,
                report.has_stored_cells,
                report.exact_page_cover_ready,
                report.page_capacity_cells,
            )
        })
    });
    c.bench_function("chunk_local_address_split", |b| {
        let shape = ChunkShape::new(3).unwrap();
        b.iter(|| {
            grid.iter()
                .map(|(address, _)| ChunkAddress::split(*address, shape))
                .collect::<Vec<_>>()
        })
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
                expected_source_version: Some(1),
                tool_or_beam: Some("fixture".into()),
                exact_source_replay_available: true,
                broad_phase_only: true,
                quantization_policy: "conservative cover".into(),
            })
        })
    });
    let swept_volume = SweptVolumeProvenance {
        source: Some(GridSource::new("bench-path", 1)),
        expected_source_version: Some(1),
        tool_or_beam: Some("fixture".into()),
        exact_source_replay_available: true,
        broad_phase_only: false,
        quantization_policy: "exact source replay required".into(),
    };
    c.bench_function("swept_volume_freshness_report", |b| {
        b.iter(|| {
            let report = swept_volume.report();
            (
                report.has_tool_or_beam,
                report.has_quantization_policy,
                report.can_stand_in_for_exact_path,
            )
        })
    });
    let candidate_manifest = VoxelCandidateManifest {
        kind: VoxelCandidateKind::SupportOrProcessMask,
        freshness: FreshnessStatus::Current,
        aggregate_certainty: AggregateCertainty::Exact,
        unknown_count: 0,
        lossy_count: 0,
        exact_replay_available: true,
        exact_evidence_count: grid.len(),
    };
    c.bench_function("voxel_candidate_report", |b| {
        b.iter(|| {
            let report = candidate_manifest.report();
            (
                report.exact_evidence_count,
                report.has_exact_evidence,
                report.promotable_as_exact,
            )
        })
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
        b.iter(|| {
            let report = coupling_manifest.report();
            (
                report.usable_as_exact_residual_evidence,
                report.has_adapter_error_bound,
                report.certified_adapter_error_bound_ready,
                report.requires_error_bounded_adapter,
            )
        })
    });
    c.bench_function("support_mask_report", |b| {
        b.iter(|| {
            let report =
                classify_support_mask(&grid, &grid, SupportDirection::new(2, -1).unwrap()).unwrap();
            (
                report.checked_cells,
                report.has_checked_cells,
                report.exact_support_mask_ready,
                report.is_conservatively_supported(),
            )
        })
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
        b.iter(|| {
            let report = artifact_manifest.report();
            (
                report.stable_id_ready,
                report.intended_domain_ready,
                report.role_supports_exact_indexing,
                report.has_aggregate_evidence,
                report.indexable_as_exact,
            )
        })
    });
    let preview_artifact_manifest = VoxelArtifactManifest {
        role: VoxelArtifactRole::PreviewArtifact,
        ..artifact_manifest.clone()
    };
    c.bench_function("preview_artifact_report", |b| {
        b.iter(|| preview_artifact_manifest.report())
    });
    c.bench_function("voxel_spatial_aggregate_facts", |b| {
        b.iter(|| {
            let report = VoxelSpatialAggregateFacts::from_grid(&grid, None).unwrap();
            (
                report.exact_bounds_ready,
                report.has_spatial_evidence,
                report.source_replay_ready,
                report.stored_cells,
            )
        })
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
        b.iter(|| {
            let report = storage_manifest.report();
            (
                report.physical_layout_ready,
                report.has_stored_cells,
                report.exact_storage_replay_ready,
                report.certified_aggregate_replay_ready,
            )
        })
    });
    let memory_manifest = VoxelMemoryBudgetManifest {
        kind: CompressedStorageKind::SparseVoxelDag,
        estimated_bytes: 4096,
        budget_bytes: 2048,
        preserves_exact_semantics_when_over_budget: true,
    };
    c.bench_function("voxel_memory_budget_report", |b| {
        b.iter(|| {
            let report = memory_manifest.report();
            (
                report.over_budget_bytes,
                report.has_memory_evidence,
                report.exact_semantics_preserved,
                report.exact_memory_budget_ready,
            )
        })
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
        b.iter(|| {
            let report = preview_manifest.report();
            (
                report.has_input_primitives,
                report.has_exported_primitives,
                report.exact_grid_topology_replay,
            )
        })
    });
    let sdf_preview_manifest = PreviewExportManifest {
        format: PreviewExportFormat::ContinuousSdfPreview,
        ..preview_manifest.clone()
    };
    c.bench_function("continuous_sdf_preview_manifest_report", |b| {
        b.iter(|| sdf_preview_manifest.report())
    });
    let adapter_contract = AdapterNumericContract::primitive_float(
        LegacyAdapterStatus::lossy(LegacyAdapterKind::PreviewRenderer, "bench display epsilon"),
        Some(r(1)),
        Some(Rational::fraction(1, 1024).unwrap().into()),
        Some(Rational::fraction(1, 512).unwrap().into()),
        AdapterToleranceStatus::Explicit,
    );
    c.bench_function("adapter_numeric_contract_report", |b| {
        b.iter(|| {
            let report = adapter_contract.report();
            (
                report.adapter_policy_ready,
                report.has_explicit_error_bound,
                report.tolerance_declaration_complete,
                report.can_contribute_certified_values,
                report.can_drive_exact_topology,
            )
        })
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
        b.iter(|| {
            let report = handoff_manifest.report();
            (report.has_aggregate_evidence, report.exact_handoff_ready)
        })
    });
    let continuous_rows = (0..64)
        .map(|i| {
            ContinuousFieldVoxelCell::new(
                continuous_field_address(&frame, [i, 0, 0]).unwrap(),
                VoxelCell::material(MaterialRegionId(1)),
            )
        })
        .collect::<Vec<_>>();
    let continuous_manifest = ContinuousFieldVoxelManifest {
        frame: frame.clone(),
        source: frame.source().cloned(),
        expected_source: frame.source().cloned(),
        expected_cell_count: continuous_rows.len(),
        cells: continuous_rows,
    };
    c.bench_function("continuous_field_voxel_intake_report", |b| {
        b.iter(|| continuous_manifest.report())
    });
    let continuous_interchange = ContinuousFieldVoxelInterchangeManifest {
        source: frame.source().cloned(),
        expected_source: frame.source().cloned(),
        coordinate_system: GridCoordinateSystem::HyperGrid,
        row_order: ContinuousFieldVoxelRowOrder::ExplicitAddresses,
        declared_depth: frame.depth(),
        declared_dimensions: [64, 64, 64],
        declared_cell_count: continuous_manifest.cells.len(),
    };
    c.bench_function("continuous_field_voxel_interchange_report", |b| {
        b.iter(|| continuous_manifest.interchange_report(&continuous_interchange))
    });
    let exact_intake_frame = GridFrame::builder()
        .depth(3)
        .source(GridSource::new("sdf:bench-direct", 1))
        .build()
        .unwrap();
    let exact_intake_source = exact_intake_frame.source().cloned();
    let mut exact_intake_rows = Vec::new();
    for z in 0..8 {
        for y in 0..8 {
            for x in 0..8 {
                exact_intake_rows.push(ContinuousFieldVoxelCell::new(
                    continuous_field_address(&exact_intake_frame, [x, y, z]).unwrap(),
                    VoxelCell::material(MaterialRegionId(3)),
                ));
            }
        }
    }
    let exact_intake_manifest = ContinuousFieldVoxelManifest {
        frame: exact_intake_frame,
        source: exact_intake_source.clone(),
        expected_source: exact_intake_source,
        expected_cell_count: exact_intake_rows.len(),
        cells: exact_intake_rows,
    };
    c.bench_function("continuous_field_exact_sparse_materialization", |b| {
        b.iter(|| {
            let prepared = exact_intake_manifest
                .materialize_exact_sparse_grid()
                .unwrap();
            (prepared.storage.len(), prepared.aggregate.has_lossy)
        })
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
    c.bench_function("voxel_trace_report", |b| {
        b.iter(|| {
            let report = trace_manifest.report();
            (
                report.dimension_count,
                report.has_operation_dimension,
                report.has_exact_evidence,
                report.exact_trace_evidence_ready,
                report.has_lossy_adapter_work,
                report.has_unknowns,
            )
        })
    });
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
        b.iter(|| {
            let report = manifest.report();
            (
                report.positive_dimensions_ready,
                report.invalid_dimension_axes,
                report.has_sample_evidence,
                report.declared_sample_slots,
                report.certified_sample_replay_ready,
                report.exact_sample_replay_ready,
            )
        })
    });
    let exact_stack_report = manifest.report();
    assert!(exact_stack_report.positive_dimensions_ready);
    assert!(exact_stack_report.has_sample_evidence);
    assert!(exact_stack_report.certified_sample_replay_ready);
    assert!(exact_stack_report.exact_sample_replay_ready);
    let overdeclared_manifest = ImageStackManifest {
        channels: 1,
        channel_mappings: vec![
            VoxelChannelMapping::FieldSample,
            VoxelChannelMapping::MaterialRegion,
        ],
        source: Some(GridSource::new("bench-stack", 1)),
        expected_source: Some(GridSource::new("bench-stack", 1)),
        ..manifest.clone()
    };
    c.bench_function("image_stack_overdeclared_channel_report", |b| {
        b.iter(|| overdeclared_manifest.report())
    });
    assert!(!overdeclared_manifest.report().certified_sample_replay_ready);

    let prepared = PreparedVoxelGrid::new(frame, grid.clone(), grid.stored_aggregate());
    let start = VoxelAddress::new(6, [0, 0, 0]).unwrap();
    let end = VoxelAddress::new(6, [31, 0, 0]).unwrap();
    c.bench_function("prepared_query_report", |b| {
        b.iter(|| {
            let report = prepared.prepared_query_report(true).unwrap();
            (
                report.non_empty_cells,
                report.has_query_evidence,
                report.exact_query_evidence_ready,
                report.cache_entries,
            )
        })
    });
    c.bench_function("address_segment_sweep", |b| {
        b.iter(|| {
            let sweep = sweep_address_segment(&prepared, start, end).unwrap();
            (
                sweep.trace.exact_address_trace_ready,
                sweep.trace.reached_end,
                sweep.exact_sweep_samples_ready,
            )
        })
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
