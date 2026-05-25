use hyperbrep::{
    BrepCoedge, BrepConstructionBlocker, BrepConstructionKind, BrepConstructionManifest, BrepEdge,
    BrepEdgeAgreementBlocker, BrepEdgeId, BrepEdgeOrientation, BrepExactSolidHandoffBlocker,
    BrepExactSurfaceHandoffBlocker, BrepExportBlocker, BrepExportFormat, BrepExportManifest,
    BrepExportScalarPolicy, BrepFace, BrepFaceAreaBlocker, BrepFaceId,
    BrepFaceTessellationManifest, BrepFeatureId, BrepHandoffPackageBlocker,
    BrepHandoffPackageManifest, BrepImportedSurfaceFamily, BrepLoop, BrepLoopId,
    BrepLossyFloatImportReport, BrepLossyImportBlocker, BrepMeshHandoffReport,
    BrepPhysicsMassBlocker, BrepPlanarExtrusionConstruction,
    BrepPlanarExtrusionConstructionBlocker, BrepPlanarRegionConstruction,
    BrepPlanarRegionConstructionBlocker, BrepShell, BrepShellBlocker, BrepShellTessellationReport,
    BrepShellVolumeBlocker, BrepSourceVersion, BrepSurface, BrepSurfaceId, BrepSurfaceSource,
    BrepTessellationBlocker, BrepTopologyValidationBlocker, BrepTriangleMeshBlocker,
    BrepTrimLoopBlocker, BrepVertex, BrepVertexId, BrepVoxelHandoffBlocker,
    BrepVoxelPackageRequest,
};
use hypercurve::{Contour2, LineSeg2, Region2, Segment2};
use hyperlimit::{Plane3, PlaneSide, Point2, Point3, classify_point_plane};
use hyperreal::Real;
use proptest::prelude::*;

fn r(value: i32) -> Real {
    Real::from(value)
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn uv(x: i32, y: i32) -> hypercurve::Point2 {
    hypercurve::Point2::new(r(x), r(y))
}

fn line2(start: hypercurve::Point2, end: hypercurve::Point2) -> Segment2 {
    Segment2::Line(LineSeg2::try_new(start, end).unwrap())
}

fn rect_region(width: i32, height: i32) -> Region2 {
    Region2::from_material_contours(vec![rect_contour(0, 0, width, height)])
}

fn rect_contour(min_x: i32, min_y: i32, max_x: i32, max_y: i32) -> Contour2 {
    Contour2::try_new(vec![
        line2(uv(min_x, min_y), uv(max_x, min_y)),
        line2(uv(max_x, min_y), uv(max_x, max_y)),
        line2(uv(max_x, max_y), uv(min_x, max_y)),
        line2(uv(min_x, max_y), uv(min_x, min_y)),
    ])
    .unwrap()
}

fn triangle_shell(reverse_second: bool) -> BrepShell {
    let second_orientation = if reverse_second {
        BrepEdgeOrientation::Reversed
    } else {
        BrepEdgeOrientation::Forward
    };
    BrepShell {
        vertices: vec![
            BrepVertex::new(BrepVertexId(0), p(0, 0, 0)),
            BrepVertex::new(BrepVertexId(1), p(1, 0, 0)),
            BrepVertex::new(BrepVertexId(2), p(0, 1, 0)),
        ],
        edges: vec![
            BrepEdge::new(BrepEdgeId(0), BrepVertexId(0), BrepVertexId(1)),
            BrepEdge::new(BrepEdgeId(1), BrepVertexId(1), BrepVertexId(2)),
            BrepEdge::new(BrepEdgeId(2), BrepVertexId(2), BrepVertexId(0)),
        ],
        surfaces: vec![BrepSurface::plane(
            BrepSurfaceId(0),
            Plane3::new(p(0, 0, 1), r(0)),
            BrepSurfaceSource::ExactConstruction,
        )],
        faces: vec![
            BrepFace::new(
                BrepFaceId(0),
                BrepSurfaceId(0),
                BrepLoop::new(
                    BrepLoopId(0),
                    vec![
                        BrepCoedge::new(BrepEdgeId(0), BrepEdgeOrientation::Forward),
                        BrepCoedge::new(BrepEdgeId(1), BrepEdgeOrientation::Forward),
                        BrepCoedge::new(BrepEdgeId(2), BrepEdgeOrientation::Forward),
                    ],
                ),
            ),
            BrepFace::new(
                BrepFaceId(1),
                BrepSurfaceId(0),
                BrepLoop::new(
                    BrepLoopId(1),
                    vec![
                        BrepCoedge::new(BrepEdgeId(0), second_orientation),
                        BrepCoedge::new(BrepEdgeId(1), second_orientation),
                        BrepCoedge::new(BrepEdgeId(2), second_orientation),
                    ],
                ),
            ),
        ],
    }
}

#[test]
fn opposite_edge_uses_are_closed_but_same_orientation_pairs_are_blocked() {
    let closed = triangle_shell(true).audit_closure();
    assert!(closed.closed);
    assert!(closed.exact_shell_ready);

    let same_direction = triangle_shell(false).audit_closure();
    assert!(!same_direction.closed);
    assert!(!same_direction.exact_shell_ready);
    assert_eq!(same_direction.same_orientation_pair_count, 3);
    assert!(
        same_direction
            .blockers
            .contains(&BrepShellBlocker::SameOrientationEdgePair)
    );
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(128))]

    #[test]
    fn generated_open_fan_reports_boundary_edges(edge_count in 1_usize..=24) {
        let vertices = (0..=edge_count)
            .map(|i| BrepVertex::new(BrepVertexId(i as u64), p(i as i32, 0, 0)))
            .collect::<Vec<_>>();
        let edges = (0..edge_count)
            .map(|i| BrepEdge::new(BrepEdgeId(i as u64), BrepVertexId(i as u64), BrepVertexId((i + 1) as u64)))
            .collect::<Vec<_>>();
        let coedges = (0..edge_count)
            .map(|i| BrepCoedge::new(BrepEdgeId(i as u64), BrepEdgeOrientation::Forward))
            .collect::<Vec<_>>();
        let shell = BrepShell {
            vertices,
            edges,
            surfaces: vec![BrepSurface::plane(
                BrepSurfaceId(0),
                Plane3::new(p(0, 0, 1), r(0)),
                BrepSurfaceSource::ExactConstruction,
            )],
            faces: vec![BrepFace::new(
                BrepFaceId(0),
                BrepSurfaceId(0),
                BrepLoop::new(BrepLoopId(0), coedges),
            )],
        };
        let report = shell.audit_closure();
        let topology = shell.validate_topology();
        let validation = shell.shell_validation_report();
        let area = shell.face_area_report(BrepFaceId(0));
        let edge_agreement = shell.edge_agreement_report();

        prop_assert!(!report.closed);
        prop_assert!(!report.exact_shell_ready);
        prop_assert_eq!(report.boundary_edge_count, edge_count);
        prop_assert!(report.blockers.contains(&BrepShellBlocker::BoundaryEdges));
        prop_assert!(!topology.topology_ready);
        prop_assert_eq!(topology.boundary_edge_count, edge_count);
        prop_assert_eq!(topology.boundary_component_count, 1);
        prop_assert!(topology.blockers.contains(&BrepTopologyValidationBlocker::BoundaryEdges));
        prop_assert!(!validation.exact_closed_shell_ready);
        prop_assert!(!validation.exact_surface_boundary_ready);
        prop_assert_eq!(validation.blocked_face_count, 1);
        prop_assert!(!area.exact_area_ready);
        prop_assert!(area.blockers.contains(&BrepFaceAreaBlocker::BrokenLoopChain));
        prop_assert!(!edge_agreement.shell_edge_agreement_ready);
        prop_assert_eq!(edge_agreement.boundary_edge_count, edge_count);
        prop_assert!(
            edge_agreement
                .blockers
                .contains(&BrepEdgeAgreementBlocker::BoundaryEdge)
        );
        let physics = shell.physics_mass_handoff_report(r(1));
        prop_assert!(!physics.exact_physics_mass_ready);
        prop_assert!(
            physics
                .blockers
                .contains(&BrepPhysicsMassBlocker::SolidReadinessNotReady)
        );
        let triangles = shell.exact_triangle_mesh_handoff_report();
        prop_assert!(!triangles.exact_triangle_mesh_ready);
        prop_assert!(
            triangles
                .blockers
                .contains(&BrepTriangleMeshBlocker::SolidReadinessNotReady)
        );
        let frame = hypervoxel::GridFrame::new(
            [r(0), r(0), r(0)],
            [r(1), r(1), r(1)],
            1,
            hypervoxel::LengthUnit::Unitless,
            None,
        ).unwrap();
        let voxel = shell.voxel_handoff_report(frame, None);
        prop_assert!(voxel.exact_aabb_handoff_ready);
        prop_assert!(!voxel.exact_triangle_source_ready);
        prop_assert!(!voxel.exact_triangle_voxelization_ready);
        prop_assert!(
            voxel
                .blockers
                .contains(&BrepVoxelHandoffBlocker::TriangleMeshNotReady)
        );
        let package = shell.handoff_package_report(
            None,
            BrepHandoffPackageManifest::basic()
                .with_voxel(BrepVoxelPackageRequest {
                    frame: hypervoxel::GridFrame::new(
                        [r(0), r(0), r(0)],
                        [r(1), r(1), r(1)],
                        1,
                        hypervoxel::LengthUnit::Unitless,
                        None,
                    ).unwrap(),
                    expected_source: None,
                    require_triangle_voxelization: false,
                }),
        );
        prop_assert!(!package.all_requested_exact_ready);
        prop_assert!(
            package
                .blockers
                .contains(&BrepHandoffPackageBlocker::TriangleMeshNotReady)
        );
    }

    #[test]
    fn generated_mesh_handoff_requires_one_exact_manifest_per_face(edge_count in 3_usize..=24, supplied_manifest in proptest::bool::ANY) {
        let vertices = (0..edge_count)
            .map(|i| BrepVertex::new(BrepVertexId(i as u64), p(i as i32, 0, 0)))
            .collect::<Vec<_>>();
        let edges = (0..edge_count)
            .map(|i| {
                BrepEdge::new(
                    BrepEdgeId(i as u64),
                    BrepVertexId(i as u64),
                    BrepVertexId(((i + 1) % edge_count) as u64),
                )
            })
            .collect::<Vec<_>>();
        let forward = (0..edge_count)
            .map(|i| BrepCoedge::new(BrepEdgeId(i as u64), BrepEdgeOrientation::Forward))
            .collect::<Vec<_>>();
        let reversed = (0..edge_count).rev()
            .map(|i| BrepCoedge::new(BrepEdgeId(i as u64), BrepEdgeOrientation::Reversed))
            .collect::<Vec<_>>();
        let shell = BrepShell {
            vertices,
            edges,
            surfaces: vec![BrepSurface::plane(
                BrepSurfaceId(0),
                Plane3::new(p(0, 0, 1), r(0)),
                BrepSurfaceSource::ExactConstruction,
            )],
            faces: vec![
                BrepFace::new(
                    BrepFaceId(0),
                    BrepSurfaceId(0),
                    BrepLoop::new(BrepLoopId(0), forward),
                ),
                BrepFace::new(
                    BrepFaceId(1),
                    BrepSurfaceId(0),
                    BrepLoop::new(BrepLoopId(1), reversed),
                ),
            ],
        };
        let manifests = if supplied_manifest {
            vec![
                BrepFaceTessellationManifest::exact_planar(BrepFaceId(0), edge_count - 2, edge_count, edge_count, 0),
                BrepFaceTessellationManifest::exact_planar(BrepFaceId(1), edge_count - 2, edge_count, edge_count, 0),
            ]
        } else {
            vec![BrepFaceTessellationManifest::exact_planar(BrepFaceId(0), edge_count - 2, edge_count, edge_count, 0)]
        };
        let shell_tessellation = BrepShellTessellationReport::from_shell_manifests(&shell, &manifests);
        prop_assert_eq!(shell_tessellation.exact_surface_handoff_ready, supplied_manifest);
        prop_assert_eq!(shell_tessellation.exact_solid_handoff_ready, supplied_manifest);
        prop_assert_eq!(shell_tessellation.source_face_count, 2);
        prop_assert_eq!(shell_tessellation.blocked_face_count, if supplied_manifest { 0 } else { 1 });

        let report = BrepMeshHandoffReport::from_shell_manifests(&shell, &manifests);

        prop_assert_eq!(report.exact_surface_handoff_ready, supplied_manifest);
        prop_assert_eq!(report.exact_solid_handoff_ready, supplied_manifest);
        prop_assert_eq!(report.tessellation, shell_tessellation);
        if supplied_manifest {
            prop_assert_eq!(report.blocked_face_count, 0);
            prop_assert_eq!(report.triangle_count, 2 * (edge_count - 2));
        } else {
            prop_assert_eq!(report.blocked_face_count, 1);
            prop_assert!(
                report.faces[1]
                    .blockers
                    .contains(&BrepTessellationBlocker::MissingManifest)
            );
        }

        let generated = shell.exact_planar_tessellation_manifests();
        prop_assert!(generated.is_empty());
        let generated_report = shell.exact_planar_mesh_handoff_report();
        prop_assert!(!generated_report.exact_surface_handoff_ready);
        prop_assert_eq!(generated_report.blocked_face_count, 2);
        prop_assert!(generated_report
            .faces
            .iter()
            .all(|face| face
                .blockers
                .contains(&BrepTessellationBlocker::MissingManifest)));
    }

    #[test]
    fn generated_construction_snapshot_rejects_stale_topology(
        edge_count in 3_usize..=24,
        mutation in 0_u8..=2,
    ) {
        let vertices = (0..=edge_count)
            .map(|i| BrepVertex::new(BrepVertexId(i as u64), p(i as i32, 0, 0)))
            .collect::<Vec<_>>();
        let edges = (0..edge_count)
            .map(|i| BrepEdge::new(BrepEdgeId(i as u64), BrepVertexId(i as u64), BrepVertexId((i + 1) as u64)))
            .collect::<Vec<_>>();
        let forward = (0..edge_count)
            .map(|i| BrepCoedge::new(BrepEdgeId(i as u64), BrepEdgeOrientation::Forward))
            .collect::<Vec<_>>();
        let reversed = (0..edge_count)
            .map(|i| BrepCoedge::new(BrepEdgeId(i as u64), BrepEdgeOrientation::Reversed))
            .collect::<Vec<_>>();
        let shell = BrepShell {
            vertices,
            edges,
            surfaces: vec![BrepSurface::plane(
                BrepSurfaceId(0),
                Plane3::new(p(0, 0, 1), r(0)),
                BrepSurfaceSource::ExactConstruction,
            )],
            faces: vec![
                BrepFace::new(
                    BrepFaceId(0),
                    BrepSurfaceId(0),
                    BrepLoop::new(BrepLoopId(0), forward),
                ),
                BrepFace::new(
                    BrepFaceId(1),
                    BrepSurfaceId(0),
                    BrepLoop::new(BrepLoopId(1), reversed),
                ),
            ],
        };
        let manifest = BrepConstructionManifest::exact(
            BrepFeatureId::new("generated:strip").unwrap(),
            BrepConstructionKind::PlanarFace,
            vec![BrepSourceVersion::new("source:strip", edge_count as u64).unwrap()],
            &shell,
        );
        let mut checked_shell = shell.clone();
        match mutation {
            0 => {}
            1 => checked_shell
                .edges
                .push(BrepEdge::new(BrepEdgeId(999), BrepVertexId(0), BrepVertexId(0))),
            _ => {
                checked_shell.edges[0] = BrepEdge::new(
                    BrepEdgeId(0),
                    BrepVertexId(1),
                    BrepVertexId(0),
                );
            }
        }
        let report = manifest.report(&checked_shell);

        prop_assert_eq!(report.construction_fresh, mutation == 0);
        prop_assert_eq!(report.topology_fingerprint_current, mutation == 0);
        prop_assert_eq!(report.topology_snapshot_current, mutation != 1);
        if mutation == 1 {
            prop_assert!(
                report
                    .blockers
                    .contains(&BrepConstructionBlocker::StaleTopologySnapshot)
            );
        }
        if mutation != 0 {
            prop_assert!(
                report
                    .blockers
                    .contains(&BrepConstructionBlocker::StaleTopologyFingerprint)
            );
        }
    }

    #[test]
    fn generated_planar_region_construction_preserves_exact_uv_bounds(
        width in 1_i32..=64,
        height in 1_i32..=64,
        unsupported_surface in proptest::bool::ANY,
    ) {
        let region = rect_region(width, height);
        let surface = if unsupported_surface {
            BrepSurface::unsupported(
                BrepSurfaceId(0),
                "nurbs-surface",
                BrepSurfaceSource::ExactConstruction,
            )
        } else {
            BrepSurface::plane(
                BrepSurfaceId(0),
                Plane3::new(p(0, 0, 1), r(0)),
                BrepSurfaceSource::ExactConstruction,
            )
        };
        let constructed = BrepPlanarRegionConstruction::from_region_on_surface(
            &region,
            surface,
            BrepFeatureId::new("generated:planar-region").unwrap(),
            vec![BrepSourceVersion::new("region:rect", width as u64 + height as u64).unwrap()],
        );

        prop_assert_eq!(constructed.exact_construction_ready, !unsupported_surface);
        if unsupported_surface {
            prop_assert!(
                constructed
                    .blockers
                    .contains(&BrepPlanarRegionConstructionBlocker::SurfaceFrameNotReady)
            );
        } else {
            let shell = constructed.shell.as_ref().unwrap();
            prop_assert_eq!(shell.vertices.len(), 4);
            prop_assert_eq!(shell.edges.len(), 4);
            let uv_bounds = shell.face_uv_bounds_report(BrepFaceId(0));
            prop_assert!(uv_bounds.exact_uv_bounds_ready);
            prop_assert_eq!(
                uv_bounds.min,
                Some(hyperlimit::Point2::new(r(0), r(0)))
            );
            prop_assert_eq!(
                uv_bounds.max,
                Some(hyperlimit::Point2::new(r(width), r(height)))
            );
            prop_assert!(
                constructed
                    .manifest
                    .as_ref()
                    .unwrap()
                    .report(shell)
                    .construction_fresh
            );
        }
    }

    #[test]
    fn generated_planar_extrusion_construction_builds_solid_prisms(
        width in 1_i32..=24,
        depth in 1_i32..=24,
        height in 1_i32..=24,
    ) {
        let region = rect_region(width, depth);
        let constructed = BrepPlanarExtrusionConstruction::vertical_prism_from_region(
            &region,
            r(0),
            r(height),
            BrepFeatureId::new("generated:extrusion").unwrap(),
            vec![BrepSourceVersion::new("region:extrusion", (width + depth + height) as u64).unwrap()],
        );

        prop_assert!(constructed.exact_construction_ready);
        prop_assert!(constructed.blockers.is_empty());
        prop_assert_eq!(constructed.source_vertex_count, 4);
        prop_assert_eq!(constructed.vertex_count, 8);
        prop_assert_eq!(constructed.edge_count, 12);
        prop_assert_eq!(constructed.face_count, 6);
        let shell = constructed.shell.as_ref().unwrap();
        let solid = shell.solid_readiness_report(constructed.manifest.as_ref());
        prop_assert!(solid.exact_solid_boundary_ready);
        prop_assert_eq!(
            solid.volume.signed_six_volume,
            Some(r(6 * width * depth * height))
        );
        prop_assert!(shell.physics_mass_handoff_report(r(1)).exact_physics_mass_ready);
        let frame = hypervoxel::GridFrame::new(
            [r(0), r(0), r(0)],
            [r(1), r(1), r(1)],
            6,
            hypervoxel::LengthUnit::Unitless,
            None,
        ).unwrap();
        let voxel = shell.voxel_handoff_report(frame, None);
        prop_assert!(voxel.exact_aabb_handoff_ready);
        prop_assert!(voxel.exact_triangle_source_ready);
        prop_assert!(voxel.exact_triangle_voxelization_ready);
        prop_assert!(voxel.exact_triangle_solid.is_some());
        prop_assert!(voxel.prepared_triangle_solid.is_some());
        prop_assert!(voxel.blockers.is_empty());
    }

    #[test]
    fn generated_planar_extrusion_construction_rejects_invalid_height(
        height in -24_i32..=0,
    ) {
        let constructed = BrepPlanarExtrusionConstruction::vertical_prism_from_region(
            &rect_region(2, 3),
            r(0),
            r(height),
            BrepFeatureId::new("generated:bad-extrusion").unwrap(),
            vec![BrepSourceVersion::new("region:bad-extrusion", 1).unwrap()],
        );

        prop_assert!(!constructed.exact_construction_ready);
        prop_assert!(
            constructed
                .blockers
                .contains(&BrepPlanarExtrusionConstructionBlocker::NonPositiveHeight)
        );
    }

    #[test]
    fn generated_planar_extrusion_construction_preserves_hole_volume(
        outer_width in 3_i32..=24,
        outer_depth in 3_i32..=24,
        hole_width in 1_i32..=8,
        hole_depth in 1_i32..=8,
        height in 1_i32..=12,
    ) {
        let hole_width = hole_width.min(outer_width - 2);
        let hole_depth = hole_depth.min(outer_depth - 2);
        let region = Region2::new(
            vec![rect_contour(0, 0, outer_width, outer_depth)],
            vec![rect_contour(1, 1, 1 + hole_width, 1 + hole_depth)],
        );
        let constructed = BrepPlanarExtrusionConstruction::vertical_prism_from_region(
            &region,
            r(0),
            r(height),
            BrepFeatureId::new("generated:holed-extrusion").unwrap(),
            vec![BrepSourceVersion::new("region:holed-extrusion", 1).unwrap()],
        );

        prop_assert!(constructed.exact_construction_ready);
        prop_assert_eq!(constructed.source_vertex_count, 8);
        prop_assert_eq!(constructed.vertex_count, 16);
        prop_assert_eq!(constructed.edge_count, 24);
        prop_assert_eq!(constructed.face_count, 10);
        let shell = constructed.shell.as_ref().unwrap();
        let solid = shell.solid_readiness_report(constructed.manifest.as_ref());
        prop_assert!(solid.exact_solid_boundary_ready);
        prop_assert_eq!(
            solid.volume.signed_six_volume,
            Some(r(6 * (outer_width * outer_depth - hole_width * hole_depth) * height))
        );
        prop_assert!(shell.physics_mass_handoff_report(r(1)).exact_physics_mass_ready);
    }

    #[test]
    fn generated_lossy_import_audit_tracks_finiteness_and_surface_support(values in proptest::collection::vec(-1000.0_f64..=1000.0, 0..48), add_nan in proptest::bool::ANY, include_unsupported in proptest::bool::ANY) {
        let mut coordinates = values;
        if add_nan {
            coordinates.push(f64::NAN);
        }
        let finite_count = coordinates.iter().filter(|value| value.is_finite()).count();
        let mut surfaces = vec![BrepImportedSurfaceFamily::Plane];
        if include_unsupported {
            surfaces.push(BrepImportedSurfaceFamily::Nurbs);
        }

        let report = BrepLossyFloatImportReport::inspect_f64(
            "generated:adapter",
            &coordinates,
            &surfaces,
            true,
            true,
        );

        prop_assert_eq!(report.finite_coordinate_count, finite_count);
        prop_assert_eq!(report.exact_dyadic_lift_count, finite_count);
        prop_assert_eq!(report.non_finite_coordinate_indexes.is_empty(), !add_nan);
        prop_assert_eq!(
            report.blockers.contains(&BrepLossyImportBlocker::NonFiniteCoordinate),
            add_nan
        );
        prop_assert_eq!(
            report.blockers.contains(&BrepLossyImportBlocker::UnsupportedSurfaceKind),
            include_unsupported
        );
        prop_assert_eq!(
            report.adapter_replay_ready,
            !add_nan && !include_unsupported && coordinates.len() % 3 == 0
        );
    }

    #[test]
    fn generated_export_report_tracks_source_ids_and_coordinate_finiteness(
        primitive_count in 0_usize..=64,
        coordinate_count in 0_usize..=192,
        finite_delta in 0_usize..=8,
        include_source in proptest::bool::ANY,
        exact_replay in proptest::bool::ANY,
    ) {
        let finite_coordinates = coordinate_count.saturating_sub(finite_delta);
        let manifest = BrepExportManifest {
            format: BrepExportFormat::Step,
            scalar_policy: BrepExportScalarPolicy::ExactText,
            source_object_ids: if include_source { vec!["shell:generated".into()] } else { Vec::new() },
            exported_primitives: primitive_count,
            exported_coordinates: coordinate_count,
            finite_exported_coordinates: finite_coordinates,
            labels_preserved: true,
            exact_replay_declared: exact_replay,
        };
        let report = manifest.report(None);

        prop_assert_eq!(
            report.blockers.contains(&BrepExportBlocker::MissingSourceObjectIds),
            !include_source
        );
        prop_assert_eq!(
            report.blockers.contains(&BrepExportBlocker::EmptyExport),
            primitive_count == 0
        );
        prop_assert_eq!(
            report.blockers.contains(&BrepExportBlocker::NonFiniteExportCoordinates),
            finite_coordinates != coordinate_count
        );
        prop_assert_eq!(
            report.blockers.contains(&BrepExportBlocker::ExternalBrepReplayMissing),
            !exact_replay
        );
        prop_assert_eq!(
            report.export_ready,
            include_source && primitive_count > 0 && finite_coordinates == coordinate_count && exact_replay
        );
    }

    #[test]
    fn generated_prepared_plane_surface_matches_hyperlimit_classifier(a in -8_i32..=8, b in -8_i32..=8, c in 1_i32..=8, d in -16_i32..=16, x in -8_i32..=8, y in -8_i32..=8, z in -8_i32..=8) {
        let plane = Plane3::new(p(a, b, c), r(d));
        let surface = BrepSurface::plane(
            BrepSurfaceId(500),
            plane.clone(),
            BrepSurfaceSource::ExactConstruction,
        );
        let prepared = surface.prepare();
        let point = p(x, y, z);
        let report = prepared.classify_point(&point);
        let expected = classify_point_plane(&point, &plane).value();

        prop_assert!(prepared.exact_replay_ready());
        prop_assert_eq!(report.side, expected);
        prop_assert_eq!(report.on_surface, expected == Some(PlaneSide::On));
        prop_assert!(report.exact_replay);
    }

    #[test]
    fn generated_axis_plane_frame_roundtrips_uv(c in 1_i32..=8, d in -16_i32..=16, u in -8_i32..=8, v in -8_i32..=8) {
        let surface = BrepSurface::plane(
            BrepSurfaceId(501),
            Plane3::new(p(0, 0, c), r(d)),
            BrepSurfaceSource::ExactConstruction,
        );
        let uv = Point2::new(r(u), r(v));
        let eval = surface.evaluate_frame_uv(uv.clone());
        prop_assert!(eval.exact_evaluation_ready);
        let point = eval.point.clone().expect("ready frame evaluates a point");
        let projection = surface.project_frame_point(point.clone());
        prop_assert!(projection.exact_projection_ready);
        prop_assert_eq!(projection.uv, Some(uv));
        prop_assert_eq!(classify_point_plane(&point, &Plane3::new(p(0, 0, c), r(d))).value(), Some(PlaneSide::On));
    }

    #[test]
    fn generated_trim_loop_report_tracks_oriented_vertex_chain(edge_count in 3_usize..=32, reverse_one in proptest::bool::ANY) {
        let vertices = (0..edge_count)
            .map(|i| BrepVertex::new(BrepVertexId(i as u64), p(i as i32, (i % 5) as i32, 0)))
            .collect::<Vec<_>>();
        let edges = (0..edge_count)
            .map(|i| {
                BrepEdge::new(
                    BrepEdgeId(i as u64),
                    BrepVertexId(i as u64),
                    BrepVertexId(((i + 1) % edge_count) as u64),
                )
            })
            .collect::<Vec<_>>();
        let coedges = (0..edge_count)
            .map(|i| {
                let orientation = if reverse_one && i == edge_count / 2 {
                    BrepEdgeOrientation::Reversed
                } else {
                    BrepEdgeOrientation::Forward
                };
                BrepCoedge::new(BrepEdgeId(i as u64), orientation)
            })
            .collect::<Vec<_>>();
        let shell = BrepShell {
            vertices,
            edges,
            surfaces: vec![BrepSurface::plane(
                BrepSurfaceId(0),
                Plane3::new(p(0, 0, 1), r(0)),
                BrepSurfaceSource::ExactConstruction,
            )],
            faces: vec![BrepFace::new(
                BrepFaceId(0),
                BrepSurfaceId(0),
                BrepLoop::new(BrepLoopId(0), coedges),
            )],
        };
        let report = shell.trim_set_report(BrepFaceId(0));

        prop_assert_eq!(report.trim_set_ready, !reverse_one);
        prop_assert_eq!(report.loops[0].closed_vertex_chain, !reverse_one);
        prop_assert_eq!(
            report.loops[0].blockers.contains(&BrepTrimLoopBlocker::VertexChainBreak),
            reverse_one
        );
        let bounds = shell.face_bounds_report(BrepFaceId(0));
        prop_assert!(bounds.exact_bounds_ready);
        prop_assert_eq!(bounds.min, Some(p(0, 0, 0)));
        prop_assert_eq!(bounds.max, Some(p((edge_count - 1) as i32, 4.min((edge_count - 1) as i32), 0)));
        let uv_bounds = shell.face_uv_bounds_report(BrepFaceId(0));
        prop_assert!(uv_bounds.exact_uv_bounds_ready);
        prop_assert_eq!(uv_bounds.min, Some(Point2::new(r(0), r(0))));
        prop_assert_eq!(
            uv_bounds.max,
            Some(Point2::new(r((edge_count - 1) as i32), r(4.min((edge_count - 1) as i32))))
        );
        let validation = shell.face_validation_report(BrepFaceId(0), None);
        prop_assert_eq!(validation.exact_face_boundary_ready, !reverse_one);
        prop_assert!(validation.exact_bounds_ready);
        prop_assert!(validation.exact_uv_bounds_ready);
        prop_assert_eq!(
            validation.uv_bounds.as_ref().map(|report| report.exact_uv_bounds_ready),
            Some(true)
        );
        prop_assert_eq!(validation.exact_face_ready, !reverse_one);
        let geometry = shell.geometry_validation_report(BrepFaceId(0));
        prop_assert_eq!(geometry.geometry_ready, !reverse_one);
        prop_assert_eq!(geometry.on_surface_vertex_count, edge_count);
        prop_assert_eq!(geometry.off_surface_vertex_count, 0);
        let edge_agreement = shell.edge_agreement_report();
        prop_assert!(!edge_agreement.shell_edge_agreement_ready);
        prop_assert_eq!(edge_agreement.boundary_edge_count, edge_count);
        prop_assert!(
            edge_agreement
                .blockers
                .contains(&BrepEdgeAgreementBlocker::BoundaryEdge)
        );
        let preflight = shell.face_aabb_preflight(BrepFaceId(0), BrepFaceId(0));
        prop_assert!(preflight.preflight_ready);
        prop_assert_eq!(preflight.relation, Some(hyperlimit::Aabb3Intersection::Touching));
        prop_assert!(preflight.requires_narrow_phase);
        let solid = shell.solid_readiness_report(None);
        prop_assert!(!solid.exact_solid_boundary_ready);
        prop_assert!(!solid.closed_shell_ready);
        prop_assert!(!solid.exact_volume_ready);
        prop_assert!(
            solid
                .volume
                .blockers
                .contains(&BrepShellVolumeBlocker::ShellClosureNotReady)
        );
        prop_assert_eq!(solid.ready_face_count, if reverse_one { 0 } else { 1 });
        let physics = shell.physics_mass_handoff_report(r(1));
        prop_assert!(!physics.exact_physics_mass_ready);
        prop_assert!(
            physics
                .blockers
                .contains(&BrepPhysicsMassBlocker::SolidReadinessNotReady)
        );
        let triangles = shell.exact_triangle_mesh_handoff_report();
        prop_assert!(!triangles.exact_triangle_mesh_ready);
        prop_assert!(
            triangles
                .blockers
                .contains(&BrepTriangleMeshBlocker::SolidReadinessNotReady)
        );
        let surface_handoff = shell.exact_surface_handoff();
        prop_assert_eq!(surface_handoff.exact_surface_handoff_ready, !reverse_one);
        prop_assert_eq!(
            surface_handoff
                .blockers
                .contains(&BrepExactSurfaceHandoffBlocker::ShellValidationNotReady),
            reverse_one
        );
        let solid_handoff = shell.exact_solid_handoff(None);
        prop_assert!(!solid_handoff.exact_solid_handoff_ready);
        prop_assert_eq!(
            solid_handoff
                .blockers
                .contains(&BrepExactSolidHandoffBlocker::SurfaceHandoffNotReady),
            reverse_one
        );
        prop_assert!(
            solid_handoff
                .blockers
                .contains(&BrepExactSolidHandoffBlocker::VolumeNotReady)
        );
        let plane_preflight = shell.face_plane_preflight(
            BrepFaceId(0),
            &Plane3::new(p(0, 0, 1), r(0)),
        );
        prop_assert!(plane_preflight.preflight_ready);
        prop_assert_eq!(
            plane_preflight.relation,
            Some(hyperlimit::PlaneAabbRelation::Intersecting)
        );
        prop_assert!(plane_preflight.requires_narrow_phase);
        let segment_preflight =
            shell.segment_face_plane_preflight(BrepFaceId(0), &p(0, 0, -1), &p(0, 0, 1));
        prop_assert!(segment_preflight.preflight_ready);
        prop_assert_eq!(
            segment_preflight.relation,
            Some(hyperlimit::PlaneSegmentRelation::Crossing)
        );
        prop_assert!(segment_preflight.requires_narrow_phase);
        let point_preflight = shell.point_face_plane_preflight(BrepFaceId(0), &p(0, 0, 0));
        prop_assert!(point_preflight.preflight_ready);
        prop_assert_eq!(point_preflight.side, Some(hyperlimit::PlaneSide::On));
        prop_assert!(point_preflight.requires_trim_replay);
        let prepared = shell.prepare_face_query(BrepFaceId(0));
        prop_assert!(prepared.prepared_query_ready);
        prop_assert_eq!(
            prepared.face_plane_preflight(&Plane3::new(p(0, 0, 1), r(0))),
            plane_preflight
        );
        prop_assert_eq!(
            prepared.segment_face_plane_preflight(&p(0, 0, -1), &p(0, 0, 1)),
            segment_preflight
        );
        prop_assert_eq!(
            prepared.point_face_plane_preflight(&p(0, 0, 0)),
            point_preflight
        );
        let points = vec![p(0, 0, 0), p(0, 0, 1)];
        let segment_start = p(0, 0, -1);
        let segment_end = p(0, 0, 1);
        let segments = vec![(&segment_start, &segment_end)];
        let batch = prepared.batch_report(&points, &segments);
        prop_assert_eq!(batch.point_query_count, 2);
        prop_assert_eq!(batch.segment_query_count, 1);
        prop_assert_eq!(batch.certified_rejection_count, 1);
        prop_assert_eq!(batch.narrow_phase_candidate_count, 2);
    }
}
