use hyperbrep::{
    BrepCoedge, BrepConstructionBlocker, BrepConstructionKind, BrepConstructionManifest, BrepEdge,
    BrepEdgeId, BrepEdgeOrientation, BrepExportBlocker, BrepExportFormat, BrepExportManifest,
    BrepExportScalarPolicy, BrepFace, BrepFaceId, BrepFaceTessellationManifest, BrepFeatureId,
    BrepImportedSurfaceFamily, BrepLoop, BrepLoopId, BrepLossyFloatImportReport,
    BrepLossyImportBlocker, BrepMeshHandoffReport, BrepShell, BrepShellBlocker,
    BrepShellTessellationReport, BrepSourceVersion, BrepSurface, BrepSurfaceId, BrepSurfaceSource,
    BrepTessellationBlocker, BrepTopologyValidationBlocker, BrepTrimLoopBlocker, BrepVertex,
    BrepVertexId,
};
use hyperlimit::{Plane3, PlaneSide, Point3, classify_point_plane};
use hyperreal::Real;
use proptest::prelude::*;

fn r(value: i32) -> Real {
    Real::from(value)
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
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
    }

    #[test]
    fn generated_construction_snapshot_rejects_stale_topology(edge_count in 3_usize..=24, mutate_shell in proptest::bool::ANY) {
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
        if mutate_shell {
            checked_shell
                .edges
                .push(BrepEdge::new(BrepEdgeId(999), BrepVertexId(0), BrepVertexId(0)));
        }
        let report = manifest.report(&checked_shell);

        prop_assert_eq!(report.construction_fresh, !mutate_shell);
        prop_assert_eq!(report.topology_snapshot_current, !mutate_shell);
        if mutate_shell {
            prop_assert!(
                report
                    .blockers
                    .contains(&BrepConstructionBlocker::StaleTopologySnapshot)
            );
        }
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
        let validation = shell.face_validation_report(BrepFaceId(0), None);
        prop_assert_eq!(validation.exact_face_boundary_ready, !reverse_one);
        prop_assert!(validation.exact_bounds_ready);
        prop_assert_eq!(validation.exact_face_ready, !reverse_one);
        let geometry = shell.geometry_validation_report(BrepFaceId(0));
        prop_assert_eq!(geometry.geometry_ready, !reverse_one);
        prop_assert_eq!(geometry.on_surface_vertex_count, edge_count);
        prop_assert_eq!(geometry.off_surface_vertex_count, 0);
        let preflight = shell.face_aabb_preflight(BrepFaceId(0), BrepFaceId(0));
        prop_assert!(preflight.preflight_ready);
        prop_assert_eq!(preflight.relation, Some(hyperlimit::Aabb3Intersection::Touching));
        prop_assert!(preflight.requires_narrow_phase);
        let solid = shell.solid_readiness_report(None);
        prop_assert!(!solid.exact_solid_boundary_ready);
        prop_assert!(!solid.closed_shell_ready);
        prop_assert_eq!(solid.ready_face_count, if reverse_one { 0 } else { 1 });
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
    }
}
