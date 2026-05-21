//! Exact-aware boundary-representation evidence carriers.
//!
//! `hyperbrep` owns retained BREP topology: vertices, edges, loops, faces,
//! surfaces, and shell validation reports. It deliberately does not perform
//! hidden healing, tolerance merging, or mesh-derived topology promotion. That
//! follows Yap, "Towards Exact Geometric Computation," *Computational Geometry*
//! 7.1-2 (1997): topology-changing decisions must be backed by exact or
//! certified predicates, while uncertain or adapter-derived evidence stays
//! explicit.

mod area;
mod bounds;
mod export;
mod import;
mod provenance;
mod query;
mod report;
mod solid;
mod surface;
mod tessellation;
mod topology;
mod trim;
mod validation;
mod volume;

pub use area::{BrepAreaProjectionAxis, BrepFaceAreaBlocker, BrepFaceAreaReport};
pub use bounds::{
    BrepFaceAabbPreflightBlocker, BrepFaceAabbPreflightReport, BrepFaceBoundsBlocker,
    BrepFaceBoundsReport, BrepShellBoundsBlocker, BrepShellBoundsReport, PreparedBrepFaceBounds,
    PreparedBrepShellBounds,
};
pub use export::{
    BrepExportBlocker, BrepExportFormat, BrepExportManifest, BrepExportReport,
    BrepExportScalarPolicy,
};
pub use import::{
    BrepImportedSurfaceFamily, BrepLossyFloatImportReport, BrepLossyImportBlocker,
    BrepPrimitiveFloatPrecision, BrepUnsupportedSurfaceRecord,
};
pub use provenance::{
    BrepConstructionBlocker, BrepConstructionKind, BrepConstructionManifest,
    BrepConstructionProvenanceReport, BrepConstructionReplayStatus, BrepFeatureId,
    BrepSelectedReference, BrepSourceVersion, BrepTopologySnapshot,
};
pub use query::{
    BrepFacePlanePreflightBlocker, BrepFacePlanePreflightReport, BrepPointFacePlaneBlocker,
    BrepPointFacePlaneReport, BrepSegmentFacePlaneBlocker, BrepSegmentFacePlaneReport,
};
pub use report::{
    BrepShellBlocker, BrepShellClosureReport, BrepSurfaceInventoryReport, BrepTopologyCounts,
    BrepTopologyValidationBlocker, BrepTopologyValidationReport,
};
pub use solid::{BrepSolidReadinessBlocker, BrepSolidReadinessReport};
pub use surface::{
    BrepSurface, BrepSurfaceBlocker, BrepSurfaceFacts, BrepSurfaceId, BrepSurfaceKind,
    BrepSurfacePointReport, BrepSurfaceSource, PreparedBrepSurface,
};
pub use tessellation::{
    BrepFaceTessellationManifest, BrepFaceTessellationReport, BrepMeshHandoffReport,
    BrepShellTessellationReport, BrepTessellationBackend, BrepTessellationBlocker,
};
pub use topology::{
    BrepCoedge, BrepEdge, BrepEdgeId, BrepEdgeOrientation, BrepFace, BrepFaceId, BrepLoop,
    BrepLoopId, BrepShell, BrepVertex, BrepVertexId,
};
pub use trim::{BrepFaceTrimSetReport, BrepTrimLoopBlocker, BrepTrimLoopReport, BrepTrimLoopRole};
pub use validation::{
    BrepFaceValidationBlocker, BrepFaceValidationReport, BrepGeometryValidationBlocker,
    BrepGeometryValidationReport, BrepShellValidationBlocker, BrepShellValidationReport,
};
pub use volume::{BrepShellOrientation, BrepShellVolumeBlocker, BrepShellVolumeReport};

#[cfg(test)]
mod tests {
    use super::*;
    use hyperlimit::{Plane3, Point3};
    use hyperreal::Real;

    fn r(value: i32) -> Real {
        Real::from(value)
    }

    fn p(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    fn plane(id: u64, nx: i32, ny: i32, nz: i32, offset: i32) -> BrepSurface {
        BrepSurface::plane(
            BrepSurfaceId(id),
            Plane3::new(p(nx, ny, nz), r(offset)),
            BrepSurfaceSource::ExactConstruction,
        )
    }

    fn edge(id: u64, start: u64, end: u64) -> BrepEdge {
        BrepEdge::new(BrepEdgeId(id), BrepVertexId(start), BrepVertexId(end))
    }

    fn coedge(edge: u64, orientation: BrepEdgeOrientation) -> BrepCoedge {
        BrepCoedge::new(BrepEdgeId(edge), orientation)
    }

    fn face(id: u64, surface: u64, edges: &[(u64, BrepEdgeOrientation)]) -> BrepFace {
        BrepFace::new(
            BrepFaceId(id),
            BrepSurfaceId(surface),
            BrepLoop::new(
                BrepLoopId(id),
                edges
                    .iter()
                    .map(|(edge, orientation)| coedge(*edge, *orientation))
                    .collect(),
            ),
        )
    }

    fn cube_shell() -> BrepShell {
        use BrepEdgeOrientation::{Forward as F, Reversed as R};
        BrepShell {
            vertices: vec![
                BrepVertex::new(BrepVertexId(0), p(0, 0, 0)),
                BrepVertex::new(BrepVertexId(1), p(1, 0, 0)),
                BrepVertex::new(BrepVertexId(2), p(1, 1, 0)),
                BrepVertex::new(BrepVertexId(3), p(0, 1, 0)),
                BrepVertex::new(BrepVertexId(4), p(0, 0, 1)),
                BrepVertex::new(BrepVertexId(5), p(1, 0, 1)),
                BrepVertex::new(BrepVertexId(6), p(1, 1, 1)),
                BrepVertex::new(BrepVertexId(7), p(0, 1, 1)),
            ],
            edges: vec![
                edge(0, 0, 1),
                edge(1, 1, 2),
                edge(2, 2, 3),
                edge(3, 3, 0),
                edge(4, 4, 5),
                edge(5, 5, 6),
                edge(6, 6, 7),
                edge(7, 7, 4),
                edge(8, 0, 4),
                edge(9, 1, 5),
                edge(10, 2, 6),
                edge(11, 3, 7),
            ],
            surfaces: vec![
                plane(0, 0, 0, -1, 0),
                plane(1, 0, 0, 1, -1),
                plane(2, 0, -1, 0, 0),
                plane(3, 0, 1, 0, -1),
                plane(4, -1, 0, 0, 0),
                plane(5, 1, 0, 0, -1),
            ],
            faces: vec![
                face(0, 0, &[(0, R), (3, R), (2, R), (1, R)]),
                face(1, 1, &[(4, F), (5, F), (6, F), (7, F)]),
                face(2, 2, &[(0, F), (9, F), (4, R), (8, R)]),
                face(3, 3, &[(2, F), (11, F), (6, R), (10, R)]),
                face(4, 4, &[(3, F), (8, F), (7, R), (11, R)]),
                face(5, 5, &[(1, F), (10, F), (5, R), (9, R)]),
            ],
        }
    }

    #[test]
    fn cube_shell_reports_closed_exact_topology() {
        let shell = cube_shell();
        let report = shell.audit_closure();

        assert_eq!(report.counts.vertex_count, 8);
        assert_eq!(report.counts.edge_count, 12);
        assert_eq!(report.counts.face_count, 6);
        assert_eq!(report.boundary_edge_count, 0);
        assert_eq!(report.nonmanifold_edge_count, 0);
        assert_eq!(report.same_orientation_pair_count, 0);
        assert!(report.closed);
        assert!(report.exact_shell_ready);
        assert!(report.surface_inventory.all_exact_planar);
    }

    #[test]
    fn topology_validation_reports_graph_summaries_for_cube() {
        let shell = cube_shell();
        let report = shell.validate_topology();

        assert!(report.topology_ready);
        assert_eq!(report.counts.vertex_count, 8);
        assert_eq!(report.counts.edge_count, 12);
        assert_eq!(report.counts.face_count, 6);
        assert_eq!(report.euler_characteristic, 2);
        assert_eq!(report.connected_component_count, 1);
        assert_eq!(report.boundary_component_count, 0);
        assert_eq!(report.isolated_vertex_count, 0);
        assert_eq!(report.boundary_edge_count, 0);
        assert_eq!(report.nonmanifold_edge_count, 0);
        assert_eq!(report.same_orientation_pair_count, 0);
    }

    #[test]
    fn planar_face_area_report_derives_exact_projected_twice_area() {
        let shell = cube_shell();
        let report = shell.face_area_report(BrepFaceId(0));

        assert!(report.exact_area_ready);
        assert_eq!(report.projection_axis, Some(BrepAreaProjectionAxis::Z));
        assert_eq!(report.loop_count, 1);
        assert_eq!(report.boundary_vertex_count, 4);
        assert_eq!(report.signed_twice_projected_area, Some(r(2)));
        assert!(!report.zero_area);
        assert!(report.positive_area);
        assert!(!report.negative_area);
        assert!(report.blockers.is_empty());
    }

    #[test]
    fn shell_validation_aggregates_topology_closure_bounds_and_faces() {
        let shell = cube_shell();
        let report = shell.shell_validation_report();

        assert!(report.topology.topology_ready);
        assert!(report.closure.exact_shell_ready);
        assert!(report.bounds.exact_bounds_ready);
        assert_eq!(report.faces.len(), 6);
        assert_eq!(report.ready_face_boundary_count, 6);
        assert_eq!(report.blocked_face_boundary_count, 0);
        assert_eq!(report.ready_face_count, 6);
        assert_eq!(report.blocked_face_count, 0);
        assert!(report.exact_surface_boundary_ready);
        assert!(report.exact_closed_shell_ready);
        assert!(report.blockers.is_empty());
    }

    #[test]
    fn shell_audit_blocks_boundary_nonmanifold_duplicate_and_degenerate_cases() {
        let mut shell = cube_shell();
        shell.faces.pop();
        shell
            .vertices
            .push(BrepVertex::new(BrepVertexId(7), p(0, 1, 1)));
        shell.edges.push(edge(12, 0, 0));
        shell.surfaces.push(plane(6, 0, 0, 0, 0));
        shell.faces.push(face(
            99,
            99,
            &[
                (0, BrepEdgeOrientation::Forward),
                (0, BrepEdgeOrientation::Forward),
            ],
        ));

        let report = shell.audit_closure();
        assert!(!report.closed);
        assert!(!report.exact_shell_ready);
        assert!(
            report
                .blockers
                .contains(&BrepShellBlocker::DuplicateVertexId)
        );
        assert!(report.blockers.contains(&BrepShellBlocker::DegenerateEdge));
        assert!(
            report
                .blockers
                .contains(&BrepShellBlocker::ZeroNormalSurface)
        );
        assert!(
            report
                .blockers
                .contains(&BrepShellBlocker::MissingFaceSurface)
        );
        assert!(report.blockers.contains(&BrepShellBlocker::BoundaryEdges));
        assert!(
            report
                .blockers
                .contains(&BrepShellBlocker::NonmanifoldEdges)
        );

        let topology = shell.validate_topology();
        assert!(!topology.topology_ready);
        assert!(
            topology
                .blockers
                .contains(&BrepTopologyValidationBlocker::DuplicateVertexId)
        );
        assert!(
            topology
                .blockers
                .contains(&BrepTopologyValidationBlocker::DegenerateEdge)
        );
        assert!(
            topology
                .blockers
                .contains(&BrepTopologyValidationBlocker::BoundaryEdges)
        );
        assert!(
            topology
                .blockers
                .contains(&BrepTopologyValidationBlocker::NonmanifoldEdges)
        );
        assert!(topology.boundary_component_count > 0);

        let validation = shell.shell_validation_report();
        assert!(!validation.exact_surface_boundary_ready);
        assert!(!validation.exact_closed_shell_ready);
        assert!(
            validation
                .blockers
                .contains(&BrepShellValidationBlocker::TopologyNotReady)
        );
        assert!(
            validation
                .blockers
                .contains(&BrepShellValidationBlocker::ShellClosureNotReady)
        );
        assert!(
            validation
                .blockers
                .contains(&BrepShellValidationBlocker::FaceValidationNotReady)
        );

        let area = shell.face_area_report(BrepFaceId(99));
        assert!(!area.exact_area_ready);
        assert!(area.blockers.contains(&BrepFaceAreaBlocker::MissingSurface));
    }

    #[test]
    fn planar_face_area_report_blocks_broken_or_degenerate_loop_evidence() {
        let mut shell = cube_shell();
        shell.edges.push(edge(99, 0, 0));
        shell.faces.push(face(
            99,
            0,
            &[
                (99, BrepEdgeOrientation::Forward),
                (0, BrepEdgeOrientation::Forward),
            ],
        ));
        let report = shell.face_area_report(BrepFaceId(99));

        assert!(!report.exact_area_ready);
        assert!(
            report
                .blockers
                .contains(&BrepFaceAreaBlocker::DegenerateEdge)
        );
        assert!(
            report
                .blockers
                .contains(&BrepFaceAreaBlocker::BrokenLoopChain)
        );
    }

    #[test]
    fn shell_volume_report_derives_exact_signed_volume_and_orientation() {
        let shell = cube_shell();
        let report = shell.shell_volume_report();

        assert!(report.exact_volume_ready);
        assert_eq!(report.face_count, 6);
        assert_eq!(report.loop_count, 6);
        assert_eq!(report.ready_face_count, 6);
        assert_eq!(report.blocked_face_count, 0);
        assert_eq!(report.signed_six_volume, Some(r(6)));
        assert_eq!(report.orientation, Some(BrepShellOrientation::Positive));
        assert!(report.positive_volume);
        assert!(!report.negative_volume);
        assert!(!report.zero_volume);
        assert!(report.blockers.is_empty());

        let mut reversed = shell.clone();
        for face in &mut reversed.faces {
            face.outer.coedges.reverse();
            for coedge in &mut face.outer.coedges {
                coedge.orientation = match coedge.orientation {
                    BrepEdgeOrientation::Forward => BrepEdgeOrientation::Reversed,
                    BrepEdgeOrientation::Reversed => BrepEdgeOrientation::Forward,
                };
            }
        }
        let reversed_report = reversed.shell_volume_report();
        assert!(reversed_report.exact_volume_ready);
        assert_eq!(reversed_report.signed_six_volume, Some(r(-6)));
        assert_eq!(
            reversed_report.orientation,
            Some(BrepShellOrientation::Negative)
        );
    }

    #[test]
    fn shell_volume_report_blocks_open_or_degenerate_evidence() {
        let mut shell = cube_shell();
        shell.faces.pop();
        shell.edges[0] = edge(0, 0, 0);
        let report = shell.shell_volume_report();

        assert!(!report.exact_volume_ready);
        assert!(
            report
                .blockers
                .contains(&BrepShellVolumeBlocker::ShellClosureNotReady)
        );
        assert!(
            report
                .blockers
                .contains(&BrepShellVolumeBlocker::FaceValidationNotReady)
        );
        assert!(
            report
                .blockers
                .contains(&BrepShellVolumeBlocker::DegenerateEdge)
        );
    }

    #[test]
    fn surface_inventory_keeps_unsupported_and_lossy_sources_explicit() {
        let surfaces = vec![
            plane(0, 0, 0, 1, 0),
            BrepSurface::unsupported(
                BrepSurfaceId(1),
                "nurbs-surface",
                BrepSurfaceSource::LossyImport,
            ),
        ];
        let report = BrepSurfaceInventoryReport::from_surfaces(&surfaces);

        assert_eq!(report.surface_count, 2);
        assert_eq!(report.planar_count, 1);
        assert_eq!(report.unsupported_count, 1);
        assert_eq!(report.lossy_source_count, 1);
        assert!(!report.all_exact_planar);
    }

    #[test]
    fn exact_face_tessellation_manifest_enables_derived_mesh_handoff() {
        let shell = cube_shell();
        let manifests = shell
            .faces
            .iter()
            .map(|face| {
                let boundary_edges = face.loops().map(|face_loop| face_loop.coedges.len()).sum();
                BrepFaceTessellationManifest::exact_planar(face.id, 2, 4, boundary_edges, 0)
            })
            .collect::<Vec<_>>();
        let shell_report = BrepShellTessellationReport::from_shell_manifests(&shell, &manifests);
        assert_eq!(shell_report.source_face_count, 6);
        assert_eq!(shell_report.ready_face_count, 6);
        assert_eq!(shell_report.blocked_face_count, 0);
        assert_eq!(shell_report.triangle_count, 12);
        assert_eq!(shell_report.lifted_vertex_count, 24);
        assert_eq!(shell_report.boundary_edge_count, 24);
        assert!(shell_report.exact_surface_handoff_ready);
        assert!(shell_report.exact_solid_handoff_ready);
        assert!(shell_report.derived_mesh_only);

        let report = BrepMeshHandoffReport::from_shell_manifests(&shell, &manifests);

        assert_eq!(report.ready_face_count, 6);
        assert_eq!(report.blocked_face_count, 0);
        assert_eq!(report.triangle_count, 12);
        assert!(report.exact_surface_handoff_ready);
        assert!(report.exact_solid_handoff_ready);
        assert!(report.derived_mesh_only);
        assert_eq!(report.tessellation, shell_report);
    }

    #[test]
    fn tessellation_manifest_blocks_lossy_or_unreplayed_output() {
        let shell = cube_shell();
        let mut manifest = BrepFaceTessellationManifest::exact_planar(BrepFaceId(0), 0, 0, 1, 2);
        manifest.backend = BrepTessellationBackend::LossyPreviewAdapter;
        manifest.exact_uv_triangulation = false;
        manifest.exact_lifted_incidence = false;
        manifest.preserves_boundary_edges = false;
        manifest.lossy_adapter_output = true;

        let report =
            BrepFaceTessellationReport::from_shell_face(&shell, BrepFaceId(0), Some(&manifest));
        assert!(!report.exact_surface_handoff_ready);
        assert!(report.derived_mesh_only);
        assert!(
            report
                .blockers
                .contains(&BrepTessellationBlocker::LossyBackend)
        );
        assert!(
            report
                .blockers
                .contains(&BrepTessellationBlocker::EmptyTriangleSet)
        );
        assert!(
            report
                .blockers
                .contains(&BrepTessellationBlocker::EmptyLiftedVertices)
        );
        assert!(
            report
                .blockers
                .contains(&BrepTessellationBlocker::MissingExactUvReplay)
        );
        assert!(
            report
                .blockers
                .contains(&BrepTessellationBlocker::MissingLiftedIncidenceReplay)
        );
        assert!(
            report
                .blockers
                .contains(&BrepTessellationBlocker::MissingBoundaryReplay)
        );
        assert!(
            report
                .blockers
                .contains(&BrepTessellationBlocker::LossyAdapterOutput)
        );
    }

    #[test]
    fn tessellation_report_requires_ready_source_trim_loop() {
        let mut shell = cube_shell();
        shell.faces[0].outer.coedges.swap(1, 2);
        let manifest = BrepFaceTessellationManifest::exact_planar(BrepFaceId(0), 2, 4, 4, 0);
        let report =
            BrepFaceTessellationReport::from_shell_face(&shell, BrepFaceId(0), Some(&manifest));

        assert!(!report.trim_set_ready);
        assert!(!report.exact_surface_handoff_ready);
        assert!(
            report
                .blockers
                .contains(&BrepTessellationBlocker::TrimLoopNotReady)
        );
    }

    #[test]
    fn shell_tessellation_report_blocks_missing_or_unready_face_manifests() {
        let mut shell = cube_shell();
        shell.faces[0].outer.coedges.swap(1, 2);
        let manifests = shell
            .faces
            .iter()
            .skip(1)
            .map(|face| {
                let boundary_edges = face.loops().map(|face_loop| face_loop.coedges.len()).sum();
                BrepFaceTessellationManifest::exact_planar(face.id, 2, 4, boundary_edges, 0)
            })
            .collect::<Vec<_>>();
        let report = BrepShellTessellationReport::from_shell_manifests(&shell, &manifests);

        assert_eq!(report.source_face_count, 6);
        assert_eq!(report.ready_face_count, 5);
        assert_eq!(report.blocked_face_count, 1);
        assert!(!report.exact_surface_handoff_ready);
        assert!(!report.exact_solid_handoff_ready);
        assert!(
            report.faces[0]
                .blockers
                .contains(&BrepTessellationBlocker::MissingManifest)
        );
        assert!(
            report.faces[0]
                .blockers
                .contains(&BrepTessellationBlocker::TrimLoopNotReady)
        );
    }

    #[test]
    fn construction_manifest_blocks_stale_or_unreplayed_handoffs() {
        let shell = cube_shell();
        let feature = BrepFeatureId::new("feature:cube").unwrap();
        let source = BrepSourceVersion::new("sketch:square", 3).unwrap();
        let manifest = BrepConstructionManifest::exact(
            feature,
            BrepConstructionKind::Extrusion,
            vec![source],
            &shell,
        );
        let report = manifest.report(&shell);
        assert!(report.construction_fresh);
        assert!(report.topology_snapshot_current);
        assert!(report.replay_accepted);

        let mut stale_shell = shell.clone();
        stale_shell.faces.pop();
        let stale_report = manifest.report(&stale_shell);
        assert!(!stale_report.construction_fresh);
        assert!(!stale_report.topology_snapshot_current);
        assert!(
            stale_report
                .blockers
                .contains(&BrepConstructionBlocker::StaleTopologySnapshot)
        );

        let mut rejected = manifest.clone();
        rejected.replay_status = BrepConstructionReplayStatus::Rejected;
        rejected
            .adapter_diagnostics
            .push("external sew tolerance used".into());
        rejected
            .selected_references
            .push(BrepSelectedReference::Face(BrepFaceId(99)));
        let rejected_report = rejected.report(&shell);
        assert!(!rejected_report.construction_fresh);
        assert!(
            rejected_report
                .blockers
                .contains(&BrepConstructionBlocker::ReplayNotAccepted)
        );
        assert!(
            rejected_report
                .blockers
                .contains(&BrepConstructionBlocker::AdapterDiagnosticsPresent)
        );
        assert!(
            rejected_report
                .blockers
                .contains(&BrepConstructionBlocker::MissingSelectedReference)
        );
    }

    #[test]
    fn mesh_handoff_replays_construction_freshness() {
        let shell = cube_shell();
        let manifests = shell
            .faces
            .iter()
            .map(|face| {
                let boundary_edges = face.loops().map(|face_loop| face_loop.coedges.len()).sum();
                BrepFaceTessellationManifest::exact_planar(face.id, 2, 4, boundary_edges, 0)
            })
            .collect::<Vec<_>>();
        let construction = BrepConstructionManifest::exact(
            BrepFeatureId::new("feature:cube").unwrap(),
            BrepConstructionKind::Extrusion,
            vec![BrepSourceVersion::new("sketch:square", 4).unwrap()],
            &shell,
        );
        let ready = BrepMeshHandoffReport::from_shell_manifests_with_construction(
            &shell,
            &manifests,
            Some(&construction),
        );
        assert!(ready.exact_solid_handoff_ready);
        assert!(ready.construction.as_ref().unwrap().construction_fresh);

        let mut stale_shell = shell.clone();
        stale_shell.edges.push(edge(99, 0, 0));
        let stale = BrepMeshHandoffReport::from_shell_manifests_with_construction(
            &stale_shell,
            &manifests,
            Some(&construction),
        );
        assert!(!stale.exact_solid_handoff_ready);
        assert!(!stale.construction.as_ref().unwrap().construction_fresh);
    }

    #[test]
    fn solid_readiness_packages_closed_shell_bounds_faces_and_construction() {
        let shell = cube_shell();
        let construction = BrepConstructionManifest::exact(
            BrepFeatureId::new("feature:cube-solid").unwrap(),
            BrepConstructionKind::Extrusion,
            vec![BrepSourceVersion::new("sketch:square", 5).unwrap()],
            &shell,
        );
        let report = shell.solid_readiness_report(Some(&construction));

        assert!(report.closed_shell_ready);
        assert!(report.all_faces_ready);
        assert!(report.exact_bounds_ready);
        assert!(report.construction_fresh);
        assert!(report.exact_volume_ready);
        assert!(report.exact_solid_boundary_ready);
        assert_eq!(report.ready_face_count, 6);
        assert_eq!(report.blocked_face_count, 0);
        assert_eq!(report.shell_bounds.min, Some(p(0, 0, 0)));
        assert_eq!(report.shell_bounds.max, Some(p(1, 1, 1)));
        assert_eq!(report.faces.len(), 6);
        assert_eq!(report.volume.signed_six_volume, Some(r(6)));
        assert_eq!(
            report.volume.orientation,
            Some(BrepShellOrientation::Positive)
        );
        assert!(report.blockers.is_empty());
    }

    #[test]
    fn solid_readiness_blocks_open_stale_or_invalid_shells() {
        let shell = cube_shell();
        let construction = BrepConstructionManifest::exact(
            BrepFeatureId::new("feature:cube-solid").unwrap(),
            BrepConstructionKind::Extrusion,
            vec![BrepSourceVersion::new("sketch:square", 6).unwrap()],
            &shell,
        );
        let mut broken = shell.clone();
        broken.faces.pop();
        broken.edges[0] = edge(0, 0, 0);
        let report = broken.solid_readiness_report(Some(&construction));

        assert!(!report.exact_solid_boundary_ready);
        assert!(!report.closed_shell_ready);
        assert!(!report.all_faces_ready);
        assert!(!report.exact_volume_ready);
        assert!(!report.construction_fresh);
        assert!(
            report
                .blockers
                .contains(&BrepSolidReadinessBlocker::ShellClosureNotReady)
        );
        assert!(
            report
                .blockers
                .contains(&BrepSolidReadinessBlocker::FaceValidationNotReady)
        );
        assert!(
            report
                .blockers
                .contains(&BrepSolidReadinessBlocker::ConstructionNotFresh)
        );
        assert!(
            report
                .blockers
                .contains(&BrepSolidReadinessBlocker::VolumeReplayUnavailable)
        );
    }

    #[test]
    fn lossy_float_import_audit_keeps_finite_dyadic_lift_separate_from_topology() {
        let report = BrepLossyFloatImportReport::inspect_f64(
            "viewer-obj",
            &[0.0, 0.5, -1.25, 2.0, 3.0, 4.0],
            &[BrepImportedSurfaceFamily::Plane],
            true,
            true,
        );

        assert_eq!(report.precision, BrepPrimitiveFloatPrecision::F64);
        assert_eq!(report.coordinate_count, 6);
        assert_eq!(report.point_count, 2);
        assert_eq!(report.finite_coordinate_count, 6);
        assert_eq!(report.exact_dyadic_lift_count, 6);
        assert_eq!(report.exact_decimal_lift_count, 0);
        assert!(report.adapter_replay_ready);
        assert!(report.lossy_adapter_only);
    }

    #[test]
    fn lossy_float_import_audit_blocks_nonfinite_missing_tolerance_and_surface_gaps() {
        let report = BrepLossyFloatImportReport::inspect_f64(
            "step-preview",
            &[0.0, f64::NAN, 1.0, f64::INFINITY],
            &[
                BrepImportedSurfaceFamily::Plane,
                BrepImportedSurfaceFamily::Nurbs,
                BrepImportedSurfaceFamily::Unknown,
            ],
            false,
            false,
        );

        assert!(!report.adapter_replay_ready);
        assert_eq!(report.point_count, 1);
        assert_eq!(report.non_finite_coordinate_indexes, vec![1, 3]);
        assert_eq!(report.unsupported_surfaces.len(), 2);
        assert_eq!(report.unknown_surface_count, 1);
        assert!(
            report
                .blockers
                .contains(&BrepLossyImportBlocker::InvalidCoordinateArity)
        );
        assert!(
            report
                .blockers
                .contains(&BrepLossyImportBlocker::NonFiniteCoordinate)
        );
        assert!(
            report
                .blockers
                .contains(&BrepLossyImportBlocker::MissingTopologyEvidence)
        );
        assert!(
            report
                .blockers
                .contains(&BrepLossyImportBlocker::MissingTolerance)
        );
        assert!(
            report
                .blockers
                .contains(&BrepLossyImportBlocker::UnsupportedSurfaceKind)
        );
        assert!(
            report
                .blockers
                .contains(&BrepLossyImportBlocker::UnknownSurfaceKind)
        );
    }

    #[test]
    fn mesh_export_requires_ready_handoff_and_keeps_adapter_boundary() {
        let shell = cube_shell();
        let manifests = shell
            .faces
            .iter()
            .map(|face| {
                let boundary_edges = face.loops().map(|face_loop| face_loop.coedges.len()).sum();
                BrepFaceTessellationManifest::exact_planar(face.id, 2, 4, boundary_edges, 0)
            })
            .collect::<Vec<_>>();
        let mesh_handoff = BrepMeshHandoffReport::from_shell_manifests(&shell, &manifests);
        let manifest = BrepExportManifest {
            format: BrepExportFormat::Obj,
            scalar_policy: BrepExportScalarPolicy::F64,
            source_object_ids: vec!["shell:cube".into()],
            exported_primitives: mesh_handoff.triangle_count,
            exported_coordinates: mesh_handoff.lifted_vertex_count * 3,
            finite_exported_coordinates: mesh_handoff.lifted_vertex_count * 3,
            labels_preserved: true,
            exact_replay_declared: true,
        };
        let report = manifest.report(Some(&mesh_handoff));

        assert!(report.export_ready);
        assert!(report.mesh_handoff_ready);
        assert!(report.export_adapter_only);
    }

    #[test]
    fn export_report_blocks_missing_sources_nonfinite_coords_and_unreplayed_external_brep() {
        let manifest = BrepExportManifest {
            format: BrepExportFormat::Step,
            scalar_policy: BrepExportScalarPolicy::Unknown,
            source_object_ids: vec!["".into()],
            exported_primitives: 0,
            exported_coordinates: 9,
            finite_exported_coordinates: 8,
            labels_preserved: false,
            exact_replay_declared: false,
        };
        let report = manifest.report(None);

        assert!(!report.export_ready);
        assert!(report.export_adapter_only);
        assert!(
            report
                .blockers
                .contains(&BrepExportBlocker::UnknownScalarPolicy)
        );
        assert!(
            report
                .blockers
                .contains(&BrepExportBlocker::MissingSourceObjectIds)
        );
        assert!(report.blockers.contains(&BrepExportBlocker::EmptyExport));
        assert!(
            report
                .blockers
                .contains(&BrepExportBlocker::NonFiniteExportCoordinates)
        );
        assert!(
            report
                .blockers
                .contains(&BrepExportBlocker::ExternalBrepReplayMissing)
        );
    }

    #[test]
    fn mesh_export_blocks_missing_or_unready_mesh_handoff() {
        let manifest = BrepExportManifest {
            format: BrepExportFormat::Gltf,
            scalar_policy: BrepExportScalarPolicy::F32,
            source_object_ids: vec!["shell:cube".into()],
            exported_primitives: 12,
            exported_coordinates: 72,
            finite_exported_coordinates: 72,
            labels_preserved: true,
            exact_replay_declared: false,
        };
        let missing = manifest.report(None);
        assert!(!missing.export_ready);
        assert!(
            missing
                .blockers
                .contains(&BrepExportBlocker::MissingMeshHandoff)
        );

        let mut open_shell = cube_shell();
        open_shell.faces.pop();
        let unready = BrepMeshHandoffReport::from_shell_manifests(&open_shell, &[]);
        let blocked = manifest.report(Some(&unready));
        assert!(!blocked.export_ready);
        assert!(
            blocked
                .blockers
                .contains(&BrepExportBlocker::MeshHandoffNotReady)
        );
    }

    #[test]
    fn prepared_plane_surface_classifies_points_with_exact_replay() {
        let surface = plane(10, 0, 0, 1, -2);
        let prepared = surface.prepare();
        assert!(prepared.exact_replay_ready());
        assert!(prepared.facts().dyadic_schedule);

        let below = prepared.classify_point(&p(0, 0, 1));
        let on = prepared.classify_point(&p(0, 0, 2));
        let above = prepared.classify_point(&p(0, 0, 3));

        assert_eq!(below.side, Some(hyperlimit::PlaneSide::Below));
        assert_eq!(on.side, Some(hyperlimit::PlaneSide::On));
        assert_eq!(above.side, Some(hyperlimit::PlaneSide::Above));
        assert!(below.exact_replay);
        assert!(on.on_surface);
    }

    #[test]
    fn prepared_surface_blocks_lossy_unsupported_or_zero_normal_sources() {
        let lossy_surface = BrepSurface::plane(
            BrepSurfaceId(11),
            Plane3::new(p(0, 0, 1), r(0)),
            BrepSurfaceSource::LossyImport,
        );
        let lossy = lossy_surface.prepare();
        assert!(!lossy.exact_replay_ready());
        assert!(
            lossy
                .classify_point(&p(0, 0, 0))
                .blockers
                .contains(&BrepSurfaceBlocker::NonExactSource)
        );

        let unsupported_surface =
            BrepSurface::unsupported(BrepSurfaceId(12), "nurbs", BrepSurfaceSource::ExactImport);
        let unsupported = unsupported_surface.prepare();
        assert!(
            unsupported
                .classify_point(&p(0, 0, 0))
                .blockers
                .contains(&BrepSurfaceBlocker::UnsupportedFamily)
        );

        let zero_normal_surface = BrepSurface::plane(
            BrepSurfaceId(13),
            Plane3::new(p(0, 0, 0), r(0)),
            BrepSurfaceSource::ExactConstruction,
        );
        let zero_normal = zero_normal_surface.prepare();
        assert!(
            zero_normal
                .classify_point(&p(0, 0, 0))
                .blockers
                .contains(&BrepSurfaceBlocker::ZeroNormal)
        );
    }

    #[test]
    fn cube_face_trim_set_reports_closed_outer_loop() {
        let shell = cube_shell();
        let report = shell.trim_set_report(BrepFaceId(0));

        assert!(report.face_found);
        assert_eq!(report.surface, Some(BrepSurfaceId(0)));
        assert!(report.outer_ready);
        assert_eq!(report.blocked_loop_count, 0);
        assert!(report.trim_set_ready);
        assert_eq!(report.loops.len(), 1);
        assert_eq!(report.loops[0].role, BrepTrimLoopRole::Outer);
        assert!(report.loops[0].closed_vertex_chain);
        assert!(report.loops[0].surface_replay_ready);
        assert!(report.loops[0].trim_loop_ready);
    }

    #[test]
    fn cube_face_bounds_report_derives_exact_vertex_aabb() {
        let shell = cube_shell();
        let report = shell.face_bounds_report(BrepFaceId(0));

        assert!(report.exact_bounds_ready);
        assert_eq!(report.vertex_count, 4);
        assert_eq!(report.min, Some(p(0, 0, 0)));
        assert_eq!(report.max, Some(p(1, 1, 0)));
        assert!(report.zero_z_extent);
        assert_eq!(report.zero_extent_axis_count, 1);

        let prepared = report.prepare().unwrap();
        assert_eq!(
            prepared.prepared.classify_point(&p(0, 0, 0)).value(),
            Some(hyperlimit::Aabb3PointLocation::Boundary)
        );
        assert_eq!(
            prepared.prepared.classify_point(&p(1, 1, 1)).value(),
            Some(hyperlimit::Aabb3PointLocation::Outside)
        );
    }

    #[test]
    fn cube_shell_bounds_report_derives_exact_shell_aabb() {
        let shell = cube_shell();
        let report = shell.shell_bounds_report();

        assert!(report.exact_bounds_ready);
        assert_eq!(report.vertex_count, 8);
        assert_eq!(report.face_count, 6);
        assert_eq!(report.min, Some(p(0, 0, 0)));
        assert_eq!(report.max, Some(p(1, 1, 1)));
        assert_eq!(report.zero_extent_axis_count, 0);

        let prepared = report.prepare().unwrap();
        assert_eq!(
            prepared.prepared.classify_point(&p(0, 0, 0)).value(),
            Some(hyperlimit::Aabb3PointLocation::Boundary)
        );
        assert_eq!(
            prepared.prepared.classify_point(&p(1, 1, 1)).value(),
            Some(hyperlimit::Aabb3PointLocation::Boundary)
        );
    }

    #[test]
    fn shell_bounds_report_blocks_empty_shells() {
        let shell = BrepShell {
            vertices: Vec::new(),
            edges: Vec::new(),
            surfaces: Vec::new(),
            faces: Vec::new(),
        };
        let report = shell.shell_bounds_report();

        assert!(!report.exact_bounds_ready);
        assert!(report.min.is_none());
        assert!(report.max.is_none());
        assert!(
            report
                .blockers
                .contains(&BrepShellBoundsBlocker::EmptyShell)
        );
    }

    #[test]
    fn face_bounds_report_blocks_missing_or_degenerate_boundary_evidence() {
        let mut shell = cube_shell();
        shell.edges[0] = edge(0, 0, 0);
        shell.faces[0]
            .outer
            .coedges
            .push(coedge(999, BrepEdgeOrientation::Forward));
        shell.edges[1] = edge(1, 1, 999);
        let report = shell.face_bounds_report(BrepFaceId(0));

        assert!(!report.exact_bounds_ready);
        assert!(report.min.is_none());
        assert!(report.max.is_none());
        assert!(
            report
                .blockers
                .contains(&BrepFaceBoundsBlocker::DegenerateEdge)
        );
        assert!(
            report
                .blockers
                .contains(&BrepFaceBoundsBlocker::MissingEdge)
        );
        assert!(
            report
                .blockers
                .contains(&BrepFaceBoundsBlocker::MissingVertex)
        );
    }

    #[test]
    fn face_aabb_preflight_certifies_disjoint_or_candidate_face_pairs() {
        let shell = cube_shell();
        let disjoint = shell.face_aabb_preflight(BrepFaceId(0), BrepFaceId(1));
        assert!(disjoint.preflight_ready);
        assert_eq!(
            disjoint.relation,
            Some(hyperlimit::Aabb3Intersection::Disjoint)
        );
        assert!(disjoint.certified_disjoint);
        assert!(!disjoint.requires_narrow_phase);

        let touching = shell.face_aabb_preflight(BrepFaceId(0), BrepFaceId(2));
        assert!(touching.preflight_ready);
        assert_eq!(
            touching.relation,
            Some(hyperlimit::Aabb3Intersection::Touching)
        );
        assert!(!touching.certified_disjoint);
        assert!(touching.requires_narrow_phase);
    }

    #[test]
    fn face_aabb_preflight_blocks_unready_bounds() {
        let mut shell = cube_shell();
        shell.faces[0]
            .outer
            .coedges
            .push(coedge(999, BrepEdgeOrientation::Forward));
        let report = shell.face_aabb_preflight(BrepFaceId(0), BrepFaceId(1));

        assert!(!report.preflight_ready);
        assert!(report.relation.is_none());
        assert!(
            report
                .blockers
                .contains(&BrepFaceAabbPreflightBlocker::FirstBoundsNotReady)
        );
    }

    #[test]
    fn face_plane_preflight_certifies_broad_phase_plane_relation() {
        let shell = cube_shell();
        let coplanar_bounds = Plane3::new(p(0, 0, 1), r(0));
        let touching = shell.face_plane_preflight(BrepFaceId(0), &coplanar_bounds);
        assert!(touching.preflight_ready);
        assert_eq!(
            touching.relation,
            Some(hyperlimit::PlaneAabbRelation::Intersecting)
        );
        assert!(touching.requires_narrow_phase);
        assert!(!touching.certified_no_plane_crossing);

        let above_face = Plane3::new(p(0, 0, 1), r(-2));
        let rejected = shell.face_plane_preflight(BrepFaceId(0), &above_face);
        assert!(rejected.preflight_ready);
        assert_eq!(
            rejected.relation,
            Some(hyperlimit::PlaneAabbRelation::Below)
        );
        assert!(rejected.certified_no_plane_crossing);
        assert!(!rejected.requires_narrow_phase);
    }

    #[test]
    fn face_plane_preflight_blocks_unready_face_bounds() {
        let mut shell = cube_shell();
        shell.faces[0]
            .outer
            .coedges
            .push(coedge(999, BrepEdgeOrientation::Forward));
        let plane = Plane3::new(p(0, 0, 1), r(0));
        let report = shell.face_plane_preflight(BrepFaceId(0), &plane);

        assert!(!report.preflight_ready);
        assert!(report.relation.is_none());
        assert!(
            report
                .blockers
                .contains(&BrepFacePlanePreflightBlocker::FaceBoundsNotReady)
        );
    }

    #[test]
    fn segment_face_plane_preflight_classifies_support_plane_relation() {
        let shell = cube_shell();
        let crossing = shell.segment_face_plane_preflight(BrepFaceId(0), &p(0, 0, -1), &p(0, 0, 1));
        assert!(crossing.preflight_ready);
        assert_eq!(
            crossing.relation,
            Some(hyperlimit::PlaneSegmentRelation::Crossing)
        );
        assert!(crossing.requires_narrow_phase);
        assert!(!crossing.certified_no_plane_contact);

        let above = shell.segment_face_plane_preflight(BrepFaceId(0), &p(0, 0, -2), &p(1, 0, -2));
        assert!(above.preflight_ready);
        assert_eq!(
            above.relation,
            Some(hyperlimit::PlaneSegmentRelation::Above)
        );
        assert!(above.certified_no_plane_contact);
        assert!(!above.requires_narrow_phase);

        let coplanar = shell.segment_face_plane_preflight(BrepFaceId(0), &p(0, 0, 0), &p(1, 0, 0));
        assert!(coplanar.preflight_ready);
        assert_eq!(
            coplanar.relation,
            Some(hyperlimit::PlaneSegmentRelation::Coplanar)
        );
        assert!(coplanar.requires_narrow_phase);
    }

    #[test]
    fn segment_face_plane_preflight_blocks_missing_or_unready_surfaces() {
        let mut shell = cube_shell();
        shell.faces[0].surface = BrepSurfaceId(999);
        let missing = shell.segment_face_plane_preflight(BrepFaceId(0), &p(0, 0, 0), &p(0, 0, 1));
        assert!(!missing.preflight_ready);
        assert!(
            missing
                .blockers
                .contains(&BrepSegmentFacePlaneBlocker::MissingSurface)
        );

        let mut unsupported = cube_shell();
        unsupported.surfaces[0] =
            BrepSurface::unsupported(BrepSurfaceId(0), "nurbs", BrepSurfaceSource::ExactImport);
        let report =
            unsupported.segment_face_plane_preflight(BrepFaceId(0), &p(0, 0, 0), &p(0, 0, 1));
        assert!(!report.preflight_ready);
        assert!(
            report
                .blockers
                .contains(&BrepSegmentFacePlaneBlocker::UnsupportedSurface)
        );
        assert!(
            report
                .blockers
                .contains(&BrepSegmentFacePlaneBlocker::SurfaceNotReady)
        );
    }

    #[test]
    fn point_face_plane_preflight_classifies_support_plane_side() {
        let shell = cube_shell();
        let on = shell.point_face_plane_preflight(BrepFaceId(0), &p(0, 0, 0));
        assert!(on.preflight_ready);
        assert_eq!(on.side, Some(hyperlimit::PlaneSide::On));
        assert!(on.on_support_plane);
        assert!(on.requires_trim_replay);
        assert!(!on.certified_off_support_plane);

        let off = shell.point_face_plane_preflight(BrepFaceId(0), &p(0, 0, 1));
        assert!(off.preflight_ready);
        assert_eq!(off.side, Some(hyperlimit::PlaneSide::Below));
        assert!(off.certified_off_support_plane);
        assert!(!off.requires_trim_replay);
    }

    #[test]
    fn point_face_plane_preflight_blocks_missing_or_unready_surfaces() {
        let mut shell = cube_shell();
        shell.faces[0].surface = BrepSurfaceId(999);
        let missing = shell.point_face_plane_preflight(BrepFaceId(0), &p(0, 0, 0));
        assert!(!missing.preflight_ready);
        assert!(
            missing
                .blockers
                .contains(&BrepPointFacePlaneBlocker::MissingSurface)
        );

        let mut unsupported = cube_shell();
        unsupported.surfaces[0] =
            BrepSurface::unsupported(BrepSurfaceId(0), "nurbs", BrepSurfaceSource::ExactImport);
        let report = unsupported.point_face_plane_preflight(BrepFaceId(0), &p(0, 0, 0));
        assert!(!report.preflight_ready);
        assert!(
            report
                .blockers
                .contains(&BrepPointFacePlaneBlocker::UnsupportedSurface)
        );
        assert!(
            report
                .blockers
                .contains(&BrepPointFacePlaneBlocker::SurfaceNotReady)
        );
    }

    #[test]
    fn face_validation_packages_surface_trim_and_optional_tessellation_evidence() {
        let shell = cube_shell();
        let manifest = BrepFaceTessellationManifest::exact_planar(BrepFaceId(0), 2, 4, 4, 0);
        let report = shell.face_validation_report(BrepFaceId(0), Some(&manifest));

        assert!(report.face_found);
        assert!(report.exact_face_boundary_ready);
        assert!(report.exact_bounds_ready);
        assert!(report.tessellation_ready);
        assert!(report.exact_face_ready);
        assert!(report.surface_facts.as_ref().unwrap().exact_replay_ready);
        assert!(report.trim_set.as_ref().unwrap().trim_set_ready);
        assert!(report.bounds.as_ref().unwrap().exact_bounds_ready);
        assert!(report.geometry.as_ref().unwrap().geometry_ready);
        assert!(
            report
                .tessellation
                .as_ref()
                .unwrap()
                .exact_surface_handoff_ready
        );
    }

    #[test]
    fn face_validation_blocks_unready_surface_trim_or_tessellation_evidence() {
        let mut shell = cube_shell();
        shell.surfaces[0] = BrepSurface::plane(
            BrepSurfaceId(0),
            Plane3::new(p(0, 0, 0), r(0)),
            BrepSurfaceSource::ExactConstruction,
        );
        shell.faces[0].outer.coedges.swap(1, 2);
        shell.edges[0] = edge(0, 0, 999);
        let mut manifest = BrepFaceTessellationManifest::exact_planar(BrepFaceId(0), 0, 0, 4, 0);
        manifest.exact_uv_triangulation = false;
        let report = shell.face_validation_report(BrepFaceId(0), Some(&manifest));

        assert!(!report.exact_face_ready);
        assert!(!report.exact_bounds_ready);
        assert!(
            report
                .blockers
                .contains(&BrepFaceValidationBlocker::SurfaceNotReady)
        );
        assert!(
            report
                .blockers
                .contains(&BrepFaceValidationBlocker::TrimSetNotReady)
        );
        assert!(
            report
                .blockers
                .contains(&BrepFaceValidationBlocker::BoundsNotReady)
        );
        assert!(
            report
                .blockers
                .contains(&BrepFaceValidationBlocker::GeometryNotReady)
        );
        assert!(
            report
                .blockers
                .contains(&BrepFaceValidationBlocker::TessellationNotReady)
        );
    }

    #[test]
    fn geometry_validation_certifies_boundary_vertices_on_support_plane() {
        let shell = cube_shell();
        let report = shell.geometry_validation_report(BrepFaceId(0));

        assert!(report.geometry_ready);
        assert!(report.trim_set_ready);
        assert_eq!(report.boundary_vertex_count, 4);
        assert_eq!(report.on_surface_vertex_count, 4);
        assert_eq!(report.off_surface_vertex_count, 0);
        assert_eq!(report.unknown_vertex_surface_count, 0);
    }

    #[test]
    fn geometry_validation_blocks_off_surface_missing_and_unsupported_evidence() {
        let mut off_surface = cube_shell();
        off_surface.vertices[0] = BrepVertex::new(BrepVertexId(0), p(0, 0, 2));
        let off_report = off_surface.geometry_validation_report(BrepFaceId(0));
        assert!(!off_report.geometry_ready);
        assert_eq!(off_report.off_surface_vertex_count, 1);
        assert!(
            off_report
                .blockers
                .contains(&BrepGeometryValidationBlocker::BoundaryVertexOffSurface)
        );

        let mut missing = cube_shell();
        missing.edges[0] = edge(0, 0, 999);
        let missing_report = missing.geometry_validation_report(BrepFaceId(0));
        assert!(!missing_report.geometry_ready);
        assert_eq!(missing_report.missing_vertex_count, 1);
        assert!(
            missing_report
                .blockers
                .contains(&BrepGeometryValidationBlocker::MissingVertex)
        );

        let mut unsupported = cube_shell();
        unsupported.surfaces[0] =
            BrepSurface::unsupported(BrepSurfaceId(0), "nurbs", BrepSurfaceSource::ExactImport);
        let unsupported_report = unsupported.geometry_validation_report(BrepFaceId(0));
        assert!(!unsupported_report.geometry_ready);
        assert!(
            unsupported_report
                .blockers
                .contains(&BrepGeometryValidationBlocker::UnsupportedSurface)
        );
        assert!(
            unsupported_report
                .blockers
                .contains(&BrepGeometryValidationBlocker::SurfaceNotReady)
        );
    }

    #[test]
    fn trim_loop_report_blocks_broken_vertex_chain() {
        let mut shell = cube_shell();
        shell.faces[0].outer.coedges.swap(1, 2);
        let report = shell.trim_set_report(BrepFaceId(0));

        assert!(!report.outer_ready);
        assert_eq!(report.blocked_loop_count, 1);
        assert!(!report.trim_set_ready);
        assert!(report.loops[0].vertex_chain_break_count > 0);
        assert!(
            report.loops[0]
                .blockers
                .contains(&BrepTrimLoopBlocker::VertexChainBreak)
        );
    }

    #[test]
    fn trim_loop_report_blocks_missing_edges_degenerate_edges_and_unready_surfaces() {
        let mut shell = cube_shell();
        shell.edges[0] = edge(0, 0, 0);
        shell.faces[0].surface = BrepSurfaceId(999);
        shell.faces[0]
            .outer
            .coedges
            .push(coedge(999, BrepEdgeOrientation::Forward));
        let report = shell.trim_set_report(BrepFaceId(0));

        assert!(!report.trim_set_ready);
        assert_eq!(report.loops[0].missing_edge_count, 1);
        assert_eq!(report.loops[0].degenerate_edge_count, 1);
        assert!(!report.loops[0].surface_replay_ready);
        assert!(
            report.loops[0]
                .blockers
                .contains(&BrepTrimLoopBlocker::MissingSurface)
        );
        assert!(
            report.loops[0]
                .blockers
                .contains(&BrepTrimLoopBlocker::MissingEdge)
        );
        assert!(
            report.loops[0]
                .blockers
                .contains(&BrepTrimLoopBlocker::DegenerateEdge)
        );
    }
}
