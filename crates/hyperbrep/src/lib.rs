//! Exact-aware boundary-representation evidence carriers.
//!
//! `hyperbrep` owns retained BREP topology: vertices, edges, loops, faces,
//! surfaces, and shell validation reports. It deliberately does not perform
//! hidden healing, tolerance merging, or mesh-derived topology promotion.
//! Topology-changing decisions require exact or certified predicates, while
//! uncertain or adapter-derived evidence stays explicit.

mod adjacency;
mod area;
mod bounds;
mod construction;
mod curve;
mod frame;
mod handoff;
mod interrogation;
mod pcurve;
mod physics;
mod query;
mod report;
mod solid;
mod surface;
mod surface_intersection;
mod topology;
mod triangle;
mod trim;
mod validation;
mod volume;
mod voxel;

pub use adjacency::{
    BrepEdgeAgreementBlocker, BrepEdgeAgreementReport, BrepEdgeUseReport,
    BrepShellEdgeAgreementReport,
};
pub use area::{BrepAreaProjectionAxis, BrepFaceAreaBlocker, BrepFaceAreaReport};
pub use bounds::{
    BrepFaceAabbPreflightBlocker, BrepFaceAabbPreflightReport, BrepFaceBoundsBlocker,
    BrepFaceBoundsReport, BrepShellBoundsBlocker, BrepShellBoundsReport,
};
pub use construction::{
    BrepPlanarExtrusionConstruction, BrepPlanarExtrusionConstructionBlocker,
    BrepPlanarRegionConstruction, BrepPlanarRegionConstructionBlocker,
};
pub use curve::{
    BrepCurve3, BrepCurveError3, BrepCurveErrorKind3, BrepCurveFamily3, BrepCurveGeometry3,
    BrepCurveOperation3, BrepCurveParameterDomain3, BrepCurveResult3, BrepLineSegment3,
    BrepNurbsCurve3, BrepRationalBezier3,
};
pub use frame::{
    BrepFaceUvBoundsBlocker, BrepFaceUvBoundsReport, BrepPlaneFrameAxis, BrepSurfaceFrameBlocker,
    BrepSurfaceFrameEvalReport, BrepSurfaceFrameProjectionReport, BrepSurfaceFrameReport,
};
pub use handoff::{
    BrepExactSolidHandoffBlocker, BrepExactSolidHandoffReport, BrepExactSurfaceHandoffBlocker,
    BrepExactSurfaceHandoffReport,
};
pub use interrogation::{
    BrepSurfaceDifferentialReport, BrepSurfaceFirstFundamentalForm,
    BrepSurfaceInterrogationBlocker, BrepSurfaceSecondFundamentalForm,
};
pub use pcurve::{
    BrepPcurve, BrepPcurveImageEqualityReport, BrepPcurveImageRelation, BrepPlanarError,
    BrepPlanarFaceEdgeUseRelation, BrepPlanarFaceEdgeUseReport, BrepPlanarFacePointLocation,
    BrepPlanarFacePointReport, BrepPlanarFaceRegion, BrepPlanarResult, BrepPlanarTrimLoop,
    BrepPlanarTrimLoopRole,
};
pub use physics::{
    BrepPhysicsFixtureHandoffReport, BrepPhysicsMassBlocker, BrepPhysicsMassHandoffReport,
    BrepPhysicsShapeBlocker, BrepPhysicsShapeHandoffReport,
};
pub use query::{
    BrepFacePlanePreflightBlocker, BrepFacePlanePreflightReport, BrepPointFacePlaneBlocker,
    BrepPointFacePlaneReport, BrepPreparedFaceQueryBatchReport, BrepPreparedFaceQueryBlocker,
    BrepSegmentFacePlaneBlocker, BrepSegmentFacePlaneReport, PreparedBrepFaceQuery,
};
pub use report::{
    BrepShellBlocker, BrepShellClosureReport, BrepSurfaceInventoryReport, BrepTopologyCounts,
    BrepTopologyValidationBlocker, BrepTopologyValidationReport,
};
pub use solid::{BrepSolidReadinessBlocker, BrepSolidReadinessReport};
pub use surface::{
    BrepSurface, BrepSurfaceBlocker, BrepSurfaceFacts, BrepSurfaceId, BrepSurfaceKind,
    BrepSurfacePointReport, PreparedBrepSurface,
};
pub use surface_intersection::{
    BrepCurveSurfaceBlocker, BrepCurveSurfaceIntersectionRelation,
    BrepCurveSurfaceIntersectionReport, BrepSurfaceIntersectionBlocker,
    BrepSurfaceIntersectionRelation, BrepSurfaceIntersectionReport, BrepSurfaceIntersectionStage,
    BrepSurfaceStationaryDistanceReport,
};
pub use topology::{
    BrepCoedge, BrepEdge, BrepEdgeId, BrepEdgeOrientation, BrepFace, BrepFaceId, BrepLoop,
    BrepLoopId, BrepShell, BrepVertex, BrepVertexId,
};
pub use triangle::{
    BrepExactTriangle3, BrepExactTriangleMeshHandoffReport, BrepTriangleMeshBlocker,
};
pub use trim::{BrepFaceTrimSetReport, BrepTrimLoopBlocker, BrepTrimLoopReport, BrepTrimLoopRole};
pub use validation::{
    BrepFaceValidationBlocker, BrepFaceValidationReport, BrepGeometryValidationBlocker,
    BrepGeometryValidationReport, BrepShellValidationBlocker, BrepShellValidationReport,
};
pub use volume::{BrepShellOrientation, BrepShellVolumeBlocker, BrepShellVolumeReport};
pub use voxel::{BrepVoxelError, BrepVoxelGeometry};

#[cfg(test)]
mod tests {
    use super::*;
    use hypercurve::{Contour2, CurvePolicy, CurveRegion2, LineSeg2, Segment2};
    use hyperlimit::{Plane3, Point3};
    use hyperreal::{Rational, Real};

    fn r(value: i32) -> Real {
        Real::from(value)
    }

    fn q(numerator: i64, denominator: u64) -> Real {
        Real::new(Rational::fraction(numerator, denominator).unwrap())
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

    fn rectangle_contour(min_x: i32, min_y: i32, max_x: i32, max_y: i32) -> Contour2 {
        Contour2::try_new(vec![
            line2(uv(min_x, min_y), uv(max_x, min_y)),
            line2(uv(max_x, min_y), uv(max_x, max_y)),
            line2(uv(max_x, max_y), uv(min_x, max_y)),
            line2(uv(min_x, max_y), uv(min_x, min_y)),
        ])
        .unwrap()
    }

    fn curve_region(material: Vec<Contour2>, holes: Vec<Contour2>) -> CurveRegion2 {
        CurveRegion2::try_from_native_contours(material, holes, &CurvePolicy::certified()).unwrap()
    }

    fn rectangle_region(min_x: i32, min_y: i32, max_x: i32, max_y: i32) -> CurveRegion2 {
        curve_region(
            vec![rectangle_contour(min_x, min_y, max_x, max_y)],
            Vec::new(),
        )
    }

    fn plane(id: u64, nx: i32, ny: i32, nz: i32, offset: i32) -> BrepSurface {
        BrepSurface::plane(BrepSurfaceId(id), Plane3::new(p(nx, ny, nz), r(offset)))
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
        let report = shell.closure_report();

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
    fn edge_agreement_report_certifies_cube_adjacent_edge_images() {
        let shell = cube_shell();
        let report = shell.edge_agreement_report();

        assert!(report.shell_edge_agreement_ready);
        assert_eq!(report.edge_count, 12);
        assert_eq!(report.ready_edge_count, 12);
        assert_eq!(report.blocked_edge_count, 0);
        assert_eq!(report.boundary_edge_count, 0);
        assert_eq!(report.nonmanifold_edge_count, 0);
        assert_eq!(report.same_orientation_pair_count, 0);
        assert_eq!(report.exact_edge_image_count, 12);
        assert!(report.blockers.is_empty());
        assert!(report.edges.iter().all(|edge| edge.use_count == 2));
        assert!(report.edges.iter().all(|edge| edge.manifold_pair_ready));
        assert!(report.edges.iter().all(|edge| edge.exact_edge_image_ready));
    }

    #[test]
    fn edge_agreement_report_blocks_boundary_orientation_and_off_surface_cases() {
        let mut open = cube_shell();
        open.faces.pop();
        let open_report = open.edge_agreement_report();
        assert!(!open_report.shell_edge_agreement_ready);
        assert_eq!(open_report.boundary_edge_count, 4);
        assert!(
            open_report
                .blockers
                .contains(&BrepEdgeAgreementBlocker::BoundaryEdge)
        );

        let mut same_orientation = cube_shell();
        same_orientation.faces[1].outer.coedges[0].orientation = BrepEdgeOrientation::Reversed;
        let same_orientation_report = same_orientation.edge_agreement_report();
        assert!(!same_orientation_report.shell_edge_agreement_ready);
        assert_eq!(same_orientation_report.same_orientation_pair_count, 1);
        assert!(
            same_orientation_report
                .blockers
                .contains(&BrepEdgeAgreementBlocker::SameOrientationPair)
        );

        let mut off_surface = cube_shell();
        off_surface.vertices[0] = BrepVertex::new(BrepVertexId(0), p(0, 0, 2));
        let off_surface_report = off_surface.edge_agreement_report();
        assert!(!off_surface_report.shell_edge_agreement_ready);
        assert!(
            off_surface_report
                .blockers
                .contains(&BrepEdgeAgreementBlocker::EndpointOffSurface)
        );
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
        assert!(report.edge_agreement.shell_edge_agreement_ready);
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

        let report = shell.closure_report();
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
        assert!(!validation.edge_agreement.shell_edge_agreement_ready);
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
                .contains(&BrepShellValidationBlocker::EdgeAgreementNotReady)
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
    fn surface_inventory_keeps_unsupported_families_explicit() {
        let surfaces = vec![
            plane(0, 0, 0, 1, 0),
            BrepSurface::unsupported(BrepSurfaceId(1), "nurbs-surface"),
        ];
        let report = BrepSurfaceInventoryReport::from_surfaces(&surfaces);

        assert_eq!(report.surface_count, 2);
        assert_eq!(report.planar_count, 1);
        assert_eq!(report.unsupported_count, 1);
        assert!(!report.all_exact_planar);
    }

    #[test]
    fn planar_region_construction_rejects_arcs_and_disconnected_material() {
        let first = rectangle_contour(0, 0, 1, 1);
        let second = rectangle_contour(3, 0, 4, 1);
        let disconnected = curve_region(vec![first, second], Vec::new());
        let disconnected_report = BrepPlanarRegionConstruction::from_region_on_surface(
            &disconnected,
            plane(0, 0, 0, 1, 0),
        );
        assert!(!disconnected_report.exact_construction_ready);
        assert!(
            disconnected_report
                .blockers
                .contains(&BrepPlanarRegionConstructionBlocker::MultipleMaterialContours)
        );

        let arc_contour = Contour2::from_bulge_vertices(&[
            hypercurve::BulgeVertex2::new(uv(0, 0), r(1)),
            hypercurve::BulgeVertex2::new(uv(1, 0), r(0)),
            hypercurve::BulgeVertex2::new(uv(0, 1), r(0)),
        ])
        .unwrap();
        let arc_region = curve_region(vec![arc_contour], Vec::new());
        let arc_report =
            BrepPlanarRegionConstruction::from_region_on_surface(&arc_region, plane(0, 0, 0, 1, 0));
        assert!(!arc_report.exact_construction_ready);
        assert!(
            arc_report
                .blockers
                .contains(&BrepPlanarRegionConstructionBlocker::UnsupportedCurveSegment)
        );
    }

    #[test]
    fn planar_extrusion_construction_builds_exact_closed_prism() {
        let region = rectangle_region(0, 0, 2, 3);
        let constructed =
            BrepPlanarExtrusionConstruction::vertical_prism_from_region(&region, r(0), r(4));

        assert!(constructed.exact_construction_ready);
        assert!(constructed.blockers.is_empty());
        assert_eq!(constructed.source_vertex_count, 4);
        assert_eq!(constructed.vertex_count, 8);
        assert_eq!(constructed.edge_count, 12);
        assert_eq!(constructed.face_count, 6);
        let shell = constructed.shell.as_ref().unwrap();
        let solid = shell.solid_readiness_report();
        assert!(solid.exact_solid_boundary_ready);
        assert!(solid.exact_volume_ready);
        assert_eq!(solid.volume.signed_six_volume, Some(r(144)));
        let physics = shell.physics_mass_handoff_report(r(1));
        assert!(physics.exact_physics_mass_ready);
        assert_eq!(physics.triangle_count, 12);
    }

    #[test]
    fn planar_extrusion_construction_rejects_zero_height_and_arcs() {
        let zero_height = BrepPlanarExtrusionConstruction::vertical_prism_from_region(
            &rectangle_region(0, 0, 1, 1),
            r(0),
            r(0),
        );
        assert!(!zero_height.exact_construction_ready);
        assert!(
            zero_height
                .blockers
                .contains(&BrepPlanarExtrusionConstructionBlocker::NonPositiveHeight)
        );

        let arc_contour = Contour2::from_bulge_vertices(&[
            hypercurve::BulgeVertex2::new(uv(0, 0), r(1)),
            hypercurve::BulgeVertex2::new(uv(1, 0), r(0)),
            hypercurve::BulgeVertex2::new(uv(0, 1), r(0)),
        ])
        .unwrap();
        let arc_region = curve_region(vec![arc_contour], Vec::new());
        let arc_report =
            BrepPlanarExtrusionConstruction::vertical_prism_from_region(&arc_region, r(0), r(1));
        assert!(!arc_report.exact_construction_ready);
        assert!(
            arc_report
                .blockers
                .contains(&BrepPlanarExtrusionConstructionBlocker::UnsupportedCurveSegment)
        );
    }

    #[test]
    fn planar_extrusion_construction_preserves_hole_volume() {
        let outer = rectangle_contour(0, 0, 4, 4);
        let hole = rectangle_contour(1, 1, 2, 2);
        let holed = curve_region(vec![outer], vec![hole]);
        let constructed =
            BrepPlanarExtrusionConstruction::vertical_prism_from_region(&holed, r(0), r(2));

        assert!(constructed.exact_construction_ready);
        assert!(constructed.blockers.is_empty());
        assert_eq!(constructed.source_vertex_count, 8);
        assert_eq!(constructed.vertex_count, 16);
        assert_eq!(constructed.edge_count, 24);
        assert_eq!(constructed.face_count, 10);
        let shell = constructed.shell.as_ref().unwrap();
        let solid = shell.solid_readiness_report();
        assert!(solid.exact_solid_boundary_ready);
        assert_eq!(solid.volume.signed_six_volume, Some(r(180)));
        let physics = shell.physics_mass_handoff_report(r(1));
        assert!(physics.exact_physics_mass_ready);
        assert_eq!(physics.triangle_count, 32);
    }

    #[test]
    fn solid_readiness_packages_closed_shell_bounds_and_faces() {
        let shell = cube_shell();
        let report = shell.solid_readiness_report();

        assert!(report.closed_shell_ready);
        assert!(report.all_faces_ready);
        assert!(report.exact_bounds_ready);
        assert!(report.edge_agreement_ready);
        assert!(report.exact_volume_ready);
        assert!(report.exact_solid_boundary_ready);
        assert_eq!(report.ready_face_count, 6);
        assert_eq!(report.blocked_face_count, 0);
        assert_eq!(report.shell_bounds.min, Some(p(0, 0, 0)));
        assert_eq!(report.shell_bounds.max, Some(p(1, 1, 1)));
        assert_eq!(report.faces.len(), 6);
        assert_eq!(report.edge_agreement.ready_edge_count, 12);
        assert_eq!(report.volume.signed_six_volume, Some(r(6)));
        assert_eq!(
            report.volume.orientation,
            Some(BrepShellOrientation::Positive)
        );
        assert!(report.blockers.is_empty());
    }

    #[test]
    fn solid_readiness_blocks_open_or_invalid_shells() {
        let shell = cube_shell();
        let mut broken = shell.clone();
        broken.faces.pop();
        broken.edges[0] = edge(0, 0, 0);
        let report = broken.solid_readiness_report();

        assert!(!report.exact_solid_boundary_ready);
        assert!(!report.closed_shell_ready);
        assert!(!report.all_faces_ready);
        assert!(!report.edge_agreement_ready);
        assert!(!report.exact_volume_ready);
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
                .contains(&BrepSolidReadinessBlocker::EdgeAgreementNotReady)
        );
        assert!(
            report
                .blockers
                .contains(&BrepSolidReadinessBlocker::VolumeReplayUnavailable)
        );
    }

    #[test]
    fn exact_retained_handoffs_package_surface_and_solid_evidence() {
        let shell = cube_shell();

        let surface = shell.exact_surface_handoff();
        assert!(surface.exact_surface_handoff_ready);
        assert!(surface.closed_shell);
        assert!(surface.nonempty_topology);
        assert!(surface.retained_brep_only);
        assert_eq!(surface.face_count, 6);
        assert_eq!(surface.vertex_count, 8);
        assert_eq!(surface.bounds.min, Some(p(0, 0, 0)));
        assert_eq!(surface.bounds.max, Some(p(1, 1, 1)));
        assert!(surface.blockers.is_empty());

        let solid = shell.exact_solid_handoff();
        assert!(solid.exact_solid_handoff_ready);
        assert!(solid.retained_brep_only);
        assert!(solid.surface.exact_surface_handoff_ready);
        assert!(solid.solid.exact_solid_boundary_ready);
        assert!(solid.volume.exact_volume_ready);
        assert_eq!(solid.volume.signed_six_volume, Some(r(6)));
        assert!(solid.blockers.is_empty());
    }

    #[test]
    fn exact_retained_handoffs_block_open_or_invalid_evidence() {
        let shell = cube_shell();
        let mut broken = shell.clone();
        broken.faces.pop();
        broken.edges[0] = edge(0, 0, 0);

        let surface = broken.exact_surface_handoff();
        assert!(!surface.exact_surface_handoff_ready);
        assert!(
            surface
                .blockers
                .contains(&BrepExactSurfaceHandoffBlocker::ShellValidationNotReady)
        );

        let solid = broken.exact_solid_handoff();
        assert!(!solid.exact_solid_handoff_ready);
        assert!(
            solid
                .blockers
                .contains(&BrepExactSolidHandoffBlocker::SurfaceHandoffNotReady)
        );
        assert!(
            solid
                .blockers
                .contains(&BrepExactSolidHandoffBlocker::SolidReadinessNotReady)
        );
        assert!(
            solid
                .blockers
                .contains(&BrepExactSolidHandoffBlocker::VolumeNotReady)
        );
    }

    #[test]
    fn physics_mass_handoff_replays_cube_into_exact_mass_properties() {
        let shell = cube_shell();
        let triangles = shell.exact_triangle_mesh_handoff_report();
        assert!(triangles.exact_triangle_mesh_ready);
        assert!(triangles.retained_brep_source);
        assert!(!triangles.lossy_or_preview_mesh);
        assert_eq!(triangles.face_count, 6);
        assert_eq!(triangles.triangle_count, 12);
        assert_eq!(triangles.triangles[0].vertices[0], p(0, 0, 0));
        assert_eq!(triangles.triangles[0].vertices[1], p(1, 0, 0));
        assert_eq!(triangles.triangles[0].vertices[2], p(1, 1, 0));

        let shape = shell.physics_shape_handoff_report();
        assert!(shape.exact_physics_shape_ready);
        assert!(shape.triangle_mesh.exact_triangle_mesh_ready);
        assert_eq!(shape.triangle_count, 12);
        assert!(shape.mesh.is_some());
        assert!(matches!(
            shape.shape,
            Some(hyperphysics::PhysicsShape3::ClosedTriangleMesh(_))
        ));

        let material = hyperphysics::ExactMaterial::new(
            hyperphysics::MaterialId::new("mat:unit").unwrap(),
            "unit",
            r(1),
        )
        .unwrap();
        let fixture = shell.physics_fixture_handoff_report("fixture:cube", material);
        assert!(fixture.exact_physics_fixture_ready);
        assert!(fixture.fixture.is_some());

        let report = shell.physics_mass_handoff_report(r(2));

        assert!(report.exact_physics_mass_ready);
        assert!(report.solid.exact_solid_boundary_ready);
        assert_eq!(report.face_count, 6);
        assert_eq!(report.triangle_count, 12);
        assert!(report.blockers.is_empty());

        let mass = report.mass_properties.unwrap();
        assert_eq!(mass.volume, r(1));
        assert_eq!(mass.mass, r(2));
        assert_eq!(mass.center_of_mass[0], q(1, 2));
        assert_eq!(mass.center_of_mass[1], q(1, 2));
        assert_eq!(mass.center_of_mass[2], q(1, 2));
        assert_eq!(mass.inertia_about_center_of_mass.xx, q(1, 3));
        assert_eq!(mass.inertia_about_center_of_mass.yy, q(1, 3));
        assert_eq!(mass.inertia_about_center_of_mass.zz, q(1, 3));
    }

    #[test]
    fn physics_mass_handoff_blocks_open_broken_loop_and_invalid_density() {
        let mut open = cube_shell();
        open.faces.pop();
        let open_triangles = open.exact_triangle_mesh_handoff_report();
        assert!(!open_triangles.exact_triangle_mesh_ready);
        assert!(
            open_triangles
                .blockers
                .contains(&BrepTriangleMeshBlocker::SolidReadinessNotReady)
        );
        let open_shape = open.physics_shape_handoff_report();
        assert!(!open_shape.exact_physics_shape_ready);
        assert!(
            open_shape
                .blockers
                .contains(&BrepPhysicsShapeBlocker::SolidReadinessNotReady)
        );
        let open_report = open.physics_mass_handoff_report(r(1));
        assert!(!open_report.exact_physics_mass_ready);
        assert!(
            open_report
                .blockers
                .contains(&BrepPhysicsMassBlocker::SolidReadinessNotReady)
        );

        let mut broken_loop = cube_shell();
        broken_loop.faces[0].inner.push(BrepLoop::new(
            BrepLoopId(99),
            vec![
                coedge(0, BrepEdgeOrientation::Forward),
                coedge(1, BrepEdgeOrientation::Forward),
                coedge(2, BrepEdgeOrientation::Forward),
            ],
        ));
        let broken_report = broken_loop.physics_mass_handoff_report(r(1));
        assert!(!broken_report.exact_physics_mass_ready);
        assert!(
            broken_report
                .blockers
                .contains(&BrepPhysicsMassBlocker::SolidReadinessNotReady)
        );
        let broken_triangles = broken_loop.exact_triangle_mesh_handoff_report();
        assert!(!broken_triangles.exact_triangle_mesh_ready);
        assert!(
            broken_triangles
                .blockers
                .contains(&BrepTriangleMeshBlocker::SolidReadinessNotReady)
        );
        let material = hyperphysics::ExactMaterial::new(
            hyperphysics::MaterialId::new("mat:unit").unwrap(),
            "unit",
            r(1),
        )
        .unwrap();
        let bad_fixture = cube_shell().physics_fixture_handoff_report("", material);
        assert!(!bad_fixture.exact_physics_fixture_ready);
        assert!(
            bad_fixture
                .blockers
                .contains(&BrepPhysicsShapeBlocker::PhysicsFixtureIdRejected)
        );

        let invalid_density = cube_shell().physics_mass_handoff_report(r(0));
        assert!(!invalid_density.exact_physics_mass_ready);
        assert!(
            invalid_density
                .blockers
                .contains(&BrepPhysicsMassBlocker::PhysicsMassRejected)
        );
        assert_eq!(
            invalid_density.physics_error,
            Some(hyperphysics::PhysicsError::NonPositiveDensity)
        );
    }

    #[test]
    fn voxel_geometry_prepares_exact_aabb_and_triangle_solid() {
        let shell = cube_shell();
        let frame = hypervoxel::GridFrame::new(
            [r(0), r(0), r(0)],
            [r(1), r(1), r(1)],
            2,
            hypervoxel::LengthUnit::Unitless,
        )
        .unwrap();
        let geometry = shell.prepare_voxel_geometry().unwrap();
        assert_eq!(geometry.exact_aabb.min, [r(0), r(0), r(0)]);
        assert_eq!(geometry.exact_aabb.max, [r(1), r(1), r(1)]);
        let (_, voxel_report, schedule) = hypervoxel::voxelize_prepared_exact_triangle_solid_mesh(
            frame,
            &geometry.triangle_solid,
            hypervoxel::MaterialRegionId(7),
            hypervoxel::VoxelizationPolicy::conservative_cover(),
        )
        .unwrap();
        assert!(voxel_report.predicate_certificates.is_fully_certified());
        assert!(schedule.boundary_aabb_rejections > 0);
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
    fn prepared_surface_blocks_unsupported_or_zero_normal_surfaces() {
        let unsupported_surface = BrepSurface::unsupported(BrepSurfaceId(12), "nurbs");
        let unsupported = unsupported_surface.prepare();
        assert!(
            unsupported
                .classify_point(&p(0, 0, 0))
                .blockers
                .contains(&BrepSurfaceBlocker::UnsupportedFamily)
        );

        let zero_normal_surface =
            BrepSurface::plane(BrepSurfaceId(13), Plane3::new(p(0, 0, 0), r(0)));
        let zero_normal = zero_normal_surface.prepare();
        assert!(
            zero_normal
                .classify_point(&p(0, 0, 0))
                .blockers
                .contains(&BrepSurfaceBlocker::ZeroNormal)
        );
    }

    #[test]
    fn planar_surface_frame_evaluates_and_projects_exact_axis_uv() {
        let surface = plane(20, 0, 0, 1, -2);
        let frame = surface.frame_report();
        assert!(frame.exact_frame_ready);
        assert_eq!(frame.axis, Some(BrepPlaneFrameAxis::Z));
        assert!(frame.blockers.is_empty());

        let uv = hyperlimit::Point2::new(r(3), r(4));
        let eval = surface.evaluate_frame_uv(uv.clone());
        assert!(eval.exact_evaluation_ready);
        assert_eq!(eval.point, Some(p(3, 4, 2)));

        let projection = surface.project_frame_point(p(3, 4, 2));
        assert!(projection.exact_projection_ready);
        assert_eq!(projection.uv, Some(uv));

        let x_surface = plane(21, 2, 0, 0, -6);
        let x_eval = x_surface.evaluate_frame_uv(hyperlimit::Point2::new(r(7), r(8)));
        assert_eq!(x_eval.frame.axis, Some(BrepPlaneFrameAxis::X));
        assert_eq!(x_eval.point, Some(p(3, 7, 8)));
        let y_surface = plane(22, 0, -2, 0, 6);
        let y_eval = y_surface.evaluate_frame_uv(hyperlimit::Point2::new(r(9), r(10)));
        assert_eq!(y_eval.frame.axis, Some(BrepPlaneFrameAxis::Y));
        assert_eq!(y_eval.point, Some(p(10, 3, 9)));
    }

    #[test]
    fn planar_surface_frame_supports_general_planes_and_blocks_unsupported_families() {
        let diagonal = BrepSurface::plane(BrepSurfaceId(23), Plane3::new(p(1, 1, 0), r(0)));
        let report = diagonal.frame_report();
        assert!(report.exact_frame_ready);
        assert_eq!(report.axis, Some(BrepPlaneFrameAxis::X));
        assert!(report.blockers.is_empty());
        let eval = diagonal.evaluate_frame_uv(hyperlimit::Point2::new(r(0), r(0)));
        assert!(eval.exact_evaluation_ready);
        assert_eq!(eval.point, Some(p(0, 0, 0)));

        let unsupported = BrepSurface::unsupported(BrepSurfaceId(24), "nurbs");
        let unsupported_report = unsupported.frame_report();
        assert!(!unsupported_report.exact_frame_ready);
        assert!(
            unsupported_report
                .blockers
                .contains(&BrepSurfaceFrameBlocker::SurfaceNotReady)
        );
        assert!(
            unsupported_report
                .blockers
                .contains(&BrepSurfaceFrameBlocker::UnsupportedSurface)
        );
    }

    #[test]
    fn face_uv_bounds_report_projects_boundary_vertices_through_frame() {
        let shell = cube_shell();
        let bottom = shell.face_uv_bounds_report(BrepFaceId(0));

        assert!(bottom.exact_uv_bounds_ready);
        assert_eq!(
            bottom.frame.as_ref().unwrap().axis,
            Some(BrepPlaneFrameAxis::Z)
        );
        assert_eq!(bottom.vertex_count, 4);
        assert_eq!(bottom.min, Some(hyperlimit::Point2::new(r(0), r(0))));
        assert_eq!(bottom.max, Some(hyperlimit::Point2::new(r(1), r(1))));
        assert!(!bottom.zero_u_extent);
        assert!(!bottom.zero_v_extent);
        assert!(bottom.blockers.is_empty());

        let left = shell.face_uv_bounds_report(BrepFaceId(4));
        assert!(left.exact_uv_bounds_ready);
        assert_eq!(
            left.frame.as_ref().unwrap().axis,
            Some(BrepPlaneFrameAxis::X)
        );
        assert_eq!(left.min, Some(hyperlimit::Point2::new(r(0), r(0))));
        assert_eq!(left.max, Some(hyperlimit::Point2::new(r(1), r(1))));
    }

    #[test]
    fn face_uv_bounds_report_blocks_missing_topology_or_unready_frame() {
        let mut missing_edge = cube_shell();
        missing_edge.faces[0]
            .outer
            .coedges
            .push(coedge(999, BrepEdgeOrientation::Forward));
        let missing_report = missing_edge.face_uv_bounds_report(BrepFaceId(0));
        assert!(!missing_report.exact_uv_bounds_ready);
        assert!(
            missing_report
                .blockers
                .contains(&BrepFaceUvBoundsBlocker::MissingEdge)
        );

        let mut diagonal = cube_shell();
        diagonal.surfaces[0] = BrepSurface::plane(BrepSurfaceId(0), Plane3::new(p(1, 1, 0), r(0)));
        let diagonal_report = diagonal.face_uv_bounds_report(BrepFaceId(0));
        assert!(diagonal_report.exact_uv_bounds_ready);
        assert_eq!(
            diagonal_report.frame.as_ref().unwrap().axis,
            Some(BrepPlaneFrameAxis::X)
        );
        assert!(diagonal_report.blockers.is_empty());
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

        let (min, max) = report.exact_bounds().unwrap();
        assert_eq!(
            hyperlimit::classify_point_aabb3(min, max, &p(0, 0, 0)).value(),
            Some(hyperlimit::Aabb3PointLocation::Boundary)
        );
        assert_eq!(
            hyperlimit::classify_point_aabb3(min, max, &p(1, 1, 1)).value(),
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

        let (min, max) = report.exact_bounds().unwrap();
        assert_eq!(
            hyperlimit::classify_point_aabb3(min, max, &p(0, 0, 0)).value(),
            Some(hyperlimit::Aabb3PointLocation::Boundary)
        );
        assert_eq!(
            hyperlimit::classify_point_aabb3(min, max, &p(1, 1, 1)).value(),
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
        unsupported.surfaces[0] = BrepSurface::unsupported(BrepSurfaceId(0), "nurbs");
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
        unsupported.surfaces[0] = BrepSurface::unsupported(BrepSurfaceId(0), "nurbs");
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
    fn prepared_face_query_reuses_surface_and_bounds_evidence() {
        let shell = cube_shell();
        let prepared = shell.prepare_face_query(BrepFaceId(0));

        assert!(prepared.prepared_query_ready);
        assert!(prepared.bounds.exact_bounds_ready);
        assert!(prepared.surface.as_ref().unwrap().exact_replay_ready());
        assert!(prepared.blockers.is_empty());

        let direct_point = shell.point_face_plane_preflight(BrepFaceId(0), &p(0, 0, 1));
        let prepared_point = prepared.point_face_plane_preflight(&p(0, 0, 1));
        assert_eq!(prepared_point, direct_point);
        assert!(prepared_point.certified_off_support_plane);

        let direct_segment =
            shell.segment_face_plane_preflight(BrepFaceId(0), &p(0, 0, -1), &p(0, 0, 1));
        let prepared_segment = prepared.segment_face_plane_preflight(&p(0, 0, -1), &p(0, 0, 1));
        assert_eq!(prepared_segment, direct_segment);
        assert!(prepared_segment.requires_narrow_phase);

        let plane = Plane3::new(p(0, 0, 1), r(-2));
        let direct_plane = shell.face_plane_preflight(BrepFaceId(0), &plane);
        let prepared_plane = prepared.face_plane_preflight(&plane);
        assert_eq!(prepared_plane, direct_plane);

        let point_queries = vec![p(0, 0, 0), p(0, 0, 1)];
        let segment_start = p(0, 0, -1);
        let segment_end = p(0, 0, 1);
        let above_start = p(0, 0, -2);
        let above_end = p(1, 0, -2);
        let segments = vec![(&segment_start, &segment_end), (&above_start, &above_end)];
        let batch = prepared.batch_report(&point_queries, &segments);
        assert!(batch.prepared_query_ready);
        assert_eq!(batch.point_query_count, 2);
        assert_eq!(batch.segment_query_count, 2);
        assert_eq!(batch.certified_rejection_count, 2);
        assert_eq!(batch.narrow_phase_candidate_count, 2);
        assert!(batch.blockers.is_empty());
    }

    #[test]
    fn prepared_face_query_reports_missing_and_unready_contexts() {
        let shell = cube_shell();
        let missing = shell.prepare_face_query(BrepFaceId(999));
        assert!(!missing.prepared_query_ready);
        assert!(
            missing
                .blockers
                .contains(&BrepPreparedFaceQueryBlocker::MissingFace)
        );
        assert!(
            missing
                .blockers
                .contains(&BrepPreparedFaceQueryBlocker::FaceBoundsNotReady)
        );

        let mut unsupported = cube_shell();
        unsupported.surfaces[0] = BrepSurface::unsupported(BrepSurfaceId(0), "nurbs");
        let prepared = unsupported.prepare_face_query(BrepFaceId(0));
        assert!(!prepared.prepared_query_ready);
        assert!(
            prepared
                .blockers
                .contains(&BrepPreparedFaceQueryBlocker::SurfaceNotReady)
        );
        assert!(
            prepared
                .blockers
                .contains(&BrepPreparedFaceQueryBlocker::UnsupportedSurface)
        );
        let point = prepared.point_face_plane_preflight(&p(0, 0, 0));
        assert!(!point.preflight_ready);
        assert!(
            point
                .blockers
                .contains(&BrepPointFacePlaneBlocker::UnsupportedSurface)
        );
    }

    #[test]
    fn face_validation_packages_surface_trim_bounds_and_geometry() {
        let shell = cube_shell();
        let report = shell.face_validation_report(BrepFaceId(0));

        assert!(report.face_found);
        assert!(report.exact_face_boundary_ready);
        assert!(report.exact_bounds_ready);
        assert!(report.exact_uv_bounds_ready);
        assert!(report.exact_face_ready);
        assert!(report.surface_facts.as_ref().unwrap().exact_replay_ready);
        assert!(report.trim_set.as_ref().unwrap().trim_set_ready);
        assert!(report.bounds.as_ref().unwrap().exact_bounds_ready);
        assert!(report.uv_bounds.as_ref().unwrap().exact_uv_bounds_ready);
        assert!(report.geometry.as_ref().unwrap().geometry_ready);
    }

    #[test]
    fn face_validation_blocks_unready_surface_trim_bounds_and_geometry() {
        let mut shell = cube_shell();
        shell.surfaces[0] = BrepSurface::plane(BrepSurfaceId(0), Plane3::new(p(0, 0, 0), r(0)));
        shell.faces[0].outer.coedges.swap(1, 2);
        shell.edges[0] = edge(0, 0, 999);
        let report = shell.face_validation_report(BrepFaceId(0));

        assert!(!report.exact_face_ready);
        assert!(!report.exact_bounds_ready);
        assert!(!report.exact_uv_bounds_ready);
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
                .contains(&BrepFaceValidationBlocker::UvBoundsNotReady)
        );
        assert!(
            report
                .blockers
                .contains(&BrepFaceValidationBlocker::GeometryNotReady)
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
        unsupported.surfaces[0] = BrepSurface::unsupported(BrepSurfaceId(0), "nurbs");
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
