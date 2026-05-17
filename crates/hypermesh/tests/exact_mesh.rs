#![cfg(feature = "exact")]

use hyperlimit::{PlaneSide, Point3, compare_reals};
use hypermesh::exact::{
    AabbIntersectionKind, CoplanarTriangleRelation, DiagnosticKind, ExactAabb3,
    ExactFaceSplitGeometryPlan, ExactFaceSplitPlan, ExactGraphVertex, ExactMesh, ExactReal,
    ExactSplitTopologyPlan, FaceRegionBoundary, FaceSplitBoundaryChain, FaceSplitBoundaryNode,
    FaceSplitEdge, FaceSplitGeometry, FaceSplitPlan, IntersectionEvent, MeshFacePairRelation,
    MeshSide, SegmentPlaneRelation, Severity, SplitEdgeChain, SplitEdgeNode,
    SplitPlanDiagnosticKind, TrianglePlaneRelation, TriangleTriangleRelation, ValidationPolicy,
    VertexLinkKind, build_intersection_graph, classify_coplanar_triangles,
    classify_face_regions_against_opposite_planes, classify_mesh_face_pair,
    classify_mesh_face_pairs, classify_triangle_against_face_plane, classify_triangle_triangle,
    intersect_segment_with_face_plane, validate_triangles, validate_triangles_with_policy,
};
use hyperreal::Real;
use proptest::prelude::*;
use std::cmp::Ordering;

fn tetrahedron() -> (Vec<f64>, Vec<usize>) {
    (
        vec![
            0.0, 0.0, 0.0, //
            1.0, 0.0, 0.0, //
            0.0, 1.0, 0.0, //
            0.0, 0.0, 1.0,
        ],
        vec![0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
}

#[test]
fn exact_mesh_accepts_closed_tetrahedron_with_certified_facts() {
    let (pos, idx) = tetrahedron();
    let mesh = ExactMesh::from_f64_triangles(&pos, &idx).unwrap();

    assert_eq!(mesh.facts().mesh.vertex_count, 4);
    assert_eq!(mesh.facts().mesh.face_count, 4);
    assert_eq!(mesh.facts().mesh.edge_count, 6);
    assert_eq!(mesh.facts().mesh.euler_characteristic, 2);
    assert!(mesh.facts().mesh.closed_manifold);
    assert!(mesh.facts().mesh.fixed_coordinates_exact_rational);
    assert!(
        mesh.facts()
            .vertices
            .iter()
            .all(|vertex| vertex.link == VertexLinkKind::Circle)
    );
    assert!(
        mesh.provenance()
            .predicates
            .iter()
            .all(|predicate| predicate.is_proof_producing())
    );
}

#[test]
fn exact_mesh_lifts_integer_grid_without_lossy_source() {
    let pos = vec![
        0, 0, 0, //
        1, 0, 0, //
        0, 1, 0, //
        0, 0, 1,
    ];
    let idx = vec![0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3];
    let mesh = ExactMesh::from_i64_triangles(&pos, &idx).unwrap();

    assert!(mesh.facts().mesh.closed_manifold);
    assert!(mesh.facts().mesh.fixed_coordinates_exact_rational);
    assert_eq!(mesh.provenance().source.label, "flat i64 triangle mesh");
}

#[test]
fn exact_bounds_reject_disjoint_face_pairs_without_narrow_phase() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[10, 0, 0, 11, 0, 0, 10, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    assert_eq!(
        left.bounds()
            .mesh
            .as_ref()
            .unwrap()
            .classify_intersection(right.bounds().mesh.as_ref().unwrap())
            .value(),
        Some(AabbIntersectionKind::Disjoint)
    );
    assert!(
        left.bounds()
            .candidate_face_pairs(right.bounds())
            .is_empty()
    );
}

#[test]
fn exact_bounds_keep_touching_faces_as_candidates() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, 0, 0, 2, 0, 0, 1, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    assert_eq!(
        left.bounds().faces[0]
            .classify_intersection(&right.bounds().faces[0])
            .value(),
        Some(AabbIntersectionKind::Touching)
    );
    assert_eq!(
        left.bounds().candidate_face_pairs(right.bounds()),
        vec![[0, 0]]
    );
}

#[test]
fn exact_bounds_can_retain_symbolic_unknown_relation() {
    let zero = ExactReal::from(0);
    let one = ExactReal::from(1);
    let pi = ExactReal::pi();
    let left = ExactAabb3 {
        min: Point3::new(zero.clone(), zero.clone(), zero.clone()),
        max: Point3::new(pi.clone(), one.clone(), one.clone()),
    };
    let right = ExactAabb3 {
        min: Point3::new(one.clone(), zero.clone(), zero.clone()),
        max: Point3::new(one.clone() + pi, one.clone(), one),
    };

    let relation = left.classify_intersection(&right);
    assert!(
        relation
            .value()
            .is_none_or(AabbIntersectionKind::needs_narrow_phase)
    );
}

#[test]
fn exact_narrow_phase_classifies_triangle_plane_side() {
    let points = [
        p3(0, 0, 0),
        p3(1, 0, 0),
        p3(0, 1, 0),
        p3(0, 0, 1),
        p3(1, 0, 1),
        p3(0, 1, 1),
    ];
    let classification = classify_triangle_against_face_plane(&points, [0, 1, 2], [3, 4, 5]);

    assert_eq!(
        classification.relation,
        TrianglePlaneRelation::StrictlyBelow
    );
    assert!(classification.all_proof_producing());
}

#[test]
fn exact_narrow_phase_classifies_coplanar_and_straddling_triangles() {
    let points = [
        p3(0, 0, 0),
        p3(1, 0, 0),
        p3(0, 1, 0),
        p3(2, 0, 0),
        p3(0, 2, 0),
        p3(1, 1, 0),
        p3(0, 0, 1),
        p3(0, 0, -1),
    ];

    let coplanar = classify_triangle_against_face_plane(&points, [0, 1, 2], [3, 4, 5]);
    let straddling = classify_triangle_against_face_plane(&points, [0, 1, 2], [0, 6, 7]);

    assert_eq!(coplanar.relation, TrianglePlaneRelation::Coplanar);
    assert_eq!(straddling.relation, TrianglePlaneRelation::Straddling);
    assert!(coplanar.all_proof_producing());
    assert!(straddling.all_proof_producing());
}

#[test]
fn exact_segment_plane_constructs_proper_crossing_as_ratio() {
    let points = [
        p3(0, 0, 0),
        p3(1, 0, 0),
        p3(0, 1, 0),
        p3(0, 0, -1),
        p3(0, 0, 1),
    ];

    let event = intersect_segment_with_face_plane(&points, [0, 1, 2], [3, 4]);

    assert_eq!(event.relation, SegmentPlaneRelation::ProperCrossing);
    assert_eq!(
        event.endpoint_sides,
        [Some(PlaneSide::Above), Some(PlaneSide::Below)]
    );
    assert!(event.all_proof_producing());
    assert_real_eq(event.parameter.as_ref().unwrap(), &half());
    let point = event.point.as_ref().unwrap();
    assert_real_eq(&point.x, &ExactReal::from(0));
    assert_real_eq(&point.y, &ExactReal::from(0));
    assert_real_eq(&point.z, &ExactReal::from(0));
}

#[test]
fn exact_segment_plane_classifies_endpoint_coplanar_and_disjoint_cases() {
    let points = [
        p3(0, 0, 0),
        p3(1, 0, 0),
        p3(0, 1, 0),
        p3(2, 0, 0),
        p3(2, 0, 1),
        p3(0, 0, 2),
        p3(1, 0, 2),
        p3(0, 0, 0),
        p3(1, 1, 0),
    ];

    let endpoint = intersect_segment_with_face_plane(&points, [0, 1, 2], [3, 4]);
    assert_eq!(endpoint.relation, SegmentPlaneRelation::EndpointOnPlane);
    assert_eq!(endpoint.endpoint_on_plane, Some(0));
    assert_real_eq(endpoint.parameter.as_ref().unwrap(), &ExactReal::from(0));

    let disjoint = intersect_segment_with_face_plane(&points, [0, 1, 2], [5, 6]);
    assert_eq!(disjoint.relation, SegmentPlaneRelation::Disjoint);
    assert!(disjoint.point.is_none());

    let coplanar = intersect_segment_with_face_plane(&points, [0, 1, 2], [7, 8]);
    assert_eq!(coplanar.relation, SegmentPlaneRelation::Coplanar);
    assert!(coplanar.parameter.is_none());
}

#[test]
fn exact_triangle_triangle_rejects_plane_separated_pair() {
    let points = [
        p3(0, 0, 0),
        p3(1, 0, 0),
        p3(0, 1, 0),
        p3(0, 0, 2),
        p3(1, 0, 2),
        p3(0, 1, 2),
    ];

    let classification = classify_triangle_triangle(&points, [0, 1, 2], [3, 4, 5]);

    assert_eq!(
        classification.relation,
        TriangleTriangleRelation::SeparatedByFirstPlane
    );
    assert!(classification.right_edge_events.is_empty());
    assert!(classification.all_proof_producing());
}

#[test]
fn exact_triangle_triangle_keeps_coplanar_overlap_for_later_graph() {
    let points = [
        p3(0, 0, 0),
        p3(2, 0, 0),
        p3(0, 2, 0),
        p3(1, 0, 0),
        p3(3, 0, 0),
        p3(1, 2, 0),
    ];

    let classification = classify_triangle_triangle(&points, [0, 1, 2], [3, 4, 5]);

    assert_eq!(
        classification.relation,
        TriangleTriangleRelation::CoplanarOverlapping
    );
    assert_eq!(
        classification.coplanar.as_ref().unwrap().relation,
        CoplanarTriangleRelation::Overlapping
    );
    assert!(classification.right_edge_events.is_empty());
    assert!(classification.left_edge_events.is_empty());
    assert!(classification.all_proof_producing());
}

#[test]
fn exact_coplanar_triangle_classifier_distinguishes_disjoint_touching_and_overlap() {
    let disjoint_points = [
        p3(0, 0, 0),
        p3(2, 0, 0),
        p3(0, 2, 0),
        p3(3, 0, 0),
        p3(5, 0, 0),
        p3(3, 2, 0),
    ];
    let touching_points = [
        p3(0, 0, 0),
        p3(2, 0, 0),
        p3(0, 2, 0),
        p3(2, 0, 0),
        p3(4, 0, 0),
        p3(2, 2, 0),
    ];
    let overlapping_points = [
        p3(0, 0, 0),
        p3(2, 0, 0),
        p3(0, 2, 0),
        p3(1, 0, 0),
        p3(3, 0, 0),
        p3(1, 2, 0),
    ];

    assert_eq!(
        classify_coplanar_triangles(&disjoint_points, [0, 1, 2], [3, 4, 5]).relation,
        CoplanarTriangleRelation::Disjoint
    );
    assert_eq!(
        classify_coplanar_triangles(&touching_points, [0, 1, 2], [3, 4, 5]).relation,
        CoplanarTriangleRelation::Touching
    );
    assert_eq!(
        classify_coplanar_triangles(&overlapping_points, [0, 1, 2], [3, 4, 5]).relation,
        CoplanarTriangleRelation::Overlapping
    );
}

#[test]
fn exact_triangle_triangle_retains_segment_plane_events_for_candidates() {
    let points = [
        p3(0, 0, 0),
        p3(2, 0, 0),
        p3(0, 2, 0),
        p3(0, 0, -1),
        p3(2, 0, 1),
        p3(0, 2, 1),
    ];

    let classification = classify_triangle_triangle(&points, [0, 1, 2], [3, 4, 5]);

    assert_eq!(classification.relation, TriangleTriangleRelation::Candidate);
    assert_eq!(classification.right_edge_events.len(), 3);
    assert!(
        classification
            .right_edge_events
            .iter()
            .filter(|event| event.relation == SegmentPlaneRelation::ProperCrossing)
            .count()
            >= 2
    );
    assert!(classification.all_proof_producing());
}

#[test]
fn exact_mesh_face_pair_classifier_uses_bounds_before_triangle_predicates() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[10, 0, 0, 11, 0, 0, 10, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let classification = classify_mesh_face_pair(&left, 0, &right, 0).unwrap();

    assert_eq!(
        classification.relation,
        MeshFacePairRelation::BoundsDisjoint
    );
    assert!(!classification.needs_graph_construction());
    assert!(classification.triangle.is_none());
}

#[test]
fn exact_mesh_face_pair_classifier_retains_triangle_candidates() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, -1, 2, 0, 1, 0, 2, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let classification = classify_mesh_face_pair(&left, 0, &right, 0).unwrap();

    assert_eq!(classification.relation, MeshFacePairRelation::Candidate);
    assert!(classification.needs_graph_construction());
    assert!(classification.triangle.is_some());
}

#[test]
fn exact_mesh_face_pair_classifier_rejects_coplanar_disjoint_pairs() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[3, 3, 0, 4, 3, 0, 3, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let classification = classify_mesh_face_pair(&left, 0, &right, 0).unwrap();

    assert_eq!(
        classification.relation,
        MeshFacePairRelation::PlaneSeparated
    );
    assert!(!classification.needs_graph_construction());
    assert_eq!(
        classification.triangle.as_ref().unwrap().relation,
        TriangleTriangleRelation::CoplanarDisjoint
    );
}

#[test]
fn exact_intersection_graph_records_noncoplanar_split_events() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, -1, 2, 0, 1, 0, 2, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let graph = build_intersection_graph(&left, &right).unwrap();

    assert_eq!(graph.face_pairs.len(), 1);
    assert!(!graph.has_unknowns());
    assert!(graph.face_pairs[0].events.iter().any(|event| matches!(
        event,
        IntersectionEvent::SegmentPlane {
            relation: SegmentPlaneRelation::ProperCrossing,
            point: Some(_),
            parameter: Some(_),
            ..
        }
    )));

    let split_plan = graph.edge_split_plan();
    assert_eq!(split_plan.unknown_orderings, 0);
    assert!(split_plan.point_count() >= 2);
    assert!(split_plan.splits.iter().all(|split| {
        split
            .points
            .iter()
            .all(|point| real_between_unit(&point.parameter))
    }));

    let vertex_plan = graph.graph_vertex_plan();
    assert_eq!(vertex_plan.unresolved_equalities, 0);
    assert!(vertex_plan.vertices.len() <= split_plan.point_count());
    assert!(
        vertex_plan
            .vertices
            .iter()
            .all(|vertex| !vertex.uses.is_empty())
    );

    let topology_plan = graph.split_topology_plan();
    assert_eq!(topology_plan.unresolved_equalities, 0);
    assert_eq!(topology_plan.unresolved_vertex_lookups, 0);
    assert_eq!(topology_plan.unknown_orderings, 0);
    assert!(topology_plan.validate().is_valid());
    assert!(!topology_plan.edge_chains.is_empty());
    assert_eq!(
        topology_plan.referenced_graph_vertices(),
        split_plan.point_count()
    );
    assert!(
        topology_plan
            .edge_chains
            .iter()
            .all(|chain| chain.nodes.len() >= 3)
    );

    let face_plan = graph.face_split_plan();
    assert!(!face_plan.faces.is_empty());
    assert!(face_plan.graph_vertex_references() >= topology_plan.referenced_graph_vertices());
    assert!(face_plan.faces.iter().all(|face| !face.edges.is_empty()));
    assert!(
        face_plan
            .validate_against_topology(&topology_plan)
            .is_valid()
    );

    let geometry_plan = graph.face_split_geometry_plan(&left, &right).unwrap();
    assert_eq!(geometry_plan.faces.len(), face_plan.faces.len());
    assert_eq!(
        geometry_plan.graph_vertex_references(),
        face_plan.graph_vertex_references()
    );
    assert!(geometry_plan.faces.iter().all(|face| {
        !face.boundary_chains.is_empty()
            && face.boundary_chains.iter().all(|chain| {
                chain.nodes.len() >= 3
                    && matches!(
                        chain.nodes.first(),
                        Some(FaceSplitBoundaryNode::OriginalVertex { .. })
                    )
                    && matches!(
                        chain.nodes.last(),
                        Some(FaceSplitBoundaryNode::OriginalVertex { .. })
                    )
                    && chain
                        .nodes
                        .iter()
                        .any(|node| matches!(node, FaceSplitBoundaryNode::GraphVertex { .. }))
            })
    }));
    assert!(
        geometry_plan
            .validate_boundary_incidence(&left, &right)
            .is_valid()
    );

    let region_plan = geometry_plan.region_plan(&left, &right);
    assert_eq!(region_plan.regions.len(), geometry_plan.faces.len());
    assert_eq!(
        region_plan.graph_vertex_references(),
        geometry_plan.graph_vertex_references()
    );
    assert!(region_plan.validate(&left, &right).is_valid());
    assert!(region_plan.regions.iter().all(|region| {
        region.boundary.len() >= 4
            && region
                .boundary
                .iter()
                .any(|node| matches!(node, FaceSplitBoundaryNode::GraphVertex { .. }))
    }));

    let region_classifications =
        classify_face_regions_against_opposite_planes(&region_plan, &left, &right);
    assert_eq!(
        region_classifications.len(),
        region_plan.regions.len() * right.triangles().len()
    );
    assert!(
        region_classifications
            .iter()
            .all(|classification| classification.all_proof_producing())
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_face_region_triangulates_through_feature_gated_hypertri() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, -1, 2, 0, 1, 0, 2, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let graph = build_intersection_graph(&left, &right).unwrap();
    let geometry = graph.face_split_geometry_plan(&left, &right).unwrap();
    let region_plan = geometry.region_plan(&left, &right);

    let triangulations =
        hypermesh::exact::triangulate_face_regions_with_earcut(&region_plan, &left, &right)
            .unwrap();

    assert_eq!(triangulations.len(), region_plan.regions.len());
    assert!(triangulations.iter().all(|triangulation| {
        triangulation.triangles.len() % 3 == 0
            && triangulation
                .triangles
                .iter()
                .all(|&index| index < triangulation.vertices.len())
    }));

    let assembly = hypermesh::exact::ExactBooleanAssemblyPlan::from_region_triangulations(
        &triangulations,
        hypermesh::exact::ExactRegionSelection::KeepAll,
    )
    .unwrap();

    assert!(!assembly.vertices.is_empty());
    assert!(!assembly.triangles.is_empty());
    assert!(assembly.validate().is_ok());
    assert!(assembly.triangles.iter().all(|triangle| {
        triangle
            .vertices
            .iter()
            .all(|&vertex| vertex < assembly.vertices.len())
    }));

    let left_only = hypermesh::exact::ExactBooleanAssemblyPlan::from_region_triangulations(
        &triangulations,
        hypermesh::exact::ExactRegionSelection::KeepLeft,
    )
    .unwrap();
    let right_only = hypermesh::exact::ExactBooleanAssemblyPlan::from_region_triangulations(
        &triangulations,
        hypermesh::exact::ExactRegionSelection::KeepRight,
    )
    .unwrap();
    assert_eq!(
        left_only.triangles.len() + right_only.triangles.len(),
        assembly.triangles.len()
    );
    assert!(
        left_only
            .triangles
            .iter()
            .all(|triangle| triangle.source_side == MeshSide::Left)
    );
    assert!(
        right_only
            .triangles
            .iter()
            .all(|triangle| triangle.source_side == MeshSide::Right)
    );

    let output = assembly
        .to_exact_mesh(ValidationPolicy::ALLOW_BOUNDARY)
        .unwrap();
    assert_eq!(output.vertices().len(), assembly.vertices.len());
    assert_eq!(output.triangles().len(), assembly.triangles.len());
    assert_eq!(
        output.provenance().source.label,
        "exact boolean assembly plan"
    );

    let pipelined = hypermesh::exact::build_selected_region_mesh(
        &left,
        &right,
        hypermesh::exact::ExactRegionSelection::KeepAll,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(pipelined.triangles().len(), output.triangles().len());

    let boolean = hypermesh::exact::boolean_selected_regions(
        &left,
        &right,
        hypermesh::exact::ExactBooleanPolicy::KEEP_ALL_BOUNDARY,
    )
    .unwrap();
    assert!(!boolean.graph_had_unknowns);
    assert_eq!(boolean.mesh.triangles().len(), output.triangles().len());
    assert_eq!(
        boolean.region_classifications.len(),
        region_plan.regions.len() * right.triangles().len()
    );

    let exact = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::SelectedRegions(
            hypermesh::exact::ExactRegionSelection::KeepAll,
        ),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(exact.mesh.triangles().len(), output.triangles().len());

    let unsupported = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap_err();
    assert!(
        unsupported
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::UnsupportedExactOperation)
    );

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::RequiresCertifiedWinding
    );
    assert!(!preflight.graph_had_unknowns);
    assert!(preflight.retained_face_pairs > 0);
    assert!(preflight.retained_events > 0);
    assert_eq!(preflight.region_count, region_plan.regions.len());
    assert_eq!(
        preflight.region_classifications.len(),
        region_plan.regions.len() * right.triangles().len()
    );
    assert!(
        preflight
            .region_classifications
            .iter()
            .all(|classification| classification.all_proof_producing())
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_handle_certified_aabb_disjoint_meshes() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[10, 0, 0, 11, 0, 0, 10, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let union = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(union.mesh.triangles().len(), 2);
    assert_eq!(union.mesh.vertices().len(), 6);

    let intersection = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(intersection.mesh.triangles().is_empty());
    assert!(intersection.mesh.vertices().is_empty());

    let difference = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(difference.mesh.triangles(), left.triangles());
    assert_eq!(difference.mesh.vertices(), left.vertices());

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedBoundsDisjoint
    );
    assert_eq!(preflight.retained_face_pairs, 0);
    assert!(preflight.region_classifications.is_empty());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_handle_structurally_identical_meshes() {
    let mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let union = hypermesh::exact::boolean_exact(
        &mesh,
        &mesh,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(union.mesh.triangles(), mesh.triangles());
    assert_eq!(union.mesh.vertices(), mesh.vertices());

    let intersection = hypermesh::exact::boolean_exact(
        &mesh,
        &mesh,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(intersection.mesh.triangles(), mesh.triangles());
    assert_eq!(intersection.mesh.vertices(), mesh.vertices());

    let difference = hypermesh::exact::boolean_exact(
        &mesh,
        &mesh,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(difference.mesh.triangles().is_empty());
    assert!(difference.mesh.vertices().is_empty());

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &mesh,
        &mesh,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedIdentical
    );
    assert_eq!(preflight.region_count, 0);
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_handle_empty_operands() {
    let empty =
        ExactMesh::from_i64_triangles_with_policy(&[], &[], ValidationPolicy::ALLOW_BOUNDARY)
            .unwrap();
    let mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let union = hypermesh::exact::boolean_exact(
        &empty,
        &mesh,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(union.mesh.triangles(), mesh.triangles());

    let intersection = hypermesh::exact::boolean_exact(
        &mesh,
        &empty,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(intersection.mesh.triangles().is_empty());

    let left_empty_difference = hypermesh::exact::boolean_exact(
        &empty,
        &mesh,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(left_empty_difference.mesh.triangles().is_empty());

    let right_empty_difference = hypermesh::exact::boolean_exact(
        &mesh,
        &empty,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(right_empty_difference.mesh.triangles(), mesh.triangles());

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &empty,
        &mesh,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedEmptyOperand
    );
    assert_eq!(preflight.retained_events, 0);
}

#[test]
fn exact_split_plan_validation_rejects_unresolved_and_malformed_topology() {
    let topology_plan = ExactSplitTopologyPlan {
        graph_vertices: vec![ExactGraphVertex {
            point: p3(0, 0, 0),
            uses: Vec::new(),
        }],
        edge_chains: vec![SplitEdgeChain {
            side: MeshSide::Left,
            edge: [0, 1],
            nodes: vec![
                SplitEdgeNode::OriginalVertex {
                    side: MeshSide::Right,
                    vertex: 0,
                },
                SplitEdgeNode::GraphVertex { graph_vertex: 7 },
                SplitEdgeNode::OriginalVertex {
                    side: MeshSide::Left,
                    vertex: 2,
                },
            ],
        }],
        unresolved_vertex_lookups: 1,
        unresolved_equalities: 1,
        unknown_orderings: 1,
    };

    let report = topology_plan.validate();

    assert!(!report.is_valid());
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == SplitPlanDiagnosticKind::UnknownOrdering)
    );
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == SplitPlanDiagnosticKind::UnresolvedEquality)
    );
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == SplitPlanDiagnosticKind::GraphVertexOutOfRange)
    );
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == SplitPlanDiagnosticKind::WrongChainEnd)
    );
}

#[test]
fn exact_face_split_plan_validation_rejects_duplicate_and_unmatched_edges() {
    let topology_plan = ExactSplitTopologyPlan {
        graph_vertices: vec![ExactGraphVertex {
            point: p3(0, 0, 0),
            uses: Vec::new(),
        }],
        edge_chains: Vec::new(),
        unresolved_vertex_lookups: 0,
        unresolved_equalities: 0,
        unknown_orderings: 0,
    };
    let face_plan = ExactFaceSplitPlan {
        faces: vec![FaceSplitPlan {
            side: MeshSide::Left,
            face: 0,
            edges: vec![
                FaceSplitEdge {
                    edge: [0, 1],
                    graph_vertices: vec![0],
                },
                FaceSplitEdge {
                    edge: [0, 1],
                    graph_vertices: vec![3],
                },
            ],
        }],
    };

    let report = face_plan.validate_against_topology(&topology_plan);

    assert!(!report.is_valid());
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == SplitPlanDiagnosticKind::DuplicateFaceSplitEdge)
    );
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == SplitPlanDiagnosticKind::GraphVertexOutOfRange)
    );
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::MissingFaceSplitSourceUse
    }));
}

#[test]
fn exact_face_split_geometry_validation_rejects_off_plane_boundary_node() {
    let mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let geometry = ExactFaceSplitGeometryPlan {
        faces: vec![FaceSplitGeometry {
            side: MeshSide::Left,
            face: 0,
            triangle: [0, 1, 2],
            boundary_chains: vec![FaceSplitBoundaryChain {
                edge: [0, 1],
                nodes: vec![FaceSplitBoundaryNode::GraphVertex {
                    graph_vertex: 0,
                    point: p3(1, 0, 1),
                }],
            }],
        }],
    };

    let report = geometry.validate_boundary_incidence(&mesh, &mesh);

    assert!(!report.is_valid());
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::BoundaryNodeOffFacePlane
    }));
}

#[test]
fn exact_face_region_validation_rejects_duplicate_boundary_nodes() {
    let mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let point = p3(0, 0, 0);
    let region_plan = hypermesh::exact::ExactFaceRegionPlan {
        regions: vec![FaceRegionBoundary {
            side: MeshSide::Left,
            face: 0,
            triangle: [0, 1, 2],
            boundary: vec![
                FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 0,
                    point: point.clone(),
                },
                FaceSplitBoundaryNode::OriginalVertex { vertex: 0, point },
            ],
        }],
    };

    let report = region_plan.validate(&mesh, &mesh);

    assert!(!report.is_valid());
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::EmptyOrShortRegionBoundary
    }));
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::DuplicateConsecutiveRegionNode
    }));
}

#[test]
fn exact_intersection_graph_records_coplanar_edge_and_vertex_events() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, 0, 0, 3, 0, 0, 1, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let graph = build_intersection_graph(&left, &right).unwrap();

    assert_eq!(graph.face_pairs.len(), 1);
    assert_eq!(
        graph.face_pairs[0].relation,
        MeshFacePairRelation::CoplanarOverlapping
    );
    assert!(graph.face_pairs[0].projection.is_some());
    assert!(
        graph.face_pairs[0]
            .events
            .iter()
            .any(|event| matches!(event, IntersectionEvent::CoplanarEdge { .. }))
    );
    assert!(
        graph.face_pairs[0]
            .events
            .iter()
            .any(|event| matches!(event, IntersectionEvent::CoplanarVertex { .. }))
    );
}

#[test]
fn exact_mesh_face_pair_classifier_rejects_out_of_range_faces() {
    let mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let err = classify_mesh_face_pair(&mesh, 1, &mesh, 0).unwrap_err();
    assert!(
        err.diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::IndexOutOfBounds)
    );
}

#[test]
fn exact_mesh_face_pair_batch_retains_only_graph_construction_pairs() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 2, 0, 0, 0, 2, 0, //
            20, 0, 0, 22, 0, 0, 20, 2, 0,
        ],
        &[0, 1, 2, 3, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, -1, 2, 0, 1, 0, 2, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let retained = classify_mesh_face_pairs(&left, &right).unwrap();

    assert_eq!(retained.len(), 1);
    assert_eq!(retained[0].left_face, 0);
    assert_eq!(retained[0].right_face, 0);
    assert_eq!(retained[0].relation, MeshFacePairRelation::Candidate);
}

#[test]
fn exact_mesh_rejects_non_finite_lossy_input_before_predicates() {
    let (mut pos, idx) = tetrahedron();
    pos[2] = f64::NAN;

    let err = ExactMesh::from_f64_triangles(&pos, &idx).unwrap_err();
    assert_eq!(err.diagnostics[0].kind, DiagnosticKind::NonFiniteCoordinate);
    assert_eq!(err.diagnostics[0].severity, Severity::Error);
    assert_eq!(err.diagnostics[0].coordinate, Some(2));
}

#[test]
fn exact_mesh_rejects_out_of_range_indices_without_panicking() {
    let (pos, mut idx) = tetrahedron();
    idx[4] = 99;

    let err = ExactMesh::from_f64_triangles(&pos, &idx).unwrap_err();
    assert!(
        err.diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::IndexOutOfBounds)
    );
}

#[test]
fn exact_validation_reports_boundary_and_duplicate_directed_edges() {
    let points = [p3(0, 0, 0), p3(1, 0, 0), p3(0, 1, 0), p3(1, 1, 0)];
    let triangles = [[0, 1, 2], [1, 3, 2]];
    let report = validate_triangles(&points, &triangles);

    assert!(!report.is_valid());
    assert_eq!(report.facts.mesh.boundary_edges, 4);
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::BoundaryEdge)
    );
}

#[test]
fn exact_validation_boundary_policy_allows_disk_links() {
    let points = [p3(0, 0, 0), p3(1, 0, 0), p3(0, 1, 0)];
    let report =
        validate_triangles_with_policy(&points, &[[0, 1, 2]], ValidationPolicy::ALLOW_BOUNDARY);

    assert!(report.is_valid());
    assert!(!report.facts.mesh.closed_manifold);
    assert_eq!(report.facts.mesh.boundary_edges, 3);
    assert!(
        report
            .facts
            .vertices
            .iter()
            .all(|vertex| vertex.link == VertexLinkKind::Disk)
    );
}

#[test]
fn exact_mesh_boundary_policy_constructs_open_mesh_explicitly() {
    let pos = vec![0, 0, 0, 1, 0, 0, 0, 1, 0];
    let idx = vec![0, 1, 2];
    let mesh =
        ExactMesh::from_i64_triangles_with_policy(&pos, &idx, ValidationPolicy::ALLOW_BOUNDARY)
            .unwrap();

    assert_eq!(mesh.facts().mesh.boundary_edges, 3);
    assert!(!mesh.facts().mesh.closed_manifold);
}

#[test]
fn exact_validation_reports_bow_tie_vertex_link() {
    let points = [
        p3(0, 0, 0),
        p3(1, 0, 0),
        p3(0, 1, 0),
        p3(-1, 0, 0),
        p3(0, 0, 1),
        p3(0, 1, 1),
        p3(0, 0, 2),
    ];
    let report = validate_triangles(&points, &[[0, 1, 2], [0, 2, 3], [0, 4, 5], [0, 5, 6]]);

    assert_eq!(report.facts.vertices[0].link, VertexLinkKind::NonManifold);
    assert_eq!(report.facts.mesh.non_manifold_vertices, 1);
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::NonManifoldVertexLink)
    );
}

#[test]
fn exact_validation_rejects_collinear_triangle_without_epsilon() {
    let points = [p3(0, 0, 0), p3(1, 1, 1), p3(2, 2, 2)];
    let report = validate_triangles(&points, &[[0, 1, 2]]);

    assert_eq!(report.facts.mesh.degenerate_triangles, 1);
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::DegenerateTriangle)
    );
}

proptest! {
    #[test]
    fn malformed_f64_imports_never_panic(
        coords in proptest::collection::vec(any::<f64>(), 0..80),
        indices in proptest::collection::vec(any::<usize>(), 0..80),
    ) {
        let _ = ExactMesh::from_f64_triangles(&coords, &indices);
    }

    #[test]
    fn repeated_triangle_vertices_are_rejected(a in 0usize..8, b in 0usize..8) {
        let points = vec![
            0.0, 0.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
        ];
        let idx = vec![a % 3, b % 3, a % 3];
        let err = ExactMesh::from_f64_triangles(&points, &idx).unwrap_err();
        prop_assert!(
            err.diagnostics
                .iter()
                .any(|diagnostic| diagnostic.kind == DiagnosticKind::DegenerateTriangle)
        );
    }

    #[test]
    fn generated_integer_vertical_segments_cross_z_plane_exactly(x in -16i32..16, y in -16i32..16, h in 1i32..16) {
        let points = [
            p3(0, 0, 0),
            p3(1, 0, 0),
            p3(0, 1, 0),
            p3(x, y, -h),
            p3(x, y, h),
        ];
        let event = intersect_segment_with_face_plane(&points, [0, 1, 2], [3, 4]);

        prop_assert_eq!(event.relation, SegmentPlaneRelation::ProperCrossing);
        prop_assert!(event.all_proof_producing());
        prop_assert!(real_eq(event.parameter.as_ref().unwrap(), &half()));
        let point = event.point.as_ref().unwrap();
        prop_assert!(real_eq(&point.x, &ExactReal::from(x)));
        prop_assert!(real_eq(&point.y, &ExactReal::from(y)));
        prop_assert!(real_eq(&point.z, &ExactReal::from(0)));
    }

    #[test]
    fn generated_triangle_pairs_with_straddling_vertex_remain_candidates(h in 1i32..16) {
        let points = [
            p3(0, 0, 0),
            p3(2, 0, 0),
            p3(0, 2, 0),
            p3(0, 0, -h),
            p3(2, 0, h),
            p3(0, 2, h),
        ];
        let classification = classify_triangle_triangle(&points, [0, 1, 2], [3, 4, 5]);

        prop_assert_eq!(classification.relation, TriangleTriangleRelation::Candidate);
        prop_assert!(classification.all_proof_producing());
        prop_assert_eq!(classification.right_edge_events.len(), 3);
    }
}

fn p3(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

fn half() -> ExactReal {
    (ExactReal::from(1) / ExactReal::from(2)).expect("nonzero denominator")
}

fn assert_real_eq(left: &ExactReal, right: &ExactReal) {
    assert!(real_eq(left, right), "expected {left} == {right}");
}

fn real_eq(left: &ExactReal, right: &ExactReal) -> bool {
    compare_reals(left, right).value() == Some(Ordering::Equal)
}

fn real_between_unit(value: &ExactReal) -> bool {
    let zero = ExactReal::from(0);
    let one = ExactReal::from(1);
    matches!(
        compare_reals(value, &zero).value(),
        Some(Ordering::Greater | Ordering::Equal)
    ) && matches!(
        compare_reals(value, &one).value(),
        Some(Ordering::Less | Ordering::Equal)
    )
}
