#![cfg(feature = "exact")]

use hyperlimit::{PlaneSide, Point3, compare_reals};
#[cfg(feature = "exact-triangulation")]
use hypermesh::exact::CoplanarProjection;
#[cfg(feature = "exact-triangulation")]
use hypermesh::exact::checked_classify_face_regions_against_opposite_planes;
#[cfg(not(feature = "exact-triangulation"))]
use hypermesh::exact::classify_face_regions_against_opposite_planes;
use hypermesh::exact::{
    AabbIntersectionKind, CoplanarTriangleRelation, DiagnosticKind, EdgeSplit, EdgeSplitPoint,
    ExactAabb3, ExactEdgeSplitPlan, ExactFaceSplitGeometryPlan, ExactFaceSplitPlan,
    ExactGraphVertex, ExactGraphVertexPlan, ExactGraphVertexUse, ExactIntersectionGraph, ExactMesh,
    ExactReal, ExactSplitTopologyPlan, FacePairEvents, FaceRegionBoundary, FaceSplitBoundaryChain,
    FaceSplitBoundaryNode, FaceSplitEdge, FaceSplitGeometry, FaceSplitPlan, IntersectionEvent,
    MeshFacePairRelation, MeshSide, SegmentPlaneRelation, Severity, SplitEdgeChain, SplitEdgeNode,
    SplitPlanDiagnosticKind, TrianglePlaneRelation, TriangleTriangleRelation, ValidationPolicy,
    VertexLinkKind, build_intersection_graph, certify_convex_solid, classify_coplanar_triangles,
    classify_mesh_face_pair, classify_mesh_face_pairs,
    classify_mesh_triangle_against_retained_face_plane,
    classify_mesh_vertices_against_convex_solid,
    classify_mesh_vertices_against_convex_solid_report, classify_point_against_convex_solid,
    classify_point_against_convex_solid_report, classify_triangle_against_face_plane,
    classify_triangle_triangle, intersect_segment_with_face_plane,
    intersect_segment_with_retained_face_plane, validate_triangles, validate_triangles_with_policy,
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
    let base_plane = &mesh.facts().faces[0].plane;
    assert_eq!(
        compare_reals(&base_plane.normal[0], &ExactReal::from(0)).value(),
        Some(Ordering::Equal)
    );
    assert_eq!(
        compare_reals(&base_plane.normal[1], &ExactReal::from(0)).value(),
        Some(Ordering::Equal)
    );
    assert_eq!(
        compare_reals(&base_plane.normal[2], &ExactReal::from(-1)).value(),
        Some(Ordering::Equal)
    );
    assert_eq!(
        compare_reals(&base_plane.offset, &ExactReal::from(0)).value(),
        Some(Ordering::Equal)
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
fn exact_narrow_phase_reuses_retained_face_plane_facts() {
    let plane = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let below = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, -2, 1, 0, -2, 0, 1, -2],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let retained =
        classify_mesh_triangle_against_retained_face_plane(&plane, 0, &below, 0).unwrap();
    let points = plane
        .vertices()
        .iter()
        .chain(below.vertices())
        .map(|point| point.to_hyperlimit_point())
        .collect::<Vec<_>>();
    let predicate = classify_triangle_against_face_plane(&points, [0, 1, 2], [3, 4, 5]);

    assert_eq!(retained.relation, predicate.relation);
    assert_eq!(retained.vertex_sides, predicate.vertex_sides);
    assert_eq!(retained.relation, TrianglePlaneRelation::StrictlyAbove);
    assert!(retained.predicates.is_empty());
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
fn exact_segment_plane_reuses_retained_face_plane_for_crossing() {
    let plane = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let p0 = p3(0, 0, -1);
    let p1 = p3(0, 0, 1);
    let retained =
        intersect_segment_with_retained_face_plane(&plane.facts().faces[0].plane, &p0, &p1);
    let predicate = intersect_segment_with_face_plane(
        &[p3(0, 0, 0), p3(1, 0, 0), p3(0, 1, 0), p0, p1],
        [0, 1, 2],
        [3, 4],
    );

    assert_eq!(retained.relation, predicate.relation);
    assert_eq!(retained.endpoint_sides, predicate.endpoint_sides);
    assert_eq!(retained.relation, SegmentPlaneRelation::ProperCrossing);
    assert!(retained.predicates.is_empty());
    assert_real_eq(retained.parameter.as_ref().unwrap(), &half());
    let point = retained.point.as_ref().unwrap();
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
fn exact_mesh_face_pair_classifier_uses_retained_planes_before_triangle_predicates() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 2, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 1, 1, 0, 2, 0, 1, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    assert!(
        left.bounds().faces[0]
            .classify_intersection(&right.bounds().faces[0])
            .value()
            .is_none_or(AabbIntersectionKind::needs_narrow_phase)
    );

    let classification = classify_mesh_face_pair(&left, 0, &right, 0).unwrap();

    assert_eq!(
        classification.relation,
        MeshFacePairRelation::PlaneSeparated
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
    let triangle = classification.triangle.as_ref().unwrap();
    assert_eq!(triangle.right_edge_events.len(), 3);
    assert_eq!(triangle.left_edge_events.len(), 3);
    assert!(
        triangle
            .right_edge_events
            .iter()
            .chain(&triangle.left_edge_events)
            .all(|event| event.predicates.is_empty())
    );
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
fn exact_edge_split_validation_rejects_missing_and_noncrossing_side_facts() {
    let split_plan = ExactEdgeSplitPlan {
        splits: vec![EdgeSplit {
            side: MeshSide::Left,
            edge: [0, 1],
            points: vec![
                EdgeSplitPoint {
                    face_pair: [0, 0],
                    plane_face: 0,
                    parameter: half(),
                    point: p3(0, 0, 0),
                    endpoint_sides: [None, Some(PlaneSide::Below)],
                },
                EdgeSplitPoint {
                    face_pair: [0, 0],
                    plane_face: 0,
                    parameter: half(),
                    point: p3(0, 0, 0),
                    endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Above)],
                },
            ],
        }],
        unknown_orderings: 1,
    };

    let report = split_plan.validate();

    assert!(!report.is_valid());
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::MissingEndpointSideFacts
    }));
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::NonCrossingEndpointSideFacts
    }));
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == SplitPlanDiagnosticKind::UnknownOrdering)
    );
}

#[test]
fn exact_checked_topology_plan_rejects_invalid_edge_split_handoff() {
    let graph = ExactIntersectionGraph {
        face_pairs: vec![FacePairEvents {
            left_face: 0,
            right_face: 0,
            relation: MeshFacePairRelation::Candidate,
            projection: None,
            events: vec![IntersectionEvent::SegmentPlane {
                segment_side: MeshSide::Left,
                edge: [0, 1],
                plane_side: MeshSide::Right,
                plane_face: 0,
                relation: SegmentPlaneRelation::ProperCrossing,
                point: Some(p3(0, 0, 0)),
                parameter: Some(half()),
                endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Above)],
            }],
        }],
    };

    let report = graph.checked_split_topology_plan().unwrap_err();

    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::NonCrossingEndpointSideFacts
    }));
    let face_report = graph.checked_face_split_plan().unwrap_err();
    assert!(face_report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::NonCrossingEndpointSideFacts
    }));
}

#[test]
fn exact_graph_vertex_plan_retains_and_validates_source_side_facts() {
    let graph = ExactIntersectionGraph {
        face_pairs: vec![FacePairEvents {
            left_face: 0,
            right_face: 0,
            relation: MeshFacePairRelation::Candidate,
            projection: None,
            events: vec![IntersectionEvent::SegmentPlane {
                segment_side: MeshSide::Left,
                edge: [0, 1],
                plane_side: MeshSide::Right,
                plane_face: 0,
                relation: SegmentPlaneRelation::ProperCrossing,
                point: Some(p3(0, 0, 0)),
                parameter: Some(half()),
                endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)],
            }],
        }],
    };

    let vertex_plan = graph.checked_graph_vertex_plan().unwrap();

    assert_eq!(vertex_plan.source_use_count(), 1);
    assert!(vertex_plan.validate().is_valid());
    assert_eq!(vertex_plan.vertices[0].uses[0].parameter, half());
    assert_eq!(
        vertex_plan.vertices[0].uses[0].endpoint_sides,
        [Some(PlaneSide::Above), Some(PlaneSide::Below)]
    );
}

#[test]
fn exact_graph_vertex_validation_rejects_unresolved_and_bad_source_facts() {
    let vertex_plan = ExactGraphVertexPlan {
        vertices: vec![
            ExactGraphVertex {
                point: p3(0, 0, 0),
                uses: Vec::new(),
            },
            ExactGraphVertex {
                point: p3(1, 0, 0),
                uses: vec![
                    ExactGraphVertexUse {
                        side: MeshSide::Left,
                        edge: [0, 1],
                        face_pair: [0, 0],
                        plane_face: 0,
                        parameter: half(),
                        endpoint_sides: [None, Some(PlaneSide::Below)],
                    },
                    ExactGraphVertexUse {
                        side: MeshSide::Right,
                        edge: [2, 3],
                        face_pair: [0, 0],
                        plane_face: 0,
                        parameter: half(),
                        endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Above)],
                    },
                ],
            },
        ],
        unresolved_equalities: 1,
    };

    let report = vertex_plan.validate();

    assert!(!report.is_valid());
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
            .any(|diagnostic| diagnostic.kind == SplitPlanDiagnosticKind::EmptyGraphVertexUses)
    );
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::MissingEndpointSideFacts
    }));
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::NonCrossingEndpointSideFacts
    }));
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
            endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)]
                | [Some(PlaneSide::Below), Some(PlaneSide::Above)],
            ..
        }
    )));

    let split_plan = graph.edge_split_plan();
    assert_eq!(split_plan.unknown_orderings, 0);
    assert!(split_plan.validate().is_valid());
    assert!(split_plan.point_count() >= 2);
    assert!(split_plan.splits.iter().all(|split| {
        split.points.iter().all(|point| {
            real_between_unit(&point.parameter)
                && matches!(
                    point.endpoint_sides,
                    [Some(PlaneSide::Above), Some(PlaneSide::Below)]
                        | [Some(PlaneSide::Below), Some(PlaneSide::Above)]
                )
        })
    }));

    let vertex_plan = graph.graph_vertex_plan();
    assert_eq!(vertex_plan.unresolved_equalities, 0);
    assert!(vertex_plan.validate().is_valid());
    assert!(vertex_plan.vertices.len() <= split_plan.point_count());
    assert!(
        vertex_plan
            .vertices
            .iter()
            .all(|vertex| !vertex.uses.is_empty())
    );

    let topology_plan = graph.split_topology_plan();
    let checked_topology_plan = graph.checked_split_topology_plan().unwrap();
    assert_eq!(checked_topology_plan, topology_plan);
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
    let checked_face_plan = graph.checked_face_split_plan().unwrap();
    assert_eq!(checked_face_plan, face_plan);
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

    #[cfg(feature = "exact-triangulation")]
    let region_classifications =
        checked_classify_face_regions_against_opposite_planes(&region_plan, &left, &right).unwrap();
    #[cfg(not(feature = "exact-triangulation"))]
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
        hypermesh::exact::checked_triangulate_face_regions_with_earcut(&region_plan, &left, &right)
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
    assembly
        .validate_source_face_incidence(&left, &right)
        .unwrap();

    let output = assembly
        .checked_to_exact_mesh_with_sources(&left, &right, ValidationPolicy::ALLOW_BOUNDARY)
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
    let blocker = preflight.blocker.as_ref().unwrap();
    assert_eq!(
        blocker.kind,
        hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding
    );
    assert!(blocker.candidate_pairs > 0);
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_region_triangulation_rejects_projected_source_drift() {
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
    let mut triangulations =
        hypermesh::exact::checked_triangulate_face_regions_with_earcut(&region_plan, &left, &right)
            .unwrap();

    triangulations[0].vertices[0] = hypertri::ExactPoint::new(Real::from(99), Real::from(99));

    let error = triangulations[0].validate().unwrap_err();
    assert!(matches!(error, hypertri::Error::InvalidInput { .. }));

    let assembly_error = hypermesh::exact::ExactBooleanAssemblyPlan::from_region_triangulations(
        &triangulations,
        hypermesh::exact::ExactRegionSelection::KeepAll,
    )
    .unwrap_err();
    assert!(matches!(
        assembly_error,
        hypertri::Error::InvalidInput { .. }
    ));
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_region_triangulation_rejects_exactly_collinear_output_triangle() {
    let triangulation = hypermesh::exact::FaceRegionTriangulation {
        side: MeshSide::Left,
        face: 0,
        projection: CoplanarProjection::Xy,
        boundary: vec![
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 0,
                point: p3(0, 0, 0),
            },
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 1,
                point: p3(1, 0, 0),
            },
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 2,
                point: p3(2, 0, 0),
            },
        ],
        vertices: vec![
            hypertri::ExactPoint::new(Real::from(0), Real::from(0)),
            hypertri::ExactPoint::new(Real::from(1), Real::from(0)),
            hypertri::ExactPoint::new(Real::from(2), Real::from(0)),
        ],
        triangles: vec![0, 1, 2],
    };

    let error = triangulation.validate().unwrap_err();
    assert!(matches!(error, hypertri::Error::InvalidInput { .. }));
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_checked_assembly_materialization_rejects_invalid_triangle_indices() {
    let assembly = hypermesh::exact::ExactBooleanAssemblyPlan {
        vertices: vec![hypermesh::exact::ExactOutputVertex {
            point: p3(0, 0, 0),
            source: FaceSplitBoundaryNode::OriginalVertex {
                vertex: 0,
                point: p3(0, 0, 0),
            },
        }],
        triangles: vec![hypermesh::exact::ExactOutputTriangle {
            vertices: [0, 1, 0],
            source_side: MeshSide::Left,
            source_face: 0,
        }],
    };

    let error = assembly
        .checked_to_exact_mesh(ValidationPolicy::ALLOW_BOUNDARY)
        .unwrap_err();

    assert!(
        error
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::IndexOutOfBounds)
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_assembly_validation_rejects_output_vertex_source_mismatch() {
    let assembly = hypermesh::exact::ExactBooleanAssemblyPlan {
        vertices: vec![
            hypermesh::exact::ExactOutputVertex {
                point: p3(0, 0, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 0,
                    point: p3(1, 0, 0),
                },
            },
            hypermesh::exact::ExactOutputVertex {
                point: p3(1, 0, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 1,
                    point: p3(1, 0, 0),
                },
            },
            hypermesh::exact::ExactOutputVertex {
                point: p3(0, 1, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 2,
                    point: p3(0, 1, 0),
                },
            },
        ],
        triangles: vec![hypermesh::exact::ExactOutputTriangle {
            vertices: [0, 1, 2],
            source_side: MeshSide::Left,
            source_face: 0,
        }],
    };

    let error = assembly.validate().unwrap_err();

    assert!(matches!(error, hypertri::Error::InvalidInput { .. }));
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_assembly_validation_rejects_distinct_handles_with_same_point() {
    let assembly = hypermesh::exact::ExactBooleanAssemblyPlan {
        vertices: vec![
            hypermesh::exact::ExactOutputVertex {
                point: p3(0, 0, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 0,
                    point: p3(0, 0, 0),
                },
            },
            hypermesh::exact::ExactOutputVertex {
                point: p3(0, 0, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 1,
                    point: p3(0, 0, 0),
                },
            },
            hypermesh::exact::ExactOutputVertex {
                point: p3(0, 1, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 2,
                    point: p3(0, 1, 0),
                },
            },
        ],
        triangles: vec![hypermesh::exact::ExactOutputTriangle {
            vertices: [0, 1, 2],
            source_side: MeshSide::Left,
            source_face: 0,
        }],
    };

    let error = assembly.validate().unwrap_err();

    assert!(matches!(error, hypertri::Error::InvalidInput { .. }));
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_assembly_source_face_incidence_rejects_off_plane_output() {
    let mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let assembly = hypermesh::exact::ExactBooleanAssemblyPlan {
        vertices: vec![
            hypermesh::exact::ExactOutputVertex {
                point: p3(0, 0, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 0,
                    point: p3(0, 0, 0),
                },
            },
            hypermesh::exact::ExactOutputVertex {
                point: p3(1, 0, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 1,
                    point: p3(1, 0, 0),
                },
            },
            hypermesh::exact::ExactOutputVertex {
                point: p3(0, 1, 1),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 2,
                    point: p3(0, 1, 1),
                },
            },
        ],
        triangles: vec![hypermesh::exact::ExactOutputTriangle {
            vertices: [0, 1, 2],
            source_side: MeshSide::Left,
            source_face: 0,
        }],
    };

    let error = assembly
        .validate_source_face_incidence(&mesh, &mesh)
        .unwrap_err();

    assert!(matches!(error, hypertri::Error::InvalidInput { .. }));
    let materialize_error = assembly
        .checked_to_exact_mesh_with_sources(&mesh, &mesh, ValidationPolicy::ALLOW_BOUNDARY)
        .unwrap_err();
    assert!(
        materialize_error
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::DegenerateTriangle)
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_checked_region_triangulation_rejects_invalid_region_before_earcut() {
    let mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let region_plan = hypermesh::exact::ExactFaceRegionPlan {
        regions: vec![FaceRegionBoundary {
            side: MeshSide::Left,
            face: 0,
            triangle: [0, 1, 2],
            boundary: vec![
                FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 0,
                    point: p3(0, 0, 0),
                },
                FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 0,
                    point: p3(0, 0, 0),
                },
            ],
        }],
    };

    let error =
        hypermesh::exact::checked_triangulate_face_regions_with_earcut(&region_plan, &mesh, &mesh)
            .unwrap_err();

    assert!(matches!(error, hypertri::Error::InvalidInput { .. }));
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
fn exact_preflight_reports_boundary_touching_policy_gap() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 0, 0, 4, 0, 0, 2, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::RequiresBoundaryPolicy
    );
    assert!(!preflight.graph_had_unknowns);
    assert_eq!(preflight.retained_face_pairs, 1);
    assert!(preflight.retained_events > 0);
    assert_eq!(preflight.region_count, 0);
    assert!(preflight.region_classifications.is_empty());
    let blocker = preflight.blocker.as_ref().unwrap();
    assert_eq!(
        blocker.kind,
        hypermesh::exact::ExactBooleanBlockerKind::NeedsBoundaryPolicy
    );
    assert_eq!(blocker.coplanar_touching_pairs, 1);
    assert_eq!(blocker.candidate_pairs, 0);
    assert_eq!(blocker.coplanar_overlapping_pairs, 0);

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

    let union = hypermesh::exact::boolean_exact_with_boundary_policy(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        hypermesh::exact::ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    assert_eq!(union.mesh.triangles().len(), 2);
    assert_eq!(union.mesh.vertices().len(), 6);

    let intersection = hypermesh::exact::boolean_exact_with_boundary_policy(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
        hypermesh::exact::ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    assert!(intersection.mesh.triangles().is_empty());

    let difference = hypermesh::exact::boolean_exact_with_boundary_policy(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        hypermesh::exact::ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    assert_eq!(difference.mesh.triangles(), left.triangles());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_handle_single_triangle_coplanar_containment() {
    let outer = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let inner = ExactMesh::from_i64_triangles_with_policy(
        &[1, 1, 0, 2, 1, 0, 1, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    assert_eq!(
        hypermesh::exact::certify_single_triangle_coplanar_containment(&inner, &outer),
        Some(hypermesh::exact::CoplanarSurfaceContainment::LeftInsideRight)
    );
    assert_eq!(
        hypermesh::exact::certify_single_triangle_coplanar_containment(&outer, &inner),
        Some(hypermesh::exact::CoplanarSurfaceContainment::RightInsideLeft)
    );
    let containment =
        hypermesh::exact::certify_single_triangle_coplanar_containment_report(&inner, &outer);
    assert_eq!(
        containment.status,
        hypermesh::exact::CoplanarSurfaceContainmentStatus::Certified(
            hypermesh::exact::CoplanarSurfaceContainment::LeftInsideRight
        )
    );
    assert_eq!(
        containment.triangle.as_ref().unwrap().relation,
        TriangleTriangleRelation::CoplanarOverlapping
    );
    assert_eq!(
        containment.coplanar.as_ref().unwrap().relation,
        CoplanarTriangleRelation::Overlapping
    );
    assert!(containment.all_proof_producing());

    let union = hypermesh::exact::boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(union.mesh.triangles(), outer.triangles());
    assert_eq!(union.mesh.vertices(), outer.vertices());

    let intersection = hypermesh::exact::boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(intersection.mesh.triangles(), inner.triangles());
    assert_eq!(intersection.mesh.vertices(), inner.vertices());

    let empty_difference = hypermesh::exact::boolean_exact(
        &inner,
        &outer,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(empty_difference.mesh.triangles().is_empty());
    assert!(empty_difference.mesh.vertices().is_empty());

    let unsupported_holed_difference = hypermesh::exact::boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap_err();
    assert!(
        unsupported_holed_difference
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::UnsupportedExactOperation)
    );

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarSurfaceContainment
    );
    assert_eq!(preflight.retained_face_pairs, 0);
    assert!(preflight.blocker.is_none());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_surface_containment_report_retains_rejection_state() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 1, 1, 0, 1, 0, 1, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let report =
        hypermesh::exact::certify_single_triangle_coplanar_containment_report(&left, &right);
    assert_eq!(
        report.status,
        hypermesh::exact::CoplanarSurfaceContainmentStatus::NotCoplanar
    );
    assert!(report.triangle.is_some());
    assert!(report.coplanar.is_none());
    assert!(report.all_proof_producing());

    let open = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0],
        &[0, 1, 2, 1, 3, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let not_single =
        hypermesh::exact::certify_single_triangle_coplanar_containment_report(&open, &right);
    assert_eq!(
        not_single.status,
        hypermesh::exact::CoplanarSurfaceContainmentStatus::NotSingleTriangle
    );
    assert!(not_single.triangle.is_none());
    assert!(not_single.coplanar.is_none());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_handle_open_surface_disjoint_with_overlapping_bounds() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 2, 0, 4, 2, 0, 2, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    assert_ne!(
        left.bounds()
            .mesh
            .as_ref()
            .unwrap()
            .classify_intersection(right.bounds().mesh.as_ref().unwrap())
            .value(),
        Some(hypermesh::exact::AabbIntersectionKind::Disjoint)
    );

    let retained = classify_mesh_face_pairs(&left, &right).unwrap();
    assert!(retained.is_empty());

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedOpenSurfaceDisjoint
    );
    assert!(preflight.blocker.is_none());

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

    let difference = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(difference.mesh.triangles(), left.triangles());
    assert_eq!(difference.mesh.vertices(), left.vertices());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_intersect_partially_overlapping_coplanar_triangles() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, 1, 0, 5, 1, 0, 1, 5, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let clipped = hypermesh::exact::intersect_single_triangle_coplanar_surfaces(&left, &right)
        .expect("partial coplanar overlap should produce a positive-area polygon");
    assert_eq!(clipped.polygon.len(), 3);
    assert_eq!(clipped.mesh.triangles().len(), 1);
    clipped.validate().unwrap();

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection
    );
    assert!(preflight.blocker.is_none());

    let intersection = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(intersection.mesh.triangles().len(), 1);
    assert_eq!(intersection.mesh.vertices().len(), 3);

    let unsupported_union = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap_err();
    assert!(
        unsupported_union
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::UnsupportedExactOperation)
    );

    let union_preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        union_preflight.support,
        hypermesh::exact::ExactBooleanSupport::RequiresPlanarArrangement
    );
    let blocker = union_preflight.blocker.as_ref().unwrap();
    assert_eq!(
        blocker.kind,
        hypermesh::exact::ExactBooleanBlockerKind::NeedsPlanarArrangement
    );
    assert_eq!(blocker.coplanar_overlapping_pairs, 1);
    assert_eq!(blocker.candidate_pairs, 0);

    let difference_preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    assert_eq!(
        difference_preflight.support,
        hypermesh::exact::ExactBooleanSupport::RequiresPlanarArrangement
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_surface_outputs_validate_public_artifacts() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, 1, 0, 5, 1, 0, 1, 5, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let mut clipped = hypermesh::exact::intersect_single_triangle_coplanar_surfaces(&left, &right)
        .expect("partial coplanar overlap should produce a positive-area polygon");
    clipped.validate().unwrap();
    clipped.polygon[1] = clipped.polygon[0].clone();
    let duplicate = clipped.validate().unwrap_err();
    assert!(
        duplicate
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::DegenerateTriangle)
    );

    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 1, 0, 1, 0, 0, 1, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let mut union = hypermesh::exact::union_single_triangle_coplanar_surfaces(&left, &right)
        .expect("diagonal-adjacent triangles should union into a square");
    union.validate().unwrap();
    union.polygon.push(p3(2, 2, 0));
    let drift = union.validate().unwrap_err();
    assert!(
        drift
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::DegenerateTriangle)
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_triangle_union_materializes_convex_edge_touching_square() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 1, 0, 1, 0, 0, 1, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let union = hypermesh::exact::union_single_triangle_coplanar_surfaces(&left, &right)
        .expect("diagonal-adjacent triangles should union into a square");
    assert_eq!(union.polygon.len(), 4);
    assert_eq!(union.mesh.triangles().len(), 2);
    union.validate().unwrap();

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(result.mesh.vertices().len(), 4);
    assert_eq!(result.mesh.triangles().len(), 2);

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarSurfaceConvexUnion
    );
    assert!(preflight.blocker.is_none());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_triangle_union_rejects_nonconvex_arrangement_gap() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 3, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 4, 3, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    assert!(hypermesh::exact::union_single_triangle_coplanar_surfaces(&left, &right).is_none());

    let union_preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        union_preflight.support,
        hypermesh::exact::ExactBooleanSupport::RequiresPlanarArrangement
    );
    let blocker = union_preflight.blocker.as_ref().unwrap();
    assert_eq!(
        blocker.kind,
        hypermesh::exact::ExactBooleanBlockerKind::NeedsPlanarArrangement
    );
    assert_eq!(blocker.coplanar_overlapping_pairs, 1);
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_triangle_difference_materializes_one_corner_cut() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[-1, -1, 0, 2, -1, 0, -1, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let difference = hypermesh::exact::difference_single_triangle_coplanar_surfaces(&left, &right)
        .expect("one strict corner cut should produce a convex difference polygon");
    assert_eq!(difference.polygon.len(), 4);
    assert_eq!(difference.mesh.vertices().len(), 4);
    assert_eq!(difference.mesh.triangles().len(), 2);
    difference.validate().unwrap();

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(result.mesh.vertices().len(), 4);
    assert_eq!(result.mesh.triangles().len(), 2);

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarSurfaceCornerDifference
    );
    assert!(preflight.blocker.is_none());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_triangle_difference_materializes_remaining_corner_cut() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[-3, 1, 0, 8, -1, 0, -3, 6, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let difference = hypermesh::exact::difference_single_triangle_coplanar_surfaces(&left, &right)
        .expect("one strict remaining corner should produce a convex difference triangle");
    assert_eq!(difference.polygon.len(), 3);
    assert_eq!(difference.mesh.vertices().len(), 3);
    assert_eq!(difference.mesh.triangles().len(), 1);
    difference.validate().unwrap();

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(result.mesh.vertices().len(), 3);
    assert_eq!(result.mesh.triangles().len(), 1);

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarSurfaceCornerDifference
    );
    assert!(preflight.blocker.is_none());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_triangle_difference_rejects_contained_hole_case() {
    let outer = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let inner = ExactMesh::from_i64_triangles_with_policy(
        &[1, 1, 0, 2, 1, 0, 1, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    assert!(
        hypermesh::exact::difference_single_triangle_coplanar_surfaces(&outer, &inner).is_none()
    );
    let preflight = hypermesh::exact::preflight_boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::RequiresPlanarArrangement
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_triangle_intersection_handles_quadrilateral_clip() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, -1, 0, 5, 3, 0, 1, 3, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let clipped = hypermesh::exact::intersect_single_triangle_coplanar_surfaces(&left, &right)
        .expect("quadrilateral overlap should produce a positive-area polygon");
    assert_eq!(clipped.polygon.len(), 4);
    assert_eq!(clipped.mesh.triangles().len(), 2);
    assert_eq!(clipped.mesh.vertices().len(), 4);
    clipped.validate().unwrap();

    let intersection = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(intersection.mesh.triangles().len(), 2);
    assert_eq!(intersection.mesh.vertices().len(), 4);
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_triangle_intersection_simplifies_edge_aligned_overlap() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 2, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let clipped = hypermesh::exact::intersect_single_triangle_coplanar_surfaces(&left, &right)
        .expect("edge-aligned overlap should produce the smaller triangle");
    assert_eq!(clipped.polygon.len(), 3);
    assert_eq!(clipped.mesh.triangles().len(), 1);
    assert_eq!(clipped.mesh.vertices().len(), 3);
    clipped.validate().unwrap();
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
fn exact_named_booleans_handle_reversed_same_indexed_surface() {
    let vertices = [
        0, 0, 0, //
        1, 0, 0, //
        0, 1, 0, //
        0, 0, 1,
    ];
    let mesh =
        ExactMesh::from_i64_triangles(&vertices, &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3]).unwrap();
    let reversed =
        ExactMesh::from_i64_triangles(&vertices, &[0, 1, 2, 0, 3, 1, 1, 3, 2, 2, 3, 0]).unwrap();

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &mesh,
        &reversed,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedSameSurface
    );
    assert_eq!(preflight.retained_events, 0);
    let report = hypermesh::exact::certify_same_surface_report(&mesh, &reversed);
    assert!(report.is_certified());
    assert_eq!(report.left_to_right, vec![0, 1, 2, 3]);
    assert_eq!(report.right_to_left, vec![0, 1, 2, 3]);
    assert_eq!(report.left_triangles, report.right_triangles);
    assert!(report.all_proof_producing());

    let union = hypermesh::exact::boolean_exact(
        &mesh,
        &reversed,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(union.mesh.triangles(), mesh.triangles());

    let intersection = hypermesh::exact::boolean_exact(
        &mesh,
        &reversed,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(intersection.mesh.triangles(), mesh.triangles());

    let difference = hypermesh::exact::boolean_exact(
        &mesh,
        &reversed,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert!(difference.mesh.triangles().is_empty());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_handle_reindexed_same_surface() {
    let left_vertices = [
        0, 0, 0, //
        1, 0, 0, //
        0, 1, 0, //
        0, 0, 1,
    ];
    let right_vertices = [
        0, 0, 1, //
        0, 0, 0, //
        0, 1, 0, //
        1, 0, 0,
    ];
    let left = ExactMesh::from_i64_triangles(&left_vertices, &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3])
        .unwrap();
    let right =
        ExactMesh::from_i64_triangles(&right_vertices, &[1, 3, 2, 1, 0, 3, 3, 0, 2, 2, 0, 1])
            .unwrap();

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedSameSurface
    );
    let report = hypermesh::exact::certify_same_surface_report(&left, &right);
    assert!(report.is_certified());
    assert_eq!(report.left_to_right, vec![1, 3, 2, 0]);
    assert_eq!(report.right_to_left, vec![3, 0, 2, 1]);
    assert_eq!(report.left_triangles, report.right_triangles);
    assert!(report.all_proof_producing());

    let intersection = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(intersection.mesh.vertices(), left.vertices());
    assert_eq!(intersection.mesh.triangles(), left.triangles());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_same_surface_report_retains_rejection_state() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let shifted = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 1, 1, 0, 1, 0, 1, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let shifted_report = hypermesh::exact::certify_same_surface_report(&left, &shifted);
    assert_eq!(
        shifted_report.status,
        hypermesh::exact::ExactSameSurfaceStatus::VertexCoordinateMismatch
    );
    assert!(!shifted_report.predicates.is_empty());
    assert!(shifted_report.all_proof_producing());

    let different_topology = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0],
        &[0, 1, 2, 1, 3, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let count_report = hypermesh::exact::certify_same_surface_report(&left, &different_topology);
    assert_eq!(
        count_report.status,
        hypermesh::exact::ExactSameSurfaceStatus::VertexCountMismatch
    );
    assert!(count_report.predicates.is_empty());
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
fn exact_convex_solid_facts_classify_points_and_vertex_sets() {
    let outer = ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, //
            10, 0, 0, //
            0, 10, 0, //
            0, 0, 10,
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();
    let inner = ExactMesh::from_i64_triangles(
        &[
            1, 1, 1, //
            2, 1, 1, //
            1, 2, 1, //
            1, 1, 2,
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();

    let facts = certify_convex_solid(&outer);
    assert!(facts.is_certified_convex());
    assert!(facts.all_proof_producing());
    assert_eq!(
        classify_point_against_convex_solid(&p3(1, 1, 1), &outer),
        hypermesh::exact::ConvexSolidPointRelation::Inside
    );
    assert_eq!(
        classify_point_against_convex_solid(&p3(0, 0, 0), &outer),
        hypermesh::exact::ConvexSolidPointRelation::Boundary
    );
    assert_eq!(
        classify_point_against_convex_solid(&p3(11, 1, 1), &outer),
        hypermesh::exact::ConvexSolidPointRelation::Outside
    );
    let inside = classify_point_against_convex_solid_report(&p3(1, 1, 1), &outer);
    assert_eq!(
        inside.relation,
        hypermesh::exact::ConvexSolidPointRelation::Inside
    );
    assert_eq!(inside.predicates.len(), outer.triangles().len());
    assert!(inside.all_proof_producing());

    let outside = classify_point_against_convex_solid_report(&p3(11, 1, 1), &outer);
    assert_eq!(
        outside.relation,
        hypermesh::exact::ConvexSolidPointRelation::Outside
    );
    assert!(!outside.predicates.is_empty());
    assert!(outside.predicates.len() <= outer.triangles().len());
    assert!(outside.all_proof_producing());

    assert_eq!(
        classify_mesh_vertices_against_convex_solid(&inner, &outer),
        hypermesh::exact::ConvexSolidMeshRelation::StrictlyInside
    );
    let containment = classify_mesh_vertices_against_convex_solid_report(&inner, &outer);
    assert_eq!(
        containment.relation,
        hypermesh::exact::ConvexSolidMeshRelation::StrictlyInside
    );
    assert!(containment.solid_facts.is_certified_convex());
    assert_eq!(containment.vertices.len(), inner.vertices().len());
    assert!(containment.all_proof_producing());

    assert_eq!(
        classify_mesh_vertices_against_convex_solid(&outer, &inner),
        hypermesh::exact::ConvexSolidMeshRelation::Outside
    );
    let separated = classify_mesh_vertices_against_convex_solid_report(&outer, &inner);
    assert_eq!(
        separated.relation,
        hypermesh::exact::ConvexSolidMeshRelation::Outside
    );
    assert_eq!(separated.vertices.len(), outer.vertices().len());
    assert!(separated.all_proof_producing());
}

#[test]
fn exact_convex_solid_reports_retain_not_certified_state() {
    let open = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let subject = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 1, 1, 0, 1, 0, 1, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let point = classify_point_against_convex_solid_report(&p3(0, 0, 0), &open);
    assert_eq!(
        point.relation,
        hypermesh::exact::ConvexSolidPointRelation::NotCertifiedConvex
    );
    assert!(point.predicates.is_empty());

    let mesh = classify_mesh_vertices_against_convex_solid_report(&subject, &open);
    assert_eq!(
        mesh.relation,
        hypermesh::exact::ConvexSolidMeshRelation::NotCertifiedConvex
    );
    assert_eq!(
        mesh.solid_facts.orientation,
        hypermesh::exact::ClosedMeshOrientation::NotClosed
    );
    assert!(mesh.vertices.is_empty());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_handle_certified_convex_containment() {
    let outer = ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, //
            10, 0, 0, //
            0, 10, 0, //
            0, 0, 10,
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();
    let inner = ExactMesh::from_i64_triangles(
        &[
            1, 1, 1, //
            2, 1, 1, //
            1, 2, 1, //
            1, 1, 2,
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();

    let union = hypermesh::exact::boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(union.mesh.triangles(), outer.triangles());
    assert_eq!(
        union.mesh.provenance().source.label,
        "exact convex containment union keeps outer left"
    );
    let preflight = hypermesh::exact::preflight_boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedConvexContainment
    );
    assert_eq!(preflight.retained_events, 0);

    let intersection = hypermesh::exact::boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(intersection.mesh.triangles(), inner.triangles());

    let difference = hypermesh::exact::boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(difference.mesh.triangles().len(), 8);
    assert!(difference.mesh.facts().mesh.closed_manifold);

    let empty_difference = hypermesh::exact::boolean_exact(
        &inner,
        &outer,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert!(empty_difference.mesh.triangles().is_empty());
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
