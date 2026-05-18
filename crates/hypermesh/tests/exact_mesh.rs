#![cfg(feature = "exact")]

use hyperlimit::{PlaneSide, Point3, compare_reals};
#[cfg(feature = "exact-triangulation")]
use hypermesh::exact::CoplanarProjection;
#[cfg(feature = "exact-triangulation")]
use hypermesh::exact::checked_classify_face_regions_against_opposite_planes;
#[cfg(not(feature = "exact-triangulation"))]
use hypermesh::exact::classify_face_regions_against_opposite_planes;
use hypermesh::exact::{
    AabbIntersectionKind, ApproximationPolicy, ConstructionProvenance, CoplanarTriangleRelation,
    DiagnosticKind, EdgeSplit, EdgeSplitPoint, ExactAabb3, ExactEdgeSplitPlan,
    ExactFaceSplitGeometryPlan, ExactFaceSplitPlan, ExactGraphVertex, ExactGraphVertexPlan,
    ExactGraphVertexUse, ExactIntersectionGraph, ExactMesh, ExactReal, ExactSplitTopologyPlan,
    FacePairEvents, FaceRegionBoundary, FaceSplitBoundaryChain, FaceSplitBoundaryNode,
    FaceSplitEdge, FaceSplitGeometry, FaceSplitPlan, IntersectionEvent, MeshFacePairRelation,
    MeshSide, MeshSource, SegmentPlaneRelation, Severity, SourceProvenance, SplitEdgeChain,
    SplitEdgeNode, SplitPlanDiagnosticKind, TrianglePlaneRelation, TriangleTriangleRelation,
    ValidationPolicy, VertexLinkKind, build_intersection_graph, certify_convex_solid,
    classify_coplanar_triangles, classify_mesh_face_pair, classify_mesh_face_pairs,
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
    mesh.facts().validate().unwrap();
    mesh.validate_retained_state().unwrap();
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
    mesh.bounds()
        .validate(mesh.vertices().len(), mesh.triangles().len())
        .unwrap();
    mesh.facts().validate().unwrap();
    mesh.validate_retained_state().unwrap();
    assert_eq!(mesh.provenance().source.label, "flat i64 triangle mesh");
}

#[test]
fn exact_provenance_validation_rejects_inconsistent_artifacts() {
    let empty_label = ConstructionProvenance::new(SourceProvenance {
        source: MeshSource::Exact,
        label: String::new(),
        approximation: ApproximationPolicy::ExactOnly,
    });
    assert_eq!(
        empty_label.validate().unwrap_err(),
        hypermesh::exact::ConstructionProvenanceValidationError::EmptySourceLabel
    );

    let lossy_as_exact = ConstructionProvenance::new(SourceProvenance {
        source: MeshSource::LossyF64,
        label: "bad source policy".to_string(),
        approximation: ApproximationPolicy::ExactOnly,
    });
    assert_eq!(
        lossy_as_exact.validate().unwrap_err(),
        hypermesh::exact::ConstructionProvenanceValidationError::SourceApproximationMismatch
    );

    let exact_as_edge = ConstructionProvenance::new(SourceProvenance {
        source: MeshSource::Exact,
        label: "bad exact policy".to_string(),
        approximation: ApproximationPolicy::EdgeOnly,
    });
    assert_eq!(
        exact_as_edge.validate().unwrap_err(),
        hypermesh::exact::ConstructionProvenanceValidationError::SourceApproximationMismatch
    );
}

#[test]
fn exact_mesh_fact_validation_rejects_inconsistent_artifacts() {
    let (pos, idx) = tetrahedron();
    let mesh = ExactMesh::from_f64_triangles(&pos, &idx).unwrap();

    let mut bad_summary = mesh.facts().clone();
    bad_summary.mesh.edge_count += 1;
    assert_eq!(
        bad_summary.validate().unwrap_err(),
        hypermesh::exact::MeshFactsValidationError::SummaryLengthMismatch {
            field: "edge_count",
            expected: 6,
            actual: 7,
        }
    );

    let mut bad_face = mesh.facts().clone();
    bad_face.faces[0].oriented.directed_edges[0] = [1, 0];
    assert_eq!(
        bad_face.validate().unwrap_err(),
        hypermesh::exact::MeshFactsValidationError::FaceDirectedEdgesMismatch {
            face: 0,
            expected: [[0, 2], [2, 1], [1, 0]],
            actual: [[1, 0], [2, 1], [1, 0]],
        }
    );

    let mut bad_edge = mesh.facts().clone();
    bad_edge.edges[0].directed_uses = [2, 0];
    assert_eq!(
        bad_edge.validate().unwrap_err(),
        hypermesh::exact::MeshFactsValidationError::EdgeUseMismatch {
            edge: [0, 1],
            expected_directed_uses: [1, 1],
            actual_directed_uses: [2, 0],
            expected_incident_faces: 2,
            actual_incident_faces: 2,
        }
    );
}

#[test]
fn exact_bounds_validation_rejects_inconsistent_artifacts() {
    let inverted = ExactAabb3 {
        min: p3(1, 0, 0),
        max: p3(0, 0, 0),
    };
    assert_eq!(
        inverted.validate().unwrap_err(),
        hypermesh::exact::BoundsValidationError::InvertedAxis
    );

    let missing_mesh = hypermesh::exact::MeshBounds {
        mesh: None,
        faces: Vec::new(),
    };
    assert_eq!(
        missing_mesh.validate(1, 0).unwrap_err(),
        hypermesh::exact::BoundsValidationError::MissingMeshBounds
    );
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
    classification.validate().unwrap();
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
    retained.validate().unwrap();
    predicate.validate().unwrap();
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
    coplanar.validate().unwrap();
    straddling.validate().unwrap();
    assert!(coplanar.all_proof_producing());
    assert!(straddling.all_proof_producing());
}

#[test]
fn exact_narrow_phase_validation_rejects_inconsistent_plane_artifacts() {
    let classification = hypermesh::exact::TrianglePlaneClassification {
        relation: TrianglePlaneRelation::Coplanar,
        vertex_sides: [
            Some(PlaneSide::Above),
            Some(PlaneSide::Above),
            Some(PlaneSide::Above),
        ],
        predicates: Vec::new(),
    };

    assert_eq!(
        classification.validate().unwrap_err(),
        hypermesh::exact::TrianglePlaneValidationError::RelationMismatch
    );
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
    event.validate().unwrap();
    assert_eq!(
        event.endpoint_sides,
        [Some(PlaneSide::Above), Some(PlaneSide::Below)]
    );
    assert!(event.all_proof_producing());
    assert_real_eq(event.parameter.as_ref().unwrap(), &half());
    let ratio = event.parameter_ratio.as_ref().unwrap();
    assert_real_eq(
        &(&ratio.numerator / &ratio.denominator).expect("nonzero crossing denominator"),
        &half(),
    );
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
    retained.validate().unwrap();
    predicate.validate().unwrap();
    assert!(retained.predicates.is_empty());
    assert_real_eq(retained.parameter.as_ref().unwrap(), &half());
    let ratio = retained.parameter_ratio.as_ref().unwrap();
    assert_real_eq(
        &(&ratio.numerator / &ratio.denominator).expect("nonzero retained crossing denominator"),
        &half(),
    );
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
    endpoint.validate().unwrap();
    assert_eq!(endpoint.endpoint_on_plane, Some(0));
    assert_real_eq(endpoint.parameter.as_ref().unwrap(), &ExactReal::from(0));

    let disjoint = intersect_segment_with_face_plane(&points, [0, 1, 2], [5, 6]);
    assert_eq!(disjoint.relation, SegmentPlaneRelation::Disjoint);
    disjoint.validate().unwrap();
    assert!(disjoint.point.is_none());

    let coplanar = intersect_segment_with_face_plane(&points, [0, 1, 2], [7, 8]);
    assert_eq!(coplanar.relation, SegmentPlaneRelation::Coplanar);
    coplanar.validate().unwrap();
    assert!(coplanar.parameter.is_none());
}

#[test]
fn exact_segment_plane_validation_rejects_inconsistent_artifacts() {
    let invalid_crossing = hypermesh::exact::SegmentPlaneIntersection {
        relation: SegmentPlaneRelation::ProperCrossing,
        point: Some(p3(0, 0, 0)),
        parameter: Some(half()),
        parameter_ratio: Some(hypermesh::exact::SegmentPlaneParameterRatio {
            numerator: ExactReal::from(1),
            denominator: ExactReal::from(2),
        }),
        endpoint_on_plane: None,
        endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Above)],
        predicates: Vec::new(),
        construction_failure: None,
    };
    assert_eq!(
        invalid_crossing.validate().unwrap_err(),
        hypermesh::exact::SegmentPlaneValidationError::ProperCrossingSideFactsMismatch
    );

    let invalid_disjoint = hypermesh::exact::SegmentPlaneIntersection {
        relation: SegmentPlaneRelation::Disjoint,
        point: Some(p3(0, 0, 0)),
        parameter: None,
        parameter_ratio: None,
        endpoint_on_plane: None,
        endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Above)],
        predicates: Vec::new(),
        construction_failure: None,
    };
    assert_eq!(
        invalid_disjoint.validate().unwrap_err(),
        hypermesh::exact::SegmentPlaneValidationError::UnexpectedConstruction
    );

    let out_of_range_crossing = hypermesh::exact::SegmentPlaneIntersection {
        relation: SegmentPlaneRelation::ProperCrossing,
        point: Some(p3(0, 0, 0)),
        parameter: Some(ExactReal::from(2)),
        parameter_ratio: Some(hypermesh::exact::SegmentPlaneParameterRatio {
            numerator: ExactReal::from(2),
            denominator: ExactReal::from(1),
        }),
        endpoint_on_plane: None,
        endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)],
        predicates: Vec::new(),
        construction_failure: None,
    };
    assert_eq!(
        out_of_range_crossing.validate().unwrap_err(),
        hypermesh::exact::SegmentPlaneValidationError::ProperCrossingParameterOutOfRange
    );

    let endpoint_that_is_really_coplanar = hypermesh::exact::SegmentPlaneIntersection {
        relation: SegmentPlaneRelation::EndpointOnPlane,
        point: Some(p3(0, 0, 0)),
        parameter: Some(ExactReal::from(0)),
        parameter_ratio: None,
        endpoint_on_plane: Some(0),
        endpoint_sides: [Some(PlaneSide::On), Some(PlaneSide::On)],
        predicates: Vec::new(),
        construction_failure: None,
    };
    assert_eq!(
        endpoint_that_is_really_coplanar.validate().unwrap_err(),
        hypermesh::exact::SegmentPlaneValidationError::EndpointSideFactsMismatch
    );

    let mismatched_ratio = hypermesh::exact::SegmentPlaneIntersection {
        relation: SegmentPlaneRelation::ProperCrossing,
        point: Some(p3(0, 0, 0)),
        parameter: Some(half()),
        parameter_ratio: Some(hypermesh::exact::SegmentPlaneParameterRatio {
            numerator: ExactReal::from(2),
            denominator: ExactReal::from(3),
        }),
        endpoint_on_plane: None,
        endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)],
        predicates: Vec::new(),
        construction_failure: None,
    };
    assert_eq!(
        mismatched_ratio.validate().unwrap_err(),
        hypermesh::exact::SegmentPlaneValidationError::ProperCrossingRatioMismatch
    );

    let failed_without_reason = hypermesh::exact::SegmentPlaneIntersection {
        relation: SegmentPlaneRelation::ConstructionFailed,
        point: None,
        parameter: None,
        parameter_ratio: None,
        endpoint_on_plane: None,
        endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)],
        predicates: Vec::new(),
        construction_failure: None,
    };
    assert_eq!(
        failed_without_reason.validate().unwrap_err(),
        hypermesh::exact::SegmentPlaneValidationError::MissingConstructionFailureReason
    );

    let failed_with_reason = hypermesh::exact::SegmentPlaneIntersection {
        relation: SegmentPlaneRelation::ConstructionFailed,
        point: None,
        parameter: None,
        parameter_ratio: None,
        endpoint_on_plane: None,
        endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)],
        predicates: Vec::new(),
        construction_failure: Some(
            hypermesh::exact::SegmentPlaneConstructionFailure::ZeroDenominator,
        ),
    };
    failed_with_reason.validate().unwrap();

    let disjoint_with_failure_reason = hypermesh::exact::SegmentPlaneIntersection {
        relation: SegmentPlaneRelation::Disjoint,
        point: None,
        parameter: None,
        parameter_ratio: None,
        endpoint_on_plane: None,
        endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Above)],
        predicates: Vec::new(),
        construction_failure: Some(
            hypermesh::exact::SegmentPlaneConstructionFailure::ParameterDivisionFailed,
        ),
    };
    assert_eq!(
        disjoint_with_failure_reason.validate().unwrap_err(),
        hypermesh::exact::SegmentPlaneValidationError::UnexpectedConstructionFailureReason
    );
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
    classification.validate().unwrap();
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
    classification.validate().unwrap();
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

    let disjoint = classify_coplanar_triangles(&disjoint_points, [0, 1, 2], [3, 4, 5]);
    let touching = classify_coplanar_triangles(&touching_points, [0, 1, 2], [3, 4, 5]);
    let overlapping = classify_coplanar_triangles(&overlapping_points, [0, 1, 2], [3, 4, 5]);

    assert_eq!(disjoint.relation, CoplanarTriangleRelation::Disjoint);
    assert_eq!(touching.relation, CoplanarTriangleRelation::Touching);
    assert_eq!(overlapping.relation, CoplanarTriangleRelation::Overlapping);
    disjoint.validate().unwrap();
    touching.validate().unwrap();
    overlapping.validate().unwrap();
}

#[test]
fn exact_coplanar_triangle_validation_rejects_inconsistent_artifacts() {
    let no_projection = hypermesh::exact::CoplanarTriangleClassification {
        projection: None,
        relation: CoplanarTriangleRelation::Overlapping,
        edge_intersections: Vec::new(),
        right_vertices_in_left: [None, None, None],
        left_vertices_in_right: [None, None, None],
        predicates: Vec::new(),
    };
    assert_eq!(
        no_projection.validate().unwrap_err(),
        hypermesh::exact::CoplanarTriangleValidationError::DecidedRelationWithoutProjection
    );

    let missing_edges = hypermesh::exact::CoplanarTriangleClassification {
        projection: Some(hypermesh::exact::CoplanarProjection::Xy),
        relation: CoplanarTriangleRelation::Disjoint,
        edge_intersections: Vec::new(),
        right_vertices_in_left: [
            Some(hyperlimit::TriangleLocation::Outside),
            Some(hyperlimit::TriangleLocation::Outside),
            Some(hyperlimit::TriangleLocation::Outside),
        ],
        left_vertices_in_right: [
            Some(hyperlimit::TriangleLocation::Outside),
            Some(hyperlimit::TriangleLocation::Outside),
            Some(hyperlimit::TriangleLocation::Outside),
        ],
        predicates: Vec::new(),
    };
    assert_eq!(
        missing_edges.validate().unwrap_err(),
        hypermesh::exact::CoplanarTriangleValidationError::MissingEdgeIntersections
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
    classification.validate().unwrap();
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
fn exact_triangle_triangle_validation_rejects_inconsistent_artifacts() {
    let points = [
        p3(0, 0, 0),
        p3(2, 0, 0),
        p3(0, 2, 0),
        p3(0, 0, -1),
        p3(2, 0, 1),
        p3(0, 2, 1),
    ];
    let mut candidate = classify_triangle_triangle(&points, [0, 1, 2], [3, 4, 5]);
    candidate.right_edge_events.clear();

    assert_eq!(
        candidate.validate().unwrap_err(),
        hypermesh::exact::TriangleTriangleValidationError::CandidateEdgeEventCountMismatch
    );

    let mut separated = classify_triangle_triangle(
        &[
            p3(0, 0, 0),
            p3(1, 0, 0),
            p3(0, 1, 0),
            p3(0, 0, 2),
            p3(1, 0, 2),
            p3(0, 1, 2),
        ],
        [0, 1, 2],
        [3, 4, 5],
    );
    separated.relation = TriangleTriangleRelation::Candidate;

    assert_eq!(
        separated.validate().unwrap_err(),
        hypermesh::exact::TriangleTriangleValidationError::PlaneRelationMismatch
    );
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
    classification.validate().unwrap();
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
    classification.validate().unwrap();
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
    classification.validate().unwrap();
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
    classification.validate().unwrap();
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
                    parameter_ratio: hypermesh::exact::SegmentPlaneParameterRatio {
                        numerator: ExactReal::from(1),
                        denominator: ExactReal::from(2),
                    },
                    point: p3(0, 0, 0),
                    endpoint_sides: [None, Some(PlaneSide::Below)],
                },
                EdgeSplitPoint {
                    face_pair: [0, 0],
                    plane_face: 0,
                    parameter: half(),
                    parameter_ratio: hypermesh::exact::SegmentPlaneParameterRatio {
                        numerator: ExactReal::from(2),
                        denominator: ExactReal::from(3),
                    },
                    point: p3(0, 0, 0),
                    endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Above)],
                },
            ],
        }],
        unknown_orderings: 1,
    };

    let report = split_plan.validate();

    assert!(!report.is_valid());
    report.validate().unwrap();
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::MissingEndpointSideFacts
    }));
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::NonCrossingEndpointSideFacts
    }));
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::InvalidConstructionRatio
    }));
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == SplitPlanDiagnosticKind::UnknownOrdering)
    );
}

#[test]
fn exact_split_plan_report_validation_rejects_malformed_diagnostics() {
    let empty_message = hypermesh::exact::SplitPlanValidationReport {
        diagnostics: vec![hypermesh::exact::SplitPlanDiagnostic {
            kind: SplitPlanDiagnosticKind::UnknownOrdering,
            message: "   ".to_string(),
            side: None,
            face: None,
            edge: None,
            graph_vertex: None,
        }],
    };
    assert_eq!(
        empty_message.validate().unwrap_err(),
        hypermesh::exact::SplitPlanReportValidationError::EmptyMessage
    );

    let missing_edge = hypermesh::exact::SplitPlanValidationReport {
        diagnostics: vec![hypermesh::exact::SplitPlanDiagnostic {
            kind: SplitPlanDiagnosticKind::WrongChainEnd,
            message: "chain end is not retained".to_string(),
            side: Some(MeshSide::Left),
            face: None,
            edge: None,
            graph_vertex: None,
        }],
    };
    assert_eq!(
        missing_edge.validate().unwrap_err(),
        hypermesh::exact::SplitPlanReportValidationError::MissingEdge
    );

    let missing_graph_vertex = hypermesh::exact::SplitPlanValidationReport {
        diagnostics: vec![hypermesh::exact::SplitPlanDiagnostic {
            kind: SplitPlanDiagnosticKind::MissingFaceSplitSourceUse,
            message: "source use missing".to_string(),
            side: Some(MeshSide::Right),
            face: Some(2),
            edge: Some([0, 1]),
            graph_vertex: None,
        }],
    };
    assert_eq!(
        missing_graph_vertex.validate().unwrap_err(),
        hypermesh::exact::SplitPlanReportValidationError::MissingGraphVertex
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
                parameter_ratio: Some(hypermesh::exact::SegmentPlaneParameterRatio {
                    numerator: ExactReal::from(1),
                    denominator: ExactReal::from(2),
                }),
                construction_failure: None,
                endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Above)],
            }],
        }],
    };

    assert_eq!(
        graph.validate().unwrap_err(),
        hypermesh::exact::IntersectionGraphValidationError::InvalidSegmentPlaneEvent
    );
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
fn exact_intersection_graph_validation_rejects_inconsistent_events() {
    let rejected_pair = FacePairEvents {
        left_face: 0,
        right_face: 0,
        relation: MeshFacePairRelation::PlaneSeparated,
        projection: None,
        events: vec![IntersectionEvent::Unknown],
    };
    assert_eq!(
        rejected_pair.validate().unwrap_err(),
        hypermesh::exact::IntersectionGraphValidationError::RejectedPairHasEvents
    );

    let missing_projection = FacePairEvents {
        left_face: 0,
        right_face: 0,
        relation: MeshFacePairRelation::CoplanarOverlapping,
        projection: None,
        events: vec![IntersectionEvent::CoplanarVertex {
            vertex_side: MeshSide::Left,
            vertex: 0,
            triangle_side: MeshSide::Right,
            triangle_face: 0,
            location: hyperlimit::TriangleLocation::Inside,
        }],
    };
    assert_eq!(
        missing_projection.validate().unwrap_err(),
        hypermesh::exact::IntersectionGraphValidationError::CoplanarPairMissingProjection
    );

    let disjoint_segment = FacePairEvents {
        left_face: 0,
        right_face: 0,
        relation: MeshFacePairRelation::Candidate,
        projection: None,
        events: vec![IntersectionEvent::SegmentPlane {
            segment_side: MeshSide::Left,
            edge: [0, 1],
            plane_side: MeshSide::Right,
            plane_face: 0,
            relation: SegmentPlaneRelation::Disjoint,
            point: None,
            parameter: None,
            parameter_ratio: None,
            construction_failure: None,
            endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Above)],
        }],
    };
    assert_eq!(
        disjoint_segment.validate().unwrap_err(),
        hypermesh::exact::IntersectionGraphValidationError::DisjointSegmentPlaneEvent
    );

    let failed_without_reason = FacePairEvents {
        left_face: 0,
        right_face: 0,
        relation: MeshFacePairRelation::Candidate,
        projection: None,
        events: vec![IntersectionEvent::SegmentPlane {
            segment_side: MeshSide::Left,
            edge: [0, 1],
            plane_side: MeshSide::Right,
            plane_face: 0,
            relation: SegmentPlaneRelation::ConstructionFailed,
            point: None,
            parameter: None,
            parameter_ratio: None,
            construction_failure: None,
            endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)],
        }],
    };
    assert_eq!(
        failed_without_reason.validate().unwrap_err(),
        hypermesh::exact::IntersectionGraphValidationError::InvalidSegmentPlaneEvent
    );

    let failed_with_reason = FacePairEvents {
        left_face: 0,
        right_face: 0,
        relation: MeshFacePairRelation::Candidate,
        projection: None,
        events: vec![IntersectionEvent::SegmentPlane {
            segment_side: MeshSide::Left,
            edge: [0, 1],
            plane_side: MeshSide::Right,
            plane_face: 0,
            relation: SegmentPlaneRelation::ConstructionFailed,
            point: None,
            parameter: None,
            parameter_ratio: None,
            construction_failure: Some(
                hypermesh::exact::SegmentPlaneConstructionFailure::ParameterDivisionFailed,
            ),
            endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)],
        }],
    };
    failed_with_reason.validate().unwrap();
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
                parameter_ratio: Some(hypermesh::exact::SegmentPlaneParameterRatio {
                    numerator: ExactReal::from(1),
                    denominator: ExactReal::from(2),
                }),
                construction_failure: None,
                endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)],
            }],
        }],
    };

    graph.validate().unwrap();
    let vertex_plan = graph.checked_graph_vertex_plan().unwrap();

    assert_eq!(vertex_plan.source_use_count(), 1);
    assert!(vertex_plan.validate().is_valid());
    assert_eq!(vertex_plan.vertices[0].uses[0].parameter, half());
    assert_eq!(
        vertex_plan.vertices[0].uses[0].endpoint_sides,
        [Some(PlaneSide::Above), Some(PlaneSide::Below)]
    );
    assert_real_eq(
        &(&vertex_plan.vertices[0].uses[0].parameter_ratio.numerator
            / &vertex_plan.vertices[0].uses[0].parameter_ratio.denominator)
            .expect("graph vertex use ratio denominator should be nonzero"),
        &half(),
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
                        parameter_ratio: hypermesh::exact::SegmentPlaneParameterRatio {
                            numerator: ExactReal::from(1),
                            denominator: ExactReal::from(2),
                        },
                        endpoint_sides: [None, Some(PlaneSide::Below)],
                    },
                    ExactGraphVertexUse {
                        side: MeshSide::Right,
                        edge: [2, 3],
                        face_pair: [0, 0],
                        plane_face: 0,
                        parameter: half(),
                        parameter_ratio: hypermesh::exact::SegmentPlaneParameterRatio {
                            numerator: ExactReal::from(2),
                            denominator: ExactReal::from(3),
                        },
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
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::InvalidConstructionRatio
    }));
}

#[test]
fn exact_mesh_face_pair_validation_rejects_inconsistent_scheduler_records() {
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
    let valid_candidate = classify_mesh_face_pair(&left, 0, &right, 0).unwrap();

    let missing_triangle = hypermesh::exact::MeshFacePairClassification {
        left_face: 0,
        right_face: 0,
        bounds: valid_candidate.bounds.clone(),
        triangle: None,
        relation: MeshFacePairRelation::Candidate,
    };
    assert_eq!(
        missing_triangle.validate().unwrap_err(),
        hypermesh::exact::MeshFacePairValidationError::MissingTriangleClassification
    );

    let mut candidate = valid_candidate.triangle.unwrap();
    candidate.left_edge_events.clear();
    let bad_candidate = hypermesh::exact::MeshFacePairClassification {
        left_face: 0,
        right_face: 0,
        bounds: valid_candidate.bounds,
        triangle: Some(candidate),
        relation: MeshFacePairRelation::Candidate,
    };
    assert_eq!(
        bad_candidate.validate().unwrap_err(),
        hypermesh::exact::MeshFacePairValidationError::CandidateMissingEdgeEvents
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

    graph.validate().unwrap();
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
            .all(|classification| classification.all_proof_producing()
                && classification.validate().is_ok())
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_face_region_plane_validation_rejects_inconsistent_artifacts() {
    let empty = hypermesh::exact::FaceRegionPlaneClassification {
        region_side: MeshSide::Left,
        region_face: 0,
        plane_side: MeshSide::Right,
        plane_face: 0,
        relation: hypermesh::exact::FaceRegionPlaneRelation::Coplanar,
        node_sides: Vec::new(),
        predicates: Vec::new(),
    };
    assert_eq!(
        empty.validate().unwrap_err(),
        hypermesh::exact::FaceRegionPlaneValidationError::EmptyNodeSides
    );

    let mut mismatched = hypermesh::exact::FaceRegionPlaneClassification {
        region_side: MeshSide::Left,
        region_face: 0,
        plane_side: MeshSide::Right,
        plane_face: 0,
        relation: hypermesh::exact::FaceRegionPlaneRelation::StrictlyAbove,
        node_sides: vec![Some(PlaneSide::Above), Some(PlaneSide::Below)],
        predicates: vec![
            hypermesh::exact::PredicateUse::from_certificate(
                hyperlimit::PredicateCertificate::ExactRealFact,
            ),
            hypermesh::exact::PredicateUse::from_certificate(
                hyperlimit::PredicateCertificate::ExactRealFact,
            ),
        ],
    };
    assert_eq!(
        mismatched.validate().unwrap_err(),
        hypermesh::exact::FaceRegionPlaneValidationError::RelationMismatch {
            expected: hypermesh::exact::FaceRegionPlaneRelation::Straddling,
            actual: hypermesh::exact::FaceRegionPlaneRelation::StrictlyAbove,
        }
    );

    mismatched.relation = hypermesh::exact::FaceRegionPlaneRelation::Straddling;
    mismatched.predicates.pop();
    assert_eq!(
        mismatched.validate().unwrap_err(),
        hypermesh::exact::FaceRegionPlaneValidationError::PredicateCountMismatch {
            expected: 2,
            actual: 1,
        }
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
    boolean.validate().unwrap();
    assert_eq!(
        boolean.kind,
        hypermesh::exact::ExactBooleanResultKind::SelectedRegions {
            selection: hypermesh::exact::ExactRegionSelection::KeepAll
        }
    );
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
    exact.validate().unwrap();
    assert_eq!(exact.mesh.triangles().len(), output.triangles().len());

    let mut bad_result = boolean.clone();
    bad_result.kind = hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
        shortcut: hypermesh::exact::ExactBooleanShortcutKind::BoundsDisjoint,
    };
    assert_eq!(
        bad_result.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::ShortcutResultHasAssemblyArtifacts
    );

    let mut bad_result = boolean.clone();
    bad_result.kind = hypermesh::exact::ExactBooleanResultKind::SelectedRegions {
        selection: hypermesh::exact::ExactRegionSelection::KeepLeft,
    };
    assert_eq!(
        bad_result.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::SelectedRegionAssemblyViolatesSelection
    );

    let mut bad_result = boolean.clone();
    bad_result.assembly.vertices[0].point = p3(99, 0, 0);
    assert_eq!(
        bad_result.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::InvalidAssembly
    );

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
    preflight.validate().unwrap();
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

    let refinement_report = hypermesh::exact::certify_refinement_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    refinement_report.validate().unwrap();
    assert_eq!(
        refinement_report.status,
        hypermesh::exact::ExactRefinementStatus::NotRequired
    );
    assert!(!refinement_report.graph_had_unknowns);
    assert!(refinement_report.blocker.is_none());

    let winding_report = hypermesh::exact::certify_winding_readiness_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    winding_report.validate().unwrap();
    assert!(winding_report.is_ready());
    assert_eq!(
        winding_report.status,
        hypermesh::exact::ExactWindingReadinessStatus::Ready
    );
    assert!(!winding_report.graph_had_unknowns);
    assert_eq!(winding_report.region_count, region_plan.regions.len());
    assert_eq!(
        winding_report.region_classifications.len(),
        preflight.region_classifications.len()
    );
    assert!(winding_report.all_proof_producing());
    assert_eq!(
        winding_report.blocker.kind,
        hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding
    );
    winding_report
        .blocker
        .validate_for_kind(hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding)
        .unwrap();
    let bad_winding_blocker = hypermesh::exact::ExactBooleanBlocker {
        kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding,
        candidate_pairs: 0,
        coplanar_overlapping_pairs: 1,
        coplanar_touching_pairs: 0,
        unknown_pairs: 0,
        construction_failed_events: 0,
    };
    assert_eq!(
        bad_winding_blocker
            .validate_for_kind(hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding),
        Err(hypermesh::exact::ExactReportValidationError::InvalidBlockerCounts)
    );
    let construction_failure_blocker = hypermesh::exact::ExactBooleanBlocker {
        kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsRefinement,
        candidate_pairs: 0,
        coplanar_overlapping_pairs: 0,
        coplanar_touching_pairs: 0,
        unknown_pairs: 0,
        construction_failed_events: 1,
    };
    construction_failure_blocker
        .validate_for_kind(hypermesh::exact::ExactBooleanBlockerKind::NeedsRefinement)
        .unwrap();
    assert_eq!(
        construction_failure_blocker
            .validate_for_kind(hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding),
        Err(hypermesh::exact::ExactReportValidationError::WrongBlockerKind)
    );
    let required_refinement = hypermesh::exact::ExactRefinementReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactRefinementStatus::Required,
        graph_had_unknowns: false,
        retained_face_pairs: 1,
        retained_events: 1,
        blocker: Some(construction_failure_blocker.clone()),
    };
    required_refinement.validate().unwrap();
    let missing_refinement_blocker = hypermesh::exact::ExactRefinementReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactRefinementStatus::Required,
        graph_had_unknowns: false,
        retained_face_pairs: 1,
        retained_events: 1,
        blocker: None,
    };
    assert_eq!(
        missing_refinement_blocker.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::MissingBlocker
    );
    let retained_event_count_mismatch = hypermesh::exact::ExactRefinementReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactRefinementStatus::Required,
        graph_had_unknowns: false,
        retained_face_pairs: 1,
        retained_events: 0,
        blocker: Some(construction_failure_blocker.clone()),
    };
    assert_eq!(
        retained_event_count_mismatch.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::InvalidBlockerCounts
    );
    let retained_pair_count_mismatch = hypermesh::exact::ExactRefinementReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactRefinementStatus::Required,
        graph_had_unknowns: true,
        retained_face_pairs: 0,
        retained_events: 1,
        blocker: Some(hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsRefinement,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 1,
            construction_failed_events: 0,
        }),
    };
    assert_eq!(
        retained_pair_count_mismatch.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::InvalidBlockerCounts
    );
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
    union.validate().unwrap();
    assert_eq!(
        union.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::BoundsDisjoint
        }
    );
    let mut bad_shortcut = union.clone();
    bad_shortcut.graph_had_unknowns = true;
    assert_eq!(
        bad_shortcut.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::ShortcutResultHasUnknownGraph
    );
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
    preflight.validate().unwrap();
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
    let boundary_report =
        hypermesh::exact::certify_boundary_touching_report(&left, &right).unwrap();
    boundary_report.validate().unwrap();
    assert!(boundary_report.is_certified());
    assert_eq!(
        boundary_report.status,
        hypermesh::exact::ExactBoundaryTouchingStatus::Certified
    );
    assert!(!boundary_report.graph_had_unknowns);
    assert_eq!(boundary_report.retained_face_pairs, 1);
    assert_eq!(boundary_report.blocker.coplanar_touching_pairs, 1);
    assert_eq!(boundary_report.blocker.candidate_pairs, 0);
    let winding_report = hypermesh::exact::certify_winding_readiness_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    winding_report.validate().unwrap();
    assert_eq!(
        winding_report.status,
        hypermesh::exact::ExactWindingReadinessStatus::BoundaryPolicyRequired
    );
    assert_eq!(
        winding_report.blocker.kind,
        hypermesh::exact::ExactBooleanBlockerKind::NeedsBoundaryPolicy
    );

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
    union.validate().unwrap();
    assert_eq!(
        union.kind,
        hypermesh::exact::ExactBooleanResultKind::BoundaryPolicyShortcut {
            operation: hypermesh::exact::ExactBooleanOperation::Union
        }
    );
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
    containment.validate().unwrap();
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

    let holed_difference = hypermesh::exact::boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    holed_difference.validate().unwrap();
    assert_eq!(
        holed_difference.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarSurfaceHoledDifference
        }
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
    report.validate().unwrap();
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
    not_single.validate().unwrap();
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
    let report = hypermesh::exact::certify_open_surface_disjoint_report(&left, &right).unwrap();
    report.validate().unwrap();
    assert!(report.is_certified());
    assert_eq!(
        report.status,
        hypermesh::exact::ExactOpenSurfaceDisjointStatus::Certified
    );
    assert!(report.left_open_surface);
    assert!(report.right_open_surface);
    assert!(!report.graph_had_unknowns);
    assert_eq!(report.retained_face_pairs, 0);
    assert_eq!(report.retained_events, 0);

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
    assert_eq!(
        union.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::OpenSurfaceDisjoint
        }
    );
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
fn exact_graph_shortcut_reports_retain_rejection_state() {
    let closed = ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, //
            1, 0, 0, //
            0, 1, 0, //
            0, 0, 1,
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();
    let open = ExactMesh::from_i64_triangles_with_policy(
        &[10, 0, 0, 11, 0, 0, 10, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_report =
        hypermesh::exact::certify_open_surface_disjoint_report(&closed, &open).unwrap();
    assert_eq!(
        open_report.status,
        hypermesh::exact::ExactOpenSurfaceDisjointStatus::NotOpenSurface
    );
    assert!(!open_report.left_open_surface);
    assert!(open_report.right_open_surface);

    let touching = ExactMesh::from_i64_triangles_with_policy(
        &[1, 0, 0, 2, 0, 0, 1, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let boundary_report =
        hypermesh::exact::certify_boundary_touching_report(&open, &touching).unwrap();
    assert_eq!(
        boundary_report.status,
        hypermesh::exact::ExactBoundaryTouchingStatus::NotBoundaryOnly
    );
    assert!(!boundary_report.is_certified());

    let impossible_open_report = hypermesh::exact::ExactOpenSurfaceDisjointReport {
        status: hypermesh::exact::ExactOpenSurfaceDisjointStatus::GraphHasFacePairs,
        left_open_surface: true,
        right_open_surface: true,
        graph_had_unknowns: false,
        retained_face_pairs: 0,
        retained_events: 0,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding,
            candidate_pairs: 1,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
    };
    assert_eq!(
        impossible_open_report.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::InvalidBlockerCounts
    );

    let impossible_boundary_report = hypermesh::exact::ExactBoundaryTouchingReport {
        status: hypermesh::exact::ExactBoundaryTouchingStatus::Certified,
        graph_had_unknowns: false,
        retained_face_pairs: 0,
        retained_events: 0,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsBoundaryPolicy,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 1,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
    };
    assert_eq!(
        impossible_boundary_report.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::InvalidBlockerCounts
    );

    let impossible_planar_report = hypermesh::exact::ExactPlanarArrangementReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactPlanarArrangementStatus::Required,
        graph_had_unknowns: false,
        retained_face_pairs: 0,
        retained_events: 0,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsPlanarArrangement,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 1,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
        arrangement_readiness: None,
    };
    assert_eq!(
        impossible_planar_report.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::InvalidBlockerCounts
    );

    let impossible_winding_report = hypermesh::exact::ExactWindingReadinessReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactWindingReadinessStatus::GraphUnknowns,
        graph_had_unknowns: true,
        retained_face_pairs: 0,
        retained_events: 0,
        region_count: 0,
        region_classifications: Vec::new(),
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsRefinement,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 1,
            construction_failed_events: 0,
        },
        arrangement_readiness: None,
    };
    assert_eq!(
        impossible_winding_report.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::InvalidBlockerCounts
    );

    let unknown_open_status_mismatch = hypermesh::exact::ExactOpenSurfaceDisjointReport {
        status: hypermesh::exact::ExactOpenSurfaceDisjointStatus::Certified,
        left_open_surface: true,
        right_open_surface: true,
        graph_had_unknowns: true,
        retained_face_pairs: 0,
        retained_events: 0,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
    };
    assert_eq!(
        unknown_open_status_mismatch.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::GraphUnknownStatusMismatch
    );

    let unknown_boundary_status_mismatch = hypermesh::exact::ExactBoundaryTouchingReport {
        status: hypermesh::exact::ExactBoundaryTouchingStatus::Certified,
        graph_had_unknowns: true,
        retained_face_pairs: 1,
        retained_events: 0,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsBoundaryPolicy,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 1,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
    };
    assert_eq!(
        unknown_boundary_status_mismatch.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::GraphUnknownStatusMismatch
    );

    let unknown_planar_status_mismatch = hypermesh::exact::ExactPlanarArrangementReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactPlanarArrangementStatus::GraphUnknowns,
        graph_had_unknowns: false,
        retained_face_pairs: 0,
        retained_events: 0,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsPlanarArrangement,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
        arrangement_readiness: None,
    };
    assert_eq!(
        unknown_planar_status_mismatch.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::GraphUnknownStatusMismatch
    );

    let unknown_winding_status_mismatch = hypermesh::exact::ExactWindingReadinessReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactWindingReadinessStatus::Ready,
        graph_had_unknowns: true,
        retained_face_pairs: 1,
        retained_events: 0,
        region_count: 0,
        region_classifications: Vec::new(),
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding,
            candidate_pairs: 1,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
        arrangement_readiness: None,
    };
    assert_eq!(
        unknown_winding_status_mismatch.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::GraphUnknownStatusMismatch
    );
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
    assert_eq!(
        intersection.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarSurfaceIntersection
        }
    );
    assert_eq!(intersection.mesh.triangles().len(), 1);
    assert_eq!(intersection.mesh.vertices().len(), 3);

    let union = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    union.validate().unwrap();
    assert_eq!(
        union.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarSurfaceArrangementUnion
        }
    );

    let union_preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    union_preflight.validate().unwrap();
    assert_eq!(
        union_preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
    );
    assert!(union_preflight.blocker.is_none());

    let union_report = hypermesh::exact::certify_planar_arrangement_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    union_report.validate().unwrap();
    assert_eq!(
        union_report.status,
        hypermesh::exact::ExactPlanarArrangementStatus::AlreadyMaterialized
    );
    assert_eq!(union_report.retained_face_pairs, 1);

    let winding_report = hypermesh::exact::certify_winding_readiness_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    winding_report.validate().unwrap();
    assert_eq!(
        winding_report.status,
        hypermesh::exact::ExactWindingReadinessStatus::PlanarArrangementAlreadyMaterialized
    );

    let intersection_report = hypermesh::exact::certify_planar_arrangement_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    intersection_report.validate().unwrap();
    assert_eq!(
        intersection_report.status,
        hypermesh::exact::ExactPlanarArrangementStatus::AlreadyMaterialized
    );
    assert!(intersection_report.arrangement_readiness.is_some());

    let difference_preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    assert_eq!(
        difference_preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementDifference
    );
    assert!(difference_preflight.blocker.is_none());
    let difference = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    difference.validate().unwrap();
    assert_eq!(
        difference.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut:
                hypermesh::exact::ExactBooleanShortcutKind::CoplanarSurfaceArrangementDifference
        }
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
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarSurfaceConvexUnion
        }
    );
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
fn exact_coplanar_triangle_union_materializes_simple_planar_arrangement() {
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

    assert!(hypermesh::exact::union_single_triangle_coplanar_surfaces(&left, &right).is_none());
    let arrangement = hypermesh::exact::arrange_single_triangle_coplanar_union(&left, &right)
        .expect("simple single-loop triangle union should materialize");
    arrangement.validate().unwrap();
    assert!(arrangement.polygon.len() >= 4);
    assert_eq!(arrangement.mesh.vertices().len(), arrangement.polygon.len());
    assert_eq!(
        arrangement.mesh.triangles().len(),
        arrangement.polygon.len() - 2
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarSurfaceArrangementUnion
        }
    );

    let union_preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        union_preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
    );
    assert!(union_preflight.blocker.is_none());
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
fn exact_coplanar_triangle_difference_materializes_contained_hole_case() {
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
    let holed = hypermesh::exact::arrange_single_triangle_coplanar_holed_difference(&outer, &inner)
        .expect("contained triangle difference should materialize one hole");
    assert_eq!(holed.outer.len(), 3);
    assert_eq!(holed.hole.len(), 3);
    assert_eq!(holed.mesh.vertices().len(), 6);
    assert!(!holed.mesh.triangles().is_empty());
    holed.validate().unwrap();

    let result = hypermesh::exact::boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    result.validate().unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarSurfaceHoledDifference
        }
    );

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarSurfaceHoledDifference
    );
    assert!(preflight.blocker.is_none());
    let report = hypermesh::exact::certify_planar_arrangement_report(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    report.validate().unwrap();
    assert_eq!(
        report.status,
        hypermesh::exact::ExactPlanarArrangementStatus::AlreadyMaterialized
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
    report.validate().unwrap();
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
    report.validate().unwrap();
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
fn exact_named_booleans_handle_coplanar_convex_surface_retriangulation() {
    let vertices = &[0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0];
    let left = ExactMesh::from_i64_triangles_with_policy(
        vertices,
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        vertices,
        &[0, 1, 3, 1, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    assert!(!hypermesh::exact::certify_same_surface_report(&left, &right).is_certified());
    let certificate = hypermesh::exact::certify_coplanar_convex_surface_equivalence(&left, &right)
        .expect("same square with opposite diagonals should certify by exact hull/area");
    certificate.validate().unwrap();
    assert_eq!(certificate.polygon.len(), 4);
    let report = hypermesh::exact::certify_coplanar_convex_surface_report(&left, &right);
    report.validate().unwrap();
    assert!(report.is_certified());
    assert_eq!(
        report.status,
        hypermesh::exact::CoplanarConvexSurfaceReportStatus::Equivalent
    );
    assert!(report.equivalence.is_some());
    assert!(report.containment.is_none());

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceEquivalence
    );
    assert!(preflight.blocker.is_none());

    let union = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    union.validate().unwrap();
    assert_eq!(
        union.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceEquivalence
        }
    );
    assert_eq!(union.mesh.triangles(), left.triangles());

    let difference = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(difference.mesh.triangles().is_empty());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_handle_coplanar_convex_surface_containment() {
    let outer = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let inner = ExactMesh::from_i64_triangles_with_policy(
        &[1, 1, 0, 2, 1, 0, 2, 2, 0, 1, 2, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let certificate = hypermesh::exact::certify_coplanar_convex_surface_containment(&outer, &inner)
        .expect("inner square should certify inside outer square");
    certificate.validate().unwrap();
    assert_eq!(
        certificate.relation,
        hypermesh::exact::CoplanarConvexSurfaceContainment::RightInsideLeft
    );
    let report = hypermesh::exact::certify_coplanar_convex_surface_report(&outer, &inner);
    report.validate().unwrap();
    assert_eq!(
        report.status,
        hypermesh::exact::CoplanarConvexSurfaceReportStatus::Contained(
            hypermesh::exact::CoplanarConvexSurfaceContainment::RightInsideLeft
        )
    );
    assert!(report.equivalence.is_none());
    assert!(report.containment.is_some());

    let holed = hypermesh::exact::arrange_coplanar_convex_surface_holed_difference(&outer, &inner)
        .expect("outer minus inner convex sheets should materialize one hole");
    holed.validate().unwrap();
    assert_eq!(holed.outer.len(), 4);
    assert_eq!(holed.hole.len(), 4);
    assert_eq!(holed.mesh.vertices().len(), 8);

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceHoledDifference
    );

    let difference = hypermesh::exact::boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    difference.validate().unwrap();
    assert_eq!(
        difference.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut:
                hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceHoledDifference
        }
    );
    assert!(!difference.mesh.triangles().is_empty());

    let intersection = hypermesh::exact::boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(intersection.mesh.triangles(), inner.triangles());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_convex_surface_union_materializes_simple_loop() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    assert!(hypermesh::exact::certify_coplanar_convex_surface_equivalence(&left, &right).is_none());
    assert!(hypermesh::exact::certify_coplanar_convex_surface_containment(&left, &right).is_none());
    let union = hypermesh::exact::arrange_coplanar_convex_surface_union(&left, &right)
        .expect("overlapping convex sheets should materialize one simple union loop");
    union.validate().unwrap();
    assert_eq!(union.polygon.len(), 8);
    assert!(!union.mesh.triangles().is_empty());

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    preflight.validate().unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementUnion
    );
    assert!(preflight.blocker.is_none());

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    result.validate().unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut:
                hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceArrangementUnion
        }
    );
    assert_eq!(result.mesh.vertices().len(), union.mesh.vertices().len());

    let union_arrangement_report = hypermesh::exact::certify_planar_arrangement_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    union_arrangement_report.validate().unwrap();
    assert_eq!(
        union_arrangement_report.status,
        hypermesh::exact::ExactPlanarArrangementStatus::AlreadyMaterialized
    );

    let intersection = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    intersection.validate().unwrap();
    assert_eq!(
        intersection.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceIntersection
    );

    let intersection_output =
        hypermesh::exact::arrange_coplanar_convex_surface_intersection(&left, &right)
            .expect("overlapping convex sheets should materialize their convex intersection");
    intersection_output.validate().unwrap();
    assert_eq!(intersection_output.polygon.len(), 4);
    let intersection_result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    intersection_result.validate().unwrap();
    assert_eq!(
        intersection_result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceIntersection
        }
    );

    let intersection_arrangement_report = hypermesh::exact::certify_planar_arrangement_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    intersection_arrangement_report.validate().unwrap();
    assert_eq!(
        intersection_arrangement_report.status,
        hypermesh::exact::ExactPlanarArrangementStatus::AlreadyMaterialized
    );
    let intersection_winding_report = hypermesh::exact::certify_winding_readiness_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    intersection_winding_report.validate().unwrap();
    assert_eq!(
        intersection_winding_report.status,
        hypermesh::exact::ExactWindingReadinessStatus::PlanarArrangementAlreadyMaterialized
    );

    let difference_preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    difference_preflight.validate().unwrap();
    assert_eq!(
        difference_preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementDifference
    );
    assert!(difference_preflight.blocker.is_none());
    let difference_arrangement_report = hypermesh::exact::certify_planar_arrangement_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    difference_arrangement_report.validate().unwrap();
    assert_eq!(
        difference_arrangement_report.status,
        hypermesh::exact::ExactPlanarArrangementStatus::AlreadyMaterialized
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_convex_surface_difference_materializes_simple_loop() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 2, 0, 6, 2, 0, 6, 6, 0, 2, 6, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let difference = hypermesh::exact::arrange_coplanar_convex_surface_difference(&left, &right)
        .expect("overlapping convex sheets should materialize one simple difference loop");
    difference.validate().unwrap();
    assert_eq!(difference.polygon.len(), 6);
    assert!(!difference.mesh.triangles().is_empty());

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    preflight.validate().unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementDifference
    );
    assert!(preflight.blocker.is_none());

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    result.validate().unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceArrangementDifference
        }
    );
    assert_eq!(
        result.mesh.vertices().len(),
        difference.mesh.vertices().len()
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_convex_surface_report_rejects_inconsistent_artifacts() {
    let vertices = &[0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0];
    let left = ExactMesh::from_i64_triangles_with_policy(
        vertices,
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        vertices,
        &[0, 1, 3, 1, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let equivalence = hypermesh::exact::certify_coplanar_convex_surface_equivalence(&left, &right)
        .expect("fixture should certify equivalent convex sheets");

    let rejected_with_certificate = hypermesh::exact::CoplanarConvexSurfaceReport {
        status: hypermesh::exact::CoplanarConvexSurfaceReportStatus::NotCertified,
        equivalence: Some(equivalence.clone()),
        containment: None,
    };
    assert_eq!(
        rejected_with_certificate.validate().unwrap_err(),
        hypermesh::exact::CoplanarConvexSurfaceReportError::UnexpectedCertificate
    );

    let missing_equivalence = hypermesh::exact::CoplanarConvexSurfaceReport {
        status: hypermesh::exact::CoplanarConvexSurfaceReportStatus::Equivalent,
        equivalence: None,
        containment: None,
    };
    assert_eq!(
        missing_equivalence.validate().unwrap_err(),
        hypermesh::exact::CoplanarConvexSurfaceReportError::MissingEquivalenceCertificate
    );

    let invalid_equivalence = hypermesh::exact::CoplanarConvexSurfaceReport {
        status: hypermesh::exact::CoplanarConvexSurfaceReportStatus::Equivalent,
        equivalence: Some(hypermesh::exact::CoplanarConvexSurfaceEquivalence {
            left_area2: hypermesh::exact::ExactReal::from(0),
            ..equivalence
        }),
        containment: None,
    };
    assert_eq!(
        invalid_equivalence.validate().unwrap_err(),
        hypermesh::exact::CoplanarConvexSurfaceReportError::InvalidEquivalenceCertificate
    );
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
    shifted_report.validate().unwrap();
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
    count_report.validate().unwrap();
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
    facts.validate().unwrap();
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
    inside.validate().unwrap();

    let outside = classify_point_against_convex_solid_report(&p3(11, 1, 1), &outer);
    assert_eq!(
        outside.relation,
        hypermesh::exact::ConvexSolidPointRelation::Outside
    );
    assert!(!outside.predicates.is_empty());
    assert!(outside.predicates.len() <= outer.triangles().len());
    assert!(outside.all_proof_producing());
    outside.validate().unwrap();

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
    containment.validate().unwrap();

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
    separated.validate().unwrap();
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
    point.validate().unwrap();

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
    mesh.validate().unwrap();
}

#[test]
fn exact_convex_solid_report_validation_rejects_inconsistent_artifacts() {
    let facts = hypermesh::exact::ConvexSolidFacts {
        orientation: hypermesh::exact::ClosedMeshOrientation::NotClosed,
        convexity: hypermesh::exact::ConvexSolidClassification::Convex,
        predicates: Vec::new(),
    };
    assert_eq!(
        facts.validate().unwrap_err(),
        hypermesh::exact::ConvexSolidReportError::NotClosedStateMismatch
    );

    let point = hypermesh::exact::ConvexSolidPointClassification {
        relation: hypermesh::exact::ConvexSolidPointRelation::NotCertifiedConvex,
        predicates: vec![hypermesh::exact::PredicateUse::from_certificate(
            hyperlimit::PredicateCertificate::ExactRealFact,
        )],
    };
    assert_eq!(
        point.validate().unwrap_err(),
        hypermesh::exact::ConvexSolidReportError::NonCertifiedPointHasPredicates
    );

    let solid_facts = hypermesh::exact::ConvexSolidFacts {
        orientation: hypermesh::exact::ClosedMeshOrientation::Positive,
        convexity: hypermesh::exact::ConvexSolidClassification::Convex,
        predicates: Vec::new(),
    };
    let mesh = hypermesh::exact::ConvexSolidMeshClassification {
        relation: hypermesh::exact::ConvexSolidMeshRelation::StrictlyInside,
        solid_facts,
        vertices: vec![hypermesh::exact::ConvexSolidPointClassification {
            relation: hypermesh::exact::ConvexSolidPointRelation::Outside,
            predicates: Vec::new(),
        }],
    };
    assert_eq!(
        mesh.validate().unwrap_err(),
        hypermesh::exact::ConvexSolidReportError::MeshRelationMismatch
    );
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
    assert_eq!(
        union.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::ConvexContainment
        }
    );
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
fn exact_split_topology_validation_rechecks_graph_vertex_source_facts() {
    let topology_plan = ExactSplitTopologyPlan {
        graph_vertices: vec![ExactGraphVertex {
            point: p3(1, 0, 0),
            uses: vec![ExactGraphVertexUse {
                side: MeshSide::Left,
                edge: [0, 1],
                face_pair: [0, 0],
                plane_face: 0,
                parameter: half(),
                parameter_ratio: hypermesh::exact::SegmentPlaneParameterRatio {
                    numerator: ExactReal::from(2),
                    denominator: ExactReal::from(3),
                },
                endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)],
            }],
        }],
        edge_chains: vec![SplitEdgeChain {
            side: MeshSide::Left,
            edge: [0, 1],
            nodes: vec![
                SplitEdgeNode::OriginalVertex {
                    side: MeshSide::Left,
                    vertex: 0,
                },
                SplitEdgeNode::GraphVertex { graph_vertex: 0 },
                SplitEdgeNode::OriginalVertex {
                    side: MeshSide::Left,
                    vertex: 1,
                },
            ],
        }],
        unresolved_vertex_lookups: 0,
        unresolved_equalities: 0,
        unknown_orderings: 0,
    };

    let report = topology_plan.validate();

    assert!(!report.is_valid());
    report.validate().unwrap();
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::InvalidConstructionRatio
            && diagnostic.graph_vertex == Some(0)
    }));
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
fn exact_face_split_plan_validation_rechecks_source_construction_facts() {
    let topology_plan = ExactSplitTopologyPlan {
        graph_vertices: vec![ExactGraphVertex {
            point: p3(1, 0, 0),
            uses: vec![ExactGraphVertexUse {
                side: MeshSide::Left,
                edge: [0, 1],
                face_pair: [0, 0],
                plane_face: 0,
                parameter: half(),
                parameter_ratio: hypermesh::exact::SegmentPlaneParameterRatio {
                    numerator: ExactReal::from(2),
                    denominator: ExactReal::from(3),
                },
                endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)],
            }],
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
            edges: vec![FaceSplitEdge {
                edge: [0, 1],
                graph_vertices: vec![0],
            }],
        }],
    };

    let report = face_plan.validate_against_topology(&topology_plan);

    assert!(!report.is_valid());
    report.validate().unwrap();
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::InvalidConstructionRatio
            && diagnostic.graph_vertex == Some(0)
    }));
    assert!(!report.diagnostics.iter().any(|diagnostic| {
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

    graph.validate().unwrap();
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
    let overlap_graph = graph.face_pairs[0]
        .coplanar_overlap_graph()
        .expect("coplanar pair should expose an overlap graph");
    overlap_graph.validate().unwrap();
    assert_eq!(overlap_graph.left_face, 0);
    assert_eq!(overlap_graph.right_face, 0);
    assert_eq!(
        overlap_graph.relation,
        MeshFacePairRelation::CoplanarOverlapping
    );
    assert!(!overlap_graph.edge_overlaps.is_empty());
    assert!(!overlap_graph.vertex_overlaps.is_empty());
    assert_eq!(graph.coplanar_overlap_graphs(), vec![overlap_graph]);
    let split_plan = graph.coplanar_overlap_split_plan(&left, &right).unwrap();
    split_plan.validate().unwrap();
    assert_eq!(split_plan.graphs.len(), 1);
    assert_eq!(split_plan.graphs[0].left_face, 0);
    assert_eq!(split_plan.graphs[0].right_face, 0);
    assert!(
        split_plan.graphs[0]
            .edge_splits
            .iter()
            .any(|split| split.interval_overlap || !split.points.is_empty())
    );

    let readiness = graph
        .coplanar_arrangement_readiness_report(&left, &right)
        .unwrap();
    readiness.validate().unwrap();
    assert!(readiness.needs_planar_cells());
    assert_eq!(
        readiness.status,
        hypermesh::exact::CoplanarArrangementReadinessStatus::NeedsPlanarCells
    );
    assert_eq!(readiness.graph_count, 1);
    assert_eq!(readiness.overlapping_graphs, 1);
    assert!(readiness.edge_overlap_count > 0);
}

#[test]
fn exact_coplanar_overlap_graph_validation_rejects_malformed_records() {
    let empty = hypermesh::exact::CoplanarOverlapGraph {
        left_face: 0,
        right_face: 0,
        relation: MeshFacePairRelation::CoplanarTouching,
        projection: hypermesh::exact::CoplanarProjection::Xy,
        edge_overlaps: Vec::new(),
        vertex_overlaps: Vec::new(),
    };
    assert_eq!(
        empty.validate().unwrap_err(),
        hypermesh::exact::CoplanarOverlapGraphValidationError::EmptyOverlapGraph
    );

    let disjoint_edge = hypermesh::exact::CoplanarOverlapGraph {
        edge_overlaps: vec![hypermesh::exact::CoplanarEdgeOverlap {
            left_edge: [0, 1],
            right_edge: [2, 3],
            relation: hyperlimit::SegmentIntersection::Disjoint,
        }],
        ..empty.clone()
    };
    assert_eq!(
        disjoint_edge.validate().unwrap_err(),
        hypermesh::exact::CoplanarOverlapGraphValidationError::DisjointEdgeOverlap
    );

    let same_side_vertex = hypermesh::exact::CoplanarOverlapGraph {
        edge_overlaps: Vec::new(),
        vertex_overlaps: vec![hypermesh::exact::CoplanarVertexOverlap {
            vertex_side: MeshSide::Left,
            vertex: 0,
            triangle_side: MeshSide::Left,
            triangle_face: 0,
            location: hyperlimit::TriangleLocation::Inside,
        }],
        ..empty
    };
    assert_eq!(
        same_side_vertex.validate().unwrap_err(),
        hypermesh::exact::CoplanarOverlapGraphValidationError::SameSideVertexOverlap
    );
}

#[test]
fn exact_coplanar_arrangement_readiness_validation_rejects_bad_counts() {
    let mut no_overlap = hypermesh::exact::CoplanarArrangementReadinessReport {
        status: hypermesh::exact::CoplanarArrangementReadinessStatus::NoCoplanarOverlap,
        graph_count: 0,
        overlapping_graphs: 0,
        touching_graphs: 0,
        edge_overlap_count: 0,
        vertex_overlap_count: 0,
        point_split_count: 0,
        interval_overlap_count: 0,
    };
    no_overlap.validate().unwrap();

    no_overlap.edge_overlap_count = 1;
    assert_eq!(
        no_overlap.validate().unwrap_err(),
        hypermesh::exact::CoplanarArrangementReadinessValidationError::NoOverlapWithEvidence
    );

    let mismatch = hypermesh::exact::CoplanarArrangementReadinessReport {
        status: hypermesh::exact::CoplanarArrangementReadinessStatus::NeedsPlanarCells,
        graph_count: 2,
        overlapping_graphs: 1,
        touching_graphs: 0,
        edge_overlap_count: 1,
        vertex_overlap_count: 0,
        point_split_count: 0,
        interval_overlap_count: 0,
    };
    assert_eq!(
        mismatch.validate().unwrap_err(),
        hypermesh::exact::CoplanarArrangementReadinessValidationError::GraphCountMismatch
    );

    let missing_overlap = hypermesh::exact::CoplanarArrangementReadinessReport {
        status: hypermesh::exact::CoplanarArrangementReadinessStatus::NeedsPlanarCells,
        graph_count: 1,
        overlapping_graphs: 0,
        touching_graphs: 1,
        edge_overlap_count: 1,
        vertex_overlap_count: 0,
        point_split_count: 0,
        interval_overlap_count: 0,
    };
    assert_eq!(
        missing_overlap.validate().unwrap_err(),
        hypermesh::exact::CoplanarArrangementReadinessValidationError::NeedsCellsMissingOverlap
    );
}

#[test]
fn exact_coplanar_overlap_split_validation_rejects_malformed_records() {
    let missing_point = hypermesh::exact::CoplanarEdgeSplitConstruction {
        overlap: hypermesh::exact::CoplanarEdgeOverlap {
            left_edge: [0, 1],
            right_edge: [2, 3],
            relation: hyperlimit::SegmentIntersection::Proper,
        },
        points: Vec::new(),
        interval_overlap: false,
    };
    assert_eq!(
        missing_point.validate().unwrap_err(),
        hypermesh::exact::CoplanarOverlapSplitValidationError::MissingPointConstruction
    );

    let missing_interval = hypermesh::exact::CoplanarEdgeSplitConstruction {
        overlap: hypermesh::exact::CoplanarEdgeOverlap {
            left_edge: [0, 1],
            right_edge: [2, 3],
            relation: hyperlimit::SegmentIntersection::CollinearOverlap,
        },
        points: Vec::new(),
        interval_overlap: false,
    };
    assert_eq!(
        missing_interval.validate().unwrap_err(),
        hypermesh::exact::CoplanarOverlapSplitValidationError::MissingIntervalConstruction
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
        prop_assert!(event.validate().is_ok());
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
        prop_assert!(classification.validate().is_ok());
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
