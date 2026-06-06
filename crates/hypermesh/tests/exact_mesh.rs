use hyperlimit::{Point3, SourceProvenance};
use hypermesh::{
    ExactArrangement, ExactBooleanOperation, ExactBooleanResultKind, ExactBoundaryBooleanPolicy,
    ExactI64MeshInputReadiness, ExactMesh, ExactRegularizationPolicy, MeshFacePairRelation,
    MeshFacePairValidationError, TriangleTriangleRelation, ValidationPolicy, boolean_exact,
    boolean_exact_with_boundary_policy, build_intersection_graph, certify_boundary_touching_report,
    classify_mesh_face_pair, classify_triangle_triangle, inspect_i64_mesh_input,
    preflight_boolean_exact, preflight_boolean_exact_with_boundary_policy,
};
use hyperreal::Real;

fn p(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

fn tetra(offset: [i64; 3]) -> ExactMesh {
    let [ox, oy, oz] = offset;
    ExactMesh::new(
        vec![
            p(ox, oy, oz),
            p(ox + 1, oy, oz),
            p(ox, oy + 1, oz),
            p(ox, oy, oz + 1),
        ],
        vec![
            hypermesh::Triangle([0, 2, 1]),
            hypermesh::Triangle([0, 1, 3]),
            hypermesh::Triangle([1, 2, 3]),
            hypermesh::Triangle([2, 0, 3]),
        ],
        SourceProvenance::exact("test tetra"),
    )
    .unwrap()
}

#[test]
fn exact_mesh_construction_retains_valid_public_facts() {
    let mesh = tetra([0, 0, 0]);

    mesh.validate_retained_state().unwrap();
    assert_eq!(mesh.facts().mesh.vertex_count, 4);
    assert_eq!(mesh.facts().mesh.face_count, 4);
    assert!(mesh.facts().mesh.closed_manifold);
    assert!(
        mesh.facts()
            .faces
            .iter()
            .all(|face| face.triangle.non_degenerate)
    );

    let report = inspect_i64_mesh_input(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    );
    let readiness = report.readiness();
    assert_eq!(readiness, ExactI64MeshInputReadiness::Ready);
    assert!(report.edge_ready());
}

#[test]
fn exact_face_pair_classifier_rejects_disjoint_bounds_publicly() {
    let left = tetra([0, 0, 0]);
    let right = tetra([3, 0, 0]);

    let pair = classify_mesh_face_pair(&left, 0, &right, 0).unwrap();
    assert_eq!(pair.relation, MeshFacePairRelation::BoundsDisjoint);
    assert!(pair.triangle.is_none());

    let graph = build_intersection_graph(&left, &right).unwrap();
    assert!(graph.face_pairs.is_empty());
}

#[test]
fn exact_triangle_classifier_reports_degenerate_coplanar_overlap() {
    let points = vec![
        p(0, 0, 0),
        p(2, 0, 0),
        p(0, 2, 0),
        p(0, 0, 0),
        p(1, 0, 0),
        p(0, 1, 0),
    ];

    let classification = classify_triangle_triangle(&points, [0, 1, 2], [3, 4, 5]);
    assert_eq!(
        classification.relation,
        TriangleTriangleRelation::CoplanarOverlapping
    );
    assert!(classification.coplanar.is_some());
    assert!(classification.validate().is_ok());
}

#[test]
fn exact_face_pair_classifier_matches_local_triangle_report() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0, 9, 9, 9],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0, -9, -9, -9],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let pair = classify_mesh_face_pair(&left, 0, &right, 0).unwrap();
    let points = vec![
        left.vertices()[0].clone(),
        left.vertices()[1].clone(),
        left.vertices()[2].clone(),
        right.vertices()[0].clone(),
        right.vertices()[1].clone(),
        right.vertices()[2].clone(),
    ];
    let direct = classify_triangle_triangle(&points, [0, 1, 2], [3, 4, 5]);

    assert_eq!(pair.relation, MeshFacePairRelation::CoplanarOverlapping);
    assert_eq!(pair.triangle.as_ref().unwrap(), &direct);
    pair.validate_against_sources(&left, &right).unwrap();
}

#[test]
fn exact_face_pair_candidate_retains_source_plane_split_events() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, -1, -1, 1, 3, 1, 1, 3, -1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let pair = classify_mesh_face_pair(&left, 0, &right, 0).unwrap();
    assert_eq!(pair.relation, MeshFacePairRelation::Candidate);

    let triangle = pair.triangle.as_ref().unwrap();
    assert_eq!(triangle.relation, TriangleTriangleRelation::Candidate);
    assert_eq!(triangle.right_edge_events.len(), 3);
    assert_eq!(triangle.left_edge_events.len(), 3);
    assert!(
        triangle
            .right_edge_events
            .iter()
            .chain(&triangle.left_edge_events)
            .all(|event| event.predicates.is_empty())
    );
    pair.validate_against_sources(&left, &right).unwrap();

    let mut truncated = pair.clone();
    truncated.triangle.as_mut().unwrap().right_edge_events.pop();
    assert_eq!(
        truncated.validate(),
        Err(MeshFacePairValidationError::CandidateMissingEdgeEvents)
    );
}

#[test]
fn exact_boolean_public_shortcuts_handle_disjoint_operands() {
    let left = tetra([0, 0, 0]);
    let right = tetra([3, 0, 0]);

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union).unwrap();
    assert!(!preflight.graph_had_unknowns);

    let union = boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(
        union.kind,
        ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Union,
            shortcut: hypermesh::ExactBooleanShortcutKind::BoundsDisjoint
        }
    );
    union.mesh.validate_retained_state().unwrap();
    union.validate_against_sources(&left, &right).unwrap();
    assert!(union.validate_against_sources(&left, &left).is_err());
    let mut relabeled = union.clone();
    relabeled.kind = ExactBooleanResultKind::CertifiedShortcut {
        operation: ExactBooleanOperation::Intersection,
        shortcut: hypermesh::ExactBooleanShortcutKind::BoundsDisjoint,
    };
    assert!(
        relabeled
            .validate_operation_against_sources(
                &left,
                &right,
                ExactBooleanOperation::Union,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .is_err()
    );

    let intersection = boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(intersection.mesh.triangles().is_empty());
}

#[test]
fn exact_arrangement_public_path_reports_blockers_or_cells() {
    let left = tetra([0, 0, 0]);
    let right = tetra([1, 0, 0]);

    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();
    assert!(arrangement.validate_against_sources(&left, &right).is_ok());
}

#[test]
fn boundary_policy_remains_explicit_for_named_booleans() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 0, 0, 0, 2, 0, 2, 2, 2],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let report = certify_boundary_touching_report(&left, &right).unwrap();
    assert!(report.is_certified(), "{report:?}");
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::ExactBooleanSupport::RequiresBoundaryPolicy,
        "{preflight:?}"
    );
    let rejected_policy_preflight = preflight_boolean_exact_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    assert_eq!(rejected_policy_preflight, preflight);

    let policy_preflight = preflight_boolean_exact_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    assert_eq!(
        policy_preflight.support,
        hypermesh::ExactBooleanSupport::CertifiedBoundaryPolicyShortcut,
        "{policy_preflight:?}"
    );
    assert!(policy_preflight.blocker.is_none(), "{policy_preflight:?}");
    assert_eq!(
        policy_preflight.retained_face_pairs,
        report.retained_face_pairs
    );
    assert_eq!(policy_preflight.retained_events, report.retained_events);
    policy_preflight
        .validate_against_sources_with_boundary_policy(
            &left,
            &right,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .unwrap();
    assert_eq!(
        policy_preflight.freshness_against_sources_with_boundary_policy(
            &left,
            &right,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        ),
        hypermesh::ExactReportFreshness::Current
    );
    assert!(
        policy_preflight
            .validate_against_sources(&left, &right)
            .is_err(),
        "strict replay should not certify a boundary-policy preflight"
    );
    assert!(
        boolean_exact(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY
        )
        .is_err()
    );

    let projected = boolean_exact_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    assert_eq!(
        projected.kind,
        ExactBooleanResultKind::BoundaryPolicyShortcut {
            operation: ExactBooleanOperation::Union
        }
    );
    projected.mesh.validate_retained_state().unwrap();
    projected.validate_against_sources(&left, &right).unwrap();

    let separated_right = ExactMesh::from_i64_triangles_with_policy(
        &[5, 0, 0, 7, 0, 0, 5, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(
        projected
            .validate_against_sources(&left, &separated_right)
            .is_err()
    );
    assert!(
        policy_preflight
            .validate_against_sources_with_boundary_policy(
                &left,
                &separated_right,
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            )
            .is_err()
    );
}
