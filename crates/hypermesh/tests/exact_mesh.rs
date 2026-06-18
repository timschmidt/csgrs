use hyperlimit::{Point3, SourceProvenance};
use hypermesh::{
    ExactBooleanOperation, ExactBooleanRequest, ExactBooleanResult, ExactBooleanWorkspace,
    ExactBoundaryBooleanPolicy, ExactI64MeshInputReadiness, ExactMesh, ExactMeshConsumerDomain,
    ExactOutputTriangleOrientation, ExactRegionSelection, ExactRegularizationPolicy,
    ExactReportFreshness, LossyF64MeshInputReadiness, MeshArtifactBlocker, MeshArtifactFaceRecord,
    MeshArtifactManifest, MeshArtifactVertexRecord, ValidationPolicy,
};
use hyperreal::Real;

fn p(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

fn exact_boolean_evaluation(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> hypermesh::ExactBooleanEvaluation {
    let mut workspace = ExactBooleanWorkspace::new(left, right);
    workspace.evaluate(request).unwrap().clone()
}

fn evaluation_materializes_arrangement_cell_complex(
    evaluation: &hypermesh::ExactBooleanEvaluation,
) -> bool {
    evaluation
        .certifications
        .arrangement_attempt
        .as_ref()
        .is_some_and(|attempt| {
            attempt.operation == evaluation.request.operation
                && attempt.policy == ExactRegularizationPolicy::REGULARIZED_SOLID
                && attempt.output_validation == evaluation.request.validation
                && attempt.materialized_arrangement_cell_complex_output()
        })
        || evaluation
            .certifications
            .winding_readiness
            .materializes_arrangement_cell_complex()
}

fn exact_boolean_result(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> ExactBooleanResult {
    let mut workspace = ExactBooleanWorkspace::new(left, right);
    workspace.materialize(request).unwrap()
}

fn exact_boolean_materialize_result(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<ExactBooleanResult, hypermesh::MeshError> {
    let mut workspace = ExactBooleanWorkspace::new(left, right);
    workspace.materialize(request)
}

fn exact_boolean_arrangement_attempt(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    policy: ExactRegularizationPolicy,
) -> hypermesh::ExactArrangementBooleanAttempt {
    assert_eq!(policy, ExactRegularizationPolicy::REGULARIZED_SOLID);
    let mut workspace = ExactBooleanWorkspace::new(left, right);
    workspace
        .evaluate(request)
        .unwrap()
        .certifications
        .arrangement_attempt
        .as_ref()
        .expect("evaluation should retain an arrangement attempt")
        .clone()
}

macro_rules! exact_adjacent_union_completion_report {
    ($left:expr, $right:expr, $request:expr $(,)?) => {
        exact_boolean_evaluation($left, $right, $request)
            .certifications
            .adjacent_union_completion
            .clone()
    };
}

fn assert_public_full_face_adjacent_union(
    left: &ExactMesh,
    right: &ExactMesh,
    expected_shared_faces: usize,
    expected_shared_patches: usize,
) -> ExactBooleanResult {
    let request = ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
    let report = exact_adjacent_union_completion_report!(left, right, request);
    assert!(report.is_certified_full_face());
    assert_eq!(report.full_face_shared_faces, expected_shared_faces);
    assert_eq!(report.full_face_shared_patches, expected_shared_patches);
    assert!(report.is_certified());
    report.validate().unwrap();
    report.validate_against_sources(left, right).unwrap();
    assert_eq!(
        report.freshness_against_sources(left, right),
        ExactReportFreshness::Current
    );

    let result = exact_boolean_result(left, right, request);
    assert!(
        result.is_certified_shortcut_for(ExactBooleanOperation::Union),
        "{result:?}"
    );
    result.validate().unwrap();
    result.validate_against_sources(left, right).unwrap();
    assert_eq!(
        result.freshness_against_sources(left, right),
        ExactReportFreshness::Current
    );
    assert!(result.mesh.facts().mesh.closed_manifold);
    assert!(!result.mesh.triangles().is_empty());
    result
}

fn assert_public_contained_face_adjacent_union(
    left: &ExactMesh,
    right: &ExactMesh,
    expected_containing_faces: usize,
    expected_contained_faces: usize,
) -> ExactBooleanResult {
    let request = ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
    let report = exact_adjacent_union_completion_report!(left, right, request);
    assert!(report.is_certified_contained_face());
    assert_eq!(report.containing_faces, expected_containing_faces);
    assert_eq!(report.contained_faces, expected_contained_faces);
    assert!(report.is_certified());
    report.validate().unwrap();
    report.validate_against_sources(left, right).unwrap();
    assert_eq!(
        report.freshness_against_sources(left, right),
        ExactReportFreshness::Current
    );

    let result = exact_boolean_result(left, right, request);
    assert!(
        result.is_certified_shortcut_for(ExactBooleanOperation::Union),
        "{result:?}"
    );
    result.validate().unwrap();
    result.validate_against_sources(left, right).unwrap();
    assert_eq!(
        result.freshness_against_sources(left, right),
        ExactReportFreshness::Current
    );
    assert!(result.mesh.facts().mesh.closed_manifold);
    assert!(!result.mesh.triangles().is_empty());
    result
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

fn tetra_from_corners(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2],
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap()
}

fn axis_aligned_box(min: [i64; 3], max: [i64; 3]) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            min[0], min[1], min[2], max[0], min[1], min[2], max[0], max[1], min[2], min[0], max[1],
            min[2], min[0], min[1], max[2], max[0], min[1], max[2], max[0], max[1], max[2], min[0],
            max[1], max[2],
        ],
        &[
            0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7,
            6, 3, 0, 4, 3, 4, 7,
        ],
    )
    .unwrap()
}

fn face_fan_box() -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, 0, 0, 2, 2, 0, 2, 2, 2, 2, 0, 2, 2, 1, 1, 0,
        ],
        &[
            0, 8, 1, //
            1, 8, 2, //
            2, 8, 3, //
            3, 8, 0, //
            4, 5, 6, //
            4, 6, 7, //
            0, 1, 5, //
            0, 5, 4, //
            1, 2, 6, //
            1, 6, 5, //
            2, 3, 7, //
            2, 7, 6, //
            3, 0, 4, //
            3, 4, 7,
        ],
    )
    .unwrap()
}

fn combine_exact_meshes(left: &ExactMesh, right: &ExactMesh, label: &'static str) -> ExactMesh {
    let right_offset = left.vertices().len();
    ExactMesh::new(
        left.vertices()
            .iter()
            .chain(right.vertices())
            .cloned()
            .collect(),
        left.triangles()
            .iter()
            .copied()
            .chain(right.triangles().iter().map(|triangle| {
                let [a, b, c] = triangle.0;
                hypermesh::Triangle([a + right_offset, b + right_offset, c + right_offset])
            }))
            .collect(),
        SourceProvenance::exact(label),
    )
    .unwrap()
}

#[test]
fn exact_boolean_evaluation_materializes_certified_result_publicly() {
    let left = tetra([0, 0, 0]);
    let right = tetra([4, 0, 0]);
    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    );

    let evaluation = exact_boolean_evaluation(&left, &right, request);

    evaluation.validate().unwrap();
    evaluation.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        evaluation.freshness_against_sources(&left, &right),
        hypermesh::ExactReportFreshness::Current
    );
    assert!(evaluation.preflight.is_certified());
    assert!(evaluation.result.as_ref().is_some());
    assert_eq!(evaluation.preflight.required_blocker_kind(), None);
    assert!(evaluation.preflight.is_certified());
    assert!(
        evaluation.result.as_ref().is_some_and(|result| {
            result.is_certified_shortcut_for(ExactBooleanOperation::Union)
        })
    );
    let stale_right = tetra([8, 0, 0]);
    assert!(
        evaluation
            .validate_against_sources(&left, &stale_right)
            .is_err()
    );
    assert_eq!(
        evaluation.freshness_against_sources(&left, &stale_right),
        hypermesh::ExactReportFreshness::SourceReplayMismatch
    );
    let mut relabeled_request = evaluation.clone();
    relabeled_request.request = ExactBooleanRequest::new(
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    );
    assert_eq!(
        relabeled_request.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    let mut stale_attempt_policy = evaluation.clone();
    stale_attempt_policy
        .certifications
        .arrangement_attempt
        .as_mut()
        .expect("named evaluation should retain arrangement attempt")
        .output_validation = ValidationPolicy::CLOSED;
    assert_eq!(
        stale_attempt_policy.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    let mut relabeled_support = evaluation.clone();
    let convex_left = axis_aligned_box([0, 0, 0], [2, 2, 2]);
    let convex_right = axis_aligned_box([1, 1, 1], [3, 3, 3]);
    relabeled_support.preflight = exact_boolean_evaluation(
        &convex_left,
        &convex_right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    )
    .preflight
    .clone();
    relabeled_support.preflight.validate().unwrap();
    assert_eq!(
        relabeled_support.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    let mut relabeled_winding_status = evaluation.clone();
    relabeled_winding_status
        .certifications
        .winding_readiness
        .operation = ExactBooleanOperation::Difference;
    relabeled_winding_status
        .certifications
        .winding_readiness
        .validate()
        .unwrap();
    assert_eq!(
        relabeled_winding_status.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
}

#[test]
fn exact_boolean_evaluation_retains_region_ownership_report() {
    let left = tetra_from_corners([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let right = tetra_from_corners([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);
    let request = ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);

    let evaluation = exact_boolean_evaluation(&left, &right, request);

    evaluation.validate().unwrap();
    evaluation.validate_against_sources(&left, &right).unwrap();
    assert!(
        evaluation_materializes_arrangement_cell_complex(&evaluation),
        "{evaluation:?}"
    );
    let disjoint_left = axis_aligned_box([0, 0, 0], [1, 1, 1]);
    let disjoint_right = axis_aligned_box([3, 3, 3], [4, 4, 4]);
    let disjoint_readiness = exact_boolean_evaluation(
        &disjoint_left,
        &disjoint_right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    )
    .certifications
    .winding_readiness
    .clone();
    let mut attempt_backed_evaluation = evaluation.clone();
    attempt_backed_evaluation.certifications.winding_readiness = disjoint_readiness;
    assert!(
        evaluation_materializes_arrangement_cell_complex(&attempt_backed_evaluation),
        "{attempt_backed_evaluation:?}"
    );
    assert!(
        evaluation
            .certifications
            .arrangement_attempt
            .as_ref()
            .is_some(),
        "named boolean certifications should retain arrangement attempt"
    );
    let ownership = evaluation
        .certifications
        .arrangement_attempt
        .as_ref()
        .and_then(|attempt| attempt.region_ownership_report.as_ref())
        .expect("named boolean certifications should retain region ownership");
    ownership.validate().unwrap();
    assert!(ownership.is_resolved());
    assert!(ownership.status.is_volume_resolved());
    assert_eq!(ownership.volume_regions, 3);
    assert_eq!(ownership.shared_owned_volumes, 1);
    evaluation
        .certifications
        .arrangement_attempt
        .as_ref()
        .and_then(|attempt| attempt.topology_assembly_report.as_ref())
        .expect("named boolean certifications should retain topology assembly")
        .validate()
        .unwrap();

    let mut missing_attempt_ownership = evaluation.clone();
    missing_attempt_ownership
        .certifications
        .arrangement_attempt
        .as_mut()
        .expect("evaluation should retain arrangement attempt")
        .region_ownership_report = None;
    assert_eq!(
        missing_attempt_ownership.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );

    let mut missing_attempt_topology = evaluation.clone();
    missing_attempt_topology
        .certifications
        .arrangement_attempt
        .as_mut()
        .expect("evaluation should retain arrangement attempt")
        .topology_assembly_report = None;
    assert_eq!(
        missing_attempt_topology.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
}

#[test]
fn exact_boolean_evaluation_materializes_boundary_policy_shortcut_by_default() {
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
    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    );

    let evaluation = exact_boolean_evaluation(&left, &right, request);

    evaluation.validate().unwrap();
    evaluation.validate_against_sources(&left, &right).unwrap();
    assert!(evaluation.preflight.is_certified());
    assert!(evaluation.result.as_ref().is_some());
    assert_eq!(evaluation.preflight.required_blocker_kind(), None);
    assert!(evaluation.preflight.is_certified());
    assert!(evaluation.preflight.is_certified_boundary_policy_shortcut());
    assert!(evaluation.preflight.has_retained_exact_evidence());
    let result = evaluation
        .result
        .as_ref()
        .expect("boundary-policy evaluation should materialize");
    result.validate().unwrap();
    result.validate_against_sources(&left, &right).unwrap();
    let mut mixed_graph_snapshot = evaluation.clone();
    mixed_graph_snapshot
        .certifications
        .refinement
        .retained_face_pairs = 0;
    mixed_graph_snapshot
        .certifications
        .refinement
        .retained_events = 0;
    mixed_graph_snapshot
        .certifications
        .refinement
        .validate()
        .unwrap();
    assert_eq!(
        mixed_graph_snapshot.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    let mut relabeled_winding_status = evaluation.clone();
    relabeled_winding_status
        .certifications
        .winding_readiness
        .operation = ExactBooleanOperation::Difference;
    relabeled_winding_status
        .certifications
        .winding_readiness
        .validate()
        .unwrap();
    assert_eq!(
        relabeled_winding_status.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    let rejected_request = ExactBooleanRequest::with_boundary_policy(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    );
    let rejected = exact_boolean_evaluation(&left, &right, rejected_request);
    rejected.validate().unwrap();
    assert!(!rejected.preflight.is_certified());
    assert!(rejected.result.as_ref().is_none());
    let mut impossible_materialization = rejected.clone();
    impossible_materialization.result = evaluation.result.as_ref().cloned();
    assert_eq!(
        impossible_materialization.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
}

fn skew_affine_box(min: [i64; 3], max: [i64; 3]) -> ExactMesh {
    let p = |u: i64, v: i64, w: i64| [2 * u + v, 2 * v, 2 * w];
    let corners = [
        p(min[0], min[1], min[2]),
        p(max[0], min[1], min[2]),
        p(max[0], max[1], min[2]),
        p(min[0], max[1], min[2]),
        p(min[0], min[1], max[2]),
        p(max[0], min[1], max[2]),
        p(max[0], max[1], max[2]),
        p(min[0], max[1], max[2]),
    ];
    ExactMesh::from_i64_triangles(
        &corners
            .iter()
            .flat_map(|point| point.iter().copied())
            .collect::<Vec<_>>(),
        &[
            0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7,
            6, 3, 0, 4, 3, 4, 7,
        ],
    )
    .unwrap()
}

fn skew_affine_mesh_from_axis_aligned(mesh: &ExactMesh, label: &'static str) -> ExactMesh {
    let ten = Real::from(10);
    ExactMesh::new(
        mesh.vertices()
            .iter()
            .map(|point| {
                Point3::new(
                    point.x.clone() + &(point.y.clone() * &ten),
                    point.y.clone(),
                    point.z.clone(),
                )
            })
            .collect(),
        mesh.triangles().to_vec(),
        SourceProvenance::exact(label),
    )
    .unwrap()
}

fn axis_aligned_l_solid(offset: [i64; 3]) -> ExactMesh {
    let [ox, oy, oz] = offset;
    let points = [
        [ox, oy, oz],
        [ox + 2, oy, oz],
        [ox + 2, oy + 1, oz],
        [ox + 1, oy + 1, oz],
        [ox + 1, oy + 2, oz],
        [ox, oy + 2, oz],
        [ox, oy, oz + 1],
        [ox + 2, oy, oz + 1],
        [ox + 2, oy + 1, oz + 1],
        [ox + 1, oy + 1, oz + 1],
        [ox + 1, oy + 2, oz + 1],
        [ox, oy + 2, oz + 1],
    ];
    ExactMesh::from_i64_triangles(
        &points
            .iter()
            .flat_map(|point| point.iter().copied())
            .collect::<Vec<_>>(),
        &[
            6, 7, 9, 7, 8, 9, 6, 9, 11, 9, 10, 11, 3, 1, 0, 3, 2, 1, 5, 3, 0, 5, 4, 3, 0, 1, 7, 0,
            7, 6, 1, 2, 8, 1, 8, 7, 2, 3, 9, 2, 9, 8, 3, 4, 10, 3, 10, 9, 4, 5, 11, 4, 11, 10, 5,
            0, 6, 5, 6, 11,
        ],
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

    let audit = mesh.audit_report().unwrap();
    audit.validate().unwrap();
    audit.validate_against_mesh(&mesh).unwrap();

    let mut impossible_predicate_counts = audit.clone();
    impossible_predicate_counts.proof_predicates =
        impossible_predicate_counts.predicate_uses.saturating_add(1);
    assert!(
        impossible_predicate_counts
            .validate()
            .is_err_and(|error| error.is_invalid_predicate_counts())
    );
    assert!(
        impossible_predicate_counts
            .validate_against_mesh(&mesh)
            .is_err_and(|error| error.is_invalid_predicate_counts())
    );

    let mut empty_topology_audit = audit.clone();
    empty_topology_audit.vertex_count = 0;
    assert!(
        empty_topology_audit
            .validate()
            .is_err_and(|error| error.is_empty_topology())
    );

    let mut empty_source_audit = audit.clone();
    empty_source_audit.source_label.clear();
    assert!(
        empty_source_audit
            .validate()
            .is_err_and(|error| error.is_empty_source_label())
    );

    let mut invalid_version_audit = audit.clone();
    invalid_version_audit.construction_version = 0;
    assert!(
        invalid_version_audit
            .validate()
            .is_err_and(|error| error.is_invalid_construction_version())
    );

    let report = ExactMesh::inspect_i64_triangles(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    );
    let readiness = report.readiness();
    assert_eq!(readiness, ExactI64MeshInputReadiness::Ready);
    assert!(report.edge_ready());

    let mut missing_integer_evidence = report.clone();
    missing_integer_evidence.exact_integer_coordinates -= 1;
    assert!(
        missing_integer_evidence
            .validate()
            .is_err_and(|error| error.is_exact_coordinate_count_mismatch())
    );
    assert_eq!(
        missing_integer_evidence.readiness(),
        ExactI64MeshInputReadiness::InvalidReport
    );

    let mut missing_checked_indices = report.clone();
    missing_checked_indices.checked_indices -= 1;
    assert!(
        missing_checked_indices
            .validate()
            .is_err_and(|error| error.is_checked_index_count_mismatch())
    );

    let mut missing_arity_diagnostic = ExactMesh::inspect_i64_triangles(&[0, 0], &[0, 1, 2]);
    missing_arity_diagnostic.diagnostics.clear();
    assert!(
        missing_arity_diagnostic
            .validate()
            .is_err_and(|error| error.is_missing_coordinate_arity_diagnostic())
    );

    let mut missing_index_arity_diagnostic = ExactMesh::inspect_i64_triangles(&[0, 0, 0], &[0, 1]);
    missing_index_arity_diagnostic.diagnostics.clear();
    assert!(
        missing_index_arity_diagnostic
            .validate()
            .is_err_and(|error| error.is_missing_index_arity_diagnostic())
    );

    let lossy_report = ExactMesh::inspect_f64_triangles(
        &[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        &[0, 1, 2],
    );
    assert_eq!(lossy_report.readiness(), LossyF64MeshInputReadiness::Ready);

    let mut missing_dyadic_evidence = lossy_report.clone();
    missing_dyadic_evidence.exact_dyadic_coordinates -= 1;
    assert!(
        missing_dyadic_evidence
            .validate()
            .is_err_and(|error| error.is_exact_coordinate_count_mismatch())
    );
    assert_eq!(
        missing_dyadic_evidence.readiness(),
        LossyF64MeshInputReadiness::InvalidReport
    );

    let mut missing_float_diagnostic =
        ExactMesh::inspect_f64_triangles(&[0.0, f64::NAN, 0.0], &[0, 1, 2]);
    assert_eq!(
        missing_float_diagnostic.readiness(),
        LossyF64MeshInputReadiness::InvalidCoordinate
    );
    missing_float_diagnostic.diagnostics.clear();
    assert!(
        missing_float_diagnostic
            .validate()
            .is_err_and(|error| error.is_exact_coordinate_count_mismatch())
    );
}

#[test]
fn exact_mesh_proposal_and_artifact_reports_are_publicly_replayable() {
    let exact = tetra([0, 0, 0]);
    let proposal = exact.proposal_report().unwrap();

    proposal.validate().unwrap();
    proposal.validate_against_mesh(&exact).unwrap();
    assert!(proposal.is_exact_construction());
    assert!(proposal.exact_input_replayed());

    let mut stale_proposal = proposal.clone();
    stale_proposal.source_label.push_str(" stale");
    assert!(stale_proposal.validate_against_mesh(&exact).is_err());

    let mut invalid_proposal_audit = proposal.clone();
    invalid_proposal_audit.audit.proof_predicates = invalid_proposal_audit
        .audit
        .predicate_uses
        .saturating_add(1);
    assert!(
        invalid_proposal_audit
            .validate()
            .is_err_and(|error| error.is_audit_replay_invalid_predicate_counts())
    );

    let artifact = exact.artifact_manifest().unwrap().report();
    artifact.validate().unwrap();
    assert!(artifact.is_hypermesh_exact());
    assert!(artifact.is_solid_handoff());
    assert!(artifact.validation_handoff_ready, "{:?}", artifact.blockers);
    assert!(artifact.blockers.is_empty());

    let mut forged_handoff_ready = artifact.clone();
    forged_handoff_ready.validation_handoff_ready = false;
    assert!(
        forged_handoff_ready
            .validate()
            .is_err_and(|error| error.is_report_mismatch("validation_handoff_ready"))
    );

    let proposal_artifact = exact.proposal_artifact_manifest(&proposal).unwrap().report();
    proposal_artifact.validate().unwrap();
    assert_eq!(proposal_artifact, artifact);

    let lossy = ExactMesh::from_f64_triangles(
        &[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();
    let lossy_proposal = lossy.proposal_report().unwrap();
    lossy_proposal.validate_against_mesh(&lossy).unwrap();
    assert!(lossy_proposal.is_lossy_primitive_float_proposal());
    assert!(lossy_proposal.proposal_accepted_after_exact_replay());

    let lossy_artifact = lossy.artifact_manifest().unwrap().report();
    lossy_artifact.validate().unwrap();
    assert!(lossy_artifact.is_hypermesh_lossy_f64_replay());
    assert!(lossy_artifact.validation_handoff_ready);
    assert!(lossy_artifact.used_lossy_float_adapter_route());

    let preview = MeshArtifactManifest::sdf_surface_nets_preview("preview", 3, 1).report();
    preview.validate().unwrap();
    assert!(preview.preview_only);
    assert!(!preview.validation_handoff_ready);
    assert!(
        preview
            .blockers
            .contains(&MeshArtifactBlocker::PreviewOrExportOnly)
    );
    assert!(
        preview
            .blockers
            .contains(&MeshArtifactBlocker::PreviewOrExportSource)
    );
    assert!(
        preview
            .blockers
            .contains(&MeshArtifactBlocker::MissingExactCoordinateReplay)
    );
    assert!(
        preview
            .blockers
            .contains(&MeshArtifactBlocker::MissingExactTopologyReplay)
    );
    let mut preview_missing_blocker = preview.clone();
    preview_missing_blocker
        .blockers
        .retain(|blocker| *blocker != MeshArtifactBlocker::MissingExactCoordinateReplay);
    assert!(
        preview_missing_blocker
            .validate()
            .is_err_and(|error| error.is_missing_exact_coordinate_replay_blocker())
    );
    let mut duplicate_preview_blocker = preview.clone();
    duplicate_preview_blocker
        .blockers
        .push(MeshArtifactBlocker::PreviewOrExportOnly);
    assert!(
        duplicate_preview_blocker
            .validate()
            .is_err_and(|error| error.is_duplicate_preview_or_export_only_blocker())
    );

    let brep_triangle_handoff = |face| {
        MeshArtifactManifest::brep_exact_triangle_handoff(
            "brep exact triangle handoff",
            1,
            vec![
                MeshArtifactVertexRecord::certified_derived_exact(0),
                MeshArtifactVertexRecord::certified_derived_exact(1),
                MeshArtifactVertexRecord::certified_derived_exact(2),
            ],
            vec![face],
        )
    };

    let repeated_vertex_handoff = brep_triangle_handoff(
        MeshArtifactFaceRecord::derived_exact_surface_handoff(0, vec![0, 1, 1]),
    )
    .report();
    repeated_vertex_handoff.validate().unwrap();
    assert!(!repeated_vertex_handoff.validation_handoff_ready);
    assert!(!repeated_vertex_handoff.topology_validation_replay_ready);
    assert!(
        repeated_vertex_handoff
            .blockers
            .contains(&MeshArtifactBlocker::FaceRepeatedVertex),
        "{repeated_vertex_handoff:?}"
    );

    let mut missing_vertex_record_manifest = brep_triangle_handoff(
        MeshArtifactFaceRecord::derived_exact_surface_handoff(0, vec![0, 1, 2]),
    );
    missing_vertex_record_manifest.declared_vertex_count += 1;
    let missing_vertex_record = missing_vertex_record_manifest.report();
    missing_vertex_record.validate().unwrap();
    assert!(!missing_vertex_record.validation_handoff_ready);
    assert!(!missing_vertex_record.coordinates_exact_replay_ready);
    assert!(
        missing_vertex_record
            .blockers
            .contains(&MeshArtifactBlocker::MissingOrMismatchedVertexRecords),
        "{missing_vertex_record:?}"
    );

    let mut stale_face_index_manifest = brep_triangle_handoff(
        MeshArtifactFaceRecord::derived_exact_surface_handoff(0, vec![0, 1, 2]),
    );
    stale_face_index_manifest.faces[0].index = 1;
    let stale_face_index = stale_face_index_manifest.report();
    stale_face_index.validate().unwrap();
    assert!(!stale_face_index.validation_handoff_ready);
    assert!(!stale_face_index.topology_validation_replay_ready);
    assert!(
        stale_face_index
            .blockers
            .contains(&MeshArtifactBlocker::FaceIndexMismatch),
        "{stale_face_index:?}"
    );
}

#[test]
fn exact_mesh_handoff_package_domains_are_publicly_replayable() {
    let solid = tetra([0, 0, 0]);
    let package = solid.handoff_package().unwrap();

    package.validate_internal().unwrap();
    package.validate_against_mesh(&solid).unwrap();
    assert!(package.has_domain(ExactMeshConsumerDomain::Surface));
    assert!(package.has_domain(ExactMeshConsumerDomain::Solid));
    assert!(package.has_domain(ExactMeshConsumerDomain::ApproximateF64View));
    assert_eq!(
        package
            .require_preferred_exact_geometry_domain_against_mesh(&solid)
            .unwrap(),
        ExactMeshConsumerDomain::Solid
    );
    let preferred = package
        .preferred_exact_geometry_report_against_mesh(&solid)
        .unwrap();
    assert_eq!(preferred.domain(), ExactMeshConsumerDomain::Solid);
    assert_eq!(preferred.audit(), &package.audit);

    let mut invalid_readiness_package = package.clone();
    invalid_readiness_package.readiness.closed_manifold = false;
    assert!(
        invalid_readiness_package
            .validate_internal()
            .is_err_and(|error| error.is_internal_mismatch("readiness"))
    );
    assert!(
        invalid_readiness_package
            .validate_against_mesh(&solid)
            .is_err()
    );

    let mut understated_surface_readiness = package.readiness.clone();
    understated_surface_readiness.surface_handoff_ready = false;
    assert!(understated_surface_readiness.validate().is_err());

    let mut stale_face_plane_readiness = package.readiness.clone();
    stale_face_plane_readiness.retained_face_planes -= 1;
    assert!(stale_face_plane_readiness.validate().is_err());

    let mut missing_bounds_readiness = package.readiness.clone();
    missing_bounds_readiness.retained_mesh_bounds = false;
    assert!(missing_bounds_readiness.validate().is_err());

    let mut invalid_surface_package = package.clone();
    invalid_surface_package
        .surface
        .as_mut()
        .unwrap()
        .nonempty_topology = false;
    assert!(
        invalid_surface_package
            .validate_internal()
            .is_err_and(|error| error.is_internal_mismatch("surface"))
    );
    assert!(
        invalid_surface_package
            .validate_against_mesh(&solid)
            .is_err()
    );

    let mut invalid_solid_package = package.clone();
    invalid_solid_package
        .solid
        .as_mut()
        .unwrap()
        .retained_face_planes -= 1;
    assert!(
        invalid_solid_package
            .validate_internal()
            .is_err_and(|error| error.is_internal_mismatch("solid"))
    );

    let mut invalid_view_package = package.clone();
    invalid_view_package
        .approximate_f64_view
        .as_mut()
        .unwrap()
        .exported_coordinates += 1;
    assert!(
        invalid_view_package
            .validate_internal()
            .is_err_and(|error| error.is_internal_mismatch("approximate_f64_view"))
    );

    let summary = package.domain_summary();
    summary.validate_against_mesh(&package, &solid).unwrap();
    assert_eq!(
        summary.preferred_exact_geometry_domain(),
        Some(ExactMeshConsumerDomain::Solid)
    );
    summary
        .require_domain_against_mesh(
            &package,
            &solid,
            ExactMeshConsumerDomain::ApproximateF64View,
        )
        .unwrap();
    assert!(summary.require_lossy_adapter().is_ok());
    assert!(summary.require_closed_volume().is_ok());

    let mut invalid_summary = summary.clone();
    invalid_summary
        .available_domains
        .push(ExactMeshConsumerDomain::Surface);
    assert!(
        invalid_summary
            .validate()
            .is_err_and(|error| error.is_summary_mismatch("available_domains"))
    );
    assert!(invalid_summary.validate_against_package(&package).is_err());

    let mut contradictory_summary = summary.clone();
    contradictory_summary.exact_geometry_domains.clear();
    assert!(
        contradictory_summary
            .validate()
            .is_err_and(|error| error.is_summary_mismatch("exact_geometry_domains"))
    );

    let stale_source = tetra([2, 0, 0]);
    assert!(package.validate_against_mesh(&stale_source).is_err());
    assert!(
        package
            .domain_report_against_mesh(&stale_source, ExactMeshConsumerDomain::Solid)
            .is_err()
    );
    assert!(
        summary
            .validate_against_mesh(&package, &stale_source)
            .is_err()
    );

    let mut stale_summary = summary.clone();
    stale_summary.lossy_adapter_count = 0;
    assert!(stale_summary.validate_against_package(&package).is_err());

    let view = package
        .approximate_f64_view
        .as_ref()
        .expect("closed exact mesh package should retain approximate view");
    view.validate_against_mesh(&solid).unwrap();
    let mut stale_view = view.clone();
    stale_view.positions[0] = 42.0;
    assert!(stale_view.validate_against_mesh(&solid).is_err());
    let mut relabeled_view = view.clone();
    relabeled_view.lossy_view = false;
    assert!(relabeled_view.validate_against_mesh(&solid).is_err());

    let open_surface = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_package = open_surface.handoff_package().unwrap();
    let open_readiness = &open_package.readiness;
    assert!(open_readiness.surface_handoff_ready);
    assert!(!open_readiness.solid_handoff_ready);
    assert!(open_readiness.boundary_allowed);

    assert!(open_package.has_domain(ExactMeshConsumerDomain::Surface));
    assert!(!open_package.has_domain(ExactMeshConsumerDomain::Solid));
    assert!(open_package.has_domain(ExactMeshConsumerDomain::ApproximateF64View));
    assert_eq!(
        open_package.preferred_exact_geometry_domain(),
        Some(ExactMeshConsumerDomain::Surface)
    );
    assert!(
        open_package
            .require_domain(ExactMeshConsumerDomain::Solid)
            .is_err()
    );
    let open_summary = open_package.domain_summary();
    assert!(!open_summary.closed_volume_ready);
    assert!(open_summary.require_closed_volume().is_err());

    let lossy = ExactMesh::from_f64_triangles_with_policy(
        &[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let lossy_package = lossy.handoff_package().unwrap();
    let lossy_readiness = &lossy_package.readiness;
    assert!(lossy_readiness.surface_handoff_ready);
    assert!(!lossy_readiness.solid_handoff_ready);
    assert!(!lossy_readiness.exact_source);

    assert!(lossy_package.has_domain(ExactMeshConsumerDomain::Surface));
    assert!(!lossy_package.has_domain(ExactMeshConsumerDomain::Solid));
    assert!(lossy_package.has_domain(ExactMeshConsumerDomain::ApproximateF64View));
    assert_eq!(
        lossy_package
            .require_preferred_exact_geometry_domain()
            .unwrap(),
        ExactMeshConsumerDomain::Surface
    );
}

#[test]
fn exact_affine_orthogonal_solid_boolean_is_publicly_replayable() {
    let left = skew_affine_box([0, 0, 0], [1, 1, 1]);
    let right = skew_affine_box([2, 0, 0], [3, 1, 1]);
    let mut workspace = ExactBooleanWorkspace::new(&left, &right);

    let operation = ExactBooleanOperation::Union;
    let result = workspace
        .materialize(ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED))
        .unwrap();
    assert!(
        result.is_certified_shortcut_for(operation),
        "{operation:?}: {result:?}"
    );
    result.validate().unwrap();
    result.validate_against_sources(&left, &right).unwrap();
    let mut stale_output = result.clone();
    stale_output.mesh = left.clone();
    assert_ne!(
        stale_output.freshness_against_sources(&left, &right),
        ExactReportFreshness::Current,
        "{operation:?}: {stale_output:?}"
    );
    assert!(result.mesh.facts().mesh.closed_manifold);
}

#[test]
fn affine_orthogonal_solid_recovers_multi_cell_basis_without_sampling_limits() {
    let left_axis = axis_aligned_l_solid([0, 0, 0]);
    let right_axis = axis_aligned_l_solid([1, 0, 0]);
    let left = skew_affine_mesh_from_axis_aligned(&left_axis, "test skew affine left L solid");
    let right = skew_affine_mesh_from_axis_aligned(&right_axis, "test skew affine right L solid");

    assert!(left.vertices().len() > 8);
    assert!(right.vertices().len() > 8);

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let preflight = exact_boolean_evaluation(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
        )
        .preflight
        .clone();
        assert!(
            preflight.is_certified_arrangement_cell_complex(),
            "{operation:?}: {preflight:?}"
        );
        assert!(preflight.blocker.is_none(), "{operation:?}: {preflight:?}");
        preflight.validate().unwrap();
        preflight.validate_against_sources(&left, &right).unwrap();

        let result = exact_boolean_result(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert!(
            result.is_arrangement_cell_complex_shortcut_for(operation),
            "{operation:?}: {result:?}"
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert!(result.mesh.facts().mesh.closed_manifold);
    }
}

#[test]
fn exact_axis_aligned_orthogonal_solid_boolean_is_publicly_replayable() {
    let left = axis_aligned_l_solid([0, 0, 0]);
    let right = axis_aligned_box([1, 0, 0], [3, 1, 1]);
    let separated_right = axis_aligned_box([5, 0, 0], [6, 1, 1]);

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let disjoint_replay = exact_boolean_result(
            &left,
            &separated_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert!(
            disjoint_replay.is_certified_shortcut_for(operation),
            "{operation:?}: {disjoint_replay:?}"
        );

        let result = exact_boolean_result(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert!(
            result.is_arrangement_cell_complex_shortcut_for(operation),
            "{operation:?}: {result:?}"
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        let mut stale_output = result.clone();
        stale_output.mesh = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 1, 0, 0, 0, 1, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        assert!(
            stale_output.validate().is_err(),
            "{operation:?}: {stale_output:?}"
        );
        assert_ne!(
            stale_output.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current,
            "{operation:?}: {stale_output:?}"
        );
        assert_eq!(
            result.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
        assert!(result.mesh.facts().mesh.closed_manifold);
    }
}

#[test]
fn axis_aligned_orthogonal_solid_accepts_face_fan_triangulated_box() {
    let fan_box = face_fan_box();
    let cutter = axis_aligned_box([1, 0, 0], [3, 2, 2]);

    let result = exact_boolean_result(
        &fan_box,
        &cutter,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        ),
    );
    assert!(
        result.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Intersection),
        "{result:?}"
    );
    result.validate().unwrap();
    result.validate_against_sources(&fan_box, &cutter).unwrap();
    assert_eq!(
        result.freshness_against_sources(&fan_box, &cutter),
        ExactReportFreshness::Current
    );
    assert!(result.mesh.facts().mesh.closed_manifold);
}

#[test]
fn axis_aligned_orthogonal_solid_materializes_multiple_cavities() {
    let outer = axis_aligned_box([0, 0, 0], [5, 2, 2]);
    let left_cavity = axis_aligned_box([1, 1, 1], [2, 2, 2]);
    let right_cavity = axis_aligned_box([3, 1, 1], [4, 2, 2]);
    let cavities = combine_exact_meshes(
        &left_cavity,
        &right_cavity,
        "test disjoint orthogonal cavity cutters",
    );

    let result = exact_boolean_result(
        &outer,
        &cavities,
        ExactBooleanRequest::new(ExactBooleanOperation::Difference, ValidationPolicy::CLOSED),
    );
    assert!(
        result.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Difference),
        "{result:?}"
    );
    result.validate().unwrap();
    result.validate_against_sources(&outer, &cavities).unwrap();
    assert_eq!(
        result.freshness_against_sources(&outer, &cavities),
        ExactReportFreshness::Current
    );
    assert!(result.mesh.facts().mesh.closed_manifold);
}

#[test]
fn affine_orthogonal_solid_recovers_face_fan_basis_from_cell_edges() {
    let fan_box = skew_affine_mesh_from_axis_aligned(
        &face_fan_box(),
        "test skew affine face-fan orthogonal box",
    );
    let cutter = skew_affine_box([1, 0, 0], [3, 2, 2]);

    let result = exact_boolean_result(
        &fan_box,
        &cutter,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        ),
    );
    assert!(
        result.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Intersection),
        "{result:?}"
    );
    result.validate().unwrap();
    result.validate_against_sources(&fan_box, &cutter).unwrap();
    assert_eq!(
        result.freshness_against_sources(&fan_box, &cutter),
        ExactReportFreshness::Current
    );
    assert!(result.mesh.facts().mesh.closed_manifold);
}

#[test]
fn exact_coplanar_volumetric_cell_evidence_is_retained_by_public_evaluation() {
    let left_a = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let left_b = tetra_from_corners([20, 0, 0], [22, 0, 0], [20, 2, 0], [20, 0, 2]);
    let left = combine_exact_meshes(&left_a, &left_b, "test disconnected same-side fixture");
    let right = tetra_from_corners([0, 0, 0], [8, 0, 0], [0, 8, 0], [0, 0, 8]);

    let evaluation = exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    evaluation.validate().unwrap();
    let preflight = &evaluation.preflight;
    assert!(
        preflight.blocker.is_some() || preflight.is_certified_arrangement_cell_complex(),
        "{preflight:?}"
    );
    assert!(
        evaluation
            .certifications
            .arrangement_attempt
            .as_ref()
            .is_some_and(|attempt| {
                attempt.operation == evaluation.request.operation
                    && attempt.resolves_requested_volume_ownership()
            })
            || evaluation_materializes_arrangement_cell_complex(&evaluation)
            || preflight.coplanar_volumetric_evidence.is_some(),
        "{evaluation:?}"
    );
    preflight
        .validate_against_sources_with_validation(&left, &right, ValidationPolicy::CLOSED)
        .unwrap();
    if evaluation
        .certifications
        .winding_readiness
        .coplanar_volumetric_evidence
        .is_some()
    {
        assert_eq!(
            preflight.coplanar_volumetric_evidence,
            evaluation
                .certifications
                .winding_readiness
                .coplanar_volumetric_evidence
        );
    } else {
        assert!(preflight.coplanar_volumetric_evidence.is_some());
    }
    let report = evaluation
        .preflight
        .coplanar_volumetric_evidence
        .as_ref()
        .expect("coplanar volumetric blocker should retain source-aware evidence");
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();
    assert!(report.obstacle.requires_coplanar_volumetric_cells());
    assert!(report.positive_area_coplanar_overlapping_pairs > 0);
    assert!(report.same_side_coplanar_overlapping_pairs > 0);

    let mut stale_counts = report.clone();
    stale_counts.retained_face_pair_count += 1;
    assert!(stale_counts.validate().is_err());

    let separated_right = tetra([10, 0, 0]);
    assert!(
        report
            .validate_against_sources(&left, &separated_right)
            .is_err()
    );
}

#[test]
fn exact_closed_convex_boolean_is_publicly_replayable() {
    let left = tetra_from_corners([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
    let right = tetra_from_corners([2, 2, 2], [8, -1, -1], [-1, 8, -1], [3, 2, 0]);
    let stale_open_right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    for (operation, predicate) in [
        (
            ExactBooleanOperation::Union,
            ExactBooleanResult::is_arrangement_cell_complex_shortcut_for
                as fn(&ExactBooleanResult, ExactBooleanOperation) -> bool,
        ),
        (
            ExactBooleanOperation::Intersection,
            ExactBooleanResult::is_certified_shortcut_for
                as fn(&ExactBooleanResult, ExactBooleanOperation) -> bool,
        ),
        (
            ExactBooleanOperation::Difference,
            ExactBooleanResult::is_certified_shortcut_for
                as fn(&ExactBooleanResult, ExactBooleanOperation) -> bool,
        ),
    ] {
        let result = exact_boolean_result(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert!(predicate(&result, operation), "{operation:?}: {result:?}");
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert_eq!(
            result.freshness_against_sources(&left, &stale_open_right),
            ExactReportFreshness::SourceReplayMismatch
        );
        let mut stale_output = result.clone();
        stale_output.mesh = left.clone();
        assert!(
            stale_output.validate().is_err(),
            "{operation:?}: {stale_output:?}"
        );
        assert_eq!(
            stale_output.freshness_against_sources(&left, &right),
            ExactReportFreshness::StaleRegionFacts,
            "{operation:?}: {stale_output:?}"
        );
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
        assert!(result.mesh.facts().mesh.closed_manifold);
    }

    let separated_left = tetra_from_corners([0, 0, 0], [2, 0, 0], [0, 2, 0], [0, 0, 2]);
    let separated_right = tetra_from_corners([1, 1, 1], [3, 1, 1], [1, 3, 1], [1, 1, 3]);
    let separated = exact_boolean_result(
        &separated_left,
        &separated_right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        ),
    );
    assert!(
        separated.is_certified_shortcut_for(ExactBooleanOperation::Intersection),
        "{separated:?}"
    );
    separated.validate().unwrap();
    separated
        .validate_against_sources(&separated_left, &separated_right)
        .unwrap();
    let mut stale_separated_output = separated.clone();
    stale_separated_output.mesh = separated_left.clone();
    assert!(
        stale_separated_output.validate().is_err(),
        "{stale_separated_output:?}"
    );
    assert_eq!(
        stale_separated_output.freshness_against_sources(&separated_left, &separated_right),
        ExactReportFreshness::StaleStatusEvidence,
        "{stale_separated_output:?}"
    );
    separated
        .validate_operation_against_sources(
            &separated_left,
            &separated_right,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    let separated_evaluation = exact_boolean_evaluation(
        &separated_left,
        &separated_right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        ),
    );
    separated_evaluation.validate().unwrap();
    let mut relabeled_winding_report = separated_evaluation.clone();
    relabeled_winding_report
        .certifications
        .closed_winding_left_in_right
        .target_closed = false;
    assert_eq!(
        relabeled_winding_report.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
        "{relabeled_winding_report:?}"
    );
    let dispatched = exact_boolean_result(
        &separated_left,
        &separated_right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        ),
    );
    assert_eq!(dispatched.kind, separated.kind);

    let contained_on_boundary = tetra_from_corners([1, 1, 0], [2, 1, 0], [1, 2, 0], [1, 1, 1]);
    let container = tetra_from_corners([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let containment = exact_boolean_result(
        &contained_on_boundary,
        &container,
        ExactBooleanRequest::new(ExactBooleanOperation::Difference, ValidationPolicy::CLOSED),
    );
    assert!(
        containment.is_certified_shortcut_for(ExactBooleanOperation::Difference),
        "{containment:?}"
    );
    containment.validate().unwrap();
    containment
        .validate_against_sources(&contained_on_boundary, &container)
        .unwrap();
    let mut stale_containment_output = containment.clone();
    stale_containment_output.mesh = container.clone();
    assert!(
        stale_containment_output.validate().is_err(),
        "{stale_containment_output:?}"
    );
    assert_eq!(
        stale_containment_output.freshness_against_sources(&contained_on_boundary, &container),
        ExactReportFreshness::StaleRegionFacts,
        "{stale_containment_output:?}"
    );
    containment
        .validate_operation_against_sources(
            &contained_on_boundary,
            &container,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert!(containment.mesh.triangles().is_empty());

    let axis_overlap = exact_boolean_result(
        &axis_aligned_box([0, 0, 0], [2, 2, 2]),
        &axis_aligned_box([1, 1, 1], [3, 3, 3]),
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(
        axis_overlap.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Union),
        "{axis_overlap:?}"
    );
}

#[test]
fn exact_full_face_adjacent_union_is_publicly_replayable() {
    let left = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let right = tetra_from_corners([0, 0, 0], [0, 4, 0], [4, 0, 0], [0, 0, -4]);

    let result = assert_public_full_face_adjacent_union(&left, &right, 1, 0);

    let mut invalid_shared_faces = exact_adjacent_union_completion_report!(
        &left,
        &right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    invalid_shared_faces.full_face_shared_faces = 0;
    assert_eq!(
        invalid_shared_faces.freshness_against_sources(&left, &right),
        ExactReportFreshness::StaleStatusEvidence
    );

    let mut invalid_output = result.clone();
    invalid_output.mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(
        invalid_output.freshness_against_sources(&left, &right),
        ExactReportFreshness::StaleRegionFacts
    );

    let separated_right = tetra_from_corners([20, 0, 0], [24, 0, 0], [20, 4, 0], [20, 0, 4]);
    assert_eq!(
        result.freshness_against_sources(&left, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );
}

#[test]
fn full_face_adjacent_union_accepts_interior_subdivided_shared_face() {
    let left = tetra_from_corners([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
    let right = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 6, 0, 0, 0, 6, 0, 2, 1, 0, 1, 2, 0, 0, 0, -6],
        &[
            0, 1, 3, //
            0, 3, 4, //
            1, 4, 3, //
            1, 2, 4, //
            2, 0, 4, //
            0, 5, 1, //
            1, 5, 2, //
            2, 5, 0,
        ],
    )
    .unwrap();

    assert_public_full_face_adjacent_union(&left, &right, 0, 1);
}

#[test]
fn full_face_adjacent_union_refines_side_faces_for_boundary_subdivided_shared_face() {
    let left = tetra_from_corners([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
    let right = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 6, 0, 0, 0, 6, 0, 3, 0, 0, 0, 0, -6],
        &[
            0, 3, 2, //
            3, 1, 2, //
            0, 4, 3, //
            3, 4, 1, //
            1, 4, 2, //
            2, 4, 0,
        ],
    )
    .unwrap();

    assert_public_full_face_adjacent_union(&left, &right, 0, 1);

    let report = exact_adjacent_union_completion_report!(
        &left,
        &right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(report.is_certified_full_face());
    assert_eq!(report.full_face_shared_faces, 0);
    assert_eq!(report.full_face_shared_patches, 1);
    assert!(!report.stronger_kernel_available);
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        report.freshness_against_sources(&left, &right),
        ExactReportFreshness::Current
    );
}

#[test]
fn full_face_adjacent_union_uses_polygon_patch_for_dual_subdivided_shared_face() {
    let left = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 6, 0, 0, 0, 6, 0, 2, 2, 0, 0, 0, 6],
        &[
            0, 3, 1, //
            1, 3, 2, //
            2, 3, 0, //
            0, 1, 4, //
            1, 2, 4, //
            2, 0, 4,
        ],
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 6, 0, 0, 0, 6, 0, 1, 1, 0, 0, 0, -6],
        &[
            0, 1, 3, //
            1, 2, 3, //
            2, 0, 3, //
            0, 4, 1, //
            1, 4, 2, //
            2, 4, 0,
        ],
    )
    .unwrap();

    assert_public_full_face_adjacent_union(&left, &right, 0, 1);
}

#[test]
fn full_face_adjacent_union_accepts_dual_boundary_subdivided_shared_face() {
    let left = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 6, 0, 0, 0, 6, 0, 3, 0, 0, 0, 0, 6],
        &[
            0, 2, 3, //
            3, 2, 1, //
            0, 3, 4, //
            3, 1, 4, //
            1, 2, 4, //
            2, 0, 4,
        ],
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 6, 0, 0, 0, 6, 0, 0, 3, 0, 0, 0, -6],
        &[
            0, 1, 3, //
            3, 1, 2, //
            0, 4, 1, //
            1, 4, 2, //
            2, 4, 3, //
            3, 4, 0,
        ],
    )
    .unwrap();

    assert_public_full_face_adjacent_union(&left, &right, 0, 1);

    let report = exact_adjacent_union_completion_report!(
        &left,
        &right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(report.is_certified_full_face());
    assert_eq!(report.full_face_shared_faces, 0);
    assert_eq!(report.full_face_shared_patches, 1);
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();
}

fn tetra_with_subdivided_base() -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[0, 0, 0, 10, 0, 0, 0, 10, 0, 0, 0, 10, 5, 0, 0],
        &[
            0, 2, 4, //
            4, 2, 1, //
            0, 4, 3, //
            4, 1, 3, //
            1, 2, 3, //
            2, 0, 3,
        ],
    )
    .unwrap()
}

fn square_pyramid_with_base() -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0, 5, 5, 10],
        &[
            0, 3, 2, //
            0, 2, 1, //
            0, 1, 4, //
            1, 2, 4, //
            2, 3, 4, //
            3, 0, 4,
        ],
    )
    .unwrap()
}

fn downward_square_pyramid_with_base(min: [i64; 2], max: [i64; 2], z: i64) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            min[0], min[1], 0, max[0], min[1], 0, max[0], max[1], 0, min[0], max[1], 0, min[0],
            min[1], z,
        ],
        &[
            0, 1, 2, //
            0, 2, 3, //
            0, 4, 1, //
            1, 4, 2, //
            2, 4, 3, //
            3, 4, 0,
        ],
    )
    .unwrap()
}

#[test]
fn adjacent_union_completion_boolean_is_publicly_replayable() {
    let left_a = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let left_b = tetra_from_corners([10, 0, 0], [12, 0, 0], [10, 2, 0], [10, 0, 2]);
    let left = combine_exact_meshes(&left_a, &left_b, "test disconnected full-face fixture");
    let right = tetra_from_corners([0, 0, 0], [0, 4, 0], [4, 0, 0], [0, 0, -4]);
    let separated_right = tetra_from_corners([20, 0, 0], [24, 0, 0], [20, 4, 0], [20, 0, 4]);

    let report = exact_adjacent_union_completion_report!(
        &left,
        &right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(report.is_certified_full_face());
    assert!(report.is_certified());
    assert!(report.full_face_shared_faces + report.full_face_shared_patches > 0);
    assert_eq!(report.contained_faces, 0);
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        report.freshness_against_sources(&left, &right),
        ExactReportFreshness::Current
    );
    assert_eq!(
        report.freshness_against_sources(&left, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );

    let result = exact_boolean_result(
        &left,
        &right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(
        result.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Union),
        "{result:?}"
    );
    result.validate().unwrap();
    result.validate_against_sources(&left, &right).unwrap();

    assert_eq!(
        result.freshness_against_sources(&left, &right),
        ExactReportFreshness::Current
    );
    let mut stale_output = result.clone();
    stale_output.mesh = left.clone();
    assert!(stale_output.validate().is_err(), "{stale_output:?}");
    assert_eq!(
        stale_output.freshness_against_sources(&left, &right),
        ExactReportFreshness::StaleRegionFacts,
        "{stale_output:?}"
    );
    assert_eq!(
        result.freshness_against_sources(&left, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );
    result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert!(result.mesh.facts().mesh.closed_manifold);

    let intersection_report = exact_adjacent_union_completion_report!(
        &left,
        &right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        ),
    );
    assert!(intersection_report.is_not_union());
    assert!(!intersection_report.is_certified());
    intersection_report.validate().unwrap();
    let mut stale_intersection_report = intersection_report.clone();
    stale_intersection_report.retained_face_pairs = 1;
    stale_intersection_report.retained_events = 1;
    stale_intersection_report.blocker.candidate_pairs = 1;
    assert!(stale_intersection_report.validate().is_err());

    let axis_left = axis_aligned_box([0, 0, 0], [1, 1, 1]);
    let axis_right = axis_aligned_box([1, 0, 0], [2, 1, 1]);
    let axis_report = exact_adjacent_union_completion_report!(
        &axis_left,
        &axis_right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(axis_report.is_axis_aligned_box_pair());
    axis_report.validate().unwrap();
    let mut stale_axis_report = axis_report.clone();
    stale_axis_report.retained_face_pairs = 1;
    stale_axis_report.retained_events = 1;
    stale_axis_report.blocker.candidate_pairs = 1;
    assert!(stale_axis_report.validate().is_err());

    let axis_replay = exact_boolean_result(
        &axis_left,
        &axis_right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(
        axis_replay.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Union),
        "{axis_replay:?}"
    );
    axis_replay
        .validate_operation_against_sources(
            &axis_left,
            &axis_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let crossing_right = tetra_from_corners([1, 1, -1], [5, 1, -1], [1, 5, -1], [1, 1, 3]);
    let crossing_report = exact_adjacent_union_completion_report!(
        &left,
        &crossing_right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(crossing_report.has_no_adjacency_certificate());
    assert!(crossing_report.blocker.requires_winding());
    assert!(crossing_report.blocker.candidate_pairs > 0);
    crossing_report.validate().unwrap();
    crossing_report
        .validate_against_sources(&left, &crossing_right)
        .unwrap();

    let mut stale_crossing = crossing_report;
    stale_crossing.blocker.unknown_pairs = 1;
    assert!(stale_crossing.validate().is_err());
}

#[test]
fn exact_open_surface_arrangement_is_publicly_replayable() {
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
    let separated_right = ExactMesh::from_i64_triangles_with_policy(
        &[8, -1, -1, 8, 3, 1, 8, 3, -1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        if !matches!(operation, ExactBooleanOperation::Intersection) {
            let closed_attempt = exact_boolean_arrangement_attempt(
                &left,
                &right,
                ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            );
            assert_eq!(closed_attempt.output_validation, ValidationPolicy::CLOSED);
            assert!(
                closed_attempt.declined_output_validation(),
                "{operation:?}: {closed_attempt:?}"
            );
            let (output_vertices, output_triangles) = closed_attempt.output_counts();
            assert!(output_vertices > 0, "{operation:?}: {closed_attempt:?}");
            assert!(output_triangles > 0, "{operation:?}: {closed_attempt:?}");
            closed_attempt.validate().unwrap();
            closed_attempt
                .validate_against_sources_with_validation(&left, &right, ValidationPolicy::CLOSED)
                .unwrap();
            assert_eq!(
                closed_attempt.freshness_against_sources_with_validation(
                    &left,
                    &right,
                    ValidationPolicy::CLOSED,
                ),
                ExactReportFreshness::Current
            );
            assert_eq!(
                closed_attempt.freshness_against_sources_with_validation(
                    &left,
                    &right,
                    ValidationPolicy::ALLOW_BOUNDARY,
                ),
                ExactReportFreshness::SourceReplayMismatch
            );
        }

        let attempt = exact_boolean_arrangement_attempt(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        );
        assert!(
            attempt.materialized_arrangement_cell_complex_shortcut(),
            "{operation:?}: {attempt:?}"
        );
        assert_eq!(attempt.output_validation, ValidationPolicy::ALLOW_BOUNDARY);
        attempt.validate().unwrap();
        attempt.validate_against_sources(&left, &right).unwrap();
        attempt
            .validate_against_sources_with_validation(
                &left,
                &right,
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        assert_eq!(
            attempt.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert_eq!(
            attempt.freshness_against_sources_with_validation(
                &left,
                &right,
                ValidationPolicy::ALLOW_BOUNDARY,
            ),
            ExactReportFreshness::Current
        );
        assert_eq!(
            attempt.freshness_against_sources_with_validation(
                &left,
                &right,
                ValidationPolicy::CLOSED,
            ),
            ExactReportFreshness::SourceReplayMismatch
        );

        let result = exact_boolean_result(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        let evaluation = exact_boolean_evaluation(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
        );
        evaluation.validate().unwrap();
        let mut stale_preflight_counts = evaluation.clone();
        stale_preflight_counts.preflight.retained_events += 1;
        stale_preflight_counts.preflight.validate().unwrap();
        assert_eq!(
            stale_preflight_counts.validate(),
            Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
            "{operation:?}: {stale_preflight_counts:?}"
        );
        assert!(!result.region_classifications.is_empty());
        assert!(!result.triangulations.is_empty());
        if matches!(operation, ExactBooleanOperation::Intersection) {
            assert!(result.mesh.triangles().is_empty());
        } else {
            assert!(!result.mesh.triangles().is_empty());
        }
        assert_eq!(
            result.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
        if matches!(operation, ExactBooleanOperation::Intersection) {
            let replay = exact_boolean_result(
                &left,
                &right,
                ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
            );
            assert!(
                replay.is_certified_shortcut_for(operation),
                "{operation:?}: {replay:?}"
            );
            replay
                .validate_operation_against_sources(
                    &left,
                    &right,
                    operation,
                    ValidationPolicy::CLOSED,
                    ExactBoundaryBooleanPolicy::Reject,
                )
                .unwrap();
        }
    }

    let union = exact_boolean_result(
        &left,
        &right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    );
    let difference = exact_boolean_result(
        &left,
        &right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    );
    let mut stale_materialization = union.clone();
    stale_materialization.assembly = difference.assembly;
    stale_materialization.mesh = difference.mesh;
    assert!(
        stale_materialization.validate().is_err(),
        "{stale_materialization:?}"
    );
    assert_eq!(
        stale_materialization.freshness_against_sources(&left, &right),
        ExactReportFreshness::StaleStatusEvidence,
        "{stale_materialization:?}"
    );
}

#[test]
fn arrangement_attempt_output_validation_is_publicly_replayable() {
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

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
    ] {
        let closed_attempt = exact_boolean_arrangement_attempt(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        );
        assert_eq!(closed_attempt.output_validation, ValidationPolicy::CLOSED);
        assert!(
            closed_attempt.declined_output_validation(),
            "{operation:?}: {closed_attempt:?}"
        );
        let (output_vertices, output_triangles) = closed_attempt.output_counts();
        assert!(output_vertices > 0, "{operation:?}: {closed_attempt:?}");
        if operation == ExactBooleanOperation::Union {
            assert!(output_triangles > 0, "{operation:?}: {closed_attempt:?}");
        }
        closed_attempt.validate().unwrap();
        closed_attempt
            .validate_against_sources(&left, &right)
            .unwrap();
        closed_attempt
            .validate_against_sources_with_validation(&left, &right, ValidationPolicy::CLOSED)
            .unwrap();
        assert_eq!(
            closed_attempt.freshness_against_sources_with_validation(
                &left,
                &right,
                ValidationPolicy::CLOSED,
            ),
            ExactReportFreshness::Current
        );
        assert_eq!(
            closed_attempt.freshness_against_sources_with_validation(
                &left,
                &right,
                ValidationPolicy::ALLOW_BOUNDARY,
            ),
            ExactReportFreshness::SourceReplayMismatch
        );

        let boundary_attempt = exact_boolean_arrangement_attempt(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        );
        assert_eq!(
            boundary_attempt.output_validation,
            ValidationPolicy::ALLOW_BOUNDARY
        );
        assert!(
            boundary_attempt.materialized_arrangement_cell_complex_shortcut(),
            "{operation:?}: {boundary_attempt:?}"
        );
        boundary_attempt.validate().unwrap();
        boundary_attempt
            .validate_against_sources_with_validation(
                &left,
                &right,
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        assert_eq!(
            boundary_attempt.freshness_against_sources_with_validation(
                &left,
                &right,
                ValidationPolicy::ALLOW_BOUNDARY,
            ),
            ExactReportFreshness::Current
        );
        assert_eq!(
            boundary_attempt.freshness_against_sources_with_validation(
                &left,
                &right,
                ValidationPolicy::CLOSED,
            ),
            ExactReportFreshness::SourceReplayMismatch
        );
    }
}

#[test]
fn exact_selected_region_boolean_is_publicly_replayable() {
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
    let separated_right = ExactMesh::from_i64_triangles_with_policy(
        &[8, -1, -1, 8, 3, 1, 8, 3, -1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let selection = ExactRegionSelection::KeepAll;
    let validation = ValidationPolicy::ALLOW_BOUNDARY;

    let result = exact_boolean_result(
        &left,
        &right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::SelectedRegions(selection),
            validation,
        ),
    );

    result.validate().unwrap();
    result.validate_against_sources(&left, &right).unwrap();
    result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::SelectedRegions(selection),
            validation,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        result.freshness_against_sources(&left, &right),
        ExactReportFreshness::Current
    );
    assert_eq!(
        result.freshness_against_sources(&left, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );
    assert!(!result.region_classifications.is_empty());
    assert!(!result.triangulations.is_empty());
    assert!(!result.assembly.triangles.is_empty());
    assert!(!result.mesh.triangles().is_empty());
    assert_eq!(
        result.mesh.validation_policy(),
        ValidationPolicy::ALLOW_BOUNDARY
    );
    let evaluation = exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::SelectedRegions(selection),
            validation,
        ),
    );
    evaluation.validate().unwrap();
    let mut stale_evaluation_region_fact = evaluation.clone();
    let classification = stale_evaluation_region_fact
        .result
        .as_mut()
        .expect("selected-region evaluation should materialize")
        .region_classifications
        .first_mut()
        .expect("selected-region result should retain region facts");
    classification.plane_face = usize::MAX;
    stale_evaluation_region_fact
        .result
        .as_ref()
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(
        stale_evaluation_region_fact.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
        "{stale_evaluation_region_fact:?}"
    );
    let mut stale_winding_handoff = evaluation.clone();
    stale_winding_handoff
        .certifications
        .winding_readiness
        .retained_events += 1;
    assert!(
        stale_winding_handoff.validate().is_err(),
        "{stale_winding_handoff:?}"
    );

    let mut stale_assembly_source_vertex = result.clone();
    let vertex = stale_assembly_source_vertex
        .assembly
        .first_original_source_vertex_mut()
        .expect("selected-region assembly should retain at least one original source vertex");
    *vertex = usize::MAX;
    stale_assembly_source_vertex.validate().unwrap();
    assert!(
        stale_assembly_source_vertex
            .validate_against_sources(&left, &right)
            .is_err()
    );
    assert_eq!(
        stale_assembly_source_vertex.freshness_against_sources(&left, &right),
        ExactReportFreshness::SourceReplayMismatch
    );

    let keep_left = exact_boolean_result(
        &left,
        &right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::SelectedRegions(ExactRegionSelection::KeepLeft),
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    );
    assert!(
        keep_left
            .validate_operation_against_sources(
                &left,
                &right,
                ExactBooleanOperation::SelectedRegions(ExactRegionSelection::KeepAll),
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .is_err()
    );
    let mut stale_materialization = result.clone();
    stale_materialization.assembly = keep_left.assembly;
    stale_materialization.mesh = keep_left.mesh;
    assert!(
        stale_materialization.validate().is_err(),
        "{stale_materialization:?}"
    );
    assert_eq!(
        stale_materialization.freshness_against_sources(&left, &right),
        ExactReportFreshness::StaleStatusEvidence,
        "{stale_materialization:?}"
    );
}

#[test]
fn exact_coplanar_mesh_overlay_arrangement_is_publicly_replayable() {
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
    let separated_right = ExactMesh::from_i64_triangles_with_policy(
        &[8, 2, 0, 12, 2, 0, 12, 6, 0, 8, 6, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let result = exact_boolean_result(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
        );
        assert!(
            result.is_arrangement_cell_complex_shortcut_for(operation),
            "{operation:?}: {result:?}"
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert!(!result.mesh.triangles().is_empty());
        assert_eq!(
            result.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
        let mut stale_output = result.clone();
        stale_output.mesh = left.clone();
        assert!(
            stale_output.validate().is_err(),
            "{operation:?}: {stale_output:?}"
        );
        assert_eq!(
            stale_output.freshness_against_sources(&left, &right),
            ExactReportFreshness::StaleRegionFacts
        );
    }

    let identical = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let identical_result = exact_boolean_result(
        &identical,
        &identical,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    );
    assert!(
        identical_result.is_certified_shortcut_for(ExactBooleanOperation::Union),
        "{identical_result:?}"
    );
}

#[test]
fn lower_dimensional_regularized_boolean_is_publicly_replayable() {
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
    let disjoint_right = ExactMesh::from_i64_triangles_with_policy(
        &[10, 0, 0, 14, 0, 0, 10, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let closed_right = tetra([0, 0, 0]);

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let preflight = exact_boolean_evaluation(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        )
        .preflight
        .clone();
        assert!(preflight.is_certified_lower_dimensional_regularized_solid());
        preflight.validate().unwrap();
        preflight
            .validate_against_sources_with_validation(&left, &right, ValidationPolicy::CLOSED)
            .unwrap();
        assert!(
            preflight
                .validate_against_sources_with_validation(
                    &left,
                    &closed_right,
                    ValidationPolicy::CLOSED,
                )
                .is_err()
        );

        let evaluation = exact_boolean_evaluation(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        let readiness = evaluation.certifications.winding_readiness.clone();
        let readiness_materialized_lower =
            readiness.is_lower_dimensional_regularized_solid_materialized();
        let readiness_materialized_arrangement = readiness.materializes_arrangement_cell_complex();
        assert!(
            readiness_materialized_lower || readiness_materialized_arrangement,
            "{operation:?}: {readiness:?}"
        );
        assert!(
            readiness.blocker.requires_winding(),
            "{operation:?}: {readiness:?}"
        );
        if readiness_materialized_lower {
            assert_eq!(readiness.retained_face_pairs, 0);
            assert_eq!(readiness.retained_events, 0);
        }
        assert_eq!(readiness.region_count, 0);
        readiness.validate().unwrap();
        if readiness_materialized_lower {
            readiness
                .validate_against_sources_with_validation(&left, &right, ValidationPolicy::CLOSED)
                .unwrap();
            assert_eq!(
                readiness.freshness_against_sources_with_validation(
                    &left,
                    &right,
                    ValidationPolicy::CLOSED,
                ),
                ExactReportFreshness::Current
            );
            assert_eq!(
                readiness.freshness_against_sources_with_validation(
                    &left,
                    &closed_right,
                    ValidationPolicy::CLOSED,
                ),
                ExactReportFreshness::SourceReplayMismatch
            );
        } else {
            assert!(evaluation_materializes_arrangement_cell_complex(
                &evaluation
            ));
            evaluation.validate().unwrap();
            evaluation.validate_against_sources(&left, &right).unwrap();
        }

        let result = exact_boolean_result(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert!(
            result.is_certified_shortcut_for(operation)
                || result.is_arrangement_cell_complex_shortcut_for(operation)
                || result.is_arrangement_cell_complex_materialized_for(operation),
            "{operation:?}: {result:?}"
        );
        assert!(result.mesh.triangles().is_empty());
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert_eq!(
            result.freshness_against_sources(&left, &closed_right),
            ExactReportFreshness::SourceReplayMismatch
        );
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();

        let disjoint_preflight = exact_boolean_evaluation(
            &left,
            &disjoint_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        )
        .preflight
        .clone();
        assert!(
            disjoint_preflight.is_certified_lower_dimensional_regularized_solid(),
            "{operation:?}: {disjoint_preflight:?}"
        );
        disjoint_preflight
            .validate_against_sources_with_validation(
                &left,
                &disjoint_right,
                ValidationPolicy::CLOSED,
            )
            .unwrap();
        let disjoint_readiness = exact_boolean_evaluation(
            &left,
            &disjoint_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        )
        .certifications
        .winding_readiness
        .clone();
        assert!(
            disjoint_readiness.is_lower_dimensional_regularized_solid_materialized(),
            "{operation:?}: {disjoint_readiness:?}"
        );
        let disjoint_result = exact_boolean_result(
            &left,
            &disjoint_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert!(
            disjoint_result.is_certified_shortcut_for(operation),
            "{operation:?}: {disjoint_result:?}"
        );
        assert!(disjoint_result.mesh.triangles().is_empty());
        assert!(disjoint_result.mesh.facts().mesh.closed_manifold);
        disjoint_result
            .validate_operation_against_sources(
                &left,
                &disjoint_right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
    }
}

#[test]
fn mixed_dimensional_regularized_solid_boolean_is_publicly_replayable() {
    let solid = axis_aligned_box([0, 0, 0], [4, 4, 4]);
    let sheet = ExactMesh::from_i64_triangles_with_policy(
        &[1, 1, 1, 3, 1, 1, 1, 3, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let stale_solid = axis_aligned_box([10, 0, 0], [14, 4, 4]);

    for (left, right, stale_left, stale_right, solid_is_left) in [
        (&solid, &sheet, &stale_solid, &sheet, true),
        (&sheet, &solid, &sheet, &stale_solid, false),
    ] {
        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            let result = exact_boolean_result(
                left,
                right,
                ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
            );
            assert!(
                result.is_certified_shortcut_for(operation),
                "{operation:?}: {result:?}"
            );
            result.validate().unwrap();
            result.validate_against_sources(left, right).unwrap();
            assert_eq!(
                result.freshness_against_sources(left, right),
                ExactReportFreshness::Current
            );
            let keeps_solid = matches!(operation, ExactBooleanOperation::Union)
                || (solid_is_left && matches!(operation, ExactBooleanOperation::Difference));
            let expected_stale_freshness = if keeps_solid {
                ExactReportFreshness::SourceReplayMismatch
            } else {
                ExactReportFreshness::Current
            };
            assert_eq!(
                result.freshness_against_sources(stale_left, stale_right),
                expected_stale_freshness
            );
            result
                .validate_operation_against_sources(
                    left,
                    right,
                    operation,
                    ValidationPolicy::CLOSED,
                    ExactBoundaryBooleanPolicy::Reject,
                )
                .unwrap();

            if keeps_solid {
                assert!(result.mesh.facts().mesh.closed_manifold);
                assert!(!result.mesh.triangles().is_empty());
            } else {
                assert!(
                    result.mesh.triangles().is_empty(),
                    "{operation:?}: {result:?}"
                );
            }
        }
    }

    let lower_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let lower_right = ExactMesh::from_i64_triangles_with_policy(
        &[1, -1, -1, 1, 3, 1, 1, 3, -1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let lower_result = exact_boolean_result(
        &lower_left,
        &lower_right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(
        lower_result.is_certified_shortcut_for(ExactBooleanOperation::Union),
        "{lower_result:?}"
    );

    let disjoint_sheet = ExactMesh::from_i64_triangles_with_policy(
        &[10, 0, 0, 14, 0, 0, 10, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    for (left, right, solid_is_left) in [
        (&solid, &disjoint_sheet, true),
        (&disjoint_sheet, &solid, false),
    ] {
        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            let preflight = exact_boolean_evaluation(
                left,
                right,
                ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
            )
            .preflight
            .clone();
            assert!(
                preflight.is_certified_mixed_dimensional_regularized_solid(),
                "{operation:?}: {preflight:?}"
            );
            let readiness = exact_boolean_evaluation(
                left,
                right,
                ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
            )
            .certifications
            .winding_readiness
            .clone();
            assert!(
                readiness.is_mixed_dimensional_regularized_solid_materialized(),
                "{operation:?}: {readiness:?}"
            );
            let result = exact_boolean_result(
                left,
                right,
                ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
            );
            assert!(
                result.is_certified_shortcut_for(operation),
                "{operation:?}: {result:?}"
            );
            result
                .validate_operation_against_sources(
                    left,
                    right,
                    operation,
                    ValidationPolicy::CLOSED,
                    ExactBoundaryBooleanPolicy::Reject,
                )
                .unwrap();
            let keeps_solid = matches!(operation, ExactBooleanOperation::Union)
                || (solid_is_left && matches!(operation, ExactBooleanOperation::Difference));
            assert_eq!(
                result.mesh.triangles().is_empty(),
                !keeps_solid,
                "{operation:?}: {result:?}"
            );

            let boundary_preflight = exact_boolean_evaluation(
                left,
                right,
                ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
            )
            .preflight
            .clone();
            assert!(
                boundary_preflight.is_certified_bounds_disjoint(),
                "{operation:?}: {boundary_preflight:?}"
            );
            let boundary_result = exact_boolean_result(
                left,
                right,
                ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
            );
            assert!(
                boundary_result.is_certified_shortcut_for(operation),
                "{operation:?}: {boundary_result:?}"
            );
            boundary_result
                .validate_operation_against_sources(
                    left,
                    right,
                    operation,
                    ValidationPolicy::ALLOW_BOUNDARY,
                    ExactBoundaryBooleanPolicy::Reject,
                )
                .unwrap();
        }
    }
}

#[test]
fn boundary_touching_policy_boolean_is_publicly_replayable() {
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
    let separated_right = ExactMesh::from_i64_triangles_with_policy(
        &[5, 0, 0, 7, 0, 0, 5, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        assert!(
            exact_boolean_materialize_result(
                &left,
                &right,
                ExactBooleanRequest::with_boundary_policy(
                    operation,
                    ValidationPolicy::ALLOW_BOUNDARY,
                    ExactBoundaryBooleanPolicy::Reject,
                ),
            )
            .is_err()
        );

        let result = exact_boolean_result(
            &left,
            &right,
            ExactBooleanRequest::with_boundary_policy(
                operation,
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            ),
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            )
            .unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert_eq!(
            result.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            )
            .unwrap();
    }

    let closed_left_a = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let closed_left_b = tetra_from_corners([10, 0, 0], [12, 0, 0], [10, 2, 0], [10, 0, 2]);
    let closed_left = combine_exact_meshes(
        &closed_left_a,
        &closed_left_b,
        "test boundary policy closed replay left",
    );
    let closed_right = tetra_from_corners([0, 0, 0], [-4, 0, 0], [0, -4, 0], [0, 0, -4]);
    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let direct = exact_boolean_result(
            &closed_left,
            &closed_right,
            ExactBooleanRequest::with_boundary_policy(
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            ),
        );
        assert!(
            direct.is_arrangement_cell_complex_shortcut_for(operation)
                || direct.is_certified_shortcut_for(operation),
            "{operation:?}: {direct:?}"
        );
        direct
            .validate_operation_against_sources(
                &closed_left,
                &closed_right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            )
            .unwrap();
        let replay = exact_boolean_result(
            &closed_left,
            &closed_right,
            ExactBooleanRequest::with_boundary_policy(
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            ),
        );
        assert!(
            replay.is_arrangement_cell_complex_shortcut_for(operation)
                || replay.is_certified_shortcut_for(operation),
            "{operation:?}: {replay:?}"
        );
        replay
            .validate_operation_against_sources(
                &closed_left,
                &closed_right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            )
            .unwrap();
    }
}

#[test]
fn closed_boundary_touching_regularized_boolean_is_publicly_replayable() {
    let left_a = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let left_b = tetra_from_corners([10, 0, 0], [12, 0, 0], [10, 2, 0], [10, 0, 2]);
    let left = combine_exact_meshes(
        &left_a,
        &left_b,
        "test disconnected closed boundary fixture",
    );
    let right = tetra_from_corners([0, 0, 0], [-4, 0, 0], [0, -4, 0], [0, 0, -4]);
    let separated_right = tetra_from_corners([100, 0, 0], [104, 0, 0], [100, 4, 0], [100, 0, 4]);

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let preflight = exact_boolean_evaluation(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
        )
        .preflight
        .clone();
        assert!(
            preflight.is_certified_arrangement_cell_complex()
                || preflight.is_certified_closed_boundary_touching(),
            "{operation:?}: {preflight:?}"
        );
        assert!(
            preflight.retained_face_pairs > 0,
            "closed boundary-touching request should retain graph evidence: {operation:?}: {preflight:?}"
        );
        preflight.validate().unwrap();
        preflight.validate_against_sources(&left, &right).unwrap();
        if let Some(evidence) = preflight.coplanar_volumetric_evidence.as_ref() {
            evidence.validate().unwrap();
            evidence.validate_against_sources(&left, &right).unwrap();
        }

        let result = exact_boolean_result(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert!(
            result.is_arrangement_cell_complex_shortcut_for(operation)
                || result.is_certified_shortcut_for(operation),
            "{operation:?}: {result:?}"
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        let evaluation = exact_boolean_evaluation(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        evaluation.validate().unwrap();
        let mut relabeled_boundary_report = evaluation.clone();
        relabeled_boundary_report
            .certifications
            .boundary_touching
            .blocker
            .unknown_pairs = 1;
        assert!(
            relabeled_boundary_report.validate().is_err(),
            "{operation:?}: {relabeled_boundary_report:?}"
        );

        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert_eq!(
            result.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
        if operation == ExactBooleanOperation::Intersection {
            let mut stale_output = result.clone();
            stale_output.mesh = left.clone();
            assert!(stale_output.validate().is_err(), "{stale_output:?}");
            let expected_freshness = if result.is_arrangement_cell_complex_shortcut_for(operation) {
                ExactReportFreshness::StaleRegionFacts
            } else {
                ExactReportFreshness::StaleStatusEvidence
            };
            assert_eq!(
                stale_output.freshness_against_sources(&left, &right),
                expected_freshness,
                "{stale_output:?}"
            );
        }
        if operation == ExactBooleanOperation::Difference {
            let mut stale_output = result.clone();
            stale_output.mesh = right.clone();
            assert!(stale_output.validate().is_err(), "{stale_output:?}");
            assert_eq!(
                stale_output.freshness_against_sources(&left, &right),
                ExactReportFreshness::StaleRegionFacts,
                "{stale_output:?}"
            );
        }
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
    }
}

#[test]
fn closed_no_volume_overlap_regularized_boolean_is_publicly_replayable() {
    let left_a = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let left_b = tetra_from_corners([10, 0, 0], [12, 0, 0], [10, 2, 0], [10, 0, 2]);
    let left = combine_exact_meshes(
        &left_a,
        &left_b,
        "test disconnected positive-area boundary fixture",
    );
    let right = tetra_from_corners([2, 0, 0], [6, 0, 0], [2, 4, 0], [2, 0, -4]);
    let separated_right = tetra_from_corners([20, 0, 0], [24, 0, 0], [20, 4, 0], [20, 0, -4]);

    let mut retained_evidence = None;

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let preflight = exact_boolean_evaluation(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
        )
        .preflight
        .clone();
        if operation == ExactBooleanOperation::Union {
            assert!(
                preflight.is_certified_arrangement_cell_complex(),
                "{operation:?}: {preflight:?}"
            );
        } else {
            assert!(
                preflight.is_certified_closed_boundary_touching(),
                "{operation:?}: {preflight:?}"
            );
        }
        assert!(
            preflight.retained_face_pairs > 0,
            "positive-area no-volume shortcut should retain graph evidence: {operation:?}: {preflight:?}"
        );
        let evidence = preflight.coplanar_volumetric_evidence.as_ref().expect(
            "positive-area no-volume shortcut should retain source-aware boundary-only evidence",
        );
        evidence.validate().unwrap();
        evidence.validate_against_sources(&left, &right).unwrap();
        assert!(evidence.positive_area_coplanar_overlapping_pairs > 0);
        if let Some(retained_evidence) = retained_evidence.as_ref() {
            assert_eq!(
                evidence, retained_evidence,
                "{operation:?}: positive-area no-volume shortcut should retain stable source-aware boundary-only evidence"
            );
        } else {
            retained_evidence = Some(evidence.clone());
        }
        preflight.validate().unwrap();
        preflight.validate_against_sources(&left, &right).unwrap();

        if matches!(
            operation,
            ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
        ) {
            let readiness = exact_boolean_evaluation(
                &left,
                &right,
                ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
            )
            .certifications
            .winding_readiness
            .clone();
            assert!(
                readiness.is_closed_boundary_touching_materialized(),
                "{operation:?}: {readiness:?}"
            );
            assert_eq!(
                readiness.coplanar_volumetric_evidence.as_ref(),
                retained_evidence.as_ref(),
                "{operation:?}: no-volume readiness should retain consumed source-aware evidence"
            );
            readiness.validate().unwrap();
            readiness.validate_against_sources(&left, &right).unwrap();
            let evaluation = exact_boolean_evaluation(
                &left,
                &right,
                ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
            );
            evaluation.validate().unwrap();
            let mut cleared_handoff_evidence = evaluation.clone();
            cleared_handoff_evidence
                .certifications
                .winding_readiness
                .coplanar_volumetric_evidence = None;
            assert!(
                cleared_handoff_evidence.validate().is_err(),
                "{operation:?}: {cleared_handoff_evidence:?}"
            );
        }

        let result = exact_boolean_result(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert!(
            if operation == ExactBooleanOperation::Union {
                result.is_arrangement_cell_complex_shortcut_for(operation)
            } else {
                result.is_certified_shortcut_for(operation)
            },
            "{operation:?}: {result:?}"
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        if operation == ExactBooleanOperation::Union {
            let mut stale_output = result.clone();
            stale_output.mesh = left.clone();
            assert!(stale_output.validate().is_err(), "{stale_output:?}");
            assert_eq!(
                stale_output.freshness_against_sources(&left, &right),
                ExactReportFreshness::StaleRegionFacts,
                "{stale_output:?}"
            );
        }
        assert_eq!(
            result.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
        if operation == ExactBooleanOperation::Union {
            assert_eq!(
                result.mesh.triangles().len(),
                left.triangles().len() + right.triangles().len()
            );
            assert!(result.mesh.facts().mesh.closed_manifold);
        }
    }
}

#[test]
fn closed_winding_shortcuts_are_publicly_replayable() {
    let separated_left_a = tetra_from_corners([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
    let separated_left_b = tetra_from_corners([10, 0, 0], [11, 0, 0], [10, 1, 0], [10, 0, 1]);
    let separated_left = combine_exact_meshes(
        &separated_left_a,
        &separated_left_b,
        "test disconnected closed winding separated fixture",
    );
    let separated_right = tetra_from_corners([5, 0, 0], [6, 0, 0], [5, 1, 0], [5, 0, 1]);
    let intersecting_right = tetra_from_corners([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let result = exact_boolean_result(
            &separated_left,
            &separated_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert!(
            result.is_arrangement_cell_complex_shortcut_for(operation)
                || result.is_arrangement_cell_complex_materialized_for(operation)
                || result.is_certified_shortcut_for(operation),
            "{operation:?}: {result:?}"
        );
        result.validate().unwrap();
        result
            .validate_against_sources(&separated_left, &separated_right)
            .unwrap();
        assert_eq!(
            result.freshness_against_sources(&separated_left, &separated_right),
            ExactReportFreshness::Current
        );
        let separated_evaluation = exact_boolean_evaluation(
            &separated_left,
            &separated_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        separated_evaluation.validate().unwrap();
        let mut relabeled_winding_report = separated_evaluation.clone();
        relabeled_winding_report
            .certifications
            .closed_winding_left_in_right
            .target_closed = false;
        assert!(
            relabeled_winding_report
                .certifications
                .closed_winding_left_in_right
                .validate()
                .is_err()
        );
        assert_eq!(
            relabeled_winding_report.validate(),
            Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
            "{operation:?}: {relabeled_winding_report:?}"
        );
        assert_eq!(
            result.freshness_against_sources(&separated_left, &intersecting_right),
            ExactReportFreshness::SourceReplayMismatch
        );
        if operation == ExactBooleanOperation::Intersection {
            let mut stale_output = result.clone();
            stale_output.mesh = separated_left.clone();
            assert!(stale_output.validate().is_err(), "{stale_output:?}");
            assert_eq!(
                stale_output.freshness_against_sources(&separated_left, &separated_right),
                ExactReportFreshness::StaleRegionFacts,
                "{stale_output:?}"
            );
        }
        result
            .validate_operation_against_sources(
                &separated_left,
                &separated_right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
    }

    let outer = tetra_from_corners([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let disjoint_shell = tetra_from_corners([20, 0, 0], [21, 0, 0], [20, 1, 0], [20, 0, 1]);
    let container = combine_exact_meshes(
        &outer,
        &disjoint_shell,
        "test disconnected closed winding containment fixture",
    );
    let contained = tetra_from_corners([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);
    let uncontained = tetra_from_corners([30, 0, 0], [31, 0, 0], [30, 1, 0], [30, 0, 1]);

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let result = exact_boolean_result(
            &container,
            &contained,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert!(
            result.is_arrangement_cell_complex_shortcut_for(operation)
                || result.is_arrangement_cell_complex_materialized_for(operation)
                || result.is_certified_shortcut_for(operation),
            "{operation:?}: {result:?}"
        );
        result.validate().unwrap();
        result
            .validate_against_sources(&container, &contained)
            .unwrap();
        assert_eq!(
            result.freshness_against_sources(&container, &contained),
            ExactReportFreshness::Current
        );
        assert_eq!(
            result.freshness_against_sources(&container, &uncontained),
            ExactReportFreshness::SourceReplayMismatch
        );
        if operation == ExactBooleanOperation::Difference {
            let mut stale_output = result.clone();
            stale_output.mesh = container.clone();
            assert!(stale_output.validate().is_err(), "{stale_output:?}");
            assert_eq!(
                stale_output.freshness_against_sources(&container, &contained),
                ExactReportFreshness::StaleRegionFacts,
                "{stale_output:?}"
            );
        }
        result
            .validate_operation_against_sources(
                &container,
                &contained,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
    }
}

#[test]
fn closed_winding_public_replay_yields_to_convex_provenance() {
    let separated_left = tetra_from_corners([0, 0, 0], [2, 0, 0], [0, 2, 0], [0, 0, 2]);
    let separated_right = tetra_from_corners([1, 1, 1], [3, 1, 1], [1, 3, 1], [1, 1, 3]);
    let container = tetra_from_corners([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let contained = tetra_from_corners([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let separated_replay = exact_boolean_result(
            &separated_left,
            &separated_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert_convex_public_replay(&separated_replay, operation);
        separated_replay
            .validate_operation_against_sources(
                &separated_left,
                &separated_right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();

        let containment_replay = exact_boolean_result(
            &container,
            &contained,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert_convex_public_replay(&containment_replay, operation);
        containment_replay
            .validate_operation_against_sources(
                &container,
                &contained,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
    }
}

fn assert_convex_public_replay(result: &ExactBooleanResult, operation: ExactBooleanOperation) {
    assert!(
        result.is_arrangement_cell_complex_shortcut_for(operation)
            || result.is_arrangement_cell_complex_materialized_for(operation)
            || result.is_certified_shortcut_for(operation),
        "{operation:?}: expected convex public replay, got {result:?}"
    );
}

#[test]
fn exact_volumetric_winding_arrangement_is_publicly_replayable() {
    let left = tetra_from_corners([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let right = tetra_from_corners([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);
    let separated_right = tetra([10, 10, 10]);
    let union_request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    );
    let mut workspace = ExactBooleanWorkspace::new(&left, &right);
    let evaluation = workspace.evaluate(union_request).unwrap().clone();
    let preflight = evaluation.preflight.clone();
    assert!(
        preflight.is_certified_arrangement_cell_complex(),
        "{preflight:?}"
    );
    preflight.validate().unwrap();

    let readiness = evaluation.certifications.winding_readiness.clone();
    assert!(
        readiness.materializes_arrangement_cell_complex(),
        "{readiness:?}"
    );
    assert_eq!(
        readiness.retained_face_pairs, preflight.retained_face_pairs,
        "{readiness:?}"
    );
    assert_eq!(readiness.retained_events, preflight.retained_events);
    assert_eq!(readiness.region_count, 0);
    readiness.validate().unwrap();

    let result = workspace.materialize(union_request).unwrap();

    if result.is_arrangement_cell_complex_materialized_for(ExactBooleanOperation::Union) {
        assert!(!result.region_classifications.is_empty());
        assert!(!result.triangulations.is_empty());
        assert!(!result.volumetric_classifications.is_empty());
        assert!(!result.assembly.triangles.is_empty());
    } else {
        assert!(
            result.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Union),
            "{result:?}"
        );
        assert!(result.topology_assembly_report.is_some(), "{result:?}");
        assert!(result.region_ownership_report.is_some(), "{result:?}");
    }

    result.validate().unwrap();
    assert_eq!(
        result.freshness_against_sources(&left, &right),
        ExactReportFreshness::Current
    );
    evaluation.validate().unwrap();
    assert!(
        evaluation.preflight.is_certified_arrangement_cell_complex(),
        "{evaluation:?}"
    );
    let mut unresolved_ownership = evaluation.clone();
    let ownership = unresolved_ownership
        .certifications
        .arrangement_attempt
        .as_mut()
        .and_then(|attempt| attempt.region_ownership_report.as_mut())
        .expect("named arrangement evaluation should retain ownership evidence");
    ownership.volume_regions += 1;
    assert_eq!(
        unresolved_ownership.freshness_against_sources(&left, &right),
        ExactReportFreshness::StaleStatusEvidence
    );
    let mut incomplete_topology = evaluation.clone();
    let topology = incomplete_topology
        .certifications
        .arrangement_attempt
        .as_mut()
        .and_then(|attempt| attempt.topology_assembly_report.as_mut())
        .expect("named arrangement evaluation should retain topology assembly evidence");
    topology.region_boundaries += 1;
    assert_eq!(
        incomplete_topology.freshness_against_sources(&left, &right),
        ExactReportFreshness::StaleStatusEvidence
    );
    let mut declined_arrangement_attempt = evaluation.clone();
    let stale_validation_attempt = declined_arrangement_attempt
        .certifications
        .arrangement_attempt
        .as_mut()
        .expect("named evaluation should retain arrangement attempt");
    stale_validation_attempt.output_validation = ValidationPolicy::CLOSED;
    assert_eq!(
        declined_arrangement_attempt.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    let mut stale_attempt_gate = evaluation.clone();
    let attempt = stale_attempt_gate
        .certifications
        .arrangement_attempt
        .as_mut()
        .expect("named evaluation should retain arrangement attempt");
    attempt.region_ownership_report = None;
    assert_eq!(
        attempt.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    assert_eq!(
        stale_attempt_gate.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    let mut stale_attempt_report = evaluation.clone();
    let attempt = stale_attempt_report
        .certifications
        .arrangement_attempt
        .as_mut()
        .expect("named evaluation should retain arrangement attempt");
    attempt.topology_assembly_report = None;
    assert_eq!(
        attempt.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    assert_eq!(
        stale_attempt_report.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    let mut stale_readiness_counts = evaluation.clone();
    stale_readiness_counts
        .certifications
        .winding_readiness
        .retained_face_pairs += 1;
    stale_readiness_counts
        .certifications
        .winding_readiness
        .retained_events += 1;
    assert!(
        stale_readiness_counts.validate().is_err(),
        "{stale_readiness_counts:?}"
    );
    if result.is_arrangement_cell_complex_materialized_for(ExactBooleanOperation::Union) {
        assert!(!result.region_classifications.is_empty());
        assert!(!result.triangulations.is_empty());
        assert!(!result.volumetric_classifications.is_empty());
        assert!(!result.assembly.triangles.is_empty());
    } else {
        assert!(
            result.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Union),
            "{result:?}"
        );
        assert!(result.topology_assembly_report.is_some(), "{result:?}");
        assert!(result.region_ownership_report.is_some(), "{result:?}");
    }
    assert!(!result.mesh.triangles().is_empty());
    assert!(
        result
            .validate_operation_against_sources(
                &left,
                &right,
                ExactBooleanOperation::Intersection,
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .is_err(),
        "{result:?}"
    );
    if result.volumetric_classifications.len() > 1 {
        let mut stale_volumetric_order = result.clone();
        stale_volumetric_order.volumetric_classifications.swap(0, 1);
        assert!(
            stale_volumetric_order.validate().is_err(),
            "{stale_volumetric_order:?}"
        );
        assert_eq!(
            stale_volumetric_order.freshness_against_sources(&left, &right),
            ExactReportFreshness::StaleRegionFacts,
            "{stale_volumetric_order:?}"
        );
    }

    let difference = workspace
        .materialize(ExactBooleanRequest::new(
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
        ))
        .unwrap();
    difference.validate().unwrap();
    if difference.is_arrangement_cell_complex_materialized_for(ExactBooleanOperation::Difference) {
        let Some(reversed_triangle) = difference.assembly.triangles.iter().position(|triangle| {
            triangle.orientation == ExactOutputTriangleOrientation::ReverseSource
        }) else {
            panic!("volumetric difference should retain a reversed source triangle");
        };
        let mut stale_difference_orientation = difference.clone();
        stale_difference_orientation.assembly.triangles[reversed_triangle].orientation =
            ExactOutputTriangleOrientation::PreserveSource;
        assert_eq!(
            stale_difference_orientation.validate(),
            Err(hypermesh::ExactReportValidationError::VolumetricMaterializedAssemblyViolatesOperation),
            "{stale_difference_orientation:?}"
        );
    } else {
        assert!(
            difference.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Difference),
            "{difference:?}"
        );
        assert!(
            difference.topology_assembly_report.is_some(),
            "{difference:?}"
        );
        assert!(
            difference.region_ownership_report.is_some(),
            "{difference:?}"
        );
    }
    assert_eq!(
        result.freshness_against_sources(&left, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );
}

#[test]
fn exact_volumetric_winding_coplanar_cap_is_publicly_certified() {
    let left = ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, //
            4, 0, 0, //
            0, 4, 0, //
            0, 0, 4, //
            2, 2, 3,
        ],
        &[
            0, 2, 1, //
            1, 2, 3, //
            2, 0, 3, //
            0, 1, 4, //
            1, 3, 4, //
            3, 0, 4,
        ],
    )
    .unwrap();
    let right = tetra_from_corners([-1, 1, 0], [3, 1, 0], [-1, 5, 0], [-1, 1, 4]);

    for operation in [
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let evaluation = exact_boolean_evaluation(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
        );
        let closure = evaluation
            .certifications
            .volumetric_boundary_closure
            .as_ref()
            .expect("coplanar closure evaluation should retain boundary closure evidence");
        assert!(
            closure.is_coplanar_closure_available(),
            "{operation:?}: {closure:?}"
        );
        closure.validate().unwrap();
        closure.validate_against_sources(&left, &right).unwrap();

        let preflight = &evaluation.preflight;
        assert!(
            preflight.is_certified_arrangement_cell_complex(),
            "{operation:?}: {preflight:?}"
        );
        preflight.validate().unwrap();
        preflight.validate_against_sources(&left, &right).unwrap();

        assert!(
            evaluation_materializes_arrangement_cell_complex(&evaluation),
            "{operation:?}: {evaluation:?}"
        );

        let result = exact_boolean_result(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert!(
            result.is_arrangement_cell_complex_shortcut_for(operation),
            "{operation:?}: {result:?}"
        );
        result.validate().unwrap();
        assert!(
            result.topology_assembly_report.is_some(),
            "{operation:?}: {result:?}"
        );
        assert!(
            result.region_ownership_report.is_some(),
            "{operation:?}: {result:?}"
        );
        assert!(
            result.mesh.facts().mesh.closed_manifold || result.mesh.triangles().is_empty(),
            "{operation:?}: {:?}",
            result.mesh.facts().mesh
        );
        result.validate_against_sources(&left, &right).unwrap();

        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current,
            "{operation:?}: {result:?}"
        );
        let mut stale_output = result.clone();
        stale_output.mesh = left.clone();
        assert!(
            stale_output.validate().is_err(),
            "{operation:?}: {stale_output:?}"
        );
        assert_eq!(
            stale_output.freshness_against_sources(&left, &right),
            ExactReportFreshness::StaleRegionFacts,
            "{operation:?}: {stale_output:?}"
        );
    }
}

#[test]
fn arrangement_cell_complex_request_materialization_is_publicly_replayable() {
    let left = ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, //
            4, 0, 0, //
            0, 4, 0, //
            0, 0, 4, //
            2, 2, 3,
        ],
        &[
            0, 2, 1, //
            1, 2, 3, //
            2, 0, 3, //
            0, 1, 4, //
            1, 3, 4, //
            3, 0, 4,
        ],
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles(
        &[1, 1, 1, 5, 1, 1, 1, 5, 1, 1, 1, 5],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();
    let stale_right = tetra([10, 10, 10]);

    let result = exact_boolean_result(
        &left,
        &right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    );
    assert!(
        result.is_arrangement_cell_complex_materialized_for(ExactBooleanOperation::Union)
            || result.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Union),
        "{result:?}"
    );
    result.validate().unwrap();
    result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert!(
        result
            .validate_operation_against_sources(
                &left,
                &stale_right,
                ExactBooleanOperation::Union,
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .is_err(),
        "operation replay must reject stale source operands"
    );
    if result.is_arrangement_cell_complex_materialized_for(ExactBooleanOperation::Union) {
        assert!(!result.region_classifications.is_empty());
        assert!(!result.triangulations.is_empty());
        assert!(!result.volumetric_classifications.is_empty());
        assert!(!result.assembly.triangles.is_empty());
    } else {
        assert!(
            result.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Union),
            "{result:?}"
        );
        assert!(result.topology_assembly_report.is_some(), "{result:?}");
        assert!(result.region_ownership_report.is_some(), "{result:?}");
    }

    let horizontal = axis_aligned_box([0, 0, 0], [2, 2, 2]);
    let vertical = axis_aligned_box([1, 1, 1], [3, 3, 3]);
    let shortcut = exact_boolean_result(
        &horizontal,
        &vertical,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(
        shortcut.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Union),
        "{shortcut:?}"
    );
    shortcut.validate().unwrap();
    shortcut
        .validate_against_sources(&horizontal, &vertical)
        .unwrap();
    shortcut
        .validate_operation_against_sources(
            &horizontal,
            &vertical,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let convex_left = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let convex_right = tetra_from_corners([1, 1, 1], [5, 1, 1], [1, 5, 1], [1, 1, 5]);
    let convex_intersection = exact_boolean_result(
        &convex_left,
        &convex_right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        ),
    );
    if convex_intersection
        .is_arrangement_cell_complex_materialized_for(ExactBooleanOperation::Intersection)
    {
        assert!(!convex_intersection.region_classifications.is_empty());
        assert!(!convex_intersection.triangulations.is_empty());
        assert!(!convex_intersection.volumetric_classifications.is_empty());
        assert!(!convex_intersection.assembly.triangles.is_empty());
    } else {
        assert!(
            convex_intersection
                .is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Intersection)
                || convex_intersection
                    .is_certified_shortcut_for(ExactBooleanOperation::Intersection),
            "{convex_intersection:?}"
        );
        if convex_intersection
            .is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Intersection)
        {
            assert!(
                convex_intersection.topology_assembly_report.is_some(),
                "{convex_intersection:?}"
            );
            assert!(
                convex_intersection.region_ownership_report.is_some(),
                "{convex_intersection:?}"
            );
        }
    }
}

#[test]
fn exact_contained_face_adjacent_union_is_publicly_replayable() {
    let left = tetra_from_corners([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let right = tetra_from_corners([1, 1, 0], [1, 3, 0], [3, 1, 0], [1, 1, -2]);
    let separated_right = tetra([20, 0, 0]);
    let subdivided_left = tetra_with_subdivided_base();
    let split_crossing_right = tetra_from_corners([1, 1, 0], [1, 4, 0], [4, 3, 0], [1, 1, -2]);
    let square_base_left = square_pyramid_with_base();
    let same_orientation_square_cap =
        tetra_from_corners([2, 2, 0], [6, 2, 0], [2, 6, 0], [2, 2, -2]);
    let square_cap_right = tetra_from_corners([2, 2, 0], [2, 6, 0], [6, 2, 0], [2, 2, -2]);
    let square_disk_cap_right = downward_square_pyramid_with_base([2, 2], [6, 6], -2);
    let two_caps_right = combine_exact_meshes(
        &tetra_from_corners([1, 1, 0], [1, 2, 0], [2, 1, 0], [1, 1, -2]),
        &tetra_from_corners([4, 1, 0], [4, 2, 0], [5, 1, 0], [4, 1, -2]),
        "test two contained caps",
    );
    let disjoint_shell = tetra_from_corners([40, 0, 0], [41, 0, 0], [40, 1, 0], [40, 0, 1]);
    let container = combine_exact_meshes(
        &left,
        &disjoint_shell,
        "test disconnected contained-face fixture",
    );
    let split_container = combine_exact_meshes(
        &subdivided_left,
        &disjoint_shell,
        "test disconnected subdivided contained-face fixture",
    );
    let square_container = combine_exact_meshes(
        &square_base_left,
        &disjoint_shell,
        "test disconnected square contained-face fixture",
    );
    let square_disk_container = combine_exact_meshes(
        &square_base_left,
        &disjoint_shell,
        "test disconnected multi-face contained-cap fixture",
    );

    let stronger_result = exact_boolean_result(
        &left,
        &right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(
        stronger_result.is_certified_shortcut_for(ExactBooleanOperation::Union),
        "{stronger_result:?}"
    );
    stronger_result.validate().unwrap();
    stronger_result
        .validate_against_sources(&left, &right)
        .unwrap();

    let result = assert_public_contained_face_adjacent_union(&container, &right, 1, 1);
    assert_public_contained_face_adjacent_union(&split_container, &split_crossing_right, 2, 1);
    assert_public_contained_face_adjacent_union(&square_container, &square_cap_right, 2, 1);
    let same_orientation_result = exact_boolean_result(
        &square_container,
        &same_orientation_square_cap,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    same_orientation_result.validate().unwrap();
    same_orientation_result
        .validate_against_sources(&square_container, &same_orientation_square_cap)
        .unwrap();
    assert_public_contained_face_adjacent_union(
        &square_disk_container,
        &square_disk_cap_right,
        2,
        2,
    );
    assert_public_contained_face_adjacent_union(&container, &two_caps_right, 1, 2);
    let multi_hole_report = exact_adjacent_union_completion_report!(
        &container,
        &two_caps_right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(multi_hole_report.is_certified_contained_face());
    assert_eq!(multi_hole_report.containing_faces, 1);
    assert_eq!(multi_hole_report.contained_faces, 2);
    multi_hole_report.validate().unwrap();
    multi_hole_report
        .validate_against_sources(&container, &two_caps_right)
        .unwrap();

    let mut missing_contained = exact_adjacent_union_completion_report!(
        &container,
        &right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    missing_contained.contained_faces = 0;
    assert!(missing_contained.validate().is_err());
    assert_ne!(
        missing_contained.freshness_against_sources(&container, &right),
        ExactReportFreshness::Current
    );

    let mut relabeled_containing = exact_adjacent_union_completion_report!(
        &container,
        &right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    relabeled_containing.contained_containing_side = None;
    assert!(relabeled_containing.validate().is_err());

    let mut impossible_counts = exact_adjacent_union_completion_report!(
        &container,
        &right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    impossible_counts.containing_faces = impossible_counts.retained_face_pairs + 1;
    assert!(impossible_counts.validate().is_err());

    let mut invalid_output = result.clone();
    invalid_output.mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(invalid_output.validate().is_err(), "{invalid_output:?}");
    assert_ne!(
        invalid_output.freshness_against_sources(&container, &right),
        ExactReportFreshness::Current
    );

    assert_eq!(
        result.freshness_against_sources(&container, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );
    let split_report = exact_adjacent_union_completion_report!(
        &split_container,
        &split_crossing_right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(split_report.is_certified_contained_face());
    assert_eq!(split_report.containing_faces, 2);
    assert_eq!(split_report.contained_faces, 1);
    split_report.validate().unwrap();
    split_report
        .validate_against_sources(&split_container, &split_crossing_right)
        .unwrap();

    let square_disk_report = exact_adjacent_union_completion_report!(
        &square_disk_container,
        &square_disk_cap_right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(square_disk_report.is_certified_contained_face());
    assert_eq!(square_disk_report.containing_faces, 2);
    assert_eq!(square_disk_report.contained_faces, 2);
    square_disk_report.validate().unwrap();
    square_disk_report
        .validate_against_sources(&square_disk_container, &square_disk_cap_right)
        .unwrap();

    let completion_report = exact_adjacent_union_completion_report!(
        &container,
        &right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(completion_report.is_certified_contained_face());
    assert!(completion_report.is_certified());
    assert_eq!(
        completion_report.full_face_shared_faces + completion_report.full_face_shared_patches,
        0
    );
    assert!(completion_report.contained_faces > 0);
    assert!(completion_report.containing_faces > 0);
    assert!(completion_report.contained_containing_side.is_some());
    completion_report.validate().unwrap();
    completion_report
        .validate_against_sources(&container, &right)
        .unwrap();
    assert_eq!(
        completion_report.freshness_against_sources(&container, &right),
        ExactReportFreshness::Current
    );
    assert_eq!(
        completion_report.freshness_against_sources(&container, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );
    let result = exact_boolean_result(
        &container,
        &right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(
        result.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Union),
        "{result:?}"
    );
    result.validate().unwrap();
    result.validate_against_sources(&container, &right).unwrap();

    assert_eq!(
        result.freshness_against_sources(&container, &right),
        ExactReportFreshness::Current
    );
    let mut stale_output = result.clone();
    stale_output.mesh = container.clone();
    assert!(stale_output.validate().is_err(), "{stale_output:?}");
    assert_eq!(
        stale_output.freshness_against_sources(&container, &right),
        ExactReportFreshness::StaleRegionFacts,
        "{stale_output:?}"
    );
    assert_eq!(
        result.freshness_against_sources(&container, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );
    result
        .validate_operation_against_sources(
            &container,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
}

#[test]
fn exact_evaluation_preflight_reports_disjoint_bounds_without_retained_pairs() {
    let left = tetra([0, 0, 0]);
    let right = tetra([3, 0, 0]);

    let evaluation = exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert_eq!(evaluation.preflight.retained_face_pairs, 0);
    assert_eq!(evaluation.preflight.retained_events, 0);
    evaluation.validate_against_sources(&left, &right).unwrap();
}

#[test]
fn public_exact_blocker_reports_replay_remaining_decisions() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let overlapping_right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let separated_right = ExactMesh::from_i64_triangles_with_policy(
        &[9, 0, 0, 10, 0, 0, 9, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let refinement = exact_boolean_evaluation(
        &left,
        &overlapping_right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    )
    .certifications
    .refinement
    .clone();
    assert!(!refinement.is_required());
    refinement.validate().unwrap();
    refinement
        .validate_against_sources(&left, &overlapping_right)
        .unwrap();
    assert_eq!(
        refinement.freshness_against_sources(&left, &overlapping_right),
        ExactReportFreshness::Current
    );
    assert_eq!(
        refinement.freshness_against_sources(&left, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );

    let planar = exact_boolean_evaluation(
        &left,
        &overlapping_right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    )
    .certifications
    .planar_arrangement
    .clone();
    assert!(planar.is_already_materialized());
    assert!(!planar.is_required());
    planar.validate().unwrap();
    planar
        .validate_against_sources(&left, &overlapping_right)
        .unwrap();
    assert_eq!(
        planar.freshness_against_sources(&left, &overlapping_right),
        ExactReportFreshness::Current
    );
    assert_eq!(
        planar.freshness_against_sources(&left, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );

    let same_surface = exact_boolean_evaluation(
        &left,
        &left,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    )
    .certifications
    .same_surface
    .clone();
    assert!(same_surface.is_certified());
    same_surface.validate().unwrap();
    same_surface.validate_against_sources(&left, &left).unwrap();
    assert_eq!(
        same_surface.freshness_against_sources(&left, &left),
        ExactReportFreshness::Current
    );
    assert_eq!(
        same_surface.freshness_against_sources(&left, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );

    let parallel_right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 1, 2, 0, 1, 0, 2, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_disjoint = exact_boolean_evaluation(
        &left,
        &parallel_right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    )
    .certifications
    .open_surface_disjoint
    .clone();
    assert!(open_disjoint.is_certified());
    open_disjoint.validate().unwrap();
    open_disjoint
        .validate_against_sources(&left, &parallel_right)
        .unwrap();
    assert_eq!(
        open_disjoint.freshness_against_sources(&left, &parallel_right),
        ExactReportFreshness::Current
    );
    assert_eq!(
        open_disjoint.freshness_against_sources(&left, &overlapping_right),
        ExactReportFreshness::SourceReplayMismatch
    );
}

#[test]
fn open_surface_disjoint_report_classifies_retained_coplanar_overlap_blocker() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 0, 0, 4, 0, 0, 4, 2, 0, 2, 2, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let report = exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    )
    .certifications
    .open_surface_disjoint
    .clone();

    assert!(!report.is_certified());
    assert!(report.blocker.requires_planar_arrangement());
    assert!(report.blocker.coplanar_overlapping_pairs > 0);
    assert!(report.retained_face_pairs > 0);
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();

    let mut relabeled = report;
    relabeled.blocker.candidate_pairs = 1;
    assert_eq!(
        relabeled.validate(),
        Err(hypermesh::ExactReportValidationError::WrongBlockerKind)
    );
}

#[test]
fn planar_arrangement_report_classifies_noncoplanar_candidates_as_winding_blocker() {
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

    let report = exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    )
    .certifications
    .planar_arrangement
    .clone();

    assert!(!report.is_required());
    assert!(!report.is_already_materialized());
    assert!(report.blocker.requires_winding());
    assert!(report.blocker.candidate_pairs > 0);
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();

    let mut stale = report;
    stale.blocker.coplanar_overlapping_pairs = 1;
    assert!(stale.validate().is_err());
}

#[test]
fn exact_boolean_public_shortcuts_handle_disjoint_operands() {
    let left = tetra([0, 0, 0]);
    let right = tetra([3, 0, 0]);

    let preflight = exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    )
    .preflight
    .clone();
    assert!(!preflight.graph_had_unknowns);

    let union = exact_boolean_result(
        &left,
        &right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(
        union.is_certified_shortcut_for(ExactBooleanOperation::Union),
        "{union:?}"
    );
    union.mesh.validate_retained_state().unwrap();
    union.validate_against_sources(&left, &right).unwrap();
    assert!(union.validate_against_sources(&left, &left).is_err());
    let intersection = exact_boolean_result(
        &left,
        &right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    );
    assert!(
        intersection
            .validate_operation_against_sources(
                &left,
                &right,
                ExactBooleanOperation::Union,
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .is_err()
    );

    assert!(intersection.mesh.triangles().is_empty());
}

#[test]
fn trivial_boolean_shortcuts_are_publicly_replayable() {
    let empty = ExactMesh::new(
        Vec::new(),
        Vec::new(),
        SourceProvenance::exact("test empty mesh"),
    )
    .unwrap();
    let solid = tetra([0, 0, 0]);
    let disjoint_solid = tetra([3, 0, 0]);
    let far_solid = tetra([20, 0, 0]);
    let farther_solid = tetra([30, 0, 0]);

    let open_identical_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_identical_right = open_identical_left.clone();
    let open_same_surface_right = ExactMesh::from_i64_triangles_with_policy(
        &[4, 0, 0, 0, 4, 0, 0, 0, 0],
        &[2, 0, 1],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_disjoint_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 4, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_disjoint_right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 1, 4, 0, 5, 0, 4, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_identical_alt_left = ExactMesh::from_i64_triangles_with_policy(
        &[10, 0, 0, 14, 0, 0, 10, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_identical_alt_right = open_identical_alt_left.clone();
    let open_same_surface_alt_right = ExactMesh::from_i64_triangles_with_policy(
        &[14, 0, 0, 10, 4, 0, 10, 0, 0],
        &[2, 0, 1],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_disjoint_alt_left = ExactMesh::from_i64_triangles_with_policy(
        &[10, 0, 0, 14, 0, 4, 10, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_disjoint_alt_right = ExactMesh::from_i64_triangles_with_policy(
        &[10, 0, 1, 14, 0, 5, 10, 4, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let assert_shortcut =
        |result: &ExactBooleanResult,
         left: &ExactMesh,
         right: &ExactMesh,
         stale_left: &ExactMesh,
         stale_right: &ExactMesh,
         operation: ExactBooleanOperation,
         validation: ValidationPolicy,
         predicate: fn(&ExactBooleanResult, ExactBooleanOperation) -> bool| {
            assert!(predicate(result, operation), "{operation:?}: {result:?}");
            result.validate().unwrap();
            result.validate_against_sources(left, right).unwrap();
            assert_eq!(
                result.freshness_against_sources(left, right),
                ExactReportFreshness::Current
            );
            assert_eq!(
                result.freshness_against_sources(stale_left, stale_right),
                ExactReportFreshness::SourceReplayMismatch
            );
            result
                .validate_operation_against_sources(
                    left,
                    right,
                    operation,
                    validation,
                    ExactBoundaryBooleanPolicy::Reject,
                )
                .unwrap();
        };

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let empty_result = exact_boolean_result(
            &empty,
            &solid,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert_shortcut(
            &empty_result,
            &empty,
            &solid,
            &solid,
            &disjoint_solid,
            operation,
            ValidationPolicy::CLOSED,
            |result, operation| result.is_certified_shortcut_for(operation),
        );
        let empty_evaluation = exact_boolean_evaluation(
            &empty,
            &solid,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        empty_evaluation.validate().unwrap();
        let mut relabeled_empty_facts = empty_evaluation.clone();
        relabeled_empty_facts.certifications.trivial.left_empty = false;
        relabeled_empty_facts.certifications.trivial.right_empty = false;
        assert_eq!(
            relabeled_empty_facts.validate(),
            Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
            "{operation:?}: {relabeled_empty_facts:?}"
        );
        if operation == ExactBooleanOperation::Union {
            assert_eq!(
                empty_result.freshness_against_sources(&empty, &disjoint_solid),
                ExactReportFreshness::SourceReplayMismatch
            );
        }

        let empty_open_result = exact_boolean_result(
            &empty,
            &open_disjoint_left,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert_shortcut(
            &empty_open_result,
            &empty,
            &open_disjoint_left,
            &solid,
            &open_disjoint_left,
            operation,
            ValidationPolicy::CLOSED,
            |result, operation| result.is_certified_shortcut_for(operation),
        );
        assert!(empty_open_result.mesh.triangles().is_empty());
        assert!(empty_open_result.mesh.facts().mesh.closed_manifold);

        let replayed_empty_open = exact_boolean_result(
            &empty,
            &open_disjoint_left,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert_eq!(replayed_empty_open.kind, empty_open_result.kind);
        assert!(replayed_empty_open.mesh.triangles().is_empty());
        assert!(
            replayed_empty_open.is_certified_shortcut_for(operation),
            "{operation:?}: {replayed_empty_open:?}"
        );

        let open_empty_result = exact_boolean_result(
            &open_disjoint_left,
            &empty,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert!(open_empty_result.is_certified_shortcut_for(operation));
        assert!(open_empty_result.mesh.triangles().is_empty());
        assert!(open_empty_result.mesh.facts().mesh.closed_manifold);
        open_empty_result
            .validate_operation_against_sources(
                &open_disjoint_left,
                &empty,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();

        let disjoint_result = exact_boolean_result(
            &solid,
            &disjoint_solid,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert_shortcut(
            &disjoint_result,
            &solid,
            &disjoint_solid,
            &solid,
            &solid,
            operation,
            ValidationPolicy::CLOSED,
            |result, operation| result.is_certified_shortcut_for(operation),
        );
        let disjoint_evaluation = exact_boolean_evaluation(
            &solid,
            &disjoint_solid,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        disjoint_evaluation.validate().unwrap();
        let mut relabeled_disjoint_facts = disjoint_evaluation.clone();
        relabeled_disjoint_facts
            .certifications
            .trivial
            .bounds_disjoint = false;
        assert_eq!(
            relabeled_disjoint_facts.validate(),
            Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
            "{operation:?}: {relabeled_disjoint_facts:?}"
        );
        if matches!(
            operation,
            ExactBooleanOperation::Union | ExactBooleanOperation::Difference
        ) {
            assert_eq!(
                disjoint_result.freshness_against_sources(&far_solid, &farther_solid),
                ExactReportFreshness::SourceReplayMismatch
            );
        }

        let identical_result = exact_boolean_result(
            &open_identical_left,
            &open_identical_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
        );
        assert_shortcut(
            &identical_result,
            &open_identical_left,
            &open_identical_right,
            &open_identical_left,
            &open_same_surface_right,
            operation,
            ValidationPolicy::ALLOW_BOUNDARY,
            |result, operation| result.is_certified_shortcut_for(operation),
        );
        let identical_evaluation = exact_boolean_evaluation(
            &open_identical_left,
            &open_identical_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
        );
        identical_evaluation.validate().unwrap();
        let mut relabeled_identity_report = identical_evaluation.clone();
        relabeled_identity_report
            .certifications
            .identical
            .left_triangles += 1;
        assert!(
            relabeled_identity_report
                .certifications
                .identical
                .validate()
                .is_err()
        );
        assert_eq!(
            relabeled_identity_report.validate(),
            Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
            "{operation:?}: {relabeled_identity_report:?}"
        );
        if matches!(
            operation,
            ExactBooleanOperation::Union | ExactBooleanOperation::Intersection
        ) {
            assert_eq!(
                identical_result
                    .freshness_against_sources(&open_identical_alt_left, &open_identical_alt_right),
                ExactReportFreshness::SourceReplayMismatch
            );
        }
        let closed_identical_result = exact_boolean_result(
            &open_identical_left,
            &open_identical_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert!(
            closed_identical_result.is_certified_shortcut_for(operation),
            "{operation:?}: {closed_identical_result:?}"
        );
        assert!(closed_identical_result.mesh.triangles().is_empty());
        assert!(closed_identical_result.mesh.facts().mesh.closed_manifold);
        closed_identical_result
            .validate_operation_against_sources(
                &open_identical_left,
                &open_identical_right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();

        let same_surface_result = exact_boolean_result(
            &open_identical_left,
            &open_same_surface_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
        );
        assert_shortcut(
            &same_surface_result,
            &open_identical_left,
            &open_same_surface_right,
            &open_identical_left,
            &open_disjoint_right,
            operation,
            ValidationPolicy::ALLOW_BOUNDARY,
            |result, operation| result.is_certified_shortcut_for(operation),
        );
        let same_surface_evaluation = exact_boolean_evaluation(
            &open_identical_left,
            &open_same_surface_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
        );
        same_surface_evaluation.validate().unwrap();
        let mut relabeled_same_surface_report = same_surface_evaluation.clone();
        relabeled_same_surface_report.certifications.same_surface = exact_boolean_evaluation(
            &open_identical_left,
            &open_disjoint_left,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
        )
        .certifications
        .same_surface
        .clone();
        relabeled_same_surface_report
            .certifications
            .same_surface
            .validate()
            .unwrap();
        assert_eq!(
            relabeled_same_surface_report.validate(),
            Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
            "{operation:?}: {relabeled_same_surface_report:?}"
        );
        if matches!(
            operation,
            ExactBooleanOperation::Union | ExactBooleanOperation::Intersection
        ) {
            assert_eq!(
                same_surface_result.freshness_against_sources(
                    &open_identical_alt_left,
                    &open_same_surface_alt_right,
                ),
                ExactReportFreshness::SourceReplayMismatch
            );
        }
        let closed_same_surface_result = exact_boolean_result(
            &open_identical_left,
            &open_same_surface_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert!(
            closed_same_surface_result.is_certified_shortcut_for(operation),
            "{operation:?}: {closed_same_surface_result:?}"
        );
        assert!(closed_same_surface_result.mesh.triangles().is_empty());
        assert!(closed_same_surface_result.mesh.facts().mesh.closed_manifold);
        closed_same_surface_result
            .validate_operation_against_sources(
                &open_identical_left,
                &open_same_surface_right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
        let lower_dimensional_evaluation = exact_boolean_evaluation(
            &open_identical_left,
            &open_same_surface_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        lower_dimensional_evaluation.validate().unwrap();
        let mut relabeled_lower_dimensional_facts = lower_dimensional_evaluation.clone();
        relabeled_lower_dimensional_facts
            .certifications
            .regularized_solid
            .left_open_surface = false;
        assert_eq!(
            relabeled_lower_dimensional_facts.validate(),
            Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
            "{operation:?}: {relabeled_lower_dimensional_facts:?}"
        );

        let mixed_dimensional_result = exact_boolean_result(
            &solid,
            &open_disjoint_left,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert_shortcut(
            &mixed_dimensional_result,
            &solid,
            &open_disjoint_left,
            &open_disjoint_left,
            &open_disjoint_right,
            operation,
            ValidationPolicy::CLOSED,
            |result, operation| result.is_certified_shortcut_for(operation),
        );
        let mixed_dimensional_evaluation = exact_boolean_evaluation(
            &solid,
            &open_disjoint_left,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        mixed_dimensional_evaluation.validate().unwrap();
        let mut relabeled_mixed_dimensional_facts = mixed_dimensional_evaluation.clone();
        relabeled_mixed_dimensional_facts
            .certifications
            .regularized_solid
            .right_open_surface = false;
        assert_eq!(
            relabeled_mixed_dimensional_facts.validate(),
            Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
            "{operation:?}: {relabeled_mixed_dimensional_facts:?}"
        );

        let open_disjoint_result = exact_boolean_result(
            &open_disjoint_left,
            &open_disjoint_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
        );
        assert_shortcut(
            &open_disjoint_result,
            &open_disjoint_left,
            &open_disjoint_right,
            &open_disjoint_left,
            &open_identical_left,
            operation,
            ValidationPolicy::ALLOW_BOUNDARY,
            |result, operation| result.is_certified_shortcut_for(operation),
        );
        let open_disjoint_evaluation = exact_boolean_evaluation(
            &open_disjoint_left,
            &open_disjoint_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
        );
        open_disjoint_evaluation.validate().unwrap();
        let mut relabeled_disjoint_report = open_disjoint_evaluation.clone();
        relabeled_disjoint_report
            .certifications
            .open_surface_disjoint = exact_boolean_evaluation(
            &solid,
            &open_disjoint_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
        )
        .certifications
        .open_surface_disjoint
        .clone();
        relabeled_disjoint_report
            .certifications
            .open_surface_disjoint
            .validate()
            .unwrap();
        assert_eq!(
            relabeled_disjoint_report.validate(),
            Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
            "{operation:?}: {relabeled_disjoint_report:?}"
        );
        if matches!(
            operation,
            ExactBooleanOperation::Union | ExactBooleanOperation::Difference
        ) {
            assert_eq!(
                open_disjoint_result
                    .freshness_against_sources(&open_disjoint_alt_left, &open_disjoint_alt_right),
                ExactReportFreshness::SourceReplayMismatch
            );
        }
    }

    let solid_disjoint = exact_boolean_result(
        &solid,
        &disjoint_solid,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(
        solid_disjoint.is_certified_shortcut_for(ExactBooleanOperation::Union),
        "{solid_disjoint:?}"
    );
    let identical_replay = exact_boolean_result(
        &open_identical_left,
        &open_identical_right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    );
    assert!(
        identical_replay.is_certified_shortcut_for(ExactBooleanOperation::Union),
        "{identical_replay:?}"
    );
    let closed_disjoint = exact_boolean_result(
        &solid,
        &disjoint_solid,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(
        closed_disjoint.is_certified_shortcut_for(ExactBooleanOperation::Union),
        "{closed_disjoint:?}"
    );
}

#[test]
fn closed_same_surface_boolean_is_publicly_replayable() {
    let left_a = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let left_b = tetra_from_corners([10, 0, 0], [14, 0, 0], [10, 4, 0], [10, 0, 4]);
    let left = combine_exact_meshes(&left_a, &left_b, "test disconnected same-surface left");
    let same_surface_a = ExactMesh::from_i64_triangles(
        &[
            4, 0, 0, //
            0, 0, 0, //
            0, 4, 0, //
            0, 0, 4,
        ],
        &[
            1, 2, 0, //
            1, 0, 3, //
            0, 2, 3, //
            2, 1, 3,
        ],
    )
    .unwrap();
    let same_surface_b = ExactMesh::from_i64_triangles(
        &[
            14, 0, 0, //
            10, 0, 0, //
            10, 4, 0, //
            10, 0, 4,
        ],
        &[
            1, 2, 0, //
            1, 0, 3, //
            0, 2, 3, //
            2, 1, 3,
        ],
    )
    .unwrap();
    let same_surface = combine_exact_meshes(
        &same_surface_a,
        &same_surface_b,
        "test disconnected same-surface right",
    );
    let stale_right = combine_exact_meshes(
        &tetra([20, 0, 0]),
        &tetra([30, 0, 0]),
        "test stale disconnected right",
    );

    for (right_index, right) in [&same_surface, &left].into_iter().enumerate() {
        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            let result = exact_boolean_result(
                &left,
                right,
                ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
            );
            assert!(
                result.is_arrangement_cell_complex_shortcut_for(operation),
                "{operation:?}: {result:?}"
            );
            result.validate().unwrap();
            let mut stale_output = result.clone();
            stale_output.mesh = stale_right.clone();
            assert!(stale_output.validate().is_err(), "{stale_output:?}");
            assert_eq!(
                stale_output.freshness_against_sources(&left, right),
                ExactReportFreshness::StaleRegionFacts,
                "{stale_output:?}"
            );
            assert_eq!(
                result.freshness_against_sources(&left, &stale_right),
                ExactReportFreshness::SourceReplayMismatch
            );
            result
                .validate_operation_against_sources(
                    &left,
                    right,
                    operation,
                    ValidationPolicy::CLOSED,
                    ExactBoundaryBooleanPolicy::Reject,
                )
                .unwrap_or_else(|error| {
                    let replay = exact_boolean_result(
                        &left,
                        right,
                        ExactBooleanRequest::with_boundary_policy(
                            operation,
                            ValidationPolicy::CLOSED,
                            ExactBoundaryBooleanPolicy::Reject,
                        ),
                    );
                    panic!(
                        "right_index={right_index} operation={operation:?} error={error:?} result={:?} replay={:?}",
                        result.kind, replay.kind
                    );
                });
        }
    }

    let open_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_right = ExactMesh::from_i64_triangles_with_policy(
        &[4, 0, 0, 0, 4, 0, 0, 0, 0],
        &[2, 0, 1],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_same_surface = exact_boolean_result(
        &open_left,
        &open_right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    );
    assert!(
        open_same_surface.is_certified_shortcut_for(ExactBooleanOperation::Union),
        "{open_same_surface:?}"
    );
    let stale_replay = exact_boolean_result(
        &left,
        &stale_right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(
        stale_replay.is_certified_shortcut_for(ExactBooleanOperation::Union),
        "{stale_replay:?}"
    );

    let convex_left = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let convex_same_surface = same_surface_a;
    let convex_same_surface_replay = exact_boolean_result(
        &convex_left,
        &convex_same_surface,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        ),
    );
    assert!(
        convex_same_surface_replay
            .is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Intersection),
        "{convex_same_surface_replay:?}"
    );
    convex_same_surface_replay.validate().unwrap();
    convex_same_surface_replay
        .validate_against_sources(&convex_left, &convex_same_surface)
        .unwrap();
}

#[test]
fn exact_boolean_attempt_public_path_reports_blockers_or_cells() {
    let left = tetra([0, 0, 0]);
    let right = tetra([1, 0, 0]);

    let attempt = exact_boolean_arrangement_attempt(
        &left,
        &right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    attempt.validate().unwrap();
    assert!(attempt.topology_assembly_complete());
    assert!(attempt.region_ownership_volume_resolved());
    attempt.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        attempt.freshness_against_sources(&left, &right),
        ExactReportFreshness::Current
    );
    let mut stale_attempt_report = attempt.clone();
    stale_attempt_report.region_ownership_report = None;
    assert_eq!(
        stale_attempt_report.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
}

#[test]
fn exact_volumetric_region_reports_replay_from_boolean_result() {
    let left = ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, //
            4, 0, 0, //
            0, 4, 0, //
            0, 0, 4, //
            2, 2, 3,
        ],
        &[
            0, 2, 1, //
            1, 2, 3, //
            2, 0, 3, //
            0, 1, 4, //
            1, 3, 4, //
            3, 0, 4,
        ],
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles(
        &[1, 1, 1, 5, 1, 1, 1, 5, 1, 1, 1, 5],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();

    let result = exact_boolean_result(
        &left,
        &right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    );
    assert!(!result.volumetric_classifications.is_empty(), "{result:?}");
    let shifted_target = tetra([10, 10, 10]);
    let (classification, triangulation) = result
        .triangulations
        .iter()
        .find_map(|triangulation| {
            result
                .volumetric_classifications
                .iter()
                .find(|classification| {
                    triangulation.side == classification.region_side
                        && triangulation.face == classification.region_face
                        && triangulation
                            .triangles
                            .chunks_exact(3)
                            .any(|triangle| triangle == classification.triangle)
                        && classification
                            .validate_against_sources(triangulation, &shifted_target)
                            .is_err()
                })
                .map(|classification| (classification, triangulation))
        })
        .expect("volumetric classification should replay from retained sources");
    let target = classification.replay_target_mesh(&left, &right);
    assert!(classification.relation.is_materialization_decided());
    classification
        .validate_against_sources(triangulation, target)
        .unwrap();

    let mut stale_attempts = classification.clone();
    stale_attempts.witness_attempts.clear();
    assert!(
        stale_attempts
            .validate_against_sources(triangulation, target)
            .is_err()
    );

    assert!(
        classification
            .validate_against_sources(triangulation, &shifted_target)
            .is_err()
    );
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
    let report = exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    )
    .certifications
    .boundary_touching
    .clone();
    assert!(report.is_certified(), "{report:?}");
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();

    let preflight = exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    )
    .preflight
    .clone();
    assert!(
        preflight.is_certified_boundary_policy_shortcut(),
        "{preflight:?}"
    );
    let rejected_policy_preflight = exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::with_boundary_policy(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        ),
    )
    .preflight
    .clone();
    assert!(
        rejected_policy_preflight.requires_boundary_policy(),
        "{rejected_policy_preflight:?}"
    );

    let policy_preflight = exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::with_boundary_policy(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        ),
    )
    .preflight
    .clone();
    assert!(
        policy_preflight.is_certified_boundary_policy_shortcut(),
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
            .is_ok(),
        "default replay should certify a boundary-policy preflight"
    );

    let rejected_readiness = exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::with_boundary_policy(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        ),
    )
    .certifications
    .winding_readiness
    .clone();
    assert!(
        rejected_readiness.requires_boundary_policy(),
        "{rejected_readiness:?}"
    );

    let policy_readiness = exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::with_boundary_policy(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        ),
    )
    .certifications
    .winding_readiness
    .clone();
    assert!(
        policy_readiness.is_boundary_policy_shortcut_materialized(),
        "{policy_readiness:?}"
    );
    assert!(policy_readiness.blocker.requires_boundary_policy());
    assert_eq!(
        policy_readiness.retained_face_pairs,
        report.retained_face_pairs
    );
    assert_eq!(policy_readiness.retained_events, report.retained_events);
    policy_readiness.validate().unwrap();
    policy_readiness
        .validate_against_sources_with_boundary_policy(
            &left,
            &right,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .unwrap();
    assert_eq!(
        policy_readiness.freshness_against_sources_with_boundary_policy(
            &left,
            &right,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        ),
        hypermesh::ExactReportFreshness::Current
    );
    assert!(
        policy_readiness
            .validate_against_sources(&left, &right)
            .is_err(),
        "strict replay should not certify a boundary-policy readiness report"
    );
    let default_result = exact_boolean_result(
        &left,
        &right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    );
    default_result.validate().unwrap();
    default_result
        .validate_against_sources(&left, &right)
        .unwrap();

    let projected = exact_boolean_result(
        &left,
        &right,
        ExactBooleanRequest::with_boundary_policy(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        ),
    );
    projected.validate().unwrap();
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

    let closed_intersection_preflight = exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::with_boundary_policy(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        ),
    )
    .preflight
    .clone();
    assert!(
        closed_intersection_preflight.is_certified_lower_dimensional_regularized_solid(),
        "{closed_intersection_preflight:?}"
    );
    closed_intersection_preflight
        .validate_against_sources_with_boundary_policy(
            &left,
            &right,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .unwrap();

    let closed_intersection_readiness = exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::with_boundary_policy(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        ),
    )
    .certifications
    .winding_readiness
    .clone();
    assert!(
        closed_intersection_readiness.is_lower_dimensional_regularized_solid_materialized(),
        "{closed_intersection_readiness:?}"
    );
    closed_intersection_readiness
        .validate_against_sources_with_boundary_policy(
            &left,
            &right,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .unwrap();

    let closed_intersection = exact_boolean_result(
        &left,
        &right,
        ExactBooleanRequest::with_boundary_policy(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        ),
    );
    assert!(
        closed_intersection.is_certified_shortcut_for(ExactBooleanOperation::Intersection),
        "{closed_intersection:?}"
    );
    assert!(closed_intersection.mesh.triangles().is_empty());
    assert!(closed_intersection.mesh.facts().mesh.closed_manifold);
    closed_intersection
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .unwrap();

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Difference,
    ] {
        let closed_policy_preflight = exact_boolean_evaluation(
            &left,
            &right,
            ExactBooleanRequest::with_boundary_policy(
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            ),
        )
        .preflight
        .clone();
        assert!(
            closed_policy_preflight.is_certified_lower_dimensional_regularized_solid(),
            "{operation:?}: {closed_policy_preflight:?}"
        );
        closed_policy_preflight
            .validate_against_sources_with_boundary_policy(
                &left,
                &right,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            )
            .unwrap();
        let closed_policy_readiness = exact_boolean_evaluation(
            &left,
            &right,
            ExactBooleanRequest::with_boundary_policy(
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            ),
        )
        .certifications
        .winding_readiness
        .clone();
        assert!(
            closed_policy_readiness.is_lower_dimensional_regularized_solid_materialized(),
            "{operation:?}: {closed_policy_readiness:?}"
        );
        let materialized = exact_boolean_result(
            &left,
            &right,
            ExactBooleanRequest::with_boundary_policy(
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            ),
        );
        assert!(
            materialized.is_certified_shortcut_for(operation),
            "{operation:?}: {materialized:?}"
        );
        materialized
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            )
            .unwrap();
        let closed_regularized = exact_boolean_result(
            &left,
            &right,
            ExactBooleanRequest::with_boundary_policy(
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            ),
        );
        assert!(
            closed_regularized.is_certified_shortcut_for(operation),
            "{operation:?}: {closed_regularized:?}"
        );
    }
}

#[test]
fn boundary_touching_report_classifies_proper_crossing_as_winding_blocker() {
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

    let report = exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
    )
    .certifications
    .boundary_touching
    .clone();

    assert!(!report.is_certified());
    assert!(report.blocker.requires_winding());
    assert!(report.blocker.candidate_pairs > 0);
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();

    let mut stale = report;
    stale.blocker.coplanar_touching_pairs = 1;
    assert!(stale.validate().is_err());
}
