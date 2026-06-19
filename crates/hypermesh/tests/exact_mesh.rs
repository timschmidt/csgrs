use hyperlimit::{Point3, SourceProvenance};
use hypermesh::{
    ExactBooleanOperation, ExactBooleanRequest, ExactBooleanResult, ExactBooleanWorkspace,
    ExactBoundaryBooleanPolicy, ExactMesh, ExactMeshConsumerDomain, ExactRegionSelection,
    ExactRegularizationPolicy, ExactReportFreshness, MeshArtifactManifest, ValidationPolicy,
};
use hyperreal::Real;

fn p(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

fn with_exact_boolean_evaluation<R>(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    f: impl FnOnce(&hypermesh::ExactBooleanEvaluation) -> R,
) -> R {
    let mut workspace = ExactBooleanWorkspace::new(left, right);
    let evaluation = workspace.evaluate(request).unwrap();
    f(evaluation)
}

fn assert_evaluation_retains_attempt_gate_reports(evaluation: &hypermesh::ExactBooleanEvaluation) {
    let attempt = evaluation
        .retained_arrangement_attempt()
        .expect("evaluation should retain an arrangement attempt");
    assert!(attempt.topology_assembly_is_complete(), "{evaluation:?}");
    assert!(attempt.region_ownership_is_resolved(), "{evaluation:?}");
}

fn exact_boolean_result(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> ExactBooleanResult {
    let mut workspace = ExactBooleanWorkspace::new(left, right);
    let result = workspace
        .materialize_ref(request)
        .cloned()
        .unwrap_or_else(|_| workspace.materialize(request).unwrap());
    workspace
        .evaluate(request)
        .unwrap()
        .validate_against_sources(left, right)
        .unwrap();
    result
}

macro_rules! with_exact_boolean_arrangement_attempt {
    ($left:expr, $right:expr, $request:expr, $policy:expr, |$attempt:ident| $body:block $(,)?) => {{
        assert_eq!($policy, ExactRegularizationPolicy::REGULARIZED_SOLID);
        let mut workspace = ExactBooleanWorkspace::new($left, $right);
        let $attempt = workspace
            .evaluate($request)
            .unwrap()
            .retained_arrangement_attempt()
            .expect("evaluation should retain an arrangement attempt");
        $body
    }};
}

fn assert_public_full_face_adjacent_union(
    left: &ExactMesh,
    right: &ExactMesh,
    _expected_shared_faces: usize,
    _expected_shared_patches: usize,
) -> ExactBooleanResult {
    let request = ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
    with_exact_boolean_evaluation(left, right, request, |evaluation| {
        evaluation.validate().unwrap();
        evaluation.validate_against_sources(left, right).unwrap();
        assert_eq!(
            evaluation.freshness_against_sources(left, right),
            ExactReportFreshness::Current
        );
    });

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
    assert!(result.mesh().facts().mesh.closed_manifold);
    assert!(!result.mesh().triangles().is_empty());
    result
}

fn assert_public_contained_face_adjacent_union(
    left: &ExactMesh,
    right: &ExactMesh,
    _expected_containing_faces: usize,
    _expected_contained_faces: usize,
) -> ExactBooleanResult {
    let request = ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
    with_exact_boolean_evaluation(left, right, request, |evaluation| {
        evaluation.validate().unwrap();
        evaluation.validate_against_sources(left, right).unwrap();
        assert_eq!(
            evaluation.freshness_against_sources(left, right),
            ExactReportFreshness::Current
        );
    });

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
    assert!(result.mesh().facts().mesh.closed_manifold);
    assert!(!result.mesh().triangles().is_empty());
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

    with_exact_boolean_evaluation(&left, &right, request, |evaluation| {
        evaluation.validate().unwrap();
        evaluation.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            evaluation.freshness_against_sources(&left, &right),
            hypermesh::ExactReportFreshness::Current
        );
        assert!(evaluation.is_certified());
        assert!(evaluation.materialized_result().is_some());
        assert!(!evaluation.has_blocker());
        assert!(evaluation.is_certified());
        assert!(evaluation.materialized_result().is_some_and(|result| {
            result.is_certified_shortcut_for(ExactBooleanOperation::Union)
        }));
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
    });
}

#[test]
fn exact_boolean_evaluation_retains_region_ownership_report() {
    let left = tetra_from_corners([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let right = tetra_from_corners([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);
    let request = ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);

    with_exact_boolean_evaluation(&left, &right, request, |evaluation| {
        evaluation.validate().unwrap();
        evaluation.validate_against_sources(&left, &right).unwrap();
        assert!(
            evaluation.materializes_arrangement_cell_complex(),
            "{evaluation:?}"
        );
        assert!(
            evaluation.retained_arrangement_attempt().is_some(),
            "named boolean certifications should retain arrangement attempt"
        );
        let attempt = evaluation
            .retained_arrangement_attempt()
            .expect("named boolean certifications should retain arrangement attempt");
        assert!(attempt.region_ownership_is_resolved());
        assert!(attempt.region_ownership_is_volume_resolved());
        assert!(attempt.topology_assembly_is_complete());
    });
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

    with_exact_boolean_evaluation(&left, &right, request, |evaluation| {
        evaluation.validate().unwrap();
        evaluation.validate_against_sources(&left, &right).unwrap();
        assert!(evaluation.is_certified());
        assert!(evaluation.materialized_result().is_some());
        assert!(!evaluation.has_blocker());
        assert!(evaluation.is_certified());
        assert!(evaluation.has_retained_graph_evidence());
        let result = evaluation
            .materialized_result()
            .expect("boundary-policy evaluation should materialize");
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
    });
    let rejected_request = ExactBooleanRequest::with_boundary_policy(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    );
    with_exact_boolean_evaluation(&left, &right, rejected_request, |rejected| {
        rejected.validate().unwrap();
        assert!(!rejected.is_certified());
        assert!(rejected.materialized_result().is_none());
    });
    assert!(
        ExactBooleanWorkspace::new(&left, &right)
            .materialize(rejected_request)
            .is_err()
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
    assert!(readiness.is_ready());
    assert!(report.edge_ready());

    let mut missing_integer_evidence = report.clone();
    missing_integer_evidence.exact_integer_coordinates -= 1;
    assert!(
        missing_integer_evidence
            .validate()
            .is_err_and(|error| error.is_exact_coordinate_count_mismatch())
    );
    assert!(missing_integer_evidence.readiness().is_invalid_report());

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
    assert!(lossy_report.readiness().is_ready());

    let mut missing_dyadic_evidence = lossy_report.clone();
    missing_dyadic_evidence.exact_dyadic_coordinates -= 1;
    assert!(
        missing_dyadic_evidence
            .validate()
            .is_err_and(|error| error.is_exact_coordinate_count_mismatch())
    );
    assert!(missing_dyadic_evidence.readiness().is_invalid_report());

    let mut missing_float_diagnostic =
        ExactMesh::inspect_f64_triangles(&[0.0, f64::NAN, 0.0], &[0, 1, 2]);
    assert!(missing_float_diagnostic.readiness().is_invalid_coordinate());
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

    let proposal_artifact = exact
        .proposal_artifact_manifest(&proposal)
        .unwrap()
        .report();
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
    assert!(preview.has_preview_or_export_only_blocker());
    assert!(preview.has_preview_or_export_source_blocker());
    assert!(preview.has_missing_exact_coordinate_replay_blocker());
    assert!(preview.has_missing_exact_topology_replay_blocker());
    let mut preview_missing_blocker = preview.clone();
    preview_missing_blocker.remove_missing_exact_coordinate_replay_blocker();
    assert!(
        preview_missing_blocker
            .validate()
            .is_err_and(|error| error.is_missing_exact_coordinate_replay_blocker())
    );
    let mut duplicate_preview_blocker = preview.clone();
    duplicate_preview_blocker.duplicate_preview_or_export_only_blocker();
    assert!(
        duplicate_preview_blocker
            .validate()
            .is_err_and(|error| error.is_duplicate_preview_or_export_only_blocker())
    );

    let brep_triangle_handoff = |face_vertices| {
        MeshArtifactManifest::brep_exact_triangle_handoff_faces(
            "brep exact triangle handoff",
            1,
            vec![face_vertices],
        )
    };

    let repeated_vertex_handoff = brep_triangle_handoff(vec![0, 1, 1]).report();
    repeated_vertex_handoff.validate().unwrap();
    assert!(!repeated_vertex_handoff.validation_handoff_ready);
    assert!(!repeated_vertex_handoff.topology_validation_replay_ready);
    assert!(
        repeated_vertex_handoff.has_face_repeated_vertex_blocker(),
        "{repeated_vertex_handoff:?}"
    );

    let mut missing_vertex_record_manifest = brep_triangle_handoff(vec![0, 1, 2]);
    missing_vertex_record_manifest.declared_vertex_count += 1;
    let missing_vertex_record = missing_vertex_record_manifest.report();
    missing_vertex_record.validate().unwrap();
    assert!(!missing_vertex_record.validation_handoff_ready);
    assert!(!missing_vertex_record.coordinates_exact_replay_ready);
    assert!(
        missing_vertex_record.has_missing_or_mismatched_vertex_records_blocker(),
        "{missing_vertex_record:?}"
    );

    let mut stale_face_index_manifest = brep_triangle_handoff(vec![0, 1, 2]);
    stale_face_index_manifest.faces[0].index = 1;
    let stale_face_index = stale_face_index_manifest.report();
    stale_face_index.validate().unwrap();
    assert!(!stale_face_index.validation_handoff_ready);
    assert!(!stale_face_index.topology_validation_replay_ready);
    assert!(
        stale_face_index.has_face_index_mismatch_blocker(),
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
    let operation = ExactBooleanOperation::Union;
    let result = exact_boolean_result(
        &left,
        &right,
        ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
    );
    assert!(
        result.is_certified_shortcut_for(operation),
        "{operation:?}: {result:?}"
    );
    result.validate().unwrap();
    assert!(result.mesh().facts().mesh.closed_manifold);
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
        with_exact_boolean_evaluation(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
            |preflight_evaluation| {
                assert!(
                    preflight_evaluation.materializes_arrangement_cell_complex(),
                    "{operation:?}: {preflight_evaluation:?}"
                );
                assert!(
                    !preflight_evaluation.has_blocker(),
                    "{operation:?}: {preflight_evaluation:?}"
                );
                preflight_evaluation.validate().unwrap();
                preflight_evaluation
                    .validate_against_sources(&left, &right)
                    .unwrap_or_else(|error| panic!("{operation:?}: {error:?}"));
            },
        );

        with_exact_boolean_evaluation(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
            |evaluation| {
                let result = evaluation
                    .materialized_result()
                    .expect("certified arrangement cell-complex evaluation should materialize");
                assert!(
                    result.is_arrangement_cell_complex_shortcut_for(operation),
                    "{operation:?}: {result:?}"
                );
                evaluation.validate().unwrap();
                evaluation.validate_against_sources(&left, &right).unwrap();
                result.validate().unwrap();
                assert!(result.mesh().facts().mesh.closed_manifold);
            },
        );
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
        assert_eq!(
            result.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
        assert!(result.mesh().facts().mesh.closed_manifold);
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
    assert!(result.mesh().facts().mesh.closed_manifold);
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
    assert!(result.mesh().facts().mesh.closed_manifold);
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
    assert!(result.mesh().facts().mesh.closed_manifold);
}

#[test]
fn exact_coplanar_volumetric_cell_evidence_is_retained_by_public_evaluation() {
    let left_a = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let left_b = tetra_from_corners([20, 0, 0], [22, 0, 0], [20, 2, 0], [20, 0, 2]);
    let left = combine_exact_meshes(&left_a, &left_b, "test disconnected same-side fixture");
    let right = tetra_from_corners([0, 0, 0], [8, 0, 0], [0, 8, 0], [0, 0, 8]);

    with_exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
        |evaluation| {
            evaluation.validate().unwrap();
            assert!(
                evaluation.has_blocker() || evaluation.materializes_arrangement_cell_complex(),
                "{evaluation:?}"
            );
            assert!(
                evaluation
                    .retained_arrangement_attempt()
                    .is_some_and(|attempt| attempt.region_ownership_resolves_requested_operation())
                    || evaluation.materializes_arrangement_cell_complex()
                    || evaluation.has_coplanar_volumetric_evidence(),
                "{evaluation:?}"
            );
            evaluation.validate_against_sources(&left, &right).unwrap();
            assert!(
                evaluation.has_coplanar_volumetric_evidence(),
                "coplanar volumetric blocker should retain source-aware evidence"
            );
            evaluation.validate_against_sources(&left, &right).unwrap();
            assert!(evaluation.requires_coplanar_volumetric_cells());

            let separated_right = tetra([10, 0, 0]);
            assert!(
                evaluation
                    .validate_against_sources(&left, &separated_right)
                    .is_err()
            );
        },
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
        assert!(result.mesh().facts().mesh.closed_manifold);
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
    with_exact_boolean_evaluation(
        &separated_left,
        &separated_right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        ),
        |separated_evaluation| {
            separated_evaluation.validate().unwrap();
            separated_evaluation
                .validate_against_sources(&separated_left, &separated_right)
                .unwrap();
        },
    );
    let dispatched = exact_boolean_result(
        &separated_left,
        &separated_right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        ),
    );
    assert!(
        dispatched.is_certified_shortcut_for(ExactBooleanOperation::Intersection),
        "{dispatched:?}"
    );
    dispatched
        .validate_against_sources(&separated_left, &separated_right)
        .unwrap();

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
    assert!(containment.mesh().triangles().is_empty());

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

    let request = ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
    with_exact_boolean_evaluation(&left, &right, request, |evaluation| {
        evaluation.validate().unwrap();
        evaluation.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            evaluation.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
    });
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

    let request = ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
    with_exact_boolean_evaluation(&left, &right, request, |evaluation| {
        evaluation.validate().unwrap();
        evaluation.validate_against_sources(&left, &right).unwrap();
    });
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

    let request = ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
    with_exact_boolean_evaluation(&left, &right, request, |evaluation| {
        evaluation.validate().unwrap();
        evaluation.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            evaluation.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert_eq!(
            evaluation.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
    });

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
    assert_eq!(
        result.freshness_against_sources(&left, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );
    assert!(result.mesh().facts().mesh.closed_manifold);

    let intersection_request = ExactBooleanRequest::new(
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    );
    with_exact_boolean_evaluation(&left, &right, intersection_request, |evaluation| {
        evaluation.validate().unwrap();
        evaluation.validate_against_sources(&left, &right).unwrap();
    });

    let axis_left = axis_aligned_box([0, 0, 0], [1, 1, 1]);
    let axis_right = axis_aligned_box([1, 0, 0], [2, 1, 1]);

    let axis_replay = exact_boolean_result(
        &axis_left,
        &axis_right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(
        axis_replay.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Union),
        "{axis_replay:?}"
    );

    let crossing_right = tetra_from_corners([1, 1, -1], [5, 1, -1], [1, 5, -1], [1, 1, 3]);
    let crossing_request =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
    with_exact_boolean_evaluation(&left, &crossing_right, crossing_request, |evaluation| {
        evaluation.validate().unwrap();
        evaluation
            .validate_against_sources(&left, &crossing_right)
            .unwrap();
    });
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
            with_exact_boolean_arrangement_attempt!(
                &left,
                &right,
                ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                |closed_attempt| {
                    closed_attempt.validate().unwrap();
                    closed_attempt
                        .validate_against_sources_for_request(
                            &left,
                            &right,
                            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
                        )
                        .unwrap();
                    assert_eq!(
                        closed_attempt.freshness_against_sources_for_request(
                            &left,
                            &right,
                            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
                        ),
                        ExactReportFreshness::Current
                    );
                    assert_eq!(
                        closed_attempt.freshness_against_sources_for_request(
                            &left,
                            &right,
                            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
                        ),
                        ExactReportFreshness::SourceReplayMismatch
                    );
                },
            );
        }

        with_exact_boolean_arrangement_attempt!(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
            ExactRegularizationPolicy::REGULARIZED_SOLID,
            |attempt| {
                attempt.validate().unwrap();
                attempt
                    .validate_against_sources_for_request(
                        &left,
                        &right,
                        ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
                    )
                    .unwrap();
                assert_eq!(
                    attempt.freshness_against_sources_for_request(
                        &left,
                        &right,
                        ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
                    ),
                    ExactReportFreshness::Current
                );
                assert_eq!(
                    attempt.freshness_against_sources_for_request(
                        &left,
                        &right,
                        ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
                    ),
                    ExactReportFreshness::SourceReplayMismatch
                );
            },
        );

        let result = exact_boolean_result(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        with_exact_boolean_evaluation(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
            |evaluation| evaluation.validate().unwrap(),
        );
        if matches!(operation, ExactBooleanOperation::Intersection) {
            assert!(result.mesh().triangles().is_empty());
        } else {
            assert!(!result.mesh().triangles().is_empty());
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
        }
    }
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
        with_exact_boolean_arrangement_attempt!(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
            ExactRegularizationPolicy::REGULARIZED_SOLID,
            |closed_attempt| {
                closed_attempt.validate().unwrap();
                closed_attempt
                    .validate_against_sources_for_request(
                        &left,
                        &right,
                        ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
                    )
                    .unwrap();
                assert_eq!(
                    closed_attempt.freshness_against_sources_for_request(
                        &left,
                        &right,
                        ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
                    ),
                    ExactReportFreshness::Current
                );
                assert_eq!(
                    closed_attempt.freshness_against_sources_for_request(
                        &left,
                        &right,
                        ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
                    ),
                    ExactReportFreshness::SourceReplayMismatch
                );
            },
        );

        with_exact_boolean_arrangement_attempt!(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
            ExactRegularizationPolicy::REGULARIZED_SOLID,
            |boundary_attempt| {
                boundary_attempt.validate().unwrap();
                boundary_attempt
                    .validate_against_sources_for_request(
                        &left,
                        &right,
                        ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
                    )
                    .unwrap();
                assert_eq!(
                    boundary_attempt.freshness_against_sources_for_request(
                        &left,
                        &right,
                        ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
                    ),
                    ExactReportFreshness::Current
                );
                assert_eq!(
                    boundary_attempt.freshness_against_sources_for_request(
                        &left,
                        &right,
                        ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
                    ),
                    ExactReportFreshness::SourceReplayMismatch
                );
            },
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
    assert_eq!(
        result.freshness_against_sources(&left, &right),
        ExactReportFreshness::Current
    );
    assert_eq!(
        result.freshness_against_sources(&left, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );
    assert!(!result.mesh().triangles().is_empty());
    assert_eq!(
        result.mesh().validation_policy(),
        ValidationPolicy::ALLOW_BOUNDARY
    );
    with_exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::SelectedRegions(selection),
            validation,
        ),
        |evaluation| {
            evaluation.validate().unwrap();
            evaluation.validate_against_sources(&left, &right).unwrap();
        },
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
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert!(!result.mesh().triangles().is_empty());
        assert_eq!(
            result.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
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
        let request = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED);
        with_exact_boolean_evaluation(&left, &right, request, |evaluation| {
            evaluation.validate().unwrap();
            evaluation.validate_against_sources(&left, &right).unwrap();
            let materialized = evaluation
                .materialized_result()
                .expect("lower-dimensional regularized evaluation should materialize");
            assert!(
                materialized.is_certified_shortcut_for(operation),
                "{operation:?}: {evaluation:?}"
            );
            assert_eq!(
                evaluation.freshness_against_sources(&left, &closed_right),
                ExactReportFreshness::SourceReplayMismatch
            );
            assert!(!evaluation.has_retained_graph_evidence());
            assert_eq!(
                evaluation.freshness_against_sources(&left, &right),
                ExactReportFreshness::Current
            );
        });

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
        assert!(result.mesh().triangles().is_empty());
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
        let disjoint_request = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED);
        with_exact_boolean_evaluation(&left, &disjoint_right, disjoint_request, |evaluation| {
            evaluation.validate().unwrap();
            evaluation
                .validate_against_sources(&left, &disjoint_right)
                .unwrap();
            let materialized = evaluation
                .materialized_result()
                .expect("disjoint lower-dimensional evaluation should materialize");
            assert!(
                materialized.is_certified_shortcut_for(operation),
                "{operation:?}: {evaluation:?}"
            );
        });
        let disjoint_result = exact_boolean_result(
            &left,
            &disjoint_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert!(
            disjoint_result.is_certified_shortcut_for(operation),
            "{operation:?}: {disjoint_result:?}"
        );
        assert!(disjoint_result.mesh().triangles().is_empty());
        assert!(disjoint_result.mesh().facts().mesh.closed_manifold);
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
            if keeps_solid {
                assert!(result.mesh().facts().mesh.closed_manifold);
                assert!(!result.mesh().triangles().is_empty());
            } else {
                assert!(
                    result.mesh().triangles().is_empty(),
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
            let result = exact_boolean_result(
                left,
                right,
                ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
            );
            assert!(
                result.is_certified_shortcut_for(operation),
                "{operation:?}: {result:?}"
            );
            let keeps_solid = matches!(operation, ExactBooleanOperation::Union)
                || (solid_is_left && matches!(operation, ExactBooleanOperation::Difference));
            assert_eq!(
                result.mesh().triangles().is_empty(),
                !keeps_solid,
                "{operation:?}: {result:?}"
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
        let mut reject_workspace = ExactBooleanWorkspace::new(&left, &right);
        let reject_evaluation = reject_workspace
            .evaluate(ExactBooleanRequest::with_boundary_policy(
                operation,
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBoundaryBooleanPolicy::Reject,
            ))
            .unwrap();
        reject_evaluation.validate().unwrap();
        assert!(
            reject_evaluation.materialized_result().is_none(),
            "{reject_evaluation:?}"
        );
        assert!(!reject_evaluation.is_certified());
        assert!(reject_evaluation.has_blocker());

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
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert_eq!(
            result.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
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
        let preflight_request =
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY);
        with_exact_boolean_evaluation(&left, &right, preflight_request, |preflight_evaluation| {
            assert!(
                preflight_evaluation.is_certified(),
                "{operation:?}: {preflight_evaluation:?}"
            );
            assert!(
                preflight_evaluation.has_retained_graph_evidence(),
                "closed boundary-touching request should retain graph evidence: {operation:?}: {preflight_evaluation:?}"
            );
            preflight_evaluation.validate().unwrap();
            preflight_evaluation
                .validate_against_sources(&left, &right)
                .unwrap_or_else(|error| panic!("{operation:?}: {error:?}"));
            assert!(preflight_evaluation.validate().is_ok());
        });

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
        with_exact_boolean_evaluation(
            &left,
            &right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
            |evaluation| evaluation.validate().unwrap(),
        );
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert_eq!(
            result.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
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

    let mut retained_requires_coplanar_volumetric_cells = None;

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let preflight_request =
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY);
        let requires_coplanar_volumetric_cells = with_exact_boolean_evaluation(
            &left,
            &right,
            preflight_request,
            |preflight_evaluation| {
                assert!(
                    preflight_evaluation.materializes_arrangement_cell_complex(),
                    "{operation:?}: {preflight_evaluation:?}"
                );
                assert!(
                    preflight_evaluation.has_retained_graph_evidence(),
                    "positive-area no-volume shortcut should retain graph evidence: {operation:?}: {preflight_evaluation:?}"
                );
                preflight_evaluation.validate().unwrap();
                assert!(
                    preflight_evaluation.has_coplanar_volumetric_evidence(),
                    "positive-area no-volume shortcut should retain source-aware boundary-only evidence"
                );
                preflight_evaluation.requires_coplanar_volumetric_cells()
            },
        );
        if let Some(retained_requires_coplanar_volumetric_cells) =
            retained_requires_coplanar_volumetric_cells
        {
            assert_eq!(
                requires_coplanar_volumetric_cells, retained_requires_coplanar_volumetric_cells,
                "{operation:?}: positive-area no-volume shortcut should retain stable source-aware boundary-only evidence policy"
            );
        } else {
            retained_requires_coplanar_volumetric_cells = Some(requires_coplanar_volumetric_cells);
        }

        if matches!(
            operation,
            ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
        ) {
            let readiness_request = ExactBooleanRequest::with_boundary_policy(
                operation,
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            );
            with_exact_boolean_evaluation(
                &left,
                &right,
                readiness_request,
                |readiness_evaluation| {
                    assert!(
                        readiness_evaluation.materializes_arrangement_cell_complex(),
                        "{operation:?}: {readiness_evaluation:?}"
                    );
                    assert!(readiness_evaluation.has_coplanar_volumetric_evidence());
                    assert_eq!(
                        readiness_evaluation.requires_coplanar_volumetric_cells(),
                        retained_requires_coplanar_volumetric_cells
                            .expect("preflight should retain coplanar volumetric evidence policy"),
                        "{operation:?}: no-volume readiness should retain consumed source-aware evidence policy"
                    );
                },
            );
            with_exact_boolean_evaluation(
                &left,
                &right,
                ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
                |evaluation| evaluation.validate().unwrap(),
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
            assert_eq!(
                result.mesh().triangles().len(),
                left.triangles().len() + right.triangles().len()
            );
            assert!(result.mesh().facts().mesh.closed_manifold);
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
        with_exact_boolean_evaluation(
            &separated_left,
            &separated_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
            |separated_evaluation| separated_evaluation.validate().unwrap(),
        );
        assert_eq!(
            result.freshness_against_sources(&separated_left, &intersecting_right),
            ExactReportFreshness::SourceReplayMismatch
        );
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
        let containment_replay = exact_boolean_result(
            &container,
            &contained,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert_convex_public_replay(&containment_replay, operation);
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
    {
        let evaluation = workspace.evaluate(union_request).unwrap();
        assert!(
            evaluation.materializes_arrangement_cell_complex(),
            "{evaluation:?}"
        );
        evaluation.validate().unwrap();

        assert!(
            evaluation.materializes_arrangement_cell_complex(),
            "{evaluation:?}"
        );
        assert!(
            evaluation.retained_arrangement_attempt().is_some(),
            "{evaluation:?}"
        );

        let result = evaluation
            .materialized_result()
            .expect("certified arrangement evaluation should retain union result");

        if !result.is_arrangement_cell_complex_materialized_for(ExactBooleanOperation::Union) {
            assert!(
                result.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Union),
                "{result:?}"
            );
            assert_evaluation_retains_attempt_gate_reports(evaluation);
        }

        result.validate().unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        evaluation.validate().unwrap();
        assert!(
            evaluation.materializes_arrangement_cell_complex(),
            "{evaluation:?}"
        );
        if !result.is_arrangement_cell_complex_materialized_for(ExactBooleanOperation::Union) {
            assert!(
                result.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Union),
                "{result:?}"
            );
            assert_evaluation_retains_attempt_gate_reports(evaluation);
        }
        assert!(!result.mesh().triangles().is_empty());
        assert!(
            !result
                .is_arrangement_cell_complex_materialized_for(ExactBooleanOperation::Intersection)
        );
        assert!(
            !result.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Intersection)
        );
        assert!(!result.is_certified_shortcut_for(ExactBooleanOperation::Intersection));
        assert_eq!(
            result.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
    }
    let difference_request = ExactBooleanRequest::new(
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    );
    let difference_evaluation = workspace.evaluate(difference_request).unwrap();
    difference_evaluation.validate().unwrap();
    difference_evaluation
        .validate_against_sources(&left, &right)
        .unwrap();
    let difference = difference_evaluation
        .materialized_result()
        .expect("certified arrangement evaluation should retain difference result");
    difference.validate().unwrap();
    if !difference.is_arrangement_cell_complex_materialized_for(ExactBooleanOperation::Difference) {
        assert!(
            difference.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Difference),
            "{difference:?}"
        );
        assert_evaluation_retains_attempt_gate_reports(difference_evaluation);
    }
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
        let evaluation_request =
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY);
        let mut workspace = ExactBooleanWorkspace::new(&left, &right);
        {
            let evaluation = workspace.evaluate(evaluation_request).unwrap();
            assert!(
                evaluation.materializes_arrangement_cell_complex(),
                "{operation:?}: {evaluation:?}"
            );
            evaluation.validate().unwrap();
            evaluation.validate_against_sources(&left, &right).unwrap();

            assert!(
                evaluation.materializes_arrangement_cell_complex(),
                "{operation:?}: {evaluation:?}"
            );
            assert_evaluation_retains_attempt_gate_reports(evaluation);
        }

        let result = workspace
            .materialize(ExactBooleanRequest::new(
                operation,
                ValidationPolicy::CLOSED,
            ))
            .unwrap();
        assert!(
            result.is_arrangement_cell_complex_shortcut_for(operation),
            "{operation:?}: {result:?}"
        );
        result.validate().unwrap();
        assert!(
            result.mesh().facts().mesh.closed_manifold || result.mesh().triangles().is_empty(),
            "{operation:?}: {:?}",
            result.mesh().facts().mesh
        );
        result.validate_against_sources(&left, &right).unwrap();

        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current,
            "{operation:?}: {result:?}"
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

    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    );
    let mut workspace = ExactBooleanWorkspace::new(&left, &right);
    let result = workspace
        .materialize_ref(request)
        .cloned()
        .unwrap_or_else(|_| workspace.materialize(request).unwrap());
    assert!(
        result.is_arrangement_cell_complex_materialized_for(ExactBooleanOperation::Union)
            || result.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Union),
        "{result:?}"
    );
    result.validate().unwrap();
    let evaluation = workspace.evaluate(request).unwrap();
    evaluation.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        evaluation.freshness_against_sources(&left, &stale_right),
        ExactReportFreshness::SourceReplayMismatch,
        "canonical replay must reject stale source operands"
    );
    if !result.is_arrangement_cell_complex_materialized_for(ExactBooleanOperation::Union) {
        assert!(
            result.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Union),
            "{result:?}"
        );
        assert_evaluation_retains_attempt_gate_reports(evaluation);
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
    let convex_left = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let convex_right = tetra_from_corners([1, 1, 1], [5, 1, 1], [1, 5, 1], [1, 1, 5]);
    let convex_intersection_request = ExactBooleanRequest::new(
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    );
    let mut convex_workspace = ExactBooleanWorkspace::new(&convex_left, &convex_right);
    let convex_intersection = convex_workspace
        .materialize_ref(convex_intersection_request)
        .cloned()
        .unwrap_or_else(|_| {
            convex_workspace
                .materialize(convex_intersection_request)
                .unwrap()
        });
    if !convex_intersection
        .is_arrangement_cell_complex_materialized_for(ExactBooleanOperation::Intersection)
    {
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
            let convex_intersection_evaluation = convex_workspace
                .evaluate(convex_intersection_request)
                .unwrap();
            assert_evaluation_retains_attempt_gate_reports(convex_intersection_evaluation);
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
    let multi_hole_request =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
    with_exact_boolean_evaluation(
        &container,
        &two_caps_right,
        multi_hole_request,
        |multi_hole_evaluation| {
            multi_hole_evaluation.validate().unwrap();
            multi_hole_evaluation
                .validate_against_sources(&container, &two_caps_right)
                .unwrap();
        },
    );

    assert_eq!(
        result.freshness_against_sources(&container, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );
    let split_request =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
    with_exact_boolean_evaluation(
        &split_container,
        &split_crossing_right,
        split_request,
        |split_evaluation| {
            split_evaluation.validate().unwrap();
            split_evaluation
                .validate_against_sources(&split_container, &split_crossing_right)
                .unwrap();
        },
    );

    let square_disk_request =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
    with_exact_boolean_evaluation(
        &square_disk_container,
        &square_disk_cap_right,
        square_disk_request,
        |square_disk_evaluation| {
            square_disk_evaluation.validate().unwrap();
            square_disk_evaluation
                .validate_against_sources(&square_disk_container, &square_disk_cap_right)
                .unwrap();
        },
    );

    let request = ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);
    with_exact_boolean_evaluation(&container, &right, request, |evaluation| {
        evaluation.validate().unwrap();
        evaluation
            .validate_against_sources(&container, &right)
            .unwrap();
        assert_eq!(
            evaluation.freshness_against_sources(&container, &right),
            ExactReportFreshness::Current
        );
        assert_eq!(
            evaluation.freshness_against_sources(&container, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
    });
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
    assert_eq!(
        result.freshness_against_sources(&container, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );
}

#[test]
fn exact_evaluation_preflight_reports_disjoint_bounds_without_retained_pairs() {
    let left = tetra([0, 0, 0]);
    let right = tetra([3, 0, 0]);

    with_exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
        |evaluation| {
            assert!(!evaluation.has_retained_graph_evidence());
            evaluation.validate_against_sources(&left, &right).unwrap();
        },
    );
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

    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    );
    with_exact_boolean_evaluation(&left, &overlapping_right, request, |evaluation| {
        evaluation
            .validate_against_sources(&left, &overlapping_right)
            .unwrap();
        assert!(
            evaluation.materialized_result().is_some()
                || evaluation.retained_arrangement_attempt().is_some()
                || evaluation.has_retained_graph_evidence(),
            "{evaluation:?}"
        );
        assert_eq!(
            evaluation.freshness_against_sources(&left, &overlapping_right),
            ExactReportFreshness::Current
        );
        assert_eq!(
            evaluation.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
    });

    let planar_request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    );
    with_exact_boolean_evaluation(
        &left,
        &overlapping_right,
        planar_request,
        |planar_evaluation| {
            assert!(
                planar_evaluation.materializes_arrangement_cell_complex(),
                "{planar_evaluation:?}"
            );
            planar_evaluation
                .validate_against_sources(&left, &overlapping_right)
                .unwrap();
            assert_eq!(
                planar_evaluation.freshness_against_sources(&left, &overlapping_right),
                ExactReportFreshness::Current
            );
            assert_eq!(
                planar_evaluation.freshness_against_sources(&left, &separated_right),
                ExactReportFreshness::SourceReplayMismatch
            );
        },
    );

    let same_request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    );
    with_exact_boolean_evaluation(&left, &left, same_request, |same_evaluation| {
        assert!(
            same_evaluation.materialized_result().is_some_and(|result| {
                result.is_certified_shortcut_for(ExactBooleanOperation::Union)
            }),
            "{same_evaluation:?}"
        );
        same_evaluation
            .validate_against_sources(&left, &left)
            .unwrap();
        assert_eq!(
            same_evaluation.freshness_against_sources(&left, &left),
            ExactReportFreshness::Current
        );
        assert_eq!(
            same_evaluation.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
    });

    let parallel_right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 1, 2, 0, 1, 0, 2, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    );
    with_exact_boolean_evaluation(&left, &parallel_right, open_request, |open_evaluation| {
        assert!(
            open_evaluation.materialized_result().is_some_and(|result| {
                result.is_certified_shortcut_for(ExactBooleanOperation::Union)
            }),
            "{open_evaluation:?}"
        );
        open_evaluation
            .validate_against_sources(&left, &parallel_right)
            .unwrap();
        assert_eq!(
            open_evaluation.freshness_against_sources(&left, &parallel_right),
            ExactReportFreshness::Current
        );
        assert_eq!(
            open_evaluation.freshness_against_sources(&left, &overlapping_right),
            ExactReportFreshness::SourceReplayMismatch
        );
    });
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

    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    );
    with_exact_boolean_evaluation(&left, &right, request, |evaluation| {
        assert!(!evaluation.is_certified(), "{evaluation:?}");
        assert!(evaluation.materialized_result().is_none(), "{evaluation:?}");
        assert!(evaluation.has_blocker(), "{evaluation:?}");
        assert!(evaluation.has_retained_graph_evidence(), "{evaluation:?}");
        evaluation.validate_against_sources(&left, &right).unwrap();
    });
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

    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    );
    with_exact_boolean_evaluation(&left, &right, request, |evaluation| {
        assert!(!evaluation.is_certified(), "{evaluation:?}");
        assert!(evaluation.materialized_result().is_none(), "{evaluation:?}");
        assert!(evaluation.has_blocker(), "{evaluation:?}");
        evaluation.validate_against_sources(&left, &right).unwrap();
    });
}

#[test]
fn exact_boolean_public_shortcuts_handle_disjoint_operands() {
    let left = tetra([0, 0, 0]);
    let right = tetra([3, 0, 0]);

    with_exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
        |preflight_evaluation| {
            assert!(preflight_evaluation.is_certified());
            assert!(
                preflight_evaluation.materialized_result().is_some(),
                "{preflight_evaluation:?}"
            );
        },
    );

    let union = exact_boolean_result(
        &left,
        &right,
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED),
    );
    assert!(
        union.is_certified_shortcut_for(ExactBooleanOperation::Union),
        "{union:?}"
    );
    union.mesh().validate_retained_state().unwrap();
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
    assert!(!intersection.is_certified_shortcut_for(ExactBooleanOperation::Union));

    assert!(intersection.mesh().triangles().is_empty());
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
         _validation: ValidationPolicy,
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
        with_exact_boolean_evaluation(
            &empty,
            &solid,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
            |evaluation| evaluation.validate().unwrap(),
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
        assert!(empty_open_result.mesh().triangles().is_empty());
        assert!(empty_open_result.mesh().facts().mesh.closed_manifold);

        let replayed_empty_open = exact_boolean_result(
            &empty,
            &open_disjoint_left,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert!(
            replayed_empty_open.is_certified_shortcut_for(operation),
            "{operation:?}: {replayed_empty_open:?}"
        );
        assert!(replayed_empty_open.mesh().triangles().is_empty());

        let open_empty_result = exact_boolean_result(
            &open_disjoint_left,
            &empty,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
        );
        assert!(open_empty_result.is_certified_shortcut_for(operation));
        assert!(open_empty_result.mesh().triangles().is_empty());
        assert!(open_empty_result.mesh().facts().mesh.closed_manifold);
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
        with_exact_boolean_evaluation(
            &solid,
            &disjoint_solid,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
            |evaluation| evaluation.validate().unwrap(),
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
        with_exact_boolean_evaluation(
            &open_identical_left,
            &open_identical_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
            |evaluation| evaluation.validate().unwrap(),
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
        assert!(closed_identical_result.mesh().triangles().is_empty());
        assert!(closed_identical_result.mesh().facts().mesh.closed_manifold);
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
        with_exact_boolean_evaluation(
            &open_identical_left,
            &open_same_surface_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
            |evaluation| evaluation.validate().unwrap(),
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
        assert!(closed_same_surface_result.mesh().triangles().is_empty());
        assert!(
            closed_same_surface_result
                .mesh()
                .facts()
                .mesh
                .closed_manifold
        );
        with_exact_boolean_evaluation(
            &open_identical_left,
            &open_same_surface_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
            |evaluation| evaluation.validate().unwrap(),
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
        with_exact_boolean_evaluation(
            &solid,
            &open_disjoint_left,
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED),
            |evaluation| evaluation.validate().unwrap(),
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
        with_exact_boolean_evaluation(
            &open_disjoint_left,
            &open_disjoint_right,
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY),
            |evaluation| evaluation.validate().unwrap(),
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
            assert_eq!(
                result.freshness_against_sources(&left, &stale_right),
                ExactReportFreshness::SourceReplayMismatch
            );
            result
                .validate_against_sources(&left, right)
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
                        "right_index={right_index} operation={operation:?} error={error:?} result={result:?} replay={replay:?}"
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

    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    );
    with_exact_boolean_arrangement_attempt!(
        &left,
        &right,
        request,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
        |attempt| {
            attempt.validate().unwrap();
            assert!(attempt.topology_assembly_is_complete());
            assert!(attempt.region_ownership_is_volume_resolved());
            attempt
                .validate_against_sources_for_request(&left, &right, request)
                .unwrap();
            assert_eq!(
                attempt.freshness_against_sources_for_request(&left, &right, request),
                ExactReportFreshness::Current
            );
            assert_eq!(
                attempt.freshness_against_sources_for_request(
                    &left,
                    &right,
                    ExactBooleanRequest::with_boundary_policy(
                        ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                        ExactBoundaryBooleanPolicy::Reject,
                    ),
                ),
                ExactReportFreshness::SourceReplayMismatch
            );
        },
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
    let shifted_target = tetra([10, 10, 10]);
    assert!(
        result
            .validate_against_sources(&left, &shifted_target)
            .is_err(),
        "{result:?}"
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
    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    );
    let default_retains_graph_evidence =
        with_exact_boolean_evaluation(&left, &right, request, |evaluation| {
            assert!(evaluation.is_certified(), "{evaluation:?}");
            assert!(evaluation.materialized_result().is_some(), "{evaluation:?}");
            evaluation.validate_against_sources(&left, &right).unwrap();
            evaluation.has_retained_graph_evidence()
        });
    with_exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::with_boundary_policy(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        ),
        |rejected_policy_evaluation| {
            assert!(
                !rejected_policy_evaluation.is_certified(),
                "{rejected_policy_evaluation:?}"
            );
            assert!(
                rejected_policy_evaluation.materialized_result().is_none(),
                "{rejected_policy_evaluation:?}"
            );
            assert!(
                rejected_policy_evaluation.has_blocker(),
                "{rejected_policy_evaluation:?}"
            );
        },
    );

    let policy_request = ExactBooleanRequest::with_boundary_policy(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    );
    with_exact_boolean_evaluation(&left, &right, policy_request, |policy_evaluation| {
        assert!(policy_evaluation.is_certified(), "{policy_evaluation:?}");
        assert!(
            policy_evaluation.materialized_result().is_some(),
            "{policy_evaluation:?}"
        );
        assert!(!policy_evaluation.has_blocker(), "{policy_evaluation:?}");
        assert_eq!(
            policy_evaluation.has_retained_graph_evidence(),
            default_retains_graph_evidence
        );
        assert!(default_retains_graph_evidence);
        policy_evaluation
            .validate_against_sources(&left, &right)
            .unwrap();
        assert_eq!(
            policy_evaluation.freshness_against_sources(&left, &right),
            hypermesh::ExactReportFreshness::Current
        );
        assert!(
            policy_evaluation
                .validate_against_sources(&left, &right)
                .is_ok(),
            "default replay should certify a boundary-policy preflight"
        );
        assert!(policy_evaluation.is_certified(), "{policy_evaluation:?}");
        policy_evaluation
            .validate_against_sources(&left, &right)
            .unwrap();
        assert_eq!(
            policy_evaluation.freshness_against_sources(&left, &right),
            hypermesh::ExactReportFreshness::Current
        );
    });
    with_exact_boolean_evaluation(
        &left,
        &right,
        ExactBooleanRequest::with_boundary_policy(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        ),
        |rejected_policy_evaluation| {
            assert!(
                !rejected_policy_evaluation.is_certified(),
                "strict replay should not certify a boundary-policy shortcut"
            );
            assert!(
                rejected_policy_evaluation.materialized_result().is_none(),
                "strict replay should not materialize a boundary-policy shortcut"
            );
            assert!(
                rejected_policy_evaluation.has_blocker(),
                "strict replay should retain the boundary-policy blocker"
            );
        },
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
    projected.mesh().validate_retained_state().unwrap();
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
    with_exact_boolean_evaluation(&left, &right, policy_request, |policy_evaluation| {
        assert!(
            policy_evaluation
                .validate_against_sources(&left, &separated_right)
                .is_err()
        );
    });

    let closed_intersection_request = ExactBooleanRequest::with_boundary_policy(
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    );
    with_exact_boolean_evaluation(
        &left,
        &right,
        closed_intersection_request,
        |closed_intersection_evaluation| {
            closed_intersection_evaluation.validate().unwrap();
            let materialized = closed_intersection_evaluation
                .materialized_result()
                .expect("closed lower-dimensional intersection should materialize");
            assert!(
                materialized.is_certified_shortcut_for(ExactBooleanOperation::Intersection),
                "{closed_intersection_evaluation:?}"
            );
            closed_intersection_evaluation
                .validate_against_sources(&left, &right)
                .unwrap();
        },
    );
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
    assert!(closed_intersection.mesh().triangles().is_empty());
    assert!(closed_intersection.mesh().facts().mesh.closed_manifold);
    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Difference,
    ] {
        let closed_policy_request = ExactBooleanRequest::with_boundary_policy(
            operation,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        );
        with_exact_boolean_evaluation(
            &left,
            &right,
            closed_policy_request,
            |closed_policy_evaluation| {
                closed_policy_evaluation.validate().unwrap();
                let materialized = closed_policy_evaluation
                    .materialized_result()
                    .expect("closed lower-dimensional policy evaluation should materialize");
                assert!(
                    materialized.is_certified_shortcut_for(operation),
                    "{operation:?}: {closed_policy_evaluation:?}"
                );
                closed_policy_evaluation
                    .validate_against_sources(&left, &right)
                    .unwrap();
            },
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

    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    );
    with_exact_boolean_evaluation(&left, &right, request, |evaluation| {
        assert!(!evaluation.is_certified(), "{evaluation:?}");
        assert!(evaluation.materialized_result().is_none(), "{evaluation:?}");
        assert!(evaluation.has_blocker(), "{evaluation:?}");
        evaluation.validate_against_sources(&left, &right).unwrap();
    });
}
