use super::*;
use crate::mesh::arrangement3d::ExactTopologyAssemblyStatus;
use crate::mesh::arrangement3d::cell_complex::{
    ExactRegionOwnershipReport, ExactRegionOwnershipStatus,
};
use crate::mesh::boolean::evidence::{
    ExactArrangementBooleanShortcutReason, meshes_are_certified_identical,
    meshes_are_certified_same_surface,
};

fn test_preflight(
    request: ExactBooleanRequest,
    left: &ExactMesh,
    right: &ExactMesh,
) -> ExactBooleanPreflight {
    exact_boolean_evaluation_for_replay_result_with_materialization(left, right, request, false)
        .unwrap()
        .preflight
        .clone()
}

fn with_test_evaluation<R>(
    request: ExactBooleanRequest,
    left: &ExactMesh,
    right: &ExactMesh,
    f: impl FnOnce(&ExactBooleanEvaluation) -> R,
) -> R {
    let evaluation =
        exact_boolean_evaluation_for_replay_result_with_materialization(left, right, request, true)
            .unwrap();
    evaluation
        .validate_with_missing_result_policy(false)
        .unwrap();
    f(&evaluation)
}

fn test_materialized_result(
    request: ExactBooleanRequest,
    left: &ExactMesh,
    right: &ExactMesh,
) -> ExactBooleanResult {
    let result =
        materialize_boolean_operation(left, right, request.operation, request.validation).unwrap();
    let evaluation =
        exact_boolean_evaluation_for_replay_result_with_materialization(left, right, request, true)
            .unwrap();
    evaluation
        .validate_with_missing_result_policy(false)
        .unwrap();
    if let Some(retained) = evaluation.result.as_ref() {
        assert!(retained.matches_retained_replay(&result));
    }
    result.validate_against_sources(left, right).unwrap();
    result
}

fn test_winding_evidence(
    request: ExactBooleanRequest,
    left: &ExactMesh,
    right: &ExactMesh,
) -> ExactWindingEvidenceReport {
    let evaluation = exact_boolean_evaluation_for_replay_result_with_materialization(
        left, right, request, false,
    )
    .unwrap();
    evaluation
        .validate_with_missing_result_policy(true)
        .unwrap();
    evaluation.certifications.winding_evidence.clone()
}

fn test_volumetric_boundary_closure(
    request: ExactBooleanRequest,
    left: &ExactMesh,
    right: &ExactMesh,
) -> ExactVolumetricBoundaryClosureReport {
    if matches!(request.operation, ExactBooleanOperation::SelectedRegions(_)) {
        return no_materialized_boundary_output_report(request.operation);
    }
    let graph = build_validated_intersection_graph(left, right).unwrap();
    volumetric_boundary_closure_report_from_graph(&graph, left, right, request.operation).unwrap()
}

fn test_planar_arrangement_report(
    request: ExactBooleanRequest,
    left: &ExactMesh,
    right: &ExactMesh,
) -> ExactPlanarArrangementReport {
    if matches!(request.operation, ExactBooleanOperation::SelectedRegions(_)) {
        return not_named_planar_arrangement_report(request.operation);
    }
    let graph = build_validated_intersection_graph(left, right).unwrap();
    let mut arrangement_cell_complex_preflight: CertifiedArrangementCellComplexPreflightCache =
        None;
    planar_arrangement_report_from_graph_with_cell_complex_cache(
        &graph,
        left,
        right,
        request.operation,
        &mut arrangement_cell_complex_preflight,
        None,
        None,
    )
    .unwrap()
}

fn exact_meshes_have_same_shape(left: &ExactMesh, right: &ExactMesh) -> bool {
    (left.vertices().len() == right.vertices().len()
        && left.vertices().iter().all(|left_point| {
            right
                .vertices()
                .iter()
                .any(|right_point| point3_exact_equal(left_point, right_point) == Some(true))
        })
        && right.vertices().iter().all(|right_point| {
            left.vertices()
                .iter()
                .any(|left_point| point3_exact_equal(left_point, right_point) == Some(true))
        })
        && left.triangles().len() == right.triangles().len())
        || exact_mesh_boundary_edges_match(left, right)
}

#[derive(Clone, Debug)]
struct ExactBoundaryEdge {
    endpoints: [Point3; 2],
    count: usize,
}

fn exact_mesh_boundary_edges_match(left: &ExactMesh, right: &ExactMesh) -> bool {
    let Some(left_edges) = exact_mesh_boundary_edges(left) else {
        return false;
    };
    let Some(right_edges) = exact_mesh_boundary_edges(right) else {
        return false;
    };
    !left_edges.is_empty()
        && left_edges.len() == right_edges.len()
        && left_edges.iter().all(|left_edge| {
            right_edges.iter().any(|right_edge| {
                left_edge.count == right_edge.count
                    && point3_edge_exact_equal(&left_edge.endpoints, &right_edge.endpoints)
                        == Some(true)
            })
        })
        && right_edges.iter().all(|right_edge| {
            left_edges.iter().any(|left_edge| {
                left_edge.count == right_edge.count
                    && point3_edge_exact_equal(&right_edge.endpoints, &left_edge.endpoints)
                        == Some(true)
            })
        })
}

fn exact_mesh_boundary_edges(mesh: &ExactMesh) -> Option<Vec<ExactBoundaryEdge>> {
    let mut edges = Vec::<ExactBoundaryEdge>::new();
    for triangle in mesh.triangles() {
        for [start, end] in triangle_edges(triangle) {
            let edge = [
                mesh.vertices().get(start)?.clone(),
                mesh.vertices().get(end)?.clone(),
            ];
            if let Some(existing) = edges
                .iter_mut()
                .find(|existing| point3_edge_exact_equal(&existing.endpoints, &edge) == Some(true))
            {
                existing.count += 1;
            } else {
                edges.push(ExactBoundaryEdge {
                    endpoints: edge,
                    count: 1,
                });
            }
        }
    }
    if edges.iter().any(|edge| edge.count > 2) {
        return None;
    }
    Some(edges.into_iter().filter(|edge| edge.count == 1).collect())
}

fn triangle_edges(triangle: &Triangle) -> [[usize; 2]; 3] {
    super::super::triangle_edges(triangle.0)
}

fn point3_edge_exact_equal(left: &[Point3; 2], right: &[Point3; 2]) -> Option<bool> {
    Some(
        (point3_exact_equal(&left[0], &right[0])? && point3_exact_equal(&left[1], &right[1])?)
            || (point3_exact_equal(&left[0], &right[1])?
                && point3_exact_equal(&left[1], &right[0])?),
    )
}

fn test_arrangement_attempt(
    request: ExactBooleanRequest,
    left: &ExactMesh,
    right: &ExactMesh,
    policy: ExactRegularizationPolicy,
) -> ExactArrangementBooleanAttempt {
    assert_eq!(policy, ExactRegularizationPolicy::REGULARIZED_SOLID);
    let graph = build_validated_intersection_graph(left, right).unwrap();
    let shortcut_facts =
        ExactBooleanSourceFacts::from_sources(left, right).arrangement_cell_complex_shortcuts;
    let mut retained_arrangement = None;
    let mut retained_attempt = None;
    replay_regularized_arrangement_attempt(
        left,
        right,
        request,
        &graph,
        &shortcut_facts,
        &mut retained_arrangement,
        &mut retained_attempt,
    )
    .unwrap();
    retained_attempt.unwrap()
}

fn test_boundary_touching_report(
    left: &ExactMesh,
    right: &ExactMesh,
) -> ExactBoundaryTouchingReport {
    let graph = build_validated_intersection_graph(left, right).unwrap();
    boundary_touching_report_from_graph(&graph, left, right).unwrap()
}

fn assert_current_arrangement_attempt(
    attempt: &ExactArrangementBooleanAttempt,
    left: &ExactMesh,
    right: &ExactMesh,
) {
    attempt.validate().unwrap();
    let request = ExactBooleanRequest::new(attempt.operation, attempt.output_validation);
    attempt
        .validate_against_sources_for_request(left, right, request)
        .unwrap();
    if attempt.stage == ExactArrangementBooleanStage::Materialized && attempt.output_triangles > 0 {
        let output_facts = attempt
            .output_facts
            .as_ref()
            .expect("materialized attempt should retain output mesh facts");
        assert_eq!(output_facts.vertex_count, attempt.output_vertices);
        assert_eq!(output_facts.face_count, attempt.output_triangles);
    }
}

fn assert_result_retains_attempt_gate_reports(
    result: &ExactBooleanResult,
    attempt: &ExactArrangementBooleanAttempt,
) {
    result.validate().unwrap();
    let (attempt_topology, attempt_ownership) = attempt
        .retained_gate_reports()
        .expect("attempt should retain complete gate reports");
    assert_eq!(
        result.topology_assembly_report.as_ref(),
        Some(attempt_topology)
    );
    assert_eq!(
        result.region_ownership_report.as_ref(),
        Some(attempt_ownership)
    );
}

fn synthetic_arrangement_attempt(
    stage: ExactArrangementBooleanStage,
    decline: Option<ExactArrangementBooleanDecline>,
) -> ExactArrangementBooleanAttempt {
    let mut attempt = not_attempted_arrangement_attempt_for_request(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    attempt.stage = stage;
    attempt.decline = decline;
    attempt
}

#[test]
fn arrangement_shortcut_reason_names_generic_blocker_stage() {
    let cases = [
        (
            synthetic_arrangement_attempt(
                ExactArrangementBooleanStage::ArrangementBuilt,
                Some(ExactArrangementBooleanDecline::ArrangementBlockers(vec![
                    ExactArrangementBlocker::UnresolvedIntersection,
                ])),
            ),
            ExactArrangementBooleanShortcutReason::ArrangementConstructionBlocked,
        ),
        (
            synthetic_arrangement_attempt(
                ExactArrangementBooleanStage::ArrangementBuilt,
                Some(ExactArrangementBooleanDecline::TopologyAssembly(
                    ExactTopologyAssemblyStatus::ArrangementBlocked,
                )),
            ),
            ExactArrangementBooleanShortcutReason::TopologyAssemblyBlocked,
        ),
        (
            synthetic_arrangement_attempt(
                ExactArrangementBooleanStage::Labeled,
                Some(ExactArrangementBooleanDecline::RegionOwnership(
                    ExactRegionOwnershipStatus::RequiresWinding,
                )),
            ),
            ExactArrangementBooleanShortcutReason::RegionOwnershipBlocked,
        ),
        (
            synthetic_arrangement_attempt(
                ExactArrangementBooleanStage::Labeled,
                Some(ExactArrangementBooleanDecline::Selection(
                    ExactArrangementBlocker::UnresolvedRegionClassification,
                )),
            ),
            ExactArrangementBooleanShortcutReason::SelectionBlocked,
        ),
        (
            synthetic_arrangement_attempt(
                ExactArrangementBooleanStage::Selected,
                Some(ExactArrangementBooleanDecline::Simplification(
                    ExactArrangementBlocker::NonManifoldCellComplex,
                )),
            ),
            ExactArrangementBooleanShortcutReason::SimplificationBlocked,
        ),
        (
            synthetic_arrangement_attempt(
                ExactArrangementBooleanStage::Simplified,
                Some(ExactArrangementBooleanDecline::Triangulation(
                    ExactArrangementBlocker::NonManifoldCellComplex,
                )),
            ),
            ExactArrangementBooleanShortcutReason::TriangulationBlocked,
        ),
        (
            synthetic_arrangement_attempt(
                ExactArrangementBooleanStage::Triangulated,
                Some(ExactArrangementBooleanDecline::OutputValidation),
            ),
            ExactArrangementBooleanShortcutReason::OutputValidationBlocked,
        ),
        (
            synthetic_arrangement_attempt(ExactArrangementBooleanStage::Materialized, None),
            ExactArrangementBooleanShortcutReason::GenericMaterializationUnavailable,
        ),
    ];

    for (attempt, expected) in cases {
        assert_eq!(attempt.recovered_shortcut_reason(), expected, "{attempt:?}");
    }
}

#[test]
fn exact_mesh_shape_accepts_same_boundary_with_different_triangulation() {
    let diagonal = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let centered = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0, 2, 2, 0],
        &[0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    assert!(exact_mesh_boundary_edges_match(&diagonal, &centered));
    assert!(exact_meshes_have_same_shape(&diagonal, &centered));
}

#[test]
fn open_surface_disjoint_graph_shortcut_replays_sources_before_acceptance() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 4, 0, 4, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let separated_right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 1, 4, 0, 5, 0, 4, 1],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let overlapping_right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 4, 0, 4, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let stale_graph = build_unvalidated_intersection_graph(&left, &separated_right).unwrap();
    assert!(
        boolean_open_surface_disjoint_meshes_from_graph(
            &stale_graph,
            &left,
            &separated_right,
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
        .is_some()
    );

    assert!(
        boolean_open_surface_disjoint_meshes_from_graph(
            &stale_graph,
            &left,
            &overlapping_right,
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
        .is_none()
    );
}

#[test]
fn certified_selected_region_materialization_rejects_stale_retained_graph() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 4, 0, 4, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let separated_right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 1, 4, 0, 5, 0, 4, 1],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let overlapping_right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 4, 0, 4, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let stale_graph = build_unvalidated_intersection_graph(&left, &separated_right).unwrap();
    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::SelectedRegions(ExactRegionSelection::KeepAll),
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    );
    let shortcut_facts =
        ExactArrangementCellComplexShortcutFacts::from_sources(&left, &overlapping_right);

    assert!(
        try_materialize_certified_boolean_support_with_artifacts(
            &left,
            &overlapping_right,
            request,
            ExactBooleanSupport::SelectedRegionPolicy,
            Some(&stale_graph),
            None,
            None,
            &shortcut_facts,
        )
        .is_err(),
        "certified selected-region materialization must replay retained graph sources"
    );
}

#[test]
fn open_surface_disjoint_preflight_prefers_specific_support_over_cell_complex() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[3, 3, 0, 5, 3, 0, 3, 5, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(!meshes_are_certified_bounds_disjoint(&left, &right));

    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    );
    let preflight = test_preflight(request, &left, &right);
    assert_eq!(
        preflight.support(),
        ExactBooleanSupport::CertifiedOpenSurfaceDisjoint,
        "{preflight:?}"
    );
    assert!(preflight.blocker().is_none(), "{preflight:?}");
    preflight.validate().unwrap();
    preflight
        .validate_against_sources_for_request(&left, &right, request)
        .unwrap();

    with_test_evaluation(request, &left, &right, |evaluation| {
        assert_eq!(
            evaluation.preflight.support(),
            ExactBooleanSupport::CertifiedOpenSurfaceDisjoint,
            "{evaluation:?}"
        );
        let materialized = evaluation
            .result
            .as_ref()
            .expect("open-surface disjoint support should materialize");
        assert!(
            materialized.is_certified_shortcut_kind_for(
                ExactBooleanOperation::Union,
                ExactBooleanShortcutKind::OpenSurfaceDisjoint,
            ),
            "{materialized:?}"
        );
        materialized.validate().unwrap();
        materialized
            .validate_against_sources(&left, &right)
            .unwrap();
    });
}

#[test]
fn coplanar_volumetric_gate_uses_source_side_evidence() {
    let boundary_left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
    let boundary_right = axis_aligned_box_i64([2, 0, 0], [4, 2, 2]);
    let boundary_graph =
        build_validated_intersection_graph(&boundary_left, &boundary_right).unwrap();
    assert!(graph_requires_coplanar_volumetric_cells(
        &ExactBooleanBlocker::from_graph(&boundary_graph, ExactBooleanBlockerKind::Winding)
    ));
    assert!(!graph_requires_coplanar_volumetric_cells_for_sources(
        &boundary_graph,
        &boundary_left,
        &boundary_right
    ));

    let same_side_left = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let same_side_right = same_side_left.clone();
    let same_side_graph =
        build_validated_intersection_graph(&same_side_left, &same_side_right).unwrap();
    assert!(graph_requires_coplanar_volumetric_cells_for_sources(
        &same_side_graph,
        &same_side_left,
        &same_side_right
    ));
}

#[test]
fn arrangement_coplanar_evidence_retains_source_handoff() {
    let boundary_left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
    let boundary_right = axis_aligned_box_i64([2, 0, 0], [4, 2, 2]);
    let boundary_graph =
        build_unvalidated_intersection_graph(&boundary_left, &boundary_right).unwrap();
    validate_graph_source_replay(&boundary_graph, &boundary_left, &boundary_right).unwrap();
    let evidence = certified_arrangement_cell_complex_coplanar_evidence(
        &boundary_graph,
        &boundary_left,
        &boundary_right,
    )
    .expect("fresh boundary-only graph should retain coplanar arrangement evidence");
    assert!(evidence.is_boundary_only_positive_area_contact());

    let stale_right = axis_aligned_box_i64([4, 0, 0], [6, 2, 2]);
    assert!(
        certified_arrangement_cell_complex_coplanar_evidence(
            &boundary_graph,
            &boundary_left,
            &stale_right,
        )
        .is_none(),
        "coplanar arrangement evidence must not survive stale source replay"
    );
    assert!(!graph_requires_coplanar_volumetric_cells_for_sources(
        &boundary_graph,
        &boundary_left,
        &stale_right
    ));
    assert!(
        coplanar_boundary_only_evidence_if_consumed(&boundary_graph, &boundary_left, &stale_right,)
            .is_err(),
        "boundary-only coplanar evidence must reject stale graph/source replay"
    );
    assert!(
        certified_closed_boundary_only_contact_from_graph(
            &boundary_graph,
            &boundary_left,
            &stale_right,
        )
        .is_err(),
        "closed boundary-only contact certification must reject stale graph/source replay"
    );
}

#[test]
fn disconnected_contained_face_adjacent_union_replays_result_source() {
    let left = tetrahedron_i64([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let right = tetrahedron_i64([1, 1, 0], [1, 3, 0], [3, 1, 0], [1, 1, -2]);
    let disjoint_shell = tetrahedron_i64([40, 0, 0], [41, 0, 0], [40, 1, 0], [40, 0, 1]);
    let container = combine_test_meshes(
        &left,
        &disjoint_shell,
        "test disconnected contained-face fixture",
    );
    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ExactMeshValidationPolicy::CLOSED,
    );

    assert_contained_face_adjacent_union_replays(&container, &right, request);

    let square_left = square_pyramid_with_base_i64();
    let square_container = combine_test_meshes(
        &square_left,
        &disjoint_shell,
        "test disconnected multi-face contained-cap fixture",
    );
    let square_disk_cap_right = downward_square_pyramid_with_base_i64([2, 2], [6, 6], -2);
    assert_contained_face_adjacent_union_replays(
        &square_container,
        &square_disk_cap_right,
        request,
    );
}

fn assert_contained_face_adjacent_union_replays(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) {
    let evaluation =
        exact_boolean_evaluation_for_replay_result_with_materialization(left, right, request, true)
            .unwrap();
    evaluation
        .validate_with_missing_result_policy(false)
        .unwrap();
    evaluation.validate_against_sources(left, right).unwrap();
    let result = evaluation
        .result
        .as_ref()
        .expect("contained-face adjacent union should materialize");
    assert!(result.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Union));
    result.validate_against_sources(left, right).unwrap();
}

#[test]
fn exact_boolean_blocker_counts_include_unknown_segment_plane_events() {
    let graph =
        super::super::graph::ExactIntersectionGraph::from_face_pairs(vec![FacePairEvents {
            left_face: 0,
            right_face: 0,
            relation: MeshFacePairRelation::Candidate,
            projection: None,
            events: vec![IntersectionEvent::SegmentPlane {
                segment_side: MeshSide::Left,
                edge: [0, 1],
                plane_side: MeshSide::Right,
                plane_face: 0,
                relation: SegmentPlaneRelation::Unknown,
                point: None,
                parameter: None,
                parameter_ratio: None,
                construction_failure: None,
                endpoint_sides: [None, Some(hyperlimit::PlaneSide::Above)],
            }],
        }]);

    let counts = retained_graph_counts(&graph);
    assert_eq!(counts.candidate_pairs(), 1);
    assert_eq!(counts.unknown_pairs(), 1);
    assert_eq!(
        counts.into_blocker(ExactBooleanBlockerKind::Refinement),
        ExactBooleanBlocker::new(ExactBooleanBlockerKind::Refinement, 1, 0, 0, 1, 0)
    );
}

#[test]
fn selected_overlay_faces_triangulate_simple_coplanar_difference_cells() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 0, 0, 4, 0, 0, 4, 2, 0, 2, 2, 0],
        &[0, 1, 2, 0, 2, 3],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let (carrier_points, projection) = coplanar_mesh_overlay_carrier(&left, &right).unwrap();
    let boundary_policy = coplanar_mesh_overlay_materialized_boundary_policy(
        &left,
        &right,
        ExactArrangement2dSetOperation::Difference,
        true,
    )
    .unwrap();
    let mut rings =
        projected_mesh_boundary_rings(ExactArrangement2dRegion::Left, &left, projection).unwrap();
    rings.extend(
        projected_mesh_boundary_rings(ExactArrangement2dRegion::Right, &right, projection).unwrap(),
    );
    let overlay = build_exact_arrangement2d_overlay_with_boundary_policy(
        &rings,
        ExactArrangement2dSetOperation::Difference,
        boundary_policy,
    );
    assert!(overlay.is_complete());
    let selected_faces = mesh_from_selected_projected_overlay_faces(
        &overlay,
        &carrier_points,
        projection,
        "test selected-face coplanar overlay difference",
    )
    .expect("selected arrangement faces should triangulate directly")
    .expect("selected arrangement faces should produce a mesh");
    let canonical = materialize_coplanar_mesh_overlay_mesh(
        &left,
        &right,
        ExactArrangement2dSetOperation::Difference,
        boundary_policy,
        "test canonical coplanar overlay difference",
        false,
    )
    .expect("canonical overlay should materialize")
    .expect("canonical overlay should produce a mesh");
    assert!(exact_meshes_have_same_shape(&selected_faces, &canonical));
    assert_eq!(
        selected_faces.facts().mesh.boundary_edges,
        canonical.facts().mesh.boundary_edges
    );

    let evidence = test_winding_evidence(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Difference,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &left,
        &right,
    );
    assert_eq!(
        evidence.status(),
        ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized,
        "{evidence:?}"
    );
    assert_eq!(evidence.blocker().kind(), ExactBooleanBlockerKind::Winding);
    evidence.validate_against_sources(&left, &right).unwrap();
}

#[test]
fn selected_region_winding_evidence_classifies_retained_graph_blocker() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 0, 0, 4, 0, 0, 4, 2, 0, 2, 2, 0],
        &[0, 1, 2, 0, 2, 3],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::SelectedRegions(ExactRegionSelection::KeepAll),
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    );
    let evidence = with_test_evaluation(request, &left, &right, |evaluation| {
        assert!(
            evaluation.result.as_ref().is_none(),
            "selected-region evaluation should retain certifications when materialization declines"
        );
        evaluation.certifications.winding_evidence.clone()
    });
    assert_eq!(
        evidence.status(),
        ExactWindingEvidenceStatus::NotNamedOperation
    );
    assert_eq!(
        evidence.blocker().kind(),
        ExactBooleanBlockerKind::PlanarArrangement
    );
    assert_eq!(evidence.blocker().coplanar_overlapping_pairs(), 1);
    assert_eq!(evidence.blocker().coplanar_touching_pairs(), 2);
    evidence.validate_against_sources(&left, &right).unwrap();

    let stale_blocker = evidence
        .blocker()
        .into_blocker(ExactBooleanBlockerKind::Winding);
    let stale = evidence.clone().with_blocker(stale_blocker);
    assert_eq!(
        stale.validate(),
        Err(ExactEvidenceValidationError::WrongBlockerKind)
    );

    let disjoint_right = ExactMesh::from_i64_triangles_with_policy(
        &[8, 0, 0, 12, 0, 0, 12, 4, 0, 8, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let disjoint_evidence = test_winding_evidence(request, &left, &disjoint_right);
    assert_eq!(
        disjoint_evidence.status(),
        ExactWindingEvidenceStatus::NotNamedOperation
    );
    assert_eq!(
        disjoint_evidence.blocker().kind(),
        ExactBooleanBlockerKind::Winding
    );
    assert_eq!(disjoint_evidence.retained_face_pairs(), 0);
    disjoint_evidence.validate().unwrap();
    disjoint_evidence
        .validate_against_sources(&left, &disjoint_right)
        .unwrap();

    let relabeled_blocker = disjoint_evidence
        .blocker()
        .into_blocker(ExactBooleanBlockerKind::BoundaryOnlyContact);
    let relabeled_empty = disjoint_evidence.with_blocker(relabeled_blocker);
    assert_eq!(
        relabeled_empty.validate(),
        Err(ExactEvidenceValidationError::WrongBlockerKind)
    );
}

#[test]
fn selected_overlay_faces_recover_point_touching_hole_components() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 8, 0, 0, 8, 8, 0, 0, 8, 0],
        &[0, 1, 2, 0, 2, 3],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0, //
            3, 3, 0, 5, 3, 0, 5, 5, 0, 3, 5, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let (carrier_points, projection) = coplanar_mesh_overlay_carrier(&left, &right).unwrap();
    let boundary_policy = coplanar_mesh_overlay_materialized_boundary_policy(
        &left,
        &right,
        ExactArrangement2dSetOperation::Difference,
        true,
    )
    .unwrap();
    let mut rings =
        projected_mesh_boundary_rings(ExactArrangement2dRegion::Left, &left, projection).unwrap();
    rings.extend(
        projected_mesh_boundary_rings(ExactArrangement2dRegion::Right, &right, projection).unwrap(),
    );
    let overlay = build_exact_arrangement2d_overlay_with_boundary_policy(
        &rings,
        ExactArrangement2dSetOperation::Difference,
        boundary_policy,
    );
    assert!(overlay.is_complete(), "{:?}", overlay.blockers);

    let selected_faces = mesh_from_selected_projected_overlay_faces(
        &overlay,
        &carrier_points,
        projection,
        "test selected-face point-touching hole overlay difference",
    )
    .expect("selected arrangement faces should recover component loops")
    .expect("selected arrangement faces should produce a mesh");
    let canonical = materialize_coplanar_mesh_overlay_mesh(
        &left,
        &right,
        ExactArrangement2dSetOperation::Difference,
        boundary_policy,
        "test canonical point-touching hole overlay difference",
        false,
    )
    .expect("canonical overlay should materialize")
    .expect("canonical overlay should produce a mesh");
    assert!(exact_meshes_have_same_shape(&selected_faces, &canonical));
}

#[test]
fn selected_overlay_faces_absorb_contained_union_components() {
    let outer_square = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let inner_square = ExactMesh::from_i64_triangles_with_policy(
        &[1, 1, 0, 2, 1, 0, 2, 2, 0, 1, 2, 0],
        &[0, 1, 2, 0, 2, 3],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let (carrier_points, projection) =
        coplanar_mesh_overlay_carrier(&outer_square, &inner_square).unwrap();
    let mut rings =
        projected_mesh_boundary_rings(ExactArrangement2dRegion::Left, &outer_square, projection)
            .unwrap();
    rings.extend(
        projected_mesh_boundary_rings(ExactArrangement2dRegion::Right, &inner_square, projection)
            .unwrap(),
    );
    let overlay = build_exact_arrangement2d_overlay_with_boundary_policy(
        &rings,
        ExactArrangement2dSetOperation::Union,
        ExactArrangement2dBoundaryPolicy::SimplifyCollinear,
    );
    assert!(overlay.is_complete(), "{:?}", overlay.blockers);

    let selected_faces = mesh_from_selected_projected_overlay_faces(
        &overlay,
        &carrier_points,
        projection,
        "test selected-face contained union overlay",
    )
    .expect("selected arrangement faces should absorb contained components")
    .expect("selected arrangement faces should produce a mesh");
    assert!(exact_meshes_have_same_shape(&selected_faces, &outer_square));
}

#[test]
fn projected_overlay_mesh_uses_certified_output_components() {
    let ring = |region, points: &[(i64, i64)]| {
        ExactArrangement2dRegionRing::new(
            region,
            points
                .iter()
                .map(|&(x, y)| Point2::new(Real::from(x), Real::from(y)))
                .collect(),
        )
    };
    let overlay = build_exact_arrangement2d_overlay(
        &[
            ring(
                ExactArrangement2dRegion::Left,
                &[(0, 0), (8, 0), (8, 8), (0, 8)],
            ),
            ring(
                ExactArrangement2dRegion::Left,
                &[(1, 1), (1, 7), (7, 7), (7, 1)],
            ),
            ring(
                ExactArrangement2dRegion::Left,
                &[(3, 3), (5, 3), (5, 5), (3, 5)],
            ),
        ],
        ExactArrangement2dSetOperation::Union,
    );
    assert!(overlay.is_complete(), "{:?}", overlay.blockers);
    assert_eq!(overlay.output_components.len(), 2);

    let mut output_only_overlay = overlay.clone();
    output_only_overlay.faces.clear();
    let carrier_points = [
        Point3::new(Real::from(0), Real::from(0), Real::from(0)),
        Point3::new(Real::from(1), Real::from(0), Real::from(0)),
        Point3::new(Real::from(0), Real::from(1), Real::from(0)),
    ];
    let projection = choose_triangle_projection(&carrier_points).unwrap();

    let mesh = mesh_from_selected_projected_overlay_faces(
        &output_only_overlay,
        &carrier_points,
        projection,
        "test certified output-component overlay",
    )
    .expect("certified output components should triangulate without face-walk replay")
    .expect("certified output components should produce a mesh");
    mesh.validate_retained_state().unwrap();
    assert!(!mesh.triangles().is_empty());

    let mut stale_overlay = overlay;
    let outer_loop = stale_overlay.output_components[0].outer_loop;
    stale_overlay.output_loops[outer_loop].points.truncate(2);
    assert!(
        mesh_from_selected_projected_overlay_faces(
            &stale_overlay,
            &carrier_points,
            projection,
            "test stale certified output-component overlay",
        )
        .expect("stale certified output components should not fail")
        .is_none()
    );
}

#[test]
fn selected_overlay_faces_recover_when_output_loop_ownership_is_blocked() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 8, 0, 0, 8, 8, 0, 0, 8, 0],
        &[0, 1, 2, 0, 2, 3],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0, //
            3, 3, 0, 5, 3, 0, 5, 5, 0, 3, 5, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let (carrier_points, projection) = coplanar_mesh_overlay_carrier(&left, &right).unwrap();
    let boundary_policy = coplanar_mesh_overlay_materialized_boundary_policy(
        &left,
        &right,
        ExactArrangement2dSetOperation::Difference,
        true,
    )
    .unwrap();
    let mut rings =
        projected_mesh_boundary_rings(ExactArrangement2dRegion::Left, &left, projection).unwrap();
    rings.extend(
        projected_mesh_boundary_rings(ExactArrangement2dRegion::Right, &right, projection).unwrap(),
    );
    let overlay = build_exact_arrangement2d_overlay_with_boundary_policy(
        &rings,
        ExactArrangement2dSetOperation::Difference,
        boundary_policy,
    );
    assert!(overlay.is_complete(), "{:?}", overlay.blockers);

    let canonical = mesh_from_selected_projected_overlay_faces(
        &overlay,
        &carrier_points,
        projection,
        "test canonical selected-face overlay",
    )
    .expect("complete overlay should materialize through output components")
    .expect("complete overlay should produce a mesh");

    let mut blocked_loop_ownership = overlay;
    blocked_loop_ownership.output_loops.clear();
    blocked_loop_ownership.output_components.clear();
    blocked_loop_ownership.blockers.push(
        ExactArrangement2dBlocker::OutputLoopBoundaryContainment {
            container_loop: 0,
            child_loop: 1,
        },
    );

    let recovered = mesh_from_selected_projected_overlay_faces(
        &blocked_loop_ownership,
        &carrier_points,
        projection,
        "test selected-face recovery overlay",
    )
    .expect("selected faces should recover when only loop ownership is blocked")
    .expect("selected-face recovery should produce a mesh");
    recovered.validate_retained_state().unwrap();
    assert!(exact_meshes_have_same_shape(&recovered, &canonical));
}

#[test]
fn selected_overlay_faces_recover_selected_boundary_topology_blockers() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 0, 0, 4, 0, 0, 4, 2, 0, 2, 2, 0],
        &[0, 1, 2, 0, 2, 3],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let (carrier_points, projection) = coplanar_mesh_overlay_carrier(&left, &right).unwrap();
    let mut rings =
        projected_mesh_boundary_rings(ExactArrangement2dRegion::Left, &left, projection).unwrap();
    rings.extend(
        projected_mesh_boundary_rings(ExactArrangement2dRegion::Right, &right, projection).unwrap(),
    );
    let mut overlay =
        build_exact_arrangement2d_overlay(&rings, ExactArrangement2dSetOperation::Difference);
    assert!(overlay.is_complete(), "{:?}", overlay.blockers);
    let canonical = mesh_from_selected_projected_overlay_faces(
        &overlay,
        &carrier_points,
        projection,
        "test canonical selected-boundary topology overlay",
    )
    .expect("complete overlay should materialize")
    .expect("complete overlay should produce a mesh");

    overlay.output_loops.clear();
    overlay.output_components.clear();
    overlay
        .blockers
        .push(ExactArrangement2dBlocker::NonManifoldSelectedBoundary { vertex: 0 });

    let recovered = mesh_from_selected_projected_overlay_faces(
        &overlay,
        &carrier_points,
        projection,
        "test recovered selected-boundary topology blocker",
    )
    .expect("selected faces should recover when topology blocker is stale")
    .expect("selected-face recovery should produce a mesh");
    recovered.validate_retained_state().unwrap();
    assert!(exact_meshes_have_same_shape(&recovered, &canonical));
    assert_eq!(
        recovered.facts().mesh.boundary_edges,
        canonical.facts().mesh.boundary_edges
    );
}

#[test]
fn coplanar_overlay_certifies_component_holed_contact_difference() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0],
        &[0, 1, 2, 0, 2, 3],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let opening_plus_hole = ExactMesh::from_i64_triangles_with_policy(
        &[
            8, 8, 0, 12, 10, 0, 8, 12, 0, //
            0, 9, 0, 10, 8, 0, 10, 12, 0, 0, 11, 0, //
            15, 15, 0, 17, 15, 0, 17, 17, 0, 15, 17, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5, 3, 5, 6, //
            7, 8, 9, 7, 9, 10,
        ],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    assert!(
        coplanar_mesh_overlay_materialized_boundary_policy(
            &left,
            &opening_plus_hole,
            ExactArrangement2dSetOperation::Difference,
            true,
        )
        .is_some()
    );
    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Difference,
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    );
    let preflight = test_preflight(request, &left, &opening_plus_hole);
    assert_eq!(
        preflight.support(),
        ExactBooleanSupport::CertifiedArrangementCellComplex,
        "{preflight:?}"
    );
    assert!(preflight.blocker().is_none(), "{preflight:?}");
    preflight
        .validate_against_sources_for_request(&left, &opening_plus_hole, request)
        .unwrap();
    let result = boolean_coplanar_mesh_overlay_optional(
        &left,
        &opening_plus_hole,
        ExactBooleanOperation::Difference,
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
    .expect("certified overlay should materialize component-holed difference");
    assert!(result.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Difference));
}

#[test]
fn coplanar_overlay_materializes_point_touching_hole_difference() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 8, 0, 0, 8, 8, 0, 0, 8, 0],
        &[0, 1, 2, 0, 2, 3],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let touching_holes = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0, //
            3, 3, 0, 5, 3, 0, 5, 5, 0, 3, 5, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let result = boolean_coplanar_mesh_overlay_optional(
        &left,
        &touching_holes,
        ExactBooleanOperation::Difference,
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
    .expect("point-touching holed difference should materialize");
    assert!(result.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Difference));
    result.mesh.validate_retained_state().unwrap();
    result
        .validate_against_sources(&left, &touching_holes)
        .unwrap();
}

#[test]
fn coplanar_overlay_materializes_containment_union_and_intersection() {
    let outer_triangle = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let inner_triangle = ExactMesh::from_i64_triangles_with_policy(
        &[1, 1, 0, 2, 1, 0, 1, 2, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let outer_square = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let inner_square = ExactMesh::from_i64_triangles_with_policy(
        &[1, 1, 0, 2, 1, 0, 2, 2, 0, 1, 2, 0],
        &[0, 1, 2, 0, 2, 3],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    for (outer, inner) in [
        (&outer_triangle, &inner_triangle),
        (&outer_square, &inner_square),
    ] {
        let union = materialize_coplanar_mesh_overlay_mesh(
            outer,
            inner,
            ExactArrangement2dSetOperation::Union,
            ExactArrangement2dBoundaryPolicy::SimplifyCollinear,
            "test coplanar containment union overlay",
            false,
        )
        .expect("containment union should materialize through arrangement overlay")
        .expect("containment union should produce a mesh");
        assert!(exact_meshes_have_same_shape(&union, outer));

        let intersection = materialize_coplanar_mesh_overlay_mesh(
            outer,
            inner,
            ExactArrangement2dSetOperation::Intersection,
            ExactArrangement2dBoundaryPolicy::SimplifyCollinear,
            "test coplanar containment intersection overlay",
            false,
        )
        .expect("containment intersection should materialize through arrangement overlay")
        .expect("containment intersection should produce a mesh");
        assert!(exact_meshes_have_same_shape(&intersection, inner));
    }
}

#[test]
fn arrangement_preempts_multi_triangle_coplanar_overlay_including_containment() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 2, 0, 6, 2, 0, 6, 6, 0, 2, 6, 0],
        &[0, 1, 2, 0, 2, 3],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(coplanar_mesh_overlay_should_preempt_surface_paths(
        &left,
        &right,
        ExactBooleanOperation::Union
    ));

    let inner = ExactMesh::from_i64_triangles_with_policy(
        &[1, 1, 0, 2, 1, 0, 1, 2, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let union = test_materialized_result(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &inner,
        &left,
    );
    assert!(union.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Union));
}

#[test]
fn materialized_arrangement_preflight_probe_certifies_full_pipeline_output() {
    let left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
    let right = axis_aligned_box_i64([1, 1, 0], [3, 3, 2]);
    let graph = build_unvalidated_intersection_graph(&left, &right).unwrap();

    let preflight = certified_arrangement_cell_complex_preflight_if_materialized(
        ExactBooleanOperation::Union,
        &graph,
        &left,
        &right,
    )
    .unwrap()
    .expect("overlapping exact boxes should materialize through arrangement");

    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    );
    preflight.validate().unwrap();
    preflight
        .validate_against_sources_for_request(&left, &right, request)
        .unwrap();
    assert_eq!(
        preflight.support(),
        ExactBooleanSupport::CertifiedArrangementCellComplex
    );
    assert!(preflight.blocker().is_none());
    assert_eq!(preflight.retained_face_pairs(), graph.face_pairs.len());
    assert_eq!(preflight.retained_events(), graph.event_count());

    let materialized = materialize_axis_aligned_orthogonal_solid_cell_output(
        &left,
        &right,
        AxisAlignedOrthogonalSolidOperation::Union,
        "test axis-aligned box arrangement preflight materialization",
        request.validation,
    )
    .unwrap()
    .expect("preflight support should have an exact orthogonal cell output");
    materialized.validate_retained_state().unwrap();
}

#[test]
fn certifications_reuse_regularized_arrangement_attempt_reports() {
    let left = tetrahedron_i64([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let right = tetrahedron_i64([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);
    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    );
    let retained_attempt = test_arrangement_attempt(
        request,
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    let evaluation = exact_boolean_evaluation_for_replay_result_with_materialization(
        &left, &right, request, true,
    )
    .unwrap();
    evaluation
        .validate_with_missing_result_policy(false)
        .unwrap();
    evaluation.validate_against_sources(&left, &right).unwrap();
    let certifications = evaluation.certifications.clone();
    certifications.validate_for_request(request).unwrap();
    let attempt = certifications
        .arrangement_attempt
        .as_ref()
        .expect("nested tetrahedra should retain an arrangement attempt");
    assert_eq!(attempt, &retained_attempt);
    assert!(
        attempt.certifies_regularized_arrangement_cell_complex_output_for_request(request),
        "{attempt:?}"
    );
    assert!(attempt.materialized_without_shortcut(), "{attempt:?}");
    assert!(attempt.topology_assembly_report.is_some());
    assert!(attempt.region_ownership_report.is_some());
}

#[test]
fn axis_aligned_orthogonal_cell_booleans_materialize_from_shortcut_support() {
    let left = axis_aligned_orthogonal_l_solid_i64();
    let right = axis_aligned_box_i64([1, 0, 0], [3, 1, 1]);

    assert!(!is_axis_aligned_box(&left));

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        assert_eq!(
            ExactArrangementCellComplexShortcutFacts::from_sources(&left, &right)
                .certified_support(operation),
            Some(ExactBooleanSupport::CertifiedArrangementCellComplex),
            "{operation:?}"
        );

        let request =
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY);
        let preflight = test_preflight(request, &left, &right);
        assert_eq!(
            preflight.support(),
            ExactBooleanSupport::CertifiedArrangementCellComplex,
            "{operation:?}: {preflight:?}"
        );
        assert!(
            preflight.blocker().is_none(),
            "{operation:?}: {preflight:?}"
        );
        let stale_preflight = preflight
            .clone()
            .with_coplanar_volumetric_retained_face_pair_count(preflight.retained_face_pairs() + 1);
        assert!(
            stale_preflight.validate().is_err(),
            "{operation:?}: {stale_preflight:?}"
        );

        let evidence = test_winding_evidence(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
            &left,
            &right,
        );
        assert_eq!(
            evidence.status(),
            ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized,
            "{operation:?}: {evidence:?}"
        );
        assert!(evidence.status().is_already_materialized());
        assert_eq!(
            evidence.blocker().kind(),
            ExactBooleanBlockerKind::Winding,
            "{operation:?}: {evidence:?}"
        );
        evidence.validate_against_sources(&left, &right).unwrap();

        let request =
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY);
        let planar = test_planar_arrangement_report(request, &left, &right);
        planar.validate().unwrap();
        planar
            .validate_against_sources_for_request(&left, &right, request)
            .unwrap();

        let direct = boolean_arrangement_orthogonal_solid_cell_recovery(
            &left,
            &right,
            operation,
            ExactMeshValidationPolicy::CLOSED,
        )
        .unwrap()
        .expect("orthogonal cell shortcut should materialize through certified replay");
        direct.validate_against_sources(&left, &right).unwrap();

        let attempt = test_arrangement_attempt(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::CLOSED),
            &left,
            &right,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        );
        attempt.validate().unwrap();
        assert_eq!(
            attempt.stage,
            ExactArrangementBooleanStage::Materialized,
            "{operation:?}: {attempt:?}"
        );
        assert!(
            attempt.certifies_regularized_arrangement_cell_complex_shortcut_for_request(
                ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::CLOSED),
            ),
            "{operation:?}: {attempt:?}"
        );

        let result = test_materialized_result(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::CLOSED),
            &left,
            &right,
        );
        assert!(
            result.is_arrangement_cell_complex_shortcut_for(operation),
            "{operation:?}: {result:?}"
        );
        result
            .validate_against_sources(&left, &right)
            .unwrap_or_else(|error| {
                panic!(
                    "{operation:?}: arrangement cell-complex shortcut source replay failed: {error:?}"
                )
            });
        assert!(
            result.mesh.facts().mesh.closed_manifold || result.mesh.triangles().is_empty(),
            "{operation:?}: {:?}",
            result.mesh.facts().mesh
        );
    }
}

#[test]
fn axis_aligned_box_predicate_certifies_box_shape() {
    let box_mesh = axis_aligned_box_i64([0, 0, 0], [1, 1, 1]);
    assert!(try_certified_axis_aligned_box_pair(&box_mesh, &box_mesh).unwrap());

    let tetrahedron = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
    assert!(!try_certified_axis_aligned_box_pair(&tetrahedron, &box_mesh).unwrap());
}

#[test]
fn axis_aligned_orthogonal_union_reaches_generic_arrangement_triangulation() {
    let left = axis_aligned_orthogonal_l_solid_i64();
    let right = axis_aligned_box_i64([1, 0, 0], [3, 1, 1]);
    let graph = build_unvalidated_intersection_graph(&left, &right).unwrap();

    let arrangement = ExactArrangement::from_intersection_graph_with_policy(
        graph,
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();
    let topology_report = arrangement.topology_assembly_report_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    let labeling_policy = arrangement_cell_complex_labeling_policy(
        &arrangement,
        Some(ExactBooleanOperation::Union),
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    let labeled = arrangement.label_regions(labeling_policy).unwrap();
    let ownership_report = labeled.region_ownership_report(&left, &right, labeling_policy);
    let selected = labeled
        .select_with_policy(
            ExactBooleanOperation::Union,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap()
        .with_gate_reports(topology_report, ownership_report);
    let simplified = selected
        .simplify_exact_with_policy(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap();
    let mesh = simplified.triangulate().unwrap();
    assert!(!mesh.triangles().is_empty());
}

#[test]
fn arrangement_cell_complex_shortcut_facts_reject_mixed_axis_and_affine_families() {
    let facts = ExactArrangementCellComplexShortcutFacts::from_supports(
        false, true, true, true, true, false, false,
    );
    assert_eq!(
        facts.validate(),
        Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
    );
}

#[test]
fn affine_box_booleans_materialize_from_certified_preflight_support() {
    let left = affine_box_i64([0, 0, 0], [2, 2, 2]);
    let right = affine_box_i64([1, 0, 0], [3, 2, 2]);

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let request =
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY);
        let preflight = test_preflight(request, &left, &right);
        assert_eq!(
            preflight.support(),
            ExactBooleanSupport::CertifiedArrangementCellComplex,
            "{operation:?}: {preflight:?}"
        );
        assert!(
            preflight.blocker().is_none(),
            "{operation:?}: {preflight:?}"
        );

        let result = test_materialized_result(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::CLOSED),
            &left,
            &right,
        );
        assert!(
            result.is_arrangement_cell_complex_shortcut_for(operation),
            "{operation:?}: {result:?}"
        );
        result.validate().unwrap();
        result
            .validate_against_sources(&left, &right)
            .unwrap_or_else(|error| {
                panic!(
                    "{operation:?}: arrangement cell-complex shortcut source replay failed: {error:?}"
                )
            });
        assert!(
            result.mesh.facts().mesh.closed_manifold,
            "{operation:?}: {:?}",
            result.mesh.facts().mesh
        );
        assert!(
            !result.mesh.triangles().is_empty(),
            "{operation:?}: {result:?}"
        );
    }
}

#[test]
fn affine_empty_intersection_materializes_without_winding_fallback() {
    let left = skew_affine_box_i64([0, 0, 0], [1, 1, 1]);
    let right = skew_affine_box_i64([2, 0, 0], [3, 1, 1]);

    assert!(!meshes_are_certified_bounds_disjoint(&left, &right));
    assert!(has_empty_affine_orthogonal_solid_cell_intersection(
        &left, &right
    ));

    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Intersection,
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    );
    let preflight = test_preflight(request, &left, &right);
    assert_eq!(
        preflight.support(),
        ExactBooleanSupport::CertifiedArrangementCellComplex,
        "{preflight:?}"
    );
    assert!(preflight.blocker().is_none(), "{preflight:?}");

    let result = test_materialized_result(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ExactMeshValidationPolicy::CLOSED,
        ),
        &left,
        &right,
    );
    assert!(result.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Intersection));
    result.validate_against_sources(&left, &right).unwrap();
    assert!(result.mesh.triangles().is_empty());
}

#[test]
fn affine_shortcut_winding_report_retains_already_materialized_status() {
    let left = skew_affine_box_i64([0, 0, 0], [2, 2, 2]);
    let right = skew_affine_box_i64([1, 1, 1], [3, 3, 3]);

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let request =
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY);
        let preflight = test_preflight(request, &left, &right);
        assert_eq!(
            preflight.support(),
            ExactBooleanSupport::CertifiedArrangementCellComplex,
            "{operation:?}: {preflight:?}"
        );

        let evidence = test_winding_evidence(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
            &left,
            &right,
        );
        assert_eq!(
            evidence.status(),
            ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized,
            "{operation:?}: {evidence:?}"
        );
        assert!(evidence.status().is_already_materialized());
        assert_eq!(
            evidence.blocker().kind(),
            ExactBooleanBlockerKind::Winding,
            "{operation:?}: {evidence:?}"
        );
        evidence.validate().unwrap();
        evidence.validate_against_sources(&left, &right).unwrap();

        let request =
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY);
        let planar = test_planar_arrangement_report(request, &left, &right);
        planar.validate().unwrap();
        planar
            .validate_against_sources_for_request(&left, &right, request)
            .unwrap();
    }
}

#[test]
fn winding_evidence_status_partition_identifies_materialized_handoffs() {
    for status in [
        ExactWindingEvidenceStatus::PlanarArrangementAlreadyMaterialized,
        ExactWindingEvidenceStatus::CoplanarVolumetricCellsAlreadyMaterialized,
        ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized,
        ExactWindingEvidenceStatus::MixedDimensionalRegularizedSolidAlreadyMaterialized,
        ExactWindingEvidenceStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized,
        ExactWindingEvidenceStatus::ConvexBooleanAlreadyMaterialized,
        ExactWindingEvidenceStatus::OpenSurfaceArrangementAlreadyMaterialized,
        ExactWindingEvidenceStatus::SurfaceEqualityAlreadyMaterialized,
        ExactWindingEvidenceStatus::ClosedBoundaryTouchingAlreadyMaterialized,
        ExactWindingEvidenceStatus::EmptyOperandAlreadyMaterialized,
        ExactWindingEvidenceStatus::BoundsDisjointAlreadyMaterialized,
        ExactWindingEvidenceStatus::OpenSurfaceDisjointAlreadyMaterialized,
        ExactWindingEvidenceStatus::ClosedWindingSeparatedAlreadyMaterialized,
        ExactWindingEvidenceStatus::ClosedWindingContainmentAlreadyMaterialized,
    ] {
        assert!(status.is_already_materialized());
    }

    for status in [
        ExactWindingEvidenceStatus::NotNamedOperation,
        ExactWindingEvidenceStatus::GraphUnknowns,
        ExactWindingEvidenceStatus::BoundaryOnlyContactRequired,
        ExactWindingEvidenceStatus::PlanarArrangementRequired,
        ExactWindingEvidenceStatus::CoplanarVolumetricCellsRequired,
        ExactWindingEvidenceStatus::VolumetricAssemblyRequired,
        ExactWindingEvidenceStatus::NoNontrivialOverlap,
        ExactWindingEvidenceStatus::Ready,
    ] {
        assert!(!status.is_already_materialized());
    }
    let ready_report = ExactWindingEvidenceReport::new(
        ExactBooleanOperation::Union,
        ExactWindingEvidenceStatus::Ready,
        false,
        1,
        1,
        1,
        Vec::new(),
        ExactBooleanBlocker::new(ExactBooleanBlockerKind::Winding, 1, 0, 0, 0, 0),
        None,
        None,
    );
    assert_eq!(ready_report.status(), ExactWindingEvidenceStatus::Ready);

    for status in [
        ExactWindingEvidenceStatus::PlanarArrangementAlreadyMaterialized,
        ExactWindingEvidenceStatus::CoplanarVolumetricCellsAlreadyMaterialized,
        ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized,
    ] {
        assert!(status.materializes_arrangement_cell_complex());
    }

    assert!(
        !ExactWindingEvidenceStatus::MixedDimensionalRegularizedSolidAlreadyMaterialized
            .materializes_arrangement_cell_complex()
    );
    assert!(
        !ExactWindingEvidenceStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized
            .materializes_arrangement_cell_complex()
    );
    assert!(
        !ExactWindingEvidenceStatus::ConvexBooleanAlreadyMaterialized
            .materializes_arrangement_cell_complex()
    );
    assert!(
        !ExactWindingEvidenceStatus::OpenSurfaceArrangementAlreadyMaterialized
            .materializes_arrangement_cell_complex()
    );
    assert!(
        !ExactWindingEvidenceStatus::SurfaceEqualityAlreadyMaterialized
            .materializes_arrangement_cell_complex()
    );
    assert!(
        !ExactWindingEvidenceStatus::ClosedBoundaryTouchingAlreadyMaterialized
            .materializes_arrangement_cell_complex()
    );
    for status in [
        ExactWindingEvidenceStatus::EmptyOperandAlreadyMaterialized,
        ExactWindingEvidenceStatus::BoundsDisjointAlreadyMaterialized,
        ExactWindingEvidenceStatus::OpenSurfaceDisjointAlreadyMaterialized,
        ExactWindingEvidenceStatus::ClosedWindingSeparatedAlreadyMaterialized,
        ExactWindingEvidenceStatus::ClosedWindingContainmentAlreadyMaterialized,
    ] {
        assert!(!status.materializes_arrangement_cell_complex());
    }

    for status in [
        ExactWindingEvidenceStatus::Ready,
        ExactWindingEvidenceStatus::NoNontrivialOverlap,
        ExactWindingEvidenceStatus::VolumetricAssemblyRequired,
    ] {
        assert!(status.routes_to_certified_winding());
    }

    for status in [
        ExactWindingEvidenceStatus::NotNamedOperation,
        ExactWindingEvidenceStatus::GraphUnknowns,
        ExactWindingEvidenceStatus::BoundaryOnlyContactRequired,
        ExactWindingEvidenceStatus::PlanarArrangementRequired,
        ExactWindingEvidenceStatus::PlanarArrangementAlreadyMaterialized,
        ExactWindingEvidenceStatus::CoplanarVolumetricCellsRequired,
        ExactWindingEvidenceStatus::CoplanarVolumetricCellsAlreadyMaterialized,
        ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized,
        ExactWindingEvidenceStatus::MixedDimensionalRegularizedSolidAlreadyMaterialized,
        ExactWindingEvidenceStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized,
        ExactWindingEvidenceStatus::ConvexBooleanAlreadyMaterialized,
        ExactWindingEvidenceStatus::OpenSurfaceArrangementAlreadyMaterialized,
        ExactWindingEvidenceStatus::SurfaceEqualityAlreadyMaterialized,
        ExactWindingEvidenceStatus::ClosedBoundaryTouchingAlreadyMaterialized,
        ExactWindingEvidenceStatus::EmptyOperandAlreadyMaterialized,
        ExactWindingEvidenceStatus::BoundsDisjointAlreadyMaterialized,
        ExactWindingEvidenceStatus::OpenSurfaceDisjointAlreadyMaterialized,
        ExactWindingEvidenceStatus::ClosedWindingSeparatedAlreadyMaterialized,
        ExactWindingEvidenceStatus::ClosedWindingContainmentAlreadyMaterialized,
    ] {
        assert!(!status.routes_to_certified_winding());
    }
}

#[test]
fn trivial_shortcuts_report_materialized_evidence() {
    let empty = empty_mesh(
        "empty operand evidence fixture",
        ExactMeshValidationPolicy::CLOSED,
    )
    .unwrap();
    let solid = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
    let far_solid = axis_aligned_box_i64([4, 0, 0], [6, 2, 2]);
    let left_open = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, //
            4, 0, 4, //
            0, 4, 0,
        ],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right_open = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 1, //
            4, 0, 5, //
            0, 4, 1,
        ],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(!meshes_are_certified_bounds_disjoint(
        &left_open,
        &right_open
    ));

    for (left, right, validation, support, status, shortcut) in [
        (
            &empty,
            &solid,
            ExactMeshValidationPolicy::CLOSED,
            ExactBooleanSupport::CertifiedEmptyOperand,
            ExactWindingEvidenceStatus::EmptyOperandAlreadyMaterialized,
            ExactBooleanShortcutKind::EmptyOperand,
        ),
        (
            &solid,
            &far_solid,
            ExactMeshValidationPolicy::CLOSED,
            ExactBooleanSupport::CertifiedBoundsDisjoint,
            ExactWindingEvidenceStatus::BoundsDisjointAlreadyMaterialized,
            ExactBooleanShortcutKind::BoundsDisjoint,
        ),
        (
            &left_open,
            &right_open,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
            ExactBooleanSupport::CertifiedOpenSurfaceDisjoint,
            ExactWindingEvidenceStatus::OpenSurfaceDisjointAlreadyMaterialized,
            ExactBooleanShortcutKind::OpenSurfaceDisjoint,
        ),
    ] {
        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            let request =
                ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY);
            let preflight = test_preflight(request, left, right);
            assert_eq!(preflight.support(), support, "{operation:?}: {preflight:?}");
            assert!(
                preflight.blocker().is_none(),
                "{operation:?}: {preflight:?}"
            );

            let evidence = test_winding_evidence(
                ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
                left,
                right,
            );
            assert_eq!(evidence.status(), status, "{operation:?}: {evidence:?}");
            assert_eq!(
                evidence.blocker().kind(),
                ExactBooleanBlockerKind::Winding,
                "{operation:?}: {evidence:?}"
            );
            assert_eq!(evidence.retained_face_pairs(), 0, "{operation:?}");
            assert_eq!(evidence.retained_events(), 0, "{operation:?}");
            assert_eq!(evidence.region_count(), 0, "{operation:?}");
            assert!(evidence.status().is_already_materialized());
            assert!(!evidence.status().materializes_arrangement_cell_complex());
            evidence.validate().unwrap();
            evidence.validate_against_sources(left, right).unwrap();

            let result = test_materialized_result(
                ExactBooleanRequest::new(operation, validation),
                left,
                right,
            );
            assert!(
                result.is_certified_shortcut_kind_for(operation, shortcut),
                "{operation:?}: {result:?}"
            );
            result.validate().unwrap();
            result.validate_against_sources(left, right).unwrap();
        }
    }
}

#[test]
fn graph_empty_containment_routes_named_booleans_through_arrangement_pipeline() {
    let outer = tetrahedron_i64([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let disjoint_shell = tetrahedron_i64([20, 0, 0], [21, 0, 0], [20, 1, 0], [20, 0, 1]);
    let container = concatenate_meshes_with_options(
        &outer,
        &disjoint_shell,
        false,
        "exact disjoint union",
        ExactMeshValidationPolicy::CLOSED,
    )
    .expect("disconnected closed container fixture should validate");
    let contained = tetrahedron_i64([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);
    let uncontained = tetrahedron_i64([30, 0, 0], [31, 0, 0], [30, 1, 0], [30, 0, 1]);

    assert!(container.facts().mesh.closed_manifold);
    assert!(contained.facts().mesh.closed_manifold);
    assert!(!meshes_are_certified_bounds_disjoint(
        &container, &contained
    ));
    let graph = build_unvalidated_intersection_graph(&container, &contained).unwrap();
    validate_graph_source_replay(&graph, &container, &contained).unwrap();
    assert!(!graph.has_unknowns());
    assert!(graph.face_pairs.is_empty());

    for (left, right, right_inside_left) in [
        (&container, &contained, true),
        (&contained, &container, false),
    ] {
        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            let request =
                ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY);
            let preflight = test_preflight(request, left, right);
            assert_eq!(
                preflight.support(),
                ExactBooleanSupport::CertifiedArrangementCellComplex,
                "{right_inside_left:?} {operation:?}: {preflight:?}"
            );
            assert!(
                preflight.blocker().is_none(),
                "{operation:?}: {preflight:?}"
            );
            preflight.validate().unwrap();
            preflight
                .validate_against_sources_for_request(left, right, request)
                .unwrap();

            let evidence = test_winding_evidence(
                ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
                left,
                right,
            );
            assert_eq!(
                evidence.status(),
                ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized,
                "{right_inside_left:?} {operation:?}: {evidence:?}"
            );
            assert_eq!(
                evidence.blocker().kind(),
                ExactBooleanBlockerKind::Winding,
                "{operation:?}: {evidence:?}"
            );
            assert_eq!(evidence.retained_face_pairs(), 0, "{operation:?}");
            assert_eq!(evidence.retained_events(), 0, "{operation:?}");
            assert!(evidence.status().is_already_materialized());
            assert!(evidence.status().materializes_arrangement_cell_complex());
            evidence.validate().unwrap();
            evidence.validate_against_sources(left, right).unwrap();

            let result = test_materialized_result(
                ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::CLOSED),
                left,
                right,
            );
            assert!(
                result.is_arrangement_cell_complex_materialized_for(operation)
                    || result.is_arrangement_cell_complex_shortcut_for(operation),
                "{right_inside_left:?} {operation:?}: {result:?}"
            );
            result.validate().unwrap();
            result.validate_against_sources(left, right).unwrap();
            let mut relabeled_closed_winding = result.clone();
            relabeled_closed_winding.topology_assembly_report = None;
            relabeled_closed_winding.region_ownership_report = None;
            relabeled_closed_winding.kind = ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut: ExactBooleanShortcutKind::ClosedWindingContainment,
            };
            relabeled_closed_winding.validate().unwrap();
            if let Err(error) = relabeled_closed_winding.validate_against_sources(left, right) {
                assert_eq!(
                    error,
                    ExactEvidenceValidationError::SourceReplayMismatch,
                    "{right_inside_left:?} {operation:?}: relabeled arrangement result should only fail source replay"
                );
            }
            let stale_sources_rejected = if right_inside_left {
                result.validate_against_sources(left, &uncontained).is_err()
            } else {
                result
                    .validate_against_sources(&uncontained, right)
                    .is_err()
            };
            assert!(
                stale_sources_rejected,
                "{right_inside_left:?} {operation:?}: {result:?}"
            );

            match (operation, right_inside_left) {
                (ExactBooleanOperation::Union, _) => {
                    assert!(exact_meshes_have_same_shape(&result.mesh, &container));
                }
                (ExactBooleanOperation::Intersection, _) => {
                    assert!(exact_meshes_have_same_shape(&result.mesh, &contained));
                }
                (ExactBooleanOperation::Difference, false) => {
                    assert!(result.mesh.triangles().is_empty());
                }
                (ExactBooleanOperation::Difference, true) => {
                    assert!(result.mesh.facts().mesh.closed_manifold);
                    assert_eq!(
                        result.mesh.triangles().len(),
                        container.triangles().len() + contained.triangles().len()
                    );
                }
                (ExactBooleanOperation::SelectedRegions(_), _) => unreachable!(),
            }
        }
    }
}

#[test]
fn graph_empty_closed_winding_separation_materializes_without_bounds_disjointness() {
    let left_a = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
    let left_b = tetrahedron_i64([10, 0, 0], [11, 0, 0], [10, 1, 0], [10, 0, 1]);
    let left = concatenate_meshes_with_options(
        &left_a,
        &left_b,
        false,
        "exact disjoint union",
        ExactMeshValidationPolicy::CLOSED,
    )
    .unwrap();
    let right = tetrahedron_i64([5, 0, 0], [6, 0, 0], [5, 1, 0], [5, 0, 1]);
    let intersecting_right = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);

    assert!(left.facts().mesh.closed_manifold);
    assert!(right.facts().mesh.closed_manifold);
    assert!(!meshes_are_certified_bounds_disjoint(&left, &right));
    let graph = build_validated_intersection_graph(&left, &right).unwrap();
    assert!(!graph.has_unknowns());
    assert!(graph.face_pairs.is_empty());

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let request =
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY);
        let preflight = test_preflight(request, &left, &right);
        assert_eq!(
            preflight.support(),
            ExactBooleanSupport::CertifiedClosedWindingSeparated,
            "{operation:?}: {preflight:?}"
        );
        assert!(
            preflight.blocker().is_none(),
            "{operation:?}: {preflight:?}"
        );
        preflight.validate().unwrap();
        preflight
            .validate_against_sources_for_request(&left, &right, request)
            .unwrap();

        let evidence = test_winding_evidence(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
            &left,
            &right,
        );
        assert_eq!(
            evidence.status(),
            ExactWindingEvidenceStatus::ClosedWindingSeparatedAlreadyMaterialized,
            "{operation:?}: {evidence:?}"
        );
        assert_eq!(
            evidence.blocker().kind(),
            ExactBooleanBlockerKind::Winding,
            "{operation:?}: {evidence:?}"
        );
        assert_eq!(evidence.retained_face_pairs(), 0, "{operation:?}");
        assert_eq!(evidence.retained_events(), 0, "{operation:?}");
        assert!(evidence.status().is_already_materialized());
        assert!(!evidence.status().materializes_arrangement_cell_complex());
        evidence.validate().unwrap();
        evidence.validate_against_sources(&left, &right).unwrap();

        let result = test_materialized_result(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::CLOSED),
            &left,
            &right,
        );
        assert!(
            result.is_certified_shortcut_kind_for(
                operation,
                ExactBooleanShortcutKind::ClosedWindingSeparated,
            ),
            "{operation:?}: {result:?}"
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        let mut relabeled_arrangement = result.clone();
        relabeled_arrangement.kind = ExactBooleanResultKind::CertifiedShortcut {
            operation,
            shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
        };
        assert_eq!(
            relabeled_arrangement.validate(),
            Err(ExactEvidenceValidationError::InvalidOutputMeshProvenance),
            "{operation:?}: closed-winding separation relabeled as arrangement must fail local provenance"
        );
        assert!(
            result
                .validate_against_sources(&left, &intersecting_right)
                .is_err(),
            "{operation:?}: {result:?}"
        );
        match operation {
            ExactBooleanOperation::Union => {
                assert_eq!(
                    result.mesh.triangles().len(),
                    left.triangles().len() + right.triangles().len()
                );
            }
            ExactBooleanOperation::Intersection => {
                assert!(result.mesh.triangles().is_empty());
            }
            ExactBooleanOperation::Difference => {
                assert!(exact_meshes_have_same_shape(&result.mesh, &left));
            }
            ExactBooleanOperation::SelectedRegions(_) => unreachable!(),
        }
    }
}

#[test]
fn mixed_dimensional_regularized_solid_reports_materialized_evidence() {
    let solid = axis_aligned_box_i64([0, 0, 0], [4, 4, 4]);
    let sheet = ExactMesh::from_i64_triangles_with_policy(
        &[1, 1, 1, 3, 1, 1, 1, 3, 1],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    for (left, right) in [(&solid, &sheet), (&sheet, &solid)] {
        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            let preflight = test_preflight(
                ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
                left,
                right,
            );
            assert_eq!(
                preflight.support(),
                ExactBooleanSupport::CertifiedMixedDimensionalRegularizedSolid,
                "{operation:?}: {preflight:?}"
            );
            assert!(
                preflight.blocker().is_none(),
                "{operation:?}: {preflight:?}"
            );

            let evidence = test_winding_evidence(
                ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
                left,
                right,
            );
            assert_eq!(
                evidence.status(),
                ExactWindingEvidenceStatus::MixedDimensionalRegularizedSolidAlreadyMaterialized,
                "{operation:?}: {evidence:?}"
            );
            assert_eq!(
                evidence.blocker().kind(),
                ExactBooleanBlockerKind::Winding,
                "{operation:?}: {evidence:?}"
            );
            assert_eq!(evidence.retained_face_pairs(), 0);
            assert_eq!(evidence.retained_events(), 0);
            assert!(evidence.status().is_already_materialized());
            assert!(!evidence.status().materializes_arrangement_cell_complex());
            evidence.validate().unwrap();
            evidence.validate_against_sources(left, right).unwrap();

            let result = test_materialized_result(
                ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::CLOSED),
                left,
                right,
            );
            assert!(
                result.is_certified_shortcut_kind_for(
                    operation,
                    ExactBooleanShortcutKind::MixedDimensionalRegularizedSolid,
                ),
                "{operation:?}: {result:?}"
            );
            result.validate().unwrap();
            result.validate_against_sources(left, right).unwrap();
            assert!(
                result.validate_against_sources(&sheet, &sheet).is_err(),
                "{operation:?}: {result:?}"
            );

            let keeps_solid = matches!(operation, ExactBooleanOperation::Union)
                || (std::ptr::eq(left, &solid)
                    && matches!(operation, ExactBooleanOperation::Difference));
            if keeps_solid {
                assert!(exact_meshes_have_same_shape(&result.mesh, &solid));
            } else {
                assert!(
                    result.mesh.triangles().is_empty(),
                    "{operation:?}: {result:?}"
                );
            }
        }
    }
}

#[test]
fn lower_dimensional_regularized_solid_reports_materialized_evidence() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, -1, -1, 1, 3, 1, 1, 3, -1],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let request = ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::CLOSED);
        with_test_evaluation(request, &left, &right, |evaluation| {
            let evidence = &evaluation.certifications.winding_evidence;
            let arrangement_materialized = operation == ExactBooleanOperation::Intersection;
            let expected_status = if arrangement_materialized {
                ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized
            } else {
                ExactWindingEvidenceStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized
            };
            assert_eq!(
                evidence.status(),
                expected_status,
                "{operation:?}: {evidence:?}"
            );
            assert_eq!(
                evidence.blocker().kind(),
                ExactBooleanBlockerKind::Winding,
                "{operation:?}: {evidence:?}"
            );
            assert_eq!(
                evidence.retained_face_pairs(),
                usize::from(arrangement_materialized)
            );
            assert_eq!(
                evidence.retained_events(),
                if arrangement_materialized { 4 } else { 0 }
            );
            assert_eq!(evidence.region_count(), 0);
            assert!(evidence.status().is_already_materialized());
            assert_eq!(
                evidence.status().materializes_arrangement_cell_complex(),
                arrangement_materialized
            );
            evidence.validate().unwrap();
            evaluation.validate_against_sources(&left, &right).unwrap();
            if arrangement_materialized {
                let result = evaluation
                    .result
                    .as_ref()
                    .expect("arrangement-backed lower-dimensional result should materialize");
                assert!(
                    result.is_arrangement_cell_complex_shortcut_for(operation)
                        || result.is_arrangement_cell_complex_materialized_for(operation),
                    "{operation:?}: {result:?}"
                );
                let mut relabeled_lower_dimensional = result.clone();
                relabeled_lower_dimensional.topology_assembly_report = None;
                relabeled_lower_dimensional.region_ownership_report = None;
                relabeled_lower_dimensional.kind = ExactBooleanResultKind::CertifiedShortcut {
                    operation,
                    shortcut: ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid,
                };
                relabeled_lower_dimensional.validate().unwrap();
                assert_eq!(
                    relabeled_lower_dimensional.validate_against_sources(&left, &right),
                    Err(ExactEvidenceValidationError::SourceReplayMismatch),
                    "{operation:?}: arrangement lower-dimensional result relabeled as regularized shortcut must not replay"
                );
            }
        });
    }
}

#[test]
fn closed_preflight_does_not_certify_boundary_only_arrangement_output() {
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
    let right = tetrahedron_i64([1, 1, 1], [5, 1, 1], [1, 5, 1], [1, 1, 5]);
    assert!(left.facts().mesh.closed_manifold, "{:?}", left.facts().mesh);
    assert!(right.facts().mesh.closed_manifold);
    let graph = build_validated_intersection_graph(&left, &right).unwrap();

    let reject_closed_request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ExactMeshValidationPolicy::CLOSED,
    );
    let preflight = test_preflight(reject_closed_request, &left, &right);
    assert_eq!(
        preflight.support(),
        ExactBooleanSupport::RequiresCertifiedWinding,
        "{preflight:?}"
    );
    assert!(preflight.blocker().is_some(), "{preflight:?}");
    preflight.validate().unwrap();
    preflight
        .validate_against_sources_for_request(&left, &right, reject_closed_request)
        .unwrap();
    let fake_shortcut = ExactBooleanResult {
        kind: ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Union,
            shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
        },
        graph_had_unknowns: false,
        region_classifications: Vec::new(),
        triangulations: Vec::new(),
        assembly: ExactBooleanAssemblyPlan {
            vertices: Vec::new(),
            triangles: Vec::new(),
        },
        volumetric_classifications: Vec::new(),
        topology_assembly_report: None,
        region_ownership_report: None,
        mesh: empty_mesh(
            "fake closed arrangement shortcut for unresolved winding case",
            ExactMeshValidationPolicy::CLOSED,
        )
        .unwrap(),
    };
    assert!(
        fake_shortcut.validate().is_err(),
        "empty arrangement-cell union shortcut must fail local shape validation"
    );
    assert!(
        fake_shortcut
            .validate_against_sources(&left, &right)
            .is_err(),
        "resolved graph alone must not certify an arrangement-cell shortcut"
    );

    let boundary_preflight = test_preflight(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &left,
        &right,
    );
    assert_eq!(
        boundary_preflight.support(),
        ExactBooleanSupport::CertifiedArrangementCellComplex,
        "{boundary_preflight:?}"
    );
    assert!(
        boundary_preflight.blocker().is_none(),
        "{boundary_preflight:?}"
    );
    assert_eq!(
        boundary_preflight.retained_face_pairs(),
        graph.face_pairs.len()
    );
    assert_eq!(boundary_preflight.retained_events(), graph.event_count());
    boundary_preflight.validate().unwrap();
    boundary_preflight
        .validate_against_sources_for_request(
            &left,
            &right,
            ExactBooleanRequest::new(
                ExactBooleanOperation::Union,
                ExactMeshValidationPolicy::ALLOW_BOUNDARY,
            ),
        )
        .unwrap();
    assert!(
        boundary_preflight
            .validate_against_sources_for_request(&left, &right, reject_closed_request)
            .is_err(),
        "closed replay should not certify an allow-boundary preflight"
    );

    let evidence = test_winding_evidence(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &left,
        &right,
    );
    assert_eq!(
        evidence.status(),
        ExactWindingEvidenceStatus::VolumetricAssemblyRequired,
        "{evidence:?}"
    );
    assert!(evidence.region_count() > 0, "{evidence:?}");
    evidence.validate().unwrap();
    evidence.validate_against_sources(&left, &right).unwrap();

    let boundary_request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    );
    with_test_evaluation(boundary_request, &left, &right, |boundary_evaluation| {
        let boundary_evidence = &boundary_evaluation.certifications.winding_evidence;
        assert_eq!(
            boundary_evidence.status(),
            ExactWindingEvidenceStatus::VolumetricAssemblyRequired,
            "{boundary_evidence:?}"
        );
        assert_eq!(
            boundary_evidence.blocker().kind(),
            ExactBooleanBlockerKind::Winding,
            "{boundary_evidence:?}"
        );
        assert_eq!(
            boundary_evidence.retained_face_pairs(),
            graph.face_pairs.len()
        );
        assert_eq!(boundary_evidence.retained_events(), graph.event_count());
        assert!(boundary_evidence.region_count() > 0);
        assert!(!boundary_evidence.status().is_already_materialized());
        assert!(
            !boundary_evidence
                .status()
                .materializes_arrangement_cell_complex()
        );
        boundary_evidence.validate().unwrap();
    });
}

#[test]
fn volumetric_boundary_closure_report_certifies_triangular_coplanar_cap() {
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
    let right = tetrahedron_i64([-1, 1, 0], [3, 1, 0], [-1, 5, 0], [-1, 1, 4]);

    let closure = test_volumetric_boundary_closure(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &left,
        &right,
    );
    assert_eq!(
        closure.status(),
        &ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable,
        "{closure:?}"
    );
    assert_eq!(closure.boundary_loops(), 1, "{closure:?}");
    assert_eq!(closure.coplanar_loop_groups(), 1, "{closure:?}");
    closure.validate().unwrap();
    closure.validate_against_sources(&left, &right).unwrap();
}

#[test]
fn volumetric_coplanar_boundary_closure_materializes_closed_output() {
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
    let right = tetrahedron_i64([-1, 1, 0], [3, 1, 0], [-1, 5, 0], [-1, 1, 4]);
    let graph = build_unvalidated_intersection_graph(&left, &right).unwrap();
    validate_graph_source_replay(&graph, &left, &right).unwrap();

    let union_closure = test_volumetric_boundary_closure(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &left,
        &right,
    );
    assert_eq!(
        union_closure.status(),
        &ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable,
        "{union_closure:?}"
    );
    union_closure.validate().unwrap();
    union_closure
        .validate_against_sources(&left, &right)
        .unwrap();

    let difference_closure = test_volumetric_boundary_closure(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Difference,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &left,
        &right,
    );
    assert_eq!(
        difference_closure.status(),
        &ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable,
        "{difference_closure:?}"
    );
    difference_closure.validate().unwrap();

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let closure = test_volumetric_boundary_closure(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
            &left,
            &right,
        );
        assert_eq!(
            closure.status(),
            &ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable,
            "{operation:?}: {closure:?}"
        );
        assert_eq!(closure.boundary_loops(), 1, "{operation:?}: {closure:?}");
        assert_eq!(
            closure.coplanar_loop_groups(),
            1,
            "{operation:?}: {closure:?}"
        );
        closure.validate().unwrap();
        closure.validate_against_sources(&left, &right).unwrap();

        let request =
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY);
        let preflight = test_preflight(request, &left, &right);
        assert_eq!(
            preflight.support(),
            ExactBooleanSupport::CertifiedArrangementCellComplex,
            "{operation:?}: {preflight:?}"
        );
        assert!(
            preflight.blocker().is_none(),
            "{operation:?}: {preflight:?}"
        );
        preflight.validate().unwrap();
        preflight
            .validate_against_sources_for_request(&left, &right, request)
            .unwrap();

        let evidence = test_winding_evidence(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
            &left,
            &right,
        );
        assert_eq!(
            evidence.status(),
            ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized,
            "{operation:?}: {evidence:?}"
        );
        evidence.validate().unwrap();

        let result = materialize_arrangement_volumetric_split_cell_result_from_graph(
            &graph,
            &left,
            &right,
            operation,
            ExactMeshValidationPolicy::CLOSED,
        )
        .unwrap()
        .expect("coplanar boundary closure should materialize closed output");
        assert!(
            result.is_arrangement_cell_complex_shortcut_for(operation),
            "{operation:?}: {result:?}"
        );
        result.validate().unwrap();
        assert!(
            result.mesh.facts().mesh.closed_manifold || result.mesh.triangles().is_empty(),
            "{operation:?}: {:?}",
            result.mesh.facts().mesh
        );
        result
            .validate_against_sources(&left, &right)
            .unwrap_or_else(|error| {
                panic!("{operation:?}: closed cap shortcut source replay failed: {error:?}")
            });

        let public = test_materialized_result(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::CLOSED),
            &left,
            &right,
        );
        assert_eq!(public.kind, result.kind, "{operation:?}: {public:?}");
        public.validate().unwrap();
    }
}

fn arrangement_attempt_certified_as_cell_complex_for_preflight_with_validation(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> bool {
    let Ok(graph) = build_validated_intersection_graph(left, right) else {
        return false;
    };
    match run_arrangement_cell_complex_attempt_from_graph(
        &graph,
        left,
        right,
        ExactBooleanRequest::new(operation, validation),
        ExactRegularizationPolicy::REGULARIZED_SOLID,
        true,
    ) {
        Ok(ArrangementCellComplexOutcome::Materialized(result, attempt)) => {
            arrangement_cell_complex_result_is_certified_for_preflight(
                &result, &attempt, left, right,
            )
        }
        Ok(ArrangementCellComplexOutcome::Declined(_)) | Err(_) => false,
    }
}

#[test]
fn arrangement_preflight_probe_keeps_boundary_valid_open_output_separate() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, -1, -1, 1, 3, 1, 1, 3, -1],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
    ] {
        assert!(
            !arrangement_attempt_certified_as_cell_complex_for_preflight_with_validation(
                &left,
                &right,
                operation,
                ExactMeshValidationPolicy::CLOSED
            )
        );
        assert!(
            !arrangement_attempt_certified_as_cell_complex_for_preflight_with_validation(
                &left,
                &right,
                operation,
                ExactMeshValidationPolicy::ALLOW_BOUNDARY
            )
        );
        let request =
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY);
        let preflight = test_preflight(request, &left, &right);
        assert!(
            matches!(
                preflight.support(),
                ExactBooleanSupport::CertifiedOpenSurfaceArrangementUnion
                    | ExactBooleanSupport::CertifiedOpenSurfaceArrangementIntersection
            ),
            "{operation:?}: {preflight:?}"
        );
    }
}

#[test]
fn arrangement_result_retains_consumed_topology_and_ownership_reports() {
    let left = tetrahedron_i64([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let right = tetrahedron_i64([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);

    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ExactMeshValidationPolicy::CLOSED,
    );
    let mut attempt = test_arrangement_attempt(
        request,
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    assert!(attempt.topology_assembly_report.is_some(), "{attempt:?}");
    assert!(attempt.region_ownership_report.is_some(), "{attempt:?}");
    let mesh = copy_mesh(
        &left,
        "exact arrangement cell-complex boolean result",
        ExactMeshValidationPolicy::CLOSED,
    )
    .unwrap();
    let result = certified_shortcut_result(
        mesh,
        ExactBooleanOperation::Union,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    );

    let outcome = materialized_arrangement_attempt_outcome(&mut attempt, result, false, None);
    let ArrangementCellComplexOutcome::Materialized(result, retained_attempt) = outcome else {
        panic!("materialized helper should return a result");
    };
    assert!(retained_attempt.materialized_without_shortcut());
    assert!(
        retained_attempt.certifies_regularized_arrangement_cell_complex_output_for_request(request)
    );
    assert_result_retains_attempt_gate_reports(&result, &retained_attempt);

    let mut missing_generic_evidence = retained_attempt.clone();
    missing_generic_evidence.topology_assembly_report = None;
    assert_eq!(
        missing_generic_evidence.validate(),
        Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
    );

    let mut stale_result = (*result).clone();
    stale_result
        .region_ownership_report
        .as_mut()
        .unwrap()
        .status = ExactRegionOwnershipStatus::RequiresWinding;
    assert_eq!(
        stale_result.validate(),
        Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
    );

    let report_attempt = test_arrangement_attempt(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    let replayable_result = (*result).clone();
    replayable_result.validate().unwrap();
    assert!(
        report_attempt.certifies_regularized_arrangement_cell_complex_output_for_request(
            ExactBooleanRequest::new(
                ExactBooleanOperation::Union,
                ExactMeshValidationPolicy::ALLOW_BOUNDARY,
            )
        )
    );
    assert!(
        retained_attempt
            .certifies_arrangement_cell_complex_output_for_operation(ExactBooleanOperation::Union,)
    );
    let mut relabeled_attempt = retained_attempt.clone();
    relabeled_attempt.operation = ExactBooleanOperation::Difference;
    assert!(
        !relabeled_attempt
            .certifies_arrangement_cell_complex_output_for_operation(ExactBooleanOperation::Union,)
    );
    let mut wrong_validation_attempt = report_attempt.clone();
    wrong_validation_attempt.output_validation = ExactMeshValidationPolicy::CLOSED;
    assert!(
        !wrong_validation_attempt
            .certifies_regularized_arrangement_cell_complex_output_for_request(
                ExactBooleanRequest::new(
                    ExactBooleanOperation::Union,
                    ExactMeshValidationPolicy::ALLOW_BOUNDARY,
                )
            )
    );
    assert_eq!(
        wrong_validation_attempt.validate_for_request_policy(
            ExactBooleanRequest::new(
                ExactBooleanOperation::Union,
                ExactMeshValidationPolicy::ALLOW_BOUNDARY,
            ),
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        ),
        Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
    );
    let mut stale_gate_count = replayable_result.clone();
    stale_gate_count
        .topology_assembly_report
        .as_mut()
        .unwrap()
        .arrangement_face_cells += 1;
    assert_eq!(
        stale_gate_count.validate(),
        Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
    );
    let mut stale_ownership_shape = replayable_result.clone();
    let ownership = stale_ownership_shape
        .region_ownership_report
        .as_mut()
        .unwrap();
    ownership.lower_dimensional_artifacts += 1;
    ownership.lower_dimensional_point_contacts += 1;
    assert_eq!(
        stale_ownership_shape.validate(),
        Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
    );
    let graph = build_validated_intersection_graph(&left, &right).unwrap();
    let mut stale_replay_report = replayable_result.clone();
    stale_replay_report
        .topology_assembly_report
        .as_mut()
        .unwrap()
        .graph_events += 1;
    stale_replay_report.validate().unwrap();
    assert_eq!(
        stale_replay_report.validate_against_sources(&left, &right),
        Err(ExactEvidenceValidationError::SourceReplayMismatch)
    );
    assert_eq!(
        validate_volumetric_arrangement_result_against_graph(
            &stale_replay_report,
            &graph,
            None,
            &left,
            &right,
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        Err(ExactEvidenceValidationError::SourceReplayMismatch)
    );
}

#[test]
fn arrangement_certification_accepts_requested_volume_ownership() {
    let ownership = ExactRegionOwnershipReport {
        status: ExactRegionOwnershipStatus::RequiresWinding,
        freshness:
            crate::mesh::arrangement3d::cell_complex::ExactLabeledCellComplexFreshness::Current,
        blockers: vec![ExactArrangementBlocker::UnresolvedRegionClassification],
        face_cells: 1,
        face_cell_boundary_nodes: 3,
        face_cell_boundary_points: 3,
        left_boundary_faces: 1,
        right_boundary_faces: 0,
        opposite_inside_faces: 0,
        opposite_outside_faces: 0,
        opposite_boundary_faces: 0,
        opposite_unknown_faces: 1,
        volume_regions: 3,
        exterior_volume_regions: 1,
        left_owned_volumes: 1,
        right_owned_volumes: 1,
        shared_owned_volumes: 0,
        unowned_bounded_volumes: 0,
        volume_adjacencies: 2,
        volume_adjacency_face_sides: 2,
        volume_adjacency_separating_faces: 2,
        volume_selection_resolved: false,
        volume_union_resolved: false,
        volume_intersection_resolved: true,
        volume_difference_resolved: true,
        lower_dimensional_artifacts: 0,
        lower_dimensional_point_contacts: 0,
        lower_dimensional_edge_contacts: 0,
        lower_dimensional_edge_endpoints: 0,
    };

    ownership.validate().unwrap();
    assert!(!ownership.is_resolved());
    assert!(ownership.resolves_operation_selection(ExactBooleanOperation::Intersection));
    assert!(ownership.resolves_operation_selection(ExactBooleanOperation::Difference));
    assert!(!ownership.resolves_operation_selection(ExactBooleanOperation::Union));
}

#[test]
fn arrangement_attempt_accepts_requested_volume_ownership() {
    let left = tetrahedron_i64([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let right = tetrahedron_i64([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);
    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Difference,
        ExactMeshValidationPolicy::CLOSED,
    );
    let mut attempt = test_arrangement_attempt(
        request,
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    assert!(attempt.region_ownership_report.is_some(), "{attempt:?}");
    assert!(attempt.volume_adjacencies > 0, "{attempt:?}");
    let mesh = copy_mesh(
        &left,
        "exact arrangement cell-complex boolean result",
        ExactMeshValidationPolicy::CLOSED,
    )
    .unwrap();
    let result = certified_shortcut_result(
        mesh,
        ExactBooleanOperation::Difference,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    );
    let ArrangementCellComplexOutcome::Materialized(_, mut retained_attempt) =
        materialized_arrangement_attempt_outcome(&mut attempt, result, false, None)
    else {
        panic!("materialized helper should return a result");
    };
    assert!(
        retained_attempt.certifies_regularized_arrangement_cell_complex_output_for_request(request)
    );
    assert_eq!(retained_attempt.shortcut_reason, None);

    let mut ownership = retained_attempt.region_ownership_report.clone().unwrap();
    ownership.status = ExactRegionOwnershipStatus::RequiresWinding;
    ownership.blockers = vec![ExactArrangementBlocker::UnresolvedRegionClassification];
    ownership.opposite_inside_faces = 0;
    ownership.opposite_outside_faces = 0;
    ownership.opposite_boundary_faces = 0;
    ownership.opposite_unknown_faces = ownership.face_cells;
    ownership.volume_selection_resolved = false;
    ownership.volume_union_resolved = false;
    ownership.volume_intersection_resolved = false;
    ownership.volume_difference_resolved = true;
    ownership.validate().unwrap();

    retained_attempt.region_ownership = Some(ownership.status);
    retained_attempt.region_ownership_report = Some(ownership.clone());
    if let Some(selected) = retained_attempt.selected_cell_complex.as_mut() {
        selected.region_ownership_report = Some(ownership.clone());
    }
    if let Some(simplified) = retained_attempt.simplified_cell_complex.as_mut() {
        simplified.region_ownership_report = Some(ownership.clone());
    }

    assert!(retained_attempt.resolves_requested_volume_ownership());
    assert!(
        retained_attempt.certifies_regularized_arrangement_cell_complex_output_for_request(request)
    );
    retained_attempt.validate().unwrap();

    let unresolved_for_difference = retained_attempt.region_ownership_report.as_mut().unwrap();
    unresolved_for_difference.volume_difference_resolved = false;
    unresolved_for_difference.validate().unwrap();
    assert!(!retained_attempt.resolves_requested_volume_ownership());
    assert!(
        !retained_attempt
            .certifies_regularized_arrangement_cell_complex_output_for_request(request)
    );
    assert_eq!(
        retained_attempt.validate(),
        Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
    );
}

#[test]
fn retained_volume_ownership_preflight_rejects_stale_source_replay() {
    let left = tetrahedron_i64([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let right = tetrahedron_i64([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);
    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Difference,
        ExactMeshValidationPolicy::CLOSED,
    );
    let mut attempt = test_arrangement_attempt(
        request,
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    let mesh = copy_mesh(
        &left,
        "exact arrangement cell-complex boolean result",
        ExactMeshValidationPolicy::CLOSED,
    )
    .unwrap();
    let result = certified_shortcut_result(
        mesh,
        ExactBooleanOperation::Difference,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    );
    let ArrangementCellComplexOutcome::Materialized(_, mut retained_attempt) =
        materialized_arrangement_attempt_outcome(&mut attempt, result, false, None)
    else {
        panic!("materialized helper should return a result");
    };

    let mut ownership = retained_attempt.region_ownership_report.clone().unwrap();
    ownership.status = ExactRegionOwnershipStatus::RequiresWinding;
    ownership.blockers = vec![ExactArrangementBlocker::UnresolvedRegionClassification];
    ownership.opposite_inside_faces = 0;
    ownership.opposite_outside_faces = 0;
    ownership.opposite_boundary_faces = 0;
    ownership.opposite_unknown_faces = ownership.face_cells;
    ownership.volume_selection_resolved = false;
    ownership.volume_union_resolved = false;
    ownership.volume_intersection_resolved = false;
    ownership.volume_difference_resolved = true;
    ownership.validate().unwrap();

    retained_attempt.region_ownership = Some(ownership.status);
    retained_attempt.region_ownership_report = Some(ownership.clone());
    if let Some(selected) = retained_attempt.selected_cell_complex.as_mut() {
        selected.region_ownership_report = Some(ownership.clone());
    }
    if let Some(simplified) = retained_attempt.simplified_cell_complex.as_mut() {
        simplified.region_ownership_report = Some(ownership);
    }
    retained_attempt.validate().unwrap();
    assert!(retained_attempt.resolves_requested_volume_ownership());

    let graph = build_validated_intersection_graph(&left, &right).unwrap();
    let error = certified_arrangement_cell_complex_preflight_from_retained_attempt(
        &graph,
        &left,
        &right,
        request,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
        &retained_attempt,
    )
    .expect_err("source-stale retained ownership must not certify preflight");
    assert!(
        format!("{error:?}").contains("SourceReplayMismatch"),
        "{error:?}"
    );
}

#[test]
fn retained_result_validation_rejects_stale_supplied_attempt() {
    let left = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
    let right = tetrahedron_i64([1, 0, 0], [2, 0, 0], [1, 1, 0], [1, 0, 1]);
    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ExactMeshValidationPolicy::CLOSED,
    );

    let result = test_materialized_result(request, &left, &right);
    result.validate_against_sources(&left, &right).unwrap();

    let mut stale_attempt = test_arrangement_attempt(
        request,
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    stale_attempt.output_triangles += 1;
    stale_attempt
        .output_facts
        .as_mut()
        .expect("materialized attempt should retain output facts")
        .face_count += 1;
    stale_attempt.validate().unwrap();

    assert_eq!(
        stale_attempt.validate_against_sources_for_request(&left, &right, request),
        Err(ExactEvidenceValidationError::SourceReplayMismatch)
    );
    assert_eq!(
        result.validate_request_against_sources_with_retained_attempt(
            &left,
            &right,
            request,
            Some(&stale_attempt),
        ),
        Err(ExactEvidenceValidationError::SourceReplayMismatch)
    );
}

#[test]
fn non_arrangement_result_validation_ignores_unrelated_stale_attempt() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[10, 10, 0, 14, 10, 0, 10, 14, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    );

    let result = test_materialized_result(request, &left, &right);
    assert!(result.is_certified_shortcut_kind_for(
        ExactBooleanOperation::Union,
        ExactBooleanShortcutKind::BoundsDisjoint,
    ));

    let stale_right = ExactMesh::from_i64_triangles_with_policy(
        &[1, 1, 0, 5, 1, 0, 1, 5, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let mut stale_attempt = test_arrangement_attempt(
        request,
        &left,
        &stale_right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    stale_attempt.output_triangles += 1;
    if let Some(output_facts) = stale_attempt.output_facts.as_mut() {
        output_facts.face_count += 1;
    }
    stale_attempt.validate().unwrap();
    assert_eq!(
        stale_attempt.validate_against_sources_for_request(&left, &right, request),
        Err(ExactEvidenceValidationError::SourceReplayMismatch)
    );

    result
        .validate_request_against_sources_with_retained_attempt(
            &left,
            &right,
            request,
            Some(&stale_attempt),
        )
        .unwrap();
}

#[test]
fn crossing_open_surface_boolean_materializes_inside_arrangement_attempt() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, -1, -1, 1, 3, 1, 1, 3, -1],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let expected_support = match operation {
            ExactBooleanOperation::Union => {
                ExactBooleanSupport::CertifiedOpenSurfaceArrangementUnion
            }
            ExactBooleanOperation::Intersection => {
                ExactBooleanSupport::CertifiedOpenSurfaceArrangementIntersection
            }
            ExactBooleanOperation::Difference => {
                ExactBooleanSupport::CertifiedOpenSurfaceArrangementDifference
            }
            ExactBooleanOperation::SelectedRegions(_) => unreachable!(),
        };
        let request =
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY);
        let preflight = test_preflight(request, &left, &right);
        assert_eq!(
            preflight.support(),
            expected_support,
            "{operation:?}: {preflight:?}"
        );
        assert!(
            preflight.blocker().is_none(),
            "{operation:?}: {preflight:?}"
        );
        assert!(preflight.region_count() > 0, "{operation:?}: {preflight:?}");
        assert!(preflight.validate().is_ok(), "{operation:?}: {preflight:?}");
        preflight
            .validate_against_sources_for_request(&left, &right, request)
            .unwrap();

        let evidence = test_winding_evidence(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
            &left,
            &right,
        );
        assert_eq!(
            evidence.status(),
            ExactWindingEvidenceStatus::OpenSurfaceArrangementAlreadyMaterialized,
            "{operation:?}: {evidence:?}"
        );
        assert_eq!(
            evidence.blocker().kind(),
            ExactBooleanBlockerKind::Winding,
            "{operation:?}: {evidence:?}"
        );
        assert_eq!(evidence.region_count(), preflight.region_count());
        assert_eq!(
            evidence.region_classifications(),
            preflight.region_classifications()
        );
        assert!(evidence.status().is_already_materialized());
        assert!(!evidence.status().materializes_arrangement_cell_complex());
        evidence.validate().unwrap();
        evidence.validate_against_sources(&left, &right).unwrap();

        let attempt = test_arrangement_attempt(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
            &left,
            &right,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        );
        assert_eq!(
            attempt.stage,
            ExactArrangementBooleanStage::Materialized,
            "{operation:?}: {attempt:?}"
        );
        assert!(
            attempt.materialized_arrangement_cell_complex_shortcut(),
            "{operation:?}: {attempt:?}"
        );
        assert!(attempt.decline.is_none(), "{operation:?}: {attempt:?}");
        if !matches!(operation, ExactBooleanOperation::Intersection) {
            assert!(attempt.output_triangles > 0, "{operation:?}: {attempt:?}");
        }
        assert_current_arrangement_attempt(&attempt, &left, &right);

        let result = test_materialized_result(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
            &left,
            &right,
        );
        assert!(
            result.is_open_surface_arrangement_for(operation)
                || result.is_arrangement_cell_complex_shortcut_for(operation),
            "{operation:?}: {result:?}"
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        if result.is_open_surface_arrangement_for(operation) {
            let mut stale_region_fact = result.clone();
            let classification = stale_region_fact
                .region_classifications
                .first_mut()
                .expect("open-surface arrangement should retain region classifications");
            match classification.relation {
                crate::mesh::boolean::region::FaceRegionPlaneRelation::StrictlyAbove => {
                    classification.relation =
                        crate::mesh::boolean::region::FaceRegionPlaneRelation::StrictlyBelow;
                    classification
                        .node_sides
                        .fill(Some(hyperlimit::PlaneSide::Below));
                }
                _ => {
                    classification.relation =
                        crate::mesh::boolean::region::FaceRegionPlaneRelation::StrictlyAbove;
                    classification
                        .node_sides
                        .fill(Some(hyperlimit::PlaneSide::Above));
                }
            }
            stale_region_fact.validate().unwrap();
            assert!(
                stale_region_fact
                    .validate_against_sources(&left, &right)
                    .is_err(),
                "{operation:?}: stale region classification should fail source replay"
            );
            let mut stale_triangulation_fact = result.clone();
            let triangulation = stale_triangulation_fact
                .triangulations
                .iter_mut()
                .find(|triangulation| triangulation.triangles.len() >= 3)
                .expect("open-surface arrangement should retain triangulations");
            triangulation.triangles.swap(0, 1);
            stale_triangulation_fact.validate().unwrap();
            assert!(
                stale_triangulation_fact
                    .validate_against_sources(&left, &right)
                    .is_err(),
                "{operation:?}: stale triangulation should fail source replay"
            );
            if matches!(operation, ExactBooleanOperation::Intersection) {
                let mut incomplete_region_set = result.clone();
                let dropped = incomplete_region_set
                    .triangulations
                    .pop()
                    .expect("open-surface arrangement should retain triangulations");
                incomplete_region_set
                    .region_classifications
                    .retain(|classification| {
                        classification.region_side != dropped.side
                            || classification.region_face != dropped.face
                    });
                incomplete_region_set.validate().unwrap();
                assert!(
                    incomplete_region_set
                        .validate_against_sources(&left, &right)
                        .is_err(),
                    "open-surface intersection must retain the complete replayed region set"
                );
            }
        }
        if result.is_open_surface_arrangement_for(operation) {
            let selection = match operation {
                ExactBooleanOperation::Union => ExactRegionSelection::KeepAll,
                ExactBooleanOperation::Intersection => ExactRegionSelection::KeepNone,
                ExactBooleanOperation::Difference => ExactRegionSelection::KeepLeft,
                ExactBooleanOperation::SelectedRegions(_) => unreachable!(),
            };
            result
                .assembly
                .validate_against_sources(&left, &right, selection)
                .unwrap();
        }
    }
}

#[test]
fn partial_face_boundary_touch_is_regularized_without_coplanar_cell_blocker() {
    let left = tetrahedron_i64([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
    let right = tetrahedron_i64([2, 2, 2], [4, 1, 1], [1, 4, 1], [3, 3, 3]);

    let intersection_request = ExactBooleanRequest::new(
        ExactBooleanOperation::Intersection,
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    );
    let intersection = test_preflight(intersection_request, &left, &right);
    assert_eq!(
        intersection.support(),
        ExactBooleanSupport::CertifiedArrangementCellComplex
    );
    assert!(intersection.retained_face_pairs() > 0, "{intersection:?}");
    assert!(intersection.blocker().is_none());
    intersection.validate().unwrap();
    intersection
        .validate_against_sources_for_request(&left, &right, intersection_request)
        .unwrap();

    let difference_request = ExactBooleanRequest::new(
        ExactBooleanOperation::Difference,
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    );
    let difference = test_preflight(difference_request, &left, &right);
    assert_eq!(
        difference.support(),
        ExactBooleanSupport::CertifiedArrangementCellComplex
    );
    assert!(difference.retained_face_pairs() > 0, "{difference:?}");
    assert!(difference.blocker().is_none());
    difference.validate().unwrap();
    difference
        .validate_against_sources_for_request(&left, &right, difference_request)
        .unwrap();

    let intersection = test_materialized_result(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ExactMeshValidationPolicy::CLOSED,
        ),
        &left,
        &right,
    );
    assert!(
        intersection.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Intersection)
    );
    assert!(intersection.mesh.triangles().is_empty());

    let difference = test_materialized_result(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Difference,
            ExactMeshValidationPolicy::CLOSED,
        ),
        &left,
        &right,
    );
    assert!(difference.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Difference));
    assert!(exact_meshes_have_same_shape(&difference.mesh, &left));
}

#[test]
fn nested_closed_shell_booleans_materialize_through_arrangement_pipeline() {
    let left = tetrahedron_i64([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let right = tetrahedron_i64([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let (expected_support, expected_shortcut) = match operation {
            ExactBooleanOperation::Union => (
                ExactBooleanSupport::CertifiedArrangementCellComplex,
                ExactBooleanShortcutKind::ArrangementCellComplex,
            ),
            ExactBooleanOperation::Intersection => (
                ExactBooleanSupport::CertifiedArrangementCellComplex,
                ExactBooleanShortcutKind::ArrangementCellComplex,
            ),
            ExactBooleanOperation::Difference => (
                ExactBooleanSupport::CertifiedArrangementCellComplex,
                ExactBooleanShortcutKind::ArrangementCellComplex,
            ),
            ExactBooleanOperation::SelectedRegions(_) => unreachable!(),
        };
        let request =
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY);
        let preflight = test_preflight(request, &left, &right);
        assert_eq!(
            preflight.support(),
            expected_support,
            "{operation:?}: {preflight:?}"
        );
        assert!(
            preflight.blocker().is_none(),
            "{operation:?}: {preflight:?}"
        );
        assert_eq!(
            preflight.retained_face_pairs(),
            0,
            "{operation:?}: {preflight:?}"
        );

        let evidence = test_winding_evidence(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
            &left,
            &right,
        );
        assert_eq!(
            evidence.status(),
            ExactWindingEvidenceStatus::ConvexBooleanAlreadyMaterialized,
            "{operation:?}: {evidence:?}"
        );
        assert_eq!(
            evidence.blocker().kind(),
            ExactBooleanBlockerKind::Winding,
            "{operation:?}: {evidence:?}"
        );
        assert_eq!(evidence.retained_face_pairs(), 0);
        assert_eq!(evidence.retained_events(), 0);
        evidence.validate().unwrap();
        evidence.validate_against_sources(&left, &right).unwrap();

        let attempt = test_arrangement_attempt(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
            &left,
            &right,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        );
        assert!(
            attempt.materialized_without_shortcut(),
            "{operation:?}: {attempt:?}"
        );
        assert!(
            attempt.certifies_regularized_arrangement_cell_complex_output_for_request(
                ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
            )
        );
        assert!(attempt.decline.is_none(), "{operation:?}: {attempt:?}");
        assert_current_arrangement_attempt(&attempt, &left, &right);

        let result = test_materialized_result(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::CLOSED),
            &left,
            &right,
        );
        assert!(
            result.is_certified_shortcut_kind_for(operation, expected_shortcut)
                || result.is_arrangement_cell_complex_materialized_for(operation),
            "{operation:?}: {result:?}"
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        result
            .validate_request_against_sources_with_retained_attempt(
                &left,
                &right,
                ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::CLOSED),
                None,
            )
            .unwrap();
        assert!(
            result.mesh.facts().mesh.closed_manifold,
            "{operation:?}: {:?}",
            result.mesh.facts().mesh
        );
    }
}

#[test]
fn closed_boundary_touching_union_materializes_without_shortcut_through_arrangement_pipeline() {
    let left = axis_aligned_box_i64([0, 0, 0], [1, 1, 1]);
    let right = axis_aligned_box_i64([1, 0, 0], [2, 1, 1]);

    let preflight = test_preflight(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &left,
        &right,
    );
    assert_eq!(
        preflight.support(),
        ExactBooleanSupport::CertifiedArrangementCellComplex,
        "{preflight:?}"
    );
    assert!(preflight.blocker().is_none(), "{preflight:?}");

    let attempt = test_arrangement_attempt(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    assert!(attempt.materialized_without_shortcut(), "{attempt:?}");
    assert!(attempt.decline.is_none(), "{attempt:?}");
    assert_current_arrangement_attempt(&attempt, &left, &right);

    let result = test_materialized_result(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::CLOSED,
        ),
        &left,
        &right,
    );
    result.validate().unwrap();
    result.validate_against_sources(&left, &right).unwrap();
    assert!(result.mesh.facts().mesh.closed_manifold);
}

#[test]
fn boundary_touching_orthogonal_shortcuts_report_materialized_evidence() {
    let left = axis_aligned_box_i64([0, 0, 0], [1, 1, 1]);
    let right = axis_aligned_box_i64([1, 0, 0], [2, 1, 1]);

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let request =
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY);
        let preflight = test_preflight(request, &left, &right);
        assert_eq!(
            preflight.support(),
            ExactBooleanSupport::CertifiedArrangementCellComplex,
            "{operation:?}: {preflight:?}"
        );
        assert!(
            preflight.blocker().is_none(),
            "{operation:?}: {preflight:?}"
        );

        let evidence = test_winding_evidence(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
            &left,
            &right,
        );
        assert_eq!(
            evidence.status(),
            ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized,
            "{operation:?}: {evidence:?}"
        );
        assert_eq!(
            evidence.blocker().kind(),
            ExactBooleanBlockerKind::Winding,
            "{operation:?}: {evidence:?}"
        );
        assert!(evidence.status().is_already_materialized());
        evidence.validate().unwrap();
        evidence.validate_against_sources(&left, &right).unwrap();
    }
}

#[test]
fn nonorthogonal_closed_boundary_touching_shortcuts_report_provenance() {
    let left_a = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let left_b = tetrahedron_i64([10, 0, 0], [12, 0, 0], [10, 2, 0], [10, 0, 2]);
    let left = concatenate_meshes_with_options(
        &left_a,
        &left_b,
        false,
        "exact disjoint union",
        ExactMeshValidationPolicy::CLOSED,
    )
    .expect("disconnected nonconvex boundary fixture should validate");
    let right = tetrahedron_i64([0, 0, 0], [-4, 0, 0], [0, -4, 0], [0, 0, -4]);
    let separated_right = tetrahedron_i64([100, 0, 0], [104, 0, 0], [100, 4, 0], [100, 0, 4]);
    let overlapping_right = tetrahedron_i64([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);
    let graph = build_unvalidated_intersection_graph(&left, &right).unwrap();
    validate_graph_source_replay(&graph, &left, &right).unwrap();
    assert!(!graph.has_unknowns());
    assert!(!graph.face_pairs.is_empty());
    assert!(
        boundary_touching_report_from_graph(&graph, &left, &right)
            .unwrap()
            .is_certified()
    );
    assert!(!test_boundary_touching_report(&left, &overlapping_right).is_certified());
    assert!(
        graph_requires_boundary_only_contact(&graph, &left, &right).unwrap(),
        "certified boundary contact should remain retained blocker evidence"
    );
    assert!(
        !graph_requires_boundary_only_contact(&graph, &left, &overlapping_right).unwrap(),
        "overlapping volume should not be classified as boundary-only policy evidence"
    );

    for (operation, support) in [
        (
            ExactBooleanOperation::Union,
            ExactBooleanSupport::CertifiedArrangementCellComplex,
        ),
        (
            ExactBooleanOperation::Intersection,
            ExactBooleanSupport::CertifiedArrangementCellComplex,
        ),
        (
            ExactBooleanOperation::Difference,
            ExactBooleanSupport::CertifiedArrangementCellComplex,
        ),
    ] {
        let request =
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY);
        let preflight = test_preflight(request, &left, &right);
        assert_eq!(preflight.support(), support, "{operation:?}: {preflight:?}");
        assert!(
            preflight.blocker().is_none(),
            "{operation:?}: {preflight:?}"
        );
        preflight.validate().unwrap();
        preflight
            .validate_against_sources_for_request(&left, &right, request)
            .unwrap();

        let evidence = test_winding_evidence(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
            &left,
            &right,
        );
        assert_eq!(
            evidence.status(),
            ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized,
            "{operation:?}: {evidence:?}"
        );
        assert_eq!(
            evidence.blocker().kind(),
            ExactBooleanBlockerKind::Winding,
            "{operation:?}: {evidence:?}"
        );
        assert!(evidence.status().is_already_materialized());
        evidence.validate().unwrap();
        evidence.validate_against_sources(&left, &right).unwrap();

        let result = test_materialized_result(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::CLOSED),
            &left,
            &right,
        );
        assert!(
            result.is_arrangement_cell_complex_materialized_for(operation)
                || result.is_arrangement_cell_complex_shortcut_for(operation),
            "{operation:?}: {result:?}"
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        assert!(
            result
                .validate_against_sources(&left, &separated_right)
                .is_err(),
            "{operation:?}: {result:?}"
        );
        let shortcut = match operation {
            ExactBooleanOperation::Union => ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion,
            ExactBooleanOperation::Intersection => {
                ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection
            }
            ExactBooleanOperation::Difference => {
                ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference
            }
            ExactBooleanOperation::SelectedRegions(_) => unreachable!("named operations only"),
        };
        let mut relabeled_boundary_shortcut = result.clone();
        relabeled_boundary_shortcut.topology_assembly_report = None;
        relabeled_boundary_shortcut.region_ownership_report = None;
        relabeled_boundary_shortcut.kind = ExactBooleanResultKind::CertifiedShortcut {
            operation,
            shortcut,
        };
        relabeled_boundary_shortcut.validate().unwrap();
        assert_eq!(
            relabeled_boundary_shortcut.validate_against_sources(&left, &right),
            Err(ExactEvidenceValidationError::SourceReplayMismatch),
            "{operation:?}: arrangement result relabeled as closed-boundary shortcut must not replay"
        );
        let topology = result
            .topology_assembly_report
            .as_ref()
            .expect("arrangement shortcut should retain topology provenance");
        assert_eq!(
            topology.status,
            ExactTopologyAssemblyStatus::Complete,
            "{operation:?}: {topology:?}"
        );
        let ownership = result
            .region_ownership_report
            .as_ref()
            .expect("arrangement shortcut should retain ownership provenance");
        assert_eq!(
            ownership.status,
            ExactRegionOwnershipStatus::VolumeResolved,
            "{operation:?}: {ownership:?}"
        );
    }
}

#[test]
fn boundary_attached_contained_tetrahedron_difference_materializes() {
    let left = tetrahedron_i64([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
    let right = tetrahedron_i64([2, 2, 2], [4, 1, 1], [1, 4, 1], [1, 1, 1]);

    let preflight = test_preflight(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Difference,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &left,
        &right,
    );
    assert_eq!(
        preflight.support(),
        ExactBooleanSupport::CertifiedArrangementCellComplex
    );
    assert!(preflight.blocker().is_none(), "{preflight:?}");
    assert!(preflight.retained_face_pairs() > 0, "{preflight:?}");
    assert!(preflight.retained_events() > 0, "{preflight:?}");
    let relabeled_preflight = preflight
        .clone()
        .with_support(ExactBooleanSupport::CertifiedConvexDifference);
    assert!(relabeled_preflight.validate().is_err());
    let evidence = test_winding_evidence(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Difference,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &left,
        &right,
    );
    assert_eq!(
        evidence.status(),
        ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized,
        "{evidence:?}"
    );
    evidence.validate().unwrap();
    evidence.validate_against_sources(&left, &right).unwrap();

    let difference = test_materialized_result(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Difference,
            ExactMeshValidationPolicy::CLOSED,
        ),
        &left,
        &right,
    );
    assert!(difference.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Difference));
    difference.validate().unwrap();
    difference.validate_against_sources(&left, &right).unwrap();
    let mut relabeled = difference.clone();
    relabeled.kind = ExactBooleanResultKind::CertifiedShortcut {
        operation: ExactBooleanOperation::Union,
        shortcut: ExactBooleanShortcutKind::ConvexDifference,
    };
    assert_eq!(
        relabeled.validate(),
        Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
    );
    difference
        .validate_request_against_sources_with_retained_attempt(
            &left,
            &right,
            ExactBooleanRequest::new(
                ExactBooleanOperation::Difference,
                ExactMeshValidationPolicy::CLOSED,
            ),
            None,
        )
        .unwrap();
    assert!(difference.mesh.triangles().len() >= left.triangles().len());
}

#[test]
fn noncoplanar_convex_report_cases_retain_graph_counts() {
    let left = tetrahedron_i64([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
    let right = tetrahedron_i64([1, 1, 1], [5, 1, 2], [1, 5, 1], [2, 1, 5]);
    let graph = build_unvalidated_intersection_graph(&left, &right).unwrap();
    validate_graph_source_replay(&graph, &left, &right).unwrap();
    assert!(!graph.has_unknowns());
    assert_eq!(graph.face_pairs.len(), 3);
    assert_eq!(graph.event_count(), 12);

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let request =
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY);
        let preflight = test_preflight(request, &left, &right);
        assert_eq!(
            preflight.support(),
            ExactBooleanSupport::CertifiedArrangementCellComplex,
            "{operation:?}: {preflight:?}"
        );
        assert_eq!(preflight.retained_face_pairs(), graph.face_pairs.len());
        assert_eq!(preflight.retained_events(), graph.event_count());
        assert!(
            preflight.blocker().is_none(),
            "{operation:?}: {preflight:?}"
        );
        preflight.validate().unwrap();
        preflight
            .validate_against_sources_for_request(&left, &right, request)
            .unwrap();

        let evidence = test_winding_evidence(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
            &left,
            &right,
        );
        let expected_evidence_status = match operation {
            ExactBooleanOperation::Union | ExactBooleanOperation::Difference => {
                ExactWindingEvidenceStatus::ConvexBooleanAlreadyMaterialized
            }
            ExactBooleanOperation::Intersection => {
                ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized
            }
            ExactBooleanOperation::SelectedRegions(_) => unreachable!(),
        };
        assert!(
            evidence.status() == expected_evidence_status
                || evidence.status()
                    == ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized,
            "{operation:?}: {evidence:?}"
        );
        assert_eq!(evidence.retained_face_pairs(), graph.face_pairs.len());
        assert_eq!(evidence.retained_events(), graph.event_count());
        assert_eq!(evidence.blocker().kind(), ExactBooleanBlockerKind::Winding);
        assert_eq!(evidence.blocker().candidate_pairs(), graph.face_pairs.len());
        evidence.validate().unwrap();
        evidence.validate_against_sources(&left, &right).unwrap();
    }
}

#[test]
fn straddling_coplanar_crossing_tetrahedron_boundary_attempt_materializes() {
    let left = tetrahedron_i64([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
    let right = tetrahedron_i64([2, 2, 2], [8, -1, -1], [-1, 8, -1], [3, 2, 0]);

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let (expected_support, expected_status, expected_shortcut) = match operation {
            ExactBooleanOperation::Union => (
                ExactBooleanSupport::CertifiedArrangementCellComplex,
                ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized,
                ExactBooleanShortcutKind::ArrangementCellComplex,
            ),
            ExactBooleanOperation::Intersection => (
                ExactBooleanSupport::CertifiedArrangementCellComplex,
                ExactWindingEvidenceStatus::ConvexBooleanAlreadyMaterialized,
                ExactBooleanShortcutKind::ConvexIntersection,
            ),
            ExactBooleanOperation::Difference => (
                ExactBooleanSupport::CertifiedArrangementCellComplex,
                ExactWindingEvidenceStatus::ConvexBooleanAlreadyMaterialized,
                ExactBooleanShortcutKind::ConvexDifference,
            ),
            ExactBooleanOperation::SelectedRegions(_) => unreachable!(),
        };
        let preflight = test_preflight(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
            &left,
            &right,
        );
        assert_eq!(
            preflight.support(),
            expected_support,
            "{operation:?}: {preflight:?}"
        );
        assert!(
            preflight.blocker().is_none(),
            "{operation:?}: {preflight:?}"
        );
        assert!(
            preflight.retained_face_pairs() > 0,
            "{operation:?}: {preflight:?}"
        );
        assert!(
            preflight.retained_events() > 0,
            "{operation:?}: {preflight:?}"
        );

        let evidence = test_winding_evidence(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
            &left,
            &right,
        );
        assert!(
            evidence.status() == expected_status
                || evidence.status()
                    == ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized,
            "{operation:?}: {evidence:?}"
        );
        evidence.validate().unwrap();
        evidence.validate_against_sources(&left, &right).unwrap();

        let result = test_materialized_result(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::CLOSED),
            &left,
            &right,
        );
        assert!(
            result.is_certified_shortcut_kind_for(operation, expected_shortcut)
                || result.is_arrangement_cell_complex_shortcut_for(operation),
            "{operation:?}: {result:?}"
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        result
            .validate_request_against_sources_with_retained_attempt(
                &left,
                &right,
                ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::CLOSED),
                None,
            )
            .unwrap();
        assert!(
            result.mesh.facts().mesh.closed_manifold,
            "{operation:?}: {:?}",
            result.mesh.facts().mesh
        );

        let attempt = test_arrangement_attempt(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
            &left,
            &right,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        );
        assert!(
            attempt.materialized_without_shortcut(),
            "{operation:?}: {attempt:?}"
        );
        assert_eq!(attempt.decline, None, "{operation:?}: {attempt:?}");
        assert!(attempt.output_triangles > 0, "{operation:?}: {attempt:?}");
        assert_current_arrangement_attempt(&attempt, &left, &right);
        if expected_support == ExactBooleanSupport::CertifiedArrangementCellComplex {
            let request =
                ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY);
            let retained_result = materialize_retained_arrangement_cell_complex_attempt(
                &left, &right, request, &attempt,
            )
            .unwrap()
            .expect("retained simplified cells should rematerialize");
            assert!(
                retained_result.is_arrangement_cell_complex_shortcut_for(operation),
                "{operation:?}: {retained_result:?}"
            );
            assert_result_retains_attempt_gate_reports(&retained_result, &attempt);
            retained_result
                .validate_against_sources(&left, &right)
                .unwrap();

            let dispatched_result = try_materialize_certified_boolean_support_with_artifacts(
                &left,
                &right,
                request,
                expected_support,
                None,
                None,
                Some(&attempt),
                &ExactArrangementCellComplexShortcutFacts::from_sources(&left, &right),
            )
            .unwrap()
            .expect("certified support should materialize");
            assert!(dispatched_result.matches_request(request));
            assert!(
                dispatched_result
                    .mesh
                    .validation_policy()
                    .satisfies(request.validation)
            );
            dispatched_result.validate().unwrap();
        }
    }
}

#[test]
fn exact_coplanar_boundary_closer_handles_multiple_planar_loops() {
    let mesh = two_open_boxes_missing_top_i64([0, 0, 0], [4, 0, 0]);
    assert_eq!(mesh.facts().mesh.boundary_edges, 8);
    assert!(!mesh.facts().mesh.closed_manifold);

    let closed = close_exact_coplanar_boundary_loops(
        &mesh,
        "test exact multi-loop coplanar boundary closure",
        ExactMeshValidationPolicy::CLOSED,
    )
    .expect("exact boundary closure should not hit a typed blocker")
    .expect("two planar cap loops should close exactly");

    assert!(closed.facts().mesh.closed_manifold);
    assert_eq!(closed.vertices().len(), mesh.vertices().len());
    assert_eq!(closed.triangles().len(), mesh.triangles().len() + 4);
}

#[test]
fn exact_coplanar_boundary_closer_can_append_cap_vertices() {
    let mesh = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, //
            4, 0, 0, //
            0, 4, 0,
        ],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("test triangle should construct");
    let mut vertices = mesh.vertices().to_vec();
    let mapped = map_cap_vertices_to_boundary_or_insert(
        &mesh,
        &[vec![0, 1, 2]],
        &mut vertices,
        vec![
            Point3::new(Real::from(4), Real::from(0), Real::from(0)),
            Point3::new(
                (Real::from(4) / &Real::from(3)).unwrap(),
                (Real::from(4) / &Real::from(3)).unwrap(),
                Real::from(0),
            ),
        ],
    )
    .expect("exact cap vertices should map or append");
    assert_eq!(mapped, vec![1, 3]);
    assert_eq!(
        vertices[3],
        Point3::new(
            (Real::from(4) / &Real::from(3)).unwrap(),
            (Real::from(4) / &Real::from(3)).unwrap(),
            Real::from(0),
        )
    );
    assert_eq!(vertices.len(), 4);
}

#[test]
fn exact_coplanar_boundary_canonicalizes_only_degenerate_self_contact_spurs() {
    let a = Point3::new(Real::from(0), Real::from(0), Real::from(0));
    let b = Point3::new(Real::from(1), Real::from(0), Real::from(0));
    let c = Point3::new(Real::from(1), Real::from(1), Real::from(0));
    let d = Point3::new(Real::from(0), Real::from(1), Real::from(0));
    let e = Point3::new(Real::from(-1), Real::from(0), Real::from(0));

    let degenerate_spur = canonicalize_degenerate_cyclic_self_contact(
        vec![a.clone(), b.clone(), a.clone(), c.clone(), d.clone()],
        &point3s_exact_equal,
    )
    .expect("exact degenerate spur canonicalization should decide");
    assert_eq!(degenerate_spur.len(), 3);
    assert_eq!(point3_exact_equal(&degenerate_spur[0], &a), Some(true));
    assert_eq!(point3_exact_equal(&degenerate_spur[1], &c), Some(true));
    assert_eq!(point3_exact_equal(&degenerate_spur[2], &d), Some(true));
    assert_eq!(
        boundary_loop_self_contact_evidence(&degenerate_spur)
            .unwrap()
            .repeated_exact_point_pairs,
        0
    );
    assert!(exact_loop_is_coplanar(&degenerate_spur).unwrap());

    let nondegenerate_self_contact = canonicalize_degenerate_cyclic_self_contact(
        vec![a.clone(), b, c, a, d, e],
        &point3s_exact_equal,
    )
    .expect("exact nondegenerate self-contact classification should decide");
    assert_eq!(nondegenerate_self_contact.len(), 6);
    assert_eq!(
        boundary_loop_self_contact_evidence(&nondegenerate_self_contact)
            .unwrap()
            .nondegenerate_cycles,
        2
    );

    let split = split_cyclic_self_contact_cycles(nondegenerate_self_contact, &point3s_exact_equal)
        .expect("exact self-contact cycle splitting should decide");
    assert_eq!(split.len(), 2);
    assert!(split.iter().all(|cycle| cycle.len() == 3));
    assert!(split.iter().all(|cycle| {
        boundary_loop_self_contact_evidence(cycle)
            .unwrap()
            .repeated_exact_point_pairs
            == 0
    }));
}

#[test]
fn exact_coplanar_boundary_closer_preserves_hole_loop_groups() {
    let mesh = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0, //
            1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0, //
            0, 0, 1, 4, 0, 1, 4, 4, 1, 0, 4, 1, //
            1, 1, 1, 3, 1, 1, 3, 3, 1, 1, 3, 1,
        ],
        &[
            0, 1, 9, 0, 9, 8, //
            1, 2, 10, 1, 10, 9, //
            2, 3, 11, 2, 11, 10, //
            3, 0, 8, 3, 8, 11, //
            4, 12, 13, 4, 13, 5, //
            5, 13, 14, 5, 14, 6, //
            6, 14, 15, 6, 15, 7, //
            7, 15, 12, 7, 12, 4,
        ],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(mesh.facts().mesh.boundary_edges, 16);

    let closed = close_exact_coplanar_boundary_loops(
        &mesh,
        "test exact annular cap closure",
        ExactMeshValidationPolicy::CLOSED,
    )
    .expect("exact annular cap closure should not hit a typed blocker")
    .expect("annular cap loop groups should close exactly");

    assert!(
        closed.facts().mesh.closed_manifold,
        "{:?}",
        closed.facts().mesh
    );
    assert_eq!(closed.vertices().len(), mesh.vertices().len());
    assert!(closed.triangles().len() > mesh.triangles().len());
    assert!(
        closed.vertices().iter().all(|point| point3_exact_equal(
            point,
            &Point3::new(Real::from(2), Real::from(2), Real::from(0))
        ) == Some(false)),
        "annular caps should not introduce a center vertex that fills the hole"
    );
}

#[test]
fn exact_coplanar_boundary_closer_orients_cap_groups_independently() {
    let mesh = two_open_boxes_missing_opposite_caps_i64([0, 0, 0], [4, 0, 0]);
    assert_eq!(mesh.facts().mesh.boundary_edges, 8);
    assert!(!mesh.facts().mesh.closed_manifold);

    let closed = close_exact_coplanar_boundary_loops(
        &mesh,
        "test exact independently oriented coplanar boundary closure",
        ExactMeshValidationPolicy::CLOSED,
    )
    .expect("exact oriented cap closure should not hit a typed blocker")
    .expect("opposite cap groups should close with independently certified orientations");

    assert!(
        closed.facts().mesh.closed_manifold,
        "{:?}",
        closed.facts().mesh
    );
    assert_eq!(closed.vertices().len(), mesh.vertices().len());
    assert_eq!(closed.triangles().len(), mesh.triangles().len() + 4);
}

#[test]
fn closed_identical_solids_route_through_arrangement_pipeline() {
    let left = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let right = left.clone();

    let preflight = test_preflight(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &left,
        &right,
    );
    assert_eq!(
        preflight.support(),
        ExactBooleanSupport::CertifiedArrangementCellComplex
    );

    let attempt = test_arrangement_attempt(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    assert_eq!(attempt.decline, None);
    assert!(attempt.materialized_without_shortcut());
    assert_current_arrangement_attempt(&attempt, &left, &right);

    let union = test_materialized_result(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::CLOSED,
        ),
        &left,
        &right,
    );
    assert!(union.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Union));
    assert!(exact_meshes_have_same_shape(&union.mesh, &left));

    let difference = test_materialized_result(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Difference,
            ExactMeshValidationPolicy::CLOSED,
        ),
        &left,
        &right,
    );
    assert!(difference.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Difference));
    assert!(difference.mesh.triangles().is_empty());
}

#[test]
fn closed_same_surface_solids_route_through_arrangement_pipeline() {
    let left = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let right = ExactMesh::from_i64_triangles(
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
    assert!(!meshes_are_certified_identical(&left, &right));
    assert!(meshes_are_certified_same_surface(&left, &right));

    let attempt = test_arrangement_attempt(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    assert_eq!(attempt.decline, None);
    assert!(attempt.materialized_without_shortcut());
    assert_current_arrangement_attempt(&attempt, &left, &right);

    let preflight = test_preflight(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &left,
        &right,
    );
    assert_eq!(
        preflight.support(),
        ExactBooleanSupport::CertifiedArrangementCellComplex
    );

    let union = test_materialized_result(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::CLOSED,
        ),
        &left,
        &right,
    );
    assert!(union.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Union));
    assert!(exact_meshes_have_same_shape(&union.mesh, &left));

    let difference = test_materialized_result(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Difference,
            ExactMeshValidationPolicy::CLOSED,
        ),
        &left,
        &right,
    );
    assert!(difference.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Difference));
    assert!(difference.mesh.triangles().is_empty());
}

#[test]
fn closed_same_surface_reversed_orientation_routes_through_arrangement_pipeline() {
    let left = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let right = ExactMesh::from_i64_triangles(
        &[
            4, 0, 0, //
            0, 0, 0, //
            0, 4, 0, //
            0, 0, 4,
        ],
        &[
            1, 0, 2, //
            1, 3, 0, //
            0, 3, 2, //
            2, 3, 1,
        ],
    )
    .unwrap();
    assert!(meshes_are_certified_same_surface(&left, &right));

    let union_attempt = test_arrangement_attempt(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    assert_eq!(union_attempt.decline, None);
    assert_eq!(union_attempt.selected_faces, 4);
    assert_eq!(
        union_attempt.volume_oriented_selected_faces + union_attempt.label_oriented_selected_faces,
        union_attempt.selected_faces
    );
    assert!(union_attempt.reversed_selected_faces <= union_attempt.selected_faces);
    assert_eq!(union_attempt.output_triangles, 4);
    assert_current_arrangement_attempt(&union_attempt, &left, &right);
    let mut stale_selected_faces = union_attempt.clone();
    stale_selected_faces.selected_faces = stale_selected_faces.face_cells + 1;
    assert_eq!(
        stale_selected_faces.validate(),
        Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
    );
    let mut stale_selected_volumes = union_attempt.clone();
    stale_selected_volumes.selected_volume_regions = stale_selected_volumes.volume_regions + 1;
    assert_eq!(
        stale_selected_volumes.validate(),
        Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
    );
    let mut stale_orientation_split = union_attempt.clone();
    stale_orientation_split.label_oriented_selected_faces += 1;
    assert_eq!(
        stale_orientation_split.validate(),
        Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
    );
    let mut stale_reversed_faces = union_attempt.clone();
    stale_reversed_faces.reversed_selected_faces = stale_reversed_faces.selected_faces + 1;
    assert_eq!(
        stale_reversed_faces.validate(),
        Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
    );
    let mut stale_volume_regions = union_attempt.clone();
    stale_volume_regions.regions = 0;
    assert_eq!(
        stale_volume_regions.validate(),
        Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
    );
    let mut stale_volume_adjacencies = union_attempt.clone();
    stale_volume_adjacencies.volume_regions = 1;
    stale_volume_adjacencies.volume_adjacencies = 1;
    assert_eq!(
        stale_volume_adjacencies.validate(),
        Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
    );
    let mut stale_union_counts = union_attempt.clone();
    stale_union_counts.output_vertices = 0;
    stale_union_counts.output_triangles = 0;
    assert_eq!(
        stale_union_counts.validate(),
        Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
    );
    let mut impossible_output_counts = union_attempt.clone();
    impossible_output_counts.output_vertices = 0;
    assert_eq!(
        impossible_output_counts.validate(),
        Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
    );

    let difference_attempt = test_arrangement_attempt(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Difference,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    assert_eq!(difference_attempt.decline, None);
    assert_eq!(difference_attempt.selected_faces, 0);
    assert_eq!(difference_attempt.output_triangles, 0);
    assert_current_arrangement_attempt(&difference_attempt, &left, &right);

    let union = test_materialized_result(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::CLOSED,
        ),
        &left,
        &right,
    );
    assert!(union.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Union));
    assert!(exact_meshes_have_same_shape(&union.mesh, &left));

    let difference = test_materialized_result(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Difference,
            ExactMeshValidationPolicy::CLOSED,
        ),
        &left,
        &right,
    );
    assert!(difference.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Difference));
    assert!(difference.mesh.triangles().is_empty());
}

#[test]
fn open_same_surface_sheets_remain_certified() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[4, 0, 0, 0, 4, 0, 0, 0, 0],
        &[2, 0, 1],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(meshes_are_certified_same_surface(&left, &right));

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let preflight = test_preflight(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
            &left,
            &right,
        );
        assert_eq!(
            preflight.support(),
            ExactBooleanSupport::CertifiedSameSurface,
            "{operation:?}: {preflight:?}"
        );
        let evidence = test_winding_evidence(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
            &left,
            &right,
        );
        assert_eq!(
            evidence.status(),
            ExactWindingEvidenceStatus::SurfaceEqualityAlreadyMaterialized,
            "{operation:?}: {evidence:?}"
        );
        assert_eq!(evidence.retained_face_pairs(), 0);
        assert_eq!(evidence.retained_events(), 0);
        evidence.validate().unwrap();
        evidence.validate_against_sources(&left, &right).unwrap();

        let result = test_materialized_result(
            ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
            &left,
            &right,
        );
        assert!(
            matches!(
                result.kind,
                ExactBooleanResultKind::CertifiedShortcut {
                    operation: result_operation,
                    ..
                } if result_operation == operation
            ),
            "{operation:?}: {result:?}"
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        result
            .validate_request_against_sources_with_retained_attempt(
                &left,
                &right,
                ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::ALLOW_BOUNDARY),
                None,
            )
            .unwrap();
        if matches!(operation, ExactBooleanOperation::Difference) {
            assert!(result.mesh.triangles().is_empty(), "{result:?}");
        } else {
            assert!(exact_meshes_have_same_shape(&result.mesh, &left));
        }
    }
}

#[test]
fn open_identical_sheets_keep_identity_shortcut() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = left.clone();

    let preflight = test_preflight(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &left,
        &right,
    );
    assert_eq!(preflight.support(), ExactBooleanSupport::CertifiedIdentical);
    let evidence = test_winding_evidence(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &left,
        &right,
    );
    assert_eq!(
        evidence.status(),
        ExactWindingEvidenceStatus::SurfaceEqualityAlreadyMaterialized,
        "{evidence:?}"
    );
    evidence.validate().unwrap();
    evidence.validate_against_sources(&left, &right).unwrap();

    let union = test_materialized_result(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        ),
        &left,
        &right,
    );
    assert!(matches!(
        union.kind,
        ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Union,
            ..
        }
    ));
    union
        .validate_request_against_sources_with_retained_attempt(
            &left,
            &right,
            ExactBooleanRequest::new(
                ExactBooleanOperation::Union,
                ExactMeshValidationPolicy::ALLOW_BOUNDARY,
            ),
            None,
        )
        .unwrap();
}

#[test]
fn graph_backed_early_shortcuts_retain_graph_counts() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = left.clone();
    let graph = build_unvalidated_intersection_graph(&left, &right).unwrap();
    validate_graph_source_replay(&graph, &left, &right).unwrap();
    assert!(!graph.face_pairs.is_empty());
    assert!(graph.event_count() > 0);

    for (validation, expected_support) in [
        (
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
            ExactBooleanSupport::CertifiedIdentical,
        ),
        (
            ExactMeshValidationPolicy::CLOSED,
            ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid,
        ),
    ] {
        let preflight = test_preflight(
            ExactBooleanRequest::new(ExactBooleanOperation::Union, validation),
            &left,
            &right,
        );
        assert_eq!(preflight.support(), expected_support, "{preflight:?}");
        assert_eq!(preflight.retained_face_pairs(), graph.face_pairs.len());
        assert_eq!(preflight.retained_events(), graph.event_count());
        assert!(preflight.blocker().is_none(), "{preflight:?}");
        preflight.validate().unwrap();
        preflight
            .validate_against_sources_for_request(
                &left,
                &right,
                ExactBooleanRequest::new(ExactBooleanOperation::Union, validation),
            )
            .unwrap();
    }
}

#[test]
fn arrangement_materialized_evidence_retains_boundary_only_evidence() {
    let left_a = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let left_b = tetrahedron_i64([10, 0, 0], [12, 0, 0], [10, 2, 0], [10, 0, 2]);
    let mut vertices = left_a.vertices().to_vec();
    let offset = vertices.len();
    vertices.extend_from_slice(left_b.vertices());
    let mut triangles = left_a.triangles().to_vec();
    triangles.extend(
        left_b
            .triangles()
            .iter()
            .map(|triangle| Triangle(triangle.0.map(|index| index + offset))),
    );
    let left = ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("evidence disconnected positive-area boundary fixture"),
        ExactMeshValidationPolicy::CLOSED,
    )
    .unwrap();
    let right = tetrahedron_i64([2, 0, 0], [6, 0, 0], [2, 4, 0], [2, 0, -4]);
    let graph = build_unvalidated_intersection_graph(&left, &right).unwrap();
    validate_graph_source_replay(&graph, &left, &right).unwrap();

    let evidence = arrangement_cell_complex_already_materialized_winding_evidence(
        &graph,
        &left,
        &right,
        ExactBooleanOperation::Union,
    );

    assert_eq!(
        evidence.status(),
        ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized
    );
    assert_eq!(
        evidence.blocker().kind(),
        ExactBooleanBlockerKind::BoundaryOnlyContact
    );
    let volumetric_evidence = evidence
        .coplanar_volumetric_evidence()
        .expect("arrangement evidence should retain boundary-only evidence");
    assert!(volumetric_evidence.is_boundary_only_positive_area_contact());
    assert_eq!(
        volumetric_evidence.retained_face_pair_count(),
        evidence.retained_face_pairs()
    );
    volumetric_evidence.validate().unwrap();
}

#[test]
fn coplanar_overlay_regularizes_nonconvex_boundary_touch_intersection_to_empty() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 10, 0, 0, 10, 4, 0, 7, 4, 0, 6, 6, 0, 10, 8, 0, 10, 12, 0, 0, 12, 0,
        ],
        &[
            0, 1, 2, //
            0, 2, 3, //
            0, 3, 4, //
            0, 4, 7, //
            7, 4, 5, //
            7, 5, 6,
        ],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[4, 12, 0, 6, 12, 0, 6, 14, 0, 4, 14, 0],
        &[0, 1, 2, 0, 2, 3],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let result = boolean_coplanar_mesh_overlay_optional(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
    .expect("regularized boundary-touch intersection should materialize through overlay");
    let graph = build_unvalidated_intersection_graph(&left, &right).unwrap();
    validate_graph_source_replay(&graph, &left, &right).unwrap();
    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Intersection,
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    );
    let preflight = test_preflight(request, &left, &right);
    assert_eq!(
        preflight.support(),
        ExactBooleanSupport::CertifiedArrangementCellComplex,
        "{preflight:?}"
    );
    assert!(preflight.blocker().is_none(), "{preflight:?}");
    assert_eq!(preflight.retained_face_pairs(), graph.face_pairs.len());
    assert_eq!(preflight.retained_events(), graph.event_count());
    preflight
        .validate_against_sources_for_request(&left, &right, request)
        .unwrap();
    assert!(result.is_arrangement_cell_complex_shortcut_for(ExactBooleanOperation::Intersection));
    assert!(result.mesh.triangles().is_empty());
}

fn tetrahedron_i64(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2],
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap()
}

fn combine_test_meshes(left: &ExactMesh, right: &ExactMesh, label: &'static str) -> ExactMesh {
    let right_offset = left.vertices().len();
    ExactMesh::new(
        left.vertices()
            .iter()
            .chain(right.vertices())
            .cloned()
            .collect(),
        left.triangles()
            .iter()
            .map(|triangle| triangle.0)
            .chain(right.triangles().iter().map(|triangle| {
                let [a, b, c] = triangle.0;
                [a + right_offset, b + right_offset, c + right_offset]
            }))
            .collect(),
        hyperlimit::SourceProvenance::exact(label),
    )
    .unwrap()
}

fn axis_aligned_box_i64(min: [i64; 3], max: [i64; 3]) -> ExactMesh {
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

fn square_pyramid_with_base_i64() -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0, 5, 5, 10],
        &[0, 3, 2, 0, 2, 1, 0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4],
    )
    .unwrap()
}

fn downward_square_pyramid_with_base_i64(min: [i64; 2], max: [i64; 2], z: i64) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            min[0], min[1], 0, max[0], min[1], 0, max[0], max[1], 0, min[0], max[1], 0, min[0],
            min[1], z,
        ],
        &[0, 1, 2, 0, 2, 3, 0, 4, 1, 1, 4, 2, 2, 4, 3, 3, 4, 0],
    )
    .unwrap()
}

fn axis_aligned_orthogonal_l_solid_i64() -> ExactMesh {
    let horizontal = axis_aligned_box_i64([0, 0, 0], [2, 1, 1]);
    let vertical = axis_aligned_box_i64([0, 1, 0], [1, 2, 1]);
    let plan = axis_aligned_orthogonal_solid_cell_plan(
        &horizontal,
        &vertical,
        AxisAlignedOrthogonalSolidOperation::Union,
    )
    .expect("L solid should have an orthogonal cell plan");
    plan.to_mesh(
        "test axis-aligned orthogonal L solid",
        ExactMeshValidationPolicy::CLOSED,
    )
    .unwrap()
}

fn affine_box_i64(min: [i64; 3], max: [i64; 3]) -> ExactMesh {
    let p = |u: i64, v: i64, w: i64| [2 * u + v, 2 * v, 2 * w];
    affine_box_from_map_i64(min, max, p)
}

fn skew_affine_box_i64(min: [i64; 3], max: [i64; 3]) -> ExactMesh {
    let p = |u: i64, v: i64, w: i64| [u + 10 * v, v, w];
    affine_box_from_map_i64(min, max, p)
}

fn affine_box_from_map_i64(
    min: [i64; 3],
    max: [i64; 3],
    p: impl Fn(i64, i64, i64) -> [i64; 3],
) -> ExactMesh {
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

fn two_open_boxes_missing_top_i64(first_min: [i64; 3], second_min: [i64; 3]) -> ExactMesh {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for min in [first_min, second_min] {
        let max = [min[0] + 2, min[1] + 2, min[2] + 2];
        let start = vertices.len() / 3;
        vertices.extend([
            min[0], min[1], min[2], max[0], min[1], min[2], max[0], max[1], min[2], min[0], max[1],
            min[2], min[0], min[1], max[2], max[0], min[1], max[2], max[0], max[1], max[2], min[0],
            max[1], max[2],
        ]);
        triangles.extend([
            start,
            start + 2,
            start + 1,
            start,
            start + 3,
            start + 2,
            start,
            start + 1,
            start + 5,
            start,
            start + 5,
            start + 4,
            start + 1,
            start + 2,
            start + 6,
            start + 1,
            start + 6,
            start + 5,
            start + 2,
            start + 3,
            start + 7,
            start + 2,
            start + 7,
            start + 6,
            start + 3,
            start,
            start + 4,
            start + 3,
            start + 4,
            start + 7,
        ]);
    }
    ExactMesh::from_i64_triangles_with_policy(
        &vertices,
        &triangles,
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
}

fn two_open_boxes_missing_opposite_caps_i64(
    missing_top_min: [i64; 3],
    missing_bottom_min: [i64; 3],
) -> ExactMesh {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for (min, missing_top) in [(missing_top_min, true), (missing_bottom_min, false)] {
        let max = [min[0] + 2, min[1] + 2, min[2] + 2];
        let start = vertices.len() / 3;
        vertices.extend([
            min[0], min[1], min[2], max[0], min[1], min[2], max[0], max[1], min[2], min[0], max[1],
            min[2], min[0], min[1], max[2], max[0], min[1], max[2], max[0], max[1], max[2], min[0],
            max[1], max[2],
        ]);
        if !missing_top {
            triangles.extend([
                start + 4,
                start + 5,
                start + 6,
                start + 4,
                start + 6,
                start + 7,
            ]);
        }
        if missing_top {
            triangles.extend([start, start + 2, start + 1, start, start + 3, start + 2]);
        }
        triangles.extend([
            start,
            start + 1,
            start + 5,
            start,
            start + 5,
            start + 4,
            start + 1,
            start + 2,
            start + 6,
            start + 1,
            start + 6,
            start + 5,
            start + 2,
            start + 3,
            start + 7,
            start + 2,
            start + 7,
            start + 6,
            start + 3,
            start,
            start + 4,
            start + 3,
            start + 4,
            start + 7,
        ]);
    }
    ExactMesh::from_i64_triangles_with_policy(
        &vertices,
        &triangles,
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
}
