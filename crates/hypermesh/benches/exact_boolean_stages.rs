use std::hint::black_box;
use std::time::{Duration, Instant};

use hypermesh::{
    CoplanarVolumetricCellEvidenceReport, ExactAdjacentUnionCompletionReport, ExactArrangement,
    ExactArrangementBooleanAttempt, ExactBooleanEvaluation, ExactBooleanOperation,
    ExactBooleanPreflight, ExactBooleanRequest, ExactBooleanResult, ExactBooleanWorkspace,
    ExactBoundaryTouchingReport, ExactIdenticalMeshReport, ExactMesh,
    ExactOpenSurfaceDisjointReport, ExactPlanarArrangementReport, ExactRefinementReport,
    ExactRegularizationPolicy, ExactSameSurfaceReport, ExactSelectedCellComplex,
    ExactSimplifiedCellComplex, ExactVolumetricBoundaryClosureReport, ExactWindingReadinessReport,
    ValidationPolicy, build_intersection_graph, triangulate_all_face_cells_with_cdt,
};

struct BenchCase {
    name: &'static str,
    left: ExactMesh,
    right: ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
    regularization: ExactRegularizationPolicy,
    iterations: usize,
}

fn main() {
    let cases = [
        BenchCase {
            name: "open_crossing_sheets",
            left: open_triangle_xy(),
            right: open_triangle_yz(),
            operation: ExactBooleanOperation::Union,
            validation: ValidationPolicy::ALLOW_BOUNDARY,
            regularization: ExactRegularizationPolicy::RETAIN_ARTIFACTS,
            iterations: 64,
        },
        BenchCase {
            name: "closed_overlapping_tetrahedra",
            left: tetra([0, 0, 0]),
            right: tetra([1, 0, 0]),
            operation: ExactBooleanOperation::Union,
            validation: ValidationPolicy::CLOSED,
            regularization: ExactRegularizationPolicy::REGULARIZED_SOLID,
            iterations: 8,
        },
    ];

    println!("hypermesh exact boolean stage timings");
    println!("case,stage,iterations,total_ns,avg_ns");
    for case in &cases {
        run_case(case);
    }
}

fn run_case(case: &BenchCase) {
    time_stage(case, "mesh_retained_state", || {
        black_box(case.left.validate_retained_state().unwrap());
        black_box(case.right.validate_retained_state().unwrap());
    });

    time_stage(case, "broad_phase_candidates", || {
        black_box(
            case.left
                .bounds()
                .candidate_face_pairs(case.right.bounds())
                .len(),
        );
    });

    time_stage(case, "intersection_graph_build", || {
        let graph = build_intersection_graph(&case.left, &case.right).unwrap();
        black_box((graph.face_pairs.len(), graph.event_count()));
    });

    let graph = build_intersection_graph(&case.left, &case.right).unwrap();
    time_stage(case, "intersection_graph_validate", || {
        black_box(
            graph
                .validate_against_meshes(&case.left, &case.right)
                .unwrap(),
        );
    });

    time_stage(case, "split_topology_plan", || {
        black_box(graph.checked_split_topology_plan().ok());
    });

    time_stage(case, "face_split_geometry", || {
        black_box(graph.face_split_geometry_plan(&case.left, &case.right).ok());
    });

    time_stage(case, "face_cell_cdt", || {
        black_box(triangulate_all_face_cells_with_cdt(&graph, &case.left, &case.right).ok());
    });

    time_stage(case, "arrangement_build", || {
        let arrangement =
            ExactArrangement::from_meshes_with_policy(&case.left, &case.right, case.regularization)
                .unwrap();
        black_box((
            arrangement.vertices.len(),
            arrangement.edges.len(),
            arrangement.face_cells.len(),
            arrangement.blockers.len(),
        ));
    });

    let arrangement =
        ExactArrangement::from_meshes_with_policy(&case.left, &case.right, case.regularization)
            .unwrap();
    time_stage(case, "topology_assembly_report", || {
        let report = arrangement.topology_assembly_report_with_policy(
            &case.left,
            &case.right,
            case.regularization,
        );
        black_box((
            report.status,
            report.graph_events,
            report.split_graph_vertices,
            report.region_boundaries,
            report.arrangement_face_cells,
        ));
    });

    time_stage(case, "region_ownership_report", || {
        let report = arrangement
            .region_ownership_report_with_policy(&case.left, &case.right, case.regularization)
            .unwrap();
        black_box((
            report.status,
            report.face_cells,
            report.opposite_unknown_faces,
            report.volume_regions,
            report.shared_owned_volumes,
        ));
    });

    time_stage(case, "cell_label_select", || {
        let selected = arrangement
            .label_regions(case.regularization)
            .and_then(|labeled| labeled.select_with_policy(case.operation, case.regularization));
        black_box(selected.ok());
    });

    let selected = arrangement
        .label_regions(case.regularization)
        .and_then(|labeled| labeled.select_with_policy(case.operation, case.regularization))
        .ok();
    if let Some(selected) = selected {
        time_stage(case, "cell_simplify", || {
            black_box(
                selected
                    .clone()
                    .simplify_exact_with_policy(case.regularization)
                    .ok(),
            );
        });

        let simplified = selected
            .clone()
            .simplify_exact_with_policy(case.regularization)
            .ok();
        if let Some(simplified) = simplified {
            time_stage(case, "cell_triangulate", || {
                black_box(simplified.triangulate().ok());
            });
        }
    }

    let request = ExactBooleanRequest::new(case.operation, case.validation);
    time_stage(case, "arrangement_attempt", || {
        black_box(
            request
                .arrangement_attempt(&case.left, &case.right, case.regularization)
                .unwrap(),
        );
    });

    time_stage(case, "boolean_preflight", || {
        black_box(request.preflight(&case.left, &case.right).unwrap());
    });

    time_stage(case, "boolean_evaluate", || {
        black_box(request.evaluate(&case.left, &case.right).unwrap());
    });

    time_stage(case, "boolean_materialize_or_block", || {
        black_box(request.materialize(&case.left, &case.right).ok());
    });

    let mut workspace = ExactBooleanWorkspace::new(&case.left, &case.right);
    workspace.graph().unwrap();
    time_stage(case, "workspace_graph_cached", || {
        let graph = workspace.graph().unwrap();
        black_box((graph.face_pairs.len(), graph.event_count()));
    });

    workspace.coplanar_volumetric_cell_evidence().unwrap();
    time_stage(
        case,
        "workspace_coplanar_volumetric_evidence_cached",
        || {
            let report = workspace.coplanar_volumetric_cell_evidence().unwrap();
            black_box((
                report.obstacle,
                report.retained_face_pair_count,
                report.candidate_pairs,
                report.coplanar_overlapping_pairs,
                report.positive_area_coplanar_overlapping_pairs,
                report.same_side_coplanar_overlapping_pairs,
            ));
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_coplanar_volumetric_evidence_from_retained_graph",
        || retained_workspace_and_coplanar_volumetric_evidence_for_case(case, request),
        |(retained_workspace, report)| {
            black_box(
                retained_workspace
                    .validate_coplanar_volumetric_cell_evidence(report)
                    .ok(),
            );
        },
    );

    workspace.arrangement(case.regularization).unwrap();
    time_stage(case, "workspace_arrangement_cached", || {
        let arrangement = workspace.arrangement(case.regularization).unwrap();
        black_box((
            arrangement.vertices.len(),
            arrangement.edges.len(),
            arrangement.face_cells.len(),
            arrangement.blockers.len(),
        ));
    });

    workspace
        .topology_assembly_report(case.regularization)
        .unwrap();
    time_stage(case, "workspace_topology_assembly_report_cached", || {
        let report = workspace
            .topology_assembly_report(case.regularization)
            .unwrap();
        black_box((
            report.status,
            report.graph_events,
            report.region_boundaries,
            report.arrangement_face_cells,
            report.arrangement_face_cell_boundary_nodes,
            report.arrangement_face_cell_boundary_points,
            report.arrangement_regions,
            report.arrangement_region_edge_incidences,
            report.arrangement_region_boundary_edges,
            report.arrangement_region_non_manifold_edges,
            report.lower_dimensional_point_contacts,
            report.lower_dimensional_edge_contacts,
            report.volume_adjacency_face_sides,
            report.volume_adjacency_separating_faces,
        ));
    });

    time_prepared_stage(
        case,
        "workspace_validate_topology_assembly_report_cached",
        || retained_workspace_and_topology_for_case(case),
        |(retained_workspace, report)| {
            black_box(
                retained_workspace
                    .validate_topology_assembly_report(case.regularization, report)
                    .ok(),
            );
        },
    );

    workspace
        .region_ownership_report(case.regularization)
        .unwrap();
    time_stage(case, "workspace_region_ownership_report_cached", || {
        let report = workspace
            .region_ownership_report(case.regularization)
            .unwrap();
        black_box((
            report.status,
            report.face_cells,
            report.face_cell_boundary_nodes,
            report.face_cell_boundary_points,
            report.opposite_unknown_faces,
            report.volume_regions,
            report.lower_dimensional_point_contacts,
            report.lower_dimensional_edge_contacts,
            report.volume_adjacency_face_sides,
            report.volume_adjacency_separating_faces,
        ));
    });

    time_prepared_stage(
        case,
        "workspace_validate_region_ownership_report_cached",
        || retained_workspace_and_region_ownership_for_case(case),
        |(retained_workspace, report)| {
            black_box(
                retained_workspace
                    .validate_region_ownership_report(case.regularization, report)
                    .ok(),
            );
        },
    );

    workspace
        .arrangement_attempt(request, case.regularization)
        .unwrap();
    time_stage(case, "workspace_arrangement_attempt_cached", || {
        let attempt = workspace
            .arrangement_attempt(request, case.regularization)
            .unwrap();
        black_box((
            attempt.stage,
            attempt.selected_faces,
            attempt.reversed_selected_faces,
            attempt.volume_oriented_selected_faces,
            attempt.label_oriented_selected_faces,
            attempt.selected_volume_regions,
        ));
    });

    time_prepared_stage(
        case,
        "workspace_validate_arrangement_attempt_cached",
        || retained_workspace_and_arrangement_attempt_for_case(case, request),
        |(retained_workspace, attempt)| {
            black_box(
                retained_workspace
                    .validate_arrangement_attempt(request, case.regularization, attempt)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_selected_from_retained_artifacts",
        || retained_workspace_for_case(case, request),
        |retained_workspace| {
            black_box(
                retained_workspace
                    .selected_cell_complex(request, case.regularization)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_simplified_from_retained_artifacts",
        || retained_workspace_for_case(case, request),
        |retained_workspace| {
            black_box(
                retained_workspace
                    .simplified_cell_complex(request, case.regularization)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "selected_validate_retained_reports",
        || retained_selected_for_case(case, request),
        |selected| {
            if let Some(selected) = selected.as_ref() {
                black_box(selected.validate().ok());
            }
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_selected_from_retained_artifacts",
        || retained_workspace_and_selected_for_case(case, request),
        |(retained_workspace, selected)| {
            if let Some(selected) = selected.as_ref() {
                black_box(
                    retained_workspace
                        .validate_selected_cell_complex(request, case.regularization, selected)
                        .ok(),
                );
            }
        },
    );

    time_prepared_stage(
        case,
        "selected_replay_validate_retained_reports",
        || retained_selected_for_case(case, request),
        |selected| {
            if let Some(selected) = selected.as_ref() {
                black_box(
                    selected
                        .validate_against_sources(&case.left, &case.right, case.regularization)
                        .ok(),
                );
            }
        },
    );

    time_prepared_stage(
        case,
        "simplified_validate_retained_reports",
        || retained_simplified_for_case(case, request),
        |simplified| {
            if let Some(simplified) = simplified.as_ref() {
                black_box((
                    simplified.validate().ok(),
                    simplified.selected_faces_before_simplification,
                    simplified.selected_boundary_nodes_before_simplification,
                    simplified.oriented_selected_faces_before_simplification,
                    simplified.reversed_selected_faces_before_simplification,
                    simplified.volume_oriented_selected_faces_before_simplification,
                    simplified.label_oriented_selected_faces_before_simplification,
                ));
            }
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_simplified_from_retained_artifacts",
        || retained_workspace_and_simplified_for_case(case, request),
        |(retained_workspace, simplified)| {
            if let Some(simplified) = simplified.as_ref() {
                black_box(
                    retained_workspace
                        .validate_simplified_cell_complex(request, case.regularization, simplified)
                        .ok(),
                );
            }
        },
    );

    time_prepared_stage(
        case,
        "simplified_replay_validate_retained_reports",
        || retained_simplified_for_case(case, request),
        |simplified| {
            if let Some(simplified) = simplified.as_ref() {
                black_box(
                    simplified
                        .validate_against_sources(&case.left, &case.right, case.regularization)
                        .ok(),
                );
            }
        },
    );

    workspace.preflight(request).unwrap();
    time_stage(case, "workspace_preflight_cached", || {
        black_box(workspace.preflight(request).unwrap());
    });

    time_prepared_stage(
        case,
        "workspace_validate_preflight_from_retained_artifacts",
        || retained_workspace_and_preflight_for_case(case, request),
        |(retained_workspace, preflight)| {
            black_box(
                retained_workspace
                    .validate_preflight(request, preflight)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_refinement_from_retained_artifacts",
        || retained_workspace_and_refinement_for_case(case, request),
        |(retained_workspace, report)| {
            black_box(
                retained_workspace
                    .validate_refinement_report(request, report)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_adjacent_union_from_retained_artifacts",
        || retained_workspace_and_adjacent_union_for_case(case, request),
        |(retained_workspace, report)| {
            black_box(
                retained_workspace
                    .validate_adjacent_union_completion_report(request, report)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_identical_mesh_from_retained_artifacts",
        || retained_workspace_and_identical_mesh_for_case(case, request),
        |(retained_workspace, report)| {
            black_box(
                retained_workspace
                    .validate_identical_mesh_report(request, report)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_same_surface_from_retained_artifacts",
        || retained_workspace_and_same_surface_for_case(case, request),
        |(retained_workspace, report)| {
            black_box(
                retained_workspace
                    .validate_same_surface_report(request, report)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_boundary_touching_from_retained_artifacts",
        || retained_workspace_and_boundary_touching_for_case(case, request),
        |(retained_workspace, report)| {
            black_box(
                retained_workspace
                    .validate_boundary_touching_report(request, report)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_open_surface_disjoint_from_retained_artifacts",
        || retained_workspace_and_open_surface_disjoint_for_case(case, request),
        |(retained_workspace, report)| {
            black_box(
                retained_workspace
                    .validate_open_surface_disjoint_report(request, report)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_volumetric_boundary_closure_from_retained_artifacts",
        || retained_workspace_and_volumetric_boundary_closure_for_case(case, request),
        |(retained_workspace, report)| {
            black_box(
                retained_workspace
                    .validate_volumetric_boundary_closure(request, report)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_winding_readiness_from_retained_artifacts",
        || retained_workspace_and_winding_readiness_for_case(case, request),
        |(retained_workspace, readiness)| {
            black_box(
                retained_workspace
                    .validate_winding_readiness(request, readiness)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_planar_arrangement_from_retained_artifacts",
        || retained_workspace_and_planar_arrangement_for_case(case, request),
        |(retained_workspace, report)| {
            black_box(
                retained_workspace
                    .validate_planar_arrangement_report(request, report)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_evaluation_from_retained_artifacts",
        || retained_workspace_for_case(case, request),
        |retained_workspace| {
            black_box(retained_workspace.evaluate(request).unwrap());
        },
    );

    time_prepared_stage(
        case,
        "workspace_materialize_from_retained_artifacts",
        || retained_workspace_for_case(case, request),
        |retained_workspace| {
            black_box(retained_workspace.materialize(request).ok());
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_evaluation_from_retained_artifacts",
        || retained_workspace_and_evaluation_for_case(case, request),
        |(retained_workspace, evaluation)| {
            black_box(retained_workspace.validate_evaluation(evaluation).ok());
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_result_from_retained_artifacts",
        || retained_workspace_and_result_for_case(case, request),
        |(retained_workspace, result)| {
            if let Some(result) = result.as_ref() {
                black_box(retained_workspace.validate_result(request, result).ok());
            }
        },
    );

    let mut materialize_cache_workspace = retained_workspace_for_case(case, request);
    materialize_cache_workspace.materialize(request).ok();
    time_stage(
        case,
        "workspace_materialize_cached_without_evaluation",
        || {
            black_box(materialize_cache_workspace.materialize(request).ok());
        },
    );

    workspace.evaluate(request).unwrap();
    time_stage(case, "workspace_evaluation_cached", || {
        black_box(workspace.evaluate(request).unwrap());
    });

    time_stage(case, "workspace_materialize_cached", || {
        black_box(workspace.materialize(request).ok());
    });
}

fn retained_workspace_for_case<'a>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
) -> ExactBooleanWorkspace<'a> {
    let mut retained_workspace = ExactBooleanWorkspace::new(&case.left, &case.right);
    retained_workspace.graph().unwrap();
    retained_workspace
        .arrangement(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap();
    retained_workspace.preflight(request).unwrap();
    retained_workspace
}

fn retained_workspace_and_preflight_for_case<'a>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
) -> (ExactBooleanWorkspace<'a>, ExactBooleanPreflight) {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    let preflight = retained_workspace.preflight(request).unwrap().clone();
    (retained_workspace, preflight)
}

fn retained_workspace_and_coplanar_volumetric_evidence_for_case<'a>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
) -> (
    ExactBooleanWorkspace<'a>,
    CoplanarVolumetricCellEvidenceReport,
) {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    let report = retained_workspace
        .coplanar_volumetric_cell_evidence()
        .unwrap()
        .clone();
    (retained_workspace, report)
}

fn retained_workspace_and_refinement_for_case<'a>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
) -> (ExactBooleanWorkspace<'a>, ExactRefinementReport) {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    let report = retained_workspace
        .refinement_report(request)
        .unwrap()
        .clone();
    (retained_workspace, report)
}

fn retained_workspace_and_adjacent_union_for_case<'a>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
) -> (
    ExactBooleanWorkspace<'a>,
    ExactAdjacentUnionCompletionReport,
) {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    let report = retained_workspace
        .adjacent_union_completion_report(request)
        .unwrap()
        .clone();
    (retained_workspace, report)
}

fn retained_workspace_and_identical_mesh_for_case<'a>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
) -> (ExactBooleanWorkspace<'a>, ExactIdenticalMeshReport) {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    let report = retained_workspace.identical_mesh_report(request).clone();
    (retained_workspace, report)
}

fn retained_workspace_and_same_surface_for_case<'a>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
) -> (ExactBooleanWorkspace<'a>, ExactSameSurfaceReport) {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    let report = retained_workspace.same_surface_report(request).clone();
    (retained_workspace, report)
}

fn retained_workspace_and_boundary_touching_for_case<'a>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
) -> (ExactBooleanWorkspace<'a>, ExactBoundaryTouchingReport) {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    let report = retained_workspace
        .boundary_touching_report(request)
        .unwrap()
        .clone();
    (retained_workspace, report)
}

fn retained_workspace_and_open_surface_disjoint_for_case<'a>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
) -> (ExactBooleanWorkspace<'a>, ExactOpenSurfaceDisjointReport) {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    let report = retained_workspace
        .open_surface_disjoint_report(request)
        .unwrap()
        .clone();
    (retained_workspace, report)
}

fn retained_workspace_and_volumetric_boundary_closure_for_case<'a>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
) -> (
    ExactBooleanWorkspace<'a>,
    ExactVolumetricBoundaryClosureReport,
) {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    let report = retained_workspace
        .volumetric_boundary_closure(request)
        .unwrap()
        .clone();
    (retained_workspace, report)
}

fn retained_workspace_and_winding_readiness_for_case<'a>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
) -> (ExactBooleanWorkspace<'a>, ExactWindingReadinessReport) {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    let readiness = retained_workspace
        .winding_readiness(request)
        .unwrap()
        .clone();
    (retained_workspace, readiness)
}

fn retained_workspace_and_planar_arrangement_for_case<'a>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
) -> (ExactBooleanWorkspace<'a>, ExactPlanarArrangementReport) {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    let report = retained_workspace
        .planar_arrangement_report(request)
        .unwrap()
        .clone();
    (retained_workspace, report)
}

fn retained_workspace_and_evaluation_for_case<'a>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
) -> (ExactBooleanWorkspace<'a>, ExactBooleanEvaluation) {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    let evaluation = retained_workspace.evaluate(request).unwrap().clone();
    (retained_workspace, evaluation)
}

fn retained_workspace_and_result_for_case<'a>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
) -> (ExactBooleanWorkspace<'a>, Option<ExactBooleanResult>) {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    let result = retained_workspace.materialize(request).ok();
    (retained_workspace, result)
}

fn retained_workspace_and_arrangement_attempt_for_case<'a>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
) -> (ExactBooleanWorkspace<'a>, ExactArrangementBooleanAttempt) {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    let attempt = retained_workspace
        .arrangement_attempt(request, case.regularization)
        .unwrap()
        .clone();
    (retained_workspace, attempt)
}

fn retained_workspace_and_region_ownership_for_case<'a>(
    case: &'a BenchCase,
) -> (
    ExactBooleanWorkspace<'a>,
    hypermesh::ExactRegionOwnershipReport,
) {
    let mut retained_workspace = retained_workspace_for_case(
        case,
        ExactBooleanRequest::new(case.operation, case.validation),
    );
    let report = retained_workspace
        .region_ownership_report(case.regularization)
        .unwrap()
        .clone();
    (retained_workspace, report)
}

fn retained_workspace_and_topology_for_case<'a>(
    case: &'a BenchCase,
) -> (
    ExactBooleanWorkspace<'a>,
    hypermesh::ExactTopologyAssemblyReport,
) {
    let mut retained_workspace = retained_workspace_for_case(
        case,
        ExactBooleanRequest::new(case.operation, case.validation),
    );
    let report = retained_workspace
        .topology_assembly_report(case.regularization)
        .unwrap()
        .clone();
    (retained_workspace, report)
}

fn retained_selected_for_case(
    case: &BenchCase,
    request: ExactBooleanRequest,
) -> Option<ExactSelectedCellComplex> {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    retained_workspace
        .selected_cell_complex(request, case.regularization)
        .ok()
        .cloned()
}

fn retained_workspace_and_selected_for_case<'a>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
) -> (ExactBooleanWorkspace<'a>, Option<ExactSelectedCellComplex>) {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    let selected = retained_workspace
        .selected_cell_complex(request, case.regularization)
        .ok()
        .cloned();
    (retained_workspace, selected)
}

fn retained_simplified_for_case(
    case: &BenchCase,
    request: ExactBooleanRequest,
) -> Option<ExactSimplifiedCellComplex> {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    retained_workspace
        .simplified_cell_complex(request, case.regularization)
        .ok()
        .cloned()
}

fn retained_workspace_and_simplified_for_case<'a>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
) -> (
    ExactBooleanWorkspace<'a>,
    Option<ExactSimplifiedCellComplex>,
) {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    let simplified = retained_workspace
        .simplified_cell_complex(request, case.regularization)
        .ok()
        .cloned();
    (retained_workspace, simplified)
}

fn time_stage<F>(case: &BenchCase, stage: &'static str, mut f: F)
where
    F: FnMut(),
{
    f();
    let start = Instant::now();
    for _ in 0..case.iterations {
        f();
    }
    let elapsed = start.elapsed();
    print_timing(case.name, stage, case.iterations, elapsed);
}

fn time_prepared_stage<T, S, F>(case: &BenchCase, stage: &'static str, mut setup: S, mut f: F)
where
    S: FnMut() -> T,
    F: FnMut(&mut T),
{
    let mut warmup = setup();
    f(&mut warmup);
    let mut prepared = Vec::with_capacity(case.iterations);
    for _ in 0..case.iterations {
        prepared.push(setup());
    }
    let start = Instant::now();
    for item in &mut prepared {
        f(item);
    }
    let elapsed = start.elapsed();
    print_timing(case.name, stage, case.iterations, elapsed);
}

fn print_timing(case: &str, stage: &str, iterations: usize, elapsed: Duration) {
    let total_ns = elapsed.as_nanos();
    let avg_ns = total_ns / iterations.max(1) as u128;
    println!("{case},{stage},{iterations},{total_ns},{avg_ns}");
}

fn open_triangle_xy() -> ExactMesh {
    ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
}

fn open_triangle_yz() -> ExactMesh {
    ExactMesh::from_i64_triangles_with_policy(
        &[1, -1, -1, 1, 3, 1, 1, 3, -1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
}

fn tetra(offset: [i64; 3]) -> ExactMesh {
    let [ox, oy, oz] = offset;
    ExactMesh::from_i64_triangles(
        &[ox, oy, oz, ox + 1, oy, oz, ox, oy + 1, oz, ox, oy, oz + 1],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap()
}
