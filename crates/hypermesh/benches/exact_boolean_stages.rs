use std::hint::black_box;
use std::time::{Duration, Instant};

use hypermesh::{
    ExactArrangement, ExactArrangementBooleanAttempt, ExactBooleanEvaluation,
    ExactBooleanOperation, ExactBooleanRequest, ExactBooleanResult, ExactBooleanWorkspace,
    ExactMesh, ExactRegularizationPolicy, ValidationPolicy,
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
            left: tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]),
            right: tetra_from_corners([1, 1, 1], [5, 1, 1], [1, 5, 1], [1, 1, 5]),
            operation: ExactBooleanOperation::Union,
            validation: ValidationPolicy::CLOSED,
            regularization: ExactRegularizationPolicy::REGULARIZED_SOLID,
            iterations: 8,
        },
        BenchCase {
            name: "closed_arrangement_cell_complex",
            left: nonconvex_closed_arrangement_left(),
            right: tetra_from_corners([1, 1, 1], [5, 1, 1], [1, 5, 1], [1, 1, 5]),
            operation: ExactBooleanOperation::Union,
            validation: ValidationPolicy::ALLOW_BOUNDARY,
            regularization: ExactRegularizationPolicy::REGULARIZED_SOLID,
            iterations: 1,
        },
        BenchCase {
            name: "open_coplanar_disjoint_sheets",
            left: open_triangle_xy(),
            right: open_triangle_xy_far_corner(),
            operation: ExactBooleanOperation::Union,
            validation: ValidationPolicy::ALLOW_BOUNDARY,
            regularization: ExactRegularizationPolicy::RETAIN_ARTIFACTS,
            iterations: 64,
        },
        BenchCase {
            name: "open_boundary_touching_sheets",
            left: open_boundary_touching_left(),
            right: open_boundary_touching_right(),
            operation: ExactBooleanOperation::Union,
            validation: ValidationPolicy::ALLOW_BOUNDARY,
            regularization: ExactRegularizationPolicy::RETAIN_ARTIFACTS,
            iterations: 64,
        },
    ];

    println!("hypermesh exact boolean stage timings");
    println!("case,stage,detail,iterations,total_ns,avg_ns");
    for case in &cases {
        run_case(case);
    }
}

fn run_case(case: &BenchCase) {
    let request = ExactBooleanRequest::new(case.operation, case.validation);
    print_metadata(
        case.name,
        "request",
        format!("{:?}/{:?}", case.operation, case.validation),
    );
    let mut metadata_workspace = ExactBooleanWorkspace::new(&case.left, &case.right);
    match metadata_workspace.evaluate(request) {
        Ok(evaluation) => print_metadata(
            case.name,
            "preflight_support",
            format!(
                "{:?};pairs={};events={}",
                evaluation.preflight().support,
                evaluation.preflight().retained_face_pairs,
                evaluation.preflight().retained_events
            ),
        ),
        Err(error) => print_metadata(case.name, "preflight_support", format!("error:{error:?}")),
    }
    match metadata_workspace.materialize(request) {
        Ok(result) => print_metadata(
            case.name,
            "materialize_kind",
            format!(
                "{:?};triangles={}",
                result.kind,
                result.mesh.triangles().len()
            ),
        ),
        Err(error) => print_metadata(case.name, "materialize_kind", format!("error:{error:?}")),
    }

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
        let mut workspace = ExactBooleanWorkspace::new(&case.left, &case.right);
        let graph = workspace.graph().unwrap();
        black_box((graph.face_pairs.len(), graph.event_count()));
    });

    let mut graph_workspace = ExactBooleanWorkspace::new(&case.left, &case.right);
    let graph = graph_workspace.graph().unwrap().clone();
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
        black_box(
            graph
                .triangulate_face_cells_with_cdt(&case.left, &case.right)
                .ok(),
        );
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

    time_stage(case, "arrangement_build_from_retained_graph", || {
        let arrangement = ExactArrangement::from_intersection_graph_with_policy(
            graph.clone(),
            &case.left,
            &case.right,
            case.regularization,
        )
        .unwrap();
        black_box((
            arrangement.vertices.len(),
            arrangement.edges.len(),
            arrangement.face_cells.len(),
            arrangement.blockers.len(),
        ));
    });

    let arrangement = ExactArrangement::from_intersection_graph_with_policy(
        graph.clone(),
        &case.left,
        &case.right,
        case.regularization,
    )
    .unwrap();
    let attempt = retained_arrangement_attempt_for_case(case, request);
    time_stage(case, "attempt_topology_assembly_report", || {
        if let Some(report) = attempt.topology_assembly_report.as_ref() {
            black_box((
                report.status,
                report.graph_events,
                report.split_graph_vertices,
                report.region_boundaries,
                report.arrangement_face_cells,
            ));
        }
    });

    time_stage(case, "attempt_region_ownership_report", || {
        if let Some(report) = attempt.region_ownership_report.as_ref() {
            black_box((
                report.status,
                report.face_cells,
                report.opposite_unknown_faces,
                report.volume_regions,
                report.shared_owned_volumes,
            ));
        }
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

    time_stage(case, "boolean_evaluate", || {
        let mut workspace = ExactBooleanWorkspace::new(&case.left, &case.right);
        black_box(workspace.evaluate(request).ok());
    });

    time_stage(case, "boolean_materialize_or_block", || {
        let mut workspace = ExactBooleanWorkspace::new(&case.left, &case.right);
        black_box(workspace.materialize(request).ok());
    });

    let mut workspace = ExactBooleanWorkspace::new(&case.left, &case.right);
    workspace.graph().unwrap();
    time_stage(case, "workspace_graph_cached", || {
        let graph = workspace.graph().unwrap();
        black_box((graph.face_pairs.len(), graph.event_count()));
    });

    workspace.evaluate(request).ok();
    time_stage(
        case,
        "workspace_coplanar_volumetric_evidence_from_evaluation",
        || {
            let evaluation = workspace.evaluate(request).unwrap();
            let report = evaluation
                .preflight()
                .coplanar_volumetric_evidence
                .as_ref()
                .or(evaluation
                    .winding_readiness_report()
                    .coplanar_volumetric_evidence
                    .as_ref());
            black_box(report.map(|report| {
                (
                    report.obstacle,
                    report.retained_face_pair_count,
                    report.candidate_pairs,
                    report.coplanar_overlapping_pairs,
                    report.positive_area_coplanar_overlapping_pairs,
                    report.same_side_coplanar_overlapping_pairs,
                )
            }));
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_coplanar_volumetric_evidence_from_evaluation",
        || retained_workspace_and_evaluation_for_case(case, request),
        |(retained_workspace, evaluation)| {
            if let Some(evaluation) = evaluation.as_ref() {
                let report = evaluation
                    .preflight()
                    .coplanar_volumetric_evidence
                    .as_ref()
                    .or(evaluation
                        .winding_readiness_report()
                        .coplanar_volumetric_evidence
                        .as_ref());
                if let Some(report) = report {
                    black_box(
                        report
                            .validate_against_sources(
                                retained_workspace.left(),
                                retained_workspace.right(),
                            )
                            .ok(),
                    );
                }
            }
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
        .arrangement_attempt(request, case.regularization)
        .unwrap();
    time_stage(
        case,
        "workspace_topology_assembly_report_from_attempt",
        || {
            let report = workspace
                .arrangement_attempt(request, case.regularization)
                .unwrap()
                .topology_assembly_report
                .as_ref();
            if let Some(report) = report {
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
            }
        },
    );

    time_prepared_stage(
        case,
        "topology_assembly_report_replay_validate_from_attempt",
        || retained_arrangement_attempt_for_case(case, request),
        |attempt| {
            if let Some(report) = attempt.topology_assembly_report.as_ref() {
                black_box(
                    report
                        .validate_against_sources(&case.left, &case.right, case.regularization)
                        .ok(),
                );
            }
        },
    );

    time_stage(
        case,
        "workspace_region_ownership_report_from_attempt",
        || {
            let report = workspace
                .arrangement_attempt(request, case.regularization)
                .unwrap()
                .region_ownership_report
                .as_ref();
            if let Some(report) = report {
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
            }
        },
    );

    time_prepared_stage(
        case,
        "region_ownership_report_replay_validate_from_attempt",
        || retained_arrangement_attempt_for_case(case, request),
        |attempt| {
            if let Some(report) = attempt.region_ownership_report.as_ref() {
                black_box(
                    report
                        .validate_against_sources(&case.left, &case.right, case.regularization)
                        .ok(),
                );
            }
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
        "attempt_validate_source_replay",
        || retained_workspace_and_arrangement_attempt_for_case(case, request),
        |(retained_workspace, attempt)| {
            black_box(
                attempt
                    .validate_against_sources(retained_workspace.left(), retained_workspace.right())
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "attempt_selected_from_retained_artifacts",
        || retained_arrangement_attempt_for_case(case, request),
        |attempt| {
            black_box(attempt.selected_cell_complex.as_ref().map(|selected| {
                (
                    selected.selected_faces.len(),
                    selected.selected_face_orientations.len(),
                    selected.selected_volume_regions.len(),
                )
            }));
        },
    );

    time_prepared_stage(
        case,
        "attempt_simplified_from_retained_artifacts",
        || retained_arrangement_attempt_for_case(case, request),
        |attempt| {
            black_box(attempt.simplified_cell_complex.as_ref().map(|simplified| {
                (
                    simplified.faces.len(),
                    simplified.selected_faces_before_simplification,
                    simplified.selected_boundary_nodes_before_simplification,
                )
            }));
        },
    );

    time_prepared_stage(
        case,
        "selected_validate_retained_reports",
        || retained_arrangement_attempt_for_case(case, request),
        |attempt| {
            if let Some(selected) = attempt.selected_cell_complex.as_ref() {
                black_box(selected.validate().ok());
            }
        },
    );

    time_prepared_stage(
        case,
        "selected_replay_validate_retained_reports",
        || retained_arrangement_attempt_for_case(case, request),
        |attempt| {
            if let Some(selected) = attempt.selected_cell_complex.as_ref() {
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
        || retained_arrangement_attempt_for_case(case, request),
        |attempt| {
            if let Some(simplified) = attempt.simplified_cell_complex.as_ref() {
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
        "simplified_replay_validate_retained_reports",
        || retained_arrangement_attempt_for_case(case, request),
        |attempt| {
            if let Some(simplified) = attempt.simplified_cell_complex.as_ref() {
                black_box(
                    simplified
                        .validate_against_sources(&case.left, &case.right, case.regularization)
                        .ok(),
                );
            }
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_refinement_from_retained_artifacts",
        || {
            retained_workspace_and_certification_for_case(case, request, |evaluation| {
                evaluation.certifications.refinement.clone()
            })
        },
        |(retained_workspace, report)| {
            black_box(
                report
                    .validate_against_sources(retained_workspace.left(), retained_workspace.right())
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_adjacent_union_from_retained_artifacts",
        || {
            retained_workspace_and_certification_for_case(case, request, |evaluation| {
                evaluation.certifications.adjacent_union_completion.clone()
            })
        },
        |(retained_workspace, report)| {
            black_box(
                report
                    .validate_against_sources(retained_workspace.left(), retained_workspace.right())
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_identical_mesh_from_retained_artifacts",
        || {
            retained_workspace_and_certification_for_case(case, request, |evaluation| {
                evaluation.certifications.identical.clone()
            })
        },
        |(retained_workspace, report)| {
            black_box(
                report
                    .validate_against_sources(retained_workspace.left(), retained_workspace.right())
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_same_surface_from_retained_artifacts",
        || {
            retained_workspace_and_certification_for_case(case, request, |evaluation| {
                evaluation.certifications.same_surface.clone()
            })
        },
        |(retained_workspace, report)| {
            black_box(
                report
                    .validate_against_sources(retained_workspace.left(), retained_workspace.right())
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_boundary_touching_from_retained_artifacts",
        || {
            retained_workspace_and_certification_for_case(case, request, |evaluation| {
                evaluation.boundary_touching_report().clone()
            })
        },
        |(retained_workspace, report)| {
            black_box(
                report
                    .validate_against_sources(retained_workspace.left(), retained_workspace.right())
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_open_surface_disjoint_from_retained_artifacts",
        || {
            retained_workspace_and_certification_for_case(case, request, |evaluation| {
                evaluation.open_surface_disjoint_report().clone()
            })
        },
        |(retained_workspace, report)| {
            black_box(
                report
                    .validate_against_sources(retained_workspace.left(), retained_workspace.right())
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_volumetric_boundary_closure_from_retained_artifacts",
        || {
            retained_workspace_and_certification_for_case(case, request, |evaluation| {
                evaluation.volumetric_boundary_closure_report().cloned()
            })
        },
        |(retained_workspace, report)| {
            if let Some(report) = report.as_ref() {
                black_box(
                    report
                        .validate_against_sources(
                            retained_workspace.left(),
                            retained_workspace.right(),
                        )
                        .ok(),
                );
            }
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_winding_readiness_from_retained_artifacts",
        || {
            retained_workspace_and_certification_for_case(case, request, |evaluation| {
                evaluation.winding_readiness_report().clone()
            })
        },
        |(retained_workspace, readiness)| {
            black_box(
                readiness
                    .validate_against_sources_with_boundary_policy(
                        retained_workspace.left(),
                        retained_workspace.right(),
                        request.validation,
                        request.boundary_policy,
                    )
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_planar_arrangement_from_retained_artifacts",
        || {
            retained_workspace_and_certification_for_case(case, request, |evaluation| {
                evaluation.planar_arrangement_report().clone()
            })
        },
        |(retained_workspace, report)| {
            black_box(
                report
                    .validate_against_sources(retained_workspace.left(), retained_workspace.right())
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_evaluation_from_retained_artifacts",
        || retained_workspace_for_case(case, request),
        |retained_workspace| {
            black_box(retained_workspace.evaluate(request).ok());
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
        "evaluation_validate_source_replay",
        || retained_workspace_and_evaluation_for_case(case, request),
        |(retained_workspace, evaluation)| {
            if let Some(evaluation) = evaluation.as_ref() {
                black_box(
                    evaluation
                        .validate_against_sources(
                            retained_workspace.left(),
                            retained_workspace.right(),
                        )
                        .ok(),
                );
            }
        },
    );

    time_prepared_stage(
        case,
        "result_validate_operation_replay",
        || retained_workspace_and_result_for_case(case, request),
        |(retained_workspace, result)| {
            if let Some(result) = result.as_ref() {
                black_box(
                    result
                        .validate_operation_against_sources(
                            retained_workspace.left(),
                            retained_workspace.right(),
                            request.operation,
                            request.validation,
                            request.boundary_policy,
                        )
                        .ok(),
                );
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

    workspace.evaluate(request).ok();
    time_stage(case, "workspace_evaluation_cached", || {
        black_box(workspace.evaluate(request).ok());
    });

    time_stage(case, "workspace_materialize_cached", || {
        black_box(workspace.materialize(request).ok());
    });
}

fn retained_graph_workspace_for_case<'a>(case: &'a BenchCase) -> ExactBooleanWorkspace<'a> {
    let mut retained_workspace = ExactBooleanWorkspace::new(&case.left, &case.right);
    retained_workspace.graph().unwrap();
    retained_workspace
}

fn retained_workspace_for_case<'a>(
    case: &'a BenchCase,
    _request: ExactBooleanRequest,
) -> ExactBooleanWorkspace<'a> {
    let mut retained_workspace = retained_graph_workspace_for_case(case);
    retained_workspace
        .arrangement(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap();
    retained_workspace
}

fn retained_workspace_and_certification_for_case<'a, T>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
    project: impl FnOnce(&ExactBooleanEvaluation) -> T,
) -> (ExactBooleanWorkspace<'a>, T) {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    let report = project(retained_workspace.evaluate(request).unwrap());
    (retained_workspace, report)
}

fn retained_workspace_and_evaluation_for_case<'a>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
) -> (ExactBooleanWorkspace<'a>, Option<ExactBooleanEvaluation>) {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    let evaluation = retained_workspace.evaluate(request).ok().cloned();
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

fn retained_arrangement_attempt_for_case(
    case: &BenchCase,
    request: ExactBooleanRequest,
) -> ExactArrangementBooleanAttempt {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    retained_workspace
        .arrangement_attempt(request, case.regularization)
        .unwrap()
        .clone()
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
    println!("{case},{stage},,{iterations},{total_ns},{avg_ns}");
}

fn print_metadata(case: &str, stage: &str, detail: String) {
    let detail: String = detail
        .chars()
        .map(|ch| match ch {
            ',' | '\n' | '\r' => ' ',
            _ => ch,
        })
        .collect();
    println!("{case},{stage},{detail},0,0,0");
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

fn open_triangle_xy_far_corner() -> ExactMesh {
    ExactMesh::from_i64_triangles_with_policy(
        &[3, 3, 0, 5, 3, 0, 3, 5, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
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

fn nonconvex_closed_arrangement_left() -> ExactMesh {
    ExactMesh::from_i64_triangles(
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
    .unwrap()
}

fn open_boundary_touching_left() -> ExactMesh {
    ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
}

fn open_boundary_touching_right() -> ExactMesh {
    ExactMesh::from_i64_triangles_with_policy(
        &[2, 0, 0, 0, 2, 0, 2, 2, 2],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
}
