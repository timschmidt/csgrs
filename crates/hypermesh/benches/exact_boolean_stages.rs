use std::hint::black_box;
use std::time::{Duration, Instant};

use hypermesh::{
    ExactArrangementBooleanAttempt, ExactBooleanEvaluation, ExactBooleanOperation,
    ExactBooleanRequest, ExactBooleanWorkspace, ExactMesh, ExactRegularizationPolicy,
    ValidationPolicy,
};

struct BenchCase {
    name: &'static str,
    left: ExactMesh,
    right: ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
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
            iterations: 64,
        },
        BenchCase {
            name: "closed_overlapping_tetrahedra",
            left: tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]),
            right: tetra_from_corners([1, 1, 1], [5, 1, 1], [1, 5, 1], [1, 1, 5]),
            operation: ExactBooleanOperation::Union,
            validation: ValidationPolicy::CLOSED,
            iterations: 8,
        },
        BenchCase {
            name: "closed_arrangement_cell_complex",
            left: nonconvex_closed_arrangement_left(),
            right: tetra_from_corners([1, 1, 1], [5, 1, 1], [1, 5, 1], [1, 1, 5]),
            operation: ExactBooleanOperation::Union,
            validation: ValidationPolicy::ALLOW_BOUNDARY,
            iterations: 1,
        },
        BenchCase {
            name: "open_coplanar_disjoint_sheets",
            left: open_triangle_xy(),
            right: open_triangle_xy_far_corner(),
            operation: ExactBooleanOperation::Union,
            validation: ValidationPolicy::ALLOW_BOUNDARY,
            iterations: 64,
        },
        BenchCase {
            name: "open_boundary_touching_sheets",
            left: open_boundary_touching_left(),
            right: open_boundary_touching_right(),
            operation: ExactBooleanOperation::Union,
            validation: ValidationPolicy::ALLOW_BOUNDARY,
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
            "materialized_result_kind",
            format!(
                "{:?};triangles={}",
                result.kind,
                result.mesh.triangles().len()
            ),
        ),
        Err(error) => print_metadata(
            case.name,
            "materialized_result_kind",
            format!("error:{error:?}"),
        ),
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

    time_stage(case, "arrangement_attempt_from_evaluation", || {
        let mut workspace = ExactBooleanWorkspace::new(&case.left, &case.right);
        let attempt = workspace
            .evaluate(request)
            .unwrap()
            .retained_arrangement_attempt()
            .cloned();
        black_box(attempt.map(|attempt| {
            (
                attempt
                    .topology_assembly_report()
                    .is_some_and(|report| report.is_complete()),
                attempt
                    .region_ownership_report()
                    .is_some_and(|report| report.status.is_volume_resolved()),
            )
        }));
    });

    let attempt = retained_arrangement_attempt_for_case(case, request);
    time_stage(case, "attempt_topology_assembly_report", || {
        if let Some(report) = attempt.topology_assembly_report() {
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
        if let Some(report) = attempt.region_ownership_report() {
            black_box((
                report.status,
                report.face_cells,
                report.opposite_unknown_faces,
                report.volume_regions,
                report.shared_owned_volumes,
            ));
        }
    });

    time_stage(case, "boolean_evaluate", || {
        let mut workspace = ExactBooleanWorkspace::new(&case.left, &case.right);
        black_box(workspace.evaluate(request).ok());
    });

    time_stage(case, "boolean_materialize", || {
        let mut workspace = ExactBooleanWorkspace::new(&case.left, &case.right);
        black_box(workspace.materialize(request).ok());
    });

    let mut workspace = ExactBooleanWorkspace::new(&case.left, &case.right);
    workspace.evaluate(request).ok();
    time_stage(
        case,
        "workspace_coplanar_volumetric_evidence_from_evaluation",
        || {
            let evaluation = workspace.evaluate(request).unwrap();
            let report = evaluation
                .preflight
                .coplanar_volumetric_evidence
                .as_ref()
                .or(evaluation
                    .certifications
                    .winding_readiness
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
        "workspace_validate_evaluation_with_coplanar_volumetric_evidence",
        || retained_workspace_and_evaluation_for_case(case, request),
        |(_retained_workspace, evaluation)| {
            if let Some(evaluation) = evaluation.as_ref() {
                let report = evaluation
                    .preflight
                    .coplanar_volumetric_evidence
                    .as_ref()
                    .or(evaluation
                        .certifications
                        .winding_readiness
                        .coplanar_volumetric_evidence
                        .as_ref());
                if let Some(report) = report {
                    black_box((
                        report.obstacle,
                        evaluation
                            .validate_against_sources(&case.left, &case.right)
                            .ok(),
                    ));
                }
            }
        },
    );

    workspace.evaluate(request).unwrap();
    time_stage(
        case,
        "workspace_topology_assembly_report_from_evaluation_attempt",
        || {
            let report = workspace
                .evaluate(request)
                .unwrap()
                .retained_arrangement_attempt()
                .and_then(|attempt| attempt.topology_assembly_report());
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
            if let Some(report) = attempt.topology_assembly_report() {
                black_box(
                    report
                        .validate_against_sources(
                            &case.left,
                            &case.right,
                            ExactRegularizationPolicy::REGULARIZED_SOLID,
                        )
                        .ok(),
                );
            }
        },
    );

    time_stage(
        case,
        "workspace_region_ownership_report_from_evaluation_attempt",
        || {
            let report = workspace
                .evaluate(request)
                .unwrap()
                .retained_arrangement_attempt()
                .and_then(|attempt| attempt.region_ownership_report());
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
            if let Some(report) = attempt.region_ownership_report() {
                black_box(
                    report
                        .validate_against_sources(
                            &case.left,
                            &case.right,
                            ExactRegularizationPolicy::REGULARIZED_SOLID,
                        )
                        .ok(),
                );
            }
        },
    );

    workspace.evaluate(request).unwrap();
    time_stage(case, "workspace_evaluation_attempt_cached", || {
        let evaluation = workspace.evaluate(request).unwrap();
        let attempt = evaluation.retained_arrangement_attempt();
        black_box(attempt.map(|attempt| {
            (
                attempt
                    .topology_assembly_report()
                    .is_some_and(|report| report.is_complete()),
                attempt
                    .region_ownership_report()
                    .is_some_and(|report| report.status.is_resolved()),
                attempt
                    .region_ownership_report()
                    .is_some_and(|report| report.status.is_volume_resolved()),
            )
        }));
    });

    time_prepared_stage(
        case,
        "attempt_validate_source_replay",
        || retained_workspace_and_arrangement_attempt_for_case(case, request),
        |(_retained_workspace, attempt)| {
            black_box(
                attempt
                    .validate_against_sources(&case.left, &case.right)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_certifications_from_retained_artifacts",
        || {
            retained_workspace_and_certification_for_case(case, request, |evaluation| {
                evaluation.certifications().clone()
            })
        },
        |(_retained_workspace, certifications)| {
            black_box(
                certifications
                    .validate_against_sources(&case.left, &case.right, request)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_certifications_with_adjacent_union_retained",
        || {
            retained_workspace_and_certification_for_case(case, request, |evaluation| {
                evaluation.certifications().clone()
            })
        },
        |(_retained_workspace, certifications)| {
            black_box(
                certifications
                    .validate_against_sources(&case.left, &case.right, request)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_certifications_with_identical_retained",
        || {
            retained_workspace_and_certification_for_case(case, request, |evaluation| {
                evaluation.certifications().clone()
            })
        },
        |(_retained_workspace, certifications)| {
            black_box(
                certifications
                    .validate_against_sources(&case.left, &case.right, request)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_certifications_with_same_surface_retained",
        || {
            retained_workspace_and_certification_for_case(case, request, |evaluation| {
                evaluation.certifications().clone()
            })
        },
        |(_retained_workspace, certifications)| {
            black_box(
                certifications
                    .validate_against_sources(&case.left, &case.right, request)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_certifications_with_boundary_touching_retained",
        || {
            retained_workspace_and_certification_for_case(case, request, |evaluation| {
                evaluation.certifications().clone()
            })
        },
        |(_retained_workspace, certifications)| {
            black_box(
                certifications
                    .validate_against_sources(&case.left, &case.right, request)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_certifications_with_open_surface_disjoint_retained",
        || {
            retained_workspace_and_certification_for_case(case, request, |evaluation| {
                evaluation.certifications().clone()
            })
        },
        |(_retained_workspace, certifications)| {
            black_box(
                certifications
                    .validate_against_sources(&case.left, &case.right, request)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_certifications_with_boundary_closure_retained",
        || {
            retained_workspace_and_certification_for_case(case, request, |evaluation| {
                evaluation.certifications().clone()
            })
        },
        |(_retained_workspace, certifications)| {
            black_box(
                certifications
                    .validate_against_sources(&case.left, &case.right, request)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_certifications_with_winding_readiness_retained",
        || {
            retained_workspace_and_certification_for_case(case, request, |evaluation| {
                evaluation.certifications().clone()
            })
        },
        |(_retained_workspace, certifications)| {
            black_box(
                certifications
                    .validate_against_sources(&case.left, &case.right, request)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_certifications_with_planar_arrangement_retained",
        || {
            retained_workspace_and_certification_for_case(case, request, |evaluation| {
                evaluation.certifications().clone()
            })
        },
        |(_retained_workspace, certifications)| {
            black_box(
                certifications
                    .validate_against_sources(&case.left, &case.right, request)
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
        |(_retained_workspace, evaluation)| {
            if let Some(evaluation) = evaluation.as_ref() {
                black_box(
                    evaluation
                        .validate_against_sources(&case.left, &case.right)
                        .ok(),
                );
            }
        },
    );

    time_prepared_stage(
        case,
        "evaluation_result_validate_retained_replay",
        || retained_workspace_and_evaluation_for_case(case, request),
        |(_retained_workspace, evaluation)| {
            if let Some(evaluation) = evaluation.as_ref() {
                black_box(
                    evaluation
                        .validate_materialized_result_against_sources(&case.left, &case.right)
                        .ok(),
                );
            }
        },
    );

    let mut materialize_cache_workspace = retained_workspace_for_case(case, request);
    materialize_cache_workspace.materialize(request).ok();
    time_stage(case, "workspace_materialization_cached", || {
        black_box(materialize_cache_workspace.materialize_ref(request).ok());
    });

    workspace.evaluate(request).ok();
    time_stage(case, "workspace_evaluation_cached", || {
        black_box(workspace.evaluate(request).ok());
    });

    time_stage(case, "workspace_cached_materialization", || {
        black_box(workspace.materialize(request).ok());
    });
}

fn retained_workspace_for_case<'a>(
    case: &'a BenchCase,
    _request: ExactBooleanRequest,
) -> ExactBooleanWorkspace<'a> {
    ExactBooleanWorkspace::new(&case.left, &case.right)
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

fn retained_workspace_and_arrangement_attempt_for_case<'a>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
) -> (ExactBooleanWorkspace<'a>, ExactArrangementBooleanAttempt) {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    let attempt = retained_workspace
        .evaluate(request)
        .unwrap()
        .retained_arrangement_attempt()
        .expect("evaluation should retain an arrangement attempt")
        .clone();
    (retained_workspace, attempt)
}

fn retained_arrangement_attempt_for_case(
    case: &BenchCase,
    request: ExactBooleanRequest,
) -> ExactArrangementBooleanAttempt {
    let mut retained_workspace = retained_workspace_for_case(case, request);
    retained_workspace
        .evaluate(request)
        .unwrap()
        .retained_arrangement_attempt()
        .expect("evaluation should retain an arrangement attempt")
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
