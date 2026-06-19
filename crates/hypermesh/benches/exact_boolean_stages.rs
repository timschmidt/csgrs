use std::hint::black_box;
use std::time::{Duration, Instant};

use hypermesh::{
    ExactBooleanOperation, ExactBooleanRequest, ExactBooleanWorkspace, ExactMesh, ValidationPolicy,
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
                "materialized={};blocked={}",
                evaluation.materialized_result().is_some(),
                evaluation.has_blocker()
            ),
        ),
        Err(error) => print_metadata(case.name, "preflight_support", format!("error:{error:?}")),
    }
    match metadata_workspace.materialize_ref(request) {
        Ok(result) => print_metadata(
            case.name,
            "materialized_result",
            format!(
                "replay_valid={};triangles={}",
                result
                    .validate_against_sources(&case.left, &case.right)
                    .is_ok(),
                result.mesh().triangles().len()
            ),
        ),
        Err(error) => print_metadata(case.name, "materialized_result", format!("error:{error:?}")),
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
            .retained_arrangement_attempt();
        black_box(attempt.map(|attempt| {
            (
                attempt.topology_assembly_is_complete(),
                attempt.region_ownership_is_volume_resolved(),
            )
        }));
    });

    let mut attempt_workspace = retained_workspace_for_case(case);
    attempt_workspace.evaluate(request).unwrap();
    time_stage(case, "attempt_topology_assembly_completion", || {
        let attempt = attempt_workspace
            .evaluate(request)
            .unwrap()
            .retained_arrangement_attempt()
            .expect("evaluation should retain an arrangement attempt");
        black_box(attempt.topology_assembly_is_complete());
    });

    time_stage(case, "attempt_region_ownership_resolution", || {
        let attempt = attempt_workspace
            .evaluate(request)
            .unwrap()
            .retained_arrangement_attempt()
            .expect("evaluation should retain an arrangement attempt");
        black_box((
            attempt.region_ownership_is_resolved(),
            attempt.region_ownership_is_volume_resolved(),
        ));
    });

    time_stage(case, "boolean_evaluate", || {
        let mut workspace = ExactBooleanWorkspace::new(&case.left, &case.right);
        black_box(workspace.evaluate(request).ok());
    });

    time_stage(case, "boolean_materialize_ref", || {
        let mut workspace = ExactBooleanWorkspace::new(&case.left, &case.right);
        black_box(workspace.materialize_ref(request).ok());
    });

    let mut workspace = ExactBooleanWorkspace::new(&case.left, &case.right);
    workspace.evaluate(request).ok();
    time_stage(case, "workspace_evaluation_validate", || {
        let evaluation = workspace.evaluate(request).unwrap();
        black_box(evaluation.validate().ok());
    });

    time_prepared_stage(
        case,
        "workspace_evaluation_source_replay",
        || retained_workspace_with_evaluation_for_case(case, request),
        |retained| {
            if let Some(evaluation) = retained.0.evaluate(retained.1).ok() {
                black_box(
                    evaluation
                        .validate_against_sources(&case.left, &case.right)
                        .ok(),
                );
            }
        },
    );

    workspace.evaluate(request).unwrap();
    time_stage(
        case,
        "workspace_topology_assembly_completion_from_evaluation_attempt",
        || {
            let attempt = workspace
                .evaluate(request)
                .unwrap()
                .retained_arrangement_attempt();
            black_box(attempt.map(|attempt| attempt.topology_assembly_is_complete()));
        },
    );

    time_prepared_stage(
        case,
        "attempt_source_replay_validate_for_topology_evidence",
        || retained_workspace_with_evaluation_for_case(case, request),
        |retained| {
            let attempt = retained
                .0
                .evaluate(retained.1)
                .unwrap()
                .retained_arrangement_attempt()
                .expect("evaluation should retain an arrangement attempt");
            black_box(
                attempt
                    .validate_against_sources_for_request(&case.left, &case.right, request)
                    .ok()
                    .zip(Some(attempt.topology_assembly_is_complete())),
            );
        },
    );

    time_stage(
        case,
        "workspace_region_ownership_resolution_from_evaluation_attempt",
        || {
            let attempt = workspace
                .evaluate(request)
                .unwrap()
                .retained_arrangement_attempt();
            black_box(attempt.map(|attempt| {
                (
                    attempt.region_ownership_is_resolved(),
                    attempt.region_ownership_is_volume_resolved(),
                    attempt.region_ownership_resolves_requested_operation(),
                )
            }));
        },
    );

    time_prepared_stage(
        case,
        "attempt_source_replay_validate_for_ownership_evidence",
        || retained_workspace_with_evaluation_for_case(case, request),
        |retained| {
            let attempt = retained
                .0
                .evaluate(retained.1)
                .unwrap()
                .retained_arrangement_attempt()
                .expect("evaluation should retain an arrangement attempt");
            black_box(
                attempt
                    .validate_against_sources_for_request(&case.left, &case.right, request)
                    .ok()
                    .zip(Some(
                        attempt.region_ownership_resolves_requested_operation(),
                    )),
            );
        },
    );

    workspace.evaluate(request).unwrap();
    time_stage(case, "workspace_evaluation_attempt_cached", || {
        let evaluation = workspace.evaluate(request).unwrap();
        let attempt = evaluation.retained_arrangement_attempt();
        black_box(attempt.map(|attempt| {
            (
                attempt.topology_assembly_is_complete(),
                attempt.region_ownership_is_resolved(),
                attempt.region_ownership_is_volume_resolved(),
            )
        }));
    });

    time_prepared_stage(
        case,
        "attempt_validate_source_replay",
        || retained_workspace_with_evaluation_for_case(case, request),
        |retained| {
            let attempt = retained
                .0
                .evaluate(retained.1)
                .unwrap()
                .retained_arrangement_attempt()
                .expect("evaluation should retain an arrangement attempt");
            black_box(
                attempt
                    .validate_against_sources_for_request(&case.left, &case.right, retained.1)
                    .ok(),
            );
        },
    );

    time_prepared_stage(
        case,
        "workspace_validate_evaluation_from_retained_artifacts",
        || retained_workspace_with_evaluation_for_case(case, request),
        |retained| {
            black_box(validate_retained_evaluation_for_case(case, retained));
        },
    );

    time_prepared_stage(
        case,
        "workspace_evaluation_from_retained_artifacts",
        || retained_workspace_for_case(case),
        |retained_workspace| {
            black_box(retained_workspace.evaluate(request).ok());
        },
    );

    time_prepared_stage(
        case,
        "workspace_materialize_ref_from_retained_artifacts",
        || retained_workspace_for_case(case),
        |retained_workspace| {
            black_box(retained_workspace.materialize_ref(request).ok());
        },
    );

    time_prepared_stage(
        case,
        "evaluation_validate_source_replay",
        || retained_workspace_with_evaluation_for_case(case, request),
        |retained| {
            black_box(validate_retained_evaluation_for_case(case, retained));
        },
    );

    time_prepared_stage(
        case,
        "evaluation_validate_retained_replay",
        || retained_workspace_with_evaluation_for_case(case, request),
        |retained| {
            black_box(validate_retained_evaluation_for_case(case, retained));
        },
    );

    let mut materialize_cache_workspace = retained_workspace_for_case(case);
    materialize_cache_workspace.materialize_ref(request).ok();
    time_stage(case, "workspace_materialization_cached", || {
        black_box(materialize_cache_workspace.materialize_ref(request).ok());
    });

    workspace.evaluate(request).ok();
    time_stage(case, "workspace_evaluation_cached", || {
        black_box(workspace.evaluate(request).ok());
    });

    time_stage(case, "workspace_cached_materialization_ref", || {
        black_box(workspace.materialize_ref(request).ok());
    });
}

fn retained_workspace_for_case<'a>(case: &'a BenchCase) -> ExactBooleanWorkspace<'a> {
    ExactBooleanWorkspace::new(&case.left, &case.right)
}

fn retained_workspace_with_evaluation_for_case<'a>(
    case: &'a BenchCase,
    request: ExactBooleanRequest,
) -> (ExactBooleanWorkspace<'a>, ExactBooleanRequest) {
    let mut retained_workspace = retained_workspace_for_case(case);
    retained_workspace.evaluate(request).ok();
    (retained_workspace, request)
}

fn validate_retained_evaluation_for_case<'a>(
    case: &BenchCase,
    retained: &mut (ExactBooleanWorkspace<'a>, ExactBooleanRequest),
) -> Option<()> {
    retained
        .0
        .evaluate(retained.1)
        .ok()?
        .validate_against_sources(&case.left, &case.right)
        .ok()
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
