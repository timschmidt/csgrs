#![no_main]

use hypermesh::{
    ExactBooleanOperation, ExactBooleanRequest, ExactBooleanWorkspace, ExactMesh,
    ExactReportFreshness, ValidationPolicy,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 12 {
        return;
    }

    let mut values = Vec::new();
    for chunk in data.chunks_exact(2).take(36) {
        let raw = i16::from_le_bytes(chunk.try_into().unwrap()) as i64;
        values.push(raw.rem_euclid(17) - 8);
    }

    exercise_mesh_boolean_workspace(&values);
    exercise_nested_shell_cavity_workspace();
});

fn exercise_mesh_boolean_workspace(values: &[i64]) {
    if values.len() < 24 {
        return;
    }
    let Some(left) = tetrahedron_from_values(&values[0..12]) else {
        return;
    };
    let Some(right) = tetrahedron_from_values(&values[12..24]) else {
        return;
    };
    exercise_workspace_requests(&left, &right, ValidationPolicy::ALLOW_BOUNDARY);
}

fn exercise_nested_shell_cavity_workspace() {
    let Some(left) = tetrahedron_with_reversed_inner() else {
        return;
    };
    let Some(right) = tetrahedron_mesh(
        &[
            [30, 0, 0],
            [31, 0, 0],
            [30, 1, 0],
            [30, 0, 1],
        ],
        ValidationPolicy::CLOSED,
    ) else {
        return;
    };
    exercise_workspace_requests(&left, &right, ValidationPolicy::CLOSED);
}

fn exercise_workspace_requests(left: &ExactMesh, right: &ExactMesh, validation: ValidationPolicy) {
    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let request = ExactBooleanRequest::new(operation, validation);
        let mut workspace = ExactBooleanWorkspace::new(left, right);
        if let Ok(evaluation) = workspace.evaluate(request) {
            let _ = evaluation.validate();
            let _ = evaluation.validate_against_sources(left, right);
            let _ = evaluation.freshness_against_sources(left, right);
            if let Some(attempt) = evaluation.retained_arrangement_attempt() {
                let _ = attempt.validate();
                let _ = attempt.freshness_against_sources_for_request(left, right, request);
            }
        }
        if let Ok(result) = workspace.materialize_ref(request) {
            let _ = result.validate();
            if let Ok(evaluation) = workspace.evaluate(request) {
                let _ = evaluation.validate_against_sources(left, right);
            }
        } else if let Ok(result) = workspace.materialize(request) {
            let _ = result.validate();
            if let Ok(evaluation) = workspace.evaluate(request) {
                let _ = evaluation.validate_against_sources(left, right);
            }
        }
    }

    let disjoint_request =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::ALLOW_BOUNDARY);
    let mut workspace = ExactBooleanWorkspace::new(left, right);
    if let Ok(evaluation) = workspace.evaluate(disjoint_request)
        && evaluation.freshness_against_sources(left, right) == ExactReportFreshness::Current
        && let Ok(result) = workspace.materialize_ref(disjoint_request)
    {
        let _ = result.validate();
    }
}

fn tetrahedron_from_values(values: &[i64]) -> Option<ExactMesh> {
    ExactMesh::from_i64_triangles_with_policy(
        values,
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

fn tetrahedron_mesh(points: &[[i64; 3]; 4], policy: ValidationPolicy) -> Option<ExactMesh> {
    ExactMesh::from_i64_triangles_with_policy(
        &[
            points[0][0],
            points[0][1],
            points[0][2],
            points[1][0],
            points[1][1],
            points[1][2],
            points[2][0],
            points[2][1],
            points[2][2],
            points[3][0],
            points[3][1],
            points[3][2],
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        policy,
    )
    .ok()
}

fn tetrahedron_with_reversed_inner() -> Option<ExactMesh> {
    let outer = [[0, 0, 0], [20, 0, 0], [0, 20, 0], [0, 0, 20]];
    let inner = [[1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]];
    let mut vertices = Vec::new();
    for point in outer.iter().chain(inner.iter()) {
        vertices.extend(point);
    }
    let shell_triangles = [[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]];
    let mut triangles = Vec::new();
    for tri in shell_triangles {
        triangles.extend([tri[0], tri[1], tri[2]]);
    }
    for tri in shell_triangles {
        triangles.extend([4 + tri[0], 4 + tri[2], 4 + tri[1]]);
    }
    ExactMesh::from_i64_triangles_with_policy(&vertices, &triangles, ValidationPolicy::CLOSED).ok()
}
