#![no_main]

use hypermesh::{
    ExactBooleanOperation, ExactBooleanRequest, ExactBooleanWorkspace, ExactMesh, ValidationPolicy,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let mut pos = Vec::new();
    let mut idx = Vec::new();

    for chunk in data.chunks_exact(8).take(48) {
        pos.push(f64::from_le_bytes(chunk.try_into().unwrap()));
    }

    for chunk in data.chunks_exact(2).skip(48).take(96) {
        idx.push(u16::from_le_bytes(chunk.try_into().unwrap()) as usize);
    }

    if let Ok(mesh) = ExactMesh::from_f64_triangles(&pos, &idx) {
        mesh.validate_retained_state().unwrap();
        exercise_workspace_against_self(&mesh, ValidationPolicy::ALLOW_BOUNDARY);
    }

    if let Some((left, right)) = generated_tetra_pair(data) {
        exercise_workspace_pair(&left, &right, ValidationPolicy::CLOSED);
    }
});

fn exercise_workspace_against_self(mesh: &ExactMesh, validation: ValidationPolicy) {
    exercise_workspace_pair(mesh, mesh, validation);
}

fn exercise_workspace_pair(left: &ExactMesh, right: &ExactMesh, validation: ValidationPolicy) {
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
            if let Some(attempt) = evaluation.retained_arrangement_attempt() {
                let _ = attempt.validate();
                let _ = attempt.validate_against_sources_with_validation(left, right, validation);
            }
        }
        if let Ok(result) = workspace.materialize_ref(request) {
            let _ = result.validate();
            let _ = result.validate_operation_against_sources(
                left,
                right,
                operation,
                validation,
                request.boundary_policy,
            );
        }
    }
}

fn generated_tetra_pair(data: &[u8]) -> Option<(ExactMesh, ExactMesh)> {
    if data.len() < 6 {
        return None;
    }
    let left_scale = i64::from(data[0] % 6) + 1;
    let right_scale = i64::from(data[1] % 6) + 1;
    let offset = [
        i64::from(data[2] % 8) - 4,
        i64::from(data[3] % 8) - 4,
        i64::from(data[4] % 8) - 4,
    ];
    Some((
        tetrahedron([0, 0, 0], left_scale)?,
        tetrahedron(offset, right_scale)?,
    ))
}

fn tetrahedron(offset: [i64; 3], scale: i64) -> Option<ExactMesh> {
    ExactMesh::from_i64_triangles_with_policy(
        &[
            offset[0],
            offset[1],
            offset[2],
            offset[0] + scale,
            offset[1],
            offset[2],
            offset[0],
            offset[1] + scale,
            offset[2],
            offset[0],
            offset[1],
            offset[2] + scale,
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        ValidationPolicy::CLOSED,
    )
    .ok()
}
