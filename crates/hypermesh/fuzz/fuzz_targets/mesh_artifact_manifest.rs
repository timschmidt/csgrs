#![no_main]

use hypermesh::{
    ExactBooleanOperation, ExactBooleanRequest, ExactBooleanWorkspace, ExactMesh,
    MeshArtifactManifest, ValidationPolicy,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let Some((left, right)) = generated_tetra_pair(data) else {
        return;
    };

    for mesh in [&left, &right] {
        mesh.validate_retained_state().unwrap();
        if let Ok(manifest) = MeshArtifactManifest::from_exact_mesh(mesh) {
            let report = manifest.report();
            assert_eq!(report.validation_handoff_ready, report.blockers.is_empty());
            if report.validation_handoff_ready {
                assert!(report.coordinates_exact_replay_ready);
                assert!(report.topology_validation_replay_ready);
                assert!(report.source_current);
            }
        }
    }

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let request = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED);
        let mut workspace = ExactBooleanWorkspace::new(&left, &right);
        if let Ok(evaluation) = workspace.evaluate(request) {
            let _ = evaluation.validate();
            let _ = evaluation.validate_against_sources(&left, &right);
        }
        if let Ok(result) = workspace.materialize(request) {
            let _ = result.validate();
            if let Ok(evaluation) = workspace.evaluate(request) {
                let _ = evaluation.validate_materialized_result_against_sources(&left, &right);
            }
        }
    }
});

fn generated_tetra_pair(data: &[u8]) -> Option<(ExactMesh, ExactMesh)> {
    if data.len() < 6 {
        return None;
    }
    let left_scale = i64::from(data[0] % 12) + 1;
    let right_scale = i64::from(data[1] % 12) + 1;
    let offset = [
        i64::from(data[2] % 16) - 8,
        i64::from(data[3] % 16) - 8,
        i64::from(data[4] % 16) - 8,
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
