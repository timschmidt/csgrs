use hypermesh::{ExactMesh, ExactMeshValidationPolicy};

pub fn exercise_mesh_kernel_pair(left: &ExactMesh, right: &ExactMesh) {
    left.validate_retained_state().unwrap();
    right.validate_retained_state().unwrap();
    let left_view = left.view();
    let right_view = right.view();
    let prepared_left = left_view.prepare_broad_phase().unwrap();
    let prepared_right = right_view.prepare_broad_phase().unwrap();

    let mut direct_candidate_pairs = 0;
    left_view
        .visit_candidate_face_pairs(right_view, &mut |_| {
            direct_candidate_pairs += 1;
        })
        .unwrap();
    let mut prepared_candidate_pairs = 0;
    prepared_left.visit_candidate_face_pairs(&prepared_right, &mut |_| {
        prepared_candidate_pairs += 1;
    });
    assert_eq!(direct_candidate_pairs, prepared_candidate_pairs);
}

pub fn generated_tetra_pair(data: &[u8]) -> Option<(ExactMesh, ExactMesh)> {
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
        ExactMeshValidationPolicy::CLOSED,
    )
    .ok()
}
