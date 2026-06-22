use hypermesh::{ExactMesh, ValidationPolicy};

pub fn exercise_mesh_kernel_pair(left: &ExactMesh, right: &ExactMesh) {
    left.validate_retained_state().unwrap();
    right.validate_retained_state().unwrap();
    let left_view = left.view();
    let right_view = right.view();

    let mut streamed_pair_count = 0usize;
    left_view
        .visit_candidate_face_pairs(right_view, |_| {
            streamed_pair_count += 1;
        })
        .unwrap();

    let prepared_pair = left_view.prepare_pair_broad_phase(right_view).unwrap();
    assert_eq!(
        prepared_pair.candidate_face_pair_count(),
        streamed_pair_count
    );

    assert_eq!(
        prepared_pair.candidate_face_pairs().len(),
        streamed_pair_count
    );
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
        ValidationPolicy::CLOSED,
    )
    .ok()
}
