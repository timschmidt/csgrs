#![no_main]

use hypermesh::{ExactMesh, ValidationPolicy};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let mut pos = Vec::new();
    let mut idx = Vec::new();

    for chunk in data.chunks_exact(8).take(96) {
        pos.push(i64::from_le_bytes(chunk.try_into().unwrap()));
    }

    for chunk in data.chunks_exact(2).skip(96).take(192) {
        idx.push(u16::from_le_bytes(chunk.try_into().unwrap()) as usize);
    }

    if let Ok(mesh) = ExactMesh::from_i64_triangles(&pos, &idx) {
        mesh.validate_retained_state().unwrap();
        exercise_mesh_pair(&mesh, &mesh);
    }

    if let Some((left, right)) = generated_tetra_pair(data) {
        exercise_mesh_pair(&left, &right);
    }
});

fn exercise_mesh_pair(left: &ExactMesh, right: &ExactMesh) {
    for result in [
        left.union(right),
        left.intersection(right),
        left.difference(right),
    ] {
        if let Ok(mesh) = result {
            let _ = mesh.validate_retained_state();
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
