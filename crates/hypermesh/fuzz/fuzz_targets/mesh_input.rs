#![no_main]

mod common;

use common::{exercise_mesh_kernel_pair, generated_tetra_pair};
use hypermesh::Mesh;
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

    if let Ok(mesh) = Mesh::from_lossy_f64_triangles(&pos, &idx) {
        mesh.view().validate_retained_state().unwrap();
        exercise_mesh_kernel_pair(&mesh, &mesh);
    }

    if let Some((left, right)) = generated_tetra_pair(data) {
        exercise_mesh_kernel_pair(&left, &right);
    }
});
