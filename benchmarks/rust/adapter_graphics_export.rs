//! Primitive-scalar graphics export benchmark.

use std::hint::black_box;
use std::time::Instant;

use csgrs::{
    Real,
    adapter::{F32, F64, GraphicsMesh, RawTriangleMesh, ScalarMesh},
    solid,
};
use hypermesh::TriangleMesh;

const SAMPLES: usize = 10;
const WARMUP: usize = 2;

fn separate_then_merge_f32(mesh: &TriangleMesh) -> GraphicsMesh<f32> {
    let buffers = RawTriangleMesh::from_native(mesh.clone())
        .graphics_mesh()
        .unwrap();
    GraphicsMesh {
        vertices: buffers
            .vertices
            .iter()
            .cloned()
            .map(|(position, normal)| {
                (
                    position.map(|value| value.to_f32_lossy().unwrap()),
                    normal.map(|value| value.to_f32_lossy().unwrap()),
                )
            })
            .collect::<Vec<_>>()
            .into(),
        indices: buffers.indices,
    }
}

fn separate_then_merge_f64(mesh: &TriangleMesh) -> GraphicsMesh<f64> {
    let buffers = RawTriangleMesh::from_native(mesh.clone())
        .graphics_mesh()
        .unwrap();
    GraphicsMesh {
        vertices: buffers
            .vertices
            .iter()
            .cloned()
            .map(|(position, normal)| {
                (
                    position.map(|value| value.to_f64_lossy().unwrap()),
                    normal.map(|value| value.to_f64_lossy().unwrap()),
                )
            })
            .collect::<Vec<_>>()
            .into(),
        indices: buffers.indices,
    }
}

fn measure<S>(name: &str, mut export: impl FnMut() -> GraphicsMesh<S>) {
    for _ in 0..WARMUP {
        black_box(export());
    }
    for sample in 0..SAMPLES {
        let start = Instant::now();
        let graphics = black_box(export());
        let elapsed = start.elapsed().as_nanos();
        let checksum = graphics.vertices.len().rotate_left(7) ^ graphics.indices.len();
        black_box(checksum);
        println!("{name},{sample},{elapsed},{}", graphics.vertices.len());
    }
}

fn main() {
    let native = solid::sphere(Real::from(10_u8), 128, 64);
    let _ = RawTriangleMesh::from_native(native.clone()).graphics_mesh();
    let f32_mesh = ScalarMesh::<F32>::from_native(native.clone());
    let f64_mesh = ScalarMesh::<F64>::from_native(native.clone());

    assert_eq!(
        f32_mesh.graphics_mesh().unwrap(),
        separate_then_merge_f32(&native)
    );
    println!("path,sample,elapsed_ns,vertices");
    measure("adapter_interleaved_f32", || {
        f32_mesh.graphics_mesh().unwrap()
    });
    measure("separate_then_merge_f32", || separate_then_merge_f32(&native));
    assert_eq!(
        f64_mesh.graphics_mesh().unwrap(),
        separate_then_merge_f64(&native)
    );
    measure("adapter_interleaved_f64", || {
        f64_mesh.graphics_mesh().unwrap()
    });
    measure("separate_then_merge_f64", || separate_then_merge_f64(&native));
}
