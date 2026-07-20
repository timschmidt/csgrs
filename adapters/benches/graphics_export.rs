use std::hint::black_box;
use std::time::Instant;

use csgrs_adapter::{F32, F64, GraphicsMesh, Mesh, RawReal, Real};

const SAMPLES: usize = 10;
const WARMUP: usize = 2;

fn separate_then_merge_f32(mesh: &csgrs_adapter::core::mesh::Mesh<()>) -> GraphicsMesh<f32> {
    let buffers = mesh.try_to_gpu_mesh_f32().unwrap();
    GraphicsMesh {
        vertices: buffers.positions.into_iter().zip(buffers.normals).collect(),
        indices: buffers.indices,
    }
}

fn separate_then_merge_f64(mesh: &csgrs_adapter::core::mesh::Mesh<()>) -> GraphicsMesh<f64> {
    let buffers = mesh.try_to_gpu_mesh_f64().unwrap();
    GraphicsMesh {
        vertices: buffers.positions.into_iter().zip(buffers.normals).collect(),
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
    let raw = Mesh::<RawReal, ()>::sphere(Real::from(10_u8), 128, 64, ())
        .unwrap()
        .into_raw();
    let _ = raw.build_graphics_mesh();
    let f32_mesh = Mesh::<F32, ()>::from_raw(raw.clone());
    let f64_mesh = Mesh::<F64, ()>::from_raw(raw.clone());

    assert_eq!(
        f32_mesh.graphics_mesh().unwrap(),
        separate_then_merge_f32(&raw)
    );
    println!("path,sample,elapsed_ns,vertices");
    measure("adapter_interleaved_f32", || {
        f32_mesh.graphics_mesh().unwrap()
    });
    measure("separate_then_merge_f32", || separate_then_merge_f32(&raw));
    assert_eq!(
        f64_mesh.graphics_mesh().unwrap(),
        separate_then_merge_f64(&raw)
    );
    measure("adapter_interleaved_f64", || {
        f64_mesh.graphics_mesh().unwrap()
    });
    measure("separate_then_merge_f64", || separate_then_merge_f64(&raw));
}
