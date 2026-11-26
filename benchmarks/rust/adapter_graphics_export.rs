//! Warm primitive-scalar graphics export timings for a fixed sampled sphere.
//!
//! Compares direct scalar-adapter buffers with native buffers followed by explicit
//! projection. Equality assertions compare their projected outputs, not fidelity
//! to an analytic sphere. Export timing excludes printing and final buffer drop.

use std::hint::black_box;
use std::time::Instant;

use csgrs::{
    Real,
    adapter::{F32, F64, GraphicsMesh, RawTriangleMesh, ScalarMesh},
    solid,
};
use hypermesh::TriangleMesh;

/// Number of timed exports emitted for each path after warmup.
const SAMPLES: usize = 10;
/// Untimed exports discarded before each path's samples.
const WARMUP: usize = 2;

/// Builds native interleaved graphics rows, then projects every position/normal to f32.
/// Clones the native mesh carrier and preserves returned indices; export/projection
/// failure panics. This materializes an intermediate native graphics buffer.
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

/// Builds native interleaved graphics rows, then projects every position/normal to f64.
/// Clones the native mesh carrier and preserves returned indices; export/projection
/// failure panics. This materializes an intermediate native graphics buffer.
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

/// Warms an export closure twice, then prints ten individual nanosecond samples.
/// Times creation/materialization through black_box, excluding count fingerprinting,
/// printing and destruction of the retained sample buffer. The count-only fingerprint
/// is consumed but not emitted; neither it nor the vertex column certifies geometry.
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

/// Builds a radius-ten 128-by-64 sampled sphere and compares two paths at each precision.
/// Native export is attempted before adapter creation; equality checks further
/// warm shared data. Times f32 paths before f64 paths without cache resets, and
/// requires exact equality of the corresponding projected graphics buffers.
fn main() {
    let native = solid::sphere(Real::from(10_u8), 128, 64).expect("valid primitive fixture");
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
