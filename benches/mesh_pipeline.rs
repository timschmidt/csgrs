//! Repeatable mesh conversion and Boolean benchmark for time and heap tracing.

use std::hint::black_box;
use std::time::Instant;

use csgrs::{Real, csg::CSG, mesh::Mesh};

fn iterations(name: &str, default: usize) -> usize {
    std::env::var(name)
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(default)
}

fn main() {
    let construction_iterations = iterations("CSGRS_CONSTRUCTION_BENCH_ITERS", 128);
    let start = Instant::now();
    let mut constructed_polygons = 0_usize;
    for _ in 0..construction_iterations {
        let mesh = Mesh::sphere(Real::from(10), 24, 12, ());
        constructed_polygons += mesh.polygons.len();
        black_box(mesh);
    }
    println!(
        "mesh_construction iterations={construction_iterations} polygons={constructed_polygons} elapsed_us={}",
        start.elapsed().as_micros()
    );

    let sphere = Mesh::sphere(Real::from(10), 24, 12, ());
    let buffer_iterations = iterations("CSGRS_BUFFER_BENCH_ITERS", 128);
    let start = Instant::now();
    let mut buffer_words = 0_usize;
    for _ in 0..buffer_iterations {
        let buffers = black_box(&sphere).to_hypermesh_buffers();
        buffer_words += buffers.positions.len() + buffers.indices.len();
        black_box(buffers);
    }
    println!(
        "mesh_buffers iterations={buffer_iterations} words={buffer_words} elapsed_us={}",
        start.elapsed().as_micros()
    );

    let left = Mesh::cube(Real::from(2), ());
    let right = Mesh::cube(Real::from(2), ()).translate(
        (Real::from(1) / Real::from(2)).expect("nonzero denominator"),
        (Real::from(1) / Real::from(3)).expect("nonzero denominator"),
        (Real::from(1) / Real::from(5)).expect("nonzero denominator"),
    );
    let boolean_iterations = iterations("CSGRS_BOOLEAN_BENCH_ITERS", 8);
    let start = Instant::now();
    let mut polygons = 0_usize;
    for _ in 0..boolean_iterations {
        let output = black_box(&left)
            .try_union(black_box(&right))
            .expect("benchmark Boolean remains certified");
        polygons += output.polygons.len();
        black_box(output);
    }
    println!(
        "mesh_boolean iterations={boolean_iterations} polygons={polygons} elapsed_us={}",
        start.elapsed().as_micros()
    );
}
