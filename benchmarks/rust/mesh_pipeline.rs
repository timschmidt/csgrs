//! Repeatable mesh conversion and Boolean benchmark for timing and heap tracing.

use std::hint::black_box;
use std::time::Instant;

use csgrs::{
    Real,
    solid::{self, SolidExt},
};
use hyperlimit::PredicatePolicy;
use hypermesh::MeshContext;

const MESH_CONTEXT: MeshContext = MeshContext::new(PredicatePolicy::STRICT);

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
        let mesh = solid::sphere(Real::from(10), 24, 12);
        constructed_polygons += mesh.triangles.len();
        black_box(mesh);
    }
    println!(
        "mesh_construction iterations={construction_iterations} polygons={constructed_polygons} elapsed_us={}",
        start.elapsed().as_micros()
    );

    let sphere = solid::sphere(Real::from(10), 24, 12);
    let buffer_iterations = iterations("CSGRS_BUFFER_BENCH_ITERS", 128);
    let start = Instant::now();
    let mut buffer_words = 0_usize;
    for _ in 0..buffer_iterations {
        let native = black_box(&sphere);
        buffer_words += native.positions.len() * 3 + native.triangles.len() * 3;
        black_box(native);
    }
    println!(
        "mesh_buffers iterations={buffer_iterations} words={buffer_words} elapsed_us={}",
        start.elapsed().as_micros()
    );

    let left = solid::cube(Real::from(2));
    let right = solid::cube(Real::from(2)).translated(
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
        polygons += output.triangles.len();
        black_box(output);
    }
    println!(
        "mesh_boolean iterations={boolean_iterations} polygons={polygons} elapsed_us={}",
        start.elapsed().as_micros()
    );

    let hull_source = solid::sphere(Real::from(10), 16, 8);
    let hull_iterations = iterations("CSGRS_HULL_BENCH_ITERS", 4);
    let start = Instant::now();
    let mut hull_polygons = 0_usize;
    for _ in 0..hull_iterations {
        let hull = solid::convex_hull(black_box(&hull_source)).expect("3D hull");
        hull_polygons += hull.triangles.len();
        black_box(hull);
    }
    println!(
        "mesh_convex_hull iterations={hull_iterations} polygons={hull_polygons} elapsed_us={}",
        start.elapsed().as_micros()
    );

    let raw_hull_points = hull_source.positions.clone();
    let raw_hull_iterations = iterations("CSGRS_RAW_HULL_BENCH_ITERS", 4);
    let start = Instant::now();
    let mut raw_hull_triangles = 0_usize;
    for _ in 0..raw_hull_iterations {
        let hull = hypermesh::convex_hull(&MESH_CONTEXT, black_box(&raw_hull_points))
            .expect("benchmark point cloud spans 3D")
            .into_value();
        raw_hull_triangles += hull.triangles.len();
        black_box(hull);
    }
    println!(
        "raw_hypermesh_hull iterations={raw_hull_iterations} triangles={raw_hull_triangles} elapsed_us={}",
        start.elapsed().as_micros()
    );

    let minkowski_left = solid::cube(Real::from(2));
    let minkowski_right = solid::cube(Real::from(3));
    let minkowski_iterations = iterations("CSGRS_MINKOWSKI_BENCH_ITERS", 16);
    let start = Instant::now();
    let mut minkowski_polygons = 0_usize;
    for _ in 0..minkowski_iterations {
        let sum =
            solid::minkowski_sum(black_box(&minkowski_left), black_box(&minkowski_right))
                .expect("Minkowski sum");
        minkowski_polygons += sum.triangles.len();
        black_box(sum);
    }
    println!(
        "mesh_minkowski iterations={minkowski_iterations} polygons={minkowski_polygons} elapsed_us={}",
        start.elapsed().as_micros()
    );
}
