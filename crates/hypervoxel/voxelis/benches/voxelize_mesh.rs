use criterion::{black_box, criterion_group, criterion_main, Criterion};

use voxelis::{chunk::Vec3, Chunk};

fn benchmark_meshing(c: &mut Criterion) {
    let offset = Vec3::ZERO;

    let mut chunk = voxelis::Chunk::new();
    chunk.generate_test_data();

    let data = chunk.to_vec(0);

    let mut test_vertices = Vec::new();
    let mut test_normals = Vec::new();
    let mut test_indices = Vec::new();
    Chunk::generate_mesh_arrays(
        &data,
        &mut test_vertices,
        &mut test_normals,
        &mut test_indices,
        offset,
    );

    println!("naive_meshing");
    println!(" vertices len: {}", test_vertices.len());
    println!("  normals len: {}", test_normals.len());
    println!("  indices len: {}", test_indices.len());

    let vertices_len = test_vertices.len();
    let normals_len = test_normals.len();
    let indices_len = test_indices.len();

    let mut vertices = Vec::with_capacity(vertices_len);
    let mut normals = Vec::with_capacity(normals_len);
    let mut indices = Vec::with_capacity(indices_len);

    c.bench_function("naive meshing", |b| {
        b.iter(|| {
            vertices.clear();
            normals.clear();
            indices.clear();

            Chunk::generate_mesh_arrays(
                black_box(&data),
                black_box(&mut vertices),
                black_box(&mut normals),
                black_box(&mut indices),
                black_box(offset),
            );
        })
    });
    c.bench_function("greedy meshing", |b| {
        b.iter(|| {
            vertices.clear();
            normals.clear();
            indices.clear();

            Chunk::generate_greedy_mesh_arrays(
                black_box(&data),
                black_box(&mut vertices),
                black_box(&mut normals),
                black_box(&mut indices),
                black_box(offset),
            );
        })
    });
}

criterion_group!(benches, benchmark_meshing);

criterion_main!(benches);
