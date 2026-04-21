use std::hint::black_box;

use criterion::{Criterion, criterion_group, criterion_main};
use realistic_blas::{Matrix3, Matrix4, Scalar, Vector3, Vector4};

fn r(value: i32) -> Scalar {
    value.into()
}

fn vector3_a() -> Vector3 {
    Vector3::new([r(3), r(4), r(5)])
}

fn vector3_b() -> Vector3 {
    Vector3::new([r(7), r(11), r(13)])
}

fn vector4_a() -> Vector4 {
    Vector4::new([r(3), r(4), r(5), r(1)])
}

fn matrix3_a() -> Matrix3 {
    Matrix3::new([[r(1), r(2), r(3)], [r(0), r(1), r(4)], [r(5), r(6), r(0)]])
}

fn matrix3_b() -> Matrix3 {
    Matrix3::new([
        [r(2), r(0), r(1)],
        [r(3), r(5), r(7)],
        [r(11), r(13), r(17)],
    ])
}

fn matrix4_a() -> Matrix4 {
    Matrix4::new([
        [r(1), r(2), r(3), r(4)],
        [r(0), r(1), r(4), r(2)],
        [r(5), r(6), r(0), r(1)],
        [r(2), r(7), r(1), r(3)],
    ])
}

fn matrix4_b() -> Matrix4 {
    Matrix4::new([
        [r(2), r(0), r(1), r(3)],
        [r(3), r(5), r(7), r(11)],
        [r(11), r(13), r(17), r(19)],
        [r(23), r(29), r(31), r(37)],
    ])
}

fn bench_vectors(c: &mut Criterion) {
    let mut group = c.benchmark_group("vectors");

    group.bench_function("vec3 dot", |b| {
        let lhs = vector3_a();
        let rhs = vector3_b();
        b.iter(|| black_box(black_box(&lhs).dot(black_box(&rhs))));
    });

    group.bench_function("vec3 magnitude", |b| {
        let vector = vector3_a();
        b.iter(|| black_box(black_box(&vector).magnitude().unwrap()));
    });

    group.bench_function("vec3 normalize", |b| {
        let vector = vector3_a();
        b.iter(|| black_box(black_box(&vector).normalize().unwrap()));
    });

    group.finish();
}

fn bench_matrix3(c: &mut Criterion) {
    let mut group = c.benchmark_group("matrix3");

    group.bench_function("mat3 determinant", |b| {
        let matrix = matrix3_a();
        b.iter(|| black_box(black_box(&matrix).determinant()));
    });

    group.bench_function("mat3 inverse", |b| {
        let matrix = matrix3_a();
        b.iter(|| black_box(black_box(matrix.clone()).inverse().unwrap()));
    });

    group.bench_function("mat3 mul mat3", |b| {
        let lhs = matrix3_a();
        let rhs = matrix3_b();
        b.iter(|| black_box(black_box(lhs.clone()) * black_box(rhs.clone())));
    });

    group.bench_function("mat3 transform vec3", |b| {
        let matrix = matrix3_a();
        let vector = vector3_a();
        b.iter(|| black_box(black_box(matrix.clone()) * black_box(vector.clone())));
    });

    group.finish();
}

fn bench_matrix4(c: &mut Criterion) {
    let mut group = c.benchmark_group("matrix4");

    group.bench_function("mat4 determinant", |b| {
        let matrix = matrix4_a();
        b.iter(|| black_box(black_box(&matrix).determinant()));
    });

    group.bench_function("mat4 inverse", |b| {
        let matrix = matrix4_a();
        b.iter(|| black_box(black_box(matrix.clone()).inverse().unwrap()));
    });

    group.bench_function("mat4 mul mat4", |b| {
        let lhs = matrix4_a();
        let rhs = matrix4_b();
        b.iter(|| black_box(black_box(lhs.clone()) * black_box(rhs.clone())));
    });

    group.bench_function("mat4 transform vec4", |b| {
        let matrix = matrix4_a();
        let vector = vector4_a();
        b.iter(|| black_box(black_box(matrix.clone()) * black_box(vector.clone())));
    });

    group.finish();
}

criterion_group!(benches, bench_vectors, bench_matrix3, bench_matrix4);
criterion_main!(benches);
