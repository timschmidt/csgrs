use criterion::{Criterion, criterion_group, criterion_main};
use realistic_blas::{Matrix3, Matrix4, Scalar, Vector3, Vector4, sqrt};

fn r(value: i32) -> Scalar {
    value.into()
}

#[cfg(feature = "hyperreal-backend")]
fn frac(numerator: i64, denominator: u64) -> Scalar {
    realistic_blas::Rational::fraction(numerator, denominator)
        .unwrap()
        .into()
}

#[cfg(not(feature = "hyperreal-backend"))]
fn frac(numerator: i64, denominator: u64) -> Scalar {
    Scalar::try_from(numerator as f64 / denominator as f64).unwrap()
}

fn bench_regression_sentinels(c: &mut Criterion) {
    c.bench_function("sentinel/scalar/cancellation_zero_status", |b| {
        let value: Scalar = ((Scalar::pi() * Scalar::e()) / Scalar::e()).unwrap() - Scalar::pi();
        b.iter(|| value.zero_status())
    });

    c.bench_function("sentinel/scalar/sqrt2_minus_convergent_sign", |b| {
        let value = sqrt(r(2)).unwrap() - frac(99, 70);
        b.iter(|| value.refine_sign_until(-128))
    });

    c.bench_function("sentinel/vector/dot_sparse_symbolic", |b| {
        let left = Vector3::new([Scalar::pi(), r(0), sqrt(r(2)).unwrap()]);
        let right = Vector3::new([frac(2, 3), Scalar::e(), r(0)]);
        b.iter(|| left.dot(&right))
    });

    c.bench_function("sentinel/matrix3/inverse_fractional", |b| {
        let matrix = Matrix3::new([
            [frac(9, 8), frac(3, 16), frac(-5, 8)],
            [frac(7, 4), frac(-11, 8), frac(13, 16)],
            [frac(5, 8), frac(17, 16), frac(19, 8)],
        ]);
        b.iter(|| matrix.clone().inverse_checked().unwrap())
    });

    c.bench_function("sentinel/matrix3/dense_transform_handle", |b| {
        let transform = Matrix3::new([
            [frac(9, 8), frac(3, 16), frac(-5, 8)],
            [frac(7, 4), frac(-11, 8), frac(13, 16)],
            [frac(5, 8), frac(17, 16), frac(19, 8)],
        ]);
        let vector = Vector3::new([frac(2, 3), frac(5, 7), frac(11, 13)]);
        let handle = transform.transform_vec3_handle();
        b.iter(|| handle.transform_vector(&vector))
    });

    c.bench_function("sentinel/matrix3/dense_transform_batch", |b| {
        let transform = Matrix3::new([
            [frac(9, 8), frac(3, 16), frac(-5, 8)],
            [frac(7, 4), frac(-11, 8), frac(13, 16)],
            [frac(5, 8), frac(17, 16), frac(19, 8)],
        ]);
        let vectors = vec![
            Vector3::new([frac(2, 3), frac(5, 7), frac(11, 13)]),
            Vector3::new([frac(17, 19), frac(23, 29), frac(31, 37)]),
            Vector3::new([frac(41, 43), frac(47, 53), frac(59, 61)]),
            Vector3::new([frac(67, 71), frac(73, 79), frac(83, 89)]),
        ];
        let handle = transform.transform_vec3_handle();
        b.iter(|| handle.transform_vector_batch(&vectors))
    });

    c.bench_function("sentinel/matrix3/dense_transform_batch_public", |b| {
        let transform = Matrix3::new([
            [frac(9, 8), frac(3, 16), frac(-5, 8)],
            [frac(7, 4), frac(-11, 8), frac(13, 16)],
            [frac(5, 8), frac(17, 16), frac(19, 8)],
        ]);
        let vectors = vec![
            Vector3::new([frac(2, 3), frac(5, 7), frac(11, 13)]),
            Vector3::new([frac(17, 19), frac(23, 29), frac(31, 37)]),
            Vector3::new([frac(41, 43), frac(47, 53), frac(59, 61)]),
            Vector3::new([frac(67, 71), frac(73, 79), frac(83, 89)]),
        ];
        b.iter(|| transform.transform_vec3_batch(&vectors))
    });

    c.bench_function("sentinel/matrix4/division_fractional", |b| {
        let numerator = Matrix4::identity();
        let divisor = Matrix4::new([
            [frac(11, 10), frac(2, 10), frac(3, 10), frac(4, 10)],
            [frac(5, 10), frac(17, 10), frac(7, 10), frac(-8, 10)],
            [frac(9, 10), frac(-10, 10), frac(23, 10), frac(12, 10)],
            [frac(-13, 10), frac(14, 10), frac(-15, 10), frac(19, 10)],
        ]);
        b.iter(|| {
            numerator
                .clone()
                .div_matrix_checked(divisor.clone())
                .unwrap()
        })
    });

    c.bench_function(
        "sentinel/matrix4/translated_diagonal_direction_transform_public",
        |b| {
            let transform = Matrix4::new([
                [r(2), r(0), r(0), r(100)],
                [r(0), r(3), r(0), r(200)],
                [r(0), r(0), r(4), r(300)],
                [r(0), r(0), r(0), r(1)],
            ]);
            let direction = Vector4::new([r(5), r(7), r(11), r(0)]);
            b.iter(|| transform.transform_vec4_direction(&direction))
        },
    );

    c.bench_function(
        "sentinel/matrix4/translated_diagonal_direction_transform_handle",
        |b| {
            let transform = Matrix4::new([
                [r(2), r(0), r(0), r(100)],
                [r(0), r(3), r(0), r(200)],
                [r(0), r(0), r(4), r(300)],
                [r(0), r(0), r(0), r(1)],
            ]);
            let direction = Vector4::new([r(5), r(7), r(11), r(0)]);
            let handle = transform.transform_vec4_handle();
            b.iter(|| handle.transform_direction_vector(&direction))
        },
    );

    c.bench_function(
        "sentinel/matrix4/translated_diagonal_point_transform_public",
        |b| {
            let transform = Matrix4::new([
                [r(2), r(0), r(0), r(100)],
                [r(0), r(3), r(0), r(200)],
                [r(0), r(0), r(4), r(300)],
                [r(0), r(0), r(0), r(1)],
            ]);
            let point = Vector4::new([r(5), r(7), r(11), r(1)]);
            b.iter(|| transform.transform_vec4_point(&point))
        },
    );

    c.bench_function(
        "sentinel/matrix4/translated_diagonal_point_transform_handle",
        |b| {
            let transform = Matrix4::new([
                [r(2), r(0), r(0), r(100)],
                [r(0), r(3), r(0), r(200)],
                [r(0), r(0), r(4), r(300)],
                [r(0), r(0), r(0), r(1)],
            ]);
            let point = Vector4::new([r(5), r(7), r(11), r(1)]);
            let handle = transform.transform_vec4_handle();
            b.iter(|| handle.transform_point_vector(&point))
        },
    );

    c.bench_function(
        "sentinel/matrix4/translated_diagonal_direction_batch_public",
        |b| {
            let transform = Matrix4::new([
                [r(2), r(0), r(0), r(100)],
                [r(0), r(3), r(0), r(200)],
                [r(0), r(0), r(4), r(300)],
                [r(0), r(0), r(0), r(1)],
            ]);
            let vectors = vec![
                Vector4::new([r(5), r(7), r(11), r(0)]),
                Vector4::new([r(13), r(17), r(19), r(0)]),
                Vector4::new([r(23), r(29), r(31), r(0)]),
                Vector4::new([r(37), r(41), r(43), r(0)]),
            ];
            b.iter(|| transform.transform_vec4_batch(&vectors))
        },
    );

    c.bench_function(
        "sentinel/matrix4/translated_diagonal_point_batch_public",
        |b| {
            let transform = Matrix4::new([
                [r(2), r(0), r(0), r(100)],
                [r(0), r(3), r(0), r(200)],
                [r(0), r(0), r(4), r(300)],
                [r(0), r(0), r(0), r(1)],
            ]);
            let vectors = vec![
                Vector4::new([r(5), r(7), r(11), r(1)]),
                Vector4::new([r(13), r(17), r(19), r(1)]),
                Vector4::new([r(23), r(29), r(31), r(1)]),
                Vector4::new([r(37), r(41), r(43), r(1)]),
            ];
            b.iter(|| transform.transform_vec4_batch(&vectors))
        },
    );

    c.bench_function(
        "sentinel/matrix4/translated_diagonal_direction_batch_handle",
        |b| {
            let transform = Matrix4::new([
                [r(2), r(0), r(0), r(100)],
                [r(0), r(3), r(0), r(200)],
                [r(0), r(0), r(4), r(300)],
                [r(0), r(0), r(0), r(1)],
            ]);
            let vectors = vec![
                Vector4::new([r(5), r(7), r(11), r(0)]),
                Vector4::new([r(13), r(17), r(19), r(0)]),
                Vector4::new([r(23), r(29), r(31), r(0)]),
                Vector4::new([r(37), r(41), r(43), r(0)]),
            ];
            let handle = transform.transform_vec4_handle();
            b.iter(|| handle.transform_vector_batch(&vectors))
        },
    );

    c.bench_function(
        "sentinel/matrix4/translated_diagonal_point_batch_handle",
        |b| {
            let transform = Matrix4::new([
                [r(2), r(0), r(0), r(100)],
                [r(0), r(3), r(0), r(200)],
                [r(0), r(0), r(4), r(300)],
                [r(0), r(0), r(0), r(1)],
            ]);
            let vectors = vec![
                Vector4::new([r(5), r(7), r(11), r(1)]),
                Vector4::new([r(13), r(17), r(19), r(1)]),
                Vector4::new([r(23), r(29), r(31), r(1)]),
                Vector4::new([r(37), r(41), r(43), r(1)]),
            ];
            let handle = transform.transform_vec4_handle();
            b.iter(|| handle.transform_vector_batch(&vectors))
        },
    );

    c.bench_function(
        "sentinel/matrix4/diagonal_affine_point_transform_handle",
        |b| {
            let transform = Matrix4::new([
                [r(2), r(0), r(0), r(0)],
                [r(0), r(3), r(0), r(0)],
                [r(0), r(0), r(4), r(0)],
                [r(0), r(0), r(0), r(1)],
            ]);
            let point = Vector4::new([r(5), r(7), r(11), r(1)]);
            let handle = transform.transform_vec4_handle();
            b.iter(|| handle.transform_vector(&point))
        },
    );

    c.bench_function("sentinel/matrix4/diagonal_affine_point_materialize", |b| {
        let transform = Matrix4::new([
            [r(2), r(0), r(0), r(0)],
            [r(0), r(3), r(0), r(0)],
            [r(0), r(0), r(4), r(0)],
            [r(0), r(0), r(0), r(1)],
        ]);
        let point = Vector4::new([r(5), r(7), r(11), r(1)]);
        let handle = transform.transform_vec4_handle();
        b.iter(|| handle.vector(&point).materialize())
    });

    c.bench_function("sentinel/matrix4/diagonal_direction_batch", |b| {
        let transform = Matrix4::new([
            [r(2), r(0), r(0), r(0)],
            [r(0), r(3), r(0), r(0)],
            [r(0), r(0), r(4), r(0)],
            [r(0), r(0), r(0), r(1)],
        ]);
        let vectors = vec![
            Vector4::new([r(5), r(7), r(11), r(0)]),
            Vector4::new([r(13), r(17), r(19), r(0)]),
            Vector4::new([r(23), r(29), r(31), r(0)]),
            Vector4::new([r(37), r(41), r(43), r(0)]),
        ];
        let handle = transform.transform_vec4_handle();
        b.iter(|| handle.transform_vector_batch(&vectors))
    });

    c.bench_function("sentinel/matrix4/diagonal_point_batch", |b| {
        let transform = Matrix4::new([
            [r(2), r(0), r(0), r(0)],
            [r(0), r(3), r(0), r(0)],
            [r(0), r(0), r(4), r(0)],
            [r(0), r(0), r(0), r(1)],
        ]);
        let vectors = vec![
            Vector4::new([r(5), r(7), r(11), r(1)]),
            Vector4::new([r(13), r(17), r(19), r(1)]),
            Vector4::new([r(23), r(29), r(31), r(1)]),
            Vector4::new([r(37), r(41), r(43), r(1)]),
        ];
        let handle = transform.transform_vec4_handle();
        b.iter(|| handle.transform_vector_batch(&vectors))
    });

    c.bench_function("sentinel/matrix4/diagonal_unknown_batch", |b| {
        let transform = Matrix4::new([
            [r(2), r(0), r(0), r(0)],
            [r(0), r(3), r(0), r(0)],
            [r(0), r(0), r(4), r(0)],
            [r(0), r(0), r(0), r(1)],
        ]);
        let vectors = vec![
            Vector4::new([r(5), r(7), r(11), frac(1, 2)]),
            Vector4::new([r(13), r(17), r(19), frac(1, 3)]),
            Vector4::new([r(23), r(29), r(31), frac(1, 5)]),
            Vector4::new([r(37), r(41), r(43), frac(1, 7)]),
        ];
        let handle = transform.transform_vec4_handle();
        b.iter(|| handle.transform_vector_batch(&vectors))
    });
}

criterion_group!(benches, bench_regression_sentinels);
criterion_main!(benches);
