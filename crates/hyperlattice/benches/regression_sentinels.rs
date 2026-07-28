use criterion::{Criterion, criterion_group, criterion_main};
use hyperlattice::{Matrix3, Matrix4, Real, SharedScaleVec, Vector3, Vector4, sqrt};
use std::hint::black_box;

fn r(value: i32) -> Real {
    value.into()
}

fn frac(numerator: i64, denominator: u64) -> Real {
    hyperlattice::Rational::fraction(numerator, denominator)
        .unwrap()
        .into()
}

fn cold_self_dot_vector() -> Vector3 {
    Vector3::new([
        frac(123_456_789_012_345, 1_u64 << 50),
        frac(-234_567_890_123_457, 1_u64 << 49),
        frac(345_678_901_234_569, 1_u64 << 48),
    ])
}

fn bench_regression_sentinels(c: &mut Criterion) {
    c.bench_function("sentinel/scalar/cancellation_zero_status", |b| {
        let value: Real = ((Real::pi() * Real::e()) / Real::e()).unwrap() - Real::pi();
        b.iter(|| value.zero_status())
    });

    c.bench_function("sentinel/scalar/sqrt2_minus_convergent_sign", |b| {
        let value = sqrt(r(2)).unwrap() - frac(99, 70);
        b.iter(|| value.refine_sign_until(-128))
    });

    c.bench_function("sentinel/vector/dot_sparse_symbolic", |b| {
        let left = Vector3::new([Real::pi(), r(0), sqrt(r(2)).unwrap()]);
        let right = Vector3::new([frac(2, 3), Real::e(), r(0)]);
        b.iter(|| left.dot(&right))
    });

    c.bench_function("sentinel/vector/self_dot_retained_dyadic", |b| {
        let vector = cold_self_dot_vector();
        b.iter(|| black_box(&vector).norm_squared())
    });

    c.bench_function("sentinel/vector/magnitude_retained_dyadic", |b| {
        let vector = cold_self_dot_vector();
        b.iter(|| black_box(&vector).magnitude().unwrap())
    });

    c.bench_function("sentinel/vector/inverse_magnitude_retained_dyadic", |b| {
        let vector = cold_self_dot_vector();
        b.iter(|| black_box(&vector).magnitude().unwrap().inverse().unwrap())
    });

    c.bench_function("sentinel/vector/normalize_retained_dyadic", |b| {
        let vector = cold_self_dot_vector();
        b.iter(|| black_box(&vector).normalize().unwrap())
    });

    c.bench_function("sentinel/vector/normalize_cold_dyadic", |b| {
        b.iter_batched(
            cold_self_dot_vector,
            |vector| black_box(vector).normalize().unwrap(),
            criterion::BatchSize::SmallInput,
        )
    });

    c.bench_function("sentinel/vector/self_dot_cold_dyadic", |b| {
        b.iter_batched(
            cold_self_dot_vector,
            |vector| black_box(vector).norm_squared(),
            criterion::BatchSize::SmallInput,
        )
    });

    c.bench_function("sentinel/vector/dot_equal_distinct_cold_dyadic", |b| {
        b.iter_batched(
            || (cold_self_dot_vector(), cold_self_dot_vector()),
            |(left, right)| black_box(left).dot(black_box(&right)),
            criterion::BatchSize::SmallInput,
        )
    });

    c.bench_function(
        "sentinel/vector/shared_scale_view_common_denominator",
        |b| {
            let vector = Vector4::new([frac(1, 7), frac(-2, 7), frac(3, 7), frac(4, 7)]);
            b.iter(|| vector.shared_scale_view())
        },
    );

    c.bench_function(
        "sentinel/vector/shared_scale_owned_common_denominator",
        |b| {
            b.iter_batched(
                || [frac(1, 7), frac(-2, 7), frac(3, 7), frac(4, 7)],
                |components| {
                    black_box(SharedScaleVec::from_components(components))
                        .expect("sevenths share a reduced denominator")
                },
                criterion::BatchSize::SmallInput,
            )
        },
    );

    c.bench_function("sentinel/matrix3/inverse_fractional", |b| {
        let matrix = Matrix3::new([
            [frac(9, 8), frac(3, 16), frac(-5, 8)],
            [frac(7, 4), frac(-11, 8), frac(13, 16)],
            [frac(5, 8), frac(17, 16), frac(19, 8)],
        ]);
        b.iter(|| matrix.clone().inverse_checked().unwrap())
    });

    c.bench_function("sentinel/matrix3/sparse_mask_product", |b| {
        let left = Matrix3::new([
            [frac(2, 3), frac(3, 5), r(0)],
            [r(0), frac(5, 7), frac(7, 11)],
            [r(0), r(0), frac(11, 13)],
        ]);
        let right = Matrix3::new([
            [frac(13, 17), r(0), r(0)],
            [frac(17, 19), frac(19, 23), r(0)],
            [r(0), frac(23, 29), frac(29, 31)],
        ]);
        b.iter(|| black_box(black_box(&left) * black_box(&right)))
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

    c.bench_function("sentinel/matrix4/determinant_fractional", |b| {
        let matrix = Matrix4::new([
            [frac(11, 10), frac(2, 10), frac(3, 10), frac(4, 10)],
            [frac(5, 10), frac(17, 10), frac(7, 10), frac(-8, 10)],
            [frac(9, 10), frac(-10, 10), frac(23, 10), frac(12, 10)],
            [frac(-13, 10), frac(14, 10), frac(-15, 10), frac(19, 10)],
        ]);
        b.iter(|| black_box(black_box(&matrix).determinant()))
    });

    c.bench_function("sentinel/matrix4/sparse_mask_product", |b| {
        let left = Matrix4::new([
            [frac(2, 3), frac(3, 5), r(0), r(0)],
            [r(0), frac(5, 7), frac(7, 11), r(0)],
            [r(0), r(0), frac(11, 13), frac(13, 17)],
            [r(0), r(0), r(0), frac(17, 19)],
        ]);
        let right = Matrix4::new([
            [frac(19, 23), r(0), r(0), r(0)],
            [frac(23, 29), frac(29, 31), r(0), r(0)],
            [r(0), frac(31, 37), frac(37, 41), r(0)],
            [r(0), r(0), frac(41, 43), frac(43, 47)],
        ]);
        b.iter(|| black_box(black_box(&left) * black_box(&right)))
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
        b.iter(|| transform.transform_vec4_direction_batch(&vectors))
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
        b.iter(|| transform.transform_vec4_point_batch(&vectors))
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
        b.iter(|| transform.transform_vec4_batch(&vectors))
    });
}

criterion_group!(benches, bench_regression_sentinels);
criterion_main!(benches);
