use criterion::{Criterion, criterion_group, criterion_main};
use realistic_blas::{Matrix3, Matrix4, Scalar, Vector3, sqrt};

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
        let value = ((Scalar::pi() * Scalar::e()) / Scalar::e()).unwrap() - Scalar::pi();
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

    c.bench_function("sentinel/matrix4/division_fractional", |b| {
        let numerator = Matrix4::identity();
        let divisor = Matrix4::new([
            [frac(11, 10), frac(2, 10), frac(3, 10), frac(4, 10)],
            [frac(5, 10), frac(17, 10), frac(7, 10), frac(-8, 10)],
            [frac(9, 10), frac(-10, 10), frac(23, 10), frac(12, 10)],
            [frac(-13, 10), frac(14, 10), frac(-15, 10), frac(19, 10)],
        ]);
        b.iter(|| numerator.clone().div_matrix_checked(divisor.clone()).unwrap())
    });
}

criterion_group!(benches, bench_regression_sentinels);
criterion_main!(benches);

