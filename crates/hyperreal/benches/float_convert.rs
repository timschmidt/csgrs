use criterion::{Criterion, black_box, criterion_group, criterion_main};
use hyperreal::{Rational, Real};

fn bench_float_convert(c: &mut Criterion) {
    let mut group = c.benchmark_group("float_convert");

    group.bench_function("f32_normal", |b| {
        b.iter(|| black_box(Rational::try_from(black_box(1.23456789_f32)).unwrap()))
    });
    group.bench_function("f64_normal", |b| {
        b.iter(|| black_box(Rational::try_from(black_box(1.23456789_f64)).unwrap()))
    });
    group.bench_function("f64_binary_fraction", |b| {
        b.iter(|| black_box(Rational::try_from(black_box(0.75_f64)).unwrap()))
    });
    group.bench_function("f64_subnormal", |b| {
        b.iter(|| black_box(Rational::try_from(black_box(f64::from_bits(2))).unwrap()))
    });
    group.bench_function("real_f32_normal", |b| {
        b.iter(|| black_box(Real::try_from(black_box(1.23456789_f32)).unwrap()))
    });
    group.bench_function("real_f64_normal", |b| {
        b.iter(|| black_box(Real::try_from(black_box(1.23456789_f64)).unwrap()))
    });
    group.bench_function("real_f64_subnormal", |b| {
        b.iter(|| black_box(Real::try_from(black_box(f64::from_bits(2))).unwrap()))
    });

    group.finish();
}

criterion_group!(benches, bench_float_convert);
criterion_main!(benches);
