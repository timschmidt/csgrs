use criterion::{BatchSize, Criterion, black_box, criterion_group, criterion_main};
use realistic::{Computable, Rational};

fn bench_computable_cache(c: &mut Criterion) {
    let mut group = c.benchmark_group("computable_cache");
    let ratio = Computable::rational(Rational::fraction(355, 113).unwrap());
    let pi = Computable::pi();

    group.bench_function("ratio_approx_cold_p128", |b| {
        b.iter_batched(
            || ratio.clone(),
            |value| black_box(value.approx(-128)),
            BatchSize::SmallInput,
        )
    });
    ratio.approx(-128);
    group.bench_function("ratio_approx_cached_p128", |b| {
        b.iter(|| black_box(ratio.approx(-128)))
    });

    group.bench_function("pi_approx_cold_p128", |b| {
        b.iter_batched(
            || pi.clone(),
            |value| black_box(value.approx(-128)),
            BatchSize::SmallInput,
        )
    });
    pi.approx(-128);
    group.bench_function("pi_approx_cached_p128", |b| {
        b.iter(|| black_box(pi.approx(-128)))
    });

    group.finish();
}

fn bench_computable_transcendentals(c: &mut Criterion) {
    let mut group = c.benchmark_group("computable_transcendentals");
    let p = -128;
    let trig_p = -96;

    let exp_input = Computable::rational(Rational::fraction(7, 5).unwrap());
    group.bench_function("exp_cold_p128", |b| {
        b.iter_batched(
            || exp_input.clone().exp(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let exp_cached = exp_input.clone().exp();
    exp_cached.approx(p);
    group.bench_function("exp_cached_p128", |b| {
        b.iter(|| black_box(exp_cached.approx(p)))
    });

    let ln_input = Computable::rational(Rational::fraction(11, 7).unwrap());
    group.bench_function("ln_cold_p128", |b| {
        b.iter_batched(
            || ln_input.clone().ln(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let ln_cached = ln_input.clone().ln();
    ln_cached.approx(p);
    group.bench_function("ln_cached_p128", |b| {
        b.iter(|| black_box(ln_cached.approx(p)))
    });

    let ln_large_input = Computable::rational(Rational::new(1024));
    group.bench_function("ln_large_cold_p128", |b| {
        b.iter_batched(
            || ln_large_input.clone().ln(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let ln_large_cached = ln_large_input.clone().ln();
    ln_large_cached.approx(p);
    group.bench_function("ln_large_cached_p128", |b| {
        b.iter(|| black_box(ln_large_cached.approx(p)))
    });

    let sqrt_input = Computable::rational(Rational::new(2));
    group.bench_function("sqrt_cold_p128", |b| {
        b.iter_batched(
            || sqrt_input.clone().sqrt(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let sqrt_cached = sqrt_input.clone().sqrt();
    sqrt_cached.approx(p);
    group.bench_function("sqrt_cached_p128", |b| {
        b.iter(|| black_box(sqrt_cached.approx(p)))
    });

    let trig_input = Computable::rational(Rational::fraction(7, 5).unwrap());
    group.bench_function("sin_cold_p96", |b| {
        b.iter_batched(
            || trig_input.clone().sin(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    let sin_cached = trig_input.clone().sin();
    sin_cached.approx(trig_p);
    group.bench_function("sin_cached_p96", |b| {
        b.iter(|| black_box(sin_cached.approx(trig_p)))
    });

    group.bench_function("cos_cold_p96", |b| {
        b.iter_batched(
            || trig_input.clone().cos(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    let cos_cached = trig_input.clone().cos();
    cos_cached.approx(trig_p);
    group.bench_function("cos_cached_p96", |b| {
        b.iter(|| black_box(cos_cached.approx(trig_p)))
    });

    group.bench_function("tan_cold_p96", |b| {
        b.iter_batched(
            || trig_input.clone().tan(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    let tan_cached = trig_input.clone().tan();
    tan_cached.approx(trig_p);
    group.bench_function("tan_cached_p96", |b| {
        b.iter(|| black_box(tan_cached.approx(trig_p)))
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_computable_cache,
    bench_computable_transcendentals
);
criterion_main!(benches);
