use criterion::{BatchSize, Criterion, black_box, criterion_group, criterion_main};
use num::bigint::BigInt;
use hyperreal::{Rational, Real, Simple};

fn bench_real_format(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_format");
    let pi = Real::pi();
    let sqrt_two = Real::new(Rational::new(2)).sqrt().unwrap();

    group.bench_function("pi_lower_exp_32", |b| {
        b.iter(|| black_box(format!("{:.32e}", black_box(&pi))))
    });
    group.bench_function("pi_display_alt_32", |b| {
        b.iter(|| black_box(format!("{:#.32}", black_box(&pi))))
    });
    group.bench_function("sqrt_two_display_alt_32", |b| {
        b.iter(|| black_box(format!("{:#.32}", black_box(&sqrt_two))))
    });

    group.finish();
}

fn bench_real_constants(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_constants");

    group.bench_function("pi", |b| b.iter(|| black_box(Real::pi())));
    group.bench_function("e", |b| b.iter(|| black_box(Real::e())));

    group.finish();
}

fn bench_simple(c: &mut Criterion) {
    let mut group = c.benchmark_group("simple");
    let source = "(* (+ pi pi) (pow (+ 3/2 4/7) 9/2) (sin (/ 1 5)))";
    let parsed: Simple = source.parse().unwrap();
    let constants_source = "(+ pi e pi e pi e pi e)";
    let constants_parsed: Simple = constants_source.parse().unwrap();
    let exact_source = "(/ (* (+ 7/5 11/7 13/9) (- 19/11 1/3)) 23/17)";
    let exact_parsed: Simple = exact_source.parse().unwrap();
    let nested_exact_source = "(* (+ (/ 7 5) (/ 11 7) (/ 13 9)) (pow (+ 5/4 3/8) 9) (/ (- 19/11 1/3) (+ 1/7 2/7)))";
    let nested_exact_parsed: Simple = nested_exact_source.parse().unwrap();

    group.bench_function("parse_nested", |b| {
        b.iter(|| black_box(black_box(source).parse::<Simple>().unwrap()))
    });
    group.bench_function("eval_nested", |b| {
        b.iter_batched(
            || parsed.clone(),
            |expr| black_box(expr.evaluate(&Default::default()).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("eval_constants", |b| {
        b.iter_batched(
            || constants_parsed.clone(),
            |expr| black_box(expr.evaluate(&Default::default()).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("eval_exact", |b| {
        b.iter_batched(
            || exact_parsed.clone(),
            |expr| black_box(expr.evaluate(&Default::default()).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("eval_nested_exact", |b| {
        b.iter_batched(
            || nested_exact_parsed.clone(),
            |expr| black_box(expr.evaluate(&Default::default()).unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

fn bench_real_powi(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_powi");
    let exact = Real::new(Rational::fraction(7, 5).unwrap());
    let irrational = Real::new(Rational::new(3)).sqrt().unwrap();
    let exp = BigInt::from(17_u8);

    group.bench_function("exact_17", |b| {
        b.iter_batched(
            || exact.clone(),
            |value| black_box(value.powi(exp.clone()).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("irrational_17", |b| {
        b.iter_batched(
            || irrational.clone(),
            |value| black_box(value.powi(exp.clone()).unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

fn bench_rational_powi(c: &mut Criterion) {
    let mut group = c.benchmark_group("rational_powi");
    let value = Rational::fraction(7, 5).unwrap();
    let exp = BigInt::from(17_u8);

    group.bench_function("exact_17", |b| {
        b.iter_batched(
            || value.clone(),
            |value| black_box(value.powi(exp.clone()).unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

fn bench_real_exact_trig(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_exact_trig");
    let pi_sixth = Real::pi() * Real::new(Rational::fraction(1, 6).unwrap());
    let pi_third = Real::pi() * Real::new(Rational::fraction(1, 3).unwrap());
    let pi_fifth = Real::pi() * Real::new(Rational::fraction(1, 5).unwrap());

    group.bench_function("sin_pi_6", |b| {
        b.iter_batched(
            || pi_sixth.clone(),
            |value| black_box(value.sin()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("cos_pi_3", |b| {
        b.iter_batched(
            || pi_third.clone(),
            |value| black_box(value.cos()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("tan_pi_5", |b| {
        b.iter_batched(
            || pi_fifth.clone(),
            |value| black_box(value.tan().unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

fn bench_real_general_trig(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_general_trig");
    let sqrt_two = Real::new(Rational::new(2)).sqrt().unwrap();
    let pi = Real::pi();
    let irrational_pi_mix = (pi * sqrt_two.clone()) / Real::new(Rational::new(5));
    let irrational_pi_mix = irrational_pi_mix.unwrap();

    group.bench_function("tan_sqrt_2", |b| {
        b.iter_batched(
            || sqrt_two.clone(),
            |value| black_box(value.tan().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("tan_pi_sqrt_2_over_5", |b| {
        b.iter_batched(
            || irrational_pi_mix.clone(),
            |value| black_box(value.tan().unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

fn bench_real_exact_ln(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_exact_ln");
    let ln_1024 = Real::new(Rational::new(1024));
    let ln_eighth = Real::new(Rational::fraction(1, 8).unwrap());
    let ln_1000 = Real::new(Rational::new(1000));

    group.bench_function("ln_1024", |b| {
        b.iter_batched(
            || ln_1024.clone(),
            |value| black_box(value.ln().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("ln_1_8", |b| {
        b.iter_batched(
            || ln_eighth.clone(),
            |value| black_box(value.ln().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("ln_1000", |b| {
        b.iter_batched(
            || ln_1000.clone(),
            |value| black_box(value.ln().unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

fn bench_real_exact_exp_log10(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_exact_exp_log10");
    let exp_ln_1000 = Real::new(Rational::new(1000)).ln().unwrap();
    let exp_ln_eighth = Real::new(Rational::fraction(1, 8).unwrap()).ln().unwrap();
    let log10_1000 = Real::new(Rational::new(1000));
    let log10_milli = Real::new(Rational::fraction(1, 1000).unwrap());

    group.bench_function("exp_ln_1000", |b| {
        b.iter_batched(
            || exp_ln_1000.clone(),
            |value| black_box(value.exp().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("exp_ln_1_8", |b| {
        b.iter_batched(
            || exp_ln_eighth.clone(),
            |value| black_box(value.exp().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("log10_1000", |b| {
        b.iter_batched(
            || log10_1000.clone(),
            |value| black_box(value.log10().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("log10_1_1000", |b| {
        b.iter_batched(
            || log10_milli.clone(),
            |value| black_box(value.log10().unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_real_format,
    bench_real_constants,
    bench_simple,
    bench_real_powi,
    bench_rational_powi,
    bench_real_exact_trig,
    bench_real_general_trig,
    bench_real_exact_ln,
    bench_real_exact_exp_log10
);
criterion_main!(benches);
