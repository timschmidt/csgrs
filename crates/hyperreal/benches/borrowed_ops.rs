use criterion::{BatchSize, Criterion, black_box, criterion_group, criterion_main};
use realistic::{Rational, Real};

fn rational(n: i64, d: u64) -> Rational {
    Rational::fraction(n, d).unwrap()
}

fn real(n: i64, d: u64) -> Real {
    Real::new(rational(n, d))
}

fn bench_rational(c: &mut Criterion) {
    let mut group = c.benchmark_group("rational_ops");
    let lhs = rational(123_456_789, 987_654_321);
    let rhs = rational(987_654_321, 123_456_789);

    group.bench_function("add_owned", |b| {
        b.iter_batched(
            || (lhs.clone(), rhs.clone()),
            |(lhs, rhs)| black_box(lhs + rhs),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("add_refs", |b| {
        b.iter(|| black_box(black_box(&lhs) + black_box(&rhs)))
    });

    group.bench_function("sub_owned", |b| {
        b.iter_batched(
            || (lhs.clone(), rhs.clone()),
            |(lhs, rhs)| black_box(lhs - rhs),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("sub_refs", |b| {
        b.iter(|| black_box(black_box(&lhs) - black_box(&rhs)))
    });

    group.bench_function("mul_owned", |b| {
        b.iter_batched(
            || (lhs.clone(), rhs.clone()),
            |(lhs, rhs)| black_box(lhs * rhs),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("mul_refs", |b| {
        b.iter(|| black_box(black_box(&lhs) * black_box(&rhs)))
    });

    group.bench_function("div_owned", |b| {
        b.iter_batched(
            || (lhs.clone(), rhs.clone()),
            |(lhs, rhs)| black_box(lhs / rhs),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("div_refs", |b| {
        b.iter(|| black_box(black_box(&lhs) / black_box(&rhs)))
    });

    group.finish();
}

fn bench_real(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_ops");
    let lhs = real(123_456_789, 987_654_321);
    let rhs = real(987_654_321, 123_456_789);

    group.bench_function("add_owned", |b| {
        b.iter_batched(
            || (lhs.clone(), rhs.clone()),
            |(lhs, rhs)| black_box(lhs + rhs),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("add_refs", |b| {
        b.iter(|| black_box(black_box(&lhs) + black_box(&rhs)))
    });

    group.bench_function("sub_owned", |b| {
        b.iter_batched(
            || (lhs.clone(), rhs.clone()),
            |(lhs, rhs)| black_box(lhs - rhs),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("sub_refs", |b| {
        b.iter(|| black_box(black_box(&lhs) - black_box(&rhs)))
    });

    group.bench_function("mul_owned", |b| {
        b.iter_batched(
            || (lhs.clone(), rhs.clone()),
            |(lhs, rhs)| black_box(lhs * rhs),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("mul_refs", |b| {
        b.iter(|| black_box(black_box(&lhs) * black_box(&rhs)))
    });

    group.bench_function("div_owned", |b| {
        b.iter_batched(
            || (lhs.clone(), rhs.clone()),
            |(lhs, rhs)| black_box((lhs / rhs).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("div_refs", |b| {
        b.iter(|| black_box((black_box(&lhs) / black_box(&rhs)).unwrap()))
    });

    group.finish();
}

fn bench_real_irrational(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_irrational_ops");
    let lhs = Real::new(Rational::new(2)).sqrt().unwrap();
    let rhs = Real::new(Rational::new(3)).sqrt().unwrap();

    group.bench_function("add_owned", |b| {
        b.iter_batched(
            || (lhs.clone(), rhs.clone()),
            |(lhs, rhs)| black_box(lhs + rhs),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("add_refs", |b| {
        b.iter(|| black_box(black_box(&lhs) + black_box(&rhs)))
    });

    group.bench_function("sub_owned", |b| {
        b.iter_batched(
            || (lhs.clone(), rhs.clone()),
            |(lhs, rhs)| black_box(lhs - rhs),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("sub_refs", |b| {
        b.iter(|| black_box(black_box(&lhs) - black_box(&rhs)))
    });

    group.bench_function("mul_owned", |b| {
        b.iter_batched(
            || (lhs.clone(), rhs.clone()),
            |(lhs, rhs)| black_box(lhs * rhs),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("mul_refs", |b| {
        b.iter(|| black_box(black_box(&lhs) * black_box(&rhs)))
    });

    group.bench_function("div_owned", |b| {
        b.iter_batched(
            || (lhs.clone(), rhs.clone()),
            |(lhs, rhs)| black_box((lhs / rhs).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("div_refs", |b| {
        b.iter(|| black_box((black_box(&lhs) / black_box(&rhs)).unwrap()))
    });

    group.finish();
}

criterion_group!(benches, bench_rational, bench_real, bench_real_irrational);
criterion_main!(benches);
