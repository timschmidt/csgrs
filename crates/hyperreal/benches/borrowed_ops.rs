use criterion::{BatchSize, Criterion, black_box, criterion_group, criterion_main};
use hyperreal::{Rational, Real};

#[path = "support/bench_docs.rs"]
mod bench_docs;

use bench_docs::{BenchDoc, BenchGroupDoc};

const OWNED_REF_BENCHES: &[BenchDoc] = &[
    BenchDoc {
        name: "add_owned",
        description: "Adds cloned owned operands.",
    },
    BenchDoc {
        name: "add_refs",
        description: "Adds borrowed operands without cloning both inputs.",
    },
    BenchDoc {
        name: "sub_owned",
        description: "Subtracts cloned owned operands.",
    },
    BenchDoc {
        name: "sub_refs",
        description: "Subtracts borrowed operands.",
    },
    BenchDoc {
        name: "mul_owned",
        description: "Multiplies cloned owned operands.",
    },
    BenchDoc {
        name: "mul_refs",
        description: "Multiplies borrowed operands.",
    },
    BenchDoc {
        name: "div_owned",
        description: "Divides cloned owned operands.",
    },
    BenchDoc {
        name: "div_refs",
        description: "Divides borrowed operands.",
    },
];

const BORROWED_OP_GROUPS: &[BenchGroupDoc] = &[
    BenchGroupDoc {
        name: "rational_ops",
        description: "Owned versus borrowed arithmetic for exact `Rational` values.",
        benches: OWNED_REF_BENCHES,
    },
    BenchGroupDoc {
        name: "real_ops",
        description: "Owned versus borrowed arithmetic for exact rational-backed `Real` values.",
        benches: OWNED_REF_BENCHES,
    },
    BenchGroupDoc {
        name: "real_irrational_ops",
        description: "Owned versus borrowed arithmetic for symbolic irrational `Real` values.",
        benches: OWNED_REF_BENCHES,
    },
];

fn rational(n: i64, d: u64) -> Rational {
    Rational::fraction(n, d).unwrap()
}

fn real(n: i64, d: u64) -> Real {
    Real::new(rational(n, d))
}

fn bench_rational(c: &mut Criterion) {
    bench_docs::write_benchmark_docs(
        "borrowed_ops",
        "Compares owned arithmetic with borrowed arithmetic for exact and irrational values.",
        BORROWED_OP_GROUPS,
    );

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
