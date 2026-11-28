//! Criterion comparison of owned and borrowed exact arithmetic on fixed operands.
//!
//! Three groups exercise rational, rational-backed `Real`, and symbolic square-root
//! operands. Owned cases clone in `iter_batched` setup, outside the measured
//! interval, and drop collected outputs afterward. Borrowed `iter` cases include
//! output destruction in the measured loop. Batching/collection overhead also
//! differs, so these are not identical complete-lifetime workloads. Operands are
//! reused and may share warmed internal evidence; no cold-cache guarantee is made.
//!
//! Results are black-boxed without an independent equality, evaluation or certainty
//! assertion. These timings alone establish neither general correctness nor a
//! resource/completion envelope. Criterion settings control sampling.
//!
//! Criterion 0.5.1 generates `benches`, which configures a default runner from CLI
//! arguments and calls the three groups then the report callback, and `main`, which
//! runs that group and requests the final summary. Those macro-generated functions
//! have no attached item documentation; their remaining coverage gap is recorded
//! in the repository Rustdoc audit.

use criterion::{BatchSize, Criterion, black_box, criterion_group, criterion_main};
use hyperreal::{Rational, Real};

/// Authored per-binary catalogue writer; reads existing estimates and logs write failures.
#[path = "support/bench_docs.rs"]
mod bench_docs;
/// Shared report aggregator invoked after the per-binary catalogue callback.
#[path = "support/benchmark_report.rs"]
mod benchmark_report;

use bench_docs::{BenchDoc, BenchGroupDoc};

/// Eight shared catalogue rows for add/subtract/multiply/divide in two ownership forms.
///
/// The owned descriptions refer to cloned inputs, but cloning occurs outside the
/// measured interval. These strings do not register or validate benchmark cases.
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

/// Catalogue order and labels for the three eight-case arithmetic groups.
///
/// Each group reuses the same row metadata; labels do not assert equivalent total
/// work across ownership forms or scalar representations.
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

/// Construct a fixed exact fraction for benchmark setup.
///
/// Panics when `d` is zero because construction is unwrapped. Current callers use
/// nonzero denominators; setup is outside the measured arithmetic loops.
fn rational(n: i64, d: u64) -> Rational {
    Rational::fraction(n, d).unwrap()
}

/// Wrap the setup fraction as a rational-backed `Real` without float conversion.
///
/// Inherits the zero-denominator panic of [`rational`]. This does not evaluate a
/// computable approximation or supply a separate certainty receipt.
fn real(n: i64, d: u64) -> Real {
    Real::new(rational(n, d))
}

/// Register eight rational operations on 123456789/987654321 and its reciprocal.
///
/// Refreshes catalogue text from stored estimates before registering the group.
/// Both operands are nonzero, so the fixed division input is valid. Owned setup
/// clones and deferred output drops are outside measurement; borrowed output drops
/// are inside it. Results are consumed by `black_box`, not checked for equality.
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

/// Register the same eight ownership cases with rational-backed `Real` operands.
///
/// The reciprocal setup fractions are constructed before timing. Division results
/// are unwrapped inside the timed routine and would panic on failure. The fixed
/// nonzero operands satisfy division's domain; no result evaluation or independent
/// correctness/receipt assertion is added. Timing boundaries follow the module overview.
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

/// Register eight ownership cases using the positive square roots of two and three.
///
/// Square-root construction and its unwraps occur before timing. Division is
/// unwrapped within the timed routine. Shared operands can retain internal work
/// across iterations; returned exact expressions are black-boxed without explicitly
/// requesting numerical evaluation or checking their mathematical values.
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

/// Refresh this binary's catalogue, then invoke the shared report callback.
///
/// Called after all three groups by the generated Criterion runner. Each writer
/// applies its own argument/environment guard and logs write errors; report work
/// is not part of the measured arithmetic closures. The catalogue reads whatever
/// estimate files currently exist, not a receipt of this run's completion.
fn finish_benchmark_report(c: &mut Criterion) {
    bench_docs::write_benchmark_docs(
        "borrowed_ops",
        "Compares owned arithmetic with borrowed arithmetic for exact and irrational values.",
        BORROWED_OP_GROUPS,
    );
    benchmark_report::finish_benchmark_report(c);
}

criterion_group!(
    benches,
    bench_rational,
    bench_real,
    bench_real_irrational,
    finish_benchmark_report
);
criterion_main!(benches);
