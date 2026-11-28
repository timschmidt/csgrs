//! Criterion cases for exact float import and outward binary64 rational export.
//!
//! Eight cases import finite normal, binary-fraction and subnormal values through
//! `Rational` or `Real`. Three export cases use a singleton, an outward-rounded
//! dyadic and a value just below the largest finite binary64 number. Imports retain
//! the exact binary float value, not the decimal spelling's ideal rational value.
//!
//! `iter` includes returned-value destruction; setup for export operands is outside
//! the measured loop. Imports unwrap construction, while exports black-box the
//! returned `Option` without checking success or endpoint values. These fixed
//! workloads are timing probes, not a complete correctness or resource contract.
//!
//! Criterion 0.5.1 generates `benches`, which configures the default runner from CLI
//! arguments and calls the conversion group then its report callback, and `main`,
//! which invokes that group and the final summary. Those generated functions lack
//! attached item docs; the repository Rustdoc audit retains that coverage gap.

use criterion::{Criterion, black_box, criterion_group, criterion_main};
use hyperreal::{Rational, Real};

/// Authored per-binary catalogue writer; reads existing estimates and logs write failures.
#[path = "support/bench_docs.rs"]
mod bench_docs;
/// Shared report aggregator invoked after the per-binary catalogue callback.
#[path = "support/benchmark_report.rs"]
mod benchmark_report;

use bench_docs::{BenchDoc, BenchGroupDoc};

/// Ordered catalogue metadata for eight imports and three enclosure exports.
///
/// The descriptions state each workload's intent; they neither register the cases
/// nor check enclosure success or exact endpoint expectations.
const FLOAT_CONVERT_GROUPS: &[BenchGroupDoc] = &[BenchGroupDoc {
    name: "float_convert",
    description: "Exact IEEE-754 imports and finite outward binary64 enclosures of rationals.",
    benches: &[
        BenchDoc {
            name: "f32_normal",
            description: "Converts a normal `f32` into an exact `Rational`.",
        },
        BenchDoc {
            name: "f64_normal",
            description: "Converts a normal `f64` into an exact `Rational`.",
        },
        BenchDoc {
            name: "f64_binary_fraction",
            description: "Converts an exactly representable binary `f64` fraction into `Rational`.",
        },
        BenchDoc {
            name: "f64_subnormal",
            description: "Converts a subnormal `f64` into an exact `Rational`.",
        },
        BenchDoc {
            name: "real_f32_normal",
            description: "Converts a normal `f32` through the public `Real::try_from` path.",
        },
        BenchDoc {
            name: "real_f64_normal",
            description: "Converts a normal `f64` through the public `Real::try_from` path.",
        },
        BenchDoc {
            name: "real_f64_binary_fraction",
            description: "Converts an exactly representable binary `f64` fraction through the public `Real::try_from` path.",
        },
        BenchDoc {
            name: "real_f64_subnormal",
            description: "Converts a subnormal `f64` through the public `Real::try_from` path.",
        },
        BenchDoc {
            name: "f64_enclosure_exact",
            description: "Exports an exactly representable dyadic as a finite binary64 singleton.",
        },
        BenchDoc {
            name: "f64_enclosure_rounded",
            description: "Exports a dyadic requiring outward binary64 rounding.",
        },
        BenchDoc {
            name: "f64_enclosure_near_max",
            description: "Exports an exact value just below binary64 MAX without an infinite endpoint.",
        },
    ],
}];

/// Register the eleven fixed float-conversion cases and refresh catalogue text.
///
/// For each of `Rational` and `Real`, import the exact binary values of
/// `1.234_567_9_f32`, `1.23456789_f64`, `0.75_f64`, and `f64::from_bits(2)` (2^-1073).
/// Each conversion is unwrapped in the timed loop; the fixed finite inputs are valid.
///
/// Construct export operands before timing: 3/4, (2^54 + 1)/8 = 2^51 + 1/8, and
/// `f64::MAX` minus the exact integer one. Their outward binary64 enclosures are
/// respectively the 3/4 singleton, endpoints 2^51 - 1/4 and 2^51 + 1/2, and the predecessor
/// of `f64::MAX` paired with `f64::MAX`. The loop consumes the returned `Option`
/// without asserting these expectations; independent acceptance tests are required.
/// Report I/O precedes the group and is outside the timed conversion closures.
fn bench_float_convert(c: &mut Criterion) {
    bench_docs::write_benchmark_docs(
        "float_convert",
        "Covers exact import of floating-point values, including public `Real` conversion overhead.",
        FLOAT_CONVERT_GROUPS,
    );

    let mut group = c.benchmark_group("float_convert");

    group.bench_function("f32_normal", |b| {
        b.iter(|| black_box(Rational::try_from(black_box(1.234_567_9_f32)).unwrap()))
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
        b.iter(|| black_box(Real::try_from(black_box(1.234_567_9_f32)).unwrap()))
    });
    group.bench_function("real_f64_normal", |b| {
        b.iter(|| black_box(Real::try_from(black_box(1.23456789_f64)).unwrap()))
    });
    group.bench_function("real_f64_binary_fraction", |b| {
        b.iter(|| black_box(Real::try_from(black_box(0.75_f64)).unwrap()))
    });
    group.bench_function("real_f64_subnormal", |b| {
        b.iter(|| black_box(Real::try_from(black_box(f64::from_bits(2))).unwrap()))
    });

    let exact = Rational::fraction(3, 4).unwrap();
    let rounded = Rational::fraction((1_i64 << 54) + 1, 8).unwrap();
    let near_max = Rational::try_from(f64::MAX).unwrap() - Rational::one();
    for (name, value) in [
        ("f64_enclosure_exact", exact),
        ("f64_enclosure_rounded", rounded),
        ("f64_enclosure_near_max", near_max),
    ] {
        group.bench_function(name, |b| {
            b.iter(|| black_box(black_box(&value).to_f64_enclosure()))
        });
    }

    group.finish();
}

/// Refresh the conversion catalogue before invoking the shared report callback.
///
/// Runs after the conversion group, outside its timed closures. Both helpers apply
/// their own skip rules and log file-write errors instead of failing the benchmark.
/// Stored estimates can predate this invocation.
fn finish_benchmark_report(c: &mut Criterion) {
    bench_docs::write_benchmark_docs(
        "float_convert",
        "Covers exact import of floating-point values, including public `Real` conversion overhead.",
        FLOAT_CONVERT_GROUPS,
    );
    benchmark_report::finish_benchmark_report(c);
}

criterion_group!(benches, bench_float_convert, finish_benchmark_report);
criterion_main!(benches);
