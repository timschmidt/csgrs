use criterion::{Criterion, black_box, criterion_group, criterion_main};
use hyperreal::{Rational, Real};

#[path = "support/bench_docs.rs"]
mod bench_docs;
#[path = "support/benchmark_report.rs"]
mod benchmark_report;

use bench_docs::{BenchDoc, BenchGroupDoc};

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
