use criterion::{Criterion, black_box, criterion_group, criterion_main};
use hyperreal::{Rational, Real};

#[path = "support/bench_docs.rs"]
mod bench_docs;

use bench_docs::{BenchDoc, BenchGroupDoc};

const FLOAT_CONVERT_GROUPS: &[BenchGroupDoc] = &[BenchGroupDoc {
    name: "float_convert",
    description: "Exact conversion from IEEE-754 floats into `Rational` and `Real` values.",
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
            name: "real_f64_subnormal",
            description: "Converts a subnormal `f64` through the public `Real::try_from` path.",
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
