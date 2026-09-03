use criterion::{Criterion, criterion_group, criterion_main};
use hypertri::{PredicatePolicy, TriangulationContext};

#[path = "support/benchmark_report.rs"]
mod benchmark_report;

const APPROX: TriangulationContext = TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);

fn bench_exact_lifted_earcut(c: &mut Criterion) {
    let concave = vec![
        [0.0, 0.0],
        [4.0, 0.0],
        [4.0, 1.0],
        [2.2, 1.0],
        [2.0, 2.0],
        [1.8, 1.0],
        [0.0, 1.0],
    ];

    c.bench_function("f64_exact_lifted_concave_earcut", |b| {
        b.iter(|| hypertri::f64::earcut(&APPROX, &concave, &[]).unwrap())
    });

    let holed = vec![
        [0.0, 0.0],
        [20.0, 0.0],
        [20.0, 20.0],
        [0.0, 20.0],
        [4.0, 4.0],
        [8.0, 4.0],
        [8.0, 8.0],
        [4.0, 8.0],
        [12.0, 12.0],
        [16.0, 12.0],
        [16.0, 16.0],
        [12.0, 16.0],
    ];

    c.bench_function("f64_exact_lifted_holed_earcut", |b| {
        b.iter(|| hypertri::f64::earcut(&APPROX, &holed, &[4, 8]).unwrap())
    });
}

criterion_group!(
    benches,
    bench_exact_lifted_earcut,
    benchmark_report::finish_benchmark_report
);
criterion_main!(benches);
