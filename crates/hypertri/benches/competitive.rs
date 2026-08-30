//! Competitive runtime-shaped benchmarks.
//!
//! These rows intentionally separate Hypertri's exact pre-lifted API from its
//! `f64` boundary adapter. Earcutr and Delaunator consume ordinary floating
//! inputs and are comparison engines, not correctness oracles for general
//! `Real` coordinates.

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use delaunator::Point;
use hypertri::{Point2, PredicatePolicy, Real, TriangulationContext};
use std::hint::black_box;
use std::time::Duration;

const APPROX: TriangulationContext = TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);

fn lift(points: &[[f64; 2]]) -> Vec<Point2> {
    points
        .iter()
        .map(|point| {
            Point2::new(
                Real::try_from(point[0]).expect("finite x"),
                Real::try_from(point[1]).expect("finite y"),
            )
        })
        .collect()
}

fn regular_polygon(count: usize) -> Vec<[f64; 2]> {
    (0..count)
        .map(|index| {
            let angle = std::f64::consts::TAU * index as f64 / count as f64;
            [1000.0 * angle.cos(), 1000.0 * angle.sin()]
        })
        .collect()
}

fn scattered_points(count: usize) -> Vec<[f64; 2]> {
    let width = (count as f64).sqrt().ceil() as usize;
    (0..count)
        .map(|index| {
            [
                ((index % width) * 100 + (index * 17) % 31) as f64,
                ((index / width) * 100 + (index * 29) % 37) as f64,
            ]
        })
        .collect()
}

fn bench_earcut_competitors(c: &mut Criterion) {
    let mut group = c.benchmark_group("competitive/earcut");
    group
        .sample_size(20)
        .warm_up_time(Duration::from_millis(250))
        .measurement_time(Duration::from_secs(1));

    for count in [32, 128] {
        let points = regular_polygon(count);
        let exact = lift(&points);
        let flat = points.iter().flat_map(|point| *point).collect::<Vec<_>>();
        group.throughput(Throughput::Elements(count as u64));
        group.bench_with_input(
            BenchmarkId::new("hypertri_exact_prelifted", count),
            &count,
            |b, _| b.iter(|| hypertri::earcut(&APPROX, black_box(&exact), &[]).unwrap()),
        );
        group.bench_with_input(
            BenchmarkId::new("hypertri_f64_boundary", count),
            &count,
            |b, _| b.iter(|| hypertri::f64::earcut(&APPROX, black_box(&points), &[]).unwrap()),
        );
        group.bench_with_input(BenchmarkId::new("earcutr_f64", count), &count, |b, _| {
            b.iter(|| earcutr::earcut(black_box(&flat), &[], 2).unwrap())
        });
    }
    group.finish();
}

fn bench_delaunay_competitors(c: &mut Criterion) {
    let mut group = c.benchmark_group("competitive/delaunay");
    group
        .sample_size(10)
        .warm_up_time(Duration::from_millis(250))
        .measurement_time(Duration::from_secs(1));

    for count in [64, 400] {
        let points = scattered_points(count);
        let exact = lift(&points);
        let delaunator_points = points
            .iter()
            .map(|point| Point {
                x: point[0],
                y: point[1],
            })
            .collect::<Vec<_>>();
        group.throughput(Throughput::Elements(count as u64));
        group.bench_with_input(
            BenchmarkId::new("hypertri_exact_prelifted", count),
            &count,
            |b, _| b.iter(|| hypertri::cdt::delaunay(&APPROX, black_box(&exact)).unwrap()),
        );
        group.bench_with_input(
            BenchmarkId::new("hypertri_exact_spatial", count),
            &count,
            |b, _| b.iter(|| hypertri::cdt::delaunay_spatial(&APPROX, black_box(&exact)).unwrap()),
        );
        group.bench_with_input(
            BenchmarkId::new("hypertri_f64_boundary", count),
            &count,
            |b, _| b.iter(|| hypertri::f64::delaunay(&APPROX, black_box(&points)).unwrap()),
        );
        group.bench_with_input(BenchmarkId::new("delaunator_f64", count), &count, |b, _| {
            b.iter(|| delaunator::triangulate(black_box(&delaunator_points)))
        });
    }
    group.finish();
}

criterion_group!(
    benches,
    bench_earcut_competitors,
    bench_delaunay_competitors
);
criterion_main!(benches);
