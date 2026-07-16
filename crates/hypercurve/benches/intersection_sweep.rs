use std::hint::black_box;
use std::time::Instant;

use hypercurve::{CurvePolicy, CurveString2, LineSeg2, Point2, Real, Segment2};

fn point(x: i32, y: i32) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn zigzag(segment_count: usize, y_offset: i32) -> CurveString2 {
    let points = (0..=segment_count)
        .map(|index| point(index as i32, y_offset + (index & 1) as i32))
        .collect::<Vec<_>>();
    CurveString2::try_new(
        points
            .windows(2)
            .map(|pair| {
                Segment2::Line(
                    LineSeg2::try_new(pair[0].clone(), pair[1].clone())
                        .expect("benchmark segments are nonzero"),
                )
            })
            .collect(),
    )
    .expect("benchmark path is connected")
}

fn zigzag_with_remote_tail(segment_count: usize, y_offset: i32) -> CurveString2 {
    let mut points = (0..=segment_count)
        .map(|index| point(index as i32, y_offset + (index & 1) as i32))
        .collect::<Vec<_>>();
    points.push(point(segment_count as i32 + 100, 0));
    points.push(point(segment_count as i32 + 101, 0));
    CurveString2::try_new(
        points
            .windows(2)
            .map(|pair| {
                Segment2::Line(
                    LineSeg2::try_new(pair[0].clone(), pair[1].clone())
                        .expect("benchmark segments are nonzero"),
                )
            })
            .collect(),
    )
    .expect("benchmark path is connected")
}

fn vertically_dense_zigzag(segment_count: usize, y_offset: i32) -> CurveString2 {
    let points = (0..=segment_count)
        .map(|index| point((index & 1) as i32, y_offset + index as i32))
        .collect::<Vec<_>>();
    CurveString2::try_new(
        points
            .windows(2)
            .map(|pair| {
                Segment2::Line(
                    LineSeg2::try_new(pair[0].clone(), pair[1].clone())
                        .expect("benchmark segments are nonzero"),
                )
            })
            .collect(),
    )
    .expect("benchmark path is connected")
}

fn vertically_dense_zigzag_with_remote_tail(segment_count: usize, y_offset: i32) -> CurveString2 {
    let mut points = (0..=segment_count)
        .map(|index| point((index & 1) as i32, y_offset + index as i32))
        .collect::<Vec<_>>();
    points.push(point(100, 0));
    points.push(point(101, 0));
    CurveString2::try_new(
        points
            .windows(2)
            .map(|pair| {
                Segment2::Line(
                    LineSeg2::try_new(pair[0].clone(), pair[1].clone())
                        .expect("benchmark segments are nonzero"),
                )
            })
            .collect(),
    )
    .expect("benchmark path is connected")
}

fn bench_direct(segment_count: usize, iterations: u32, policy: &CurvePolicy) {
    let first = zigzag(segment_count, 0);
    let second = zigzag_with_remote_tail(segment_count, 100);
    let expected_pairs = first.len() * second.len();
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let result = black_box(&first)
            .intersect_curve_string_with_report(black_box(&second), black_box(policy))
            .expect("separated exact paths should be decidable");
        assert_eq!(result.report().candidate_pair_count(), expected_pairs);
        assert_eq!(
            result.report().skipped_aabb_pair_count() + result.report().tested_pair_count(),
            expected_pairs
        );
        assert!(result.intersections().is_empty());
        checksum = checksum.wrapping_add(black_box(result.report().skipped_aabb_pair_count()));
    }
    let elapsed = started.elapsed();
    println!(
        "curve_string_x_sparse_direct_{segment_count}: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
}

fn bench_prepared(segment_count: usize, iterations: u32, policy: &CurvePolicy) {
    let first = zigzag(segment_count, 0);
    let second = zigzag_with_remote_tail(segment_count, 100);
    let expected_pairs = first.len() * second.len();
    let first = first.prepare_topology_queries(policy);
    let second = second.prepare_topology_queries(policy);
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let result = black_box(&first)
            .intersect_prepared_curve_string_with_report(black_box(&second), black_box(policy))
            .expect("separated prepared paths should be decidable");
        assert_eq!(result.report().candidate_pair_count(), expected_pairs);
        assert_eq!(
            result.report().skipped_aabb_pair_count() + result.report().tested_pair_count(),
            expected_pairs
        );
        assert!(result.intersections().is_empty());
        checksum = checksum.wrapping_add(black_box(result.report().skipped_aabb_pair_count()));
    }
    let elapsed = started.elapsed();
    println!(
        "curve_string_x_sparse_prepared_{segment_count}: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
}

fn bench_x_dense(segment_count: usize, iterations: u32, policy: &CurvePolicy) {
    let first = vertically_dense_zigzag(segment_count, 0);
    let second = vertically_dense_zigzag_with_remote_tail(segment_count, 10_000);
    let expected_pairs = first.len() * second.len();
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let result = black_box(&first)
            .intersect_curve_string_with_report(black_box(&second), black_box(policy))
            .expect("separated exact paths should be decidable");
        assert_eq!(result.report().candidate_pair_count(), expected_pairs);
        assert_eq!(
            result.report().skipped_aabb_pair_count() + result.report().tested_pair_count(),
            expected_pairs
        );
        assert!(result.intersections().is_empty());
        checksum = checksum.wrapping_add(black_box(result.report().skipped_aabb_pair_count()));
    }
    let elapsed = started.elapsed();
    println!(
        "curve_string_x_dense_direct_{segment_count}: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
}

fn main() {
    let policy = CurvePolicy::certified();
    for (segment_count, iterations) in [(32, 100), (64, 50), (128, 20), (512, 3)] {
        bench_direct(segment_count, iterations, &policy);
        bench_prepared(segment_count, iterations, &policy);
        bench_x_dense(segment_count, iterations, &policy);
    }
}
