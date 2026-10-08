use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BulgeVertex2, Contour2, CurveContext, CurveString2, LineSeg2, Point2, Real, Segment2,
};

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

fn diagonal_ribbon(rung_count: usize, y_offset: i32) -> Contour2 {
    let lower = (0..=rung_count).map(|index| point(index as i32, index as i32 + y_offset));
    let upper = (0..=rung_count)
        .rev()
        .map(|index| point(index as i32, index as i32 + y_offset + 1));
    Contour2::from_bulge_vertices(
        &lower
            .chain(upper)
            .map(|point| BulgeVertex2::new(point, Real::zero()))
            .collect::<Vec<_>>(),
    )
    .expect("diagonal ribbon is a closed simple contour")
}

fn bench_direct(segment_count: usize, iterations: u32, policy: &CurveContext) {
    let first = zigzag(segment_count, 0);
    let second = zigzag_with_remote_tail(segment_count, 100);
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let result = black_box(&first)
            .intersect_curve_string(black_box(&second), black_box(policy))
            .expect("separated exact paths should be decidable");
        assert!(result.is_empty());
        checksum = checksum.wrapping_add(black_box(result.len()));
    }
    let elapsed = started.elapsed();
    println!(
        "curve_string_x_sparse_direct_{segment_count}: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
}

fn bench_x_dense(segment_count: usize, iterations: u32, policy: &CurveContext) {
    let first = vertically_dense_zigzag(segment_count, 0);
    let second = vertically_dense_zigzag_with_remote_tail(segment_count, 10_000);
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let result = black_box(&first)
            .intersect_curve_string(black_box(&second), black_box(policy))
            .expect("separated exact paths should be decidable");
        assert!(result.is_empty());
        checksum = checksum.wrapping_add(black_box(result.len()));
    }
    let elapsed = started.elapsed();
    println!(
        "curve_string_x_dense_direct_{segment_count}: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
}

fn bench_sparse_contours(rung_count: usize, iterations: u32, policy: &CurveContext) {
    let first = diagonal_ribbon(rung_count, 0);
    let second = diagonal_ribbon(rung_count, rung_count as i32 / 2 + 2);
    let started = Instant::now();
    let mut checksum = 0;
    for _ in 0..iterations {
        let intersections = black_box(&first)
            .intersect_contour(black_box(&second), black_box(policy))
            .expect("separated exact contours should be decidable");
        assert!(intersections.is_empty());
        checksum += black_box(intersections.len());
    }
    let elapsed = started.elapsed();
    println!(
        "contour_interval_sparse_{rung_count}_rungs: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
}

fn main() {
    let policy = CurveContext::STRICT;
    for (segment_count, iterations) in [(32, 100), (64, 50), (128, 20), (512, 3)] {
        bench_direct(segment_count, iterations, &policy);
        bench_x_dense(segment_count, iterations, &policy);
    }
    for (segment_count, iterations) in [(64, 100), (128, 50), (512, 10)] {
        bench_sparse_contours(segment_count, iterations, &policy);
    }
}
