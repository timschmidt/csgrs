#[path = "../tests/support/mod.rs"]
mod support;
use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    Curve2, CurveContext, CurvePath2, CurveRegion2, FillRule, LineSeg2, Point2, QuadraticBezier2,
    Real, RegionPointLocation,
};

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

/// The graph `y = k x^2` on `x` in `[0, 1]`, leaving the origin horizontally.
fn parabola(k: i32) -> QuadraticBezier2 {
    QuadraticBezier2::new(
        Point2::new(Real::zero(), Real::zero()),
        Point2::new(q(1, 2), Real::zero()),
        Point2::new(Real::one(), Real::from(k)),
    )
}

/// A lens whose two sides share the origin with one horizontal tangent, so
/// admission orders the branch vertex by curvature.
fn tangent_lens(a: i32, b: i32, policy: &CurveContext) -> CurvePath2 {
    let upper = Curve2::from(parabola(b))
        .reversed(policy)
        .expect("reversal is exact")
        .into_value();
    let closing = LineSeg2::try_new(
        Point2::new(Real::one(), Real::from(a)),
        Point2::new(Real::one(), Real::from(b)),
    )
    .expect("distinct curvatures give a nonzero closing segment");
    CurvePath2::try_new(vec![Curve2::from(parabola(a)), closing.into(), upper])
        .expect("the lens boundary is connected")
}

fn bench_lens(name: &str, a: i32, b: i32, iterations: u32) {
    let policy = CurveContext::STRICT;
    let path = tangent_lens(a, b, &policy);
    let inside = Point2::new(q(1, 2), q(a + b, 8)).into();
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let region = crate::support::under(&policy, || {
            CurveRegion2::try_from_boundary_paths(
                black_box(std::slice::from_ref(&path)),
                FillRule::EvenOdd,
            )
        })
        .expect("the tangent lens is admitted")
        .into_value();
        let location = crate::support::under(&policy, || region.classify_point(black_box(&inside)))
            .expect("classification completes")
            .value;
        assert_eq!(location, RegionPointLocation::Inside);
        checksum += black_box(region.len());
    }
    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
}

fn main() {
    bench_lens("bezier_tangent_order_second_order_lens", 1, 2, 200);
    bench_lens(
        "bezier_tangent_order_near_equal_curvature_lens",
        15,
        16,
        200,
    );
}
