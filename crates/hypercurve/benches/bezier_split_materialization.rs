use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BezierParameter2, Classification, CubicBezier2, CurvePolicy, CurveResult, Point2, Real,
};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("benchmark unexpectedly uncertain: {reason:?}"),
    }
}

fn main() -> CurveResult<()> {
    let policy = CurvePolicy::certified();
    let curve = CubicBezier2::new(p(0, 0), p(2, 6), p(6, -2), p(8, 0));
    let parameters = [
        decided(BezierParameter2::exact(q(1, 4), &policy)?),
        decided(BezierParameter2::exact(q(1, 2), &policy)?),
        decided(BezierParameter2::exact(q(3, 4), &policy)?),
    ];

    let iterations = 25_000_u32;
    let started = Instant::now();
    let mut total = 0_usize;
    for _ in 0..iterations {
        let materialization = decided(curve.split_at_parameters(&parameters, &policy)?);
        total += black_box(materialization.fragments().len());
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_split_materialization_cubic: {iterations} iterations in {elapsed:?} ({:?}/iter), total={total}",
        elapsed / iterations
    );

    Ok(())
}
