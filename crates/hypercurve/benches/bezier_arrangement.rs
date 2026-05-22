use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BezierArrangementGraph2, BezierParameter2, Classification, CurvePolicy, CurveResult, Point2,
    QuadraticBezier2, Real,
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
    let split = [decided(BezierParameter2::exact(q(1, 2), &policy)?)];
    let mut materializations = Vec::new();
    for index in 0..64 {
        let curve = QuadraticBezier2::new(
            p(index * 2, 0),
            p(index * 2 + 1, if index % 2 == 0 { 2 } else { -2 }),
            p(index * 2 + 2, 0),
        );
        materializations.push(decided(curve.split_at_parameters(&split, &policy)?));
    }
    let graph = BezierArrangementGraph2::from_split_materializations(&materializations);

    let iterations = 5_000_u32;
    let started = Instant::now();
    let mut total = 0_usize;
    for _ in 0..iterations {
        let traversal = decided(graph.traverse_branch_free(&policy));
        total += black_box(traversal.len());
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_arrangement_branch_free: {iterations} iterations in {elapsed:?} ({:?}/iter), total={total}",
        elapsed / iterations
    );

    Ok(())
}
