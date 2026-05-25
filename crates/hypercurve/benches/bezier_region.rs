use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BezierArrangementGraph2, BezierParameter2, BezierRegion2, BezierRetainedEndpointEnvelope2,
    BezierRetainedRegion2, Classification, CurvePolicy, CurveResult, Point2, QuadraticBezier2,
    Real,
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
    let half = decided(BezierParameter2::exact(q(1, 2), &policy)?);
    let upper = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let lower = QuadraticBezier2::new(p(4, 0), p(2, -4), p(0, 0));
    let graph = BezierArrangementGraph2::from_split_materializations(&[
        decided(upper.split_at_parameters(std::slice::from_ref(&half), &policy)?),
        decided(lower.split_at_parameters(std::slice::from_ref(&half), &policy)?),
    ]);
    let traversal = decided(graph.traverse_branch_free(&policy));

    let iterations = 20_000_u32;
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let region = decided(BezierRegion2::from_arrangement_traversal(
            &graph, &traversal,
        ));
        checksum ^= black_box(format!("{:?}", region.signed_area()?).len());
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_region_materialization: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );

    let retained_traversal = decided(graph.traverse_retained_with_tangent_order(&policy));
    let started = Instant::now();
    let mut retained_checksum = 0_usize;
    for _ in 0..iterations {
        let region = decided(BezierRetainedRegion2::from_retained_arrangement_traversal(
            &graph,
            &retained_traversal,
        ));
        retained_checksum ^= black_box(format!("{:?}", region.signed_area()?).len());
        if let Classification::Decided(envelope) =
            BezierRetainedEndpointEnvelope2::from_region(&region, &policy)
        {
            retained_checksum ^= black_box(format!("{:?}", envelope.envelope()).len());
        }
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_retained_region_materialization: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={retained_checksum}",
        elapsed / iterations
    );

    Ok(())
}
