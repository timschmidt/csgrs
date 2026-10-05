#[path = "../tests/support/mod.rs"]
mod support;
use std::hint::black_box;
use std::time::Instant;

use hypercurve::{Classification, Curve2, CurveContext, Point2, RationalBezier2, Real};

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

/// Resolves a rational curve against its exact tail: one certified partial
/// overlap whose shared span splits the full curve.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let policy = CurveContext::STRICT;
    let rational_curve =
        RationalBezier2::try_new(vec![p(0, 0), p(2, 2), p(4, 0)], vec![r(1), r(1), r(1)])?;
    let rational_tail = decided(crate::support::under_classified_result(&policy, || {
        rational_curve.subcurve_between_exact(&q(1, 2), &r(1))
    })?);
    let curve = Curve2::from(rational_curve);
    let tail = Curve2::from(rational_tail);

    let iterations = 250_u32;
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let topology =
            crate::support::under(&policy, || curve.intersection_topology(&tail))?.into_value();
        assert_eq!(topology.result().overlaps().len(), 1);
        checksum ^= black_box(
            topology.first().len() + topology.second().len() + topology.result().overlaps().len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_rational_overlap: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
    Ok(())
}
