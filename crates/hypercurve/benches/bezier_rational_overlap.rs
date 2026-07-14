use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BezierArrangementFragment2, BezierArrangementGraph2, BezierParameter2, BezierSplitFragment2,
    BezierSubcurve2, Classification, CurvePolicy, CurveResult, Point2, QuadraticBezier2,
    RationalBezier2, Real,
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

fn line_fragment(
    source: usize,
    start: Point2,
    control: Point2,
    end: Point2,
) -> BezierArrangementFragment2 {
    BezierArrangementFragment2::new(
        source,
        0,
        BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(r(0)),
            end: BezierParameter2::Exact(r(1)),
            curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(start, control, end)),
        },
    )
}

fn main() -> CurveResult<()> {
    let policy = CurvePolicy::certified();
    let rational_curve =
        RationalBezier2::try_new(vec![p(0, 0), p(2, 2), p(4, 0)], vec![r(1), r(1), r(1)])?;
    let rational_tail = decided(rational_curve.subcurve_between_exact(&q(1, 2), &r(1), &policy)?);
    let graph = BezierArrangementGraph2::new(vec![
        BezierArrangementFragment2::new(
            0,
            0,
            BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(r(0)),
                end: BezierParameter2::Exact(r(1)),
                curve: BezierSubcurve2::Rational(rational_curve),
            },
        ),
        BezierArrangementFragment2::new(
            1,
            0,
            BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(r(0)),
                end: BezierParameter2::Exact(r(1)),
                curve: BezierSubcurve2::Rational(rational_tail),
            },
        ),
        line_fragment(2, p(4, 0), p(4, 1), p(4, 2)),
        line_fragment(3, p(4, 2), p(2, 2), p(0, 2)),
        line_fragment(4, p(0, 2), p(0, 1), p(0, 0)),
    ])?;

    let iterations = 250_u32;
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let traversal = decided(graph.traverse_retained_splitting_rational_overlaps(&policy));
        checksum ^= black_box(
            traversal.refinement().graph().len()
                + traversal.refinement().resolved_overlaps().len()
                + traversal.traversal().len()
                + traversal
                    .refined_traversal()
                    .shadowed_fragment_indices()
                    .len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_rational_overlap: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
    Ok(())
}
