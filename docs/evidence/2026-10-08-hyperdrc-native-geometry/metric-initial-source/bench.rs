//! Native metric cost probe, with exact oracles checked outside timed loops.
//!
//! `warm` reuses the same inputs after one query. `construct_and_query` includes
//! fresh fixture construction and the first query, including any scalar work
//! in its oracle construction. Neither row is a comparison with a previous
//! implementation. Set HYPERCURVE_METRIC_ITERATIONS, or use --test for one
//! iteration per fixture. Allocation counts are not measured by this harness.

use std::{hint::black_box, time::Instant};

use hypercurve::{CircularArc2, LineSeg2, Point2, Real, Segment2};

fn p(x: i32, y: i32) -> Point2 {
    Point2::from_values(x, y)
}

fn line(start: Point2, end: Point2) -> Segment2 {
    Segment2::Line(LineSeg2::try_new(start, end).unwrap())
}

fn arc(start: Point2, end: Point2, center: Point2) -> Segment2 {
    Segment2::Arc(CircularArc2::try_from_center(start, end, center, false).unwrap())
}

type Fixture = fn() -> (Segment2, Segment2, Real);

const FIXTURES: [(&str, Fixture); 6] = [
    ("line_line", || {
        (line(p(0, 0), p(2, 0)), line(p(3, 1), p(3, 3)), 2.into())
    }),
    ("line_arc", || {
        (
            line(p(-2, 3), p(2, 3)),
            arc(p(1, 0), p(-1, 0), p(0, 0)),
            4.into(),
        )
    }),
    ("arc_arc", || {
        (
            arc(p(0, -1), p(0, 1), p(0, 0)),
            arc(p(5, 2), p(5, -2), p(5, 0)),
            4.into(),
        )
    }),
    ("concentric", || {
        (
            arc(p(1, 0), p(-1, 0), p(0, 0)),
            arc(p(2, 0), p(-2, 0), p(0, 0)),
            Real::one(),
        )
    }),
    ("disjoint_sweeps", || {
        (
            arc(p(1, 0), p(0, 1), p(0, 0)),
            arc(p(-1, 0), p(0, -1), p(0, 0)),
            2.into(),
        )
    }),
    ("independent_radicals", || {
        let a = Real::from(2).sqrt().unwrap();
        let b = Real::from(3).sqrt().unwrap();
        let gap = Real::from(8) - &a - &b;
        (
            arc(
                Point2::new(Real::zero(), -a.clone()),
                Point2::new(Real::zero(), a),
                p(0, 0),
            ),
            arc(
                Point2::new(8.into(), b.clone()),
                Point2::new(8.into(), -b),
                p(8, 0),
            ),
            &gap * &gap,
        )
    }),
];

fn assert_oracle(actual: &Real, expected: &Real) {
    assert_eq!(
        hyperlimit::compare_reals(actual, expected, hyperlimit::PredicatePolicy::STRICT).value(),
        Some(std::cmp::Ordering::Equal),
    );
}

fn main() {
    let iterations = if std::env::args().any(|arg| arg == "--test") {
        1
    } else {
        std::env::var("HYPERCURVE_METRIC_ITERATIONS")
            .ok()
            .map(|value| {
                value
                    .parse::<usize>()
                    .expect("iterations must be a positive integer")
            })
            .unwrap_or(1_000)
    };
    assert!(iterations > 0);
    for (name, fixture) in FIXTURES {
        let (left, right, expected) = fixture();
        let mut last = left.distance_squared(&right).unwrap();
        assert_oracle(&last, &expected);
        let start = Instant::now();
        for _ in 0..iterations {
            last = black_box(
                black_box(&left)
                    .distance_squared(black_box(&right))
                    .unwrap(),
            );
        }
        let elapsed = start.elapsed().as_nanos();
        assert_oracle(&last, &expected);
        println!(
            "{{\"fixture\":\"{name}\",\"mode\":\"warm\",\"iterations\":{iterations},\"elapsed_ns\":{elapsed},\"oracle\":\"passed\"}}"
        );

        let start = Instant::now();
        for _ in 0..iterations {
            let (left, right, _) = black_box(fixture)();
            last = black_box(left.distance_squared(&right).unwrap());
        }
        let elapsed = start.elapsed().as_nanos();
        assert_oracle(&last, &expected);
        println!(
            "{{\"fixture\":\"{name}\",\"mode\":\"construct_and_query\",\"iterations\":{iterations},\"elapsed_ns\":{elapsed},\"oracle\":\"passed\"}}"
        );
    }
}
