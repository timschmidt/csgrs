//! Complete repeated coordinate/derivative queries, including prepared graphs.
use hypercurve::{
    Aabb2, HomogeneousControl2, Point2, RationalBezier2,
    interval_ranges::{RationalBezierRangeEnclosure, prepare_rational_bezier_ranges},
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperreal::Real;
use hypersolve::expression_program::ExpressionBudget;
use std::{hint::black_box, time::Instant};
fn source(case: usize) -> RationalBezier2 {
    match case {
        0 => RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(1, 0),
                Point2::from_values(2, 1),
            ],
            vec![Real::one(); 3],
        )
        .unwrap(),
        1 => RationalBezier2::from_homogeneous_controls(vec![
            HomogeneousControl2::new(Real::from(0), Real::from(1), Real::from(1)),
            HomogeneousControl2::new(Real::from(1), Real::from(1), Real::from(2)),
        ])
        .unwrap(),
        _ => {
            let n = 8;
            RationalBezier2::try_new(
                (0..=n)
                    .map(|i| {
                        Point2::new(
                            (Real::from(i) / Real::from(n)).unwrap(),
                            Real::from(i32::from(i == n)),
                        )
                    })
                    .collect(),
                vec![Real::one(); n as usize + 1],
            )
            .unwrap()
        }
    }
}
fn complete(
    source: &RationalBezier2,
    shared: bool,
    graph: &mut ExpressionBudget,
    arithmetic: &mut DyadicBudget,
) -> Vec<RationalBezierRangeEnclosure> {
    let prepared = shared.then(|| prepare_rational_bezier_ranges(source, graph).unwrap());
    let mut reports = Vec::new();
    for i in 0..16 {
        let parameters = DyadicInterval::new(i.into(), (i + 1).into(), -4).unwrap();
        let report = match prepared.as_ref() {
            Some(p) => p.enclose(&parameters, -32, graph, arithmetic).unwrap(),
            None => prepare_rational_bezier_ranges(source, graph)
                .unwrap()
                .enclose(&parameters, -32, graph, arithmetic)
                .unwrap(),
        };
        assert_eq!(report.parameters(), &parameters);
        let projected = report.native_bounds(graph, arithmetic).unwrap();
        let query = Aabb2::new_unchecked(Point2::from_values(3, 3), Point2::from_values(4, 4));
        assert!(
            !projected
                .bounds()
                .expect("nonempty cell")
                .overlaps(&query)
                .unwrap()
        );
        reports.push(report);
    }
    reports
}
fn main() {
    let iterations: u32 = std::env::var("CURVE_RANGE_ITERATIONS")
        .map(|s| s.parse().unwrap())
        .unwrap_or(32);
    let samples: u32 = std::env::var("CURVE_RANGE_SAMPLES")
        .map(|s| s.parse().unwrap())
        .unwrap_or(30);
    for (case, name) in ["polynomial", "rational", "degree8"]
        .into_iter()
        .enumerate()
    {
        let source = source(case);
        let expected = complete(
            &source,
            false,
            &mut ExpressionBudget::new(1_000_000, 100_000_000),
            &mut DyadicBudget::new(16384, 1_000_000_000_000),
        );
        let check = complete(
            &source,
            true,
            &mut ExpressionBudget::new(1_000_000, 100_000_000),
            &mut DyadicBudget::new(16384, 1_000_000_000_000),
        );
        for (a, b) in expected.iter().zip(check.iter()) {
            assert_eq!(
                a.evidence().program().program().definition(),
                b.evidence().program().program().definition()
            );
            assert_eq!(a.evidence().arguments(), b.evidence().arguments());
            assert_eq!(a.evidence().values(), b.evidence().values());
        }
        for sample in 0..samples + 2 {
            for shared in if sample % 2 == 0 {
                [false, true]
            } else {
                [true, false]
            } {
                let start = Instant::now();
                for _ in 0..iterations {
                    let reports = complete(
                        black_box(&source),
                        shared,
                        &mut ExpressionBudget::new(1_000_000, 100_000_000),
                        &mut DyadicBudget::new(16384, 1_000_000_000_000),
                    );
                    for (r, e) in reports.iter().zip(&expected) {
                        assert_eq!(r.evidence().values(), e.evidence().values());
                    }
                    drop(black_box(reports));
                }
                if sample >= 2 {
                    println!(
                        "{name}/{} sample={} iterations={iterations} total_ns={}",
                        if shared { "shared" } else { "independent" },
                        sample - 2,
                        start.elapsed().as_nanos()
                    );
                }
            }
        }
    }
}
