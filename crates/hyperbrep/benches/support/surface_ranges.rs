//! Shared complete workload for timing and external allocation observation.
use hyperbrep::{
    Point3, Surface,
    interval_ranges::{SurfaceRangeEnclosure, prepare_rational_bezier_surface_ranges},
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice::BoxN;
use hyperlimit::{Certainty, PredicateOutcome, PredicatePolicy, ordered_aabb3s_intersect};
use hyperreal::{Rational, Real};
use hypersolve::expression_program::ExpressionBudget;

pub const GRAPH_WORK: usize = 1_000_000_000;
pub const INTERVAL_WORK: u64 = 1_000_000_000_000;
pub fn graph() -> ExpressionBudget {
    ExpressionBudget::new(1_000_000, GRAPH_WORK)
}
pub fn arithmetic() -> DyadicBudget {
    DyadicBudget::new(16384, INTERVAL_WORK)
}

pub fn source(case: usize) -> Surface {
    match case {
        0 => Surface::rational_bezier(
            vec![
                vec![
                    Point3::new(0.into(), 0.into(), 0.into()),
                    Point3::new(1.into(), 0.into(), 0.into()),
                ],
                vec![
                    Point3::new(0.into(), 2.into(), 0.into()),
                    Point3::new(1.into(), 2.into(), 1.into()),
                ],
            ],
            vec![vec![1.into(); 2]; 2],
        )
        .unwrap(),
        1 => Surface::rational_bezier(
            vec![
                vec![
                    Point3::new(0.into(), 0.into(), 0.into()),
                    Point3::new((Real::one() / Real::from(2)).unwrap(), 0.into(), 0.into()),
                ],
                vec![
                    Point3::new(0.into(), 1.into(), 0.into()),
                    Point3::new((Real::one() / Real::from(2)).unwrap(), 1.into(), 0.into()),
                ],
            ],
            vec![vec![1.into(), 2.into()]; 2],
        )
        .unwrap(),
        2 => Surface::rational_bezier(
            (0..=3)
                .map(|j| {
                    (0..=8)
                        .map(|i| {
                            Point3::new(
                                (Real::from(i) / Real::from(8)).unwrap(),
                                (Real::from(j) / Real::from(3)).unwrap(),
                                i32::from(i == 8 && j == 3).into(),
                            )
                        })
                        .collect()
                })
                .collect(),
            vec![vec![1.into(); 9]; 4],
        )
        .unwrap(),
        _ => panic!("unknown surface workload"),
    }
}

pub fn complete(
    source: &Surface,
    shared: bool,
    graph: &mut ExpressionBudget,
    arithmetic: &mut DyadicBudget,
) -> Vec<SurfaceRangeEnclosure> {
    let prepared = shared.then(|| prepare_rational_bezier_surface_ranges(source, graph).unwrap());
    let mut reports = Vec::new();
    for u in 0..4 {
        for v in 0..4 {
            let parameters = BoxN::new([
                DyadicInterval::new(u.into(), (u + 1).into(), -2).unwrap(),
                DyadicInterval::new(v.into(), (v + 1).into(), -2).unwrap(),
            ])
            .unwrap();
            let report = match &prepared {
                Some(p) => p.enclose(&parameters, -32, graph, arithmetic).unwrap(),
                None => prepare_rational_bezier_surface_ranges(source, graph)
                    .unwrap()
                    .enclose(&parameters, -32, graph, arithmetic)
                    .unwrap(),
            };
            assert_eq!(report.parameters(), &parameters);
            let projected = report.native_bounds(graph, arithmetic).unwrap();
            assert!(std::ptr::eq(projected.evidence(), &report));
            let bounds = projected.bounds().unwrap();
            assert!(matches!(
                ordered_aabb3s_intersect(
                    &bounds.mins,
                    &bounds.maxs,
                    &Point3::new(4.into(), 4.into(), 4.into()),
                    &Point3::new(5.into(), 5.into(), 5.into()),
                    PredicatePolicy::STRICT
                ),
                PredicateOutcome::Decided {
                    value: false,
                    certainty: Certainty::Exact,
                    ..
                }
            ));
            reports.push(report);
        }
    }
    reports
}

/// Independent extrema for these monotone formulas on nonnegative UV boxes.
pub fn validate(case: usize, reports: &[SurfaceRangeEnclosure]) {
    assert_eq!(reports.len(), 16);
    let power = |x: &Rational, n: i32| x.clone().powi(n.into()).unwrap();
    for (cell, report) in reports.iter().enumerate() {
        let cell = i32::try_from(cell).unwrap();
        let u0 = Rational::from(cell / 4) / Rational::from(4);
        let u1 = Rational::from(cell / 4 + 1) / Rational::from(4);
        let v0 = Rational::from(cell % 4) / Rational::from(4);
        let v1 = Rational::from(cell % 4 + 1) / Rational::from(4);
        let expected = match case {
            0 => vec![
                (u0.clone(), u1.clone()),
                (Rational::from(2) * &v0, Rational::from(2) * &v1),
                (&u0 * &v0, &u1 * &v1),
                (1.into(), 1.into()),
                (0.into(), 0.into()),
                (v0, v1),
                (0.into(), 0.into()),
                (2.into(), 2.into()),
                (u0, u1),
            ],
            1 => vec![
                (&u0 / (Rational::one() + &u0), &u1 / (Rational::one() + &u1)),
                (v0, v1),
                (0.into(), 0.into()),
                (
                    Rational::one() / power(&(Rational::one() + &u1), 2),
                    Rational::one() / power(&(Rational::one() + &u0), 2),
                ),
                (0.into(), 0.into()),
                (0.into(), 0.into()),
                (0.into(), 0.into()),
                (1.into(), 1.into()),
                (0.into(), 0.into()),
            ],
            2 => vec![
                (u0.clone(), u1.clone()),
                (v0.clone(), v1.clone()),
                (power(&u0, 8) * power(&v0, 3), power(&u1, 8) * power(&v1, 3)),
                (1.into(), 1.into()),
                (0.into(), 0.into()),
                (
                    Rational::from(8) * power(&u0, 7) * power(&v0, 3),
                    Rational::from(8) * power(&u1, 7) * power(&v1, 3),
                ),
                (0.into(), 0.into()),
                (1.into(), 1.into()),
                (
                    Rational::from(3) * power(&u0, 8) * power(&v0, 2),
                    Rational::from(3) * power(&u1, 8) * power(&v1, 2),
                ),
            ],
            _ => panic!("unknown surface workload"),
        };
        for (range, (lo, hi)) in report
            .coordinates()
            .into_iter()
            .chain(report.partials().into_iter().flatten())
            .zip(expected)
        {
            let range = range.to_rational(&mut arithmetic()).unwrap();
            assert!(
                range.contains_point(&lo) && range.contains_point(&hi),
                "case {case}, cell {cell}, [{lo},{hi}] outside {range:?}"
            );
        }
    }
}

pub fn compare(left: &[SurfaceRangeEnclosure], right: &[SurfaceRangeEnclosure]) {
    assert_eq!(left.len(), right.len());
    for (a, b) in left.iter().zip(right) {
        assert_eq!(
            a.evidence().program().program().definition(),
            b.evidence().program().program().definition()
        );
        assert_eq!(a.parameters(), b.parameters());
        assert_eq!(a.evidence().values(), b.evidence().values());
    }
}
