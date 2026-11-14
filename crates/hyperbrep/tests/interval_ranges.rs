#![cfg(feature = "interval-ranges")]

use hyperbrep::{Point3, Surface, interval_ranges::prepare_rational_bezier_surface_ranges};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice::BoxN;
use hyperlimit::{Certainty, PredicateOutcome, PredicatePolicy, ordered_aabb3s_intersect};
use hypersolve::expression_program::ExpressionBudget;

fn d(lower: i64, upper: i64, exponent: i32) -> DyadicInterval {
    DyadicInterval::new(lower.into(), upper.into(), exponent).unwrap()
}

#[test]
fn native_asymmetric_patch_ranges_feed_certified_spatial_exclusion() {
    // Rows increase v, columns increase u: S(u,v) = (u, 2v, uv).
    let surface = Surface::rational_bezier(
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
        vec![vec![1.into(), 1.into()]; 2],
    )
    .unwrap();
    let mut graph = ExpressionBudget::new(1_000_000, 100_000_000);
    let mut arithmetic = DyadicBudget::new(16384, 1_000_000_000_000);
    let prepared = prepare_rational_bezier_surface_ranges(&surface, &mut graph).unwrap();
    let domain = BoxN::new([d(1, 2, -2), d(2, 3, -2)]).unwrap();
    let report = prepared
        .enclose(&domain, -64, &mut graph, &mut arithmetic)
        .unwrap();
    assert_eq!(report.parameters(), &domain);
    assert_eq!(
        report.coordinates(),
        [&d(1, 2, -2), &d(2, 3, -1), &d(1, 3, -3)]
    );
    assert_eq!(
        report.partials()[0],
        [&d(1, 1, 0), &d(0, 0, 0), &d(2, 3, -2)]
    );
    assert_eq!(
        report.partials()[1],
        [&d(0, 0, 0), &d(2, 2, 0), &d(1, 2, -2)]
    );
    let projected = report.native_bounds(&mut graph, &mut arithmetic).unwrap();
    assert!(std::ptr::eq(projected.evidence(), &report));
    let bounds = projected.bounds().unwrap();
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        assert!(matches!(
            ordered_aabb3s_intersect(
                &bounds.mins,
                &bounds.maxs,
                &Point3::new(2.into(), 2.into(), 2.into()),
                &Point3::new(3.into(), 3.into(), 3.into()),
                policy,
            ),
            PredicateOutcome::Decided {
                value: false,
                certainty: Certainty::Exact,
                ..
            }
        ));
    }
}
