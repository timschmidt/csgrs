//! Point identity across independently retained polynomial event fibers.

use hypercurve::{Axis2, CurvePoint2, PolynomialCurveSet2, Real};
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};
use hypersolve::{AlgebraicFiberRootIsolationConfig, BivariatePolynomial};
use std::cmp::Ordering;

fn curves(rows: &[&[i64]]) -> PolynomialCurveSet2 {
    let polynomial = BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    );
    let topology = decompose_bivariate_curve(
        &polynomial,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-2).into(), 2.into()],
        },
        BivariateTopologyConfig::default(),
    )
    .unwrap();
    PolynomialCurveSet2::from_topology(topology, AlgebraicFiberRootIsolationConfig::default())
        .unwrap()
}

fn interior_and_event() -> (CurvePoint2, CurvePoint2) {
    let first = curves(&[&[0, 0, 1], &[0], &[-1]]);
    let second = curves(&[&[0, -1, 2], &[1, -2]]);
    let half = (Real::one() / Real::from(2)).unwrap();
    (
        first.curves()[3].point_at(&half.into()).unwrap(),
        second.curves()[1].end().clone(),
    )
}

#[test]
fn interior_and_event_points_compare_with_retained_base_relations() {
    // y=x in F=(y-x)(y+x), and the event (1/2,1/2) of
    // G=(y-x)(2y-1), selected independently after the original sets drop.
    for reverse in [false, true] {
        let (first, second) = interior_and_event();
        let (first, second) = if reverse {
            (second, first)
        } else {
            (first, second)
        };
        for policy in [
            hypercurve::PredicatePolicy::STRICT,
            hypercurve::PredicatePolicy::APPROXIMATE_512,
            hypercurve::PredicatePolicy::STRICT,
        ] {
            let outcome = hypercurve::evaluate_under(policy, || {
                let result = hypersolve::work_budget::with_budget(
                    hypersolve::work_budget::WorkBudget {
                        refinements: 512,
                        promotions: 1,
                    },
                    || first.coincides_with(&second),
                );
                assert!(result.result.unwrap().unwrap());
                assert_eq!(
                    first.compare_coordinate(&second, Axis2::X).unwrap(),
                    Ordering::Equal
                );
                assert_eq!(
                    first.compare_coordinate(&second, Axis2::Y).unwrap(),
                    Ordering::Equal
                );
            });
            assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
            outcome.certified().unwrap();
        }
    }
}

#[test]
fn replayed_event_points_keep_singleton_order_and_distinct_neighbors() {
    let half = (Real::one() / Real::from(2)).unwrap();
    let tiny = (Real::one() / Real::from(1_u64 << 50)).unwrap();
    let tiny = &tiny * &tiny;
    for offset in [-tiny.clone(), Real::zero(), tiny] {
        let (interior, event) = interior_and_event();
        let hypercurve::CurvePointDefinition2::PolynomialFiber { parameter, .. } =
            event.definition().unwrap()
        else {
            panic!("the event retains its original selected base");
        };
        let y = &half + &offset;
        let definition = hypercurve::CurvePointDefinition2::PolynomialFiber {
            equation: BivariatePolynomial::new(vec![vec![-y.clone(), Real::one()]]),
            parameter,
            root: hypersolve::IsolatedRootInterval {
                lower: y.clone(),
                upper: y.clone(),
                exact_root: Some(y.clone()),
                distinct_root_count: 1,
            },
        };
        // Native replay validates this line relation and selector; the flags
        // above are not accepted as proof. Drop the original event carrier.
        drop(event);
        let replay = definition.into_point().unwrap();
        let expected = half
            .exact_rational_ref()
            .unwrap()
            .partial_cmp(y.exact_rational_ref().unwrap())
            .unwrap();
        for (a, b, expected) in [
            (&interior, &replay, expected),
            (&replay, &interior, expected.reverse()),
        ] {
            let result = hypersolve::work_budget::with_budget(
                hypersolve::work_budget::WorkBudget {
                    refinements: 512,
                    promotions: 1,
                },
                || a.compare_coordinate(b, Axis2::Y),
            );
            assert_eq!(result.result.unwrap().unwrap(), expected);
            assert_eq!(a.coincides_with(b).unwrap(), expected == Ordering::Equal);
            assert_eq!(a.compare_coordinate(b, Axis2::X).unwrap(), Ordering::Equal);
        }
    }
    let (interior, event) = interior_and_event();
    let replay = event.definition().unwrap().into_point().unwrap();
    drop(event);
    assert!(interior.coincides_with(&replay).unwrap());
    let opposite = curves(&[&[0, 0, 1], &[0], &[-1]]).curves()[2]
        .point_at(&half.into())
        .unwrap();
    assert_eq!(
        opposite.compare_coordinate(&replay, Axis2::Y).unwrap(),
        Ordering::Less
    );
    assert!(!opposite.coincides_with(&replay).unwrap());
}

#[test]
fn singleton_point_order_retries_after_counted_exhaustion() {
    let (first, second) = interior_and_event();
    let denied =
        hypersolve::work_budget::with_budget(hypersolve::work_budget::WorkBudget::ZERO, || {
            first.compare_coordinate(&second, Axis2::Y)
        });
    let reached = denied.result.unwrap_err();
    assert_eq!((reached.limit, reached.used), (0, 0));
    assert!(reached.requested > 0);
    let retry = hypersolve::work_budget::with_budget(
        hypersolve::work_budget::WorkBudget {
            refinements: 512,
            promotions: 1,
        },
        || first.compare_coordinate(&second, Axis2::Y),
    );
    assert_eq!(retry.result.unwrap().unwrap(), Ordering::Equal);
    assert!(first.coincides_with(&second).unwrap());
}
