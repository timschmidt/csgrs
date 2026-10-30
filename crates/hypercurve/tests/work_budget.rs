use std::cmp::Ordering;

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, BudgetExceeded, CancellationToken, Curve2, CurveCertainty,
    ExactCurveError, Point2, PredicatePolicy, RationalBezier2, Real, UncertaintyReason, WorkBudget,
    WorkKind, WorkUsage, evaluate_with_budget, evaluate_with_cancellable_budget,
};

fn rational(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

#[test]
fn overlapping_simple_and_even_roots_retry_under_a_larger_budget() {
    for coefficients in [vec![-1, 0, 2], vec![1, 0, -4, 0, 4]] {
        let polynomial = BezierParameterPolynomial::try_new_power_basis(
            coefficients.into_iter().map(Real::from).collect(),
        )
        .unwrap();
        let interval = BezierParameterInterval::try_new(rational(2, 3), rational(3, 4)).unwrap();
        let irrational = BezierParameter2::Algebraic(
            BezierAlgebraicParameter2::try_isolate(polynomial, interval).unwrap(),
        );
        let rational = BezierParameter2::Exact(rational(353_553, 500_000));
        for policy in [
            PredicatePolicy::STRICT,
            PredicatePolicy::APPROXIMATE_512,
            PredicatePolicy::STRICT,
        ] {
            for limit in [0, 1] {
                let exhausted = evaluate_with_budget(
                    policy,
                    WorkBudget {
                        refinements: limit,
                        promotions: 0,
                    },
                    || {
                        let result = rational.cmp_by_refinement(&irrational);
                        assert!(matches!(
                            result,
                            Err(ExactCurveError::Blocked(blocker))
                                if blocker.reason() == UncertaintyReason::BudgetExhausted
                        ));
                        result
                    },
                );
                assert_eq!(
                    exhausted.result.unwrap_err(),
                    BudgetExceeded {
                        kind: WorkKind::RootRefinement,
                        limit,
                        used: limit,
                        requested: 1,
                    }
                );
                assert_eq!(exhausted.usage.refinements, limit);
                assert_eq!(exhausted.usage.promotions, 0);
            }
            let completed = evaluate_with_budget(
                policy,
                WorkBudget {
                    refinements: 128,
                    promotions: 0,
                },
                || rational.cmp_by_refinement(&irrational),
            );
            assert!(completed.usage.refinements > 1);
            let completed = completed.result.unwrap();
            assert_eq!(completed.certainty(), CurveCertainty::Certified);
            assert_eq!(completed.certified().unwrap().unwrap(), Ordering::Less);
        }
    }
}

#[test]
fn zero_budget_reuses_direct_exact_ordering() {
    let left = BezierParameter2::Exact(rational(1, 3));
    let right = BezierParameter2::Exact(rational(1, 2));
    let outcome = evaluate_with_budget(PredicatePolicy::STRICT, WorkBudget::ZERO, || {
        left.cmp_by_refinement(&right)
    });
    assert_eq!(outcome.usage, WorkUsage::default());
    assert_eq!(
        outcome.result.unwrap().certified().unwrap().unwrap(),
        Ordering::Less
    );
}

#[test]
fn rational_curve_intersections_retry_after_denied_elimination() {
    let curve = |points: [(i32, i32); 4]| {
        Curve2::from(
            RationalBezier2::try_new(
                points
                    .into_iter()
                    .map(|(x, y)| Point2::from_values(x, y))
                    .collect(),
                [1, 2, 2, 1].into_iter().map(Real::from).collect(),
            )
            .unwrap(),
        )
    };
    let first = curve([(0, 0), (0, 1), (1, 0), (1, 1)]);
    let second = curve([(0, 1), (0, 0), (1, 1), (1, 0)]);
    let denied = evaluate_with_budget(
        PredicatePolicy::STRICT,
        WorkBudget {
            refinements: 4096,
            promotions: 0,
        },
        || {
            let result = first.intersect_curve(&second).unwrap();
            assert!(!result.is_complete());
            assert!(result.contacts().is_empty());
            assert!(result.blockers().iter().any(|blocker| matches!(
                blocker.kind(),
                hypercurve::CurveIntersectionPairBlockerKind2::Uncertain(
                    UncertaintyReason::BudgetExhausted
                )
            )));
        },
    );
    assert_eq!(
        denied.result.unwrap_err().kind,
        WorkKind::AlgebraicPromotion
    );
    assert_eq!(denied.usage.promotions, 0);
    let retry = evaluate_with_budget(
        PredicatePolicy::STRICT,
        WorkBudget {
            refinements: 4096,
            promotions: 128,
        },
        || first.intersect_curve(&second),
    );
    assert!(retry.usage.promotions > 0);
    let retry = retry.result.unwrap();
    assert_eq!(retry.certainty(), CurveCertainty::Certified);
    let actual = retry.certified().unwrap().unwrap();
    assert!(actual.is_complete());
    assert!(actual.blockers().is_empty());
    // Equal x parameters reduce y=1-y to (2t-1)(7t^2-7t+1)=0.
    assert_eq!(actual.contacts().len(), 3);
    let expected = first.intersect_curve(&second).unwrap();
    assert_eq!(actual.contacts(), expected.contacts());
    assert_eq!(actual.overlaps(), expected.overlaps());
    assert_eq!(actual.span_pair_count(), expected.span_pair_count());
}

#[test]
fn polynomial_isolation_shares_a_total_subdivision_budget() {
    let polynomial = BezierParameterPolynomial::try_new_power_basis(
        [-36, 1368, -11034, 31728, -38280, 16620]
            .into_iter()
            .map(Real::from)
            .collect(),
    )
    .unwrap();
    let expected = polynomial.isolate_unit_interval_roots().unwrap();
    assert_eq!(expected.len(), 5);
    for limit in [0, 1] {
        let failed = evaluate_with_budget(
            PredicatePolicy::STRICT,
            WorkBudget {
                refinements: limit,
                promotions: 0,
            },
            || polynomial.isolate_unit_interval_roots(),
        );
        assert_eq!(failed.result.unwrap_err().kind, WorkKind::RootRefinement);
        assert_eq!(failed.usage.refinements, limit);
    }
    let complete = evaluate_with_budget(
        PredicatePolicy::STRICT,
        WorkBudget {
            refinements: 512,
            promotions: 0,
        },
        || polynomial.isolate_unit_interval_roots(),
    );
    assert!(complete.usage.refinements > 1);
    assert_eq!(
        complete.result.unwrap().certified().unwrap().unwrap(),
        expected
    );
}

fn crossing_rational_cubics() -> (Curve2, Curve2) {
    let curve = |points: [(i32, i32); 4]| {
        Curve2::from(
            RationalBezier2::try_new(
                points
                    .into_iter()
                    .map(|(x, y)| Point2::from_values(x, y))
                    .collect(),
                [1, 2, 2, 1].into_iter().map(Real::from).collect(),
            )
            .unwrap(),
        )
    };
    (
        curve([(0, 0), (0, 1), (1, 0), (1, 1)]),
        curve([(0, 1), (0, 0), (1, 1), (1, 0)]),
    )
}

#[test]
fn cancelled_intersections_publish_nothing_and_retry_with_a_new_token() {
    let (first, second) = crossing_rational_cubics();
    let cancelled = CancellationToken::new();
    cancelled.cancel();
    let denied = evaluate_with_cancellable_budget(
        PredicatePolicy::STRICT,
        WorkBudget::UNLIMITED,
        &cancelled,
        || {
            // Inner consumers see an ordinary resource blocker, never contacts.
            match first.intersect_curve(&second) {
                Ok(partial) => {
                    assert!(!partial.is_complete());
                    assert!(partial.contacts().is_empty());
                }
                Err(ExactCurveError::Blocked(blocker)) => {
                    assert_eq!(blocker.reason(), UncertaintyReason::BudgetExhausted);
                }
                Err(other) => panic!("unexpected failure {other:?}"),
            }
        },
    );
    assert_eq!(denied.result.unwrap_err().kind, WorkKind::Cancellation);
    assert_eq!(denied.usage, WorkUsage::default());
    // A cancelled attempt leaves no cached blocker on these curves.
    let retry = evaluate_with_cancellable_budget(
        PredicatePolicy::STRICT,
        WorkBudget::UNLIMITED,
        &CancellationToken::new(),
        || first.intersect_curve(&second),
    );
    assert!(retry.usage.promotions > 0);
    let retry = retry.result.unwrap().certified().unwrap().unwrap();
    assert!(retry.is_complete());
    // Equal x parameters reduce y=1-y to (2t-1)(7t^2-7t+1)=0.
    assert_eq!(retry.contacts().len(), 3);
    let (fresh_first, fresh_second) = crossing_rational_cubics();
    let expected = fresh_first.intersect_curve(&fresh_second).unwrap();
    assert_eq!(retry.contacts(), expected.contacts());
    assert_eq!(retry.overlaps(), expected.overlaps());
}

/// Another thread cancels while fresh intersections run. Each attempt either
/// observes the request and publishes nothing, or completed first with the
/// independent expected contacts; it never publishes a partial result.
#[test]
fn concurrent_cancellation_never_publishes_a_partial_intersection() {
    let (first, second) = crossing_rational_cubics();
    let expected = first.intersect_curve(&second).unwrap();
    let mut outcomes = [0_usize; 2];
    for delay in [0_u64, 1, 5, 20] {
        let token = CancellationToken::new();
        let worker = {
            let token = token.clone();
            std::thread::spawn(move || {
                let (first, second) = crossing_rational_cubics();
                evaluate_with_cancellable_budget(
                    PredicatePolicy::STRICT,
                    WorkBudget::UNLIMITED,
                    &token,
                    || first.intersect_curve(&second),
                )
            })
        };
        std::thread::sleep(std::time::Duration::from_millis(delay));
        token.cancel();
        let outcome = worker.join().unwrap();
        match outcome.result {
            Err(exceeded) => {
                assert_eq!(exceeded.kind, WorkKind::Cancellation);
                outcomes[0] += 1;
            }
            Ok(provisional) => {
                let actual = provisional.certified().unwrap().unwrap();
                assert!(actual.is_complete());
                assert_eq!(actual.contacts(), expected.contacts());
                outcomes[1] += 1;
            }
        }
    }
    assert_eq!(outcomes.iter().sum::<usize>(), 4);
}
