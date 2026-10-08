use std::cmp::Ordering;

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, BudgetExceeded, CurveCertainty, ExactCurveError, PredicatePolicy,
    Real, UncertaintyReason, WorkBudget, WorkKind, WorkUsage, evaluate_with_budget,
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
