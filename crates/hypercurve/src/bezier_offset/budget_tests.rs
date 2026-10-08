use super::*;
use crate::{WorkBudget, WorkKind, evaluate_with_budget};

fn fraction(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn parameter(coefficients: Vec<Real>, lower: Real, upper: Real) -> BezierParameter2 {
    BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(coefficients).unwrap(),
            BezierParameterInterval::try_new(lower, upper).unwrap(),
        )
        .unwrap(),
    )
}

#[test]
fn selected_pair_subresultant_budget_preserves_even_contact_after_selected_degree_drop() {
    let third = fraction(1, 3);
    let four_fifths = fraction(4, 5);
    let alpha = parameter(
        vec![-fraction(1, 2), Real::zero(), Real::one()],
        fraction(2, 3),
        fraction(3, 4),
    );
    let beta = parameter(
        vec![
            &third * &four_fifths,
            Real::zero(),
            -(&third + &four_fifths),
            Real::zero(),
            Real::one(),
        ],
        fraction(1, 2),
        fraction(2, 3),
    );
    // (t-alpha)^2 + (beta^2-1/3)t^3 has an even contact at t=alpha.
    // Its cubic coefficient vanishes only in beta's selected factor.
    let mut coefficients = vec![vec![vec![Real::zero(); 4]; 3]; 3];
    coefficients[0][0][2] = Real::one();
    coefficients[1][0][1] = Real::from(-2);
    coefficients[2][0][0] = Real::one();
    coefficients[0][0][3] = -third;
    coefficients[0][2][3] = Real::one();
    let projected = TrivariatePolynomial { coefficients };
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let project = || {
            projected_selected_trivariate_candidate_has_subresultant_root_with_resultant_limit(
                &projected, &alpha, &beta, &alpha, 0, &policy,
            )
            .unwrap()
        };
        assert_eq!(project(), Classification::Decided(true));
        for promotions in [0, 1] {
            let denied = evaluate_with_budget(
                policy.predicate_policy(),
                WorkBudget {
                    refinements: 4096,
                    promotions,
                },
                || {
                    assert_eq!(
                        project(),
                        Classification::Uncertain(UncertaintyReason::BudgetExhausted)
                    );
                },
            );
            assert_eq!(
                denied.result.unwrap_err().kind,
                WorkKind::AlgebraicPromotion
            );
            assert_eq!(denied.usage.promotions, promotions);
        }
        let retry = evaluate_with_budget(
            policy.predicate_policy(),
            WorkBudget {
                refinements: 4096,
                promotions: 64,
            },
            project,
        );
        assert_eq!(
            retry.result.unwrap().certified().unwrap(),
            Classification::Decided(true)
        );
    }
}

#[test]
fn selected_trivariate_projection_propagates_norm_budget_without_tensor_fallback() {
    let first = parameter(
        vec![-fraction(1, 2), Real::zero(), Real::zero(), Real::one()],
        fraction(3, 4),
        fraction(4, 5),
    );
    let second = BezierParameter2::Exact(Real::zero());
    // t=x^2 with x^3=1/2 needs the fixed-rank first-axis norm.
    let polynomial = TrivariatePolynomial::from_coefficients(vec![
        vec![vec![Real::zero(), Real::one()]],
        vec![vec![Real::zero()]],
        vec![vec![-Real::one()]],
    ])
    .unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let project = || {
            selected_trivariate_third_axis_parameters_with_resultant_limit(
                &polynomial,
                &first,
                &second,
                SelectedThirdAxisDomain2::Finite(&CurveParameterRange2::unit()),
                32,
                &policy,
            )
            .unwrap()
        };
        let expected = project();
        let Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(parameters)) =
            &expected
        else {
            panic!("the retained cubic source must yield its exact square");
        };
        assert_eq!(parameters.len(), 1);
        let denied = evaluate_with_budget(
            policy.predicate_policy(),
            WorkBudget {
                refinements: 4096,
                promotions: 0,
            },
            || {
                assert!(matches!(
                    project(),
                    Classification::Uncertain(UncertaintyReason::BudgetExhausted)
                ))
            },
        );
        assert_eq!(
            denied.result.unwrap_err().kind,
            WorkKind::AlgebraicPromotion
        );
        let retry = evaluate_with_budget(
            policy.predicate_policy(),
            WorkBudget {
                refinements: 4096,
                promotions: 64,
            },
            project,
        );
        let Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(retried)) =
            retry.result.unwrap().certified().unwrap()
        else {
            panic!("the exact projection must remain available after exhaustion");
        };
        assert_eq!(&retried, parameters);
    }
}
