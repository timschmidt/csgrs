use super::*;
use crate::{WorkBudget, WorkKind, evaluate_with_budget};

fn fraction(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn source(square_denominator: i32, lower: Real, upper: Real) -> AlgebraicRootRepresentation {
    let parameter = BezierAlgebraicParameter2::try_isolate(
        BezierParameterPolynomial::try_new_power_basis(vec![
            -fraction(1, square_denominator),
            Real::zero(),
            Real::one(),
        ])
        .unwrap(),
        BezierParameterInterval::try_new(lower, upper).unwrap(),
    )
    .unwrap();
    parameter_representation(&parameter, &CurveContext::STRICT)
}

#[test]
fn dense_projection_charges_only_needed_selected_axis_resultants_and_retries_exactly() {
    let sources = [
        source(2, fraction(2, 3), fraction(3, 4)),
        source(3, fraction(1, 2), fraction(2, 3)),
    ];
    // The sum needs two norms. For the product, the first norm contains only
    // y^2, so reduction by y^2=1/3 removes the other source axis exactly.
    for (source_terms, required_promotions) in [(vec![4, 2], 2), (vec![6], 1)] {
        let mut coefficients = vec![Real::zero(); 8];
        coefficients[1] = Real::one();
        for index in source_terms {
            coefficients[index] = -Real::one();
        }
        let relation = DenseTensorPolynomial::try_new(vec![2, 2, 2], coefficients).unwrap();
        let project = || selected_dense_last_axis_projection(&relation, &sources).unwrap();
        let expected = project();
        assert!(matches!(expected, Classification::Decided(_)));
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for promotions in 0..required_promotions {
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
                    promotions: required_promotions,
                },
                project,
            );
            assert_eq!(retry.result.unwrap().certified().unwrap(), expected);
            assert_eq!(retry.usage.promotions, required_promotions);
        }
    }
}

#[test]
fn bivariate_projection_adapter_preserves_budget_failure_through_degree_continuation() {
    let first = BivariatePolynomial::new(vec![vec![Real::zero(), -Real::one()], vec![Real::one()]]);
    let second = BivariatePolynomial::new(vec![
        vec![-fraction(1, 2)],
        vec![Real::zero()],
        vec![Real::one()],
    ]);
    let project = || {
        hypersolve::curve_resultant::resultant_bivariate_polynomial_system_complete(
            &first,
            &second,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig {
                min_precision: -512,
                max_resultant_degree: 0,
            },
        )
    };
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let Classification::Decided(Some(expected)) =
            resultant_parameter_polynomial(project(), &policy).unwrap()
        else {
            panic!("the exact unbudgeted resultant must yield a polynomial");
        };
        let denied = evaluate_with_budget(policy.predicate_policy(), WorkBudget::ZERO, || {
            assert!(matches!(
                resultant_parameter_polynomial(project(), &policy).unwrap(),
                Classification::Uncertain(UncertaintyReason::BudgetExhausted)
            ));
        });
        assert_eq!(
            denied.result.unwrap_err().kind,
            WorkKind::AlgebraicPromotion
        );
        let retry = evaluate_with_budget(
            policy.predicate_policy(),
            WorkBudget {
                refinements: 4096,
                promotions: 1,
            },
            || resultant_parameter_polynomial(project(), &policy).unwrap(),
        );
        let Classification::Decided(Some(actual)) = retry.result.unwrap().certified().unwrap()
        else {
            panic!("the larger budget must retain the exact polynomial");
        };
        assert_eq!(actual.coefficients(), expected.coefficients());
        assert_eq!(retry.usage.promotions, 1);
    }
}
