use super::*;
use crate::{WorkBudget, WorkKind, evaluate_with_budget};

fn allowance(promotions: usize) -> WorkBudget {
    WorkBudget {
        refinements: 4096,
        promotions,
    }
}

fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|r| r.iter().copied().map(Real::from).collect())
            .collect(),
    )
}

#[test]
fn selector_splits_and_branch_factor_removal_preserve_resource_failure_and_exact_retry() {
    let shared = polynomial(&[&[0, 0, 1], &[-1]]);
    let first = polynomial(&[&[1, 1], &[1]]);
    let second = polynomial(&[&[3, 1], &[2]]);
    let component = bivariate_multiply(&shared, &first);
    let branch = bivariate_multiply(&shared, &second);
    let config = CurveIntersectionResultantConfig {
        max_resultant_degree: 0,
        ..Default::default()
    };
    for axis in [
        CurveResultantParameter::First,
        CurveResultantParameter::Second,
    ] {
        let split = || {
            split_parameter_component_at_selector_boundary(
                &component,
                std::slice::from_ref(&branch),
                axis,
                config,
            )
        };
        let remove = || {
            remove_implicit_parameter_component_zero_branch_factors(
                &component, &branch, axis, config,
            )
        };
        let expected_split = split();
        let expected_remove = remove();
        assert!(matches!(expected_split, Classification::Decided(Some(_))));
        assert!(matches!(expected_remove, Classification::Decided(Some(_))));
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let denied = evaluate_with_budget(policy.predicate_policy(), allowance(0), || {
                assert_eq!(
                    split(),
                    Classification::Uncertain(UncertaintyReason::BudgetExhausted)
                );
            });
            assert_eq!(
                denied.result.unwrap_err().kind,
                WorkKind::AlgebraicPromotion
            );
            let denied = evaluate_with_budget(policy.predicate_policy(), allowance(0), || {
                assert_eq!(
                    remove(),
                    Classification::Uncertain(UncertaintyReason::BudgetExhausted)
                );
            });
            assert_eq!(
                denied.result.unwrap_err().kind,
                WorkKind::AlgebraicPromotion
            );
            let retry = evaluate_with_budget(policy.predicate_policy(), allowance(32), || {
                (split(), remove())
            });
            assert_eq!(
                retry.result.unwrap().certified().unwrap(),
                (expected_split.clone(), expected_remove.clone())
            );
            let direct = evaluate_with_budget(policy.predicate_policy(), WorkBudget::ZERO, || {
                split_parameter_component_at_selector_boundary(
                    &component,
                    std::slice::from_ref(&shared),
                    axis,
                    config,
                )
            });
            assert!(matches!(
                direct.result.unwrap().certified().unwrap(),
                Classification::Decided(Some(_))
            ));
            assert_eq!(direct.usage, Default::default());
        }
    }
}

#[test]
fn multiplicity_reduction_does_not_publish_a_partially_reduced_component() {
    let quadratic = polynomial(&[&[0, 0, 1], &[-1]]);
    let linear = polynomial(&[&[1, 1], &[1]]);
    let repeated = bivariate_multiply(
        &bivariate_multiply(&quadratic, &quadratic),
        &bivariate_multiply(&bivariate_multiply(&linear, &linear), &linear),
    );
    let config = CurveIntersectionResultantConfig::default();
    for axis in [
        CurveResultantParameter::First,
        CurveResultantParameter::Second,
    ] {
        let reduce = || reduce_implicit_parameter_component_multiplicity(&repeated, axis, config);
        let expected = reduce();
        assert!(matches!(expected, Classification::Decided(Some(_))));
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for units in 0..2 {
                let denied =
                    evaluate_with_budget(policy.predicate_policy(), allowance(units), || {
                        assert_eq!(
                            reduce(),
                            Classification::Uncertain(UncertaintyReason::BudgetExhausted)
                        )
                    });
                assert_eq!(denied.result.unwrap_err().used, units);
            }
            let retry = evaluate_with_budget(policy.predicate_policy(), allowance(32), reduce);
            assert_eq!(retry.result.unwrap().certified().unwrap(), expected);
        }
    }
}

#[test]
fn parallel_source_components_and_candidate_saturation_keep_exhaustion_typed() {
    let factor = polynomial(&[&[-2], &[1]]);
    let equations = [
        bivariate_multiply(&factor, &polynomial(&[&[0, 1], &[-1]])),
        bivariate_multiply(&factor, &polynomial(&[&[1, 1]])),
    ];
    let config = CurveIntersectionResultantConfig::default();
    let source = || parallel_source_parameter_components_from_equations(equations.clone(), config);
    let expected = source();
    assert!(matches!(expected, Classification::Decided(ref components) if !components.is_empty()));
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let denied = evaluate_with_budget(policy.predicate_policy(), allowance(1), || {
            assert_eq!(
                source(),
                Classification::Uncertain(UncertaintyReason::BudgetExhausted)
            )
        });
        assert_eq!(denied.result.unwrap_err().used, 1);
        let retry = evaluate_with_budget(policy.predicate_policy(), allowance(16), source);
        assert_eq!(retry.result.unwrap().certified().unwrap(), expected);
        let denied = evaluate_with_budget(policy.predicate_policy(), allowance(0), || {
            assert!(matches!(
                parallel_intersection_candidate_system(
                    equations.clone(),
                    CurveIntersectionCandidates2::DegenerateResultant,
                    &policy
                )
                .unwrap(),
                Classification::Uncertain(UncertaintyReason::BudgetExhausted)
            ));
        });
        assert!(denied.result.is_err());
        let denied = evaluate_with_budget(policy.predicate_policy(), allowance(0), || {
            assert!(matches!(
                parameter_component_system(&equations, &polynomial(&[&[1]]), &policy, config)
                    .unwrap(),
                Classification::Uncertain(UncertaintyReason::BudgetExhausted)
            ));
        });
        assert!(denied.result.is_err());
        assert!(matches!(
            certify_regular_implicit_parameter_component_fallback(
                &factor,
                &factor,
                CurveResultantParameter::First,
                &policy,
                config,
                Classification::Uncertain(UncertaintyReason::BudgetExhausted),
            )
            .unwrap(),
            Classification::Uncertain(UncertaintyReason::BudgetExhausted)
        ));
    }
}

#[test]
fn trivariate_factor_sign_propagates_normalization_exhaustion_before_policy_fallback() {
    let first = [polynomial(&[&[1, 1], &[1]]), polynomial(&[&[1], &[1]])];
    let second = [polynomial(&[&[2, 3], &[2]]), polynomial(&[&[2, 1]])];
    let product = TrivariatePolynomial::multiply(
        &trivariate_from_axis_bivariate_coefficients(&first, 0, [1, 2]).unwrap(),
        &trivariate_from_axis_bivariate_coefficients(&second, 0, [1, 2]).unwrap(),
    )
    .unwrap();
    let parameter = BezierParameter2::Exact((Real::one() / Real::from(4)).unwrap());
    let sign =
        || trivariate_bounded_factor_sign(&product, &parameter, &parameter, &parameter).unwrap();
    let expected = sign();
    assert_eq!(expected, Classification::Decided(Some(RealSign::Positive)));
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let denied = evaluate_with_budget(policy.predicate_policy(), allowance(0), || {
            assert_eq!(
                sign(),
                Classification::Uncertain(UncertaintyReason::BudgetExhausted)
            )
        });
        assert_eq!(
            denied.result.unwrap_err().kind,
            WorkKind::AlgebraicPromotion
        );
        let retry = evaluate_with_budget(policy.predicate_policy(), allowance(32), sign);
        assert_eq!(retry.result.unwrap().certified().unwrap(), expected);
    }
}
