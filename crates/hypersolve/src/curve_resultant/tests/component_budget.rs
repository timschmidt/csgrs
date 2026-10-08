use super::*;
use crate::bivariate_components::{
    extract_bivariate_system_components, parameter_component_bivariate_polynomial_system_complete,
};
use crate::trivariate_arithmetic::*;
use crate::work_budget::{WorkBudget, WorkKind, consume, with_budget};

fn allowance(promotions: usize) -> WorkBudget {
    WorkBudget {
        refinements: 4096,
        promotions,
    }
}

fn polynomial(rows: &[&[i64]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|r| r.iter().copied().map(real).collect())
            .collect(),
    )
}

fn assert_denied(report: &BivariatePolynomialComponentReport) {
    assert_eq!(
        report.status,
        BivariatePolynomialComponentStatus::BudgetExhausted
    );
    assert!(report.numerator_coefficients.is_empty());
    assert!(report.denominator_coefficients.is_empty());
    assert!(report.implicit_component.is_none());
    assert!(report.reduced_equations.is_none());
    assert!(report.determinant_error.is_none());
}

#[test]
fn common_components_retry_complete_rational_and_implicit_evidence_on_both_axes() {
    for component in [
        polynomial(&[&[0, 2], &[-1, 1], &[-1]]),
        polynomial(&[&[-1, 0, 1], &[0, -1]]),
    ] {
        let first = multiply_bivariate(&component, &polynomial(&[&[1, 1]]));
        let second = multiply_bivariate(&component, &polynomial(&[&[3, 1], &[1]]));
        for axis in [
            CurveResultantParameter::First,
            CurveResultantParameter::Second,
        ] {
            let config = CurveIntersectionResultantConfig {
                max_resultant_degree: 0,
                ..Default::default()
            };
            let project = || {
                parameter_component_bivariate_polynomial_system_complete(
                    &first, &second, axis, config,
                )
            };
            let expected = project();
            assert!(matches!(
                expected.status,
                BivariatePolynomialComponentStatus::Rational
                    | BivariatePolynomialComponentStatus::Implicit
            ));
            let capped =
                parameter_component_bivariate_polynomial_system(&first, &second, axis, config);
            if capped.status == BivariatePolynomialComponentStatus::DegreeBoundExceeded {
                let bounded = with_budget(WorkBudget::ZERO, || {
                    parameter_component_bivariate_polynomial_system(&first, &second, axis, config)
                });
                assert_eq!(bounded.result.unwrap(), capped);
                assert_eq!(bounded.usage, Default::default());
            } else {
                // A linear input is reused directly; it needs no interpolation degree admission.
                assert_eq!(capped, expected);
            }
            let denied = with_budget(allowance(0), || assert_denied(&project()));
            assert_eq!(
                denied.result.unwrap_err().kind,
                WorkKind::AlgebraicPromotion
            );
            let retry = with_budget(allowance(1), project);
            assert_eq!(retry.result.unwrap(), expected);
            assert_eq!(retry.usage.promotions, 1);
        }
    }
}

#[test]
fn zero_equation_components_and_nested_scopes_do_not_reset_or_double_charge() {
    let zero = polynomial(&[&[0]]);
    let line = polynomial(&[&[0, 1], &[-1]]);
    let config = CurveIntersectionResultantConfig::default();
    let project = |first, second| {
        parameter_component_bivariate_polynomial_system(
            first,
            second,
            CurveResultantParameter::First,
            config,
        )
    };
    let expected = project(&zero, &line);
    assert_eq!(
        expected.status,
        BivariatePolynomialComponentStatus::Rational
    );
    assert_eq!(expected.reduced_equations.as_ref().unwrap()[0], zero);
    let denied = with_budget(allowance(1), || {
        assert_eq!(project(&zero, &line), expected);
        let child = with_budget(allowance(8), || assert_denied(&project(&line, &zero)));
        assert_eq!(child.result.unwrap_err().used, 1);
    });
    assert_eq!(
        denied.result.unwrap_err().kind,
        WorkKind::AlgebraicPromotion
    );
    assert_eq!(denied.usage.promotions, 1);
    let retry = with_budget(allowance(2), || {
        [project(&zero, &line), project(&line, &zero)]
    });
    assert_eq!(retry.result.unwrap(), [expected, project(&line, &zero)]);
    assert_eq!(retry.usage.promotions, 2);
    let trivial = with_budget(WorkBudget::ZERO, || project(&zero, &zero));
    assert_eq!(
        trivial.result.unwrap().status,
        BivariatePolynomialComponentStatus::NoSupportedComponent
    );
    let exhausted = with_budget(WorkBudget::ZERO, || {
        assert!(consume(WorkKind::RootRefinement, 1).is_err());
        assert_denied(&project(&zero, &zero));
        assert_eq!(
            extract_bivariate_polynomial_system_axis_factors(&zero, &zero).status,
            BivariatePolynomialAxisFactorStatus::BudgetExhausted
        );
    });
    assert_eq!(exhausted.result.unwrap_err().kind, WorkKind::RootRefinement);
}

#[test]
fn axis_factor_extraction_and_normalized_retry_preserve_authored_residuals() {
    let first = polynomial(&[&[1, 2], &[3]]);
    let second = polynomial(&[&[4], &[5, 6]]);
    for scaled in [false, true] {
        let mut equations = [
            multiply_axis_factors(&first, &[real(-2), real(1)], &[real(3), real(1)]),
            multiply_axis_factors(&second, &[real(-2), real(1)], &[real(3), real(1)]),
        ];
        if scaled {
            scale_bivariate_in_place(&mut equations[0], &real(2).sqrt().unwrap());
            scale_bivariate_in_place(&mut equations[1], &real(3).sqrt().unwrap());
        }
        let project =
            || extract_bivariate_polynomial_system_axis_factors(&equations[0], &equations[1]);
        let expected = project();
        assert_eq!(
            expected.status,
            BivariatePolynomialAxisFactorStatus::Reduced
        );
        let required = 1;
        for units in 0..required {
            let denied = with_budget(allowance(units), || {
                let report = project();
                assert_eq!(
                    report.status,
                    BivariatePolynomialAxisFactorStatus::BudgetExhausted
                );
                assert!(report.first_parameter_factor.is_empty());
                assert!(report.second_parameter_factor.is_empty());
                assert!(report.reduced_equations.is_none());
            });
            assert_eq!(
                denied.result.unwrap_err().kind,
                WorkKind::AlgebraicPromotion
            );
            assert_eq!(denied.usage.promotions, units);
        }
        let retry = with_budget(allowance(required), project);
        assert_eq!(retry.result.unwrap(), expected);
        assert_eq!(retry.usage.promotions, required);
        if scaled {
            // Exercise the cold projective retry explicitly: this fixture is
            // already solvable by the initial field-GCD route on current Real forms.
            let retry = || {
                retry_bivariate_polynomial_system_axis_factors(&equations[0], &equations[1])
                    .unwrap()
            };
            let normalized = retry();
            assert_eq!(
                normalized.status,
                BivariatePolynomialAxisFactorStatus::Reduced
            );
            let denied = with_budget(allowance(1), || {
                consume(WorkKind::AlgebraicPromotion, 1).unwrap();
                assert_eq!(
                    retry().status,
                    BivariatePolynomialAxisFactorStatus::BudgetExhausted
                );
            });
            assert_eq!(denied.result.unwrap_err().used, 1);
            let admitted = with_budget(allowance(2), || {
                consume(WorkKind::AlgebraicPromotion, 1).unwrap();
                retry()
            });
            assert_eq!(admitted.result.unwrap(), normalized);
            assert_eq!(admitted.usage.promotions, 2);
        }
        for (residual, authored) in expected.reduced_equations.unwrap().iter().zip(equations) {
            assert_bivariate_exactly_equal(
                &multiply_axis_factors(
                    residual,
                    &expected.first_parameter_factor,
                    &expected.second_parameter_factor,
                ),
                &authored,
            );
        }
    }
    let zero = polynomial(&[&[0]]);
    let trivial = with_budget(WorkBudget::ZERO, || {
        extract_bivariate_polynomial_system_axis_factors(&zero, &zero)
    });
    assert_eq!(
        trivial.result.unwrap().status,
        BivariatePolynomialAxisFactorStatus::ZeroSystem
    );
    assert_eq!(trivial.usage, Default::default());
}

#[test]
fn rootless_axis_saturation_distinguishes_exhaustion_from_a_rootful_factor() {
    let factor = polynomial(&[&[-2], &[1]]);
    let equations = [
        multiply_bivariate(&factor, &polynomial(&[&[1, 1]])),
        multiply_bivariate(&factor, &polynomial(&[&[2, 1]])),
    ];
    let denied = with_budget(allowance(0), || {
        assert_eq!(
            saturate_rootless_bivariate_axis_factors(&equations, [[&real(0), &real(1)]; 2]),
            Classification::Uncertain(UncertaintyReason::BudgetExhausted)
        );
    });
    assert!(denied.result.is_err());
    for (upper, rootless) in [(1, true), (3, false)] {
        let retry = with_budget(allowance(1), || {
            saturate_rootless_bivariate_axis_factors(&equations, [[&real(0), &real(upper)]; 2])
        });
        let Classification::Decided(reduced) = retry.result.unwrap() else {
            panic!("saturation must be decided");
        };
        assert_eq!(reduced.is_some(), rootless);
        assert_eq!(retry.usage.promotions, 1);
    }
}

#[test]
fn complete_component_extraction_cannot_publish_support_before_budgeted_residual_work() {
    let factor = polynomial(&[&[-2], &[1]]);
    let equations = [
        multiply_bivariate(&factor, &polynomial(&[&[0, 1], &[-1]])),
        multiply_bivariate(&factor, &polynomial(&[&[1, 1]])),
    ];
    let project = || {
        extract_bivariate_system_components(
            equations.clone(),
            CurveIntersectionResultantConfig::default(),
        )
    };
    let denied = with_budget(allowance(1), || {
        assert!(matches!(
            project(),
            Classification::Uncertain(UncertaintyReason::BudgetExhausted)
        ))
    });
    assert!(denied.result.is_err());
    assert_eq!(denied.usage.promotions, 1);
    let Classification::Decided(expected) = project() else {
        panic!("exact support must be retained");
    };
    let retry = with_budget(allowance(16), project);
    let Classification::Decided(actual) = retry.result.unwrap() else {
        panic!("larger budget must retain support");
    };
    assert_eq!(actual.support, expected.support);
    assert_eq!(actual.residual_equations, expected.residual_equations);
    assert!(actual.support.is_some());
}

#[test]
fn trivariate_factor_normalization_discards_partial_content_and_retries_exact_division() {
    let factor = [polynomial(&[&[1, 1], &[1]]), polynomial(&[&[1]])];
    let cofactor = [polynomial(&[&[2, 2], &[1]]), polynomial(&[&[1]])];
    let content = polynomial(&[&[-1, 0, 1], &[0, -1]]);
    let raw = factor.each_ref().map(|f| multiply_bivariate(&content, f));
    for axis in 0..3 {
        let remaining: [usize; 2] = (0..3)
            .filter(|a| *a != axis)
            .collect::<Vec<_>>()
            .try_into()
            .unwrap();
        let polynomial = TrivariatePolynomial::multiply(
            &trivariate_from_axis_bivariate_coefficients(&factor, axis, remaining).unwrap(),
            &trivariate_from_axis_bivariate_coefficients(&cofactor, axis, remaining).unwrap(),
        )
        .unwrap();
        let normalize = || {
            trivariate_normalize_and_divide_linear_axis_factor(
                &polynomial,
                axis,
                raw.clone(),
                CurveIntersectionResultantConfig::default(),
            )
        };
        let expected = normalize().unwrap().unwrap();
        for units in 0..2 {
            let denied = with_budget(allowance(units), || {
                assert_eq!(normalize().unwrap_err().kind, WorkKind::AlgebraicPromotion)
            });
            assert_eq!(denied.result.unwrap_err().used, units);
        }
        let retry = with_budget(allowance(2), normalize);
        let actual = retry.result.unwrap().unwrap().unwrap();
        assert_eq!(actual, expected);
        let recovered =
            trivariate_from_axis_bivariate_coefficients(&actual.0, axis, remaining).unwrap();
        assert_eq!(
            TrivariatePolynomial::multiply(&recovered, &actual.1).unwrap(),
            polynomial
        );
        let ready = with_budget(WorkBudget::ZERO, || {
            trivariate_normalize_and_divide_linear_axis_factor(
                &polynomial,
                axis,
                factor.clone(),
                CurveIntersectionResultantConfig::default(),
            )
        });
        assert!(ready.result.unwrap().unwrap().is_some());
        assert_eq!(ready.usage, Default::default());
        let exhausted = with_budget(WorkBudget::ZERO, || {
            assert!(consume(WorkKind::RootRefinement, 1).is_err());
            assert_eq!(normalize().unwrap_err().kind, WorkKind::RootRefinement);
            assert!(
                trivariate_quadratic_axis_factorizations(&polynomial, axis, Default::default())
                    .is_err()
            );
            assert!(
                trivariate_rational_multi_affine_axis_factorizations(
                    &polynomial,
                    axis,
                    Default::default()
                )
                .is_err()
            );
        });
        assert!(exhausted.result.is_err());
    }
}
