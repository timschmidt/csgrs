use super::*;
use crate::tensor_resultant::{
    DenseTensorPolynomial, TensorConstraintResultantStatus,
    resultant_tensor_polynomial_univariate_constraint,
};
use crate::work_budget::{WorkBudget, WorkKind, with_budget};

fn budget(promotions: usize) -> WorkBudget {
    WorkBudget {
        refinements: 4096,
        promotions,
    }
}

#[test]
fn bivariate_degree_continuation_uses_one_shared_promotion_and_retries_exactly() {
    let first = BivariatePolynomial::new(vec![vec![real(0), real(-1)], vec![real(1)]]);
    let second = BivariatePolynomial::new(vec![vec![real(-1)], vec![real(0)], vec![real(1)]]);
    let zero = BivariatePolynomial::new(vec![vec![real(0)]]);
    let config = CurveIntersectionResultantConfig {
        min_precision: -512,
        max_resultant_degree: 0,
    };
    for axis in [
        CurveResultantParameter::First,
        CurveResultantParameter::Second,
    ] {
        let project =
            || resultant_bivariate_polynomial_system_complete(&first, &second, axis, config);
        let expected = project();
        assert_eq!(
            expected.status,
            CurveIntersectionResultantStatus::Constructed
        );
        assert!(expected.degree_bound > config.max_resultant_degree);
        let degree_limited = with_budget(WorkBudget::ZERO, || {
            resultant_bivariate_polynomial_system(&first, &second, axis, config)
        });
        assert_eq!(
            degree_limited.result.unwrap().status,
            CurveIntersectionResultantStatus::DegreeBoundExceeded
        );
        assert_eq!(degree_limited.usage.promotions, 0);
        let degenerate = with_budget(WorkBudget::ZERO, || {
            resultant_bivariate_polynomial_system(&first, &zero, axis, config)
        });
        assert_eq!(
            degenerate.result.unwrap().resultant_coefficients,
            vec![Real::zero()]
        );
        let denied = with_budget(budget(0), || {
            let report = project();
            assert_eq!(
                report.status,
                CurveIntersectionResultantStatus::BudgetExhausted
            );
            assert!(report.resultant_coefficients.is_empty());
            assert!(report.resultant_error.is_none());
            // A later zero equation cannot publish a result from the exhausted scope.
            assert_eq!(
                resultant_bivariate_polynomial_system(&first, &zero, axis, config).status,
                CurveIntersectionResultantStatus::BudgetExhausted
            );
        });
        assert_eq!(
            denied.result.unwrap_err().kind,
            WorkKind::AlgebraicPromotion
        );
        let retry = with_budget(budget(1), project);
        assert_eq!(retry.result.unwrap(), expected);
        assert_eq!(retry.usage.promotions, 1);
        let nested = with_budget(budget(1), || {
            assert_eq!(with_budget(budget(8), project).result.unwrap(), expected);
            assert_eq!(
                project().status,
                CurveIntersectionResultantStatus::BudgetExhausted
            );
        });
        assert_eq!(nested.result.unwrap_err().used, 1);
        assert_eq!(nested.usage.promotions, 1);
    }
}

#[test]
fn polynomial_and_rational_curve_wrappers_preserve_budget_failure_and_exact_reports() {
    let parabola =
        PolynomialParametricCurve2::new(vec![real(0), real(1)], vec![real(0), real(0), real(1)]);
    let horizontal = PolynomialParametricCurve2::new(vec![real(0), real(1)], vec![real(1)]);
    let rational_parabola = RationalParametricCurve2::from_polynomial(&parabola);
    let rational_horizontal = RationalParametricCurve2::from_polynomial(&horizontal);
    for axis in [
        CurveResultantParameter::First,
        CurveResultantParameter::Second,
    ] {
        for rational in [false, true] {
            let project = || {
                if rational {
                    resultant_rational_parametric_curve_intersection(
                        &rational_parabola,
                        &rational_horizontal,
                        axis,
                        CurveIntersectionResultantConfig::default(),
                    )
                } else {
                    resultant_parametric_curve_intersection(
                        &parabola,
                        &horizontal,
                        axis,
                        CurveIntersectionResultantConfig::default(),
                    )
                }
            };
            let expected = project();
            assert_eq!(
                expected.status,
                CurveIntersectionResultantStatus::Constructed
            );
            let denied = with_budget(budget(0), || {
                for _ in 0..2 {
                    let report = project();
                    assert_eq!(
                        report.status,
                        CurveIntersectionResultantStatus::BudgetExhausted
                    );
                    assert!(report.resultant_coefficients.is_empty());
                }
            });
            assert_eq!(
                denied.result.unwrap_err().kind,
                WorkKind::AlgebraicPromotion
            );
            let retry = with_budget(budget(1), project);
            assert_eq!(retry.result.unwrap(), expected);
            assert_eq!(retry.usage.promotions, 1);
        }
    }
}

#[test]
fn tensor_axis_resultants_share_limits_and_preserve_the_correlated_product() {
    // t-x*y, with x^2=2 and y^2=3. Complete elimination retains (t^2-6)^2.
    let mut coefficients = vec![real(0); 8];
    coefficients[1] = real(1);
    coefficients[6] = real(-1);
    let relation = DenseTensorPolynomial::try_new(vec![2, 2, 2], coefficients).unwrap();
    let first_constraint = [real(-2), real(0), real(1)];
    let second_constraint = [real(-3), real(0), real(1)];
    for axis in 0..3 {
        let project = || {
            resultant_tensor_polynomial_univariate_constraint(
                &relation,
                &first_constraint,
                axis,
                -512,
            )
        };
        let expected = project();
        assert_eq!(
            expected.status,
            TensorConstraintResultantStatus::Constructed
        );
        let denied = with_budget(budget(0), || {
            let report = project();
            assert_eq!(
                report.status,
                TensorConstraintResultantStatus::BudgetExhausted
            );
            assert!(report.resultant.is_none());
            assert!(report.resultant_error.is_none());
        });
        assert_eq!(
            denied.result.unwrap_err().kind,
            WorkKind::AlgebraicPromotion
        );
        let retry = with_budget(budget(1), project);
        assert_eq!(retry.result.unwrap(), expected);
        assert_eq!(retry.usage.promotions, 1);
    }
    let first =
        resultant_tensor_polynomial_univariate_constraint(&relation, &first_constraint, 0, -512);
    let first_tensor = first.resultant.as_ref().unwrap();
    let expected = resultant_tensor_polynomial_univariate_constraint(
        first_tensor,
        &second_constraint,
        0,
        -512,
    );
    assert_eq!(
        expected.resultant.as_ref().unwrap().coefficients(),
        &[real(36), real(0), real(-12), real(0), real(1)]
    );
    let project = || {
        let first = resultant_tensor_polynomial_univariate_constraint(
            &relation,
            &first_constraint,
            0,
            -512,
        );
        resultant_tensor_polynomial_univariate_constraint(
            first.resultant.as_ref().unwrap(),
            &second_constraint,
            0,
            -512,
        )
    };
    let denied = with_budget(budget(1), || {
        let report = project();
        assert_eq!(
            report.status,
            TensorConstraintResultantStatus::BudgetExhausted
        );
        assert!(report.resultant.is_none());
    });
    assert_eq!(denied.result.unwrap_err().used, 1);
    assert_eq!(denied.usage.promotions, 1);
    let retry = with_budget(budget(2), project);
    assert_eq!(retry.result.unwrap(), expected);
    assert_eq!(retry.usage.promotions, 2);
}
