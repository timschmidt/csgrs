use super::*;
use crate::work_budget::{WorkBudget, WorkKind, consume, with_budget};

fn allowance(promotions: usize) -> WorkBudget {
    WorkBudget {
        refinements: 4096,
        promotions,
    }
}

fn config(max_resultant_degree: usize) -> CurveIntersectionResultantConfig {
    CurveIntersectionResultantConfig {
        min_precision: -512,
        max_resultant_degree,
    }
}

fn quadratic_fiber(axis: TrivariatePolynomialAxis) -> TrivariatePolynomial {
    let mut coefficients = vec![vec![vec![real(0); 3]; 3]; 3];
    for index in 0..3 {
        let mut exponents = [0; 3];
        exponents[index] = if index == axis.index() { 2 } else { 1 };
        coefficients[exponents[0]][exponents[1]][exponents[2]] = if index == axis.index() {
            real(1)
        } else {
            real(-1)
        };
    }
    TrivariatePolynomial::new(coefficients)
}

fn cubic_constraint(axis: TrivariatePolynomialAxis) -> TrivariatePolynomial {
    let mut coefficients = vec![vec![vec![real(0); 4]; 4]; 4];
    coefficients[0][0][0] = real(-2);
    let mut exponents = [0; 3];
    exponents[axis.index()] = 3;
    coefficients[exponents[0]][exponents[1]][exponents[2]] = real(1);
    TrivariatePolynomial::new(coefficients)
}

fn quadratic_fourth_axis_fiber() -> QuadrivariatePolynomial {
    let mut coefficients = vec![vec![vec![vec![real(0); 3]; 2]; 2]; 2];
    coefficients[0][0][0][2] = real(1);
    coefficients[1][0][0][0] = real(-1);
    coefficients[0][1][0][0] = real(-1);
    coefficients[0][0][1][0] = real(-1);
    QuadrivariatePolynomial::new(coefficients)
}

fn assert_admission<R: std::fmt::Debug + PartialEq>(
    project: impl Fn(CurveIntersectionResultantConfig) -> R,
    assert_denied: impl Fn(&R),
    assert_degree_limited: impl Fn(&R),
) {
    let expected = project(config(32));
    let degree_limited = with_budget(WorkBudget::ZERO, || project(config(0)));
    assert_degree_limited(&degree_limited.result.unwrap());
    assert_eq!(degree_limited.usage.promotions, 0);
    let denied = with_budget(allowance(0), || assert_denied(&project(config(32))));
    assert_eq!(
        denied.result.unwrap_err().kind,
        WorkKind::AlgebraicPromotion
    );
    assert_eq!(denied.usage.promotions, 0);
    let retry = with_budget(allowance(1), || project(config(32)));
    assert_eq!(retry.result.unwrap(), expected);
    assert_eq!(retry.usage.promotions, 1);
    let already_exhausted = with_budget(WorkBudget::ZERO, || {
        assert!(consume(WorkKind::RootRefinement, 1).is_err());
        assert_denied(&project(config(0)));
    });
    assert_eq!(
        already_exhausted.result.unwrap_err().kind,
        WorkKind::RootRefinement
    );
}

fn assert_terminal_reuse<R: std::fmt::Debug + PartialEq>(
    project: impl Fn() -> R,
    assert_denied: impl Fn(&R),
) {
    let expected = project();
    let reused = with_budget(WorkBudget::ZERO, &project);
    assert_eq!(reused.result.unwrap(), expected);
    assert_eq!(reused.usage, Default::default());
    let denied = with_budget(WorkBudget::ZERO, || {
        assert!(consume(WorkKind::RootRefinement, 1).is_err());
        assert_denied(&project());
    });
    assert_eq!(denied.result.unwrap_err().kind, WorkKind::RootRefinement);
}

#[test]
fn fixed_rank_norms_preserve_both_retained_dimensions_on_exact_budget_retry() {
    let constraint = [real(-2), real(0), real(0), real(1)];
    for axis in [
        TrivariatePolynomialAxis::First,
        TrivariatePolynomialAxis::Second,
        TrivariatePolynomialAxis::Third,
    ] {
        let fiber = quadratic_fiber(axis);
        let project = |config| {
            resultant_trivariate_polynomial_univariate_constraint(&fiber, &constraint, axis, config)
        };
        assert_eq!(
            project(config(32)).status,
            TrivariateConstraintResultantStatus::Constructed
        );
        assert_admission(
            project,
            |report| {
                assert_eq!(
                    report.status,
                    TrivariateConstraintResultantStatus::BudgetExhausted
                );
                assert!(report.resultant.is_none());
                assert!(report.resultant_error.is_none());
            },
            |report| {
                assert_eq!(
                    report.status,
                    TrivariateConstraintResultantStatus::DegreeBoundExceeded
                )
            },
        );
    }
    let fiber = quadratic_fourth_axis_fiber();
    let project = |config| {
        resultant_quadrivariate_polynomial_fourth_axis_constraint(&fiber, &constraint, config)
    };
    assert_eq!(
        project(config(32)).status,
        TrivariateConstraintResultantStatus::Constructed
    );
    assert_admission(
        project,
        |report| {
            assert_eq!(
                report.status,
                TrivariateConstraintResultantStatus::BudgetExhausted
            );
            assert!(report.resultant.is_none());
            assert!(report.resultant_error.is_none());
        },
        |report| {
            assert_eq!(
                report.status,
                TrivariateConstraintResultantStatus::DegreeBoundExceeded
            )
        },
    );
}

#[test]
fn fixed_rank_subresultants_charge_interpolation_and_reuse_terminal_operands() {
    let constraint = [real(-2), real(0), real(0), real(1)];
    for axis in [
        TrivariatePolynomialAxis::First,
        TrivariatePolynomialAxis::Second,
        TrivariatePolynomialAxis::Third,
    ] {
        let fiber = quadratic_fiber(axis);
        let project = |config| {
            subresultant_trivariate_polynomial_univariate_constraint(
                &fiber,
                &constraint,
                axis,
                1,
                config,
            )
        };
        assert_eq!(
            project(config(32)).status,
            TrivariateConstraintSubresultantStatus::Constructed
        );
        assert_admission(
            project,
            |report| {
                assert_eq!(
                    report.status,
                    TrivariateConstraintSubresultantStatus::BudgetExhausted
                );
                assert!(report.coefficients.is_empty());
                assert!(report.determinant_error.is_none());
            },
            |report| {
                assert_eq!(
                    report.status,
                    TrivariateConstraintSubresultantStatus::DegreeBoundExceeded
                )
            },
        );
        let terminal = || {
            subresultant_trivariate_polynomial_univariate_constraint(
                &fiber,
                &constraint,
                axis,
                2,
                config(32),
            )
        };
        let expected = terminal();
        assert_eq!(
            expected.status,
            TrivariateConstraintSubresultantStatus::Constructed
        );
        assert_terminal_reuse(terminal, |report| {
            assert_eq!(
                report.status,
                TrivariateConstraintSubresultantStatus::BudgetExhausted
            );
            assert!(report.coefficients.is_empty());
        });

        let second = cubic_constraint(axis);
        for order in [0, 1] {
            let project = |config| {
                subresultant_trivariate_polynomial_system(&fiber, &second, axis, order, config)
            };
            assert_eq!(
                project(config(32)).status,
                TrivariatePolynomialSystemSubresultantStatus::Constructed
            );
            assert_admission(
                project,
                |report| {
                    assert_eq!(
                        report.status,
                        TrivariatePolynomialSystemSubresultantStatus::BudgetExhausted
                    );
                    assert!(report.coefficients.is_empty());
                    assert!(report.determinant_error.is_none());
                },
                |report| {
                    assert_eq!(
                        report.status,
                        TrivariatePolynomialSystemSubresultantStatus::DegreeBoundExceeded
                    )
                },
            );
        }
        let terminal =
            || subresultant_trivariate_polynomial_system(&fiber, &second, axis, 2, config(32));
        let expected = terminal();
        assert_eq!(
            expected.status,
            TrivariatePolynomialSystemSubresultantStatus::Constructed
        );
        assert_terminal_reuse(terminal, |report| {
            assert_eq!(
                report.status,
                TrivariatePolynomialSystemSubresultantStatus::BudgetExhausted
            );
            assert!(report.coefficients.is_empty());
        });
    }
    let fiber = quadratic_fourth_axis_fiber();
    let project = |config| {
        subresultant_quadrivariate_polynomial_fourth_axis_constraint(&fiber, &constraint, 1, config)
    };
    assert_eq!(
        project(config(32)).status,
        TrivariateConstraintSubresultantStatus::Constructed
    );
    assert_admission(
        project,
        |report| {
            assert_eq!(
                report.status,
                TrivariateConstraintSubresultantStatus::BudgetExhausted
            );
            assert!(report.coefficients.is_empty());
            assert!(report.determinant_error.is_none());
        },
        |report| {
            assert_eq!(
                report.status,
                TrivariateConstraintSubresultantStatus::DegreeBoundExceeded
            )
        },
    );
    let terminal = || {
        subresultant_quadrivariate_polynomial_fourth_axis_constraint(
            &fiber,
            &constraint,
            2,
            config(32),
        )
    };
    let expected = terminal();
    assert_eq!(
        expected.status,
        TrivariateConstraintSubresultantStatus::Constructed
    );
    assert_terminal_reuse(terminal, |report| {
        assert_eq!(
            report.status,
            TrivariateConstraintSubresultantStatus::BudgetExhausted
        );
        assert!(report.coefficients.is_empty());
    });
}

#[test]
fn constrained_fiber_gcd_orders_cannot_reset_the_enclosing_allowance() {
    let axis = TrivariatePolynomialAxis::Third;
    let fiber = quadratic_fiber(axis);
    let constraint = [real(-2), real(0), real(0), real(1)];
    let norm = || {
        resultant_trivariate_polynomial_univariate_constraint(&fiber, &constraint, axis, config(32))
    };
    let remainder = || {
        subresultant_trivariate_polynomial_univariate_constraint(
            &fiber,
            &constraint,
            axis,
            1,
            config(32),
        )
    };
    let expected = (norm(), remainder());
    let denied = with_budget(allowance(1), || {
        assert_eq!(norm(), expected.0);
        let inner = with_budget(allowance(8), || {
            let report = remainder();
            assert_eq!(
                report.status,
                TrivariateConstraintSubresultantStatus::BudgetExhausted
            );
            assert!(report.coefficients.is_empty());
        });
        assert_eq!(inner.result.unwrap_err().used, 1);
    });
    assert_eq!(
        denied.result.unwrap_err().kind,
        WorkKind::AlgebraicPromotion
    );
    assert_eq!(denied.usage.promotions, 1);
    let retry = with_budget(allowance(2), || (norm(), remainder()));
    assert_eq!(retry.result.unwrap(), expected);
    assert_eq!(retry.usage.promotions, 2);
}
