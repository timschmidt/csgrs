use super::*;
use crate::project_selected_tensor_fiber_via_tagged_norm;
use crate::tensor_resultant::DenseTensorPolynomial;
use crate::work_budget::{WorkBudget, WorkKind, with_budget};

fn budget(promotions: usize) -> WorkBudget {
    WorkBudget {
        refinements: 4096,
        promotions,
    }
}

#[test]
fn quotient_norm_exhaustion_is_typed_and_both_axes_retry_exactly() {
    let first = BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-1)]]);
    let second = BivariatePolynomial::new(vec![vec![real(0), real(-1)], vec![real(1)]]);
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        // Retain a reducible carrier as well as the minimal quadratic.
        for coefficients in [
            vec![real(-1), real(0), real(2)],
            vec![real(2), real(-1), real(-4), real(2)],
        ] {
            let source = represented_root(coefficients, rational(2, 3), rational(3, 4), policy);
            for (relation, axis) in [
                (&first, CurveResultantParameter::First),
                (&second, CurveResultantParameter::Second),
            ] {
                let project =
                    || project_bivariate_fiber_at_algebraic_parameter(relation, axis, &source, 8);
                let expected = project();
                assert_eq!(expected.status, AlgebraicFiberProjectionStatus::Constructed);
                let denied = with_budget(budget(0), || {
                    let report = project();
                    assert_eq!(
                        report.status,
                        AlgebraicFiberProjectionStatus::BudgetExhausted
                    );
                    assert!(report.coefficients.is_empty());
                });
                assert_eq!(
                    denied.result.unwrap_err().kind,
                    WorkKind::AlgebraicPromotion
                );
                assert_eq!(denied.usage.promotions, 0);
                let retry = with_budget(budget(1), project);
                assert_eq!(retry.result.unwrap(), expected);
                assert_eq!(retry.usage.promotions, 1);
            }
        }
    }
}

#[test]
fn saturated_local_and_global_images_preserve_all_evidence_after_budget_retry() {
    // F=(u-alpha)(u-1)^2, G=(u-1)(z-u). The foreign u=1 component
    // must be saturated and retained explicitly in either successful report.
    let fiber = BivariatePolynomial::new(vec![
        vec![real(0), real(1), real(-2), real(1)],
        vec![real(-1), real(2), real(-1)],
    ]);
    let image = BivariatePolynomial::new(vec![
        vec![real(0), real(-1)],
        vec![real(1), real(1)],
        vec![real(-1)],
    ]);
    let config = AlgebraicFiberPolynomialImageProjectionConfig {
        max_fiber_degree: 3,
        max_retained_degree: 2,
        max_image_degree_bound: 3,
    };
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let source = represented_root(vec![real(-2), real(0), real(1)], real(1), real(2), policy);
        for project_image in [
            project_algebraic_fiber_polynomial_image,
            project_algebraic_fiber_polynomial_image_relation,
        ] {
            let project = || {
                project_image(
                    &fiber,
                    CurveResultantParameter::First,
                    &image,
                    CurveResultantParameter::Second,
                    &source,
                    config,
                    policy,
                )
            };
            let expected = project();
            assert_eq!(
                expected.status,
                AlgebraicFiberPolynomialImageProjectionStatus::Constructed
            );
            assert_eq!(expected.certainty, Certainty::Exact);
            assert!(expected.retained_relation.is_some());
            assert!(expected.identically_zero_fiber_factor.is_some());
            let denied = with_budget(budget(0), || {
                let report = project();
                assert_eq!(
                    report.status,
                    AlgebraicFiberPolynomialImageProjectionStatus::BudgetExhausted
                );
                assert!(report.coefficients.is_empty());
                assert!(report.retained_relation.is_none());
                assert!(report.identically_zero_fiber_factor.is_none());
            });
            assert_eq!(
                denied.result.unwrap_err().kind,
                WorkKind::AlgebraicPromotion
            );
            assert_eq!(denied.usage.promotions, 0);
            let retry = with_budget(budget(1), project);
            assert_eq!(retry.result.unwrap(), expected);
            assert_eq!(retry.usage.promotions, 1);
        }
    }
}

#[test]
fn degenerate_fiber_images_need_no_norm_but_cannot_escape_an_exhausted_scope() {
    let zero = BivariatePolynomial::new(vec![vec![real(0)]]);
    let constant = BivariatePolynomial::new(vec![vec![real(1)]]);
    let fiber = BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-1)]]);
    let image = BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(-1)]]);
    let config = AlgebraicFiberPolynomialImageProjectionConfig {
        max_fiber_degree: 1,
        max_retained_degree: 2,
        max_image_degree_bound: 1,
    };
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let source = represented_root(vec![real(-2), real(0), real(1)], real(1), real(2), policy);
        for project in [
            project_algebraic_fiber_polynomial_image,
            project_algebraic_fiber_polynomial_image_relation,
        ] {
            for (fiber, image, status) in [
                (
                    &zero,
                    &image,
                    AlgebraicFiberPolynomialImageProjectionStatus::IdenticallyZeroFiber,
                ),
                (
                    &constant,
                    &image,
                    AlgebraicFiberPolynomialImageProjectionStatus::ConstantNonzeroFiber,
                ),
                (
                    &fiber,
                    &zero,
                    AlgebraicFiberPolynomialImageProjectionStatus::IdenticallyZeroImageRelation,
                ),
            ] {
                let run = || {
                    project(
                        fiber,
                        CurveResultantParameter::First,
                        image,
                        CurveResultantParameter::Second,
                        &source,
                        config,
                        policy,
                    )
                };
                let ready = with_budget(WorkBudget::ZERO, run);
                assert_eq!(ready.result.unwrap().status, status);
                assert_eq!(ready.usage, Default::default());
                let denied = with_budget(WorkBudget::ZERO, || {
                    assert!(crate::work_budget::consume(WorkKind::AlgebraicPromotion, 1).is_err());
                    let report = run();
                    assert_eq!(
                        report.status,
                        AlgebraicFiberPolynomialImageProjectionStatus::BudgetExhausted
                    );
                    assert!(report.coefficients.is_empty());
                    assert!(report.retained_relation.is_none());
                });
                assert_eq!(
                    denied.result.unwrap_err().kind,
                    WorkKind::AlgebraicPromotion
                );
            }
        }
    }
}

#[test]
fn tagged_projection_shares_its_budget_with_the_final_quotient_norm() {
    // (x-y)(8t^3-1) has a finite cubic fiber at (sqrt(2),-sqrt(2)),
    // and target-wide components at the foreign tuples x=y.
    let mut coefficients = vec![real(0); 16];
    for (index, value) in [(8, -1), (11, 8), (4, 1), (7, -8)] {
        coefficients[index] = real(value);
    }
    let relation = DenseTensorPolynomial::try_new(vec![2, 2, 4], coefficients).unwrap();
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for coefficients in [
            vec![real(-2), real(0), real(1)],
            vec![
                real(-8),
                real(0),
                real(12),
                real(0),
                real(-6),
                real(0),
                real(1),
            ],
        ] {
            let positive = represented_root(coefficients.clone(), real(1), real(2), policy);
            let mut negative = represented_root(coefficients, real(-2), real(-1), policy);
            negative.constraint_index = 1;
            negative.symbol = SymbolId(1);
            let sources = [positive, negative];
            let project = || project_selected_tensor_fiber_via_tagged_norm(&relation, &sources);
            let expected = project();
            assert_eq!(expected.status, AlgebraicFiberProjectionStatus::Constructed);
            for promotions in [0, 1] {
                let denied = with_budget(budget(promotions), || {
                    let report = project();
                    assert_eq!(
                        report.status,
                        AlgebraicFiberProjectionStatus::BudgetExhausted
                    );
                    assert!(report.coefficients.is_empty());
                });
                assert_eq!(
                    denied.result.unwrap_err().kind,
                    WorkKind::AlgebraicPromotion
                );
                assert_eq!(denied.usage.promotions, promotions);
            }
            let retry = with_budget(budget(2), project);
            assert_eq!(retry.result.unwrap(), expected);
            assert_eq!(retry.usage.promotions, 2);
        }
    }
}
