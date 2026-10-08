use super::*;
use crate::work_budget::{WorkBudget, WorkKind, with_budget};
use crate::{AlgebraicRootBinaryTransformStatus, SymbolId, transform_algebraic_roots_binary};

fn root(coefficients: &[i64], lower: Real, upper: Real) -> AlgebraicRootRepresentation {
    let mut root = AlgebraicRootRepresentation {
        constraint_index: 2,
        symbol: SymbolId(3),
        interval_index: 4,
        polynomial_coefficients: coefficients.iter().copied().map(Real::from).collect(),
        interval: IsolatedRootInterval {
            lower,
            upper,
            exact_root: None,
            distinct_root_count: 1,
        },
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    };
    root.validation = validate_algebraic_root_representation(&root, PredicatePolicy::STRICT);
    assert!(root.is_valid());
    root
}

fn fraction(numerator: i64, denominator: u64) -> Real {
    Rational::fraction(numerator, denominator).unwrap().into()
}

#[test]
fn rational_map_budget_failure_cannot_poison_elimination_or_later_selected_roots() {
    let positive = root(&[0, -2, 0, 1], Real::one(), Real::from(2));
    let negative = root(&[0, -2, 0, 1], Real::from(-2), Real::from(-1));
    let numerator = [Real::zero(), Real::one(), Real::one()];
    let denominator = [Real::one()];
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let expected =
            transform_algebraic_root_rational_image(&positive, &numerator, &denominator, policy);
        assert_eq!(
            expected.status,
            AlgebraicRootRationalImageStatus::Transformed
        );
        let map = AlgebraicRootRationalMap::new(
            &positive.polynomial_coefficients,
            &numerator,
            &denominator,
            policy,
        );
        let denied = with_budget(
            WorkBudget {
                refinements: 4096,
                promotions: 0,
            },
            || {
                let report = map.transform(&positive);
                assert_eq!(
                    report.status,
                    AlgebraicRootRationalImageStatus::BudgetExhausted
                );
                assert!(report.representation.is_none());
                assert!(report.numerator_image.is_none());
                assert!(report.denominator_image.is_none());
                assert!(report.quotient.is_none());
            },
        );
        assert_eq!(
            denied.result.unwrap_err().kind,
            WorkKind::AlgebraicPromotion
        );
        assert_eq!(denied.usage.promotions, 0);
        assert!(map.resultant_polynomial.get().is_none());

        let retry = with_budget(
            WorkBudget {
                refinements: 4096,
                promotions: 1,
            },
            || map.transform(&positive),
        );
        assert_eq!(retry.result.unwrap(), expected);
        assert_eq!(retry.usage.promotions, 1);
        assert!(map.resultant_polynomial.get().is_none());

        assert_eq!(map.transform(&positive), expected);
        assert!(map.resultant_polynomial.get().unwrap().is_some());
        for source in [&positive, &negative] {
            let expected =
                transform_algebraic_root_rational_image(source, &numerator, &denominator, policy);
            let cached = with_budget(
                WorkBudget {
                    refinements: 4096,
                    promotions: 0,
                },
                || map.transform(source),
            );
            assert_eq!(cached.result.unwrap(), expected);
            assert_eq!(cached.usage.promotions, 0);
        }
    }
}

#[test]
fn polynomial_images_share_nested_promotion_limits_and_retry_exactly() {
    let source = root(&[-2, 0, 1], Real::one(), Real::from(2));
    let image = [Real::zero(), Real::one(), Real::one()];
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let transform = || transform_algebraic_root_polynomial_image(&source, &image, policy);
        let expected = transform();
        assert_eq!(
            expected.status,
            AlgebraicRootPolynomialImageStatus::Transformed
        );
        let nested = with_budget(
            WorkBudget {
                refinements: 4096,
                promotions: 1,
            },
            || {
                let first = with_budget(
                    WorkBudget {
                        refinements: 4096,
                        promotions: 4,
                    },
                    transform,
                );
                assert_eq!(first.result.unwrap(), expected);
                let denied = transform();
                assert_eq!(
                    denied.status,
                    AlgebraicRootPolynomialImageStatus::BudgetExhausted
                );
                assert!(denied.representation.is_none());
                let constant =
                    transform_algebraic_root_polynomial_image(&source, &[Real::one()], policy);
                assert_eq!(
                    constant.status,
                    AlgebraicRootPolynomialImageStatus::BudgetExhausted
                );
                assert!(constant.representation.is_none());
            },
        );
        assert_eq!(
            nested.result.unwrap_err().kind,
            WorkKind::AlgebraicPromotion
        );
        assert_eq!(nested.usage.promotions, 1);
        let retry = with_budget(
            WorkBudget {
                refinements: 4096,
                promotions: 1,
            },
            transform,
        );
        assert_eq!(retry.result.unwrap(), expected);
    }
}

#[test]
fn binary_images_report_promotion_and_refinement_exhaustion_through_arithmetic() {
    let left = root(&[-2, 0, 1], fraction(7, 5), fraction(3, 2));
    let right = root(&[-3, 0, 1], fraction(17, 10), fraction(9, 5));
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for operation in [
            AlgebraicRootArithmeticOp::Add,
            AlgebraicRootArithmeticOp::Subtract,
            AlgebraicRootArithmeticOp::Multiply,
            AlgebraicRootArithmeticOp::Divide,
        ] {
            let transform = || transform_algebraic_roots_binary(&left, &right, operation, policy);
            let expected = transform();
            assert_eq!(
                expected.status,
                AlgebraicRootBinaryTransformStatus::Transformed
            );
            for (budget, kind) in [
                (
                    WorkBudget {
                        refinements: 4096,
                        promotions: 0,
                    },
                    WorkKind::AlgebraicPromotion,
                ),
                (
                    WorkBudget {
                        refinements: 0,
                        promotions: 1,
                    },
                    WorkKind::RootRefinement,
                ),
            ] {
                let denied = with_budget(budget, || {
                    let report = transform();
                    assert_eq!(
                        report.status,
                        AlgebraicRootBinaryTransformStatus::BudgetExhausted
                    );
                    assert!(report.representation.is_none());
                });
                assert_eq!(denied.result.unwrap_err().kind, kind);
                let arithmetic = with_budget(budget, || {
                    let report = arithmetic_algebraic_root_representations(
                        &left,
                        Some(&right),
                        operation,
                        policy,
                    );
                    assert_eq!(
                        report.status,
                        AlgebraicRootArithmeticStatus::BudgetExhausted
                    );
                    assert!(report.exact_result.is_none());
                    assert!(report.result_representation.is_none());
                });
                assert_eq!(arithmetic.result.unwrap_err().kind, kind);
            }
            let retry = with_budget(
                WorkBudget {
                    refinements: 4096,
                    promotions: 1,
                },
                transform,
            );
            assert_eq!(retry.result.unwrap(), expected);
        }
    }
}

#[test]
fn algebraic_image_budget_preserves_zero_cost_exact_and_affine_paths() {
    let source = root(&[-2, 0, 1], Real::one(), Real::from(2));
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for image in [
            vec![Real::from(7)],
            vec![Real::one(), Real::one()],
            vec![Real::zero(), Real::zero(), Real::one()],
        ] {
            let expected = transform_algebraic_root_polynomial_image(&source, &image, policy);
            assert_eq!(
                expected.status,
                AlgebraicRootPolynomialImageStatus::Transformed
            );
            let bounded = with_budget(WorkBudget::ZERO, || {
                transform_algebraic_root_polynomial_image(&source, &image, policy)
            });
            assert_eq!(bounded.result.unwrap(), expected);
            let expected =
                transform_algebraic_root_rational_image(&source, &image, &[Real::one()], policy);
            assert_eq!(
                expected.status,
                AlgebraicRootRationalImageStatus::Transformed
            );
            let bounded = with_budget(WorkBudget::ZERO, || {
                transform_algebraic_root_rational_image(&source, &image, &[Real::one()], policy)
            });
            assert_eq!(bounded.result.unwrap(), expected);
        }
    }
}

#[test]
fn rational_image_domain_refinement_exhaustion_stops_before_elimination() {
    let source = root(&[-2, 0, 1], Real::one(), Real::from(2));
    let numerator = [Real::one()];
    let denominator = [Real::from(2), Real::from(-2), Real::one()];
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let transform =
            || transform_algebraic_root_rational_image(&source, &numerator, &denominator, policy);
        let expected = transform();
        assert_eq!(
            expected.status,
            AlgebraicRootRationalImageStatus::Transformed
        );
        let denied = with_budget(WorkBudget::ZERO, || {
            let report = transform();
            assert_eq!(
                report.status,
                AlgebraicRootRationalImageStatus::BudgetExhausted
            );
            assert!(report.representation.is_none());
            assert!(report.numerator_image.is_none());
        });
        assert_eq!(denied.result.unwrap_err().kind, WorkKind::RootRefinement);
        let retry = with_budget(
            WorkBudget {
                refinements: 4096,
                promotions: 1,
            },
            transform,
        );
        assert_eq!(retry.result.unwrap(), expected);
    }
}
