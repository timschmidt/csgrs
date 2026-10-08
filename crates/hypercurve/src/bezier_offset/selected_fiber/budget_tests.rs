use super::*;
use crate::{WorkBudget, WorkKind, evaluate_with_budget};

fn fraction(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn selected_parameter(policy: &CurveContext) -> BezierAlgebraicSelectedFiberParameter2 {
    let alpha = BezierAlgebraicParameter2::try_isolate(
        BezierParameterPolynomial::try_new_power_basis(vec![
            -fraction(1, 2),
            Real::zero(),
            Real::one(),
        ])
        .unwrap(),
        BezierParameterInterval::try_new(fraction(2, 3), fraction(3, 4)).unwrap(),
    )
    .unwrap();
    // (u-alpha)(u+1)^2, selecting u=alpha. The repeated foreign component
    // must remain explicit when it is saturated out of an image relation.
    let interval = IsolatedRootInterval {
        lower: alpha.interval().start().clone(),
        upper: alpha.interval().end().clone(),
        exact_root: None,
        distinct_root_count: 1,
    };
    BezierAlgebraicSelectedFiberAuthority2::new(
        BivariatePolynomial::new(vec![
            vec![Real::zero(), Real::one(), Real::from(2), Real::one()],
            vec![Real::from(-1), Real::from(-2), Real::from(-1)],
        ]),
        alpha,
        policy,
    )
    .parameter(interval)
}

#[test]
fn selected_image_helpers_stop_on_exhaustion_and_retry_with_exact_saturation_evidence() {
    let image = BivariatePolynomial::new(vec![
        vec![Real::zero(), Real::one()],
        vec![Real::from(-1), Real::one()],
        vec![Real::from(-1)],
    ]);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let reference = selected_parameter(&policy);
        let expected_eliminant = reference
            .represented_polynomial_image_eliminant(&image, &policy)
            .unwrap();
        assert!(matches!(
            expected_eliminant,
            Classification::Decided(Some(_))
        ));
        let Classification::Decided(Some(expected_relation)) = reference
            .retained_polynomial_image_relation(&image, &policy)
            .unwrap()
        else {
            panic!("the unbudgeted retained image must construct");
        };
        for retained_relation in [false, true] {
            let parameter = selected_parameter(&policy);
            let denied = evaluate_with_budget(policy.predicate_policy(), WorkBudget::ZERO, || {
                if retained_relation {
                    assert!(matches!(
                        parameter
                            .retained_polynomial_image_relation(&image, &policy)
                            .unwrap(),
                        Classification::Uncertain(UncertaintyReason::BudgetExhausted)
                    ));
                } else {
                    assert_eq!(
                        parameter
                            .represented_polynomial_image_eliminant(&image, &policy)
                            .unwrap(),
                        Classification::Uncertain(UncertaintyReason::BudgetExhausted)
                    );
                }
            });
            assert_eq!(
                denied.result.unwrap_err().kind,
                WorkKind::AlgebraicPromotion
            );
            assert_eq!(denied.usage.promotions, 0);
            let retry = evaluate_with_budget(
                policy.predicate_policy(),
                WorkBudget {
                    refinements: 4096,
                    promotions: 1,
                },
                || {
                    if retained_relation {
                        let Classification::Decided(Some(actual)) = parameter
                            .retained_polynomial_image_relation(&image, &policy)
                            .unwrap()
                        else {
                            panic!("a larger budget must retain the exact image");
                        };
                        assert_eq!(actual.relation, expected_relation.relation);
                        assert_eq!(actual.global_schedule, expected_relation.global_schedule);
                        assert_eq!(
                            actual.identically_zero_source_factor,
                            expected_relation.identically_zero_source_factor
                        );
                        assert_eq!(
                            actual.identically_zero_image_relation,
                            expected_relation.identically_zero_image_relation
                        );
                    } else {
                        assert_eq!(
                            parameter
                                .represented_polynomial_image_eliminant(&image, &policy)
                                .unwrap(),
                            expected_eliminant
                        );
                    }
                },
            );
            assert!(retry.result.unwrap().certified().is_some());
            assert_eq!(retry.usage.promotions, 1);
        }
    }
}
