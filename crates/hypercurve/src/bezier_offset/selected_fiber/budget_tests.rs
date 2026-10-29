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

#[test]
fn selected_field_import_reuses_one_authority_after_budget_retry_and_refinement() {
    use crate::bezier_offset::selected_field_fiber;
    use crate::selected_field::{self as field, FieldError};
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let construction = crate::policy::resolve_certified_operation(&policy, |context| {
            if policy == CurveContext::APPROXIMATE_512 {
                context.observe_approximate_512();
            }
            Ok::<_, CurveError>(selected_parameter(context))
        })
        .unwrap();
        assert_eq!(
            construction.certainty,
            if policy == CurveContext::APPROXIMATE_512 {
                crate::CurveCertainty::Approximate512Consumed
            } else {
                crate::CurveCertainty::Certified
            }
        );
        let parameter = construction.value;
        let denied = hypersolve::work_budget::with_budget(WorkBudget::ZERO, || {
            assert!(matches!(
                selected_field_fiber(&parameter, &policy),
                Err(FieldError::Uncertain(UncertaintyReason::BudgetExhausted))
            ));
        });
        assert_eq!(
            denied.result.unwrap_err().kind,
            WorkKind::AlgebraicPromotion
        );
        let values = selected_field_fiber(&parameter, &policy).unwrap();
        let clone = parameter.clone();
        let Classification::Decided(refined) = parameter.refined(2, &policy).unwrap() else {
            panic!("a valid fiber refinement must complete");
        };
        for source in [&parameter, &clone, &refined] {
            let warm = hypersolve::work_budget::with_budget(WorkBudget::ZERO, || {
                selected_field_fiber(source, &policy).unwrap()
            });
            let reused = warm.result.unwrap();
            assert_eq!(warm.usage.promotions, 0);
            assert!(reused[0].field().shares_authority(values[0].field()));
            assert!(reused[1].field().shares_authority(values[1].field()));
        }
        let denied = hypersolve::work_budget::with_budget(WorkBudget::ZERO, || {
            assert!(hypersolve::work_budget::consume(WorkKind::RootRefinement, 1).is_err());
            assert!(matches!(
                selected_field_fiber(&parameter, &policy),
                Err(FieldError::Uncertain(UncertaintyReason::BudgetExhausted))
            ));
        });
        assert!(denied.result.is_err());
        let opposite = if policy == CurveContext::STRICT {
            CurveContext::APPROXIMATE_512
        } else {
            CurveContext::STRICT
        };
        if policy == CurveContext::APPROXIMATE_512 {
            assert!(matches!(
                selected_field_fiber(&parameter, &opposite),
                Err(FieldError::Context(CurveError::IncompatiblePredicatePolicy))
            ));
        } else {
            let reused = selected_field_fiber(&parameter, &opposite).unwrap();
            assert!(reused[0].field().shares_authority(values[0].field()));
        }
        let translated = parameter.translated(&Real::one());
        let translated = selected_field_fiber(&translated, &policy).unwrap();
        let [original, translated] = field::joined(&values[1], &translated[1], &policy).unwrap();
        assert_eq!(
            field::decided(
                translated
                    .subtract(&original)
                    .unwrap()
                    .subtract(&original.field().constant(Real::one()).unwrap())
                    .unwrap()
                    .sign(&policy)
                    .unwrap()
            )
            .unwrap(),
            RealSign::Zero
        );
    }
}

#[test]
fn selected_field_import_concurrent_queries_share_the_retained_winner() {
    use crate::bezier_offset::selected_field_fiber;
    let parameter = selected_parameter(&CurveContext::STRICT);
    let barrier = std::sync::Barrier::new(4);
    let values = std::thread::scope(|scope| {
        let workers: Vec<_> = (0..4)
            .map(|_| {
                scope.spawn(|| {
                    barrier.wait();
                    selected_field_fiber(&parameter, &CurveContext::STRICT).unwrap()
                })
            })
            .collect();
        workers
            .into_iter()
            .map(|worker| worker.join().unwrap())
            .collect::<Vec<_>>()
    });
    for pair in values.windows(2) {
        assert!(pair[0][0].field().shares_authority(pair[1][0].field()));
        assert!(pair[0][1].field().shares_authority(pair[1][1].field()));
    }
}
