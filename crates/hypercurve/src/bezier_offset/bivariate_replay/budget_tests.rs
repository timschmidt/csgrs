use super::*;
use crate::{WorkBudget, WorkKind, evaluate_with_budget};

fn allowance(promotions: usize) -> WorkBudget {
    WorkBudget {
        refinements: 4096,
        promotions,
    }
}

fn fixture() -> ([BivariatePolynomial; 2], BezierParameter2, BezierParameter2) {
    let half = (Real::one() / Real::from(2)).unwrap();
    let first = BivariatePolynomial::new(vec![
        vec![Real::zero(), Real::one()],
        vec![],
        vec![],
        vec![-Real::one()],
    ]);
    let second = BivariatePolynomial::new(vec![vec![-half.clone(), Real::one()]]);
    let retained = BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![
                -half.clone(),
                Real::zero(),
                Real::zero(),
                Real::one(),
            ])
            .unwrap(),
            BezierParameterInterval::try_new(Real::zero(), Real::one()).unwrap(),
        )
        .unwrap(),
    );
    ([first, second], retained, BezierParameter2::Exact(half))
}

#[test]
fn cofactor_lift_cache_does_not_retain_exhaustion_and_reuses_complete_evidence() {
    let ([first, second], _, _) = fixture();
    let config = CurveIntersectionResultantConfig {
        max_resultant_degree: 0,
        ..CurveIntersectionResultantConfig::default()
    };
    let expected = linear_parameter_lifts_bivariate_polynomial_system_complete(
        &first,
        &second,
        CurveResultantParameter::First,
        config,
    );
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let mut cache = None;
        let denied = evaluate_with_budget(policy.predicate_policy(), allowance(0), || {
            assert_eq!(
                cached_linear_parameter_lifts(
                    &first,
                    &second,
                    CurveResultantParameter::First,
                    config,
                    &mut cache
                )
                .map(|_| ()),
                Err(UncertaintyReason::BudgetExhausted)
            );
        });
        assert_eq!(
            denied.result.unwrap_err().kind,
            WorkKind::AlgebraicPromotion
        );
        assert!(cache.is_none());
        let retry = evaluate_with_budget(policy.predicate_policy(), allowance(1), || {
            cached_linear_parameter_lifts(
                &first,
                &second,
                CurveResultantParameter::First,
                config,
                &mut cache,
            )
            .unwrap()
            .clone()
        });
        assert_eq!(retry.result.unwrap().certified().unwrap(), expected);
        assert_eq!(retry.usage.promotions, 1);
        let cached = evaluate_with_budget(policy.predicate_policy(), WorkBudget::ZERO, || {
            cached_linear_parameter_lifts(
                &first,
                &second,
                CurveResultantParameter::First,
                config,
                &mut cache,
            )
            .unwrap()
            .clone()
        });
        assert_eq!(cached.result.unwrap().certified().unwrap(), expected);
        assert_eq!(cached.usage, Default::default());
        let exhausted = evaluate_with_budget(policy.predicate_policy(), WorkBudget::ZERO, || {
            assert!(hypersolve::work_budget::consume(WorkKind::RootRefinement, 1).is_err());
            assert_eq!(
                cached_linear_parameter_lifts(
                    &first,
                    &second,
                    CurveResultantParameter::First,
                    config,
                    &mut cache
                )
                .map(|_| ()),
                Err(UncertaintyReason::BudgetExhausted)
            );
        });
        assert_eq!(exhausted.result.unwrap_err().kind, WorkKind::RootRefinement);
        assert_eq!(cache.as_ref(), Some(&expected));
    }
}

#[test]
fn cofactor_lift_adapter_keeps_resource_failure_distinct_and_retries_certified_pairing() {
    let ([first, second], retained, lifted) = fixture();
    let config = CurveIntersectionResultantConfig {
        max_resultant_degree: 0,
        ..CurveIntersectionResultantConfig::default()
    };
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let project = || {
            linear_parameter_lifts_bivariate_polynomial_system_complete(
                &first,
                &second,
                CurveResultantParameter::First,
                config,
            )
        };
        let mut denied_report = None;
        let denied = evaluate_with_budget(policy.predicate_policy(), allowance(0), || {
            denied_report = Some(project());
        });
        assert_eq!(
            denied.result.unwrap_err().kind,
            WorkKind::AlgebraicPromotion
        );
        // The report must preserve its resource reason after the scope ends.
        assert_eq!(
            parameter_pair_matches_linear_lift(
                &denied_report.unwrap(),
                &retained,
                &lifted,
                &policy
            )
            .unwrap(),
            Classification::Uncertain(UncertaintyReason::BudgetExhausted)
        );
        let retry = evaluate_with_budget(policy.predicate_policy(), allowance(1), || {
            parameter_pair_matches_linear_lift(&project(), &retained, &lifted, &policy).unwrap()
        });
        assert!(matches!(
            retry.result.unwrap().certified().unwrap(),
            Classification::Decided(BivariateParameterPairReplay::LinearLift(
                CurveResultantParameter::First,
                _
            ))
        ));
        assert_eq!(retry.usage.promotions, 1);
        let mut cache = [None, None];
        let exhausted = evaluate_with_budget(policy.predicate_policy(), WorkBudget::ZERO, || {
            assert!(hypersolve::work_budget::consume(WorkKind::RootRefinement, 1).is_err());
            assert_eq!(
                replay_bivariate_parameter_pair(
                    &first, &second, &retained, &lifted, &policy, config, &mut cache
                )
                .unwrap(),
                Classification::Uncertain(UncertaintyReason::BudgetExhausted)
            );
        });
        assert_eq!(exhausted.result.unwrap_err().kind, WorkKind::RootRefinement);
        assert!(cache.iter().all(Option::is_none));
    }
}
