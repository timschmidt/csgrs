use super::*;
use crate::{
    BezierParameterInterval, BezierParameterPolynomial, CurveCertainty, WorkBudget, WorkKind,
    evaluate_with_budget,
};

fn fraction(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

#[test]
fn coordinate_promotion_exhaustion_preserves_typed_failure_and_lazy_point_retries() {
    let fresh_parameter = || {
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(
                [-1, 1, 0, 1].into_iter().map(Real::from).collect(),
            )
            .unwrap(),
            BezierParameterInterval::try_new(fraction(2, 3), fraction(3, 4)).unwrap(),
        )
        .unwrap()
    };
    let curve = RationalBezier2::try_new(
        vec![
            Point2::new(Real::zero(), Real::zero()),
            Point2::new(fraction(1, 2), fraction(1, 2)),
            Point2::new(Real::from(2), Real::one()),
        ],
        vec![Real::one(); 3],
    )
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        // The reference must not prepopulate the tested parameter's point cache.
        let parameter = fresh_parameter();
        let Classification::Decided(expected) = curve
            .point_at_algebraic_parameter(&fresh_parameter(), &policy)
            .unwrap()
        else {
            panic!("the cubic parameter's coordinate image must be constructible");
        };
        let source = RationalBezierAlgebraicPointImage2::from_parametric_source(
            curve.clone(),
            parameter.clone(),
            &policy,
        );
        let denied = evaluate_with_budget(
            policy.predicate_policy(),
            WorkBudget {
                refinements: 4096,
                promotions: 0,
            },
            || {
                assert_eq!(
                    curve
                        .point_at_algebraic_parameter(&parameter, &policy)
                        .unwrap(),
                    Classification::Uncertain(UncertaintyReason::BudgetExhausted),
                );
                assert!(source.resolved(&policy).is_none());
            },
        );
        assert_eq!(
            denied.result.unwrap_err().kind,
            WorkKind::AlgebraicPromotion
        );
        assert!(source.parametric_source().unwrap().resolved.get().is_none());

        let retry = evaluate_with_budget(
            policy.predicate_policy(),
            WorkBudget {
                refinements: 4096,
                promotions: 1,
            },
            || source.resolved(&policy).cloned(),
        );
        assert_eq!(retry.usage.promotions, 1);
        let retry = retry.result.unwrap();
        assert_eq!(retry.certainty(), CurveCertainty::Certified);
        assert_eq!(retry.certified().unwrap().unwrap(), expected);

        let cached = evaluate_with_budget(policy.predicate_policy(), WorkBudget::ZERO, || {
            source.resolved(&policy).cloned()
        });
        assert_eq!(
            cached.result.unwrap().certified().unwrap().unwrap(),
            expected
        );
        assert_eq!(cached.usage.promotions, 0);
        assert_eq!(cached.usage.refinements, 0);
    }
}
