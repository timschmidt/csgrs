use super::*;
use crate::work_budget::{WorkBudget, WorkKind, consume, with_budget};

fn allowance(promotions: usize) -> WorkBudget {
    WorkBudget {
        refinements: 4096,
        promotions,
    }
}

fn equations(degree_drop: bool) -> [BivariatePolynomial; 2] {
    [
        if degree_drop {
            BivariatePolynomial::new(vec![
                vec![real(0), real(1), real(-1)],
                vec![real(-1), real(0), real(1)],
            ])
        } else {
            BivariatePolynomial::new(vec![vec![real(0), real(0), real(1)], vec![real(-1)]])
        },
        BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![], vec![real(-1)]]),
    ]
}

fn assert_denied(report: &CurveIntersectionParameterLiftReport) {
    assert_eq!(
        report.status,
        CurveIntersectionParameterLiftStatus::BudgetExhausted
    );
    assert!(report.maps.is_empty());
    assert!(report.determinant_error.is_none());
}

#[test]
fn cofactor_lifts_share_degree_continuation_and_retry_every_exact_map() {
    for degree_drop in [false, true] {
        let [first, second] = equations(degree_drop);
        for axis in [
            CurveResultantParameter::First,
            CurveResultantParameter::Second,
        ] {
            let limited = CurveIntersectionResultantConfig {
                max_resultant_degree: 0,
                ..CurveIntersectionResultantConfig::default()
            };
            let expected = linear_parameter_lifts_bivariate_polynomial_system(
                &first,
                &second,
                axis,
                CurveIntersectionResultantConfig::default(),
            );
            assert_eq!(
                expected.status,
                CurveIntersectionParameterLiftStatus::Constructed
            );
            assert!(!expected.maps.is_empty());
            let bounded = with_budget(WorkBudget::ZERO, || {
                linear_parameter_lifts_bivariate_polynomial_system(&first, &second, axis, limited)
            });
            assert_eq!(
                bounded.result.unwrap().status,
                CurveIntersectionParameterLiftStatus::DegreeBoundExceeded
            );
            assert_eq!(bounded.usage, Default::default());
            let project = || {
                linear_parameter_lifts_bivariate_polynomial_system_complete(
                    &first, &second, axis, limited,
                )
            };
            let denied = with_budget(allowance(0), || assert_denied(&project()));
            assert_eq!(
                denied.result.unwrap_err().kind,
                WorkKind::AlgebraicPromotion
            );
            assert_eq!(denied.usage.promotions, 0);
            let retry = with_budget(allowance(1), project);
            assert_eq!(retry.result.unwrap(), expected);
            assert_eq!(retry.usage.promotions, 1);
        }
    }
}

#[test]
fn cofactor_lifts_cannot_reset_an_enclosing_allowance_or_escape_prior_exhaustion() {
    let [first, second] = equations(true);
    let project = |axis| {
        linear_parameter_lifts_bivariate_polynomial_system(
            &first,
            &second,
            axis,
            CurveIntersectionResultantConfig::default(),
        )
    };
    let expected = (
        project(CurveResultantParameter::First),
        project(CurveResultantParameter::Second),
    );
    let denied = with_budget(allowance(1), || {
        assert_eq!(project(CurveResultantParameter::First), expected.0);
        let child = with_budget(allowance(8), || {
            assert_denied(&project(CurveResultantParameter::Second))
        });
        assert_eq!(child.result.unwrap_err().used, 1);
    });
    assert_eq!(
        denied.result.unwrap_err().kind,
        WorkKind::AlgebraicPromotion
    );
    assert_eq!(denied.usage.promotions, 1);
    let retry = with_budget(allowance(2), || {
        (
            project(CurveResultantParameter::First),
            project(CurveResultantParameter::Second),
        )
    });
    assert_eq!(retry.result.unwrap(), expected);
    assert_eq!(retry.usage.promotions, 2);

    let empty = BivariatePolynomial::new(vec![]);
    let rejected = || {
        linear_parameter_lifts_bivariate_polynomial_system(
            &empty,
            &second,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig::default(),
        )
    };
    assert_eq!(
        with_budget(WorkBudget::ZERO, rejected)
            .result
            .unwrap()
            .status,
        CurveIntersectionParameterLiftStatus::EmptyEquation
    );
    let exhausted = with_budget(WorkBudget::ZERO, || {
        assert!(consume(WorkKind::RootRefinement, 1).is_err());
        assert_denied(&rejected());
    });
    assert_eq!(exhausted.result.unwrap_err().kind, WorkKind::RootRefinement);
}
