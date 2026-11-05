use super::*;
use crate::bezier_offset::BezierAlgebraicSelectedFiberParameter2;
use crate::{
    BezierAlgebraicParameter2, BezierParameterInterval, BezierParameterPolynomial, LineSeg2,
};
use hypersolve::work_budget::{WorkBudget, with_budget};
use hypersolve::{BivariatePolynomial, IsolatedRootInterval};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn exact<T: std::fmt::Debug>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(value) => value,
        other => panic!("exact fixture required: {other:?}"),
    }
}

fn fixture(policy: &CurveContext) -> (Span, CurveParameter2, CurvePoint2, Real) {
    // alpha=(3-sqrt(5))/2 and beta=(1-sqrt(7/2-8*alpha))/2.
    // Independently, 32*alpha+16*beta^2-16*beta-10=0 selects
    // beta in (4/25,17/100]. This is the original recut's defining fiber.
    let alpha = BezierAlgebraicParameter2::try_isolate(
        BezierParameterPolynomial::try_new_power_basis(vec![(-1).into(), 3.into(), (-1).into()])
            .unwrap(),
        BezierParameterInterval::try_new(q(3, 8), q(2, 5)).unwrap(),
    )
    .unwrap();
    let selected = exact(
        BezierAlgebraicSelectedFiberParameter2::from_checked_roots(
            BivariatePolynomial::new(vec![
                vec![(-10).into(), (-16).into(), 16.into()],
                vec![32.into()],
            ]),
            alpha,
            &[IsolatedRootInterval {
                lower: q(4, 25),
                upper: q(17, 100),
                exact_root: None,
                distinct_root_count: 1,
            }],
            policy,
        )
        .unwrap(),
    )
    .pop()
    .unwrap();
    let alpha = (Real::from(3) - Real::from(5).sqrt().unwrap()) / Real::from(2);
    let beta = ((Real::one() - (q(7, 2) - Real::from(8) * alpha.unwrap()).sqrt().unwrap())
        / Real::from(2))
    .unwrap();
    let line: Curve2 = LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0))
        .unwrap()
        .into();
    let mut span = spans(&line, policy).unwrap().pop().unwrap();
    span.range = CurveParameterRange2::try_new(beta.clone().into(), Real::one().into()).unwrap();
    let point: CurvePoint2 = Point2::new(beta.clone(), Real::zero()).into();
    span.endpoint_points = Some([point.clone(), Point2::from_values(1, 0).into()]);
    (
        span,
        CurveParameter2::from_selected_fiber(selected),
        point,
        beta,
    )
}

#[test]
fn retained_endpoint_identity_precedes_scalar_equality_refinement() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let (span, mut parameter, point, beta) = fixture(&policy);
        let pair = Pair {
            first: &span,
            second: &span,
            indices: [0, 0],
            policy: &policy,
        };
        let report = with_budget(WorkBudget::ZERO, || {
            pair.admit_parameter(&span, &mut parameter, &point)
        });
        assert!(report.result.unwrap().unwrap());
        assert_eq!(parameter, CurveParameter2::from(beta));
        assert_eq!(report.usage.refinements, 0);
        assert_eq!(report.usage.promotions, 0);
    }
}

#[test]
fn missing_endpoint_proof_retains_exact_or_typed_resource_outcomes() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let (mut span, parameter, point, _) = fixture(&policy);
        span.endpoint_points = None;
        let pair = Pair {
            first: &span,
            second: &span,
            indices: [0, 0],
            policy: &policy,
        };
        let report = with_budget(WorkBudget::ZERO, || {
            // A warmed independent scalar proof can also decide this exact
            // query. Otherwise the complete fallback must preserve exhaustion.
            match pair.admit_parameter(&span, &mut parameter.clone(), &point) {
                Ok(inside) => assert!(inside),
                Err(error) => assert!(matches!(error, ExactCurveError::Blocked(ref b)
                    if b.reason() == UncertaintyReason::BudgetExhausted)),
            }
        });
        if report.result.is_ok() {
            assert_eq!(report.usage.refinements, 0);
            assert_eq!(report.usage.promotions, 0);
        }
        // Exhaustion must not poison a new proof-backed retry.
        let (span, mut parameter, point, _) = fixture(&policy);
        let pair = Pair {
            first: &span,
            second: &span,
            indices: [0, 0],
            policy: &policy,
        };
        let retry = with_budget(WorkBudget::ZERO, || {
            pair.admit_parameter(&span, &mut parameter, &point)
        });
        assert!(retry.result.unwrap().unwrap());
    }
}

#[test]
fn coincident_endpoint_on_retraced_support_does_not_admit_the_other_preimage() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let (mut span, parameter, _, beta) = fixture(&policy);
        // Q(t)=(2t(1-t),0) has Q(beta)=Q(1-beta), but 1-beta is
        // outside [beta,1/2]. Point equality alone must not admit that cut.
        span.support =
            CurveSupport2::Bezier(BezierSubcurve2::Quadratic(crate::QuadraticBezier2::new(
                Point2::from_values(0, 0),
                Point2::from_values(1, 0),
                Point2::from_values(0, 0),
            )));
        span.range = CurveParameterRange2::try_new(beta.clone().into(), q(1, 2).into()).unwrap();
        let point: CurvePoint2 =
            Point2::new(Real::from(2) * &beta * (Real::one() - &beta), Real::zero()).into();
        span.endpoint_points = Some([point.clone(), Point2::new(q(1, 2), Real::zero()).into()]);
        let mut other = parameter.affine_image(&-Real::one(), &Real::one()).unwrap();
        let pair = Pair {
            first: &span,
            second: &span,
            indices: [0, 0],
            policy: &policy,
        };
        let report = with_budget(WorkBudget::ZERO, || {
            pair.admit_parameter(&span, &mut other, &point)
        });
        assert!(!report.result.unwrap().unwrap());
    }
}
