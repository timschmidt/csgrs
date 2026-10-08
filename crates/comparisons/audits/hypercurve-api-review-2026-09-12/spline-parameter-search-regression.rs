
#[test]
fn spline_parameter_search_preserves_every_discontinuous_knot_side() {
    const SPANS: i32 = 65;
    let controls: Vec<_> = (0..SPANS).flat_map(|i| [p(2 * i, 0), p(2 * i + 1, 0)]).collect();
    let knots: Vec<_> = (0..=SPANS).flat_map(|i| [r(i * i), r(i * i)]).collect();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let polynomial = hypercurve::PolynomialSplineCurve2::try_new(1, controls.clone(), knots.clone(), &policy).unwrap().into_value();
        let rational = NurbsCurve2::try_new(1, controls.clone(), (0..SPANS).flat_map(|_| [r(1), r(3)]).collect(), knots.clone(), &policy).unwrap().into_value();
        for (curve, local_x, local_dx) in [
            (Curve2::from(polynomial), q(1, 4), r(1)),
            (Curve2::from(rational), q(1, 2), q(4, 3)),
        ] {
            for i in 0..SPANS {
                let width = r(2 * i + 1);
                let parameter = r(i * i) + &width * q(1, 4);
                let point = curve.point_at(&parameter.clone().into(), &policy).unwrap();
                assert_eq!(point.certainty, hypercurve::CurveCertainty::Certified);
                assert_eq!(point.value.coordinates(), Some(&Point2::new(r(2 * i) + &local_x, r(0))));
                let derivative = curve.derivative_at(&parameter, &policy).unwrap();
                assert_eq!(derivative.certainty, hypercurve::CurveCertainty::Certified);
                assert_eq!(derivative.value.dx(), &(&local_dx / &width).unwrap());
                assert_eq!(derivative.value.dy(), &r(0));
            }
            for i in 0..=SPANS {
                let parameter = r(i * i).into();
                for (side, expected_x) in [
                    (CurveParameterSide2::Left, if i == 0 { 0 } else { 2 * i - 1 }),
                    (CurveParameterSide2::Right, if i == SPANS { 2 * i - 1 } else { 2 * i }),
                ] {
                    let point = curve.point_at_side(&parameter, side, &policy).unwrap();
                    assert_eq!(point.certainty, hypercurve::CurveCertainty::Certified);
                    assert_eq!(point.value.coordinates(), Some(&p(expected_x, 0)));
                }
                if i > 0 && i < SPANS {
                    assert!(matches!(curve.point_at(&parameter, &policy), Err(ExactCurveError::Blocked(blocker)) if blocker.reason() == hypercurve::UncertaintyReason::Boundary));
                }
            }
            for parameter in [r(-1), r(SPANS * SPANS + 1)] {
                assert!(matches!(curve.point_at(&parameter.into(), &policy), Err(ExactCurveError::Invalid { cause: CurveError::InvalidCurveParameter, .. })));
            }
        }
    }
}
