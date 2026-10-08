#[cfg(test)]
mod stationary_continuous_family_regression {
    use super::*;

    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }

    #[test]
    fn continuous_fillets_preserve_stationary_reparameterization() {
        // P(u)=(3u²/8,9u⁴/64) is the same parabola as the rational
        // continuous-family counterexample, with a stationary source endpoint.
        let base = RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(0, 0),
                Point2::new(q(1, 16), Real::zero()),
                Point2::new(q(3, 16), Real::zero()),
                Point2::new(q(3, 8), q(9, 64)),
            ],
            vec![Real::one(); 5],
        )
        .unwrap()
        .parallel_left(Real::zero())
        .unwrap();
        let parameter =
            CurveParameter2::from((Real::from(5).sqrt().unwrap() / Real::from(3)).unwrap());
        let center = CurvePoint2::from(Point2::new(q(-175, 4992), q(4699, 7488)));
        let contacts = [
            CurvePoint2::from(Point2::new(q(-95, 2496), q(4753, 7488))),
            CurvePoint2::from(Point2::new(q(-5, 156), q(4645, 7488))),
        ];
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let curves = [q(41, 64), q(5, 8)].map(|d| {
                let start = CurvePoint2::from(Point2::new(Real::zero(), d.clone()));
                let end =
                    CurvePoint2::from(Point2::new(q(3, 8) - q(3, 5) * &d, q(9, 64) + q(4, 5) * &d));
                Curve2::from(
                    crate::bezier_split::BezierSelectedFiberFragment2::new(
                        crate::bezier_split::BezierSelectedFiberSource2::AnalyticParallel(
                            base.with_distance(d),
                        ),
                        CurveParameterRange2::unit(),
                        start,
                        end,
                    )
                    .reversed(),
                )
            });
            let path = CurvePath2::try_new_with_policy(curves.into(), &policy)
                .unwrap()
                .value;
            for reversed in [false, true] {
                let path = if reversed {
                    path.reversed(&policy).unwrap().value
                } else {
                    path.clone()
                };
                let request = CurveFillet2::new(q(1, 128));
                let unconstrained =
                    path.fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy);
                assert!(
                    matches!(
                        unconstrained,
                        Err(ExactCurveError::Invalid {
                            cause: CurveError::FilletConstraintRequired,
                            ..
                        })
                    ),
                    "radius-only family error={:?}",
                    unconstrained.as_ref().err()
                );
                let mut requests = Vec::new();
                let mut centered = request.clone();
                centered.center = Some(center.clone());
                requests.push(centered);
                for axis in 0..2 {
                    for contact in [
                        CurveFilletContact2::Parameter(parameter.clone()),
                        CurveFilletContact2::Point(
                            contacts[if reversed { 1 - axis } else { axis }].clone(),
                        ),
                    ] {
                        let mut selected = request.clone();
                        selected.contacts[axis] = Some(contact);
                        requests.push(selected);
                    }
                }
                for selected in requests {
                    let outcome = path
                        .fillet_vertex(1, &selected, CurveCornerMode2::TrimOnly, &policy)
                        .unwrap();
                    assert_eq!(outcome.certainty, crate::CurveCertainty::Certified);
                    assert_eq!(outcome.value.candidate_count(), 1);
                    let edited = &outcome.value.solutions()[0];
                    for (actual, expected) in [
                        (edited.curves()[0].end(), &contacts[usize::from(reversed)]),
                        (
                            edited.curves().last().unwrap().start(),
                            &contacts[usize::from(!reversed)],
                        ),
                    ] {
                        let same = actual.coincides_with(expected, &policy);
                        assert_eq!(same.certainty, crate::CurveCertainty::Certified);
                        assert_eq!(same.value, Classification::Decided(true));
                    }
                    for inserted in &edited.curves()[1..edited.curves().len() - 1] {
                        let mut preparation =
                            crate::bezier_region::CornerCarrierPreparation2::from_curve(
                                inserted, true,
                            );
                        preparation
                            .prepare(CurveOperation2::Fillet, &policy)
                            .unwrap();
                        let (actual_center, radius_squared, clockwise) = match preparation
                            .exact_carrier(true, CurveOperation2::Fillet, &policy)
                            .unwrap()
                        {
                            ExactCornerCarrier2::Arc(circle) => (
                                CurvePoint2::from(circle.center().clone()),
                                circle.radius_squared(),
                                circle.is_clockwise(),
                            ),
                            ExactCornerCarrier2::RetainedRationalArc(circle) => {
                                let circle = circle.support();
                                (
                                    CurvePoint2::from(circle.center().clone()),
                                    circle.radius_squared(),
                                    circle.is_clockwise(),
                                )
                            }
                            ExactCornerCarrier2::AlgebraicCusp(fragment) => {
                                let circle = fragment.semicircle();
                                let actual_center =
                                    match circle.center_point_evidence(&policy).unwrap() {
                                        Classification::Decided(center) => center,
                                        Classification::Uncertain(reason) => {
                                            panic!("circle center must replay: {reason:?}")
                                        }
                                    };
                                (
                                    actual_center,
                                    circle.radial_distance() * circle.radial_distance(),
                                    circle.is_clockwise() ^ fragment.is_reversed(),
                                )
                            }
                            _ => panic!("the independently known fillet must retain its circle"),
                        };
                        assert_eq!(
                            crate::classify::real_sign(&(radius_squared - q(1, 16384)), &policy),
                            Some(RealSign::Zero)
                        );
                        let center_match = actual_center.coincides_with(&center, &policy);
                        assert_eq!(center_match.certainty, crate::CurveCertainty::Certified);
                        assert_eq!(center_match.value, Classification::Decided(true));
                        assert_eq!(clockwise, !reversed);
                    }
                }
            }
        }
    }
}
