#[cfg(test)]
mod algebraic_bridge_fillet_regression {
    use super::*;
    use crate::bezier_split::{BezierSelectedFiberFragment2, BezierSelectedFiberSource2};

    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }

    #[test]
    fn fillet_extension_owns_the_algebraic_endpoint_bridge() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(polynomial) =
                crate::BezierParameterPolynomial::try_new_power_basis(
                    vec![-Real::one(), Real::zero(), Real::from(2)],
                    &policy,
                )
                .unwrap()
            else {
                panic!("the endpoint polynomial must be exact");
            };
            let Classification::Decided(mut roots) =
                polynomial.isolate_unit_interval_roots(&policy).unwrap()
            else {
                panic!("the positive endpoint root must be isolated");
            };
            assert_eq!(roots.len(), 1);
            let endpoint: CurveParameter2 = roots.pop().unwrap().into();
            assert!(matches!(
                endpoint.as_bezier_parameter(),
                Some(BezierParameter2::Algebraic(_))
            ));
            let parallel = QuadraticBezier2::new(
                Point2::from_values(0, 0),
                Point2::new(q(1, 2), Real::zero()),
                Point2::from_values(1, 1),
            )
            .parallel_left(Real::zero())
            .unwrap();
            let Classification::Decided(incident) = parallel
                .incident_domain_from_parameter(
                    &endpoint,
                    crate::BezierParameterRayDirection2::Increasing,
                    &policy,
                )
                .unwrap()
            else {
                panic!("the parabola has an unbounded regular continuation");
            };
            assert_eq!(
                crate::classify::real_sign(&(incident.anchor() - q(23, 32)), &policy),
                Some(RealSign::Positive),
                "the independently chosen contact must lie before the chart anchor"
            );
            assert_eq!(
                incident
                    .contains_extension_parameter(&q(23, 32).into(), &policy)
                    .unwrap(),
                Classification::Decided(true)
            );
            let join = Point2::new(q(1, 2).sqrt().unwrap(), q(1, 2));
            let previous: Curve2 = BezierSelectedFiberFragment2::new(
                BezierSelectedFiberSource2::AnalyticParallel(parallel),
                CurveParameterRange2::new_validated(Real::zero().into(), endpoint),
                Point2::from_values(0, 0).into(),
                join.clone().into(),
            )
            .into();
            let next = LineSeg2::try_new(join, Point2::new(-Real::one(), q(1, 2)))
                .unwrap()
                .into();
            let path = CurvePath2::try_new_with_policy(vec![previous, next], &policy)
                .unwrap()
                .value;
            // At P(23/32) the right normal is (23,-16)/sqrt(785).
            // Equating its circle-center height with the line's y=1/2+r
            // gives this exact, strictly positive radius.
            let root = Real::from(785).sqrt().unwrap();
            let radius =
                (Real::from(17) * &root / (Real::from(1024) * (&root + Real::from(16)))).unwrap();
            let center = Point2::new(
                q(23, 32) + (Real::from(23) * &radius / &root).unwrap(),
                q(1, 2) + &radius,
            );
            let contacts = [
                CurvePoint2::from(Point2::new(q(23, 32), q(529, 1024))),
                CurvePoint2::from(Point2::new(center.x().clone(), q(1, 2))),
            ];
            for reversed in [false, true] {
                let path = if reversed {
                    path.reversed(&policy).unwrap().value
                } else {
                    path.clone()
                };
                let mut request = CurveFillet2::new(radius.clone());
                request.center = Some(center.clone().into());
                let result = match path.fillet_vertex(
                    1,
                    &request,
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                ) {
                    Ok(result) => result,
                    Err(ExactCurveError::Blocked(blocker)) => {
                        panic!("known bridge fillet blocked: {:?}", blocker.reason())
                    }
                    Err(_) => panic!("known bridge fillet rejected"),
                };
                assert_eq!(result.certainty, crate::CurveCertainty::Certified);
                assert_eq!(
                    result.value.candidate_count(),
                    1,
                    "the bridge owns one independently known exact center"
                );
                let solution = result.value.into_solutions().pop().unwrap();
                for expected in &contacts {
                    assert!(
                        solution.curves().windows(2).any(|pair| {
                            pair[0].end().coincides_with(expected, &policy).value
                                == Classification::Decided(true)
                                && pair[1].start().coincides_with(expected, &policy).value
                                    == Classification::Decided(true)
                        }),
                        "each independently known contact must be a retained path junction"
                    );
                }
            }
        }
    }
}
