#[cfg(test)]
mod incident_parallel_cusp_nonlinear_regression {
    use super::*;
    use crate::bezier_split::{BezierSelectedFiberFragment2, BezierSelectedFiberSource2};

    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }

    fn same(actual: &CurvePoint2, expected: &CurvePoint2, policy: &CurveContext) {
        let result = actual.coincides_with(expected, policy);
        assert_eq!(result.certainty, crate::CurveCertainty::Certified);
        assert_eq!(result.value, Classification::Decided(true));
    }

    fn incident_cusp_case() {
        // P(t)=(t-t^3/3,t^2) has positive speed 1+t^2 on the entire real axis.
        // Its distance-2 left parallel has derivative
        // (1-4/(1+t^2)^2)*(1-t^2,2t). At t=1 it has the cusp Q=(-4/3,1).
        // The surviving t<1 branch approaches Q with tangent (0,-1).
        // The authored parallel ends at t=3/4, so this contact is on its ray.
        let contact = Point2::new(q(-4, 3), Real::one());
        let join = Point2::new(q(-2097, 1600), q(449, 400));
        let center = Point2::new(q(-1747, 1200), Real::one());
        let radius = q(49, 400);
        let vx = join.x() - center.x();
        let vy = join.y() - center.y();
        let square = &vx * &vx + &vy * &vy;
        let delta = q(697, 4800);
        // The lower of the two independent circle tangents from J. Its
        // clockwise direction agrees with both surviving curve tangents.
        let line_contact = Point2::new(
            center.x() + ((&radius * &radius * &vx + &radius * &delta * &vy) / &square).unwrap(),
            center.y() + ((&radius * &radius * &vy - &radius * &delta * &vx) / &square).unwrap(),
        );
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let sign = |value: Real| crate::classify::real_sign(&value, &policy);
            assert_eq!(
                sign(&delta * &delta + &radius * &radius - &square),
                Some(RealSign::Zero),
            );
            let rx = line_contact.x() - center.x();
            let ry = line_contact.y() - center.y();
            let tx = line_contact.x() - join.x();
            let ty = line_contact.y() - join.y();
            assert_eq!(
                sign(&rx * &rx + &ry * &ry - &radius * &radius),
                Some(RealSign::Zero)
            );
            assert_eq!(sign(&rx * &tx + &ry * &ty), Some(RealSign::Zero));
            assert_eq!(sign(&ry * &tx - &rx * &ty), Some(RealSign::Positive));
            assert_eq!(sign(rx), Some(RealSign::Positive));
            assert_eq!(sign(ry), Some(RealSign::Negative));
            let parallel = CubicBezier2::new(
                Point2::from_values(0, 0),
                Point2::new(q(1, 3), Real::zero()),
                Point2::new(q(2, 3), q(1, 3)),
                Point2::new(q(2, 3), Real::one()),
            )
            .parallel_left(Real::from(2))
            .unwrap();
            for (parameter, expected) in [(q(3, 4), &join), (Real::one(), &contact)] {
                let Classification::Decided(actual) =
                    parallel.point_at(&parameter, &policy).unwrap()
                else {
                    panic!("the source normal is defined at the authored endpoint and offset cusp");
                };
                same(&actual.into(), &expected.clone().into(), &policy);
            }
            let previous: Curve2 = BezierSelectedFiberFragment2::new(
                BezierSelectedFiberSource2::AnalyticParallel(parallel),
                CurveParameterRange2::new_validated(Real::zero().into(), q(3, 4).into()),
                Point2::from_values(0, 2).into(),
                join.clone().into(),
            )
            .into();
            let dx = line_contact.x() - join.x();
            let dy = line_contact.y() - join.y();
            let nx = -dy.clone();
            let ny = dx.clone();
            // R(u)=J+2uD+u(u-1/2)^2 N, N perpendicular to D=L-J.
            // R(1/2)=L and R'(1/2)=2D; R' dot D=2|D|²>0.
            // For u>=0 the added normal term points away from the circle;
            // for u<0 its tangential distance exceeds delta>radius. Thus
            // L is the unique contact with this constrained circle.
            let control = |d: &Real, n: &Real, origin: &Real, along: Real, normal: Real| {
                origin + along * d + normal * n
            };
            let next: Curve2 = CubicBezier2::new(
                join.clone(),
                Point2::new(
                    control(&dx, &nx, join.x(), q(2, 3), q(1, 12)),
                    control(&dy, &ny, join.y(), q(2, 3), q(1, 12)),
                ),
                Point2::new(
                    control(&dx, &nx, join.x(), q(4, 3), q(-1, 6)),
                    control(&dy, &ny, join.y(), q(4, 3), q(-1, 6)),
                ),
                Point2::new(
                    control(&dx, &nx, join.x(), Real::from(2), q(1, 4)),
                    control(&dy, &ny, join.y(), Real::from(2), q(1, 4)),
                ),
            )
            .into();
            let path = CurvePath2::try_new_with_policy(vec![previous, next], &policy)
                .unwrap()
                .value;
            for reversed in [false, true] {
                let path = if reversed {
                    path.reversed(&policy).unwrap().value
                } else {
                    path.clone()
                };
                for constraint in ["center", "parameter", "point"] {
                    let mut request = CurveFillet2::new(radius.clone());
                    match constraint {
                        "center" => request.center = Some(center.clone().into()),
                        "parameter" => {
                            request.contacts[usize::from(reversed)] =
                                Some(CurveFilletContact2::Parameter(Real::one().into()))
                        }
                        "point" => {
                            request.contacts[usize::from(reversed)] =
                                Some(CurveFilletContact2::Point(contact.clone().into()))
                        }
                        _ => unreachable!(),
                    }
                    let result = match path.fillet_vertex(
                        1,
                        &request,
                        CurveCornerMode2::TrimOrExtend,
                        &policy,
                    ) {
                        Ok(result) => result,
                        Err(ExactCurveError::Blocked(blocker)) => {
                            panic!(
                                "known incident cusp fillet blocked: {:?}; reversed={reversed}, constraint={constraint}, nonlinear_partner=true",
                                blocker.reason()
                            )
                        }
                        Err(ExactCurveError::Invalid { cause, .. }) => panic!(
                            "known incident cusp fillet invalid: {:?}",
                            std::mem::discriminant(&cause)
                        ),
                    };
                    assert_eq!(result.certainty, crate::CurveCertainty::Certified);
                    assert_eq!(
                        result.value.candidate_count(),
                        1,
                        "the incident cusp has one constrained nonzero fillet"
                    );
                    let solution = &result.value.solutions()[0];
                    for expected in [contact.clone(), line_contact.clone()] {
                        let expected = CurvePoint2::from(expected);
                        assert!(
                            solution.curves().windows(2).any(|pair| {
                                pair[0].end().coincides_with(&expected, &policy).value
                                    == Classification::Decided(true)
                                    && pair[1].start().coincides_with(&expected, &policy).value
                                        == Classification::Decided(true)
                            }),
                            "the independently known contact must remain a path junction"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn incident_parallel_cusp_fillet_retains_nonlinear_partner_contact() {
        incident_cusp_case();
    }
}
