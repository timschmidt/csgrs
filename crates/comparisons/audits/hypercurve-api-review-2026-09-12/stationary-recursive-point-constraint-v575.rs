#[cfg(test)]
mod stationary_recursive_point_constraint_regression {
    use super::*;

    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }

    fn decided<T>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("fixture must be certified: {reason:?}"),
        }
    }

    fn retained_point(point: &Point2, policy: &CurveContext) -> CurvePoint2 {
        // A chord with direction (3,4) has left unit normal (-4/5,3/5).
        // Its independently authored first endpoint plus that normal is
        // exactly the requested point, retained as a procedural displacement.
        let origin = Point2::new(point.x() + q(4, 5), point.y() - q(3, 5));
        let end = Point2::new(origin.x() + Real::from(3), origin.y() + Real::from(4));
        let source = decided(crate::BezierAlgebraicChord2::try_new(
            origin.into(), end.into(), policy,
        ).unwrap());
        let (point_image, _) = crate::BezierAlgebraicChordParallelPoint2::new_pair(
            source, Real::one(), Real::zero(), Real::zero(), policy,
        );
        let result = CurvePoint2::from(point_image);
        assert!(result.coordinates().is_none());
        let same = result.coincides_with(&CurvePoint2::from(point.clone()), policy);
        assert_eq!(same.certainty, crate::CurveCertainty::Certified);
        assert_eq!(same.value, Classification::Decided(true));
        result
    }

    #[test]
    fn stationary_fillet_family_accepts_independent_recursive_point_constraints() {
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
        let center = Point2::new(q(-175, 4992), q(4699, 7488));
        let contacts = [
            Point2::new(q(-95, 2496), q(4753, 7488)),
            Point2::new(q(-5, 156), q(4645, 7488)),
        ];
        let mut failures = Vec::new();
        for (policy_index, policy) in [CurveContext::STRICT, CurveContext::APPROXIMATE_512]
            .into_iter()
            .enumerate()
        {
            let curves = [q(41, 64), q(5, 8)].map(|distance| {
                let start = CurvePoint2::from(Point2::new(Real::zero(), distance.clone()));
                let end = CurvePoint2::from(Point2::new(
                    q(3, 8) - q(3, 5) * &distance,
                    q(9, 64) + q(4, 5) * &distance,
                ));
                Curve2::from(
                    crate::bezier_split::BezierSelectedFiberFragment2::new(
                        crate::bezier_split::BezierSelectedFiberSource2::AnalyticParallel(
                            base.with_distance(distance),
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
            let retained_center = retained_point(&center, &policy);
            let retained_contacts = contacts
                .each_ref()
                .map(|point| retained_point(point, &policy));
            for reversed in [false, true] {
                let path = if reversed {
                    path.reversed(&policy).unwrap().value
                } else {
                    path.clone()
                };
                for selection in 0..3 {
                    let mut request = CurveFillet2::new(q(1, 128));
                    if selection == 0 {
                        request.center = Some(retained_center.clone());
                    } else {
                        let axis = selection - 1;
                        request.contacts[axis] = Some(CurveFilletContact2::Point(
                            retained_contacts[if reversed { 1 - axis } else { axis }].clone(),
                        ));
                    }
                    let result = match path.fillet_vertex(
                        1,
                        &request,
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    ) {
                        Ok(result) => result,
                        Err(error) => {
                            failures.push((
                                policy_index,
                                reversed,
                                selection,
                                format!("{error:?}"),
                            ));
                            continue;
                        }
                    };
                    assert_eq!(result.certainty, crate::CurveCertainty::Certified);
                    assert_eq!(result.value.candidate_count(), 1);
                    let result = &result.value.solutions()[0];
                    for (actual, expected) in [
                        (result.curves()[0].end(), &contacts[usize::from(reversed)]),
                        (
                            result.curves().last().unwrap().start(),
                            &contacts[usize::from(!reversed)],
                        ),
                    ] {
                        let same =
                            actual.coincides_with(&CurvePoint2::from(expected.clone()), &policy);
                        assert_eq!(same.certainty, crate::CurveCertainty::Certified);
                        assert_eq!(same.value, Classification::Decided(true));
                    }
                }
            }
        }
        assert!(
            failures.is_empty(),
            "retained exact point constraints must select the same fillets: {failures:?}"
        );
    }
}
