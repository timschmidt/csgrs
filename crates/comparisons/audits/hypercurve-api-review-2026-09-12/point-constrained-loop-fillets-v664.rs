#[cfg(test)]
mod constrained_regular_loop_fillet_regression {
    use super::*;

    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }

    #[test]
    fn point_constraint_preserves_distinct_regular_loop_contacts() {
        // P(t)=(u^2-1/16,u^3-u/16), u=t-1/2. P(1/4)=P(3/4)=0,
        // with tangents (-4,1)/8 and (4,1)/8. The source is regular:
        // its x derivative vanishes only at u=0, where y'=-1/16.
        // Both left normal centers lie on the previous vertical line's
        // left parallel for r=3(17+sqrt(17))/256. Their y coordinates have
        // opposite signs; the lower contact trims and the upper extends.
        let root = Real::from(17).sqrt().unwrap();
        let radius = q(3, 256) * (Real::from(17) + &root);
        let height = q(3, 64) * (Real::one() + &root);
        let origin: CurvePoint2 = Point2::from_values(0, 0).into();
        let join = Point2::new(q(3, 16), q(-3, 32));
        let curve = CubicBezier2::new(
            join.clone(),
            Point2::new(q(-7, 48), q(13, 96)),
            Point2::new(q(-7, 48), q(-13, 96)),
            Point2::new(q(3, 16), q(3, 32)),
        );
        for (policy_index, policy) in [CurveContext::STRICT, CurveContext::APPROXIMATE_512]
            .into_iter().enumerate()
        {
            let path = CurvePath2::try_new_with_policy(vec![
                LineSeg2::try_new(Point2::new(q(3, 16), -Real::from(2)), join.clone())
                    .unwrap().into(),
                curve.clone().into(),
            ], &policy).unwrap().value;
            for reversed in [false, true] {
                let path = if reversed { path.reversed(&policy).unwrap().value } else { path.clone() };
                let axis = usize::from(!reversed);
                for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                    let mut request = CurveFillet2::new(radius.clone());
                    request.contacts[axis] = Some(CurveFilletContact2::Point(origin.clone()));
                    let solve = |request: &CurveFillet2| {
                        path.fillet_vertex(1, request, mode, &policy).unwrap_or_else(|error| {
                            match error {
                                ExactCurveError::Blocked(blocker) => panic!(
                                    "regular loop: policy={policy_index}, reversed={reversed}, mode={mode:?}, reason={:?}", blocker.reason()),
                                ExactCurveError::Invalid { cause, .. } => panic!(
                                    "regular loop invalid: {:?}", std::mem::discriminant(&cause)),
                            }
                        })
                    };
                    let selected = solve(&request);
                    assert_eq!(selected.certainty, crate::CurveCertainty::Certified);
                    let expected = if mode == CurveCornerMode2::TrimOnly { 1 } else { 2 };
                    assert_eq!(selected.value.candidate_count(), expected);
                    for positive in [false, true] {
                        let y = if positive { height.clone() } else { -height.clone() };
                        request.center = Some(Point2::new(q(3,16) - &radius, y.clone()).into());
                        let selected = solve(&request);
                        assert_eq!(selected.certainty, crate::CurveCertainty::Certified);
                        let admitted = !positive || mode == CurveCornerMode2::TrimOrExtend;
                        assert_eq!(selected.value.candidate_count(), usize::from(admitted));
                        if admitted {
                            let expected_line: CurvePoint2 = Point2::new(q(3,16), y).into();
                            let result = &selected.value.solutions()[0];
                            for contact in [&origin, &expected_line] {
                                assert!(result.curves().windows(2).any(|pair| {
                                    pair[0].end().same_point(contact, &policy) == Classification::Decided(true)
                                        && pair[1].start().same_point(contact, &policy) == Classification::Decided(true)
                                }));
                            }
                        }
                    }
                    request.center = Some(Point2::from_values(4, 4).into());
                    assert_eq!(solve(&request).value.no_solution_reason(), Some(CurveCornerNoSolution2::UnsatisfiedConstraints));
                }
            }
        }
    }
}
