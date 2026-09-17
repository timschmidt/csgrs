use hypercurve::*;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::from_values(x, y)
}

fn certified<T>(outcome: CurveOutcome<T>) -> T {
    assert_eq!(outcome.certainty, CurveCertainty::Certified);
    outcome.value
}

fn decided<T>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("{reason:?}"),
    }
}

fn range(
    start: CurveParameter2,
    end: CurveParameter2,
    policy: &CurveContext,
) -> CurveParameterRange2 {
    decided(CurveParameterRange2::try_new(start, end, policy).unwrap())
}

fn same(actual: &CurvePoint2, expected: &CurvePoint2, policy: &CurveContext) {
    assert!(decided(certified(actual.coincides_with(expected, policy))));
}

#[test]
fn exterior_selected_bezier_ranges_retain_their_chart_through_repeated_cuts() {
    // Q(t) = (t-2, (t-2)^2), restricted to a finite exterior chart.
    let source = BezierSubcurve2::Quadratic(QuadraticBezier2::new(
        p(-2, 4),
        Point2::new(q(-3, 2), Real::from(2)),
        p(-1, 1),
    ));
    let root = Real::from(2) + q(1, 2).sqrt().unwrap();
    let expected: CurvePoint2 = Point2::new(q(1, 2).sqrt().unwrap(), q(1, 2)).into();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let whole = certified(
            Curve2::try_from_bezier_range(
                source.clone(),
                range(Real::from(2).into(), Real::from(3).into(), &policy),
                &policy,
            )
            .unwrap(),
        );
        let horizontal = Curve2::from(
            LineSeg2::try_new(
                Point2::new(Real::from(-1), q(1, 2)),
                Point2::new(Real::from(2), q(1, 2)),
            )
            .unwrap(),
        );
        let intersections = certified(whole.intersect_curve(&horizontal, &policy).unwrap());
        assert!(
            intersections.is_complete(),
            "{:?}",
            intersections.blockers()
        );
        assert_eq!(intersections.contacts().len(), 1);
        let selected = decided(
            intersections.contacts()[0]
                .first()
                .parameter(&policy)
                .unwrap(),
        );
        assert!(selected.scalar().is_none());
        assert_eq!(
            decided(certified(
                selected.compare(&root.clone().into(), &policy).unwrap()
            )),
            std::cmp::Ordering::Equal
        );
        for reverse in [false, true] {
            let endpoints = if reverse {
                (selected.clone(), Real::from(2).into())
            } else {
                (Real::from(2).into(), selected.clone())
            };
            let mut curve = certified(
                Curve2::try_from_bezier_range(
                    source.clone(),
                    range(endpoints.0, endpoints.1, &policy),
                    &policy,
                )
                .unwrap(),
            );
            for cut in 1..=16 {
                assert_eq!(curve.parameter_domain().end(), &selected);
                same(
                    &certified(curve.point_at(&selected, &policy).unwrap()),
                    &expected,
                    &policy,
                );
                same(
                    &if reverse { curve.start() } else { curve.end() },
                    &expected,
                    &policy,
                );
                let lower = Real::from(2) + q(cut, 32);
                curve = certified(
                    curve
                        .subcurve(lower.clone().into(), selected.clone(), &policy)
                        .unwrap(),
                );
                let offset = &lower - Real::from(2);
                let lower_point: CurvePoint2 =
                    Point2::new(offset.clone(), &offset * &offset).into();
                same(
                    &if reverse { curve.end() } else { curve.start() },
                    &lower_point,
                    &policy,
                );
            }
            let reversed = certified(curve.reversed(&policy).unwrap());
            assert_eq!(reversed.parameter_domain(), curve.parameter_domain());
            same(&reversed.start(), &curve.end(), &policy);
            same(&reversed.end(), &curve.start(), &policy);
        }
    }
}

#[test]
fn rational_range_admission_checks_only_the_retained_interval() {
    // W(t) = 1-4t+2t². Both roots are outside the retained intervals.
    let source = RationalBezier2::try_new(
        vec![p(0, 0), p(0, 1), p(1, 0)],
        vec![Real::one(), -Real::one(), -Real::one()],
    )
    .unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for (start, end) in [
            (Real::zero(), q(1, 4)),
            (q(3, 4), Real::one()),
            (Real::from(2), q(5, 2)),
        ] {
            for reverse in [false, true] {
                let endpoints = if reverse {
                    (end.clone(), start.clone())
                } else {
                    (start.clone(), end.clone())
                };
                let curve = certified(
                    Curve2::try_from_bezier_range(
                        BezierSubcurve2::Rational(source.clone()),
                        range(endpoints.0.into(), endpoints.1.into(), &policy),
                        &policy,
                    )
                    .unwrap(),
                );
                assert_eq!(
                    curve.parameter_domain().scalar_endpoints(),
                    Some((&start, &end))
                );
                for t in [start.clone(), (&start + &end) * q(1, 2), end.clone()] {
                    let weight = Real::one() - Real::from(4) * &t + Real::from(2) * &t * &t;
                    let expected = Point2::new(
                        (-(&t * &t) / &weight).unwrap(),
                        (-Real::from(2) * &t * (Real::one() - &t) / &weight).unwrap(),
                    );
                    same(
                        &certified(curve.point_at(&t.into(), &policy).unwrap()),
                        &expected.into(),
                        &policy,
                    );
                }
            }
        }
        // A finite endpoint pair is insufficient when the interval crosses a pole.
        for (start, end) in [(0, 1), (1, 2), (2, 0)] {
            let error = Curve2::try_from_bezier_range(
                BezierSubcurve2::Rational(source.clone()),
                range(Real::from(start).into(), Real::from(end).into(), &policy),
                &policy,
            )
            .unwrap_err();
            assert_eq!(error.operation(), CurveOperation2::Construction);
            assert!(
                matches!(error, ExactCurveError::Blocked(blocker) if blocker.reason() == UncertaintyReason::Boundary)
            );
        }
    }
}

#[test]
fn bezier_range_construction_reuses_selected_fiber_parameters() {
    // The chamfer joins (-1,0) to (u²,2u), where u²=sqrt(5)-2.
    // Its crossing at x=-1/2 has y=u/(1+u²), retained in that field.
    let squared = Real::from(5).sqrt().unwrap() - Real::from(2);
    let y = (squared.clone().sqrt().unwrap() / (Real::one() + &squared)).unwrap();
    let expected: CurvePoint2 = Point2::new(q(-1, 2), y).into();
    let source = BezierSubcurve2::Quadratic(QuadraticBezier2::new(
        Point2::new(q(-1, 2), Real::from(-1)),
        Point2::new(q(-1, 2), q(1, 2)),
        Point2::new(q(-1, 2), Real::from(2)),
    ));
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let path = CurvePath2::try_new(vec![
            LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap().into(),
            QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2)).into(),
        ])
        .unwrap();
        let CurveCornerSolutions2::Unique(path) = certified(
            path.chamfer_vertex_by_setbacks(
                1,
                Real::one(),
                Real::one(),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .unwrap(),
        ) else {
            panic!("one selected chamfer")
        };
        let crossing = Curve2::from(source.clone());
        let intersections = certified(
            path.curves()[1]
                .intersect_curve(&crossing, &policy)
                .unwrap(),
        );
        assert!(
            intersections.is_complete(),
            "{:?}",
            intersections.blockers()
        );
        assert_eq!(intersections.contacts().len(), 1);
        let selected = decided(
            intersections.contacts()[0]
                .second()
                .parameter(&policy)
                .unwrap(),
        );
        assert!(selected.as_bezier_parameter().is_none());
        same(intersections.contacts()[0].point(), &expected, &policy);
        for reverse in [false, true] {
            let endpoints = if reverse {
                (selected.clone(), Real::zero().into())
            } else {
                (Real::zero().into(), selected.clone())
            };
            let curve = certified(
                Curve2::try_from_bezier_range(
                    source.clone(),
                    range(endpoints.0, endpoints.1, &policy),
                    &policy,
                )
                .unwrap(),
            );
            assert_eq!(curve.parameter_domain().end(), &selected);
            same(
                &certified(curve.point_at(&selected, &policy).unwrap()),
                &expected,
                &policy,
            );
            same(
                &if reverse { curve.start() } else { curve.end() },
                &expected,
                &policy,
            );
            let split = certified(curve.split_at(q(1, 4).into(), &policy).unwrap());
            same(&split.0.end(), &split.1.start(), &policy);
            same(&split.0.start(), &curve.start(), &policy);
            same(&split.1.end(), &curve.end(), &policy);
        }
    }
}
