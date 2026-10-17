use hypercurve::{
    CurvePath2, CurveRegion2, FillRule, LineSeg2, OffsetCap, OffsetCornerStyle2, Point2,
    PredicatePolicy, Real, RegionPointLocation, WorkBudget, WorkUsage, evaluate_under,
    evaluate_with_budget,
};

fn same(actual: &Real, expected: &Real) {
    assert_eq!(
        hyperlimit::compare_reals(actual, expected, PredicatePolicy::STRICT).value(),
        Some(std::cmp::Ordering::Equal)
    );
}

#[test]
fn translated_reversed_strokes_keep_analytic_measurements_and_membership() {
    // Independent Euclidean lengths for these integer directions. The source
    // translation and radius remain exact, including their symbolic values.
    let cases = [
        (1, 1, Real::from(2).sqrt().unwrap()),
        (3, 4, Real::from(5)),
        (-2, 3, Real::from(13).sqrt().unwrap()),
        (0, -2, Real::from(2)),
    ];
    for (dx, dy, length) in cases {
        for radius in [
            (Real::one() / Real::from(7)).unwrap(),
            Real::from(3).sqrt().unwrap(),
        ] {
            let start = Point2::new(Real::pi() + Real::from(17), Real::from(-7));
            let end = Point2::new(start.x() + Real::from(dx), start.y() + Real::from(dy));
            let midpoint = Point2::new(
                ((start.x() + end.x()) / Real::from(2)).unwrap(),
                ((start.y() + end.y()) / Real::from(2)).unwrap(),
            );
            for reverse in [false, true] {
                let (a, b) = if reverse {
                    (&end, &start)
                } else {
                    (&start, &end)
                };
                let path = CurvePath2::try_new(vec![
                    LineSeg2::try_new(a.clone(), b.clone()).unwrap().into(),
                ])
                .unwrap();
                for cap in [OffsetCap::Butt, OffsetCap::Round] {
                    let mut expected = Real::from(2) * &radius * &length;
                    if cap == OffsetCap::Round {
                        expected += Real::pi() * &radius * &radius;
                    }
                    for policy in [
                        PredicatePolicy::STRICT,
                        PredicatePolicy::APPROXIMATE_512,
                        PredicatePolicy::STRICT,
                    ] {
                        let provisional = evaluate_under(policy, || {
                            CurveRegion2::stroke_path(
                                &path,
                                radius.clone(),
                                &OffsetCornerStyle2::Round,
                                cap,
                            )
                        });
                        let region = provisional
                            .certified()
                            .expect("no approximate evidence required")
                            .unwrap();
                        let rebuilt = CurveRegion2::try_from_boundary_paths(
                            &region.boundary_paths().unwrap(),
                            FillRule::NonZero,
                        )
                        .unwrap();
                        for r in [&region, &rebuilt] {
                            same(&r.filled_area().unwrap().unwrap(), &expected);
                            assert_eq!(
                                r.classify_point(&midpoint.clone().into()).unwrap(),
                                RegionPointLocation::Inside
                            );
                            assert_eq!(
                                r.classify_point(&start.clone().into()).unwrap(),
                                if cap == OffsetCap::Butt {
                                    RegionPointLocation::Boundary
                                } else {
                                    RegionPointLocation::Inside
                                }
                            );
                            if cap == OffsetCap::Round {
                                let bounds = r.bounds().unwrap().unwrap();
                                for (actual, expected) in [
                                    (bounds.min_x(), start.x() + Real::from(dx.min(0)) - &radius),
                                    (bounds.max_x(), start.x() + Real::from(dx.max(0)) + &radius),
                                    (bounds.min_y(), start.y() + Real::from(dy.min(0)) - &radius),
                                    (bounds.max_y(), start.y() + Real::from(dy.max(0)) + &radius),
                                ] {
                                    same(actual, &expected);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn exact_extreme_axis_strokes_complete_without_refinement_budget() {
    for (center, length, radius) in [
        ("1e400", "4", "1"),
        ("0", "1e-400", "1e-400"),
        ("1e400", "1e-400", "1e-400"),
    ] {
        let center: Real = center.parse().unwrap();
        let length: Real = length.parse().unwrap();
        let radius: Real = radius.parse().unwrap();
        for cap in [OffsetCap::Butt, OffsetCap::Round] {
            let evaluated = evaluate_with_budget(PredicatePolicy::STRICT, WorkBudget::ZERO, || {
                let source = CurvePath2::try_new(vec![
                    LineSeg2::try_new(
                        Point2::new(center.clone(), Real::zero()),
                        Point2::new(&center + &length, Real::zero()),
                    )
                    .unwrap()
                    .into(),
                ])
                .unwrap();
                let region = CurveRegion2::stroke_path(
                    &source,
                    radius.clone(),
                    &OffsetCornerStyle2::Round,
                    cap,
                )
                .unwrap();
                let mut expected = Real::from(2) * &radius * &length;
                if cap == OffsetCap::Round {
                    expected += Real::pi() * &radius * &radius;
                }
                same(&region.filled_area().unwrap().unwrap(), &expected);
                assert_eq!(
                    region
                        .classify_point(
                            &Point2::new(
                                &center + (&length / Real::from(2)).unwrap(),
                                Real::zero()
                            )
                            .into()
                        )
                        .unwrap(),
                    RegionPointLocation::Inside
                );
                region
            });
            assert_eq!(evaluated.usage, WorkUsage::default());
            let region = evaluated.result.unwrap().certified().unwrap();
            assert!(!region.is_empty());
        }
    }
}
