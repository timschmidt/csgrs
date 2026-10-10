use hypercurve::{
    Curve2, CurveGeometry2, CurveParameterRange2, CurvePath2, CurveRegion2, FillRule,
    Point2, PredicatePolicy, QuadraticBezier2, Real, RegionPointLocation, evaluate_under,
};

fn lens(x: &Real, y: &Real, scale: &Real, retained: bool, reversed: bool, warm: bool) -> CurveRegion2 {
    let map = |dx: i32, dy: i32| {
        let point = Point2::new(x + scale * Real::from(dx), y + scale * Real::from(dy));
        if warm {
            for precision in [-32, -1024, -32] {
                point.x().certified_rational_interval(precision).unwrap();
                point.y().certified_rational_interval(precision).unwrap();
            }
        }
        point
    };
    // The normalized lens has x=2t-1, y=+/-2t(1-t), t in [0,1].
    // The retained chart substitutes t=u-1 and keeps u in [1,2].
    let curves = if retained {
        let range = CurveParameterRange2::try_new(Real::one().into(), Real::from(2).into()).unwrap();
        vec![
            Curve2::try_from_bezier_range(CurveGeometry2::QuadraticBezier(QuadraticBezier2::new(map(-3, 4), map(-2, 1), map(-1, 0))), range.clone()).unwrap(),
            Curve2::try_from_bezier_range(CurveGeometry2::QuadraticBezier(QuadraticBezier2::new(map(3, -4), map(2, -1), map(1, 0))), range).unwrap(),
        ]
    } else {
        vec![
            QuadraticBezier2::new(map(-1, 0), map(0, -1), map(1, 0)).into(),
            QuadraticBezier2::new(map(1, 0), map(0, 1), map(-1, 0)).into(),
        ]
    };
    let path = CurvePath2::try_new(curves).unwrap();
    let path = if reversed { path.reversed().unwrap() } else { path };
    CurveRegion2::try_from_boundary_paths(&[path], FillRule::NonZero).unwrap()
}

#[test]
fn retained_lens_centers_survive_extreme_finite_translations() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for (origin, scale) in [("0", "1"), ("1e308", "1e307"), ("-1e308", "1e307")] {
            evaluate_under(policy, || {
                let origin: Real = origin.parse().unwrap();
                let scale: Real = scale.parse().unwrap();
                let region = lens(&origin, &origin, &scale, true, false, false);
                assert!(!region.is_empty());
                assert_eq!(
                    region.classify_point(&Point2::new(origin.clone(), origin).into()).unwrap(),
                    RegionPointLocation::Inside,
                );
            }).certified().unwrap();
        }
    }
}

#[test]
fn exact_lens_oracle_is_independent_of_scale_chart_direction_and_warmth() {
    use RegionPointLocation::{Boundary, Inside, Outside};
    let frames = [
        ("0", "0", "1"),
        ("1e308", "1e308", "1e307"),
        ("-1e308", "-1e308", "1e307"),
        ("1e308", "-1e308", "1e307"),
        ("-1e308", "1e308", "1e307"),
        ("1e200", "-1e200", "1e199"),
        ("1e308", "1e308", "-1e307"),
        ("0", "0", "1e-400"),
        ("1e308", "-1e308", "1e-400"),
        ("1e400", "-1e400", "1"),
        ("0", "0", "1e200"),
    ];
    // Coordinates are quarter-units in the normalized lens. At x=0 the
    // boundary has y=+/-1/2; its other vertices are (+/-1,0).
    let queries = [(0, 0, Inside), (0, 1, Inside), (0, -1, Inside), (0, 2, Boundary), (0, -2, Boundary), (0, 3, Outside), (0, -3, Outside), (-4, 0, Boundary), (4, 0, Boundary), (8, 0, Outside)];
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        evaluate_under(policy, || {
            for (x, y, scale) in frames {
                let x: Real = x.parse().unwrap();
                let y: Real = y.parse().unwrap();
                let scale: Real = scale.parse().unwrap();
                for retained in [false, true] {
                    for reversed in [false, true] {
                        for warm in [false, true] {
                            let region = lens(&x, &y, &scale, retained, reversed, warm);
                            assert!(!region.is_empty());
                            assert_eq!(region.boundary_loops().len(), 1);
                            // Integral of 1-x^2 on [-1,1], then affine scaling.
                            let area = (Real::from(4) * &scale * &scale / Real::from(3)).unwrap();
                            let actual = region.filled_area().unwrap().unwrap();
                            assert_eq!((&actual - &area).exact_rational_normal_form(), Real::zero().exact_rational());
                            for (dx, dy, expected) in queries {
                                let quarter_x = (Real::from(dx) / Real::from(4)).unwrap();
                                let quarter_y = (Real::from(dy) / Real::from(4)).unwrap();
                                let query = Point2::new(&x + &scale * quarter_x, &y + &scale * quarter_y).into();
                                for current in [&region, &region, &region.clone()] {
                                    assert_eq!(current.classify_point(&query).unwrap(), expected);
                                }
                            }
                        }
                    }
                }
            }
        }).certified().unwrap();
    }
}
