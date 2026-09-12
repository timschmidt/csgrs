use hypercurve::{
    Classification, Curve2, CurveCertainty, CurveContext, CurvePath2, CurveRegion2,
    CurveRegionLoopRole, FillRule, LineSeg2, OffsetCornerStyle2, Point2, RationalBezier2, Real,
    RegionPointLocation,
};

fn point(x: i32, y: i32) -> Point2 {
    Point2::from_values(x, y)
}

fn ratio(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn rational_convex_cap() -> CurveRegion2 {
    // W(t) = 1 + 5t + t^5 is positive. The x-derivative numerator is
    // strictly positive and the curvature numerator is negative inside
    // (0, 1), so this bounds a simple convex cap with the closing line.
    // Both endpoint curvatures vanish, but the squared speeds are 200 and
    // 1250/49: a parallel derivative is not a unit tangent.
    let curve = RationalBezier2::try_new(
        vec![
            point(0, 0),
            point(1, 1),
            point(2, 2),
            point(3, 2),
            point(4, 1),
            point(5, 0),
        ],
        [1, 2, 3, 4, 5, 7].into_iter().map(Real::from).collect(),
    )
    .unwrap();
    let path = CurvePath2::try_new(vec![
        Curve2::from(curve),
        Curve2::from(LineSeg2::try_new(point(5, 0), point(0, 0)).unwrap()),
    ])
    .unwrap();
    CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value()
}

#[test]
fn normalized_rational_cap_round_offset_retains_exact_unit_directions() {
    let cap = rational_convex_cap()
        .regularized_region(&CurveContext::STRICT)
        .unwrap()
        .into_value();
    let offset = cap
        .offset(
            ratio(1, 10),
            &OffsetCornerStyle2::Round,
            &CurveContext::STRICT,
        )
        .expect("a convex rational cap has an exactly representable round offset");
    assert_eq!(offset.certainty, CurveCertainty::Certified);
    assert_eq!(offset.value.boundary_loops().len(), 1);

    for (sample, expected) in [
        (point(2, 1), RegionPointLocation::Inside),
        (point(0, 0), RegionPointLocation::Inside),
        (
            Point2::new(Real::from(2), ratio(-1, 20)),
            RegionPointLocation::Inside,
        ),
        (
            Point2::new(Real::from(2), ratio(-1, 5)),
            RegionPointLocation::Outside,
        ),
        (point(2, 3), RegionPointLocation::Outside),
    ] {
        let location = offset
            .value
            .classify_point(&sample, &CurveContext::STRICT)
            .unwrap();
        assert_eq!(location.certainty, CurveCertainty::Certified);
        assert_eq!(location.value, Classification::Decided(expected));
    }
}
