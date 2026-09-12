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
    assert_cap_round_offset(&cap);
}

#[test]
fn authored_rational_cap_offset_does_not_require_a_signed_area_representation() {
    let cap = rational_convex_cap();
    assert_eq!(
        cap.signed_area(&CurveContext::STRICT).unwrap().value,
        Classification::Decided(None),
    );
    assert_cap_round_offset(&cap);
}

fn assert_cap_round_offset(cap: &CurveRegion2) {
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

#[test]
fn zero_offset_regularizes_overlapping_authored_material() {
    let rectangle = |left, right| {
        let points = [
            point(left, 0),
            point(right, 0),
            point(right, 2),
            point(left, 2),
            point(left, 0),
        ];
        CurvePath2::try_new(
            points
                .windows(2)
                .map(|pair| {
                    Curve2::from(LineSeg2::try_new(pair[0].clone(), pair[1].clone()).unwrap())
                })
                .collect(),
        )
        .unwrap()
    };
    let region = CurveRegion2::try_from_signed_boundary_paths_with_loop_semantics(
        &[rectangle(0, 3), rectangle(2, 5)],
        &[CurveRegionLoopRole::Material; 2],
        &[FillRule::NonZero; 2],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let result = region
        .offset(
            Real::zero(),
            &OffsetCornerStyle2::Round,
            &CurveContext::STRICT,
        )
        .unwrap();
    assert_eq!(result.certainty, CurveCertainty::Certified);
    assert_eq!(result.value.boundary_loops().len(), 1);
    assert_eq!(
        result
            .value
            .filled_area(&CurveContext::STRICT)
            .unwrap()
            .value,
        Classification::Decided(Some(Real::from(10))),
    );

    #[cfg(feature = "dispatch-trace")]
    {
        hyperreal::dispatch_trace::reset();
        let replayed = hyperreal::dispatch_trace::with_recording(|| {
            result.value.regularized_region(&CurveContext::STRICT)
        })
        .unwrap();
        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(replayed.value, result.value);
        assert_eq!(
            trace.path_count("hypercurve", "regularization-successor", "forced-bijection"),
            0,
            "a compact native boundary must retain its normalization certificate",
        );
    }
}

#[test]
fn curved_offset_regularizes_interior_folds_despite_convex_endpoint_turns() {
    // The upper boundary is y = 1 - 8*x^2*(1-x)^2. Every corner
    // turns left, but the upper curve is concave near its middle. At distance
    // 1/4 its parallel folds: the middle parameter maps to (1/2, 3/4),
    // which must be strictly inside the regularized dilation.
    let top = RationalBezier2::try_new(
        vec![
            point(1, 1),
            Point2::new(ratio(3, 4), Real::one()),
            Point2::new(ratio(1, 2), ratio(-1, 3)),
            Point2::new(ratio(1, 4), Real::one()),
            point(0, 1),
        ],
        vec![Real::one(); 5],
    )
    .unwrap();
    let path = CurvePath2::try_new(vec![
        LineSeg2::try_new(point(0, 0), point(1, 0)).unwrap().into(),
        LineSeg2::try_new(point(1, 0), point(1, 1)).unwrap().into(),
        top.into(),
        LineSeg2::try_new(point(0, 1), point(0, 0)).unwrap().into(),
    ])
    .unwrap();
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let sample = Point2::new(ratio(1, 2), ratio(3, 4));
    let source_witness = Point2::new(ratio(1, 3), ratio(49, 81));
    // This exact boundary witness is strictly less than 1/4 from sample:
    // squared distance is 5125/104976 < 1/16.
    assert_eq!(
        source_witness.distance_squared(&sample),
        ratio(5125, 104976)
    );
    assert_eq!(
        region
            .classify_point(&source_witness, &CurveContext::STRICT)
            .unwrap()
            .value,
        Classification::Decided(RegionPointLocation::Boundary),
    );
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::reset();
    #[cfg(feature = "dispatch-trace")]
    let _trace_guard = hyperreal::dispatch_trace::recording_scope();
    let offset = region
        .offset(
            ratio(1, 4),
            &OffsetCornerStyle2::Round,
            &CurveContext::STRICT,
        )
        .unwrap();
    assert_eq!(offset.certainty, CurveCertainty::Certified);
    assert_eq!(offset.value.boundary_loops().len(), 1);
    #[cfg(feature = "dispatch-trace")]
    assert_eq!(
        hyperreal::dispatch_trace::take_trace().path_count(
            "hypercurve",
            "curve-region-exact-offset-regularization",
            "convex-boundary-certificate"
        ),
        0,
        "left-turning junctions alone do not certify a convex curve",
    );
    for sample in [
        Point2::new(ratio(1, 2), ratio(74, 100)),
        sample,
        Point2::new(ratio(1, 2), ratio(76, 100)),
    ] {
        assert_eq!(
            offset
                .value
                .classify_point(&sample, &CurveContext::STRICT)
                .unwrap()
                .value,
            Classification::Decided(RegionPointLocation::Inside),
            "the raw fold and both incident faces belong to the regularized dilation",
        );
    }
}
