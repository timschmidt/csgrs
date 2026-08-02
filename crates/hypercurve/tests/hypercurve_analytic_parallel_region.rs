use hypercurve::{
    BezierParallelFragment2, BezierParameter2, BezierParameterRange2, BezierRetainedCurveEnvelope2,
    BezierRetainedEndpointEnvelope2, BezierSplitFragment2, Classification, CurveContext,
    CurveRegion2, CurveRegionBoundaryLoop2, Point2, QuadraticBezier2, Real,
};

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn exact_parameter(value: i64, policy: &CurveContext) -> BezierParameter2 {
    match BezierParameter2::exact(Real::from(value), policy).unwrap() {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => panic!("unexpected parameter uncertainty: {reason:?}"),
    }
}

fn range(start: i64, end: i64, policy: &CurveContext) -> BezierParameterRange2 {
    match BezierParameterRange2::try_new(
        exact_parameter(start, policy),
        exact_parameter(end, policy),
        policy,
    )
    .unwrap()
    {
        Classification::Decided(range) => range,
        Classification::Uncertain(reason) => panic!("unexpected range uncertainty: {reason:?}"),
    }
}

fn line_parallel_fragment(
    start: Point2,
    midpoint: Point2,
    end: Point2,
    distance: i64,
    start_parameter: i64,
    end_parameter: i64,
    policy: &CurveContext,
) -> BezierParallelFragment2 {
    let parallel = QuadraticBezier2::new(start, midpoint, end)
        .parallel_left(Real::from(distance))
        .unwrap();
    match BezierParallelFragment2::try_new(
        parallel,
        range(start_parameter, end_parameter, policy),
        policy,
    )
    .unwrap()
    {
        Classification::Decided(fragment) => fragment,
        Classification::Uncertain(reason) => {
            panic!("unexpected analytic-parallel uncertainty: {reason:?}")
        }
    }
}

fn assert_real_equal(left: &Real, right: &Real) {
    assert_eq!(left.partial_cmp(right), Some(std::cmp::Ordering::Equal));
}

fn check_policy(policy: CurveContext) {
    let fragment = line_parallel_fragment(point(0, 0), point(2, 0), point(4, 0), 1, 1, 0, &policy);
    assert!(fragment.is_reversed());
    assert_eq!(
        fragment.range().exact_endpoints(),
        Some((&Real::zero(), &Real::one()))
    );
    let representative = match fragment.representative_point(&policy).unwrap() {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            panic!("unexpected representative-point uncertainty: {reason:?}")
        }
    };
    assert_real_equal(representative.x(), &Real::from(2));
    assert_real_equal(representative.y(), &Real::one());

    let edges = [
        (point(0, 0), point(2, 0), point(4, 0)),
        (point(4, 0), point(4, 2), point(4, 4)),
        (point(4, 4), point(2, 4), point(0, 4)),
        (point(0, 4), point(0, 2), point(0, 0)),
    ];
    let fragments = edges
        .into_iter()
        .map(|(start, midpoint, end)| {
            BezierSplitFragment2::AnalyticParallel(line_parallel_fragment(
                start, midpoint, end, 0, 0, 1, &policy,
            ))
        })
        .collect();
    let boundary = CurveRegionBoundaryLoop2::new(fragments, &policy).unwrap();

    let endpoint_envelope = match BezierRetainedEndpointEnvelope2::from_loop(&boundary, &policy) {
        Classification::Decided(envelope) => envelope,
        Classification::Uncertain(reason) => {
            panic!("unexpected endpoint-envelope uncertainty: {reason:?}")
        }
    };
    assert_eq!(endpoint_envelope.native_endpoint_count(), 8);

    let region = CurveRegion2::new(vec![boundary]).unwrap();
    assert!(region.has_algebraic_fragments());
    let curve_envelope = match BezierRetainedCurveEnvelope2::from_region(&region, &policy) {
        Classification::Decided(envelope) => envelope,
        Classification::Uncertain(reason) => {
            panic!("unexpected curve-envelope uncertainty: {reason:?}")
        }
    };
    assert_eq!(curve_envelope.exact_fragment_count(), 4);
    assert_real_equal(curve_envelope.envelope().min_x(), &Real::zero());
    assert_real_equal(curve_envelope.envelope().min_y(), &Real::zero());
    assert_real_equal(curve_envelope.envelope().max_x(), &Real::from(4));
    assert_real_equal(curve_envelope.envelope().max_y(), &Real::from(4));
}

#[test]
fn analytic_parallel_fragments_retain_exact_region_evidence_under_both_policies() {
    check_policy(CurveContext::STRICT);
    check_policy(CurveContext::APPROXIMATE_512);
}
