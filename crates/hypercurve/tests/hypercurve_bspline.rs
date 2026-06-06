use hypercurve::{
    BezierBoundaryLoop2, BezierRegion2, BezierSubcurve2, Classification, CurveError, CurvePolicy,
    Point2, PolynomialBSplineCurve2, RationalBSplineCurve2, RationalQuadraticBSplineCurve2, Real,
    RetainedCurveCacheSummary2, RetainedCurveFamily2, RetainedCurveIdentity2,
    RetainedCurvePeriodicity1, RetainedCurveProfile2, RetainedEndpointEvidence2,
    RetainedParameterDomain1, RetainedSpanAxisMonotonicity, RetainedTopologyStatus,
    RetainedTrimDirection, RetainedTrimInterval1,
};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
    }
}

fn assert_topology_error<T>(result: Result<T, CurveError>) {
    assert!(matches!(result, Err(CurveError::Topology(_))));
}

fn assert_point_eq(left: &Point2, right: &Point2) {
    assert_eq!(
        left.x().partial_cmp(right.x()),
        Some(std::cmp::Ordering::Equal)
    );
    assert_eq!(
        left.y().partial_cmp(right.y()),
        Some(std::cmp::Ordering::Equal)
    );
}

#[test]
fn quadratic_bspline_extracts_bezier_spans_by_exact_knot_insertion() {
    let spline = decided(
        PolynomialBSplineCurve2::try_new(
            2,
            vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
            vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
            &policy(),
        )
        .unwrap(),
    );
    let extraction = decided(spline.extract_bezier_spans(&policy()).unwrap());

    assert_eq!(extraction.inserted_knot_count(), 1);
    assert_eq!(extraction.spans().len(), 2);
    match &extraction.spans()[0] {
        BezierSubcurve2::Quadratic(curve) => {
            assert_point_eq(curve.start(), &p(0, 0));
            assert_point_eq(curve.control(), &p(2, 4));
            assert_point_eq(curve.end(), &p(3, 4));
        }
        other => panic!("expected quadratic span, got {other:?}"),
    }
    match &extraction.spans()[1] {
        BezierSubcurve2::Quadratic(curve) => {
            assert_point_eq(curve.start(), &p(3, 4));
            assert_point_eq(curve.control(), &p(4, 4));
            assert_point_eq(curve.end(), &p(6, 0));
        }
        other => panic!("expected quadratic span, got {other:?}"),
    }
}

#[test]
fn cubic_bspline_extracts_spans_with_degree_multiplicity_at_internal_knot() {
    let spline = decided(
        PolynomialBSplineCurve2::try_new(
            3,
            vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
            vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
            &policy(),
        )
        .unwrap(),
    );
    let extraction = decided(spline.extract_bezier_spans(&policy()).unwrap());

    assert_eq!(extraction.inserted_knot_count(), 2);
    assert_eq!(extraction.spans().len(), 2);
    match &extraction.spans()[0] {
        BezierSubcurve2::Cubic(curve) => {
            assert_point_eq(curve.start(), &p(0, 0));
            assert_point_eq(curve.control1(), &p(1, 3));
            assert_point_eq(curve.control2(), &p(2, 3));
            assert_point_eq(curve.end(), &p(3, 3));
        }
        other => panic!("expected cubic span, got {other:?}"),
    }
    match &extraction.spans()[1] {
        BezierSubcurve2::Cubic(curve) => {
            assert_point_eq(curve.start(), &p(3, 3));
            assert_point_eq(curve.control1(), &p(4, 3));
            assert_point_eq(curve.control2(), &p(5, 3));
            assert_point_eq(curve.end(), &p(6, 0));
        }
        other => panic!("expected cubic span, got {other:?}"),
    }
}

#[test]
fn bspline_constructor_rejects_unclamped_or_degenerate_knot_vectors() {
    assert_eq!(
        PolynomialBSplineCurve2::try_new(
            2,
            vec![p(0, 0), p(1, 1), p(2, 0)],
            vec![r(0), r(0), r(1), r(1), r(1), r(1)],
            &policy(),
        ),
        Err(CurveError::InvalidBSpline)
    );
    assert_eq!(
        PolynomialBSplineCurve2::try_new(
            2,
            vec![p(0, 0), p(1, 1), p(2, 0)],
            vec![r(0), r(0), r(0), r(0), r(0), r(0)],
            &policy(),
        ),
        Err(CurveError::InvalidBSpline)
    );
}

#[test]
fn extracted_bspline_spans_feed_existing_bezier_region_area() {
    let upper = decided(
        PolynomialBSplineCurve2::try_new(
            2,
            vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
            vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
            &policy(),
        )
        .unwrap(),
    );
    let lower = decided(
        PolynomialBSplineCurve2::try_new(
            2,
            vec![p(6, 0), p(4, -4), p(2, -4), p(0, 0)],
            vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
            &policy(),
        )
        .unwrap(),
    );
    let mut fragments = Vec::new();
    fragments.extend(
        decided(upper.extract_bezier_spans(&policy()).unwrap())
            .spans()
            .to_vec(),
    );
    fragments.extend(
        decided(lower.extract_bezier_spans(&policy()).unwrap())
            .spans()
            .to_vec(),
    );
    let region = BezierRegion2::new(vec![BezierBoundaryLoop2::new(fragments)]);

    assert_eq!(region.signed_area().unwrap(), Some(q(-88, 3)));
}

#[test]
fn rational_quadratic_bspline_extracts_homogeneous_bezier_spans() {
    let spline = decided(
        RationalQuadraticBSplineCurve2::try_new(
            vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
            vec![r(1), r(2), r(4), r(1)],
            vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
            &policy(),
        )
        .unwrap(),
    );
    let extraction = decided(spline.extract_bezier_spans(&policy()).unwrap());

    assert_eq!(extraction.inserted_knot_count(), 1);
    assert_eq!(extraction.spans().len(), 2);
    assert_eq!(
        extraction.refined_weights(),
        &[r(1), r(2), r(3), r(4), r(1)]
    );
    match &extraction.spans()[0] {
        BezierSubcurve2::RationalQuadratic(curve) => {
            assert_point_eq(curve.start(), &p(0, 0));
            assert_point_eq(curve.control(), &p(2, 4));
            assert_point_eq(curve.end(), &Point2::new(q(10, 3), r(4)));
            assert_eq!(curve.start_weight(), &r(1));
            assert_eq!(curve.control_weight(), &r(2));
            assert_eq!(curve.end_weight(), &r(3));
        }
        other => panic!("expected rational quadratic span, got {other:?}"),
    }
    match &extraction.spans()[1] {
        BezierSubcurve2::RationalQuadratic(curve) => {
            assert_point_eq(curve.start(), &Point2::new(q(10, 3), r(4)));
            assert_point_eq(curve.control(), &p(4, 4));
            assert_point_eq(curve.end(), &p(6, 0));
            assert_eq!(curve.start_weight(), &r(3));
            assert_eq!(curve.control_weight(), &r(4));
            assert_eq!(curve.end_weight(), &r(1));
        }
        other => panic!("expected rational quadratic span, got {other:?}"),
    }
}

#[test]
fn equal_weight_quadratic_nurbs_matches_polynomial_bspline_spans() {
    let controls = vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)];
    let knots = vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)];
    let polynomial = decided(
        PolynomialBSplineCurve2::try_new(2, controls.clone(), knots.clone(), &policy()).unwrap(),
    );
    let rational = decided(
        RationalQuadraticBSplineCurve2::try_new(
            controls,
            vec![r(1), r(1), r(1), r(1)],
            knots,
            &policy(),
        )
        .unwrap(),
    );
    let polynomial = decided(polynomial.extract_bezier_spans(&policy()).unwrap());
    let rational = decided(rational.extract_bezier_spans(&policy()).unwrap());

    for (polynomial_span, rational_span) in polynomial.spans().iter().zip(rational.spans()) {
        let BezierSubcurve2::Quadratic(polynomial) = polynomial_span else {
            panic!("expected polynomial quadratic")
        };
        let BezierSubcurve2::RationalQuadratic(rational) = rational_span else {
            panic!("expected rational quadratic")
        };
        assert_point_eq(polynomial.start(), rational.start());
        assert_point_eq(polynomial.control(), rational.control());
        assert_point_eq(polynomial.end(), rational.end());
        assert_eq!(
            rational.weights(),
            [&Real::one(), &Real::one(), &Real::one()]
        );
    }
}

#[test]
fn retained_rational_cubic_bspline_extracts_bezier_span_reports() {
    let spline = decided(
        RationalBSplineCurve2::try_new(
            3,
            vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
            vec![r(1), r(2), r(4), r(8), r(16)],
            vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
            &policy(),
        )
        .unwrap(),
    );
    let extraction = decided(spline.extract_bezier_spans(&policy()).unwrap());

    assert_eq!(spline.degree(), 3);
    assert_eq!(extraction.degree(), 3);
    assert_eq!(extraction.inserted_knot_count(), 2);
    assert_eq!(extraction.refined_control_points().len(), 7);
    assert_eq!(extraction.refined_weights().len(), 7);
    assert_eq!(extraction.spans().len(), 2);
    for span in extraction.spans() {
        assert_eq!(span.degree(), 3);
        assert_eq!(span.control_points().len(), 4);
        assert_eq!(span.weights().len(), 4);
    }
    assert_eq!(extraction.spans()[0].knot_interval(), (&r(0), &r(1)));
    assert_eq!(extraction.spans()[1].knot_interval(), (&r(1), &r(2)));
    assert_point_eq(
        &extraction.spans()[0].control_points()[3],
        &extraction.spans()[1].control_points()[0],
    );
    assert_eq!(
        extraction.spans()[0].weights()[3],
        extraction.spans()[1].weights()[0]
    );
}

#[test]
fn equal_weight_retained_rational_cubic_matches_polynomial_cubic_spans() {
    let controls = vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)];
    let knots = vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)];
    let polynomial = decided(
        PolynomialBSplineCurve2::try_new(3, controls.clone(), knots.clone(), &policy()).unwrap(),
    );
    let rational = decided(
        RationalBSplineCurve2::try_new(3, controls, vec![r(1); 5], knots, &policy()).unwrap(),
    );
    let polynomial = decided(polynomial.extract_bezier_spans(&policy()).unwrap());
    let rational = decided(rational.extract_bezier_spans(&policy()).unwrap());

    assert_eq!(rational.spans().len(), polynomial.spans().len());
    for (polynomial_span, rational_span) in polynomial.spans().iter().zip(rational.spans()) {
        let BezierSubcurve2::Cubic(polynomial) = polynomial_span else {
            panic!("expected polynomial cubic")
        };
        assert_eq!(rational_span.degree(), 3);
        assert_point_eq(polynomial.start(), &rational_span.control_points()[0]);
        assert_point_eq(polynomial.control1(), &rational_span.control_points()[1]);
        assert_point_eq(polynomial.control2(), &rational_span.control_points()[2]);
        assert_point_eq(polynomial.end(), &rational_span.control_points()[3]);
        assert_eq!(rational_span.weights(), &[r(1), r(1), r(1), r(1)]);
    }
}

#[test]
fn retained_rational_quadratic_spans_promote_to_native_conic_topology() {
    let spline = decided(
        RationalBSplineCurve2::try_new(
            2,
            vec![p(0, 0), p(2, 4), p(4, 0)],
            vec![r(1), r(2), r(3)],
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            &policy(),
        )
        .unwrap(),
    );
    let extraction = decided(spline.extract_bezier_spans(&policy()).unwrap());
    let native = decided(extraction.native_subcurves(&policy()).unwrap());

    assert_eq!(native.len(), 1);
    match &native[0] {
        BezierSubcurve2::RationalQuadratic(curve) => {
            assert_point_eq(curve.start(), &p(0, 0));
            assert_point_eq(curve.control(), &p(2, 4));
            assert_point_eq(curve.end(), &p(4, 0));
        }
        other => panic!("expected native rational quadratic span, got {other:?}"),
    }
}

#[test]
fn equal_weight_retained_rational_cubic_spans_feed_native_region_area() {
    let upper = decided(
        RationalBSplineCurve2::try_new(
            3,
            vec![p(0, 0), p(1, 3), p(5, 3), p(6, 0)],
            vec![r(7), r(7), r(7), r(7)],
            vec![r(0), r(0), r(0), r(0), r(1), r(1), r(1), r(1)],
            &policy(),
        )
        .unwrap(),
    );
    let lower = decided(
        RationalBSplineCurve2::try_new(
            3,
            vec![p(6, 0), p(5, -3), p(1, -3), p(0, 0)],
            vec![r(7), r(7), r(7), r(7)],
            vec![r(0), r(0), r(0), r(0), r(1), r(1), r(1), r(1)],
            &policy(),
        )
        .unwrap(),
    );
    let mut fragments = Vec::new();
    fragments.extend(decided(
        decided(upper.extract_bezier_spans(&policy()).unwrap())
            .native_subcurves(&policy())
            .unwrap(),
    ));
    fragments.extend(decided(
        decided(lower.extract_bezier_spans(&policy()).unwrap())
            .native_subcurves(&policy())
            .unwrap(),
    ));
    let region = BezierRegion2::new(vec![BezierBoundaryLoop2::new(fragments)]);

    assert!(region.signed_area().unwrap().is_some());
}

#[test]
fn nonuniform_retained_rational_cubic_spans_do_not_promote_to_native_topology() {
    let spline = decided(
        RationalBSplineCurve2::try_new(
            3,
            vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
            vec![r(1), r(2), r(4), r(8), r(16)],
            vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
            &policy(),
        )
        .unwrap(),
    );
    let extraction = decided(spline.extract_bezier_spans(&policy()).unwrap());
    let report = decided(extraction.native_topology_report(&policy()).unwrap());

    assert!(!report.is_fully_native_exact());
    assert_eq!(report.span_reports().len(), extraction.spans().len());
    assert!(report.span_reports().iter().all(|span| {
        span.degree() == 3
            && span.status() == RetainedTopologyStatus::Unsupported
            && span.native_subcurve().is_none()
            && span.status().is_retained_evidence()
    }));

    assert_eq!(
        extraction.native_subcurves(&policy()).unwrap(),
        Classification::Uncertain(hypercurve::UncertaintyReason::Unsupported)
    );
}

#[test]
fn equal_weight_rational_cubic_topology_report_names_native_exact_spans() {
    let spline = decided(
        RationalBSplineCurve2::try_new(
            3,
            vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
            vec![r(5), r(5), r(5), r(5), r(5)],
            vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
            &policy(),
        )
        .unwrap(),
    );
    let extraction = decided(spline.extract_bezier_spans(&policy()).unwrap());
    let report = decided(extraction.native_topology_report(&policy()).unwrap());

    assert!(report.is_fully_native_exact());
    assert_eq!(report.span_reports().len(), 2);
    for (index, span) in report.span_reports().iter().enumerate() {
        assert_eq!(span.span_index(), index);
        assert_eq!(span.degree(), 3);
        assert_eq!(span.status(), RetainedTopologyStatus::NativeExact);
        assert!(matches!(
            span.native_subcurve(),
            Some(BezierSubcurve2::Cubic(_))
        ));
    }

    let native = decided(extraction.native_subcurves(&policy()).unwrap());
    assert_eq!(native.len(), report.span_reports().len());
}

#[test]
fn retained_bspline_profile_reports_exact_domain_trim_and_endpoints() {
    let spline = decided(
        PolynomialBSplineCurve2::try_new(
            3,
            vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
            vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
            &policy(),
        )
        .unwrap(),
    );
    let profile = decided(spline.retained_curve_profile(42, &policy()).unwrap());

    assert_eq!(
        profile.identity().family(),
        RetainedCurveFamily2::PolynomialBSpline
    );
    assert_eq!(profile.identity().source_index(), 42);
    assert_eq!(profile.domain().start(), &r(0));
    assert_eq!(profile.domain().end(), &r(2));
    assert_eq!(profile.trim().start(), &r(0));
    assert_eq!(profile.trim().end(), &r(2));
    assert_eq!(profile.trim().direction(), RetainedTrimDirection::Forward);
    assert_eq!(
        profile.periodicity(),
        &RetainedCurvePeriodicity1::NonPeriodic
    );
    assert_eq!(
        profile.topology_status(),
        RetainedTopologyStatus::NativeExact
    );
    assert_eq!(profile.endpoints().start_parameter(), &r(0));
    assert_eq!(profile.endpoints().end_parameter(), &r(2));
    assert_eq!(profile.endpoints().start_point(), &p(0, 0));
    assert_eq!(profile.endpoints().end_point(), &p(6, 0));
    assert_eq!(profile.cache_summary().control_count(), 5);
    assert_eq!(profile.cache_summary().knot_count(), 9);
    assert_eq!(profile.cache_summary().span_count(), 2);
    assert_eq!(profile.cache_summary().native_span_count(), 2);
    assert_eq!(profile.cache_summary().retained_span_count(), 0);
}

#[test]
fn retained_curve_cache_summary_rejects_inconsistent_span_counts() {
    assert_topology_error(RetainedCurveCacheSummary2::new(5, 9, 2, 2, 1));
    assert_topology_error(RetainedCurveCacheSummary2::new(5, 9, 0, 0, 0));
    assert_topology_error(RetainedCurveCacheSummary2::new(0, 9, 2, 2, 0));
}

#[test]
fn retained_curve_profile_rejects_mismatched_endpoint_evidence_without_blocking_trim() {
    let policy = policy();
    let domain = decided(RetainedParameterDomain1::try_new(r(0), r(2), &policy).unwrap());
    let full_trim = decided(RetainedTrimInterval1::try_new(r(0), r(2), &domain, &policy).unwrap());
    let partial_trim =
        decided(RetainedTrimInterval1::try_new(r(0), r(1), &domain, &policy).unwrap());
    let cache = RetainedCurveCacheSummary2::new(5, 9, 2, 2, 0).unwrap();
    let identity = RetainedCurveIdentity2::new(RetainedCurveFamily2::PolynomialBSpline, 42);
    let endpoints = RetainedEndpointEvidence2::new(r(0), r(2), p(0, 0), p(2, 0));
    RetainedCurveProfile2::new(
        identity,
        domain.clone(),
        full_trim.clone(),
        RetainedCurvePeriodicity1::NonPeriodic,
        RetainedTopologyStatus::NativeExact,
        endpoints,
        cache.clone(),
    )
    .unwrap();

    let endpoints = RetainedEndpointEvidence2::new(r(0), r(2), p(0, 0), p(2, 0));
    RetainedCurveProfile2::new(
        identity,
        domain.clone(),
        partial_trim,
        RetainedCurvePeriodicity1::NonPeriodic,
        RetainedTopologyStatus::NativeExact,
        endpoints,
        cache.clone(),
    )
    .unwrap();

    let bad_endpoints = RetainedEndpointEvidence2::new(r(0), r(1), p(0, 0), p(2, 0));
    assert_topology_error(RetainedCurveProfile2::new(
        identity,
        domain.clone(),
        full_trim,
        RetainedCurvePeriodicity1::NonPeriodic,
        RetainedTopologyStatus::NativeExact,
        bad_endpoints,
        cache.clone(),
    ));
}

#[test]
fn retained_rational_cubic_profile_keeps_unsupported_spans_as_evidence() {
    let spline = decided(
        RationalBSplineCurve2::try_new(
            3,
            vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
            vec![r(1), r(2), r(4), r(8), r(16)],
            vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
            &policy(),
        )
        .unwrap(),
    );
    let profile = decided(spline.retained_curve_profile(7, &policy()).unwrap());

    assert_eq!(
        profile.identity().family(),
        RetainedCurveFamily2::RationalBSpline
    );
    assert_eq!(
        profile.topology_status(),
        RetainedTopologyStatus::Unsupported
    );
    assert_eq!(profile.cache_summary().span_count(), 2);
    assert_eq!(profile.cache_summary().native_span_count(), 0);
    assert_eq!(profile.cache_summary().retained_span_count(), 2);
}

#[test]
fn retained_bspline_span_facts_report_native_bounds_and_monotonicity() {
    let spline = decided(
        PolynomialBSplineCurve2::try_new(
            2,
            vec![p(0, 0), p(1, 0), p(2, 0)],
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            &policy(),
        )
        .unwrap(),
    );
    let extraction = decided(spline.extract_bezier_spans(&policy()).unwrap());
    let facts = decided(extraction.span_fact_report(&policy()).unwrap());

    assert_eq!(facts.span_facts().len(), 1);
    let span = &facts.span_facts()[0];
    assert_eq!(span.knot_interval(), (&r(0), &r(1)));
    assert_eq!(span.bounds().min(), &p(0, 0));
    assert_eq!(span.bounds().max(), &p(2, 0));
    assert_eq!(
        span.x_monotonicity(),
        RetainedSpanAxisMonotonicity::CertifiedMonotone
    );
    assert_eq!(
        span.y_monotonicity(),
        RetainedSpanAxisMonotonicity::CertifiedMonotone
    );
    assert_eq!(span.topology_status(), RetainedTopologyStatus::NativeExact);
    assert!(span.weight_domain().is_none());
}

#[test]
fn retained_rational_quadratic_span_facts_include_weight_domain() {
    let spline = decided(
        RationalQuadraticBSplineCurve2::try_new(
            vec![p(0, 0), p(1, 1), p(2, 0)],
            vec![r(1), r(2), r(3)],
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            &policy(),
        )
        .unwrap(),
    );
    let extraction = decided(spline.extract_bezier_spans(&policy()).unwrap());
    let facts = decided(extraction.span_fact_report(&policy()).unwrap());
    let weight_domain = facts.span_facts()[0]
        .weight_domain()
        .expect("rational span reports weights");

    assert_eq!(weight_domain.weight_count(), 3);
    assert_eq!(weight_domain.certified_nonzero_count(), 3);
    assert!(weight_domain.all_weights_certified_nonzero());
    assert_eq!(
        facts.span_facts()[0].topology_status(),
        RetainedTopologyStatus::NativeExact
    );
}

#[test]
fn retained_rational_quadratic_span_facts_follow_refined_knot_windows() {
    let spline = decided(
        RationalQuadraticBSplineCurve2::try_new(
            vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
            vec![r(1), r(2), r(4), r(1)],
            vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
            &policy(),
        )
        .unwrap(),
    );
    let extraction = decided(spline.extract_bezier_spans(&policy()).unwrap());
    let facts = decided(extraction.span_fact_report(&policy()).unwrap());

    assert_eq!(facts.span_facts().len(), 2);
    assert_eq!(facts.span_facts()[0].knot_interval(), (&r(0), &r(1)));
    assert_eq!(facts.span_facts()[1].knot_interval(), (&r(1), &r(2)));
    assert!(
        facts
            .span_facts()
            .iter()
            .all(|span| span
                .weight_domain()
                .is_some_and(|weights| weights.weight_count() == 3
                    && weights.certified_nonzero_count() == 3
                    && weights.all_weights_certified_nonzero()))
    );
}

#[test]
fn retained_rational_cubic_span_facts_keep_control_hull_and_unsupported_monotonicity() {
    let spline = decided(
        RationalBSplineCurve2::try_new(
            3,
            vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
            vec![r(1), r(2), r(4), r(8), r(16)],
            vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
            &policy(),
        )
        .unwrap(),
    );
    let extraction = decided(spline.extract_bezier_spans(&policy()).unwrap());
    let facts = decided(extraction.span_fact_report(&policy()).unwrap());

    assert_eq!(facts.span_facts().len(), 2);
    assert!(facts.span_facts().iter().all(|span| {
        span.topology_status() == RetainedTopologyStatus::Unsupported
            && span.x_monotonicity() == RetainedSpanAxisMonotonicity::Unsupported
            && span.y_monotonicity() == RetainedSpanAxisMonotonicity::Unsupported
            && span
                .weight_domain()
                .is_some_and(|weights| weights.all_weights_certified_nonzero())
    }));
    assert_eq!(facts.span_facts()[0].bounds().min(), &p(0, 0));
    assert_eq!(
        facts.span_facts()[0].bounds().max(),
        &Point2::new(q(11, 3), r(3))
    );
}

#[test]
fn retained_trim_interval_rejects_out_of_domain_and_accepts_reversal() {
    let spline = decided(
        PolynomialBSplineCurve2::try_new(
            2,
            vec![p(0, 0), p(1, 2), p(3, 2), p(4, 0)],
            vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
            &policy(),
        )
        .unwrap(),
    );
    let profile = decided(spline.retained_curve_profile(9, &policy()).unwrap());
    let reversed =
        decided(RetainedTrimInterval1::try_new(r(2), r(0), profile.domain(), &policy()).unwrap());

    assert_eq!(reversed.direction(), RetainedTrimDirection::Reversed);
    assert_eq!(
        RetainedTrimInterval1::try_new(r(3), r(0), profile.domain(), &policy()),
        Err(CurveError::InvalidBezierRange)
    );
}

#[test]
fn retained_rational_bspline_rejects_unsupported_degree_and_zero_weight() {
    assert_eq!(
        RationalBSplineCurve2::try_new(
            1,
            vec![p(0, 0), p(1, 1)],
            vec![r(1), r(1)],
            vec![r(0), r(0), r(1), r(1)],
            &policy(),
        )
        .unwrap(),
        Classification::Uncertain(hypercurve::UncertaintyReason::Unsupported)
    );
    assert_eq!(
        RationalBSplineCurve2::try_new(usize::MAX, Vec::new(), Vec::new(), Vec::new(), &policy())
            .unwrap(),
        Classification::Uncertain(hypercurve::UncertaintyReason::Unsupported)
    );
    assert_eq!(
        RationalBSplineCurve2::try_new(
            3,
            vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
            vec![r(1), r(2), r(0), r(8), r(16)],
            vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
            &policy(),
        ),
        Err(CurveError::ZeroRationalBezierWeight)
    );
}

#[test]
fn rational_bspline_rejects_zero_or_uncertain_refined_weights() {
    assert_eq!(
        RationalQuadraticBSplineCurve2::try_new(
            vec![p(0, 0), p(1, 1), p(2, 1)],
            vec![r(1), r(0), r(1)],
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            &policy(),
        ),
        Err(CurveError::ZeroRationalBezierWeight)
    );

    let spline = decided(
        RationalQuadraticBSplineCurve2::try_new(
            vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
            vec![r(1), r(1), r(-1), r(1)],
            vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
            &policy(),
        )
        .unwrap(),
    );
    assert_eq!(
        spline.extract_bezier_spans(&policy()),
        Err(CurveError::ZeroRationalBezierWeight)
    );
}

#[test]
fn extracted_rational_bspline_spans_feed_conic_region_area() {
    let upper = decided(
        RationalQuadraticBSplineCurve2::try_new(
            vec![p(0, 0), p(2, 2), p(4, 2), p(6, 0)],
            vec![r(1), q(1, 2), q(1, 2), r(1)],
            vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
            &policy(),
        )
        .unwrap(),
    );
    let lower = decided(
        RationalQuadraticBSplineCurve2::try_new(
            vec![p(6, 0), p(4, -2), p(2, -2), p(0, 0)],
            vec![r(1), q(1, 2), q(1, 2), r(1)],
            vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
            &policy(),
        )
        .unwrap(),
    );
    let mut fragments = Vec::new();
    fragments.extend(
        decided(upper.extract_bezier_spans(&policy()).unwrap())
            .spans()
            .to_vec(),
    );
    fragments.extend(
        decided(lower.extract_bezier_spans(&policy()).unwrap())
            .spans()
            .to_vec(),
    );
    let region = BezierRegion2::new(vec![BezierBoundaryLoop2::new(fragments)]);

    assert!(region.signed_area().unwrap().is_some());
}
