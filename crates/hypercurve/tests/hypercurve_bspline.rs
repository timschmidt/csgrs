use hypercurve::{
    Aabb2, BezierBoundaryLoop2, BezierRegion2, BezierSubcurve2, Classification, CurveError,
    CurvePolicy, Point2, PolynomialBSplineCurve2, QuadraticBezier2, RationalBSplineCurve2,
    RationalBSplineNativeTopologyEvidence2, RationalBezier2, RationalBezierSpanTopologyEvidence2,
    RationalBezierSpanTopologyPath2, RationalQuadraticBSplineCurve2, Real,
    RetainedBSplineSpanFactEvidence2, RetainedBSplineSpanFacts2, RetainedSpanAxisMonotonicity,
    RetainedSpanWeightDomainEvidence2, RetainedTopologyStatus,
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
    CurvePolicy::STRICT
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

fn span_topology_evidence(
    span_index: usize,
    degree: usize,
    knot_start: Real,
    knot_end: Real,
    status: RetainedTopologyStatus,
    decision_path: RationalBezierSpanTopologyPath2,
    native_subcurve: Option<BezierSubcurve2>,
) -> Result<RationalBezierSpanTopologyEvidence2, CurveError> {
    RationalBezierSpanTopologyEvidence2::new(
        span_index,
        degree,
        knot_start,
        knot_end,
        status,
        decision_path,
        native_subcurve,
    )
}

fn general_rational_cubic() -> BezierSubcurve2 {
    BezierSubcurve2::Rational(
        RationalBezier2::try_new(
            vec![p(0, 0), p(1, 3), p(3, 3), p(4, 0)],
            vec![r(1), r(2), r(3), r(4)],
        )
        .unwrap(),
    )
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
fn linear_bspline_spans_are_elevated_exactly() {
    let spline = decided(
        PolynomialBSplineCurve2::try_new(
            1,
            vec![p(0, 0), p(2, 2), p(4, 0)],
            vec![r(0), r(0), r(1), r(2), r(2)],
            &policy(),
        )
        .unwrap(),
    );
    let extraction = decided(spline.extract_bezier_spans(&policy()).unwrap());

    assert_eq!(extraction.degree(), 1);
    assert_eq!(extraction.spans().len(), 2);
    let BezierSubcurve2::Quadratic(first) = &extraction.spans()[0] else {
        panic!("linear span was not elevated to a quadratic");
    };
    assert_point_eq(first.start(), &p(0, 0));
    assert_point_eq(first.control(), &p(1, 1));
    assert_point_eq(first.end(), &p(2, 2));
}

#[test]
fn rational_linear_span_preserves_homogeneous_parameterization() {
    let spline = decided(
        RationalBSplineCurve2::try_new(
            1,
            vec![p(0, 0), p(4, 0)],
            vec![r(1), r(3)],
            vec![r(0), r(0), r(1), r(1)],
            &policy(),
        )
        .unwrap(),
    );
    let extraction = decided(spline.extract_bezier_spans(&policy()).unwrap());
    let evidence = decided(extraction.native_topology_evidence(&policy()).unwrap());

    assert_eq!(evidence.span_evidence().len(), 1);
    assert_eq!(
        evidence.span_evidence()[0].decision_path(),
        RationalBezierSpanTopologyPath2::NativeRationalLinearSpan
    );
    let Some(BezierSubcurve2::RationalQuadratic(curve)) =
        evidence.span_evidence()[0].native_subcurve()
    else {
        panic!("linear NURBS span was not elevated homogeneously");
    };
    assert_point_eq(curve.control(), &p(3, 0));
    assert_eq!(curve.weights(), [&r(1), &r(2), &r(3)]);
}

#[test]
fn singular_rational_linear_elevation_stays_retained() {
    let spline = decided(
        RationalBSplineCurve2::try_new(
            1,
            vec![p(0, 0), p(4, 0)],
            vec![r(1), r(-1)],
            vec![r(0), r(0), r(1), r(1)],
            &policy(),
        )
        .unwrap(),
    );
    let extraction = decided(spline.extract_bezier_spans(&policy()).unwrap());
    let evidence = decided(extraction.native_topology_evidence(&policy()).unwrap());

    assert_eq!(
        evidence.span_evidence()[0].decision_path(),
        RationalBezierSpanTopologyPath2::RetainedSingularLinearSpan
    );
    assert_eq!(
        extraction.native_subcurves(&policy()).unwrap(),
        Classification::Uncertain(hypercurve::UncertaintyReason::Unsupported)
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
fn bspline_constructor_rejects_degenerate_knot_vectors() {
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
fn unclamped_uniform_bspline_refines_active_domain_endpoints_exactly() {
    let spline = decided(
        PolynomialBSplineCurve2::try_new(
            2,
            vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
            (0..=6).map(r).collect(),
            &policy(),
        )
        .unwrap(),
    );
    let extraction = decided(spline.extract_bezier_spans(&policy()).unwrap());

    assert_eq!(extraction.inserted_knot_count(), 3);
    assert_eq!(extraction.spans().len(), 2);
    let BezierSubcurve2::Quadratic(first) = &extraction.spans()[0] else {
        panic!("unclamped quadratic did not extract a quadratic first span");
    };
    let BezierSubcurve2::Quadratic(second) = &extraction.spans()[1] else {
        panic!("unclamped quadratic did not extract a quadratic second span");
    };
    assert_eq!(
        first.control_points(),
        [&Point2::new(r(1), r(2)), &p(2, 4), &p(3, 4)]
    );
    assert_eq!(
        second.control_points(),
        [&p(3, 4), &p(4, 4), &Point2::new(r(5), r(2))]
    );

    assert_eq!(first.start(), &Point2::new(r(1), r(2)));
    assert_eq!(second.end(), &Point2::new(r(5), r(2)));

    let facts = decided(extraction.span_fact_evidence(&policy()).unwrap());
    assert_eq!(facts.span_facts().len(), 2);
    assert_eq!(facts.span_facts()[0].knot_interval(), (&r(2), &r(3)));
    assert_eq!(facts.span_facts()[1].knot_interval(), (&r(3), &r(4)));

    let rational = decided(
        RationalQuadraticBSplineCurve2::try_new(
            vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
            vec![r(1), r(2), r(3), r(4)],
            (0..=6).map(r).collect(),
            &policy(),
        )
        .unwrap(),
    );
    let rational_extraction = decided(rational.extract_bezier_spans(&policy()).unwrap());
    let rational_facts = decided(rational_extraction.span_fact_evidence(&policy()).unwrap());
    assert_eq!(rational_facts.span_facts().len(), 2);
    assert_eq!(
        rational_facts.span_facts()[0].knot_interval(),
        (&r(2), &r(3))
    );
    assert_eq!(
        rational_facts.span_facts()[1].knot_interval(),
        (&r(3), &r(4))
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
    let region = BezierRegion2::new(vec![BezierBoundaryLoop2::new(fragments).unwrap()]).unwrap();

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
fn retained_rational_cubic_bspline_extracts_bezier_span_evidence() {
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
    let evidence = decided(extraction.native_topology_evidence(&policy()).unwrap());
    let native = decided(extraction.native_subcurves(&policy()).unwrap());

    assert_eq!(evidence.span_evidence().len(), 1);
    assert_eq!(
        evidence.span_evidence()[0].decision_path(),
        RationalBezierSpanTopologyPath2::NativeRationalQuadraticSpan
    );
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
    let region = BezierRegion2::new(vec![BezierBoundaryLoop2::new(fragments).unwrap()]).unwrap();

    assert!(region.signed_area().unwrap().is_some());
}

#[test]
fn nonuniform_rational_cubic_spans_promote_without_degree_reduction() {
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
    let evidence = decided(extraction.native_topology_evidence(&policy()).unwrap());

    assert!(evidence.is_fully_native_exact());
    assert_eq!(evidence.span_evidence().len(), extraction.spans().len());
    assert!(evidence.span_evidence().iter().all(|span| {
        span.degree() == 3
            && span.status() == RetainedTopologyStatus::NativeExact
            && span.decision_path() == RationalBezierSpanTopologyPath2::NativeGeneralRationalSpan
            && matches!(span.native_subcurve(), Some(BezierSubcurve2::Rational(_)))
    }));

    let native = decided(extraction.native_subcurves(&policy()).unwrap());
    assert_eq!(native.len(), extraction.spans().len());
    assert!(
        native
            .iter()
            .all(|span| matches!(span, BezierSubcurve2::Rational(_)))
    );
}

#[test]
fn equal_weight_rational_cubic_topology_evidence_names_native_exact_spans() {
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
    let evidence = decided(extraction.native_topology_evidence(&policy()).unwrap());

    assert!(evidence.is_fully_native_exact());
    assert_eq!(evidence.span_evidence().len(), 2);
    for (index, span) in evidence.span_evidence().iter().enumerate() {
        assert_eq!(span.span_index(), index);
        assert_eq!(span.degree(), 3);
        assert_eq!(span.status(), RetainedTopologyStatus::NativeExact);
        assert_eq!(
            span.decision_path(),
            RationalBezierSpanTopologyPath2::NativeEqualWeightCubicSpan
        );
        assert!(matches!(
            span.native_subcurve(),
            Some(BezierSubcurve2::Cubic(_))
        ));
    }

    let native = decided(extraction.native_subcurves(&policy()).unwrap());
    assert_eq!(native.len(), evidence.span_evidence().len());
}
#[test]
fn retained_span_weight_evidence_rejects_inconsistent_counts() {
    assert_topology_error(RetainedSpanWeightDomainEvidence2::new(0, 0, true));
    assert_topology_error(RetainedSpanWeightDomainEvidence2::new(3, 4, false));
    assert_topology_error(RetainedSpanWeightDomainEvidence2::new(3, 2, true));
    assert_topology_error(RetainedSpanWeightDomainEvidence2::new(3, 3, false));
}

#[test]
fn retained_span_fact_constructors_reject_forged_evidence() {
    let bounds = Aabb2::from_point(p(0, 0));

    assert_topology_error(RetainedBSplineSpanFactEvidence2::new(Vec::new()));
    assert_topology_error(RetainedBSplineSpanFacts2::new(
        0,
        r(0),
        r(1),
        bounds.clone(),
        RetainedSpanAxisMonotonicity::CertifiedMonotone,
        RetainedSpanAxisMonotonicity::Unsupported,
        RetainedTopologyStatus::Unsupported,
        Some(RetainedSpanWeightDomainEvidence2::new(3, 3, true).unwrap()),
    ));
    assert_topology_error(RetainedBSplineSpanFacts2::new(
        0,
        r(0),
        r(1),
        bounds.clone(),
        RetainedSpanAxisMonotonicity::CertifiedMonotone,
        RetainedSpanAxisMonotonicity::Unsupported,
        RetainedTopologyStatus::Unresolved,
        Some(RetainedSpanWeightDomainEvidence2::new(3, 2, false).unwrap()),
    ));
    assert_topology_error(RetainedBSplineSpanFacts2::new(
        0,
        r(0),
        r(1),
        bounds.clone(),
        RetainedSpanAxisMonotonicity::Unsupported,
        RetainedSpanAxisMonotonicity::Unsupported,
        RetainedTopologyStatus::Unsupported,
        None,
    ));
    assert_topology_error(RetainedBSplineSpanFacts2::new(
        0,
        r(0),
        r(1),
        bounds.clone(),
        RetainedSpanAxisMonotonicity::Unsupported,
        RetainedSpanAxisMonotonicity::Unsupported,
        RetainedTopologyStatus::Unresolved,
        None,
    ));
    for topology_status in [
        RetainedTopologyStatus::CertifiedApproximation,
        RetainedTopologyStatus::DisplayOrExport,
        RetainedTopologyStatus::ImportedLossy,
    ] {
        assert_topology_error(RetainedBSplineSpanFacts2::new(
            0,
            r(0),
            r(1),
            bounds.clone(),
            RetainedSpanAxisMonotonicity::Unsupported,
            RetainedSpanAxisMonotonicity::Unsupported,
            topology_status,
            Some(RetainedSpanWeightDomainEvidence2::new(3, 3, true).unwrap()),
        ));
    }
    assert_topology_error(RetainedBSplineSpanFacts2::new(
        0,
        r(0),
        r(1),
        bounds.clone(),
        RetainedSpanAxisMonotonicity::Unsupported,
        RetainedSpanAxisMonotonicity::CertifiedMonotone,
        RetainedTopologyStatus::NativeExact,
        None,
    ));
    assert_topology_error(RetainedBSplineSpanFacts2::new(
        0,
        r(0),
        r(1),
        bounds.clone(),
        RetainedSpanAxisMonotonicity::CertifiedMonotone,
        RetainedSpanAxisMonotonicity::Unsupported,
        RetainedTopologyStatus::NativeExact,
        Some(RetainedSpanWeightDomainEvidence2::new(3, 3, true).unwrap()),
    ));
    assert_topology_error(RetainedBSplineSpanFacts2::new(
        0,
        r(0),
        r(1),
        bounds.clone(),
        RetainedSpanAxisMonotonicity::CertifiedMonotone,
        RetainedSpanAxisMonotonicity::CertifiedMonotone,
        RetainedTopologyStatus::NativeExact,
        Some(RetainedSpanWeightDomainEvidence2::new(3, 2, false).unwrap()),
    ));
    assert_topology_error(RetainedBSplineSpanFacts2::new(
        0,
        r(1),
        r(1),
        bounds.clone(),
        RetainedSpanAxisMonotonicity::CertifiedMonotone,
        RetainedSpanAxisMonotonicity::CertifiedMonotone,
        RetainedTopologyStatus::NativeExact,
        None,
    ));
    assert_topology_error(RetainedBSplineSpanFacts2::new(
        0,
        r(2),
        r(1),
        bounds.clone(),
        RetainedSpanAxisMonotonicity::CertifiedMonotone,
        RetainedSpanAxisMonotonicity::CertifiedMonotone,
        RetainedTopologyStatus::NativeExact,
        None,
    ));
    assert_topology_error(RetainedBSplineSpanFacts2::new(
        0,
        r(0),
        r(1),
        Aabb2::new_unchecked(p(1, 0), p(0, 0)),
        RetainedSpanAxisMonotonicity::CertifiedMonotone,
        RetainedSpanAxisMonotonicity::CertifiedMonotone,
        RetainedTopologyStatus::NativeExact,
        None,
    ));

    let first_fact = RetainedBSplineSpanFacts2::new(
        0,
        r(0),
        r(1),
        bounds.clone(),
        RetainedSpanAxisMonotonicity::CertifiedMonotone,
        RetainedSpanAxisMonotonicity::CertifiedMonotone,
        RetainedTopologyStatus::NativeExact,
        None,
    )
    .unwrap();
    let gapped_fact = RetainedBSplineSpanFacts2::new(
        1,
        r(2),
        r(3),
        bounds.clone(),
        RetainedSpanAxisMonotonicity::CertifiedMonotone,
        RetainedSpanAxisMonotonicity::CertifiedMonotone,
        RetainedTopologyStatus::NativeExact,
        None,
    )
    .unwrap();
    assert_topology_error(RetainedBSplineSpanFactEvidence2::new(vec![
        first_fact,
        gapped_fact,
    ]));

    let skipped_index = RetainedBSplineSpanFacts2::new(
        1,
        r(0),
        r(1),
        bounds,
        RetainedSpanAxisMonotonicity::CertifiedMonotone,
        RetainedSpanAxisMonotonicity::CertifiedMonotone,
        RetainedTopologyStatus::NativeExact,
        None,
    )
    .unwrap();
    assert_topology_error(RetainedBSplineSpanFactEvidence2::new(vec![skipped_index]));
}

#[test]
fn retained_rational_span_topology_evidence_reject_forged_native_evidence() {
    assert_topology_error(RationalBSplineNativeTopologyEvidence2::new(Vec::new()));
    assert_topology_error(span_topology_evidence(
        0,
        1,
        r(0),
        r(1),
        RetainedTopologyStatus::Unsupported,
        RationalBezierSpanTopologyPath2::NativeGeneralRationalSpan,
        None,
    ));
    assert_topology_error(span_topology_evidence(
        0,
        2,
        r(1),
        r(1),
        RetainedTopologyStatus::Unsupported,
        RationalBezierSpanTopologyPath2::RetainedControlNetShapeMismatch,
        None,
    ));
    assert_topology_error(span_topology_evidence(
        0,
        2,
        r(2),
        r(1),
        RetainedTopologyStatus::Unsupported,
        RationalBezierSpanTopologyPath2::RetainedControlNetShapeMismatch,
        None,
    ));
    assert_topology_error(span_topology_evidence(
        0,
        2,
        r(0),
        r(1),
        RetainedTopologyStatus::NativeExact,
        RationalBezierSpanTopologyPath2::NativeRationalQuadraticSpan,
        None,
    ));
    assert_topology_error(span_topology_evidence(
        0,
        2,
        r(0),
        r(1),
        RetainedTopologyStatus::Unsupported,
        RationalBezierSpanTopologyPath2::RetainedControlNetShapeMismatch,
        Some(BezierSubcurve2::Quadratic(QuadraticBezier2::new(
            p(0, 0),
            p(1, 0),
            p(2, 0),
        ))),
    ));
    assert_topology_error(span_topology_evidence(
        0,
        3,
        r(0),
        r(1),
        RetainedTopologyStatus::Unsupported,
        RationalBezierSpanTopologyPath2::NativeGeneralRationalSpan,
        None,
    ));
    for topology_status in [
        RetainedTopologyStatus::Unresolved,
        RetainedTopologyStatus::CertifiedApproximation,
        RetainedTopologyStatus::DisplayOrExport,
        RetainedTopologyStatus::ImportedLossy,
    ] {
        assert_topology_error(span_topology_evidence(
            0,
            3,
            r(0),
            r(1),
            topology_status,
            RationalBezierSpanTopologyPath2::NativeGeneralRationalSpan,
            Some(general_rational_cubic()),
        ));
    }

    let skipped_index = span_topology_evidence(
        1,
        3,
        r(0),
        r(1),
        RetainedTopologyStatus::NativeExact,
        RationalBezierSpanTopologyPath2::NativeGeneralRationalSpan,
        Some(general_rational_cubic()),
    )
    .unwrap();
    assert_topology_error(RationalBSplineNativeTopologyEvidence2::new(vec![
        skipped_index,
    ]));

    let first_evidence = span_topology_evidence(
        0,
        2,
        r(0),
        r(1),
        RetainedTopologyStatus::Unsupported,
        RationalBezierSpanTopologyPath2::RetainedControlNetShapeMismatch,
        None,
    )
    .unwrap();
    let gapped_evidence = span_topology_evidence(
        1,
        2,
        r(2),
        r(3),
        RetainedTopologyStatus::Unsupported,
        RationalBezierSpanTopologyPath2::RetainedControlNetShapeMismatch,
        None,
    )
    .unwrap();
    assert_topology_error(RationalBSplineNativeTopologyEvidence2::new(vec![
        first_evidence.clone(),
        gapped_evidence,
    ]));

    let mixed_degree_evidence = span_topology_evidence(
        1,
        3,
        r(1),
        r(2),
        RetainedTopologyStatus::NativeExact,
        RationalBezierSpanTopologyPath2::NativeGeneralRationalSpan,
        Some(general_rational_cubic()),
    )
    .unwrap();
    assert_topology_error(RationalBSplineNativeTopologyEvidence2::new(vec![
        first_evidence,
        mixed_degree_evidence,
    ]));
}

#[test]
fn retained_bspline_span_facts_evidence_native_bounds_and_monotonicity() {
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
    let facts = decided(extraction.span_fact_evidence(&policy()).unwrap());

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
    let facts = decided(extraction.span_fact_evidence(&policy()).unwrap());
    let weight_domain = facts.span_facts()[0]
        .weight_domain()
        .expect("rational span evidence weights");

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
    let facts = decided(extraction.span_fact_evidence(&policy()).unwrap());

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
fn retained_rational_cubic_span_facts_certify_control_hull_and_monotonicity() {
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
    let facts = decided(extraction.span_fact_evidence(&policy()).unwrap());

    assert_eq!(facts.span_facts().len(), 2);
    assert!(facts.span_facts().iter().all(|span| {
        span.topology_status() == RetainedTopologyStatus::NativeExact
            && span.x_monotonicity() == RetainedSpanAxisMonotonicity::CertifiedMonotone
            && span.y_monotonicity() == RetainedSpanAxisMonotonicity::CertifiedMonotone
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
fn retained_degree_four_nurbs_span_certifies_stationary_monotone_axis() {
    let spline = decided(
        RationalBSplineCurve2::try_new(
            4,
            vec![
                p(0, 0),
                Point2::new(q(3, 4), r(0)),
                Point2::new(q(1, 2), r(0)),
                Point2::new(q(1, 4), r(0)),
                p(1, 0),
            ],
            vec![r(1); 5],
            vec![r(0), r(0), r(0), r(0), r(0), r(1), r(1), r(1), r(1), r(1)],
            &policy(),
        )
        .unwrap(),
    );
    let extraction = decided(spline.extract_bezier_spans(&policy()).unwrap());
    let topology = decided(extraction.native_topology_evidence(&policy()).unwrap());
    let facts = decided(extraction.span_fact_evidence(&policy()).unwrap());

    assert_eq!(
        topology.span_evidence()[0].decision_path(),
        RationalBezierSpanTopologyPath2::NativeGeneralRationalSpan
    );
    assert_eq!(
        facts.span_facts()[0].x_monotonicity(),
        RetainedSpanAxisMonotonicity::CertifiedMonotone
    );
    assert_eq!(
        facts.span_facts()[0].y_monotonicity(),
        RetainedSpanAxisMonotonicity::CertifiedMonotone
    );
}

#[test]
fn retained_rational_bspline_rejects_invalid_degree_and_zero_weight() {
    assert_eq!(
        RationalBSplineCurve2::try_new(0, vec![p(0, 0)], vec![r(1)], vec![r(0), r(1)], &policy(),)
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
    let region = BezierRegion2::new(vec![BezierBoundaryLoop2::new(fragments).unwrap()]).unwrap();

    assert!(region.signed_area().unwrap().is_some());
}
