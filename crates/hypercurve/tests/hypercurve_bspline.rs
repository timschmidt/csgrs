mod support;

use hypercurve::{
    BezierBoundaryLoop2, BezierSubcurve2, Classification, CurveContext, CurveError, CurveRegion2,
    Point2, PolynomialBSplineCurve2, RationalBSplineCurve2, Real, RetainedSpanAxisMonotonicity,
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

fn policy() -> CurveContext {
    CurveContext::STRICT
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
    }
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
    let native = extraction.native_subcurves(&policy());
    let BezierSubcurve2::Rational(curve) = &native[0] else {
        panic!("expected the original degree-one rational evaluator");
    };
    assert_eq!(curve.degree(), 1);
    assert_eq!(curve.weights(), &[r(1), r(3)]);
    assert_point_eq(&curve.point_at(&q(1, 2), &policy()).unwrap(), &p(3, 0));
}

#[test]
fn rational_linear_span_retains_its_denominator_pole() {
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
    let native = extraction.native_subcurves(&policy());
    let BezierSubcurve2::Rational(curve) = &native[0] else {
        panic!("expected rational span");
    };
    assert_eq!(curve.degree(), 1);
    assert_point_eq(curve.start(), &p(0, 0));
    assert_point_eq(curve.end(), &p(4, 0));
    assert!(curve.point_at(&q(1, 2), &policy()).is_err());
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
    assert_eq!(
        PolynomialBSplineCurve2::try_new(
            2,
            vec![p(0, 0), p(1, 1), p(2, 0)],
            vec![r(0), r(0), r(0), r(2), r(1), r(1)],
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
        RationalBSplineCurve2::try_new(
            2,
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
fn extracted_bspline_spans_feed_unified_region_area() {
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
    let region = CurveRegion2::new(vec![
        BezierBoundaryLoop2::new(fragments, &CurveContext::STRICT)
            .unwrap()
            .into(),
    ])
    .unwrap();

    assert_eq!(
        decided(region.signed_area(&policy()).unwrap().into_value()),
        Some(q(-88, 3))
    );
}

#[test]
fn rational_quadratic_bspline_extracts_homogeneous_bezier_spans() {
    let spline = decided(
        RationalBSplineCurve2::try_new(
            2,
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
        extraction
            .refined_homogeneous_controls()
            .iter()
            .map(|control| control.weight().clone())
            .collect::<Vec<_>>(),
        &[r(1), r(2), r(3), r(4), r(1)]
    );
    match extraction.spans()[0].native_subcurve(&policy()) {
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
    match extraction.spans()[1].native_subcurve(&policy()) {
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
        RationalBSplineCurve2::try_new(2, controls, vec![r(1), r(1), r(1), r(1)], knots, &policy())
            .unwrap(),
    );
    let polynomial = decided(polynomial.extract_bezier_spans(&policy()).unwrap());
    let rational = decided(rational.extract_bezier_spans(&policy()).unwrap());

    for (polynomial_span, rational_span) in polynomial.spans().iter().zip(rational.spans()) {
        let BezierSubcurve2::Quadratic(polynomial) = polynomial_span else {
            panic!("expected polynomial quadratic")
        };
        let BezierSubcurve2::RationalQuadratic(rational) = rational_span.native_subcurve(&policy())
        else {
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
    assert_eq!(extraction.refined_homogeneous_controls().len(), 7);
    assert_eq!(extraction.spans().len(), 2);
    for span in extraction.spans() {
        assert_eq!(span.curve().degree(), 3);
        assert_eq!(span.curve().affine_control_points().unwrap().len(), 4);
        assert_eq!(span.curve().weights().len(), 4);
    }
    assert_eq!(extraction.spans()[0].knot_interval(), (&r(0), &r(1)));
    assert_eq!(extraction.spans()[1].knot_interval(), (&r(1), &r(2)));
    assert_point_eq(
        &extraction.spans()[0]
            .curve()
            .affine_control_points()
            .unwrap()[3],
        &extraction.spans()[1]
            .curve()
            .affine_control_points()
            .unwrap()[0],
    );
    assert_eq!(
        extraction.spans()[0].curve().weights()[3],
        extraction.spans()[1].curve().weights()[0]
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
        assert_eq!(rational_span.curve().degree(), 3);
        assert_point_eq(
            polynomial.start(),
            &rational_span.curve().affine_control_points().unwrap()[0],
        );
        assert_point_eq(
            polynomial.control1(),
            &rational_span.curve().affine_control_points().unwrap()[1],
        );
        assert_point_eq(
            polynomial.control2(),
            &rational_span.curve().affine_control_points().unwrap()[2],
        );
        assert_point_eq(
            polynomial.end(),
            &rational_span.curve().affine_control_points().unwrap()[3],
        );
        assert_eq!(rational_span.curve().weights(), &[r(1), r(1), r(1), r(1)]);
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
    let native = extraction.native_subcurves(&policy());
    assert_eq!(native.len(), 1);
    let BezierSubcurve2::RationalQuadratic(curve) = &native[0] else {
        panic!("expected conic specialization");
    };
    assert_point_eq(curve.start(), &p(0, 0));
    assert_point_eq(curve.control(), &p(2, 4));
    assert_point_eq(curve.end(), &p(4, 0));
}

#[test]
fn equal_weight_retained_rational_cubic_spans_feed_unified_region_area() {
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
    fragments.extend(
        decided(upper.extract_bezier_spans(&policy()).unwrap()).native_subcurves(&policy()),
    );
    fragments.extend(
        decided(lower.extract_bezier_spans(&policy()).unwrap()).native_subcurves(&policy()),
    );
    let region = CurveRegion2::new(vec![
        BezierBoundaryLoop2::new(fragments, &CurveContext::STRICT)
            .unwrap()
            .into(),
    ])
    .unwrap();

    assert!(decided(region.signed_area(&policy()).unwrap().into_value()).is_some());
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
    let native = extraction.native_subcurves(&policy());
    assert_eq!(native.len(), extraction.spans().len());
    for (span, native) in extraction.spans().iter().zip(&native) {
        let BezierSubcurve2::Rational(curve) = native else {
            panic!("expected general rational span");
        };
        assert_eq!(curve.degree(), 3);
        assert!(std::ptr::eq(
            curve.homogeneous_controls(),
            span.curve().homogeneous_controls()
        ));
    }
}

#[test]
fn equal_weight_rational_cubic_spans_specialize_to_polynomial_cubics() {
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
    let native = extraction.native_subcurves(&policy());
    assert_eq!(native.len(), 2);
    assert!(
        native
            .iter()
            .all(|span| matches!(span, BezierSubcurve2::Cubic(_)))
    );
    assert_eq!(extraction.spans()[0].knot_interval(), (&r(0), &r(1)));
    assert_eq!(extraction.spans()[1].knot_interval(), (&r(1), &r(2)));
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
}

#[test]
fn rational_quadratic_span_facts_certify_bounds_and_extrema() {
    let spline = decided(
        RationalBSplineCurve2::try_new(
            2,
            vec![p(0, 0), p(1, 1), p(2, 0)],
            vec![r(1), r(2), r(3)],
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            &policy(),
        )
        .unwrap(),
    );
    let extraction = decided(spline.extract_bezier_spans(&policy()).unwrap());
    let facts = decided(extraction.span_fact_evidence(&policy()).unwrap());
    let span = &facts.span_facts()[0];
    assert_eq!(span.bounds().min(), &p(0, 0));
    assert_eq!(span.bounds().max(), &p(2, 1));
    assert_eq!(
        span.x_monotonicity(),
        RetainedSpanAxisMonotonicity::CertifiedMonotone
    );
    assert_eq!(
        span.y_monotonicity(),
        RetainedSpanAxisMonotonicity::HasInteriorExtrema
    );
}

#[test]
fn retained_rational_quadratic_span_facts_follow_refined_knot_windows() {
    let spline = decided(
        RationalBSplineCurve2::try_new(
            2,
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
        span.x_monotonicity() == RetainedSpanAxisMonotonicity::CertifiedMonotone
            && span.y_monotonicity() == RetainedSpanAxisMonotonicity::CertifiedMonotone
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
    let facts = decided(extraction.span_fact_evidence(&policy()).unwrap());

    assert_eq!(extraction.spans()[0].curve().degree(), 4);
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
        RationalBSplineCurve2::try_new(0, vec![p(0, 0)], vec![r(1)], vec![r(0), r(1)], &policy(),),
        Err(CurveError::InvalidBSpline)
    );
    assert_eq!(
        RationalBSplineCurve2::try_new(usize::MAX, Vec::new(), Vec::new(), Vec::new(), &policy()),
        Err(CurveError::InvalidBSpline)
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
fn affine_authoring_rejects_zero_weights_and_extraction_rejects_infinite_endpoints() {
    assert_eq!(
        RationalBSplineCurve2::try_new(
            2,
            vec![p(0, 0), p(1, 1), p(2, 1)],
            vec![r(1), r(0), r(1)],
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            &policy(),
        ),
        Err(CurveError::ZeroRationalBezierWeight)
    );

    let spline = decided(
        RationalBSplineCurve2::try_new(
            2,
            vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
            vec![r(1), r(1), r(-1), r(1)],
            vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
            &policy(),
        )
        .unwrap(),
    );
    assert_eq!(
        spline.extract_bezier_spans(&policy()),
        Ok(Classification::Uncertain(
            hypercurve::UncertaintyReason::Boundary
        ))
    );
}

#[test]
fn extracted_rational_bspline_spans_feed_conic_region_area() {
    let upper = decided(
        RationalBSplineCurve2::try_new(
            2,
            vec![p(0, 0), p(2, 2), p(4, 2), p(6, 0)],
            vec![r(1), q(1, 2), q(1, 2), r(1)],
            vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
            &policy(),
        )
        .unwrap(),
    );
    let lower = decided(
        RationalBSplineCurve2::try_new(
            2,
            vec![p(6, 0), p(4, -2), p(2, -2), p(0, 0)],
            vec![r(1), q(1, 2), q(1, 2), r(1)],
            vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
            &policy(),
        )
        .unwrap(),
    );
    let mut fragments = Vec::new();
    fragments.extend(
        decided(upper.extract_bezier_spans(&policy()).unwrap()).native_subcurves(&policy()),
    );
    fragments.extend(
        decided(lower.extract_bezier_spans(&policy()).unwrap()).native_subcurves(&policy()),
    );
    let region = CurveRegion2::new(vec![
        BezierBoundaryLoop2::new(fragments, &CurveContext::STRICT)
            .unwrap()
            .into(),
    ])
    .unwrap();

    assert!(decided(region.signed_area(&policy()).unwrap().into_value()).is_some());
}
