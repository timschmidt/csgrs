use hypercurve::{
    BezierBoundaryLoop2, BezierRegion2, BezierSubcurve2, Classification, CurveError, CurvePolicy,
    Point2, PolynomialBSplineCurve2, RationalBSplineCurve2, RationalQuadraticBSplineCurve2, Real,
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
