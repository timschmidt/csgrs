//! Exact Bezier extraction of polynomial and rational B-splines.

use crate::{
    Classification, Curve2, CurveContext, CurveError, CurveGeometry2, CurvePath2, CurveRegion2,
    ExactCurveError, NurbsCurve2, Point2, PolynomialSplineCurve2, Real, UncertaintyReason,
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
    assert!(left.x().partial_cmp(right.x()) == Some(std::cmp::Ordering::Equal));
    assert!(left.y().partial_cmp(right.y()) == Some(std::cmp::Ordering::Equal));
}

#[test]
fn linear_bspline_spans_are_elevated_exactly() {
    let spline = PolynomialSplineCurve2::try_new_with_policy(
        1,
        vec![p(0, 0), p(2, 2), p(4, 0)],
        vec![r(0), r(0), r(1), r(2), r(2)],
        &policy(),
    )
    .unwrap()
    .into_value();
    let extraction = spline.bezier_decomposition(&policy()).unwrap().into_value();

    assert_eq!(extraction.degree(), 1);
    assert_eq!(extraction.spans().len(), 2);
    let CurveGeometry2::QuadraticBezier(first) = &extraction.spans()[0] else {
        panic!("linear span was not elevated to a quadratic");
    };
    assert_point_eq(first.start(), &p(0, 0));
    assert_point_eq(first.control(), &p(1, 1));
    assert_point_eq(first.end(), &p(2, 2));
}

#[test]
fn rational_linear_span_preserves_homogeneous_parameterization() {
    let spline = NurbsCurve2::try_new_with_policy(
        1,
        vec![p(0, 0), p(4, 0)],
        vec![r(1), r(3)],
        vec![r(0), r(0), r(1), r(1)],
        &policy(),
    )
    .unwrap()
    .into_value();
    let extraction = spline.bezier_decomposition(&policy()).unwrap().into_value();
    let native = extraction.native_subcurves(&policy());
    let CurveGeometry2::RationalBezier(curve) = &native[0] else {
        panic!("expected the original degree-one rational evaluator");
    };
    assert_eq!(curve.degree(), 1);
    assert!(curve.weights() == [r(1), r(3)]);
    assert_point_eq(
        &curve.point_at_with_policy(&q(1, 2), &policy()).unwrap(),
        &p(3, 0),
    );
}

#[test]
fn rational_linear_span_retains_its_denominator_pole() {
    let spline = NurbsCurve2::try_new_with_policy(
        1,
        vec![p(0, 0), p(4, 0)],
        vec![r(1), r(-1)],
        vec![r(0), r(0), r(1), r(1)],
        &policy(),
    )
    .unwrap()
    .into_value();
    let extraction = spline.bezier_decomposition(&policy()).unwrap().into_value();
    let native = extraction.native_subcurves(&policy());
    let CurveGeometry2::RationalBezier(curve) = &native[0] else {
        panic!("expected rational span");
    };
    assert_eq!(curve.degree(), 1);
    assert_point_eq(curve.start(), &p(0, 0));
    assert_point_eq(curve.end(), &p(4, 0));
    assert!(curve.point_at_with_policy(&q(1, 2), &policy()).is_err());
}

#[test]
fn quadratic_bspline_extracts_bezier_spans_by_exact_knot_insertion() {
    let spline = PolynomialSplineCurve2::try_new_with_policy(
        2,
        vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
        vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
        &policy(),
    )
    .unwrap()
    .into_value();
    let extraction = spline.bezier_decomposition(&policy()).unwrap().into_value();

    assert_eq!(extraction.inserted_knot_count(), 1);
    assert_eq!(extraction.spans().len(), 2);
    match &extraction.spans()[0] {
        CurveGeometry2::QuadraticBezier(curve) => {
            assert_point_eq(curve.start(), &p(0, 0));
            assert_point_eq(curve.control(), &p(2, 4));
            assert_point_eq(curve.end(), &p(3, 4));
        }
        _ => panic!("expected quadratic span"),
    }
    match &extraction.spans()[1] {
        CurveGeometry2::QuadraticBezier(curve) => {
            assert_point_eq(curve.start(), &p(3, 4));
            assert_point_eq(curve.control(), &p(4, 4));
            assert_point_eq(curve.end(), &p(6, 0));
        }
        _ => panic!("expected quadratic span"),
    }
}

#[test]
fn cubic_bspline_extracts_spans_with_degree_multiplicity_at_internal_knot() {
    let spline = PolynomialSplineCurve2::try_new_with_policy(
        3,
        vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
        vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
        &policy(),
    )
    .unwrap()
    .into_value();
    let extraction = spline.bezier_decomposition(&policy()).unwrap().into_value();

    assert_eq!(extraction.inserted_knot_count(), 2);
    assert_eq!(extraction.spans().len(), 2);
    match &extraction.spans()[0] {
        CurveGeometry2::CubicBezier(curve) => {
            assert_point_eq(curve.start(), &p(0, 0));
            assert_point_eq(curve.control1(), &p(1, 3));
            assert_point_eq(curve.control2(), &p(2, 3));
            assert_point_eq(curve.end(), &p(3, 3));
        }
        _ => panic!("expected cubic span"),
    }
    match &extraction.spans()[1] {
        CurveGeometry2::CubicBezier(curve) => {
            assert_point_eq(curve.start(), &p(3, 3));
            assert_point_eq(curve.control1(), &p(4, 3));
            assert_point_eq(curve.control2(), &p(5, 3));
            assert_point_eq(curve.end(), &p(6, 0));
        }
        _ => panic!("expected cubic span"),
    }
}

#[test]
fn bspline_constructor_rejects_degenerate_knot_vectors() {
    assert!(matches!(
        PolynomialSplineCurve2::try_new_with_policy(
            2,
            vec![p(0, 0), p(1, 1), p(2, 0)],
            vec![r(0), r(0), r(1), r(1), r(1), r(1)],
            &policy(),
        ),
        Err(ExactCurveError::Invalid {
            cause: CurveError::InvalidBSpline,
            ..
        })
    ));
    assert!(matches!(
        PolynomialSplineCurve2::try_new_with_policy(
            2,
            vec![p(0, 0), p(1, 1), p(2, 0)],
            vec![r(0), r(0), r(0), r(0), r(0), r(0)],
            &policy(),
        ),
        Err(ExactCurveError::Invalid {
            cause: CurveError::InvalidBSpline,
            ..
        })
    ));
    assert!(matches!(
        PolynomialSplineCurve2::try_new_with_policy(
            2,
            vec![p(0, 0), p(1, 1), p(2, 0)],
            vec![r(0), r(0), r(0), r(2), r(1), r(1)],
            &policy(),
        ),
        Err(ExactCurveError::Invalid {
            cause: CurveError::InvalidBSpline,
            ..
        })
    ));
}

#[test]
fn unclamped_uniform_bspline_refines_active_domain_endpoints_exactly() {
    let spline = PolynomialSplineCurve2::try_new_with_policy(
        2,
        vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
        (0..=6).map(r).collect(),
        &policy(),
    )
    .unwrap()
    .into_value();
    let extraction = spline.bezier_decomposition(&policy()).unwrap().into_value();

    assert_eq!(extraction.inserted_knot_count(), 3);
    assert_eq!(extraction.spans().len(), 2);
    let CurveGeometry2::QuadraticBezier(first) = &extraction.spans()[0] else {
        panic!("unclamped quadratic did not extract a quadratic first span");
    };
    let CurveGeometry2::QuadraticBezier(second) = &extraction.spans()[1] else {
        panic!("unclamped quadratic did not extract a quadratic second span");
    };
    assert!(first.control_points() == [&Point2::new(r(1), r(2)), &p(2, 4), &p(3, 4)]);
    assert!(second.control_points() == [&p(3, 4), &p(4, 4), &Point2::new(r(5), r(2))]);

    assert!(first.start() == &Point2::new(r(1), r(2)));
    assert!(second.end() == &Point2::new(r(5), r(2)));

    let rational = NurbsCurve2::try_new_with_policy(
        2,
        vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
        vec![r(1), r(2), r(3), r(4)],
        (0..=6).map(r).collect(),
        &policy(),
    )
    .unwrap()
    .into_value();
    let rational_extraction = rational
        .bezier_decomposition(&policy())
        .unwrap()
        .into_value();
    assert_eq!(rational_extraction.spans().len(), 2);
    assert!(rational_extraction.spans()[0].knot_interval() == (&r(2), &r(3)));
    assert!(rational_extraction.spans()[1].knot_interval() == (&r(3), &r(4)));
}

#[test]
fn extracted_bspline_spans_feed_unified_region_area() {
    let upper = PolynomialSplineCurve2::try_new_with_policy(
        2,
        vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
        vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
        &policy(),
    )
    .unwrap()
    .into_value();
    let lower = PolynomialSplineCurve2::try_new_with_policy(
        2,
        vec![p(6, 0), p(4, -4), p(2, -4), p(0, 0)],
        vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
        &policy(),
    )
    .unwrap()
    .into_value();
    let mut fragments = Vec::new();
    fragments.extend(
        upper
            .bezier_decomposition(&policy())
            .unwrap()
            .into_value()
            .spans()
            .to_vec(),
    );
    fragments.extend(
        lower
            .bezier_decomposition(&policy())
            .unwrap()
            .into_value()
            .spans()
            .to_vec(),
    );
    let path = CurvePath2::try_new(fragments.into_iter().map(Curve2::from).collect()).unwrap();
    let region = CurveRegion2::try_from_boundary_paths_with_policy(
        &[path],
        crate::FillRule::EvenOdd,
        &policy(),
    )
    .unwrap()
    .into_value();

    assert!(
        decided(
            region
                .signed_area_with_policy(&policy())
                .unwrap()
                .into_value()
        ) == Some(q(88, 3))
    );
}

#[test]
fn rational_quadratic_bspline_extracts_homogeneous_bezier_spans() {
    let spline = NurbsCurve2::try_new_with_policy(
        2,
        vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
        vec![r(1), r(2), r(4), r(1)],
        vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
        &policy(),
    )
    .unwrap()
    .into_value();
    let extraction = spline.bezier_decomposition(&policy()).unwrap().into_value();

    assert_eq!(extraction.inserted_knot_count(), 1);
    assert_eq!(extraction.spans().len(), 2);
    assert!(
        extraction
            .refined_homogeneous_controls()
            .iter()
            .map(|control| control.weight().clone())
            .collect::<Vec<_>>()
            == [r(1), r(2), r(3), r(4), r(1)]
    );
    match extraction.spans()[0].native_subcurve(&policy()) {
        CurveGeometry2::RationalQuadraticBezier(curve) => {
            assert_point_eq(curve.start(), &p(0, 0));
            assert_point_eq(curve.control(), &p(2, 4));
            assert_point_eq(curve.end(), &Point2::new(q(10, 3), r(4)));
            assert!(curve.start_weight() == &r(1));
            assert!(curve.control_weight() == &r(2));
            assert!(curve.end_weight() == &r(3));
        }
        _ => panic!("expected rational quadratic span"),
    }
    match extraction.spans()[1].native_subcurve(&policy()) {
        CurveGeometry2::RationalQuadraticBezier(curve) => {
            assert_point_eq(curve.start(), &Point2::new(q(10, 3), r(4)));
            assert_point_eq(curve.control(), &p(4, 4));
            assert_point_eq(curve.end(), &p(6, 0));
            assert!(curve.start_weight() == &r(3));
            assert!(curve.control_weight() == &r(4));
            assert!(curve.end_weight() == &r(1));
        }
        _ => panic!("expected rational quadratic span"),
    }
}

#[test]
fn equal_weight_quadratic_nurbs_matches_polynomial_bspline_spans() {
    let controls = vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)];
    let knots = vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)];
    let polynomial =
        PolynomialSplineCurve2::try_new_with_policy(2, controls.clone(), knots.clone(), &policy())
            .unwrap()
            .into_value();
    let rational = NurbsCurve2::try_new_with_policy(
        2,
        controls,
        vec![r(1), r(1), r(1), r(1)],
        knots,
        &policy(),
    )
    .unwrap()
    .into_value();
    let polynomial = polynomial
        .bezier_decomposition(&policy())
        .unwrap()
        .into_value();
    let rational = rational
        .bezier_decomposition(&policy())
        .unwrap()
        .into_value();

    for (polynomial_span, rational_span) in polynomial.spans().iter().zip(rational.spans()) {
        let CurveGeometry2::QuadraticBezier(polynomial) = polynomial_span else {
            panic!("expected polynomial quadratic")
        };
        let CurveGeometry2::RationalQuadraticBezier(rational) =
            rational_span.native_subcurve(&policy())
        else {
            panic!("expected rational quadratic")
        };
        assert_point_eq(polynomial.start(), rational.start());
        assert_point_eq(polynomial.control(), rational.control());
        assert_point_eq(polynomial.end(), rational.end());
        assert!(rational.weights() == [&Real::one(), &Real::one(), &Real::one()]);
    }
}

#[test]
fn retained_rational_cubic_bspline_extracts_bezier_span_evidence() {
    let spline = NurbsCurve2::try_new_with_policy(
        3,
        vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
        vec![r(1), r(2), r(4), r(8), r(16)],
        vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
        &policy(),
    )
    .unwrap()
    .into_value();
    let extraction = spline.bezier_decomposition(&policy()).unwrap().into_value();

    assert_eq!(spline.degree(), 3);
    assert_eq!(extraction.degree(), 3);
    assert_eq!(extraction.inserted_knot_count(), 2);
    assert_eq!(extraction.refined_homogeneous_controls().len(), 7);
    assert_eq!(extraction.spans().len(), 2);
    for span in extraction.spans() {
        assert_eq!(span.curve().degree(), 3);
        assert!(span.curve().affine_control_points().unwrap().len() == 4);
        assert!(span.curve().weights().len() == 4);
    }
    assert!(extraction.spans()[0].knot_interval() == (&r(0), &r(1)));
    assert!(extraction.spans()[1].knot_interval() == (&r(1), &r(2)));
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
    assert!(
        extraction.spans()[0].curve().weights()[3] == extraction.spans()[1].curve().weights()[0]
    );
}

#[test]
fn equal_weight_retained_rational_cubic_matches_polynomial_cubic_spans() {
    let controls = vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)];
    let knots = vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)];
    let polynomial =
        PolynomialSplineCurve2::try_new_with_policy(3, controls.clone(), knots.clone(), &policy())
            .unwrap()
            .into_value();
    let rational = NurbsCurve2::try_new_with_policy(3, controls, vec![r(1); 5], knots, &policy())
        .unwrap()
        .into_value();
    let polynomial = polynomial
        .bezier_decomposition(&policy())
        .unwrap()
        .into_value();
    let rational = rational
        .bezier_decomposition(&policy())
        .unwrap()
        .into_value();

    assert_eq!(rational.spans().len(), polynomial.spans().len());
    for (polynomial_span, rational_span) in polynomial.spans().iter().zip(rational.spans()) {
        let CurveGeometry2::CubicBezier(polynomial) = polynomial_span else {
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
        assert!(rational_span.curve().weights() == [r(1), r(1), r(1), r(1)]);
    }
}

#[test]
fn retained_rational_quadratic_spans_promote_to_native_conic_topology() {
    let spline = NurbsCurve2::try_new_with_policy(
        2,
        vec![p(0, 0), p(2, 4), p(4, 0)],
        vec![r(1), r(2), r(3)],
        vec![r(0), r(0), r(0), r(1), r(1), r(1)],
        &policy(),
    )
    .unwrap()
    .into_value();
    let extraction = spline.bezier_decomposition(&policy()).unwrap().into_value();
    let native = extraction.native_subcurves(&policy());
    assert_eq!(native.len(), 1);
    let CurveGeometry2::RationalQuadraticBezier(curve) = &native[0] else {
        panic!("expected conic specialization");
    };
    assert_point_eq(curve.start(), &p(0, 0));
    assert_point_eq(curve.control(), &p(2, 4));
    assert_point_eq(curve.end(), &p(4, 0));
}

#[test]
fn equal_weight_retained_rational_cubic_spans_feed_unified_region_area() {
    let upper = NurbsCurve2::try_new_with_policy(
        3,
        vec![p(0, 0), p(1, 3), p(5, 3), p(6, 0)],
        vec![r(7), r(7), r(7), r(7)],
        vec![r(0), r(0), r(0), r(0), r(1), r(1), r(1), r(1)],
        &policy(),
    )
    .unwrap()
    .into_value();
    let lower = NurbsCurve2::try_new_with_policy(
        3,
        vec![p(6, 0), p(5, -3), p(1, -3), p(0, 0)],
        vec![r(7), r(7), r(7), r(7)],
        vec![r(0), r(0), r(0), r(0), r(1), r(1), r(1), r(1)],
        &policy(),
    )
    .unwrap()
    .into_value();
    let mut fragments = Vec::new();
    fragments.extend(
        upper
            .bezier_decomposition(&policy())
            .unwrap()
            .into_value()
            .native_subcurves(&policy()),
    );
    fragments.extend(
        lower
            .bezier_decomposition(&policy())
            .unwrap()
            .into_value()
            .native_subcurves(&policy()),
    );
    let path = CurvePath2::try_new(fragments.into_iter().map(Curve2::from).collect()).unwrap();
    let region = CurveRegion2::try_from_boundary_paths_with_policy(
        &[path],
        crate::FillRule::EvenOdd,
        &policy(),
    )
    .unwrap()
    .into_value();

    assert!(
        decided(
            region
                .signed_area_with_policy(&policy())
                .unwrap()
                .into_value()
        )
        .is_some()
    );
}

#[test]
fn nonuniform_rational_cubic_spans_promote_without_degree_reduction() {
    let spline = NurbsCurve2::try_new_with_policy(
        3,
        vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
        vec![r(1), r(2), r(4), r(8), r(16)],
        vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
        &policy(),
    )
    .unwrap()
    .into_value();
    let extraction = spline.bezier_decomposition(&policy()).unwrap().into_value();
    let native = extraction.native_subcurves(&policy());
    assert_eq!(native.len(), extraction.spans().len());
    for (span, native) in extraction.spans().iter().zip(&native) {
        let CurveGeometry2::RationalBezier(curve) = native else {
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
    let spline = NurbsCurve2::try_new_with_policy(
        3,
        vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
        vec![r(5), r(5), r(5), r(5), r(5)],
        vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
        &policy(),
    )
    .unwrap()
    .into_value();
    let extraction = spline.bezier_decomposition(&policy()).unwrap().into_value();
    let native = extraction.native_subcurves(&policy());
    assert_eq!(native.len(), 2);
    assert!(
        native
            .iter()
            .all(|span| matches!(span, CurveGeometry2::CubicBezier(_)))
    );
    assert!(extraction.spans()[0].knot_interval() == (&r(0), &r(1)));
    assert!(extraction.spans()[1].knot_interval() == (&r(1), &r(2)));
}

#[test]
fn retained_rational_bspline_rejects_invalid_degree_and_zero_weight() {
    assert!(matches!(
        NurbsCurve2::try_new_with_policy(0, vec![p(0, 0)], vec![r(1)], vec![r(0), r(1)], &policy(),),
        Err(ExactCurveError::Invalid {
            cause: CurveError::InvalidBSpline,
            ..
        })
    ));
    assert!(matches!(
        NurbsCurve2::try_new_with_policy(usize::MAX, Vec::new(), Vec::new(), Vec::new(), &policy()),
        Err(ExactCurveError::Invalid {
            cause: CurveError::InvalidBSpline,
            ..
        })
    ));
    assert!(matches!(
        NurbsCurve2::try_new_with_policy(
            3,
            vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
            vec![r(1), r(2), r(0), r(8), r(16)],
            vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
            &policy(),
        ),
        Err(ExactCurveError::Invalid {
            cause: CurveError::ZeroRationalBezierWeight,
            ..
        })
    ));
}

#[test]
fn affine_authoring_rejects_zero_weights_and_extraction_rejects_infinite_endpoints() {
    assert!(matches!(
        NurbsCurve2::try_new_with_policy(
            2,
            vec![p(0, 0), p(1, 1), p(2, 1)],
            vec![r(1), r(0), r(1)],
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            &policy(),
        ),
        Err(ExactCurveError::Invalid {
            cause: CurveError::ZeroRationalBezierWeight,
            ..
        })
    ));

    let spline = NurbsCurve2::try_new_with_policy(
        2,
        vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
        vec![r(1), r(1), r(-1), r(1)],
        vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
        &policy(),
    )
    .unwrap()
    .into_value();
    assert!(
        matches!(spline.bezier_decomposition(&policy()), Err(ExactCurveError::Blocked(blocker)) if blocker.reason() == UncertaintyReason::Boundary)
    );
}

#[test]
fn extracted_rational_bspline_spans_feed_conic_region_area() {
    let upper = NurbsCurve2::try_new_with_policy(
        2,
        vec![p(0, 0), p(2, 2), p(4, 2), p(6, 0)],
        vec![r(1), q(1, 2), q(1, 2), r(1)],
        vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
        &policy(),
    )
    .unwrap()
    .into_value();
    let lower = NurbsCurve2::try_new_with_policy(
        2,
        vec![p(6, 0), p(4, -2), p(2, -2), p(0, 0)],
        vec![r(1), q(1, 2), q(1, 2), r(1)],
        vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
        &policy(),
    )
    .unwrap()
    .into_value();
    let mut fragments = Vec::new();
    fragments.extend(
        upper
            .bezier_decomposition(&policy())
            .unwrap()
            .into_value()
            .native_subcurves(&policy()),
    );
    fragments.extend(
        lower
            .bezier_decomposition(&policy())
            .unwrap()
            .into_value()
            .native_subcurves(&policy()),
    );
    let path = CurvePath2::try_new(fragments.into_iter().map(Curve2::from).collect()).unwrap();
    let region = CurveRegion2::try_from_boundary_paths_with_policy(
        &[path],
        crate::FillRule::EvenOdd,
        &policy(),
    )
    .unwrap()
    .into_value();

    assert!(
        decided(
            region
                .signed_area_with_policy(&policy())
                .unwrap()
                .into_value()
        )
        .is_some()
    );
}
