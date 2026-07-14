use hypercurve::{
    BezierSubcurve2, CurveError, CurveFamily2, CurveOperation2, CurveSource2, ExactCurveError,
    Point2, PolynomialSplineCurve2, Real, SplinePeriodicity2,
};

fn r(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn q(numerator: i32, denominator: i32) -> Real {
    (r(numerator) / r(denominator)).unwrap()
}

fn two_span_cubic(source: CurveSource2) -> PolynomialSplineCurve2 {
    PolynomialSplineCurve2::try_new_with_source(
        3,
        vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
        vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
        source,
    )
    .unwrap()
}

#[test]
fn linear_polynomial_spline_evaluates_elevated_spans() {
    let source = CurveSource2::new(39);
    let curve = PolynomialSplineCurve2::try_new_with_source(
        1,
        vec![p(0, 0), p(2, 2), p(4, 0)],
        vec![r(0), r(0), r(1), r(2), r(2)],
        source,
    )
    .unwrap();
    let half = (r(1) / r(2)).unwrap();
    let three_halves = (r(3) / r(2)).unwrap();

    assert_eq!(curve.degree(), 1);
    assert_eq!(curve.point_at(&half).unwrap(), p(1, 1));
    assert_eq!(curve.point_at(&three_halves).unwrap(), p(3, 1));
    assert_eq!(curve.bezier_decomposition().unwrap().spans().len(), 2);
    assert!(
        curve
            .bezier_decomposition()
            .unwrap()
            .spans()
            .iter()
            .all(|span| matches!(span, BezierSubcurve2::Quadratic(_)))
    );
}

#[test]
fn polynomial_spline_clones_share_one_decomposition() {
    let source = CurveSource2::with_version(40, 3);
    let curve = two_span_cubic(source);
    let clone = curve.clone();

    assert!(!curve.is_bezier_decomposition_cached());
    let first = curve.bezier_decomposition().unwrap();
    let second = clone.bezier_decomposition().unwrap();

    assert!(curve.is_bezier_decomposition_cached());
    assert!(std::ptr::eq(first, second));
    assert_eq!(first.spans().len(), 2);
    assert_eq!(first.intervals(), &[(r(0), r(1)), (r(1), r(2))]);
    assert_eq!(first.intervals().len(), first.spans().len());

    let spans = curve.bezier_spans().unwrap().collect::<Vec<_>>();
    assert_eq!(spans.len(), 2);
    assert_eq!(spans[0].span_index(), 0);
    assert_eq!(spans[1].span_index(), 1);
    assert_eq!(spans[0].source(), Some(source));
    assert_eq!(spans[0].knot_interval(), (&r(0), &r(1)));
    assert_eq!(spans[1].knot_interval(), (&r(1), &r(2)));
    assert!(std::ptr::eq(spans[0].curve(), &first.spans()[0]));
}

#[test]
fn polynomial_spline_evaluates_exact_source_parameters() {
    let curve = PolynomialSplineCurve2::try_new(
        2,
        vec![p(0, 0), p(1, 2), p(2, 0)],
        vec![r(0), r(0), r(0), r(1), r(1), r(1)],
    )
    .unwrap();
    let half = (r(1) / r(2)).unwrap();

    assert_eq!(curve.point_at(&r(0)).unwrap(), p(0, 0));
    assert_eq!(curve.point_at(&half).unwrap(), p(1, 1));
    assert_eq!(curve.point_at(&r(1)).unwrap(), p(2, 0));
    assert!(curve.is_bezier_decomposition_cached());
}

#[test]
fn higher_degree_polynomial_spline_uses_exact_unit_weight_bezier_spans() {
    let source = CurveSource2::with_version(44, 2);
    let curve = PolynomialSplineCurve2::try_new_with_source(
        4,
        vec![p(0, 0), p(1, 4), p(2, 0), p(3, 4), p(4, 0)],
        [vec![r(0); 5], vec![r(1); 5]].concat(),
        source,
    )
    .unwrap();

    assert_eq!(curve.degree(), 4);
    assert_eq!(curve.point_at(&(r(1) / r(2)).unwrap()).unwrap(), p(2, 2));
    let spans = curve.bezier_spans().unwrap().collect::<Vec<_>>();
    assert_eq!(spans.len(), 1);
    assert_eq!(spans[0].source(), Some(source));
    let BezierSubcurve2::Rational(span) = spans[0].curve() else {
        panic!("degree-four polynomial span did not use the general exact carrier");
    };
    assert_eq!(span.degree(), 4);
    assert!(span.weights().iter().all(|weight| *weight == r(1)));

    let clone = curve.clone();
    assert!(!curve.is_rational_span_cache_cached());
    let derivatives = clone.derivatives_at(&(r(1) / r(2)).unwrap(), 4).unwrap();
    assert!(curve.is_rational_span_cache_cached());
    assert_eq!((derivatives[0].dx(), derivatives[0].dy()), (&r(4), &r(0)));
    assert_eq!((derivatives[1].dx(), derivatives[1].dy()), (&r(0), &r(0)));
    assert_eq!((derivatives[2].dx(), derivatives[2].dy()), (&r(0), &r(0)));
    assert_eq!(
        (derivatives[3].dx(), derivatives[3].dy()),
        (&r(0), &r(-768))
    );
}

#[test]
fn unclamped_polynomial_spline_retains_exact_active_domain_endpoints() {
    let source = CurveSource2::with_version(46, 1);
    let curve = PolynomialSplineCurve2::try_new_with_source(
        2,
        vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
        (0..=6).map(r).collect(),
        source,
    )
    .unwrap();

    assert_eq!(curve.parameter_domain(), (&r(2), &r(4)));
    assert_eq!(curve.start(), &Point2::new(r(1), r(2)));
    assert_eq!(curve.end(), &Point2::new(r(5), r(2)));
    assert_eq!(curve.point_at(&r(2)).unwrap(), curve.start().clone());
    assert_eq!(curve.point_at(&r(4)).unwrap(), curve.end().clone());
    assert!(curve.is_bezier_decomposition_cached());
    assert_eq!(curve.source(), Some(source));

    let reversed = curve.reversed().unwrap();
    assert_eq!(reversed.start(), curve.end());
    assert_eq!(reversed.end(), curve.start());
    assert_eq!(
        reversed.point_at(&r(3)).unwrap(),
        curve.point_at(&r(3)).unwrap()
    );
}

#[test]
fn polynomial_spline_corner_requires_explicit_derivative_side() {
    let curve = PolynomialSplineCurve2::try_new(
        1,
        vec![p(0, 0), p(1, 0), p(1, 1)],
        vec![r(0), r(0), r(1), r(2), r(2)],
    )
    .unwrap();

    assert!(matches!(
        curve.derivative_at(&r(1)),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.reason() == hypercurve::UncertaintyReason::Boundary
    ));
    let left = curve
        .derivative_at_side(&r(1), hypercurve::CurveParameterSide2::Left)
        .unwrap();
    let right = curve
        .derivative_at_side(&r(1), hypercurve::CurveParameterSide2::Right)
        .unwrap();
    assert_eq!((left.dx(), left.dy()), (&r(1), &r(0)));
    assert_eq!((right.dx(), right.dy()), (&r(0), &r(1)));
}

#[test]
fn discontinuous_polynomial_knot_requires_explicit_point_side() {
    let source = CurveSource2::new(47);
    let curve = PolynomialSplineCurve2::try_new_with_source(
        2,
        vec![p(0, 0), p(1, 1), p(2, 0), p(10, 0), p(11, 1), p(12, 0)],
        vec![r(0), r(0), r(0), r(1), r(1), r(1), r(2), r(2), r(2)],
        source,
    )
    .unwrap();

    assert!(matches!(
        curve.point_at(&r(1)),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.reason() == hypercurve::UncertaintyReason::Boundary
                && blocker.source() == Some(source)
    ));
    assert_eq!(
        curve
            .point_at_side(&r(1), hypercurve::CurveParameterSide2::Left)
            .unwrap(),
        p(2, 0)
    );
    assert_eq!(
        curve
            .point_at_side(&r(1), hypercurve::CurveParameterSide2::Right)
            .unwrap(),
        p(10, 0)
    );
}

#[test]
fn polynomial_spline_interior_knot_uses_retained_span_boundary() {
    let curve = two_span_cubic(CurveSource2::new(41));
    let decomposition = curve.bezier_decomposition().unwrap();
    let expected = match &decomposition.spans()[0] {
        BezierSubcurve2::Cubic(span) => span.end().clone(),
        _ => panic!("cubic B-spline produced a non-cubic span"),
    };

    assert_eq!(curve.point_at(&r(1)).unwrap(), expected);
}

#[test]
fn polynomial_spline_reversal_preserves_domain_source_and_image() {
    let source = CurveSource2::with_version(45, 7);
    let curve = two_span_cubic(source);
    let reversed = curve.reversed().unwrap();

    assert_eq!(reversed.parameter_domain(), curve.parameter_domain());
    assert_eq!(reversed.source(), Some(source));
    assert_eq!(reversed.start(), curve.end());
    assert_eq!(reversed.end(), curve.start());
    assert_eq!(
        reversed.point_at(&(r(1) / r(2)).unwrap()).unwrap(),
        curve.point_at(&(r(3) / r(2)).unwrap()).unwrap()
    );
    assert_eq!(reversed.reversed().unwrap(), curve);
}

#[test]
fn polynomial_spline_knot_insertion_split_and_subcurve_are_exact() {
    let source = CurveSource2::with_version(48, 3);
    let curve = PolynomialSplineCurve2::try_new_with_source(
        2,
        vec![p(0, 0), p(2, 4), p(4, 0)],
        vec![r(0), r(0), r(0), r(2), r(2), r(2)],
        source,
    )
    .unwrap();
    let samples = [
        r(0),
        (r(1) / r(2)).unwrap(),
        r(1),
        (r(3) / r(2)).unwrap(),
        r(2),
    ];
    let expected = samples
        .iter()
        .map(|parameter| curve.point_at(parameter).unwrap())
        .collect::<Vec<_>>();

    let inserted = curve.insert_knot(r(1)).unwrap();
    assert_eq!(inserted.source(), Some(source));
    assert_eq!(
        inserted.control_points().len(),
        curve.control_points().len() + 1
    );
    assert_eq!(
        samples
            .iter()
            .map(|parameter| inserted.point_at(parameter).unwrap())
            .collect::<Vec<_>>(),
        expected
    );

    let (left, right) = curve.split_at(r(1)).unwrap();
    assert_eq!(left.parameter_domain(), (&r(0), &r(1)));
    assert_eq!(right.parameter_domain(), (&r(1), &r(2)));
    assert_eq!(left.end(), &curve.point_at(&r(1)).unwrap());
    assert_eq!(right.start(), left.end());

    let middle = curve
        .subcurve((r(1) / r(2)).unwrap(), (r(3) / r(2)).unwrap())
        .unwrap();
    assert_eq!(middle.source(), Some(source));
    assert_eq!(
        middle.start(),
        &curve.point_at(&(r(1) / r(2)).unwrap()).unwrap()
    );
    assert_eq!(
        middle.end(),
        &curve.point_at(&(r(3) / r(2)).unwrap()).unwrap()
    );
}

#[test]
fn invalid_polynomial_spline_reports_context_and_source() {
    let source = CurveSource2::with_version(42, 9);
    let error = PolynomialSplineCurve2::try_new_with_source(
        4,
        vec![p(0, 0), p(1, 1), p(2, 1), p(3, 1), p(4, 0)],
        vec![r(0); 10],
        source,
    )
    .unwrap_err();

    assert_eq!(error.operation(), CurveOperation2::Construction);
    assert_eq!(error.family(), CurveFamily2::PolynomialBSpline);
    assert_eq!(error.source(), Some(source));
    assert!(matches!(
        error,
        ExactCurveError::Invalid {
            cause: CurveError::InvalidBSpline,
            ..
        }
    ));
}

#[test]
fn polynomial_spline_out_of_domain_evaluation_is_contextual() {
    let source = CurveSource2::new(43);
    let curve = two_span_cubic(source);
    let error = curve.point_at(&r(3)).unwrap_err();

    assert_eq!(error.operation(), CurveOperation2::Evaluation);
    assert_eq!(error.family(), CurveFamily2::PolynomialBSpline);
    assert_eq!(error.source(), Some(source));
    assert!(matches!(
        error,
        ExactCurveError::Invalid {
            cause: CurveError::InvalidCurveParameter,
            ..
        }
    ));
}

#[test]
fn periodic_polynomial_spline_wraps_and_reuses_exact_native_evaluation() {
    let source = CurveSource2::with_version(44, 2);
    let curve = PolynomialSplineCurve2::try_new_periodic_with_source(
        2,
        vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)],
        (0..=4).map(r).collect(),
        source,
    )
    .unwrap();

    assert!(matches!(
        curve.periodicity(),
        SplinePeriodicity2::Periodic { .. }
    ));
    assert_eq!(curve.period(), Some(&r(4)));
    assert_eq!(curve.source(), Some(source));
    assert_eq!(curve.start(), curve.end());
    assert_eq!(curve.point_at_wrapped(&r(-1)).unwrap(), p(0, 1));
    assert_eq!(curve.point_at_wrapped(&r(5)).unwrap(), p(2, 1));
    assert_eq!(
        curve.derivatives_at_wrapped(&q(11, 2), 4).unwrap(),
        curve.derivatives_at(&q(3, 2), 4).unwrap()
    );
}

#[test]
fn periodic_polynomial_editing_preserves_only_whole_curve_periodicity() {
    let curve = PolynomialSplineCurve2::try_new_periodic(
        2,
        vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)],
        (0..=4).map(r).collect(),
    )
    .unwrap();

    let inserted = curve.insert_knot(q(1, 2)).unwrap();
    assert_eq!(inserted.period(), curve.period());
    assert_eq!(
        inserted.point_at_wrapped(&r(5)).unwrap(),
        curve.point_at_wrapped(&r(5)).unwrap()
    );

    let reversed = curve.reversed().unwrap();
    assert_eq!(reversed.period(), curve.period());
    assert_eq!(
        reversed.point_at_wrapped(&r(1)).unwrap(),
        curve.point_at_wrapped(&r(3)).unwrap()
    );

    let (left, right) = curve.split_at(r(2)).unwrap();
    assert_eq!(left.period(), None);
    assert_eq!(right.period(), None);
}

#[test]
fn periodic_polynomial_spline_reports_layout_and_wrapping_errors() {
    let invalid = PolynomialSplineCurve2::try_new_periodic(
        3,
        vec![p(0, 0), p(1, 0), p(1, 1)],
        vec![r(0), r(1), r(2), r(3)],
    )
    .unwrap_err();
    assert!(matches!(
        invalid,
        ExactCurveError::Invalid {
            cause: CurveError::InvalidPeriodicSpline,
            ..
        }
    ));

    let open = two_span_cubic(CurveSource2::new(45));
    let error = open.point_at_wrapped(&r(3)).unwrap_err();
    assert!(matches!(
        error,
        ExactCurveError::Invalid {
            cause: CurveError::CurveIsNotPeriodic,
            ..
        }
    ));
}
