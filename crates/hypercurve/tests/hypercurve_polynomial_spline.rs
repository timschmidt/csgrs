use hypercurve::{
    BezierSubcurve2, CurveContext, CurveError, CurveFamily2, CurveOperation2, ExactCurveError,
    Point2, PolynomialSplineCurve2, Real, SplinePeriodicity2,
};
use hypercurve::{Curve2, CurveParameterSide2, Similarity2};

fn r(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn q(numerator: i32, denominator: i32) -> Real {
    (r(numerator) / r(denominator)).unwrap()
}

fn two_span_cubic() -> PolynomialSplineCurve2 {
    PolynomialSplineCurve2::try_new(
        3,
        vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
        vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value()
}

fn terminal_polynomial_spline() -> (PolynomialSplineCurve2, Real) {
    let half = q(1, 2);
    let symbolic_half = &half + ((Real::pi() + Real::e()) - (Real::e() + Real::pi()));
    let curve = PolynomialSplineCurve2::try_new(
        2,
        vec![p(0, 0), p(1, 2), p(2, 0), p(3, -2), p(4, 0)],
        vec![
            r(0),
            r(0),
            r(0),
            half,
            symbolic_half.clone(),
            r(1),
            r(1),
            r(1),
        ],
        &CurveContext::APPROXIMATE_512,
    )
    .expect("the terminal policy must retain the symbolically repeated knot")
    .into_value();
    (curve, symbolic_half)
}

#[test]
fn polynomial_spline_construction_obeys_terminal_policy_without_replacing_knots() {
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    let symbolic_end = r(1) + undecidable_zero.clone();
    let controls = vec![p(0, 0), p(2, 0)];
    let knots = vec![r(0), r(0), symbolic_end.clone(), r(1)];

    assert!(matches!(
        PolynomialSplineCurve2::try_new(
            1,
            controls.clone(),
            knots.clone(),
            &CurveContext::STRICT,
        ),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Construction
                && blocker.reason() == hypercurve::UncertaintyReason::Ordering
    ));

    let constructed =
        PolynomialSplineCurve2::try_new(1, controls, knots.clone(), &CurveContext::APPROXIMATE_512)
            .expect("the terminal policy must validate the symbolic clamped knot");
    assert_eq!(
        constructed.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(constructed.value.knots(), knots);
    assert_eq!(constructed.value.parameter_domain().1, &symbolic_end);

    let half = q(1, 2);
    let symbolic_half = &half + undecidable_zero;
    let evaluation_curve = PolynomialSplineCurve2::try_new(
        2,
        vec![p(0, 0), p(1, 2), p(2, 0), p(3, -2), p(4, 0)],
        vec![
            r(0),
            r(0),
            r(0),
            half,
            symbolic_half.clone(),
            r(1),
            r(1),
            r(1),
        ],
        &CurveContext::APPROXIMATE_512,
    )
    .expect("the terminal policy must retain the symbolically repeated interior knot")
    .into_value();

    assert!(matches!(
        evaluation_curve.bezier_decomposition(&CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::BezierDecomposition
                && blocker.reason() == hypercurve::UncertaintyReason::Ordering
    ));
    let decomposition = evaluation_curve
        .bezier_decomposition(&CurveContext::APPROXIMATE_512)
        .expect("the terminal policy must decompose the exact symbolic carrier");
    assert_eq!(
        decomposition.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(decomposition.value.spans().len(), 2);
    assert!(
        decomposition
            .value
            .refined_knots()
            .iter()
            .any(|knot| knot == &symbolic_half)
    );
    let spans = evaluation_curve
        .bezier_spans(&CurveContext::APPROXIMATE_512)
        .expect("borrowed spans must preserve terminal decomposition ownership");
    assert_eq!(
        spans.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(spans.into_value().len(), 2);

    assert!(matches!(
        evaluation_curve.point_at(&symbolic_half, &CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Evaluation
                && blocker.reason() == hypercurve::UncertaintyReason::Ordering
    ));
    let point = evaluation_curve
        .point_at(&symbolic_half, &CurveContext::APPROXIMATE_512)
        .expect("the terminal policy must evaluate the exact symbolic knot");
    assert_eq!(
        point.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(point.value, p(2, 0));
    let derivatives = evaluation_curve
        .derivatives_at_side(
            &symbolic_half,
            2,
            CurveParameterSide2::Left,
            &CurveContext::APPROXIMATE_512,
        )
        .expect("higher derivatives must use the selected terminal policy");
    assert_eq!(
        derivatives.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(derivatives.value.len(), 2);

    let top_level = Curve2::from(evaluation_curve);
    let top_level_point = top_level
        .as_view()
        .point_at(&symbolic_half, &CurveContext::APPROXIMATE_512)
        .expect("CurveView2 must preserve spline evaluation certainty");
    assert_eq!(
        top_level_point.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(top_level_point.value, p(2, 0));
    assert!(matches!(
        top_level.point_at(&symbolic_half, &CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Evaluation
                && blocker.reason() == hypercurve::UncertaintyReason::Ordering
    ));
}

#[test]
fn polynomial_subdivision_reconstruction_obeys_terminal_policy() {
    let curve = two_span_cubic();
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    let parameter = r(1) + undecidable_zero;

    assert!(matches!(
        curve.split_at(parameter.clone(), &CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Subdivision
                && blocker.reason() == hypercurve::UncertaintyReason::Ordering
    ));

    let split = curve
        .split_at(parameter.clone(), &CurveContext::APPROXIMATE_512)
        .expect("the terminal policy must reach the unit-weight NURBS kernel");
    assert_eq!(
        split.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    let (left, right) = split.into_value();
    assert_eq!(left.parameter_domain(), (&r(0), &parameter));
    assert_eq!(right.parameter_domain(), (&parameter, &r(2)));
    assert_eq!(left.end(), right.start());

    let clamped = curve
        .clamped_subcurve(parameter.clone(), r(2), &CurveContext::APPROXIMATE_512)
        .expect("terminal policy must reach clamped polynomial reconstruction");
    assert_eq!(
        clamped.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(clamped.value.parameter_domain(), (&parameter, &r(2)));

    assert!(matches!(
        curve.subcurve(parameter, r(2), &CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Subdivision
                && blocker.reason() == hypercurve::UncertaintyReason::Ordering
    ));
}

#[test]
fn polynomial_exact_edits_obey_terminal_policy_through_unit_weight_nurbs() {
    let (curve, symbolic_half) = terminal_polynomial_spline();
    let knot = q(3, 4);

    let strict_insertion = curve
        .insert_knot(knot.clone(), &CurveContext::STRICT)
        .unwrap_err();
    assert!(matches!(
        strict_insertion,
        ExactCurveError::Blocked(blocker)
            if blocker.operation() == CurveOperation2::KnotInsertion
                && blocker.family() == CurveFamily2::PolynomialBSpline
    ));
    let inserted = curve
        .insert_knot(knot, &CurveContext::APPROXIMATE_512)
        .expect("the terminal policy must refine the polynomial carrier");
    assert_eq!(
        inserted.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(inserted.value.control_points().len(), 6);
    assert!(
        inserted
            .value
            .knots()
            .iter()
            .any(|value| value == &symbolic_half)
    );
    assert_eq!(
        curve
            .insert_knot(q(3, 4), &CurveContext::STRICT)
            .unwrap_err(),
        strict_insertion
    );

    assert!(matches!(
        curve.reversed(&CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Reversal
                && blocker.family() == CurveFamily2::PolynomialBSpline
    ));
    let reversed = curve
        .reversed(&CurveContext::APPROXIMATE_512)
        .expect("the terminal policy must validate reflected polynomial knots");
    assert_eq!(
        reversed.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(reversed.value.start(), curve.end());
    assert_eq!(reversed.value.end(), curve.start());

    let transform = Similarity2::try_from_real_affine(
        Real::one(),
        Real::zero(),
        Real::zero(),
        Real::one(),
        r(5),
        r(-2),
    )
    .unwrap();
    assert!(matches!(
        curve.transform_similarity(&transform, &CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Transformation
                && blocker.family() == CurveFamily2::PolynomialBSpline
    ));
    let transformed = Curve2::from(curve.clone())
        .transform_similarity(&transform, &CurveContext::APPROXIMATE_512)
        .expect("Curve2 must propagate the policy through polynomial reconstruction");
    assert_eq!(
        transformed.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert!(matches!(
        Curve2::from(curve).reversed(&CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Reversal
    ));
}

#[test]
fn linear_polynomial_spline_evaluates_elevated_spans() {
    let curve = PolynomialSplineCurve2::try_new(
        1,
        vec![p(0, 0), p(2, 2), p(4, 0)],
        vec![r(0), r(0), r(1), r(2), r(2)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let half = (r(1) / r(2)).unwrap();
    let three_halves = (r(3) / r(2)).unwrap();

    assert_eq!(curve.degree(), 1);
    assert_eq!(
        curve
            .point_at(&half, &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(1, 1)
    );
    assert_eq!(
        curve
            .point_at(&three_halves, &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(3, 1)
    );
    assert_eq!(
        curve
            .bezier_decomposition(&CurveContext::STRICT)
            .unwrap()
            .into_value()
            .spans()
            .len(),
        2
    );
    assert!(
        curve
            .bezier_decomposition(&CurveContext::STRICT)
            .unwrap()
            .into_value()
            .spans()
            .iter()
            .all(|span| matches!(span, BezierSubcurve2::Quadratic(_)))
    );
}

#[test]
fn polynomial_spline_clones_share_one_decomposition() {
    let curve = two_span_cubic();
    let clone = curve.clone();

    let first = curve
        .bezier_decomposition(&CurveContext::STRICT)
        .unwrap()
        .into_value();
    let second = clone
        .bezier_decomposition(&CurveContext::STRICT)
        .unwrap()
        .into_value();

    assert!(std::ptr::eq(first, second));
    assert_eq!(first.spans().len(), 2);
    assert_eq!(first.intervals(), &[(r(0), r(1)), (r(1), r(2))]);
    assert_eq!(first.intervals().len(), first.spans().len());

    let spans = curve
        .bezier_spans(&CurveContext::STRICT)
        .unwrap()
        .into_value()
        .collect::<Vec<_>>();
    assert_eq!(spans.len(), 2);
    assert_eq!(spans[0].span_index(), 0);
    assert_eq!(spans[1].span_index(), 1);
    assert_eq!(spans[0].knot_interval(), (&r(0), &r(1)));
    assert_eq!(spans[1].knot_interval(), (&r(1), &r(2)));
    assert!(std::ptr::eq(spans[0].curve(), &first.spans()[0]));
}
#[test]
fn higher_degree_polynomial_spline_uses_exact_unit_weight_bezier_spans() {
    let curve = PolynomialSplineCurve2::try_new(
        4,
        vec![p(0, 0), p(1, 4), p(2, 0), p(3, 4), p(4, 0)],
        [vec![r(0); 5], vec![r(1); 5]].concat(),
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();

    assert_eq!(curve.degree(), 4);
    assert_eq!(
        curve
            .point_at(&(r(1) / r(2)).unwrap(), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(2, 2)
    );
    let spans = curve
        .bezier_spans(&CurveContext::STRICT)
        .unwrap()
        .into_value()
        .collect::<Vec<_>>();
    assert_eq!(spans.len(), 1);
    let BezierSubcurve2::Rational(span) = spans[0].curve() else {
        panic!("degree-four polynomial span did not use the general exact carrier");
    };
    assert_eq!(span.degree(), 4);
    assert!(span.weights().iter().all(|weight| *weight == r(1)));

    let clone = curve.clone();
    let derivatives = clone
        .derivatives_at(&(r(1) / r(2)).unwrap(), 4, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(
        curve
            .derivatives_at(&(r(1) / r(2)).unwrap(), 4, &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        derivatives
    );
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
    let curve = PolynomialSplineCurve2::try_new(
        2,
        vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
        (0..=6).map(r).collect(),
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();

    assert_eq!(curve.parameter_domain(), (&r(2), &r(4)));
    assert_eq!(curve.start(), &Point2::new(r(1), r(2)));
    assert_eq!(curve.end(), &Point2::new(r(5), r(2)));
    assert_eq!(
        curve
            .point_at(&r(2), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve.start().clone()
    );
    assert_eq!(
        curve
            .point_at(&r(4), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve.end().clone()
    );

    let reversed = curve.reversed(&CurveContext::STRICT).unwrap().into_value();
    assert_eq!(reversed.start(), curve.end());
    assert_eq!(reversed.end(), curve.start());
    assert_eq!(
        reversed
            .point_at(&r(3), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve
            .point_at(&r(3), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
}

#[test]
fn polynomial_spline_corner_requires_explicit_derivative_side() {
    let curve = PolynomialSplineCurve2::try_new(
        1,
        vec![p(0, 0), p(1, 0), p(1, 1)],
        vec![r(0), r(0), r(1), r(2), r(2)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();

    assert!(matches!(
        curve.derivative_at(&r(1), &CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.reason() == hypercurve::UncertaintyReason::Boundary
    ));
    let left = curve
        .derivative_at_side(
            &r(1),
            hypercurve::CurveParameterSide2::Left,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value();
    let right = curve
        .derivative_at_side(
            &r(1),
            hypercurve::CurveParameterSide2::Right,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value();
    assert_eq!((left.dx(), left.dy()), (&r(1), &r(0)));
    assert_eq!((right.dx(), right.dy()), (&r(0), &r(1)));
}

#[test]
fn discontinuous_polynomial_knot_requires_explicit_point_side() {
    let curve = PolynomialSplineCurve2::try_new(
        2,
        vec![p(0, 0), p(1, 1), p(2, 0), p(10, 0), p(11, 1), p(12, 0)],
        vec![r(0), r(0), r(0), r(1), r(1), r(1), r(2), r(2), r(2)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();

    assert!(matches!(
        curve.point_at(&r(1), &CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.reason() == hypercurve::UncertaintyReason::Boundary
    ));
    assert_eq!(
        curve
            .point_at_side(
                &r(1),
                hypercurve::CurveParameterSide2::Left,
                &CurveContext::STRICT
            )
            .unwrap()
            .into_value(),
        p(2, 0)
    );
    assert_eq!(
        curve
            .point_at_side(
                &r(1),
                hypercurve::CurveParameterSide2::Right,
                &CurveContext::STRICT
            )
            .unwrap()
            .into_value(),
        p(10, 0)
    );
}

#[test]
fn polynomial_spline_interior_knot_uses_retained_span_boundary() {
    let curve = two_span_cubic();
    let decomposition = curve
        .bezier_decomposition(&CurveContext::STRICT)
        .unwrap()
        .into_value();
    let expected = match &decomposition.spans()[0] {
        BezierSubcurve2::Cubic(span) => span.end().clone(),
        _ => panic!("cubic B-spline produced a non-cubic span"),
    };

    assert_eq!(
        curve
            .point_at(&r(1), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        expected
    );
}

#[test]
fn polynomial_spline_reversal_preserves_domain_source_and_image() {
    let curve = two_span_cubic();
    let reversed = curve.reversed(&CurveContext::STRICT).unwrap().into_value();

    assert_eq!(reversed.parameter_domain(), curve.parameter_domain());
    assert_eq!(reversed.start(), curve.end());
    assert_eq!(reversed.end(), curve.start());
    assert_eq!(
        reversed
            .point_at(&(r(1) / r(2)).unwrap(), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve
            .point_at(&(r(3) / r(2)).unwrap(), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
    assert_eq!(
        reversed
            .reversed(&CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve
    );
}

#[test]
fn polynomial_spline_knot_insertion_split_and_subcurve_are_exact() {
    let curve = PolynomialSplineCurve2::try_new(
        2,
        vec![p(0, 0), p(2, 4), p(4, 0)],
        vec![r(0), r(0), r(0), r(2), r(2), r(2)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let samples = [
        r(0),
        (r(1) / r(2)).unwrap(),
        r(1),
        (r(3) / r(2)).unwrap(),
        r(2),
    ];
    let expected = samples
        .iter()
        .map(|parameter| {
            curve
                .point_at(parameter, &CurveContext::STRICT)
                .unwrap()
                .into_value()
        })
        .collect::<Vec<_>>();

    let inserted = curve
        .insert_knot(r(1), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(
        inserted.control_points().len(),
        curve.control_points().len() + 1
    );
    assert_eq!(
        samples
            .iter()
            .map(|parameter| inserted
                .point_at(parameter, &CurveContext::STRICT)
                .unwrap()
                .into_value())
            .collect::<Vec<_>>(),
        expected
    );

    let (left, right) = curve
        .split_at(r(1), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(left.parameter_domain(), (&r(0), &r(1)));
    assert_eq!(right.parameter_domain(), (&r(1), &r(2)));
    assert_eq!(
        left.end(),
        &curve
            .point_at(&r(1), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
    assert_eq!(right.start(), left.end());

    let middle = curve
        .subcurve(
            (r(1) / r(2)).unwrap(),
            (r(3) / r(2)).unwrap(),
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value();
    assert_eq!(
        middle.start(),
        &curve
            .point_at(&(r(1) / r(2)).unwrap(), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
    assert_eq!(
        middle.end(),
        &curve
            .point_at(&(r(3) / r(2)).unwrap(), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
}

#[test]
fn invalid_polynomial_spline_evidence_context_and_source() {
    let error = PolynomialSplineCurve2::try_new(
        4,
        vec![p(0, 0), p(1, 1), p(2, 1), p(3, 1), p(4, 0)],
        vec![r(0); 10],
        &CurveContext::STRICT,
    )
    .unwrap_err();

    assert_eq!(error.operation(), CurveOperation2::Construction);
    assert_eq!(error.family(), CurveFamily2::PolynomialBSpline);
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
    let curve = two_span_cubic();
    let error = curve.point_at(&r(3), &CurveContext::STRICT).unwrap_err();

    assert_eq!(error.operation(), CurveOperation2::Evaluation);
    assert_eq!(error.family(), CurveFamily2::PolynomialBSpline);
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
    let curve = PolynomialSplineCurve2::try_new_periodic(
        2,
        vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)],
        (0..=4).map(r).collect(),
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();

    assert!(matches!(
        curve.periodicity(),
        SplinePeriodicity2::Periodic { .. }
    ));
    assert_eq!(curve.period(), Some(&r(4)));
    assert_eq!(curve.start(), curve.end());
    assert_eq!(
        curve
            .point_at_wrapped(&r(-1), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(0, 1)
    );
    assert_eq!(
        curve
            .point_at_wrapped(&r(5), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(2, 1)
    );
    assert_eq!(
        curve
            .derivatives_at_wrapped(&q(11, 2), 4, &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve
            .derivatives_at(&q(3, 2), 4, &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
}

#[test]
fn periodic_polynomial_wrapping_obeys_terminal_policy() {
    let curve = PolynomialSplineCurve2::try_new_periodic(
        2,
        vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)],
        (0..=4).map(r).collect(),
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    let wrapped_seam = r(4) + undecidable_zero;

    assert!(matches!(
        curve.point_at_wrapped(&wrapped_seam, &CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Evaluation
                && blocker.reason() == hypercurve::UncertaintyReason::Ordering
    ));
    let point = curve
        .point_at_wrapped(&wrapped_seam, &CurveContext::APPROXIMATE_512)
        .expect("the terminal policy must resolve an undecidable periodic seam");
    assert_eq!(
        point.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(point.value, curve.start().clone());

    let derivative = curve
        .derivative_at_wrapped_side(
            &wrapped_seam,
            CurveParameterSide2::Right,
            &CurveContext::APPROXIMATE_512,
        )
        .expect("wrapped derivative selection must share the terminal policy");
    assert_eq!(
        derivative.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(
        derivative.value,
        curve
            .derivative_at_side(&r(0), CurveParameterSide2::Right, &CurveContext::STRICT,)
            .unwrap()
            .into_value()
    );

    let top_level = Curve2::from(curve);
    let top_level_point = top_level
        .as_view()
        .point_at_wrapped(&wrapped_seam, &CurveContext::APPROXIMATE_512)
        .expect("CurveView2 must preserve wrapped terminal certainty");
    assert_eq!(
        top_level_point.certainty,
        hypercurve::CurveCertainty::Approximate512Consumed
    );
    assert_eq!(top_level_point.value, top_level.start().clone());
}

#[test]
fn periodic_polynomial_editing_preserves_only_whole_curve_periodicity() {
    let curve = PolynomialSplineCurve2::try_new_periodic(
        2,
        vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)],
        (0..=4).map(r).collect(),
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();

    let inserted = curve
        .insert_knot(q(1, 2), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(inserted.period(), curve.period());
    assert_eq!(
        inserted
            .point_at_wrapped(&r(5), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve
            .point_at_wrapped(&r(5), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );

    let reversed = curve.reversed(&CurveContext::STRICT).unwrap().into_value();
    assert_eq!(reversed.period(), curve.period());
    assert_eq!(
        reversed
            .point_at_wrapped(&r(1), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        curve
            .point_at_wrapped(&r(3), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );

    let (left, right) = curve
        .split_at(r(2), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(left.period(), None);
    assert_eq!(right.period(), None);

    let clamped = curve
        .clamped_subcurve(r(0), r(1), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(clamped.period(), None);
    assert_eq!(clamped.parameter_domain(), (&r(0), &r(1)));
    assert_eq!(clamped.knots(), &[r(0), r(0), r(0), r(1), r(1), r(1)]);
    assert_eq!(
        clamped.start(),
        &curve
            .point_at(&r(0), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
    assert_eq!(
        clamped.end(),
        &curve
            .point_at(&r(1), &CurveContext::STRICT)
            .unwrap()
            .into_value()
    );
}

#[test]
fn periodic_polynomial_spline_evidence_layout_and_wrapping_errors() {
    let invalid = PolynomialSplineCurve2::try_new_periodic(
        3,
        vec![p(0, 0), p(1, 0), p(1, 1)],
        vec![r(0), r(1), r(2), r(3)],
        &CurveContext::STRICT,
    )
    .unwrap_err();
    assert!(matches!(
        invalid,
        ExactCurveError::Invalid {
            cause: CurveError::InvalidPeriodicSpline,
            ..
        }
    ));

    let open = two_span_cubic();
    let error = open
        .point_at_wrapped(&r(3), &CurveContext::STRICT)
        .unwrap_err();
    assert!(matches!(
        error,
        ExactCurveError::Invalid {
            cause: CurveError::CurveIsNotPeriodic,
            ..
        }
    ));
}
