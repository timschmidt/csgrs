use hypercurve::{
    BezierLineImageFitRelation, BezierParameter2, BezierParameterRange2, CircularArc2,
    CubicBezier2, Curve2, CurveCertainty, CurveFamily2, CurveGeometry2, CurveParameterRange2,
    CurvePath2, CurveRegion2, ExactCurveError, FillRule, LineSeg2, Point2, PredicatePolicy,
    QuadraticBezier2, RationalBezier2, Real, Segment2, UncertaintyReason,
};

fn p(x: i32, y: i32) -> Point2 {
    Point2::from_values(x, y)
}

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn line_image(curve: &Curve2, start: Point2, end: Point2) {
    for _ in 0..2 {
        let image = curve.native_segment().unwrap();
        assert!(std::ptr::eq(image.source(), curve));
        let Segment2::Line(line) = image.segment() else {
            panic!("expected a line image");
        };
        assert!(
            hypercurve::CurvePoint2::from(line.start().clone())
                .coincides_with(&start.clone().into())
                .unwrap()
        );
        assert!(
            hypercurve::CurvePoint2::from(line.end().clone())
                .coincides_with(&end.clone().into())
                .unwrap()
        );
    }
}

fn unsupported(curve: &Curve2) {
    assert!(matches!(
        curve.native_segment(),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.reason() == UncertaintyReason::Unsupported
    ));
}

#[test]
fn native_and_source_restricted_lines_keep_source_charts_and_reversal() {
    let curve = Curve2::from(LineSeg2::try_new(p(-2, 3), p(6, 3)).unwrap());
    line_image(&curve, p(-2, 3), p(6, 3));
    let restricted = curve
        .restrict_source(CurveParameterRange2::try_new(q(1, 4).into(), q(3, 4).into()).unwrap())
        .unwrap();
    assert!(restricted.geometry().is_none());
    assert!(restricted.source_restriction().is_some());
    line_image(&restricted, p(0, 3), p(4, 3));
    line_image(&restricted.reversed().unwrap(), p(4, 3), p(0, 3));
    let (start, end) = restricted.parameter_endpoints();
    assert_eq!(start.scalar(), Some(&q(1, 4)));
    assert_eq!(end.scalar(), Some(&q(3, 4)));
}

#[test]
fn mixed_region_chord_retains_its_source_and_exact_orientation() {
    let path = CurvePath2::try_new(vec![
        QuadraticBezier2::new(p(-2, 4), p(0, -4), p(2, 4)).into(),
        LineSeg2::try_new(p(2, 4), p(-2, 4)).unwrap().into(),
    ])
    .unwrap();
    let region = CurveRegion2::try_from_boundary_paths(&[path], FillRule::EvenOdd).unwrap();
    let curves = region.boundary_loops()[0].curves();
    let chord = curves
        .iter()
        .find(|curve| curve.family() == CurveFamily2::Line && curve.geometry().is_none())
        .unwrap();
    assert!(chord.parameter_domain().start().scalar().is_none());
    line_image(chord, p(2, 4), p(-2, 4));
    line_image(&chord.reversed().unwrap(), p(-2, 4), p(2, 4));
    assert!(chord.parameter_domain().start().scalar().is_none());
    unsupported(
        curves
            .iter()
            .find(|curve| curve.family() == CurveFamily2::QuadraticBezier)
            .unwrap(),
    );
}

#[test]
fn native_circle_and_restricted_conic_keep_complete_arc_traversal() {
    let curve =
        Curve2::from(CircularArc2::try_from_center(p(1, 0), p(1, 0), p(0, 0), false).unwrap());
    let image = curve.native_segment().unwrap();
    assert!(std::ptr::eq(image.source(), &curve));
    let Segment2::Arc(arc) = image.segment() else {
        panic!("expected native circle");
    };
    assert_eq!(arc.start(), &p(1, 0));
    assert_eq!(arc.end(), &p(1, 0));
    assert!(!arc.is_clockwise());
    let quarter = curve
        .restrict_source(
            CurveParameterRange2::try_new(Real::zero().into(), q(1, 4).into()).unwrap(),
        )
        .unwrap();
    assert!(quarter.geometry().is_none());
    for (source, start, end, clockwise) in [
        (quarter.clone(), p(1, 0), p(0, 1), false),
        (quarter.reversed().unwrap(), p(0, 1), p(1, 0), true),
    ] {
        let image = source.native_segment().unwrap();
        let Segment2::Arc(arc) = image.segment() else {
            panic!("expected a quarter circle");
        };
        assert_eq!(arc.start(), &start);
        assert_eq!(arc.end(), &end);
        assert_eq!(arc.center(), &p(0, 0));
        assert_eq!(arc.is_clockwise(), clockwise);
    }
}

#[test]
fn represented_chord_offsets_stay_distinct_in_approximate_ambient_scope() {
    let gap: Real = "1e-12".parse().unwrap();
    let start = Point2::new(Real::pi(), gap.clone());
    let end = Point2::new(Real::pi() + Real::one(), gap);
    let curve = Curve2::try_line(start.clone().into(), end.clone().into()).unwrap();
    assert!(curve.geometry().is_none());
    let view =
        hypercurve::evaluate_under(PredicatePolicy::APPROXIMATE_512, || curve.native_segment());
    assert_eq!(view.certainty(), CurveCertainty::Certified);
    let view = view.into_unverified().unwrap();
    assert!(std::ptr::eq(view.source(), &curve));
    line_image(&curve, start, end);
}

#[test]
fn regular_line_parallel_reuses_the_complete_source_proof() {
    let source = QuadraticBezier2::new(p(0, 0), p(2, 0), p(4, 0));
    let range = BezierParameterRange2::try_new(
        BezierParameter2::Exact(Real::zero()),
        BezierParameter2::Exact(Real::one()),
    )
    .unwrap();
    let curve =
        Curve2::try_analytic_parallel(source.parallel_left(Real::one()).unwrap(), range).unwrap();
    assert_eq!(curve.family(), CurveFamily2::AnalyticParallel);
    line_image(&curve, p(0, 1), p(4, 1));
    line_image(&curve.reversed().unwrap(), p(4, 1), p(0, 1));
}

#[test]
fn source_line_fit_cannot_certify_a_nonmonotone_trim() {
    // x(t) = 4t(1-t)^3 + t^4 has all controls in [0,1], but reverses
    // between 1/4 and 1/2. Its point at 13/50 exceeds both trim endpoints.
    let source = RationalBezier2::try_new(
        vec![p(0, 0), p(1, 0), p(0, 0), p(0, 0), p(1, 0)],
        vec![Real::one(); 5],
    )
    .unwrap();
    assert!(matches!(
        source.fit_exact_line_image().unwrap(),
        BezierLineImageFitRelation::Fit(_)
    ));
    let curve = Curve2::from(source);
    let middle = curve.point_at(&q(13, 50).into()).unwrap();
    let x = middle.coordinates().unwrap().x();
    assert_eq!(x, &q(2_662_517, 6_250_000));
    assert_eq!(
        x.partial_cmp(&q(109, 256)),
        Some(std::cmp::Ordering::Greater)
    );
    let trim = curve
        .restrict_source(CurveParameterRange2::try_new(q(1, 4).into(), q(1, 2).into()).unwrap())
        .unwrap();
    unsupported(&trim);
    unsupported(&curve);

    // The source x=t² is injective only on its original unit chart.
    let exterior = Curve2::try_from_bezier_range(
        CurveGeometry2::QuadraticBezier(QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0))),
        CurveParameterRange2::try_new(Real::from(-1).into(), q(1, 2).into()).unwrap(),
    )
    .unwrap();
    assert!(exterior.geometry().is_none());
    unsupported(&exterior);
}

#[test]
fn multiple_spans_are_not_silently_merged() {
    let curve = Curve2::try_nurbs(
        1,
        vec![p(0, 0), p(1, 0), p(2, 0)],
        vec![Real::one(); 3],
        vec![0.into(), 0.into(), 1.into(), 2.into(), 2.into()],
    )
    .unwrap();
    assert_eq!(curve.native_bezier_fragments().unwrap().len(), 2);
    unsupported(&curve);
}

#[test]
fn native_monotone_polynomial_and_single_span_nurbs_images_remain_oriented() {
    for curve in [
        Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 0), p(4, 0))),
        Curve2::from(CubicBezier2::new(p(0, 0), p(0, 0), p(2, 0), p(4, 0))),
        Curve2::from(
            RationalBezier2::try_new(
                vec![p(0, 0), p(2, 0), p(4, 0)],
                vec![1.into(), 2.into(), 1.into()],
            )
            .unwrap(),
        ),
        Curve2::try_nurbs(
            2,
            vec![p(0, 0), p(2, 0), p(4, 0)],
            vec![Real::one(); 3],
            vec![0.into(), 0.into(), 0.into(), 1.into(), 1.into(), 1.into()],
        )
        .unwrap(),
    ] {
        assert!(curve.geometry().is_some());
        line_image(&curve, p(0, 0), p(4, 0));
        line_image(&curve.reversed().unwrap(), p(4, 0), p(0, 0));
    }
}
