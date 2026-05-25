use hypercurve::{
    BezierBoundaryLoop2, BezierRegion2, BezierSubcurve2, Classification, CurveError, CurvePolicy,
    Point2, PolynomialBSplineCurve2, Real,
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
