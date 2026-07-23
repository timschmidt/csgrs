use hypercurve::{
    BezierFlatteningOptions, Classification, CubicBezier2, Curve2, CurvePolicy, Point2,
    QuadraticBezier2, RationalBezier2, Real,
};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (r(numerator) / r(denominator)).expect("nonzero denominator")
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn quadratic_de_casteljau(curve: &QuadraticBezier2, parameter: Real) -> Point2 {
    let left = curve.start().lerp(curve.control(), parameter.clone());
    let right = curve.control().lerp(curve.end(), parameter.clone());
    left.lerp(&right, parameter)
}

fn cubic_de_casteljau(curve: &CubicBezier2, parameter: Real) -> Point2 {
    let first = curve.start().lerp(curve.control1(), parameter.clone());
    let second = curve.control1().lerp(curve.control2(), parameter.clone());
    let third = curve.control2().lerp(curve.end(), parameter.clone());
    let left = first.lerp(&second, parameter.clone());
    let right = second.lerp(&third, parameter.clone());
    left.lerp(&right, parameter)
}

#[test]
fn optimized_polynomial_evaluation_matches_de_casteljau_exactly() {
    let quadratic = QuadraticBezier2::new(p(-3, 2), p(5, 11), p(13, -7));
    let cubic = CubicBezier2::new(p(-3, 2), p(2, 13), p(9, -8), p(15, 4));
    let root_half = (r(2).sqrt().expect("positive radicand") / r(2)).expect("nonzero denominator");
    let parameters = [
        r(-2),
        q(-1, 2),
        r(0),
        q(1, 4),
        q(1, 2),
        q(3, 4),
        r(1),
        q(3, 2),
        root_half,
    ];

    for parameter in parameters {
        assert_eq!(
            quadratic.point_at(parameter.clone()),
            quadratic_de_casteljau(&quadratic, parameter.clone())
        );
        assert_eq!(
            cubic.point_at(parameter.clone()),
            cubic_de_casteljau(&cubic, parameter)
        );
    }
}

#[test]
fn certified_exact_scalar_segmentation_covers_rational_bezier_and_nurbs() {
    let policy = CurvePolicy::certified();
    let options = BezierFlatteningOptions::try_new(q(1, 64), 16, &policy).unwrap();
    let rational = Curve2::from(
        RationalBezier2::try_new(
            vec![p(0, 0), p(1, 3), p(3, -2), p(5, 0)],
            vec![r(1), r(2), r(3), r(1)],
        )
        .unwrap(),
    );
    let nurbs = Curve2::try_nurbs(
        2,
        vec![p(0, 0), p(2, 4), p(4, 0)],
        vec![r(1), r(2), r(1)],
        vec![r(0), r(0), r(0), r(1), r(1), r(1)],
    )
    .unwrap();

    for curve in [rational, nurbs] {
        let Classification::Decided(segmented) =
            curve.segment_certified(&options, &policy).unwrap()
        else {
            panic!("same-sign rational carrier should segment with a control-hull certificate");
        };
        assert_eq!(segmented.points().first(), Some(curve.start()));
        assert_eq!(segmented.points().last(), Some(curve.end()));
        assert!(segmented.certificate().segment_count() > 1);
        assert!(
            segmented
                .points()
                .iter()
                .all(|point| point.x().exact_rational_ref().is_some()
                    && point.y().exact_rational_ref().is_some())
        );
    }
}
