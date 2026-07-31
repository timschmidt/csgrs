use hypercurve::{
    BezierFlatteningOptions, Classification, CubicBezier2, Curve2, CurveContext, Point2,
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

fn cubic_bernstein(curve: &CubicBezier2, parameter: Real) -> Point2 {
    let one_minus_t = Real::one() - &parameter;
    let one_minus_t_squared = &one_minus_t * &one_minus_t;
    let parameter_squared = &parameter * &parameter;
    let start_weight = &one_minus_t_squared * &one_minus_t;
    let control1_weight = (&one_minus_t_squared * &parameter) * Real::from(3);
    let control2_weight = (&one_minus_t * &parameter_squared) * Real::from(3);
    let end_weight = &parameter_squared * &parameter;
    Point2::new(
        curve.start().x() * &start_weight
            + curve.control1().x() * &control1_weight
            + curve.control2().x() * &control2_weight
            + curve.end().x() * &end_weight,
        curve.start().y() * &start_weight
            + curve.control1().y() * &control1_weight
            + curve.control2().y() * &control2_weight
            + curve.end().y() * &end_weight,
    )
}

#[test]
fn optimized_polynomial_evaluation_matches_de_casteljau_exactly() {
    let quadratic = QuadraticBezier2::new(p(-3, 2), p(5, 11), p(13, -7));
    let cubic = CubicBezier2::new(p(-3, 2), p(2, 13), p(9, -8), p(15, 4));
    let sqrt_two = r(2).sqrt().expect("positive radicand");
    let symbolic_cubic = CubicBezier2::new(
        Point2::new(sqrt_two.clone(), r(2)),
        Point2::new(r(2), sqrt_two.clone()),
        Point2::new(-sqrt_two.clone(), r(5)),
        Point2::new(r(7), -sqrt_two),
    );
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
            cubic_de_casteljau(&cubic, parameter.clone())
        );
        let symbolic_expected = if parameter.exact_rational_ref().is_some() {
            cubic_bernstein(&symbolic_cubic, parameter.clone())
        } else {
            cubic_de_casteljau(&symbolic_cubic, parameter.clone())
        };
        assert_eq!(symbolic_cubic.point_at(parameter), symbolic_expected);
    }
}

#[test]
fn certified_exact_scalar_segmentation_covers_rational_bezier_and_nurbs() {
    let policy = CurveContext::STRICT;
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
        &policy,
    )
    .unwrap()
    .into_value();

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
