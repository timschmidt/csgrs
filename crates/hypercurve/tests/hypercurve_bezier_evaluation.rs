use hypercurve::{CubicBezier2, Point2, QuadraticBezier2, Real};

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
