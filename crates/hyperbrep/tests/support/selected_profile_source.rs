use hyperbrep::*;
use hypercurve::{
    BooleanOp, Curve2, CurvePath2, CurveRegion2, FillRule, LineSeg2, QuadraticBezier2,
};

fn p(x: i32, y: i32) -> hypercurve::Point2 {
    hypercurve::Point2::from_values(x, y)
}

pub(super) fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

pub(super) fn line(a: hypercurve::Point2, b: hypercurve::Point2) -> Curve2 {
    LineSeg2::try_new(a, b).unwrap().into()
}

pub fn profile(xor: bool, affine: bool, reverse: bool) -> (CurveRegion2, Real) {
    // x = 4t - 2, y = x^2; material lies between this parabola and y = 4.
    let path = CurvePath2::try_new(vec![
        QuadraticBezier2::new(p(-2, 4), p(0, -4), p(2, 4)).into(),
        line(p(2, 4), p(-2, 4)),
    ])
    .unwrap();
    let path = if reverse {
        path.reversed().unwrap()
    } else {
        path
    };
    let source = CurveRegion2::try_from_boundary_paths(&[path], FillRule::EvenOdd).unwrap();
    let points = [p(-3, 2), p(3, 2), p(3, 5), p(-3, 5)];
    let clip = CurvePath2::try_new(
        (0..4)
            .map(|i| line(points[i].clone(), points[(i + 1) % 4].clone()))
            .collect(),
    )
    .unwrap();
    let clip = CurveRegion2::try_from_boundary_paths(&[clip], FillRule::EvenOdd).unwrap();
    let mut region = source
        .boolean_region(
            &clip,
            if xor {
                BooleanOp::Xor
            } else {
                BooleanOp::Difference
            },
        )
        .unwrap();
    assert!(region.has_algebraic_fragments());
    // Integral of 2-x^2 from -sqrt(2) to sqrt(2); XOR adds the other difference.
    let root = Real::from(2).sqrt().unwrap();
    let mut area = if xor {
        q(22, 3) + q(16, 3) * root
    } else {
        q(8, 3) * root
    };
    if affine {
        region = region
            .transform_affine(
                &(-2).into(),
                &0.into(),
                &0.into(),
                &3.into(),
                &7.into(),
                &(-1).into(),
            )
            .unwrap();
        area *= Real::from(6);
    }
    (region, area)
}

pub fn replay(model: &Model) -> Model {
    let text = model.to_json().unwrap();
    let cold = RawModel::from_json(&text, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), text);
    cold
}
