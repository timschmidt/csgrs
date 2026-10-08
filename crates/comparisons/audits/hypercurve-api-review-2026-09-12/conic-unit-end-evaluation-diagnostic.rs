#![allow(dead_code, unused_imports)]
use hypercurve::{
    BezierAlgebraicImageStatus, BezierAlgebraicParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Classification, CurveContext, Point2, QuadraticBezier2,
    RationalQuadraticBezier2, Real,
};
use hypercurve::{CubicBezier2, RationalBezier2};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn policy() -> CurveContext {
    CurveContext::STRICT
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::from_values(x, y)
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
    }
}

fn polynomial(coefficients: Vec<Real>) -> BezierParameterPolynomial {
    decided(BezierParameterPolynomial::try_new_power_basis(coefficients, &policy()).unwrap())
}

fn interval(start: Real, end: Real) -> BezierParameterInterval {
    decided(BezierParameterInterval::try_new(start, end, &policy()).unwrap())
}

fn isolate(
    polynomial: BezierParameterPolynomial,
    interval: BezierParameterInterval,
) -> BezierAlgebraicParameter2 {
    decided(BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy()).unwrap())
}

fn sqrt_half_parameter() -> BezierAlgebraicParameter2 {
    isolate(polynomial(vec![r(-1), r(0), r(2)]), interval(q(1, 2), r(1)))
}

fn main() {
    for (weight_case, weights) in [[r(1),Real::pi(),r(1)], [r(1),q(1,2).sqrt().unwrap(),r(1)]].into_iter().enumerate() {
      for reverse in [false,true] { for swap_axes in [false,true] {
        let mut points=[p(0,0),p(2,4),p(6,0)]; let mut weights=weights.clone();
        if swap_axes { points=points.map(|p| Point2::new(p.y().clone(),p.x().clone())); }
        if reverse {points.reverse(); weights.reverse();}
        let conic=RationalQuadraticBezier2::try_new(points[0].clone(),points[1].clone(),points[2].clone(),weights[0].clone(),weights[1].clone(),weights[2].clone()).unwrap();
        let general=RationalBezier2::from(conic.clone());
        for (parameter_case,t) in [q(1,2), (r(1)/Real::pi()).unwrap(), (r(2).sqrt().unwrap()/r(2)).unwrap()].into_iter().enumerate() {
          let parameter=isolate(polynomial(vec![-t.clone(),r(1)]), interval(r(0),r(1)));
          let image=conic.point_at_algebraic_parameter(&parameter,&CurveContext::STRICT).unwrap();
          for (family, point) in [("conic",decided(conic.point_at(t.clone(),&CurveContext::STRICT))), ("general",general.point_at(&t,&CurveContext::STRICT).unwrap())] {
            let x=image.x().unwrap().compare_to_real(point.x(),&CurveContext::STRICT);
            let y=image.y().unwrap().compare_to_real(point.y(),&CurveContext::STRICT);
            if x!=Classification::Decided(std::cmp::Ordering::Equal) || y!=Classification::Decided(std::cmp::Ordering::Equal) {println!("weight={weight_case} reverse={reverse} swap_axes={swap_axes} parameter={parameter_case} family={family}: x={x:?} y={y:?}");}
          }
        }
      }}
    }
}
