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
    let t = (Real::one() / Real::pi()).unwrap();
    let conic = RationalQuadraticBezier2::try_new(p(0,0), p(2,4), p(6,0), r(1), r(1)-(Real::pi()/r(2)).unwrap(), r(1)-Real::pi()).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        println!("pole {policy:?}: {:?}", conic.point_at(t.clone(), &policy));
    }
    for weights in [[r(1),r(2),r(3)], [r(2),r(3),r(5)], [r(1),q(-1,2),r(1)], [r(2),Real::pi(),r(3)]] {
      let conic = RationalQuadraticBezier2::try_new(p(0,0),p(2,4),p(6,0),weights[0].clone(),weights[1].clone(),weights[2].clone()).unwrap();
      for t in [q(1,2), (r(1)/Real::pi()).unwrap(), (r(2).sqrt().unwrap()/r(2)).unwrap()] {
        let parameter=isolate(polynomial(vec![-t.clone(),r(1)]), interval(r(0),r(1)));
        let point=conic.point_at_algebraic_parameter(&parameter,&CurveContext::STRICT).unwrap();
        let exact=decided(conic.point_at(t.clone(),&CurveContext::STRICT));
        println!("weights={weights:?} t={t:?} x={:?} y={:?}",point.x().unwrap().compare_to_real(exact.x(),&CurveContext::STRICT),point.y().unwrap().compare_to_real(exact.y(),&CurveContext::STRICT));
      }
    }
}
