//! Open closure case: compare a selected arc image with expanded Real coordinates.
//!
//! The curve's selected parameter transport and source chart are certified.
//! On the evaluation checkpoint, coincidence with the independently expanded
//! scalar evaluation returns Uncertain(Unsupported); coordinate comparison
//! reaches an unresolved Real-coefficient polynomial sign.

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, CircularArc2, Classification, Curve2, CurveContext,
    CurveParameter2, Point2,
};
use hypercurve::Real;

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("fixture: {reason:?}"),
    }
}

fn main() {
    let policy = CurveContext::STRICT;
    let polynomial = decided(
        BezierParameterPolynomial::try_new_power_basis(
            vec![-Real::one(), Real::zero(), Real::from(2)], &policy,
        ).unwrap(),
    );
    let interval = decided(
        BezierParameterInterval::try_new(
            (Real::one() / Real::from(2)).unwrap(), Real::one(), &policy,
        ).unwrap(),
    );
    let selected = CurveParameter2::from(BezierParameter2::Algebraic(decided(
        BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy).unwrap(),
    )));
    let represented = (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap();
    let curve = Curve2::from(CircularArc2::try_from_center(
        Point2::from_values(1, 0),
        Point2::new((Real::from(3) / Real::from(5)).unwrap(), (Real::from(4) / Real::from(5)).unwrap()),
        Point2::from_values(0, 0), true,
    ).unwrap());
    let selected_point = curve.point_at(&selected, &policy).unwrap().value;
    let scalar_point = curve.point_at(&represented.into(), &policy).unwrap().value;
    println!("{:?}", selected_point.coincides_with(&scalar_point, &policy));
}
