use hypercurve::{LineSeg2, Point2};
use hyperreal::{Rational, Real};

fn rational(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).unwrap())
}

#[test]
fn line_subdivision_preserves_constant_coordinate_identity() {
    let x = (Real::from(5).sqrt().unwrap() - Real::from(3)) / Real::from(6);
    let x = x.unwrap();
    let line = LineSeg2::try_new(
        Point2::new(x.clone(), Real::zero()),
        Point2::new(x.clone(), Real::one()),
    )
    .unwrap();

    let interior = line.point_at(rational(2, 5));
    assert_eq!(interior.x(), &x);
}
