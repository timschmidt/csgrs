#![cfg(feature = "real-ball")]

use hyperinterval::{BallRelation, IntervalError, RationalInterval, RealBall};
use hyperreal::{Rational, Real};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

#[test]
fn reflected_ball_enclosures_keep_both_closed_endpoints_after_composition() {
    let third = Rational::fraction(1, 3).unwrap();
    let source = RealBall::new(Real::from(-1), Rational::one()).unwrap();
    let reflected = source.affine(&(-&third), &Real::from(third.clone()));
    // Reflecting and scaling [-2, 0] and then translating by 1/3 gives [1/3, 1].
    let doubled = reflected.add(&reflected);
    let difference = doubled.sub(&reflected);
    let cases = [(&reflected, 2, 1), (&doubled, 4, 2), (&difference, 2, 3)];
    for (ball, numerator, radius_numerator) in cases {
        let center = Rational::fraction(numerator, 3).unwrap();
        let radius = Rational::fraction(radius_numerator, 3).unwrap();
        assert_eq!(ball.center().exact_rational_ref(), Some(&center));
        assert_eq!(ball.radius(), &radius);
        for precision in [0, -7, -8, -128, 0] {
            // At 2^precision spacing, the fractional grid position is (n * 2^-p mod 3) / 3.
            let residue = numerator * if precision % 2 == 0 { 1 } else { 2 } % 3;
            let grid = Rational::from(2).powi(precision.into()).unwrap();
            let lower = &center - &grid * Rational::fraction(residue, 3).unwrap() - &radius;
            let upper = &center + &grid * Rational::fraction(3 - residue, 3).unwrap() + &radius;
            assert_eq!(
                ball.to_rational(precision).unwrap(),
                RationalInterval::new(lower, upper).unwrap()
            );
        }
    }
}

#[test]
fn unavailable_symbolic_displacement_can_be_retried_in_both_orders() {
    for warm in [false, true] {
        let mut root = Real::from(2).sqrt().unwrap();
        if warm {
            assert!(root.certified_rational_interval(-64).is_some());
        }
        let stop = Arc::new(AtomicBool::new(true));
        root.abort(stop.clone());
        let zero_ball = RealBall::point(Real::zero());
        let root_ball = RealBall::point(root.clone());
        assert_eq!(
            zero_ball.classify_point(&root, -64),
            Err(IntervalError::EnclosureUnavailable)
        );
        assert_eq!(
            root_ball.classify_point(&Real::zero(), -64),
            Err(IntervalError::EnclosureUnavailable)
        );
        stop.store(false, Ordering::Relaxed);
        assert_eq!(
            zero_ball.classify_point(&root, -64).unwrap(),
            BallRelation::Excluded
        );
        assert_eq!(
            root_ball.classify_point(&Real::zero(), -64).unwrap(),
            BallRelation::Excluded
        );
    }
}
