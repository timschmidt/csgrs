#![cfg(feature = "real-ball")]

use hyperinterval::{BallRelation, RealBall};
use hyperreal::{Rational, Real};

#[test]
fn affine_ball_classifies_exact_rational_displacements_at_a_closed_boundary() {
    // Original [-2, 0]. Reflect, scale by 1/3 and translate by 1/3 to get [1/3, 1].
    let third = Rational::fraction(1, 3).unwrap();
    let original = RealBall::new(Real::from(-1), Rational::one()).unwrap();
    let ball = original.affine(&(-&third), &Real::from(third.clone()));
    let delta = Rational::fraction(1, 1000).unwrap();
    for (point, expected) in [
        (&third - &delta, BallRelation::Excluded),
        (third.clone(), BallRelation::Contained),
        (&third + &delta, BallRelation::Contained),
        (&Rational::one() - &delta, BallRelation::Contained),
        (Rational::one(), BallRelation::Contained),
        (&Rational::one() + &delta, BallRelation::Excluded),
    ] {
        for precision in [0, -8, -128, 0] {
            assert_eq!(
                ball.classify_point(&Real::from(point.clone()), precision)
                    .unwrap(),
                expected,
                "point {point:?}, precision {precision}"
            );
        }
    }
}

#[test]
fn closed_rational_balls_agree_with_integer_membership() {
    // The common denominator cancels, so the oracle uses only signed integers.
    for denominator in [3, 5, 7] {
        for center in -2_i64..=2 {
            for radius in 0..=2 {
                let ball = RealBall::new(
                    Real::from(Rational::fraction(center, denominator).unwrap()),
                    Rational::fraction(radius, denominator).unwrap(),
                )
                .unwrap();
                for value in -5_i64..=5 {
                    let point = Real::from(Rational::fraction(value, denominator).unwrap());
                    let expected = if (value - center).abs() <= radius {
                        BallRelation::Contained
                    } else {
                        BallRelation::Excluded
                    };
                    for precision in [1, -5, -80] {
                        assert_eq!(ball.classify_point(&point, precision).unwrap(), expected);
                    }
                }
            }
        }
    }
}

#[test]
fn symbolic_membership_preserves_uncertainty_and_cancellation_on_retry() {
    use hyperinterval::IntervalError;
    use std::sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    };

    let ball = RealBall::point(Real::from(Rational::fraction(7, 5).unwrap()));
    let mut point = Real::from(2).sqrt().unwrap();
    assert_eq!(
        ball.classify_point(&point, 0).unwrap(),
        BallRelation::Unresolved
    );
    let stop = Arc::new(AtomicBool::new(true));
    point.abort(stop.clone());
    assert_eq!(
        ball.classify_point(&point, -64),
        Err(IntervalError::EnclosureUnavailable)
    );
    stop.store(false, Ordering::Relaxed);
    assert_eq!(
        ball.classify_point(&point, -64).unwrap(),
        BallRelation::Excluded
    );
}
