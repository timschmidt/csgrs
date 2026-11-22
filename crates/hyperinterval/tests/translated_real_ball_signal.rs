#![cfg(feature = "real-ball")]

use hyperinterval::{BallRelation, IntervalError, RealBall};
use hyperreal::{Rational, Real};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

#[test]
fn membership_preserves_cancellation_through_nonzero_rational_translation() {
    // |sqrt(2) - 7/5| < 1/4 follows by squaring the two positive endpoints.
    for radius in [Rational::zero(), Rational::fraction(1, 4).unwrap()] {
        let mut root = Real::from(2).sqrt().unwrap();
        let q = Real::from(Rational::fraction(7, 5).unwrap());
        let expected = if radius == Rational::zero() {
            BallRelation::Excluded
        } else {
            BallRelation::Contained
        };
        let stop = Arc::new(AtomicBool::new(false));
        root.abort(stop.clone());
        for (point, center) in [(&root, &q), (&q, &root)] {
            let ball = RealBall::new(center.clone(), radius.clone()).unwrap();
            for _ in 0..2 {
                stop.store(true, Ordering::Relaxed);
                assert_eq!(
                    ball.classify_point(point, -64),
                    Err(IntervalError::EnclosureUnavailable)
                );
                stop.store(false, Ordering::Relaxed);
                assert_eq!(ball.classify_point(point, -64).unwrap(), expected);
            }
        }
    }
}
