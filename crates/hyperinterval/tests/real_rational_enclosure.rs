#![cfg(feature = "hyperreal")]

use hyperinterval::{IntervalError, RationalInterval};
use hyperreal::{Rational, Real};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

#[test]
fn rational_points_survive_every_precision_and_subsequent_set_arithmetic() {
    for n in [-7, 0, 7] {
        let x = Rational::fraction(n, 3).unwrap();
        let source = Real::from(x.clone());
        for precision in [i32::MIN, -127, 0, 4, i32::MAX] {
            let enclosure = RationalInterval::from_real(&source, precision).unwrap();
            assert_eq!(enclosure.bounds(), Some((&x, &x)));
            assert_eq!(
                enclosure.scale(&Rational::from(-3)),
                RationalInterval::point(Rational::from(-n))
            );
        }
    }
}

#[test]
fn scaled_symbolic_bounds_keep_their_sign_and_can_be_retried_after_abort() {
    for scale in [-1, 1] {
        for warm in [false, true] {
            let mut value =
                Real::from(Rational::fraction(scale, 3).unwrap()) * Real::from(2).sqrt().unwrap();
            if warm {
                RationalInterval::from_real(&value, -48).unwrap();
            }
            let stop = Arc::new(AtomicBool::new(true));
            value.abort(stop.clone());
            assert_eq!(
                RationalInterval::from_real(&value, -16),
                Err(IntervalError::EnclosureUnavailable)
            );
            stop.store(false, Ordering::Relaxed);
            for p in [-16, -64, -16] {
                let enclosure = RationalInterval::from_real(&value, p).unwrap();
                // Restore the positive branch and verify that squaring brackets 2.
                let positive = enclosure.scale(&Rational::from(3 * scale));
                let (lo, hi) = positive.bounds().unwrap();
                assert!(lo > &Rational::zero());
                assert!(lo * lo < Rational::from(2));
                assert!(hi * hi > Rational::from(2));
                // At this precision the unscaled root bounds differ by two grid units.
                let step = Rational::from(2).powi(p.into()).unwrap();
                assert_eq!(hi - lo, &step + &step);
            }
        }
    }
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn conversion_performs_one_checked_scalar_enclosure() {
    use hyperreal::dispatch_trace;

    for scale in [-1, 0, 1] {
        let source =
            || Real::from(Rational::fraction(scale, 3).unwrap()) * Real::from(2).sqrt().unwrap();
        let (reference, value) = (source(), source());
        for p in [-16, -64, -16] {
            dispatch_trace::reset();
            let expected = dispatch_trace::with_recording(|| {
                let [lo, hi] = reference.certified_rational_interval(p).unwrap();
                RationalInterval::new(lo, hi).unwrap()
            });
            let counts = dispatch_trace::take_trace();
            let actual = dispatch_trace::with_recording(|| RationalInterval::from_real(&value, p));
            assert_eq!(dispatch_trace::take_trace(), counts);
            assert_eq!(actual, Ok(expected));
        }
    }
}

#[cfg(not(target_env = "msvc"))]
#[path = "../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;

#[cfg(not(target_env = "msvc"))]
#[path = "support/rational_oracle.rs"]
mod rational_oracle;

#[cfg(not(target_env = "msvc"))]
#[test]
fn scaled_pi_encloses_both_directed_mpfr_bounds() {
    use rational_oracle::mpq;
    use rug::{
        Float,
        float::{Constant, Round},
    };

    mpfr_cache::cleanup_on_thread_exit();
    let pi_bound = |round| {
        Float::with_val_round(512, Constant::Pi, round)
            .0
            .to_rational()
            .unwrap()
    };
    let (lo, hi) = (pi_bound(Round::Down), pi_bound(Round::Up));
    for (n, d) in [(-37, 11), (1, 1), (1001, 7)] {
        let scale = Rational::fraction(n, d).unwrap();
        let scaled = Real::from(scale.clone()) * Real::pi();
        let factor = mpq(&scale);
        let (lower, upper) = if n < 0 { (&hi, &lo) } else { (&lo, &hi) };
        for p in [0, -16, -128, -256, -16] {
            let interval = RationalInterval::from_real(&scaled, p).unwrap();
            let (a, b) = interval.bounds().unwrap();
            assert!(mpq(a) <= factor.clone() * lower);
            assert!(mpq(b) >= factor.clone() * upper);
        }
    }
}
