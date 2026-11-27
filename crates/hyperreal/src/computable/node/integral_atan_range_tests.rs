/// Qualify raw IntegralAtan(n), which means atan(1/n), for nonzero integer denominators.
/// Directed MPFR enclosures cover unit identities and nonunit series under precision
/// reuse, cancellation and concurrent access. Each MPFR-using thread registers cleanup;
/// the finite cases do not certify arbitrary precision arithmetic or concurrency schedules.
#[cfg(test)]
mod integral_atan_range_tests {
    use super::*;
    use rug::{Float, Integer, Rational as Q, float::Round};

    /// Build IntegralAtan(n) with no initial bound/sign evidence or abort signal.
    /// Convert the integer exactly without reducing the tag; nonzero n is a caller
    /// precondition for approximation, and this helper performs no domain validation.
    fn imported(n: &Integer) -> Computable {
        Computable {
            internal: Arc::new(Node::new(
                Approximation::IntegralAtan(n.to_string().parse().unwrap()),
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    /// Enclose atan(1/n) for a nonzero integer by directed 2048-bit MPFR evaluation.
    /// Form the reciprocal as an exact rational before outward input rounding, then use
    /// atan's monotonicity. Invalid division or nonfinite endpoint conversion panics.
    fn oracle(n: &Integer) -> [Q; 2] {
        let q = Q::from((Integer::from(1), n.clone()));
        let mut lo = Float::with_val_round(2048, &q, Round::Down).0;
        let mut hi = Float::with_val_round(2048, &q, Round::Up).0;
        lo.atan_round(Round::Down);
        hi.atan_round(Round::Up);
        [lo.to_rational().unwrap(), hi.to_rational().unwrap()]
    }

    /// Require the integer approximation's closed one-unit interval to contain the entire oracle interval.
    /// Interpret the returned integer exactly as a*2^p, with error radius 2^p, using Rug rationals.
    /// This checks containment, not correct rounding or equality of cold and warm samples.
    /// Fixture precisions must permit the signed shifts (in particular p must not be i32::MIN);
    /// conversion or a failed enclosure panics. The supplied bounds must already be valid.
    fn encloses(value: &Computable, p: i32, bounds: &[Q; 2]) {
        let a = Integer::from_str_radix(&value.approx(p).to_string(), 10).unwrap();
        let unit = if p > 0 {
            Q::from(Integer::from(1) << p)
        } else {
            Q::from((1, Integer::from(1) << -p))
        };
        let center = Q::from(a) * &unit;
        assert!(
            Q::from(&center - &unit) <= bounds[0] && Q::from(&center + &unit) >= bounds[1],
            "p={p}, center={center}, unit={unit}"
        );
    }

    /// Check both signs of denominators 1,2,5,239 and 2^512 at 13 precision requests.
    /// Fresh and reused imported nodes must each enclose the independent atan(1/n)
    /// interval; unit denominators exercise the native angle identity instead of its series.
    #[test]
    fn imported_integral_atan_encloses_unit_and_nonunit_denominators() {
        crate::mpfr_test::cleanup_on_thread_exit();
        let mut magnitudes: Vec<Integer> = [1, 2, 5, 239].into_iter().map(Integer::from).collect();
        magnitudes.push(Integer::from(1) << 512);
        for magnitude in magnitudes {
            for n in [magnitude.clone(), -magnitude] {
                let bounds = oracle(&n);
                let warm = imported(&n);
                for p in [4, 2, 1, 0, -1, -8, -32, -128, -512, -8, 2, 1, 0] {
                    encloses(&imported(&n), p, &bounds);
                    encloses(&warm, p, &bounds);
                }
            }
        }
    }

    /// Require cancelled atan(1/±1) requests to leave the queried numerical cache empty.
    /// After clearing the flag, compare the -128 retry with a subsequent warm call and
    /// check its MPFR enclosure. The abort integer is discarded and entry cancellation
    /// alone is covered; this does not assert every child cache remains empty.
    #[test]
    fn integral_atan_unit_identity_recovers_after_cancellation() {
        use std::sync::atomic::{AtomicBool, Ordering};
        crate::mpfr_test::cleanup_on_thread_exit();
        for n in [Integer::from(-1), Integer::from(1)] {
            let value = imported(&n);
            let signal = Arc::new(AtomicBool::new(true));
            let _ = value.approx_signal(&Some(signal.clone()), -128);
            assert!(value.cached().is_none());
            signal.store(false, Ordering::Relaxed);
            let resumed = value.approx_signal(&Some(signal), -128);
            assert_eq!(resumed, value.approx(-128));
            encloses(&value, -128, &oracle(&n));
        }
    }

    /// Check six scoped readers of each shared unit-denominator atan node at mixed precisions.
    /// Workers register MPFR cleanup and assert enclosures; three sequential requests after
    /// joining check reuse. This finite concurrency witness does not establish race freedom.
    #[test]
    fn integral_atan_unit_identity_preserves_concurrent_enclosures() {
        crate::mpfr_test::cleanup_on_thread_exit();
        for n in [Integer::from(-1), Integer::from(1)] {
            let value = imported(&n);
            let bounds = oracle(&n);
            std::thread::scope(|scope| {
                for p in [-8, -32, -128, -512, 0, -32] {
                    let copy = value.clone();
                    let bounds = bounds.clone();
                    scope.spawn(move || {
                        crate::mpfr_test::cleanup_on_thread_exit();
                        encloses(&copy, p, &bounds);
                    });
                }
            });
            for p in [2, 0, -8] {
                encloses(&value, p, &bounds);
            }
        }
    }
}
