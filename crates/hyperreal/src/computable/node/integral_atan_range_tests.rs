#[cfg(test)]
mod integral_atan_range_tests {
    use super::*;
    use rug::{Float, Integer, Rational as Q, float::Round};

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

    fn oracle(n: &Integer) -> [Q; 2] {
        let q = Q::from((Integer::from(1), n.clone()));
        let mut lo = Float::with_val_round(2048, &q, Round::Down).0;
        let mut hi = Float::with_val_round(2048, &q, Round::Up).0;
        lo.atan_round(Round::Down);
        hi.atan_round(Round::Up);
        [lo.to_rational().unwrap(), hi.to_rational().unwrap()]
    }

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
