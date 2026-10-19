#[cfg(test)]
mod acos_complement_range_tests {
    use super::*;
    use rug::{Float, Integer, Rational as Q, float::Round};

    fn imported(r: &Q) -> Computable {
        Computable {
            internal: Arc::new(Node::new(
                Approximation::AcosNegativeRational(r.to_string().parse().unwrap()),
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    fn oracle(r: &Q) -> [Q; 2] {
        let x = -r.clone();
        // acos is decreasing: the upper input bound supplies the lower output.
        let mut lo = Float::with_val_round(2048, &x, Round::Up).0;
        let mut hi = Float::with_val_round(2048, &x, Round::Down).0;
        lo.acos_round(Round::Down);
        hi.acos_round(Round::Up);
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
    fn negative_acos_complement_encloses_rational_and_endpoint_inputs() {
        crate::mpfr_test::cleanup_on_thread_exit();
        let mut inputs = vec![
            Q::from(0),
            Q::from((1, 16)),
            Q::from((1, 3)),
            Q::from((1, 2)),
            Q::from((3, 5)),
            Q::from((7, 10)),
            Q::from((49, 50)),
            Q::from((63, 64)),
            Q::from(1),
            Q::from((1, Integer::from(1) << 512)),
        ];
        for bits in [64, 512] {
            inputs.push(Q::from(1) - Q::from((1, Integer::from(1) << bits)));
        }
        assert_eq!(inputs.len(), 12);
        for r in inputs {
            let bounds = oracle(&r);
            for public in [false, true] {
                let build = || {
                    if public {
                        Computable::rational((-r.clone()).to_string().parse().unwrap()).acos()
                    } else {
                        imported(&r)
                    }
                };
                let warm = build();
                for p in [4, 2, 1, 0, -1, -2, -8, -32, -128, -512, -8, 2, 0] {
                    encloses(&build(), p, &bounds);
                    encloses(&warm, p, &bounds);
                }
            }
        }
    }

    #[test]
    fn negative_acos_complement_recovers_after_cancellation() {
        use std::sync::atomic::{AtomicBool, Ordering};
        crate::mpfr_test::cleanup_on_thread_exit();
        for r in [
            Q::from(0),
            Q::from(1),
            Q::from((49, 50)),
            Q::from(1) - Q::from((1, Integer::from(1) << 512)),
        ] {
            let value = imported(&r);
            let signal = Arc::new(AtomicBool::new(true));
            let _ = value.approx_signal(&Some(signal.clone()), -128);
            assert!(value.cached().is_none());
            signal.store(false, Ordering::Relaxed);
            let resumed = value.approx_signal(&Some(signal), -128);
            assert_eq!(resumed, value.approx(-128));
            encloses(&value, -128, &oracle(&r));
        }
    }

    #[test]
    fn negative_acos_complement_preserves_concurrent_enclosures() {
        crate::mpfr_test::cleanup_on_thread_exit();
        for r in [Q::from((1, 3)), Q::from((49, 50))] {
            let value = imported(&r);
            let bounds = oracle(&r);
            std::thread::scope(|scope| {
                for p in [-1, -32, -128, -512, 0, -2] {
                    let copy = value.clone();
                    let bounds = bounds.clone();
                    scope.spawn(move || {
                        crate::mpfr_test::cleanup_on_thread_exit();
                        encloses(&copy, p, &bounds);
                    });
                }
            });
            for p in [2, 0, -1] {
                encloses(&value, p, &bounds);
            }
        }
    }
}
