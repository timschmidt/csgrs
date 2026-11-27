/// Qualify acos of negative rational arguments, including its stored nonnegative magnitude.
/// Directed MPFR endpoints check public construction and raw complement tags across
/// precision reuse, cancellation and concurrent access. Finite schedules do not prove
/// race freedom or total resource bounds; MPFR cleanup is registered on each using thread.
#[cfg(test)]
mod acos_complement_range_tests {
    use super::*;
    use rug::{Float, Integer, Rational as Q, float::Round};

    /// Build AcosNegativeRational(r) with invalid initial bound/sign evidence and no signal.
    /// The payload r is the nonnegative magnitude, so the represented angle is acos(-r).
    /// Fixtures must supply 0<=r<=1; parsing is exact, but this helper does not validate the domain.
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

    /// Enclose acos(-r) at 2048 bits for fixture magnitudes 0<=r<=1.
    /// Because acos decreases, round -r upward for the lower output and downward for the upper;
    /// then round the function results outward and convert them to exact rationals.
    /// Out-of-domain/nonfinite conversion would panic rather than return a typed failure.
    fn oracle(r: &Q) -> [Q; 2] {
        let x = -r.clone();
        // acos is decreasing: the upper input bound supplies the lower output.
        let mut lo = Float::with_val_round(2048, &x, Round::Up).0;
        let mut hi = Float::with_val_round(2048, &x, Round::Down).0;
        lo.acos_round(Round::Down);
        hi.acos_round(Round::Up);
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

    /// Check 12 magnitudes through raw complement tags and public acos(-r) construction.
    /// Include both endpoints, tiny inputs, and distances 2^-64 and 2^-512 from one.
    /// Thirteen precision requests exercise cold and reused nodes against independent
    /// MPFR bounds; public endpoint simplification need not retain the imported tag.
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

    /// Check four raw complement nodes leave no numerical cache when cancelled on entry.
    /// Clear the shared signal, retry at -128, compare with the now-warm ordinary call,
    /// and require an MPFR enclosure. The discarded abort integer is not certified output;
    /// this tests entry cancellation, not interruption at every internal stage.
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

    /// Check six scoped readers sharing each of two raw complement nodes at mixed precisions.
    /// Each worker requires its own MPFR enclosure and registers cleanup; three further
    /// requests check the shared node after joining. This finite schedule is not a race-freedom proof.
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
