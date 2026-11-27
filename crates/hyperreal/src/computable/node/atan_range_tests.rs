/// Qualify imported generic atan series nodes independently of constructor range hints.
/// Directed MPFR bounds cover rational and symbolic children, half-angle reduction,
/// admissible boundary samples, cache reuse and entry cancellation. Fixtures register
/// per-thread MPFR cleanup; finite cases do not prove the full real/precision domain.
#[cfg(test)]
mod atan_range_tests {
    use super::*;
    use rug::{Float, Integer, Rational as Q, float::Round};

    /// Build a node with no initial bound, exact-sign evidence or abort signal, bypassing constructors.
    /// The supplied tag is authoritative input to the kernel test, not a range/domain certificate.
    /// Callers must choose defined fixture operands; this helper performs no validation.
    fn raw(kind: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(
                kind,
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    /// Enclose principal atan(q) by outward rounding a finite rational at 2048 bits.
    /// Monotonicity permits downward evaluation at the lower input and upward evaluation
    /// at the upper. Return exact rational endpoints; nonfinite conversion panics.
    fn oracle(q: &Q) -> [Q; 2] {
        let mut lo = Float::with_val_round(2048, q, Round::Down).0;
        let mut hi = Float::with_val_round(2048, q, Round::Up).0;
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

    /// Check 25 signed rational inputs through PrescaledAtan with direct and wrapped children.
    /// A raw Add(q,0) must lack exact_rational metadata, selecting generic handling while
    /// preserving the same value. Nine precision requests check cold and reused enclosures,
    /// including near-half, huge and tiny arguments; no dispatch-count equality is asserted.
    #[test]
    fn imported_atan_series_preserve_rational_and_generic_arguments() {
        crate::mpfr_test::cleanup_on_thread_exit();
        let mut inputs = vec![Q::from(0)];
        for q in [
            Q::from((1, 16)),
            Q::from((1, 3)),
            Q::from((127, 256)),
            Q::from((1, 2)),
            Q::from((129, 256)),
            Q::from((3, 4)),
            Q::from(1),
            Q::from(2),
            Q::from(4),
            Q::from(1000),
            Q::from(Integer::from(1) << 512),
            Q::from((1, Integer::from(1) << 512)),
        ] {
            inputs.extend([q.clone(), -q]);
        }
        for q in inputs {
            let bounds = oracle(&q);
            let rational: Rational = q.to_string().parse().unwrap();
            for generic in [false, true] {
                let build = || {
                    let leaf = Computable::rational(rational.clone());
                    let child = if generic {
                        let child = raw(Approximation::Add(leaf, Computable::zero()));
                        assert!(child.exact_rational().is_none());
                        child
                    } else {
                        leaf
                    };
                    raw(Approximation::PrescaledAtan(child))
                };
                let warm = build();
                for p in [2, 0, -1, -32, -128, -512, -8, 2, 0] {
                    encloses(&build(), p, &bounds);
                    encloses(&warm, p, &bounds);
                }
            }
        }
    }

    /// Check both signs of sqrt(n)*2^shift for n=2,5,17 and four shifts from -32 to 32.
    /// Outward MPFR sqrt followed by exact binary scaling and outward atan supplies the
    /// independent bounds; negate by swapping endpoints. Require nonrational child metadata
    /// and cold/reused enclosures at seven requests, without asserting an internal trace.
    #[test]
    fn atan_half_angle_fallback_encloses_symbolic_children() {
        crate::mpfr_test::cleanup_on_thread_exit();
        for n in [2, 5, 17] {
            for shift in [-32, -2, 0, 32] {
                let mut lo = Float::with_val(2048, n);
                let mut hi = lo.clone();
                lo.sqrt_round(Round::Down);
                hi.sqrt_round(Round::Up);
                lo <<= shift;
                hi <<= shift;
                lo.atan_round(Round::Down);
                hi.atan_round(Round::Up);
                let bounds = [lo.to_rational().unwrap(), hi.to_rational().unwrap()];
                for negative in [false, true] {
                    let bounds = if negative {
                        [-bounds[1].clone(), -bounds[0].clone()]
                    } else {
                        bounds.clone()
                    };
                    let build = || {
                        let child = Computable::rational(Rational::new(n))
                            .sqrt()
                            .shift_left(shift);
                        let child = if negative { child.negate() } else { child };
                        assert!(child.exact_rational().is_none());
                        raw(Approximation::PrescaledAtan(child))
                    };
                    let warm = build();
                    for p in [2, 0, -32, -128, -512, -8, 2] {
                        encloses(&build(), p, &bounds);
                        encloses(&warm, p, &bounds);
                    }
                }
            }
        }
    }

    /// Inject all admissible integer samples for selected signed inputs near the half-unit boundary.
    /// Five request/working-precision pairs and quarter-unit perturbations yield 390
    /// samples within the one-unit contract. Require a generic Add child, then check
    /// each result at the original request, -1024 and the original request again,
    /// for 1170 enclosures. Exhaustiveness is over this finite sample grid, not all inputs.
    #[test]
    fn atan_range_proof_handles_every_allowed_working_sample_rounding() {
        crate::mpfr_test::cleanup_on_thread_exit();
        let mut checks = 0;
        // Actual working precisions of the original recurrence. The injected
        // sample is admitted only when it satisfies the public +/-1 contract.
        for (p, s) in [
            (0_i32, -12_i32),
            (-1, -13),
            (-32, -46),
            (-128, -144),
            (-512, -530),
        ] {
            let threshold = Integer::from(1) << (-s - 1);
            for delta in -8_i32..=8 {
                for negative in [false, true] {
                    let q = Q::from((threshold.clone() * 4 + delta, Integer::from(1) << (-s + 2)));
                    let q = if negative { -q } else { q };
                    let base = threshold.clone() + delta.div_euclid(4);
                    let base = if negative { -base } else { base };
                    let bounds = oracle(&q);
                    for rounding in -1..=1 {
                        let sample: Integer = base.clone() + rounding;
                        let unit = Q::from((1, Integer::from(1) << -s));
                        if (Q::from(sample.clone()) * &unit - &q).abs() > unit {
                            continue;
                        }
                        let child = raw(Approximation::Add(
                            Computable::rational(q.to_string().parse().unwrap()),
                            Computable::zero(),
                        ));
                        assert!(child.exact_rational().is_none());
                        child
                            .internal
                            .store_cache_value(s, sample.to_string().parse().unwrap());
                        let value = raw(Approximation::PrescaledAtan(child));
                        for precision in [p, -1024, p] {
                            encloses(&value, precision, &bounds);
                            checks += 1;
                        }
                    }
                }
            }
        }
        assert_eq!(checks, 1170);
    }

    /// Check entry cancellation on generic atan children of value ±4 leaves the root uncached.
    /// Clear the shared flag, retry at -128, compare with the subsequent warm call and
    /// require an MPFR enclosure. The abort placeholder is discarded; child-cache state
    /// and cancellation at every reduction stage are not checked.
    #[test]
    fn aborted_atan_range_reduction_recovers_without_publishing_a_cache() {
        use std::sync::atomic::{AtomicBool, Ordering};
        crate::mpfr_test::cleanup_on_thread_exit();
        for n in [-4, 4] {
            let child = raw(Approximation::Add(
                Computable::rational(Rational::new(n)),
                Computable::zero(),
            ));
            let value = raw(Approximation::PrescaledAtan(child));
            let signal = Arc::new(AtomicBool::new(true));
            let _ = value.approx_signal(&Some(signal.clone()), -128);
            assert!(value.cached().is_none());
            signal.store(false, Ordering::Relaxed);
            let resumed = value.approx_signal(&Some(signal), -128);
            assert_eq!(resumed, value.approx(-128));
            encloses(&value, -128, &oracle(&Q::from(n)));
        }
    }
}
