#[cfg(test)]
mod evaluation_failure_tests {
    use super::*;
    use crate::Real;
    use std::sync::atomic::{AtomicBool, Ordering};

    type Reducer = fn(Computable) -> Approximation;

    fn raw_node(kind: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(
                kind,
                BoundCache::Valid(BoundInfo::Unknown),
                ExactSignCache::Unknown,
            )),
            signal: None,
        }
    }

    #[test]
    fn explicit_signal_controls_every_deferred_reduction() {
        let reducers: [(&str, Reducer); 6] = [
            ("sin", Approximation::SinDeferred),
            ("cos", Approximation::CosDeferred),
            ("tan", Approximation::TanDeferred),
            ("ln", Approximation::LnDeferred),
            ("atan", Approximation::AtanDeferred),
            ("exp", Approximation::PrescaledExp),
        ];
        let operand = || raw_node(Approximation::Add(Computable::pi(), Computable::e()));
        let mut failures = Vec::new();
        for (name, reduce) in reducers {
            let reference = raw_node(reduce(operand())).approx(-64).unwrap();
            for explicit_signal in [None, Some(Arc::new(AtomicBool::new(false)))] {
                let mut input = operand();
                input.abort(Arc::new(AtomicBool::new(true)));
                let value = raw_node(reduce(input));
                let result = value.approx_signal(&explicit_signal, -64);
                // Independent successful reductions may choose adjacent integers.
                if !matches!(&result, Ok(sample) if (sample - &reference).magnitude() <= &BigUint::from(2_u8))
                {
                    failures.push(format!(
                        "{name}, explicit Some={}: {result:?}",
                        explicit_signal.is_some()
                    ));
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn every_observed_reduction_stop_reaches_the_root() {
        let reducers: [(&str, Reducer); 9] = [
            ("sin", Approximation::SinDeferred),
            ("cos", Approximation::CosDeferred),
            ("tan", Approximation::TanDeferred),
            ("ln", Approximation::LnDeferred),
            ("atan", Approximation::AtanDeferred),
            ("exp", Approximation::PrescaledExp),
            ("ln1p", Approximation::PrescaledLn),
            ("asinh", Approximation::AsinhDirect),
            ("acosh", Approximation::AcoshDirect),
        ];
        let mut failures = Vec::new();
        for (name, reduce) in reducers {
            let mut completed = false;
            for stop_after in 0..512 {
                let input = raw_node(Approximation::Add(Computable::pi(), Computable::e()));
                let value = raw_node(reduce(input));
                let signal = Arc::new(AtomicBool::new(false));
                CLEAR_STOP_AFTER_POLLS.with(|polls| polls.set(Some(stop_after)));
                let result = value.approx_signal(&Some(signal.clone()), -64);
                let observed = CLEAR_STOP_AFTER_POLLS.with(|polls| polls.take().is_none());
                assert!(!signal.load(Ordering::Relaxed));
                if !observed {
                    assert!(result.is_ok(), "{name}: {result:?}");
                    completed = true;
                    break;
                }
                if result != Err(Problem::Exhausted) || value.internal.cache_snapshot().is_some() {
                    failures.push(format!("{name}, poll {stop_after}: {result:?}"));
                }
                assert!(value.approx(-64).is_ok(), "{name} must remain retryable");
            }
            assert!(completed, "{name} did not finish within the poll sweep");
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn series_domain_probe_preserves_observed_cancellation() {
        let input = raw_node(Approximation::Add(
            Computable::rational(Rational::fraction(1, 16).unwrap()),
            Computable::zero(),
        ));
        let value = raw_node(Approximation::PrescaledAtanh(input));
        let signal = Arc::new(AtomicBool::new(false));
        // Root, dispatcher and kernel entry polls pass. The domain probe
        // observes cancellation, after which the external flag is cleared.
        CLEAR_STOP_AFTER_POLLS.with(|polls| polls.set(Some(3)));
        let result = value.approx_signal(&Some(signal.clone()), 1);
        assert!(CLEAR_STOP_AFTER_POLLS.with(|polls| polls.get().is_none()));
        assert!(!signal.load(Ordering::Relaxed));
        assert_eq!(result, Err(Problem::Exhausted));
        assert!(value.internal.cache_snapshot().is_none());
        assert_eq!(value.approx(1), Ok(BigInt::zero()));
    }

    #[test]
    fn observed_cancellation_survives_a_cleared_external_flag() {
        let value = Computable::prescaled_atan(BigInt::from(5));
        let signal = Arc::new(AtomicBool::new(false));
        // Two entry polls pass; the series observes cancellation on its first
        // poll, after which the caller's flag has already been cleared.
        CLEAR_STOP_AFTER_POLLS.with(|polls| polls.set(Some(2)));
        assert_eq!(
            value.approx_signal(&Some(signal.clone()), -128),
            Err(Problem::Exhausted)
        );
        assert!(!signal.load(Ordering::Relaxed));
        assert!(value.internal.cache_snapshot().is_none());
        let retry = value.approx(-128).unwrap();
        // Alternating atan series: 1/5 - 1/(3*5^3) < atan(1/5) < 1/5.
        let scale = BigInt::one() << 128;
        assert!((&retry + 1) * 375 > &scale * 74);
        assert!((&retry - 1) * 5 < scale);
    }

    #[test]
    fn extreme_precision_cannot_poison_the_shared_pi_cache() {
        for warm in [false, true] {
            let pi = Computable::pi();
            if warm {
                pi.approx(-256).unwrap();
            }
            assert_eq!(pi.approx(Precision::MIN), Err(Problem::OutOfRange));
            for precision in [Precision::MIN + 1, Precision::MIN + 2, Precision::MIN + 16] {
                assert_eq!(pi.approx(precision), Err(Problem::Exhausted));
            }
            for precision in [0, -64, -256] {
                let approximation = pi.approx(precision).unwrap();
                let scale = BigInt::one() << precision.unsigned_abs();
                assert!(&approximation + 1 > &scale * 3);
                assert!(&approximation - 1 < &scale * 4);
            }
            let [lower, upper] = Real::pi().certified_rational_interval(-64).unwrap();
            assert!(lower > Rational::new(3));
            assert!(upper < Rational::new(4));
        }
    }

    #[test]
    fn adjacent_precision_failures_remain_retryable_for_nonconstant_nodes() {
        let cosine = Computable::one().cos();
        let product = Computable::pi().multiply(Computable::e());
        for value in [&cosine, &product] {
            for precision in [Precision::MIN + 1, Precision::MIN + 2, Precision::MIN + 16] {
                assert_eq!(value.approx(precision), Err(Problem::Exhausted));
                assert!(value.internal.cache_snapshot().is_none());
            }
            assert!(value.approx(-64).unwrap() > BigInt::one());
        }
    }

    #[test]
    fn exact_zero_and_coarse_requests_do_not_need_large_allocations() {
        assert_eq!(
            Computable::zero().approx(Precision::MIN),
            Ok(BigInt::zero())
        );
        for value in [Computable::one(), Computable::pi(), Computable::e()] {
            assert_eq!(value.approx(Precision::MAX), Ok(BigInt::zero()));
        }
        let square = Approximation::Square(Computable::zero());
        assert_eq!(square.approximate(&None, -64), Ok(BigInt::zero()));
    }

    #[test]
    fn negative_square_roots_do_not_publish_positive_facts() {
        for radicand in [
            Computable::rational(Rational::new(-1)),
            Computable::rational(Rational::new(-4)),
            Computable::pi().negate(),
        ] {
            let root = radicand.sqrt();
            assert_eq!(root.exact_sign(), None);
            assert_eq!(root.zero_status(), ZeroKnowledge::Unknown);
            for precision in [0, -64] {
                assert_eq!(root.approx(precision), Err(Problem::SqrtNegative));
                assert!(root.internal.cache_snapshot().is_none());
                assert_eq!(root.exact_sign(), None);
            }
        }
    }

    #[test]
    fn square_root_rejects_negative_radicands_discovered_during_refinement() {
        // The alternating sine series proves sin(1/2) < 1/2. This expression
        // has no retained structural sign. Magnitude refinement must not send
        // a negative integer sample to BigInt::sqrt.
        for precision in [-16, -64, -256] {
            let half = Computable::rational(Rational::fraction(1, 2).unwrap());
            let radicand = half.clone().sin().add(half.negate());
            assert_eq!(radicand.exact_sign(), None);
            let root = radicand.sqrt();
            assert_eq!(root.approx(precision), Err(Problem::SqrtNegative));
            assert!(root.internal.cache_snapshot().is_none());
        }
    }

    #[test]
    fn square_root_accepts_every_valid_zero_cache_endpoint() {
        for precision in [0, -16, -64] {
            for sample in [-1, 0, 1] {
                // An opaque exact zero with each allowed one-unit rounding.
                // Retain Unknown facts so the kernel must inspect the sample.
                let input = Computable {
                    internal: Arc::new(Node::new(
                        Approximation::Add(Computable::zero(), Computable::zero()),
                        BoundCache::Valid(BoundInfo::Unknown),
                        ExactSignCache::Unknown,
                    )),
                    signal: None,
                };
                input
                    .internal
                    .store_cache_value(2 * precision - 1, BigInt::from(sample));
                let root = Approximation::Sqrt(input);
                assert_eq!(root.approximate(&None, precision), Ok(BigInt::zero()));
            }
        }
    }
}
