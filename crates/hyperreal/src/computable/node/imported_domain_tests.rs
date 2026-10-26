#[cfg(all(test, feature = "serde"))]
mod imported_domain_tests {
    use super::*;

    fn raw(approximation: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(
                approximation,
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    fn affine(q: Rational, a: Rational) -> Computable {
        raw(Approximation::Add(
            Computable::rational(q),
            raw(Approximation::Multiply(
                Computable::rational(a),
                Computable::pi(),
            )),
        ))
    }

    #[test]
    fn rational_affine_pi_images_replay_nonzero_without_a_sign_claim() {
        for q in [
            Rational::fraction(-3, 5).unwrap(),
            Rational::fraction(3, 5).unwrap(),
        ] {
            for a in [
                Rational::fraction(-7, 3).unwrap(),
                Rational::zero(),
                Rational::fraction(5, 2).unwrap(),
            ] {
                let argument = affine(q.clone(), a);
                for approximation in [
                    Approximation::PrescaledSin(argument.clone()),
                    Approximation::PrescaledCos(argument.clone()),
                ] {
                    let image = raw(approximation);
                    for wrapped in [
                        image.clone(),
                        raw(Approximation::Negate(image.clone())),
                        raw(Approximation::Offset(image.clone(), i32::MIN)),
                        raw(Approximation::Offset(image.clone(), i32::MAX)),
                    ] {
                        for _ in 0..2 {
                            assert!(wrapped.imported_affine_pi_trig_nonzero());
                            assert!(wrapped.cached().is_none());
                            assert!(matches!(
                                wrapped.internal.facts.exact_sign(),
                                ExactSignCache::Invalid
                            ));
                        }
                    }
                    assert!(image.cached().is_none());
                }
                assert!(argument.cached().is_none());
            }
        }
    }

    #[test]
    fn affine_pi_replay_declines_poles_and_unsupported_coefficients() {
        let pi = Computable::pi();
        for argument in [
            affine(Rational::zero(), Rational::one()),
            affine(Rational::zero(), Rational::fraction(1, 2).unwrap()),
            affine(Rational::zero(), Rational::zero()),
            raw(Approximation::Add(
                Computable::one(),
                raw(Approximation::Square(pi.clone())),
            )),
            raw(Approximation::Add(
                raw(Approximation::Sqrt(Computable::rational(Rational::new(2)))),
                pi.clone(),
            )),
            raw(Approximation::Add(
                raw(Approximation::PrescaledSin(Computable::one())),
                pi.clone(),
            )),
            raw(Approximation::Add(
                raw(Approximation::Multiply(
                    Computable::zero(),
                    Computable::one(),
                )),
                pi,
            )),
        ] {
            for approximation in [
                Approximation::PrescaledSin(argument.clone()),
                Approximation::PrescaledCos(argument.clone()),
            ] {
                let image = raw(approximation);
                assert!(!image.imported_affine_pi_trig_nonzero());
                assert!(image.cached().is_none());
                assert!(matches!(
                    image.internal.facts.exact_sign(),
                    ExactSignCache::Invalid
                ));
            }
        }
    }

    #[test]
    fn affine_pi_replay_limits_decline_without_poisoning_fresh_attempts() {
        let image = raw(Approximation::PrescaledSin(affine(
            Rational::one(),
            Rational::one(),
        )));
        let mut wrapped = image.clone();
        for _ in 0..47 {
            wrapped = raw(Approximation::Negate(wrapped));
        }
        assert!(wrapped.imported_affine_pi_trig_nonzero());
        wrapped = raw(Approximation::Negate(wrapped));
        assert!(!wrapped.imported_affine_pi_trig_nonzero());
        assert!(wrapped.cached().is_none());
        assert!(image.imported_affine_pi_trig_nonzero());

        fn wide(depth: usize) -> Computable {
            if depth == 0 {
                affine(Rational::one(), Rational::one())
            } else {
                raw(Approximation::Add(wide(depth - 1), wide(depth - 1)))
            }
        }
        let oversized = raw(Approximation::PrescaledSin(wide(8)));
        assert!(!oversized.imported_affine_pi_trig_nonzero());
        assert!(oversized.cached().is_none());
        assert!(image.imported_affine_pi_trig_nonzero());
    }

    #[test]
    fn affine_pi_replay_handles_extreme_argument_shifts_without_expansion() {
        for shift in [i32::MIN, i32::MAX] {
            let argument = raw(Approximation::Offset(
                affine(Rational::one(), Rational::one()),
                shift,
            ));
            let image = raw(Approximation::PrescaledSin(argument));
            // 2^shift * (1 + pi) is a rational affine pi expression with a
            // nonzero rational part, for every finite integer shift. Proving
            // that fact must not materialize the power-of-two coefficient.
            assert!(image.imported_affine_pi_trig_nonzero());
            assert!(image.cached().is_none());
        }
    }

    #[test]
    fn small_trig_signs_keep_extreme_shifts_symbolic() {
        for shift in [i32::MIN, i32::MIN + 1, -16_384, -3] {
            for negative in [false, true] {
                let argument = raw(Approximation::Offset(
                    affine(Rational::one(), Rational::one()),
                    shift,
                ));
                let argument = if negative {
                    raw(Approximation::Negate(argument))
                } else {
                    argument
                };
                // 3 < pi < 4 proves 0 < (1+pi)/8 < 1. Every
                // further negative binary shift stays in that interval.
                let sign = if negative { Sign::Minus } else { Sign::Plus };
                for (approximation, expected) in [
                    (Approximation::PrescaledSin(argument.clone()), sign),
                    (Approximation::PrescaledTan(argument.clone()), sign),
                    (Approximation::PrescaledCot(argument.clone()), sign),
                    (Approximation::PrescaledCos(argument.clone()), Sign::Plus),
                ] {
                    let image = raw(approximation);
                    assert_eq!(image.exact_sign(), Some(expected));
                    assert_eq!(image.exact_sign(), Some(expected));
                    assert!(image.cached().is_none());
                    assert!(argument.cached().is_none());
                }
            }
        }
    }

    #[test]
    fn small_trig_range_proof_declines_without_sign_or_approximation_facts() {
        let base = affine(Rational::one(), Rational::one());
        let outside = raw(Approximation::Offset(base.clone(), -2));
        let huge = raw(Approximation::Offset(base.clone(), i32::MAX));
        let ambiguous = raw(Approximation::Add(
            Computable::pi(),
            raw(Approximation::Negate(Computable::pi())),
        ));
        let mut deep = raw(Approximation::Offset(base.clone(), -3));
        for _ in 0..32 {
            deep = raw(Approximation::Negate(deep));
        }
        for value in [outside, huge, ambiguous, deep] {
            assert_eq!(value.small_trig_argument_sign(), None);
            assert!(value.cached().is_none());
            assert_eq!(value.internal.facts.exact_sign(), ExactSignCache::Invalid);
        }
        let shifted = raw(Approximation::Offset(
            raw(Approximation::Offset(
                raw(Approximation::Offset(base, -3)),
                i32::MAX,
            )),
            i32::MIN + 1,
        ));
        // The two extreme shifts cancel exactly; no intermediate exponent
        // truncation or floating-point scaling is permitted.
        assert_eq!(shifted.small_trig_argument_sign(), Some(Sign::Plus));
        assert!(shifted.cached().is_none());
        let mut aborted = shifted;
        let signal = Arc::new(std::sync::atomic::AtomicBool::new(true));
        aborted.abort(signal.clone());
        assert_eq!(aborted.small_trig_argument_sign(), None);
        signal.store(false, std::sync::atomic::Ordering::Relaxed);
        assert_eq!(aborted.small_trig_argument_sign(), Some(Sign::Plus));
    }

    #[test]
    fn small_trig_range_matches_integer_inequalities_at_unit_boundaries() {
        for n in -64_i64..=64 {
            for shift in -10_i32..=10 {
                let value = raw(Approximation::Offset(
                    raw(Approximation::Int(BigInt::from(n))),
                    shift,
                ));
                let within_unit = if shift >= 0 {
                    n.unsigned_abs() * (1_u64 << shift) <= 1
                } else {
                    n.unsigned_abs() <= 1_u64 << shift.unsigned_abs()
                };
                let expected = within_unit.then(|| BigInt::from(n).sign());
                assert_eq!(
                    value.small_trig_argument_sign(),
                    expected,
                    "{n} * 2^{shift}"
                );
                assert!(value.cached().is_none());
            }
        }
        for shift in [
            i32::MIN,
            -1075,
            -1074,
            -1024,
            -1023,
            1023,
            1074,
            1075,
            i32::MAX,
        ] {
            for n in [-1, 0, 1] {
                let value = raw(Approximation::Offset(
                    raw(Approximation::Int(BigInt::from(n))),
                    shift,
                ));
                let expected = (n == 0 || shift < 0).then(|| BigInt::from(n).sign());
                assert_eq!(
                    value.small_trig_argument_sign(),
                    expected,
                    "{n} * 2^{shift}"
                );
            }
        }
    }

    #[test]
    fn affine_pi_argument_work_and_cancellation_are_shared_and_retryable() {
        let mut argument = affine(Rational::one(), Rational::one());
        // The affine form uses five visits. Wrappers consume the same 48-visit
        // argument allowance, including unsuccessful alternative branches.
        for _ in 0..43 {
            argument = raw(Approximation::Negate(argument));
        }
        let mut image = raw(Approximation::PrescaledSin(argument.clone()));
        assert!(image.imported_affine_pi_trig_nonzero());
        let too_deep = raw(Approximation::PrescaledSin(raw(Approximation::Negate(
            argument,
        ))));
        assert!(!too_deep.imported_affine_pi_trig_nonzero());
        assert!(too_deep.cached().is_none());
        let stopped = Arc::new(AtomicBool::new(true));
        image.abort(stopped.clone());
        assert!(!image.imported_affine_pi_trig_nonzero());
        assert!(image.cached().is_none());
        stopped.store(false, std::sync::atomic::Ordering::Relaxed);
        assert!(image.imported_affine_pi_trig_nonzero());
        assert!(image.cached().is_none());
        assert!(matches!(
            image.internal.facts.exact_sign(),
            ExactSignCache::Invalid
        ));
    }
}
