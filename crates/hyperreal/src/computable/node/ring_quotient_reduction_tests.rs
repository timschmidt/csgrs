#[cfg(test)]
mod ring_quotient_reduction_tests {
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

    fn factored_quotient(negative: bool) -> (Computable, Computable, Computable) {
        let a = raw(Approximation::Add(
            Computable::rational(Rational::from(4)),
            raw(Approximation::PrescaledSin(Computable::rational(
                Rational::fraction(1, 3).unwrap(),
            ))),
        ));
        let b = raw(Approximation::Add(
            Computable::one(),
            raw(Approximation::PrescaledSin(Computable::rational(
                Rational::fraction(1, 5).unwrap(),
            ))),
        ));
        let mut numerator = raw(Approximation::Add(
            raw(Approximation::Square(a.clone())),
            raw(Approximation::Negate(raw(Approximation::Square(b.clone())))),
        ));
        let mut denominator = raw(Approximation::Add(
            a.clone(),
            raw(Approximation::Negate(b.clone())),
        ));
        if negative {
            numerator = raw(Approximation::Negate(numerator));
            denominator = raw(Approximation::Negate(denominator));
        }
        let quotient = raw(Approximation::Multiply(
            numerator,
            raw(Approximation::Inverse(denominator.clone())),
        ));
        (quotient, raw(Approximation::Add(a, b)), denominator)
    }

    fn repeated_square(mut value: Computable, count: usize) -> Computable {
        for _ in 0..count {
            value = raw(Approximation::Square(value));
        }
        value
    }

    fn identity(quotient: Computable, polynomial: Computable) -> Computable {
        // ((a^2-b^2)/(a-b))^8 - (a+b)^8 = 0, provided a-b != 0.
        // The reduced degree is 8; unreduced numerators reach degree 16.
        raw(Approximation::Add(
            repeated_square(quotient, 3),
            raw(Approximation::Negate(repeated_square(polynomial, 3))),
        ))
    }

    fn assert_unproved(value: &Computable) {
        assert!(matches!(
            value.internal.facts.exact_sign(),
            ExactSignCache::Invalid
        ));
        assert!(value.cached().is_none());
    }

    #[test]
    fn exact_polynomial_quotients_survive_repeated_squaring() {
        for negative in [false, true] {
            let (quotient, polynomial, denominator) = factored_quotient(negative);
            assert_eq!(
                denominator.sign_until(-64),
                Some(if negative {
                    RealSign::Negative
                } else {
                    RealSign::Positive
                })
            );
            let value = identity(quotient, polynomial);
            assert_eq!(value.bounded_quotient_rational(), Some(Rational::zero()));
            assert!(value.cached().is_none());
        }
    }

    #[test]
    fn factor_reduction_requires_the_retained_nonzero_witness() {
        let (quotient, polynomial, denominator) = factored_quotient(false);
        let value = identity(quotient, polynomial);
        assert_eq!(denominator.immediate_sign(), None);
        assert_eq!(value.bounded_quotient_rational(), None);
        assert_unproved(&value);
        assert_eq!(denominator.sign_until(-64), Some(RealSign::Positive));
        assert_eq!(value.bounded_quotient_rational(), Some(Rational::zero()));
    }

    #[test]
    fn factor_reduction_keeps_degree_limits_and_cached_depth_checks() {
        let (quotient, _, denominator) = factored_quotient(false);
        assert_eq!(denominator.sign_until(-64), Some(RealSign::Positive));
        let fourth = repeated_square(quotient.clone(), 2);
        let eighth = raw(Approximation::Square(fourth.clone()));
        let mut recognition = RingRecognition::new(&eighth);
        assert!(
            recognition
                .expand_quotient(&fourth, RingRecognition::MAX_DEPTH)
                .is_some()
        );
        // Cached children must not let a recovery bypass a zero child depth.
        assert!(recognition.expand_quotient(&eighth, 1).is_none());
        assert_unproved(&eighth);
        let sixteenth = repeated_square(quotient, 4);
        assert!(
            RingRecognition::new(&sixteenth)
                .expand_quotient(&sixteenth, RingRecognition::MAX_DEPTH)
                .is_none()
        );
        assert_unproved(&sixteenth);
    }

    #[test]
    fn factor_reduction_preserves_exhaustion_cancellation_and_recovery() {
        let (quotient, polynomial, denominator) = factored_quotient(false);
        assert_eq!(denominator.sign_until(-64), Some(RealSign::Positive));
        let mut value = identity(quotient, polynomial);
        let mut limited = RingRecognition::new(&value);
        limited.operations_left = 8;
        assert!(
            limited
                .expand_quotient(&value, RingRecognition::MAX_DEPTH)
                .is_none()
        );
        assert_unproved(&value);
        drop(limited);
        let signal = Arc::new(AtomicBool::new(true));
        value.abort(signal.clone());
        assert_eq!(value.bounded_quotient_rational(), None);
        assert_unproved(&value);
        signal.store(false, std::sync::atomic::Ordering::Relaxed);
        assert_eq!(value.bounded_quotient_rational(), Some(Rational::zero()));
        assert!(value.cached().is_none());
    }
}
