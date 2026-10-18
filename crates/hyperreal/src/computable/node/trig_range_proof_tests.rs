#[cfg(all(test, feature = "serde"))]
mod trig_range_proof_tests {
    use super::*;
    use serde_json::{Value, json};

    fn raw_trig(kind: &str, argument: Value) -> Computable {
        serde_json::from_value(json!({"internal": {kind: argument}})).unwrap()
    }

    fn pi_multiple(q: Rational) -> Value {
        serde_json::to_value(Computable::pi().multiply(Computable::rational(q))).unwrap()
    }

    fn assert_unwarmed_sign(kind: &str, argument: Value, expected: RealSign) {
        let value = raw_trig(kind, argument);
        let wire = serde_json::to_value(&value).unwrap();
        assert!(value.cached().is_none());
        assert_eq!(
            value.immediate_sign(),
            Some(expected),
            "proof replay on import"
        );
        for _ in 0..2 {
            assert_eq!(
                value.structural_facts().sign,
                Some(expected),
                "{kind}: {wire}"
            );
            assert!(
                value.cached().is_none(),
                "sign inspection must not approximate"
            );
        }
        assert_eq!(serde_json::to_value(value).unwrap(), wire);
    }

    #[test]
    fn reduced_pi_graphs_prove_sign_without_approximation_after_import() {
        // Sine and tangent have the sign of q for -1/2 < q < 1/2.
        // Reimported graphs have no serialized cache facts or Real class tag.
        for kind in ["PrescaledSin", "PrescaledTan"] {
            for (q, sign) in [
                (Rational::fraction(1, 5).unwrap(), RealSign::Positive),
                (Rational::fraction(-1, 5).unwrap(), RealSign::Negative),
                (Rational::fraction(49, 100).unwrap(), RealSign::Positive),
                (Rational::fraction(-49, 100).unwrap(), RealSign::Negative),
            ] {
                assert_unwarmed_sign(kind, pi_multiple(q), sign);
            }
            // The shifted graph denotes pi*2^(-160); prove sign without building
            // or refining a number whose absolute size is used as a tolerance.
            let pi = serde_json::to_value(Computable::pi()).unwrap();
            let tiny = json!({"internal": {"Offset": [pi, -160]}});
            assert_unwarmed_sign(kind, tiny, RealSign::Positive);
        }
    }

    #[test]
    fn range_proofs_do_not_certify_poles_or_wrong_quadrants() {
        for kind in ["PrescaledSin", "PrescaledTan"] {
            for (q, sine, tangent) in [
                (Rational::zero(), Some(RealSign::Zero), Some(RealSign::Zero)),
                (
                    Rational::fraction(1, 2).unwrap(),
                    Some(RealSign::Positive),
                    None,
                ),
                (
                    Rational::fraction(3, 4).unwrap(),
                    Some(RealSign::Positive),
                    Some(RealSign::Negative),
                ),
                (Rational::one(), Some(RealSign::Zero), Some(RealSign::Zero)),
                (
                    Rational::fraction(3, 2).unwrap(),
                    Some(RealSign::Negative),
                    None,
                ),
            ] {
                let value = raw_trig(kind, pi_multiple(q));
                let expected = if kind == "PrescaledSin" {
                    sine
                } else {
                    tangent
                };
                if let Some(sign) = value.structural_facts().sign {
                    assert_eq!(Some(sign), expected, "{kind}");
                }
                assert!(value.cached().is_none());
            }
        }
    }

    #[test]
    fn extreme_shifts_and_unsupported_graphs_do_not_expand_coefficients() {
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
        let tiny = raw(Approximation::Offset(Computable::pi(), i32::MIN));
        let negative_tiny = raw(Approximation::Negate(tiny.clone()));
        for (argument, sign) in [(tiny, Sign::Plus), (negative_tiny, Sign::Minus)] {
            for approximation in [
                Approximation::PrescaledSin(argument.clone()),
                Approximation::PrescaledTan(argument.clone()),
            ] {
                let value = raw(approximation);
                assert_eq!(value.exact_sign(), Some(sign));
                assert!(value.cached().is_none());
                assert!(argument.cached().is_none());
            }
        }

        let mut deep = raw(Approximation::Offset(Computable::pi(), -160));
        for _ in 0..33 {
            deep = raw(Approximation::Negate(deep));
        }
        let overflowing_shift = raw(Approximation::Offset(
            raw(Approximation::Offset(Computable::pi(), i32::MAX)),
            2,
        ));
        // A matching-looking pi factor cannot cancel an undefined operand.
        let undefined = raw(Approximation::Multiply(
            raw(Approximation::Inverse(Computable::zero())),
            Computable::pi(),
        ));
        for argument in [deep, overflowing_shift, undefined] {
            for approximation in [
                Approximation::PrescaledSin(argument.clone()),
                Approximation::PrescaledTan(argument.clone()),
            ] {
                let value = raw(approximation);
                assert_eq!(value.exact_sign(), None);
                assert!(value.cached().is_none());
                assert!(argument.cached().is_none());
            }
        }
    }
}
