#[cfg(test)]
mod exp_relation_tests {
    use super::*;
    use num::BigRational;

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
    fn difference(a: Computable, b: Computable) -> Computable {
        raw(Approximation::Add(a, raw(Approximation::Negate(b))))
    }
    fn from_q(q: &BigRational) -> Computable {
        Computable::rational(
            Rational::from_bigint_fraction(q.numer().clone(), q.denom().to_biguint().unwrap())
                .unwrap(),
        )
    }
    fn q(n: i64, d: i64) -> BigRational {
        BigRational::new(n.into(), d.into())
    }

    #[test]
    fn independent_rational_log_coefficients_and_unequal_controls() {
        let mut seed = 15793u64;
        for case in 0..512 {
            let mut next = || {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                seed >> 32
            };
            let atom = || Computable::rational(Rational::fraction(3, 8).unwrap()).sin();
            let mut weight = q(1, 1);
            let mut constant = q((next() % 17) as i64 - 8, 8);
            let mut binary = q(0, 1);
            let exponent = atom().add(from_q(&constant));
            let mut value = raw(Approximation::PrescaledExp(exponent));
            for _ in 0..5 {
                match next() % 4 {
                    0 => {
                        let shift = (next() % 7) as i32 - 3;
                        value = raw(Approximation::Offset(value, shift));
                        binary += q(i64::from(shift), 1);
                    }
                    1 => {
                        value = raw(Approximation::Sqrt(value));
                        weight /= q(2, 1);
                        constant /= q(2, 1);
                        binary /= q(2, 1);
                    }
                    2 => {
                        value = raw(Approximation::Square(value));
                        weight *= q(2, 1);
                        constant *= q(2, 1);
                        binary *= q(2, 1);
                    }
                    _ => {
                        value = raw(Approximation::Inverse(value));
                        weight = -weight;
                        constant = -constant;
                        binary = -binary;
                    }
                }
            }
            let rhs_argument = atom()
                .multiply(from_q(&weight))
                .add(from_q(&constant))
                .add(Computable::ln2().multiply(from_q(&binary)));
            let rhs = raw(Approximation::PrescaledExp(rhs_argument.clone()));
            let zero = difference(value.clone(), rhs);
            assert!(zero.exact_positive_exp_difference_zero(), "case={case}");
            assert_eq!(zero.sign_until(-64), Some(RealSign::Zero), "case={case}");
            let unequal = raw(Approximation::PrescaledExp(
                rhs_argument.add(from_q(&q(1, 128))),
            ));
            let negative = difference(value, unequal);
            assert!(
                !negative.exact_positive_exp_difference_zero(),
                "case={case}"
            );
            assert_eq!(
                negative.sign_until(-256),
                Some(RealSign::Negative),
                "case={case}"
            );
        }
    }

    #[test]
    fn products_and_inverse_roots_require_positive_exponential_leaves() {
        let x = Computable::rational(Rational::new(2)).sqrt();
        let y = Computable::rational(Rational::new(3)).sqrt();
        let left = raw(Approximation::Sqrt(raw(Approximation::Multiply(
            x.clone().exp(),
            y.clone().exp(),
        ))));
        let right = (x.clone().add(y).shift_right(1)).exp();
        assert_eq!(
            difference(left, right).sign_until(-64),
            Some(RealSign::Zero)
        );
        let bad = raw(Approximation::Sqrt(Computable::rational(Rational::new(-1))));
        assert!(!difference(bad, x.exp()).exact_positive_exp_difference_zero());
    }

    #[test]
    fn cached_failure_does_not_prevent_later_separation() {
        let x = Computable::rational(Rational::fraction(1, 3).unwrap());
        let tiny = Computable::rational(
            Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << 1024usize).unwrap(),
        );
        let value = difference(x.clone().exp().sqrt(), x.shift_right(1).add(tiny).exp());
        assert_eq!(value.sign_until(-64), None);
        assert_eq!(value.internal.facts.exact_sign(), ExactSignCache::Unknown);
        assert_eq!(value.sign_until(-64), None);
        assert_eq!(value.sign_until(-1200), Some(RealSign::Negative));
    }

    #[test]
    fn nonzero_remainders_and_both_subtraction_orientations() {
        let atom = || Computable::rational(Rational::fraction(3, 8).unwrap()).sin();
        for bits in [7usize, 127, 511, 767, 999] {
            for numerator in [-7i64, -3, -1, 1, 3, 7] {
                let delta = BigRational::new(numerator.into(), BigInt::one() << bits);
                let a = raw(Approximation::Sqrt(raw(
                    Approximation::PrescaledExp(atom()),
                )));
                let b = raw(Approximation::PrescaledExp(
                    atom().shift_right(1).add(from_q(&delta)),
                ));
                let expected = if numerator < 0 {
                    Sign::Plus
                } else {
                    Sign::Minus
                };
                let expected_public = if numerator < 0 {
                    RealSign::Positive
                } else {
                    RealSign::Negative
                };
                let ordinary = difference(a.clone(), b.clone());
                let reordered = raw(Approximation::Add(
                    raw(Approximation::Negate(b.clone())),
                    a.clone(),
                ));
                let reversed = difference(b, a);
                assert_eq!(
                    ordinary.exact_positive_exp_difference_sign(),
                    Some(expected)
                );
                assert_eq!(
                    reordered.exact_positive_exp_difference_sign(),
                    Some(expected)
                );
                assert_eq!(
                    reversed.exact_positive_exp_difference_sign(),
                    Some(negate_sign(expected))
                );
                assert_eq!(ordinary.sign_until(-64), Some(expected_public));
                assert_eq!(reordered.sign_until(-64), Some(expected_public));
                assert_eq!(
                    ordinary.cached(),
                    None,
                    "structural proof must not request an approximation"
                );
                assert_eq!(reordered.cached(), None);
                assert_eq!(
                    ordinary.internal.facts.exact_sign(),
                    ExactSignCache::Valid(expected)
                );
            }
        }
    }

    #[test]
    fn nonzero_proof_preserves_numeric_caches_and_recovers_after_serde() {
        let x = Computable::rational(Rational::new(2)).sqrt();
        let a = x.clone().exp().sqrt();
        let delta = BigRational::new(BigInt::one(), BigInt::one() << 767usize);
        let b = x.shift_right(1).add(from_q(&delta)).exp();
        let _ = a.approx(-512);
        let _ = b.approx(-512);
        let before = (a.cached(), b.cached());
        let value = difference(a.clone(), b.clone());
        assert_eq!(value.sign_until(-64), Some(RealSign::Negative));
        assert_eq!((a.cached(), b.cached()), before);
        #[cfg(feature = "serde")]
        {
            let encoded = serde_json::to_string(&value).unwrap();
            let restored: Computable = serde_json::from_str(&encoded).unwrap();
            assert_eq!(restored.sign_until(-64), Some(RealSign::Negative));
        }
    }

    #[test]
    fn proof_preserves_hot_operand_and_root_caches() {
        let x = Computable::rational(Rational::new(2)).sqrt();
        let operand = x.clone().exp();
        let root = operand.clone().sqrt();
        let _ = operand.approx(-768);
        let _ = root.approx(-512);
        let before = (operand.cached(), root.cached());
        let zero = difference(root.clone(), x.shift_right(1).exp());
        assert_eq!(zero.zero_status(), ZeroKnowledge::Zero);
        assert_eq!(zero.zero_status(), ZeroKnowledge::Zero);
        assert_eq!((operand.cached(), root.cached()), before);
        assert!(matches!(
            root.internal.approximation,
            Approximation::Sqrt(_)
        ));
    }

    #[test]
    fn structural_limits_fail_closed() {
        let mut exponent = Computable::zero();
        for i in 1..=18 {
            let atom = Computable::rational(Rational::fraction(i, 64).unwrap()).sin();
            exponent = exponent.add(atom);
        }
        let a = raw(Approximation::Sqrt(raw(Approximation::PrescaledExp(
            exponent.clone(),
        ))));
        let b = raw(Approximation::PrescaledExp(exponent.shift_right(1)));
        assert!(!difference(a, b).exact_positive_exp_difference_zero());
        let mut nested = raw(Approximation::PrescaledExp(Computable::one()));
        for _ in 0..140 {
            nested = raw(Approximation::Sqrt(nested));
        }
        assert!(!difference(nested.clone(), nested).exact_positive_exp_difference_zero());
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serialized_graph_recovers_proof_without_serialized_facts() {
        let x = Computable::rational(Rational::new(2)).sqrt();
        let zero = difference(x.clone().exp().sqrt(), x.shift_right(1).exp());
        let before = serde_json::to_string(&zero).unwrap();
        assert_eq!(zero.sign_until(-64), Some(RealSign::Zero));
        assert_eq!(serde_json::to_string(&zero).unwrap(), before);
        let restored: Computable = serde_json::from_str(&before).unwrap();
        assert_eq!(restored.sign_until(-64), Some(RealSign::Zero));
    }
}
