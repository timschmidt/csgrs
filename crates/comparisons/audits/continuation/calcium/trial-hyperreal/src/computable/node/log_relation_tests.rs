#[cfg(test)]
mod log_relation_tests {
    use super::*;
    use crate::{CertifiedRealEquality, Real};
    use std::sync::atomic::{AtomicBool, Ordering as AtomicOrdering};

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

    fn raw_difference(left: Computable, right: Computable) -> Computable {
        raw(Approximation::Add(left, raw(Approximation::Negate(right))))
    }

    fn field(a: i64, d: i64) -> Real {
        Real::from(a) + Real::from(d).sqrt().unwrap()
    }

    #[test]
    fn log_relations_reject_zero_negative_and_non_algebraic_arguments() {
        for residual in [-3, -2, -1] {
            let logarithm = raw(Approximation::PrescaledLn(Computable::rational(
                Rational::new(residual),
            )));
            let difference = raw_difference(logarithm.clone(), logarithm);
            assert!(
                !difference.log_relation_is_zero(-512),
                "residual {residual}"
            );
        }
        let logarithm = Computable::pi().ln();
        assert!(!raw_difference(logarithm.clone(), logarithm).log_relation_is_zero(-512));
        let not_a_log_relation = raw_difference(Computable::pi(), Computable::pi());
        assert!(!not_a_log_relation.log_relation_is_zero(-512));
    }

    #[test]
    fn log_relations_coefficients_and_traversal_are_bounded() {
        let logarithm = Computable::rational(Rational::fraction(1, 3).unwrap()).ln();
        let too_large = raw(Approximation::Multiply(
            logarithm.clone(),
            Computable::integer(BigInt::from(65)),
        ));
        assert!(!raw_difference(too_large.clone(), too_large).log_relation_is_zero(-512));
        let mut long = logarithm.clone();
        for _ in 0..140 {
            long = raw(Approximation::Add(long, logarithm.clone()));
        }
        assert!(!raw_difference(long.clone(), long).log_relation_is_zero(-512));
        let large_shift = raw(Approximation::Offset(logarithm, i32::MAX));
        assert!(!raw_difference(large_shift.clone(), large_shift).log_relation_is_zero(-512));
    }

    #[test]
    fn log_relations_require_exact_identity_even_below_numeric_resolution() {
        for d in [2, 3, 5, 7, 11] {
            for bits in [64_usize, 128, 256, 512] {
                let x = field(1, d);
                let square = &x * &x;
                let epsilon = Real::from(
                    Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << bits).unwrap(),
                );
                let left = Real::from(2) * x.ln().unwrap();
                let right = (square + epsilon).ln().unwrap();
                for floor in [-64, -256, -1024] {
                    assert!(
                        !matches!(
                            left.certified_eq_until(&right, floor),
                            CertifiedRealEquality::Equal { .. }
                        ),
                        "d={d}, bits={bits}, floor={floor}"
                    );
                }
                assert!(matches!(
                    left.certified_eq_until(&right, -2048),
                    CertifiedRealEquality::NotEqual { .. }
                ));
            }
        }
    }

    #[test]
    fn log_relations_preserve_both_signs_below_approximation_resolution() {
        for bits in [128_usize, 512, 1024, 2048] {
            let epsilon =
                Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << bits).unwrap();
            for scale in [Rational::one(), Rational::fraction(-3, 5).unwrap()] {
                let x = Computable::rational(Rational::new(17) + &epsilon).ln();
                let y = Computable::rational(Rational::new(17)).ln();
                let difference = x
                    .add(y.negate())
                    .multiply(Computable::rational(scale.clone()));
                let expected = if scale.sign() == Sign::Plus {
                    RealSign::Positive
                } else {
                    RealSign::Negative
                };
                for floor in [-16, -128, -256] {
                    assert_eq!(
                        difference.log_relation_sign(floor),
                        Some(expected),
                        "bits={bits}, floor={floor}"
                    );
                    assert_eq!(difference.sign_until(floor), Some(expected));
                }
            }
        }
    }

    #[test]
    fn log_relation_signs_match_independent_rational_products() {
        use num::BigRational;

        let mut state = 0x5eeda11_u64;
        let mut next = || {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            state >> 32
        };
        let mut decisions = 0;
        for case in 0..2000 {
            let mut value = Computable::zero();
            let mut product = BigRational::one();
            for _ in 0..(2 + next() % 3) {
                let numerator = (2 + next() % 40) as i64;
                let denominator = 2 + next() % 6;
                let coefficient = (1 + next() % 3) as i32 * if next() & 1 == 0 { 1 } else { -1 };
                let base = BigRational::new(BigInt::from(numerator), BigInt::from(denominator));
                if coefficient > 0 {
                    product *= base.pow(coefficient);
                } else {
                    product /= base.pow(-coefficient);
                }
                let logarithm =
                    Computable::rational(Rational::fraction(numerator, denominator).unwrap()).ln();
                value = value.add(
                    logarithm.multiply(Computable::rational(Rational::new(i64::from(coefficient)))),
                );
            }
            let expected = match product.cmp(&BigRational::one()) {
                std::cmp::Ordering::Less => RealSign::Negative,
                std::cmp::Ordering::Equal => RealSign::Zero,
                std::cmp::Ordering::Greater => RealSign::Positive,
            };
            if let Some(actual) = value.log_relation_sign(-64) {
                decisions += 1;
                assert_eq!(actual, expected, "case={case}");
            }
            assert_eq!(value.sign_until(-512), Some(expected), "public case={case}");
        }
        assert!(
            decisions >= 1500,
            "insufficient exercised proof decisions: {decisions}"
        );
    }

    #[test]
    fn log_relations_products_quotients_roots_and_rational_coefficients() {
        for d in [2, 3, 5, 7] {
            let x = field(1, d);
            let y = field(2, d);
            let product = &x * &y;
            let quotient = (&x / &y).unwrap();
            let cases = [
                (
                    x.clone().ln().unwrap() + y.clone().ln().unwrap(),
                    product.ln().unwrap(),
                ),
                (
                    x.clone().ln().unwrap() - y.clone().ln().unwrap(),
                    quotient.ln().unwrap(),
                ),
                (
                    x.clone().ln().unwrap(),
                    Real::from(2) * x.clone().sqrt().unwrap().ln().unwrap(),
                ),
            ];
            for (left, right) in cases {
                for (n, den) in [(1, 1), (-1, 1), (2, 3), (-3, 5), (5, 7)] {
                    let scale = Real::from(Rational::fraction(n, den).unwrap());
                    let l = &left * &scale;
                    let r = &right * &scale;
                    assert!(
                        matches!(
                            l.certified_eq_until(&r, -1024),
                            CertifiedRealEquality::Equal { .. }
                        ),
                        "d={d}, scale={n}/{den}"
                    );
                }
            }
        }
    }

    #[test]
    fn log_relation_query_order_and_concurrent_sharing_are_consistent() {
        let x = field(1, 2);
        let expanded = Real::from(3) + Real::from(2) * Real::from(2).sqrt().unwrap();
        let left = Real::from(2) * x.ln().unwrap();
        let right = expanded.ln().unwrap();
        std::thread::scope(|scope| {
            for order in [[-64, -1024, -16, -256], [-256, -16, -1024, -64]] {
                let left = &left;
                let right = &right;
                scope.spawn(move || {
                    for floor in order {
                        assert!(matches!(
                            left.certified_eq_until(right, floor),
                            CertifiedRealEquality::Equal { .. }
                        ));
                    }
                });
            }
        });
    }

    #[test]
    fn log_relation_cancellation_does_not_publish_a_proof() {
        let logarithm = Computable::rational(Rational::fraction(1, 3).unwrap()).ln();
        let mut difference = raw_difference(logarithm.clone(), logarithm);
        let stopped = Arc::new(AtomicBool::new(true));
        difference.abort(stopped.clone());
        assert!(!difference.log_relation_is_zero(-512));
        assert!(difference.cached().is_none());
        stopped.store(false, AtomicOrdering::Relaxed);
        assert!(difference.log_relation_is_zero(-512));
    }

    #[cfg(feature = "serde")]
    #[test]
    fn log_relation_serialization_reconstructs_the_proof_from_expression() {
        let x = field(1, 2);
        let expanded = Real::from(3) + Real::from(2) * Real::from(2).sqrt().unwrap();
        let left = Real::from(2) * x.ln().unwrap();
        let right = expanded.ln().unwrap();
        for _ in 0..2 {
            let lhs: Real = serde_json::from_str(&serde_json::to_string(&left).unwrap()).unwrap();
            let rhs: Real = serde_json::from_str(&serde_json::to_string(&right).unwrap()).unwrap();
            assert!(matches!(
                lhs.certified_eq_until(&rhs, -512),
                CertifiedRealEquality::Equal { .. }
            ));
            assert!(matches!(
                left.certified_eq_until(&right, -512),
                CertifiedRealEquality::Equal { .. }
            ));
        }
    }
}
