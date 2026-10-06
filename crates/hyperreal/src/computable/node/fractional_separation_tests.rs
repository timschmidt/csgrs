#[cfg(test)]
mod fractional_separation_tests {
    use super::*;
    use crate::Real;

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

    fn inverse_identity(depth: usize, a: i32, b: i32) -> Computable {
        let mut radicand = Real::from(2);
        for _ in 1..depth {
            radicand = Real::from(2) + radicand.sqrt().unwrap();
        }
        let root = radicand.clone().sqrt().unwrap();
        let x = Real::from(a) + Real::from(b) * &root;
        let conjugate = Real::from(a) - Real::from(b) * root;
        let norm = Real::from(a * a) - radicand;
        ((Real::one() / x).unwrap() - (conjugate / norm).unwrap()).fold()
    }

    #[test]
    fn fractional_separation_deep_inverse_identities_and_perturbations() {
        for depth in 1..=6 {
            let mut maximum_bound = 0;
            for a in -3..=3 {
                for b in [-1, 1] {
                    let identity = inverse_identity(depth, a, b);
                    let bound = identity
                        .algebraic_separation_bound_bits()
                        .expect("finite metadata");
                    maximum_bound = maximum_bound.max(bound);
                    assert!(bound <= 1_022, "depth={depth}, a={a}, b={b}, bound={bound}");
                    assert_eq!(identity.sign_until(-1_024), Some(RealSign::Zero));
                    // Rebuild the identity for each sign/scale, so a cached zero
                    // cannot bypass the perturbed expression's certificate.
                    for bits in [128, 512] {
                        for (sign, expected) in [(-1, RealSign::Negative), (1, RealSign::Positive)]
                        {
                            let delta = Rational::from_bigint_fraction(
                                BigInt::from(sign),
                                BigUint::one() << bits,
                            )
                            .unwrap();
                            let value =
                                inverse_identity(depth, a, b).add(Computable::rational(delta));
                            assert_ne!(value.sign_until(-64), Some(RealSign::Zero));
                            assert_eq!(
                                value.sign_until(-1_024),
                                Some(expected),
                                "depth={depth}, a={a}, b={b}, bits={bits}"
                            );
                        }
                    }
                }
            }
            eprintln!("fractional separation depth={depth} maximum_bound={maximum_bound}");
        }
    }

    #[test]
    fn fractional_separation_inverse_swaps_conjugate_bounds_without_norm_growth() {
        let mut metadata = AlgebraicSeparation::rational(&Rational::new(17)).unwrap();
        let root = Computable::rational(Rational::new(2)).sqrt();
        metadata.insert_generator(root, 2).unwrap();
        metadata.denominator_log2 = 3;
        metadata.conjugate_log2 = 5;
        let inverse = metadata.clone().inverse().unwrap();
        assert_eq!((inverse.denominator_log2, inverse.conjugate_log2), (5, 3));
        let twice = inverse.inverse().unwrap();
        assert_eq!((twice.denominator_log2, twice.conjugate_log2), (3, 5));
        assert_eq!(twice.field_degree(), metadata.field_degree());
    }

    #[test]
    fn fractional_separation_rejects_zero_and_unproved_inverse_denominators() {
        let zero = raw(Approximation::Int(BigInt::zero()));
        assert!(
            raw(Approximation::Inverse(zero))
                .algebraic_separation_bound_bits()
                .is_none()
        );
        let unknown = raw(Approximation::Add(
            Computable::pi(),
            Computable::integer(BigInt::from(-3)),
        ));
        assert!(
            raw(Approximation::Inverse(unknown))
                .algebraic_separation_bound_bits()
                .is_none()
        );
    }

    #[test]
    fn fractional_separation_roots_with_negative_integral_denominators_match_mpfr() {
        use rug::{Float, Integer, Rational as Q, float::Round};
        // Both numerator and denominator are negative. Roots of their positive
        // quotient still have an integral numerator gamma*root(beta/gamma),
        // whose selected sign may be negative. No principal-root shortcut is valid.
        for numerator in 2..=8 {
            for denominator in 2..=8 {
                for degree in [2, 3, 4, 5, 7] {
                    let negative = |n| {
                        raw(Approximation::Negate(raw(Approximation::Add(
                            Computable::integer(BigInt::from(n)),
                            raw(Approximation::Sqrt(Computable::integer(BigInt::from(2)))),
                        ))))
                    };
                    let quotient = raw(Approximation::Multiply(
                        negative(numerator),
                        raw(Approximation::Inverse(negative(denominator))),
                    ));
                    let value = if degree == 2 {
                        raw(Approximation::Sqrt(quotient))
                    } else {
                        raw(Approximation::NthRoot(quotient, degree))
                    };
                    assert_eq!(value.sign_until(-64), Some(RealSign::Positive));
                    let bound = value
                        .algebraic_separation_bound_bits()
                        .expect("root fraction metadata");
                    assert!(bound < 256);
                    let mut lo = Float::with_val(1024, 2);
                    let mut hi = lo.clone();
                    lo.sqrt_round(Round::Down);
                    hi.sqrt_round(Round::Up);
                    let lr = lo.to_rational().unwrap();
                    let hr = hi.to_rational().unwrap();
                    let numerator_lo = lr.clone() + numerator;
                    let numerator_hi = hr.clone() + numerator;
                    let denominator_lo = lr + denominator;
                    let denominator_hi = hr + denominator;
                    let mut lo =
                        Float::with_val_round(1024, numerator_lo / denominator_hi, Round::Down).0;
                    let mut hi =
                        Float::with_val_round(1024, numerator_hi / denominator_lo, Round::Up).0;
                    lo.root_round(degree, Round::Down);
                    hi.root_round(degree, Round::Up);
                    let lower = lo.to_rational().unwrap();
                    let upper = hi.to_rational().unwrap();
                    let separation = Q::from((1, Integer::from(1) << bound as u32));
                    assert!(lower >= separation, "invalid separation bound");
                    let unit = Q::from((1, Integer::from(1) << 256_u32));
                    let center = Q::from(
                        Integer::from_str_radix(&value.approx(-256).to_string(), 10).unwrap(),
                    ) * &unit;
                    assert!(Q::from(&center - &unit) <= lower && Q::from(&center + &unit) >= upper);
                }
            }
        }
    }
}
