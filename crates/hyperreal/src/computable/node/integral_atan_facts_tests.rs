#[cfg(test)]
mod integral_atan_facts_tests {
    use super::*;

    fn imported(n: &BigInt) -> Computable {
        Computable {
            internal: Arc::new(Node::new(
                Approximation::IntegralAtan(n.clone()),
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    fn denominators() -> Vec<BigInt> {
        [
            BigInt::one(),
            BigInt::from(2),
            BigInt::from(5),
            BigInt::from(239),
            BigInt::one() << 512,
        ]
        .into_iter()
        .flat_map(|n| [-n.clone(), n])
        .collect()
    }

    fn squared_expectations(n: &BigInt) -> [Rational; 2] {
        let square = n * n;
        let denominator = (&square + BigInt::one()).to_biguint().unwrap();
        [
            Rational::from_bigint_fraction(BigInt::one(), denominator.clone()).unwrap(),
            Rational::from_bigint_fraction(square, denominator).unwrap(),
        ]
    }

    fn exact_square(value: Computable, expected: &Rational) {
        let delta = value.square().add(Computable::rational(-expected.clone()));
        assert_eq!(delta.sign_until(0), Some(RealSign::Zero));
    }

    #[test]
    fn integral_atan_readers_retain_exact_argument_and_family() {
        for n in denominators() {
            let angle = imported(&n);
            let sign = if n.sign() == Sign::Minus { -1 } else { 1 };
            let expected =
                Rational::from_bigint_fraction(BigInt::from(sign), n.magnitude().clone()).unwrap();
            assert_eq!(angle.atan_rational_argument(), Some(expected.clone()));
            assert_eq!(
                angle.atan_argument().unwrap().exact_rational(),
                Some(expected)
            );
            assert!(angle.internal.contains_inverse_trig_or_pi());
            assert!(angle.cached().is_none());
            assert!(
                matches!(&angle.internal.approximation, Approximation::IntegralAtan(stored) if stored == &n)
            );
        }
        let undefined = imported(&BigInt::zero());
        assert!(undefined.atan_rational_argument().is_none());
        assert!(undefined.atan_argument().is_none());
    }

    #[test]
    fn integral_atan_facts_compose_through_trig_and_arithmetic() {
        for n in denominators() {
            let angle = imported(&n);
            let [sine_square, cosine_square] = squared_expectations(&n);
            let sign = if n.sign() == Sign::Minus {
                RealSign::Negative
            } else {
                RealSign::Positive
            };
            for warm in [false, true] {
                if warm {
                    let _ = angle.approx(-128);
                }
                let sine = angle.clone().sin();
                let cosine = angle.clone().cos();
                assert_eq!(sine.sign_until(0), Some(sign));
                assert_eq!(cosine.sign_until(0), Some(RealSign::Positive));
                exact_square(sine, &sine_square);
                exact_square(cosine, &cosine_square);

                let shifted = Computable::pi().add(angle.clone());
                let shifted_sine = shifted.clone().sin();
                let shifted_cosine = shifted.cos();
                assert_eq!(shifted_cosine.sign_until(0), Some(RealSign::Negative));
                assert_eq!(
                    shifted_sine.sign_until(0),
                    Some(if sign == RealSign::Positive {
                        RealSign::Negative
                    } else {
                        RealSign::Positive
                    })
                );
                exact_square(shifted_sine, &sine_square);
                exact_square(shifted_cosine, &cosine_square);

                for orientation in [-1, 1] {
                    let complement = Computable::pi()
                        .shift_right(1)
                        .multiply_rational(Rational::new(orientation))
                        .add(angle.clone().negate());
                    let sine = complement.clone().sin();
                    let cosine = complement.cos();
                    assert_eq!(
                        sine.sign_until(0),
                        Some(if orientation == 1 {
                            RealSign::Positive
                        } else {
                            RealSign::Negative
                        })
                    );
                    assert_eq!(
                        cosine.sign_until(0),
                        Some(if (orientation == 1) == (sign == RealSign::Positive) {
                            RealSign::Positive
                        } else {
                            RealSign::Negative
                        })
                    );
                    exact_square(sine, &cosine_square);
                    exact_square(cosine, &sine_square);
                }
                if !warm {
                    assert!(angle.cached().is_none());
                }
            }
        }
        // atan is strictly increasing; these fresh imported arguments are
        // exact rationals with 1/2 > 1/5. No refinement is needed for the order.
        let difference = imported(&BigInt::from(2)).add(imported(&BigInt::from(5)).negate());
        assert_eq!(difference.sign_until(0), Some(RealSign::Positive));
    }

    #[test]
    fn integral_atan_facts_survive_concurrent_reuse() {
        for n in [-5, -1, 1, 5] {
            let n = BigInt::from(n);
            let angle = imported(&n);
            let expected = squared_expectations(&n);
            std::thread::scope(|scope| {
                for warm in [false, true, false, true] {
                    let angle = angle.clone();
                    let expected = expected.clone();
                    scope.spawn(move || {
                        if warm {
                            let _ = angle.approx(-128);
                        }
                        exact_square(angle.clone().sin(), &expected[0]);
                        exact_square(angle.cos(), &expected[1]);
                    });
                }
            });
            exact_square(angle.clone().sin(), &expected[0]);
            exact_square(angle.cos(), &expected[1]);
        }
    }
}
