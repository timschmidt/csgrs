#[cfg(test)]
mod atan_numeric_probe_tests {
    use super::algebraic_leaf_tests::measure;
    use super::*;
    use crate::EvaluationBudget;
    use crate::computable::budget::BudgetScope;

    fn root(route: &str, bits: usize, negative: bool) -> Computable {
        let n: BigInt = (BigInt::one() << (bits - 1)) + 1;
        assert_eq!(n.bits(), bits as u64);
        let signed = if negative { -n.clone() } else { n.clone() };
        let value = match route {
            "integer" => Computable::rational(Rational::from_bigint(signed)).atan(),
            "reciprocal" => Computable::rational(
                Rational::from_bigint_fraction(
                    BigInt::from(if negative { -1_i32 } else { 1 }),
                    n.magnitude().clone(),
                )
                .unwrap(),
            )
            .atan(),
            "integral" => Computable::raw_periodic_operand(Approximation::IntegralAtan(signed)),
            "deferred" => Computable::raw_periodic_operand(Approximation::AtanDeferred(
                Computable::rational(Rational::from_bigint(signed)),
            )),
            "opaque" => Computable::rational(Rational::from_bigint(signed))
                .add(Computable::pi())
                .atan(),
            _ => unreachable!(),
        };
        match route {
            "integer" | "reciprocal" => {
                let rational_node = if negative {
                    let Approximation::Negate(child) = &value.internal.approximation else {
                        panic!("negative public rational atan lost its symmetry node");
                    };
                    child
                } else {
                    &value
                };
                let Approximation::AtanRational(r) = &rational_node.internal.approximation else {
                    panic!("public rational atan did not retain its rational input");
                };
                assert_eq!(r.sign(), Sign::Plus);
                assert_eq!(
                    r.stored_component_bits(),
                    if route == "integer" {
                        (bits as u64, 1)
                    } else {
                        (1, bits as u64)
                    }
                );
            }
            "integral" => assert!(matches!(
                value.internal.approximation,
                Approximation::IntegralAtan(_)
            )),
            "deferred" | "opaque" => assert!(matches!(
                value.internal.approximation,
                Approximation::AtanDeferred(_)
            )),
            _ => unreachable!(),
        }
        assert!(value.internal.cache_snapshot().is_none());
        value
    }

    fn survey(route: &str) {
        let zero = EvaluationBudget::new(0);
        drop(BudgetScope::enter(&zero));
        Computable::pi().approx(-32).unwrap();
        for bits in [65, 4097, 65_537, 1_000_001] {
            for negative in [false, true] {
                for warm in [false, true] {
                    for units in [0, 1, 8, 32] {
                        let value = root(route, bits, negative);
                        // A failed preparation remains an explicitly recorded
                        // partial warmup; it must not be mislabeled as a cache hit.
                        let preparation = warm.then(|| {
                            value.approx_with_budget(4, &EvaluationBudget::new(1_000_000))
                        });
                        if matches!(preparation, Some(Ok(_))) {
                            assert_eq!(preparation, Some(Ok(BigInt::zero())));
                            assert_eq!(value.internal.cache_snapshot(), Some((4, BigInt::zero())));
                        }
                        assert!(
                            preparation
                                .as_ref()
                                .is_none_or(|r| r.is_ok() || *r == Err(Problem::Exhausted))
                        );
                        let before = value.internal.cache_snapshot();
                        let sign_before = value.immediate_sign();
                        let budget = EvaluationBudget::new(units);
                        let (answer, allocations) =
                            measure(|| value.approx_with_budget(0, &budget));
                        let after = value.internal.cache_snapshot();
                        eprintln!(
                            "atan numeric survey route={route}, bits={bits}, negative={negative}, warm={warm}, preparation={preparation:?}, units={units}, answer={answer:?}, consumed={}, remaining={}, finite={}, before={before:?}, after={after:?}, allocations={allocations:?}",
                            budget.consumed(),
                            budget.remaining(),
                            value.internal.facts.finite_real()
                        );
                        assert_eq!(budget.consumed() + budget.remaining(), units);
                        match &answer {
                            Ok(sample) => {
                                // atan(1/n) has magnitude below 2^-64. The other
                                // routes have magnitude between 1 and 2, since
                                // |n| >= 2^64+1 and the opaque offset is pi < 4.
                                if matches!(route, "reciprocal" | "integral") {
                                    assert!(sample.magnitude() <= &BigUint::one());
                                } else {
                                    assert!(
                                        !sample.is_zero()
                                            && sample.magnitude() <= &BigUint::from(2_u8)
                                    );
                                }
                                if !sample.is_zero() {
                                    assert_eq!(
                                        sample.sign(),
                                        if negative { Sign::Minus } else { Sign::Plus }
                                    );
                                }
                                assert_eq!(value.approx_with_budget(0, &zero), answer);
                                assert_eq!(value.approx(0), answer);
                            }
                            Err(Problem::Exhausted) => assert_eq!(after, before),
                            Err(error) => panic!("valid atan argument returned {error:?}"),
                        }
                        if before.is_some() {
                            assert_eq!(value.approx_with_budget(4, &zero), Ok(BigInt::zero()));
                        }
                        let sign = value.sign_until_with_budget(0, &zero);
                        assert!(sign.is_none_or(|s| s
                            == if negative {
                                RealSign::Negative
                            } else {
                                RealSign::Positive
                            }));
                        if sign_before.is_some() {
                            assert_eq!(sign, sign_before);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn atan_numeric_public_integer() {
        survey("integer");
    }
    #[test]
    fn atan_numeric_public_reciprocal() {
        survey("reciprocal");
    }
    #[test]
    fn atan_numeric_internal_integral() {
        survey("integral");
    }
    #[test]
    fn atan_numeric_internal_deferred() {
        survey("deferred");
    }
    #[test]
    fn atan_numeric_public_opaque() {
        survey("opaque");
    }
}
