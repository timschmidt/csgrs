#[cfg(test)]
mod rational_preflight_tests {
    use super::algebraic_leaf_tests::measure;
    use super::*;
    use crate::EvaluationBudget;
    use crate::computable::budget::BudgetScope;

    fn raw(kind: Approximation) -> Computable {
        Computable::raw_periodic_operand(kind)
    }

    fn lazy_parts(bits: usize, n: BigUint, d: BigUint, negative: bool) -> Rational {
        let factor = (BigUint::one() << (bits - 1)) + BigUint::one();
        Rational::from_parts_raw_unreduced(
            if negative { Sign::Minus } else { Sign::Plus },
            n * &factor,
            d * factor,
        )
    }

    fn lazy(bits: usize, n: u8, d: u8, negative: bool) -> Rational {
        lazy_parts(bits, BigUint::from(n), BigUint::from(d), negative)
    }

    fn child_domain(kind: u8, value: Rational) -> Computable {
        let child = raw(Approximation::Ratio(value));
        raw(match kind {
            0 => Approximation::Sqrt(child),
            1 => Approximation::NthRoot(child, 3),
            2 => Approximation::Inverse(child),
            3 => Approximation::LnDeferred(child),
            4 => Approximation::AsinDeferred(child),
            5 => Approximation::AcosPositive(child),
            6 => Approximation::AtanhDirect(child),
            7 => Approximation::AcoshDirect(child),
            _ => unreachable!(),
        })
    }

    fn literal_domain(kind: u8, value: Rational) -> Computable {
        raw(match kind {
            0 => Approximation::AsinRational(value),
            1 => Approximation::AcosPositiveRational(value),
            2 => Approximation::AcosNegativeRational(value),
            3 => Approximation::AtanhRational(value),
            4 => Approximation::PrescaledLnRational(value),
            5 => Approximation::BinaryScaledLnRational {
                residual: value,
                shift: 0,
            },
            _ => unreachable!(),
        })
    }

    fn valid_domain(kind: u8, bits: Option<usize>) -> Computable {
        let (n, d, negative) = if kind == 7 {
            (7, 3, false)
        } else {
            (3, 7, kind >= 12)
        };
        let value = match bits {
            Some(bits) => lazy(bits, n, d, negative),
            None => Rational::fraction(
                if negative {
                    -i64::from(n)
                } else {
                    i64::from(n)
                },
                u64::from(d),
            )
            .unwrap(),
        };
        if kind < 8 {
            child_domain(kind, value)
        } else {
            literal_domain(kind - 8, value)
        }
    }

    fn domain_allowance(kind: u8) -> u64 {
        if kind < 8 { 3 } else { 1 }
    }

    #[test]
    fn rational_preflight_lazy_leaf_stops_before_reduction() {
        let mut observations = Vec::new();
        for bits in [65, 4097, 65_537, 1_000_001] {
            for negative in [false, true] {
                let value = raw(Approximation::Ratio(lazy(bits, 3, 7, negative)));
                let facts = value.internal.facts.snapshot();
                let budget = EvaluationBudget::new(1);
                let _scope = BudgetScope::enter(&budget);
                let (answer, allocations) = measure(|| value.bounded_rational_value());
                eprintln!(
                    "rational leaf bits={bits}, negative={negative}, answer={}, exhausted={}, allocations={allocations:?}",
                    answer.is_some(),
                    evaluation_exhausted()
                );
                observations.push((answer.is_none(), evaluation_exhausted(), allocations));
                assert_eq!(budget.consumed(), 1);
                assert_eq!(value.internal.facts.snapshot(), facts);
                assert!(value.internal.cache_snapshot().is_none());
            }
        }
        assert!(
            observations
                .iter()
                .all(|(none, exhausted, a)| *none && *exhausted && a.calls == 0 && a.bytes == 0)
        );
    }

    #[test]
    fn rational_preflight_lazy_boundaries_stop_before_reduction() {
        let mut observations = Vec::new();
        for value_bits in [4096, 4097] {
            for denominator in [false, true] {
                for negative in [false, true] {
                    for bits in [65, 4097] {
                        let high = BigUint::one() << (value_bits - 1);
                        let (n, d) = if denominator {
                            (BigUint::one(), high)
                        } else {
                            (high, BigUint::one())
                        };
                        let value = raw(Approximation::Ratio(lazy_parts(bits, n, d, negative)));
                        let budget = EvaluationBudget::new(1);
                        let _scope = BudgetScope::enter(&budget);
                        let (answer, allocations) = measure(|| value.bounded_rational_value());
                        eprintln!(
                            "rational boundary value_bits={value_bits}, denominator={denominator}, negative={negative}, factor_bits={bits}, answer={}, exhausted={}, allocations={allocations:?}",
                            answer.is_some(),
                            evaluation_exhausted()
                        );
                        observations.push((answer.is_none(), evaluation_exhausted(), allocations));
                        assert_eq!(budget.consumed(), 1);
                    }
                }
            }
        }
        assert!(
            observations
                .iter()
                .all(|(none, exhausted, a)| *none && *exhausted && a.calls == 0 && a.bytes == 0)
        );
    }

    #[test]
    fn rational_preflight_child_domains_stop_before_reduction() {
        let mut observations = Vec::new();
        for bits in [65, 4097, 65_537, 1_000_001] {
            for kind in 0..8 {
                let value = valid_domain(kind, Some(bits));
                let facts = value.internal.facts.snapshot();
                let budget = EvaluationBudget::new(3);
                let _scope = BudgetScope::enter(&budget);
                let (answer, allocations) = measure(|| value.has_finite_real_certificate());
                eprintln!(
                    "child domain bits={bits}, kind={kind}, answer={answer}, exhausted={}, allocations={allocations:?}",
                    evaluation_exhausted()
                );
                observations.push((answer, evaluation_exhausted(), allocations));
                assert_eq!(budget.consumed(), 3);
                assert_eq!(value.internal.facts.snapshot(), facts);
                assert!(!value.internal.facts.finite_real());
                assert!(value.internal.cache_snapshot().is_none());
            }
        }
        assert!(
            observations.iter().all(|(answer, exhausted, a)| !answer
                && *exhausted
                && a.calls == 0
                && a.bytes == 0)
        );
    }

    #[test]
    fn rational_preflight_literal_domains_stop_before_reduction() {
        let mut observations = Vec::new();
        for bits in [65, 4097, 65_537, 1_000_001] {
            for kind in 8..14 {
                let value = valid_domain(kind, Some(bits));
                let budget = EvaluationBudget::new(1);
                let _scope = BudgetScope::enter(&budget);
                let (answer, allocations) = measure(|| value.has_finite_real_certificate());
                eprintln!(
                    "literal domain bits={bits}, kind={kind}, answer={answer}, exhausted={}, allocations={allocations:?}",
                    evaluation_exhausted()
                );
                observations.push((answer, evaluation_exhausted(), allocations));
                assert_eq!(budget.consumed(), 1);
                assert!(!value.internal.facts.finite_real());
                assert!(value.internal.cache_snapshot().is_none());
            }
        }
        assert!(
            observations.iter().all(|(answer, exhausted, a)| !answer
                && *exhausted
                && a.calls == 0
                && a.bytes == 0)
        );
    }

    #[test]
    fn rational_preflight_public_exhaustion_has_no_width_dependent_allocation() {
        // Warm only the allowance stack, outside the measured public calls.
        let zero = EvaluationBudget::new(0);
        drop(BudgetScope::enter(&zero));
        let mut observations = Vec::new();
        for kind in 0..14 {
            let control = valid_domain(kind, None);
            let control_budget = EvaluationBudget::new(domain_allowance(kind));
            let (answer, control_allocations) =
                measure(|| control.approx_with_budget(-16, &control_budget));
            assert_eq!(answer, Err(Problem::Exhausted));
            for bits in [65, 4097, 65_537, 1_000_001] {
                let value = valid_domain(kind, Some(bits));
                let budget = EvaluationBudget::new(domain_allowance(kind));
                let (answer, allocations) = measure(|| value.approx_with_budget(-16, &budget));
                eprintln!(
                    "public domain bits={bits}, kind={kind}, answer={answer:?}, control={control_allocations:?}, allocations={allocations:?}"
                );
                assert_eq!(answer, Err(Problem::Exhausted));
                assert_eq!(
                    budget.consumed() + budget.remaining(),
                    domain_allowance(kind)
                );
                assert!(value.internal.cache_snapshot().is_none());
                observations.push((allocations, control_allocations));
            }
        }
        assert!(
            observations
                .iter()
                .all(|(a, control)| a.calls <= control.calls && a.bytes <= control.bytes)
        );
    }

    #[test]
    fn rational_preflight_generous_boundaries_preserve_exact_values() {
        for value_bits in [4096, 4097] {
            for denominator in [false, true] {
                for negative in [false, true] {
                    let high = BigUint::one() << (value_bits - 1);
                    let (n, d) = if denominator {
                        (BigUint::one(), high)
                    } else {
                        (high, BigUint::one())
                    };
                    let sign = if negative { Sign::Minus } else { Sign::Plus };
                    let exact = Rational::from_bigint_fraction(
                        BigInt::from_biguint(sign, n.clone()),
                        d.clone(),
                    )
                    .unwrap();
                    for bits in [65, 4097, 65_537] {
                        for metered in [false, true] {
                            let value = raw(Approximation::Ratio(lazy_parts(
                                bits,
                                n.clone(),
                                d.clone(),
                                negative,
                            )));
                            let budget = EvaluationBudget::new(1_000_000_000);
                            let scope = metered.then(|| BudgetScope::enter(&budget));
                            let answer = value.bounded_rational_value();
                            assert_eq!(answer, (value_bits == 4096).then(|| exact.clone()));
                            assert!(!evaluation_exhausted());
                            drop(scope);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn rational_preflight_cheap_rejections_and_sign_domains_stay_cheap() {
        for bits in [4097, 65_537, 1_000_001] {
            for integer in [false, true] {
                let n = BigInt::one() << (bits - 1);
                let value = raw(if integer {
                    Approximation::Int(n)
                } else {
                    Approximation::Ratio(Rational::from_bigint(n))
                });
                let budget = EvaluationBudget::new(1);
                let _scope = BudgetScope::enter(&budget);
                let (answer, allocations) = measure(|| value.bounded_rational_value());
                assert!(answer.is_none());
                assert!(!evaluation_exhausted());
                assert_eq!(budget.consumed(), 1);
                assert_eq!((allocations.calls, allocations.bytes), (0, 0));
            }
        }
        for bits in [65, 4097, 65_537, 1_000_001] {
            for kind in [4, 5] {
                let value = literal_domain(kind, lazy(bits, 3, 7, false));
                let budget = EvaluationBudget::new(1);
                let _scope = BudgetScope::enter(&budget);
                let (answer, allocations) = measure(|| value.has_finite_real_certificate());
                assert!(answer);
                assert!(!evaluation_exhausted());
                assert_eq!(budget.consumed(), 1);
                assert_eq!((allocations.calls, allocations.bytes), (0, 0));
            }
        }
    }

    #[test]
    fn rational_preflight_every_recognition_cutpoint_preserves_retry() {
        for bits in [65, 256, 4097] {
            for kind in 0..3 {
                let fixture = || {
                    let mut value = raw(Approximation::Ratio(lazy(bits, 3, 7, false)));
                    if kind == 1 {
                        for _ in 0..8 {
                            value = raw(Approximation::Offset(value, 0));
                        }
                    } else if kind == 2 {
                        value = raw(Approximation::Inverse(value));
                    }
                    value
                };
                let expected = if kind == 2 {
                    Rational::fraction(7, 3).unwrap()
                } else {
                    Rational::fraction(3, 7).unwrap()
                };
                let generous = EvaluationBudget::new(100_000);
                {
                    let _scope = BudgetScope::enter(&generous);
                    assert_eq!(fixture().bounded_rational_value(), Some(expected.clone()));
                }
                let needed = generous.consumed();
                assert!(needed > 1 && needed < 4096);
                eprintln!("rational cutpoints bits={bits}, kind={kind}, needed={needed}");
                for units in 0..=needed + 2 {
                    let value = fixture();
                    let facts = value.internal.facts.snapshot();
                    let budget = EvaluationBudget::new(units);
                    {
                        let _scope = BudgetScope::enter(&budget);
                        assert_eq!(
                            value.bounded_rational_value(),
                            (units >= needed).then(|| expected.clone())
                        );
                        assert_eq!(evaluation_exhausted(), units < needed);
                    }
                    assert_eq!(budget.consumed() + budget.remaining(), units);
                    assert_eq!(value.internal.facts.snapshot(), facts);
                    assert!(value.internal.cache_snapshot().is_none());
                    assert_eq!(value.bounded_rational_value(), Some(expected.clone()));
                }
            }
        }
    }

    #[test]
    fn rational_preflight_public_retries_preserve_completed_proofs() {
        for bits in [65, 4097, 65_537] {
            for kind in 0..14 {
                let value = valid_domain(kind, Some(bits));
                let low = EvaluationBudget::new(domain_allowance(kind));
                assert_eq!(value.approx_with_budget(-16, &low), Err(Problem::Exhausted));
                let generous = EvaluationBudget::new(1_000_000_000);
                let sample = value.approx_with_budget(-64, &generous).unwrap();
                let reference = valid_domain(kind, None).approx(-64).unwrap();
                assert!((&sample - reference).abs() <= BigInt::from(2));
                let expected = if kind == 3 || kind >= 12 {
                    RealSign::Negative
                } else {
                    RealSign::Positive
                };
                assert_eq!(value.sign_until_with_budget(-64, &generous), Some(expected));
                let zero = EvaluationBudget::new(0);
                assert_eq!(
                    value.clone().sign_until_with_budget(0, &zero),
                    Some(expected)
                );
                assert_eq!(value.approx_with_budget(-64, &zero), Ok(sample));
                assert_eq!(zero.consumed(), 0);
            }
        }
    }

    #[test]
    fn rational_preflight_domain_errors_and_root_identities_are_preserved() {
        for bits in [65, 4097, 65_537] {
            for kind in [0, 1, 3, 4, 5, 6, 7] {
                let argument = match kind {
                    0 | 1 | 3 => lazy(bits, 3, 7, true),
                    7 => lazy(bits, 3, 7, false),
                    _ => lazy(bits, 7, 3, false),
                };
                let value = child_domain(kind, argument);
                let expected = if kind == 0 {
                    Problem::SqrtNegative
                } else {
                    Problem::NotANumber
                };
                let generous = EvaluationBudget::new(1_000_000_000);
                assert_eq!(value.approx_with_budget(-32, &generous), Err(expected));
                assert!(value.internal.cache_snapshot().is_none());
                assert_eq!(value.approx(-32), Err(expected));
            }
            for kind in 0..6 {
                let value = literal_domain(kind, lazy(bits, 7, 3, kind >= 4));
                let generous = EvaluationBudget::new(1_000_000_000);
                assert_eq!(
                    value.approx_with_budget(-32, &generous),
                    Err(Problem::NotANumber)
                );
                assert!(value.internal.cache_snapshot().is_none());
            }
            let a = child_domain(0, lazy(bits, 4, 9, false));
            let b = child_domain(1, lazy(bits, 8, 27, false));
            assert_eq!(
                a.bounded_rational_value(),
                Some(Rational::fraction(2, 3).unwrap())
            );
            assert_eq!(
                b.bounded_rational_value(),
                Some(Rational::fraction(2, 3).unwrap())
            );
        }
    }
}
