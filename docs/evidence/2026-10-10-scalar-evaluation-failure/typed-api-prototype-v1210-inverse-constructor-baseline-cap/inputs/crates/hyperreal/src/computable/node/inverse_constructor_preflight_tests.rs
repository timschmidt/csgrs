#[cfg(test)]
mod inverse_constructor_preflight_tests {
    use super::algebraic_leaf_tests::measure;
    use super::inverse_constructor_probe_tests::public_difference;
    use super::*;
    use crate::EvaluationBudget;
    use crate::computable::budget::BudgetScope;

    fn expected(reversed: bool) -> RealSign {
        if reversed {
            RealSign::Negative
        } else {
            RealSign::Positive
        }
    }

    fn raw_integer_difference(bits: usize, reversed: bool) -> Computable {
        let raw = Computable::raw_periodic_operand;
        let argument = |bits| {
            raw(Approximation::AtanDeferred(raw(Approximation::Int(
                (BigInt::one() << (bits - 1)) + 1,
            ))))
        };
        let left = argument(bits);
        let right = argument(bits - 1);
        if reversed {
            raw(Approximation::Add(right, raw(Approximation::Negate(left))))
        } else {
            raw(Approximation::Add(left, raw(Approximation::Negate(right))))
        }
    }

    #[test]
    fn inverse_constructor_public_pi_preflight_stops_wide_offset_copies() {
        let mut observations = Vec::new();
        for reversed in [false, true] {
            for units in [32, 40, 48, 64] {
                let control = public_difference(8, true, reversed);
                let control_budget = EvaluationBudget::new(units);
                let (_, control_allocations) =
                    measure(|| control.sign_until_with_budget(0, &control_budget));
                for bits in [4097, 65_537, 1_000_001] {
                    let value = public_difference(bits, true, reversed);
                    let budget = EvaluationBudget::new(units);
                    let (answer, allocations) =
                        measure(|| value.sign_until_with_budget(0, &budget));
                    eprintln!(
                        "constructor public pi bits={bits}, reversed={reversed}, units={units}, answer={answer:?}, consumed={}, control={control_allocations:?}, allocations={allocations:?}",
                        budget.consumed()
                    );
                    assert!(answer.is_none_or(|answer| answer == expected(reversed)));
                    assert_eq!(budget.consumed() + budget.remaining(), units);
                    assert!(value.internal.cache_snapshot().is_none());
                    observations.push((allocations.bytes, control_allocations.bytes));
                }
            }
        }
        assert!(observations.iter().all(|(bytes, control)| bytes <= control));
    }

    #[test]
    fn inverse_constructor_raw_integer_preflight_stops_wide_leaf_copies() {
        // This internal shape is distinct from public atan(rational), which
        // uses AtanRational and has its own preflight preservation test below.
        let mut observations = Vec::new();
        for reversed in [false, true] {
            for units in [24, 32, 40, 48] {
                let control = raw_integer_difference(8, reversed);
                let control_budget = EvaluationBudget::new(units);
                let (_, control_allocations) =
                    measure(|| control.sign_until_with_budget(0, &control_budget));
                for bits in [4097, 65_537, 1_000_001] {
                    let value = raw_integer_difference(bits, reversed);
                    let budget = EvaluationBudget::new(units);
                    let (answer, allocations) =
                        measure(|| value.sign_until_with_budget(0, &budget));
                    eprintln!(
                        "constructor raw integer bits={bits}, reversed={reversed}, units={units}, answer={answer:?}, consumed={}, control={control_allocations:?}, allocations={allocations:?}",
                        budget.consumed()
                    );
                    assert!(answer.is_none_or(|answer| answer == expected(reversed)));
                    assert_eq!(budget.consumed() + budget.remaining(), units);
                    assert!(value.internal.cache_snapshot().is_none());
                    observations.push((allocations.bytes, control_allocations.bytes));
                }
            }
        }
        assert!(observations.iter().all(|(bytes, control)| bytes <= control));
    }

    #[test]
    fn inverse_constructor_public_integer_atan_stays_preflighted() {
        for reversed in [false, true] {
            for units in [0, 16, 24, 32] {
                let mut control = None;
                for bits in [4097, 65_537, 1_000_001] {
                    let value = public_difference(bits, false, reversed);
                    let budget = EvaluationBudget::new(units);
                    let (answer, allocations) =
                        measure(|| value.sign_until_with_budget(0, &budget));
                    assert_eq!(answer, None);
                    assert_eq!(budget.consumed() + budget.remaining(), units);
                    let measured = (allocations.calls, allocations.bytes);
                    if let Some(control) = control {
                        assert_eq!(measured, control);
                    } else {
                        control = Some(measured);
                    }
                    assert!(value.internal.cache_snapshot().is_none());
                    assert_eq!(value.immediate_sign(), None);
                }
            }
        }
    }

    #[test]
    fn inverse_constructor_every_public_sign_cutpoint_preserves_retry() {
        for bits in [8, 65, 256] {
            for reversed in [false, true] {
                let generous = EvaluationBudget::new(1_000_000);
                assert_eq!(
                    public_difference(bits, true, reversed).sign_until_with_budget(0, &generous),
                    Some(expected(reversed))
                );
                let needed = generous.consumed();
                assert!(needed > 1 && needed < 4096);
                eprintln!(
                    "constructor cutpoints bits={bits}, reversed={reversed}, needed={needed}"
                );
                for units in 0..=needed + 2 {
                    let value = public_difference(bits, true, reversed);
                    let budget = EvaluationBudget::new(units);
                    let answer = value.sign_until_with_budget(0, &budget);
                    assert_eq!(answer, (units >= needed).then(|| expected(reversed)));
                    assert_eq!(budget.consumed() + budget.remaining(), units);
                    if answer.is_none() {
                        assert_eq!(value.immediate_sign(), None);
                        assert!(value.internal.cache_snapshot().is_none());
                    }
                    assert_eq!(value.sign_until(0), Some(expected(reversed)));
                    let zero = EvaluationBudget::new(0);
                    assert_eq!(
                        value.clone().sign_until_with_budget(0, &zero),
                        Some(expected(reversed))
                    );
                    assert_eq!(zero.consumed(), 0);
                }
            }
        }
    }

    #[test]
    fn inverse_constructor_wide_retries_preserve_coarse_and_zero_work_proofs() {
        for bits in [65, 4097, 65_537] {
            for reversed in [false, true] {
                let value = public_difference(bits, true, reversed);
                let low = EvaluationBudget::new(32);
                assert_eq!(value.sign_until_with_budget(0, &low), None);
                let generous = EvaluationBudget::new(1_000_000_000);
                assert_eq!(
                    value.sign_until_with_budget(0, &generous),
                    Some(expected(reversed))
                );
                let sample = value.approx_with_budget(-16, &generous).unwrap();
                // The difference has magnitude below 2^-16 at these widths.
                // A coarse sample must never erase its independently proved sign.
                assert!(sample.abs() <= BigInt::one());
                let zero = EvaluationBudget::new(0);
                assert_eq!(
                    value.clone().sign_until_with_budget(0, &zero),
                    Some(expected(reversed))
                );
                assert_eq!(value.approx_with_budget(-16, &zero), Ok(sample));
                assert_eq!(zero.consumed(), 0);
            }
        }
    }

    #[test]
    fn inverse_constructor_public_cancellation_remains_exact() {
        for bits in [65, 4097, 65_537] {
            let argument = || {
                Computable::rational(Rational::from_bigint((BigInt::one() << (bits - 1)) + 1))
                    .add(Computable::pi())
                    .atan()
            };
            let value = argument().add(argument().negate());
            assert_eq!(value.exact_rational(), Some(Rational::zero()));
            let zero = EvaluationBudget::new(0);
            assert_eq!(value.sign_until_with_budget(0, &zero), Some(RealSign::Zero));
            assert_eq!(zero.consumed(), 0);
        }
    }

    #[test]
    fn inverse_constructor_infallible_construction_retains_exact_leaves() {
        // Public constructors have no allowance result. Their exact leaf
        // folding must survive even if a surrounding private scope has failed;
        // bounded query helpers must propagate their own failure separately.
        for bits in [65, 4097, 65_537, 1_000_001] {
            let n = (BigInt::one() << (bits - 1)) + 1;
            let exact = Rational::from_bigint(n);
            let value = Computable::rational(exact.clone());
            let negative = Computable::rational(-exact.clone());
            let budget = EvaluationBudget::new(0);
            let negated;
            let cancelled;
            {
                let _scope = BudgetScope::enter(&budget);
                assert!(!charge_evaluation_work());
                negated = value.clone().negate();
                cancelled = value.add(negative);
                assert!(evaluation_exhausted());
            }
            assert_eq!(negated.exact_rational(), Some(-exact));
            assert_eq!(cancelled.exact_rational(), Some(Rational::zero()));
            assert_eq!(budget.consumed(), 0);
        }
    }

    fn opaque_offset_difference(bits: usize, kind: u8, reversed: bool) -> Computable {
        let argument = |bits| {
            let offset = if kind == 0 {
                Computable::rational(Rational::new(2)).ln()
            } else {
                Computable::rational(Rational::new(2)).sqrt()
            };
            let value =
                Computable::rational(Rational::from_bigint((BigInt::one() << (bits - 1)) + 1))
                    .add(offset)
                    .atan();
            assert!(matches!(
                value.internal.approximation,
                Approximation::AtanDeferred(_)
            ));
            value
        };
        let left = argument(bits);
        let right = argument(bits - 1);
        let value = if reversed {
            right.add(left.negate())
        } else {
            left.add(right.negate())
        };
        assert!(matches!(
            value.internal.approximation,
            Approximation::Add(_, _)
        ));
        assert_eq!(value.immediate_sign(), None);
        assert!(value.internal.cache_snapshot().is_none());
        value
    }

    #[test]
    fn inverse_constructor_public_opaque_preflight_stops_wide_offset_copies() {
        let mut observations = Vec::new();
        for kind in [0, 1] {
            for reversed in [false, true] {
                for units in [32, 48, 64] {
                    let control = opaque_offset_difference(8, kind, reversed);
                    let control_budget = EvaluationBudget::new(units);
                    let (_, control_allocations) =
                        measure(|| control.sign_until_with_budget(0, &control_budget));
                    for bits in [4097, 65_537, 1_000_001] {
                        let value = opaque_offset_difference(bits, kind, reversed);
                        let budget = EvaluationBudget::new(units);
                        let (answer, allocations) =
                            measure(|| value.sign_until_with_budget(0, &budget));
                        eprintln!(
                            "constructor public opaque kind={kind}, bits={bits}, reversed={reversed}, units={units}, answer={answer:?}, consumed={}, control={control_allocations:?}, allocations={allocations:?}",
                            budget.consumed()
                        );
                        assert!(answer.is_none_or(|answer| answer == expected(reversed)));
                        assert_eq!(budget.consumed() + budget.remaining(), units);
                        assert!(value.internal.cache_snapshot().is_none());
                        observations.push((allocations.bytes, control_allocations.bytes));
                    }
                }
            }
        }
        assert!(observations.iter().all(|(bytes, control)| bytes <= control));
    }

    #[test]
    fn inverse_constructor_opaque_retries_preserve_completed_proofs() {
        for kind in [0, 1] {
            for bits in [65, 4097, 65_537] {
                for reversed in [false, true] {
                    let value = opaque_offset_difference(bits, kind, reversed);
                    let low = EvaluationBudget::new(16);
                    assert_eq!(value.sign_until_with_budget(0, &low), None);
                    let generous = EvaluationBudget::new(1_000_000_000);
                    assert_eq!(
                        value.sign_until_with_budget(0, &generous),
                        Some(expected(reversed))
                    );
                    let sample = value.approx_with_budget(-16, &generous).unwrap();
                    assert!(sample.abs() <= BigInt::one());
                    let zero = EvaluationBudget::new(0);
                    assert_eq!(
                        value.clone().sign_until_with_budget(0, &zero),
                        Some(expected(reversed))
                    );
                    assert_eq!(value.approx_with_budget(-16, &zero), Ok(sample));
                    assert_eq!(zero.consumed(), 0);
                }
            }
        }
    }

    #[test]
    fn inverse_constructor_every_opaque_sign_cutpoint_preserves_retry() {
        for kind in [0, 1] {
            for bits in [8, 65, 256] {
                for reversed in [false, true] {
                    let generous = EvaluationBudget::new(1_000_000);
                    assert_eq!(
                        opaque_offset_difference(bits, kind, reversed)
                            .sign_until_with_budget(0, &generous),
                        Some(expected(reversed))
                    );
                    let needed = generous.consumed();
                    assert!(needed > 1 && needed < 4096);
                    eprintln!(
                        "constructor opaque cutpoints kind={kind}, bits={bits}, reversed={reversed}, needed={needed}"
                    );
                    for units in 0..=needed + 2 {
                        let value = opaque_offset_difference(bits, kind, reversed);
                        let budget = EvaluationBudget::new(units);
                        let answer = value.sign_until_with_budget(0, &budget);
                        assert_eq!(answer, (units >= needed).then(|| expected(reversed)));
                        assert_eq!(budget.consumed() + budget.remaining(), units);
                        if answer.is_none() {
                            assert_eq!(value.immediate_sign(), None);
                            assert!(value.internal.cache_snapshot().is_none());
                        }
                        assert_eq!(value.sign_until(0), Some(expected(reversed)));
                        let zero = EvaluationBudget::new(0);
                        assert_eq!(
                            value.clone().sign_until_with_budget(0, &zero),
                            Some(expected(reversed))
                        );
                        assert_eq!(zero.consumed(), 0);
                    }
                }
            }
        }
    }
}
