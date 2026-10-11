#[cfg(test)]
mod structural_comparison_budget_tests {
    use super::algebraic_leaf_tests::measure;
    use super::*;
    use crate::EvaluationBudget;
    use crate::computable::budget::BudgetScope;

    fn raw(kind: Approximation) -> Computable {
        Computable::raw_periodic_operand(kind)
    }

    fn integer(bits: usize, different: bool) -> BigInt {
        let value = (BigInt::one() << (bits - 1)) + BigInt::one();
        if different {
            value + (BigInt::one() << (bits - 2))
        } else {
            value
        }
    }

    fn fraction(bits: usize, different: bool) -> Rational {
        let value = BigUint::one() << (bits - 1);
        Rational::from_bigint_fraction(
            BigInt::from(&value + if different { 2_u8 } else { 1_u8 }),
            &value + if different { 4_u8 } else { 3_u8 },
        )
        .unwrap()
    }

    fn lazy_fraction(bits: usize, different: bool) -> Rational {
        let factor = (BigUint::one() << (bits - 1)) + BigUint::one();
        Rational::from_parts_raw_unreduced(
            Sign::Plus,
            &factor * if different { 5_u8 } else { 3_u8 },
            &factor * 7_u8,
        )
    }

    fn rational_kind(kind: usize, value: Rational) -> Approximation {
        match kind {
            0 => Approximation::Ratio(value),
            1 => Approximation::PrescaledLnRational(value),
            2 => Approximation::AtanRational(value),
            3 => Approximation::AsinRational(value),
            4 => Approximation::AcosPositiveRational(value),
            5 => Approximation::AcosNegativeRational(value),
            6 => Approximation::AsinhRational(value),
            7 => Approximation::AtanhRational(value),
            8 => Approximation::PrescaledCosRational(value),
            9 => Approximation::CosLargeRational(value),
            10 => Approximation::PrescaledCosHalfPiMinusRational(value),
            11 => Approximation::PrescaledSinRational(value),
            12 => Approximation::SinLargeRational(value),
            13 => Approximation::PrescaledSinHalfPiMinusRational(value),
            14 => Approximation::PrescaledCotHalfPiMinusRational(value),
            15 => Approximation::TanLargeRational(value),
            16 => Approximation::PrescaledTanRational(value),
            _ => unreachable!(),
        }
    }

    fn compare(left: &Computable, right: &Computable, units: u64) -> (Option<bool>, u64, bool) {
        let budget = EvaluationBudget::new(units);
        let (answer, exhausted) = {
            let _scope = BudgetScope::enter(&budget);
            (
                Computable::budgeted_structural_eq(left, right),
                evaluation_exhausted(),
            )
        };
        assert_eq!(budget.consumed() + budget.remaining(), units);
        (answer, budget.consumed(), exhausted)
    }

    #[test]
    fn wide_structural_integer_comparison_requires_payload_work() {
        let mut observations = Vec::new();
        for bits in [65, 4097, 65_537, 1_000_001] {
            for different in [false, true] {
                for negative in [false, true] {
                    for atan in [false, true] {
                        let leaf = |different| {
                            let value = integer(bits, different);
                            let value = if negative { -value } else { value };
                            raw(if atan {
                                Approximation::IntegralAtan(value)
                            } else {
                                Approximation::Int(value)
                            })
                        };
                        let left = leaf(false);
                        let right = leaf(different);
                        assert_eq!(
                            Computable::internal_structural_eq(&left, &right),
                            !different
                        );
                        let result = compare(&left, &right, 1);
                        eprintln!(
                            "wide integer bits={bits}, different={different}, negative={negative}, atan={atan}, result={result:?}"
                        );
                        observations.push(result);
                    }
                }
            }
        }
        assert!(observations.iter().all(|r| *r == (None, 1, true)));
    }

    #[test]
    fn wide_structural_rational_variants_require_payload_work() {
        let mut observations = Vec::new();
        for bits in [65, 4097, 65_537, 1_000_001] {
            for different in [false, true] {
                let left = fraction(bits, false);
                let right = fraction(bits, different);
                assert_ne!(left.storage_identity(), right.storage_identity());
                for kind in 0..17 {
                    let left = raw(rational_kind(kind, left.clone()));
                    let right = raw(rational_kind(kind, right.clone()));
                    let result = compare(&left, &right, 1);
                    eprintln!(
                        "wide rational bits={bits}, different={different}, kind={kind}, result={result:?}"
                    );
                    observations.push(result);
                }
            }
        }
        assert!(observations.iter().all(|r| *r == (None, 1, true)));
    }

    #[test]
    fn wide_structural_scaled_log_payload_requires_work() {
        for bits in [65, 4097, 65_537, 1_000_001] {
            for different in [false, true] {
                let leaf = |different| {
                    raw(Approximation::BinaryScaledLnRational {
                        residual: fraction(bits, different),
                        shift: 7,
                    })
                };
                let result = compare(&leaf(false), &leaf(different), 1);
                eprintln!("wide log bits={bits}, different={different}, result={result:?}");
                assert_eq!(result, (None, 1, true));
            }
        }
    }

    #[test]
    fn wide_structural_quantile_seed_requires_work() {
        let probability = Computable::rational(Rational::fraction(1, 3).unwrap());
        for bits in [65, 4097, 65_537, 1_000_001] {
            for different in [false, true] {
                let leaf = |different| {
                    raw(Approximation::NormalQuantile(Box::new(
                        NormalQuantileData {
                            p: probability.clone(),
                            seed: integer(bits, different),
                            seed_prec: -16,
                        },
                    )))
                };
                let result = compare(&leaf(false), &leaf(different), 1);
                eprintln!("wide seed bits={bits}, different={different}, result={result:?}");
                assert_eq!(result, (None, 1, true));
            }
        }
    }

    #[test]
    fn wide_structural_identity_and_cheap_rejections_remain_available() {
        for bits in [65, 4097, 65_537, 1_000_001] {
            let integer = integer(bits, false);
            let left = raw(Approximation::Int(integer.clone()));
            assert_eq!(compare(&left, &left, 0), (Some(true), 0, false));
            let zero = EvaluationBudget::new(0);
            {
                let _scope = BudgetScope::enter(&zero);
                assert!(!charge_evaluation_work());
                assert_eq!(Computable::budgeted_structural_eq(&left, &left), Some(true));
                assert!(Computable::internal_structural_eq(&left, &left));
            }
            for right in [
                raw(Approximation::Int(-integer.clone())),
                raw(Approximation::Int(integer << 1)),
                raw(Approximation::One),
            ] {
                assert_eq!(compare(&left, &right, 1), (Some(false), 1, false));
            }
            let value = fraction(bits, false);
            for kind in 0..17 {
                let left = raw(rational_kind(kind, value.clone()));
                let same = raw(rational_kind(kind, value.clone()));
                let opposite = raw(rational_kind(kind, -value.clone()));
                assert_eq!(compare(&left, &same, 1), (Some(true), 1, false));
                assert_eq!(compare(&left, &opposite, 1), (Some(false), 1, false));
            }
        }
    }

    #[test]
    fn wide_structural_lazy_payload_stops_before_reduction() {
        for bits in [65, 4097, 65_537, 1_000_001] {
            let left = raw(Approximation::Ratio(lazy_fraction(bits, false)));
            let right = raw(Approximation::Ratio(lazy_fraction(bits, false)));
            let budget = EvaluationBudget::new(1);
            let _scope = BudgetScope::enter(&budget);
            let (answer, allocations) =
                measure(|| Computable::budgeted_structural_eq(&left, &right));
            eprintln!(
                "lazy comparison bits={bits}, answer={answer:?}, allocations={allocations:?}"
            );
            assert_eq!(answer, None);
            assert!(evaluation_exhausted());
            assert_eq!((allocations.calls, allocations.bytes), (0, 0));
        }
    }

    #[test]
    fn wide_structural_symbolic_read_does_not_canonicalize_before_charge() {
        let mut observations = Vec::new();
        for bits in [65, 4097, 65_537, 1_000_001] {
            let value = lazy_fraction(bits, false);
            let budget = EvaluationBudget::new(1);
            let _scope = BudgetScope::enter(&budget);
            let (answer, allocations) = measure(|| SymbolicWork { metered: true }.read(&value));
            eprintln!("lazy read bits={bits}, answer={answer:?}, allocations={allocations:?}");
            observations.push((answer, allocations, evaluation_exhausted()));
        }
        assert!(observations.iter().all(|(answer, allocation, exhausted)| {
            answer.is_none() && *exhausted && allocation.calls == 0 && allocation.bytes == 0
        }));
    }

    #[test]
    fn wide_structural_cross_product_stops_before_allocation() {
        let mut observations = Vec::new();
        for bits in [65, 4097, 65_537, 1_000_001] {
            let left = raw(Approximation::Ratio(fraction(bits, false)));
            let right = raw(Approximation::Ratio(fraction(bits, true)));
            let budget = EvaluationBudget::new(1);
            let _scope = BudgetScope::enter(&budget);
            let (answer, allocations) =
                measure(|| Computable::budgeted_structural_eq(&left, &right));
            eprintln!("cross product bits={bits}, answer={answer:?}, allocations={allocations:?}");
            observations.push((answer, allocations, evaluation_exhausted()));
        }
        assert!(observations.iter().all(|(answer, allocation, exhausted)| {
            answer.is_none() && *exhausted && allocation.calls == 0 && allocation.bytes == 0
        }));
    }

    #[test]
    fn wide_structural_metadata_rejection_avoids_payload_allocation() {
        for bits in [65, 4097, 65_537, 1_000_001] {
            let left = raw(Approximation::BinaryScaledLnRational {
                residual: fraction(bits, false),
                shift: 7,
            });
            let right = raw(Approximation::BinaryScaledLnRational {
                residual: fraction(bits, true),
                shift: 8,
            });
            let budget = EvaluationBudget::new(1);
            let _scope = BudgetScope::enter(&budget);
            let (answer, allocations) =
                measure(|| Computable::budgeted_structural_eq(&left, &right));
            eprintln!(
                "shift rejection bits={bits}, answer={answer:?}, allocations={allocations:?}"
            );
            assert_eq!(answer, Some(false));
            assert!(!evaluation_exhausted());
            assert_eq!((allocations.calls, allocations.bytes), (0, 0));
        }
    }

    fn tree(kind: usize, different: bool, depth: usize) -> Computable {
        let mut value = raw(match kind {
            0 => Approximation::Int(integer(256, different)),
            1 => Approximation::Ratio(fraction(256, different)),
            _ => Approximation::Ratio(lazy_fraction(256, different)),
        });
        for _ in 0..depth {
            value = raw(Approximation::Offset(value, 0));
        }
        value
    }

    #[test]
    fn wide_structural_every_cutpoint_preserves_graphs_and_retry() {
        for kind in 0..3 {
            for different in [false, true] {
                for depth in [0, 80] {
                    let left = tree(kind, false, depth);
                    let right = tree(kind, different, depth);
                    let result = compare(&left, &right, 100_000);
                    assert_eq!(result.0, Some(!different));
                    assert!(!result.2);
                    let needed = result.1;
                    assert!(needed > 0 && needed < 4096);
                    eprintln!(
                        "comparison cutpoints kind={kind}, different={different}, depth={depth}, needed={needed}"
                    );
                    for units in 0..=needed + 2 {
                        let left = tree(kind, false, depth);
                        let right = tree(kind, different, depth);
                        let facts = (
                            left.internal.facts.snapshot(),
                            right.internal.facts.snapshot(),
                        );
                        let answer = compare(&left, &right, units);
                        assert_eq!(answer.0, (units >= needed).then_some(!different));
                        assert_eq!(answer.2, units < needed);
                        assert_eq!(
                            (
                                left.internal.facts.snapshot(),
                                right.internal.facts.snapshot()
                            ),
                            facts
                        );
                        assert!(left.internal.cache_snapshot().is_none());
                        assert!(right.internal.cache_snapshot().is_none());
                        let exhausted = EvaluationBudget::new(0);
                        let _scope = BudgetScope::enter(&exhausted);
                        assert!(!charge_evaluation_work());
                        assert_eq!(
                            Computable::internal_structural_eq(&left, &right),
                            !different
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn wide_structural_public_sign_keeps_completed_proof_at_zero_work() {
        for bits in [4097, 65_537] {
            let atom = || {
                raw(Approximation::AtanDeferred(raw(Approximation::Ratio(
                    fraction(bits, false),
                ))))
            };
            let value = raw(Approximation::Add(
                atom(),
                raw(Approximation::Negate(atom())),
            ));
            assert_eq!(value.immediate_sign(), None);
            let low = EvaluationBudget::new(32);
            let answer = value.sign_until_with_budget(0, &low);
            eprintln!(
                "public comparison bits={bits}, low_answer={answer:?}, consumed={}",
                low.consumed()
            );
            assert_eq!(answer, None);
            assert_eq!(value.immediate_sign(), None);
            assert!(value.internal.cache_snapshot().is_none());
            let generous = EvaluationBudget::new(1_000_000_000);
            assert_eq!(
                value.sign_until_with_budget(0, &generous),
                Some(RealSign::Zero)
            );
            eprintln!(
                "public comparison bits={bits}, successful work={}",
                generous.consumed()
            );
            let zero = EvaluationBudget::new(0);
            assert_eq!(value.sign_until_with_budget(0, &zero), Some(RealSign::Zero));
            assert_eq!(zero.consumed(), 0);
        }
    }

    #[test]
    fn wide_structural_shared_dags_preserve_equality() {
        fn diamond(different: bool) -> Computable {
            let mut value = tree(0, different, 0);
            for _ in 0..24 {
                value = raw(Approximation::Add(
                    raw(Approximation::Offset(value.clone(), 0)),
                    value,
                ));
            }
            value
        }
        for different in [false, true] {
            let left = diamond(false);
            let right = diamond(different);
            assert_eq!(
                Computable::internal_structural_eq(&left, &right),
                !different
            );
            let result = compare(&left, &right, 10_000);
            assert_eq!(result.0, Some(!different));
            assert!(!result.2);
            assert!(left.internal.cache_snapshot().is_none());
            assert!(right.internal.cache_snapshot().is_none());
        }
    }
}
