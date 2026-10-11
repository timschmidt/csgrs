#[cfg(test)]
mod algebraic_preflight_tests {
    use super::algebraic_leaf_tests::measure;
    use super::*;
    use crate::EvaluationBudget;
    use crate::computable::budget::BudgetScope;

    fn raw(kind: Approximation) -> Computable {
        Computable::raw_periodic_operand(kind)
    }

    fn lazy(bits: usize, numerator: u8, denominator: u8, negative: bool) -> Rational {
        let factor = (BigUint::one() << (bits - 1)) + BigUint::one();
        Rational::from_parts_raw_unreduced(
            if negative { Sign::Minus } else { Sign::Plus },
            &factor * numerator,
            factor * denominator,
        )
    }

    fn root(value: Rational, degree: u32) -> Computable {
        let value = raw(Approximation::Ratio(value));
        raw(if degree == 2 {
            Approximation::Sqrt(value)
        } else {
            Approximation::NthRoot(value, degree)
        })
    }

    fn empty() -> AlgebraicSeparation {
        AlgebraicSeparation {
            denominator_log2: 0,
            conjugate_log2: 0,
            generators: Vec::new(),
        }
    }

    fn shape(value: AlgebraicSeparation) -> (u64, u64, Option<u64>, Option<u64>) {
        (
            value.denominator_log2,
            value.conjugate_log2,
            value.field_degree(),
            value.separation_bits(),
        )
    }

    #[test]
    fn algebraic_preflight_lazy_metadata_stops_before_reduction() {
        let mut observations = Vec::new();
        for bits in [65, 4097, 65_537, 1_000_001] {
            for negative in [false, true] {
                let value = lazy(bits, 3, 7, negative);
                let budget = EvaluationBudget::new(1);
                let _scope = BudgetScope::enter(&budget);
                let (answer, allocations) =
                    measure(|| AlgebraicSeparation::rational_with_work(&value, true).map(shape));
                eprintln!(
                    "lazy metadata bits={bits}, negative={negative}, answer={answer:?}, allocations={allocations:?}"
                );
                observations.push((answer, allocations, evaluation_exhausted()));
                assert_eq!(budget.consumed() + budget.remaining(), 1);
            }
        }
        assert!(observations.iter().all(|(answer, allocations, exhausted)| {
            answer.is_none() && *exhausted && allocations.calls == 0 && allocations.bytes == 0
        }));
    }

    #[test]
    fn algebraic_preflight_lazy_candidate_stops_before_reduction() {
        let mut observations = Vec::new();
        for bits in [65, 4097, 65_537, 1_000_001] {
            for degree in [2, 3, 4] {
                let value = root(lazy(bits, 3, 7, false), degree);
                let budget = EvaluationBudget::new(1);
                let _scope = BudgetScope::enter(&budget);
                let (answer, allocations) =
                    measure(|| empty().rational_root_depends_on_existing(&value, degree));
                eprintln!(
                    "lazy candidate bits={bits}, degree={degree}, answer={answer:?}, allocations={allocations:?}"
                );
                observations.push((answer, allocations, evaluation_exhausted()));
                assert_eq!(budget.consumed(), 1);
            }
        }
        assert!(observations.iter().all(|(answer, allocations, exhausted)| {
            answer.is_none() && *exhausted && allocations.calls == 0 && allocations.bytes == 0
        }));
    }

    #[test]
    fn algebraic_preflight_lazy_base_stops_before_reduction() {
        let mut observations = Vec::new();
        for bits in [65, 4097, 65_537, 1_000_001] {
            for degree in [2, 3, 4] {
                let candidate = root(Rational::new(5), degree);
                let metadata = AlgebraicSeparation {
                    generators: vec![AlgebraicGenerator {
                        root: root(lazy(bits, 3, 7, false), degree),
                        degree,
                    }],
                    ..empty()
                };
                let budget = EvaluationBudget::new(3);
                let _scope = BudgetScope::enter(&budget);
                let (answer, allocations) =
                    measure(|| metadata.rational_root_depends_on_existing(&candidate, degree));
                eprintln!(
                    "lazy base bits={bits}, degree={degree}, answer={answer:?}, allocations={allocations:?}"
                );
                observations.push((answer, allocations, evaluation_exhausted()));
                assert_eq!(budget.consumed(), 3);
            }
        }
        assert!(observations.iter().all(|(answer, allocations, exhausted)| {
            answer.is_none() && *exhausted && allocations.calls == 0 && allocations.bytes == 0
        }));
    }

    #[test]
    fn algebraic_preflight_coefficient_charge_inspects_only_storage() {
        let mut observations = Vec::new();
        for bits in [65, 4097, 65_537, 1_000_001] {
            for side in 0..3 {
                for units in [1, u64::MAX] {
                    let left = if side != 1 {
                        lazy(bits, 3, 7, false)
                    } else {
                        Rational::one()
                    };
                    let right = if side != 0 {
                        lazy(bits, 5, 11, false)
                    } else {
                        Rational::one()
                    };
                    let budget = EvaluationBudget::new(units);
                    let _scope = BudgetScope::enter(&budget);
                    let (answer, allocations) =
                        measure(|| AlgebraicSeparation::charge_coefficient_work(&left, &right));
                    eprintln!(
                        "lazy coefficient bits={bits}, side={side}, units={units}, answer={answer:?}, consumed={}, allocations={allocations:?}",
                        budget.consumed()
                    );
                    observations.push((
                        answer,
                        allocations,
                        evaluation_exhausted(),
                        units,
                        budget.consumed(),
                    ));
                    assert_eq!(budget.consumed() + budget.remaining(), units);
                }
            }
        }
        assert!(
            observations
                .iter()
                .all(|(answer, allocations, exhausted, units, consumed)| {
                    answer.is_some() == (*units == u64::MAX)
                        && *exhausted == (*units == 1)
                        && (*units == 1 || *consumed > 1)
                        && allocations.calls == 0
                        && allocations.bytes == 0
                })
        );
    }

    #[test]
    fn algebraic_preflight_zero_allowance_allocates_no_metadata_vector() {
        let mut deep = raw(Approximation::Ratio(Rational::fraction(3, 7).unwrap()));
        for _ in 0..80 {
            deep = raw(Approximation::Offset(deep, 0));
        }
        let values = [
            raw(Approximation::One),
            raw(Approximation::Int(BigInt::from(7))),
            raw(Approximation::Ratio(Rational::fraction(3, 7).unwrap())),
            raw(Approximation::SinDeferred(Computable::one())),
            deep,
        ];
        let mut observations = Vec::new();
        for (kind, value) in values.into_iter().enumerate() {
            let facts = value.internal.facts.snapshot();
            let budget = EvaluationBudget::new(0);
            let _scope = BudgetScope::enter(&budget);
            let (answer, allocations) = measure(|| value.algebraic_separation_bound_bits());
            eprintln!("empty metadata kind={kind}, answer={answer:?}, allocations={allocations:?}");
            assert_eq!(answer, None);
            assert!(evaluation_exhausted());
            assert_eq!(budget.consumed(), 0);
            assert_eq!(value.internal.facts.snapshot(), facts);
            assert!(value.internal.cache_snapshot().is_none());
            observations.push(allocations);
        }
        assert!(observations.iter().all(|a| a.calls == 0 && a.bytes == 0));
    }

    #[test]
    fn algebraic_preflight_generous_work_preserves_reduced_metadata() {
        for bits in [65, 4097, 65_537, 1_000_001] {
            for negative in [false, true] {
                for metered in [false, true] {
                    let value = lazy(bits, 3, 7, negative);
                    let budget = EvaluationBudget::new(1_000_000_000);
                    let _scope = BudgetScope::enter(&budget);
                    assert_eq!(
                        AlgebraicSeparation::rational_with_work(&value, metered).map(shape),
                        Some((3, 2, Some(1), Some(3)))
                    );
                    assert!(!evaluation_exhausted());
                    if metered {
                        assert!(budget.consumed() > 1);
                    } else {
                        assert_eq!(budget.consumed(), 0);
                    }
                }
            }
        }
    }

    #[test]
    fn algebraic_preflight_cheap_rejections_keep_their_work_contract() {
        for bits in [131_074, 1_000_001] {
            let value =
                Rational::from_bigint_fraction(BigInt::one() << (bits - 1), BigUint::from(3_u8))
                    .unwrap();
            let budget = EvaluationBudget::new(0);
            let _scope = BudgetScope::enter(&budget);
            let (answer, allocations) =
                measure(|| AlgebraicSeparation::rational_with_work(&value, true));
            assert!(answer.is_none());
            assert!(!evaluation_exhausted());
            assert_eq!((allocations.calls, allocations.bytes), (0, 0));
        }
        for bits in [65, 4097, 65_537, 1_000_001] {
            for (negative, actual_degree) in [(true, 2), (false, 3)] {
                let value = root(lazy(bits, 3, 7, negative), actual_degree);
                let budget = EvaluationBudget::new(1);
                let _scope = BudgetScope::enter(&budget);
                let (answer, allocations) =
                    measure(|| empty().rational_root_depends_on_existing(&value, 2));
                assert_eq!(answer, Some(false));
                assert!(!evaluation_exhausted());
                assert_eq!(budget.consumed(), 1);
                assert_eq!((allocations.calls, allocations.bytes), (0, 0));
            }
        }
    }

    #[test]
    fn algebraic_preflight_lazy_logarithm_boundaries_match_canonical_values() {
        for denominator in [false, true] {
            for just_above in [false, true] {
                for negative in [false, true] {
                    let high = (BigUint::one() << 131_072) + u8::from(just_above);
                    let (n, d) = if denominator {
                        (BigUint::one(), high)
                    } else {
                        (high, BigUint::one())
                    };
                    let sign = if negative { Sign::Minus } else { Sign::Plus };
                    let canonical = Rational::from_bigint_fraction(
                        BigInt::from_biguint(sign, n.clone()),
                        d.clone(),
                    )
                    .unwrap();
                    let expected =
                        AlgebraicSeparation::rational_with_work(&canonical, false).map(shape);
                    assert_eq!(expected.is_some(), !just_above);
                    for bits in [65, 4097] {
                        let factor = (BigUint::one() << (bits - 1)) + BigUint::one();
                        let value =
                            Rational::from_parts_raw_unreduced(sign, &n * &factor, &d * factor);
                        let budget = EvaluationBudget::new(1_000_000_000);
                        let _scope = BudgetScope::enter(&budget);
                        assert_eq!(
                            AlgebraicSeparation::rational_with_work(&value, true).map(shape),
                            expected
                        );
                        assert!(!evaluation_exhausted());
                    }
                }
            }
        }
    }

    #[test]
    fn algebraic_preflight_every_metadata_cutpoint_preserves_retry() {
        for bits in [65, 256, 4097] {
            for depth in [0, 80] {
                let fixture = || {
                    let mut value = raw(Approximation::Ratio(lazy(bits, 3, 7, false)));
                    for _ in 0..depth {
                        value = raw(Approximation::Offset(value, 0));
                    }
                    value
                };
                let generous = EvaluationBudget::new(100_000);
                {
                    let _scope = BudgetScope::enter(&generous);
                    assert_eq!(fixture().algebraic_separation_bound_bits(), Some(3));
                }
                let needed = generous.consumed();
                assert!(needed > 1 && needed < 4096);
                eprintln!("lazy metadata cutpoints bits={bits}, depth={depth}, needed={needed}");
                for units in 0..=needed + 2 {
                    let value = fixture();
                    let facts = value.internal.facts.snapshot();
                    let budget = EvaluationBudget::new(units);
                    {
                        let _scope = BudgetScope::enter(&budget);
                        assert_eq!(
                            value.algebraic_separation_bound_bits(),
                            (units >= needed).then_some(3)
                        );
                        assert_eq!(evaluation_exhausted(), units < needed);
                    }
                    assert_eq!(budget.consumed() + budget.remaining(), units);
                    assert_eq!(value.internal.facts.snapshot(), facts);
                    assert!(value.internal.cache_snapshot().is_none());
                    assert_eq!(value.algebraic_separation_bound_bits(), Some(3));
                }
            }
        }
    }

    #[test]
    fn algebraic_preflight_public_retries_keep_completed_zero_proofs() {
        for bits in [65, 4097, 65_537] {
            for degree in [2, 3, 4] {
                for units in [0, 8, 128] {
                    let value = raw(Approximation::Add(
                        root(lazy(bits, 6, 1, false), degree),
                        raw(Approximation::Negate(raw(Approximation::Multiply(
                            root(lazy(bits, 2, 1, false), degree),
                            root(lazy(bits, 3, 1, false), degree),
                        )))),
                    ));
                    let low = EvaluationBudget::new(units);
                    if let Some(sign) = value.sign_until_with_budget(-1024, &low) {
                        assert_eq!(sign, RealSign::Zero);
                    }
                    assert_eq!(low.consumed() + low.remaining(), units);
                    let generous = EvaluationBudget::new(1_000_000_000);
                    assert_eq!(
                        value.sign_until_with_budget(-1024, &generous),
                        Some(RealSign::Zero)
                    );
                    let zero = EvaluationBudget::new(0);
                    assert_eq!(
                        value.clone().sign_until_with_budget(0, &zero),
                        Some(RealSign::Zero)
                    );
                    assert_eq!(zero.consumed(), 0);
                    assert!(value.approx(-1024).unwrap().abs() <= BigInt::one());
                }
            }
        }
    }
}
