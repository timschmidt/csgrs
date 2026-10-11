#[cfg(test)]
mod algebraic_budget_tests {
    use super::*;
    use crate::EvaluationBudget;
    use crate::computable::budget::BudgetScope;

    fn raw(kind: Approximation) -> Computable {
        Computable::raw_periodic_operand(kind)
    }

    fn root(value: Rational, degree: u32) -> Computable {
        let value = Computable::rational(value);
        raw(if degree == 2 {
            Approximation::Sqrt(value)
        } else {
            Approximation::NthRoot(value, degree)
        })
    }

    fn identity(degree: u32, count: usize, inverse: bool, perturbation: i32) -> Computable {
        let mut product = Rational::one();
        let mut roots = Computable::one();
        for prime in [2, 3, 5].into_iter().take(count) {
            product *= Rational::new(prime);
            roots = raw(Approximation::Multiply(roots, root(Rational::new(prime), degree)));
        }
        let mut combined = root(product, degree);
        if inverse {
            combined = raw(Approximation::Inverse(combined));
            roots = raw(Approximation::Inverse(roots));
        }
        let difference = raw(Approximation::Add(combined, raw(Approximation::Negate(roots))));
        let delta = Rational::from_bigint_fraction(
            BigInt::from(perturbation),
            BigUint::one() << 256,
        ).unwrap();
        raw(Approximation::Add(difference, Computable::rational(delta)))
    }

    #[test]
    fn algebraic_budget_metadata_interruptions_preserve_facts_and_retry() {
        let expected = identity(2, 2, false, 0).algebraic_separation_bound_bits().unwrap();
        let generous = EvaluationBudget::new(100_000);
        {
            let _scope = BudgetScope::enter(&generous);
            assert_eq!(identity(2, 2, false, 0).algebraic_separation_bound_bits(), Some(expected));
        }
        let needed = generous.consumed();
        assert!(needed > 0 && needed < 4096, "metadata must charge shared work");
        let mut interruptions = 0;
        let mut completed = 0;
        for units in 0..=needed + 2 {
            let value = identity(2, 2, false, 0);
            let allowance = EvaluationBudget::new(units);
            {
                let _scope = BudgetScope::enter(&allowance);
                match value.algebraic_separation_bound_bits() {
                    Some(bound) => {
                        assert_eq!(bound, expected);
                        assert!(!evaluation_exhausted());
                        completed += 1;
                    }
                    None => {
                        assert!(evaluation_exhausted());
                        interruptions += 1;
                    }
                }
                assert!(value.internal.cache_snapshot().is_none());
                assert!(matches!(value.internal.facts.snapshot(),
                    (BoundCache::Invalid, ExactSignCache::Invalid)));
            }
            assert_eq!(allowance.consumed() + allowance.remaining(), units);
            assert_eq!(value.algebraic_separation_bound_bits(), Some(expected));
            assert_eq!(value.sign_until(-1024), Some(RealSign::Zero));
            assert_eq!(value.sign_until_with_budget(0, &EvaluationBudget::new(0)), Some(RealSign::Zero));
        }
        assert!(interruptions > 0 && completed > 0);
    }

    fn seeded(left: Rational, right: Rational) -> AlgebraicSeparation {
        AlgebraicSeparation {
            denominator_log2: 0,
            conjugate_log2: 0,
            generators: vec![
                AlgebraicGenerator { root: root(left, 2), degree: 2 },
                AlgebraicGenerator { root: root(right, 2), degree: 2 },
            ],
        }
    }

    #[test]
    fn algebraic_budget_dependency_search_charges_coefficient_growth() {
        let mut costs = Vec::new();
        for bits in [16, 256, 1024] {
            let left = Rational::from_bigint((BigInt::one() << bits) + 1);
            let right = Rational::from_bigint((BigInt::one() << bits) + 3);
            let dependent = root(&left * &right, 2);
            let generous = EvaluationBudget::new(1_000_000);
            {
                let _scope = BudgetScope::enter(&generous);
                let mut metadata = seeded(left.clone(), right.clone());
                assert_eq!(metadata.insert_generator(dependent.clone(), 2), Some(()));
                assert_eq!(metadata.generators.len(), 2);
            }
            let needed = generous.consumed();
            assert!(needed > 0, "dependency search must charge shared work");
            costs.push(needed);
            for units in [0, 1, needed / 4, needed - 1, needed, needed + 1] {
                let mut metadata = seeded(left.clone(), right.clone());
                let allowance = EvaluationBudget::new(units);
                {
                    let _scope = BudgetScope::enter(&allowance);
                    let answer = metadata.insert_generator(dependent.clone(), 2);
                    assert_eq!(answer.is_some(), units >= needed);
                    assert_eq!(evaluation_exhausted(), units < needed);
                    assert_eq!(metadata.generators.len(), 2, "interruption cannot declare independence");
                }
                assert_eq!(allowance.consumed() + allowance.remaining(), units);
                assert_eq!(metadata.insert_generator(dependent.clone(), 2), Some(()));
                assert_eq!(metadata.generators.len(), 2);
            }
        }
        assert!(costs.windows(2).all(|pair| pair[0] < pair[1]), "wider coefficients need more allowance: {costs:?}");
        let mut independent = seeded(Rational::new(2), Rational::new(3));
        assert_eq!(independent.insert_generator(root(Rational::new(5), 2), 2), Some(()));
        assert_eq!(independent.generators.len(), 3);
    }

    #[test]
    fn algebraic_budget_generator_comparison_can_decline_without_inequality() {
        let deep = || {
            let mut value = Computable::rational(Rational::new(2));
            for _ in 0..512 {
                value = raw(Approximation::Offset(value, 0));
            }
            raw(Approximation::Sqrt(value))
        };
        let left = deep();
        let right = deep();
        let mut metadata = AlgebraicSeparation {
            denominator_log2: 0,
            conjugate_log2: 1,
            generators: vec![AlgebraicGenerator { root: left.clone(), degree: 2 }],
        };
        {
            let _scope = BudgetScope::enter(&EvaluationBudget::new(20));
            assert_eq!(metadata.insert_generator(right.clone(), 2), None);
            assert!(evaluation_exhausted());
            assert_eq!(metadata.generators.len(), 1);
            assert!(Computable::internal_structural_eq(&left, &right));
        }
        {
            let _scope = BudgetScope::enter(&EvaluationBudget::new(10_000));
            assert_eq!(metadata.insert_generator(right, 2), Some(()));
            assert_eq!(metadata.generators.len(), 1);
        }
    }

    #[test]
    fn algebraic_budget_public_queries_preserve_exact_perturbations_and_proofs() {
        for degree in [2, 3, 4] {
            for count in [2, 3] {
                for inverse in [false, true] {
                    for (perturbation, expected) in [(-1, RealSign::Negative), (0, RealSign::Zero), (1, RealSign::Positive)] {
                        for units in [0, 4, 16, 256, 100_000] {
                            let value = identity(degree, count, inverse, perturbation);
                            let exact = BigInt::from(perturbation) << 768;
                            assert!((value.approx(-1024).unwrap() - &exact).abs() <= BigInt::one());
                            let previous = value.immediate_sign();
                            let allowance = EvaluationBudget::new(units);
                            let answer = value.sign_until_with_budget(-1024, &allowance);
                            if let Some(answer) = answer {
                                assert_eq!(answer, expected);
                            } else {
                                assert_eq!(value.immediate_sign(), None);
                            }
                            if let Some(previous) = previous {
                                assert_eq!(answer, Some(previous));
                            }
                            assert_eq!(allowance.consumed() + allowance.remaining(), units);
                            assert_eq!(value.sign_until(-1024), Some(expected));
                            let alias = value.clone();
                            let spent = EvaluationBudget::new(0);
                            assert_eq!(alias.sign_until_with_budget(0, &spent), Some(expected));
                            assert!((alias.approx_with_budget(-1024, &spent).unwrap() - exact).abs() <= BigInt::one());
                            assert_eq!(spent.consumed(), 0);
                        }
                    }
                }
            }
        }
    }
}
