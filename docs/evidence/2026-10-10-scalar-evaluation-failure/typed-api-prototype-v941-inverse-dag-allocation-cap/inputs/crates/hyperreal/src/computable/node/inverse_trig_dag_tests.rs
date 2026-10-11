#[cfg(test)]
mod inverse_trig_dag_tests {
    use super::*;
    use crate::EvaluationBudget;
    use crate::computable::budget::BudgetScope;

    fn raw(kind: Approximation) -> Computable {
        Computable::raw_periodic_operand(kind)
    }

    fn atom(kind: usize) -> Computable {
        if kind == 2 {
            raw(Approximation::AcosPositive(Computable::rational(
                Rational::fraction(1, 3).unwrap(),
            )))
        } else {
            raw(Approximation::AtanDeferred(Computable::rational(
                Rational::new(2),
            )))
        }
    }

    fn add(left: Computable, right: Computable) -> Computable {
        raw(Approximation::Add(left, right))
    }

    fn doubling(kind: usize, depth: usize) -> Computable {
        let mut value = atom(kind);
        for _ in 0..depth {
            value = add(value.clone(), value);
        }
        value
    }

    fn diamond(kind: usize, depth: usize) -> Computable {
        let mut value = atom(kind);
        for _ in 0..depth {
            value = add(raw(Approximation::Offset(value.clone(), 1)), value);
        }
        value
    }

    // These fixtures contain exactly one inverse-trig argument and no rational
    // or pi offset. Validate the complete form, then return its coefficient.
    fn coefficient(kind: usize, value: &Computable, depth: usize) -> Option<Rational> {
        let (constant, pi, coefficient, argument) = match kind {
            0 => {
                let f = value.pi_atan_linear_form(depth)?;
                (f.constant, f.pi, f.atan, f.argument?)
            }
            1 => {
                let mut f = value.pi_atan_terms_form(depth)?;
                assert_eq!(f.terms.len(), 1);
                let (argument, coefficient) = f.terms.pop().unwrap();
                (f.constant, f.pi, coefficient, argument)
            }
            _ => {
                let f = value.pi_acos_linear_form(depth)?;
                (f.constant, f.pi, f.acos, f.argument?)
            }
        };
        assert_eq!(constant, Rational::zero());
        assert_eq!(pi, Rational::zero());
        assert_eq!(
            argument.exact_rational(),
            Some(if kind == 2 {
                Rational::fraction(1, 3).unwrap()
            } else {
                Rational::new(2)
            })
        );
        Some(coefficient)
    }

    fn ordinary_doubling(kind: usize) {
        let value = doubling(kind, 28);
        let facts = value.internal.facts.snapshot();
        assert_eq!(
            coefficient(kind, &value, 32),
            Some(Rational::from_bigint(BigInt::one() << 28))
        );
        assert!(value.internal.cache_snapshot().is_none());
        assert_eq!(value.internal.facts.snapshot(), facts);
    }

    #[test]
    fn inverse_trig_dag_ordinary_atan_linear_preserves_coefficient() {
        ordinary_doubling(0);
    }

    #[test]
    fn inverse_trig_dag_ordinary_atan_terms_preserves_coefficient() {
        ordinary_doubling(1);
    }

    #[test]
    fn inverse_trig_dag_ordinary_acos_linear_preserves_coefficient() {
        ordinary_doubling(2);
    }

    #[test]
    fn inverse_trig_dag_different_paths_preserve_coefficient() {
        for kind in 0..3 {
            let value = diamond(kind, 24);
            assert_eq!(
                coefficient(kind, &value, 64),
                Some(Rational::from_bigint(BigInt::from(3).pow(24)))
            );
            assert!(value.internal.cache_snapshot().is_none());
        }
    }

    #[test]
    fn inverse_trig_dag_depth_limit_is_path_specific() {
        for kind in 0..3 {
            for reverse in [false, true] {
                for depth in 0..=7 {
                    let shared = doubling(kind, 1);
                    let nested = raw(Approximation::Offset(
                        raw(Approximation::Offset(shared.clone(), 0)),
                        0,
                    ));
                    let value = if reverse {
                        add(nested, shared)
                    } else {
                        add(shared, nested)
                    };
                    assert_eq!(
                        coefficient(kind, &value, depth),
                        (depth >= 5).then(|| Rational::new(4)),
                        "kind={kind}, reverse={reverse}, depth={depth}"
                    );
                }
            }
        }
    }

    #[test]
    fn inverse_trig_dag_shared_work_finishes_in_ten_thousand_units() {
        for kind in 0..3 {
            let value = doubling(kind, 28);
            let budget = EvaluationBudget::new(10_000);
            {
                let _scope = BudgetScope::enter(&budget);
                assert_eq!(
                    coefficient(kind, &value, 32),
                    Some(Rational::from_bigint(BigInt::one() << 28))
                );
                assert!(!evaluation_exhausted());
            }
            assert_eq!(budget.consumed() + budget.remaining(), 10_000);
            eprintln!(
                "inverse-trig DAG kind={kind}, successful work={}",
                budget.consumed()
            );
        }
    }

    #[test]
    fn inverse_trig_dag_every_cutpoint_discards_partial_forms() {
        for kind in 0..3 {
            let generous = EvaluationBudget::new(100_000);
            let value = doubling(kind, 5);
            {
                let _scope = BudgetScope::enter(&generous);
                assert_eq!(coefficient(kind, &value, 32), Some(Rational::new(32)));
            }
            let needed = generous.consumed();
            assert!(needed > 0 && needed < 4096);
            for units in 0..=needed + 2 {
                let value = doubling(kind, 5);
                let facts = value.internal.facts.snapshot();
                let budget = EvaluationBudget::new(units);
                {
                    let _scope = BudgetScope::enter(&budget);
                    let result = coefficient(kind, &value, 32);
                    assert_eq!(result.is_some(), units >= needed);
                    if let Some(result) = result {
                        assert_eq!(result, Rational::new(32));
                    }
                    assert_eq!(evaluation_exhausted(), units < needed);
                }
                assert_eq!(budget.consumed() + budget.remaining(), units);
                assert_eq!(value.internal.facts.snapshot(), facts);
                assert!(value.internal.cache_snapshot().is_none());
                assert_eq!(coefficient(kind, &value, 32), Some(Rational::new(32)));
            }
            eprintln!("inverse-trig DAG cutpoints kind={kind}, successful work={needed}");
        }
    }

    #[test]
    fn inverse_trig_dag_unshared_reconstruction_has_the_same_form() {
        fn expanded(kind: usize, depth: usize) -> Computable {
            if depth == 0 {
                atom(kind)
            } else {
                add(expanded(kind, depth - 1), expanded(kind, depth - 1))
            }
        }
        for kind in 0..3 {
            for depth in 0..=7 {
                let shared = doubling(kind, depth);
                let rebuilt = expanded(kind, depth);
                let expected = Some(Rational::from_bigint(BigInt::one() << depth));
                assert_eq!(coefficient(kind, &shared, 32), expected);
                assert_eq!(coefficient(kind, &rebuilt, 32), expected);
                let budget = EvaluationBudget::new(100_000);
                let _scope = BudgetScope::enter(&budget);
                assert_eq!(coefficient(kind, &shared, 32), expected);
                assert_eq!(coefficient(kind, &rebuilt, 32), expected);
                assert!(!evaluation_exhausted());
            }
        }
    }

    #[test]
    fn inverse_trig_dag_shared_undefined_cancellation_has_no_certificate() {
        for kind in 0..2 {
            let mut value = if kind == 0 {
                raw(Approximation::AtanDeferred(raw(Approximation::Inverse(
                    Computable::zero(),
                ))))
            } else {
                raw(Approximation::AcosPositive(Computable::rational(
                    Rational::new(2),
                )))
            };
            for _ in 0..24 {
                value = add(value.clone(), value);
            }
            let value = add(value.clone(), raw(Approximation::Negate(value)));
            assert_eq!(value.inverse_trig_linear_sign(), None);
            let budget = EvaluationBudget::new(10_000);
            let _scope = BudgetScope::enter(&budget);
            assert_eq!(value.inverse_trig_linear_sign(), None);
            assert!(!evaluation_exhausted());
            assert_eq!(value.immediate_sign(), None);
            assert!(value.internal.cache_snapshot().is_none());
        }
    }

    #[test]
    fn inverse_trig_dag_allocation_growth_tracks_shared_nodes() {
        use super::algebraic_leaf_tests::measure;

        let mut observations = Vec::new();
        for kind in 0..3 {
            // Initialize constants outside the allocation window. Fixtures,
            // assertions and printing are also outside that window.
            assert_eq!(coefficient(kind, &atom(kind), 32), Some(Rational::one()));
            let mut counts = Vec::new();
            for depth in [6, 12] {
                let value = doubling(kind, depth);
                let (answer, allocations) = measure(|| match kind {
                    0 => value.pi_atan_linear_form(32).map(|f| f.atan),
                    1 => value.pi_atan_terms_form(32).map(|f| f.terms[0].1.clone()),
                    _ => value.pi_acos_linear_form(32).map(|f| f.acos),
                });
                assert_eq!(answer, Some(Rational::from_bigint(BigInt::one() << depth)));
                eprintln!(
                    "inverse-trig DAG allocation kind={kind}, depth={depth}, allocations={allocations:?}"
                );
                counts.push(allocations);
            }
            observations.push((counts[0], counts[1]));
        }
        // Doubling the distinct-node depth must not retain the tree's 64-fold
        // growth in allocation requests. Allow room for cache capacity steps.
        assert!(
            observations.iter().all(|(small, large)| {
                large.calls <= small.calls * 8 && large.bytes <= small.bytes * 8
            }),
            "{observations:?}"
        );
    }
}
