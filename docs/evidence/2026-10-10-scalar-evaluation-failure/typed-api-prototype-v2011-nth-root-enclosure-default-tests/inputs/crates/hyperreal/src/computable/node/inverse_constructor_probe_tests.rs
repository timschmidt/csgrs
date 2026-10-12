#[cfg(test)]
mod inverse_constructor_probe_tests {
    use super::algebraic_leaf_tests::measure;
    use super::*;
    use crate::EvaluationBudget;
    use crate::computable::budget::BudgetScope;

    pub(super) fn public_difference(bits: usize, with_pi: bool, reversed: bool) -> Computable {
        let argument = |bits| {
            let n = (BigInt::one() << (bits - 1)) + 1;
            let value = Computable::rational(Rational::from_bigint(n));
            let value = if with_pi {
                value.add(Computable::pi())
            } else {
                value
            };
            let value = value.atan();
            assert!(if with_pi {
                matches!(value.internal.approximation, Approximation::AtanDeferred(_))
            } else {
                matches!(value.internal.approximation, Approximation::AtanRational(_))
            });
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
    fn inverse_constructor_public_budget_survey() {
        let zero = EvaluationBudget::new(0);
        drop(BudgetScope::enter(&zero));
        for with_pi in [false, true] {
            for bits in [8, 65, 4097, 65_537, 1_000_001] {
                for reversed in [false, true] {
                    for units in [0, 16, 24, 32, 40, 48, 64, 128] {
                        let value = public_difference(bits, with_pi, reversed);
                        let budget = EvaluationBudget::new(units);
                        let (answer, allocations) =
                            measure(|| value.sign_until_with_budget(0, &budget));
                        eprintln!(
                            "inverse constructor survey with_pi={with_pi}, bits={bits}, reversed={reversed}, units={units}, answer={answer:?}, consumed={}, remaining={}, allocations={allocations:?}",
                            budget.consumed(),
                            budget.remaining()
                        );
                        let expected = if reversed {
                            RealSign::Negative
                        } else {
                            RealSign::Positive
                        };
                        assert!(answer.is_none_or(|answer| answer == expected));
                        assert_eq!(budget.consumed() + budget.remaining(), units);
                    }
                }
            }
        }
    }
}
