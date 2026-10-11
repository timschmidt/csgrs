#[cfg(test)]
mod normal_form_budget_tests {
    use super::*;
    use crate::EvaluationBudget;
    use crate::computable::budget::BudgetScope;

    fn raw(kind: Approximation) -> Computable {
        Computable::raw_periodic_operand(kind)
    }

    fn integer(value: i64) -> Computable {
        Computable::rational(Rational::new(value))
    }

    fn add(a: Computable, b: Computable) -> Computable {
        raw(Approximation::Add(a, b))
    }

    fn multiply(a: Computable, b: Computable) -> Computable {
        raw(Approximation::Multiply(a, b))
    }

    fn subtract(a: Computable, b: Computable) -> Computable {
        add(a, raw(Approximation::Negate(b)))
    }

    fn pi_identity() -> Computable {
        let root = raw(Approximation::Sqrt(integer(2)));
        let base = add(Computable::pi(), root.clone());
        // Deliberately share the binomial so the normal-form memo also clones
        // nontrivial coefficients, independently of numeric expression caches.
        let square = multiply(base.clone(), base);
        let expanded = add(
            add(
                raw(Approximation::Square(Computable::pi())),
                multiply(integer(2), multiply(Computable::pi(), root)),
            ),
            integer(2),
        );
        subtract(square, expanded)
    }

    #[test]
    fn normal_form_budget_public_sign_retry_does_not_cache_tentative_unknown() {
        // Preserve the public replay's complete coefficients and structure:
        // (16*pi + 5*sqrt(2))^2 - ((16*pi)^2 + 160*pi*sqrt(2) + 50).
        // Numeric warming leaves the root's sign to the symbolic proof search.
        let fixture = || {
            let root = || raw(Approximation::Sqrt(integer(2)));
            let ap = || multiply(integer(16), Computable::pi());
            let lhs = raw(Approximation::Square(add(
                ap(),
                multiply(integer(5), root()),
            )));
            let rhs = add(
                add(
                    raw(Approximation::Square(ap())),
                    multiply(integer(160), multiply(Computable::pi(), root())),
                ),
                integer(50),
            );
            subtract(lhs, rhs)
        };
        for warm in [false, true] {
            for units in [0, 1, 4, 8, 16, 32, 64, 128, 256, 512, 100_000] {
                let value = fixture();
                if warm {
                    assert!(value.approx(-512).unwrap().abs() <= BigInt::one());
                }
                let budget = EvaluationBudget::new(units);
                let answer = value.sign_until_with_budget(-512, &budget);
                assert!(answer.is_none() || answer == Some(RealSign::Zero));
                assert_eq!(budget.consumed() + budget.remaining(), units);
                assert_eq!(
                    value.sign_until(-512),
                    Some(RealSign::Zero),
                    "warm={warm}, units={units}"
                );
                assert_eq!(
                    value.sign_until_with_budget(0, &EvaluationBudget::new(0)),
                    Some(RealSign::Zero)
                );
            }
        }
    }

    #[test]
    fn normal_form_budget_laurent_allowance_sweep_preserves_retry_and_facts() {
        let generous = EvaluationBudget::new(100_000);
        {
            let _scope = BudgetScope::enter(&generous);
            assert_eq!(
                pi_identity().bounded_laurent_rational(64),
                Some(Rational::zero())
            );
        }
        let needed = generous.consumed();
        assert!(
            needed > 0 && needed < 4096,
            "normal-form traversal must charge work"
        );
        let mut stopped = 0;
        let mut completed = 0;
        for units in 0..=needed + 2 {
            let value = pi_identity();
            let budget = EvaluationBudget::new(units);
            {
                let _scope = BudgetScope::enter(&budget);
                match value.bounded_laurent_rational(64) {
                    Some(result) => {
                        assert_eq!(result, Rational::zero());
                        assert!(!evaluation_exhausted());
                        completed += 1;
                    }
                    None => {
                        assert!(evaluation_exhausted());
                        assert_eq!(value.immediate_sign(), None);
                        assert!(matches!(
                            value.internal.facts.snapshot(),
                            (BoundCache::Invalid, ExactSignCache::Invalid)
                        ));
                        stopped += 1;
                    }
                }
                assert!(value.internal.cache_snapshot().is_none());
            }
            assert_eq!(budget.consumed() + budget.remaining(), units);
            assert_eq!(value.bounded_laurent_rational(64), Some(Rational::zero()));
            let zero = EvaluationBudget::new(0);
            assert_eq!(value.sign_until_with_budget(0, &zero), Some(RealSign::Zero));
            assert_eq!(zero.consumed(), 0);
        }
        assert!(stopped > 0 && completed > 0);
        eprintln!(
            "normal-form sweep successful work={needed}, stopped={stopped}, completed={completed}"
        );
    }

    fn rational_product(bits: usize) -> Computable {
        let b: BigInt = BigInt::one() << bits;
        let a = Computable::rational(Rational::from_bigint(&b + 1));
        let b = Computable::rational(Rational::from_bigint(b));
        multiply(add(a.clone(), b.clone()), subtract(a, b))
    }

    #[test]
    fn normal_form_budget_coefficient_sizes_increase_required_work() {
        let mut costs = Vec::new();
        for bits in [16, 256, 1024] {
            // (2^b + 1 + 2^b)(2^b + 1 - 2^b) = 2^(b+1) + 1.
            let expected = Rational::from_bigint((BigInt::one() << (bits + 1)) + 1);
            let generous = EvaluationBudget::new(1_000_000);
            {
                let _scope = BudgetScope::enter(&generous);
                assert_eq!(
                    rational_product(bits).bounded_laurent_rational(64),
                    Some(expected.clone())
                );
            }
            let needed = generous.consumed();
            assert!(needed > 0, "coefficient operations must charge work");
            costs.push(needed);
            for units in [0, 1, needed / 4, needed - 1, needed, needed + 1] {
                let value = rational_product(bits);
                let budget = EvaluationBudget::new(units);
                {
                    let _scope = BudgetScope::enter(&budget);
                    let answer = value.bounded_laurent_rational(64);
                    assert_eq!(answer.is_some(), units >= needed);
                    if let Some(answer) = answer {
                        assert_eq!(answer, expected);
                    } else {
                        assert!(evaluation_exhausted());
                        assert_eq!(value.immediate_sign(), None);
                    }
                }
                assert_eq!(budget.consumed() + budget.remaining(), units);
                assert_eq!(value.bounded_laurent_rational(64), Some(expected.clone()));
                assert_eq!(
                    value.sign_until_with_budget(0, &EvaluationBudget::new(0)),
                    Some(RealSign::Positive)
                );
            }
        }
        assert!(
            costs.windows(2).all(|pair| pair[0] < pair[1]),
            "coefficient work: {costs:?}"
        );
        eprintln!("normal-form coefficient costs={costs:?}");
    }

    #[test]
    fn normal_form_budget_atom_comparison_declines_without_false_inequality() {
        let atom = || {
            let mut value = integer(2);
            for _ in 0..512 {
                value = raw(Approximation::Offset(value, 0));
            }
            raw(Approximation::Sqrt(value))
        };
        let left = atom();
        let right = atom();
        let positive = || RadicalCoefficient::atom(left.clone());
        let negative = || RadicalCoefficient {
            rational: Rational::zero(),
            radicals: Vec::new(),
            atoms: vec![(right.clone(), Rational::new(-1))],
        };
        {
            let _scope = BudgetScope::enter(&EvaluationBudget::new(20));
            assert!(
                positive()
                    .add::<true>(negative())
                    .is_none()
            );
            assert!(evaluation_exhausted());
            assert!(Computable::internal_structural_eq(&left, &right));
        }
        {
            let _scope = BudgetScope::enter(&EvaluationBudget::new(10_000));
            assert!(
                positive()
                    .add::<true>(negative())
                    .unwrap()
                    .is_zero()
            );
        }
    }

    #[test]
    fn normal_form_budget_rational_recognition_and_domain_retries() {
        let input = || raw(Approximation::Inverse(rational_product(256)));
        let expected =
            Rational::from_bigint_fraction(BigInt::one(), (BigUint::one() << 257) + 1_u8).unwrap();
        let generous = EvaluationBudget::new(100_000);
        {
            let _scope = BudgetScope::enter(&generous);
            assert_eq!(input().bounded_rational_value(), Some(expected.clone()));
        }
        let needed = generous.consumed();
        assert!(
            needed > 0 && needed < 4096,
            "rational recursion must charge work"
        );
        for units in 0..=needed + 2 {
            let value = input();
            let budget = EvaluationBudget::new(units);
            {
                let _scope = BudgetScope::enter(&budget);
                let answer = value.bounded_rational_value();
                assert_eq!(answer.is_some(), units >= needed);
                if let Some(answer) = answer {
                    assert_eq!(answer, expected);
                } else {
                    assert!(evaluation_exhausted());
                }
            }
            assert!(value.internal.cache_snapshot().is_none());
            assert_eq!(budget.consumed() + budget.remaining(), units);
            assert_eq!(value.bounded_rational_value(), Some(expected.clone()));
            assert!(value.has_finite_real_certificate());
        }
        let cases = [
            (
                raw(Approximation::Square(integer(3))),
                Some(Rational::new(9)),
            ),
            (raw(Approximation::Sqrt(integer(9))), Some(Rational::new(3))),
            (
                raw(Approximation::NthRoot(integer(27), 3)),
                Some(Rational::new(3)),
            ),
            (
                raw(Approximation::Offset(integer(3), -2)),
                Rational::fraction(3, 4).ok(),
            ),
            (raw(Approximation::Inverse(integer(0))), None),
            (raw(Approximation::Sqrt(integer(-1))), None),
            (raw(Approximation::NthRoot(integer(-27), 3)), None),
        ];
        for (value, expected) in cases {
            {
                let _scope = BudgetScope::enter(&EvaluationBudget::new(0));
                assert_eq!(value.bounded_rational_value(), None);
            }
            assert_eq!(value.bounded_rational_value(), expected);
        }
        eprintln!("bounded rational sweep successful work={needed}");
    }

    #[test]
    fn normal_form_budget_prevents_unbounded_coefficient_expansion() {
        // This first assertion fails on the old evaluator, before the full
        // explosive graph below is reached. Run the regression under process
        // caps as well, so a partial accounting repair cannot allocate freely.
        {
            let _scope = BudgetScope::enter(&EvaluationBudget::new(0));
            assert!(
                raw(Approximation::Square(integer(3)))
                    .pi_laurent_polynomial(64)
                    .is_none()
            );
        }
        let squares = |depth| {
            let mut value = integer(3);
            for _ in 0..depth {
                value = raw(Approximation::Square(value));
            }
            value
        };
        let huge = squares(42);
        let budget = EvaluationBudget::new(512);
        {
            let _scope = BudgetScope::enter(&budget);
            assert!(huge.pi_laurent_polynomial(64).is_none());
            assert!(evaluation_exhausted());
        }
        assert!(huge.internal.cache_snapshot().is_none());
        assert_eq!(budget.consumed() + budget.remaining(), 512);
        assert_eq!(
            squares(8).bounded_laurent_rational(64),
            Some(Rational::from_bigint(BigInt::from(3).pow(256)))
        );
        let offset = raw(Approximation::Offset(Computable::pi(), 1_000_000));
        {
            let _scope = BudgetScope::enter(&EvaluationBudget::new(20));
            assert!(offset.pi_laurent_polynomial(64).is_none());
            assert!(evaluation_exhausted());
        }
        {
            let _scope = BudgetScope::enter(&EvaluationBudget::new(1_000_000));
            assert!(offset.pi_laurent_polynomial(64).is_some());
        }
    }

    #[test]
    fn normal_form_budget_never_certifies_undefined_cancellation() {
        for units in [0, 8, 256, 100_000] {
            for product in [false, true] {
                let bad = raw(Approximation::Sqrt(integer(-1)));
                let value = if product {
                    multiply(integer(0), bad)
                } else {
                    subtract(bad.clone(), bad)
                };
                {
                    let _scope = BudgetScope::enter(&EvaluationBudget::new(units));
                    assert_eq!(value.bounded_laurent_rational(64), None);
                    assert_eq!(value.immediate_sign(), None);
                }
                assert_eq!(value.bounded_laurent_rational(64), None);
                assert_eq!(value.approx(-64), Err(Problem::SqrtNegative));
            }
        }
    }

    #[test]
    fn normal_form_budget_rejects_unsupported_shifts_before_allocation_charge() {
        // These shifts already exceed the existing coefficient expansion limit.
        // Keep their exact Offset nodes opaque without charging for an
        // allocation the recognizer would never perform. The full serialized
        // cotangent/ln(2) replay retains its original i32::MAX shift as well.
        for shift in [i32::MIN, i32::MAX, Computable::MAX_APPROXIMATION_BITS + 1] {
            let value = raw(Approximation::Offset(Computable::pi(), shift));
            let budget = EvaluationBudget::new(64);
            let _scope = BudgetScope::enter(&budget);
            let form = value.pi_laurent_polynomial(64).unwrap();
            assert!(!evaluation_exhausted());
            assert!(budget.remaining() > 0);
            assert!(
                matches!(form.terms.as_slice(), [(0, coefficient)] if coefficient.atoms.len() == 1)
            );
            assert_eq!(value.internal.cache_snapshot(), None);
        }
    }
}
