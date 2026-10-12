#[cfg(test)]
mod inverse_trig_budget_tests {
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

    fn atan_identity(bits: usize) -> Computable {
        let b = BigInt::one() << bits;
        let a: BigInt = &b * 3 + 1;
        let atom = || raw(Approximation::AtanDeferred(integer(2)));
        subtract(
            add(
                multiply(
                    Computable::rational(Rational::from_bigint(a.clone())),
                    atom(),
                ),
                multiply(
                    Computable::rational(Rational::from_bigint(b.clone())),
                    atom(),
                ),
            ),
            multiply(Computable::rational(Rational::from_bigint(a + b)), atom()),
        )
    }

    fn acos_identity() -> Computable {
        let argument = || Computable::rational(Rational::fraction(1, 3).unwrap());
        subtract(
            add(
                raw(Approximation::AsinDeferred(argument())),
                raw(Approximation::AcosPositive(argument())),
            ),
            raw(Approximation::Offset(Computable::pi(), -1)),
        )
    }

    #[test]
    fn inverse_trig_budget_all_parser_cutpoints_discard_partial_forms() {
        for kind in 0..3 {
            let fixture = || {
                if kind == 2 {
                    acos_identity()
                } else {
                    atan_identity(4)
                }
            };
            let recognize = |value: &Computable| match kind {
                0 => value
                    .pi_atan_linear_form(32)
                    .map(|f| (f.constant, f.pi, f.atan)),
                1 => value.pi_atan_terms_form(32).map(|f| {
                    assert!(f.terms.is_empty());
                    (f.constant, f.pi, Rational::zero())
                }),
                _ => value
                    .pi_acos_linear_form(32)
                    .map(|f| (f.constant, f.pi, f.acos)),
            };
            let expected = Some((Rational::zero(), Rational::zero(), Rational::zero()));
            let generous = EvaluationBudget::new(100_000);
            {
                let _scope = BudgetScope::enter(&generous);
                assert_eq!(recognize(&fixture()), expected);
            }
            let needed = generous.consumed();
            assert!(
                needed > 0 && needed < 4096,
                "parser kind={kind} must debit work"
            );
            for units in 0..=needed + 2 {
                let value = fixture();
                let budget = EvaluationBudget::new(units);
                {
                    let _scope = BudgetScope::enter(&budget);
                    let result = recognize(&value);
                    assert_eq!(result.is_some(), units >= needed);
                    if result.is_some() {
                        assert_eq!(result, expected);
                        assert!(!evaluation_exhausted());
                    } else {
                        assert!(evaluation_exhausted());
                    }
                    assert!(value.internal.cache_snapshot().is_none());
                    assert_eq!(value.immediate_sign(), None);
                }
                assert_eq!(budget.consumed() + budget.remaining(), units);
                assert_eq!(recognize(&value), expected);
                assert_eq!(value.sign_until(-512), Some(RealSign::Zero));
                assert_eq!(
                    value.sign_until_with_budget(0, &EvaluationBudget::new(0)),
                    Some(RealSign::Zero)
                );
            }
            eprintln!("inverse-trig parser kind={kind}, successful work={needed}");
        }
    }

    #[test]
    fn inverse_trig_budget_wide_coefficients_cost_more() {
        let mut costs = Vec::new();
        for bits in [16, 256, 1024] {
            let budget = EvaluationBudget::new(1_000_000);
            {
                let _scope = BudgetScope::enter(&budget);
                let form = atan_identity(bits).pi_atan_terms_form(32).unwrap();
                assert!(form.terms.is_empty());
                assert_eq!(form.constant, Rational::zero());
            }
            assert!(budget.consumed() > 0);
            costs.push(budget.consumed());
        }
        assert!(costs.windows(2).all(|pair| pair[0] < pair[1]), "{costs:?}");
        eprintln!("inverse-trig coefficient costs={costs:?}");
    }

    #[test]
    fn inverse_trig_budget_atom_comparison_can_stop() {
        let atom = || {
            let mut argument = integer(2);
            for _ in 0..512 {
                argument = raw(Approximation::Offset(argument, 0));
            }
            raw(Approximation::AtanDeferred(argument))
        };
        let value = subtract(atom(), atom());
        let low = EvaluationBudget::new(32);
        {
            let _scope = BudgetScope::enter(&low);
            assert!(value.pi_atan_terms_form(32).is_none());
            assert!(evaluation_exhausted());
        }
        let generous = EvaluationBudget::new(10_000);
        {
            let _scope = BudgetScope::enter(&generous);
            assert!(value.pi_atan_terms_form(32).unwrap().terms.is_empty());
        }
    }

    #[test]
    fn inverse_trig_budget_shared_dag_stops_without_expansion() {
        let mut value = raw(Approximation::AtanDeferred(integer(2)));
        for _ in 0..28 {
            value = add(value.clone(), value);
        }
        let budget = EvaluationBudget::new(128);
        let _scope = BudgetScope::enter(&budget);
        assert!(value.pi_atan_terms_form(32).is_none());
        assert!(evaluation_exhausted());
        assert_eq!(budget.remaining(), 0);
        assert!(value.internal.cache_snapshot().is_none());
    }

    #[test]
    fn inverse_trig_budget_pi_helpers_and_shift_limits() {
        let value = subtract(
            multiply(integer(9), Computable::pi()),
            multiply(integer(4), Computable::pi()),
        );
        let budget = EvaluationBudget::new(0);
        {
            let _scope = BudgetScope::enter(&budget);
            assert!(value.retained_pi_rational_multiple(32).is_none());
            assert!(evaluation_exhausted());
        }
        assert_eq!(
            value.retained_pi_rational_multiple(32),
            Some(Rational::new(5))
        );
        let sine = raw(Approximation::PrescaledSin(raw(Approximation::Offset(
            Computable::pi(),
            -2,
        ))));
        let budget = EvaluationBudget::new(0);
        {
            let _scope = BudgetScope::enter(&budget);
            assert!(sine.canonical_sin_pi_term().is_none());
            assert!(evaluation_exhausted());
        }
        assert_eq!(
            sine.canonical_sin_pi_term(),
            Some((Rational::fraction(1, 4).unwrap(), Rational::one()))
        );
        for shift in [i32::MIN, i32::MAX, Computable::MAX_APPROXIMATION_BITS + 1] {
            let value = raw(Approximation::Offset(Computable::pi(), shift));
            let budget = EvaluationBudget::new(64);
            let _scope = BudgetScope::enter(&budget);
            assert!(value.pi_atan_linear_form(32).is_none());
            assert!(value.pi_atan_terms_form(32).is_none());
            assert!(value.pi_acos_linear_form(32).is_none());
            assert!(value.retained_pi_rational_multiple(32).is_none());
            assert!(
                !evaluation_exhausted(),
                "unsupported shift {shift} must be rejected before allocation debit"
            );
        }
    }

    #[test]
    fn inverse_trig_budget_argument_clones_require_work() {
        let wide = Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << 1024).unwrap();
        for kind in [
            Approximation::AtanRational(wide.clone()),
            Approximation::AsinRational(wide.clone()),
            Approximation::AcosPositiveRational(wide),
        ] {
            let value = raw(kind);
            let budget = EvaluationBudget::new(1);
            let _scope = BudgetScope::enter(&budget);
            assert!(value.atan_argument().is_none());
            assert!(value.asin_argument().is_none());
            assert!(value.acos_argument().is_none());
            assert!(evaluation_exhausted());
        }
    }

    #[test]
    fn inverse_trig_budget_public_retry_and_retained_proofs() {
        for kind in 0..2 {
            for warm in [false, true] {
                for units in [0, 1, 8, 32, 128, 512, 100_000] {
                    let value = if kind == 0 {
                        atan_identity(4)
                    } else {
                        acos_identity()
                    };
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
                        "kind={kind}, warm={warm}, units={units}"
                    );
                    assert_eq!(
                        value.sign_until_with_budget(0, &EvaluationBudget::new(0)),
                        Some(RealSign::Zero)
                    );
                }
            }
        }
    }

    #[test]
    fn inverse_trig_budget_cached_domain_does_not_publish_an_interrupted_sign() {
        let fixture = || {
            let value = atan_identity(256);
            assert!(value.has_finite_real_certificate());
            assert_eq!(value.immediate_sign(), None);
            value
        };
        let value = fixture();
        let generous = EvaluationBudget::new(100_000);
        {
            let _scope = BudgetScope::enter(&generous);
            assert_eq!(value.exact_sign(), Some(Sign::NoSign));
        }
        let needed = generous.consumed();
        assert!(needed > 0 && needed < 4096);
        for units in 0..=needed + 2 {
            let value = fixture();
            let budget = EvaluationBudget::new(units);
            {
                let _scope = BudgetScope::enter(&budget);
                let answer = value.exact_sign();
                assert_eq!(answer.is_some(), units >= needed);
                if let Some(sign) = answer {
                    assert_eq!(sign, Sign::NoSign);
                    assert!(!evaluation_exhausted());
                } else {
                    assert!(evaluation_exhausted());
                    assert_eq!(value.immediate_sign(), None);
                    assert_eq!(value.internal.facts.exact_sign(), ExactSignCache::Invalid);
                }
            }
            assert_eq!(budget.consumed() + budget.remaining(), units);
            assert_eq!(value.sign_until(-512), Some(RealSign::Zero));
            assert_eq!(
                value.sign_until_with_budget(0, &EvaluationBudget::new(0)),
                Some(RealSign::Zero)
            );
        }
        eprintln!("inverse-trig cached-domain sign successful work={needed}");
    }

    #[test]
    fn inverse_trig_budget_undefined_cancellation_stays_unknown() {
        for units in [0, 8, 256, 100_000] {
            let bad = raw(Approximation::AtanDeferred(raw(Approximation::Sqrt(
                integer(-1),
            ))));
            let value = subtract(bad.clone(), bad);
            assert_eq!(
                value.sign_until_with_budget(-64, &EvaluationBudget::new(units)),
                None
            );
            assert_eq!(value.sign_until(-64), None);
            assert_eq!(value.approx(-64), Err(Problem::SqrtNegative));
            assert!(!value.has_finite_real_certificate());
        }
    }
}
