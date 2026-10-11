#[cfg(test)]
mod evaluation_budget_tests {
    use super::*;
    use crate::EvaluationBudget;
    use crate::computable::budget::{BudgetScope, charge};
    use rug::{Float, Integer, Rational as Q, float::Round};

    fn raw(kind: Approximation) -> Computable {
        Computable::raw_periodic_operand(kind)
    }

    fn add(a: Computable, b: Computable) -> Computable { raw(Approximation::Add(a, b)) }

    fn input() -> Computable {
        add(raw(Approximation::Sqrt(Computable::rational(Rational::new(2)))),
            raw(Approximation::LnDeferred(Computable::rational(Rational::fraction(7, 5).unwrap()))))
    }

    fn oracle() -> (Q, Q) {
        let mut sqrt_lo = Float::with_val(1024, 2);
        let mut sqrt_hi = sqrt_lo.clone();
        sqrt_lo.sqrt_round(Round::Down);
        sqrt_hi.sqrt_round(Round::Up);
        let (mut ln_lo, _) = Float::with_val_round(1024, Q::from((7, 5)), Round::Down);
        let (mut ln_hi, _) = Float::with_val_round(1024, Q::from((7, 5)), Round::Up);
        ln_lo.ln_round(Round::Down);
        ln_hi.ln_round(Round::Up);
        (sqrt_lo.to_rational().unwrap() + ln_lo.to_rational().unwrap(),
         sqrt_hi.to_rational().unwrap() + ln_hi.to_rational().unwrap())
    }

    fn check(sample: &BigInt, p: i32, bounds: &(Q, Q)) {
        let unit = Q::from((1, Integer::from(1) << -p));
        let center = Q::from(Integer::from_str_radix(&sample.to_string(), 10).unwrap()) * &unit;
        assert!(Q::from(&center - &unit) <= bounds.0 && Q::from(&center + &unit) >= bounds.1);
    }

    #[test]
    fn shared_evaluation_budget_exhaustion_keeps_numeric_publication_transactional() {
        let bounds = oracle();
        let generous = EvaluationBudget::new(100_000);
        check(&input().approx_with_budget(-128, &generous).unwrap(), -128, &bounds);
        let needed = generous.consumed();
        assert!(needed > 10 && needed < 4096);
        let mut failures = 0;
        let mut successes = 0;
        for units in 0..=needed + 2 {
            let value = input();
            let allowance = EvaluationBudget::new(units);
            match value.approx_with_budget(-128, &allowance) {
                Ok(sample) => { check(&sample, -128, &bounds); successes += 1; }
                Err(error) => {
                    assert_eq!(error, Problem::Exhausted);
                    assert!(value.internal.cache_snapshot().is_none());
                    failures += 1;
                }
            }
            assert!(allowance.consumed() <= units);
            assert_eq!(allowance.consumed() + allowance.remaining(), units);
            check(&value.approx_with_budget(-128, &EvaluationBudget::new(100_000)).unwrap(), -128, &bounds);
            let before = value.structural_facts();
            let spent = EvaluationBudget::new(0);
            check(&value.approx_with_budget(-8, &spent).unwrap(), -8, &bounds);
            assert_eq!(spent.consumed(), 0);
            assert_eq!(value.structural_facts(), before);
        }
        assert!(failures > 10 && successes > 0);
    }

    #[test]
    fn shared_evaluation_budget_spans_nested_and_successive_requests() {
        let generous = EvaluationBudget::new(100_000);
        input().approx_with_budget(-128, &generous).unwrap();
        let needed = generous.consumed();
        let allowance = EvaluationBudget::new(needed);
        let clone = allowance.clone();
        let first = input();
        first.approx_with_budget(-128, &allowance).unwrap();
        assert_eq!(clone.consumed(), needed);
        assert_eq!(input().approx_with_budget(-128, &clone), Err(Problem::Exhausted));
        assert_eq!(allowance.remaining(), 0);
        // Reusing a completed root does not start another independent budget.
        assert!(first.approx_with_budget(-8, &clone).is_ok());
        assert_eq!(clone.remaining(), 0);
        assert!(input().approx_with_budget(-128, &EvaluationBudget::new(needed)).is_ok());
    }

    #[test]
    fn shared_evaluation_budget_reuses_high_precision_signs() {
        let (pi, _) = Float::with_val_round(1536, rug::float::Constant::Pi, Round::Down);
        let scaled = Float::with_val(1536, &pi << 1100);
        let floor = scaled.to_integer_round(Round::Down).unwrap().0;
        let rational = Rational::from_bigint_fraction(
            BigInt::parse_bytes(floor.to_string().as_bytes(), 10).unwrap(), BigUint::one() << 1100).unwrap();
        let lower = pi.to_rational().unwrap() - Q::from((floor, Integer::from(1) << 1100));
        assert!(lower > Q::from((1, Integer::from(1) << 1200)));
        let fresh = || add(Computable::pi(), raw(Approximation::Negate(Computable::rational(rational.clone()))));
        let warm = fresh();
        warm.approx(-1216).unwrap();
        let before = warm.structural_facts();
        assert_eq!(before.sign, Some(RealSign::Positive));
        let zero = EvaluationBudget::new(0);
        assert_eq!(warm.sign_until_with_budget(0, &zero), Some(RealSign::Positive));
        assert!(warm.approx_with_budget(-8, &zero).is_ok());
        assert_eq!(warm.structural_facts(), before);
        assert_eq!(zero.consumed(), 0);
        let cold = fresh();
        assert_eq!(cold.sign_until_with_budget(0, &zero), None);
        assert_eq!(cold.approx_with_budget(-1216, &zero), Err(Problem::Exhausted));
        assert_eq!(cold.sign_until(-1216), Some(RealSign::Positive));
    }

    fn atom(depth: usize) -> Computable {
        let mut value = raw(Approximation::Sqrt(Computable::rational(Rational::new(2))));
        for _ in 0..depth { value = raw(Approximation::Offset(value, 0)); }
        raw(Approximation::PrescaledAtan(value))
    }

    #[test]
    fn shared_evaluation_budget_structural_comparison_reports_unknown() {
        let left = atom(512);
        let right = atom(512);
        let low = EvaluationBudget::new(20);
        {
            let _scope = BudgetScope::enter(&low);
            assert_eq!(Computable::budgeted_structural_eq(&left, &right), None);
            assert!(evaluation_exhausted());
            // The legacy exact comparator retains its complete semantics.
            assert!(Computable::internal_structural_eq(&left, &right));
        }
        {
            let _scope = BudgetScope::enter(&EvaluationBudget::new(10_000));
            assert_eq!(Computable::budgeted_structural_eq(&left, &right), Some(true));
            assert_eq!(Computable::budgeted_structural_eq(&left, &Computable::one()), Some(false));
        }
        let period = raw(Approximation::Offset(Computable::pi(), i32::MAX));
        let angle = add(period, add(left, raw(Approximation::Negate(right))));
        let value = raw(Approximation::PrescaledSin(angle));
        value.ensure_finite_real(&None).unwrap();
        assert_eq!(value.approx_with_budget(-64, &EvaluationBudget::new(20)), Err(Problem::Exhausted));
        assert!(value.internal.cache_snapshot().is_none());
        assert_eq!(value.approx_with_budget(-64, &EvaluationBudget::new(100_000)), Ok(BigInt::zero()));
    }

    #[test]
    fn shared_evaluation_budget_does_not_memoize_interrupted_proofs() {
        let value = add(add(raw(Approximation::Sqrt(Computable::rational(Rational::new(2)))),
            raw(Approximation::Sqrt(Computable::rational(Rational::new(3))))), Computable::rational(Rational::new(-3)));
        {
            let _scope = BudgetScope::enter(&EvaluationBudget::new(0));
            assert_eq!(value.radical_tower_sign(), None);
            assert!(!value.internal.cache.radical_tower_declined());
            assert_eq!(value.cheap_bound(), BoundInfo::Unknown);
            assert_eq!(value.exact_sign(), None);
            assert!(matches!(value.internal.facts.snapshot(), (BoundCache::Invalid, ExactSignCache::Invalid)));
        }
        assert_eq!(value.radical_tower_sign(), Some(RealSign::Positive));
        assert_eq!(value.sign_until_with_budget(0, &EvaluationBudget::new(0)), Some(RealSign::Positive));
        assert!(value.quadratic_tower_parts().is_some());
        let zero = EvaluationBudget::new(0);
        {
            let _scope = BudgetScope::enter(&zero);
            assert_eq!(value.quadratic_tower_sign(), Some(RealSign::Positive));
        }
        assert_eq!(zero.consumed(), 0);
    }

    #[test]
    fn shared_evaluation_budget_is_atomic_and_request_scopes_are_independent() {
        let allowance = EvaluationBudget::new(97);
        let threads: Vec<_> = (0..8).map(|_| {
            let allowance = allowance.clone();
            std::thread::spawn(move || {
                let _scope = BudgetScope::enter(&allowance);
                let mut completed = 0;
                while charge(1) { completed += 1; }
                assert!(evaluation_exhausted());
                completed
            })
        }).collect();
        let completed: u64 = threads.into_iter().map(|thread| thread.join().unwrap()).sum();
        assert_eq!(completed, 97);
        assert_eq!(allowance.remaining(), 0);
        assert_eq!(allowance.consumed(), 97);
        assert!(!evaluation_exhausted());
        assert!(charge(u64::MAX));
        let fresh = EvaluationBudget::new(1);
        {
            let _scope = BudgetScope::enter(&fresh);
            assert!(charge(1));
        }
        assert_eq!(fresh.consumed(), 1);
    }

    #[test]
    fn shared_evaluation_budget_nested_scopes_cannot_reset_or_hide_limits() {
        let outer = EvaluationBudget::new(3);
        let inner = EvaluationBudget::new(1);
        {
            let _outer = BudgetScope::enter(&outer);
            {
                let _inner = BudgetScope::enter(&inner);
                let _same_outer = BudgetScope::enter(&outer);
                assert!(charge(1));
                assert_eq!(outer.consumed(), 1);
                assert_eq!(inner.consumed(), 1);
                assert!(!charge(1));
            }
            assert!(evaluation_exhausted());
            assert!(!charge(1));
        }
        {
            let _scope = BudgetScope::enter(&outer);
            assert!(charge(1));
        }
        assert_eq!(outer.consumed(), 2);
        let unwound = EvaluationBudget::new(1);
        let caught = std::panic::catch_unwind(|| {
            let _scope = BudgetScope::enter(&unwound);
            assert!(charge(1));
            panic!("exercise scope cleanup");
        });
        assert!(caught.is_err());
        assert!(!evaluation_exhausted());
        assert!(charge(1));
    }
}
