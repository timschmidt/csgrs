#[inline]
pub(crate) fn evaluation_should_stop(signal: &Option<Signal>) -> bool {
    should_stop(signal) || !crate::computable::budget::charge(1)
}

#[inline]
fn evaluation_exhausted() -> bool {
    crate::computable::budget::exhausted()
}

#[inline]
fn charge_evaluation_work() -> bool {
    crate::computable::budget::charge(1)
}

impl Computable {
    /// Approximate with a shared cooperative work allowance.
    ///
    /// Nested evaluation, numerical iteration and metered structural/domain
    /// visits consume the same allowance. An observed budget failure returns
    /// `Problem::Exhausted` without publishing a partial numeric result. Completed
    /// child proofs/caches remain reusable. A finished root approximation can
    /// answer a later query without consuming further work units.
    ///
    /// See [`crate::EvaluationBudget`] for the unit definition and limitations:
    /// this does not bound individual big-integer operations, bytes or time.
    pub fn approx_with_budget(&self, p: Precision, budget: &crate::EvaluationBudget) -> Result<BigInt, Problem> {
        let _scope = crate::computable::budget::BudgetScope::enter(budget);
        let result = self.approx(p);
        if evaluation_exhausted() { Err(Problem::Exhausted) } else { result }
    }

    /// Try to certify a sign using a shared cooperative work allowance.
    ///
    /// A previously retained root sign remains available even with no remaining
    /// work. An unfinished proof is Unknown and is never memoized as a permanent
    /// failure solely because this caller's allowance ran out.
    pub fn sign_until_with_budget(&self, min_precision: Precision, budget: &crate::EvaluationBudget) -> Option<RealSign> {
        if let Some(sign) = self.immediate_sign() { return Some(sign); }
        let _scope = crate::computable::budget::BudgetScope::enter(budget);
        let result = self.sign_until(min_precision);
        if evaluation_exhausted() { self.immediate_sign() } else { result }
    }
}
