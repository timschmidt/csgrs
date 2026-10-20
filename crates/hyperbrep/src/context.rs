//! Authoritative operation scopes shared by BREP entry points.
//!
//! Hyperlimit receives an explicit strict policy at each predicate call.
//! Hypercurve's principal APIs additionally need a strict scope, because a
//! caller may have an active provisional or display-preview operation.

use hypersolve::work_budget::{BudgetOutcome, WorkBudget, with_budget};

/// Explicit strict policy and solver-work limits for a synchronous operation.
///
/// [`Self::evaluate`] shares the selected budget across all nested BREP,
/// Hypercurve and Hypersolve calls on the current thread. It records admitted
/// root refinements and algebraic promotions; exhausted work is a resource
/// failure, never an empty intersection or a geometric invalidity.
///
/// These are the existing instrumented solver units, not a bound on all BREP
/// work, elapsed time, coefficient sizes or allocations. Scalar precision
/// refinement and BREP geometry evaluations are not counted by these limits.
/// See [`hypersolve::work_budget`] for the current instrumentation frontier.
///
/// The context is reusable configuration. Each evaluation starts a new scope;
/// nested scopes also charge their parents and cannot replenish them. Evaluate
/// a sequence of operations inside one closure to share a total allowance.
/// Scopes restore on return and unwinding. They do not follow futures or
/// propagate to spawned threads; worker allocation is not implemented here.
///
/// ```
/// use hyperbrep::{OperationContext, Point3, Real, WorkBudget, builder};
///
/// let context = OperationContext::strict(WorkBudget {
///     refinements: 512,
///     promotions: 64,
/// });
/// let attempt = context.evaluate(|| {
///     builder::cuboid(
///         Point3::origin(),
///         Point3::new(Real::one(), Real::one(), Real::one()),
///     )
/// });
/// // Resource failure is separate from the constructor's domain error.
/// let bodies = attempt.result??;
/// assert_eq!(bodies.len(), 1);
/// assert!(attempt.usage.refinements <= context.work_budget().refinements);
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct OperationContext {
    budget: WorkBudget,
}

impl OperationContext {
    /// Selects strict predicate authority and explicit solver-work limits.
    pub const fn strict(budget: WorkBudget) -> Self {
        Self { budget }
    }

    /// Returns the predicate policy enforced by this operation scope.
    pub const fn predicate_policy(&self) -> hyperlimit::PredicatePolicy {
        crate::STRICT_PREDICATES
    }

    /// Returns the limits used for each evaluation of this context.
    pub const fn work_budget(&self) -> WorkBudget {
        self.budget
    }

    /// Runs synchronous work under strict predicates and a shared solver budget.
    ///
    /// The outer result reports resource exhaustion before the callback's
    /// return value can be consumed. Any domain error or uncertainty stays in
    /// that return value. Ignoring a nested exhausted result does not clear
    /// exhaustion of this scope. A smaller child allowance can fail without
    /// exhausting its parent.
    ///
    /// The returned usage counts admitted work, including nested calls and
    /// excluding denied attempts. Warm certificates can reduce this count.
    /// This scope controls kernel decisions; it does not certify arbitrary
    /// caller calculations or roll back callback side effects.
    pub fn evaluate<T>(&self, evaluate: impl FnOnce() -> T) -> BudgetOutcome<T> {
        with_budget(self.budget, || strict(evaluate))
    }
}

/// Suspends an enclosing curve approximation/preview policy for one BREP
/// operation. Hypercurve restores the caller's scope on return or unwinding.
///
/// This controls predicate authority only; it preserves any active shared work
/// budget. Cancellation and detailed proof receipts remain separate work.
#[inline]
pub(crate) fn strict<T>(evaluate: impl FnOnce() -> T) -> T {
    hypercurve::evaluate_under(crate::STRICT_PREDICATES, evaluate)
        .certified()
        .expect("Hypercurve's STRICT scope always returns a certified value")
}
