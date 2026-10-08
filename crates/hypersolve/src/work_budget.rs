//! Shared work accounting for one synchronous exact computation.
//!
//! A budget limits admitted root-subdivision and algebraic-promotion attempts.
//! It does not select a predicate policy or permit approximation. Exhaustion
//! is a resource failure, not a proof that a value is zero or unsupported.
//!
//! These units count algorithmic work, not elapsed time, bytes, or scalar
//! arithmetic. A single admitted attempt can still manipulate large exact
//! coefficients. Only instrumented paths participate; this is not yet a
//! whole-solver resource bound. Callers must not infer a hard time limit from
//! the counters. Work outside a budget keeps its existing behavior.
//!
//! Current instrumentation includes native univariate Sturm isolation and
//! singleton refinement, represented-root sign-change refinement, tensor-image
//! promotion, and Hypercurve's Bernstein/Sturm parameter subdivisions, native
//! recursive-field refinements, and selected-fiber/local-field promotions.
//! The shared ordered-field Bernstein isolator, selected-fiber Sturm isolator,
//! reusable fiber-root refiner, and retained local-field root refinements also
//! participate. Other elimination entry points, remaining curve dispatch paths,
//! scalar precision refinement, and arbitrary caller arithmetic are not
//! comprehensively instrumented. Those paths need additional work
//! before any whole-operation completion or time bound can be claimed.
//!
//! One promotion unit admits one instrumented projection entry, regardless of
//! its degree or number of eliminated axes; root subdivisions performed inside
//! it consume separate refinement units. Usage depends on retained evidence,
//! representation, and algorithm version. A ready proof may require no units.
//! Resource blockers must not be retained as mathematical capability facts.
//!
//! Nested scopes share their actual charges: an inner budget cannot replenish
//! its parent. A failed charge spends no units. An exhausted scope cannot
//! publish a successful result, even if an intermediate caller ignores the
//! failure. A smaller inner budget may fail without exhausting its parent.
//!
//! Scopes are thread-local and restore on return and unwinding. They do not
//! follow a future or propagate to spawned threads; configure each synchronous
//! worker explicitly.

use std::cell::{Cell, RefCell};
use std::fmt;

/// Maximum admitted work of each kind in a synchronous operation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct WorkBudget {
    /// Root isolation subdivisions and singleton-isolator refinement attempts.
    pub refinements: usize,
    /// Attempts to project retained algebraic evidence into a new root carrier.
    pub promotions: usize,
}

impl WorkBudget {
    /// Admit no new refinement or promotion work. Existing proofs can be reused.
    pub const ZERO: Self = Self {
        refinements: 0,
        promotions: 0,
    };
}

/// Actual work admitted by a budget, including nested operations.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct WorkUsage {
    /// Admitted root-subdivision/refinement attempts.
    pub refinements: usize,
    /// Admitted algebraic-promotion attempts.
    pub promotions: usize,
}

/// The resource whose next unit could not be admitted.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WorkKind {
    /// One root-isolation subdivision or singleton-isolator refinement.
    RootRefinement,
    /// One algebraic projection/promotion attempt.
    AlgebraicPromotion,
}

/// A denied work request, before any of its units were performed.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BudgetExceeded {
    /// Work that the operation attempted to perform.
    pub kind: WorkKind,
    /// The exhausted scope's configured maximum.
    pub limit: usize,
    /// Units already admitted in that scope.
    pub used: usize,
    /// Additional units requested by the denied attempt.
    pub requested: usize,
}

impl fmt::Display for BudgetExceeded {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "{:?} budget exhausted: {} of {} units used; {} more requested",
            self.kind, self.used, self.limit, self.requested
        )
    }
}

impl std::error::Error for BudgetExceeded {}

/// An operation's value or resource failure, paired with admitted work.
#[derive(Clone, Debug, Eq, PartialEq)]
#[must_use = "budget exhaustion must be handled before using the operation result"]
pub struct BudgetOutcome<T> {
    /// A value is published only if this scope and its parents did not exhaust.
    /// The value can itself be a domain-specific `Result` or classification.
    pub result: Result<T, BudgetExceeded>,
    /// Work actually admitted within this scope, excluding denied attempts.
    pub usage: WorkUsage,
}

struct Frame {
    budget: WorkBudget,
    usage: WorkUsage,
    exhausted: Option<BudgetExceeded>,
}

impl Frame {
    const fn available(&self, kind: WorkKind) -> (usize, usize) {
        match kind {
            WorkKind::RootRefinement => (self.budget.refinements, self.usage.refinements),
            WorkKind::AlgebraicPromotion => (self.budget.promotions, self.usage.promotions),
        }
    }

    fn spend(&mut self, kind: WorkKind, amount: usize) {
        match kind {
            WorkKind::RootRefinement => self.usage.refinements += amount,
            WorkKind::AlgebraicPromotion => self.usage.promotions += amount,
        }
    }
}

std::thread_local! {
    // The common unbudgeted path need not initialize or borrow the frame stack.
    static ACTIVE: Cell<bool> = const { Cell::new(false) };
    static FRAMES: RefCell<Vec<Frame>> = const { RefCell::new(Vec::new()) };
}

/// Whether a caller has selected a work budget on this thread.
///
/// Operation caches use this to avoid retaining budget-dependent blockers as
/// permanent facts about an input's mathematical capabilities.
#[inline]
pub fn is_active() -> bool {
    ACTIVE.with(Cell::get)
}

/// The innermost scope's first resource failure, if any.
#[inline]
pub fn exhaustion() -> Option<BudgetExceeded> {
    if !is_active() {
        return None;
    }
    FRAMES.with_borrow(|frames| frames.last().and_then(|frame| frame.exhausted))
}

/// Admit work before performing it, charging every active enclosing budget.
///
/// Kernel extensions call this at the actual work site, not at a caller's
/// speculative estimate. Without an active scope this is a no-op. A denied
/// request leaves all counters unchanged and exhausts the limiting scope and
/// its children. Larger subsequent scopes must retry the operation; they
/// cannot clear an enclosing failure.
#[inline]
pub fn consume(kind: WorkKind, amount: usize) -> Result<(), BudgetExceeded> {
    if !is_active() {
        return Ok(());
    }
    FRAMES.with_borrow_mut(|frames| {
        if let Some(error) = frames.last().and_then(|frame| frame.exhausted) {
            return Err(error);
        }
        for index in 0..frames.len() {
            let (limit, used) = frames[index].available(kind);
            if amount > limit - used {
                let error = BudgetExceeded {
                    kind,
                    limit,
                    used,
                    requested: amount,
                };
                for frame in &mut frames[index..] {
                    frame.exhausted = Some(error);
                }
                return Err(error);
            }
        }
        for frame in frames {
            frame.spend(kind, amount);
        }
        Ok(())
    })
}

/// Evaluate one synchronous operation with a shared, explicit work budget.
///
/// A resource failure takes precedence over any value the callback returns.
/// Domain errors and predicate uncertainty remain inside the callback's own
/// return type. Panic unwinding restores the enclosing scope without refunding
/// work already admitted by that scope.
pub fn with_budget<T>(budget: WorkBudget, operation: impl FnOnce() -> T) -> BudgetOutcome<T> {
    struct Scope(bool);

    impl Scope {
        fn finish(mut self) -> Frame {
            self.0 = false;
            FRAMES.with_borrow_mut(|frames| {
                let frame = frames.pop().expect("a budget scope owns its frame");
                ACTIVE.with(|active| active.set(!frames.is_empty()));
                frame
            })
        }
    }

    impl Drop for Scope {
        fn drop(&mut self) {
            if self.0 {
                FRAMES.with_borrow_mut(|frames| {
                    let _ = frames.pop();
                    ACTIVE.with(|active| active.set(!frames.is_empty()));
                });
            }
        }
    }

    FRAMES.with_borrow_mut(|frames| {
        let exhausted = frames.last().and_then(|frame| frame.exhausted);
        frames.push(Frame {
            budget,
            usage: WorkUsage::default(),
            exhausted,
        });
        ACTIVE.with(|active| active.set(true));
    });
    let scope = Scope(true);
    let value = operation();
    let frame = scope.finish();
    BudgetOutcome {
        result: match frame.exhausted {
            Some(error) => Err(error),
            None => Ok(value),
        },
        usage: frame.usage,
    }
}

#[cfg(test)]
mod tests;
