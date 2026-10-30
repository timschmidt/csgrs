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
//! participate. Complete ordered-field Sturm discovery charges each dyadic
//! subdivision, including exclusion of separately emitted exact roots.
//! Its root counts and coefficient arithmetic are not bounded by that counter.
//! Polynomial, rational, and binary represented-root images charge
//! their eliminant constructions; rational maps can reuse a cached eliminant
//! without a new promotion charge. Constant and affine shortcuts need no
//! promotion units. Selected-fiber polynomial-image norms, bivariate quotient
//! norms, tagged tensor-fiber projections, bivariate system resultants,
//! constrained tensor-axis resultants, fixed-rank three/four-variable norms,
//! their subresultants, selected polynomial-field extension validation,
//! bivariate cofactor parameter lifts, common-component
//! discovery, and axis-factor extraction also participate.
//! Bivariate finite-fiber topology charges its construction entry, its existing
//! projection/fiber operations, and connection-strip or outer-separator
//! refinement attempts. Reusing a topology for a rational slice charges only
//! the root work actually required; its earlier graph proof is not rebuilt.
//! Other elimination entry points,
//! remaining curve dispatch paths,
//! scalar precision refinement, and arbitrary caller arithmetic are not
//! comprehensively instrumented. Those paths need additional work
//! before any whole-operation completion or time bound can be claimed.
//!
//! A new selected polynomial extension charges one promotion unit before its
//! singleton root count and local monic normalization. Reusing an admitted
//! field or performing its arithmetic charges no extension unit. The count
//! and arithmetic are not bounded by this entry counter.
//!
//! Selected polynomial value bracketing and enclosure refinement charge one
//! root-refinement unit per new bound comparison. Existing checked bounds and
//! structural zeros can be reused without a new comparison. This count does
//! not bound the algebraic work inside a sign decision.
//! Optional selected-field sign filters additionally charge each performed
//! selected-root bisection, including nested coefficient selections. Native
//! source refinement keeps its existing shared accounting and evidence reuse.
//! Rational-coefficient queries retain the exact remainder path without these
//! optional refinements. The counters still do not bound coefficient arithmetic.
//!
//! One promotion unit admits one instrumented projection entry, regardless of
//! its degree or number of eliminated axes; root subdivisions performed inside
//! it consume separate refinement units. Nested projection entries share the
//! budget and charge additional units: a tagged tensor-fiber projection charges
//! its entry, each needed tensor-axis resultant, and its final bivariate quotient
//! norm. Degree-limit admission alone does not charge a bivariate or fixed-rank
//! resultant. Fixed-rank subresultants also charge once before interpolation;
//! a terminal-order subresultant reuses an input without a promotion charge.
//! Cofactor parameter lifts charge one attempt after degree admission; their
//! curve replay cache can reuse a completed report without a new charge and
//! never retains a report of resource exhaustion.
//! Common-component discovery charges once after degree admission, including
//! its sampled-GCD and subresultant continuations. Axis-factor extraction
//! charges once for a nonzero system and again before a projective-scale retry.
//! Successive factor removals share the allowance. Saturation, multiplicity
//! reduction, selector splitting, and trivariate factor normalization preserve
//! exhaustion instead of publishing partially processed supports or factors.
//! Usage depends on retained evidence,
//! representation, and algorithm version. A ready proof may require no units.
//! Resource blockers must not be retained as mathematical capability facts.
//! Budgeted rational-map requests may read existing elimination caches but
//! leave new eliminants uncached, including failed construction attempts.
//!
//! Nested scopes share their actual charges: an inner budget cannot replenish
//! its parent. A failed charge spends no units. An exhausted scope cannot
//! publish a successful result, even if an intermediate caller ignores the
//! failure. A smaller inner budget may fail without exhausting its parent.
//!
//! Scopes are thread-local and restore on return and unwinding. They do not
//! follow a future or propagate to spawned threads; configure each synchronous
//! worker explicitly.
//!
//! A scope can also carry a [`CancellationToken`]. Cancellation is cooperative:
//! the computing thread observes it at its next admitted unit or exhaustion
//! check, never inside one exact arithmetic step, so the delay before an
//! instrumented operation stops is bounded by its longest uninstrumented
//! stretch, not by a time limit. A cancelled scope then behaves as an exhausted
//! one with [`WorkKind::Cancellation`]: it publishes no value and its callers
//! unwind through their ordinary typed resource failures. Cancelling an
//! enclosing scope's token stops every nested scope; cancelling a nested
//! scope's token leaves its parent free to continue. Tokens are `Send` and
//! `Sync`, so another thread may cancel, and a worker can join the same request
//! by entering its own scope with a clone. A cancelled token stays cancelled;
//! a retry uses a new token and repeats the work its earlier attempt had not
//! retained as a proof.
//!
//! Some long loops charge no units but still observe [`exhaustion`] between
//! steps, so cancelled or exhausted scopes leave them promptly: integer
//! polynomial Bareiss determinants, pseudo-quotient multiplication matrices and
//! dense tensor products. They report their existing "unavailable" outcome,
//! never a zero or partial value. Other uninstrumented arithmetic, such as one
//! large integer product or a scalar refinement, still runs to completion.
//!
//! [`with_coefficient_bit_limit`] bounds the size of exact coefficients that
//! those same constructions produce: Bareiss entries, pseudo-quotient matrix
//! entries and exact rational dense tensor product coefficients report their
//! bit length through [`observe_coefficient_bits`], and a larger one fails
//! with [`WorkKind::CoefficientGrowth`] before it is used. Unlike cancellation
//! the bound is deterministic. It covers only these constructions: other
//! coefficient arithmetic, such as subresultant and Sturm sequences or scalar
//! approximations, is not measured, so it is not a memory bound.

use std::cell::{Cell, RefCell};
use std::fmt;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

/// Maximum admitted work of each kind in a synchronous operation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct WorkBudget {
    /// Root isolation subdivisions, singleton refinements, and selected-field bound comparisons.
    pub refinements: usize,
    /// Admitted algebraic projection, lifting, and common-factor attempts.
    pub promotions: usize,
}

impl WorkBudget {
    /// Admit no new refinement or promotion work. Existing proofs can be reused.
    pub const ZERO: Self = Self {
        refinements: 0,
        promotions: 0,
    };

    /// Admit any number of instrumented units, for scopes that are limited only
    /// by a [`CancellationToken`].
    pub const UNLIMITED: Self = Self {
        refinements: usize::MAX,
        promotions: usize::MAX,
    };
}

/// A cooperative request to stop one or more budgeted operations.
///
/// Clones share the request. See the [module documentation](self) for when
/// cancellation is observed and how it nests.
#[derive(Clone, Debug, Default)]
pub struct CancellationToken(Arc<AtomicBool>);

impl CancellationToken {
    /// A token that has not been cancelled.
    pub fn new() -> Self {
        Self::default()
    }

    /// Request that every scope carrying this token stop at its next check.
    pub fn cancel(&self) {
        self.0.store(true, Ordering::Release);
    }

    /// Whether [`cancel`](Self::cancel) has been called on this token or a clone.
    pub fn is_cancelled(&self) -> bool {
        self.0.load(Ordering::Acquire)
    }
}

/// Actual work admitted by a budget, including nested operations.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct WorkUsage {
    /// Admitted root subdivisions, singleton refinements, and selected-field bound comparisons.
    pub refinements: usize,
    /// Admitted algebraic-promotion attempts.
    pub promotions: usize,
}

/// The resource whose next unit could not be admitted.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WorkKind {
    /// One root subdivision, singleton refinement, or selected-field bound comparison.
    RootRefinement,
    /// One algebraic projection/promotion attempt.
    AlgebraicPromotion,
    /// Any further work after the scope's [`CancellationToken`] was cancelled.
    Cancellation,
    /// An instrumented exact coefficient larger than the scope's bit limit.
    CoefficientGrowth,
}

/// A denied work request, before any of its units were performed.
///
/// A [`WorkKind::Cancellation`] failure has zero `limit` and `used`; it
/// reports only the size of the request that observed the cancellation. A
/// [`WorkKind::CoefficientGrowth`] failure reports the bit limit as `limit`,
/// zero `used`, and the observed coefficient's bit length as `requested`.
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

impl BudgetExceeded {
    const fn cancelled(requested: usize) -> Self {
        Self {
            kind: WorkKind::Cancellation,
            limit: 0,
            used: 0,
            requested,
        }
    }
}

impl fmt::Display for BudgetExceeded {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.kind == WorkKind::CoefficientGrowth {
            return write!(
                formatter,
                "coefficient growth limit exceeded: {} bits against a limit of {}",
                self.requested, self.limit
            );
        }
        if self.kind == WorkKind::Cancellation {
            return write!(
                formatter,
                "work cancelled; {} more units requested",
                self.requested
            );
        }
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
    cancellation: Option<CancellationToken>,
    max_coefficient_bits: Option<u64>,
}

impl Frame {
    const fn available(&self, kind: WorkKind) -> (usize, usize) {
        match kind {
            WorkKind::RootRefinement => (self.budget.refinements, self.usage.refinements),
            WorkKind::AlgebraicPromotion => (self.budget.promotions, self.usage.promotions),
            WorkKind::Cancellation | WorkKind::CoefficientGrowth => (0, 0),
        }
    }

    fn spend(&mut self, kind: WorkKind, amount: usize) {
        match kind {
            WorkKind::RootRefinement => self.usage.refinements += amount,
            WorkKind::AlgebraicPromotion => self.usage.promotions += amount,
            WorkKind::Cancellation | WorkKind::CoefficientGrowth => {}
        }
    }
}

// The outermost cancelled scope exhausts itself and every nested scope; its
// parents are unaffected, as with a smaller inner budget.
fn observe_cancellation(frames: &mut [Frame], requested: usize) -> Option<BudgetExceeded> {
    let index = frames.iter().position(|frame| {
        frame
            .cancellation
            .as_ref()
            .is_some_and(CancellationToken::is_cancelled)
    })?;
    let error = BudgetExceeded::cancelled(requested);
    for frame in &mut frames[index..] {
        frame.exhausted.get_or_insert(error);
    }
    frames.last().and_then(|frame| frame.exhausted)
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
    FRAMES.with_borrow_mut(|frames| {
        frames
            .last()
            .and_then(|frame| frame.exhausted)
            .or_else(|| observe_cancellation(frames, 0))
    })
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
        if let Some(error) = observe_cancellation(frames, amount) {
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
    enter(budget, None, None, operation)
}

/// Evaluate one synchronous operation with a work budget and a cancellation token.
///
/// This is [`with_budget`] with cooperative cancellation: once `cancellation`
/// is cancelled, from any thread, the operation's next admitted unit or
/// exhaustion check fails with [`WorkKind::Cancellation`] and no value is
/// published. A value completed before any check observed the request is
/// returned normally. Use [`WorkBudget::UNLIMITED`] to bound the operation by
/// cancellation alone.
pub fn with_cancellable_budget<T>(
    budget: WorkBudget,
    cancellation: &CancellationToken,
    operation: impl FnOnce() -> T,
) -> BudgetOutcome<T> {
    enter(budget, Some(cancellation.clone()), None, operation)
}

/// Evaluate one synchronous operation whose instrumented exact coefficients
/// may not exceed `bits` bits.
///
/// The scope admits any number of work units; nest it inside [`with_budget`]
/// or [`with_cancellable_budget`] to combine limits. An observed coefficient
/// larger than `bits` fails with [`WorkKind::CoefficientGrowth`] and no value
/// is published. A nested limit that fails leaves its parent running. See the
/// [module documentation](self) for the constructions that report sizes.
pub fn with_coefficient_bit_limit<T>(bits: u64, operation: impl FnOnce() -> T) -> BudgetOutcome<T> {
    enter(WorkBudget::UNLIMITED, None, Some(bits), operation)
}

/// Report the bit length of an exact coefficient that an instrumented
/// construction has produced, before it is used.
///
/// Fails, exhausting the outermost scope whose limit it exceeds and every
/// scope nested in it, when the coefficient is larger than an active limit.
/// An already exhausted or cancelled scope also fails. Without an active
/// scope this is a no-op.
#[inline]
pub fn observe_coefficient_bits(bits: u64) -> Result<(), BudgetExceeded> {
    if !is_active() {
        return Ok(());
    }
    FRAMES.with_borrow_mut(|frames| {
        if let Some(error) = frames.last().and_then(|frame| frame.exhausted) {
            return Err(error);
        }
        if let Some(error) = observe_cancellation(frames, 0) {
            return Err(error);
        }
        let Some(index) = frames
            .iter()
            .position(|frame| frame.max_coefficient_bits.is_some_and(|limit| bits > limit))
        else {
            return Ok(());
        };
        let limit = frames[index].max_coefficient_bits.unwrap_or(0);
        let error = BudgetExceeded {
            kind: WorkKind::CoefficientGrowth,
            limit: usize::try_from(limit).unwrap_or(usize::MAX),
            used: 0,
            requested: usize::try_from(bits).unwrap_or(usize::MAX),
        };
        for frame in &mut frames[index..] {
            frame.exhausted.get_or_insert(error);
        }
        Err(frames
            .last()
            .and_then(|frame| frame.exhausted)
            .unwrap_or(error))
    })
}

fn enter<T>(
    budget: WorkBudget,
    cancellation: Option<CancellationToken>,
    max_coefficient_bits: Option<u64>,
    operation: impl FnOnce() -> T,
) -> BudgetOutcome<T> {
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
            cancellation,
            max_coefficient_bits,
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
