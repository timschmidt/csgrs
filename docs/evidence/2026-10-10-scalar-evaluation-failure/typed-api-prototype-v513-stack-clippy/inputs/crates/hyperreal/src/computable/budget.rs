use std::{
    cell::{Cell, RefCell},
    rc::Rc,
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
    },
};

/// A caller-owned, shared allowance for cooperative evaluation work.
///
/// Clones share the same remaining allowance, including across threads and
/// successive queries. Nested scalar requests cannot replenish it. A work unit
/// can represent an evaluator checkpoint, a metered structural visit, a
/// numerical-series iteration or coefficient-size work in algebraic separation
/// searches. The units are implementation-defined, not CPU instructions.
///
/// This is not a byte or elapsed-time limit. Individual integer/rational
/// operations and unmetered algebraic helpers can do additional work between
/// checkpoints. Precision limits still apply independently. Completed caches
/// and facts remain reusable after an allowance is spent.
#[derive(Clone, Debug)]
pub struct EvaluationBudget {
    inner: Arc<Allowance>,
}

#[derive(Debug)]
struct Allowance {
    limit: u64,
    remaining: AtomicU64,
}

impl EvaluationBudget {
    /// Create an allowance shared by this value and all its clones.
    pub fn new(work_units: u64) -> Self {
        Self {
            inner: Arc::new(Allowance {
                limit: work_units,
                remaining: AtomicU64::new(work_units),
            }),
        }
    }

    /// Remaining work units. Concurrent queries may consume more after this read.
    pub fn remaining(&self) -> u64 {
        self.inner.remaining.load(Ordering::Relaxed)
    }

    /// Work units consumed across all queries sharing this allowance.
    pub fn consumed(&self) -> u64 {
        self.inner.limit - self.remaining()
    }

    fn consume(&self, units: u64) -> bool {
        self.inner
            .remaining
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |remaining| {
                remaining.checked_sub(units)
            })
            .is_ok()
    }
}

struct ActiveBudget {
    budget: EvaluationBudget,
    failed: Cell<bool>,
    parent: Option<Rc<ActiveBudget>>,
}

thread_local! {
    static ACTIVE: RefCell<Option<Rc<ActiveBudget>>> = const { RefCell::new(None) };
}

/// Synchronous dynamic scope propagates one explicit root allowance through
/// legacy internal calls, including constructors that make nested requests.
/// No budget is stored on immutable expression nodes or serialized with them.
/// Rc makes the guard thread-bound; Drop also restores the parent on unwinding.
pub(super) struct BudgetScope {
    previous: Option<Rc<ActiveBudget>>,
}

impl BudgetScope {
    pub(super) fn enter(budget: &EvaluationBudget) -> Self {
        ACTIVE.with(|slot| {
            let previous = slot.borrow().clone();
            let mut cursor = previous.as_deref();
            while let Some(frame) = cursor {
                if Arc::ptr_eq(&frame.budget.inner, &budget.inner) {
                    // Reentering an existing allowance must neither double
                    // charge it nor hide a different intervening allowance.
                    return Self { previous };
                }
                cursor = frame.parent.as_deref();
            }
            *slot.borrow_mut() = Some(Rc::new(ActiveBudget {
                budget: budget.clone(),
                failed: Cell::new(false),
                parent: previous.clone(),
            }));
            Self { previous }
        })
    }
}

impl Drop for BudgetScope {
    fn drop(&mut self) {
        ACTIVE.with(|slot| *slot.borrow_mut() = self.previous.take());
    }
}

#[inline]
pub(super) fn exhausted() -> bool {
    ACTIVE.with(|slot| {
        let active = slot.borrow();
        let mut cursor = active.as_deref();
        while let Some(frame) = cursor {
            if frame.failed.get() {
                return true;
            }
            cursor = frame.parent.as_deref();
        }
        false
    })
}

#[inline]
pub(super) fn charge(units: u64) -> bool {
    ACTIVE.with(|slot| {
        let active = slot.borrow();
        let mut cursor = active.as_deref();
        while let Some(frame) = cursor {
            if frame.failed.get() || !frame.budget.consume(units) {
                // Failure is sticky for this request, not the shared allowance:
                // a later request may reuse a finished cache with zero new work.
                let mut mark = active.as_deref();
                while let Some(frame) = mark {
                    frame.failed.set(true);
                    mark = frame.parent.as_deref();
                }
                return false;
            }
            cursor = frame.parent.as_deref();
        }
        true
    })
}
