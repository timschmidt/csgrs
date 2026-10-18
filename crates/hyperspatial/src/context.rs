use std::{
    cmp::Ordering,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering as AtomicOrdering},
    },
};

use hyperlimit::{PredicateOutcome, PredicatePolicy, compare_reals};
use hyperreal::Real;

use crate::{Decision, Error, InputProvenance, Receipt, Result};

/// Limits on Hyperspatial orchestration, not bounds on scalar heap bytes/time.
#[derive(Clone, Copy, Debug)]
pub struct Limits {
    /// Maximum work checkpoints, including operation entries and predicates.
    pub max_steps: u64,
    /// Maximum number of elements in one atomic batch.
    pub max_batch_len: usize,
    /// Maximum retained evidence path length.
    pub max_receipt_depth: usize,
}

impl Default for Limits {
    fn default() -> Self {
        Self {
            max_steps: 1_000_000,
            max_batch_len: 100_000,
            max_receipt_depth: 128,
        }
    }
}

/// Explicit predicate policy and cooperative resource accounting.
///
/// A step counts a Hyperspatial checkpoint, not every scalar instruction.
/// Hyperlimit currently owns its fixed certified-refinement precision limit.
/// Cancellation is observed before/after native calls and never attached to
/// retained Real values. Individual scalar calls have no new time/heap bound.
#[derive(Debug)]
pub struct Context {
    policy: PredicatePolicy,
    limits: Limits,
    steps: u64,
    cancelled: Arc<AtomicBool>,
}

impl Context {
    /// Creates a fresh evaluation context with an explicit decision policy.
    pub fn new(policy: PredicatePolicy, limits: Limits) -> Self {
        Self {
            policy,
            limits,
            steps: 0,
            cancelled: Arc::new(AtomicBool::new(false)),
        }
    }
    /// Decision policy used for new local predicates.
    pub fn policy(&self) -> PredicatePolicy {
        self.policy
    }
    /// Configured orchestration limits.
    pub fn limits(&self) -> Limits {
        self.limits
    }
    /// Number of work checkpoints consumed so far, including failed operations.
    pub fn steps_used(&self) -> u64 {
        self.steps
    }
    /// Shared cooperative cancellation flag; setting it does not mutate results.
    pub fn cancellation_signal(&self) -> Arc<AtomicBool> {
        self.cancelled.clone()
    }

    pub(crate) fn checkpoint(&mut self) -> Result<()> {
        if self.cancelled.load(AtomicOrdering::Relaxed) {
            return Err(Error::Cancelled);
        }
        if self.steps >= self.limits.max_steps {
            return Err(Error::ResourceExhausted("work checkpoints"));
        }
        self.steps += 1;
        Ok(())
    }

    pub(crate) fn compare(
        &mut self,
        claim: &'static str,
        left: &Real,
        right: &Real,
        decisions: &mut Vec<Decision>,
    ) -> Result<Ordering> {
        self.checkpoint()?;
        let outcome = compare_reals(left, right, self.policy);
        self.checkpoint()?;
        match outcome {
            PredicateOutcome::Decided { value, .. } => {
                decisions.push(Decision {
                    claim,
                    left: left.clone(),
                    right: right.clone(),
                    outcome,
                });
                Ok(value)
            }
            PredicateOutcome::Unknown { needed, stage } => Err(Error::Unknown {
                claim,
                needed,
                stage,
            }),
        }
    }

    pub(crate) fn receipt(
        &mut self,
        operation: &'static str,
        parents: Vec<Receipt>,
        decisions: Vec<Decision>,
        input: Option<InputProvenance>,
    ) -> Result<Receipt> {
        self.checkpoint()?;
        let depth = parents.iter().map(Receipt::depth).max().unwrap_or(0);
        if depth >= self.limits.max_receipt_depth {
            return Err(Error::ResourceExhausted("evidence depth"));
        }
        Ok(Receipt::build(operation, parents, decisions, input))
    }
}
