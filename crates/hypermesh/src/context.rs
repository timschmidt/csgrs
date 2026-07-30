//! Immutable predicate policy and compact operation outcomes.

use std::cell::Cell;

use hyperlimit::{Certainty, PredicateOutcome, PredicatePolicy};

use crate::error::{HypermeshError, HypermeshResult};

/// Immutable policy selected for one mesh operation.
#[repr(transparent)]
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct MeshContext {
    predicates: PredicatePolicy,
}

impl MeshContext {
    /// Construct a context with the selected Hyperlimit predicate policy.
    pub const fn new(predicates: PredicatePolicy) -> Self {
        Self { predicates }
    }

    /// Return the selected predicate policy.
    pub const fn predicate_policy(self) -> PredicatePolicy {
        self.predicates
    }
}

/// Aggregate certainty consumed by a completed mesh operation.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum MeshCertainty {
    /// Every consumed predicate was exact or certified.
    Certified,
    /// At least one decision consumed the policy-authorized 512-bit terminal.
    Approximate512Consumed,
}

/// A completed value paired with its aggregate predicate certainty.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MeshOutcome<T> {
    /// Completed operation value.
    pub value: T,
    /// Weakest certainty consumed while producing `value`.
    pub certainty: MeshCertainty,
}

impl<T> MeshOutcome<T> {
    const fn new(value: T, certainty: MeshCertainty) -> Self {
        Self { value, certainty }
    }

    /// Transform the completed value without changing its certainty.
    pub fn map<U>(self, map: impl FnOnce(T) -> U) -> MeshOutcome<U> {
        MeshOutcome::new(map(self.value), self.certainty)
    }

    /// Consume the outcome and return its value.
    pub fn into_value(self) -> T {
        self.value
    }
}

/// Operation-local owner of policy and aggregate predicate certainty.
#[derive(Debug)]
pub(crate) struct DecisionContext {
    policy: PredicatePolicy,
    certainty: Cell<MeshCertainty>,
}

impl DecisionContext {
    pub(crate) const fn new(context: &MeshContext) -> Self {
        Self {
            policy: context.predicate_policy(),
            certainty: Cell::new(MeshCertainty::Certified),
        }
    }

    pub(crate) const fn policy(&self) -> PredicatePolicy {
        self.policy
    }

    pub(crate) fn certainty(&self) -> MeshCertainty {
        self.certainty.get()
    }

    pub(crate) const fn isolated(&self) -> Self {
        Self {
            policy: self.policy,
            certainty: Cell::new(MeshCertainty::Certified),
        }
    }

    pub(crate) fn finish<T>(&self, value: T) -> MeshOutcome<T> {
        MeshOutcome::new(value, self.certainty.get())
    }

    pub(crate) fn absorb(&self, certainty: MeshCertainty) {
        if certainty == MeshCertainty::Approximate512Consumed {
            self.certainty.set(certainty);
        }
    }

    /// Consume one required Hyperlimit decision.
    pub(crate) fn decide<T>(
        &self,
        outcome: PredicateOutcome<T>,
        predicate: &'static str,
    ) -> HypermeshResult<T> {
        self.probe(outcome)
            .ok_or(HypermeshError::PredicateUndecided { predicate })
    }

    /// Consume one optional decision while allowing a caller to try another
    /// sufficient proof path when this predicate is undecided.
    pub(crate) fn probe<T>(&self, outcome: PredicateOutcome<T>) -> Option<T> {
        match outcome {
            PredicateOutcome::Decided {
                value, certainty, ..
            } => {
                if certainty == Certainty::Approximate {
                    self.certainty.set(MeshCertainty::Approximate512Consumed);
                }
                Some(value)
            }
            PredicateOutcome::Unknown { .. } => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn policy_context_and_certainty_are_one_byte() {
        assert_eq!(core::mem::size_of::<MeshContext>(), 1);
        assert_eq!(core::mem::size_of::<MeshCertainty>(), 1);
    }
}
