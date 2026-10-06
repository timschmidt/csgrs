//! Immutable policy input and compact operation outcomes.

use hyperlimit::PredicatePolicy;

/// Immutable policy selected for one triangulation operation.
#[repr(transparent)]
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct TriangulationContext {
    predicates: PredicatePolicy,
}

impl TriangulationContext {
    /// Construct a context with the selected Hyperlimit predicate policy.
    pub const fn new(predicates: PredicatePolicy) -> Self {
        Self { predicates }
    }

    /// Return the selected predicate policy.
    pub const fn predicate_policy(self) -> PredicatePolicy {
        self.predicates
    }
}

/// Aggregate certainty consumed by a completed triangulation operation.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum TriangulationCertainty {
    /// Every consumed predicate was exact or certified.
    Certified,
    /// At least one decision consumed the policy-authorized 512-bit terminal.
    Approximate512Consumed,
}

/// A completed value paired with its aggregate predicate certainty.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TriangulationOutcome<T> {
    /// Completed operation value.
    pub value: T,
    /// Weakest certainty consumed while producing `value`.
    pub certainty: TriangulationCertainty,
}

impl<T> TriangulationOutcome<T> {
    pub(crate) const fn new(value: T, certainty: TriangulationCertainty) -> Self {
        Self { value, certainty }
    }

    /// Transform the completed value without changing its certainty.
    pub fn map<U>(self, map: impl FnOnce(T) -> U) -> TriangulationOutcome<U> {
        TriangulationOutcome::new(map(self.value), self.certainty)
    }

    /// Consume the outcome and return its value.
    pub fn into_value(self) -> T {
        self.value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn policy_context_and_certainty_remain_one_byte() {
        assert_eq!(core::mem::size_of::<TriangulationContext>(), 1);
        assert_eq!(core::mem::size_of::<TriangulationCertainty>(), 1);
    }
}
