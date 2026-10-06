//! Crate-local access to the centralized Hyperlimit decision policy.

use core::cmp::Ordering;

use hyperlimit::{PredicateOutcome, Sign, classify_real_sign, compare_reals};
use hyperreal::{Real, RealSign};

pub(crate) trait RealPredicateExt {
    /// Compare two values through Hyperlimit's centralized decision cascade.
    fn predicate_cmp(&self, other: &Self) -> Option<Ordering>;

    /// Classify one sign through Hyperlimit's centralized decision cascade.
    fn predicate_sign(&self) -> Option<RealSign>;

    /// Decide `self < other`, preserving an undecided comparison as `None`.
    fn predicate_lt(&self, other: &Self) -> Option<bool> {
        self.predicate_cmp(other)
            .map(|value| value == Ordering::Less)
    }

    /// Decide `self <= other`, preserving an undecided comparison as `None`.
    #[cfg(feature = "layout")]
    fn predicate_le(&self, other: &Self) -> Option<bool> {
        self.predicate_cmp(other)
            .map(|value| value != Ordering::Greater)
    }

    /// Decide `self > other`, preserving an undecided comparison as `None`.
    #[cfg(feature = "layout")]
    fn predicate_gt(&self, other: &Self) -> Option<bool> {
        self.predicate_cmp(other)
            .map(|value| value == Ordering::Greater)
    }

    /// Decide `self >= other`, preserving an undecided comparison as `None`.
    #[cfg(feature = "layout")]
    fn predicate_ge(&self, other: &Self) -> Option<bool> {
        self.predicate_cmp(other)
            .map(|value| value != Ordering::Less)
    }

    /// Decide numeric equality, preserving an undecided comparison as `None`.
    fn predicate_eq(&self, other: &Self) -> Option<bool> {
        self.predicate_cmp(other)
            .map(|value| value == Ordering::Equal)
    }

    /// Decide numeric inequality, preserving an undecided comparison as `None`.
    fn predicate_ne(&self, other: &Self) -> Option<bool> {
        self.predicate_cmp(other)
            .map(|value| value != Ordering::Equal)
    }
}

impl RealPredicateExt for Real {
    #[inline]
    fn predicate_cmp(&self, other: &Self) -> Option<Ordering> {
        compare_reals(self, other, crate::PREDICATE_POLICY).value()
    }

    #[inline]
    fn predicate_sign(&self) -> Option<RealSign> {
        match classify_real_sign(self, crate::PREDICATE_POLICY) {
            PredicateOutcome::Decided {
                value: Sign::Negative,
                ..
            } => Some(RealSign::Negative),
            PredicateOutcome::Decided {
                value: Sign::Zero, ..
            } => Some(RealSign::Zero),
            PredicateOutcome::Decided {
                value: Sign::Positive,
                ..
            } => Some(RealSign::Positive),
            PredicateOutcome::Unknown { .. } => None,
        }
    }
}
