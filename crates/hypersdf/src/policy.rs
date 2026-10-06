//! Central predicate-policy boundary for retained SDF evaluation.
//!
//! Comparisons that construct a scalar, interval, transform, gradient, or
//! topology certificate must remain strict: an approximate branch choice
//! cannot be repaired by a later predicate. Only a terminal decision whose
//! report retains `Certainty::Approximate` may use Hyperlimit's configured
//! final approximation.

use core::cmp::Ordering;

use hyperlimit::{PredicateOutcome, PredicatePolicy};
use hyperreal::Real;

/// Policy for comparisons whose result feeds another exact construction.
pub(crate) const CONSTRUCTION_PREDICATE_POLICY: PredicatePolicy = PredicatePolicy::STRICT;

/// Policy for terminal decisions retained with their certainty evidence.
pub(crate) const FINAL_PREDICATE_POLICY: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

/// Compare retained scalars without allowing a terminal approximate branch.
#[inline(always)]
pub(crate) fn compare_reals_for_construction(
    left: &Real,
    right: &Real,
) -> PredicateOutcome<Ordering> {
    hyperlimit::compare_reals(left, right, CONSTRUCTION_PREDICATE_POLICY)
}

/// Compare retained scalars for a final report-bearing decision.
#[inline(always)]
pub(crate) fn compare_reals_for_final_decision(
    left: &Real,
    right: &Real,
) -> PredicateOutcome<Ordering> {
    hyperlimit::compare_reals(left, right, FINAL_PREDICATE_POLICY)
}
