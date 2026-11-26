//! Local receipt bookkeeping for predicates that consume multiple decisions.
//!
//! Decided children contribute the weakest certainty label and highest-ranked
//! stage seen so far. These are summaries, not a chronological trace, source
//! provenance or a resource counter. An Unknown is returned to the caller
//! without changing the accumulated receipt; the caller decides whether to
//! stop, so this module does not itself retain a first unknown or cancel work.

use crate::predicate::{Certainty, Escalation, PredicateOutcome, RefinementNeed};

#[derive(Clone, Copy)]
/// Aggregate qualification of the decided children consumed by one classifier.
pub(crate) struct DecisionTrace {
    /// Weakest consumed label in the order Exact, Filtered, Approximate.
    pub(crate) certainty: Certainty,
    /// Highest dispatch-stage rank consumed, independent of chronological order.
    pub(crate) stage: Escalation,
}

impl Default for DecisionTrace {
    /// Start with Exact/Structural, the identity for joining later decision receipts.
    /// This initial state alone does not prove any geometric claim.
    fn default() -> Self {
        Self {
            certainty: Certainty::Exact,
            stage: Escalation::Structural,
        }
    }
}

#[derive(Clone, Copy)]
/// An unresolved child's capability hint and stage, separate from earlier decided receipts.
pub(crate) struct UnknownDecision {
    /// The child's hint, forwarded without reinterpretation as invalidity or exhaustion.
    pub(crate) needed: RefinementNeed,
    /// The stage where that child stopped, not the aggregate stage of previous children.
    pub(crate) stage: Escalation,
}

impl UnknownDecision {
    /// Rebuild a typed Unknown outcome without a value or earlier aggregate receipt.
    #[inline]
    pub(crate) fn into_outcome<T>(self) -> PredicateOutcome<T> {
        PredicateOutcome::unknown(self.needed, self.stage)
    }
}

/// Extract a decided value while joining its receipt, or return an unresolved child unchanged.
/// The trace is untouched on Err. Callers must publish the accumulated receipt
/// with their eventual result; this helper does not check policy admissibility
/// or decide whether a child was logically necessary.
#[inline]
pub(crate) fn decided<T>(
    outcome: PredicateOutcome<T>,
    trace: &mut DecisionTrace,
) -> Result<T, UnknownDecision> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => {
            trace.certainty = max_certainty(trace.certainty, certainty);
            trace.stage = max_stage(trace.stage, stage);
            Ok(value)
        }
        PredicateOutcome::Unknown { needed, stage } => Err(UnknownDecision { needed, stage }),
    }
}

/// Retain the weaker certainty label, choosing the left on equal ranks.
/// Exact and Filtered both certify the result; Approximate marks qualification.
#[inline]
pub(crate) fn max_certainty(left: Certainty, right: Certainty) -> Certainty {
    if certainty_rank(left) >= certainty_rank(right) {
        left
    } else {
        right
    }
}

/// Rank receipt labels for conservative composition: Exact=0, Filtered=1, Approximate=2.
#[inline]
pub(crate) fn certainty_rank(certainty: Certainty) -> u8 {
    match certainty {
        Certainty::Exact => 0,
        Certainty::Filtered => 1,
        Certainty::Approximate => 2,
    }
}

/// Retain the higher-ranked stage, choosing the left on ties; no timing order is inferred.
#[inline]
pub(crate) fn max_stage(left: Escalation, right: Escalation) -> Escalation {
    if stage_rank(left) >= stage_rank(right) {
        left
    } else {
        right
    }
}

/// Rank Structural, Filter, Exact, Refined and Undecided for outcome summaries.
/// Exact recognition may run after refinement, so these numbers are not a timeline.
#[inline]
pub(crate) fn stage_rank(stage: Escalation) -> u8 {
    match stage {
        Escalation::Structural => 0,
        Escalation::Filter => 1,
        Escalation::Exact => 2,
        Escalation::Refined => 3,
        Escalation::Undecided => 4,
    }
}
