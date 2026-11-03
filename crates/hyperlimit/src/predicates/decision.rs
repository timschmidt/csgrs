//! Decision-trace bookkeeping shared by predicates that combine several
//! sub-decisions into one outcome: the strongest certainty and the latest
//! escalation stage consumed so far, and the first unknown sub-decision.

use crate::predicate::{Certainty, Escalation, PredicateOutcome, RefinementNeed};

#[derive(Clone, Copy)]
pub(crate) struct DecisionTrace {
    pub(crate) certainty: Certainty,
    pub(crate) stage: Escalation,
}

impl Default for DecisionTrace {
    fn default() -> Self {
        Self {
            certainty: Certainty::Exact,
            stage: Escalation::Structural,
        }
    }
}

#[derive(Clone, Copy)]
pub(crate) struct UnknownDecision {
    pub(crate) needed: RefinementNeed,
    pub(crate) stage: Escalation,
}

impl UnknownDecision {
    #[inline]
    pub(crate) fn into_outcome<T>(self) -> PredicateOutcome<T> {
        PredicateOutcome::unknown(self.needed, self.stage)
    }
}

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

#[inline]
pub(crate) fn max_certainty(left: Certainty, right: Certainty) -> Certainty {
    if certainty_rank(left) >= certainty_rank(right) {
        left
    } else {
        right
    }
}

#[inline]
pub(crate) fn certainty_rank(certainty: Certainty) -> u8 {
    match certainty {
        Certainty::Exact => 0,
        Certainty::Filtered => 1,
        Certainty::Approximate => 2,
    }
}

#[inline]
pub(crate) fn max_stage(left: Escalation, right: Escalation) -> Escalation {
    if stage_rank(left) >= stage_rank(right) {
        left
    } else {
        right
    }
}

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
