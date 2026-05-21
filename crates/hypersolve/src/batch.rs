//! Deterministic batched candidate certification.
//!
//! Large sketches often produce several independent candidate assignments:
//! dragged positions, branch choices, direct algebraic proposals, or lossy
//! iteration snapshots. This module batches exact residual replay while keeping
//! the final report order deterministic. It does not introduce a primitive
//! floating acceptance threshold or hide failed rows. That follows Yap's
//! Exact Geometric Computation boundary: proposal generation may be batched or
//! parallelized, but certified residual facts decide acceptance. See Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997).

use crate::certification::{
    CandidateCertificationConfig, CandidateCertificationReport, CertifiedCandidateStatus,
    certify_candidate_with_config,
};
use crate::eval::EvaluationContext;
use crate::prepared::PreparedProblem;

/// Error returned while building a deterministic batch predicate schedule.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BatchPredicateScheduleError {
    /// Work items must contain at least one active source row.
    ZeroRowsPerWorkItem,
}

/// Configuration for deterministic candidate-row work scheduling.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BatchPredicateScheduleConfig {
    /// Maximum active source rows placed into one work item.
    pub max_rows_per_work_item: usize,
}

impl Default for BatchPredicateScheduleConfig {
    fn default() -> Self {
        Self {
            max_rows_per_work_item: 1,
        }
    }
}

/// One deterministic candidate-row work item.
///
/// This is a scheduling report, not proof evidence. It describes independent
/// exact replay work that may be executed sequentially or in parallel, but the
/// final candidate acceptance still comes from row certification. That
/// separation follows Yap, "Towards Exact Geometric Computation" (1997):
/// scheduling is implementation detail, exact predicates decide truth.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BatchPredicateWorkItem {
    /// Stable work-item ordinal.
    pub work_index: usize,
    /// Candidate context index from caller input order.
    pub candidate_index: usize,
    /// Source constraint indices included in this work item.
    pub source_constraints: Vec<usize>,
}

/// Deterministic schedule for batched candidate predicate replay.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BatchPredicateScheduleReport {
    /// Number of candidate contexts scheduled.
    pub candidate_count: usize,
    /// Number of active source rows per candidate.
    pub active_row_count: usize,
    /// Maximum active source rows requested per work item.
    pub max_rows_per_work_item: usize,
    /// Stable work items in execution/report order.
    pub work_items: Vec<BatchPredicateWorkItem>,
}

impl BatchPredicateScheduleReport {
    /// Number of work items in the deterministic schedule.
    pub fn work_item_count(&self) -> usize {
        self.work_items.len()
    }
}

/// Batch-level status for one candidate replay.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BatchCandidateStatus {
    /// Every active row was certified satisfied.
    Certified,
    /// At least one active row was certified violated.
    Rejected,
    /// No row was proved violated, but at least one row remained uncertain.
    Unknown,
    /// At least one row failed during exact expression/domain evaluation.
    DomainFailure,
}

/// Deterministic replay result for one candidate in a batch.
#[derive(Clone, Debug, PartialEq)]
pub struct BatchCandidateReplay {
    /// Candidate ordinal from the caller-supplied input order.
    pub candidate_index: usize,
    /// Exact replay report for this candidate.
    pub certification: CandidateCertificationReport,
    /// Candidate-level status derived from the row report.
    pub status: BatchCandidateStatus,
    /// First source constraint index that did not certify satisfied.
    pub first_failed_constraint: Option<usize>,
    /// Source constraints with certified violations.
    pub violated_constraints: Vec<usize>,
    /// Source constraints left explicitly uncertain.
    pub unknown_constraints: Vec<usize>,
    /// Source constraints that failed exact expression/domain evaluation.
    pub domain_failure_constraints: Vec<usize>,
}

/// Deterministic batch certification report.
#[derive(Clone, Debug, PartialEq)]
pub struct BatchCandidateCertificationReport {
    /// Per-candidate reports in input order.
    pub candidates: Vec<BatchCandidateReplay>,
    /// Number of candidate contexts examined.
    pub candidate_count: usize,
    /// Number of candidates certified as exact solutions.
    pub certified_candidates: usize,
    /// Number of candidates rejected by certified violations.
    pub rejected_candidates: usize,
    /// Number of candidates with unresolved rows and no certified violation.
    pub unknown_candidates: usize,
    /// Number of candidates with exact expression/domain failures.
    pub domain_failure_candidates: usize,
}

impl BatchCandidateCertificationReport {
    /// Returns true when at least one candidate certified all active rows.
    pub fn has_certified_candidate(&self) -> bool {
        self.certified_candidates > 0
    }
}

/// Certify candidate contexts in deterministic input order with the default
/// replay policy.
pub fn certify_candidate_batch(
    prepared: &PreparedProblem<'_>,
    candidates: &[EvaluationContext],
) -> BatchCandidateCertificationReport {
    certify_candidate_batch_with_config(
        prepared,
        candidates,
        CandidateCertificationConfig::default(),
    )
}

/// Certify candidate contexts in deterministic input order with an explicit
/// replay policy.
///
/// This is intentionally a report-oriented facade over
/// [`crate::certify_candidate_with_config`]. Future parallel implementations
/// can split the same independent candidate work, but they must preserve this
/// stable output order and the failed-row probes.
pub fn certify_candidate_batch_with_config(
    prepared: &PreparedProblem<'_>,
    candidates: &[EvaluationContext],
    config: CandidateCertificationConfig,
) -> BatchCandidateCertificationReport {
    let candidates = candidates
        .iter()
        .enumerate()
        .map(|(candidate_index, context)| {
            let certification = certify_candidate_with_config(prepared, context, config);
            replay_from_certification(candidate_index, certification)
        })
        .collect::<Vec<_>>();

    let certified_candidates = candidates
        .iter()
        .filter(|candidate| candidate.status == BatchCandidateStatus::Certified)
        .count();
    let rejected_candidates = candidates
        .iter()
        .filter(|candidate| candidate.status == BatchCandidateStatus::Rejected)
        .count();
    let unknown_candidates = candidates
        .iter()
        .filter(|candidate| candidate.status == BatchCandidateStatus::Unknown)
        .count();
    let domain_failure_candidates = candidates
        .iter()
        .filter(|candidate| candidate.status == BatchCandidateStatus::DomainFailure)
        .count();

    BatchCandidateCertificationReport {
        candidate_count: candidates.len(),
        candidates,
        certified_candidates,
        rejected_candidates,
        unknown_candidates,
        domain_failure_candidates,
    }
}

/// Build a deterministic candidate/row work schedule for exact replay.
///
/// The schedule chunks active source constraints for each candidate. It is
/// intentionally separate from [`certify_candidate_batch_with_config`] so a
/// parallel backend can consume the same stable work list without changing the
/// certification report shape or using primitive-float acceptance shortcuts.
pub fn schedule_candidate_batch_predicates(
    prepared: &PreparedProblem<'_>,
    candidate_count: usize,
    config: BatchPredicateScheduleConfig,
) -> Result<BatchPredicateScheduleReport, BatchPredicateScheduleError> {
    if config.max_rows_per_work_item == 0 {
        return Err(BatchPredicateScheduleError::ZeroRowsPerWorkItem);
    }

    let active_rows = prepared
        .constraints()
        .iter()
        .enumerate()
        .filter_map(|(source_constraint, facts)| facts.active.then_some(source_constraint))
        .collect::<Vec<_>>();

    let mut work_items = Vec::new();
    for candidate_index in 0..candidate_count {
        for chunk in active_rows.chunks(config.max_rows_per_work_item) {
            work_items.push(BatchPredicateWorkItem {
                work_index: work_items.len(),
                candidate_index,
                source_constraints: chunk.to_vec(),
            });
        }
    }

    Ok(BatchPredicateScheduleReport {
        candidate_count,
        active_row_count: active_rows.len(),
        max_rows_per_work_item: config.max_rows_per_work_item,
        work_items,
    })
}

fn replay_from_certification(
    candidate_index: usize,
    certification: CandidateCertificationReport,
) -> BatchCandidateReplay {
    let violated_constraints = certification
        .rows
        .iter()
        .filter(|row| row.status.is_certified_violation())
        .map(|row| row.constraint_index)
        .collect::<Vec<_>>();
    let unknown_constraints = certification
        .rows
        .iter()
        .filter(|row| row.status.is_unknown())
        .map(|row| row.constraint_index)
        .collect::<Vec<_>>();
    let domain_failure_constraints = certification
        .rows
        .iter()
        .filter(|row| {
            matches!(
                row.status,
                CertifiedCandidateStatus::DomainFailure { .. }
                    | CertifiedCandidateStatus::InvalidBallRadius
            )
        })
        .map(|row| row.constraint_index)
        .collect::<Vec<_>>();

    let first_failed_constraint = certification
        .rows
        .iter()
        .find(|row| !row.status.is_certified_satisfied())
        .map(|row| row.constraint_index);
    let status = if !domain_failure_constraints.is_empty() {
        BatchCandidateStatus::DomainFailure
    } else if !violated_constraints.is_empty() {
        BatchCandidateStatus::Rejected
    } else if !unknown_constraints.is_empty() {
        BatchCandidateStatus::Unknown
    } else if certification.all_satisfied() {
        BatchCandidateStatus::Certified
    } else {
        BatchCandidateStatus::Unknown
    };

    BatchCandidateReplay {
        candidate_index,
        certification,
        status,
        first_failed_constraint,
        violated_constraints,
        unknown_constraints,
        domain_failure_constraints,
    }
}

#[cfg(test)]
mod tests {
    use proptest::prelude::*;

    use super::*;
    use crate::{Constraint, Expr, Problem, SymbolId};
    use hyperreal::Real;

    fn problem_with_rows(row_count: usize) -> Problem {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", Real::zero());
        for row in 0..row_count {
            let mut constraint = Constraint::equality(format!("row {row}"), x.clone());
            constraint.active = row % 3 != 1;
            problem.add_constraint(constraint);
        }
        problem
    }

    #[test]
    fn predicate_schedule_chunks_active_rows_deterministically() {
        let problem = problem_with_rows(5);
        let prepared = PreparedProblem::new(&problem);
        let schedule = schedule_candidate_batch_predicates(
            &prepared,
            2,
            BatchPredicateScheduleConfig {
                max_rows_per_work_item: 2,
            },
        )
        .unwrap();

        assert_eq!(schedule.candidate_count, 2);
        assert_eq!(schedule.active_row_count, 3);
        assert_eq!(schedule.work_item_count(), 4);
        assert_eq!(schedule.work_items[0].candidate_index, 0);
        assert_eq!(schedule.work_items[0].source_constraints, vec![0, 2]);
        assert_eq!(schedule.work_items[1].source_constraints, vec![3]);
        assert_eq!(schedule.work_items[2].candidate_index, 1);
        assert_eq!(schedule.work_items[2].source_constraints, vec![0, 2]);
    }

    #[test]
    fn predicate_schedule_rejects_zero_sized_chunks() {
        let problem = problem_with_rows(1);
        let prepared = PreparedProblem::new(&problem);

        assert_eq!(
            schedule_candidate_batch_predicates(
                &prepared,
                1,
                BatchPredicateScheduleConfig {
                    max_rows_per_work_item: 0,
                },
            )
            .unwrap_err(),
            BatchPredicateScheduleError::ZeroRowsPerWorkItem
        );
    }

    proptest! {
        #[test]
        fn generated_predicate_schedules_cover_every_active_row_per_candidate(
            row_count in 0_usize..32,
            candidate_count in 0_usize..16,
            chunk in 1_usize..8,
        ) {
            let problem = problem_with_rows(row_count);
            let prepared = PreparedProblem::new(&problem);
            let schedule = schedule_candidate_batch_predicates(
                &prepared,
                candidate_count,
                BatchPredicateScheduleConfig { max_rows_per_work_item: chunk },
            ).unwrap();
            let active_rows = prepared
                .constraints()
                .iter()
                .enumerate()
                .filter_map(|(source_constraint, facts)| facts.active.then_some(source_constraint))
                .collect::<Vec<_>>();

            prop_assert_eq!(schedule.active_row_count, active_rows.len());
            prop_assert_eq!(schedule.candidate_count, candidate_count);
            for candidate_index in 0..candidate_count {
                let scheduled_rows = schedule
                    .work_items
                    .iter()
                    .filter(|item| item.candidate_index == candidate_index)
                    .flat_map(|item| item.source_constraints.iter().copied())
                    .collect::<Vec<_>>();
                prop_assert_eq!(scheduled_rows, active_rows.as_slice());
            }
            let work_indices_are_stable = schedule
                .work_items
                .iter()
                .enumerate()
                .all(|(index, item)| item.work_index == index);
            prop_assert!(work_indices_are_stable);
        }
    }
}
