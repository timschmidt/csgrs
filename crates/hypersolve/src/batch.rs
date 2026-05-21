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
