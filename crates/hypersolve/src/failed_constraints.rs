//! Exact failed-constraint and redundancy diagnostics.
//!
//! Candidate certification answers whether each active residual row is
//! satisfied, violated, unknown, or domain-failed. CAD/sketch solver APIs also
//! need a source-row diagnostic list for failed-constraint displays and
//! SolveSpace-style status reporting. This module builds that list without
//! introducing tolerances: it combines exact residual replay with exact affine
//! rank checks. That follows Yap's exact-geometric-computation boundary:
//! numerical iteration may propose a candidate, but failure explanations must
//! be certified or explicitly undecided. See C. K. Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997). The rank
//! redundancy test uses Bareiss-backed affine rank reports; see Bareiss,
//! "Sylvester's Identity and Multistep Integer-Preserving Gaussian
//! Elimination" (1968).

use crate::certification::{
    CandidateCertificationConfig, CandidateCertificationReport, CertifiedCandidateRow,
    CertifiedCandidateStatus, certify_candidate_with_config,
};
use crate::eval::EvaluationContext;
use crate::model::ConstraintKind;
use crate::prepared::PreparedProblem;
use crate::rank::{ExactAffineRankReport, ExactAffineRankStatus, analyze_exact_affine_rank};

/// Per-source-row diagnostic status for failed-constraint reporting.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum FailedConstraintStatus {
    /// A structurally constant active row was proved impossible.
    ///
    /// For equality-like rows this means a nonzero constant residual. For
    /// inequalities this means a constant signed residual that violates the
    /// normalized inequality. It is a model contradiction, not merely a bad
    /// candidate.
    CertifiedContradiction,
    /// The current candidate was proved to violate this row, but the row is
    /// not structurally constant.
    CertifiedCandidateViolation,
    /// The row is exactly rank-redundant in the active affine equality system.
    RankRedundant,
    /// The row only carries lossy proposal evidence; exact replay has not been
    /// performed for it.
    DidNotConvergeOnly,
    /// Exact replay hit a domain/evaluation failure.
    DomainFailure,
    /// Exact replay completed but bounded sign refinement did not decide.
    BoundedUnknown,
}

impl FailedConstraintStatus {
    /// Returns whether this row blocks accepting the current candidate.
    pub const fn blocks_candidate_acceptance(&self) -> bool {
        matches!(
            self,
            Self::CertifiedContradiction
                | Self::CertifiedCandidateViolation
                | Self::DidNotConvergeOnly
                | Self::DomainFailure
                | Self::BoundedUnknown
        )
    }
}

/// One source-row diagnostic emitted for a failed or redundant constraint.
#[derive(Clone, Debug, PartialEq)]
pub struct FailedConstraintRow {
    /// Source constraint index in the original problem.
    pub constraint_index: usize,
    /// Constraint name copied for diagnostics.
    pub name: String,
    /// Constraint kind used during residual normalization.
    pub kind: ConstraintKind,
    /// Diagnostic status.
    pub status: FailedConstraintStatus,
    /// Candidate-certification row that justified this diagnostic, when the
    /// diagnostic came from exact or proposal-only replay.
    pub certification: Option<CertifiedCandidateRow>,
    /// Exact affine rank after removing this row, when redundancy was checked.
    pub rank_without_row: Option<ExactAffineRankReport>,
}

/// Report for exact failed-constraint diagnostics.
#[derive(Clone, Debug, PartialEq)]
pub struct FailedConstraintReport {
    /// Rows that are failed, unresolved, proposal-only, or rank-redundant.
    pub rows: Vec<FailedConstraintRow>,
    /// Candidate certification consumed by the diagnostic pass.
    pub certification: CandidateCertificationReport,
    /// Exact affine rank of the original prepared problem.
    pub affine_rank: ExactAffineRankReport,
    /// Number of rows that block accepting the current candidate.
    pub blocking_rows: usize,
    /// Number of structurally certified contradictions.
    pub certified_contradictions: usize,
    /// Number of nonconstant candidate violations.
    pub certified_candidate_violations: usize,
    /// Number of exact rank-redundant rows.
    pub rank_redundant_rows: usize,
    /// Number of rows with lossy/proposal-only evidence.
    pub did_not_converge_only_rows: usize,
    /// Number of rows with domain failures.
    pub domain_failure_rows: usize,
    /// Number of rows left bounded-unknown.
    pub bounded_unknown_rows: usize,
}

impl FailedConstraintReport {
    /// Returns true when any diagnostic row blocks accepting the candidate.
    pub fn has_blocking_rows(&self) -> bool {
        self.blocking_rows > 0
    }

    /// Returns true when the report found only exact rank redundancy.
    pub fn only_rank_redundancy(&self) -> bool {
        !self.rows.is_empty() && self.blocking_rows == 0
    }
}

/// Certify a candidate and emit exact failed-constraint diagnostics.
pub fn diagnose_failed_constraints(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
) -> FailedConstraintReport {
    diagnose_failed_constraints_with_config(
        prepared,
        context,
        CandidateCertificationConfig::default(),
        CandidateCertificationConfig::default().min_precision,
    )
}

/// Certify a candidate and emit exact failed-constraint diagnostics with an
/// explicit residual-sign and affine-rank policy.
pub fn diagnose_failed_constraints_with_config(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
    certification_config: CandidateCertificationConfig,
    rank_min_precision: i32,
) -> FailedConstraintReport {
    let certification = certify_candidate_with_config(prepared, context, certification_config);
    diagnose_failed_constraints_from_certification(prepared, certification, rank_min_precision)
}

/// Emit failed-constraint diagnostics from an already-built certification
/// report.
///
/// This overload lets callers preserve proposal-only reports, such as
/// [`crate::report_lossy_adapter_only_candidate`], while still getting the same
/// source-row diagnostic shape used by exact replay.
pub fn diagnose_failed_constraints_from_certification(
    prepared: &PreparedProblem<'_>,
    certification: CandidateCertificationReport,
    rank_min_precision: i32,
) -> FailedConstraintReport {
    let affine_rank = analyze_exact_affine_rank(prepared, rank_min_precision);
    let mut rows = Vec::new();

    for certification_row in &certification.rows {
        if let Some(status) = failure_status(prepared, certification_row) {
            rows.push(FailedConstraintRow {
                constraint_index: certification_row.constraint_index,
                name: certification_row.name.clone(),
                kind: certification_row.kind,
                status,
                certification: Some(certification_row.clone()),
                rank_without_row: None,
            });
        }
    }

    if matches!(
        affine_rank.status,
        ExactAffineRankStatus::Certified | ExactAffineRankStatus::Inconsistent
    ) {
        for certification_row in &certification.rows {
            if rows
                .iter()
                .any(|row| row.constraint_index == certification_row.constraint_index)
            {
                continue;
            }
            if !matches!(
                certification_row.status,
                CertifiedCandidateStatus::CertifiedZero { .. }
            ) {
                continue;
            }
            let Some(rank_without_row) = rank_without_source_row(
                prepared,
                certification_row.constraint_index,
                rank_min_precision,
            ) else {
                continue;
            };
            if rank_without_row.status == affine_rank.status
                && rank_without_row.coefficient_rank == affine_rank.coefficient_rank
                && rank_without_row.augmented_rank == affine_rank.augmented_rank
            {
                rows.push(FailedConstraintRow {
                    constraint_index: certification_row.constraint_index,
                    name: certification_row.name.clone(),
                    kind: certification_row.kind,
                    status: FailedConstraintStatus::RankRedundant,
                    certification: Some(certification_row.clone()),
                    rank_without_row: Some(rank_without_row),
                });
            }
        }
    }

    rows.sort_by_key(|row| row.constraint_index);
    build_report(rows, certification, affine_rank)
}

fn failure_status(
    prepared: &PreparedProblem<'_>,
    row: &CertifiedCandidateRow,
) -> Option<FailedConstraintStatus> {
    match &row.status {
        status if status.is_certified_violation() => {
            if prepared
                .constraints()
                .get(row.constraint_index)
                .is_some_and(|facts| facts.is_constant_row())
            {
                Some(FailedConstraintStatus::CertifiedContradiction)
            } else {
                Some(FailedConstraintStatus::CertifiedCandidateViolation)
            }
        }
        CertifiedCandidateStatus::LossyAdapterOnly { .. } => {
            Some(FailedConstraintStatus::DidNotConvergeOnly)
        }
        CertifiedCandidateStatus::BoundedUnknown { .. } => {
            Some(FailedConstraintStatus::BoundedUnknown)
        }
        CertifiedCandidateStatus::DomainFailure { .. }
        | CertifiedCandidateStatus::InvalidBallRadius => {
            Some(FailedConstraintStatus::DomainFailure)
        }
        _ => None,
    }
}

fn rank_without_source_row(
    prepared: &PreparedProblem<'_>,
    constraint_index: usize,
    min_precision: i32,
) -> Option<ExactAffineRankReport> {
    let constraint = prepared.problem().constraints.get(constraint_index)?;
    if !constraint.active
        || !matches!(
            constraint.kind,
            ConstraintKind::Equality | ConstraintKind::Soft
        )
        || prepared.affine_residuals().get(constraint_index)?.is_none()
    {
        return None;
    }
    let mut reduced = prepared.problem().clone();
    reduced.constraints[constraint_index].active = false;
    let reduced_prepared = PreparedProblem::new(&reduced);
    Some(analyze_exact_affine_rank(&reduced_prepared, min_precision))
}

fn build_report(
    rows: Vec<FailedConstraintRow>,
    certification: CandidateCertificationReport,
    affine_rank: ExactAffineRankReport,
) -> FailedConstraintReport {
    let blocking_rows = rows
        .iter()
        .filter(|row| row.status.blocks_candidate_acceptance())
        .count();
    let certified_contradictions = rows
        .iter()
        .filter(|row| row.status == FailedConstraintStatus::CertifiedContradiction)
        .count();
    let certified_candidate_violations = rows
        .iter()
        .filter(|row| row.status == FailedConstraintStatus::CertifiedCandidateViolation)
        .count();
    let rank_redundant_rows = rows
        .iter()
        .filter(|row| row.status == FailedConstraintStatus::RankRedundant)
        .count();
    let did_not_converge_only_rows = rows
        .iter()
        .filter(|row| row.status == FailedConstraintStatus::DidNotConvergeOnly)
        .count();
    let domain_failure_rows = rows
        .iter()
        .filter(|row| row.status == FailedConstraintStatus::DomainFailure)
        .count();
    let bounded_unknown_rows = rows
        .iter()
        .filter(|row| row.status == FailedConstraintStatus::BoundedUnknown)
        .count();

    FailedConstraintReport {
        rows,
        certification,
        affine_rank,
        blocking_rows,
        certified_contradictions,
        certified_candidate_violations,
        rank_redundant_rows,
        did_not_converge_only_rows,
        domain_failure_rows,
        bounded_unknown_rows,
    }
}
