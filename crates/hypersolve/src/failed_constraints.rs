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

/// Result of deactivating one blocking row during failed-constraint search.
///
/// This is the first exact removal-search layer for SolveSpace-like failed
/// constraint diagnostics. It does not claim a globally minimal unsat core;
/// it only answers the cheap and deterministic question "does removing this
/// one blocking row clear the current candidate under exact replay?" Yap's
/// exact-geometric-computation boundary still applies: every probe reruns
/// exact residual diagnostics instead of trusting numerical convergence
/// labels. See Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997).
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum FailedConstraintRemovalStatus {
    /// Removing this single row clears every blocking diagnostic row.
    ClearsAllBlockingRows,
    /// Blocking rows remain after this row is removed.
    StillBlocking {
        /// Number of blocking rows in the reduced problem.
        blocking_rows: usize,
    },
}

/// One single-row failed-constraint removal probe.
#[derive(Clone, Debug, PartialEq)]
pub struct FailedConstraintRemovalProbe {
    /// Source constraint index deactivated for this probe.
    pub constraint_index: usize,
    /// Constraint name copied for diagnostics.
    pub name: String,
    /// Original diagnostic status that made this row blocking.
    pub original_status: FailedConstraintStatus,
    /// Result after deactivating only this row.
    pub removal_status: FailedConstraintRemovalStatus,
}

/// Deterministic single-row removal-search report.
#[derive(Clone, Debug, PartialEq)]
pub struct FailedConstraintRemovalSearchReport {
    /// Original exact failed-constraint diagnostics.
    pub original: FailedConstraintReport,
    /// One probe per original blocking row.
    pub probes: Vec<FailedConstraintRemovalProbe>,
    /// Number of probes whose single removal clears all blocking rows.
    pub clearing_single_removals: usize,
}

/// One two-row failed-constraint removal probe.
#[derive(Clone, Debug, PartialEq)]
pub struct FailedConstraintPairRemovalProbe {
    /// First source constraint index deactivated for this probe.
    pub first_constraint_index: usize,
    /// Second source constraint index deactivated for this probe.
    pub second_constraint_index: usize,
    /// First constraint name copied for diagnostics.
    pub first_name: String,
    /// Second constraint name copied for diagnostics.
    pub second_name: String,
    /// Result after deactivating this pair.
    pub removal_status: FailedConstraintRemovalStatus,
}

/// Deterministic two-row removal-search report.
#[derive(Clone, Debug, PartialEq)]
pub struct FailedConstraintPairRemovalSearchReport {
    /// Original exact failed-constraint diagnostics.
    pub original: FailedConstraintReport,
    /// One probe per pair of original blocking rows.
    pub probes: Vec<FailedConstraintPairRemovalProbe>,
    /// Number of pairs whose removal clears all blocking rows.
    pub clearing_pair_removals: usize,
}

/// One bounded set-removal probe for failed-constraint diagnostics.
#[derive(Clone, Debug, PartialEq)]
pub struct FailedConstraintSetRemovalProbe {
    /// Source constraint indices deactivated for this probe.
    pub constraint_indices: Vec<usize>,
    /// Constraint names copied for diagnostics in the same order as
    /// [`Self::constraint_indices`].
    pub names: Vec<String>,
    /// Result after deactivating this set.
    pub removal_status: FailedConstraintRemovalStatus,
}

/// Deterministic bounded-cardinality removal-search report.
///
/// This is a bounded exact replay search, not a full proof of global minimal
/// unsatisfiable cores. It gives callers a report-bearing way to ask whether
/// removing any blocking set up to `max_cardinality` clears the current
/// candidate. Every probe reruns exact failed-constraint diagnostics, following
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997), rather than trusting numerical failed-solve labels.
#[derive(Clone, Debug, PartialEq)]
pub struct FailedConstraintSetRemovalSearchReport {
    /// Original exact failed-constraint diagnostics.
    pub original: FailedConstraintReport,
    /// Maximum set cardinality requested by the caller.
    pub max_cardinality: usize,
    /// One probe per blocking-row combination up to `max_cardinality`.
    pub probes: Vec<FailedConstraintSetRemovalProbe>,
    /// Number of probed sets whose removal clears all blocking rows.
    pub clearing_removals: usize,
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

impl FailedConstraintRemovalSearchReport {
    /// Return whether any single-row removal clears the current blocking set.
    pub fn has_single_removal_resolution(&self) -> bool {
        self.clearing_single_removals > 0
    }
}

impl FailedConstraintPairRemovalSearchReport {
    /// Return whether any pair removal clears the current blocking set.
    pub fn has_pair_removal_resolution(&self) -> bool {
        self.clearing_pair_removals > 0
    }
}

impl FailedConstraintSetRemovalSearchReport {
    /// Return whether any bounded set removal clears the current blocking set.
    pub fn has_removal_resolution(&self) -> bool {
        self.clearing_removals > 0
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

/// Probe each blocking row by deactivating it and rerunning exact diagnostics.
///
/// This is deliberately a first-order failed-constraint search, not a full
/// minimal unsat-core extractor. It gives UI and API callers a certified,
/// deterministic candidate list for "try removing this one constraint" while
/// leaving multi-row core minimization explicit future work.
pub fn search_failed_constraint_single_removals(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
) -> FailedConstraintRemovalSearchReport {
    search_failed_constraint_single_removals_with_config(
        prepared,
        context,
        CandidateCertificationConfig::default(),
        CandidateCertificationConfig::default().min_precision,
    )
}

/// Probe every pair of blocking rows by deactivating both rows.
///
/// This is the first bounded multi-row search layer. It is useful when no
/// single removal clears the candidate but a pair does. It remains deliberately
/// finite and report-bearing rather than a general minimal unsat-core
/// extractor; each pair is replayed through exact diagnostics following Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997).
pub fn search_failed_constraint_pair_removals(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
) -> FailedConstraintPairRemovalSearchReport {
    search_failed_constraint_pair_removals_with_config(
        prepared,
        context,
        CandidateCertificationConfig::default(),
        CandidateCertificationConfig::default().min_precision,
    )
}

/// Probe all blocking-row sets up to `max_cardinality`.
///
/// This generalizes the single and pair helpers while keeping the search
/// explicitly bounded. `max_cardinality == 0` returns the original diagnostic
/// report with no probes.
pub fn search_failed_constraint_set_removals(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
    max_cardinality: usize,
) -> FailedConstraintSetRemovalSearchReport {
    search_failed_constraint_set_removals_with_config(
        prepared,
        context,
        max_cardinality,
        CandidateCertificationConfig::default(),
        CandidateCertificationConfig::default().min_precision,
    )
}

/// Probe bounded blocking-row sets with explicit certification and rank
/// policies.
pub fn search_failed_constraint_set_removals_with_config(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
    max_cardinality: usize,
    certification_config: CandidateCertificationConfig,
    rank_min_precision: i32,
) -> FailedConstraintSetRemovalSearchReport {
    let original = diagnose_failed_constraints_with_config(
        prepared,
        context,
        certification_config,
        rank_min_precision,
    );
    let blocking = original
        .rows
        .iter()
        .filter(|row| row.status.blocks_candidate_acceptance())
        .collect::<Vec<_>>();
    let capped_cardinality = max_cardinality.min(blocking.len());
    let mut probes = Vec::new();
    let mut clearing_removals = 0;
    for size in 1..=capped_cardinality {
        let mut selected = Vec::with_capacity(size);
        collect_failed_constraint_set_removal_probes(
            prepared,
            context,
            &blocking,
            size,
            0,
            &mut selected,
            certification_config,
            rank_min_precision,
            &mut probes,
            &mut clearing_removals,
        );
    }
    FailedConstraintSetRemovalSearchReport {
        original,
        max_cardinality,
        probes,
        clearing_removals,
    }
}

/// Probe every blocking-row pair with explicit certification and rank policies.
pub fn search_failed_constraint_pair_removals_with_config(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
    certification_config: CandidateCertificationConfig,
    rank_min_precision: i32,
) -> FailedConstraintPairRemovalSearchReport {
    let original = diagnose_failed_constraints_with_config(
        prepared,
        context,
        certification_config,
        rank_min_precision,
    );
    let blocking = original
        .rows
        .iter()
        .filter(|row| row.status.blocks_candidate_acceptance())
        .collect::<Vec<_>>();
    let mut probes = Vec::new();
    let mut clearing_pair_removals = 0;
    for first in 0..blocking.len() {
        for second in (first + 1)..blocking.len() {
            let first_row = blocking[first];
            let second_row = blocking[second];
            let mut reduced = prepared.problem().clone();
            if let Some(constraint) = reduced.constraints.get_mut(first_row.constraint_index) {
                constraint.active = false;
            }
            if let Some(constraint) = reduced.constraints.get_mut(second_row.constraint_index) {
                constraint.active = false;
            }
            let reduced_prepared = PreparedProblem::new(&reduced);
            let reduced_report = diagnose_failed_constraints_with_config(
                &reduced_prepared,
                context,
                certification_config,
                rank_min_precision,
            );
            let removal_status = if reduced_report.blocking_rows == 0 {
                clearing_pair_removals += 1;
                FailedConstraintRemovalStatus::ClearsAllBlockingRows
            } else {
                FailedConstraintRemovalStatus::StillBlocking {
                    blocking_rows: reduced_report.blocking_rows,
                }
            };
            probes.push(FailedConstraintPairRemovalProbe {
                first_constraint_index: first_row.constraint_index,
                second_constraint_index: second_row.constraint_index,
                first_name: first_row.name.clone(),
                second_name: second_row.name.clone(),
                removal_status,
            });
        }
    }
    FailedConstraintPairRemovalSearchReport {
        original,
        probes,
        clearing_pair_removals,
    }
}

/// Probe each blocking row with explicit certification and rank policies.
pub fn search_failed_constraint_single_removals_with_config(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
    certification_config: CandidateCertificationConfig,
    rank_min_precision: i32,
) -> FailedConstraintRemovalSearchReport {
    let original = diagnose_failed_constraints_with_config(
        prepared,
        context,
        certification_config,
        rank_min_precision,
    );
    let mut probes = Vec::new();
    let mut clearing_single_removals = 0;
    for row in original
        .rows
        .iter()
        .filter(|row| row.status.blocks_candidate_acceptance())
    {
        let mut reduced = prepared.problem().clone();
        if let Some(constraint) = reduced.constraints.get_mut(row.constraint_index) {
            constraint.active = false;
        }
        let reduced_prepared = PreparedProblem::new(&reduced);
        let reduced_report = diagnose_failed_constraints_with_config(
            &reduced_prepared,
            context,
            certification_config,
            rank_min_precision,
        );
        let removal_status = if reduced_report.blocking_rows == 0 {
            clearing_single_removals += 1;
            FailedConstraintRemovalStatus::ClearsAllBlockingRows
        } else {
            FailedConstraintRemovalStatus::StillBlocking {
                blocking_rows: reduced_report.blocking_rows,
            }
        };
        probes.push(FailedConstraintRemovalProbe {
            constraint_index: row.constraint_index,
            name: row.name.clone(),
            original_status: row.status.clone(),
            removal_status,
        });
    }
    FailedConstraintRemovalSearchReport {
        original,
        probes,
        clearing_single_removals,
    }
}

#[allow(clippy::too_many_arguments)]
fn collect_failed_constraint_set_removal_probes(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
    blocking: &[&FailedConstraintRow],
    target_size: usize,
    start: usize,
    selected: &mut Vec<usize>,
    certification_config: CandidateCertificationConfig,
    rank_min_precision: i32,
    probes: &mut Vec<FailedConstraintSetRemovalProbe>,
    clearing_removals: &mut usize,
) {
    if selected.len() == target_size {
        let selected_rows = selected
            .iter()
            .map(|index| blocking[*index])
            .collect::<Vec<_>>();
        let mut reduced = prepared.problem().clone();
        for row in &selected_rows {
            if let Some(constraint) = reduced.constraints.get_mut(row.constraint_index) {
                constraint.active = false;
            }
        }
        let reduced_prepared = PreparedProblem::new(&reduced);
        let reduced_report = diagnose_failed_constraints_with_config(
            &reduced_prepared,
            context,
            certification_config,
            rank_min_precision,
        );
        let removal_status = if reduced_report.blocking_rows == 0 {
            *clearing_removals += 1;
            FailedConstraintRemovalStatus::ClearsAllBlockingRows
        } else {
            FailedConstraintRemovalStatus::StillBlocking {
                blocking_rows: reduced_report.blocking_rows,
            }
        };
        probes.push(FailedConstraintSetRemovalProbe {
            constraint_indices: selected_rows
                .iter()
                .map(|row| row.constraint_index)
                .collect(),
            names: selected_rows.iter().map(|row| row.name.clone()).collect(),
            removal_status,
        });
        return;
    }

    let remaining_slots = target_size - selected.len();
    let last_start = blocking.len().saturating_sub(remaining_slots);
    for index in start..=last_start {
        selected.push(index);
        collect_failed_constraint_set_removal_probes(
            prepared,
            context,
            blocking,
            target_size,
            index + 1,
            selected,
            certification_config,
            rank_min_precision,
            probes,
            clearing_removals,
        );
        selected.pop();
    }
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
