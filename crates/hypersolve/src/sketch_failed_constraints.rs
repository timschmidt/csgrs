//! Sketch-level failed-constraint diagnostics.
//!
//! Generic residual diagnostics know only lowered row indices. Retained sketch
//! callers need the same exact evidence grouped back onto semantic source
//! constraints and generated-row provenance. This module performs that join
//! without introducing tolerances: it lowers the sketch, runs exact candidate
//! diagnostics, then replays the residual-index mapping carried by
//! [`crate::sketch::SketchGeneratedRow`]. The design follows the exact Exact
//! Geometric Computation boundary: numerical state may be a proposal, but
//! failed-constraint evidence is accepted only as exact/certified diagnostics
//! or explicit lowering failure. The source-facing
//! grouping mirrors the user-visible failed-constraint reporting described in
//! the geometric-constraint-solver model.

use crate::certification::{CandidateCertificationConfig, certify_candidate_with_config};
use crate::eval::context_from_problem;
use crate::failed_constraints::{
    FailedConstraintReport, FailedConstraintRow, FailedConstraintStatus,
    diagnose_failed_constraints_from_certification,
};
use crate::prepared::PreparedProblem;
use crate::sketch::{
    SketchConstraintHandle, SketchGeneratedRow, SketchGeneratedRowStatus, SketchLoweringReport,
    SketchSolveProblem,
};

/// One sketch-source diagnostic row.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchFailedConstraintRow {
    /// Retained source constraint handle.
    pub constraint: SketchConstraintHandle,
    /// Retained source constraint name.
    pub constraint_name: String,
    /// Generated-row provenance copied from sketch lowering.
    pub generated: SketchGeneratedRow,
    /// Exact generic failed-row diagnostic, when lowering produced a residual.
    pub failed: Option<FailedConstraintRow>,
    /// Compact status for sketch consumers.
    pub status: SketchFailedConstraintStatus,
}

/// Sketch-level failed-constraint row status.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SketchFailedConstraintStatus {
    /// Lowering did not produce the row, so exact residual replay could not
    /// run for this source constraint.
    LoweringFailure(SketchGeneratedRowStatus),
    /// Exact residual replay found a blocking failed row.
    ExactFailure(FailedConstraintStatus),
    /// Exact rank replay found a redundant generated row.
    RankRedundant,
}

impl SketchFailedConstraintStatus {
    /// Returns whether this status blocks accepting the current sketch state.
    pub const fn blocks_candidate_acceptance(&self) -> bool {
        match self {
            Self::LoweringFailure(_) => true,
            Self::ExactFailure(status) => status.blocks_candidate_acceptance(),
            Self::RankRedundant => false,
        }
    }
}

/// Report joining exact failed-row diagnostics to retained sketch constraints.
#[derive(Clone, Debug)]
pub struct SketchFailedConstraintReport {
    /// Lowered problem and row provenance.
    pub lowering: SketchLoweringReport,
    /// Generic exact failed-constraint report over the lowered problem.
    pub exact: FailedConstraintReport,
    /// Sketch-source diagnostic rows.
    pub rows: Vec<SketchFailedConstraintRow>,
    /// Number of sketch-source rows that block accepting the current state.
    pub blocking_rows: usize,
    /// Number of source rows that failed before residual replay because
    /// lowering could not generate the row.
    pub lowering_failure_rows: usize,
    /// Number of source rows with exact/certified blocking diagnostics.
    pub exact_failure_rows: usize,
    /// Number of source rows reported as exact rank-redundant.
    pub rank_redundant_rows: usize,
}

impl SketchFailedConstraintReport {
    /// Returns true when at least one sketch-source row blocks acceptance.
    pub fn has_blocking_rows(&self) -> bool {
        self.blocking_rows > 0
    }

    /// Returns true when the report found only exact rank redundancy.
    pub fn only_rank_redundancy(&self) -> bool {
        !self.rows.is_empty()
            && self.blocking_rows == 0
            && self.rank_redundant_rows == self.rows.len()
    }
}

/// Diagnose failed constraints for a retained sketch using default settings.
pub fn diagnose_sketch_failed_constraints(
    sketch: &SketchSolveProblem,
) -> SketchFailedConstraintReport {
    diagnose_sketch_failed_constraints_with_config(
        sketch,
        CandidateCertificationConfig::default(),
        CandidateCertificationConfig::default().min_precision,
    )
}

/// Diagnose failed constraints for a retained sketch with explicit settings.
///
/// The function lowers the sketch once, certifies the current exact parameter
/// values as a candidate, and maps each failed lowered row back to the retained
/// source constraint that generated it. Rows that did not lower are reported
/// directly as sketch lowering failures.
pub fn diagnose_sketch_failed_constraints_with_config(
    sketch: &SketchSolveProblem,
    certification_config: CandidateCertificationConfig,
    rank_min_precision: i32,
) -> SketchFailedConstraintReport {
    let lowering = sketch.lower_to_problem();
    let prepared = PreparedProblem::new(&lowering.problem);
    let context = context_from_problem(&lowering.problem);
    let certification = certify_candidate_with_config(&prepared, &context, certification_config);
    let exact = diagnose_failed_constraints_from_certification(
        &prepared,
        certification,
        rank_min_precision,
    );
    let rows = join_sketch_failed_rows(sketch, &lowering, &exact);
    build_report(lowering, exact, rows)
}

fn join_sketch_failed_rows(
    sketch: &SketchSolveProblem,
    lowering: &SketchLoweringReport,
    exact: &FailedConstraintReport,
) -> Vec<SketchFailedConstraintRow> {
    let mut rows = Vec::new();

    for generated in lowering.rows.iter().filter(|row| {
        row.status != SketchGeneratedRowStatus::Generated
            && row.status != SketchGeneratedRowStatus::ReferenceOnly
    }) {
        rows.push(SketchFailedConstraintRow {
            constraint: generated.constraint,
            constraint_name: constraint_name(sketch, generated.constraint),
            generated: generated.clone(),
            failed: None,
            status: SketchFailedConstraintStatus::LoweringFailure(generated.status.clone()),
        });
    }

    for failed in &exact.rows {
        let Some(generated) = lowering
            .rows
            .iter()
            .find(|row| row.residual_index == Some(failed.constraint_index))
        else {
            continue;
        };
        let status = if failed.status == FailedConstraintStatus::RankRedundant {
            SketchFailedConstraintStatus::RankRedundant
        } else {
            SketchFailedConstraintStatus::ExactFailure(failed.status.clone())
        };
        rows.push(SketchFailedConstraintRow {
            constraint: generated.constraint,
            constraint_name: constraint_name(sketch, generated.constraint),
            generated: generated.clone(),
            failed: Some(failed.clone()),
            status,
        });
    }

    rows.sort_by_key(|row| {
        (
            row.constraint.0,
            row.generated.residual_index.unwrap_or(usize::MAX),
            row.generated.name.clone(),
        )
    });
    rows
}

fn build_report(
    lowering: SketchLoweringReport,
    exact: FailedConstraintReport,
    rows: Vec<SketchFailedConstraintRow>,
) -> SketchFailedConstraintReport {
    let blocking_rows = rows
        .iter()
        .filter(|row| row.status.blocks_candidate_acceptance())
        .count();
    let lowering_failure_rows = rows
        .iter()
        .filter(|row| matches!(row.status, SketchFailedConstraintStatus::LoweringFailure(_)))
        .count();
    let exact_failure_rows = rows
        .iter()
        .filter(|row| matches!(row.status, SketchFailedConstraintStatus::ExactFailure(_)))
        .count();
    let rank_redundant_rows = rows
        .iter()
        .filter(|row| row.status == SketchFailedConstraintStatus::RankRedundant)
        .count();

    SketchFailedConstraintReport {
        lowering,
        exact,
        rows,
        blocking_rows,
        lowering_failure_rows,
        exact_failure_rows,
        rank_redundant_rows,
    }
}

fn constraint_name(sketch: &SketchSolveProblem, handle: SketchConstraintHandle) -> String {
    sketch
        .constraints()
        .get(handle.0 as usize)
        .filter(|constraint| constraint.handle == handle)
        .map(|constraint| constraint.name.clone())
        .unwrap_or_else(|| format!("stale sketch constraint {}", handle.0))
}
