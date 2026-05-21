//! Exact branch enumeration reports.
//!
//! Direct algebraic construction is useful for sketches with symmetric or
//! ambiguous solutions, but it is not by itself a proof of a solver decision.
//! This module keeps Yap's construction/proof split explicit: every direct
//! branch generated from a retained univariate quadratic row is replayed
//! through exact residual certification before it can be treated as certified.
//! See Yap, "Towards Exact Geometric Computation," *Computational Geometry*
//! 7.1-2 (1997).

use hyperreal::{Real, RealSign};

use crate::certification::{
    CandidateCertificationConfig, CandidateCertificationReport, certify_candidate_with_config,
};
use crate::eval::EvaluationContext;
use crate::model::ConstraintKind;
use crate::prepared::PreparedProblem;
use crate::symbolic::SymbolId;

/// Branch-level status for one direct univariate quadratic row or root.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactBranchStatus {
    /// The source row does not currently have a retained direct quadratic form.
    UnsupportedRow,
    /// The leading coefficient sign could not be certified structurally.
    UnsupportedCoefficientSign,
    /// The exact square-root object required by the quadratic formula failed.
    UnsupportedSquareRoot,
    /// Exact division failed while constructing a root candidate.
    UnsupportedDivision,
    /// The source row was a retained quadratic with certified negative
    /// discriminant.
    NoRealRoots,
    /// The root candidate satisfied every active row under exact replay.
    ReplayCertified,
    /// Exact replay ran, but at least one active row was not certified
    /// satisfied.
    ReplayRejected,
}

/// One explicit solution branch, rejection, or unsupported row.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactSolutionBranch {
    /// Source constraint index that produced this branch report.
    pub constraint_index: usize,
    /// Solver symbol solved by the source row, when the row exposed one.
    pub symbol: Option<SymbolId>,
    /// Root ordinal within the source row, if a root candidate exists.
    pub root_index: Option<usize>,
    /// Exact root value, if a root candidate exists.
    pub root: Option<Real>,
    /// Full exact replay report for the candidate, if a root exists.
    pub certification: Option<CandidateCertificationReport>,
    /// Branch-level status.
    pub status: ExactBranchStatus,
}

/// Summary for direct univariate quadratic branch enumeration.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBranchEnumerationReport {
    /// Per-row and per-root branch reports in source-row order.
    pub branches: Vec<ExactSolutionBranch>,
    /// Number of root candidates that replayed as exact solutions.
    pub certified_branches: usize,
    /// Number of root candidates rejected by exact replay.
    pub rejected_branches: usize,
    /// Number of active equality rows that cannot enter this direct brancher.
    pub unsupported_rows: usize,
    /// Number of retained quadratic rows with no real roots.
    pub no_real_root_rows: usize,
}

impl ExactBranchEnumerationReport {
    /// Returns true when at least one enumerated root candidate replayed exactly.
    pub fn has_certified_branch(&self) -> bool {
        self.certified_branches > 0
    }
}

/// Enumerate direct univariate quadratic branches with the default replay
/// policy.
///
/// Each candidate is produced by exact quadratic formula construction and then
/// replayed through [`crate::certify_candidate`]. The root construction follows
/// the same retained-row discipline as
/// [`crate::solve_direct_univariate_quadratic_equalities`], while the replay
/// boundary follows Yap's Exact Geometric Computation model rather than a
/// primitive floating acceptance threshold.
pub fn enumerate_direct_univariate_quadratic_branches(
    prepared: &PreparedProblem<'_>,
    base_context: &EvaluationContext,
) -> ExactBranchEnumerationReport {
    enumerate_direct_univariate_quadratic_branches_with_config(
        prepared,
        base_context,
        CandidateCertificationConfig::default(),
    )
}

/// Enumerate direct univariate quadratic branches with an explicit replay
/// policy.
///
/// Unsupported rows are retained as report rows instead of being silently
/// skipped. This makes downstream sketch branch selection auditable: a caller
/// can distinguish "no real root", "not handled by this brancher", and
/// "candidate rejected by exact residual replay".
pub fn enumerate_direct_univariate_quadratic_branches_with_config(
    prepared: &PreparedProblem<'_>,
    base_context: &EvaluationContext,
    config: CandidateCertificationConfig,
) -> ExactBranchEnumerationReport {
    let mut report = ExactBranchEnumerationReport {
        branches: Vec::new(),
        certified_branches: 0,
        rejected_branches: 0,
        unsupported_rows: 0,
        no_real_root_rows: 0,
    };

    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active || constraint.kind != ConstraintKind::Equality {
            continue;
        }
        let Some(quadratic) = &prepared.univariate_quadratic_residuals()[constraint_index] else {
            report.unsupported_rows += 1;
            report.branches.push(ExactSolutionBranch {
                constraint_index,
                symbol: None,
                root_index: None,
                root: None,
                certification: None,
                status: ExactBranchStatus::UnsupportedRow,
            });
            continue;
        };

        let roots = match direct_quadratic_roots(
            quadratic.quadratic(),
            quadratic.linear(),
            quadratic.constant(),
        ) {
            Ok(roots) => roots,
            Err(status) => {
                report.unsupported_rows += 1;
                report.branches.push(ExactSolutionBranch {
                    constraint_index,
                    symbol: Some(quadratic.symbol()),
                    root_index: None,
                    root: None,
                    certification: None,
                    status,
                });
                continue;
            }
        };

        if roots.is_empty() {
            report.no_real_root_rows += 1;
            report.branches.push(ExactSolutionBranch {
                constraint_index,
                symbol: Some(quadratic.symbol()),
                root_index: None,
                root: None,
                certification: None,
                status: ExactBranchStatus::NoRealRoots,
            });
            continue;
        }

        for (root_index, root) in roots.into_iter().enumerate() {
            let mut candidate = base_context.clone();
            candidate.bind(quadratic.symbol(), root.clone());
            let certification = certify_candidate_with_config(prepared, &candidate, config);
            let status = if certification.all_satisfied() {
                report.certified_branches += 1;
                ExactBranchStatus::ReplayCertified
            } else {
                report.rejected_branches += 1;
                ExactBranchStatus::ReplayRejected
            };
            report.branches.push(ExactSolutionBranch {
                constraint_index,
                symbol: Some(quadratic.symbol()),
                root_index: Some(root_index),
                root: Some(root),
                certification: Some(certification),
                status,
            });
        }
    }

    report
}

fn direct_quadratic_roots(
    quadratic: &Real,
    linear: &Real,
    constant: &Real,
) -> Result<Vec<Real>, ExactBranchStatus> {
    match quadratic.structural_facts().sign {
        Some(RealSign::Zero) => return Err(ExactBranchStatus::UnsupportedRow),
        Some(RealSign::Negative | RealSign::Positive) => {}
        None => return Err(ExactBranchStatus::UnsupportedCoefficientSign),
    }

    let four = Real::from(4);
    let two = Real::from(2);
    let discriminant =
        linear.clone() * linear.clone() - four * quadratic.clone() * constant.clone();
    match discriminant.structural_facts().sign {
        Some(RealSign::Negative) => Ok(Vec::new()),
        Some(RealSign::Zero) => {
            let denominator = two * quadratic.clone();
            Ok(vec![
                (-linear.clone() / denominator)
                    .map_err(|_| ExactBranchStatus::UnsupportedDivision)?,
            ])
        }
        Some(RealSign::Positive) | None => {
            let sqrt = discriminant
                .sqrt()
                .map_err(|_| ExactBranchStatus::UnsupportedSquareRoot)?;
            let denominator = two * quadratic.clone();
            let first = (((-linear.clone()) + sqrt.clone()) / denominator.clone())
                .map_err(|_| ExactBranchStatus::UnsupportedDivision)?;
            let second = (((-linear.clone()) - sqrt) / denominator)
                .map_err(|_| ExactBranchStatus::UnsupportedDivision)?;
            Ok(vec![first, second])
        }
    }
}
