//! Prepared nonlinear solver blocks.
//!
//! `PreparedProblem` records row facts close to the model. `PreparedSolverBlock`
//! is the next scheduling layer: it partitions active rows into the categories
//! a nonlinear backend needs before it decides whether to run substitution,
//! affine elimination, polynomial handling, or an external numeric proposal
//! engine. This follows Yap's guidance that exact geometry needs packages above
//! BigNumber for expressions and geometric objects, and mirrors SolveSpace's
//! explicit substitution/soluble-equation/Jacobian passes while keeping
//! primitive numeric iteration outside the proof boundary.

use crate::prepared::{PreparedConstraintFacts, PreparedProblem};
use crate::symbolic::ExprDegree;

/// Solver scheduling class for one source constraint row.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SolverBlockRowKind {
    /// Source row is disabled and not part of the active solve.
    Inactive,
    /// Active constant row proved exactly zero before iteration.
    ConstantCertifiedZero,
    /// Active constant row proved nonzero before iteration.
    ConstantCertifiedContradiction,
    /// Active constant row whose sign cannot be certified structurally.
    ConstantUnknown,
    /// Active affine row with a prepared coefficient block.
    PreparedAffine,
    /// Active affine row that was recognized but not extracted.
    AffineUnprepared,
    /// Active polynomial row of degree at least two.
    Polynomial,
    /// Active non-polynomial row.
    NonPolynomial,
}

impl SolverBlockRowKind {
    /// Returns whether this row can be handled without nonlinear iteration.
    pub const fn is_direct(self) -> bool {
        matches!(
            self,
            Self::Inactive
                | Self::ConstantCertifiedZero
                | Self::ConstantCertifiedContradiction
                | Self::ConstantUnknown
                | Self::PreparedAffine
                | Self::AffineUnprepared
        )
    }

    /// Returns whether this row needs a nonlinear proposal backend.
    pub const fn needs_nonlinear_proposal(self) -> bool {
        matches!(self, Self::Polynomial | Self::NonPolynomial)
    }
}

/// Prepared scheduling metadata for one row.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SolverBlockRow {
    /// Source constraint index.
    pub constraint_index: usize,
    /// Row name copied for diagnostics.
    pub name: String,
    /// Solver scheduling kind.
    pub kind: SolverBlockRowKind,
    /// Number of structurally dependent variable columns.
    pub dependent_column_count: usize,
}

/// Problem-wide solver-block summary.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PreparedSolverBlockFacts {
    /// Total source constraints.
    pub row_count: usize,
    /// Active source constraints.
    pub active_row_count: usize,
    /// Disabled source constraints.
    pub inactive_row_count: usize,
    /// Active rows solved or rejected by constant facts.
    pub constant_row_count: usize,
    /// Active constant rows that are exact contradictions.
    pub constant_contradiction_count: usize,
    /// Active rows with prepared affine coefficient blocks.
    pub prepared_affine_row_count: usize,
    /// Active polynomial rows of degree at least two.
    pub polynomial_nonlinear_row_count: usize,
    /// Active polynomial rows with prepared univariate quadratic coefficient blocks.
    pub prepared_univariate_quadratic_row_count: usize,
    /// Active polynomial rows with prepared degree-at-most-two coefficient blocks.
    pub prepared_quadratic_row_count: usize,
    /// Active non-polynomial rows.
    pub non_polynomial_row_count: usize,
    /// Number of rows that require a nonlinear proposal engine.
    pub nonlinear_proposal_row_count: usize,
    /// Total structural Jacobian nonzeros across active rows.
    pub structural_jacobian_nonzeros: usize,
}

impl PreparedSolverBlockFacts {
    /// Returns whether the active problem is purely affine after preparation.
    pub fn all_active_rows_affine_or_constant(&self) -> bool {
        self.active_row_count > 0
            && self.nonlinear_proposal_row_count == 0
            && self.non_polynomial_row_count == 0
    }

    /// Returns whether exact structural facts already prove infeasibility.
    pub fn has_exact_constant_contradiction(&self) -> bool {
        self.constant_contradiction_count > 0
    }
}

/// Solver scheduling layer built from a prepared problem.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PreparedSolverBlock {
    rows: Vec<SolverBlockRow>,
    facts: PreparedSolverBlockFacts,
}

impl PreparedSolverBlock {
    /// Build solver-block scheduling metadata from a prepared problem.
    pub fn new(prepared: &PreparedProblem<'_>) -> Self {
        let mut rows = Vec::with_capacity(prepared.constraints().len());
        let mut inactive_row_count = 0;
        let mut constant_row_count = 0;
        let mut constant_contradiction_count = 0;
        let mut prepared_affine_row_count = 0;
        let mut polynomial_nonlinear_row_count = 0;
        let mut prepared_univariate_quadratic_row_count = 0;
        let mut prepared_quadratic_row_count = 0;
        let mut non_polynomial_row_count = 0;
        let mut nonlinear_proposal_row_count = 0;
        let mut structural_jacobian_nonzeros = 0;

        for (constraint_index, facts) in prepared.constraints().iter().enumerate() {
            let kind = classify_row(
                facts,
                prepared.affine_residuals()[constraint_index].is_some(),
            );
            if !facts.active {
                inactive_row_count += 1;
            } else {
                structural_jacobian_nonzeros += facts.dependent_columns.len();
                match kind {
                    SolverBlockRowKind::ConstantCertifiedZero
                    | SolverBlockRowKind::ConstantCertifiedContradiction
                    | SolverBlockRowKind::ConstantUnknown => constant_row_count += 1,
                    SolverBlockRowKind::PreparedAffine => prepared_affine_row_count += 1,
                    SolverBlockRowKind::Polynomial => {
                        polynomial_nonlinear_row_count += 1;
                        if prepared.univariate_quadratic_residuals()[constraint_index].is_some() {
                            prepared_univariate_quadratic_row_count += 1;
                        }
                        if prepared.quadratic_residuals()[constraint_index].is_some() {
                            prepared_quadratic_row_count += 1;
                        }
                    }
                    SolverBlockRowKind::NonPolynomial => non_polynomial_row_count += 1,
                    SolverBlockRowKind::Inactive | SolverBlockRowKind::AffineUnprepared => {}
                }
                if kind == SolverBlockRowKind::ConstantCertifiedContradiction {
                    constant_contradiction_count += 1;
                }
                if kind.needs_nonlinear_proposal() {
                    nonlinear_proposal_row_count += 1;
                }
            }
            rows.push(SolverBlockRow {
                constraint_index,
                name: facts.name.clone(),
                kind,
                dependent_column_count: facts.dependent_columns.len(),
            });
        }

        let facts = PreparedSolverBlockFacts {
            row_count: rows.len(),
            active_row_count: prepared.facts().active_constraint_count,
            inactive_row_count,
            constant_row_count,
            constant_contradiction_count,
            prepared_affine_row_count,
            polynomial_nonlinear_row_count,
            prepared_univariate_quadratic_row_count,
            prepared_quadratic_row_count,
            non_polynomial_row_count,
            nonlinear_proposal_row_count,
            structural_jacobian_nonzeros,
        };

        Self { rows, facts }
    }

    /// Return per-row scheduling metadata.
    pub fn rows(&self) -> &[SolverBlockRow] {
        &self.rows
    }

    /// Return problem-wide scheduling facts.
    pub const fn facts(&self) -> &PreparedSolverBlockFacts {
        &self.facts
    }
}

fn classify_row(facts: &PreparedConstraintFacts, has_prepared_affine: bool) -> SolverBlockRowKind {
    if !facts.active {
        return SolverBlockRowKind::Inactive;
    }
    if facts.is_constant_row() {
        if facts.is_known_zero_constant_row() {
            return SolverBlockRowKind::ConstantCertifiedZero;
        }
        if facts.is_known_nonzero_constant_row() {
            return SolverBlockRowKind::ConstantCertifiedContradiction;
        }
        return SolverBlockRowKind::ConstantUnknown;
    }
    if facts.is_affine_row() {
        return if has_prepared_affine {
            SolverBlockRowKind::PreparedAffine
        } else {
            SolverBlockRowKind::AffineUnprepared
        };
    }
    match facts.residual.degree {
        ExprDegree::Polynomial(_) => SolverBlockRowKind::Polynomial,
        ExprDegree::NonPolynomial => SolverBlockRowKind::NonPolynomial,
        ExprDegree::Constant => SolverBlockRowKind::ConstantUnknown,
    }
}
