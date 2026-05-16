//! Prepared solver metadata for repeated residual and Jacobian work.
//!
//! The prepared types in this module cache structural facts about constraints
//! and residual expressions without evaluating variables or exporting to
//! primitive floats. They are advisory scheduling data: a dense f64 linear
//! adapter may consume the sparsity map, while a future exact/hyperlattice
//! backend can consume the same facts to choose symbolic, affine, sparse, or
//! non-polynomial rows. The design follows Yap's exact-geometric-computation
//! separation between object structure and arithmetic-package selection; see
//! Yap, "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997).

use crate::eval::{EvalError, EvaluationContext, ResidualEvaluation, evaluate_residuals};
use crate::model::{ConstraintKind, Problem};
use crate::symbolic::{ExprDegree, ExprFacts, SymbolId};

/// Cached structural facts for one active or inactive constraint.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PreparedConstraintFacts {
    /// Constraint name copied from the source problem for diagnostics.
    pub name: String,
    /// Constraint kind.
    pub kind: ConstraintKind,
    /// Whether the source constraint is enabled.
    pub active: bool,
    /// Conservative expression facts for the residual.
    pub residual: ExprFacts,
    /// Variable columns whose symbolic IDs occur in the residual expression.
    pub dependent_columns: Vec<usize>,
}

impl PreparedConstraintFacts {
    /// Returns whether the residual is structurally affine in solve variables.
    pub fn is_affine_row(&self) -> bool {
        self.residual.degree == ExprDegree::Polynomial(1)
    }

    /// Returns whether the residual is structurally constant.
    pub fn is_constant_row(&self) -> bool {
        self.residual.degree == ExprDegree::Constant
    }

    /// Returns whether the residual can be handled as a polynomial row.
    pub fn is_polynomial_row(&self) -> bool {
        self.residual.degree.is_polynomial()
    }

    /// Returns whether a variable column is structurally present in this row.
    pub fn depends_on_column(&self, column: usize) -> bool {
        self.dependent_columns.binary_search(&column).is_ok()
    }
}

/// Prepared problem-wide facts for solver strategy selection.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PreparedProblemFacts {
    /// Number of variables in the source problem.
    pub variable_count: usize,
    /// Number of constraints in the source problem.
    pub constraint_count: usize,
    /// Number of active constraints.
    pub active_constraint_count: usize,
    /// Number of active residual rows that are structurally affine.
    pub affine_active_rows: usize,
    /// Number of active residual rows that are structurally polynomial.
    pub polynomial_active_rows: usize,
    /// Number of active residual rows that are structurally non-polynomial.
    pub non_polynomial_active_rows: usize,
    /// Number of structurally nonzero Jacobian entries implied by dependency
    /// sets, before runtime inequality activation.
    pub structural_jacobian_nonzeros: usize,
}

impl PreparedProblemFacts {
    /// Returns whether every active row is structurally affine.
    pub fn all_active_rows_affine(&self) -> bool {
        self.active_constraint_count > 0 && self.affine_active_rows == self.active_constraint_count
    }

    /// Returns whether every active row is structurally polynomial.
    pub fn all_active_rows_polynomial(&self) -> bool {
        self.active_constraint_count > 0
            && self.polynomial_active_rows == self.active_constraint_count
    }
}

/// Borrowed prepared view of a [`Problem`].
#[derive(Clone, Debug)]
pub struct PreparedProblem<'a> {
    problem: &'a Problem,
    constraints: Vec<PreparedConstraintFacts>,
    facts: PreparedProblemFacts,
    jacobian_sparsity: Vec<Vec<bool>>,
}

impl<'a> PreparedProblem<'a> {
    /// Prepare solver metadata from a borrowed problem.
    ///
    /// Preparation walks each residual expression once, preserving dependency
    /// and degree facts near the model layer rather than rediscovering them
    /// inside dense linear-solver loops. This is a cache-populating API, not an
    /// evaluation API: variable values and inequality activation are still
    /// handled by residual evaluation.
    pub fn new(problem: &'a Problem) -> Self {
        let mut constraints = Vec::with_capacity(problem.constraints.len());
        let mut active_constraint_count = 0_usize;
        let mut affine_active_rows = 0_usize;
        let mut polynomial_active_rows = 0_usize;
        let mut non_polynomial_active_rows = 0_usize;
        let mut structural_jacobian_nonzeros = 0_usize;
        let mut jacobian_sparsity = Vec::with_capacity(problem.constraints.len());

        for constraint in &problem.constraints {
            let residual = constraint.residual.structural_facts();
            let dependent_columns = dependent_columns(problem, &residual);
            let row_sparsity = row_sparsity(problem.variables.len(), &dependent_columns);

            if constraint.active {
                active_constraint_count += 1;
                structural_jacobian_nonzeros += dependent_columns.len();
                match residual.degree {
                    ExprDegree::Constant => {
                        polynomial_active_rows += 1;
                    }
                    ExprDegree::Polynomial(1) => {
                        affine_active_rows += 1;
                        polynomial_active_rows += 1;
                    }
                    ExprDegree::Polynomial(_) => {
                        polynomial_active_rows += 1;
                    }
                    ExprDegree::NonPolynomial => {
                        non_polynomial_active_rows += 1;
                    }
                }
            }

            constraints.push(PreparedConstraintFacts {
                name: constraint.name.clone(),
                kind: constraint.kind,
                active: constraint.active,
                residual,
                dependent_columns,
            });
            jacobian_sparsity.push(row_sparsity);
        }

        let facts = PreparedProblemFacts {
            variable_count: problem.variables.len(),
            constraint_count: problem.constraints.len(),
            active_constraint_count,
            affine_active_rows,
            polynomial_active_rows,
            non_polynomial_active_rows,
            structural_jacobian_nonzeros,
        };

        Self {
            problem,
            constraints,
            facts,
            jacobian_sparsity,
        }
    }

    /// Returns the borrowed source problem.
    pub const fn problem(&self) -> &'a Problem {
        self.problem
    }

    /// Returns cached problem-wide facts.
    pub const fn facts(&self) -> &PreparedProblemFacts {
        &self.facts
    }

    /// Returns cached per-constraint facts.
    pub fn constraints(&self) -> &[PreparedConstraintFacts] {
        &self.constraints
    }

    /// Returns the structural Jacobian sparsity matrix.
    ///
    /// Rows correspond to source constraints, including inactive constraints,
    /// so diagnostics can preserve source indexing. Inactive rows remain useful
    /// for UI/reporting but should be ignored by numerical adapters.
    pub fn jacobian_sparsity(&self) -> &[Vec<bool>] {
        &self.jacobian_sparsity
    }

    /// Evaluate residuals against a context using the source problem.
    ///
    /// The prepared metadata is intentionally not required for correctness;
    /// stale or missing preparation should be a performance miss rather than a
    /// semantic change. Callers that cache a `PreparedProblem` can use this
    /// helper to keep evaluation and metadata ownership adjacent.
    pub fn evaluate_residuals(
        &self,
        context: &EvaluationContext,
    ) -> Result<Vec<ResidualEvaluation>, EvalError> {
        evaluate_residuals(self.problem, context)
    }
}

fn dependent_columns(problem: &Problem, facts: &ExprFacts) -> Vec<usize> {
    let mut columns = Vec::new();
    for (column, variable) in problem.variables.iter().enumerate() {
        if facts.dependencies.contains(&variable.symbol) {
            columns.push(column);
        }
    }
    columns
}

fn row_sparsity(variable_count: usize, dependent_columns: &[usize]) -> Vec<bool> {
    let mut row = vec![false; variable_count];
    for &column in dependent_columns {
        row[column] = true;
    }
    row
}

/// Returns whether an expression dependency set contains a symbol.
///
/// This small helper keeps future sparse backends from depending directly on
/// the concrete set implementation used by [`ExprFacts`].
pub fn facts_depend_on_symbol(facts: &ExprFacts, symbol: SymbolId) -> bool {
    facts.dependencies.contains(&symbol)
}
