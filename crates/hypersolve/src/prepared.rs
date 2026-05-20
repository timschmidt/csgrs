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

use crate::affine::{PreparedAffineResidual, prepare_affine_residual};
use crate::eval::{EvalError, EvaluationContext, ResidualEvaluation, positive_part};
use crate::model::{ConstraintKind, Problem};
use crate::polynomial::{
    PreparedQuadraticResidual, PreparedUnivariateQuadraticResidual, prepare_quadratic_residual,
    prepare_univariate_quadratic_residual,
};
use crate::symbolic::{Expr, ExprDegree, ExprFacts, SymbolId};
use hyperreal::RealSign;

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
    /// Exact sign of a structurally constant residual, when the scalar layer
    /// can certify it from retained `Real` facts.
    ///
    /// `None` means either the row depends on solver variables or the constant
    /// sign is not structurally known. This is advisory solver metadata, not a
    /// feasibility proof for variable-dependent constraints. It follows Yap's
    /// object-fact discipline: keep cheap exact facts near the residual block
    /// so solver strategies can avoid unnecessary dense numerical work; see
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7.1-2 (1997).
    pub residual_constant_sign: Option<RealSign>,
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

    /// Returns whether this row is a structurally constant residual known to be zero.
    pub fn is_known_zero_constant_row(&self) -> bool {
        self.is_constant_row() && self.residual_constant_sign == Some(RealSign::Zero)
    }

    /// Returns whether this row is a structurally constant residual known to be nonzero.
    pub fn is_known_nonzero_constant_row(&self) -> bool {
        self.is_constant_row()
            && matches!(
                self.residual_constant_sign,
                Some(RealSign::Positive | RealSign::Negative)
            )
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
    /// Number of active residual rows with prepared affine coefficient blocks.
    pub prepared_affine_active_rows: usize,
    /// Number of active residual rows that are structurally polynomial.
    pub polynomial_active_rows: usize,
    /// Number of active polynomial rows with prepared univariate quadratic blocks.
    pub prepared_univariate_quadratic_active_rows: usize,
    /// Number of active polynomial rows with prepared degree-at-most-two blocks.
    pub prepared_quadratic_active_rows: usize,
    /// Number of active residual rows that are structurally non-polynomial.
    pub non_polynomial_active_rows: usize,
    /// Number of active constant residual rows whose exact value is known zero.
    pub known_zero_constant_active_rows: usize,
    /// Number of active constant residual rows whose exact value is known nonzero.
    pub known_nonzero_constant_active_rows: usize,
    /// Number of active constant residual rows whose exact sign is not known
    /// structurally.
    pub unknown_sign_constant_active_rows: usize,
    /// Number of structurally nonzero Jacobian entries implied by dependency
    /// sets, before runtime inequality activation.
    pub structural_jacobian_nonzeros: usize,
}

impl PreparedProblemFacts {
    /// Returns whether every active row is structurally affine.
    pub fn all_active_rows_affine(&self) -> bool {
        self.active_constraint_count > 0 && self.affine_active_rows == self.active_constraint_count
    }

    /// Returns whether every active row has a prepared affine coefficient block.
    pub fn all_active_rows_prepared_affine(&self) -> bool {
        self.active_constraint_count > 0
            && self.prepared_affine_active_rows == self.active_constraint_count
    }

    /// Returns whether every active row is structurally polynomial.
    pub fn all_active_rows_polynomial(&self) -> bool {
        self.active_constraint_count > 0
            && self.polynomial_active_rows == self.active_constraint_count
    }

    /// Returns whether any active row is a constant row known to be nonzero.
    ///
    /// Such a row is an exact structural contradiction for equality constraints
    /// and a useful early diagnostic for future exact solvers. Inequality
    /// activation policy still belongs to evaluation and solver layers.
    pub fn has_known_nonzero_constant_residual(&self) -> bool {
        self.known_nonzero_constant_active_rows > 0
    }
}

/// Borrowed prepared view of a [`Problem`].
#[derive(Clone, Debug)]
pub struct PreparedProblem<'a> {
    problem: &'a Problem,
    constraints: Vec<PreparedConstraintFacts>,
    affine_residuals: Vec<Option<PreparedAffineResidual>>,
    univariate_quadratic_residuals: Vec<Option<PreparedUnivariateQuadraticResidual>>,
    quadratic_residuals: Vec<Option<PreparedQuadraticResidual>>,
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
        let mut prepared_affine_active_rows = 0_usize;
        let mut polynomial_active_rows = 0_usize;
        let mut prepared_univariate_quadratic_active_rows = 0_usize;
        let mut prepared_quadratic_active_rows = 0_usize;
        let mut non_polynomial_active_rows = 0_usize;
        let mut known_zero_constant_active_rows = 0_usize;
        let mut known_nonzero_constant_active_rows = 0_usize;
        let mut unknown_sign_constant_active_rows = 0_usize;
        let mut structural_jacobian_nonzeros = 0_usize;
        let mut jacobian_sparsity = Vec::with_capacity(problem.constraints.len());
        let mut affine_residuals = Vec::with_capacity(problem.constraints.len());
        let mut univariate_quadratic_residuals = Vec::with_capacity(problem.constraints.len());
        let mut quadratic_residuals = Vec::with_capacity(problem.constraints.len());

        for constraint in &problem.constraints {
            let residual = constraint.residual.structural_facts();
            let residual_constant_sign = residual_constant_sign(&constraint.residual, &residual);
            let dependent_columns = dependent_columns(problem, &residual);
            let row_sparsity = row_sparsity(problem.variables.len(), &dependent_columns);
            let affine_residual = (residual.degree == ExprDegree::Polynomial(1)
                || residual.degree == ExprDegree::Constant)
                .then(|| prepare_affine_residual(&constraint.residual, problem))
                .flatten();
            let univariate_quadratic_residual = (residual.degree == ExprDegree::Polynomial(2))
                .then(|| prepare_univariate_quadratic_residual(&constraint.residual, problem))
                .flatten();
            let quadratic_residual = (residual.degree == ExprDegree::Polynomial(2))
                .then(|| prepare_quadratic_residual(&constraint.residual, problem))
                .flatten();

            if constraint.active {
                active_constraint_count += 1;
                structural_jacobian_nonzeros += dependent_columns.len();
                match residual.degree {
                    ExprDegree::Constant => {
                        polynomial_active_rows += 1;
                        if affine_residual.is_some() {
                            prepared_affine_active_rows += 1;
                        }
                        match residual_constant_sign {
                            Some(RealSign::Zero) => known_zero_constant_active_rows += 1,
                            Some(RealSign::Positive | RealSign::Negative) => {
                                known_nonzero_constant_active_rows += 1;
                            }
                            None => unknown_sign_constant_active_rows += 1,
                        }
                    }
                    ExprDegree::Polynomial(1) => {
                        affine_active_rows += 1;
                        polynomial_active_rows += 1;
                        if affine_residual.is_some() {
                            prepared_affine_active_rows += 1;
                        }
                    }
                    ExprDegree::Polynomial(_) => {
                        polynomial_active_rows += 1;
                        if univariate_quadratic_residual.is_some() {
                            prepared_univariate_quadratic_active_rows += 1;
                        }
                        if quadratic_residual.is_some() {
                            prepared_quadratic_active_rows += 1;
                        }
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
                residual_constant_sign,
                dependent_columns,
            });
            jacobian_sparsity.push(row_sparsity);
            affine_residuals.push(affine_residual);
            univariate_quadratic_residuals.push(univariate_quadratic_residual);
            quadratic_residuals.push(quadratic_residual);
        }

        let facts = PreparedProblemFacts {
            variable_count: problem.variables.len(),
            constraint_count: problem.constraints.len(),
            active_constraint_count,
            affine_active_rows,
            prepared_affine_active_rows,
            polynomial_active_rows,
            prepared_univariate_quadratic_active_rows,
            prepared_quadratic_active_rows,
            non_polynomial_active_rows,
            known_zero_constant_active_rows,
            known_nonzero_constant_active_rows,
            unknown_sign_constant_active_rows,
            structural_jacobian_nonzeros,
        };

        Self {
            problem,
            constraints,
            affine_residuals,
            univariate_quadratic_residuals,
            quadratic_residuals,
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

    /// Returns prepared affine residual rows in source-constraint order.
    ///
    /// `None` marks non-affine rows or affine shapes that could not be safely
    /// extracted. Missing affine preparation is only a performance miss: the
    /// original expression tree remains authoritative for evaluation.
    pub fn affine_residuals(&self) -> &[Option<PreparedAffineResidual>] {
        &self.affine_residuals
    }

    /// Returns prepared univariate quadratic residual blocks by source row.
    pub fn univariate_quadratic_residuals(&self) -> &[Option<PreparedUnivariateQuadraticResidual>] {
        &self.univariate_quadratic_residuals
    }

    /// Returns prepared degree-at-most-two residual blocks by source row.
    pub fn quadratic_residuals(&self) -> &[Option<PreparedQuadraticResidual>] {
        &self.quadratic_residuals
    }

    /// Evaluate one source residual through the best retained exact package.
    ///
    /// Prepared affine rows use fixed product-sum coefficient blocks; prepared
    /// quadratic rows use retained constant, linear, square, and cross terms;
    /// all other rows fall back to the original expression tree. This is the
    /// solver analogue of Yap's recommendation to choose an arithmetic package
    /// from object structure before expanding into scalar questions. See Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    /// (1997).
    pub fn evaluate_constraint_residual(
        &self,
        constraint_index: usize,
        context: &EvaluationContext,
    ) -> Result<hyperreal::Real, EvalError> {
        if let Some(affine) = &self.affine_residuals[constraint_index] {
            return Ok(affine.eval_real(&self.problem.variables, context.bindings())?);
        }
        if let Some(quadratic) = &self.quadratic_residuals[constraint_index] {
            return Ok(quadratic.eval_real(&self.problem.variables, context.bindings())?);
        }
        Ok(self.problem.constraints[constraint_index]
            .residual
            .eval_real(context.bindings())?)
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
        let mut residuals = Vec::new();
        for (constraint_index, constraint) in self.problem.constraints.iter().enumerate() {
            if !constraint.active {
                continue;
            }
            let value = self.evaluate_constraint_residual(constraint_index, context)?;
            let signed = match constraint.kind {
                ConstraintKind::Equality | ConstraintKind::Soft => value,
                ConstraintKind::LessOrEqual => positive_part(value),
                ConstraintKind::GreaterOrEqual => positive_part(-value),
            };
            let facts = signed.structural_facts();
            let weighted = signed.clone() * constraint.weight.clone();
            residuals.push(ResidualEvaluation {
                name: constraint.name.clone(),
                dense_solver_estimate: signed.to_f64_lossy(),
                weighted_dense_solver_estimate: weighted.to_f64_lossy(),
                sign: facts.sign,
                value: signed,
            });
        }
        Ok(residuals)
    }
}

fn residual_constant_sign(expression: &Expr, facts: &ExprFacts) -> Option<RealSign> {
    if facts.degree != ExprDegree::Constant || !facts.dependencies.is_empty() {
        return None;
    }
    match expression {
        Expr::Constant(value) => value.structural_facts().sign,
        _ => None,
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
