//! Direct exact reductions for simple solver rows.
//!
//! SolveSpace performs substitution and "soluble alone" passes before Newton
//! iteration. This module adds the exact-stack version for prepared affine
//! equality rows with one active variable. The result is a candidate assignment
//! produced by exact `hyperreal::Real` arithmetic; callers still replay and
//! certify the full problem before trusting it. This keeps the optimization in
//! Yap's expression/object layer instead of making a lossy nonlinear backend
//! responsible for obvious exact algebra.

use hyperreal::{Real, RealSign};

use crate::eval::EvaluationContext;
use crate::model::ConstraintKind;
use crate::prepared::PreparedProblem;
use crate::symbolic::SymbolId;

/// Exact direct solution for one affine equality row.
#[derive(Clone, Debug, PartialEq)]
pub struct DirectAffineSolution {
    /// Source constraint index that produced the solution.
    pub constraint_index: usize,
    /// Solver symbol solved by this row.
    pub symbol: SymbolId,
    /// Exact candidate value.
    pub value: Real,
}

/// Exact equality substitution `left = right + offset`.
///
/// This is the first union-find-friendly payload for SolveSpace-style
/// substitution passes. It deliberately records the source row and exact offset
/// instead of mutating the problem immediately; callers can choose whether to
/// use it for simplification, diagnostics, or candidate initialization before
/// replaying the full problem.
#[derive(Clone, Debug, PartialEq)]
pub struct EqualitySubstitution {
    /// Source constraint index.
    pub constraint_index: usize,
    /// Symbol being substituted.
    pub left: SymbolId,
    /// Reference symbol.
    pub right: SymbolId,
    /// Exact offset such that `left = right + offset`.
    pub offset: Real,
}

/// Failure modes for direct affine reduction.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DirectSolveError {
    /// The row coefficient was structurally unknown, so selecting the variable
    /// would require an unproved branch.
    UnknownCoefficientSign {
        /// Source constraint index.
        constraint_index: usize,
        /// Variable column.
        variable_column: usize,
    },
    /// The exact division needed to isolate a variable failed.
    UnsupportedDivision {
        /// Source constraint index.
        constraint_index: usize,
    },
}

/// Solve every active one-variable affine equality row exactly.
///
/// This is intentionally conservative. Rows with multiple structurally
/// nonzero coefficients, non-equality kinds, unknown coefficient signs, or
/// unsupported exact division are not silently approximated.
pub fn solve_direct_affine_equalities(
    prepared: &PreparedProblem<'_>,
) -> Result<Vec<DirectAffineSolution>, DirectSolveError> {
    let mut solutions = Vec::new();
    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active || constraint.kind != ConstraintKind::Equality {
            continue;
        }
        let Some(affine) = &prepared.affine_residuals()[constraint_index] else {
            continue;
        };
        let mut nonzero = None;
        for (column, coefficient) in affine.coefficients().iter().enumerate() {
            match coefficient.structural_facts().sign {
                Some(RealSign::Zero) => {}
                Some(RealSign::Negative | RealSign::Positive) => {
                    if nonzero.replace((column, coefficient)).is_some() {
                        nonzero = None;
                        break;
                    }
                }
                None => {
                    return Err(DirectSolveError::UnknownCoefficientSign {
                        constraint_index,
                        variable_column: column,
                    });
                }
            }
        }
        let Some((column, coefficient)) = nonzero else {
            continue;
        };
        let value = (-affine.constant().clone() / coefficient.clone())
            .map_err(|_| DirectSolveError::UnsupportedDivision { constraint_index })?;
        solutions.push(DirectAffineSolution {
            constraint_index,
            symbol: prepared.problem().variables[column].symbol,
            value,
        });
    }
    Ok(solutions)
}

/// Find exact two-variable equality substitutions from prepared affine rows.
///
/// Rows of the form `x - y + c = 0` become `x = y - c`. The implementation is
/// intentionally narrow: it accepts only structurally signed unit coefficients
/// and exact equality rows, so this pass cannot silently turn approximate
/// algebra into solver topology.
pub fn find_equality_substitutions(
    prepared: &PreparedProblem<'_>,
) -> Result<Vec<EqualitySubstitution>, DirectSolveError> {
    let mut substitutions = Vec::new();
    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active || constraint.kind != ConstraintKind::Equality {
            continue;
        }
        let Some(affine) = &prepared.affine_residuals()[constraint_index] else {
            continue;
        };
        let mut positive = None;
        let mut negative = None;
        let mut unsupported = false;
        for (column, coefficient) in affine.coefficients().iter().enumerate() {
            match coefficient.structural_facts().sign {
                Some(RealSign::Zero) => {}
                Some(RealSign::Positive) if *coefficient == Real::one() => {
                    unsupported |= positive.replace(column).is_some();
                }
                Some(RealSign::Negative) if *coefficient == -Real::one() => {
                    unsupported |= negative.replace(column).is_some();
                }
                Some(RealSign::Positive | RealSign::Negative) => {
                    unsupported = true;
                }
                None => {
                    return Err(DirectSolveError::UnknownCoefficientSign {
                        constraint_index,
                        variable_column: column,
                    });
                }
            }
        }
        if unsupported {
            continue;
        }
        let (Some(left_column), Some(right_column)) = (positive, negative) else {
            continue;
        };
        substitutions.push(EqualitySubstitution {
            constraint_index,
            left: prepared.problem().variables[left_column].symbol,
            right: prepared.problem().variables[right_column].symbol,
            offset: -affine.constant().clone(),
        });
    }
    Ok(substitutions)
}

/// Apply exact equality substitutions to a candidate context.
///
/// This is a candidate-update helper, not a proof. It is useful after
/// `find_equality_substitutions` has identified `x = y + c` rows and the
/// caller wants a candidate that satisfies those rows before exact replay. The
/// full residual certification step remains mandatory, preserving Yap's
/// distinction between construction/proposal and certified decision.
pub fn apply_equality_substitutions(
    context: &mut EvaluationContext,
    substitutions: &[EqualitySubstitution],
) -> Result<usize, DirectSolveError> {
    let mut applied = 0;
    for substitution in substitutions {
        let Some(right_value) = context.bindings().get(&substitution.right).cloned() else {
            continue;
        };
        context.bind(substitution.left, right_value + substitution.offset.clone());
        applied += 1;
    }
    Ok(applied)
}
