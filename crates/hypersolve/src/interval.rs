//! Affine interval-box certification for solver candidates.
//!
//! This module is the first variable-box proof stage for `hypersolve`. For an
//! affine residual `c + a*x + b*y`, a box around the candidate maps to an exact
//! residual ball with radius `|a|*rx + |b|*ry`. The final sign proof is
//! delegated to `hyperlimit`'s certified ball filter, keeping this layer inside
//! Yap's proof-producing filter model rather than turning solver tolerances
//! into topology. The interval-box shape follows Moore's interval-analysis
//! tradition, while the exact/candidate separation follows Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997).

use std::collections::HashMap;

use hyperlimit::{PredicatePolicy, compare_reals_with_policy};
use hyperreal::{Real, RealSign};

use crate::certification::{
    CandidateResidualBall, certify_candidate, certify_candidate_with_residual_balls,
};
use crate::eval::EvaluationContext;
use crate::prepared::PreparedProblem;
use crate::symbolic::SymbolId;

/// Exact radius around one solver variable.
#[derive(Clone, Debug, PartialEq)]
pub struct VariableBall {
    /// Solver symbol.
    pub symbol: SymbolId,
    /// Exact nonnegative radius.
    pub radius: Real,
}

/// Errors that make affine interval certification invalid.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AffineIntervalError {
    /// One supplied variable radius was structurally negative.
    NegativeVariableRadius {
        /// Symbol whose radius was invalid.
        symbol: SymbolId,
    },
    /// A coefficient sign was not certifiable, so `|coefficient|` could not be
    /// formed without an approximate branch.
    UnknownCoefficientSign {
        /// Source constraint index.
        constraint_index: usize,
        /// Variable column in source-problem order.
        variable_column: usize,
    },
}

/// Certify a candidate using affine variable-box residual enclosures.
///
/// Rows without prepared affine residuals fall back to exact point replay. For
/// prepared affine rows, this computes an exact residual ball and asks
/// `hyperlimit` to certify the whole ball sign. Missing variable radii are
/// treated as exact zero-radius variables.
pub fn certify_affine_interval_candidate(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
    variable_balls: &[VariableBall],
    policy: PredicatePolicy,
) -> Result<crate::CandidateCertificationReport, AffineIntervalError> {
    let radius_by_symbol = validate_variable_balls(variable_balls)?;
    let mut residual_balls = Vec::new();
    let mut active_row = 0_usize;

    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active {
            continue;
        }
        if let Some(affine) = &prepared.affine_residuals()[constraint_index] {
            let mut radius = Real::zero();
            for (column, coefficient) in affine.coefficients().iter().enumerate() {
                let variable = &prepared.problem().variables[column];
                let Some(variable_radius) = radius_by_symbol.get(&variable.symbol) else {
                    continue;
                };
                let coefficient_abs =
                    abs_real(coefficient).ok_or(AffineIntervalError::UnknownCoefficientSign {
                        constraint_index,
                        variable_column: column,
                    })?;
                radius = radius + coefficient_abs * variable_radius.clone();
            }
            residual_balls.push(CandidateResidualBall { active_row, radius });
        }
        active_row += 1;
    }

    if residual_balls.is_empty() {
        return Ok(certify_candidate(prepared, context));
    }
    Ok(certify_candidate_with_residual_balls(
        prepared,
        context,
        &residual_balls,
        policy,
    ))
}

fn validate_variable_balls(
    variable_balls: &[VariableBall],
) -> Result<HashMap<SymbolId, Real>, AffineIntervalError> {
    let mut radii = HashMap::new();
    for ball in variable_balls {
        if ball.radius.structural_facts().sign == Some(RealSign::Negative) {
            return Err(AffineIntervalError::NegativeVariableRadius {
                symbol: ball.symbol,
            });
        }
        radii.insert(ball.symbol, ball.radius.clone());
    }
    Ok(radii)
}

fn abs_real(value: &Real) -> Option<Real> {
    match compare_reals_with_policy(value, &Real::zero(), PredicatePolicy::default()).value()? {
        std::cmp::Ordering::Less => Some(-value.clone()),
        std::cmp::Ordering::Equal | std::cmp::Ordering::Greater => Some(value.clone()),
    }
}
