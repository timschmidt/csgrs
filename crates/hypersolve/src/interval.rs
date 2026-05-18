//! Affine interval-box certification for solver candidates.
//!
//! This module is the first variable-box proof stage for `hypersolve`. For an
//! affine residual `c + a*x + b*y`, a box around the candidate maps to an exact
//! residual ball with radius `|a|*rx + |b|*ry`. For prepared quadratic rows,
//! the same proof stage uses exact Taylor enclosures: univariate
//! `|(2*a*x0 + b)|*r + |a|*r^2`, and multivariate
//! `sum_i |grad_i(x0)|*r_i + sum_ij |q_ij|*r_i*r_j`. The final sign proof is
//! delegated to `hyperlimit`'s certified ball filter, keeping this layer inside Yap's
//! proof-producing filter model rather than turning solver tolerances into
//! topology. The interval-box shape follows Moore's interval-analysis
//! tradition and the Taylor-model/Krawczyk validation lineage, while the
//! exact/candidate separation follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7.1-2 (1997).

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

/// Errors that make prepared quadratic interval certification invalid.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum QuadraticIntervalError {
    /// One supplied variable radius was structurally negative.
    NegativeVariableRadius {
        /// Symbol whose radius was invalid.
        symbol: SymbolId,
    },
    /// The candidate did not bind the variable used by a prepared quadratic row.
    UnboundCandidateSymbol {
        /// Symbol required by the prepared row.
        symbol: SymbolId,
    },
    /// A coefficient, derivative, or Taylor term sign was not certifiable, so
    /// an absolute value could not be formed without an approximate branch.
    UnknownMagnitudeSign {
        /// Source constraint index.
        constraint_index: usize,
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

/// Certify a candidate using prepared univariate quadratic interval enclosures.
///
/// This is the first nonlinear interval proof stage in `hypersolve`. It does
/// not run interval Newton or Krawczyk iteration; instead it encloses each
/// already-prepared quadratic residual over the supplied variable ball using
/// the exact Taylor form
///
/// `f(x0 + dx) = f(x0) + (2*a*x0 + b)*dx + a*dx^2`.
///
/// The resulting residual ball is then certified by the same `hyperlimit`
/// ball-sign filter as affine interval replay. Missing variable radii are
/// treated as zero-radius variables, so callers can use this as a strict point
/// replay with optional local proof radii.
pub fn certify_quadratic_interval_candidate(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
    variable_balls: &[VariableBall],
    policy: PredicatePolicy,
) -> Result<crate::CandidateCertificationReport, QuadraticIntervalError> {
    let radius_by_symbol =
        validate_variable_balls(variable_balls).map_err(|error| match error {
            AffineIntervalError::NegativeVariableRadius { symbol } => {
                QuadraticIntervalError::NegativeVariableRadius { symbol }
            }
            AffineIntervalError::UnknownCoefficientSign { .. } => {
                unreachable!("variable-ball validation cannot produce coefficient-sign errors")
            }
        })?;
    let mut residual_balls = Vec::new();
    let mut active_row = 0_usize;

    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active {
            continue;
        }
        if let Some(quadratic) = &prepared.univariate_quadratic_residuals()[constraint_index] {
            let variable_radius = radius_by_symbol
                .get(&quadratic.symbol())
                .cloned()
                .unwrap_or_else(Real::zero);
            let candidate = context.bindings().get(&quadratic.symbol()).ok_or(
                QuadraticIntervalError::UnboundCandidateSymbol {
                    symbol: quadratic.symbol(),
                },
            )?;
            let derivative = quadratic.quadratic().clone() * Real::from(2) * candidate.clone()
                + quadratic.linear().clone();
            let derivative_abs = abs_real(&derivative)
                .ok_or(QuadraticIntervalError::UnknownMagnitudeSign { constraint_index })?;
            let quadratic_abs = abs_real(quadratic.quadratic())
                .ok_or(QuadraticIntervalError::UnknownMagnitudeSign { constraint_index })?;
            let radius = derivative_abs * variable_radius.clone()
                + quadratic_abs * variable_radius.clone() * variable_radius;
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

/// Certify a candidate using prepared multivariate quadratic interval enclosures.
///
/// This extends [`certify_quadratic_interval_candidate`] from single-symbol
/// rows to retained cross terms. For a prepared quadratic residual
///
/// `f(x) = c + sum_i l_i*x_i + sum_ij q_ij*x_i*x_j`,
///
/// the exact enclosure around candidate `x0` and variable radii `r_i` is
///
/// `|f(x)-f(x0)| <= sum_i |grad_i(x0)|*r_i + sum_ij |q_ij|*r_i*r_j`.
///
/// The formula is a Taylor-model style bound over the retained polynomial
/// package; it does not certify uniqueness of a nonlinear root. That later
/// step belongs to interval Newton/Krawczyk proof operators.
pub fn certify_multivariate_quadratic_interval_candidate(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
    variable_balls: &[VariableBall],
    policy: PredicatePolicy,
) -> Result<crate::CandidateCertificationReport, QuadraticIntervalError> {
    let radius_by_symbol =
        validate_variable_balls(variable_balls).map_err(|error| match error {
            AffineIntervalError::NegativeVariableRadius { symbol } => {
                QuadraticIntervalError::NegativeVariableRadius { symbol }
            }
            AffineIntervalError::UnknownCoefficientSign { .. } => {
                unreachable!("variable-ball validation cannot produce coefficient-sign errors")
            }
        })?;
    let mut residual_balls = Vec::new();
    let mut active_row = 0_usize;

    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active {
            continue;
        }
        if let Some(quadratic) = &prepared.quadratic_residuals()[constraint_index] {
            let mut gradients: HashMap<SymbolId, Real> = HashMap::new();
            for term in quadratic.linear_terms() {
                gradients.insert(term.symbol, term.coefficient.clone());
            }
            for term in quadratic.quadratic_terms() {
                let first_value = context
                    .bindings()
                    .get(&term.first)
                    .ok_or(QuadraticIntervalError::UnboundCandidateSymbol { symbol: term.first })?;
                let second_value = context.bindings().get(&term.second).ok_or(
                    QuadraticIntervalError::UnboundCandidateSymbol {
                        symbol: term.second,
                    },
                )?;
                if term.first == term.second {
                    let contribution =
                        term.coefficient.clone() * Real::from(2) * first_value.clone();
                    let entry = gradients.entry(term.first).or_insert_with(Real::zero);
                    *entry = entry.clone() + contribution;
                } else {
                    let first_entry = gradients.entry(term.first).or_insert_with(Real::zero);
                    *first_entry =
                        first_entry.clone() + term.coefficient.clone() * second_value.clone();
                    let second_entry = gradients.entry(term.second).or_insert_with(Real::zero);
                    *second_entry =
                        second_entry.clone() + term.coefficient.clone() * first_value.clone();
                }
            }

            let mut radius = Real::zero();
            for (symbol, gradient) in gradients {
                let variable_radius = radius_by_symbol
                    .get(&symbol)
                    .cloned()
                    .unwrap_or_else(Real::zero);
                let gradient_abs = abs_real(&gradient)
                    .ok_or(QuadraticIntervalError::UnknownMagnitudeSign { constraint_index })?;
                radius = radius + gradient_abs * variable_radius;
            }
            for term in quadratic.quadratic_terms() {
                let first_radius = radius_by_symbol
                    .get(&term.first)
                    .cloned()
                    .unwrap_or_else(Real::zero);
                let second_radius = radius_by_symbol
                    .get(&term.second)
                    .cloned()
                    .unwrap_or_else(Real::zero);
                let coefficient_abs = abs_real(&term.coefficient)
                    .ok_or(QuadraticIntervalError::UnknownMagnitudeSign { constraint_index })?;
                radius = radius + coefficient_abs * first_radius * second_radius;
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
