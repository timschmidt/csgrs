//! Interval-box certification for solver candidates.
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

use std::cmp::Ordering;
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

/// Status for an affine interval-Newton/Krawczyk box certificate.
///
/// This report is intentionally narrow: it only handles square active affine
/// equality systems. In that case the Jacobian is constant, so an exact Newton
/// step is the affine Krawczyk image with zero interval remainder. The
/// certificate proves that the unique affine root lies inside the supplied
/// variable box; it does not accept nonlinear rows or inequality activation by
/// approximation. This follows Krawczyk's interval-operator criterion
/// (R. Krawczyk, "Newton-Algorithmen zur Bestimmung von Nullstellen mit
/// Fehlerschranken", Computing 4, 1969) while keeping Yap's exact replay
/// boundary explicit.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AffineKrawczykStatus {
    /// The exact affine Newton/Krawczyk image lies inside every variable ball.
    CertifiedUniqueRoot,
    /// The active system is not square in all problem variables.
    ShapeMismatch {
        /// Number of variables in the source problem.
        variables: usize,
        /// Number of active equality rows considered.
        equality_rows: usize,
    },
    /// An active row was not an equality row.
    NonEqualityRow {
        /// Source constraint index.
        constraint_index: usize,
    },
    /// An active equality row did not have a prepared affine block.
    NonAffineRow {
        /// Source constraint index.
        constraint_index: usize,
    },
    /// The candidate context did not bind a required symbol.
    UnboundCandidateSymbol {
        /// Missing symbol.
        symbol: SymbolId,
    },
    /// No radius was supplied for a solved variable.
    MissingVariableRadius {
        /// Missing symbol.
        symbol: SymbolId,
    },
    /// One supplied variable radius was negative.
    NegativeVariableRadius {
        /// Symbol whose radius was invalid.
        symbol: SymbolId,
    },
    /// A radius sign could not be certified without leaving the exact filter.
    UnknownVariableRadiusSign {
        /// Symbol whose radius was undecided.
        symbol: SymbolId,
    },
    /// The affine matrix could not be inverted exactly with certified pivots.
    SingularOrUnsupportedPivot {
        /// Pivot column where elimination failed.
        pivot: usize,
    },
    /// A root coordinate could not be proved inside its variable ball.
    RootOutsideBox {
        /// Symbol whose Newton/Krawczyk displacement exceeded its radius.
        symbol: SymbolId,
    },
    /// The exact comparison between displacement and radius was undecided.
    UndecidedContainment {
        /// Symbol whose containment comparison was undecided.
        symbol: SymbolId,
    },
}

/// Exact displacement for one variable in an affine Krawczyk certificate.
#[derive(Clone, Debug, PartialEq)]
pub struct AffineKrawczykVariableStep {
    /// Solver symbol.
    pub symbol: SymbolId,
    /// Candidate center supplied by the caller.
    pub candidate: Real,
    /// Exact Newton/Krawczyk displacement from the candidate center.
    pub step: Real,
    /// Exact root coordinate `candidate + step`.
    pub certified_root: Real,
    /// Exact variable-ball radius used for containment.
    pub radius: Real,
}

/// Report for a square affine interval-Newton/Krawczyk proof attempt.
#[derive(Clone, Debug, PartialEq)]
pub struct AffineKrawczykReport {
    /// Final proof status.
    pub status: AffineKrawczykStatus,
    /// Number of variables in the source problem.
    pub variable_count: usize,
    /// Number of active equality rows considered.
    pub equality_rows: usize,
    /// Per-variable exact Newton/Krawczyk displacements.
    pub steps: Vec<AffineKrawczykVariableStep>,
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

/// Certify a square affine equality system with an exact Krawczyk box.
///
/// For affine rows `f(x) = A*x + c`, the interval Newton/Krawczyk operator has
/// no nonlinear interval remainder when `A` is exactly invertible. This helper
/// solves `A*step = -f(candidate)` over `Real`, then proves
/// `|step_i| <= radius_i` for every variable. A successful report therefore
/// proves that the unique affine root is inside the caller's box.
pub fn certify_affine_krawczyk_box(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
    variable_balls: &[VariableBall],
    policy: PredicatePolicy,
) -> AffineKrawczykReport {
    let variable_count = prepared.problem().variables.len();
    let mut matrix = Vec::new();
    let mut rhs = Vec::new();

    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active {
            continue;
        }
        if constraint.kind != crate::ConstraintKind::Equality {
            return affine_krawczyk_report(
                AffineKrawczykStatus::NonEqualityRow { constraint_index },
                variable_count,
                matrix.len(),
                Vec::new(),
            );
        }
        let Some(affine) = &prepared.affine_residuals()[constraint_index] else {
            return affine_krawczyk_report(
                AffineKrawczykStatus::NonAffineRow { constraint_index },
                variable_count,
                matrix.len(),
                Vec::new(),
            );
        };
        let residual =
            match affine.eval_real(prepared.problem().variables.as_slice(), context.bindings()) {
                Ok(value) => value,
                Err(error) => {
                    let symbol = match error {
                        crate::symbolic::ExprEvalError::UnboundSymbol(symbol) => symbol.id,
                        crate::symbolic::ExprEvalError::PreparedShapeMismatch { .. } => {
                            return affine_krawczyk_report(
                                AffineKrawczykStatus::NonAffineRow { constraint_index },
                                variable_count,
                                matrix.len(),
                                Vec::new(),
                            );
                        }
                        _ => {
                            return affine_krawczyk_report(
                                AffineKrawczykStatus::NonAffineRow { constraint_index },
                                variable_count,
                                matrix.len(),
                                Vec::new(),
                            );
                        }
                    };
                    return affine_krawczyk_report(
                        AffineKrawczykStatus::UnboundCandidateSymbol { symbol },
                        variable_count,
                        matrix.len(),
                        Vec::new(),
                    );
                }
            };
        matrix.push(affine.coefficients().to_vec());
        rhs.push(-residual);
    }

    if matrix.len() != variable_count {
        return affine_krawczyk_report(
            AffineKrawczykStatus::ShapeMismatch {
                variables: variable_count,
                equality_rows: matrix.len(),
            },
            variable_count,
            matrix.len(),
            Vec::new(),
        );
    }

    let radius_by_symbol = match validate_krawczyk_variable_balls(variable_balls, policy) {
        Ok(radii) => radii,
        Err(status) => {
            return affine_krawczyk_report(status, variable_count, matrix.len(), Vec::new());
        }
    };
    let step = match solve_exact_linear_system(matrix, rhs, policy) {
        Ok(step) => step,
        Err(status) => {
            return affine_krawczyk_report(status, variable_count, variable_count, Vec::new());
        }
    };

    let mut steps = Vec::with_capacity(variable_count);
    for (variable, step) in prepared.problem().variables.iter().zip(step) {
        let Some(candidate) = context.bindings().get(&variable.symbol) else {
            return affine_krawczyk_report(
                AffineKrawczykStatus::UnboundCandidateSymbol {
                    symbol: variable.symbol,
                },
                variable_count,
                variable_count,
                steps,
            );
        };
        let Some(radius) = radius_by_symbol.get(&variable.symbol) else {
            return affine_krawczyk_report(
                AffineKrawczykStatus::MissingVariableRadius {
                    symbol: variable.symbol,
                },
                variable_count,
                variable_count,
                steps,
            );
        };
        let displacement = match abs_real(&step) {
            Some(value) => value,
            None => {
                return affine_krawczyk_report(
                    AffineKrawczykStatus::UndecidedContainment {
                        symbol: variable.symbol,
                    },
                    variable_count,
                    variable_count,
                    steps,
                );
            }
        };
        match compare_reals_with_policy(&displacement, radius, policy).value() {
            Some(Ordering::Less | Ordering::Equal) => {}
            Some(Ordering::Greater) => {
                return affine_krawczyk_report(
                    AffineKrawczykStatus::RootOutsideBox {
                        symbol: variable.symbol,
                    },
                    variable_count,
                    variable_count,
                    steps,
                );
            }
            None => {
                return affine_krawczyk_report(
                    AffineKrawczykStatus::UndecidedContainment {
                        symbol: variable.symbol,
                    },
                    variable_count,
                    variable_count,
                    steps,
                );
            }
        }
        steps.push(AffineKrawczykVariableStep {
            symbol: variable.symbol,
            candidate: candidate.clone(),
            certified_root: candidate.clone() + step.clone(),
            step,
            radius: radius.clone(),
        });
    }

    affine_krawczyk_report(
        AffineKrawczykStatus::CertifiedUniqueRoot,
        variable_count,
        variable_count,
        steps,
    )
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

fn validate_krawczyk_variable_balls(
    variable_balls: &[VariableBall],
    policy: PredicatePolicy,
) -> Result<HashMap<SymbolId, Real>, AffineKrawczykStatus> {
    let mut radii = HashMap::new();
    for ball in variable_balls {
        match compare_reals_with_policy(&ball.radius, &Real::zero(), policy).value() {
            Some(Ordering::Less) => {
                return Err(AffineKrawczykStatus::NegativeVariableRadius {
                    symbol: ball.symbol,
                });
            }
            Some(Ordering::Equal | Ordering::Greater) => {
                radii.insert(ball.symbol, ball.radius.clone());
            }
            None => {
                return Err(AffineKrawczykStatus::UnknownVariableRadiusSign {
                    symbol: ball.symbol,
                });
            }
        }
    }
    Ok(radii)
}

fn solve_exact_linear_system(
    mut matrix: Vec<Vec<Real>>,
    mut rhs: Vec<Real>,
    policy: PredicatePolicy,
) -> Result<Vec<Real>, AffineKrawczykStatus> {
    let n = rhs.len();
    for pivot in 0..n {
        let pivot_row = (pivot..n).find(|&row| {
            !matches!(
                compare_reals_with_policy(&matrix[row][pivot], &Real::zero(), policy).value(),
                Some(Ordering::Equal) | None
            )
        });
        let Some(pivot_row) = pivot_row else {
            return Err(AffineKrawczykStatus::SingularOrUnsupportedPivot { pivot });
        };
        if pivot_row != pivot {
            matrix.swap(pivot_row, pivot);
            rhs.swap(pivot_row, pivot);
        }

        let pivot_value = matrix[pivot][pivot].clone();
        for value in matrix[pivot].iter_mut().skip(pivot) {
            *value = (value.clone() / pivot_value.clone())
                .map_err(|_| AffineKrawczykStatus::SingularOrUnsupportedPivot { pivot })?;
        }
        rhs[pivot] = (rhs[pivot].clone() / pivot_value)
            .map_err(|_| AffineKrawczykStatus::SingularOrUnsupportedPivot { pivot })?;
        let pivot_tail = matrix[pivot][pivot..].to_vec();
        let pivot_rhs = rhs[pivot].clone();

        for row in 0..n {
            if row == pivot {
                continue;
            }
            let factor = matrix[row][pivot].clone();
            if compare_reals_with_policy(&factor, &Real::zero(), policy).value()
                == Some(Ordering::Equal)
            {
                continue;
            }
            for (value, pivot_value) in matrix[row].iter_mut().skip(pivot).zip(&pivot_tail) {
                *value = value.clone() - factor.clone() * pivot_value.clone();
            }
            rhs[row] = rhs[row].clone() - factor * pivot_rhs.clone();
        }
    }
    Ok(rhs)
}

fn affine_krawczyk_report(
    status: AffineKrawczykStatus,
    variable_count: usize,
    equality_rows: usize,
    steps: Vec<AffineKrawczykVariableStep>,
) -> AffineKrawczykReport {
    AffineKrawczykReport {
        status,
        variable_count,
        equality_rows,
        steps,
    }
}

fn abs_real(value: &Real) -> Option<Real> {
    match compare_reals_with_policy(value, &Real::zero(), PredicatePolicy::default()).value()? {
        std::cmp::Ordering::Less => Some(-value.clone()),
        std::cmp::Ordering::Equal | std::cmp::Ordering::Greater => Some(value.clone()),
    }
}
