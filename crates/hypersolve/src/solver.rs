use hyperreal::Real;

use crate::diagnostics::{ConvergenceReason, SolveReport};
use crate::eval::context_from_problem;
use crate::jacobian::{
    FiniteDifferenceConfig, finite_difference_jacobian, symbolic_jacobian_prepared,
};
use crate::linalg::{DenseLinearBackend, LinearBackend};
use crate::model::Problem;
use crate::prepared::PreparedProblem;

#[derive(Clone, Debug)]
pub struct SolverConfig {
    pub max_iterations: usize,
    pub residual_tolerance: Real,
    pub step_tolerance: Real,
    pub damping: Real,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            max_iterations: 32,
            residual_tolerance: real_from_dense_solver_f64(1.0e-9),
            step_tolerance: real_from_dense_solver_f64(1.0e-9),
            damping: real_from_dense_solver_f64(1.0e-6),
        }
    }
}

#[derive(Clone, Debug)]
pub struct SolverState {
    pub problem: Problem,
    pub config: SolverConfig,
}

pub fn solve_damped_least_squares(mut state: SolverState) -> SolveReport {
    let backend = DenseLinearBackend;
    let mut last_residuals = Vec::new();
    let mut linear_reports = Vec::new();
    let residual_tolerance = state
        .config
        .residual_tolerance
        .to_f64_lossy()
        .unwrap_or(0.0);
    let step_tolerance = state.config.step_tolerance.to_f64_lossy().unwrap_or(0.0);
    let damping = state.config.damping.to_f64_lossy().unwrap_or(1.0e-6);

    for iteration in 0..state.config.max_iterations {
        let prepared = PreparedProblem::new(&state.problem);
        let context = context_from_problem(&state.problem);
        let Ok(residuals) = prepared.evaluate_residuals(&context) else {
            return SolveReport {
                reason: ConvergenceReason::EvaluationFailed,
                iterations: iteration,
                residuals: last_residuals,
                linear_reports,
            };
        };
        let numeric = residuals
            .iter()
            .filter_map(|row| row.weighted_dense_solver_estimate)
            .collect::<Vec<_>>();
        let norm = numeric
            .iter()
            .map(|value| value * value)
            .sum::<f64>()
            .sqrt();
        last_residuals = residuals;
        if norm <= residual_tolerance {
            return SolveReport {
                reason: ConvergenceReason::Converged,
                iterations: iteration,
                residuals: last_residuals,
                linear_reports,
            };
        }

        let jacobian = match symbolic_jacobian_prepared(&prepared, &context) {
            Ok(jacobian) => jacobian,
            Err(_) => {
                let Ok(jacobian) = finite_difference_jacobian(
                    &state.problem,
                    &context,
                    FiniteDifferenceConfig::default(),
                ) else {
                    return SolveReport {
                        reason: ConvergenceReason::EvaluationFailed,
                        iterations: iteration,
                        residuals: last_residuals,
                        linear_reports,
                    };
                };
                jacobian
            }
        };
        // f64 is confined to this dense linear-solver edge. The surrounding
        // model, residuals, bounds, and tolerances remain hyperreal values.
        let Ok((step, linear_report)) = backend.solve_damped_normal(&jacobian, &numeric, damping)
        else {
            return SolveReport {
                reason: ConvergenceReason::LinearSolveFailed,
                iterations: iteration,
                residuals: last_residuals,
                linear_reports,
            };
        };
        linear_reports.push(linear_report);
        let step_norm = step.iter().map(|value| value * value).sum::<f64>().sqrt();
        if step_norm <= step_tolerance {
            return SolveReport {
                reason: ConvergenceReason::StepTooSmall,
                iterations: iteration,
                residuals: last_residuals,
                linear_reports,
            };
        }
        for (variable, delta) in state.problem.variables.iter_mut().zip(step) {
            if variable.fixed {
                continue;
            }
            let current = variable.value.to_f64_lossy().unwrap_or(0.0);
            let mut next = current + delta;
            if let Some(lower) = variable.lower.as_ref().and_then(Real::to_f64_lossy) {
                next = next.max(lower);
            }
            if let Some(upper) = variable.upper.as_ref().and_then(Real::to_f64_lossy) {
                next = next.min(upper);
            }
            variable.value = Real::try_from(next).unwrap_or_else(|_| variable.value.clone());
        }
    }

    SolveReport {
        reason: ConvergenceReason::MaxIterations,
        iterations: state.config.max_iterations,
        residuals: last_residuals,
        linear_reports,
    }
}

fn real_from_dense_solver_f64(value: f64) -> Real {
    Real::try_from(value).expect("solver defaults are finite dyadic dense-solver values")
}
