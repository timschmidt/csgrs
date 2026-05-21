use hyperreal::Real;

use crate::diagnostics::{
    ConvergenceReason, ProposalEngineKind, ProposalEnginePrecision, ProposalEngineReport,
    SolveReport,
};
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
    /// Numerical engine used only to propose candidate coordinates.
    ///
    /// The current implementation supports dense damped least squares and the
    /// named Levenberg-Marquardt route. Dogleg uses a dense lossy
    /// trust-region proposal. These routes follow the least-squares damping family of
    /// Levenberg, "A Method for the Solution of Certain Non-Linear Problems in
    /// Least Squares" (1944), and Marquardt, "An Algorithm for Least-Squares
    /// Estimation of Nonlinear Parameters" (1963), plus Powell's dogleg hybrid
    /// method (M. J. D. Powell, "A Hybrid Method for Nonlinear Equations",
    /// 1970). Other named engines are exposed so callers and tests can
    /// distinguish unsupported proposal requests from exact certification
    /// failures.
    pub proposal_engine: ProposalEngineKind,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            max_iterations: 32,
            residual_tolerance: real_from_dense_solver_f64(1.0e-9),
            step_tolerance: real_from_dense_solver_f64(1.0e-9),
            damping: real_from_dense_solver_f64(1.0e-6),
            proposal_engine: ProposalEngineKind::DampedLeastSquares,
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
    let proposal_engine = proposal_engine_report(state.config.proposal_engine);
    if !proposal_engine.supported {
        return SolveReport {
            reason: ConvergenceReason::UnsupportedProposalEngine,
            iterations: 0,
            proposal_engine,
            residuals: last_residuals,
            linear_reports,
        };
    }
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
                proposal_engine,
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
                proposal_engine,
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
                        proposal_engine,
                        residuals: last_residuals,
                        linear_reports,
                    };
                };
                jacobian
            }
        };
        // f64 is confined to this dense linear-solver edge. The surrounding
        // model, residuals, bounds, and tolerances remain hyperreal values.
        // DampedLeastSquares and LevenbergMarquardt use a damped normal step;
        // Dogleg uses a trust-region step. Per Yap (1997), exact/certified
        // replay, not this lossy proposal, decides acceptance.
        let step_result = match state.config.proposal_engine {
            ProposalEngineKind::Dogleg => backend.solve_dogleg(&jacobian, &numeric, damping),
            ProposalEngineKind::DampedLeastSquares | ProposalEngineKind::LevenbergMarquardt => {
                backend.solve_damped_normal(&jacobian, &numeric, damping)
            }
            ProposalEngineKind::PowellHybrid
            | ProposalEngineKind::Bfgs
            | ProposalEngineKind::Sqp => {
                unreachable!("unsupported proposal engines return before iteration")
            }
        };
        let Ok((step, linear_report)) = step_result else {
            return SolveReport {
                reason: ConvergenceReason::LinearSolveFailed,
                iterations: iteration,
                proposal_engine,
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
                proposal_engine,
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
        proposal_engine,
        residuals: last_residuals,
        linear_reports,
    }
}

fn proposal_engine_report(requested: ProposalEngineKind) -> ProposalEngineReport {
    if requested.is_implemented() {
        ProposalEngineReport {
            requested,
            used: Some(requested),
            precision: ProposalEnginePrecision::LossyF64,
            supported: true,
        }
    } else {
        ProposalEngineReport {
            requested,
            used: None,
            precision: ProposalEnginePrecision::Unsupported,
            supported: false,
        }
    }
}

fn real_from_dense_solver_f64(value: f64) -> Real {
    Real::try_from(value).expect("solver defaults are finite dyadic dense-solver values")
}
