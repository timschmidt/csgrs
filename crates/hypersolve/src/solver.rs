use hyperreal::Real;

use crate::diagnostics::{
    ConvergenceReason, ProposalEngineKind, ProposalEnginePrecision, ProposalEngineReport,
    ProposalPreprocessingReport, SolveReport,
};
use crate::direct::{
    find_equality_substitutions, solve_direct_affine_equalities,
    solve_direct_univariate_quadratic_equalities,
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
    /// named Levenberg-Marquardt route. ModifiedNewtonLeastSquares names the
    /// SolveSpace-style symbolic-Jacobian/damped-Newton proposal shape.
    /// PowellHybrid and Dogleg use dense lossy trust-region proposals, BFGS
    /// uses a retained dense inverse-Hessian approximation, and SQP uses the
    /// current equality least-squares QP relaxation. These routes follow the
    /// least-squares damping family of
    /// Levenberg, "A Method for the Solution of Certain Non-Linear Problems in
    /// Least Squares" (1944), and Marquardt, "An Algorithm for Least-Squares
    /// Estimation of Nonlinear Parameters" (1963), plus Powell's dogleg hybrid
    /// method (M. J. D. Powell, "A Hybrid Method for Nonlinear Equations",
    /// 1970), and the BFGS quasi-Newton family of Broyden/Fletcher/Goldfarb/
    /// Shanno (1970). SQP here means the local equality-constrained quadratic
    /// least-squares proposal described in Nocedal and Wright, *Numerical
    /// Optimization*, 2nd ed. (2006), not a proof of constrained optimality.
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
    let preprocessing = proposal_preprocessing_report(&state.problem, state.config.proposal_engine);
    if !proposal_engine.supported {
        return SolveReport {
            reason: ConvergenceReason::UnsupportedProposalEngine,
            iterations: 0,
            proposal_engine,
            residuals: last_residuals,
            linear_reports,
            preprocessing,
        };
    }
    let residual_tolerance = state
        .config
        .residual_tolerance
        .to_f64_lossy()
        .unwrap_or(0.0);
    let step_tolerance = state.config.step_tolerance.to_f64_lossy().unwrap_or(0.0);
    let damping = state.config.damping.to_f64_lossy().unwrap_or(1.0e-6);
    let mut inverse_hessian = identity_matrix(state.problem.variables.len());
    let mut previous_point: Option<Vec<f64>> = None;
    let mut previous_gradient: Option<Vec<f64>> = None;

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
                preprocessing,
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
                preprocessing,
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
                        preprocessing,
                    };
                };
                jacobian
            }
        };
        // f64 is confined to this dense linear-solver edge. The surrounding
        // model, residuals, bounds, and tolerances remain hyperreal values.
        let gradient = dense_gradient(&jacobian, &numeric);
        if state.config.proposal_engine == ProposalEngineKind::Bfgs
            && let (Some(previous_point), Some(previous_gradient)) =
                (&previous_point, &previous_gradient)
        {
            let current_point = dense_point(&state.problem);
            let s = current_point
                .iter()
                .zip(previous_point)
                .map(|(current, previous)| current - previous)
                .collect::<Vec<_>>();
            let y = gradient
                .iter()
                .zip(previous_gradient)
                .map(|(current, previous)| current - previous)
                .collect::<Vec<_>>();
            update_inverse_hessian_bfgs(&mut inverse_hessian, &s, &y);
        }
        previous_point = Some(dense_point(&state.problem));
        previous_gradient = Some(gradient.clone());

        // DampedLeastSquares, LevenbergMarquardt, ModifiedNewtonLeastSquares,
        // and the equality-SQP relaxation use a damped normal step;
        // PowellHybrid and Dogleg use a trust-region step; BFGS uses the
        // retained inverse-Hessian direction.
        // Per Yap (1997), exact/certified replay, not this lossy proposal,
        // decides acceptance.
        let step_result = match state.config.proposal_engine {
            ProposalEngineKind::PowellHybrid | ProposalEngineKind::Dogleg => {
                backend.solve_dogleg(&jacobian, &numeric, damping)
            }
            ProposalEngineKind::Bfgs => {
                backend.solve_bfgs_direction(&inverse_hessian, &gradient, damping)
            }
            ProposalEngineKind::DampedLeastSquares
            | ProposalEngineKind::LevenbergMarquardt
            | ProposalEngineKind::ModifiedNewtonLeastSquares
            | ProposalEngineKind::Sqp => backend.solve_damped_normal(&jacobian, &numeric, damping),
        };
        let Ok((step, linear_report)) = step_result else {
            return SolveReport {
                reason: ConvergenceReason::LinearSolveFailed,
                iterations: iteration,
                proposal_engine,
                residuals: last_residuals,
                linear_reports,
                preprocessing,
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
                preprocessing,
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
        preprocessing,
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

fn proposal_preprocessing_report(
    problem: &Problem,
    requested: ProposalEngineKind,
) -> ProposalPreprocessingReport {
    if requested != ProposalEngineKind::ModifiedNewtonLeastSquares {
        return ProposalPreprocessingReport::not_requested();
    }
    let prepared = PreparedProblem::new(problem);
    let substitutions = match find_equality_substitutions(&prepared) {
        Ok(substitutions) => substitutions.len(),
        Err(_) => {
            return ProposalPreprocessingReport {
                requested: true,
                equality_substitutions: 0,
                affine_soluble_alone_rows: 0,
                quadratic_soluble_alone_rows: 0,
                completed: false,
            };
        }
    };
    let affine_soluble = match solve_direct_affine_equalities(&prepared) {
        Ok(solutions) => solutions.len(),
        Err(_) => {
            return ProposalPreprocessingReport {
                requested: true,
                equality_substitutions: substitutions,
                affine_soluble_alone_rows: 0,
                quadratic_soluble_alone_rows: 0,
                completed: false,
            };
        }
    };
    let quadratic_soluble = match solve_direct_univariate_quadratic_equalities(&prepared) {
        Ok(solutions) => solutions.len(),
        Err(_) => {
            return ProposalPreprocessingReport {
                requested: true,
                equality_substitutions: substitutions,
                affine_soluble_alone_rows: affine_soluble,
                quadratic_soluble_alone_rows: 0,
                completed: false,
            };
        }
    };
    ProposalPreprocessingReport {
        requested: true,
        equality_substitutions: substitutions,
        affine_soluble_alone_rows: affine_soluble,
        quadratic_soluble_alone_rows: quadratic_soluble,
        completed: true,
    }
}

fn dense_point(problem: &Problem) -> Vec<f64> {
    problem
        .variables
        .iter()
        .map(|variable| variable.value.to_f64_lossy().unwrap_or(0.0))
        .collect()
}

fn dense_gradient(jacobian: &[Vec<f64>], residuals: &[f64]) -> Vec<f64> {
    let width = jacobian.first().map(Vec::len).unwrap_or(0);
    let mut gradient = vec![0.0; width];
    for (row, residual) in jacobian.iter().zip(residuals) {
        for column in 0..width {
            gradient[column] += row[column] * residual;
        }
    }
    gradient
}

fn identity_matrix(width: usize) -> Vec<Vec<f64>> {
    let mut matrix = vec![vec![0.0; width]; width];
    for (index, row) in matrix.iter_mut().enumerate() {
        row[index] = 1.0;
    }
    matrix
}

fn update_inverse_hessian_bfgs(inverse_hessian: &mut Vec<Vec<f64>>, s: &[f64], y: &[f64]) {
    let ys = dot(y, s);
    if ys <= f64::EPSILON {
        return;
    }
    let hy = mat_vec(inverse_hessian, y);
    let yhy = dot(y, &hy);
    if yhy.abs() <= f64::EPSILON {
        return;
    }
    for row in 0..inverse_hessian.len() {
        for column in 0..inverse_hessian.len() {
            inverse_hessian[row][column] += s[row] * s[column] / ys - hy[row] * hy[column] / yhy;
        }
    }
}

fn mat_vec(matrix: &[Vec<f64>], vector: &[f64]) -> Vec<f64> {
    matrix
        .iter()
        .map(|row| {
            row.iter()
                .zip(vector)
                .map(|(left, right)| left * right)
                .sum()
        })
        .collect()
}

fn dot(left: &[f64], right: &[f64]) -> f64 {
    left.iter()
        .zip(right)
        .map(|(left, right)| left * right)
        .sum()
}

fn real_from_dense_solver_f64(value: f64) -> Real {
    Real::try_from(value).expect("solver defaults are finite dyadic dense-solver values")
}
