use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use hyperlimit::{PredicatePolicy, compare_reals_with_policy};
use hyperreal::Real;

use crate::diagnostics::{
    ConvergenceReason, ProposalEngineKind, ProposalEnginePrecision, ProposalEngineReport,
    ProposalPreprocessingReport, SolveReport,
};
use crate::direct::{
    build_equality_substitution_classes, find_equality_substitutions,
    solve_direct_affine_equalities, solve_direct_univariate_quadratic_equalities,
};
use crate::eval::context_from_problem;
use crate::jacobian::{
    FiniteDifferenceConfig, finite_difference_jacobian, symbolic_jacobian_prepared,
};
use crate::linalg::{DenseLinearBackend, LinearBackend};
use crate::model::{Problem, VariableId};
use crate::prepared::PreparedProblem;
use crate::symbolic::SymbolId;

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
    /// Dragged-parameter proposal weights for
    /// [`ProposalEngineKind::ModifiedNewtonLeastSquares`].
    ///
    /// These rows model the SolveSpace-style "prefer this edited parameter
    /// target" behavior as lossy proposal-only soft rows. They never become
    /// exact proof obligations; final acceptance still requires exact residual
    /// replay. See Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997), for the proof boundary.
    pub dragged_parameters: Vec<DraggedParameterWeight>,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            max_iterations: 32,
            residual_tolerance: real_from_dense_solver_f64(1.0e-9),
            step_tolerance: real_from_dense_solver_f64(1.0e-9),
            damping: real_from_dense_solver_f64(1.0e-6),
            proposal_engine: ProposalEngineKind::DampedLeastSquares,
            dragged_parameters: Vec::new(),
        }
    }
}

/// Proposal-only weight pulling one variable toward a dragged target.
///
/// The row added to the dense modified-Newton proposal is
/// `weight * (variable - target)`. It is intentionally not inserted into the
/// exact [`Problem`]: this mirrors SolveSpace-style interactive dragging while
/// preserving Yap's rule that approximate proposal aids are not topology or
/// feasibility evidence. See Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Debug, PartialEq)]
pub struct DraggedParameterWeight {
    /// Variable to bias during proposal generation.
    pub variable: VariableId,
    /// Desired proposal target value.
    pub target: Real,
    /// Strictly positive proposal weight.
    pub weight: Real,
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
    let mut preprocessing = proposal_preprocessing_report(&state.problem, &state.config);
    let mut seeded_symbols =
        apply_modified_newton_affine_seeds(&mut state.problem, &state.config, &mut preprocessing);
    apply_modified_newton_quadratic_seeds(
        &mut state.problem,
        &state.config,
        &mut preprocessing,
        &mut seeded_symbols,
    );
    apply_modified_newton_substitution_seeds(
        &mut state.problem,
        &state.config,
        &mut preprocessing,
        &mut seeded_symbols,
    );
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
        let mut numeric = residuals
            .iter()
            .filter_map(|row| row.weighted_dense_solver_estimate)
            .collect::<Vec<_>>();
        let dragged_rows = dragged_parameter_dense_rows(&state.problem, &state.config);
        numeric.extend(dragged_rows.residuals.iter().copied());
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

        let mut jacobian = match symbolic_jacobian_prepared(&prepared, &context) {
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
        jacobian.extend(dragged_rows.jacobian);
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
    config: &SolverConfig,
) -> ProposalPreprocessingReport {
    let requested = config.proposal_engine;
    if requested != ProposalEngineKind::ModifiedNewtonLeastSquares {
        return ProposalPreprocessingReport::not_requested();
    }
    let dragged_rows = dragged_parameter_dense_rows(problem, config);
    let prepared = PreparedProblem::new(problem);
    let substitutions = match find_equality_substitutions(&prepared) {
        Ok(substitutions) => substitutions.len(),
        Err(_) => {
            return ProposalPreprocessingReport {
                requested: true,
                equality_substitutions: 0,
                substitution_seed_classes: 0,
                rejected_substitution_seed_classes: 0,
                substitution_seed_assignments: 0,
                affine_soluble_alone_rows: 0,
                quadratic_soluble_alone_rows: 0,
                affine_seed_assignments: 0,
                rejected_affine_seed_assignments: 0,
                quadratic_seed_assignments: 0,
                rejected_quadratic_seed_assignments: 0,
                dragged_parameter_weights: dragged_rows.valid_count,
                invalid_dragged_parameter_weights: dragged_rows.invalid_count,
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
                substitution_seed_classes: 0,
                rejected_substitution_seed_classes: 0,
                substitution_seed_assignments: 0,
                affine_soluble_alone_rows: 0,
                quadratic_soluble_alone_rows: 0,
                affine_seed_assignments: 0,
                rejected_affine_seed_assignments: 0,
                quadratic_seed_assignments: 0,
                rejected_quadratic_seed_assignments: 0,
                dragged_parameter_weights: dragged_rows.valid_count,
                invalid_dragged_parameter_weights: dragged_rows.invalid_count,
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
                substitution_seed_classes: 0,
                rejected_substitution_seed_classes: 0,
                substitution_seed_assignments: 0,
                affine_soluble_alone_rows: affine_soluble,
                quadratic_soluble_alone_rows: 0,
                affine_seed_assignments: 0,
                rejected_affine_seed_assignments: 0,
                quadratic_seed_assignments: 0,
                rejected_quadratic_seed_assignments: 0,
                dragged_parameter_weights: dragged_rows.valid_count,
                invalid_dragged_parameter_weights: dragged_rows.invalid_count,
                completed: false,
            };
        }
    };
    ProposalPreprocessingReport {
        requested: true,
        equality_substitutions: substitutions,
        substitution_seed_classes: 0,
        rejected_substitution_seed_classes: 0,
        substitution_seed_assignments: 0,
        affine_soluble_alone_rows: affine_soluble,
        quadratic_soluble_alone_rows: quadratic_soluble,
        affine_seed_assignments: 0,
        rejected_affine_seed_assignments: 0,
        quadratic_seed_assignments: 0,
        rejected_quadratic_seed_assignments: 0,
        dragged_parameter_weights: dragged_rows.valid_count,
        invalid_dragged_parameter_weights: dragged_rows.invalid_count,
        completed: true,
    }
}

fn apply_modified_newton_affine_seeds(
    problem: &mut Problem,
    config: &SolverConfig,
    preprocessing: &mut ProposalPreprocessingReport,
) -> BTreeSet<SymbolId> {
    let mut seeded = BTreeSet::new();
    if config.proposal_engine != ProposalEngineKind::ModifiedNewtonLeastSquares {
        return seeded;
    }
    let solutions = {
        let prepared = PreparedProblem::new(problem);
        solve_direct_affine_equalities(&prepared)
    };
    let Ok(solutions) = solutions else {
        return seeded;
    };
    for solution in solutions {
        let Some(variable_index) = problem
            .variables
            .iter()
            .position(|variable| variable.symbol == solution.symbol)
        else {
            preprocessing.rejected_affine_seed_assignments += 1;
            continue;
        };
        let variable = &mut problem.variables[variable_index];
        if variable.fixed
            || !value_within_bounds(
                &solution.value,
                variable.lower.as_ref(),
                variable.upper.as_ref(),
            )
        {
            preprocessing.rejected_affine_seed_assignments += 1;
            continue;
        }
        variable.value = solution.value;
        seeded.insert(solution.symbol);
        preprocessing.affine_seed_assignments += 1;
    }
    seeded
}

fn apply_modified_newton_quadratic_seeds(
    problem: &mut Problem,
    config: &SolverConfig,
    preprocessing: &mut ProposalPreprocessingReport,
    seeded_symbols: &mut BTreeSet<SymbolId>,
) {
    if config.proposal_engine != ProposalEngineKind::ModifiedNewtonLeastSquares {
        return;
    }
    let solutions = {
        let prepared = PreparedProblem::new(problem);
        solve_direct_univariate_quadratic_equalities(&prepared)
    };
    let Ok(solutions) = solutions else {
        return;
    };
    for solution in solutions {
        if solution.roots.len() != 1 || seeded_symbols.contains(&solution.symbol) {
            preprocessing.rejected_quadratic_seed_assignments += 1;
            continue;
        }
        let root = solution.roots[0].clone();
        let Some(variable_index) = problem
            .variables
            .iter()
            .position(|variable| variable.symbol == solution.symbol)
        else {
            preprocessing.rejected_quadratic_seed_assignments += 1;
            continue;
        };
        let variable = &mut problem.variables[variable_index];
        if variable.fixed
            || !value_within_bounds(&root, variable.lower.as_ref(), variable.upper.as_ref())
        {
            preprocessing.rejected_quadratic_seed_assignments += 1;
            continue;
        }
        variable.value = root;
        seeded_symbols.insert(solution.symbol);
        preprocessing.quadratic_seed_assignments += 1;
    }
}

fn apply_modified_newton_substitution_seeds(
    problem: &mut Problem,
    config: &SolverConfig,
    preprocessing: &mut ProposalPreprocessingReport,
    seeded_symbols: &mut BTreeSet<SymbolId>,
) {
    if config.proposal_engine != ProposalEngineKind::ModifiedNewtonLeastSquares {
        return;
    }
    let classes = {
        let prepared = PreparedProblem::new(problem);
        let Ok(substitutions) = find_equality_substitutions(&prepared) else {
            return;
        };
        let Ok(classes) = build_equality_substitution_classes(&substitutions) else {
            return;
        };
        classes
    };
    let symbol_to_variable = problem
        .variables
        .iter()
        .enumerate()
        .map(|(index, variable)| (variable.symbol, index))
        .collect::<BTreeMap<_, _>>();

    for class in classes {
        if class.members.len() < 2 {
            continue;
        }
        let Some(representative_value) =
            substitution_representative_seed(problem, &symbol_to_variable, &class, seeded_symbols)
        else {
            preprocessing.rejected_substitution_seed_classes += 1;
            continue;
        };
        let mut proposed = Vec::new();
        let mut rejected = false;
        for member in &class.members {
            let Some(&variable_index) = symbol_to_variable.get(&member.symbol) else {
                rejected = true;
                break;
            };
            let value = representative_value.clone() + member.offset_from_representative.clone();
            let variable = &problem.variables[variable_index];
            if variable.fixed && variable.value != value {
                rejected = true;
                break;
            }
            if !value_within_bounds(&value, variable.lower.as_ref(), variable.upper.as_ref()) {
                rejected = true;
                break;
            }
            proposed.push((variable_index, member.symbol, value));
        }
        if rejected {
            preprocessing.rejected_substitution_seed_classes += 1;
            continue;
        }
        for (variable_index, symbol, value) in proposed {
            if problem.variables[variable_index].fixed {
                continue;
            }
            problem.variables[variable_index].value = value;
            seeded_symbols.insert(symbol);
            preprocessing.substitution_seed_assignments += 1;
        }
        preprocessing.substitution_seed_classes += 1;
    }
}

/// Pick the exact representative value used to seed one substitution class.
///
/// SolveSpace-style substitution treats rows like `x = y + c` as symbolic
/// pre-Newton construction. Following Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997), this helper only
/// constructs a proposal state; exact residual replay remains the proof of
/// feasibility. Already exact-seeded or fixed members are preferred anchors so
/// direct affine/quadratic evidence is not overwritten by arbitrary initial
/// coordinates.
fn substitution_representative_seed(
    problem: &Problem,
    symbol_to_variable: &BTreeMap<SymbolId, usize>,
    class: &crate::direct::EqualitySubstitutionClass,
    seeded_symbols: &BTreeSet<SymbolId>,
) -> Option<Real> {
    let mut anchored: Option<Real> = None;
    for member in &class.members {
        let variable = problem
            .variables
            .get(*symbol_to_variable.get(&member.symbol)?)?;
        if seeded_symbols.contains(&member.symbol) || variable.fixed {
            let representative_value =
                variable.value.clone() - member.offset_from_representative.clone();
            if let Some(existing) = &anchored {
                if existing != &representative_value {
                    return None;
                }
            } else {
                anchored = Some(representative_value);
            }
        }
    }
    if anchored.is_some() {
        return anchored;
    }
    let representative = class
        .members
        .iter()
        .find(|member| member.symbol == class.representative)?;
    let variable = problem
        .variables
        .get(*symbol_to_variable.get(&representative.symbol)?)?;
    Some(variable.value.clone() - representative.offset_from_representative.clone())
}

fn value_within_bounds(value: &Real, lower: Option<&Real>, upper: Option<&Real>) -> bool {
    if let Some(lower) = lower
        && matches!(
            compare_reals_with_policy(value, lower, PredicatePolicy::default()).value(),
            Some(Ordering::Less) | None
        )
    {
        return false;
    }
    if let Some(upper) = upper
        && matches!(
            compare_reals_with_policy(value, upper, PredicatePolicy::default()).value(),
            Some(Ordering::Greater) | None
        )
    {
        return false;
    }
    true
}

#[derive(Clone, Debug, Default)]
struct DraggedParameterDenseRows {
    residuals: Vec<f64>,
    jacobian: Vec<Vec<f64>>,
    valid_count: usize,
    invalid_count: usize,
}

fn dragged_parameter_dense_rows(
    problem: &Problem,
    config: &SolverConfig,
) -> DraggedParameterDenseRows {
    if config.proposal_engine != ProposalEngineKind::ModifiedNewtonLeastSquares {
        return DraggedParameterDenseRows::default();
    }
    let mut rows = DraggedParameterDenseRows::default();
    for dragged in &config.dragged_parameters {
        let Some(variable) = problem.variables.get(dragged.variable.0 as usize) else {
            rows.invalid_count += 1;
            continue;
        };
        if variable.fixed {
            rows.invalid_count += 1;
            continue;
        }
        if compare_reals_with_policy(&dragged.weight, &Real::zero(), PredicatePolicy::default())
            .value()
            != Some(Ordering::Greater)
        {
            rows.invalid_count += 1;
            continue;
        }
        let Some(current) = variable.value.to_f64_lossy() else {
            rows.invalid_count += 1;
            continue;
        };
        let Some(target) = dragged.target.to_f64_lossy() else {
            rows.invalid_count += 1;
            continue;
        };
        let Some(weight) = dragged.weight.to_f64_lossy() else {
            rows.invalid_count += 1;
            continue;
        };
        if !current.is_finite() || !target.is_finite() || !weight.is_finite() {
            rows.invalid_count += 1;
            continue;
        }
        let mut row = vec![0.0; problem.variables.len()];
        row[dragged.variable.0 as usize] = weight;
        rows.residuals.push((current - target) * weight);
        rows.jacobian.push(row);
        rows.valid_count += 1;
    }
    rows
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
