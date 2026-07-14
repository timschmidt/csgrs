use hyperreal::{Real, RealSign};

use crate::eval::{EvalError, EvaluationContext};
use crate::model::{ConstraintKind, Problem};
use crate::prepared::PreparedProblem;

#[derive(Clone, Debug)]
pub struct FiniteDifferenceConfig {
    pub step: Real,
}

impl Default for FiniteDifferenceConfig {
    fn default() -> Self {
        Self {
            step: Real::try_from(1.0e-6).expect("finite dyadic finite-difference step"),
        }
    }
}

pub fn finite_difference_jacobian(
    problem: &Problem,
    context: &EvaluationContext,
    config: FiniteDifferenceConfig,
) -> Result<Vec<Vec<f64>>, EvalError> {
    let step_estimate =
        config
            .step
            .to_f64_lossy()
            .ok_or(EvalError::DenseSolverEstimateUnavailable(
                "finite-difference step is not f64-estimable",
            ))?;
    let baseline = crate::eval::evaluate_residuals(problem, context)?
        .into_iter()
        .filter_map(|row| row.weighted_dense_solver_estimate)
        .collect::<Vec<_>>();
    let mut jacobian = vec![vec![0.0; problem.variables.len()]; baseline.len()];

    for (column, variable) in problem.variables.iter().enumerate() {
        if variable.fixed {
            continue;
        }
        let mut perturbed = context.clone();
        let shifted = variable.value.clone() + config.step.clone();
        perturbed.bind(variable.symbol, shifted);
        let next = crate::eval::evaluate_residuals(problem, &perturbed)?
            .into_iter()
            .filter_map(|row| row.weighted_dense_solver_estimate)
            .collect::<Vec<_>>();
        for (row, (next, base)) in next.iter().zip(&baseline).enumerate() {
            jacobian[row][column] = (next - base) / step_estimate;
        }
    }

    Ok(jacobian)
}

pub fn symbolic_jacobian(
    problem: &Problem,
    context: &EvaluationContext,
) -> Result<Vec<Vec<f64>>, EvalError> {
    symbolic_jacobian_with_sparsity(problem, context, None)
}

/// Evaluate a symbolic Jacobian using cached dependency sparsity.
///
/// The returned matrix is still the dense `f64` adapter shape required by the
/// current normal-equation backend. The difference is semantic ownership:
/// structural zero columns are selected from [`PreparedProblem`] facts instead
/// of rediscovered by differentiating every variable in every row. That follows
/// the exact-geometric-computation split between preserved object structure
/// and approximate arithmetic adapters; see the exact-geometric-computation model.
pub fn symbolic_jacobian_prepared(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
) -> Result<Vec<Vec<f64>>, EvalError> {
    symbolic_jacobian_with_sparsity(
        prepared.problem(),
        context,
        Some(prepared.jacobian_sparsity()),
    )
}

fn symbolic_jacobian_with_sparsity(
    problem: &Problem,
    context: &EvaluationContext,
    structural_sparsity: Option<&[Vec<bool>]>,
) -> Result<Vec<Vec<f64>>, EvalError> {
    let mut rows = Vec::new();
    for (constraint_index, constraint) in problem.constraints.iter().enumerate() {
        if !constraint.active {
            continue;
        }
        let residual = constraint.residual.eval_real(context.bindings())?;
        let active = match constraint.kind {
            ConstraintKind::Equality | ConstraintKind::Soft => true,
            ConstraintKind::LessOrEqual => !matches!(
                residual.structural_facts().sign,
                Some(RealSign::Negative | RealSign::Zero)
            ),
            ConstraintKind::GreaterOrEqual => !matches!(
                residual.structural_facts().sign,
                Some(RealSign::Positive | RealSign::Zero)
            ),
        };
        let mut row = Vec::with_capacity(problem.variables.len());
        if !active {
            row.resize(problem.variables.len(), 0.0);
            rows.push(row);
            continue;
        }
        let sparsity_row = structural_sparsity.and_then(|rows| rows.get(constraint_index));
        for (column, variable) in problem.variables.iter().enumerate() {
            if sparsity_row
                .and_then(|row| row.get(column))
                .is_some_and(|depends| !depends)
            {
                row.push(0.0);
                continue;
            }
            let derivative = constraint.residual.derivative(variable.symbol);
            let value = derivative.eval_real(context.bindings())?;
            // Structural-dispatch note: derivative expressions can carry a
            // sparsity pattern and exact-rational denominator scale. Preserve
            // those facts in a future hyperreal/hyperlattice linear backend so
            // this dense-solver conversion is needed only for external f64
            // solvers.
            row.push(
                (value * constraint.weight.clone())
                    .to_f64_lossy()
                    .unwrap_or(0.0),
            );
        }
        rows.push(row);
    }
    Ok(rows)
}
