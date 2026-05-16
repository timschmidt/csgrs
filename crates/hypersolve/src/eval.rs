use std::collections::HashMap;

use hyperreal::{Real, RealSign};

use crate::model::{ConstraintKind, Problem};
use crate::symbolic::{ExprEvalError, SymbolId};

#[derive(Clone, Debug, Default)]
pub struct EvaluationContext {
    bindings: HashMap<SymbolId, Real>,
    generation: u64,
}

impl EvaluationContext {
    pub fn bind(&mut self, symbol: SymbolId, value: Real) {
        self.bindings.insert(symbol, value);
        self.generation += 1;
    }

    pub fn generation(&self) -> u64 {
        self.generation
    }

    pub fn bindings(&self) -> &HashMap<SymbolId, Real> {
        &self.bindings
    }
}

#[derive(Clone, Debug)]
pub struct ResidualEvaluation {
    pub name: String,
    pub value: Real,
    pub dense_solver_estimate: Option<f64>,
    pub weighted_dense_solver_estimate: Option<f64>,
    pub sign: Option<RealSign>,
}

#[derive(Clone, Debug)]
pub enum EvalError {
    Expr(ExprEvalError),
    DenseSolverEstimateUnavailable(&'static str),
}

impl From<ExprEvalError> for EvalError {
    fn from(value: ExprEvalError) -> Self {
        Self::Expr(value)
    }
}

pub fn context_from_problem(problem: &Problem) -> EvaluationContext {
    let mut context = EvaluationContext::default();
    for variable in &problem.variables {
        context.bind(variable.symbol, variable.value.clone());
    }
    context
}

pub fn evaluate_residuals(
    problem: &Problem,
    context: &EvaluationContext,
) -> Result<Vec<ResidualEvaluation>, EvalError> {
    let mut residuals = Vec::new();
    for constraint in &problem.constraints {
        if !constraint.active {
            continue;
        }
        let value = constraint.residual.eval_real(context.bindings())?;
        let signed = match constraint.kind {
            ConstraintKind::Equality | ConstraintKind::Soft => value,
            ConstraintKind::LessOrEqual => positive_part(value),
            ConstraintKind::GreaterOrEqual => positive_part(-value),
        };
        let facts = signed.structural_facts();
        // f64 values are dense-solver estimates for the external linear
        // backend only.
        // The residual value and active/inactive decision remain hyperreal.
        let weighted = signed.clone() * constraint.weight.clone();
        residuals.push(ResidualEvaluation {
            name: constraint.name.clone(),
            dense_solver_estimate: signed.to_f64_approx(),
            weighted_dense_solver_estimate: weighted.to_f64_approx(),
            sign: facts.sign,
            value: signed,
        });
    }
    Ok(residuals)
}

fn positive_part(value: Real) -> Real {
    match value.structural_facts().sign {
        Some(RealSign::Negative) => Real::zero(),
        Some(RealSign::Zero) => Real::zero(),
        _ => value,
    }
}
