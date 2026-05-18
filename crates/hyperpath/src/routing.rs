//! Continuous routing parameters delegated to `hypersolve`.
//!
//! PCB autorouters can use graph search, maze routing, SAT, or network flow to
//! propose topology, but length tuning is a continuous parameter problem once a
//! topology is fixed. This module creates a deliberately small
//! `hypersolve` problem for one exact extra-length variable. The resulting
//! candidate still has to be checked by path predicates before it can become
//! geometry, matching Yap's separation between numeric proposals and certified
//! geometric decisions. For the routing background, see Yan, Ma, and Wong,
//! "Advances in PCB Routing," *Journal of Information Processing* 2014.

use hyperreal::Real;
use hypersolve::{
    CandidateCertificationReport, Constraint, Expr, PreparedProblem, Problem, SymbolId,
    certify_candidate, context_from_problem,
};

/// Exact length-match solve model for one continuous extension parameter.
#[derive(Clone, Debug)]
pub struct LengthMatchProblem {
    /// Solver problem containing the extension variable.
    pub problem: Problem,
    /// Symbol used for the extra length variable.
    pub extra_length_symbol: SymbolId,
}

/// Build a one-variable exact residual `current + extra - target = 0`.
pub fn build_length_match_problem(
    current: Real,
    target: Real,
    initial_extra: Real,
) -> LengthMatchProblem {
    let mut problem = Problem::default();
    let variable = problem.add_variable("extra_length", initial_extra);
    if let Some(row) = problem.variables.get_mut(variable.0 as usize) {
        row.lower = Some(Real::zero());
    }
    let symbol = SymbolId(variable.0);
    let residual = Expr::real(current) + Expr::symbol(symbol, "extra_length") - Expr::real(target);
    problem.add_constraint(Constraint::equality("length match", residual));
    LengthMatchProblem {
        problem,
        extra_length_symbol: symbol,
    }
}

/// Certify the current extra-length candidate by exact residual replay.
pub fn certify_length_extension(model: &LengthMatchProblem) -> CandidateCertificationReport {
    let prepared = PreparedProblem::new(&model.problem);
    let context = context_from_problem(&model.problem);
    certify_candidate(&prepared, &context)
}
