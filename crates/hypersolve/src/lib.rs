//! Iterative constraint solving experiments built on `hyperreal`.
//!
//! This first pass establishes the public seams from `PLAN.md`: symbolic
//! expressions with variable leaves, residual evaluation, a linear-solver
//! facade, a predicate/classification facade, and a small iterative solver
//! skeleton.

pub mod affine;
pub mod diagnostics;
pub mod domain;
pub mod eval;
pub mod jacobian;
pub mod linalg;
pub mod model;
pub mod predicates;
pub mod prepared;
pub mod solver;
pub mod symbolic;

pub use affine::PreparedAffineResidual;
pub use diagnostics::{ConvergenceReason, SolveReport};
pub use eval::{
    EvalError, EvaluationContext, ResidualEvaluation, context_from_problem, evaluate_residuals,
};
pub use linalg::{
    DenseLinearBackend, LinearAdapterKind, LinearAdapterPrecision, LinearBackend, LinearSolveError,
    LinearSolveReport,
};
pub use model::{Constraint, ConstraintKind, Problem, Variable, VariableId};
pub use predicates::{Classification, PredicateBackend, PredicateReport};
pub use prepared::{
    PreparedConstraintFacts, PreparedProblem, PreparedProblemFacts, facts_depend_on_symbol,
};
pub use solver::{SolverConfig, SolverState, solve_damped_least_squares};
pub use symbolic::{Expr, ExprDegree, ExprEvalError, ExprFacts, SymbolId, SymbolRef};
