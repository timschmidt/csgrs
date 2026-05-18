//! Iterative constraint solving experiments built on `hyperreal`.
//!
//! This first pass establishes the public seams from `PLAN.md`: symbolic
//! expressions with variable leaves, residual evaluation, a linear-solver
//! facade, a predicate/classification facade, and a small iterative solver
//! skeleton.

pub mod affine;
pub mod certification;
pub mod diagnostics;
pub mod direct;
pub mod domain;
pub mod eval;
pub mod interval;
pub mod jacobian;
pub mod linalg;
pub mod model;
pub mod predicates;
pub mod prepared;
pub mod solver;
pub mod solver_block;
pub mod symbolic;

pub use affine::PreparedAffineResidual;
pub use certification::{
    CandidateCertificationConfig, CandidateCertificationReport, CandidateResidualBall,
    CertifiedCandidateRow, CertifiedCandidateStatus, certify_candidate,
    certify_candidate_with_config, certify_candidate_with_residual_balls,
};
pub use diagnostics::{ConvergenceReason, SolveReport};
pub use direct::{
    DirectAffineSolution, DirectSolveError, EqualitySubstitution, apply_equality_substitutions,
    find_equality_substitutions, solve_direct_affine_equalities,
};
pub use eval::{
    EvalError, EvaluationContext, ResidualEvaluation, context_from_problem, evaluate_residuals,
};
pub use interval::{AffineIntervalError, VariableBall, certify_affine_interval_candidate};
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
pub use solver_block::{
    PreparedSolverBlock, PreparedSolverBlockFacts, SolverBlockRow, SolverBlockRowKind,
};
pub use symbolic::{Expr, ExprDegree, ExprEvalError, ExprFacts, SymbolId, SymbolRef};
