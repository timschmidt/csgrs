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
pub mod polynomial;
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
    DirectAffineSolution, DirectQuadraticSolution, DirectSolveError, EqualitySubstitution,
    EqualitySubstitutionClass, EqualitySubstitutionClassMember, EqualitySubstitutionCycle,
    EqualitySubstitutionProblem, EqualitySubstitutionValidation, apply_equality_substitutions,
    build_equality_substitution_classes, find_equality_substitution_cycles,
    find_equality_substitutions, solve_direct_affine_equalities,
    solve_direct_univariate_quadratic_equalities, validate_equality_substitutions,
};
pub use domain::geometry::{
    Point2 as SolverPoint2, named_equation, point_coincidence_equations, squared_distance_equation,
    squared_distance_expr, tangent_cross_expr, tangent_dot_expr, tangent_parallel_equation,
    tangent_same_direction_constraint,
};
pub use domain::pcb::{
    PcbConstraintSet, center_clearance_squared_constraint, differential_pair_skew_equation,
};
pub use domain::toolpath::{
    RectangularRegion, ToolpathConstraintSet, bezier_offset_sample_constraints,
    constant_feed_time_equation, length_match_equation, rectangular_difference_area_equation,
    rectangular_region_area_equation, rectangular_region_containment_constraints,
};
pub use eval::{
    EvalError, EvaluationContext, ResidualEvaluation, context_from_problem, evaluate_residuals,
};
pub use interval::{
    AffineIntervalError, QuadraticIntervalError, VariableBall, certify_affine_interval_candidate,
    certify_multivariate_quadratic_interval_candidate, certify_quadratic_interval_candidate,
};
pub use linalg::{
    DenseLinearBackend, LinearAdapterKind, LinearAdapterPrecision, LinearBackend, LinearSolveError,
    LinearSolveReport,
};
pub use model::{Constraint, ConstraintKind, Problem, Variable, VariableId};
pub use polynomial::{
    PreparedLinearTerm, PreparedQuadraticResidual, PreparedQuadraticTerm,
    PreparedUnivariateQuadraticResidual, prepare_quadratic_residual,
    prepare_univariate_quadratic_residual,
};
pub use predicates::{Classification, PredicateBackend, PredicateReport};
pub use prepared::{
    PreparedConstraintFacts, PreparedProblem, PreparedProblemFacts, facts_depend_on_symbol,
};
pub use solver::{SolverConfig, SolverState, solve_damped_least_squares};
pub use solver_block::{
    PreparedSolverBlock, PreparedSolverBlockFacts, SolverBlockRow, SolverBlockRowKind,
};
pub use symbolic::{Expr, ExprDegree, ExprEvalError, ExprFacts, SymbolId, SymbolRef};
