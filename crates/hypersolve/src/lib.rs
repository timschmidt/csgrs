//! Iterative constraint solving experiments built on `hyperreal`.
//!
//! This first pass establishes the public seams from `PLAN.md`: symbolic
//! expressions with variable leaves, residual evaluation, a linear-solver
//! facade, a predicate/classification facade, and a small iterative solver
//! skeleton.

pub mod active_set;
pub mod affine;
pub mod algebraic;
pub mod alpha;
pub mod bareiss;
pub mod batch;
pub mod branches;
pub mod certification;
pub mod diagnostics;
pub mod direct;
pub mod domain;
pub mod domain_certification;
pub mod eval;
pub mod failed_constraints;
pub mod interval;
pub mod jacobian;
pub mod linalg;
pub mod model;
pub mod polynomial;
pub mod predicates;
pub mod prepared;
pub mod rank;
pub mod residual_replay;
pub mod resultant;
pub mod root_isolation;
pub mod sketch;
pub mod sketch_builders;
pub mod sketch_certificates;
pub mod sketch_degeneracy;
pub mod sketch_domains;
pub mod sketch_entity_domains;
pub mod sketch_fixtures;
pub mod sketch_units;
pub mod solver;
pub mod solver_block;
pub mod sparse_pattern;
pub mod symbolic;

pub use active_set::{
    ActiveSetAffineRegenerationReport, ActiveSetAffineRegenerationStatus, ActiveSetAuditReport,
    ActiveSetAuditRow, ActiveSetLoopConfig, ActiveSetLoopIteration, ActiveSetLoopReport,
    ActiveSetLoopStatus, ActiveSetRowStatus, ActiveSetUpdateAction, ActiveSetUpdateReport,
    ActiveSetUpdateRow, audit_active_mask, audit_active_set, propose_active_set_update,
    regenerate_active_set_affine_candidate, run_active_set_update_loop,
};
pub use affine::PreparedAffineResidual;
pub use algebraic::{
    AlgebraicRootArithmeticOp, AlgebraicRootArithmeticReport, AlgebraicRootArithmeticStatus,
    AlgebraicRootComparisonReport, AlgebraicRootComparisonStatus, AlgebraicRootKind,
    AlgebraicRootRefinementComparisonConfig, AlgebraicRootRefinementComparisonReport,
    AlgebraicRootRepresentation, AlgebraicRootRepresentationReport,
    AlgebraicRootRepresentationStatus, AlgebraicRootValidationReport,
    AlgebraicRootValidationStatus, arithmetic_algebraic_root_representations,
    compare_algebraic_root_representations, compare_algebraic_root_representations_with_refinement,
    represent_univariate_algebraic_roots, represent_univariate_algebraic_roots_from_reports,
    validate_algebraic_root_representation,
};
pub use alpha::{
    UnivariateQuadraticAlphaReport, UnivariateQuadraticAlphaRow, UnivariateQuadraticAlphaStatus,
    certify_univariate_quadratic_alpha,
};
pub use bareiss::{
    BareissDeterminantReport, BareissError, BareissPivot, BareissSolveReport, SparseBareissError,
    SparseBareissSolveReport, determinant_bareiss, solve_dense_linear_system_bareiss,
    solve_sparse_linear_system_bareiss,
};
pub use batch::{
    BatchCandidateCertificationReport, BatchCandidateReplay, BatchCandidateStatus,
    BatchPredicateScheduleConfig, BatchPredicateScheduleError, BatchPredicateScheduleReport,
    BatchPredicateWorkItem, certify_candidate_batch, certify_candidate_batch_with_config,
    schedule_candidate_batch_predicates,
};
pub use branches::{
    ExactBranchEnumerationReport, ExactBranchStatus, ExactSolutionBranch,
    enumerate_direct_univariate_quadratic_branches,
    enumerate_direct_univariate_quadratic_branches_with_config,
};
pub use certification::{
    CandidateCertificationConfig, CandidateCertificationReport, CandidateResidualBall,
    CertifiedCandidateRow, CertifiedCandidateStatus, certify_candidate,
    certify_candidate_with_config, certify_candidate_with_residual_balls,
    report_lossy_adapter_only_candidate,
};
pub use diagnostics::{
    ConvergenceReason, ProposalEngineKind, ProposalEnginePrecision, ProposalEngineReport,
    SolveReport,
};
pub use direct::{
    DirectAffineSolution, DirectAffineSystemAssignment, DirectAffineSystemReport,
    DirectAffineSystemStatus, DirectQuadraticCandidateReport, DirectQuadraticCandidateStatus,
    DirectQuadraticSolution, DirectSolveError, EliminatedAffineRow, EqualitySubstitution,
    EqualitySubstitutionClass, EqualitySubstitutionClassApplicationReport,
    EqualitySubstitutionClassApplicationRow, EqualitySubstitutionClassApplicationStatus,
    EqualitySubstitutionClassMember, EqualitySubstitutionCycle,
    EqualitySubstitutionEliminationReport, EqualitySubstitutionProblem,
    EqualitySubstitutionValidation, apply_equality_substitution_classes,
    apply_equality_substitutions, build_equality_substitution_classes,
    certify_direct_univariate_quadratic_roots,
    certify_direct_univariate_quadratic_roots_with_config,
    eliminate_affine_rows_with_substitution_classes, find_equality_substitution_cycles,
    find_equality_substitutions, solve_direct_affine_equalities, solve_direct_affine_system,
    solve_direct_univariate_quadratic_equalities, validate_equality_substitutions,
};
pub use domain::geometry::{
    Point2 as SolverPoint2, named_equation, point_coincidence_equations, squared_distance_equation,
    squared_distance_expr, tangent_cross_expr, tangent_dot_expr, tangent_parallel_equation,
    tangent_same_direction_constraint,
};
pub use domain_certification::{
    CandidateDomainReport, DomainCheck, DomainCheckKind, DomainCheckStatus,
    certify_candidate_domains,
};
pub use eval::{
    EvalError, EvaluationContext, ResidualEvaluation, context_from_problem, evaluate_residuals,
};
pub use failed_constraints::{
    FailedConstraintReport, FailedConstraintRow, FailedConstraintStatus,
    diagnose_failed_constraints, diagnose_failed_constraints_from_certification,
    diagnose_failed_constraints_with_config,
};
pub use interval::{
    AffineIntervalError, AffineKrawczykReport, AffineKrawczykStatus, AffineKrawczykVariableStep,
    IntervalBoxCertificationPackage, IntervalBoxCertificationReport,
    IntervalBoxCertificationStatus, MultivariateQuadraticKrawczykReport,
    MultivariateQuadraticKrawczykResidual, MultivariateQuadraticKrawczykStatus,
    MultivariateQuadraticKrawczykVariable, QuadraticIntervalError, QuadraticKrawczykReport,
    QuadraticKrawczykRow, QuadraticKrawczykStatus, VariableBall, certify_affine_interval_candidate,
    certify_affine_krawczyk_box, certify_interval_box_candidate,
    certify_multivariate_quadratic_interval_candidate, certify_multivariate_quadratic_krawczyk_box,
    certify_quadratic_interval_candidate, certify_univariate_quadratic_krawczyk_box,
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
pub use rank::{ExactAffineRankReport, ExactAffineRankStatus, analyze_exact_affine_rank};
pub use residual_replay::{
    DenseResidualReplayError, DenseResidualReplayReport, PreparedSparseLinearSystem,
    SparseResidualBatchReplay, SparseResidualBatchReport, SparseResidualBatchStatus,
    SparseResidualReplayError, SparseResidualReplayReport, SparseResidualReplayRow,
    SparseResidualTerm, prepare_sparse_linear_residual_system, replay_dense_linear_residuals,
    replay_sparse_linear_residual_batch, replay_sparse_linear_residual_batch_prepared,
    replay_sparse_linear_residuals,
};
pub use resultant::{
    ResultantInputSide, UnivariateResultantError, UnivariateResultantPairInput,
    UnivariateResultantPairReport, UnivariateResultantPairStatus, UnivariateResultantReport,
    UnivariateResultantScheduleReport, UnivariateSubresultantChainError,
    UnivariateSubresultantChainReport, UnivariateSubresultantChainStep,
    resultant_univariate_polynomials, schedule_univariate_resultant_pairs,
    subresultant_chain_univariate_polynomials,
};
pub use root_isolation::{
    AlgebraicRootCandidateReport, AlgebraicRootCandidateStatus, BernsteinRootCountReport,
    BernsteinRootCountStatus, BernsteinSubdivisionConfig, BernsteinSubdivisionInterval,
    BernsteinSubdivisionIntervalStatus, BernsteinSubdivisionReport, BernsteinSubdivisionStatus,
    DescartesRootCountReport, DescartesRootCountStatus, IsolatedRootInterval,
    IsolatedRootRefinementReport, IsolatedRootRefinementStatus, RootIsolationConfig,
    RootIsolationStatus, RootMultiplicityStatus, UnivariateRootIsolationReport,
    certify_isolated_rational_root_witnesses, certify_isolated_rational_root_witnesses_with_config,
    count_bernstein_univariate_polynomial_interval_expr,
    count_bernstein_univariate_polynomial_interval_roots,
    count_descartes_univariate_polynomial_expr, count_descartes_univariate_polynomial_roots,
    isolate_univariate_polynomial_expr, isolate_univariate_polynomial_expr_with_config,
    isolate_univariate_polynomial_roots, isolate_univariate_polynomial_roots_with_config,
    refine_isolated_univariate_polynomial_interval,
    subdivide_bernstein_univariate_polynomial_interval_expr,
    subdivide_bernstein_univariate_polynomial_interval_roots,
};
pub use sketch::{
    SketchArcOfCircle2, SketchCircle2, SketchConstraint, SketchConstraintHandle,
    SketchConstraintKind, SketchCubic2, SketchDistance, SketchEntity, SketchEntityDomain,
    SketchEntityHandle, SketchEntityKind, SketchGeneratedRow, SketchGeneratedRowStatus,
    SketchGroupHandle, SketchLineSegment2, SketchLoweringReport, SketchNormal2, SketchNormal3,
    SketchParameter, SketchParameterDomain, SketchParameterHandle, SketchPoint2, SketchPoint3,
    SketchResidualForm, SketchResidualFormKind, SketchResidualFormRole, SketchResidualFormsReport,
    SketchResidualFormsStatus, SketchResidualStrategy, SketchRoundTripMetadata,
    SketchRoundTripRole, SketchSolveProblem, SketchWorkplane,
};
pub use sketch_builders::{
    SketchConstraintBuildReport, SketchConstraintFamily, distance as sketch_distance_builders,
    incidence as sketch_incidence_builders, objective as sketch_objective_builders,
    orientation as sketch_orientation_builders, ranges as sketch_range_builders,
};
pub use sketch_certificates::{
    SketchConstructionCertificate, SketchConstructionCertificateStatus,
    SketchConstructionProvenance, SketchConstructionTraceCounts, certify_sketch_construction,
};
pub use sketch_degeneracy::{
    SketchDegeneracyCheck, SketchDegeneracyKind, SketchDegeneracyReport, SketchDegeneracyStatus,
    preflight_sketch_degeneracies, preflight_sketch_degeneracies_with_policy,
};
pub use sketch_domains::{
    SketchParameterDomainCheck, SketchParameterDomainKind, SketchParameterDomainReport,
    SketchParameterDomainStatus, preflight_sketch_parameter_domains,
    preflight_sketch_parameter_domains_with_policy,
};
pub use sketch_entity_domains::{
    SketchEntityDomainCheck, SketchEntityDomainKind, SketchEntityDomainReport,
    SketchEntityDomainStatus, preflight_sketch_entity_domains,
    preflight_sketch_entity_domains_with_policy,
};
pub use sketch_fixtures::{
    SketchCompatibilityFixture, SketchCompatibilityFixtureKind, SketchCompatibilityReplayReport,
    replay_sketch_compatibility_fixture, sketch_compatibility_fixtures,
};
pub use sketch_units::{
    SketchUnitToleranceObjectKind, SketchUnitToleranceReport, SketchUnitToleranceRow,
    SketchUnitToleranceStatus, audit_sketch_unit_tolerances,
    audit_sketch_unit_tolerances_with_policy,
};
pub use solver::{SolverConfig, SolverState, solve_damped_least_squares};
pub use solver_block::{
    PreparedSolverBlock, PreparedSolverBlockFacts, SolverBlockRow, SolverBlockRowKind,
};
pub use sparse_pattern::{
    SparsePatternEntry, SparsePatternEntryStatus, SparsePatternError,
    SymbolicSparseEliminationStep, SymbolicSparseFactorizationReport,
    analyze_sparse_bareiss_elimination_pattern,
};
pub use symbolic::{Expr, ExprDegree, ExprEvalError, ExprFacts, SymbolId, SymbolRef};
