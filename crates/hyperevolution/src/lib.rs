//! Exact-aware evolutionary and proposal-search carriers.
//!
//! `hyperevolution` owns candidate encodings, populations, fitness reports,
//! exact comparison policies, archives, and replay policies for the Hyper
//! ecosystem. It treats stochastic search as proposal generation: accepted
//! candidates must replay through exact/certified predicates, residuals, or
//! domain reports.
//!
//! This follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997
//! (<https://doi.org/10.1016/0925-7721(95)00040-2>) at the optimization
//! boundary. Search heuristics such as Holland's genetic algorithms
//! ("Adaptation in Natural and Artificial Systems", 1975) and Kirkpatrick,
//! Gelatt, and Vecchi's simulated annealing ("Optimization by Simulated
//! Annealing", *Science* 220(4598), 1983) can propose candidates, but exact
//! ranking and replay evidence remain first-class data.

pub mod domain;
pub mod fitness;
pub mod gp;
pub mod identity;
pub mod oracle;
pub mod search;

pub use domain::{
    DomainReplayManifest, DomainReplayReport, DomainReplayTarget, domain_replay_manifest,
};
pub use fitness::{
    FitnessComparison, FitnessDirection, FitnessInterval, FitnessIntervalComparison, FitnessReport,
    FitnessValue, ParetoRelation,
};
pub use gp::{
    GpRealExpr, GpValidationIssue, GpValidationLimits, GpValidationReport, eval_gp_batch,
};
pub use hyperreal::Real;
pub use identity::CandidateId;
pub use oracle::{
    BlackBoxEvaluationReport, ConstructionDependency, EvaluationCacheKey, EvaluationCost,
    FitnessOracle, ReplayHook, SurrogateDecision, SurrogateScreen, SurrogateScreenReport,
    SurrogateStage, evaluate_candidate_with_oracle, surrogate_allows_archive_promotion,
};
pub use search::{
    AnnealingAcceptance, Archive, Candidate, CrossoverPolicy, DiversityRelation, DiversityReport,
    Genome, HillClimbPolicy, HillClimbReport, HillClimbStopReason, HillClimbStrategy,
    MutationPolicy, Population, ReplayPolicy, ReplayStatus, SelectionError, SelectionPolicy,
    SelectionReport, SimulatedAnnealingPolicy, VariationError,
    classify_simulated_annealing_neighbor, crossover_one_point, exact_structural_diversity,
    hill_climb_exact, mutate_exact_delta, select_exact_best, select_tournament_by_indices,
};
