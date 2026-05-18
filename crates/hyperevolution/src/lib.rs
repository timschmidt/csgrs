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

pub mod fitness;
pub mod identity;
pub mod search;

pub use fitness::{
    FitnessComparison, FitnessDirection, FitnessReport, FitnessValue, ParetoRelation,
};
pub use hyperreal::Real;
pub use identity::CandidateId;
pub use search::{
    Archive, Candidate, CrossoverPolicy, Genome, HillClimbPolicy, MutationPolicy, Population,
    ReplayPolicy, ReplayStatus, SelectionPolicy, SimulatedAnnealingPolicy,
};
