//! Black-box objective and surrogate-screening surfaces.
//!
//! A black-box evaluator may be opaque, expensive, or backed by a simulator,
//! but it still must return report-bearing evidence. This module defines a
//! small oracle boundary that carries exact fitness, evaluation cost, cache
//! keys, construction dependencies, replay hooks, and surrogate screening
//! status together. COCO/BBOB-style black-box benchmarks can use this shape as
//! performance fixtures, while Hyper domain candidates still need exact replay
//! before acceptance. Heuristic or lossy proposal stages are useful, but
//! exact/certified evidence decides promotion. The README lists the benchmark
//! and exact-computation references.

use crate::{Candidate, FitnessReport, ReplayStatus};

/// Stable cache key returned by a black-box evaluator.
#[derive(Clone, Debug, Eq, Hash, PartialEq)]
pub struct EvaluationCacheKey(pub String);

/// Construction dependency recorded by an evaluator.
#[derive(Clone, Debug, Eq, Hash, PartialEq)]
pub struct ConstructionDependency(pub String);

/// Replay hook that a domain owner can use to recertify a candidate.
#[derive(Clone, Debug, Eq, Hash, PartialEq)]
pub struct ReplayHook {
    /// Domain or crate expected to own replay.
    pub domain: String,
    /// Stable replay target inside that domain.
    pub target: String,
}

/// Evaluation cost reported by an oracle.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EvaluationCost {
    /// Number of objective calls spent.
    pub calls: usize,
    /// Abstract cost units for domain-specific accounting.
    pub units: usize,
}

impl EvaluationCost {
    /// One ordinary black-box call.
    pub const fn one_call() -> Self {
        Self { calls: 1, units: 1 }
    }
}

/// Named surrogate or approximate screening stage.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SurrogateStage {
    /// Human-readable stage name.
    pub name: String,
    /// Whether this stage is lossy/approximate.
    pub lossy: bool,
}

/// Surrogate screening decision.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SurrogateDecision {
    /// Candidate may proceed to exact/domain replay.
    PromoteToReplay,
    /// Candidate is only a proposal and must not enter accepted archives.
    ProposalOnly,
    /// Candidate was rejected by the surrogate screen.
    Reject,
}

/// Full report from a black-box objective call.
#[derive(Clone, Debug, PartialEq)]
pub struct BlackBoxEvaluationReport {
    /// Candidate fitness report.
    pub fitness: FitnessReport,
    /// Evaluation cost.
    pub cost: EvaluationCost,
    /// Optional deterministic cache key.
    pub cache_key: Option<EvaluationCacheKey>,
    /// Construction dependencies consumed by the evaluation.
    pub dependencies: Vec<ConstructionDependency>,
    /// Replay hooks that can recertify the candidate.
    pub replay_hooks: Vec<ReplayHook>,
    /// Surrogate stage, when this was not direct exact/domain replay.
    pub surrogate_stage: Option<SurrogateStage>,
}

impl BlackBoxEvaluationReport {
    /// Returns whether the report is eligible for accepted archive promotion.
    pub fn is_promotable(&self) -> bool {
        self.surrogate_stage.is_none() && self.fitness.replay == ReplayStatus::Accepted
    }

    /// Returns whether exact/domain replay is still required before promotion.
    pub fn requires_replay(&self) -> bool {
        self.surrogate_stage.is_some() || self.fitness.replay != ReplayStatus::Accepted
    }
}

/// Trait for opaque domain or benchmark objectives.
pub trait FitnessOracle {
    /// Evaluate a candidate and return report-bearing fitness evidence.
    fn evaluate(&self, candidate: &Candidate) -> BlackBoxEvaluationReport;
}

/// Trait for approximate screening stages.
pub trait SurrogateScreen {
    /// Screen a candidate before exact/domain replay.
    fn screen(&self, candidate: &Candidate) -> SurrogateScreenReport;
}

/// Report from a surrogate screen.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SurrogateScreenReport {
    /// Candidate id text for diagnostics.
    pub candidate: String,
    /// Stage that produced this decision.
    pub stage: SurrogateStage,
    /// Screening decision.
    pub decision: SurrogateDecision,
    /// Optional deterministic cache key.
    pub cache_key: Option<EvaluationCacheKey>,
}

/// Evaluate a candidate through an oracle.
pub fn evaluate_candidate_with_oracle<O: FitnessOracle>(
    oracle: &O,
    candidate: &Candidate,
) -> BlackBoxEvaluationReport {
    oracle.evaluate(candidate)
}

/// Returns whether a surrogate-screened candidate may enter an accepted archive.
///
/// Surrogate output alone is never promotable. Even `PromoteToReplay` only
/// means the candidate should be sent to exact/domain replay next.
pub fn surrogate_allows_archive_promotion(report: &SurrogateScreenReport) -> bool {
    let _ = report;
    false
}
