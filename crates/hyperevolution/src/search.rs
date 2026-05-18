//! Candidate, population, archive, and proposal-policy carriers.

use hyperreal::Real;

use crate::{CandidateId, FitnessDirection, FitnessReport, FitnessValue, ParetoRelation};

/// Exact replay status for a proposed candidate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReplayStatus {
    /// Domain replay accepted the candidate.
    Accepted,
    /// Domain replay rejected the candidate.
    Rejected,
    /// Replay has not certified a decision.
    Unknown,
}

/// Exact genome encoding.
#[derive(Clone, Debug, PartialEq)]
pub struct Genome {
    /// Exact genes.
    pub genes: Vec<Real>,
}

/// Candidate and replay policy.
#[derive(Clone, Debug, PartialEq)]
pub struct Candidate {
    /// Candidate id.
    pub id: CandidateId,
    /// Candidate genome.
    pub genome: Genome,
    /// Replay policy required before acceptance.
    pub replay_policy: ReplayPolicy,
}

/// Candidate population.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Population {
    /// Candidate set.
    pub candidates: Vec<Candidate>,
}

/// Replay policy for accepted proposals.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ReplayPolicy {
    /// Seed used to reproduce proposal generation.
    pub seed: u64,
    /// Whether exact replay is required before archive insertion.
    pub require_exact_replay: bool,
}

/// Selection policy carrier.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SelectionPolicy {
    /// Deterministic best-by-exact-fitness selection.
    ExactBest,
    /// Tournament proposal policy.
    Tournament { size: usize },
}

/// Mutation policy carrier.
#[derive(Clone, Debug, PartialEq)]
pub enum MutationPolicy {
    /// Add an exact delta to one gene.
    ExactDelta { gene: usize, delta: Box<Real> },
    /// External stochastic proposal.
    StochasticProposal { name: String },
}

/// Crossover policy carrier.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CrossoverPolicy {
    /// One-point crossover proposal.
    OnePoint { index: usize },
    /// No crossover.
    None,
}

/// Simulated annealing proposal schedule.
#[derive(Clone, Debug, PartialEq)]
pub struct SimulatedAnnealingPolicy {
    /// Initial exact temperature surrogate.
    pub initial_temperature: Real,
    /// Cooling ratio as an exact scalar.
    pub cooling_ratio: Real,
}

/// Hill-climbing proposal policy.
#[derive(Clone, Debug, PartialEq)]
pub struct HillClimbPolicy {
    /// Exact step size.
    pub step: Real,
    /// Maximum proposal count.
    pub max_steps: usize,
}

/// Non-dominated archive.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Archive {
    reports: Vec<FitnessReport>,
}

impl Population {
    /// Adds a candidate to the population.
    pub fn push(&mut self, candidate: Candidate) {
        self.candidates.push(candidate);
    }
}

impl Archive {
    /// Returns archived reports.
    pub fn reports(&self) -> &[FitnessReport] {
        &self.reports
    }

    /// Inserts a report when exact replay permits it.
    pub fn insert_replayed(&mut self, report: FitnessReport) -> bool {
        if report.replay != ReplayStatus::Accepted {
            return false;
        }
        self.reports.push(report);
        true
    }

    /// Inserts a Pareto report while removing dominated accepted reports.
    pub fn insert_non_dominated(
        &mut self,
        report: FitnessReport,
        direction: FitnessDirection,
    ) -> bool {
        if report.replay != ReplayStatus::Accepted {
            return false;
        }
        if self.reports.iter().any(|existing| {
            existing.value.compare_pareto(&report.value, direction) == ParetoRelation::Dominates
        }) {
            return false;
        }
        self.reports.retain(|existing| {
            report.value.compare_pareto(&existing.value, direction) != ParetoRelation::Dominates
        });
        self.reports.push(report);
        true
    }
}

impl FitnessReport {
    /// Creates a scalar fitness report.
    pub fn scalar(candidate: CandidateId, value: Real, replay: ReplayStatus) -> Self {
        Self {
            candidate,
            value: FitnessValue::Scalar(Box::new(value)),
            replay,
            evidence: Vec::new(),
        }
    }
}
