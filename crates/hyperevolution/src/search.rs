//! Candidate, population, archive, and proposal-policy carriers.
//!
//! Local search is still proposal generation: a neighbor can be evaluated
//! quickly, but promotion depends on exact/certified fitness and replay status.
//! The hill-climb helper uses deterministic first/best improvement, while exact
//! evidence—not primitive-float surrogate scores—ranks candidates. Selection
//! and variation operators preserve exact genes and return reportable
//! unsupported/unknown states instead of treating hidden random floats as
//! proof. The README collects the underlying search references.

use hyperreal::Real;

use crate::{
    CandidateId, FitnessComparison, FitnessDirection, FitnessReport, FitnessValue, ParetoRelation,
};

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

/// Result of exact/certified selection.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SelectionError {
    /// Candidate and report slices had different lengths.
    CandidateReportCountMismatch,
    /// The selection set was empty.
    EmptyPopulation,
    /// A fitness report did not name the candidate at the same aligned index.
    CandidateReportIdMismatch {
        /// Misaligned index.
        index: usize,
    },
    /// A tournament index was outside the candidate slice.
    TournamentIndexOutOfBounds {
        /// Bad index.
        index: usize,
        /// Candidate count.
        candidate_count: usize,
    },
    /// No candidate in the selection set had accepted replay evidence.
    NoAcceptedReplay,
    /// Exact/certified fitness comparison could not choose one winner.
    UnknownComparison,
}

/// Report from a deterministic selection helper.
#[derive(Clone, Debug, PartialEq)]
pub struct SelectionReport {
    /// Selected candidate.
    pub candidate: Candidate,
    /// Selected candidate fitness.
    pub fitness: FitnessReport,
    /// Number of candidates inspected.
    pub inspected: usize,
}

/// Result of exact variation operators.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum VariationError {
    /// The requested gene index did not exist.
    GeneIndexOutOfBounds {
        /// Bad index.
        index: usize,
        /// Genome length.
        len: usize,
    },
    /// Parent genomes had different arity.
    GenomeLengthMismatch {
        /// First parent length.
        left: usize,
        /// Second parent length.
        right: usize,
    },
    /// The crossover point was outside `0..=len`.
    CrossoverIndexOutOfBounds {
        /// Bad index.
        index: usize,
        /// Genome length.
        len: usize,
    },
}

/// Exact structural diversity relation between two genomes.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DiversityRelation {
    /// Genomes are exactly equal term-for-term.
    Identical,
    /// Genomes differ in at least one exact gene.
    Distinct,
    /// Genomes have different arity, so equality is not a same-shape diversity metric.
    ShapeMismatch {
        /// First genome length.
        left: usize,
        /// Second genome length.
        right: usize,
    },
}

/// Pairwise exact diversity report for a population.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DiversityReport {
    /// Number of candidates inspected.
    pub candidate_count: usize,
    /// Number of pairwise comparisons.
    pub pair_count: usize,
    /// Pairs that are exactly identical.
    pub identical_pairs: usize,
    /// Pairs that are exactly distinct with matching arity.
    pub distinct_pairs: usize,
    /// Pairs with incompatible genome lengths.
    pub shape_mismatch_pairs: usize,
}

/// Simulated annealing proposal schedule.
#[derive(Clone, Debug, PartialEq)]
pub struct SimulatedAnnealingPolicy {
    /// Initial exact temperature surrogate.
    pub initial_temperature: Real,
    /// Cooling ratio as an exact scalar.
    pub cooling_ratio: Real,
}

/// Report-bearing simulated-annealing acceptance decision.
///
/// Better exact/certified neighbors are accepted deterministically. Worse
/// neighbors require evaluating an exponential acceptance probability, which is
/// a proposal-stage computation unless a caller supplies an exact/certified
/// probabilistic comparison. Exact promotion remains separate from stochastic
/// acceptance.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AnnealingAcceptance {
    /// Neighbor is exactly better and can be accepted.
    AcceptImprovement,
    /// Neighbor ties exactly and can be accepted as a plateau move.
    AcceptEqual,
    /// Neighbor is worse; a named probabilistic proposal stage is required.
    RequiresProbabilisticProposal,
    /// Neighbor replay did not certify acceptance.
    RejectUncertifiedReplay,
    /// Fitness comparison was unknown or unsupported.
    UnknownComparison,
    /// Temperature or cooling data is outside the exact supported domain.
    InvalidSchedule,
}

/// Hill-climbing proposal policy.
#[derive(Clone, Debug, PartialEq)]
pub struct HillClimbPolicy {
    /// Exact step size.
    pub step: Real,
    /// Maximum proposal count.
    pub max_steps: usize,
    /// Improvement scan strategy.
    pub strategy: HillClimbStrategy,
    /// Whether exact ties may be accepted as plateau moves.
    pub accept_equal_plateau: bool,
    /// Number of recent genomes excluded from revisiting.
    pub tabu_memory: usize,
}

/// Deterministic hill-climb scan strategy.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HillClimbStrategy {
    /// Accept the first exact improvement in deterministic neighbor order.
    FirstImprovement,
    /// Evaluate all one-step neighbors and accept the best exact improvement.
    BestImprovement,
}

/// Reason a hill-climb run stopped.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HillClimbStopReason {
    /// No exact accepted neighbor improved the current candidate.
    LocalOptimum,
    /// The configured proposal limit was reached.
    BudgetExhausted,
    /// The initial candidate did not replay as accepted.
    InitialReplayRejected,
    /// The initial fitness could not participate in exact total comparison.
    InitialFitnessUnsupported,
}

/// Report from deterministic exact hill climbing.
#[derive(Clone, Debug, PartialEq)]
pub struct HillClimbReport {
    /// Starting candidate.
    pub initial: Candidate,
    /// Final candidate after accepted moves.
    pub final_candidate: Candidate,
    /// Final fitness report.
    pub final_fitness: FitnessReport,
    /// Stopping reason.
    pub reason: HillClimbStopReason,
    /// Number of neighbor evaluations performed.
    pub evaluations: usize,
    /// Number of accepted moves.
    pub accepted_steps: usize,
    /// Fitness reports for accepted candidates, including the initial report.
    pub accepted_reports: Vec<FitnessReport>,
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

impl HillClimbPolicy {
    /// Builds a first-improvement hill-climb policy with no plateau or tabu memory.
    pub fn first_improvement(step: Real, max_steps: usize) -> Self {
        Self {
            step,
            max_steps,
            strategy: HillClimbStrategy::FirstImprovement,
            accept_equal_plateau: false,
            tabu_memory: 0,
        }
    }

    /// Builds a best-improvement hill-climb policy with no plateau or tabu memory.
    pub fn best_improvement(step: Real, max_steps: usize) -> Self {
        Self {
            step,
            max_steps,
            strategy: HillClimbStrategy::BestImprovement,
            accept_equal_plateau: false,
            tabu_memory: 0,
        }
    }
}

/// Runs deterministic exact hill climbing over one-step genome neighbors.
///
/// Each iteration proposes `gene - step` and `gene + step` for every gene in
/// order. A neighbor can replace the current candidate only when the evaluator
/// returns [`ReplayStatus::Accepted`] and the exact fitness comparison reports
/// an improvement, or an exact tie when plateau moves are enabled. Unknown,
/// overlapping interval, rejected, or lossy-surrogate-only evidence never
/// becomes an implicit improvement.
pub fn hill_climb_exact<F>(
    initial: Candidate,
    policy: &HillClimbPolicy,
    direction: FitnessDirection,
    mut evaluate: F,
) -> HillClimbReport
where
    F: FnMut(&Candidate) -> FitnessReport,
{
    let initial_report = evaluate(&initial);
    if initial_report.replay != ReplayStatus::Accepted {
        return HillClimbReport {
            initial: initial.clone(),
            final_candidate: initial,
            final_fitness: initial_report.clone(),
            reason: HillClimbStopReason::InitialReplayRejected,
            evaluations: 1,
            accepted_steps: 0,
            accepted_reports: vec![initial_report],
        };
    }
    if !fitness_supports_total_order(&initial_report.value) {
        return HillClimbReport {
            initial: initial.clone(),
            final_candidate: initial,
            final_fitness: initial_report.clone(),
            reason: HillClimbStopReason::InitialFitnessUnsupported,
            evaluations: 1,
            accepted_steps: 0,
            accepted_reports: vec![initial_report],
        };
    }

    let mut current = initial.clone();
    let mut current_report = initial_report;
    let mut accepted_reports = vec![current_report.clone()];
    let mut recent = vec![current.genome.clone()];
    let mut evaluations = 1_usize;
    let mut accepted_steps = 0_usize;

    for _ in 0..policy.max_steps {
        let mut selected: Option<(Candidate, FitnessReport)> = None;
        match policy.strategy {
            HillClimbStrategy::FirstImprovement => {
                for neighbor in first_step_neighbors(&current, &policy.step) {
                    if recent_contains(&recent, &neighbor.genome) {
                        continue;
                    }
                    let report = evaluate(&neighbor);
                    evaluations += 1;
                    if !is_acceptable_neighbor(&current_report, &report, direction, policy) {
                        continue;
                    }
                    selected = Some((neighbor, report));
                    break;
                }
            }
            HillClimbStrategy::BestImprovement => {
                for neighbor in one_step_neighbors(&current, &policy.step) {
                    if recent_contains(&recent, &neighbor.genome) {
                        continue;
                    }
                    let report = evaluate(&neighbor);
                    evaluations += 1;
                    if !is_acceptable_neighbor(&current_report, &report, direction, policy) {
                        continue;
                    }
                    let replace = selected
                        .as_ref()
                        .map(|(_, selected_report)| {
                            report
                                .value
                                .compare_total(&selected_report.value, direction)
                                == FitnessComparison::Better
                        })
                        .unwrap_or(true);
                    if replace {
                        selected = Some((neighbor, report));
                    }
                }
            }
        }

        let Some((candidate, report)) = selected else {
            return HillClimbReport {
                initial,
                final_candidate: current,
                final_fitness: current_report,
                reason: HillClimbStopReason::LocalOptimum,
                evaluations,
                accepted_steps,
                accepted_reports,
            };
        };
        current = candidate;
        current_report = report;
        accepted_steps += 1;
        accepted_reports.push(current_report.clone());
        recent.push(current.genome.clone());
        if policy.tabu_memory > 0 && recent.len() > policy.tabu_memory + 1 {
            recent.remove(0);
        }
    }

    HillClimbReport {
        initial,
        final_candidate: current,
        final_fitness: current_report,
        reason: HillClimbStopReason::BudgetExhausted,
        evaluations,
        accepted_steps,
        accepted_reports,
    }
}

/// Classifies one simulated-annealing neighbor acceptance decision.
///
/// This helper deliberately does not approximate `exp(-delta / T)` with a
/// primitive float. Exact improvements and ties are accepted immediately;
/// worse accepted-replay neighbors return
/// [`AnnealingAcceptance::RequiresProbabilisticProposal`] so a caller can route
/// the stochastic draw through a seeded, report-bearing proposal stage.
pub fn classify_simulated_annealing_neighbor(
    current: &FitnessReport,
    neighbor: &FitnessReport,
    policy: &SimulatedAnnealingPolicy,
    direction: FitnessDirection,
) -> AnnealingAcceptance {
    if policy.initial_temperature <= Real::zero()
        || policy.cooling_ratio <= Real::zero()
        || policy.cooling_ratio > Real::one()
    {
        return AnnealingAcceptance::InvalidSchedule;
    }
    if neighbor.replay != ReplayStatus::Accepted {
        return AnnealingAcceptance::RejectUncertifiedReplay;
    }
    match neighbor.value.compare_total(&current.value, direction) {
        FitnessComparison::Better => AnnealingAcceptance::AcceptImprovement,
        FitnessComparison::Equal => AnnealingAcceptance::AcceptEqual,
        FitnessComparison::Worse => AnnealingAcceptance::RequiresProbabilisticProposal,
        FitnessComparison::Unknown => AnnealingAcceptance::UnknownComparison,
    }
}

/// Selects the exact best accepted candidate from aligned candidates/reports.
///
/// Rejected and unknown replay reports are ignored. Ties keep the first
/// candidate in source order so replay is deterministic. Any unknown
/// comparison between accepted candidates is returned as an explicit error.
pub fn select_exact_best(
    candidates: &[Candidate],
    reports: &[FitnessReport],
    direction: FitnessDirection,
) -> Result<SelectionReport, SelectionError> {
    select_best_from_indices(candidates, reports, 0..candidates.len(), direction)
}

/// Selects the exact best accepted candidate from explicit tournament indices.
///
/// The caller owns randomness and can store/replay the chosen indices. This
/// helper only performs exact ranking and replay gating; it does not sample
/// hidden random bits.
pub fn select_tournament_by_indices(
    candidates: &[Candidate],
    reports: &[FitnessReport],
    indices: &[usize],
    direction: FitnessDirection,
) -> Result<SelectionReport, SelectionError> {
    select_best_from_indices(candidates, reports, indices.iter().copied(), direction)
}

/// Applies an exact additive mutation to one gene.
pub fn mutate_exact_delta(
    candidate: &Candidate,
    gene: usize,
    delta: Real,
    child_id: CandidateId,
) -> Result<Candidate, VariationError> {
    if gene >= candidate.genome.genes.len() {
        return Err(VariationError::GeneIndexOutOfBounds {
            index: gene,
            len: candidate.genome.genes.len(),
        });
    }
    let mut genome = candidate.genome.clone();
    genome.genes[gene] = genome.genes[gene].clone() + delta;
    Ok(Candidate {
        id: child_id,
        genome,
        replay_policy: candidate.replay_policy.clone(),
    })
}

/// Produces two exact one-point crossover children.
///
/// `index == 0` swaps whole genomes; `index == len` copies both parents. Both
/// endpoints are valid because they are deterministic variation proposals, not
/// proof of diversity or quality.
pub fn crossover_one_point(
    left: &Candidate,
    right: &Candidate,
    index: usize,
    left_child_id: CandidateId,
    right_child_id: CandidateId,
) -> Result<(Candidate, Candidate), VariationError> {
    let len = left.genome.genes.len();
    if right.genome.genes.len() != len {
        return Err(VariationError::GenomeLengthMismatch {
            left: len,
            right: right.genome.genes.len(),
        });
    }
    if index > len {
        return Err(VariationError::CrossoverIndexOutOfBounds { index, len });
    }
    let mut left_genes = Vec::with_capacity(len);
    let mut right_genes = Vec::with_capacity(len);
    left_genes.extend_from_slice(&left.genome.genes[..index]);
    left_genes.extend_from_slice(&right.genome.genes[index..]);
    right_genes.extend_from_slice(&right.genome.genes[..index]);
    right_genes.extend_from_slice(&left.genome.genes[index..]);
    Ok((
        Candidate {
            id: left_child_id,
            genome: Genome { genes: left_genes },
            replay_policy: left.replay_policy.clone(),
        },
        Candidate {
            id: right_child_id,
            genome: Genome { genes: right_genes },
            replay_policy: right.replay_policy.clone(),
        },
    ))
}

/// Reports exact structural genome diversity for a population.
///
/// This deliberately does not infer diversity from approximate coordinate
/// distance. Exact equality and arity mismatches are enough for a first
/// diversity invariant; richer certified distance enclosures should be added as
/// separate report-bearing metrics.
pub fn exact_structural_diversity(candidates: &[Candidate]) -> DiversityReport {
    let mut report = DiversityReport {
        candidate_count: candidates.len(),
        pair_count: 0,
        identical_pairs: 0,
        distinct_pairs: 0,
        shape_mismatch_pairs: 0,
    };
    for left in 0..candidates.len() {
        for right in (left + 1)..candidates.len() {
            report.pair_count += 1;
            match compare_genome_structure(&candidates[left].genome, &candidates[right].genome) {
                DiversityRelation::Identical => report.identical_pairs += 1,
                DiversityRelation::Distinct => report.distinct_pairs += 1,
                DiversityRelation::ShapeMismatch { .. } => report.shape_mismatch_pairs += 1,
            }
        }
    }
    report
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

fn first_step_neighbors<'a>(
    candidate: &'a Candidate,
    step: &'a Real,
) -> impl Iterator<Item = Candidate> + 'a {
    candidate
        .genome
        .genes
        .iter()
        .enumerate()
        .flat_map(move |(gene_index, _)| {
            [("minus", -step.clone()), ("plus", step.clone())]
                .into_iter()
                .map(move |(label, signed_step)| {
                    let mut genome = candidate.genome.clone();
                    genome.genes[gene_index] = genome.genes[gene_index].clone() + signed_step;
                    let id = CandidateId::new(format!(
                        "{}:hill:{gene_index}:{label}",
                        candidate.id.as_str()
                    ))
                    .expect("generated hill-climb id is non-empty");
                    Candidate {
                        id,
                        genome,
                        replay_policy: candidate.replay_policy.clone(),
                    }
                })
        })
}

fn one_step_neighbors(candidate: &Candidate, step: &Real) -> Vec<Candidate> {
    let mut neighbors = Vec::with_capacity(candidate.genome.genes.len() * 2);
    for (gene_index, _) in candidate.genome.genes.iter().enumerate() {
        for (label, signed_step) in [("minus", -step.clone()), ("plus", step.clone())] {
            let mut genome = candidate.genome.clone();
            genome.genes[gene_index] = genome.genes[gene_index].clone() + signed_step;
            let id = CandidateId::new(format!(
                "{}:hill:{gene_index}:{label}",
                candidate.id.as_str()
            ))
            .expect("generated hill-climb id is non-empty");
            neighbors.push(Candidate {
                id,
                genome,
                replay_policy: candidate.replay_policy.clone(),
            });
        }
    }
    neighbors
}

fn is_acceptable_neighbor(
    current: &FitnessReport,
    neighbor: &FitnessReport,
    direction: FitnessDirection,
    policy: &HillClimbPolicy,
) -> bool {
    if neighbor.replay != ReplayStatus::Accepted {
        return false;
    }
    match neighbor.value.compare_total(&current.value, direction) {
        FitnessComparison::Better => true,
        FitnessComparison::Equal => policy.accept_equal_plateau,
        FitnessComparison::Worse | FitnessComparison::Unknown => false,
    }
}

fn fitness_supports_total_order(value: &FitnessValue) -> bool {
    matches!(
        value,
        FitnessValue::Scalar(_) | FitnessValue::Lexicographic(_) | FitnessValue::Interval(_)
    )
}

fn recent_contains(recent: &[Genome], genome: &Genome) -> bool {
    recent.iter().any(|existing| existing == genome)
}

fn compare_genome_structure(left: &Genome, right: &Genome) -> DiversityRelation {
    if left.genes.len() != right.genes.len() {
        return DiversityRelation::ShapeMismatch {
            left: left.genes.len(),
            right: right.genes.len(),
        };
    }
    if left == right {
        DiversityRelation::Identical
    } else {
        DiversityRelation::Distinct
    }
}

fn select_best_from_indices<I>(
    candidates: &[Candidate],
    reports: &[FitnessReport],
    indices: I,
    direction: FitnessDirection,
) -> Result<SelectionReport, SelectionError>
where
    I: IntoIterator<Item = usize>,
{
    if candidates.len() != reports.len() {
        return Err(SelectionError::CandidateReportCountMismatch);
    }
    if candidates.is_empty() {
        return Err(SelectionError::EmptyPopulation);
    }
    let mut inspected = 0_usize;
    let mut selected: Option<usize> = None;
    for index in indices {
        if index >= candidates.len() {
            return Err(SelectionError::TournamentIndexOutOfBounds {
                index,
                candidate_count: candidates.len(),
            });
        }
        inspected += 1;
        if reports[index].candidate != candidates[index].id {
            return Err(SelectionError::CandidateReportIdMismatch { index });
        }
        if reports[index].replay != ReplayStatus::Accepted {
            continue;
        }
        let Some(current) = selected else {
            selected = Some(index);
            continue;
        };
        match reports[index]
            .value
            .compare_total(&reports[current].value, direction)
        {
            FitnessComparison::Better => selected = Some(index),
            FitnessComparison::Equal | FitnessComparison::Worse => {}
            FitnessComparison::Unknown => return Err(SelectionError::UnknownComparison),
        }
    }
    let Some(index) = selected else {
        return Err(SelectionError::NoAcceptedReplay);
    };
    Ok(SelectionReport {
        candidate: candidates[index].clone(),
        fitness: reports[index].clone(),
        inspected,
    })
}
