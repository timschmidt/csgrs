use std::hint::black_box;
use std::time::Instant;

use hyperevolution::{
    BlackBoxEvaluationReport, Candidate, CandidateId, EvaluationCacheKey, EvaluationCost,
    FitnessDirection, FitnessInterval, FitnessIntervalComparison, FitnessOracle, FitnessReport,
    FitnessValue, Genome, GpRealExpr, GpValidationLimits, HillClimbPolicy, Real, ReplayPolicy,
    ReplayStatus, crossover_one_point, evaluate_candidate_with_oracle, exact_structural_diversity,
    hill_climb_exact, mutate_exact_delta, select_exact_best,
};

struct BenchOracle;

impl FitnessOracle for BenchOracle {
    fn evaluate(&self, candidate: &Candidate) -> BlackBoxEvaluationReport {
        let x = candidate.genome.genes[0].clone();
        BlackBoxEvaluationReport {
            fitness: FitnessReport::scalar(
                candidate.id.clone(),
                x.clone() * x,
                ReplayStatus::Accepted,
            ),
            cost: EvaluationCost::one_call(),
            cache_key: Some(EvaluationCacheKey(format!(
                "bench:{}",
                candidate.id.as_str()
            ))),
            dependencies: Vec::new(),
            replay_hooks: Vec::new(),
            surrogate_stage: None,
        }
    }
}

fn main() {
    let left = FitnessReport {
        candidate: CandidateId::new("left").unwrap(),
        value: FitnessValue::Lexicographic(vec![Real::from(1), Real::from(2), Real::from(3)]),
        replay: ReplayStatus::Accepted,
        evidence: Vec::new(),
    };
    let right = FitnessReport {
        candidate: CandidateId::new("right").unwrap(),
        value: FitnessValue::Lexicographic(vec![Real::from(1), Real::from(2), Real::from(4)]),
        replay: ReplayStatus::Accepted,
        evidence: Vec::new(),
    };
    let iterations = 100_000_u32;
    let started = Instant::now();
    let mut better = 0_usize;
    let interval_left = FitnessInterval::new(Real::from(1), Real::from(2));
    let interval_right = FitnessInterval::new(Real::from(5), Real::from(8));
    let mut interval_better = 0_usize;
    let hill_policy = HillClimbPolicy::best_improvement(Real::from(1), 8);
    let hill_start = Candidate {
        id: CandidateId::new("hill").unwrap(),
        genome: Genome {
            genes: vec![Real::from(4)],
        },
        replay_policy: ReplayPolicy {
            seed: 0,
            require_exact_replay: true,
        },
    };
    let mut hill_steps = 0_usize;
    let selection_candidates = vec![
        hill_start.clone(),
        Candidate {
            id: CandidateId::new("select-2").unwrap(),
            genome: Genome {
                genes: vec![Real::from(2)],
            },
            replay_policy: hill_start.replay_policy.clone(),
        },
    ];
    let selection_reports = vec![
        FitnessReport::scalar(
            selection_candidates[0].id.clone(),
            Real::from(16),
            ReplayStatus::Accepted,
        ),
        FitnessReport::scalar(
            selection_candidates[1].id.clone(),
            Real::from(4),
            ReplayStatus::Accepted,
        ),
    ];
    let mut selected = 0_usize;
    let mut variation_genes = 0_usize;
    let mut distinct_pairs = 0_usize;
    let oracle = BenchOracle;
    let mut oracle_promoted = 0_usize;
    let gp_expr = GpRealExpr::Mul(
        Box::new(GpRealExpr::Input(0)),
        Box::new(GpRealExpr::Add(
            Box::new(GpRealExpr::Input(1)),
            Box::new(GpRealExpr::Constant(Box::new(Real::from(1)))),
        )),
    );
    let gp_limits = GpValidationLimits {
        input_arity: 2,
        max_depth: 4,
        max_nodes: 8,
    };
    let mut gp_nodes = 0_usize;

    for _ in 0..iterations {
        let comparison = black_box(&left)
            .value
            .compare_total(&right.value, FitnessDirection::Minimize);
        if matches!(comparison, hyperevolution::FitnessComparison::Better) {
            better += 1;
        }
        let interval_comparison = black_box(&interval_left)
            .compare(black_box(&interval_right), FitnessDirection::Minimize);
        if matches!(interval_comparison, FitnessIntervalComparison::Better) {
            interval_better += 1;
        }
        let hill_report = hill_climb_exact(
            black_box(hill_start.clone()),
            &hill_policy,
            FitnessDirection::Minimize,
            |candidate| {
                let x = candidate.genome.genes[0].clone();
                FitnessReport::scalar(candidate.id.clone(), x.clone() * x, ReplayStatus::Accepted)
            },
        );
        hill_steps += hill_report.accepted_steps;
        let selection = select_exact_best(
            &selection_candidates,
            &selection_reports,
            FitnessDirection::Minimize,
        )
        .unwrap();
        if selection.candidate.id.as_str() == "select-2" {
            selected += 1;
        }
        let mutated = mutate_exact_delta(
            black_box(&hill_start),
            0,
            Real::from(1),
            CandidateId::new("bench-mutated").unwrap(),
        )
        .unwrap();
        let (child, _) = crossover_one_point(
            black_box(&hill_start),
            black_box(&mutated),
            0,
            CandidateId::new("bench-child-a").unwrap(),
            CandidateId::new("bench-child-b").unwrap(),
        )
        .unwrap();
        variation_genes += child.genome.genes.len();
        distinct_pairs += exact_structural_diversity(&selection_candidates).distinct_pairs;
        let gp_report = gp_expr.validate(gp_limits);
        if gp_report.is_valid() {
            gp_nodes += gp_report.nodes;
        }
        if evaluate_candidate_with_oracle(&oracle, black_box(&hill_start)).is_promotable() {
            oracle_promoted += 1;
        }
    }

    let elapsed = started.elapsed();
    println!(
        "exact_lexicographic_interval_hill_evolution_gp_and_oracle_fitness: {iterations} iterations in {elapsed:?} ({:?}/iter), better={better}, interval_better={interval_better}, hill_steps={hill_steps}, selected={selected}, variation_genes={variation_genes}, distinct_pairs={distinct_pairs}, gp_nodes={gp_nodes}, oracle_promoted={oracle_promoted}",
        elapsed / iterations
    );
}
