use hyperevolution::{
    AnnealingAcceptance, Archive, BlackBoxEvaluationReport, Candidate, CandidateId,
    ConstructionDependency, DomainReplayReport, DomainReplayTarget, EvaluationCacheKey,
    EvaluationCost, FitnessComparison, FitnessDirection, FitnessInterval,
    FitnessIntervalComparison, FitnessOracle, FitnessReport, FitnessValue, Genome, GpRealExpr,
    GpValidationIssue, GpValidationLimits, HillClimbPolicy, HillClimbStopReason, ParetoRelation,
    Real, ReplayHook, ReplayPolicy, ReplayStatus, SelectionError, SimulatedAnnealingPolicy,
    SurrogateDecision, SurrogateScreenReport, SurrogateStage, VariationError,
    classify_simulated_annealing_neighbor, crossover_one_point, domain_replay_manifest,
    evaluate_candidate_with_oracle, exact_structural_diversity, hill_climb_exact,
    mutate_exact_delta, select_exact_best, select_tournament_by_indices,
    surrogate_allows_archive_promotion,
};
use proptest::prelude::*;

fn id(value: &str) -> CandidateId {
    CandidateId::new(value).unwrap()
}

fn candidate(name: &str, genes: Vec<Real>) -> Candidate {
    Candidate {
        id: id(name),
        genome: Genome { genes },
        replay_policy: ReplayPolicy {
            seed: 0,
            require_exact_replay: true,
        },
    }
}

fn square_fitness(candidate: &Candidate) -> FitnessReport {
    let x = candidate.genome.genes[0].clone();
    FitnessReport::scalar(candidate.id.clone(), x.clone() * x, ReplayStatus::Accepted)
}

struct SquareOracle;

impl FitnessOracle for SquareOracle {
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
                "square:{}",
                candidate.id.as_str()
            ))),
            dependencies: vec![ConstructionDependency("fixture-domain".into())],
            replay_hooks: vec![ReplayHook {
                domain: "fixture".into(),
                target: candidate.id.as_str().into(),
            }],
            surrogate_stage: None,
        }
    }
}

#[test]
fn exact_scalar_and_lexicographic_fitness_compare_without_float_surrogates() {
    let low = FitnessValue::Scalar(Box::new(Real::from(1)));
    let high = FitnessValue::Scalar(Box::new(Real::from(2)));
    assert_eq!(
        low.compare_total(&high, FitnessDirection::Minimize),
        FitnessComparison::Better
    );
    assert_eq!(
        low.compare_total(&high, FitnessDirection::Maximize),
        FitnessComparison::Worse
    );

    let left = FitnessValue::Lexicographic(vec![Real::from(1), Real::from(9)]);
    let right = FitnessValue::Lexicographic(vec![Real::from(1), Real::from(10)]);
    assert_eq!(
        left.compare_total(&right, FitnessDirection::Minimize),
        FitnessComparison::Better
    );
}

#[test]
fn pareto_archive_keeps_only_accepted_non_dominated_reports() {
    let mut archive = Archive::default();
    let dominated = FitnessReport {
        candidate: id("dominated"),
        value: FitnessValue::Pareto(vec![Real::from(5), Real::from(5)]),
        replay: ReplayStatus::Accepted,
        evidence: vec!["fixture".into()],
    };
    let dominant = FitnessReport {
        candidate: id("dominant"),
        value: FitnessValue::Pareto(vec![Real::from(3), Real::from(4)]),
        replay: ReplayStatus::Accepted,
        evidence: vec!["fixture".into()],
    };
    let rejected = FitnessReport {
        candidate: id("rejected"),
        value: FitnessValue::Pareto(vec![Real::from(1), Real::from(1)]),
        replay: ReplayStatus::Rejected,
        evidence: vec![],
    };

    assert!(archive.insert_non_dominated(dominated, FitnessDirection::Minimize));
    assert!(!archive.insert_non_dominated(rejected, FitnessDirection::Minimize));
    assert!(archive.insert_non_dominated(dominant, FitnessDirection::Minimize));
    assert_eq!(archive.reports().len(), 1);
    assert_eq!(archive.reports()[0].candidate.as_str(), "dominant");
}

#[test]
fn pareto_relation_reports_non_dominated_vectors() {
    let left = FitnessValue::Pareto(vec![Real::from(1), Real::from(5)]);
    let right = FitnessValue::Pareto(vec![Real::from(2), Real::from(4)]);
    assert_eq!(
        left.compare_pareto(&right, FitnessDirection::Minimize),
        ParetoRelation::NonDominated
    );
}

#[test]
fn interval_fitness_comparison_reports_overlap_and_invalid_bounds() {
    let certain_low = FitnessInterval::new(Real::from(1), Real::from(2));
    let certain_high = FitnessInterval::new(Real::from(5), Real::from(7));
    assert_eq!(
        certain_low.compare(&certain_high, FitnessDirection::Minimize),
        FitnessIntervalComparison::Better
    );
    assert_eq!(
        certain_low.compare(&certain_high, FitnessDirection::Maximize),
        FitnessIntervalComparison::Worse
    );

    let overlapping = FitnessInterval::new(Real::from(2), Real::from(6));
    assert_eq!(
        certain_high.compare(&overlapping, FitnessDirection::Minimize),
        FitnessIntervalComparison::Overlapping
    );
    assert_eq!(
        FitnessValue::Interval(Box::new(certain_high.clone())).compare_total(
            &FitnessValue::Interval(Box::new(overlapping)),
            FitnessDirection::Minimize
        ),
        FitnessComparison::Unknown
    );

    let invalid = FitnessInterval::new(Real::from(10), Real::from(9));
    assert_eq!(
        invalid.compare(&certain_high, FitnessDirection::Minimize),
        FitnessIntervalComparison::InvalidBounds
    );
}

#[test]
fn exact_hill_climb_reaches_integer_quadratic_local_optimum() {
    let policy = HillClimbPolicy::best_improvement(Real::from(1), 8);
    let report = hill_climb_exact(
        candidate("start", vec![Real::from(3)]),
        &policy,
        FitnessDirection::Minimize,
        square_fitness,
    );

    assert_eq!(report.reason, HillClimbStopReason::LocalOptimum);
    assert_eq!(report.accepted_steps, 3);
    assert_eq!(report.final_candidate.genome.genes, vec![Real::from(0)]);
    assert_eq!(
        report.final_fitness.value,
        FitnessValue::Scalar(Box::new(Real::from(0)))
    );
}

#[test]
fn exact_hill_climb_rejects_uncertified_neighbors() {
    let policy = HillClimbPolicy::first_improvement(Real::from(1), 4);
    let report = hill_climb_exact(
        candidate("start", vec![Real::from(1)]),
        &policy,
        FitnessDirection::Minimize,
        |candidate| {
            let x = candidate.genome.genes[0].clone();
            let replay = if x == Real::from(0) {
                ReplayStatus::Rejected
            } else {
                ReplayStatus::Accepted
            };
            FitnessReport::scalar(candidate.id.clone(), x, replay)
        },
    );

    assert_eq!(report.reason, HillClimbStopReason::LocalOptimum);
    assert_eq!(report.accepted_steps, 0);
    assert_eq!(report.final_candidate.genome.genes, vec![Real::from(1)]);
}

#[test]
fn simulated_annealing_acceptance_keeps_worse_moves_report_bearing() {
    let policy = SimulatedAnnealingPolicy {
        initial_temperature: Real::from(10),
        cooling_ratio: Real::from(1),
    };
    let current = FitnessReport::scalar(id("current"), Real::from(5), ReplayStatus::Accepted);
    let better = FitnessReport::scalar(id("better"), Real::from(4), ReplayStatus::Accepted);
    let worse = FitnessReport::scalar(id("worse"), Real::from(6), ReplayStatus::Accepted);
    let rejected = FitnessReport::scalar(id("rejected"), Real::from(4), ReplayStatus::Rejected);

    assert_eq!(
        classify_simulated_annealing_neighbor(
            &current,
            &better,
            &policy,
            FitnessDirection::Minimize
        ),
        AnnealingAcceptance::AcceptImprovement
    );
    assert_eq!(
        classify_simulated_annealing_neighbor(
            &current,
            &worse,
            &policy,
            FitnessDirection::Minimize
        ),
        AnnealingAcceptance::RequiresProbabilisticProposal
    );
    assert_eq!(
        classify_simulated_annealing_neighbor(
            &current,
            &rejected,
            &policy,
            FitnessDirection::Minimize
        ),
        AnnealingAcceptance::RejectUncertifiedReplay
    );

    let invalid = SimulatedAnnealingPolicy {
        initial_temperature: Real::from(0),
        cooling_ratio: Real::from(1),
    };
    assert_eq!(
        classify_simulated_annealing_neighbor(
            &current,
            &better,
            &invalid,
            FitnessDirection::Minimize
        ),
        AnnealingAcceptance::InvalidSchedule
    );
}

#[test]
fn exact_selection_uses_replay_gated_fitness_without_float_fallback() {
    let candidates = vec![
        candidate("a", vec![Real::from(0)]),
        candidate("b", vec![Real::from(0)]),
        candidate("c", vec![Real::from(0)]),
    ];
    let reports = vec![
        FitnessReport::scalar(id("a"), Real::from(7), ReplayStatus::Accepted),
        FitnessReport::scalar(id("b"), Real::from(1), ReplayStatus::Rejected),
        FitnessReport::scalar(id("c"), Real::from(3), ReplayStatus::Accepted),
    ];

    let selected = select_exact_best(&candidates, &reports, FitnessDirection::Minimize).unwrap();
    assert_eq!(selected.candidate.id.as_str(), "c");
    assert_eq!(selected.inspected, 3);

    let tournament =
        select_tournament_by_indices(&candidates, &reports, &[0, 2], FitnessDirection::Minimize)
            .unwrap();
    assert_eq!(tournament.candidate.id.as_str(), "c");

    let bad = select_tournament_by_indices(&candidates, &reports, &[9], FitnessDirection::Minimize)
        .unwrap_err();
    assert_eq!(
        bad,
        SelectionError::TournamentIndexOutOfBounds {
            index: 9,
            candidate_count: 3
        }
    );
}

#[test]
fn exact_mutation_and_one_point_crossover_preserve_genome_shape() {
    let left = candidate("left", vec![Real::from(1), Real::from(2), Real::from(3)]);
    let right = candidate("right", vec![Real::from(8), Real::from(9), Real::from(10)]);

    let mutated = mutate_exact_delta(&left, 1, Real::from(5), id("mutated")).unwrap();
    assert_eq!(
        mutated.genome.genes,
        vec![Real::from(1), Real::from(7), Real::from(3)]
    );
    assert_eq!(
        mutate_exact_delta(&left, 4, Real::from(1), id("bad")).unwrap_err(),
        VariationError::GeneIndexOutOfBounds { index: 4, len: 3 }
    );

    let (child_a, child_b) =
        crossover_one_point(&left, &right, 2, id("child-a"), id("child-b")).unwrap();
    assert_eq!(
        child_a.genome.genes,
        vec![Real::from(1), Real::from(2), Real::from(10)]
    );
    assert_eq!(
        child_b.genome.genes,
        vec![Real::from(8), Real::from(9), Real::from(3)]
    );

    let short = candidate("short", vec![Real::from(1)]);
    assert_eq!(
        crossover_one_point(&left, &short, 1, id("x"), id("y")).unwrap_err(),
        VariationError::GenomeLengthMismatch { left: 3, right: 1 }
    );
}

#[test]
fn exact_structural_diversity_counts_equal_distinct_and_shape_mismatch_pairs() {
    let report = exact_structural_diversity(&[
        candidate("a", vec![Real::from(1), Real::from(2)]),
        candidate("b", vec![Real::from(1), Real::from(2)]),
        candidate("c", vec![Real::from(1), Real::from(3)]),
        candidate("d", vec![Real::from(1)]),
    ]);

    assert_eq!(report.candidate_count, 4);
    assert_eq!(report.pair_count, 6);
    assert_eq!(report.identical_pairs, 1);
    assert_eq!(report.distinct_pairs, 2);
    assert_eq!(report.shape_mismatch_pairs, 3);
}

#[test]
fn gp_real_expr_validates_arity_bloat_and_zero_division_before_eval() {
    let expression = GpRealExpr::Div(
        Box::new(GpRealExpr::Add(
            Box::new(GpRealExpr::Input(2)),
            Box::new(GpRealExpr::Constant(Box::new(Real::from(1)))),
        )),
        Box::new(GpRealExpr::Constant(Box::new(Real::zero()))),
    );

    let report = expression.validate(GpValidationLimits {
        input_arity: 2,
        max_depth: 2,
        max_nodes: 4,
    });

    assert!(!report.is_valid());
    assert!(
        report
            .issues
            .contains(&GpValidationIssue::InputOutOfBounds { input: 2, arity: 2 })
    );
    assert!(report.issues.contains(&GpValidationIssue::DepthExceeded {
        depth: 3,
        max_depth: 2
    }));
    assert!(
        report
            .issues
            .contains(&GpValidationIssue::NodeBudgetExceeded {
                nodes: 5,
                max_nodes: 4
            })
    );
    assert!(report.issues.contains(&GpValidationIssue::DivisionByZero));
}

#[test]
fn gp_real_expr_evaluates_exactly_after_validation() {
    let expression = GpRealExpr::Mul(
        Box::new(GpRealExpr::Add(
            Box::new(GpRealExpr::Input(0)),
            Box::new(GpRealExpr::Constant(Box::new(Real::from(2)))),
        )),
        Box::new(GpRealExpr::Input(1)),
    );
    let report = expression.validate(GpValidationLimits {
        input_arity: 2,
        max_depth: 4,
        max_nodes: 7,
    });

    assert!(report.is_valid());
    assert_eq!(
        expression.eval(&[Real::from(3), Real::from(4)]).unwrap(),
        Real::from(20)
    );
}

#[test]
fn black_box_oracle_reports_cache_cost_dependencies_and_replay_hooks() {
    let candidate = candidate("oracle", vec![Real::from(4)]);
    let report = evaluate_candidate_with_oracle(&SquareOracle, &candidate);

    assert!(report.is_promotable());
    assert!(!report.requires_replay());
    assert_eq!(report.cost, EvaluationCost::one_call());
    assert_eq!(
        report.cache_key,
        Some(EvaluationCacheKey("square:oracle".into()))
    );
    assert_eq!(
        report.dependencies,
        vec![ConstructionDependency("fixture-domain".into())]
    );
    assert_eq!(report.replay_hooks[0].domain, "fixture");
    assert_eq!(
        report.fitness.value,
        FitnessValue::Scalar(Box::new(Real::from(16)))
    );
}

#[test]
fn surrogate_screening_is_never_archive_promotion_by_itself() {
    let report = SurrogateScreenReport {
        candidate: "candidate".into(),
        stage: SurrogateStage {
            name: "cheap-float-screen".into(),
            lossy: true,
        },
        decision: SurrogateDecision::PromoteToReplay,
        cache_key: Some(EvaluationCacheKey("surrogate:candidate".into())),
    };

    assert!(!surrogate_allows_archive_promotion(&report));
}

#[test]
fn domain_replay_manifest_names_hyper_owner_without_taking_domain_truth() {
    let manifest = domain_replay_manifest(
        id("seed"),
        DomainReplayTarget::Hypersolve,
        "residual-block-7",
    );

    assert_eq!(manifest.hook.domain, "hypersolve");
    assert_eq!(manifest.hook.target, "residual-block-7");
    assert_eq!(
        manifest.cache_key,
        Some(EvaluationCacheKey("domain-replay:residual-block-7".into()))
    );

    let report = DomainReplayReport {
        manifest,
        status: ReplayStatus::Unknown,
        evidence: vec!["interval proof pending".into()],
    };
    assert!(!report.is_accepted());
    assert!(report.needs_followup());
}

proptest! {
    #[test]
    fn empty_candidate_ids_are_rejected(id in "\\PC*") {
        if id.is_empty() {
            prop_assert!(CandidateId::new(id).is_err());
        } else {
            prop_assert!(CandidateId::new(id).is_ok());
        }
    }

    #[test]
    fn generated_disjoint_interval_ordering_is_exact(
        left_low in -100_i16..=100,
        left_width in 0_i16..=20,
        gap in 1_i16..=20,
        right_width in 0_i16..=20,
    ) {
        let left_high = left_low + left_width;
        let right_low = left_high + gap;
        let right_high = right_low + right_width;
        let left = FitnessInterval::new(
            Real::from(i64::from(left_low)),
            Real::from(i64::from(left_high)),
        );
        let right = FitnessInterval::new(
            Real::from(i64::from(right_low)),
            Real::from(i64::from(right_high)),
        );

        prop_assert_eq!(
            left.compare(&right, FitnessDirection::Minimize),
            FitnessIntervalComparison::Better
        );
        prop_assert_eq!(
            left.compare(&right, FitnessDirection::Maximize),
            FitnessIntervalComparison::Worse
        );
    }

    #[test]
    fn generated_integer_square_hill_climb_reaches_zero(start in -20_i16..=20) {
        let policy = HillClimbPolicy::best_improvement(Real::from(1), usize::from(start.unsigned_abs()) + 2);
        let report = hill_climb_exact(
            candidate("generated", vec![Real::from(i64::from(start))]),
            &policy,
            FitnessDirection::Minimize,
            square_fitness,
        );

        prop_assert_eq!(report.reason, HillClimbStopReason::LocalOptimum);
        prop_assert_eq!(report.final_candidate.genome.genes, vec![Real::from(0)]);
    }

    #[test]
    fn generated_exact_delta_mutation_changes_only_one_gene(
        a in -20_i16..=20,
        b in -20_i16..=20,
        index in 0_usize..2,
        delta in -10_i16..=10,
    ) {
        let base = candidate("base", vec![Real::from(i64::from(a)), Real::from(i64::from(b))]);
        let mutated = mutate_exact_delta(
            &base,
            index,
            Real::from(i64::from(delta)),
            id("generated-mutated"),
        ).unwrap();

        for gene_index in 0..2 {
            let expected = if gene_index == index {
                base.genome.genes[gene_index].clone() + Real::from(i64::from(delta))
            } else {
                base.genome.genes[gene_index].clone()
            };
            prop_assert_eq!(mutated.genome.genes[gene_index].clone(), expected);
        }
    }

    #[test]
    fn generated_oracle_cache_keys_replay_candidate_ids(value in -20_i16..=20) {
        let candidate = candidate(&format!("oracle-{value}"), vec![Real::from(i64::from(value))]);
        let report = evaluate_candidate_with_oracle(&SquareOracle, &candidate);

        prop_assert_eq!(
            report.cache_key.clone(),
            Some(EvaluationCacheKey(format!("square:oracle-{value}")))
        );
        prop_assert_eq!(report.replay_hooks[0].target.as_str(), candidate.id.as_str());
        prop_assert!(report.is_promotable());
    }
}
