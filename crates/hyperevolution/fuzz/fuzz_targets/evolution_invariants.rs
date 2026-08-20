//! Exact evolutionary carriers across every pair of Hyperreal representations.

#![no_main]

use hyperevolution::{
    Archive, Candidate, CandidateId, FitnessDirection, FitnessReport, FitnessValue, Genome,
    GpRealExpr, GpValidationLimits, Population, ReplayStatus, crossover_one_point,
    domain_replay_manifest, exact_structural_diversity, mutate_exact_delta, select_exact_best,
};
use hyperreal::{Rational, Real, StructuralKind};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let values = representative_values();
    let split = usize::from(data.first().copied().unwrap_or(0)) % 3;

    for (left_index, left) in values.iter().enumerate() {
        for (right_index, right) in values.iter().enumerate() {
            let add = GpRealExpr::Add(
                Box::new(GpRealExpr::Constant(Box::new(left.clone()))),
                Box::new(GpRealExpr::Constant(Box::new(right.clone()))),
            );
            let validation = add.validate(GpValidationLimits {
                input_arity: 0,
                max_depth: 3,
                max_nodes: 3,
            });
            assert!(validation.is_valid());
            assert_eq!(add.eval(&[]).expect("constant GP expression"), left + right);

            let left_candidate = candidate("left", left_index, left, right);
            let right_candidate = candidate("right", right_index, right, left);
            let left_report = report(&left_candidate, left.clone());
            let right_report = report(&right_candidate, right.clone());

            let _ = left_report
                .value
                .compare_total(&right_report.value, FitnessDirection::Minimize);
            let selected = select_exact_best(
                &[left_candidate.clone(), right_candidate.clone()],
                &[left_report.clone(), right_report.clone()],
                FitnessDirection::Minimize,
            );
            assert!(selected.is_ok() || values_are_not_ordered(left, right));

            let child = mutate_exact_delta(
                &left_candidate,
                right_index % 2,
                right.clone(),
                id("mutation", left_index * 8 + right_index),
            )
            .expect("two-gene genome");
            assert_eq!(child.genome.genes.len(), 2);

            let (first, second) = crossover_one_point(
                &left_candidate,
                &right_candidate,
                split,
                id("cross-a", left_index * 8 + right_index),
                id("cross-b", left_index * 8 + right_index),
            )
            .expect("equal arity and bounded split");
            assert_eq!(first.genome.genes.len(), 2);
            assert_eq!(second.genome.genes.len(), 2);

            let diversity =
                exact_structural_diversity(&[left_candidate.clone(), right_candidate.clone()]);
            assert_eq!(diversity.candidate_count, 2);
            assert_eq!(diversity.pair_count, 1);

            let mut population = Population::default();
            population.push(left_candidate.clone());
            population.push(right_candidate);
            assert_eq!(population.candidates.len(), 2);

            let mut archive = Archive::default();
            assert!(archive.insert_replayed(left_report));
            assert_eq!(archive.reports().len(), 1);

            let manifest = domain_replay_manifest(
                left_candidate.id.clone(),
                hyperevolution::DomainReplayTarget::Hyperphysics,
                format!("{left_index}-{right_index}"),
            );
            assert_eq!(manifest.candidate, left_candidate.id);
        }
    }
});

fn values_are_not_ordered(left: &Real, right: &Real) -> bool {
    FitnessValue::Scalar(Box::new(left.clone())).compare_total(
        &FitnessValue::Scalar(Box::new(right.clone())),
        FitnessDirection::Minimize,
    ) == hyperevolution::FitnessComparison::Unknown
}

fn candidate(prefix: &str, index: usize, first: &Real, second: &Real) -> Candidate {
    Candidate {
        id: id(prefix, index),
        genome: Genome {
            genes: vec![first.clone(), second.clone()],
        },
        proposal_seed: index as u64,
    }
}

fn report(candidate: &Candidate, value: Real) -> FitnessReport {
    FitnessReport {
        candidate: candidate.id.clone(),
        value: FitnessValue::Scalar(Box::new(value)),
        replay: ReplayStatus::Accepted,
        evidence: Vec::new(),
    }
}

fn id(prefix: &str, index: usize) -> CandidateId {
    CandidateId::new(format!("{prefix}-{index}")).expect("nonempty id")
}

fn representative_values() -> Vec<Real> {
    let pi_squared = &Real::pi() * &Real::pi();
    let values = vec![
        Real::new(Rational::fraction(3, 2).expect("valid rational")),
        Real::pi(),
        Real::e(),
        Real::new(Rational::new(2)).sqrt().expect("positive"),
        Real::new(Rational::new(3)).ln().expect("positive"),
        Real::new(Rational::fraction(1, 5).expect("valid rational")).sin_pi(),
        pi_squared * Real::e(),
        Real::new(Rational::one()).sin(),
    ];
    assert_eq!(
        values
            .iter()
            .map(|value| value.detailed_facts().symbolic.kind)
            .collect::<Vec<_>>(),
        vec![
            StructuralKind::ExactRational,
            StructuralKind::PiLike,
            StructuralKind::ExpLike,
            StructuralKind::SqrtLike,
            StructuralKind::LogLike,
            StructuralKind::TrigExact,
            StructuralKind::ProductConstant,
            StructuralKind::ComputableOpaque,
        ]
    );
    values
}
