use hyperevolution::{
    Archive, Candidate, CandidateId, FitnessComparison, FitnessDirection, FitnessReport, Genome,
    Real, ReplayStatus, mutate_exact_delta,
};

fn main() {
    let seed = Candidate {
        id: CandidateId::new("seed").expect("non-empty id"),
        genome: Genome {
            genes: vec![Real::from(2), Real::from(3)],
        },
        proposal_seed: 42,
    };
    let child = mutate_exact_delta(
        &seed,
        0,
        Real::from(-1),
        CandidateId::new("child").expect("non-empty id"),
    )
    .expect("gene 0 exists");

    let before = FitnessReport::scalar(seed.id, Real::from(4), ReplayStatus::Accepted);
    let after = FitnessReport::scalar(child.id, Real::from(1), ReplayStatus::Accepted);
    assert_eq!(
        after
            .value
            .compare_total(&before.value, FitnessDirection::Minimize),
        FitnessComparison::Better,
    );

    let mut archive = Archive::default();
    assert!(archive.insert_replayed(after));
}
