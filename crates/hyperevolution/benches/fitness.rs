use std::hint::black_box;
use std::time::Instant;

use hyperevolution::{
    CandidateId, FitnessDirection, FitnessReport, FitnessValue, Real, ReplayStatus,
};

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

    for _ in 0..iterations {
        let comparison = black_box(&left)
            .value
            .compare_total(&right.value, FitnessDirection::Minimize);
        if matches!(comparison, hyperevolution::FitnessComparison::Better) {
            better += 1;
        }
    }

    let elapsed = started.elapsed();
    println!(
        "exact_lexicographic_fitness: {iterations} iterations in {elapsed:?} ({:?}/iter), better={better}",
        elapsed / iterations
    );
}
