use hyperevolution::{
    Archive, CandidateId, FitnessComparison, FitnessDirection, FitnessReport, FitnessValue,
    ParetoRelation, Real, ReplayStatus,
};
use proptest::prelude::*;

fn id(value: &str) -> CandidateId {
    CandidateId::new(value).unwrap()
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

proptest! {
    #[test]
    fn empty_candidate_ids_are_rejected(id in "\\PC*") {
        if id.is_empty() {
            prop_assert!(CandidateId::new(id).is_err());
        } else {
            prop_assert!(CandidateId::new(id).is_ok());
        }
    }
}
