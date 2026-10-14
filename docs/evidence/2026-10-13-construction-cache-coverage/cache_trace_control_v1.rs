#[test]
fn diagnostic_cache_counters_observe_publication_and_zero_work_replay() {
    use crate::construction_cache_trace::{Event, with_recording};
    let value = raw(Approximation::Add(
        Computable::rational(Rational::new(3)),
        Computable::rational(Rational::new(5)),
    ));
    let clone = value.clone();
    let (first, cold) = with_recording(|| retained_query(&value, 8, 512));
    assert_eq!(first.0.and_then(rational), Some(Rational::new(8)));
    assert!(!first.2);
    assert!(cold[Event::RetainedLookup as usize] > 0);
    assert_eq!(cold[Event::RetainedHit as usize], 0);
    assert_eq!(cold[Event::ClosedCopySuccess as usize], 1);
    assert_eq!(cold[Event::StorePublished as usize], 1);
    assert_eq!(cold[Event::EvaluationCellAllocated as usize], 1);
    assert_eq!(cold[Event::EvaluationCellPublished as usize], 1);
    assert_eq!(cold[Event::EvaluationCellRaceLost as usize], 0);
    assert_eq!(
        cold[Event::EvaluationCellRequestedBytes as usize],
        std::mem::size_of::<std::sync::RwLock<CachedEvaluation>>() as u64
    );
    let (warm, replay) = with_recording(|| retained_query(&clone, 0, 0));
    assert_eq!(warm.0.and_then(rational), Some(Rational::new(8)));
    assert_eq!(warm.1, 0);
    assert!(warm.2);
    assert_eq!(replay[Event::RetainedLookup as usize], 1);
    assert_eq!(replay[Event::RetainedHit as usize], 1);
    assert_eq!(replay[Event::RetainedTermsCopied as usize], 1);
    assert_eq!(replay[Event::RetainedRadicalsCopied as usize], 0);
    assert_eq!(replay[Event::StoreAttempt as usize], 0);
    assert_eq!(replay[Event::EvaluationCellAllocated as usize], 0);
    assert_eq!(replay[Event::NormalizerWorkVisit as usize], 0);
    assert_eq!(replay[Event::NormalizerBudgetDecline as usize], 0);
}
