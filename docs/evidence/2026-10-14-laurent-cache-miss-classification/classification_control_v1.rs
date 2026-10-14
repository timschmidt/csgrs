#[test]
fn diagnostic_lookup_classes_observe_actual_cache_states() {
    use crate::construction_cache_trace::{
        Event, LOOKUP_KIND_NAMES, lookup_counts, with_recording,
    };

    fn expect_classes(expected: &[(&str, [u64; 3])]) {
        let mut counts = [[0; 3]; LOOKUP_KIND_NAMES.len()];
        for (name, outcomes) in expected {
            let index = LOOKUP_KIND_NAMES
                .iter()
                .position(|kind| kind == name)
                .unwrap();
            counts[index] = *outcomes;
        }
        assert_eq!(lookup_counts(), counts);
    }

    let leaf = raw(Approximation::Ratio(Rational::new(7)));
    let (direct, counts) = with_recording(|| leaf.internal.cache.pi_laurent_polynomial());
    assert!(direct.is_none());
    assert_eq!(counts[Event::RetainedLookup as usize], 1);
    expect_classes(&[("Unclassified", [1, 0, 0])]);

    let (empty, counts) = with_recording(|| retained_query(&leaf, 0, 0));
    assert!(empty.0.is_none());
    assert_eq!(counts[Event::RetainedLookup as usize], 1);
    assert_eq!(counts[Event::NormalizerBudgetDecline as usize], 1);
    expect_classes(&[("Ratio", [1, 0, 0])]);

    leaf.internal.cache.store(-32, leaf.approx(-32));
    let cached = leaf.cached();
    assert!(cached.is_some());
    let (initialized, counts) = with_recording(|| retained_query(&leaf, 0, 0));
    assert!(initialized.0.is_none());
    assert_eq!(counts[Event::RetainedLookup as usize], 1);
    assert_eq!(counts[Event::NormalizerBudgetDecline as usize], 1);
    assert_eq!(leaf.cached(), cached);
    expect_classes(&[("Ratio", [0, 1, 0])]);

    let value = raw(Approximation::Add(
        raw(Approximation::Ratio(Rational::new(3))),
        raw(Approximation::Ratio(Rational::new(5))),
    ));
    let clone = value.clone();
    let weak = Arc::downgrade(&value.internal);
    let (empty, counts) = with_recording(|| retained_query(&value, 0, 0));
    assert!(empty.0.is_none());
    assert_eq!(counts[Event::RetainedLookup as usize], 1);
    expect_classes(&[("Add", [1, 0, 0])]);

    value
        .internal
        .cache
        .store_quadratic_tower(&Tower::rational(Rational::new(8)));
    let (initialized, counts) = with_recording(|| retained_query(&value, 0, 0));
    assert!(initialized.0.is_none());
    assert_eq!(counts[Event::RetainedLookup as usize], 1);
    expect_classes(&[("Add", [0, 1, 0])]);

    let (proof, counts) = with_recording(|| retained_query(&value, 64, 512));
    assert_eq!(proof.0.and_then(rational), Some(Rational::new(8)));
    assert!(!proof.2);
    assert_eq!(counts[Event::RetainedLookup as usize], 3);
    assert_eq!(counts[Event::RetainedHit as usize], 0);
    assert_eq!(counts[Event::StorePublished as usize], 1);
    assert_eq!(counts[Event::EvaluationCellAllocated as usize], 0);
    expect_classes(&[("Add", [0, 1, 0]), ("Ratio", [2, 0, 0])]);

    drop(clone);
    assert_eq!(Arc::strong_count(&value.internal), 1);
    let (proof, counts) = with_recording(|| retained_query(&value, 0, 0));
    assert_eq!(proof.0.and_then(rational), Some(Rational::new(8)));
    assert_eq!(proof.1, 0);
    assert!(proof.2);
    assert_eq!(counts[Event::RetainedLookup as usize], 1);
    assert_eq!(counts[Event::RetainedHit as usize], 1);
    assert_eq!(counts[Event::NormalizerWorkVisit as usize], 0);
    assert_eq!(counts[Event::NormalizerBudgetDecline as usize], 0);
    expect_classes(&[("Add", [0, 0, 1])]);
    assert!(value.cached().is_none());
    assert_eq!(
        value
            .internal
            .cache
            .quadratic_tower()
            .unwrap()
            .even
            .rational,
        Rational::new(8)
    );
    drop(value);
    assert!(weak.upgrade().is_none());
}
