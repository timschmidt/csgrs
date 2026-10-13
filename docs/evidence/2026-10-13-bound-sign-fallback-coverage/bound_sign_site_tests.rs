use super::bound_sign_reference_tests::{affine, difference, raw, signed_bound};
use super::*;
use crate::bound_sign_diagnostic::Report;

#[test]
fn existing_affine_bound_tags_are_observed_without_bypassing_fallback() {
    for (integer, sign, result_index, bound_index) in
        [(3, Sign::Minus, 1, 3), (5, Sign::Plus, 3, 4)]
    {
        for (bound, tag) in [
            (BoundCache::Invalid, 0),
            (BoundCache::Valid(BoundInfo::Unknown), 1),
            (signed_bound(None), 5),
            (signed_bound(Some(sign)), bound_index),
        ] {
            let value = affine(integer, bound);
            let clone = value.clone();
            let report = Report::begin();
            assert_eq!(value.exact_sign(), Some(sign));
            assert_eq!(clone.exact_sign(), Some(sign));
            let counts = report.finish();
            let mut starts = [0; 7];
            starts[tag] = 1;
            let mut results = [[0; 4]; 7];
            results[tag][result_index] = 1;
            assert_eq!(counts.starts, starts);
            assert_eq!(counts.results, results);
            assert_eq!(counts.unfinished(), [0; 7]);
            assert!(value.internal.cache.pi_laurent_polynomial().is_some());
            assert_eq!(value.internal.facts.bound(), bound);
            assert_eq!(Arc::strong_count(&value.internal), 2);
            assert!(value.cached().is_none());
        }
    }
}

#[test]
fn certified_zero_still_runs_the_original_cancellation() {
    let root = Computable::sqrt_rational(Rational::new(13));
    let value = raw(
        Approximation::Add(
            root.clone(),
            raw(Approximation::Negate(root), BoundCache::Invalid),
        ),
        BoundCache::Valid(BoundInfo::Zero),
    );
    let clone = value.clone();
    let report = Report::begin();
    assert_eq!(value.exact_sign(), Some(Sign::NoSign));
    let counts = report.finish();
    assert_eq!(counts.starts, [0, 0, 1, 0, 0, 0, 0]);
    assert_eq!(counts.results[2], [0, 0, 1, 0]);
    assert_eq!(counts.unfinished(), [0; 7]);
    assert!(clone.internal.cache.pi_laurent_polynomial().is_some());
    assert!(value.cached().is_none());
}

#[test]
fn known_negative_bound_does_not_hide_the_original_decline() {
    let value = difference(signed_bound(Some(Sign::Minus)));
    let clone = value.clone();
    let report = Report::begin();
    assert_eq!(value.exact_sign(), None);
    assert_eq!(clone.exact_sign(), None);
    let counts = report.finish();
    assert_eq!(counts.starts, [0, 0, 0, 1, 0, 0, 0]);
    assert_eq!(counts.results[3], [1, 0, 0, 0]);
    assert_eq!(counts.unfinished(), [0; 7]);
    assert_eq!(value.immediate_sign(), Some(RealSign::Negative));
    assert!(value.internal.cache.pi_laurent_polynomial().is_some());
    assert!(value.cached().is_none());
}

#[test]
fn cached_sign_paths_do_not_count_a_fallback() {
    for (cached, expected) in [
        (ExactSignCache::Unknown, None),
        (ExactSignCache::Valid(Sign::Plus), Some(Sign::Plus)),
    ] {
        let value = affine(5, signed_bound(Some(Sign::Plus)));
        value.internal.facts.replace_exact_sign(cached);
        let report = Report::begin();
        assert_eq!(value.exact_sign(), expected);
        let counts = report.finish();
        assert_eq!(counts.starts, [0; 7]);
        assert_eq!(counts.results, [[0; 4]; 7]);
        assert!(value.cached().is_none());
    }
}

#[test]
fn inconsistent_nonzero_zero_sign_is_observed_without_reinterpretation() {
    // Deliberately malformed test metadata: the diagnostic must expose it,
    // execute the original arithmetic, and never substitute this fake sign.
    let value = affine(5, signed_bound(Some(Sign::NoSign)));
    let report = Report::begin();
    assert_eq!(value.exact_sign(), Some(Sign::Plus));
    let counts = report.finish();
    assert_eq!(counts.starts, [0, 0, 0, 0, 0, 0, 1]);
    assert_eq!(counts.results[6], [0, 0, 0, 1]);
    assert_eq!(counts.unfinished(), [0; 7]);
    assert!(value.cached().is_none());
}
