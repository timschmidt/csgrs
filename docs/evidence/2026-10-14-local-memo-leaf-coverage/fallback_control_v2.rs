#[test]
fn diagnostic_memo_counts_cover_the_original_retry() {
    use crate::construction_cache_trace::{Event, with_recording as cache_recording};
    let (((), old), counts) = crate::memo_leaf_trace::with_recording(|| {
        cache_recording(expansion_at_a_term_limit_retries_the_original_proof)
    });
    assert_eq!(old[Event::ProjectRetryOriginal as usize], 1);
    assert!(old[Event::OriginalVisit as usize] > 0);
    let stores: u64 = counts
        .iter()
        .flat_map(|kind| kind.iter())
        .map(|mode| mode[0][0])
        .sum();
    let hits: u64 = counts
        .iter()
        .flat_map(|kind| kind.iter())
        .map(|mode| mode[1][0])
        .sum();
    assert_eq!(stores, old[Event::LocalMemoStore as usize]);
    assert_eq!(hits, old[Event::LocalMemoHit as usize]);
    assert!(counts.iter().any(|kind| kind[1][0][0] > 0));
    assert!(counts.iter().any(|kind| kind[1][1][0] > 0));
}
