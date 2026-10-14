//! Diagnostic-only counters in an isolated source snapshot.

use std::cell::Cell;

#[derive(Clone, Copy)]
pub(crate) enum Event {
    ProjectEntry,
    ProjectRetainedSuccess,
    ProjectRetryOriginal,
    ProjectDeclineWithoutReuse,
    RetainedVisit,
    OriginalVisit,
    LocalMemoHit,
    LocalMemoStore,
    RetainedLookup,
    RetainedHit,
    RetainedTermsCopied,
    RetainedRadicalsCopied,
    StoreAttempt,
    ClosedCopyAttempt,
    ClosedCopyDeclinedOpaque,
    ClosedCopySuccess,
    ClosedTermsCopied,
    ClosedRadicalsCopied,
    StorePublished,
    StoreAlreadyPresent,
    EvaluationCellAllocated,
    EvaluationCellRequestedBytes,
    EvaluationCellPublished,
    EvaluationCellRaceLost,
    ClosedBoxRequestedBytes,
    NormalizerWorkVisit,
    NormalizerBudgetDecline,
}

pub const EVENT_NAMES: [&str; 27] = [
    "project_entry",
    "project_retained_success",
    "project_retry_original",
    "project_decline_without_reuse",
    "retained_visit",
    "original_visit",
    "local_memo_hit",
    "local_memo_store",
    "retained_lookup",
    "retained_hit",
    "retained_terms_copied",
    "retained_radicals_copied",
    "store_attempt",
    "closed_copy_attempt",
    "closed_copy_declined_opaque",
    "closed_copy_success",
    "closed_terms_copied",
    "closed_radicals_copied",
    "store_published",
    "store_already_present",
    "evaluation_cell_allocated",
    "evaluation_cell_requested_bytes",
    "evaluation_cell_published",
    "evaluation_cell_race_lost",
    "closed_box_requested_bytes",
    "normalizer_work_visit",
    "normalizer_budget_decline",
];

thread_local! {
    static ACTIVE: Cell<bool> = const { Cell::new(false) };
    static COUNTS: Cell<[u64; EVENT_NAMES.len()]> = const { Cell::new([0; EVENT_NAMES.len()]) };
}

pub(crate) fn record(event: Event, amount: usize) {
    if !ACTIVE.with(Cell::get) {
        return;
    }
    COUNTS.with(|cell| {
        let mut counts = cell.get();
        counts[event as usize] += amount as u64;
        cell.set(counts);
    });
}

pub fn with_recording<T>(body: impl FnOnce() -> T) -> (T, [u64; EVENT_NAMES.len()]) {
    assert!(
        !ACTIVE.with(|cell| cell.replace(true)),
        "nested diagnostic scope"
    );
    COUNTS.with(|cell| cell.set([0; EVENT_NAMES.len()]));
    struct Finish;
    impl Drop for Finish {
        fn drop(&mut self) {
            ACTIVE.with(|cell| cell.set(false));
        }
    }
    let guard = Finish;
    let result = body();
    let counts = COUNTS.with(Cell::get);
    drop(guard);
    (result, counts)
}
