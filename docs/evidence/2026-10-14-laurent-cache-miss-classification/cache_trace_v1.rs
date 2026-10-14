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

pub const LOOKUP_KIND_NAMES: [&str; 61] = [
    "Int",
    "One",
    "Constant",
    "Inverse",
    "Negate",
    "Add",
    "Multiply",
    "LinearCombination3",
    "Square",
    "Ratio",
    "Offset",
    "PrescaledExp",
    "Expm1",
    "Sqrt",
    "PrescaledLn",
    "PrescaledLnRational",
    "BinaryScaledLnRational",
    "IntegralAtan",
    "PrescaledAtan",
    "AtanDeferred",
    "AtanRational",
    "AsinRational",
    "PrescaledAsin",
    "AsinDeferred",
    "AcosPositive",
    "AcosPositiveRational",
    "AcosNegativeRational",
    "AcoshNearOne",
    "AcoshDirect",
    "AsinhNearZero",
    "AsinhDirect",
    "PrescaledAsinh",
    "AsinhRational",
    "AtanhDirect",
    "PrescaledAtanh",
    "AtanhRational",
    "PrescaledCos",
    "PrescaledCosRational",
    "CosLargeRational",
    "PrescaledCosHalfPiMinusRational",
    "PrescaledSin",
    "PrescaledSinRational",
    "SinLargeRational",
    "PrescaledSinHalfPiMinusRational",
    "PrescaledCotHalfPiMinusRational",
    "TanLargeRational",
    "PrescaledTan",
    "PrescaledTanRational",
    "PrescaledCot",
    "ErfSeries",
    "Erfc",
    "NormalSf",
    "NormalInterval",
    "LogPnorm",
    "LogNormalSf",
    "LogDnorm",
    "NormalQuantile",
    "NthRoot",
    "SincSmall",
    "CoscSmall",
    "Unclassified",
];

pub const LOOKUP_OUTCOME_NAMES: [&str; 3] = ["empty_cell", "initialized_miss", "hit"];

#[derive(Clone, Copy)]
pub(crate) enum LookupOutcome {
    EmptyCell,
    InitializedMiss,
    Hit,
}

thread_local! {
    static LOOKUP_KIND: Cell<usize> = const { Cell::new(LOOKUP_KIND_NAMES.len() - 1) };
    static LOOKUPS: [Cell<[u64; 3]>; LOOKUP_KIND_NAMES.len()] =
        const { [const { Cell::new([0; 3]) }; LOOKUP_KIND_NAMES.len()] };
    static ACTIVE: Cell<bool> = const { Cell::new(false) };
    static COUNTS: Cell<[u64; EVENT_NAMES.len()]> = const { Cell::new([0; EVENT_NAMES.len()]) };
}

pub(crate) fn set_lookup_kind(kind: usize) {
    if ACTIVE.with(Cell::get) {
        LOOKUP_KIND.with(|cell| cell.set(kind));
    }
}

pub(crate) fn record_lookup_outcome(outcome: LookupOutcome) {
    if !ACTIVE.with(Cell::get) {
        return;
    }
    let kind = LOOKUP_KIND.with(|cell| cell.replace(LOOKUP_KIND_NAMES.len() - 1));
    LOOKUPS.with(|cells| {
        let mut counts = cells[kind].get();
        counts[outcome as usize] += 1;
        cells[kind].set(counts);
    });
}

pub fn lookup_counts() -> [[u64; 3]; LOOKUP_KIND_NAMES.len()] {
    LOOKUPS.with(|cells| std::array::from_fn(|index| cells[index].get()))
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
    LOOKUP_KIND.with(|cell| cell.set(LOOKUP_KIND_NAMES.len() - 1));
    LOOKUPS.with(|cells| {
        for cell in cells {
            cell.set([0; 3]);
        }
    });
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
