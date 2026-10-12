//! Diagnostic-only thread-local counters. No expression or certificate owners.
use std::cell::RefCell;

#[derive(Default)]
struct Counters {
    calls: [[u64; 5]; 4],
    first_nanos: [[u128; 5]; 4],
    retry_nanos: [[u128; 5]; 4],
    first_nodes: [[u64; 5]; 4],
}

impl Drop for Counters {
    fn drop(&mut self) {
        for (kind, label) in ["rational", "quadratic", "pi_multiple", "compact"]
            .into_iter()
            .enumerate()
        {
            if self.calls[kind].iter().any(|count| *count != 0) {
                eprintln!(
                    "LAURENT_CACHE_DIAGNOSTIC {{\"projection\":\"{label}\",\"calls\":{:?},\"first_nanos\":{:?},\"retry_nanos\":{:?},\"first_nodes\":{:?}}",
                    self.calls[kind], self.first_nanos[kind], self.retry_nanos[kind],
                    self.first_nodes[kind],
                );
            }
        }
    }
}

thread_local! {
    static COUNTERS: RefCell<Counters> = RefCell::new(Counters::default());
}

/// Outcomes: cold success, retained success, cold decline, retry success,
/// retry decline. Durations include timer/instrumentation perturbation and
/// describe only the recognizer/projection, not subsequent reconstruction.
pub(super) fn record(kind: usize, outcome: usize, first: u128, retry: u128, nodes: usize) {
    COUNTERS.with(|counters| {
        let mut counters = counters.borrow_mut();
        counters.calls[kind][outcome] += 1;
        counters.first_nanos[kind][outcome] += first;
        counters.retry_nanos[kind][outcome] += retry;
        counters.first_nodes[kind][outcome] += nodes as u64;
    });
}
