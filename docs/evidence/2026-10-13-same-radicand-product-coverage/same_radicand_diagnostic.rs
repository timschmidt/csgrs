//! Diagnostic-only counts of existing single-radical multiplication branches.
//!
//! Observations contain vector lengths and existing comparison outcomes only.
//! No scalar owners, queries, arithmetic, callbacks or cache writes are added.
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

static ACTIVE: AtomicBool = AtomicBool::new(false);
static CALLS: AtomicU64 = AtomicU64::new(0);
static NARROW: AtomicU64 = AtomicU64::new(0);
static BRANCHES: [AtomicU64; 2] = [const { AtomicU64::new(0) }; 2];
static OUTPUTS: [[AtomicU64; 4]; 2] = [const { [const { AtomicU64::new(0) }; 4] }; 2];

/// Counts read after all observed workers have returned.
#[derive(Debug, PartialEq, Eq)]
pub struct Snapshot {
    /// All entries, including unsupported opaque products.
    pub calls: u64,
    /// Entries with one radical and no opaque atoms in each operand.
    pub narrow: u64,
    /// Actual existing radicand comparisons: unequal, then equal.
    pub branches: [u64; 2],
    /// Completed narrow products, unequal then equal, by output radical count.
    /// Columns mean zero, one, two, and three or more stored radical terms.
    pub outputs: [[u64; 4]; 2],
}

/// Prints observations after the original assertion, including unwinding.
pub struct Report {
    finished: bool,
}

impl Report {
    /// Begins while no other report or measured worker is active.
    pub fn begin() -> Self {
        assert!(!ACTIVE.load(Ordering::Acquire));
        CALLS.store(0, Ordering::Relaxed);
        NARROW.store(0, Ordering::Relaxed);
        for counter in &BRANCHES {
            counter.store(0, Ordering::Relaxed);
        }
        for row in &OUTPUTS {
            for counter in row {
                counter.store(0, Ordering::Relaxed);
            }
        }
        ACTIVE.store(true, Ordering::Release);
        Self { finished: false }
    }

    /// Ends after joined workers return, without printing.
    pub fn finish(mut self) -> Snapshot {
        self.finished = true;
        finish()
    }
}

fn snapshot() -> Snapshot {
    Snapshot {
        calls: CALLS.load(Ordering::Relaxed),
        narrow: NARROW.load(Ordering::Relaxed),
        branches: std::array::from_fn(|index| BRANCHES[index].load(Ordering::Relaxed)),
        outputs: std::array::from_fn(|index| {
            std::array::from_fn(|field| OUTPUTS[index][field].load(Ordering::Relaxed))
        }),
    }
}

fn finish() -> Snapshot {
    assert!(ACTIVE.swap(false, Ordering::AcqRel));
    snapshot()
}

impl Drop for Report {
    fn drop(&mut self) {
        if !self.finished {
            let counters = finish();
            eprintln!(
                "SAME_RADICAND_INPUT\t{}\t{}",
                counters.calls, counters.narrow
            );
            for index in 0..2 {
                let row = counters.outputs[index];
                eprintln!(
                    "SAME_RADICAND_RESULT\t{}\t{}\t{}\t{}\t{}\t{}",
                    index, counters.branches[index], row[0], row[1], row[2], row[3]
                );
            }
        }
    }
}

/// Counts entry shapes without inspecting values or changing the operands.
/// Returns whether both operands have exactly one radical and no opaque atoms.
pub fn record_inputs(lengths: [usize; 4]) -> bool {
    let narrow = lengths == [1, 0, 1, 0];
    if ACTIVE.load(Ordering::Acquire) {
        CALLS.fetch_add(1, Ordering::Relaxed);
        if narrow {
            NARROW.fetch_add(1, Ordering::Relaxed);
        }
    }
    narrow
}

/// Records the outcome inside the existing comparison branch of a narrow pair.
pub fn record_branch(equal: bool) {
    if ACTIVE.load(Ordering::Acquire) {
        BRANCHES[usize::from(equal)].fetch_add(1, Ordering::Relaxed);
    }
}

/// Records stored vector length at the unchanged successful return site.
pub fn record_result(equal: bool, radical_terms: usize) {
    if ACTIVE.load(Ordering::Acquire) {
        OUTPUTS[usize::from(equal)][radical_terms.min(3)].fetch_add(1, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bins_disabled_reset_and_unwind_are_observable() {
        let report = Report::begin();
        for equal in [false, true] {
            for terms in 0..5 {
                assert!(record_inputs([1, 0, 1, 0]));
                record_branch(equal);
                record_result(equal, terms);
            }
        }
        for lengths in [[0, 0, 0, 0], [1, 1, 1, 0], [2, 0, 1, 0]] {
            assert!(!record_inputs(lengths));
        }
        let actual = report.finish();
        assert_eq!(actual.calls, 13);
        assert_eq!(actual.narrow, 10);
        assert_eq!(actual.branches, [5, 5]);
        assert_eq!(actual.outputs, [[1, 1, 1, 2], [1, 1, 1, 2]]);
        assert!(record_inputs([1, 0, 1, 0]));
        record_branch(true);
        record_result(true, 1);
        assert_eq!(snapshot(), actual);
        let empty = Snapshot {
            calls: 0,
            narrow: 0,
            branches: [0; 2],
            outputs: [[0; 4]; 2],
        };
        assert_eq!(Report::begin().finish(), empty);
        assert!(
            std::panic::catch_unwind(|| {
                let _report = Report::begin();
                record_inputs([1, 0, 1, 0]);
                record_branch(true);
                panic!("intentional diagnostic calibration unwind");
            })
            .is_err()
        );
        assert!(!ACTIVE.load(Ordering::Acquire));
        assert_eq!(snapshot().branches, [0, 1]);
        assert_eq!(Report::begin().finish(), empty);
    }

    #[test]
    fn joined_workers_contribute_to_every_counter() {
        let report = Report::begin();
        std::thread::scope(|scope| {
            for worker in 0..4 {
                scope.spawn(move || {
                    for _ in 0..1000 {
                        assert!(record_inputs([1, 0, 1, 0]));
                        record_branch(worker % 2 == 0);
                        record_result(worker % 2 == 0, worker);
                    }
                });
            }
        });
        let actual = report.finish();
        assert_eq!(actual.calls, 4000);
        assert_eq!(actual.narrow, 4000);
        assert_eq!(actual.branches, [2000, 2000]);
        assert_eq!(actual.outputs, [[0, 1000, 0, 1000], [1000, 0, 1000, 0]]);
    }
}
