//! Diagnostic-only coefficient shape and executed linear-product counters.
//!
//! Inputs are vector lengths and stored-sign zero flags. No scalar owners,
//! approximation queries, timestamps, allocation hooks or dynamic maps are kept.
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

const SHAPES: [&str; 4] = ["rational", "radical", "opaque", "mixed"];
const TERMS: [&str; 4] = ["left_radical", "right_radical", "left_atom", "right_atom"];
static ACTIVE: AtomicBool = AtomicBool::new(false);
static ELIGIBLE: [[AtomicU64; 5]; 64] = [const { [const { AtomicU64::new(0) }; 5] }; 64];
static LINEAR: [[AtomicU64; 2]; 4] = [const { [const { AtomicU64::new(0) }; 2] }; 4];

/// The original multiplier's four linear contribution loops, in source order.
#[derive(Clone, Copy)]
pub enum LinearTerm {
    /// An owned-left radical scaled by the right rational constant.
    LeftRadical,
    /// A borrowed-right radical scaled by the left rational constant.
    RightRadical,
    /// An owned-left opaque atom scaled by the right rational constant.
    LeftAtom,
    /// A borrowed-right opaque atom scaled by the left rational constant.
    RightAtom,
}

/// Fixed counters collected after all measured workers have returned.
#[derive(Debug, PartialEq, Eq)]
pub struct Snapshot {
    /// Calls and four vector-length sums after the unsupported-product guard.
    /// Index is `(4 * left_shape + right_shape) * 4 + 2 * left_zero + right_zero`.
    /// Passing that guard does not guarantee later term insertion succeeds.
    pub eligible: [[u64; 5]; 64],
    /// Actual loop entries and entries with a zero scale, in [`LinearTerm`] order.
    pub linear: [[u64; 2]; 4],
}

/// Prints counters on normal return or unwinding after the original assertion.
pub struct Report {
    finished: bool,
}

impl Report {
    /// Starts while no prior diagnostic or measured worker is active.
    pub fn begin() -> Self {
        assert!(!ACTIVE.load(Ordering::Acquire));
        for row in &ELIGIBLE {
            for value in row {
                value.store(0, Ordering::Relaxed);
            }
        }
        for row in &LINEAR {
            for value in row {
                value.store(0, Ordering::Relaxed);
            }
        }
        ACTIVE.store(true, Ordering::Release);
        Self { finished: false }
    }

    /// Ends after its workers have returned, without printing.
    pub fn finish(mut self) -> Snapshot {
        self.finished = true;
        finish()
    }
}

fn snapshot() -> Snapshot {
    Snapshot {
        eligible: std::array::from_fn(|index| {
            std::array::from_fn(|field| ELIGIBLE[index][field].load(Ordering::Relaxed))
        }),
        linear: std::array::from_fn(|index| {
            std::array::from_fn(|field| LINEAR[index][field].load(Ordering::Relaxed))
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
            for (index, row) in counters.eligible.into_iter().enumerate() {
                let pair = index / 4;
                eprintln!(
                    "COEFFICIENT_ZERO_SHAPE\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}",
                    SHAPES[pair / 4],
                    SHAPES[pair % 4],
                    (index / 2) % 2,
                    index % 2,
                    row[0],
                    row[1],
                    row[2],
                    row[3],
                    row[4],
                );
            }
            for (index, row) in counters.linear.into_iter().enumerate() {
                eprintln!(
                    "COEFFICIENT_ZERO_TERM\t{}\t{}\t{}",
                    TERMS[index], row[0], row[1]
                );
            }
        }
    }
}

fn shape(radicals: usize, atoms: usize) -> usize {
    usize::from(radicals != 0) + 2 * usize::from(atoms != 0)
}

/// Observes inputs immediately after the original unsupported-product guard.
pub fn record_eligible(lengths: [usize; 4], zero: [bool; 2]) {
    if !ACTIVE.load(Ordering::Acquire) {
        return;
    }
    let pair = 4 * shape(lengths[0], lengths[1]) + shape(lengths[2], lengths[3]);
    let index = 4 * pair + 2 * usize::from(zero[0]) + usize::from(zero[1]);
    for (counter, value) in ELIGIBLE[index].iter().zip([
        1,
        lengths[0] as u64,
        lengths[1] as u64,
        lengths[2] as u64,
        lengths[3] as u64,
    ]) {
        counter.fetch_add(value, Ordering::Relaxed);
    }
}

/// Records one actual entry to an existing linear-term loop.
pub fn record_linear(term: LinearTerm, zero_scale: bool) {
    if !ACTIVE.load(Ordering::Acquire) {
        return;
    }
    let row = &LINEAR[term as usize];
    row[0].fetch_add(1, Ordering::Relaxed);
    row[1].fetch_add(u64::from(zero_scale), Ordering::Relaxed);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bins_loops_disabled_reset_and_unwind_are_observable() {
        let report = Report::begin();
        for left in 0..4 {
            for right in 0..4 {
                let lengths = [
                    usize::from(left & 1 != 0) * 3,
                    usize::from(left & 2 != 0) * 5,
                    usize::from(right & 1 != 0) * 7,
                    usize::from(right & 2 != 0) * 11,
                ];
                for flags in 0..4 {
                    let count = (4 * left + right) * 4 + flags + 1;
                    for _ in 0..count {
                        record_eligible(lengths, [flags & 2 != 0, flags & 1 != 0]);
                    }
                }
            }
        }
        for (index, term) in [
            LinearTerm::LeftRadical,
            LinearTerm::RightRadical,
            LinearTerm::LeftAtom,
            LinearTerm::RightAtom,
        ]
        .into_iter()
        .enumerate()
        {
            for iteration in 0..index + 3 {
                record_linear(term, iteration % 2 == 0);
            }
        }
        let actual = report.finish();
        for (index, row) in actual.eligible.into_iter().enumerate() {
            let pair = index / 4;
            let left = pair / 4;
            let right = pair % 4;
            let count = (index + 1) as u64;
            assert_eq!(
                row,
                [
                    count,
                    u64::from(left & 1 != 0) * 3 * count,
                    u64::from(left & 2 != 0) * 5 * count,
                    u64::from(right & 1 != 0) * 7 * count,
                    u64::from(right & 2 != 0) * 11 * count
                ]
            );
        }
        assert_eq!(actual.linear, [[3, 2], [4, 2], [5, 3], [6, 3]]);
        record_eligible([1, 2, 3, 4], [true, true]);
        record_linear(LinearTerm::LeftAtom, true);
        assert_eq!(snapshot(), actual, "disabled calls must not mutate counters");
        let empty = Snapshot {
            eligible: [[0; 5]; 64],
            linear: [[0; 2]; 4],
        };
        assert_eq!(Report::begin().finish(), empty);
        assert!(
            std::panic::catch_unwind(|| {
                let _report = Report::begin();
                record_linear(LinearTerm::RightAtom, true);
                panic!("intentional diagnostic calibration unwind");
            })
            .is_err()
        );
        assert!(!ACTIVE.load(Ordering::Acquire));
        assert_eq!(snapshot().linear, [[0, 0], [0, 0], [0, 0], [1, 1]]);
        assert_eq!(Report::begin().finish(), empty);
    }

    #[test]
    fn joined_workers_contribute_to_both_counter_sets() {
        let report = Report::begin();
        std::thread::scope(|scope| {
            for _ in 0..4 {
                scope.spawn(|| {
                    for _ in 0..1000 {
                        record_eligible([2, 0, 0, 0], [true, false]);
                        record_linear(LinearTerm::LeftRadical, false);
                        record_linear(LinearTerm::LeftRadical, false);
                    }
                });
            }
        });
        let actual = report.finish();
        assert_eq!(actual.eligible[18], [4000, 8000, 0, 0, 0]);
        assert_eq!(actual.eligible.iter().map(|row| row[0]).sum::<u64>(), 4000);
        assert_eq!(actual.linear, [[8000, 0], [0, 0], [0, 0], [0, 0]]);
    }
}
