//! Diagnostic-only counts of existing coefficient vector shapes.
//!
//! No scalar queries, owners, timestamps, allocation hooks or dynamic maps.
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

const NAMES: [&str; 4] = ["rational", "radical", "opaque", "mixed"];
static ACTIVE: AtomicBool = AtomicBool::new(false);
static COUNTERS: [[AtomicU64; 5]; 16] =
    [const { [const { AtomicU64::new(0) }; 5] }; 16];

/// Calls and summed vector lengths for each left/right shape pair.
pub type Snapshot = [[u64; 5]; 16];

/// Prints the fixed counters on normal return or unwinding.
pub struct Report {
    finished: bool,
}

impl Report {
    /// Starts a diagnostic while no prior diagnostic or measured worker is active.
    pub fn begin() -> Self {
        assert!(!ACTIVE.load(Ordering::Acquire));
        for row in &COUNTERS {
            for value in row {
                value.store(0, Ordering::Relaxed);
            }
        }
        ACTIVE.store(true, Ordering::Release);
        Self { finished: false }
    }

    /// Ends the diagnostic after its workers have returned.
    pub fn finish(mut self) -> Snapshot {
        self.finished = true;
        finish()
    }
}

fn finish() -> Snapshot {
    assert!(ACTIVE.swap(false, Ordering::AcqRel));
    std::array::from_fn(|index| {
        std::array::from_fn(|field| COUNTERS[index][field].load(Ordering::Relaxed))
    })
}

impl Drop for Report {
    fn drop(&mut self) {
        if !self.finished {
            let snapshot = finish();
            for (index, row) in snapshot.into_iter().enumerate() {
                eprintln!(
                    "COEFFICIENT_SHAPE\t{}\t{}\t{}\t{}\t{}\t{}\t{}",
                    NAMES[index / 4], NAMES[index % 4], row[0], row[1], row[2], row[3], row[4],
                );
            }
        }
    }
}

fn shape(radicals: usize, atoms: usize) -> usize {
    usize::from(radicals != 0) + 2 * usize::from(atoms != 0)
}

/// Observes vector lengths already present at entry to coefficient multiplication.
pub fn record(left_radicals: usize, left_atoms: usize, right_radicals: usize, right_atoms: usize) {
    if !ACTIVE.load(Ordering::Acquire) {
        return;
    }
    let index = 4 * shape(left_radicals, left_atoms) + shape(right_radicals, right_atoms);
    for (counter, value) in COUNTERS[index].iter().zip([
        1, left_radicals as u64, left_atoms as u64, right_radicals as u64, right_atoms as u64,
    ]) {
        counter.fetch_add(value, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_pair_counts_calls_and_lengths_without_retaining_state() {
        let report = Report::begin();
        for left in 0..4 {
            for right in 0..4 {
                let lengths = [
                    usize::from(left & 1 != 0) * 3,
                    usize::from(left & 2 != 0) * 5,
                    usize::from(right & 1 != 0) * 7,
                    usize::from(right & 2 != 0) * 11,
                ];
                for _ in 0..(left * 4 + right + 1) {
                    record(lengths[0], lengths[1], lengths[2], lengths[3]);
                }
            }
        }
        let actual = report.finish();
        for (index, row) in actual.into_iter().enumerate() {
            let left = index / 4;
            let right = index % 4;
            let count = (index + 1) as u64;
            assert_eq!(row, [
                count,
                u64::from(left & 1 != 0) * 3 * count,
                u64::from(left & 2 != 0) * 5 * count,
                u64::from(right & 1 != 0) * 7 * count,
                u64::from(right & 2 != 0) * 11 * count,
            ]);
        }
        record(1, 2, 3, 4);
        assert_eq!(Report::begin().finish(), [[0; 5]; 16]);
    }

    #[test]
    fn joined_workers_contribute_to_the_same_fixed_counters() {
        let report = Report::begin();
        std::thread::scope(|scope| {
            for _ in 0..4 {
                scope.spawn(|| {
                    for _ in 0..1000 {
                        record(2, 0, 0, 0);
                    }
                });
            }
        });
        let actual = report.finish();
        assert_eq!(actual[4], [4000, 8000, 0, 0, 0]);
        assert_eq!(actual.iter().map(|row| row[0]).sum::<u64>(), 4000);
    }
}
