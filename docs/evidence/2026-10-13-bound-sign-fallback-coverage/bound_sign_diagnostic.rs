//! Diagnostic-only counts at the unchanged exact-sign quadratic fallback.
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

static ACTIVE: AtomicBool = AtomicBool::new(false);
static STARTS: [AtomicU64; 7] = [const { AtomicU64::new(0) }; 7];
static RESULTS: [[AtomicU64; 4]; 7] = [const { [const { AtomicU64::new(0) }; 4] }; 7];

/// Existing bound tag, with no magnitude or scalar owners retained.
#[derive(Clone, Copy)]
pub enum BoundTag {
    /// No stored bound.
    Invalid,
    /// Stored unknown bound.
    Unknown,
    /// Certified zero.
    Zero,
    /// Certified negative sign.
    Negative,
    /// Certified positive sign.
    Positive,
    /// Nonzero without a sign.
    Unsigned,
    /// Representable but inconsistent nonzero/zero-sign metadata.
    NonzeroZeroSign,
}

/// Result of the original quadratic sign computation.
#[derive(Clone, Copy)]
pub enum Outcome {
    /// Original fallback declined.
    Unknown,
    /// Original fallback proved a negative value.
    Negative,
    /// Original fallback proved zero.
    Zero,
    /// Original fallback proved a positive value.
    Positive,
}

/// Counts read after observed workers have returned.
#[derive(Debug, PartialEq, Eq)]
pub struct Snapshot {
    /// Started fallbacks in BoundTag order.
    pub starts: [u64; 7],
    /// Completed fallbacks in BoundTag then Outcome order.
    pub results: [[u64; 4]; 7],
}

impl Snapshot {
    /// Started observations without a completed result.
    pub fn unfinished(&self) -> [u64; 7] {
        std::array::from_fn(|i| {
            self.starts[i]
                .checked_sub(self.results[i].iter().sum())
                .expect("a completed observation has one start")
        })
    }
}

/// A token contains only the entry tag and whether counting was enabled.
#[must_use]
pub struct Observation(Option<usize>);

impl Observation {
    /// Counts entry metadata without changing any proof or computation.
    pub fn begin(tag: BoundTag) -> Self {
        if ACTIVE.load(Ordering::Acquire) {
            let tag = tag as usize;
            STARTS[tag].fetch_add(1, Ordering::Relaxed);
            Self(Some(tag))
        } else {
            Self(None)
        }
    }

    /// Counts the already-computed original result, consuming the token once.
    pub fn complete(self, outcome: Outcome) {
        if let Some(tag) = self.0 {
            RESULTS[tag][outcome as usize].fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// Prints after the original board assertion, including unwinding.
pub struct Report {
    finished: bool,
}

impl Report {
    /// Starts with no other active report or observed worker.
    pub fn begin() -> Self {
        assert!(!ACTIVE.load(Ordering::Acquire));
        for counter in &STARTS {
            counter.store(0, Ordering::Relaxed);
        }
        for row in &RESULTS {
            for counter in row {
                counter.store(0, Ordering::Relaxed);
            }
        }
        ACTIVE.store(true, Ordering::Release);
        Self { finished: false }
    }

    /// Stops after workers have joined, without printing.
    pub fn finish(mut self) -> Snapshot {
        self.finished = true;
        finish()
    }
}

fn snapshot() -> Snapshot {
    Snapshot {
        starts: std::array::from_fn(|i| STARTS[i].load(Ordering::Relaxed)),
        results: std::array::from_fn(|i| {
            std::array::from_fn(|j| RESULTS[i][j].load(Ordering::Relaxed))
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
            let counts = finish();
            let unfinished = counts.unfinished();
            for (i, remaining) in unfinished.iter().enumerate() {
                let row = counts.results[i];
                eprintln!(
                    "BOUND_SIGN_FALLBACK\t{}\t{}\t{}\t{}\t{}\t{}\t{}",
                    i, counts.starts[i], row[0], row[1], row[2], row[3], remaining
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_tag_result_and_mismatch_is_preserved() {
        let report = Report::begin();
        for tag in [
            BoundTag::Invalid,
            BoundTag::Unknown,
            BoundTag::Zero,
            BoundTag::Negative,
            BoundTag::Positive,
            BoundTag::Unsigned,
            BoundTag::NonzeroZeroSign,
        ] {
            for result in [
                Outcome::Unknown,
                Outcome::Negative,
                Outcome::Zero,
                Outcome::Positive,
            ] {
                Observation::begin(tag).complete(result);
            }
        }
        let counts = report.finish();
        assert_eq!(counts.starts, [4; 7]);
        assert_eq!(counts.results, [[1; 4]; 7]);
        assert_eq!(counts.unfinished(), [0; 7]);
    }

    #[test]
    fn disabled_reset_and_unfinished_unwind_are_preserved() {
        let empty = Snapshot {
            starts: [0; 7],
            results: [[0; 4]; 7],
        };
        assert_eq!(Report::begin().finish(), empty);
        Observation::begin(BoundTag::Positive).complete(Outcome::Negative);
        assert_eq!(snapshot(), empty);
        assert!(
            std::panic::catch_unwind(|| {
                let _report = Report::begin();
                let _unfinished = Observation::begin(BoundTag::Negative);
                panic!("intentional calibration unwind");
            })
            .is_err()
        );
        assert!(!ACTIVE.load(Ordering::Acquire));
        assert_eq!(snapshot().starts, [0, 0, 0, 1, 0, 0, 0]);
        assert_eq!(snapshot().unfinished(), [0, 0, 0, 1, 0, 0, 0]);
        assert_eq!(Report::begin().finish(), empty);
    }

    #[test]
    fn joined_workers_contribute_without_lost_counts() {
        let report = Report::begin();
        std::thread::scope(|scope| {
            for _ in 0..4 {
                scope.spawn(|| {
                    for _ in 0..1000 {
                        Observation::begin(BoundTag::Positive).complete(Outcome::Positive);
                    }
                });
            }
        });
        let counts = report.finish();
        assert_eq!(counts.starts, [0, 0, 0, 0, 4000, 0, 0]);
        assert_eq!(counts.results[4], [0, 0, 0, 4000]);
        assert_eq!(counts.unfinished(), [0; 7]);
    }
}
