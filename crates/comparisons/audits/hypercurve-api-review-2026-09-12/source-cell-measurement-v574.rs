// Copied-source diagnostic only. No production use or recursive geometry output.
#[cfg(test)]
mod source_cell_measurement {
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::time::Instant;

    static COUNTS: [AtomicU64; 3] = [const { AtomicU64::new(0) }; 3];
    static NANOS: [AtomicU64; 3] = [const { AtomicU64::new(0) }; 3];

    pub(super) struct Timer {
        bucket: usize,
        start: Instant,
    }

    impl Timer {
        pub(super) fn new(bucket: usize) -> Self {
            Self { bucket, start: Instant::now() }
        }
    }

    impl Drop for Timer {
        fn drop(&mut self) {
            COUNTS[self.bucket].fetch_add(1, Ordering::Relaxed);
            NANOS[self.bucket].fetch_add(
                u64::try_from(self.start.elapsed().as_nanos()).unwrap(),
                Ordering::Relaxed,
            );
        }
    }

    pub(super) struct Session;

    impl Drop for Session {
        fn drop(&mut self) {
            for (index, name) in ["direction_schedule", "pair_source_cells", "pair_cell_selection"]
                .into_iter().enumerate()
            {
                eprintln!(
                    "SOURCE_CELL_WORK {name} calls={} seconds={:.6}",
                    COUNTS[index].load(Ordering::Relaxed),
                    NANOS[index].load(Ordering::Relaxed) as f64 / 1_000_000_000.0,
                );
            }
        }
    }
}
