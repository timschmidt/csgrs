//! Diagnostic-only counts of existing retained quadratic-projection work.
//!
//! Contexts contain cache addresses only; no address is dereferenced and no
//! scalar, cache guard, approximation or other owner is retained here.
use std::cell::Cell;
use std::marker::PhantomData;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

const ROLES: [&str; 2] = ["root", "recursive"];
const SHAPES: [&str; 6] = [
    "empty",
    "rational",
    "quadratic",
    "multiple_radicals",
    "nonzero_power",
    "multiple_powers",
];
static ACTIVE: AtomicBool = AtomicBool::new(false);
static QUERIES: [AtomicU64; 5] = [const { AtomicU64::new(0) }; 5];
static READS: [[AtomicU64; 3]; 2] = [const { [const { AtomicU64::new(0) }; 3] }; 2];
static COPIES: [[[AtomicU64; 4]; 6]; 2] =
    [const { [const { [const { AtomicU64::new(0) }; 4] }; 6] }; 2];

thread_local! {
    static ROOT: Cell<Option<usize>> = const { Cell::new(None) };
}

/// Fixed counters read only after all observed workers have returned.
#[derive(Debug, PartialEq, Eq)]
pub struct Snapshot {
    /// Started, succeeded, declined, unwound and original-retry root queries.
    pub queries: [u64; 5],
    /// Missing cell, missing closed form and executed copy, root then recursive.
    pub reads: [[u64; 3]; 2],
    /// Copies, term entries, radical entries and nonempty radical vectors.
    /// Indexed by root/recursive, then the six printed shape names.
    pub copies: [[[u64; 4]; 6]; 2],
}

/// An explicit observation lifetime, ended after its workers have joined.
pub struct Report {
    finished: bool,
}

impl Report {
    /// Reset counters and begin while no other report or observed work is active.
    pub fn begin() -> Self {
        assert!(!ACTIVE.load(Ordering::Acquire));
        for counter in &QUERIES {
            counter.store(0, Ordering::Relaxed);
        }
        for row in &READS {
            for counter in row {
                counter.store(0, Ordering::Relaxed);
            }
        }
        for role in &COPIES {
            for row in role {
                for counter in row {
                    counter.store(0, Ordering::Relaxed);
                }
            }
        }
        ACTIVE.store(true, Ordering::Release);
        Self { finished: false }
    }

    /// Finish without printing, after every observed worker has returned.
    pub fn finish(mut self) -> Snapshot {
        self.finished = true;
        finish()
    }
}

fn snapshot() -> Snapshot {
    Snapshot {
        queries: std::array::from_fn(|i| QUERIES[i].load(Ordering::Relaxed)),
        reads: std::array::from_fn(|i| {
            std::array::from_fn(|j| READS[i][j].load(Ordering::Relaxed))
        }),
        copies: std::array::from_fn(|i| {
            std::array::from_fn(|j| {
                std::array::from_fn(|k| COPIES[i][j][k].load(Ordering::Relaxed))
            })
        }),
    }
}

fn finish() -> Snapshot {
    assert!(ACTIVE.swap(false, Ordering::AcqRel));
    snapshot()
}

impl Drop for Report {
    fn drop(&mut self) {
        if self.finished {
            return;
        }
        let counters = finish();
        let q = counters.queries;
        eprintln!(
            "RETAINED_PROJECTION_QUERY\t{}\t{}\t{}\t{}\t{}",
            q[0], q[1], q[2], q[3], q[4]
        );
        for (role, reads) in counters.reads.into_iter().enumerate() {
            eprintln!(
                "RETAINED_PROJECTION_READ\t{}\t{}\t{}\t{}",
                ROLES[role], reads[0], reads[1], reads[2]
            );
            for (shape, row) in counters.copies[role].into_iter().enumerate() {
                eprintln!(
                    "RETAINED_PROJECTION_COPY\t{}\t{}\t{}\t{}\t{}\t{}",
                    ROLES[role], SHAPES[shape], row[0], row[1], row[2], row[3]
                );
            }
        }
    }
}

/// Restores the previous thread-local root on return or unwind.
pub(crate) struct RootScope {
    previous: Option<usize>,
    active: bool,
    succeeded: bool,
    // A scope must be dropped on the thread whose context it replaced.
    _not_send: PhantomData<*mut ()>,
}

impl RootScope {
    pub(crate) fn begin(address: usize) -> Self {
        let active = ACTIVE.load(Ordering::Acquire);
        let previous = if active {
            QUERIES[0].fetch_add(1, Ordering::Relaxed);
            ROOT.with(|root| root.replace(Some(address)))
        } else {
            None
        };
        Self {
            previous,
            active,
            succeeded: false,
            _not_send: PhantomData,
        }
    }

    pub(crate) fn success(&mut self) {
        self.succeeded = true;
    }
}

impl Drop for RootScope {
    fn drop(&mut self) {
        if self.active {
            let outcome = if std::thread::panicking() {
                3
            } else if self.succeeded {
                1
            } else {
                2
            };
            QUERIES[outcome].fetch_add(1, Ordering::Relaxed);
            ROOT.with(|root| root.set(self.previous));
        }
    }
}

fn role(address: usize) -> Option<usize> {
    if !ACTIVE.load(Ordering::Acquire) {
        return None;
    }
    ROOT.with(|root| root.get().map(|root| usize::from(root != address)))
}

pub(crate) fn record_cell(address: usize, exists: bool) {
    if !exists && let Some(role) = role(address) {
        READS[role][0].fetch_add(1, Ordering::Relaxed);
    }
}

pub(crate) fn record_missing_form(address: usize) {
    if let Some(role) = role(address) {
        READS[role][1].fetch_add(1, Ordering::Relaxed);
    }
}

pub(crate) fn record_copy(
    address: usize,
    shape: impl FnOnce() -> (usize, Option<(i16, usize)>, usize, usize),
) {
    let Some(role) = role(address) else {
        return;
    };
    let (terms, first, radicals, vectors) = shape();
    let bin = match (terms, first) {
        (0, _) => 0,
        (1, Some((0, 0))) => 1,
        (1, Some((0, 1))) => 2,
        (1, Some((0, _))) => 3,
        (1, Some(_)) => 4,
        _ => 5,
    };
    READS[role][2].fetch_add(1, Ordering::Relaxed);
    for (counter, count) in
        COPIES[role][bin]
            .iter()
            .zip([1, terms as u64, radicals as u64, vectors as u64])
    {
        counter.fetch_add(count, Ordering::Relaxed);
    }
}

pub(crate) fn record_retry(address: usize) {
    if role(address) == Some(0) {
        QUERIES[4].fetch_add(1, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_shapes_roles_reset_and_disabled_callbacks_are_observed() {
        let empty = Snapshot {
            queries: [0; 5],
            reads: [[0; 3]; 2],
            copies: [[[0; 4]; 6]; 2],
        };
        assert_eq!(Report::begin().finish(), empty);
        record_copy(7, || panic!("disabled callback must not run"));
        let report = Report::begin();
        record_copy(7, || panic!("out-of-scope callback must not run"));
        {
            let mut scope = RootScope::begin(7);
            for address in [7, 11] {
                record_cell(address, false);
                record_cell(address, true);
                record_missing_form(address);
                for shape in [
                    (0, None, 0, 0),
                    (1, Some((0, 0)), 0, 0),
                    (1, Some((0, 1)), 1, 1),
                    (1, Some((0, 3)), 3, 1),
                    (1, Some((-1, 2)), 2, 1),
                    (3, Some((0, 1)), 5, 2),
                ] {
                    record_copy(address, || shape);
                }
            }
            record_retry(7);
            record_retry(11);
            scope.success();
        }
        let actual = report.finish();
        assert_eq!(actual.queries, [1, 1, 0, 0, 1]);
        assert_eq!(actual.reads, [[1, 1, 6]; 2]);
        assert_eq!(
            actual.copies,
            [[
                [1, 0, 0, 0],
                [1, 1, 0, 0],
                [1, 1, 1, 1],
                [1, 1, 3, 1],
                [1, 1, 2, 1],
                [1, 3, 5, 2],
            ]; 2]
        );
        record_missing_form(7);
        record_retry(7);
        assert_eq!(snapshot(), actual);
        assert_eq!(Report::begin().finish(), empty);
    }

    #[test]
    fn nested_scopes_restore_context_on_return_and_unwind() {
        let report = Report::begin();
        {
            let mut outer = RootScope::begin(13);
            {
                let _inner = RootScope::begin(17);
                record_cell(17, false);
                record_cell(13, false);
            }
            assert!(
                std::panic::catch_unwind(|| {
                    let _inner = RootScope::begin(19);
                    record_missing_form(19);
                    panic!("intentional scope unwind");
                })
                .is_err()
            );
            record_retry(13);
            record_cell(13, false);
            outer.success();
        }
        assert!(ROOT.with(|root| root.get().is_none()));
        record_cell(13, false);
        let actual = report.finish();
        assert_eq!(actual.queries, [3, 1, 1, 1, 1]);
        assert_eq!(actual.reads, [[2, 1, 0], [1, 0, 0]]);
    }

    #[test]
    fn joined_workers_keep_separate_roots_and_aggregate_counts() {
        let report = Report::begin();
        std::thread::scope(|threads| {
            for worker in 0..4 {
                threads.spawn(move || {
                    for _ in 0..1000 {
                        let mut root = RootScope::begin(worker);
                        record_copy(worker, || (1, Some((0, 1)), 1, 1));
                        record_cell(worker + 10, false);
                        root.success();
                    }
                    assert!(ROOT.with(|root| root.get().is_none()));
                });
            }
        });
        let actual = report.finish();
        assert_eq!(actual.queries, [4000, 4000, 0, 0, 0]);
        assert_eq!(actual.reads, [[0, 0, 4000], [4000, 0, 0]]);
        assert_eq!(actual.copies[0][2], [4000, 4000, 4000, 4000]);
        assert_eq!(actual.copies[1], [[0; 4]; 6]);
    }

    #[test]
    fn report_unwind_ends_recording_after_query_scope_drops() {
        assert!(
            std::panic::catch_unwind(|| {
                let _report = Report::begin();
                let _query = RootScope::begin(23);
                record_cell(23, false);
                panic!("intentional report unwind");
            })
            .is_err()
        );
        assert!(!ACTIVE.load(Ordering::Acquire));
        assert!(ROOT.with(|root| root.get().is_none()));
        assert_eq!(snapshot().queries, [1, 0, 0, 1, 0]);
        assert_eq!(snapshot().reads, [[1, 0, 0], [0, 0, 0]]);
        assert_eq!(Report::begin().finish().queries, [0; 5]);
    }
}
