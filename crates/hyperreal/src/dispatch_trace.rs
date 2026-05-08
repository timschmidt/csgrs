use std::cell::Cell;
use std::collections::BTreeMap;
use std::sync::{Mutex, OnceLock};

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DispatchCount {
    pub layer: &'static str,
    pub operation: &'static str,
    pub path: &'static str,
    pub count: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
struct DispatchKey {
    layer: &'static str,
    operation: &'static str,
    path: &'static str,
}

static COUNTS: OnceLock<Mutex<BTreeMap<DispatchKey, u64>>> = OnceLock::new();

thread_local! {
    static RECORDING: Cell<bool> = const { Cell::new(false) };
}

fn counts() -> &'static Mutex<BTreeMap<DispatchKey, u64>> {
    COUNTS.get_or_init(|| Mutex::new(BTreeMap::new()))
}

pub struct RecordingGuard {
    previous: bool,
}

impl Drop for RecordingGuard {
    fn drop(&mut self) {
        RECORDING.with(|recording| recording.set(self.previous));
    }
}

pub fn reset() {
    counts()
        .lock()
        .expect("dispatch trace lock poisoned")
        .clear();
}

pub fn recording_scope() -> RecordingGuard {
    let previous = RECORDING.with(|recording| {
        let previous = recording.get();
        recording.set(true);
        previous
    });
    RecordingGuard { previous }
}

pub fn with_recording<T>(f: impl FnOnce() -> T) -> T {
    let _guard = recording_scope();
    f()
}

pub fn record(layer: &'static str, operation: &'static str, path: &'static str) {
    if !RECORDING.with(Cell::get) {
        return;
    }
    let key = DispatchKey {
        layer,
        operation,
        path,
    };
    *counts()
        .lock()
        .expect("dispatch trace lock poisoned")
        .entry(key)
        .or_insert(0) += 1;
}

pub fn snapshot() -> Vec<DispatchCount> {
    counts()
        .lock()
        .expect("dispatch trace lock poisoned")
        .iter()
        .map(|(key, count)| DispatchCount {
            layer: key.layer,
            operation: key.operation,
            path: key.path,
            count: *count,
        })
        .collect()
}

pub fn take() -> Vec<DispatchCount> {
    let mut counts = counts().lock().expect("dispatch trace lock poisoned");
    let snapshot = counts
        .iter()
        .map(|(key, count)| DispatchCount {
            layer: key.layer,
            operation: key.operation,
            path: key.path,
            count: *count,
        })
        .collect();
    counts.clear();
    snapshot
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dispatch_trace_records_only_inside_scope() {
        reset();
        record("real", "sin", "ignored");
        assert!(snapshot().is_empty());

        with_recording(|| {
            record("real", "sin", "path");
            record("real", "sin", "path");
            record("computable", "sin", "other");
        });

        let counts = take();
        assert_eq!(counts.len(), 2);
        assert!(counts.iter().any(|entry| {
            entry.layer == "real"
                && entry.operation == "sin"
                && entry.path == "path"
                && entry.count == 2
        }));
        assert!(snapshot().is_empty());
    }
}
