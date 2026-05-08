use std::cell::Cell;
use std::collections::BTreeMap;
use std::sync::{Mutex, OnceLock};

use num::{BigUint, One, Zero};

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DispatchCount {
    pub layer: &'static str,
    pub operation: &'static str,
    pub path: &'static str,
    pub count: u64,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct CommonFactorBuckets {
    pub none: u64,
    pub power_of_two: u64,
    pub small: u64,
    pub medium: u64,
    pub large: u64,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RationalTraceStats {
    pub temporary_rationals: u64,
    pub reductions: u64,
    pub gcds: u64,
    pub common_factors: CommonFactorBuckets,
    pub peak_operand_bits: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
struct DispatchKey {
    layer: &'static str,
    operation: &'static str,
    path: &'static str,
}

static COUNTS: OnceLock<Mutex<BTreeMap<DispatchKey, u64>>> = OnceLock::new();
static RATIONAL_STATS: OnceLock<Mutex<RationalTraceStats>> = OnceLock::new();

thread_local! {
    static RECORDING: Cell<bool> = const { Cell::new(false) };
}

fn counts() -> &'static Mutex<BTreeMap<DispatchKey, u64>> {
    COUNTS.get_or_init(|| Mutex::new(BTreeMap::new()))
}

fn rational_stats() -> &'static Mutex<RationalTraceStats> {
    RATIONAL_STATS.get_or_init(|| Mutex::new(RationalTraceStats::default()))
}

fn is_recording() -> bool {
    RECORDING.with(Cell::get)
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
    reset_rational_stats();
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
    if !is_recording() {
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

fn update_peak(stats: &mut RationalTraceStats, value: &BigUint) {
    stats.peak_operand_bits = stats.peak_operand_bits.max(value.bits());
}

fn record_common_factor(stats: &mut RationalTraceStats, divisor: &BigUint) {
    if divisor.is_zero() || divisor.is_one() {
        stats.common_factors.none += 1;
    } else if divisor.trailing_zeros() == Some(divisor.bits() - 1) {
        stats.common_factors.power_of_two += 1;
    } else {
        match divisor.bits() {
            0..=8 => stats.common_factors.small += 1,
            9..=64 => stats.common_factors.medium += 1,
            _ => stats.common_factors.large += 1,
        }
    }
}

pub fn record_rational_temporary() {
    if !is_recording() {
        return;
    }
    rational_stats()
        .lock()
        .expect("rational trace lock poisoned")
        .temporary_rationals += 1;
}

pub fn record_rational_reduction(numerator: &BigUint, denominator: &BigUint) {
    if !is_recording() {
        return;
    }
    let mut stats = rational_stats()
        .lock()
        .expect("rational trace lock poisoned");
    stats.reductions += 1;
    update_peak(&mut stats, numerator);
    update_peak(&mut stats, denominator);
}

pub fn record_rational_gcd(left: &BigUint, right: &BigUint, divisor: &BigUint) {
    if !is_recording() {
        return;
    }
    let mut stats = rational_stats()
        .lock()
        .expect("rational trace lock poisoned");
    stats.gcds += 1;
    update_peak(&mut stats, left);
    update_peak(&mut stats, right);
    update_peak(&mut stats, divisor);
    record_common_factor(&mut stats, divisor);
}

pub fn record_rational_power_of_two_common_factor(shift: u64) {
    if !is_recording() {
        return;
    }
    let mut stats = rational_stats()
        .lock()
        .expect("rational trace lock poisoned");
    if shift == 0 {
        stats.common_factors.none += 1;
    } else {
        stats.common_factors.power_of_two += 1;
    }
}

pub fn reset_rational_stats() {
    *rational_stats()
        .lock()
        .expect("rational trace lock poisoned") = RationalTraceStats::default();
}

pub fn snapshot_rational_stats() -> RationalTraceStats {
    *rational_stats()
        .lock()
        .expect("rational trace lock poisoned")
}

pub fn take_rational_stats() -> RationalTraceStats {
    let mut stats = rational_stats()
        .lock()
        .expect("rational trace lock poisoned");
    let snapshot = *stats;
    *stats = RationalTraceStats::default();
    snapshot
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

    #[test]
    fn rational_trace_records_reductions_and_gcds() {
        use crate::Rational;

        reset();
        with_recording(|| {
            let left = Rational::fraction(6, 8).unwrap();
            let right = Rational::fraction(9, 10).unwrap();
            let _ = left + right;
        });

        let stats = take_rational_stats();
        assert!(stats.temporary_rationals > 0);
        assert!(stats.reductions > 0);
        assert!(stats.gcds > 0);
        assert!(stats.peak_operand_bits > 0);
    }
}
