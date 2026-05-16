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

/// Aggregated trace count for one `(layer, operation)` pair.
///
/// This is the smallest stable reporting unit for cross-stack exact geometry
/// traces. `hyperlattice` and `hyperlimit` both record into this module when
/// their trace features are enabled, so operation summaries let benchmark
/// harnesses correlate matrix/vector fact use, predicate stages, scalar
/// reducers, approximation requests, and cache hits without depending on the
/// private dispatch labels of any one crate. The design follows Yap's
/// exact-geometric-computation model: observe which arithmetic package and
/// object-fact boundary was selected before judging performance. See Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997).
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OperationSummary {
    /// Trace layer, such as `real`, `hyperlimit`, or `hyperlattice_matrix`.
    pub layer: &'static str,
    /// Operation name recorded by the caller.
    pub operation: &'static str,
    /// Total count for all paths under this operation.
    pub count: u64,
}

/// Aggregated trace count for one layer.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LayerSummary {
    /// Trace layer, such as `real`, `hyperlimit`, or `hyperlattice_vector`.
    pub layer: &'static str,
    /// Total count for every operation and path under this layer.
    pub count: u64,
}

/// Unified snapshot of dispatch labels plus rational reducer statistics.
///
/// The snapshot is intentionally a read-only report value. It does not own
/// caches and it does not certify geometry; it only records which exact
/// arithmetic and object-fact paths were exercised during a recording scope.
/// This gives Criterion benches and regression tests one cross-crate view of
/// the computation ladder described by Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TraceSnapshot {
    /// Raw `(layer, operation, path)` counts.
    pub dispatch: Vec<DispatchCount>,
    /// Rational reducer counters collected during the same recording window.
    pub rational: RationalTraceStats,
}

impl TraceSnapshot {
    /// Return the count for one exact dispatch path.
    pub fn path_count(&self, layer: &str, operation: &str, path: &str) -> u64 {
        self.dispatch
            .iter()
            .find(|entry| {
                entry.layer == layer && entry.operation == operation && entry.path == path
            })
            .map_or(0, |entry| entry.count)
    }

    /// Return the total count for one layer.
    pub fn layer_count(&self, layer: &str) -> u64 {
        self.dispatch
            .iter()
            .filter(|entry| entry.layer == layer)
            .map(|entry| entry.count)
            .sum()
    }

    /// Return the total count for one `(layer, operation)` pair.
    pub fn operation_count(&self, layer: &str, operation: &str) -> u64 {
        self.dispatch
            .iter()
            .filter(|entry| entry.layer == layer && entry.operation == operation)
            .map(|entry| entry.count)
            .sum()
    }

    /// Return counts grouped by layer.
    pub fn layer_summaries(&self) -> Vec<LayerSummary> {
        let mut grouped = BTreeMap::<&'static str, u64>::new();
        for entry in &self.dispatch {
            *grouped.entry(entry.layer).or_insert(0) += entry.count;
        }
        grouped
            .into_iter()
            .map(|(layer, count)| LayerSummary { layer, count })
            .collect()
    }

    /// Return counts grouped by `(layer, operation)`.
    pub fn operation_summaries(&self) -> Vec<OperationSummary> {
        let mut grouped = BTreeMap::<(&'static str, &'static str), u64>::new();
        for entry in &self.dispatch {
            *grouped.entry((entry.layer, entry.operation)).or_insert(0) += entry.count;
        }
        grouped
            .into_iter()
            .map(|((layer, operation), count)| OperationSummary {
                layer,
                operation,
                count,
            })
            .collect()
    }
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

/// Return a unified snapshot without clearing counters.
pub fn snapshot_trace() -> TraceSnapshot {
    TraceSnapshot {
        dispatch: snapshot(),
        rational: snapshot_rational_stats(),
    }
}

/// Return a unified snapshot and clear all trace counters.
pub fn take_trace() -> TraceSnapshot {
    TraceSnapshot {
        dispatch: take(),
        rational: take_rational_stats(),
    }
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

    #[test]
    fn unified_trace_snapshot_groups_cross_stack_counts() {
        reset();
        with_recording(|| {
            record("hyperlimit", "resolve_real_sign", "structural-real-facts");
            record("hyperlimit", "resolve_real_sign", "exact-predicate");
            record("hyperlattice_matrix", "query", "matrix4-structural-facts");
            record("hyperlattice_matrix", "query", "matrix4-structural-facts");
            record("real", "detailed_facts", "pi-like");
            record_rational_temporary();
        });

        let snapshot = snapshot_trace();
        assert_eq!(
            snapshot.path_count("hyperlattice_matrix", "query", "matrix4-structural-facts"),
            2
        );
        assert_eq!(
            snapshot.operation_count("hyperlimit", "resolve_real_sign"),
            2
        );
        assert_eq!(snapshot.layer_count("hyperlattice_matrix"), 2);
        assert_eq!(snapshot.rational.temporary_rationals, 1);

        let operations = snapshot.operation_summaries();
        assert!(operations.iter().any(|entry| {
            entry.layer == "hyperlimit"
                && entry.operation == "resolve_real_sign"
                && entry.count == 2
        }));

        let layers = snapshot.layer_summaries();
        assert!(
            layers
                .iter()
                .any(|entry| { entry.layer == "real" && entry.count == 1 })
        );

        let taken = take_trace();
        assert_eq!(taken.layer_count("hyperlimit"), 2);
        assert!(snapshot_trace().dispatch.is_empty());
        assert_eq!(snapshot_trace().rational, RationalTraceStats::default());
    }
}
