use std::cell::RefCell;
use std::collections::{BTreeMap, HashMap};
use std::hash::{BuildHasherDefault, Hash, Hasher};

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
/// object-fact boundary was selected before judging performance.
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

/// Coarse semantic correlation of one trace snapshot.
///
/// This summary is deliberately conservative: it classifies existing
/// cross-stack trace labels into broad buckets that match Yap's exact
/// geometric computation stack: object facts, scalar facts, exact reducers,
/// certified or lossy approximation boundaries, refinement, caches, and
/// fallbacks. The raw labels remain available for detailed profiling; this
/// type gives benchmark reports one stable, crate-independent view for asking
/// whether a run spent time preserving structure or rediscovering scalar facts.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct TraceCorrelationSummary {
    /// Total recorded dispatch events.
    pub dispatch_events: u64,
    /// Events recorded by predicate layers such as `hyperlimit`.
    pub predicate_events: u64,
    /// Events recorded by vector, matrix, or algebra object layers.
    pub linear_algebra_events: u64,
    /// Events that appear to use object-level facts, schedules, or prepared
    /// handles.
    pub object_fact_events: u64,
    /// Events that appear to query scalar-owned facts.
    pub scalar_fact_events: u64,
    /// Events that appear to ask for nontrivial/detailed fact packages instead
    /// of only cheap structural tags.
    pub detailed_fact_events: u64,
    /// Events that report an unknown, uncertain, or sign-missing fact.
    pub unknown_fact_events: u64,
    /// Events that appear to classify exact-rational representation kind, such
    /// as dyadic, shared-denominator, or exact-set eligibility.
    pub exact_rational_kind_events: u64,
    /// Events that appear to query sign, zero, or ordering facts.
    pub sign_or_zero_query_events: u64,
    /// Events that appear to select exact rational, determinant, or
    /// product-sum reducers.
    pub exact_reducer_events: u64,
    /// Events that appear to enter approximate, lossy, or primitive-float
    /// adapter paths.
    pub approximation_events: u64,
    /// Events that appear to start approximation or export an approximate view.
    pub approximation_start_events: u64,
    /// Events that appear to hit or consume an approximation cache.
    pub approximation_cache_events: u64,
    /// Events that appear to refine a `Real` or certified sign.
    pub refinement_events: u64,
    /// Events that appear to be predicate decision stages such as filters,
    /// exact predicate resolution, refinement, or explicit uncertainty.
    pub predicate_decision_stage_events: u64,
    /// Events that appear to hit, create, or consume prepared/cached state.
    pub cache_events: u64,
    /// Events that appear to abort, reject, report domain errors, or fall back
    /// to generic/unknown paths.
    pub fallback_or_abort_events: u64,
    /// Rational temporary counter from the same recording window.
    pub rational_temporaries: u64,
    /// Rational reduction counter from the same recording window.
    pub rational_reductions: u64,
    /// Rational GCD counter from the same recording window.
    pub rational_gcds: u64,
}

/// Unified snapshot of dispatch labels plus rational reducer statistics.
///
/// The snapshot is intentionally a read-only report value. It does not own
/// caches and it does not certify geometry; it only records which exact
/// arithmetic and object-fact paths were exercised during a recording scope.
/// This gives Criterion benches and regression tests one cross-crate view of
/// the computation ladder.
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

    /// Return a coarse semantic correlation summary for this snapshot.
    ///
    /// The classifier is a reporting aid, not a correctness certificate. It
    /// intentionally derives from public trace labels plus rational reducer
    /// counters so benchmark harnesses can correlate predicate stages, matrix
    /// fact use, reducer pressure, approximation requests, cache hits, and
    /// fallback paths without depending on each crate's private data
    /// structures.
    pub fn correlation_summary(&self) -> TraceCorrelationSummary {
        let mut summary = TraceCorrelationSummary {
            rational_temporaries: self.rational.temporary_rationals,
            rational_reductions: self.rational.reductions,
            rational_gcds: self.rational.gcds,
            ..TraceCorrelationSummary::default()
        };

        for entry in &self.dispatch {
            summary.dispatch_events += entry.count;
            if is_predicate_layer(entry.layer) {
                summary.predicate_events += entry.count;
            }
            if is_linear_algebra_layer(entry.layer) {
                summary.linear_algebra_events += entry.count;
            }
            if is_object_fact_event(entry) {
                summary.object_fact_events += entry.count;
            }
            if is_scalar_fact_event(entry) {
                summary.scalar_fact_events += entry.count;
            }
            if is_detailed_fact_event(entry) {
                summary.detailed_fact_events += entry.count;
            }
            if is_unknown_fact_event(entry) {
                summary.unknown_fact_events += entry.count;
            }
            if is_exact_rational_kind_event(entry) {
                summary.exact_rational_kind_events += entry.count;
            }
            if is_sign_or_zero_query_event(entry) {
                summary.sign_or_zero_query_events += entry.count;
            }
            if is_exact_reducer_event(entry) {
                summary.exact_reducer_events += entry.count;
            }
            if is_approximation_event(entry) {
                summary.approximation_events += entry.count;
            }
            if is_approximation_start_event(entry) {
                summary.approximation_start_events += entry.count;
            }
            if is_approximation_cache_event(entry) {
                summary.approximation_cache_events += entry.count;
            }
            if is_refinement_event(entry) {
                summary.refinement_events += entry.count;
            }
            if is_predicate_decision_stage_event(entry) {
                summary.predicate_decision_stage_events += entry.count;
            }
            if is_cache_event(entry) {
                summary.cache_events += entry.count;
            }
            if is_fallback_or_abort_event(entry) {
                summary.fallback_or_abort_events += entry.count;
            }
        }

        summary
    }
}

fn contains_label_part(value: &str, needle: &str) -> bool {
    value.contains(needle)
}

fn entry_contains(entry: &DispatchCount, needle: &str) -> bool {
    contains_label_part(entry.layer, needle)
        || contains_label_part(entry.operation, needle)
        || contains_label_part(entry.path, needle)
}

fn is_predicate_layer(layer: &str) -> bool {
    layer == "hyperlimit" || layer.contains("predicate")
}

fn is_linear_algebra_layer(layer: &str) -> bool {
    layer.starts_with("hyperlattice")
}

fn is_object_fact_event(entry: &DispatchCount) -> bool {
    entry.layer != "real"
        && (entry_contains(entry, "facts")
            || entry_contains(entry, "structural")
            || entry_contains(entry, "shared-scale")
            || entry_contains(entry, "schedule")
            || entry_contains(entry, "prepared"))
}

fn is_scalar_fact_event(entry: &DispatchCount) -> bool {
    entry.layer == "real"
        && (entry_contains(entry, "facts")
            || entry_contains(entry, "exact-set")
            || entry_contains(entry, "zero")
            || entry_contains(entry, "domain")
            || entry_contains(entry, "sign"))
}

fn is_detailed_fact_event(entry: &DispatchCount) -> bool {
    entry_contains(entry, "detailed")
        || entry_contains(entry, "exact_set_facts")
        || entry_contains(entry, "exact-set")
        || entry_contains(entry, "structural-facts")
        || entry_contains(entry, "geometry_facts")
}

fn is_unknown_fact_event(entry: &DispatchCount) -> bool {
    entry_contains(entry, "unknown")
        || entry_contains(entry, "uncertain")
        || entry_contains(entry, "missing-sign")
        || entry_contains(entry, "nonzero-no-sign")
        || entry_contains(entry, "unavailable")
}

fn is_exact_rational_kind_event(entry: &DispatchCount) -> bool {
    entry_contains(entry, "exact-rational")
        || entry_contains(entry, "rational-det")
        || entry_contains(entry, "rational-kind")
        || entry_contains(entry, "exact-set")
        || entry_contains(entry, "exact_set")
        || entry_contains(entry, "dyadic")
        || entry_contains(entry, "shared-denominator")
        || entry_contains(entry, "common-denominator")
}

fn is_sign_or_zero_query_event(entry: &DispatchCount) -> bool {
    entry_contains(entry, "sign")
        || entry_contains(entry, "zero")
        || entry_contains(entry, "compare")
        || entry_contains(entry, "ordering")
        || entry_contains(entry, "domain")
}

fn is_exact_reducer_event(entry: &DispatchCount) -> bool {
    entry_contains(entry, "exact")
        || entry_contains(entry, "rational")
        || entry_contains(entry, "determinant")
        || entry_contains(entry, "product-sum")
        || entry_contains(entry, "signed-product-sum")
        || entry_contains(entry, "kernel")
        || entry_contains(entry, "dyadic")
        || entry_contains(entry, "shared-denominator")
}

fn is_approximation_event(entry: &DispatchCount) -> bool {
    entry_contains(entry, "approx")
        || entry_contains(entry, "lossy")
        || entry_contains(entry, "f64")
        || entry_contains(entry, "float")
}

fn is_approximation_start_event(entry: &DispatchCount) -> bool {
    is_approximation_event(entry)
        && !is_approximation_cache_event(entry)
        && (entry_contains(entry, "start")
            || entry_contains(entry, "export")
            || entry_contains(entry, "lossy")
            || entry_contains(entry, "generic")
            || entry_contains(entry, "approx"))
}

fn is_approximation_cache_event(entry: &DispatchCount) -> bool {
    is_approximation_event(entry)
        && (entry_contains(entry, "cache")
            || entry_contains(entry, "cached")
            || entry_contains(entry, "hit"))
}

fn is_refinement_event(entry: &DispatchCount) -> bool {
    entry_contains(entry, "refine")
        || entry_contains(entry, "refinement")
        || entry_contains(entry, "certified-sign")
}

fn is_predicate_decision_stage_event(entry: &DispatchCount) -> bool {
    is_predicate_layer(entry.layer)
        && (entry_contains(entry, "resolve")
            || entry_contains(entry, "decide")
            || entry_contains(entry, "filter")
            || entry_contains(entry, "exact")
            || entry_contains(entry, "refine")
            || entry_contains(entry, "real-determinant")
            || entry_contains(entry, "det")
            || entry_contains(entry, "decided")
            || entry_contains(entry, "positive")
            || entry_contains(entry, "negative")
            || entry_contains(entry, "zero")
            || entry_contains(entry, "unknown")
            || entry_contains(entry, "uncertain"))
}

fn is_cache_event(entry: &DispatchCount) -> bool {
    entry_contains(entry, "cache")
        || entry_contains(entry, "cached")
        || entry_contains(entry, "prepared")
}

fn is_fallback_or_abort_event(entry: &DispatchCount) -> bool {
    entry_contains(entry, "abort")
        || entry_contains(entry, "fallback")
        || entry_contains(entry, "generic")
        || entry_contains(entry, "unknown")
        || entry_contains(entry, "rejected")
        || entry_contains(entry, "domain-error")
        || entry_contains(entry, "div-by-zero")
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct CommonFactorBuckets {
    pub none: u64,
    pub power_of_two: u64,
    pub small: u64,
    pub medium: u64,
    pub large: u64,
}

/// Operand-width distribution for exact rational GCD calls.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct GcdOperandWidthBuckets {
    /// Both operands fit in 64 bits.
    pub both_u64: u64,
    /// Exactly one operand needs 65 through 128 bits.
    pub mixed_u64_u128: u64,
    /// Both operands need 65 through 128 bits.
    pub both_u128: u64,
    /// At least one operand exceeds 128 bits.
    pub arbitrary_precision: u64,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RationalTraceStats {
    pub temporary_rationals: u64,
    pub reductions: u64,
    pub gcds: u64,
    pub common_factors: CommonFactorBuckets,
    /// GCD calls grouped by the smallest native storage that holds their operands.
    pub gcd_operand_widths: GcdOperandWidthBuckets,
    pub peak_operand_bits: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
struct DispatchKey {
    layer: &'static str,
    operation: &'static str,
    path: &'static str,
}

/// Hot-path identity for compile-time trace labels.
///
/// Trace labels are required to be static, so a call site presents the same
/// string addresses on every invocation. Keying the live counter table by
/// those addresses avoids repeatedly comparing label contents inside exact
/// arithmetic loops. Snapshots merge identity-distinct labels with equal text
/// back into the public content-keyed representation.
#[derive(Clone, Copy)]
struct ActiveDispatchKey {
    layer: &'static str,
    operation: &'static str,
    path: &'static str,
}

impl PartialEq for ActiveDispatchKey {
    fn eq(&self, other: &Self) -> bool {
        std::ptr::eq(self.layer, other.layer)
            && std::ptr::eq(self.operation, other.operation)
            && std::ptr::eq(self.path, other.path)
    }
}

impl Eq for ActiveDispatchKey {}

impl Hash for ActiveDispatchKey {
    fn hash<H: Hasher>(&self, state: &mut H) {
        state.write_usize(self.layer.as_ptr() as usize);
        state.write_usize(self.layer.len());
        state.write_usize(self.operation.as_ptr() as usize);
        state.write_usize(self.operation.len());
        state.write_usize(self.path.as_ptr() as usize);
        state.write_usize(self.path.len());
    }
}

#[derive(Default)]
struct DispatchHasher(u64);

impl Hasher for DispatchHasher {
    fn finish(&self) -> u64 {
        self.0
    }

    fn write(&mut self, bytes: &[u8]) {
        for byte in bytes {
            self.0 ^= u64::from(*byte);
            self.0 = self.0.wrapping_mul(0x100_0000_01b3);
        }
    }

    fn write_usize(&mut self, value: usize) {
        let mut mixed = value as u64;
        mixed ^= mixed >> 30;
        mixed = mixed.wrapping_mul(0xbf58_476d_1ce4_e5b9);
        mixed ^= mixed >> 27;
        mixed = mixed.wrapping_mul(0x94d0_49bb_1331_11eb);
        mixed ^= mixed >> 31;
        self.0 = self.0.rotate_left(21) ^ mixed;
    }
}

type ActiveCounts = HashMap<ActiveDispatchKey, u64, BuildHasherDefault<DispatchHasher>>;

#[derive(Default)]
struct TraceState {
    recording: bool,
    counts: ActiveCounts,
    rational: RationalTraceStats,
}

thread_local! {
    static TRACE_STATE: RefCell<TraceState> = RefCell::new(TraceState::default());
}

pub struct RecordingGuard {
    previous: bool,
}

impl Drop for RecordingGuard {
    fn drop(&mut self) {
        TRACE_STATE.with(|state| state.borrow_mut().recording = self.previous);
    }
}

pub fn reset() {
    TRACE_STATE.with(|state| {
        let mut state = state.borrow_mut();
        state.counts.clear();
        state.rational = RationalTraceStats::default();
    });
}

pub fn recording_scope() -> RecordingGuard {
    let previous = TRACE_STATE.with(|state| {
        let mut state = state.borrow_mut();
        let previous = state.recording;
        state.recording = true;
        previous
    });
    RecordingGuard { previous }
}

pub fn with_recording<T>(f: impl FnOnce() -> T) -> T {
    let _guard = recording_scope();
    f()
}

#[inline]
pub fn record(layer: &'static str, operation: &'static str, path: &'static str) {
    TRACE_STATE.with(|state| {
        let mut state = state.borrow_mut();
        if state.recording {
            record_active(&mut state.counts, layer, operation, path);
        }
    });
}

#[cold]
#[inline(never)]
fn record_active(
    counts: &mut ActiveCounts,
    layer: &'static str,
    operation: &'static str,
    path: &'static str,
) {
    let key = ActiveDispatchKey {
        layer,
        operation,
        path,
    };
    *counts.entry(key).or_insert(0) += 1;
}

pub(crate) fn record_sign_refinement_precision(operation: &'static str, precision: i32) {
    let bucket = match precision {
        0.. => "p>=0",
        -32..=-1 => "-32<=p<0",
        -64..=-33 => "-64<=p<-32",
        -128..=-65 => "-128<=p<-64",
        -256..=-129 => "-256<=p<-128",
        -512..=-257 => "-512<=p<-256",
        -1024..=-513 => "-1024<=p<-512",
        -2048..=-1025 => "-2048<=p<-1024",
        _ => "p<-2048",
    };
    record("computable", operation, bucket);
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
    TRACE_STATE.with(|state| {
        let mut state = state.borrow_mut();
        if state.recording {
            state.rational.temporary_rationals += 1;
        }
    });
}

pub fn record_rational_reduction(numerator: &BigUint, denominator: &BigUint) {
    TRACE_STATE.with(|state| {
        let mut state = state.borrow_mut();
        if state.recording {
            state.rational.reductions += 1;
            update_peak(&mut state.rational, numerator);
            update_peak(&mut state.rational, denominator);
        }
    });
}

pub fn record_rational_gcd(left: &BigUint, right: &BigUint, divisor: &BigUint) {
    TRACE_STATE.with(|state| {
        let mut state = state.borrow_mut();
        if !state.recording {
            return;
        }
        let stats = &mut state.rational;
        stats.gcds += 1;
        let left_bits = left.bits();
        let right_bits = right.bits();
        let widest = left_bits.max(right_bits);
        let narrowest = left_bits.min(right_bits);
        if widest <= 64 {
            stats.gcd_operand_widths.both_u64 += 1;
        } else if widest <= 128 && narrowest <= 64 {
            stats.gcd_operand_widths.mixed_u64_u128 += 1;
        } else if widest <= 128 {
            stats.gcd_operand_widths.both_u128 += 1;
        } else {
            stats.gcd_operand_widths.arbitrary_precision += 1;
        }
        update_peak(stats, left);
        update_peak(stats, right);
        update_peak(stats, divisor);
        record_common_factor(stats, divisor);
    });
}

pub fn record_rational_power_of_two_common_factor(shift: u64) {
    TRACE_STATE.with(|state| {
        let mut state = state.borrow_mut();
        if !state.recording {
            return;
        }
        let stats = &mut state.rational;
        if shift == 0 {
            stats.common_factors.none += 1;
        } else {
            stats.common_factors.power_of_two += 1;
        }
    });
}

pub fn reset_rational_stats() {
    TRACE_STATE.with(|state| state.borrow_mut().rational = RationalTraceStats::default());
}

pub fn snapshot_rational_stats() -> RationalTraceStats {
    TRACE_STATE.with(|state| state.borrow().rational)
}

pub fn take_rational_stats() -> RationalTraceStats {
    TRACE_STATE.with(|state| {
        let mut state = state.borrow_mut();
        std::mem::take(&mut state.rational)
    })
}

fn collect_dispatch_counts(
    counts: impl IntoIterator<Item = (ActiveDispatchKey, u64)>,
) -> Vec<DispatchCount> {
    let mut grouped = BTreeMap::<DispatchKey, u64>::new();
    for (key, count) in counts {
        *grouped
            .entry(DispatchKey {
                layer: key.layer,
                operation: key.operation,
                path: key.path,
            })
            .or_insert(0) += count;
    }
    grouped
        .into_iter()
        .map(|(key, count)| DispatchCount {
            layer: key.layer,
            operation: key.operation,
            path: key.path,
            count,
        })
        .collect()
}

pub fn snapshot() -> Vec<DispatchCount> {
    TRACE_STATE.with(|state| {
        collect_dispatch_counts(
            state
                .borrow()
                .counts
                .iter()
                .map(|(key, count)| (*key, *count)),
        )
    })
}

pub fn take() -> Vec<DispatchCount> {
    TRACE_STATE.with(|state| {
        let mut state = state.borrow_mut();
        collect_dispatch_counts(state.counts.drain())
    })
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
    fn dispatch_snapshot_merges_equal_labels_from_distinct_static_storage() {
        let label = || -> &'static str { Box::leak(String::from("same").into_boxed_str()) };
        let first = (label(), label(), label());
        let second = (label(), label(), label());
        assert!(!std::ptr::eq(first.0, second.0));

        reset();
        with_recording(|| {
            record(first.0, first.1, first.2);
            record(second.0, second.1, second.2);
        });

        assert_eq!(
            take(),
            vec![DispatchCount {
                layer: "same",
                operation: "same",
                path: "same",
                count: 2,
            }]
        );
    }

    #[test]
    fn sign_refinement_trace_records_attempt_and_decision_precision_buckets() {
        reset();
        with_recording(|| {
            record_sign_refinement_precision("sign_until_attempt_precision", 0);
            record_sign_refinement_precision("sign_until_attempt_precision", -16);
            record_sign_refinement_precision("sign_until_decision_precision", -16);
        });

        let snapshot = take_trace();
        assert_eq!(
            snapshot.path_count("computable", "sign_until_attempt_precision", "p>=0"),
            1
        );
        assert_eq!(
            snapshot.path_count("computable", "sign_until_attempt_precision", "-32<=p<0"),
            1
        );
        assert_eq!(
            snapshot.path_count("computable", "sign_until_decision_precision", "-32<=p<0"),
            1
        );
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
        assert_eq!(
            stats.gcds,
            stats.gcd_operand_widths.both_u64
                + stats.gcd_operand_widths.mixed_u64_u128
                + stats.gcd_operand_widths.both_u128
                + stats.gcd_operand_widths.arbitrary_precision
        );
        assert!(stats.peak_operand_bits > 0);
    }

    #[test]
    fn known_dyadic_quotient_trace_distinguishes_word_and_wide_results() {
        use crate::Rational;

        let numerator = Rational::fraction(5, 32).unwrap();
        let denominator = Rational::fraction(11, 64).unwrap();
        let large_scale_denominator = Rational::from_bigint_fraction(
            num::BigInt::from(3_u8),
            num::BigUint::from(1_u8) << 200,
        )
        .unwrap();
        let wide_origin = Rational::from_bigint(num::BigInt::from(1_u8) << 220_usize);
        let delta = Rational::fraction(35, 64).unwrap();
        let affine_numerator = Rational::fraction(55, 32).unwrap();
        let affine_denominator = Rational::new(45);

        reset();
        with_recording(|| {
            Rational::quotient_known_dyadic(&numerator, &denominator).unwrap();
            Rational::quotient_known_dyadic(&Rational::one(), &large_scale_denominator).unwrap();
            Rational::parameterized_point2_known_dyadic(
                [&wide_origin, &wide_origin],
                [&delta, &delta],
                &affine_numerator,
                &affine_denominator,
            )
            .unwrap();
        });

        let snapshot = take_trace();
        assert_eq!(
            snapshot.path_count("rational", "div", "known-dyadic-word-cross-cancel"),
            2
        );
        assert_eq!(
            snapshot.path_count("rational", "div", "known-dyadic-cross-cancel"),
            1
        );
        assert_eq!(
            snapshot.path_count("rational", "div", "known-dyadic-affine-factor-cross-cancel"),
            2
        );
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

    #[test]
    fn correlation_summary_groups_exact_geometry_ladder_events() {
        reset();
        with_recording(|| {
            record("hyperlimit", "orient2d", "exact-rational-kernel");
            record("hyperlattice_matrix", "query", "matrix4-structural-facts");
            record("real", "exact_set_facts", "scan");
            record("real", "approximation", "cached-f64-hit");
            record("real", "domain_facts", "sqrt-domain-positive");
            record("real", "to_f64_lossy", "lossy-export");
            record("real", "certified_sign", "bounded-refinement");
            record("hyperlimit", "resolve_real_sign", "unknown-fallback");
            record_rational_temporary();
            record_rational_power_of_two_common_factor(3);
        });

        let summary = snapshot_trace().correlation_summary();
        assert_eq!(summary.dispatch_events, 8);
        assert_eq!(summary.predicate_events, 2);
        assert_eq!(summary.linear_algebra_events, 1);
        assert_eq!(summary.object_fact_events, 1);
        assert_eq!(summary.scalar_fact_events, 3);
        assert_eq!(summary.detailed_fact_events, 2);
        assert_eq!(summary.unknown_fact_events, 1);
        assert_eq!(summary.exact_rational_kind_events, 2);
        assert_eq!(summary.sign_or_zero_query_events, 3);
        assert_eq!(summary.exact_reducer_events, 2);
        assert_eq!(summary.approximation_events, 2);
        assert_eq!(summary.approximation_start_events, 1);
        assert_eq!(summary.approximation_cache_events, 1);
        assert_eq!(summary.refinement_events, 1);
        assert_eq!(summary.predicate_decision_stage_events, 2);
        assert_eq!(summary.fallback_or_abort_events, 1);
        assert_eq!(summary.rational_temporaries, 1);
        assert_eq!(summary.rational_reductions, 0);
        assert_eq!(summary.rational_gcds, 0);
    }

    #[test]
    fn concurrent_recording_scopes_keep_independent_snapshots() {
        let threads = (0..8)
            .map(|index| {
                std::thread::spawn(move || {
                    reset();
                    with_recording(|| {
                        for _ in 0..=index {
                            record("real", "parallel", "thread-local");
                            record_rational_temporary();
                        }
                    });
                    take_trace()
                })
            })
            .collect::<Vec<_>>();

        for (index, thread) in threads.into_iter().enumerate() {
            let snapshot = thread.join().expect("recording thread panicked");
            assert_eq!(snapshot.layer_count("real"), index as u64 + 1);
            assert_eq!(snapshot.rational.temporary_rationals, index as u64 + 1);
        }
    }
}
