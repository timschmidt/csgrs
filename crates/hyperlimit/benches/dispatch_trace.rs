//! Optional dispatch sampling for the Hyperlimit predicate benchmark executable.
//!
//! With dispatch-trace, records calling-thread events outside Criterion timing
//! loops and retains named snapshots for a Markdown report. The global row map
//! persists between run-start and write calls; workers need their own tracing
//! arrangement. Without the feature, callbacks are not invoked and no report is
//! written. Samples are diagnostic observations, not correctness, certainty, or
//! completed-workload evidence. Timing comparisons require non-tracing builds.

#![allow(dead_code, unused_imports)]

/// Feature-enabled sampling and Markdown reporting over Hyperreal's thread-local trace state.
///
/// The run switch and retained row map are process-global; recording counters are
/// per-thread. This helper neither merges worker traces nor validates sample results.
#[cfg(feature = "dispatch-trace")]
mod enabled {
    use std::collections::BTreeMap;
    use std::fs;
    use std::hint::black_box;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::{Mutex, OnceLock};

    /// Lazily allocated global row map, ordered by name and retained for the process lifetime.
    ///
    /// Successful nonempty samples replace equal names. Starting/writing a run does
    /// not clear this map, and there is no row-count or retained-memory limit.
    static ROWS: OnceLock<Mutex<BTreeMap<String, hyperreal::dispatch_trace::TraceSnapshot>>> =
        OnceLock::new();
    /// Sticky process-global switch permitting sampling after begin_trace_run; initially false.
    static TRACE_RUN: AtomicBool = AtomicBool::new(false);

    /// Return the lazily initialized global map mutex without clearing or locking its rows.
    fn rows() -> &'static Mutex<BTreeMap<String, hyperreal::dispatch_trace::TraceSnapshot>> {
        ROWS.get_or_init(|| Mutex::new(BTreeMap::new()))
    }

    /// Enable subsequent samples without clearing earlier rows or resetting counters.
    ///
    /// Uses a relaxed global flag; there is no matching end-run operation.
    pub fn begin_trace_run() {
        TRACE_RUN.store(true, Ordering::Relaxed);
    }

    /// Sample one closure once on the calling thread and retain a nonempty dispatch snapshot.
    ///
    /// Before enablement, does not invoke the closure or convert the name. Once enabled,
    /// resets this thread's counters, records the closure, and drains the resulting
    /// dispatch and rational counters. Worker threads are not collected automatically.
    /// An empty dispatch vector discards the snapshot, including any rational-only
    /// counts, and leaves an older same-name row untouched. Otherwise converts the
    /// name and replaces that row in the global map.
    ///
    /// Not an isolated nested recorder: reset/drain can destroy an enclosing recording's
    /// counters, though the scalar recording guard restores its prior enabled flag.
    /// Captured values and name expressions are still created by the caller before
    /// entry, even when sampling is skipped. No correctness or certainty check occurs.
    ///
    /// # Panics
    ///
    /// Sample/name-conversion panics propagate; a poisoned row mutex panics. If the
    /// sample panics, no row is inserted and the post-sample drain is not reached.
    pub fn trace_row(name: impl Into<String>, sample: impl FnOnce()) {
        if !TRACE_RUN.load(Ordering::Relaxed) {
            return;
        }

        hyperreal::dispatch_trace::reset();
        hyperreal::dispatch_trace::with_recording(|| {
            sample();
        });
        let trace = hyperreal::dispatch_trace::take_trace();
        if trace.dispatch.is_empty() {
            return;
        }

        rows()
            .lock()
            .expect("dispatch trace rows lock poisoned")
            .insert(name.into(), trace);
    }

    /// Record one aggregate row by visiting the case slice in order and black-boxing each reference.
    ///
    /// Delegates to trace_row, inheriting its thread scope, reset/drain, duplicate-row,
    /// and panic behavior. No per-case normalization, warm-up, or outcome validation
    /// occurs. Before enablement the callback is never invoked; an empty slice normally
    /// produces no dispatch row.
    pub fn trace_cases<T>(name: impl Into<String>, cases: &[T], mut sample: impl FnMut(&T)) {
        trace_row(name, || {
            for case in cases {
                sample(black_box(case));
            }
        });
    }

    /// Overwrite dispatch_trace.md in the current working directory from all retained rows.
    ///
    /// An empty map leaves any existing file untouched. Holds the global map mutex
    /// through formatting and file I/O, emits rows in name order, and keeps them afterward.
    /// Correlation categories may overlap; operation summaries group labels, and rational
    /// peak operand bits are not a heap-memory measurement. No timing or completion
    /// certificate is produced. Row/label text is inserted without Markdown escaping.
    ///
    /// The direct write is not atomic; errors are printed to stderr without being
    /// returned, and a failed write can leave partial output. Calling this function does
    /// not require the run flag to be enabled.
    ///
    /// # Panics
    ///
    /// Panics on a poisoned row mutex. Formatting while holding the lock can poison it
    /// if it unwinds; there is no recovery path here.
    pub fn write_report() {
        let rows = rows().lock().expect("dispatch trace rows lock poisoned");
        if rows.is_empty() {
            return;
        }

        let mut out = String::new();
        out.push_str("# Hyperlimit Dispatch Trace\n\n");
        out.push_str("Generated by running `cargo bench --bench predicates --features dispatch-trace -- --write-dispatch-trace-md`. Each row is sampled outside Criterion's measured loop. Use a non-tracing benchmark build for timing comparisons; the trace feature intentionally compiles diagnostic hooks into hyperlimit and hyperreal.\n\n");
        out.push_str("## Correlation Summary\n\n");
        out.push_str("| Benchmark Row | Dispatch | Predicate | Linear Algebra | Object Facts | Scalar Facts | Detailed Facts | Unknown Facts | Rational Kinds | Sign/Zero Queries | Exact Reducers | Approximation | Approx Starts | Approx Cache | Refinement | Predicate Stages | Cache | Fallback/Abort | Rational Temps | Rational Reductions | Rational GCDs |\n");
        out.push_str("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n");
        for (row, trace) in rows.iter() {
            let summary = trace.correlation_summary();
            out.push_str(&format!(
                "| `{}` | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |\n",
                row,
                summary.dispatch_events,
                summary.predicate_events,
                summary.linear_algebra_events,
                summary.object_fact_events,
                summary.scalar_fact_events,
                summary.detailed_fact_events,
                summary.unknown_fact_events,
                summary.exact_rational_kind_events,
                summary.sign_or_zero_query_events,
                summary.exact_reducer_events,
                summary.approximation_events,
                summary.approximation_start_events,
                summary.approximation_cache_events,
                summary.refinement_events,
                summary.predicate_decision_stage_events,
                summary.cache_events,
                summary.fallback_or_abort_events,
                summary.rational_temporaries,
                summary.rational_reductions,
                summary.rational_gcds
            ));
        }
        out.push('\n');
        out.push_str("## Dispatch Paths\n\n");
        out.push_str("| Benchmark Row | Layer | Operation | Path | Count |\n");
        out.push_str("| --- | --- | --- | --- | ---: |\n");
        for (row, trace) in rows.iter() {
            for count in &trace.dispatch {
                out.push_str(&format!(
                    "| `{}` | `{}` | `{}` | `{}` | {} |\n",
                    row, count.layer, count.operation, count.path, count.count
                ));
            }
        }
        out.push_str("\n## Operation Summary\n\n");
        out.push_str("| Benchmark Row | Layer | Operation | Count |\n");
        out.push_str("| --- | --- | --- | ---: |\n");
        for (row, trace) in rows.iter() {
            for summary in trace.operation_summaries() {
                out.push_str(&format!(
                    "| `{}` | `{}` | `{}` | {} |\n",
                    row, summary.layer, summary.operation, summary.count
                ));
            }
        }
        out.push_str("\n## Rational Reducer Summary\n\n");
        out.push_str("| Benchmark Row | Temporaries | Reductions | GCDs | Peak Operand Bits |\n");
        out.push_str("| --- | ---: | ---: | ---: | ---: |\n");
        for (row, trace) in rows.iter() {
            out.push_str(&format!(
                "| `{}` | {} | {} | {} | {} |\n",
                row,
                trace.rational.temporary_rationals,
                trace.rational.reductions,
                trace.rational.gcds,
                trace.rational.peak_operand_bits
            ));
        }

        if let Err(error) = fs::write("dispatch_trace.md", out) {
            eprintln!("failed to update dispatch_trace.md: {error}");
        }
    }
}

/// Disabled-feature shims preserving benchmark call sites without executing trace samples.
///
/// Arguments and captured values are still constructed and dropped normally; no
/// recording state or report file is created by these shims.
#[cfg(not(feature = "dispatch-trace"))]
mod enabled {
    /// No-op run starter when dispatch-trace is disabled; creates no state.
    pub fn begin_trace_run() {}

    /// Accept a parallel build's row-sampling arguments without invoking the closure or converting the name.
    ///
    /// Owned arguments are dropped normally. This shim exists only when parallel is
    /// enabled and dispatch-trace is disabled.
    #[cfg(feature = "parallel")]
    pub fn trace_row(name: impl Into<String>, sample: impl FnOnce()) {
        let _ = name;
        let _ = sample;
    }

    /// Accept a case-sampling call without visiting cases, invoking the callback, or converting the name.
    ///
    /// Owned arguments are dropped normally; caller-side argument construction still occurs.
    pub fn trace_cases<T>(name: impl Into<String>, cases: &[T], sample: impl FnMut(&T)) {
        let _ = name;
        let _ = cases;
        let _ = sample;
    }

    /// No-op report writer when dispatch-trace is disabled; leaves any existing file untouched.
    pub fn write_report() {}
}

/// Case-sampling alias selecting the enabled recorder or disabled no-op implementation.
pub(crate) use enabled::trace_cases as trace_dispatch_cases;
/// Row-sampling alias available with parallel, using a no-op if dispatch-trace is absent.
///
/// Availability does not imply automatic capture of parallel worker events.
#[cfg(feature = "parallel")]
pub(crate) use enabled::trace_row as trace_dispatch_row;

/// Enable feature-backed sampling, or do nothing without dispatch-trace.
///
/// The enabled path leaves retained rows intact and has no matching end-run reset.
pub(crate) fn begin_dispatch_trace_run() {
    enabled::begin_trace_run();
}

/// Write retained trace rows through the feature-backed reporter, or do nothing without tracing.
///
/// Inherits current-directory output, retained-state, logging, and panic behavior
/// from the enabled report writer; no result or completion receipt is returned.
pub(crate) fn write_dispatch_trace_report() {
    enabled::write_report();
}
