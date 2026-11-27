/// Collect untimed thread-local samples into process-global report rows.
///
/// The mutexes protect stored rows, not the entire recording transaction. Each sample
/// resets/drains its calling thread's Hyperreal counters, discarding any outer trace;
/// other threads' work is not merged. Scalar caches are retained. Panics propagate,
/// restoring the recorder's prior enable flag but leaving its counters undrained.
/// There is no work/memory budget or concurrency-wide run boundary. Trace enablement
/// persists once begun; report writing does not stop it.
#[cfg(feature = "hyperreal-dispatch-trace")]
mod enabled {
    use std::collections::BTreeMap;
    use std::fs;
    use std::hint::black_box;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::{Mutex, OnceLock};

    /// Lazily allocated named snapshots; later nonempty samples replace equal names.
    static ROWS: OnceLock<Mutex<BTreeMap<String, hyperreal::dispatch_trace::TraceSnapshot>>> =
        OnceLock::new();
    /// Append-only profile rows between begin_trace_run calls; duplicate labels remain.
    static MATRIX_PROFILE_ROWS: OnceLock<Mutex<Vec<MatrixProfileRow>>> = OnceLock::new();
    /// Process-wide relaxed enable flag, initially false and never reset by this module.
    static TRACE_RUN: AtomicBool = AtomicBool::new(false);
    /// First-initialized substring list; empty means all names and later set attempts fail.
    ///
    /// trace_cases can initialize this to empty even while tracing is inactive, causing
    /// later begin_trace_run filter requests to be ignored for the rest of the process.
    static TRACE_FILTER: OnceLock<Vec<String>> = OnceLock::new();

    /// Label-derived operation estimates for one matrix sample, not executed scalar totals.
    ///
    /// Recognized aggregate hooks expand to nominal add/multiply counts; nested labels
    /// can also contribute, so counts need not partition actual arithmetic or CPU work.
    /// Unrecognized labels and layers are ignored. Default initializes all counters to zero.
    #[derive(Clone, Debug, Default)]
    struct ScalarOpProfile {
        /// Recognized direct/cached additions plus nominal aggregate additions.
        adds: u64,
        /// Recognized direct/cached subtractions; aggregate signs are not reconstructed.
        subs: u64,
        /// Recognized direct/cached multiplications plus nominal aggregate multiplications.
        muls: u64,
        /// Counts scalar_op paths beginning div-; not all divisions inside dependencies.
        divs: u64,
        /// Counts inverse-owned and inverse-ref scalar_method labels.
        inverses: u64,
        /// Recognized active dot/linear hooks and signed-product-sum2-prefixed events.
        signed_product_sums: u64,
    }

    /// Owned matrix sample metadata and diagnostic counts for normalized reporting.
    ///
    /// The caller supplies calls; this record does not prove how many operations ran or
    /// whether they succeeded. Zero calls and empty trace data remain admissible.
    #[derive(Clone, Debug)]
    struct MatrixProfileRow {
        /// Caller label used as the primary lexicographic report sort key.
        matrix: String,
        /// Caller operation label used as the secondary report sort key.
        kernel: String,
        /// Caller representation label, with from-f64 and rational ranked before others.
        input: String,
        /// Declared call count cast from usize, used as a display denominator without validation.
        calls: u64,
        /// Nominal operation counts derived from selected hyperlattice labels.
        scalar_ops: ScalarOpProfile,
        /// Raw rational counters and peak operand-bit statistic for the same sample window.
        rational_stats: hyperreal::dispatch_trace::RationalTraceStats,
        /// Sum of events whose operation label contains constructor, across all layers.
        constructor_events: u64,
    }

    /// Get the lazily initialized process-global snapshot map without locking it.
    fn rows() -> &'static Mutex<BTreeMap<String, hyperreal::dispatch_trace::TraceSnapshot>> {
        ROWS.get_or_init(|| Mutex::new(BTreeMap::new()))
    }

    /// Get the lazily initialized process-global profile list without locking it.
    fn matrix_profile_rows() -> &'static Mutex<Vec<MatrixProfileRow>> {
        MATRIX_PROFILE_ROWS.get_or_init(|| Mutex::new(Vec::new()))
    }

    /// Match any stored fragment as a case-sensitive substring of the entire row name.
    ///
    /// Initializes an unset filter to empty, which accepts every name permanently unless
    /// the process restarts. Does not consult TRACE_RUN; no regex or token boundaries apply.
    fn should_capture(name: &str) -> bool {
        let filters = TRACE_FILTER.get_or_init(Vec::new);
        if filters.is_empty() {
            return true;
        }

        filters.iter().any(|filter| name.contains(filter))
    }

    /// Clear both row stores, attempt to install a filter, and enable future samples.
    ///
    /// Splits a supplied filter on commas, trims fragments and removes empties. None or
    /// only empty fragments mean all names. OnceLock accepts only the first initialization;
    /// later calls clear rows but cannot replace even an empty existing filter. The ignored
    /// set result is not reported. Poisoned store locks panic. Separate clears and relaxed
    /// flag storage do not form an atomic session boundary for concurrent callers.
    pub fn begin_trace_run(filter: Option<&str>) {
        rows()
            .lock()
            .expect("dispatch trace rows lock poisoned")
            .clear();
        matrix_profile_rows()
            .lock()
            .expect("matrix profile rows lock poisoned")
            .clear();
        if let Some(raw_filter) = filter {
            let filters = raw_filter
                .split(',')
                .map(str::trim)
                .filter(|filter| !filter.is_empty())
                .map(str::to_owned)
                .collect::<Vec<String>>();
            let _ = TRACE_FILTER.set(filters);
        } else {
            let _ = TRACE_FILTER.set(Vec::new());
        }
        TRACE_RUN.store(true, Ordering::Relaxed);
    }

    /// Run one enabled, name-matching closure and retain a nonempty dispatch snapshot.
    ///
    /// Inactive calls return before converting name or invoking sample. Active matching
    /// calls reset current-thread counters, run sample once, then drain dispatch and rational
    /// statistics. A snapshot with no dispatch labels is discarded even if it contains
    /// rational-only work; an existing row with that name remains unchanged. Otherwise the
    /// new snapshot replaces that key. No completion/value/count assertion is made.
    /// Poisoned storage locks or sample panics propagate; no scalar caches are reset.
    pub fn trace_row(name: impl Into<String>, sample: impl FnOnce()) {
        if !TRACE_RUN.load(Ordering::Relaxed) {
            return;
        }

        let name = name.into();
        if !should_capture(&name) {
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
            .insert(name, trace);
    }

    /// Record one aggregate pass over the supplied cases, not one row per case.
    ///
    /// Converts name and consults should_capture before trace_row checks run enablement,
    /// so even an inactive call can initialize the once-only filter. Matching active calls
    /// black-box each borrowed case and invoke sample in slice order. Empty cases produce
    /// an empty dispatch snapshot and are omitted. Inputs are not cloned or reconstructed,
    /// and there is no per-case work limit, cold-cache reset or result validation.
    pub fn trace_cases<T>(name: impl Into<String>, cases: &[T], mut sample: impl FnMut(&T)) {
        let name = name.into();
        if !should_capture(&name) {
            return;
        }

        trace_row(name, || {
            for case in cases {
                sample(black_box(case));
            }
        });
    }

    /// Estimate nominal arithmetic from recognized labels in exactly the hyperlattice layer.
    ///
    /// Counts scalar_op add-/sub-/mul-/div- prefixes and cached add/sub/mul labels directly.
    /// Dot3 and linear-combination3 contribute three multiplies and two adds; dimension four
    /// contributes four and three. Active variants also count a signed product sum. Affine
    /// specialized hooks contribute n multiplies and n adds. Inverse method labels count
    /// once; signed-product-sum2 prefixes count once, with single-term expanding to one
    /// multiply and sparse-two to two multiplies plus one add. Other labels contribute zero.
    ///
    /// These are heuristic expansions, not measured instruction counts or evidence that
    /// each nominal operation ran. Nested events can double-count work. Ordinary u64
    /// addition/multiplication has no saturation or explicit overflow handling.
    fn scalar_op_profile(counts: &[hyperreal::dispatch_trace::DispatchCount]) -> ScalarOpProfile {
        let mut profile = ScalarOpProfile::default();
        for count in counts.iter().filter(|count| count.layer == "hyperlattice") {
            match (count.operation, count.path) {
                ("scalar_op", path) if path.starts_with("add-") => profile.adds += count.count,
                ("scalar_op", path) if path.starts_with("sub-") => profile.subs += count.count,
                ("scalar_op", path) if path.starts_with("mul-") => profile.muls += count.count,
                ("scalar_op", path) if path.starts_with("div-") => profile.divs += count.count,
                ("scalar_fast_path", "add-cached") => profile.adds += count.count,
                ("scalar_fast_path", "sub-cached") => profile.subs += count.count,
                ("scalar_fast_path", "mul-cached") => profile.muls += count.count,
                ("scalar_fast_path", "dot3-active" | "dot3-same-active") => {
                    profile.muls += count.count * 3;
                    profile.adds += count.count * 2;
                    profile.signed_product_sums += count.count;
                }
                ("scalar_fast_path", "dot4-active" | "dot4-same-active") => {
                    profile.muls += count.count * 4;
                    profile.adds += count.count * 3;
                    profile.signed_product_sums += count.count;
                }
                ("scalar_fast_path", "linear-combination3-specialized") => {
                    profile.muls += count.count * 3;
                    profile.adds += count.count * 2;
                }
                ("scalar_fast_path", "linear-combination3-active") => {
                    profile.muls += count.count * 3;
                    profile.adds += count.count * 2;
                    profile.signed_product_sums += count.count;
                }
                ("scalar_fast_path", "linear-combination4-specialized") => {
                    profile.muls += count.count * 4;
                    profile.adds += count.count * 3;
                }
                ("scalar_fast_path", "linear-combination4-active") => {
                    profile.muls += count.count * 4;
                    profile.adds += count.count * 3;
                    profile.signed_product_sums += count.count;
                }
                ("scalar_fast_path", "affine-combination3-specialized") => {
                    profile.muls += count.count * 3;
                    profile.adds += count.count * 3;
                }
                ("scalar_fast_path", "affine-combination4-specialized") => {
                    profile.muls += count.count * 4;
                    profile.adds += count.count * 4;
                }
                ("scalar_method", "inverse-owned" | "inverse-ref") => {
                    profile.inverses += count.count;
                }
                ("scalar_fast_path", path) if path.starts_with("signed-product-sum2-") => {
                    profile.signed_product_sums += count.count;
                    match path {
                        "signed-product-sum2-single-term" => profile.muls += count.count,
                        "signed-product-sum2-sparse-two" => {
                            profile.muls += count.count * 2;
                            profile.adds += count.count;
                        }
                        _ => {}
                    }
                }
                _ => {}
            }
        }
        profile
    }

    /// Sum counts with constructor anywhere in the operation label, regardless of layer.
    ///
    /// This textual heuristic does not measure allocations or uniquely created objects.
    /// The ordinary u64 sum has no saturation or explicit overflow outcome.
    fn constructor_events(counts: &[hyperreal::dispatch_trace::DispatchCount]) -> u64 {
        counts
            .iter()
            .filter(|count| count.operation.contains("constructor"))
            .map(|count| count.count)
            .sum()
    }

    /// Append one enabled, matching sample's aggregate profile, including empty samples.
    ///
    /// Filters the composite name matrix_profile_row::matrix/kernel/input. Once admitted,
    /// runs sample exactly once between reset and take_trace on the current thread; calls
    /// is only caller-supplied normalization metadata, not a repetition count. Zero calls
    /// are retained and display as n/a. The raw dispatch list is reduced to selected scalar
    /// and constructor counts, while rational statistics are retained. Rows are appended
    /// without deduplication or result assertions; they do not enter the ordinary row map.
    /// Poisoned locks and sample panics propagate. Fixture work outside sample is excluded.
    pub fn trace_matrix_profile_row(
        matrix: &'static str,
        kernel: &'static str,
        input: &'static str,
        calls: usize,
        sample: impl FnOnce(),
    ) {
        if !TRACE_RUN.load(Ordering::Relaxed) {
            return;
        }

        let name = format!("matrix_profile_row::{matrix}/{kernel}/{input}");
        if !should_capture(&name) {
            return;
        }

        hyperreal::dispatch_trace::reset();
        hyperreal::dispatch_trace::with_recording(|| {
            sample();
        });
        let trace = hyperreal::dispatch_trace::take_trace();
        let row = MatrixProfileRow {
            matrix: matrix.to_owned(),
            kernel: kernel.to_owned(),
            input: input.to_owned(),
            calls: calls as u64,
            scalar_ops: scalar_op_profile(&trace.dispatch),
            rational_stats: trace.rational,
            constructor_events: constructor_events(&trace.dispatch),
        };

        matrix_profile_rows()
            .lock()
            .expect("matrix profile rows lock poisoned")
            .push(row);
    }

    /// Format a lossy binary64 ratio of count to declared calls, or n/a for zero calls.
    ///
    /// Rounds to zero decimals when within 0.005 of an integer; otherwise uses two decimals
    /// at values at least ten and three below ten. Casting u64 can lose integer precision.
    /// This is display normalization, not a confidence interval or an exact rational mean.
    fn per_call(value: u64, calls: u64) -> String {
        if calls == 0 {
            return "n/a".to_owned();
        }
        let value = value as f64 / calls as f64;
        if (value - value.round()).abs() < 0.005 {
            format!("{value:.0}")
        } else if value >= 10.0 {
            format!("{value:.2}")
        } else {
            format!("{value:.3}")
        }
    }

    /// Format five rational reduction-factor buckets per declared call using per_call.
    ///
    /// Reports none, power-of-two, small (up to eight bits), medium (up to 64 bits) and
    /// large counts. These are recorded factor categories, not scalar magnitudes, a memory
    /// histogram or normalized probabilities; zero calls produces n/a for every bucket.
    /// Zero/unit factors enter none; powers of two are separated before width buckets.
    fn common_factor_distribution(
        stats: hyperreal::dispatch_trace::RationalTraceStats,
        calls: u64,
    ) -> String {
        let buckets = stats.common_factors;
        format!(
            "none={}, pow2={}, <=8b={}, <=64b={}, >64b={}",
            per_call(buckets.none, calls),
            per_call(buckets.power_of_two, calls),
            per_call(buckets.small, calls),
            per_call(buckets.medium, calls),
            per_call(buckets.large, calls)
        )
    }

    /// Rank from-f64 first, rational second and every other input label equally last.
    fn input_rank(input: &str) -> u8 {
        match input {
            "from-f64" => 0,
            "rational" => 1,
            _ => 2,
        }
    }

    /// Overwrite working-directory dispatch_trace.md from currently retained rows.
    ///
    /// Presence of HYPERLATTICE_SKIP_BENCHMARK_REPORTS returns before locking. Empty stores
    /// also return, leaving any existing report untouched. Both locks remain held during
    /// rendering and writing; poisoned locks panic. Profile rows are cloned and stably
    /// sorted by matrix, kernel and input_rank, preserving insertion order on equal keys.
    /// The ordinary snapshot map supplies lexicographic names and recorder-sorted paths.
    ///
    /// Per-call profile values divide by caller-supplied calls using lossy display ratios.
    /// Temps/ctors adds rational temporary and constructor-event counts, which may overlap;
    /// it is not allocation count. Peak operand bits is copied without division. Correlation
    /// categories can overlap and do not certify outcomes. The generated prose's scalar
    /// counts describe heuristic hook expansion, not observed scalar operations.
    ///
    /// Labels are not escaped for Markdown. No revision, actual feature configuration,
    /// validation result or timing samples are recorded. Writes have no atomic replacement,
    /// locking against other processes or parent-directory creation; I/O errors log without
    /// returning failure. Does not clear stores, disable sampling or parse command-line flags.
    pub fn write_report() {
        if std::env::var_os("HYPERLATTICE_SKIP_BENCHMARK_REPORTS").is_some() {
            return;
        }

        let rows = rows().lock().expect("dispatch trace rows lock poisoned");
        let matrix_rows = matrix_profile_rows()
            .lock()
            .expect("matrix profile rows lock poisoned");
        if rows.is_empty() && matrix_rows.is_empty() {
            return;
        }

        let mut out = String::new();
        out.push_str("# Hyperreal Dispatch Trace\n\n");
        out.push_str("Generated by running `cargo bench --bench mathbench --features hyperreal-dispatch-trace -- --write-dispatch-trace-md`. Each row is sampled outside Criterion's measured loop. Use the default benchmark build for timing comparisons; the trace feature intentionally compiles diagnostic hooks into hyperreal.\n\n");

        if !matrix_rows.is_empty() {
            out.push_str("## Matrix Kernel Profile\n\n");
            out.push_str("Per-call values are one unmeasured sample pass divided by the sampled calls. `dot3`/`dot4` and `linear`/`affine` fast paths are expanded into their scalar add/mul counts. `Signed product sums` counts the short determinant/cofactor polynomial hooks that delay exact-rational canonicalization. Common-factor buckets are rational reduction events per call; `pow2` is the dyadic shift-only path.\n\n");
            out.push_str("| Matrix | Kernel | Input | Calls | Real +/call | Real -/call | Real */call | Real div/call | Real inv/call | Signed product sums/call | Rational reductions/call | GCDs/call | Temps/ctors/call | Peak operand bits | Common factors/call |\n");
            out.push_str("| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |\n");
            let mut rows = matrix_rows.clone();
            rows.sort_by(|left, right| {
                (
                    left.matrix.as_str(),
                    left.kernel.as_str(),
                    input_rank(left.input.as_str()),
                )
                    .cmp(&(
                        right.matrix.as_str(),
                        right.kernel.as_str(),
                        input_rank(right.input.as_str()),
                    ))
            });
            for row in rows {
                let temp_events = row.constructor_events + row.rational_stats.temporary_rationals;
                out.push_str(&format!(
                    "| `{}` | `{}` | `{}` | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |\n",
                    row.matrix,
                    row.kernel,
                    row.input,
                    row.calls,
                    per_call(row.scalar_ops.adds, row.calls),
                    per_call(row.scalar_ops.subs, row.calls),
                    per_call(row.scalar_ops.muls, row.calls),
                    per_call(row.scalar_ops.divs, row.calls),
                    per_call(row.scalar_ops.inverses, row.calls),
                    per_call(row.scalar_ops.signed_product_sums, row.calls),
                    per_call(row.rational_stats.reductions, row.calls),
                    per_call(row.rational_stats.gcds, row.calls),
                    per_call(temp_events, row.calls),
                    row.rational_stats.peak_operand_bits,
                    common_factor_distribution(row.rational_stats, row.calls),
                ));
            }
            out.push('\n');
        }

        if !rows.is_empty() {
            out.push_str("## Correlation Summary\n\n");
            out.push_str("This table groups raw cross-stack labels into Yap-aligned diagnostic buckets. It is a report aid, not a correctness certificate; raw paths below remain the detailed source.\n\n");
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

            out.push_str("## Dispatch Counts\n\n");
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
        }

        if let Err(error) = fs::write("dispatch_trace.md", out) {
            eprintln!("failed to update dispatch_trace.md: {error}");
        }
    }
}

/// Feature-disabled trace shims preserving call signatures without recording.
///
/// Argument expressions and closure construction still occur at call sites; the shims
/// never invoke sample or convert names through `Into<String>`. Owned arguments/captures
/// are dropped normally. No filters, counters, stores or report files are created.
#[cfg(not(feature = "hyperreal-dispatch-trace"))]
mod enabled {
    /// Ignore the requested filter when tracing is not compiled in.
    pub fn begin_trace_run(_filter: Option<&str>) {}

    /// Drop name and sample without conversion or invocation in the disabled build.
    pub fn trace_row(name: impl Into<String>, sample: impl FnOnce()) {
        let _ = name;
        let _ = sample;
    }

    /// Ignore the borrowed case slice and drop the name/closure without calling sample.
    pub fn trace_cases<T>(name: impl Into<String>, cases: &[T], sample: impl FnMut(&T)) {
        let _ = name;
        let _ = cases;
        let _ = sample;
    }

    /// Ignore profile metadata and drop sample without running it in the disabled build.
    pub fn trace_matrix_profile_row(
        matrix: &'static str,
        kernel: &'static str,
        input: &'static str,
        calls: usize,
        sample: impl FnOnce(),
    ) {
        let _ = matrix;
        let _ = kernel;
        let _ = input;
        let _ = calls;
        let _ = sample;
    }

    /// Return without inspecting environment, trace stores or files when tracing is disabled.
    pub fn write_report() {}
}

use enabled::{
    trace_cases as trace_dispatch_cases, trace_matrix_profile_row, trace_row as trace_dispatch_row,
};

/// Forward run setup to the feature-selected implementation; disabled builds do nothing.
fn begin_dispatch_trace_run(filter: Option<&str>) {
    enabled::begin_trace_run(filter);
}

/// Forward report writing to the feature-selected implementation, preserving its gate/errors.
fn write_dispatch_trace_report() {
    enabled::write_report();
}
