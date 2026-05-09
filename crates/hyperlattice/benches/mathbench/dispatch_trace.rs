#[cfg(feature = "hyperreal-dispatch-trace")]
mod enabled {
    use std::collections::BTreeMap;
    use std::fs;
    use std::hint::black_box;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::{Mutex, OnceLock};

    static ROWS: OnceLock<Mutex<BTreeMap<String, Vec<hyperreal::dispatch_trace::DispatchCount>>>> =
        OnceLock::new();
    static MATRIX_PROFILE_ROWS: OnceLock<Mutex<Vec<MatrixProfileRow>>> = OnceLock::new();
    static TRACE_RUN: AtomicBool = AtomicBool::new(false);
    static TRACE_FILTER: OnceLock<Vec<String>> = OnceLock::new();

    #[derive(Clone, Debug, Default)]
    struct ScalarOpProfile {
        adds: u64,
        subs: u64,
        muls: u64,
        divs: u64,
        inverses: u64,
    }

    #[derive(Clone, Debug)]
    struct MatrixProfileRow {
        matrix: String,
        kernel: String,
        input: String,
        calls: u64,
        scalar_ops: ScalarOpProfile,
        rational_stats: hyperreal::dispatch_trace::RationalTraceStats,
        constructor_events: u64,
    }

    fn rows() -> &'static Mutex<BTreeMap<String, Vec<hyperreal::dispatch_trace::DispatchCount>>> {
        ROWS.get_or_init(|| Mutex::new(BTreeMap::new()))
    }

    fn matrix_profile_rows() -> &'static Mutex<Vec<MatrixProfileRow>> {
        MATRIX_PROFILE_ROWS.get_or_init(|| Mutex::new(Vec::new()))
    }

    fn should_capture(name: &str) -> bool {
        let filters = TRACE_FILTER.get_or_init(Vec::new);
        if filters.is_empty() {
            return true;
        }

        filters.iter().any(|filter| name.contains(filter))
    }

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
        let counts = hyperreal::dispatch_trace::take();
        if counts.is_empty() {
            return;
        }

        rows()
            .lock()
            .expect("dispatch trace rows lock poisoned")
            .insert(name.into(), counts);
    }

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

    fn scalar_op_profile(counts: &[hyperreal::dispatch_trace::DispatchCount]) -> ScalarOpProfile {
        let mut profile = ScalarOpProfile::default();
        for count in counts
            .iter()
            .filter(|count| count.layer == "realistic_blas")
        {
            match (count.operation, count.path) {
                ("scalar_op", path) if path.starts_with("add-") => profile.adds += count.count,
                ("scalar_op", path) if path.starts_with("sub-") => profile.subs += count.count,
                ("scalar_op", path) if path.starts_with("mul-") => profile.muls += count.count,
                ("scalar_op", path) if path.starts_with("div-") => profile.divs += count.count,
                ("scalar_fast_path", "add-cached") => profile.adds += count.count,
                ("scalar_fast_path", "sub-cached") => profile.subs += count.count,
                ("scalar_fast_path", "mul-cached") => profile.muls += count.count,
                ("scalar_fast_path", "dot3-backend") => {
                    // Dot-product fast paths intentionally hide their scalar
                    // adds/muls from the generic operator trace. Expand them
                    // here so matrix kernels remain comparable across inputs.
                    profile.muls += count.count * 3;
                    profile.adds += count.count * 2;
                }
                ("scalar_fast_path", "dot4-backend") => {
                    profile.muls += count.count * 4;
                    profile.adds += count.count * 3;
                }
                ("scalar_fast_path", "linear-combination3-specialized") => {
                    profile.muls += count.count * 3;
                    profile.adds += count.count * 2;
                }
                ("scalar_fast_path", "linear-combination4-specialized") => {
                    profile.muls += count.count * 4;
                    profile.adds += count.count * 3;
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
                _ => {}
            }
        }
        profile
    }

    fn constructor_events(counts: &[hyperreal::dispatch_trace::DispatchCount]) -> u64 {
        counts
            .iter()
            .filter(|count| count.operation.contains("constructor"))
            .map(|count| count.count)
            .sum()
    }

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
        let counts = hyperreal::dispatch_trace::take();
        let rational_stats = hyperreal::dispatch_trace::take_rational_stats();
        let row = MatrixProfileRow {
            matrix: matrix.to_owned(),
            kernel: kernel.to_owned(),
            input: input.to_owned(),
            calls: calls as u64,
            scalar_ops: scalar_op_profile(&counts),
            rational_stats,
            constructor_events: constructor_events(&counts),
        };

        matrix_profile_rows()
            .lock()
            .expect("matrix profile rows lock poisoned")
            .push(row);
    }

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

    fn input_rank(input: &str) -> u8 {
        match input {
            "from-f64" => 0,
            "rational" => 1,
            _ => 2,
        }
    }

    pub fn write_report() {
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
            out.push_str("Per-call values are one unmeasured sample pass divided by the sampled calls. `dot3`/`dot4` and `linear`/`affine` fast paths are expanded into their scalar add/mul counts. Common-factor buckets are rational reduction events per call; `pow2` is the dyadic shift-only path.\n\n");
            out.push_str("| Matrix | Kernel | Input | Calls | Scalar +/call | Scalar -/call | Scalar */call | Scalar div/call | Scalar inv/call | Rational reductions/call | GCDs/call | Temps/ctors/call | Peak operand bits | Common factors/call |\n");
            out.push_str("| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |\n");
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
                    "| `{}` | `{}` | `{}` | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |\n",
                    row.matrix,
                    row.kernel,
                    row.input,
                    row.calls,
                    per_call(row.scalar_ops.adds, row.calls),
                    per_call(row.scalar_ops.subs, row.calls),
                    per_call(row.scalar_ops.muls, row.calls),
                    per_call(row.scalar_ops.divs, row.calls),
                    per_call(row.scalar_ops.inverses, row.calls),
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
            out.push_str("## Dispatch Counts\n\n");
            out.push_str("| Benchmark Row | Layer | Operation | Path | Count |\n");
            out.push_str("| --- | --- | --- | --- | ---: |\n");
            for (row, counts) in rows.iter() {
                for count in counts {
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

#[cfg(not(feature = "hyperreal-dispatch-trace"))]
mod enabled {
    pub fn begin_trace_run(_filter: Option<&str>) {}

    pub fn trace_row(name: impl Into<String>, sample: impl FnOnce()) {
        let _ = name;
        let _ = sample;
    }

    pub fn trace_cases<T>(name: impl Into<String>, cases: &[T], sample: impl FnMut(&T)) {
        let _ = name;
        let _ = cases;
        let _ = sample;
    }

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

    pub fn write_report() {}
}

use enabled::{
    trace_cases as trace_dispatch_cases, trace_matrix_profile_row, trace_row as trace_dispatch_row,
};

fn begin_dispatch_trace_run(filter: Option<&str>) {
    enabled::begin_trace_run(filter);
}

fn write_dispatch_trace_report() {
    enabled::write_report();
}
