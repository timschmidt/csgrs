//! Small dependency-free harness shared by the Rust benchmark binaries.
//! Callers supply completed-work measurements and any correctness assertions.
//! Temperature is a recorded label, not an instruction to flush caches or restart
//! a process. Environment filters are read per selection, while sample/iteration
//! settings are captured in [`Config`]. Output labels are not CSV-escaped.

use std::hint::black_box;
use std::time::{Duration, Instant};

/// Column order for each sample row; printing the header is an explicit caller action.
pub const CSV_HEADER: &str = "engine,temperature,suite,benchmark,case,sample,iterations,elapsed_ns,work_units,output_size,checksum";

#[derive(Clone)]
/// Captured repetition counts and display labels for dependency-free benchmark loops.
/// Selection gates remain live environment queries; this record imposes no resource budget.
pub struct Config {
    /// Ordinary timed samples, clamped to at least one.
    samples: usize,
    /// Untimed ordinary repetitions of the full iteration loop.
    warmup: usize,
    /// Timed samples for suites selected by `sample_plan`, clamped to at least one.
    stress_samples: usize,
    /// Untimed repetitions for those suites, permitting zero.
    stress_warmup: usize,
    /// Saturating multiplier for caller iteration counts when no override is set.
    iteration_scale: usize,
    /// Optional total iterations per sample, clamped to at least one and bypassing scale.
    iterations_override: Option<usize>,
    /// Saturating offset added to emitted zero-based sample numbers.
    sample_offset: usize,
    /// Authored `cold`/`warm` CSV label; no cache management is implied.
    temperature: String,
}

impl Config {
    /// Reads `CSGRS_BENCH_*` repetition settings, defaulting malformed numeric values.
    /// Defaults: 10 samples, 2 warmups, 1 stress sample, 0 stress warmups, scale 1,
    /// no iteration override and offset 0. Samples/scale/override clamp to one.
    /// Prints warnings when dangerous/stress gates are enabled. Temperature
    /// defaults to `warm`; a different readable value besides `cold` panics.
    pub fn from_env() -> Self {
        if dangerous_enabled() {
            eprintln!(
                "WARNING: dangerous benchmark workloads are enabled and may exhaust system memory"
            );
        }
        if stress_enabled() {
            eprintln!(
                "WARNING: stress benchmark workloads are enabled and may consume tens of GiB"
            );
        }
        Self {
            samples: env_usize("CSGRS_BENCH_SAMPLES", 10).max(1),
            warmup: env_usize("CSGRS_BENCH_WARMUP", 2),
            stress_samples: env_usize("CSGRS_BENCH_STRESS_SAMPLES", 1).max(1),
            stress_warmup: env_usize("CSGRS_BENCH_STRESS_WARMUP", 0),
            iteration_scale: env_usize("CSGRS_BENCH_SCALE", 1).max(1),
            iterations_override: env_optional_usize("CSGRS_BENCH_ITERATIONS")
                .map(|value| value.max(1)),
            sample_offset: env_usize("CSGRS_BENCH_SAMPLE_OFFSET", 0),
            temperature: benchmark_temperature(),
        }
    }

    /// Runs the standard measured loop with engine label `csgrs`; see [`Self::run_engine`].
    pub fn run<F>(&self, suite: &str, benchmark: &str, case: &str, iterations: usize, mut f: F)
    where
        F: FnMut() -> Measurement,
    {
        self.run_engine("csgrs", suite, benchmark, case, iterations, &mut f);
    }

    // This shared module is compiled independently into benchmark binaries
    // that do not all use the published-workload selection helper.
    #[allow(dead_code)]
    /// Queries current suite gates and substring filters, without executing a workload.
    /// The config's stored counts and temperature do not affect selection.
    pub fn selects(&self, suite: &str, benchmark: &str, case: &str) -> bool {
        selected(suite, benchmark, case)
    }

    /// Run a workload whose closure supplies the precise timed interval.
    ///
    /// This is useful when correctness validation must run for every sample
    /// but is outside an externally published timing boundary.
    /// Warmups execute the same closure but discard its duration/measurement.
    /// Timed rows sum caller-provided durations with saturation; the harness does
    /// not verify their boundary or wall-clock accuracy. Iteration scaling,
    /// selection and dispatch recording otherwise follow [`Self::run_engine`].
    #[allow(dead_code)]
    pub fn run_with_elapsed<F>(
        &self,
        suite: &str,
        benchmark: &str,
        case: &str,
        iterations: usize,
        mut f: F,
    ) where
        F: FnMut() -> (Duration, Measurement),
    {
        if !selected(suite, benchmark, case) {
            return;
        }
        let iterations = self
            .iterations_override
            .unwrap_or_else(|| iterations.saturating_mul(self.iteration_scale).max(1));
        let (samples, warmup) = self.sample_plan(suite);
        let mut measure = || {
            for _ in 0..warmup {
                for _ in 0..iterations {
                    black_box(f());
                }
            }
            for sample in 0..samples {
                let mut elapsed = Duration::ZERO;
                let mut measurement = Measurement::default();
                for _ in 0..iterations {
                    let (operation_elapsed, operation_measurement) = black_box(f());
                    elapsed = elapsed.saturating_add(operation_elapsed);
                    measurement = measurement.combine(operation_measurement);
                }
                emit(
                    "csgrs",
                    &self.temperature,
                    suite,
                    benchmark,
                    case,
                    self.sample_offset.saturating_add(sample),
                    iterations,
                    elapsed,
                    measurement,
                );
            }
        };

        #[cfg(feature = "dispatch-trace")]
        {
            hyperreal::dispatch_trace::reset();
            hyperreal::dispatch_trace::with_recording(|| {
                hyperreal::dispatch_trace::record(
                    "csgrs-benchmark",
                    "entry",
                    "recorded-workload",
                );
                measure();
            });
            print_dispatch_trace(suite, benchmark, case);
        }

        #[cfg(not(feature = "dispatch-trace"))]
        measure();
    }

    /// Runs selected warmups then emits one CSV row per sample under an arbitrary engine label.
    /// Uses the override or saturating scaled iteration count, with at least one
    /// iteration. Times closure calls, measurement combination and loop overhead;
    /// fixture setup outside the closure and CSV printing are outside the timer.
    /// Closure results pass through `black_box`; correctness/receipt validation
    /// remains the caller's responsibility. Panics stop the run without a typed row.
    ///
    /// With `dispatch-trace`, resets tracing once for this selected workload and
    /// records warmups plus all samples, including a synthetic entry event. Trace
    /// reporting follows the loop. Neither trace nor temperature resets geometry caches.
    pub fn run_engine<F>(
        &self,
        engine: &str,
        suite: &str,
        benchmark: &str,
        case: &str,
        iterations: usize,
        mut f: F,
    ) where
        F: FnMut() -> Measurement,
    {
        if !selected(suite, benchmark, case) {
            return;
        }

        let iterations = self
            .iterations_override
            .unwrap_or_else(|| iterations.saturating_mul(self.iteration_scale).max(1));
        let (samples, warmup) = self.sample_plan(suite);
        let mut measure = || {
            for _ in 0..warmup {
                for _ in 0..iterations {
                    black_box(f());
                }
            }

            for sample in 0..samples {
                let start = Instant::now();
                let mut measurement = Measurement::default();
                for _ in 0..iterations {
                    measurement = measurement.combine(black_box(f()));
                }
                emit(
                    engine,
                    &self.temperature,
                    suite,
                    benchmark,
                    case,
                    self.sample_offset.saturating_add(sample),
                    iterations,
                    start.elapsed(),
                    measurement,
                );
            }
        };

        #[cfg(feature = "dispatch-trace")]
        {
            hyperreal::dispatch_trace::reset();
            hyperreal::dispatch_trace::with_recording(|| {
                hyperreal::dispatch_trace::record(
                    "csgrs-benchmark",
                    "entry",
                    "recorded-workload",
                );
                measure();
            });
            print_dispatch_trace(suite, benchmark, case);
        }

        #[cfg(not(feature = "dispatch-trace"))]
        measure();
    }

    /// Uses stress counts for corpus, stress, dangerous, competitive-large/-yeahright/-full
    /// and published-solidean suites; all other names use ordinary counts.
    fn sample_plan(&self, suite: &str) -> (usize, usize) {
        if matches!(
            suite,
            "corpus"
                | "stress"
                | "dangerous"
                | "competitive-large"
                | "competitive-yeahright"
                | "competitive-full"
                | "published-solidean"
        ) {
            (self.stress_samples, self.stress_warmup)
        } else {
            (self.samples, self.warmup)
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
/// Caller-defined per-iteration counters, accumulated across a timed sample.
/// The default is all zeros; no field alone proves geometry or workload completion.
pub struct Measurement {
    /// Caller-defined completed work count; aggregate sums saturate.
    pub work_units: u64,
    /// Caller-defined output size/count; aggregate sums saturate.
    pub output_size: u64,
    /// Caller-defined fingerprint; aggregate sums wrap modulo `2^64`.
    pub checksum: u64,
}

impl Measurement {
    /// Stores counters verbatim without validating their interpretation or relationship.
    pub const fn new(work_units: u64, output_size: u64, checksum: u64) -> Self {
        Self {
            work_units,
            output_size,
            checksum,
        }
    }

    /// Saturates work/output totals and wraps checksum addition modulo `2^64`.
    const fn combine(self, other: Self) -> Self {
        Self {
            work_units: self.work_units.saturating_add(other.work_units),
            output_size: self.output_size.saturating_add(other.output_size),
            checksum: self.checksum.wrapping_add(other.checksum),
        }
    }
}

/// Prints the fixed CSV header to standard output; run methods do not print it themselves.
pub fn print_header() {
    println!("{CSV_HEADER}");
}

#[cfg(feature = "dispatch-trace")]
/// Takes the current trace, asserts at least one event and prints summaries to stderr.
/// The run methods add an entry event, so this check alone does not establish
/// that native geometry dispatch occurred or that any expected count matched.
fn print_dispatch_trace(suite: &str, benchmark: &str, case: &str) {
    let trace = hyperreal::dispatch_trace::take_trace();
    let correlation = trace.correlation_summary();
    assert!(
        correlation.dispatch_events > 0,
        "{suite}/{benchmark}/{case} did not emit a benchmark path trace"
    );
    eprintln!("dispatch benchmark: {suite}/{benchmark}/{case}");
    eprintln!("dispatch correlation: {correlation:?}");
    for summary in trace.operation_summaries() {
        eprintln!(
            "dispatch operation: {}/{}/{}",
            summary.layer, summary.operation, summary.count
        );
    }
    for summary in trace.dispatch {
        eprintln!(
            "dispatch path: {}/{}/{}/{}",
            summary.layer, summary.operation, summary.path, summary.count
        );
    }
}

/// Parses a readable environment value as `usize`, falling back on absence or parse failure.
fn env_usize(name: &str, default: usize) -> usize {
    std::env::var(name)
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(default)
}

/// Returns a parsed `usize`, or `None` for absent, non-Unicode or malformed input.
fn env_optional_usize(name: &str) -> Option<usize> {
    std::env::var(name).ok()?.parse().ok()
}

/// Accepts exact `cold`/`warm` labels, defaults unreadable/absent values to `warm`,
/// and panics on other readable values. Does not alter execution temperature.
fn benchmark_temperature() -> String {
    match std::env::var("CSGRS_BENCH_TEMPERATURE").as_deref() {
        Ok("cold") => "cold".into(),
        Ok("warm") | Err(_) => "warm".into(),
        Ok(other) => panic!("CSGRS_BENCH_TEMPERATURE must be 'cold' or 'warm', got {other:?}"),
    }
}

/// Applies corpus-Boolean skip, dangerous opt-in and stress opt-in/skip gates in order.
/// A readable skip value other than `0` enables skipping; opt-in requires `1`.
/// Then admits any trimmed nonempty comma-separated substring of `suite/benchmark/case`.
/// Absent/unreadable filter selects all gated workloads; an empty filter selects none.
fn selected(suite: &str, benchmark: &str, case: &str) -> bool {
    if suite == "corpus"
        && benchmark == "boolean_all"
        && std::env::var("CSGRS_BENCH_SKIP_CORPUS_BOOLEANS").is_ok_and(|value| value != "0")
    {
        return false;
    }
    if suite == "dangerous" && !dangerous_enabled() {
        return false;
    }
    if suite == "stress"
        && (!stress_enabled()
            || std::env::var("CSGRS_BENCH_SKIP_STRESS").is_ok_and(|value| value != "0"))
    {
        return false;
    }
    let Ok(filter) = std::env::var("CSGRS_BENCH_FILTER") else {
        return true;
    };
    let qualified = format!("{suite}/{benchmark}/{case}");
    filter
        .split(',')
        .map(str::trim)
        .filter(|part| !part.is_empty())
        .any(|part| qualified.contains(part))
}

/// Enables the dangerous suite only for readable `CSGRS_BENCH_ENABLE_DANGEROUS=1`.
fn dangerous_enabled() -> bool {
    std::env::var("CSGRS_BENCH_ENABLE_DANGEROUS").is_ok_and(|value| value == "1")
}

/// Enables the stress suite only for readable `CSGRS_BENCH_ENABLE_STRESS=1`.
fn stress_enabled() -> bool {
    std::env::var("CSGRS_BENCH_ENABLE_STRESS").is_ok_and(|value| value == "1")
}

#[allow(clippy::too_many_arguments)]
/// Prints one unescaped CSV row with nanosecond duration and already-aggregated counters.
/// String labels must avoid delimiters/newlines for well-formed CSV; no validation is added.
fn emit(
    engine: &str,
    temperature: &str,
    suite: &str,
    benchmark: &str,
    case: &str,
    sample: usize,
    iterations: usize,
    elapsed: Duration,
    measurement: Measurement,
) {
    println!(
        "{engine},{temperature},{suite},{benchmark},{case},{sample},{iterations},{},{},{},{}",
        elapsed.as_nanos(),
        measurement.work_units,
        measurement.output_size,
        measurement.checksum
    );
}
