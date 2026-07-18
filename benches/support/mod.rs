//! Small dependency-free benchmark harness shared by the benchmark binaries.

use std::hint::black_box;
use std::time::{Duration, Instant};

pub const CSV_HEADER: &str = "engine,temperature,suite,benchmark,case,sample,iterations,elapsed_ns,work_units,output_size,checksum";

#[derive(Clone)]
pub struct Config {
    samples: usize,
    warmup: usize,
    iteration_scale: usize,
    iterations_override: Option<usize>,
    sample_offset: usize,
    temperature: String,
}

impl Config {
    pub fn from_env() -> Self {
        Self {
            samples: env_usize("CSGRS_BENCH_SAMPLES", 10).max(1),
            warmup: env_usize("CSGRS_BENCH_WARMUP", 2),
            iteration_scale: env_usize("CSGRS_BENCH_SCALE", 1).max(1),
            iterations_override: env_optional_usize("CSGRS_BENCH_ITERATIONS")
                .map(|value| value.max(1)),
            sample_offset: env_usize("CSGRS_BENCH_SAMPLE_OFFSET", 0),
            temperature: benchmark_temperature(),
        }
    }

    pub fn run<F>(&self, suite: &str, benchmark: &str, case: &str, iterations: usize, mut f: F)
    where
        F: FnMut() -> Measurement,
    {
        if !selected(suite, benchmark, case) {
            return;
        }

        let iterations = self
            .iterations_override
            .unwrap_or_else(|| iterations.saturating_mul(self.iteration_scale).max(1));
        let mut measure = || {
            for _ in 0..self.warmup {
                for _ in 0..iterations {
                    black_box(f());
                }
            }

            for sample in 0..self.samples {
                let start = Instant::now();
                let mut measurement = Measurement::default();
                for _ in 0..iterations {
                    measurement = measurement.combine(black_box(f()));
                }
                emit(
                    "csgrs",
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
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Measurement {
    pub work_units: u64,
    pub output_size: u64,
    pub checksum: u64,
}

impl Measurement {
    pub const fn new(work_units: u64, output_size: u64, checksum: u64) -> Self {
        Self {
            work_units,
            output_size,
            checksum,
        }
    }

    const fn combine(self, other: Self) -> Self {
        Self {
            work_units: self.work_units.saturating_add(other.work_units),
            output_size: self.output_size.saturating_add(other.output_size),
            checksum: self.checksum.wrapping_add(other.checksum),
        }
    }
}

pub fn print_header() {
    println!("{CSV_HEADER}");
}

#[cfg(feature = "dispatch-trace")]
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

fn env_usize(name: &str, default: usize) -> usize {
    std::env::var(name)
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(default)
}

fn env_optional_usize(name: &str) -> Option<usize> {
    std::env::var(name).ok()?.parse().ok()
}

fn benchmark_temperature() -> String {
    match std::env::var("CSGRS_BENCH_TEMPERATURE").as_deref() {
        Ok("cold") => "cold".into(),
        Ok("warm") | Err(_) => "warm".into(),
        Ok(other) => panic!("CSGRS_BENCH_TEMPERATURE must be 'cold' or 'warm', got {other:?}"),
    }
}

fn selected(suite: &str, benchmark: &str, case: &str) -> bool {
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

#[allow(clippy::too_many_arguments)]
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
