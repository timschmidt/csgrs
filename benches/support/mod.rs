//! Small dependency-free benchmark harness shared by the benchmark binaries.

use std::hint::black_box;
use std::time::{Duration, Instant};

pub const CSV_HEADER: &str =
    "engine,suite,benchmark,case,sample,iterations,elapsed_ns,work_units,output_size,checksum";

#[derive(Clone, Copy)]
pub struct Config {
    samples: usize,
    warmup: usize,
    iteration_scale: usize,
}

impl Config {
    pub fn from_env() -> Self {
        Self {
            samples: env_usize("CSGRS_BENCH_SAMPLES", 10).max(1),
            warmup: env_usize("CSGRS_BENCH_WARMUP", 2),
            iteration_scale: env_usize("CSGRS_BENCH_SCALE", 1).max(1),
        }
    }

    pub fn run<F>(&self, suite: &str, benchmark: &str, case: &str, iterations: usize, mut f: F)
    where
        F: FnMut() -> Measurement,
    {
        if !selected(suite, benchmark, case) {
            return;
        }

        let iterations = iterations.saturating_mul(self.iteration_scale).max(1);
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
                suite,
                benchmark,
                case,
                sample,
                iterations,
                start.elapsed(),
                measurement,
            );
        }
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

fn env_usize(name: &str, default: usize) -> usize {
    std::env::var(name)
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(default)
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
    suite: &str,
    benchmark: &str,
    case: &str,
    sample: usize,
    iterations: usize,
    elapsed: Duration,
    measurement: Measurement,
) {
    println!(
        "{engine},{suite},{benchmark},{case},{sample},{iterations},{},{},{},{}",
        elapsed.as_nanos(),
        measurement.work_units,
        measurement.output_size,
        measurement.checksum
    );
}
