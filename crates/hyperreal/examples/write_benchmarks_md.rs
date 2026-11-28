//! Regenerate Hyperreal's benchmarks.md from stored Criterion results and its manifest.
//!
//! This command performs file I/O but runs no benchmarks. The destination is the
//! crate directory compiled into CARGO_MANIFEST_DIR. Criterion input resolution
//! prefers CRITERION_HOME, then CARGO_TARGET_DIR, then executable-path discovery.
//! The shared writer replaces/appends its marked report section; a failed read of
//! an existing report falls back to a fresh heading before writing. Writes are
//! not atomic. This direct command does not honor the benchmark callback's
//! HYPERREAL_SKIP_BENCHMARK_REPORTS switch. Counts describe discovered records and
//! registered suites, not successful execution of every benchmark.

/// Share the benchmark-report implementation with timing targets.
/// The included module owns input parsing, suite discovery, section replacement
/// and filesystem errors; this facade adds no validation or timing measurements.
#[path = "../benches/support/benchmark_report.rs"]
#[allow(dead_code)]
mod benchmark_report;

/// Write the shared report and print its path, row count, compared-row count
/// and suite count. Propagate the writer's I/O errors to process termination; a
/// stdout failure can panic after the report has already been changed. Missing
/// Criterion data can yield zero rows without making report generation fail.
fn main() -> std::io::Result<()> {
    let summary = benchmark_report::write_benchmarks_md()?;
    println!(
        "updated {} from {} Criterion rows, {} comparisons, and {} benchmark suites",
        summary.path.display(),
        summary.rows,
        summary.comparisons,
        summary.suites,
    );
    Ok(())
}
