//! Regenerate Hyperlattice's benchmark Markdown from stored Criterion results.
//!
//! From the workspace root: `cargo run -p hyperlattice --example write_benchmarks_md`.
//! Does not execute timing workloads. Calls the writer directly, bypassing its
//! automatic callback's environment/argv skip gate, including the skip variable.
//! Destination is this crate's benchmarks.md. See the shared module for input
//! selection, replacement, and non-atomic-write semantics. I/O errors propagate.

/// Shared writer also included by mathbench, regression_sentinels, and retained_fuzz.
#[path = "../benches/support/benchmark_report.rs"]
#[allow(dead_code)]
mod benchmark_report;

/// Write the generated section, then print its destination and stored-data inventory.
///
/// Propagates collection/catalog/write errors. Comparisons counts participating
/// rows, not groups or pairwise comparisons. Does not apply the automatic skip gate
/// or establish that the stored results are current, correct, or comparable.
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
