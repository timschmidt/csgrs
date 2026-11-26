//! Regenerate Hyperlimit's benchmark Markdown from stored Criterion files.
//!
//! From the workspace root: `cargo run -p hyperlimit --example write_benchmarks_md`.
//! Does not execute timing workloads. Directly calls the writer, bypassing the
//! automatic benchmark callback's environment/argv skip gate. Destination is this
//! crate's benchmarks.md; source selection and replacement limits are documented
//! in the shared report module. Failures propagate through main's I/O Result.

/// Shared implementation also compiled into the predicate and retained-fuzz benchmark binaries.
#[path = "../benches/benchmark_report.rs"]
#[allow(dead_code)]
mod benchmark_report;

/// Rewrite the generated report and print destination and stored-data counts.
///
/// Propagates collection/catalog/write errors. Counts are report inventory, not
/// freshly completed workloads; the writer does not honor the automatic skip gate.
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
