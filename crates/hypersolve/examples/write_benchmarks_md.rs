#[path = "../benches/support/benchmark_report.rs"]
#[allow(dead_code)]
mod benchmark_report;

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
