#[path = "../benches/benchmark_report.rs"]
mod benchmark_report;

fn main() -> std::io::Result<()> {
    let summary = benchmark_report::write_benchmarks_md()?;
    println!(
        "updated {} from {} Criterion benchmark results",
        summary.path.display(),
        summary.rows
    );
    Ok(())
}
