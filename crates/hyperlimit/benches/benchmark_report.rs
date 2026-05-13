use std::cmp::Ordering;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use serde_json::Value;

#[derive(Debug)]
pub struct ReportSummary {
    pub path: PathBuf,
    pub rows: usize,
}

#[derive(Debug)]
struct Row {
    full_id: String,
    group_id: String,
    function_id: String,
    value_str: String,
    mean_ns: f64,
    mean_low_ns: f64,
    mean_high_ns: f64,
    median_ns: f64,
    change_mean: Option<f64>,
}

pub fn write_benchmarks_md() -> io::Result<ReportSummary> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let criterion_dir = root.join("target").join("criterion");
    let mut rows = Vec::new();

    if criterion_dir.exists() {
        collect_rows(&criterion_dir, &mut rows)?;
    }
    rows.sort_by(compare_rows);

    let path = root.join("benchmarks.md");
    fs::write(&path, render_markdown(&rows))?;

    Ok(ReportSummary {
        path,
        rows: rows.len(),
    })
}

fn collect_rows(dir: &Path, rows: &mut Vec<Row>) -> io::Result<()> {
    for entry in fs::read_dir(dir)? {
        let entry = entry?;
        let path = entry.path();
        if path.is_dir() {
            if path.file_name().and_then(|name| name.to_str()) == Some("new") {
                if let Some(row) = read_row(&path)? {
                    rows.push(row);
                }
            } else {
                collect_rows(&path, rows)?;
            }
        }
    }
    Ok(())
}

fn read_row(new_dir: &Path) -> io::Result<Option<Row>> {
    let estimates = match read_json(&new_dir.join("estimates.json"))? {
        Some(value) => value,
        None => return Ok(None),
    };
    let benchmark = match read_json(&new_dir.join("benchmark.json"))? {
        Some(value) => value,
        None => return Ok(None),
    };

    let Some(full_id) = string_field(&benchmark, "full_id") else {
        return Ok(None);
    };

    let group_id = string_field(&benchmark, "group_id").unwrap_or_else(|| infer_part(&full_id, 0));
    let function_id =
        string_field(&benchmark, "function_id").unwrap_or_else(|| infer_part(&full_id, 1));
    let value_str =
        string_field(&benchmark, "value_str").unwrap_or_else(|| infer_part(&full_id, 2));

    let Some(mean_ns) = point_estimate(&estimates, "mean") else {
        return Ok(None);
    };
    let mean_low_ns = confidence_bound(&estimates, "mean", "lower_bound").unwrap_or(mean_ns);
    let mean_high_ns = confidence_bound(&estimates, "mean", "upper_bound").unwrap_or(mean_ns);
    let median_ns = point_estimate(&estimates, "median").unwrap_or(mean_ns);

    let change_mean = new_dir
        .parent()
        .map(|bench_dir| bench_dir.join("change").join("estimates.json"))
        .and_then(|path| read_json(&path).ok().flatten())
        .and_then(|value| point_estimate(&value, "mean"));

    Ok(Some(Row {
        full_id,
        group_id,
        function_id,
        value_str,
        mean_ns,
        mean_low_ns,
        mean_high_ns,
        median_ns,
        change_mean,
    }))
}

fn read_json(path: &Path) -> io::Result<Option<Value>> {
    if !path.exists() {
        return Ok(None);
    }

    let text = fs::read_to_string(path)?;
    serde_json::from_str(&text).map(Some).map_err(|error| {
        io::Error::new(
            io::ErrorKind::InvalidData,
            format!("failed to parse {}: {error}", path.display()),
        )
    })
}

fn string_field(value: &Value, field: &str) -> Option<String> {
    value.get(field)?.as_str().map(ToOwned::to_owned)
}

fn point_estimate(value: &Value, section: &str) -> Option<f64> {
    value.get(section)?.get("point_estimate")?.as_f64()
}

fn confidence_bound(value: &Value, section: &str, bound: &str) -> Option<f64> {
    value
        .get(section)?
        .get("confidence_interval")?
        .get(bound)?
        .as_f64()
}

fn infer_part(full_id: &str, index: usize) -> String {
    full_id.split('/').nth(index).unwrap_or("").to_owned()
}

fn compare_rows(left: &Row, right: &Row) -> Ordering {
    left.group_id
        .cmp(&right.group_id)
        .then_with(|| left.function_id.cmp(&right.function_id))
        .then_with(|| left.value_str.cmp(&right.value_str))
        .then_with(|| left.full_id.cmp(&right.full_id))
}

fn render_markdown(rows: &[Row]) -> String {
    let mut md = String::new();
    md.push_str("# Benchmarks\n\n");
    md.push_str("This file is generated from Criterion output under `target/criterion`.\n\n");
    md.push_str(&format!(
        "Generated at Unix timestamp `{}`.\n\n",
        generated_timestamp()
    ));
    md.push_str("## Commands\n\n");
    md.push_str("Run the default benchmark suite and update this file:\n\n");
    md.push_str("```sh\ncargo bench --bench predicates\n```\n\n");
    md.push_str("Run dispatch tracing separately and update `dispatch_trace.md`:\n\n");
    md.push_str(
        "```sh\ncargo bench --bench predicates --features dispatch-trace,hyperlattice -- --write-dispatch-trace-md\n```\n\n",
    );
    md.push_str("Regenerate this file from existing Criterion output:\n\n");
    md.push_str("```sh\ncargo run --example write_benchmarks_md\n```\n\n");
    md.push_str("Run optional scalar representation benchmarks:\n\n");
    md.push_str("```sh\nRUSTFLAGS='-Ctarget-cpu=haswell' cargo bench --bench predicates --features hyperreal,hyperlattice,interval\n```\n\n");
    md.push_str(
        "Open Criterion's detailed HTML report at `target/criterion/report/index.html`.\n\n",
    );

    md.push_str("## Latest Results\n\n");
    if rows.is_empty() {
        md.push_str(
            "No Criterion results were found. Run `cargo bench --bench predicates` first.\n",
        );
        return md;
    }

    md.push_str(
        "| Predicate | Representation | Workload | Mean | 95% CI | Median | Change vs Baseline |\n",
    );
    md.push_str("| --- | --- | --- | ---: | ---: | ---: | ---: |\n");
    for row in rows {
        md.push_str(&format!(
            "| `{}` | `{}` | `{}` | {} | {} - {} | {} | {} |\n",
            row.group_id,
            row.function_id,
            row.value_str,
            format_duration(row.mean_ns),
            format_duration(row.mean_low_ns),
            format_duration(row.mean_high_ns),
            format_duration(row.median_ns),
            format_change(row.change_mean),
        ));
    }

    md
}

fn generated_timestamp() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_secs())
        .unwrap_or(0)
}

fn format_duration(ns: f64) -> String {
    if ns < 1_000.0 {
        format!("{ns:.2} ns")
    } else if ns < 1_000_000.0 {
        format!("{:.2} us", ns / 1_000.0)
    } else if ns < 1_000_000_000.0 {
        format!("{:.2} ms", ns / 1_000_000.0)
    } else {
        format!("{:.2} s", ns / 1_000_000_000.0)
    }
}

fn format_change(change: Option<f64>) -> String {
    match change {
        Some(change) => format!("{:+.2}%", change * 100.0),
        None => "-".to_owned(),
    }
}
