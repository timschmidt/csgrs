use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

use serde_json::Value;

const CRATE_TITLE: &str = "Hypertri";
const SKIP_ENV: &str = "HYPERTRI_SKIP_BENCHMARK_REPORTS";
const TIMING_COMMAND: &str = "cargo bench --features all-algorithms,runtime-select,f64-interop";
const BEGIN: &str = "<!-- BEGIN COMPLETE BENCHMARK REPORT -->";
const END: &str = "<!-- END COMPLETE BENCHMARK REPORT -->";

#[derive(Debug)]
pub struct ReportSummary {
    pub path: PathBuf,
    pub rows: usize,
    pub comparisons: usize,
    pub suites: usize,
}

#[derive(Debug)]
struct Row {
    full_id: String,
    group_id: String,
    function_id: String,
    value_str: String,
    throughput: String,
    mean_ns: f64,
    mean_low_ns: f64,
    mean_high_ns: f64,
    median_ns: f64,
    change_mean: Option<f64>,
}

#[derive(Debug)]
struct BenchSuite {
    name: String,
    required_features: Vec<String>,
    criterion: bool,
}

pub fn finish_benchmark_report(_: &mut criterion::Criterion) {
    if reports_disabled() {
        return;
    }
    match write_benchmarks_md() {
        Ok(summary) => eprintln!(
            "updated {} from {} Criterion rows, {} comparisons, and {} benchmark suites",
            summary.path.display(),
            summary.rows,
            summary.comparisons,
            summary.suites,
        ),
        Err(error) => eprintln!("failed to update benchmarks.md: {error}"),
    }
}

pub fn reports_disabled() -> bool {
    let args: Vec<_> = std::env::args().collect();
    std::env::var_os(SKIP_ENV).is_some()
        || !args.iter().any(|arg| arg == "--bench")
        || args
            .iter()
            .any(|arg| arg == "--test" || arg == "--list" || arg == "--help")
}

pub fn write_benchmarks_md() -> io::Result<ReportSummary> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let mut rows = collect_rows(&criterion_dir(&root))?;
    rows.sort_by(|left, right| left.full_id.cmp(&right.full_id));
    let suites = read_bench_suites(&root)?;
    let (section, comparisons) = render_section(&rows, &suites);
    let path = root.join("benchmarks.md");
    let current = fs::read_to_string(&path).unwrap_or_else(|_| {
        format!(
            "# {CRATE_TITLE} Benchmarks\n\nThis file is updated automatically by the benchmark binaries.\n"
        )
    });
    fs::write(&path, replace_section(&current, &section))?;

    Ok(ReportSummary {
        path,
        rows: rows.len(),
        comparisons,
        suites: suites.len(),
    })
}

fn criterion_dir(root: &Path) -> PathBuf {
    match std::env::var_os("CARGO_TARGET_DIR") {
        Some(path) if Path::new(&path).is_absolute() => PathBuf::from(path).join("criterion"),
        Some(path) => root.join(path).join("criterion"),
        None => root.join("target").join("criterion"),
    }
}

fn collect_rows(criterion_dir: &Path) -> io::Result<Vec<Row>> {
    let mut rows = BTreeMap::new();
    if criterion_dir.exists() {
        collect_rows_from(criterion_dir, &mut rows)?;
    }
    Ok(rows.into_values().collect())
}

fn collect_rows_from(dir: &Path, rows: &mut BTreeMap<String, Row>) -> io::Result<()> {
    for entry in fs::read_dir(dir)? {
        let path = entry?.path();
        if !path.is_dir() {
            continue;
        }
        if path.file_name().and_then(|name| name.to_str()) == Some("new") {
            if let Some(row) = read_row(&path)? {
                rows.insert(row.full_id.clone(), row);
            }
        } else {
            collect_rows_from(&path, rows)?;
        }
    }
    Ok(())
}

fn read_row(new_dir: &Path) -> io::Result<Option<Row>> {
    let Some(estimates) = read_json(&new_dir.join("estimates.json"))? else {
        return Ok(None);
    };
    let Some(benchmark) = read_json(&new_dir.join("benchmark.json"))? else {
        return Ok(None);
    };
    let Some(full_id) = string_field(&benchmark, "full_id") else {
        return Ok(None);
    };
    let Some(mean_ns) = point_estimate(&estimates, "mean") else {
        return Ok(None);
    };
    let mean_low_ns = confidence_bound(&estimates, "mean", "lower_bound").unwrap_or(mean_ns);
    let mean_high_ns = confidence_bound(&estimates, "mean", "upper_bound").unwrap_or(mean_ns);
    let change_mean = new_dir
        .parent()
        .map(|bench_dir| bench_dir.join("change").join("estimates.json"))
        .and_then(|path| read_json(&path).ok().flatten())
        .and_then(|value| point_estimate(&value, "mean"));

    Ok(Some(Row {
        group_id: string_field(&benchmark, "group_id").unwrap_or_default(),
        function_id: string_field(&benchmark, "function_id").unwrap_or_default(),
        value_str: string_field(&benchmark, "value_str").unwrap_or_default(),
        throughput: format_throughput(benchmark.get("throughput")),
        median_ns: point_estimate(&estimates, "median").unwrap_or(mean_ns),
        full_id,
        mean_ns,
        mean_low_ns,
        mean_high_ns,
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

fn format_throughput(value: Option<&Value>) -> String {
    let Some(object) = value.and_then(Value::as_object) else {
        return "-".to_owned();
    };
    let Some((unit, count)) = object.iter().next() else {
        return "-".to_owned();
    };
    let unit = match unit.as_str() {
        "Elements" => "elements",
        "Bytes" => "bytes",
        "BytesDecimal" => "decimal bytes",
        other => other,
    };
    format!("{} {unit}", count.as_u64().unwrap_or(0))
}

fn read_bench_suites(root: &Path) -> io::Result<Vec<BenchSuite>> {
    let manifest = fs::read_to_string(root.join("Cargo.toml"))?;
    let mut suites = Vec::new();
    let mut name: Option<String> = None;
    let mut required_features = Vec::new();
    let mut in_bench = false;

    for raw_line in manifest.lines() {
        let line = raw_line.split('#').next().unwrap_or("").trim();
        if line == "[[bench]]" {
            push_suite(root, &mut suites, name.take(), &mut required_features);
            in_bench = true;
        } else if line.starts_with('[') {
            push_suite(root, &mut suites, name.take(), &mut required_features);
            in_bench = false;
        } else if in_bench && name.is_none() {
            name = assignment_string(line, "name");
        } else if in_bench && line.starts_with("required-features") {
            required_features = quoted_values(line);
        }
    }
    push_suite(root, &mut suites, name, &mut required_features);
    suites.sort_by(|left, right| left.name.cmp(&right.name));
    Ok(suites)
}

fn push_suite(
    root: &Path,
    suites: &mut Vec<BenchSuite>,
    name: Option<String>,
    required_features: &mut Vec<String>,
) {
    let Some(name) = name else {
        required_features.clear();
        return;
    };
    let source = root.join("benches").join(format!("{name}.rs"));
    let criterion = fs::read_to_string(source)
        .map(|text| text.contains("criterion::") || text.contains("criterion_group!"))
        .unwrap_or(false);
    suites.push(BenchSuite {
        name,
        required_features: std::mem::take(required_features),
        criterion,
    });
}

fn assignment_string(line: &str, key: &str) -> Option<String> {
    let (left, right) = line.split_once('=')?;
    if left.trim() != key {
        return None;
    }
    right
        .trim()
        .strip_prefix('"')?
        .strip_suffix('"')
        .map(ToOwned::to_owned)
}

fn quoted_values(line: &str) -> Vec<String> {
    line.split('"')
        .skip(1)
        .step_by(2)
        .map(ToOwned::to_owned)
        .collect()
}

fn render_section(rows: &[Row], suites: &[BenchSuite]) -> (String, usize) {
    let comparisons = comparison_groups(rows);
    let mut out = String::from(BEGIN);
    out.push_str("\n## Complete generated benchmark report\n\n");
    out.push_str("Every registered benchmark target is catalogued below. Every Criterion result found under `target/criterion` is included without a name or implementation filter; non-Criterion targets write their own linked reports. Each timing binary refreshes this section after it runs.\n\n");
    out.push_str("Run the complete non-instrumented timing set with:\n\n```sh\n");
    out.push_str(TIMING_COMMAND);
    out.push_str("\n```\n\nRegenerate this Markdown from stored Criterion data without rerunning benchmarks:\n\n```sh\ncargo run --example write_benchmarks_md\n```\n\n");

    out.push_str("### Registered benchmark suites\n\n");
    out.push_str("| Target | Kind | Required features | Command | Generated report |\n");
    out.push_str("| --- | --- | --- | --- | --- |\n");
    for suite in suites {
        let features = if suite.required_features.is_empty() {
            "default".to_owned()
        } else {
            suite.required_features.join(", ")
        };
        let mut command = format!("cargo bench --bench {}", suite.name);
        if !suite.required_features.is_empty() {
            command.push_str(" --features ");
            command.push_str(&suite.required_features.join(","));
        }
        let (kind, report) = if suite.criterion {
            ("Criterion timing", "this file".to_owned())
        } else if suite.name.contains("dispatch_trace") {
            let report = if suite.name.starts_with("api_") {
                "api_dispatch_trace.md"
            } else {
                "dispatch_trace.md"
            };
            ("diagnostic", format!("[{report}]({report})"))
        } else {
            let report = format!("{}_benchmarks.md", suite.name);
            ("custom timing", format!("[{report}]({report})"))
        };
        out.push_str(&format!(
            "| `{}` | {kind} | `{}` | `{command}` | {report} |\n",
            markdown(&suite.name),
            markdown(&features),
        ));
    }

    out.push_str("\n### Comparative results\n\n");
    out.push_str("Rows sharing a Criterion group and input are compared when they expose distinct implementations. Ratios are elapsed time relative to the fastest stored row; they do not imply identical guarantees or output semantics.\n\n");
    if comparisons.is_empty() {
        out.push_str("No paired Criterion results are currently stored.\n");
    } else {
        out.push_str("| Group | Input | Implementation | Mean | Relative to fastest |\n");
        out.push_str("| --- | --- | --- | ---: | ---: |\n");
        for ((group, input), compared_rows) in &comparisons {
            let fastest = compared_rows
                .iter()
                .map(|row| row.mean_ns)
                .fold(f64::INFINITY, f64::min);
            for row in compared_rows {
                out.push_str(&format!(
                    "| `{}` | `{}` | `{}` | {} | {:.2}x |\n",
                    markdown(group),
                    markdown(if input.is_empty() { "-" } else { input }),
                    markdown(&row.function_id),
                    format_duration(row.mean_ns),
                    row.mean_ns / fastest,
                ));
            }
        }
    }

    out.push_str("\n### All Criterion results\n\n");
    if rows.is_empty() {
        out.push_str("No local Criterion estimates are stored yet. Running any timing target adds its results here without removing results from the other targets.\n");
    } else {
        out.push_str("| Benchmark | Mean | 95% CI | Median | Change vs baseline | Throughput |\n");
        out.push_str("| --- | ---: | ---: | ---: | ---: | ---: |\n");
        for row in rows {
            out.push_str(&format!(
                "| `{}` | {} | {} - {} | {} | {} | {} |\n",
                markdown(&row.full_id),
                format_duration(row.mean_ns),
                format_duration(row.mean_low_ns),
                format_duration(row.mean_high_ns),
                format_duration(row.median_ns),
                format_change(row.change_mean),
                row.throughput,
            ));
        }
    }
    out.push('\n');
    out.push_str(END);
    out.push('\n');
    (out, comparisons.values().map(Vec::len).sum())
}

fn comparison_groups(rows: &[Row]) -> BTreeMap<(String, String), Vec<&Row>> {
    let mut candidates: BTreeMap<(String, String), Vec<&Row>> = BTreeMap::new();
    for row in rows.iter().filter(|row| !row.function_id.is_empty()) {
        candidates
            .entry((row.group_id.clone(), row.value_str.clone()))
            .or_default()
            .push(row);
    }
    candidates.retain(|(group, input), group_rows| {
        let implementations = group_rows
            .iter()
            .map(|row| row.function_id.as_str())
            .collect::<BTreeSet<_>>();
        implementations.len() > 1
            && (!input.is_empty() || group.contains("competitive") || group.contains("comparison"))
    });
    for group_rows in candidates.values_mut() {
        group_rows.sort_by(|left, right| {
            left.mean_ns
                .total_cmp(&right.mean_ns)
                .then_with(|| left.function_id.cmp(&right.function_id))
        });
    }
    candidates
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
    change
        .map(|value| format!("{:+.2}%", value * 100.0))
        .unwrap_or_else(|| "-".to_owned())
}

fn markdown(value: &str) -> String {
    value.replace('|', "\\|").replace('`', "\\`")
}

fn replace_section(current: &str, section: &str) -> String {
    let Some(start) = current.find(BEGIN) else {
        return format!("{}\n\n{}", current.trim_end(), section);
    };
    let Some(relative_end) = current[start..].find(END) else {
        return format!("{}\n\n{}", current[..start].trim_end(), section);
    };
    let end = start + relative_end + END.len();
    format!(
        "{}\n\n{}\n\n{}",
        current[..start].trim_end(),
        section.trim_end(),
        current[end..].trim_start(),
    )
    .trim_end()
    .to_owned()
        + "\n"
}

#[cfg(test)]
mod tests {
    #[test]
    fn replacement_is_idempotent() {
        let first = super::replace_section(
            "# Benchmarks\n",
            "<!-- BEGIN COMPLETE BENCHMARK REPORT -->\none\n<!-- END COMPLETE BENCHMARK REPORT -->\n",
        );
        let second = super::replace_section(
            &first,
            "<!-- BEGIN COMPLETE BENCHMARK REPORT -->\ntwo\n<!-- END COMPLETE BENCHMARK REPORT -->\n",
        );
        assert_eq!(second.matches(super::BEGIN).count(), 1);
        assert!(second.contains("two"));
        assert!(!second.contains("one"));
    }

    #[test]
    fn manifest_catalog_matches_registered_targets() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
        let suites = super::read_bench_suites(root).unwrap();
        assert_eq!(suites.len(), 7);
        assert!(suites.iter().any(|suite| suite.name == "competitive"));
        assert!(suites.iter().any(|suite| !suite.criterion));
    }
}
