//! Shared Criterion JSON-to-Markdown writer for Hyperlattice benchmark binaries.
//!
//! Catalogs explicit manifest suites and stored new-result rows, including data
//! from older or unrelated runs in a shared result tree. Timing ratios do not
//! establish equivalent completed contracts. Direct writes replace the marked
//! section in this crate's benchmarks.md; this module executes no benchmarks.
//! Automatic callers apply an environment/argv gate; the writer example calls
//! the writer directly. Traversal/rendering have no work, depth, or memory budget.

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

use serde_json::Value;

/// Title used only when a readable existing benchmarks.md cannot be loaded.
const CRATE_TITLE: &str = "Hyperlattice";
/// Presence-based environment switch suppressing automatic benchmark report callbacks.
const SKIP_ENV: &str = "HYPERLATTICE_SKIP_BENCHMARK_REPORTS";
/// Command printed in the report, assuming execution in this crate context.
const TIMING_COMMAND: &str = "cargo bench";
/// Literal opening marker locating the first replaceable generated section.
const BEGIN: &str = "<!-- BEGIN COMPLETE BENCHMARK REPORT -->";
/// Literal closing marker searched for after the first opening marker.
const END: &str = "<!-- END COMPLETE BENCHMARK REPORT -->";

/// Counts and destination from a completed report-file write.
///
/// Reports stored data and manifest inventory, not newly executed or certified
/// workloads. Comparisons counts participating rows, not groups or pairwise ratios.
#[derive(Debug)]
pub struct ReportSummary {
    /// Destination benchmarks.md path written by this invocation.
    pub path: PathBuf,
    /// Number of collected rows after duplicate full-ID replacement.
    pub rows: usize,
    /// Total rows in retained comparison groups, not pair count or group count.
    pub comparisons: usize,
    /// Number of named explicit bench entries found by the lightweight manifest scan.
    pub suites: usize,
}

/// One stored Criterion result normalized for table rendering.
///
/// Timing values use the JSON nanosecond convention. Missing bounds/median may be
/// filled with the mean; optional change may be absent after a read/parse failure.
/// Carries no revision, feature, policy, confidence-level, or completion certificate.
#[derive(Debug)]
struct Row {
    /// Required stored benchmark identifier and deduplication key.
    full_id: String,
    /// Stored group label, or empty when missing/non-string.
    group_id: String,
    /// Stored implementation label, or empty when missing/non-string.
    function_id: String,
    /// Stored input label, or empty when missing/non-string.
    value_str: String,
    /// Formatted workload count/unit text, not a per-second rate.
    throughput: String,
    /// Required numeric mean estimate interpreted as nanoseconds.
    mean_ns: f64,
    /// Stored lower mean confidence bound, defaulting to mean when absent/non-numeric.
    mean_low_ns: f64,
    /// Stored upper mean confidence bound, defaulting to mean when absent/non-numeric.
    mean_high_ns: f64,
    /// Stored median estimate, defaulting to mean when absent/non-numeric.
    median_ns: f64,
    /// Optional fractional mean change from sibling change estimates, with errors suppressed.
    change_mean: Option<f64>,
}

/// Lightweight manifest benchmark entry with a source-text Criterion heuristic.
///
/// Does not prove that the target was built, executed, or generated its linked report.
#[derive(Debug)]
struct BenchSuite {
    /// Manifest name used for conventional source paths, commands, and sorting.
    name: String,
    /// Quoted feature strings found after the name on a same-line assignment.
    required_features: Vec<String>,
    /// Whether conventional source text contains a recognized Criterion substring.
    criterion: bool,
}

/// Criterion-compatible callback that conditionally rewrites benchmarks.md and logs the outcome.
///
/// Ignores the supplied Criterion instance and reads stored files instead. Uses
/// [`reports_disabled`] to avoid writes in skipped/test/list/help modes. I/O failures
/// are printed to stderr without failing the benchmark process or returning an error.
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

/// Decide whether automatic reporting is disabled from environment presence and argv.
///
/// Returns true if the skip variable exists (even empty), no exact --bench argument
/// exists, or any exact --test/--list/--help argument exists. Does not parse option
/// values, short flags, or invocation context beyond these string tests.
pub fn reports_disabled() -> bool {
    let args: Vec<_> = std::env::args().collect();
    std::env::var_os(SKIP_ENV).is_some()
        || !args.iter().any(|arg| arg == "--bench")
        || args
            .iter()
            .any(|arg| arg == "--test" || arg == "--list" || arg == "--help")
}

/// Rewrite the crate's generated benchmark section from stored estimates and manifest entries.
///
/// Uses compile-time CARGO_MANIFEST_DIR as the crate root, loads/sorts rows by full
/// ID, and catalogs explicit bench entries. Does not run benchmarks or honor the
/// automatic-report skip gate; direct callers always attempt the write.
///
/// Any failure to read the existing benchmarks.md substitutes a fresh title/header.
/// The first marked section is replaced; a missing closing marker drops the old
/// tail from the opening marker onward. Writes directly with fs::write, without a
/// temporary file, atomic replacement, or concurrent-writer coordination. Earlier
/// collection/catalog errors propagate; write failure may leave a partial file.
/// Success describes the stored data written, not workload coverage or correctness.
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

/// Choose the stored-result root from environment, executable ancestry, or a crate fallback.
///
/// CRITERION_HOME wins verbatim, including relative paths resolved by later I/O
/// against the process working directory. Otherwise an absolute CARGO_TARGET_DIR
/// gets /criterion appended; a relative target directory is joined to crate root.
/// With neither variable, finds the first executable ancestor containing a criterion
/// directory, or uses root/target, then appends criterion. Does not verify that the
/// selected tree belongs to this crate or the current run.
fn criterion_dir(root: &Path) -> PathBuf {
    // CRITERION_HOME is where Criterion itself writes when it is set; the
    // repository bench-report script gives each crate its own.
    if let Some(home) = std::env::var_os("CRITERION_HOME") {
        return PathBuf::from(home);
    }
    match std::env::var_os("CARGO_TARGET_DIR") {
        Some(path) if Path::new(&path).is_absolute() => PathBuf::from(path).join("criterion"),
        Some(path) => root.join(path).join("criterion"),
        // Bench binaries run from <target>/<profile>/deps, and Criterion writes
        // to <target>/criterion: the workspace target in a monorepo, not
        // necessarily this crate's own.
        None => std::env::current_exe()
            .ok()
            .and_then(|exe| {
                exe.ancestors()
                    .find(|dir| dir.join("criterion").is_dir())
                    .map(Path::to_path_buf)
            })
            .unwrap_or_else(|| root.join("target"))
            .join("criterion"),
    }
}

/// Collect available result rows into full-ID order, returning empty for a missing root.
///
/// Duplicate IDs overwrite earlier visited rows; traversal order, not timestamp,
/// selects the winner. Directory/read/required-JSON errors propagate. An empty
/// result does not establish that no benchmarks exist or have been executed.
fn collect_rows(criterion_dir: &Path) -> io::Result<Vec<Row>> {
    let mut rows = BTreeMap::new();
    if criterion_dir.exists() {
        collect_rows_from(criterion_dir, &mut rows)?;
    }
    Ok(rows.into_values().collect())
}

/// Recursively visit directories, reading each child named new as a Criterion result.
///
/// Other directories recurse; ordinary files are ignored. A new directory is not
/// recursed into even if its row is absent. Inserts by full ID, overwriting earlier
/// duplicates in filesystem enumeration order. Directory tests follow symlinks;
/// there is no cycle/depth guard or crate/run filter. Errors from visited directory
/// reads and required row files propagate.
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

/// Read a new-result directory, skipping missing required files/fields and mean estimates.
///
/// Required estimates.json and benchmark.json I/O/parse errors propagate. Missing
/// full_id or numeric mean gives None. Bounds and median default to mean; absent
/// group/function/input strings become empty. Optional sibling change estimates
/// silently ignore I/O/parse errors. Does not validate positivity, interval ordering,
/// confidence level, or whether the row completed the intended contract.
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

/// Return parsed JSON for an existing readable path, None when exists reports false.
///
/// Read errors propagate; parse errors become InvalidData with the displayed path.
/// The existence probe and read are separate, so a disappearing file can still fail.
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

/// Clone an object field only when it is a JSON string; missing or other types yield None.
fn string_field(value: &Value, field: &str) -> Option<String> {
    value.get(field)?.as_str().map(ToOwned::to_owned)
}

/// Extract section.point_estimate as f64 without validating its statistical meaning or sign.
fn point_estimate(value: &Value, section: &str) -> Option<f64> {
    value.get(section)?.get("point_estimate")?.as_f64()
}

/// Extract a named numeric confidence bound without checking confidence level or interval order.
fn confidence_bound(value: &Value, section: &str, bound: &str) -> Option<f64> {
    value
        .get(section)?
        .get("confidence_interval")?
        .get(bound)?
        .as_f64()
}

/// Format the first entry of a throughput object as a count and unit, or a dash if absent.
///
/// Maps Elements/Bytes/BytesDecimal to display labels; unknown units are used
/// verbatim. A count not representable as u64 becomes zero. This is workload size
/// per sample, not an elapsed-time throughput rate, and unit text is not escaped.
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

/// Scan explicit `[[bench]]` blocks in Cargo.toml with a line-oriented parser.
///
/// Recognizes a quoted name first, then same-line quoted required-features values.
/// Strips text after # even inside strings; does not support general TOML escaping,
/// multiline arrays, features before name, automatic target discovery, or custom
/// source paths. Flushes at table boundaries and sorts by name. Manifest read
/// errors propagate; missing/unreadable conventional source files only affect the
/// Criterion heuristic. This inventory is not an execution record.
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

/// Append a named manifest suite and consume its pending features, or clear features without a name.
///
/// Checks benches/name.rs for criterion:: or criterion_group! substrings. Read
/// failure means non-Criterion; comments can trigger the heuristic and other source
/// layouts can be missed. No parser or manifest path override is consulted.
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

/// Read key = double-quoted-text from one already-trimmed manifest line.
///
/// Requires the entire trimmed right side to begin/end with quotes. Returns the
/// interior literally, without TOML escape decoding or quote-content validation.
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

/// Collect alternating double-quote-delimited substrings without parsing arrays or escapes.
fn quoted_values(line: &str) -> Vec<String> {
    line.split('"')
        .skip(1)
        .step_by(2)
        .map(ToOwned::to_owned)
        .collect()
}

/// Render catalog, paired timing rows, and all supplied rows between generated-section markers.
///
/// Returns the Markdown and number of participating comparison rows. Relative time
/// is mean divided by the group's fastest stored mean, without positivity checks
/// or proof of equivalent workloads. Labels intervals 95% regardless of source
/// confidence level, and trusts fallback-filled row values. Commands assume crate
/// context; linked reports are not checked for existence. Inserts a fixed mathbench
/// trace-mode row in addition to the supplied catalog.
///
/// No timestamp/revision/feature/certainty filtering occurs. The report's completeness
/// language refers to this stored-row and manifest scan, not independently verified
/// coverage of registered targets or successful exact workloads.
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
    out.push_str("| `mathbench` trace mode | diagnostic | `hyperreal-dispatch-trace` | `cargo bench --bench mathbench --features hyperreal-dispatch-trace -- --write-dispatch-trace-md` | [dispatch_trace.md](dispatch_trace.md) |\n");

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

/// Group rows by exact group/input strings when multiple distinct function IDs exist.
///
/// Ignores empty function IDs. Keeps a group only with a nonempty input or a group
/// name containing lowercase competitive or comparison. Sorts each retained group
/// by total f64 mean order, then function ID; duplicate implementations are not
/// deduplicated. Does not establish equivalent semantics, guarantees, or revisions.
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

/// Display nanoseconds in ns/us/ms/s using powers-of-1000 cutoffs and two decimals.
///
/// Does not reject negative or nonfinite input; this is formatting, not validation.
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

/// Display an optional fractional mean change as a signed percentage, or a dash if absent.
fn format_change(change: Option<f64>) -> String {
    change
        .map(|value| format!("{:+.2}%", value * 100.0))
        .unwrap_or_else(|| "-".to_owned())
}

/// Prefix pipes and backticks with backslashes for table text.
///
/// Does not normalize newlines, escape arbitrary Markdown/HTML, or guarantee valid
/// content when the caller wraps the result in an inline code span.
fn markdown(value: &str) -> String {
    value.replace('|', "\\|").replace('`', "\\`")
}

/// Replace the first marked report, append if absent, or truncate an unterminated marked tail.
///
/// With both markers, preserves surrounding text while normalizing boundary
/// whitespace and a final newline. With only BEGIN, drops everything from it onward.
/// Does not validate marker nesting or repair multiple generated sections.
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

/// Finite checks for section replacement and this crate's explicit benchmark inventory.
#[cfg(test)]
mod tests {
    /// Check replacing one generated payload keeps one opening marker and removes the old payload.
    ///
    /// Despite the name, does not compare repeated identical replacements or exercise
    /// missing END, multiple markers, suffix preservation, or filesystem writes.
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

    /// Check seven explicit manifest suites, two selected names, and at least one non-Criterion heuristic result.
    ///
    /// Does not assert all names, feature lists, or source classifications; does not run
    /// any suite or validate general TOML parsing.
    #[test]
    fn manifest_catalog_matches_registered_targets() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
        let suites = super::read_bench_suites(root).unwrap();
        assert_eq!(suites.len(), 7);
        assert!(suites.iter().any(|suite| suite.name == "complex_jacobi"));
        assert!(suites.iter().any(|suite| suite.name == "mathbench"));
        assert!(suites.iter().any(|suite| !suite.criterion));
    }
}
