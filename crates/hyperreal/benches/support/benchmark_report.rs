//! Shared stored-result Markdown writer for Hyperreal benchmarks and its report example.
//!
//! Discover Criterion new directories, retain usable rows keyed by full_id, catalog
//! explicit benchmark declarations with a small manifest text parser, and replace
//! one marked section in the crate's benchmarks.md. No benchmarks are executed and
//! no timing, statistical-confidence, source-revision or completed-workload claim
//! is independently validated. The generated report's completeness wording is
//! limited by these discovery rules; missing/malformed fields can omit rows.
//!
//! The callback honors the skip environment variable and benchmark arguments;
//! the direct writer does not. CRITERION_HOME can isolate this crate's stored data;
//! otherwise shared target directories can contribute other crates' results.
//! Output is assembled in memory, then written directly without locking or atomic
//! replacement. Existing-report read failures fall back to a new heading. Recursive
//! filesystem traversal, input size and retained rows have no explicit work, memory,
//! symlink-cycle or cancellation bound. This is development evidence formatting,
//! not authoritative geometry or a proof of equivalent benchmark contracts.

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

use serde_json::Value;

/// Display title used only when an existing report cannot be read.
const CRATE_TITLE: &str = "Hyperreal";
/// Presence of this environment variable disables the callback, including an empty value.
const SKIP_ENV: &str = "HYPERREAL_SKIP_BENCHMARK_REPORTS";
/// Printed suggested timing command; does not execute or prove coverage of feature-gated targets.
const TIMING_COMMAND: &str = "cargo bench --features simple";
/// Literal first-section opening delimiter; matching uses substring search, not Markdown parsing.
const BEGIN: &str = "<!-- BEGIN COMPLETE BENCHMARK REPORT -->";
/// Literal closing delimiter searched after the first opening marker.
const END: &str = "<!-- END COMPLETE BENCHMARK REPORT -->";

/// Counts and destination returned after a successful report write.
/// Counts describe discovered records, not completed benchmark executions. Derived
/// Debug displays the stored fields without reading or regenerating the report.
#[derive(Debug)]
pub struct ReportSummary {
    /// Crate-root benchmarks.md path written by this invocation.
    pub path: PathBuf,
    /// Number of retained usable full_id records after duplicate-key replacement.
    pub rows: usize,
    /// Number of rows participating in qualifying comparison groups, not the number of groups.
    pub comparisons: usize,
    /// Number of named explicit bench entries recognized by the manifest text parser.
    pub suites: usize,
}

/// One usable stored Criterion record, with numeric values projected to binary64.
/// This is reporting data, not an exact scalar certificate or validation of timing
/// semantics. Missing optional statistics may be replaced by the mean. Derived
/// Debug displays fields only; it does not load or check the original files.
#[derive(Debug)]
struct Row {
    /// Criterion full identifier used for deduplication and report ordering.
    full_id: String,
    /// Group label used with value_str to form comparison keys; missing/non-string becomes empty.
    group_id: String,
    /// Implementation label; an empty label excludes the row from comparisons.
    function_id: String,
    /// Input label; missing/non-string becomes empty and affects comparison eligibility.
    value_str: String,
    /// Formatted first throughput-object entry, or a dash when absent; malformed counts become zero.
    throughput: String,
    /// Mean point estimate in nanoseconds; sign and statistical plausibility are not validated.
    mean_ns: f64,
    /// Mean confidence lower bound, falling back to mean_ns if unavailable.
    mean_low_ns: f64,
    /// Mean confidence upper bound, falling back to mean_ns if unavailable.
    mean_high_ns: f64,
    /// Median point estimate, falling back to mean_ns if unavailable.
    median_ns: f64,
    /// Optional relative mean change; missing or unreadable/malformed change data is ignored.
    change_mean: Option<f64>,
}

/// Manifest-text benchmark entry and a source-text classification heuristic.
/// Neither required features nor the guessed report path are validated by Cargo here.
/// Derived Debug displays the stored metadata without inspecting files.
#[derive(Debug)]
struct BenchSuite {
    /// Recognized quoted name; explicit path overrides and automatic target discovery are not used.
    name: String,
    /// Quoted tokens from a recognized required-features line after the name, without TOML unescaping.
    required_features: Vec<String>,
    /// True when the guessed benches/name.rs text contains criterion:: or criterion_group!, even in comments.
    criterion: bool,
}

/// Criterion callback that conditionally regenerates the shared report after prior
/// registered work. The Criterion argument is unused: data comes from files.
/// Return immediately when reports_disabled; otherwise print summary or error to
/// stderr and do not propagate writer errors as benchmark failure. I/O panics are
/// not caught, and this callback does not verify which benchmarks actually ran.
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

/// Disable callback writes if the skip variable is present, --bench is absent, or
/// any exact --test, --list or --help argument is present. Inspect all arguments,
/// including the executable name, without parsing option values. This is a simple
/// process-mode guard, not validation that a complete timing run has finished.
pub fn reports_disabled() -> bool {
    let args: Vec<_> = std::env::args().collect();
    std::env::var_os(SKIP_ENV).is_some()
        || !args.iter().any(|arg| arg == "--bench")
        || args
            .iter()
            .any(|arg| arg == "--test" || arg == "--list" || arg == "--help")
}

/// Collect stored rows and suite metadata relative to the compiled CARGO_MANIFEST_DIR,
/// render a section, and write benchmarks.md. Sort retained rows by full_id; duplicate
/// selection already occurred during filesystem traversal. Return discovery/read,
/// JSON or write errors except where individual helpers explicitly ignore them.
///
/// Any error reading the existing report, including invalid UTF-8, is replaced by
/// a fresh heading; a successful subsequent write can overwrite that previous file.
/// The write is direct, non-atomic and unlocked, with no rollback for partial writes.
/// Missing result directories can yield a successful zero-row report. Ignore the
/// callback skip switch here. No freshness, full-suite completion, statistical
/// validity, equivalent-workload or total-resource guarantee is established.
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

/// Choose stored-result location: CRITERION_HOME verbatim first, then CARGO_TARGET_DIR
/// plus criterion (relative target paths are joined to the crate root). Without either,
/// search executable ancestors for the first containing a criterion directory, else
/// fall back to root/target/criterion. A relative CRITERION_HOME follows process cwd.
/// Existence/type of the final chosen location is not validated here; environment
/// selection does not prove that results belong to this crate or current revision.
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

/// Return usable rows sorted lexically by full_id through a BTreeMap. A path for
/// which exists returns false yields an empty vector; otherwise traverse recursively.
/// Propagate traversal/required JSON errors. Duplicate IDs keep the last encountered
/// row, whose selection depends on filesystem iteration order, not sorted paths.
fn collect_rows(criterion_dir: &Path) -> io::Result<Vec<Row>> {
    let mut rows = BTreeMap::new();
    if criterion_dir.exists() {
        collect_rows_from(criterion_dir, &mut rows)?;
    }
    Ok(rows.into_values().collect())
}

/// Visit child directories in filesystem order, reading a directory named new as
/// one Criterion leaf and recursing into other directories. Ignore nondirectories;
/// a new leaf's children are not searched. Insert usable rows by full_id, replacing
/// an earlier match. Propagate read_dir/entry/required-row errors; missing fields can
/// silently skip a row. is_dir follows links, with no cycle/depth/file-count bound
/// or deterministic duplicate-choice rule. The supplied map retains prior inserts
/// if a later error is returned.
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

/// Read estimates.json before benchmark.json from a new directory. Missing files,
/// missing/non-string full_id or absent/non-numeric mean point estimate yield None;
/// required-file read/JSON errors propagate. Optional labels default to empty,
/// confidence bounds and median to the mean, and throughput uses its formatter.
///
/// Read sibling change/estimates.json opportunistically, ignoring its I/O/parse errors.
/// No positive-duration, confidence-level, endpoint-order, schema-version or provenance
/// check is made; numeric projection uses f64. A present row is sufficient for report
/// formatting, not evidence of a valid or comparable completed benchmark.
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

/// Return None when exists reports false; otherwise read all UTF-8 text and parse
/// JSON. Propagate read errors and wrap syntax errors as InvalidData with the path.
/// This is not a schema check or atomic snapshot, and no input-size bound is applied.
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

/// Clone a named JSON string; missing fields and other JSON types return None.
fn string_field(value: &Value, field: &str) -> Option<String> {
    value.get(field)?.as_str().map(ToOwned::to_owned)
}

/// Project section.point_estimate to f64 if present and numeric.
/// No duration, confidence or statistical-domain validation accompanies the projection.
fn point_estimate(value: &Value, section: &str) -> Option<f64> {
    value.get(section)?.get("point_estimate")?.as_f64()
}

/// Project a named section.confidence_interval bound to f64 when available.
/// Ignore the recorded confidence level and do not compare lower/upper bounds.
fn confidence_bound(value: &Value, section: &str, bound: &str) -> Option<f64> {
    value
        .get(section)?
        .get("confidence_interval")?
        .get(bound)?
        .as_f64()
}

/// Format the first entry in a throughput object, or a dash for absent/non-object
/// or empty input. Recognize Elements, Bytes and BytesDecimal labels; preserve other
/// unit strings. Use zero when the selected count is not a u64 and ignore later
/// entries. This is presentation fallback, not throughput schema validation.
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

/// Scan Cargo.toml text for explicit `[[bench]]` sections, strip each line at its first
/// hash, and recognize a double-quoted name plus quoted required-features tokens.
/// Features before the name are ignored; multi-line arrays, escaped/quoted syntax,
/// path overrides and automatic Cargo target discovery are not fully parsed.
///
/// Flush a named entry at each section boundary/end, classify its guessed source,
/// and sort by name. Manifest read errors propagate, while source inspection errors
/// become non-Criterion. This is a parser for current simple manifest formatting,
/// not a general TOML/Cargo inventory or feature-validation authority.
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

/// Append a named suite, taking and clearing its collected feature strings. Without
/// a name, just clear features. Read root/benches/name.rs and classify by literal
/// criterion:: or criterion_group! substring; read failure means false. Ignore Cargo
/// path overrides and syntax context, so comments can affect the heuristic.
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

/// Split at the first equals sign, require an exact trimmed key, and remove one
/// leading/trailing double quote from the trimmed value. Return the interior without
/// TOML escape decoding or validation; comments must already have been handled.
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

/// Collect alternating pieces after splitting on double quotes, including an
/// unclosed final quoted piece. Does not parse arrays, escapes, comments or TOML types.
fn quoted_values(line: &str) -> Vec<String> {
    line.split('"')
        .skip(1)
        .step_by(2)
        .map(ToOwned::to_owned)
        .collect()
}

/// Assemble marked Markdown for suite metadata, qualified comparison groups and all
/// supplied rows. Preserve suite/row slice order; comparison_groups supplies ordered
/// group keys and per-group rows. Report mean-time ratios relative to the fastest
/// stored row without validating positive denominators or equivalent semantics.
/// Return the number of compared rows, not groups.
///
/// Source-text classification selects Criterion, dispatch diagnostic or guessed custom
/// report links, without checking their existence. The confidence header is fixed at
/// 95% even though input confidence levels are not checked. Numeric formatting is
/// binary64 presentation; escaping is partial and no arbitrary-input Markdown safety
/// is promised. This function neither executes benchmarks nor proves report coverage,
/// freshness or statistical validity. The entire section is retained in memory.
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

/// Group rows with nonempty function IDs by (group_id,value_str). Keep groups with
/// at least two distinct function labels and either a nonempty input label or a group
/// containing the case-sensitive substring competitive/comparison. Duplicate rows
/// for one function are retained. Sort each group by total f64 mean order, then
/// function ID; group keys are lexically ordered. Shared labels do not establish
/// identical input, output, guarantees, environment or completed work.
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

/// Format nanoseconds with two decimal places, choosing ns/us/ms/s at powers of
/// 1000. Do not validate negative or nonfinite values; floating-point formatting
/// and threshold comparisons determine their displayed form. No precision/error
/// receipt is attached to this human-readable timing view.
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

/// Multiply a present relative change by 100 and print a signed two-decimal percent;
/// None becomes a dash. No statistical significance or finite-value check is applied.
fn format_change(change: Option<f64>) -> String {
    change
        .map(|value| format!("{:+.2}%", value * 100.0))
        .unwrap_or_else(|| "-".to_owned())
}

/// Prefix pipe and backtick characters with backslashes, leaving all other text
/// unchanged. In particular, newlines, backslashes and HTML are not sanitized; this
/// limited table-label escaping is not a general Markdown or inline-code encoder.
fn markdown(value: &str) -> String {
    value.replace('|', "\\|").replace('`', "\\`")
}

/// Replace the first opening marker through the first following closing marker,
/// trimming surrounding whitespace and ending that result with one newline. Later
/// marker pairs remain untouched. With no opening marker, append the supplied section
/// after the trimmed document and two newlines. With no following closing marker,
/// discard everything from the opening marker onward before appending the section.
///
/// Markers are literal substrings, even in code/text; this does not validate section
/// structure. The appended branches preserve the section's own final whitespace.
/// The function allocates a new string and performs no filesystem write.
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
/// Two finite smoke fixtures for marked-section replacement and this crate's current manifest.
/// They do not establish parser completeness, filesystem safety or benchmark validity.
mod tests {
    #[test]
    /// Append a marked section, replace its content once, then require one opening
    /// marker, the new text and absence of the old text. Despite the name, this does
    /// not assert byte equality after repeating the same replacement, or test duplicate
    /// markers and missing closing markers.
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
    /// Read the compiled crate's current manifest and require fifteen parsed entries,
    /// two selected names and at least one non-Criterion classification. This checks a
    /// finite inventory shape, not every target/name/feature against Cargo metadata or
    /// a general TOML parser.
    fn manifest_catalog_matches_registered_targets() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
        let suites = super::read_bench_suites(root).unwrap();
        assert_eq!(suites.len(), 15);
        assert!(
            suites
                .iter()
                .any(|suite| suite.name == "elliptic_integrals")
        );
        assert!(suites.iter().any(|suite| suite.name == "gmp_api"));
        assert!(suites.iter().any(|suite| !suite.criterion));
    }
}
