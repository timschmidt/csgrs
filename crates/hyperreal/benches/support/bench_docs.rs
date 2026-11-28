//! Per-binary Markdown catalogue shared by six Criterion benchmark targets.
//!
//! Authored group/benchmark labels select stored estimates; this is a presentation
//! helper, not a JSON parser or a validator of benchmark completion or statistics.
//! It may run before measurements and display stale files. Read/parse failures are
//! shown as `not run`; report read failures create a fresh document, and write
//! failures are logged. Writes are unlocked and non-atomic, with no explicit input
//! size, work, memory or cancellation bound.

use std::fs;
use std::path::Path;

/// One authored benchmark row and its explanatory text.
///
/// The name selects an on-disk estimate within its group; neither string is escaped
/// for Markdown or checked against Criterion registration.
pub struct BenchDoc {
    /// Benchmark path component and verbatim output label within the group.
    pub name: &'static str,
    /// Authored measurement description inserted verbatim into a Markdown table cell.
    pub description: &'static str,
}

/// One ordered group of benchmark rows for the per-binary catalogue.
///
/// This metadata does not register or execute benchmarks. Names and descriptions
/// are trusted text rather than validated Criterion identifiers.
pub struct BenchGroupDoc {
    /// Group path component below the Criterion directory and verbatim heading.
    pub name: &'static str,
    /// Authored group explanation inserted verbatim below the heading.
    pub description: &'static str,
    /// Rows to render in slice order, including rows with no readable estimate.
    pub benches: &'static [BenchDoc],
}

/// Refresh the first marked section for this benchmark binary in `benchmarks.md`.
///
/// The file is relative to the compile-time crate manifest directory. Presence of
/// `HYPERREAL_SKIP_BENCHMARK_REPORTS` (even empty), or an exact `--test`, `--list`
/// or `--help` argument, suppresses all work. No `--bench` argument is required.
/// Estimates come from the currently resolved Criterion directory, potentially
/// from an earlier run; group metadata supplies the catalogue order.
///
/// Any existing-report read error, including invalid UTF-8, substitutes the default
/// document. A missing end marker discards the old tail from the begin marker.
/// The final write is direct, unlocked and non-atomic. Write errors are printed to
/// stderr and are not returned or converted into a failing benchmark outcome.
pub fn write_benchmark_docs(
    bench_binary: &'static str,
    description: &'static str,
    groups: &'static [BenchGroupDoc],
) {
    if std::env::var_os("HYPERREAL_SKIP_BENCHMARK_REPORTS").is_some()
        || std::env::args().any(|arg| arg == "--test" || arg == "--list" || arg == "--help")
    {
        return;
    }
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let path = root.join("benchmarks.md");
    let begin = format!("<!-- BEGIN {bench_binary} -->");
    let end = format!("<!-- END {bench_binary} -->");
    let section = render_section(root, bench_binary, description, groups, &begin, &end);
    let current = fs::read_to_string(&path).unwrap_or_else(|_| default_document());
    let next = replace_section(&current, &begin, &end, &section);

    if let Err(err) = fs::write(&path, next) {
        eprintln!("failed to update {}: {err}", path.display());
    }
}

/// Initial report text used for every existing-report read failure.
///
/// The text advertises latest results, 95% confidence and `target/criterion/`;
/// it does not verify freshness/confidence or describe directory overrides.
fn default_document() -> String {
    [
        "# Benchmark Reference\n",
        "This file is updated by the Criterion benchmark binaries. Run `cargo bench` to refresh the benchmark output catalogue.\n",
        "Each table includes the latest Criterion mean and 95% confidence interval when results are available. Raw Criterion reports remain under `target/criterion/`.\n",
    ]
    .join("\n")
}

/// Render supplied groups and rows between caller-provided marker strings.
///
/// Names and descriptions are emitted without Markdown escaping. Estimates are
/// formatted as durations; any absent, unreadable or unrecognized estimate becomes
/// `not run`, which is not proof that the benchmark was never run. The fixed 95% CI
/// heading does not validate the stored confidence level, endpoint order or
/// statistical meaning. No unlisted on-disk benchmarks are discovered.
fn render_section(
    root: &Path,
    bench_binary: &str,
    description: &str,
    groups: &[BenchGroupDoc],
    begin: &str,
    end: &str,
) -> String {
    let mut out = String::new();
    out.push_str(begin);
    out.push('\n');
    out.push_str(&format!("## `{bench_binary}`\n\n"));
    out.push_str(description);
    out.push_str("\n\n");

    for group in groups {
        out.push_str(&format!("### `{}`\n\n", group.name));
        out.push_str(group.description);
        out.push_str("\n\n");
        out.push_str("| Benchmark output | Mean | 95% CI | What it measures |\n");
        out.push_str("| --- | ---: | ---: | --- |\n");
        for bench in group.benches {
            let estimate = read_estimate(root, group.name, bench.name);
            let (mean, interval) = match estimate {
                Some(estimate) => (
                    format_duration(estimate.mean),
                    format!(
                        "{} - {}",
                        format_duration(estimate.lower),
                        format_duration(estimate.upper)
                    ),
                ),
                None => ("not run".to_owned(), "not run".to_owned()),
            };
            out.push_str(&format!(
                "| `{}/{}` | {} | {} | {} |\n",
                group.name, bench.name, mean, interval, bench.description
            ));
        }
        out.push('\n');
    }

    out.push_str(end);
    out.push('\n');
    out
}

/// Three numbers extracted from stored text and interpreted as nanoseconds.
///
/// Construction establishes only that each numeric token parses as `f64`; it does
/// not establish JSON validity, finiteness, nonnegativity or interval ordering.
struct Estimate {
    /// Value found after the first `point_estimate` marker following the mean marker.
    mean: f64,
    /// Value found after the first `lower_bound` marker, displayed as a lower endpoint.
    lower: f64,
    /// Value found after the first `upper_bound` marker, displayed as an upper endpoint.
    upper: f64,
}

/// Read `group/bench/new/estimates.json` under the resolved Criterion directory.
///
/// This performs exact text-marker searches, not JSON parsing: it requires
/// `"mean":` without intervening whitespace, then searches the entire remaining
/// suffix for three numeric fields. Fields from later objects or invalid JSON can
/// therefore be accepted. File errors, missing markers and failed number parses
/// all return `None`; no freshness, object scope or confidence level is checked.
fn read_estimate(root: &Path, group: &str, bench: &str) -> Option<Estimate> {
    let path = criterion_dir(root)
        .join(group)
        .join(bench)
        .join("new")
        .join("estimates.json");
    let json = fs::read_to_string(path).ok()?;
    let mean_section = json.get(json.find("\"mean\":")?..)?;
    let mean = extract_number_after(mean_section, "\"point_estimate\":")?;
    let lower = extract_number_after(mean_section, "\"lower_bound\":")?;
    let upper = extract_number_after(mean_section, "\"upper_bound\":")?;
    Some(Estimate { mean, lower, upper })
}

/// Find the first numeric-looking token anywhere after the first exact marker.
///
/// The scan skips to a sign, dot or ASCII digit, consumes signs/dots/exponent
/// letters/digits, and parses that slice as `f64`. It is not a JSON value parser:
/// `null` may be skipped to a later field, quoted numbers can be accepted, and an
/// overflowing exponent can produce infinity. Missing markers/tokens or a failed
/// parse return `None`. No numeric domain or finite-value guard is applied.
fn extract_number_after(input: &str, marker: &str) -> Option<f64> {
    let rest = input.get(input.find(marker)? + marker.len()..)?;
    let start = rest.find(|c: char| c == '-' || c == '+' || c == '.' || c.is_ascii_digit())?;
    let rest = &rest[start..];
    let end = rest
        .find(|c: char| {
            !(c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E' || c.is_ascii_digit())
        })
        .unwrap_or(rest.len());
    rest[..end].parse().ok()
}

/// Display nanoseconds with two decimals below 1,000 and three in larger units.
///
/// Successive thresholds at 1,000, 1,000,000 and 1,000,000,000 select ns, us, ms and
/// s. This rounds only report text, not authoritative geometry. Negative and
/// nonfinite inputs are not rejected and follow the same comparisons/formatting.
fn format_duration(nanos: f64) -> String {
    if nanos < 1_000.0 {
        format!("{nanos:.2} ns")
    } else if nanos < 1_000_000.0 {
        format!("{:.3} us", nanos / 1_000.0)
    } else if nanos < 1_000_000_000.0 {
        format!("{:.3} ms", nanos / 1_000_000.0)
    } else {
        format!("{:.3} s", nanos / 1_000_000_000.0)
    }
}

/// Replace the first begin marker through its first following end marker.
///
/// With no begin marker, append the supplied section after the trimmed document.
/// With no following end marker, discard the old tail starting at the begin marker.
/// For a matched pair, trim the prefix end and suffix start and separate nonempty
/// suffix content with blank lines. Trailing suffix whitespace is retained; only
/// an empty suffix forces a final newline. Later marked sections are untouched.
/// No Markdown syntax or marker uniqueness is validated.
fn replace_section(current: &str, begin: &str, end: &str, section: &str) -> String {
    let Some(start) = current.find(begin) else {
        let mut next = current.trim_end().to_owned();
        next.push_str("\n\n");
        next.push_str(section);
        return next;
    };
    let Some(relative_end) = current[start..].find(end) else {
        let mut next = current[..start].trim_end().to_owned();
        next.push_str("\n\n");
        next.push_str(section);
        return next;
    };
    let end_index = start + relative_end + end.len();
    let suffix = current[end_index..].trim_start();
    let mut next = String::new();
    next.push_str(current[..start].trim_end());
    next.push_str("\n\n");
    next.push_str(section.trim_end());
    if suffix.is_empty() {
        next.push('\n');
    } else {
        next.push_str("\n\n");
        next.push_str(suffix);
    }
    next
}

/// Resolve the estimate directory using environment overrides and the executable.
///
/// `CRITERION_HOME` is used verbatim, including relative paths resolved by later I/O
/// against the process working directory. Otherwise `CARGO_TARGET_DIR/criterion`
/// is used, joining a relative target directory to `root`. With neither override,
/// choose the first executable ancestor containing a `criterion` directory; if
/// none is found (or executable lookup fails), use `root/target/criterion`.
/// Existence and ownership are not checked for overrides or the fallback.
fn criterion_dir(root: &Path) -> std::path::PathBuf {
    if let Some(home) = std::env::var_os("CRITERION_HOME") {
        return home.into();
    }
    match std::env::var_os("CARGO_TARGET_DIR") {
        Some(path) if Path::new(&path).is_absolute() => Path::new(&path).join("criterion"),
        Some(path) => root.join(path).join("criterion"),
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
