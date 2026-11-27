//! Retained slow-case selection and reporting for a benchmark-specific replay callback.
//!
//! Samples deterministic target/seed identities using wall-clock probes, retains
//! maximum historical minima, and rotates a bounded returned promotion set. It does
//! not execute cargo-fuzz, decode corpus bytes, assert geometric outcomes, or preserve
//! certainty receipts. History and input reads, initial sampling, and re-probing have
//! no overall time/work/memory limit. Report writes are best-effort direct updates;
//! the skip flag suppresses writes/scoring but not collection or promotion re-probes.
//! Generated report terminology does not establish exact completion, replay fidelity,
//! statistical significance, or equivalent workloads across runs.

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::hint::black_box;
use std::path::{Path, PathBuf};
use std::time::Instant;

/// Crate-relative Markdown history file used for retaining worst observed probe values.
const REPORT_NAME: &str = "slow_performers.txt";
/// Crate-relative tab-separated file storing the promoted target/seed replay set.
const PROMOTED_NAME: &str = "promoted_slow_offenders.txt";
/// Crate-relative Markdown file containing the current promoted-score section.
const BENCHMARKS_NAME: &str = "benchmarks.md";
/// Number of immediate wall-clock probes per case; the minimum is retained.
const SAMPLE_REPEATS: usize = 5;
/// Maximum history rows retained after deduplication and worst-first sorting.
const REPORT_LIMIT: usize = 1_000;
/// Default total case count across all targets, despite the target-oriented constant name.
const DEFAULT_TARGET_CASES: usize = 20_000;
/// Maximum fastest existing cases released per refresh and first-pass historical additions.
const PROMOTION_ROTATION: usize = 1;
/// Upper bound on the returned promoted set and desired backfill size, not a guaranteed count.
const PROMOTED_TARGET: usize = 100;
/// Opening marker for the first replaceable promoted-score section.
const SCORE_SECTION_BEGIN: &str = "<!-- BEGIN promoted_slow_offender_score -->";
/// Closing marker searched after a score-section opening marker.
const SCORE_SECTION_END: &str = "<!-- END promoted_slow_offender_score -->";
/// Comment prefix carrying the current average probe time as an integer nanosecond value.
const SCORE_NANOS_PREFIX: &str = "<!-- promoted_slow_score_nanos:";
/// Comment prefix carrying the previous average probe time.
const SCORE_PREVIOUS_NANOS_PREFIX: &str = "<!-- promoted_slow_previous_score_nanos:";
/// Comment prefix carrying the last signed difference between successive score averages.
const SCORE_DELTA_PREFIX: &str = "<!-- promoted_slow_score_delta_nanos:";

/// Names configuring this crate's retained-case benchmark reporting.
///
/// Paths still derive from compile-time CARGO_MANIFEST_DIR. Names do not validate
/// target replayability, results, measurement comparability, or resource bounds.
#[derive(Clone, Copy)]
pub struct Config {
    /// Display name in history and score reports.
    pub crate_title: &'static str,
    /// Benchmark target name interpolated into generated commands.
    pub bench_target: &'static str,
    /// Presence-based switch suppressing persistence and score probes during refresh.
    pub skip_env: &'static str,
    /// Environment variable selecting the total initial probe-case count.
    pub case_count_env: &'static str,
}

/// Target/seed identity paired with a nanosecond timing value.
///
/// The meaning of nanos depends on provenance: fresh best-of-five, retained maximum
/// of such probes, or parsed history. There is no stored revision, policy, machine,
/// success receipt, or timing-distribution evidence.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TimedCase {
    /// Callback dispatch name; replayability is a caller/loader precondition.
    pub target: String,
    /// Deterministic callback seed, not an encoded libFuzzer input.
    pub seed: u64,
    /// Probe duration or retained/parsed historical value, without measurement provenance.
    pub nanos: u128,
}

/// Render stored callback identities for reports and Criterion labels, without validating replayability.
impl TimedCase {
    /// Format the replay identity as seed followed by its decimal u64 in square brackets.
    fn input(&self) -> String {
        format!("seed[{}]", self.seed)
    }

    /// Combine a sanitized target label with the decimal seed for Criterion registration.
    ///
    /// Sanitization is lossy, so distinct arbitrary target names can collide.
    pub fn criterion_name(&self) -> String {
        format!("{}_seed_{}", sanitize(&self.target), self.seed)
    }
}

/// Promoted cases selected for subsequent Criterion replay, without persistence-success status.
pub struct Refresh {
    /// Rotated cases for Criterion replay, possibly fewer than 100 or containing pre-existing duplicates.
    pub promoted: Vec<TimedCase>,
}

/// Current arithmetic-mean probe score and finite differences from stored score metadata.
///
/// These differences are between refresh values, not time-normalized derivatives.
/// The promoted sets and measurement conditions need not match between refreshes.
#[derive(Clone, Copy, Debug)]
struct PromotedScore {
    /// Number of newly timed cases, or assumed 100 when reconstructing prior metadata.
    cases: usize,
    /// Stored prior average or current average when no usable baseline exists.
    previous_nanos: u128,
    /// Integer mean of fresh best-of-five probes, subject to saturating-sum behavior.
    average_nanos: u128,
    /// Current average minus prior average, using unchecked signed-range conversion/arithmetic.
    delta_nanos: i128,
    /// Change in delta from prior metadata; not divided by elapsed time.
    derivative_nanos: i128,
}

/// Skip sampling unless argv contains exact --bench and none of --test/--list/--help.
///
/// Does not inspect the report skip environment variable or parse general CLI syntax.
pub fn metadata_only_invocation() -> bool {
    let args: Vec<_> = std::env::args().collect();
    !args.iter().any(|arg| arg == "--bench")
        || args
            .iter()
            .any(|arg| arg == "--test" || arg == "--list" || arg == "--help")
}

/// Extract, sort, and deduplicate the first quoted name in each explicit bin block.
///
/// Uses a line scan, not a TOML parser: strips everything after # even in strings,
/// recognizes exact `[[bin]]` headers, and ends a block at another table header or
/// its first name. Does not validate paths, executability, or generator mappings.
pub fn fuzz_targets_from_manifest(manifest: &str) -> Vec<String> {
    let mut targets = Vec::new();
    let mut in_bin = false;
    for raw_line in manifest.lines() {
        let line = raw_line.split('#').next().unwrap_or("").trim();
        if line == "[[bin]]" {
            in_bin = true;
            continue;
        }
        if line.starts_with('[') {
            in_bin = false;
            continue;
        }
        if in_bin && let Some(name) = assignment_string(line, "name") {
            targets.push(name);
            in_bin = false;
        }
    }
    targets.sort();
    targets.dedup();
    targets
}

/// Probe a configured total number of target/seed pairs, then sort slowest first.
///
/// A positive usize environment value overrides the 20,000 total default; absent,
/// non-Unicode, invalid, or zero values fall back to it. Targets are selected round
/// robin in supplied order, with seed = ordinal / target count. A count smaller
/// than target count leaves some targets unsampled. Each case invokes five probes;
/// there is no independent time/work/memory bound or report-skip check here.
///
/// # Panics
///
/// Panics for an empty target list, a seed ordinal not fitting u64, or a panicking
/// callback. Requested capacity is allocated before sampling.
pub fn collect_cases(
    config: Config,
    targets: &[String],
    run_case: fn(&str, u64),
) -> Vec<TimedCase> {
    assert!(
        !targets.is_empty(),
        "the fuzz manifest must register a target"
    );
    let target_cases = std::env::var(config.case_count_env)
        .ok()
        .and_then(|value| value.parse().ok())
        .filter(|count: &usize| *count > 0)
        .unwrap_or(DEFAULT_TARGET_CASES);
    let mut cases = Vec::with_capacity(target_cases);
    for ordinal in 0..target_cases {
        let target = &targets[ordinal % targets.len()];
        let seed = u64::try_from(ordinal / targets.len()).expect("case index fits u64");
        cases.push(time_case(target, seed, run_case));
    }
    sort_worst_first(&mut cases);
    cases
}

/// Invoke one target/seed callback five times and retain minimum elapsed nanoseconds, at least one.
///
/// No explicit warm-up, timeout, isolation, correctness assertion, or certainty
/// inspection occurs. Callback work, input construction, and cache effects are
/// included; black-boxing does not prove successful completion. Panics propagate.
fn time_case(target: &str, seed: u64, run_case: fn(&str, u64)) -> TimedCase {
    let mut best = u128::MAX;
    for _ in 0..SAMPLE_REPEATS {
        let start = Instant::now();
        run_case(black_box(target), black_box(seed));
        best = best.min(start.elapsed().as_nanos());
    }
    TimedCase {
        target: target.to_owned(),
        seed,
        nanos: best.max(1),
    }
}

/// Merge history, rotate the promoted set, and optionally write reports and a fresh score.
///
/// Uses the crate's compile-time manifest root. Rotation, including re-timing all
/// accepted existing promoted rows, happens even when the skip environment variable
/// is present. With writes enabled, writes promoted data, history, then scored
/// Markdown in that order. Write errors only log; updates are not transactional.
/// Returns the rotated set, not the freshly re-timed score rows or write status.
///
/// Previous-score eligibility depends only on whether the promoted file existed
/// before refresh, not whether its contents parsed. Historical rows are not filtered
/// against current targets before promotion; callers need replayable history or a
/// later callback can panic. No workload-equivalence or completion check occurs.
pub fn refresh(
    config: Config,
    targets: &[String],
    current: &[TimedCase],
    run_case: fn(&str, u64),
) -> Refresh {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let report_path = root.join(REPORT_NAME);
    let promoted_path = root.join(PROMOTED_NAME);
    let had_promoted_history = promoted_path.exists();
    let historical = merge_historical_cases(&report_path, current);
    let promoted = rotate_promoted_cases(&promoted_path, &historical, targets, run_case);

    if std::env::var_os(config.skip_env).is_none() {
        write_promoted_cases(config, &promoted_path, &promoted);
        write_report(
            config,
            &report_path,
            targets,
            current,
            &historical,
            &promoted,
        );
        if let Some((score, timed)) =
            score_promoted_cases(&promoted, run_case, had_promoted_history)
        {
            update_benchmarks_score(config, &root.join(BENCHMARKS_NAME), score, &timed);
        }
    }

    Refresh { promoted }
}

/// Read a matching key and a wholly double-quoted right side, returning its literal interior.
///
/// Does not decode escapes or implement TOML string syntax.
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

/// Keep ASCII letters, digits, and underscores; replace each other character with an underscore.
fn sanitize(value: &str) -> String {
    value
        .chars()
        .map(|ch| {
            if ch.is_ascii_alphanumeric() || ch == '_' {
                ch
            } else {
                '_'
            }
        })
        .collect()
}

/// Sort descending nanos, breaking ties by ascending target text and seed.
fn sort_worst_first(cases: &mut [TimedCase]) {
    cases.sort_by(|left, right| {
        right
            .nanos
            .cmp(&left.nanos)
            .then_with(|| left.target.cmp(&right.target))
            .then_with(|| left.seed.cmp(&right.seed))
    });
}

/// Display integer nanoseconds with ns/us/ms/s cutoffs and three decimals for scaled units.
///
/// Scaled output converts to f64 and can lose precision; it is not exact persistence
/// of arbitrary u128 values when later parsed from the human-readable history.
fn format_duration(nanos: u128) -> String {
    if nanos < 1_000 {
        format!("{nanos} ns")
    } else if nanos < 1_000_000 {
        format!("{:.3} us", nanos as f64 / 1_000.0)
    } else if nanos < 1_000_000_000 {
        format!("{:.3} ms", nanos as f64 / 1_000_000.0)
    } else {
        format!("{:.3} s", nanos as f64 / 1_000_000_000.0)
    }
}

/// Format a signed nanosecond difference, using unsigned_abs for negative i128 values.
fn format_signed_duration(nanos: i128) -> String {
    if nanos < 0 {
        format!("-{}", format_duration(nanos.unsigned_abs()))
    } else {
        format_duration(nanos as u128)
    }
}

/// Parse the first numeric/unit whitespace tokens and round their scaled f64 value to u128.
///
/// Accepts ns/us/ms/s and ignores trailing tokens. Missing/unparseable tokens or an
/// unknown unit return None. No nonnegative/finite/range check occurs: float-to-u128
/// casting can clamp values, and rounding can lose the original integer duration.
fn parse_duration_nanos(value: &str) -> Option<u128> {
    let mut parts = value.split_whitespace();
    let number = parts.next()?.parse::<f64>().ok()?;
    let multiplier = match parts.next()? {
        "ns" => 1.0,
        "us" => 1_000.0,
        "ms" => 1_000_000.0,
        "s" => 1_000_000_000.0,
        _ => return None,
    };
    Some((number * multiplier).round() as u128)
}

/// Extract the first backtick-delimited substring without interpreting escapes or Markdown.
fn extract_tick_value(value: &str) -> Option<String> {
    let start = value.find('`')? + 1;
    let end = value[start..].find('`')? + start;
    Some(value[start..end].to_owned())
}

/// Parse an exact seed[...] wrapper containing a u64 decimal value, or return None.
fn parse_seed(input: &str) -> Option<u64> {
    input.strip_prefix("seed[")?.strip_suffix(']')?.parse().ok()
}

/// Parse target/seed/timing columns from a prior human-readable Markdown history.
///
/// Read failure returns an empty list. Only pipe-prefixed lines without --- or Rank
/// are considered; malformed columns are skipped. Family and rank are ignored,
/// targets are not validated, and duration parsing may round/clamp. No deduplication
/// occurs until merging, and the input file is read in full without a size bound.
fn read_existing_cases(report_path: &Path) -> Vec<TimedCase> {
    let Ok(report) = fs::read_to_string(report_path) else {
        return Vec::new();
    };
    report
        .lines()
        .filter(|line| line.starts_with('|') && !line.contains("---") && !line.contains("Rank"))
        .filter_map(|line| {
            let columns: Vec<_> = line.split('|').map(str::trim).collect();
            let nanos = parse_duration_nanos(columns.get(2)?)?;
            let target = extract_tick_value(columns.get(4)?)?;
            let input = extract_tick_value(columns.get(5)?)?;
            Some(TimedCase {
                target,
                seed: parse_seed(&input)?,
                nanos,
            })
        })
        .collect()
}

/// Deduplicate by target/seed, retain each maximum nanos, and keep the 1,000 slowest rows.
///
/// Current probes cannot reduce retained history times. Does not compare revisions,
/// measurement conditions, validity, or policy, nor filter inactive target names.
fn merge_case_sets(previous: Vec<TimedCase>, current: &[TimedCase]) -> Vec<TimedCase> {
    let mut merged: BTreeMap<(String, u64), TimedCase> = BTreeMap::new();
    for case in previous.into_iter().chain(current.iter().cloned()) {
        merged
            .entry((case.target.clone(), case.seed))
            .and_modify(|retained| retained.nanos = retained.nanos.max(case.nanos))
            .or_insert(case);
    }
    let mut merged: Vec<_> = merged.into_values().collect();
    sort_worst_first(&mut merged);
    merged.truncate(REPORT_LIMIT);
    merged
}

/// Read prior Markdown history and merge it with current probes using maximum-time retention.
fn merge_historical_cases(report_path: &Path, current: &[TimedCase]) -> Vec<TimedCase> {
    merge_case_sets(read_existing_cases(report_path), current)
}

/// Read tab-separated promoted rows whose target is currently registered and seed/time parse.
///
/// Requires four fields but ignores the saved Criterion name; skips comments, blank
/// lines, malformed rows, and inactive targets. Read failure returns empty. Keeps
/// duplicates and zero timings, with no input-size or row-count bound at this stage.
fn read_promoted_cases(promoted_path: &Path, targets: &[String]) -> Vec<TimedCase> {
    let Ok(contents) = fs::read_to_string(promoted_path) else {
        return Vec::new();
    };
    let valid: BTreeSet<_> = targets.iter().map(String::as_str).collect();
    contents
        .lines()
        .filter(|line| !line.trim().is_empty() && !line.trim_start().starts_with('#'))
        .filter_map(|line| {
            let mut parts = line.splitn(4, '\t');
            let target = parts.next()?.to_owned();
            let input = parts.next()?;
            let nanos = parts.next()?.parse().ok()?;
            let _criterion_name = parts.next()?;
            valid.contains(target.as_str()).then_some(TimedCase {
                target,
                seed: parse_seed(input)?,
                nanos,
            })
        })
        .collect()
}

/// Re-probe one promoted case and keep the maximum of its prior time and current best-of-five.
fn time_promoted_case(case: &TimedCase, run_case: fn(&str, u64)) -> TimedCase {
    let mut timed = time_case(&case.target, case.seed, run_case);
    timed.nanos = timed.nanos.max(case.nanos);
    timed
}

/// Re-time existing promotions, release the fastest one, and backfill from ordered history.
///
/// Existing timings remain maxima after re-probing. Excludes released target/seed
/// keys from additions in this refresh; adds the first eligible historical row,
/// then backfills toward 100 before sorting/truncating. History order is trusted
/// to be worst-first. Existing duplicates are not removed, historical target names
/// are not validated, and new historical additions are not re-timed here. The
/// result can have fewer than 100 rows when eligible history is insufficient.
/// All existing rows are timed before truncation; callback panics propagate.
fn rotate_case_sets(
    existing: Vec<TimedCase>,
    historical: &[TimedCase],
    run_case: fn(&str, u64),
) -> Vec<TimedCase> {
    let mut retained: Vec<_> = existing
        .iter()
        .map(|case| time_promoted_case(case, run_case))
        .collect();
    sort_worst_first(&mut retained);
    let removed = retained.len().min(PROMOTION_ROTATION);
    let released = if removed == 0 {
        Vec::new()
    } else {
        retained.split_off(retained.len() - removed)
    };

    let released_keys: BTreeSet<_> = released
        .iter()
        .map(|case| (case.target.as_str(), case.seed))
        .collect();
    let mut added = 0;
    for candidate in historical {
        let key = (candidate.target.as_str(), candidate.seed);
        if retained
            .iter()
            .any(|case| case.target == candidate.target && case.seed == candidate.seed)
            || released_keys.contains(&key)
        {
            continue;
        }
        retained.push(candidate.clone());
        added += 1;
        if added == PROMOTION_ROTATION {
            break;
        }
    }

    for candidate in historical {
        if retained.len() >= PROMOTED_TARGET {
            break;
        }
        let key = (candidate.target.as_str(), candidate.seed);
        if retained
            .iter()
            .any(|case| case.target == candidate.target && case.seed == candidate.seed)
            || released_keys.contains(&key)
        {
            continue;
        }
        retained.push(candidate.clone());
    }
    sort_worst_first(&mut retained);
    retained.truncate(PROMOTED_TARGET);
    retained
}

/// Filter existing promoted rows by current targets, then rotate against unfiltered history.
///
/// The historical input must remain replayable; only the loaded existing file is
/// checked against targets. Inherits re-probing, duplicate, and short-set behavior.
fn rotate_promoted_cases(
    promoted_path: &Path,
    historical: &[TimedCase],
    targets: &[String],
    run_case: fn(&str, u64),
) -> Vec<TimedCase> {
    rotate_case_sets(
        read_promoted_cases(promoted_path, targets),
        historical,
        run_case,
    )
}

/// Overwrite the promoted tab-separated file, logging write errors without returning status.
///
/// Writes supplied rows in order without enforcing uniqueness, count, escaping, or
/// replayability. The header's 100-case/replayable wording is not validated here.
/// The direct write is neither atomic nor coordinated with other report updates.
fn write_promoted_cases(config: Config, path: &Path, cases: &[TimedCase]) {
    let mut out = format!(
        "# Generated by `cargo bench --bench {}`.\n",
        config.bench_target
    );
    out.push_str("# Each refresh promotes the worst replayable historical fuzz offender and releases the fastest promoted case.\n");
    out.push_str(
        "# The set is backfilled from worst historical offenders until it contains 100 cases.\n",
    );
    out.push_str("# Format: fuzz_target<TAB>input<TAB>worst_nanos<TAB>criterion_name\n");
    for case in cases {
        out.push_str(&format!(
            "{}\t{}\t{}\t{}\n",
            case.target,
            case.input(),
            case.nanos,
            case.criterion_name()
        ));
    }
    if let Err(error) = fs::write(path, out) {
        eprintln!("failed to write {}: {error}", path.display());
    }
}

/// Overwrite human-readable timing history and per-target sample/retention counts.
///
/// Consumes supplied order and counts without enforcing the stated retention policy.
/// The fixed family label is cargo-fuzz, but generated seeds need not be libFuzzer
/// corpus bytes. Zero-sample targets are printed as not sampled. Replaces backticks
/// in target labels, not arbitrary Markdown; duration formatting can lose precision.
/// Write errors log only, and the update is not atomic.
fn write_report(
    config: Config,
    path: &Path,
    targets: &[String],
    current: &[TimedCase],
    cases: &[TimedCase],
    promoted: &[TimedCase],
) {
    let mut out = format!("# {} Slow Performer History\n\n", config.crate_title);
    out.push_str(&format!(
        "Generated by `cargo bench --bench {}`. Timings are best-of-five wall-clock probes of deterministic inputs shaped by the registered cargo-fuzz targets.\n\n",
        config.bench_target
    ));
    out.push_str(&format!(
        "Latest run sampled {} deterministic fuzz cases. This table merges crate-local history by `family + target + input`, keeps each case's worst observed time, and retains the {REPORT_LIMIT} worst cases.\n\n",
        current.len()
    ));
    out.push_str("## Worst Performers\n\n");
    out.push_str("| Rank | Worst Time | Family | Target | Input |\n");
    out.push_str("| ---: | ---: | --- | --- | --- |\n");
    for (rank, case) in cases.iter().enumerate() {
        out.push_str(&format!(
            "| {} | {} | `cargo-fuzz` | `{}` | `{}` |\n",
            rank + 1,
            format_duration(case.nanos),
            case.target.replace('`', "'"),
            case.input()
        ));
    }
    out.push_str("\n## Fuzz Target Coverage\n\n");
    out.push_str("Every target registered in `fuzz/Cargo.toml` is listed, including targets whose cases were faster than the global 1,000-case history cutoff.\n\n");
    out.push_str("| Fuzz target | Cases sampled | Worst current time | Cases in history | Cases promoted |\n");
    out.push_str("| --- | ---: | ---: | ---: | ---: |\n");
    for target in targets {
        let sampled = current
            .iter()
            .filter(|case| case.target == target.as_str())
            .count();
        let worst_current = current
            .iter()
            .filter(|case| case.target == target.as_str())
            .map(|case| case.nanos)
            .max()
            .map(format_duration)
            .unwrap_or_else(|| "not sampled".to_owned());
        let retained = cases
            .iter()
            .filter(|case| case.target == target.as_str())
            .count();
        let promoted = promoted
            .iter()
            .filter(|case| case.target == target.as_str())
            .count();
        out.push_str(&format!(
            "| `{}` | {sampled} | {worst_current} | {retained} | {promoted} |\n",
            target.replace('`', "'")
        ));
    }
    out.push_str("\n## Retention Policy\n\n");
    out.push_str("- Every registered fuzz target contributes deterministic replay seeds.\n");
    out.push_str("- Each refresh releases the fastest promoted case and promotes the worst eligible historical offender.\n");
    out.push_str("- `promoted_slow_offenders.txt` is the durable 100-case replay set used for dedicated Criterion rows and the lexicase score.\n");
    if let Err(error) = fs::write(path, out) {
        eprintln!("failed to write {}: {error}", path.display());
    }
}

/// Return the integer mean of nanos, or None for an empty slice.
///
/// Uses a saturating u128 sum then truncating integer division. Once the sum
/// saturates this no longer equals the exact arithmetic mean.
fn average_nanos(cases: &[TimedCase]) -> Option<u128> {
    (!cases.is_empty()).then(|| {
        cases
            .iter()
            .fold(0_u128, |sum, case| sum.saturating_add(case.nanos))
            / cases.len() as u128
    })
}

/// Freshly re-probe promotions, average their minima, and compare with optional stored metadata.
///
/// Sorts fresh timings worst-first and returns None for an empty set. If previous
/// score use is allowed, reads benchmarks.md; absent/invalid metadata uses the
/// current average as baseline and zero prior delta. Computes delta and change in
/// delta without checking that the cases/configuration/machine are comparable.
/// Despite report wording, this is an arithmetic mean, not lexicase selection.
/// u128-to-i128 casts and signed subtraction are unchecked for representable range;
/// extreme values can wrap or overflow. Callback panics propagate.
fn score_promoted_cases(
    promoted: &[TimedCase],
    run_case: fn(&str, u64),
    use_previous_score: bool,
) -> Option<(PromotedScore, Vec<TimedCase>)> {
    let mut timed: Vec<_> = promoted
        .iter()
        .map(|case| time_case(&case.target, case.seed, run_case))
        .collect();
    sort_worst_first(&mut timed);
    let average_nanos = average_nanos(&timed)?;
    let previous = use_previous_score
        .then(|| {
            read_previous_promoted_score(
                &PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(BENCHMARKS_NAME),
            )
        })
        .flatten();
    let previous_nanos = previous
        .map(|score| score.average_nanos)
        .unwrap_or(average_nanos);
    let previous_delta = previous.map(|score| score.delta_nanos).unwrap_or(0);
    let delta_nanos = average_nanos as i128 - previous_nanos as i128;
    Some((
        PromotedScore {
            cases: timed.len(),
            previous_nanos,
            average_nanos,
            delta_nanos,
            derivative_nanos: delta_nanos - previous_delta,
        },
        timed,
    ))
}

/// Return the first parseable prefixed HTML-comment integer, ignoring malformed matching lines.
fn parse_metadata_i128(contents: &str, prefix: &str) -> Option<i128> {
    contents.lines().find_map(|line| {
        let value = line.trim().strip_prefix(prefix)?.trim();
        value.strip_suffix("-->")?.trim().parse().ok()
    })
}

/// Read score metadata, returning None on I/O failure or invalid/negative required averages.
///
/// Previous average defaults to current, missing/invalid delta to zero. Reconstructs
/// cases as 100 and derivative as zero without reading those quantities; no source
/// identity or configuration comparison is possible from this record.
fn read_previous_promoted_score(path: &Path) -> Option<PromotedScore> {
    let contents = fs::read_to_string(path).ok()?;
    let average_nanos = parse_metadata_i128(&contents, SCORE_NANOS_PREFIX)?;
    let previous_nanos =
        parse_metadata_i128(&contents, SCORE_PREVIOUS_NANOS_PREFIX).unwrap_or(average_nanos);
    Some(PromotedScore {
        cases: PROMOTED_TARGET,
        previous_nanos: previous_nanos.try_into().ok()?,
        average_nanos: average_nanos.try_into().ok()?,
        delta_nanos: parse_metadata_i128(&contents, SCORE_DELTA_PREFIX).unwrap_or(0),
        derivative_nanos: 0,
    })
}

/// Render score metadata, finite-difference metrics, and up to ten supplied timing rows.
///
/// Labels the value deterministic lexicase, but calculation is the arithmetic mean
/// of wall-clock probe minima and timings are not deterministic. Assumes supplied
/// rows are worst-first; does not sort, validate score consistency, or compare
/// completed contracts. Target escaping only replaces backticks.
fn promoted_score_section(config: Config, score: PromotedScore, timed: &[TimedCase]) -> String {
    let mut out = format!("{SCORE_SECTION_BEGIN}\n## `promoted_slow_offender_score`\n\n");
    out.push_str(&format!(
        "Deterministic lexicase score for {}'s retained fuzz offenders. The score is the average current best-of-five replay time; lower is better. Delta compares with the previous score, and derivative is the change in delta.\n\n",
        config.crate_title
    ));
    out.push_str(&format!(
        "{SCORE_NANOS_PREFIX} {} -->\n{SCORE_PREVIOUS_NANOS_PREFIX} {} -->\n{SCORE_DELTA_PREFIX} {} -->\n\n",
        score.average_nanos, score.previous_nanos, score.delta_nanos
    ));
    out.push_str("| Metric | Value |\n| --- | ---: |\n");
    out.push_str(&format!("| Cases scored | {} |\n", score.cases));
    out.push_str(&format!(
        "| Average score | {} |\n| Delta | {} |\n| Delta derivative | {} |\n\n",
        format_duration(score.average_nanos),
        format_signed_duration(score.delta_nanos),
        format_signed_duration(score.derivative_nanos)
    ));
    out.push_str("| Rank | Current Time | Fuzz target | Input |\n");
    out.push_str("| ---: | ---: | --- | --- |\n");
    for (rank, case) in timed.iter().take(10).enumerate() {
        out.push_str(&format!(
            "| {} | {} | `{}` | `{}` |\n",
            rank + 1,
            format_duration(case.nanos),
            case.target.replace('`', "'"),
            case.input()
        ));
    }
    out.push_str(&format!("\n{SCORE_SECTION_END}\n"));
    out
}

/// Replace the first complete score marker pair, or prepend a new section if either marker is missing.
///
/// Preserves the rest byte-for-byte. An unterminated old section remains after the
/// prepended section, so duplicate markers can accumulate. Does not validate nesting.
fn replace_section(contents: &str, section: &str) -> String {
    let Some(start) = contents.find(SCORE_SECTION_BEGIN) else {
        return format!("{section}\n{contents}");
    };
    let Some(relative_end) = contents[start..].find(SCORE_SECTION_END) else {
        return format!("{section}\n{contents}");
    };
    let end = start + relative_end + SCORE_SECTION_END.len();
    format!("{}{}{}", &contents[..start], section, &contents[end..])
}

/// Rewrite the score section, substituting a fresh header when reading existing Markdown fails.
///
/// Writes directly and logs errors without returning success/failure. No atomic
/// replacement or coordination with the separate benchmark report writer occurs.
fn update_benchmarks_score(config: Config, path: &Path, score: PromotedScore, timed: &[TimedCase]) {
    let section = promoted_score_section(config, score, timed);
    let contents = fs::read_to_string(path).unwrap_or_else(|_| {
        format!(
            "# {} Benchmarks\n\nThis file is updated automatically by the benchmark binaries.\n",
            config.crate_title
        )
    });
    if let Err(error) = fs::write(path, replace_section(&contents, &section)) {
        eprintln!("failed to write {}: {error}", path.display());
    }
}
