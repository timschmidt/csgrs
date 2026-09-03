use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::hint::black_box;
use std::path::{Path, PathBuf};
use std::time::Instant;

const REPORT_NAME: &str = "slow_performers.txt";
const PROMOTED_NAME: &str = "promoted_slow_offenders.txt";
const BENCHMARKS_NAME: &str = "benchmarks.md";
const SAMPLE_REPEATS: usize = 5;
const REPORT_LIMIT: usize = 1_000;
const DEFAULT_TARGET_CASES: usize = 20_000;
const PROMOTION_ROTATION: usize = 1;
const PROMOTED_TARGET: usize = 100;
const SCORE_SECTION_BEGIN: &str = "<!-- BEGIN promoted_slow_offender_score -->";
const SCORE_SECTION_END: &str = "<!-- END promoted_slow_offender_score -->";
const SCORE_NANOS_PREFIX: &str = "<!-- promoted_slow_score_nanos:";
const SCORE_PREVIOUS_NANOS_PREFIX: &str = "<!-- promoted_slow_previous_score_nanos:";
const SCORE_DELTA_PREFIX: &str = "<!-- promoted_slow_score_delta_nanos:";

#[derive(Clone, Copy)]
pub struct Config {
    pub crate_title: &'static str,
    pub bench_target: &'static str,
    pub skip_env: &'static str,
    pub case_count_env: &'static str,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TimedCase {
    pub target: String,
    pub seed: u64,
    pub nanos: u128,
}

impl TimedCase {
    fn input(&self) -> String {
        format!("seed[{}]", self.seed)
    }

    pub fn criterion_name(&self) -> String {
        format!("{}_seed_{}", sanitize(&self.target), self.seed)
    }
}

pub struct Refresh {
    pub promoted: Vec<TimedCase>,
}

#[derive(Clone, Copy, Debug)]
struct PromotedScore {
    cases: usize,
    previous_nanos: u128,
    average_nanos: u128,
    delta_nanos: i128,
    derivative_nanos: i128,
}

pub fn metadata_only_invocation() -> bool {
    let args: Vec<_> = std::env::args().collect();
    !args.iter().any(|arg| arg == "--bench")
        || args
            .iter()
            .any(|arg| arg == "--test" || arg == "--list" || arg == "--help")
}

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

fn sort_worst_first(cases: &mut [TimedCase]) {
    cases.sort_by(|left, right| {
        right
            .nanos
            .cmp(&left.nanos)
            .then_with(|| left.target.cmp(&right.target))
            .then_with(|| left.seed.cmp(&right.seed))
    });
}

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

fn format_signed_duration(nanos: i128) -> String {
    if nanos < 0 {
        format!("-{}", format_duration(nanos.unsigned_abs()))
    } else {
        format_duration(nanos as u128)
    }
}

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

fn extract_tick_value(value: &str) -> Option<String> {
    let start = value.find('`')? + 1;
    let end = value[start..].find('`')? + start;
    Some(value[start..end].to_owned())
}

fn parse_seed(input: &str) -> Option<u64> {
    input.strip_prefix("seed[")?.strip_suffix(']')?.parse().ok()
}

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

fn merge_historical_cases(report_path: &Path, current: &[TimedCase]) -> Vec<TimedCase> {
    merge_case_sets(read_existing_cases(report_path), current)
}

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

fn time_promoted_case(case: &TimedCase, run_case: fn(&str, u64)) -> TimedCase {
    let mut timed = time_case(&case.target, case.seed, run_case);
    timed.nanos = timed.nanos.max(case.nanos);
    timed
}

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

fn average_nanos(cases: &[TimedCase]) -> Option<u128> {
    (!cases.is_empty()).then(|| {
        cases
            .iter()
            .fold(0_u128, |sum, case| sum.saturating_add(case.nanos))
            / cases.len() as u128
    })
}

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

fn parse_metadata_i128(contents: &str, prefix: &str) -> Option<i128> {
    contents.lines().find_map(|line| {
        let value = line.trim().strip_prefix(prefix)?.trim();
        value.strip_suffix("-->")?.trim().parse().ok()
    })
}

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
