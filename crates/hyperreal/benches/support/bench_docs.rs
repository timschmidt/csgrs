use std::fs;
use std::path::Path;

pub struct BenchDoc {
    pub name: &'static str,
    pub description: &'static str,
}

pub struct BenchGroupDoc {
    pub name: &'static str,
    pub description: &'static str,
    pub benches: &'static [BenchDoc],
}

pub fn write_benchmark_docs(
    bench_binary: &'static str,
    description: &'static str,
    groups: &'static [BenchGroupDoc],
) {
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

fn default_document() -> String {
    [
        "# Benchmark Reference\n",
        "This file is updated by the Criterion benchmark binaries. Run `cargo bench` to refresh the benchmark output catalogue.\n",
        "Each table includes the latest Criterion mean and 95% confidence interval when results are available. Raw Criterion reports remain under `target/criterion/`.\n",
    ]
    .join("\n")
}

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

struct Estimate {
    mean: f64,
    lower: f64,
    upper: f64,
}

fn read_estimate(root: &Path, group: &str, bench: &str) -> Option<Estimate> {
    let path = root
        .join("target")
        .join("criterion")
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
    let mut next = String::new();
    next.push_str(current[..start].trim_end());
    next.push_str("\n\n");
    next.push_str(section.trim_end());
    next.push_str("\n\n");
    next.push_str(current[end_index..].trim_start());
    next
}
