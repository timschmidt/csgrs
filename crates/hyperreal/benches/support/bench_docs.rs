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
    let section = render_section(bench_binary, description, groups, &begin, &end);
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
        "The timings themselves are emitted by Criterion under `target/criterion/`; this file documents the benchmark IDs and what each one measures.\n",
    ]
    .join("\n")
}

fn render_section(
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
        out.push_str("| Benchmark output | What it measures |\n");
        out.push_str("| --- | --- |\n");
        for bench in group.benches {
            out.push_str(&format!(
                "| `{}/{}` | {} |\n",
                group.name, bench.name, bench.description
            ));
        }
        out.push('\n');
    }

    out.push_str(end);
    out.push('\n');
    out
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
