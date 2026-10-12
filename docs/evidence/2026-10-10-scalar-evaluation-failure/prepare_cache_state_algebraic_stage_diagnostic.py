"""Freeze a single instrumented diagnostic of the full failed algebraic fixture."""
import difflib
import hashlib
import json
import os
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    protocol_path = E / "cache-state-algebraic-stage-protocol-v2000.json"
    probe = E / "cache-state-algebraic-stage-probe-v2000.rs"
    patch = E / "cache-state-algebraic-stage-probe-v2000.patch"
    assert all(not path.exists() for path in [protocol_path, probe, patch])
    rejection_path = E / "cache-state-merge-cost-rejection-v1943.json"
    rejection = read(rejection_path)
    assert not rejection["cost_gate_passed"] and not rejection["runtime_retained"]
    original_phase = E / "typed-api-prototype-v1941-cache-state-merge-algebraic-cost"
    original_probe = original_phase / "probe.rs"
    before = original_probe.read_text()
    after = before

    def replace(old, new):
        nonlocal after
        assert after.count(old) == 1, old
        after = after.replace(old, new)

    marker = '''fn diagnostic_mark(stage: &str, iteration: Option<usize>, start: &Instant) {
    let stat = std::fs::read_to_string("/proc/self/stat").unwrap();
    let fields: Vec<_> = stat.rsplit_once(')').unwrap().1.split_whitespace().collect();
    let user_ticks: u64 = fields[11].parse().unwrap();
    let system_ticks: u64 = fields[12].parse().unwrap();
    eprintln!(
        "{}",
        json!({"diagnostic_stage":stage, "iteration":iteration,
            "wall_ns":start.elapsed().as_nanos().to_string(),
            "user_ticks":user_ticks, "system_ticks":system_ticks})
    );
}

'''
    replace("fn main() {", marker + "fn main() {")
    replace("        let values: Vec<_> = (0..iterations)\n            .map(|_| {\n",
            "        let diagnostic_start = Instant::now();\n"
            "        diagnostic_mark(\"start\", None, &diagnostic_start);\n"
            "        let values: Vec<_> = (0..iterations)\n            .map(|index| {\n"
            "                diagnostic_mark(\"before_fixture\", Some(index), &diagnostic_start);\n")
    replace("                let value = fixture(degree, count, bits, inverse, 0);\n",
            "                let value = fixture(degree, count, bits, inverse, 0);\n"
            "                diagnostic_mark(\"after_fixture\", Some(index), &diagnostic_start);\n")
    replace("                sample(&value, -16384, 0, None);\n",
            "                sample(&value, -16384, 0, None);\n"
            "                diagnostic_mark(\"after_warmup\", Some(index), &diagnostic_start);\n")
    replace("        let start = Instant::now();\n",
            "        diagnostic_mark(\"before_queries\", None, &diagnostic_start);\n"
            "        let start = Instant::now();\n")
    replace("        println!(\n",
            "        let query_elapsed = start.elapsed();\n"
            "        diagnostic_mark(\"after_queries\", None, &diagnostic_start);\n"
            "        println!(\n")
    replace('"elapsed_ns":start.elapsed().as_nanos().to_string()',
            '"elapsed_ns":query_elapsed.as_nanos().to_string()')
    # Fixture construction and the independent sample/sign assertions stay
    # byte-identical; only stage markers and query-time capture are added.
    assert before[before.index("fn raw("):before.index("fn main()")] == after[after.index("fn raw("):after.index("fn diagnostic_mark(")]
    commands = read(original_phase / "commands.json")
    build = next(row["command"] for row in commands if row["name"] == "build").copy()
    binary = ROOT / "target/cache-state-algebraic-stage-diagnostic-v2000/probe"
    build[build.index(str(original_probe))] = str(probe)
    build[-1] = str(binary)
    summary = read(original_phase / "summary.json")
    assert all(sha(Path(name)) == digest for name, digest in summary["libraries_sha256"].items())
    warmup = read(E / "cache-state-merge-focused-cost-v1943/preparation.json")
    assert sha(Path(warmup["build_command"][-1])) == warmup["prelude_sha256"]
    probe.write_text(after)
    format_command = ["rustfmt", "--edition", "2024", str(probe)]
    subprocess.run(format_command, check=True)
    after = probe.read_text()
    assert before[before.index("fn raw("):before.index("fn main()")] == after[after.index("fn raw("):after.index("fn diagnostic_mark(")]
    patch.write_text("".join(difflib.unified_diff(before.splitlines(True), after.splitlines(True),
                                               fromfile=str(original_probe.relative_to(E)), tofile=probe.name)))
    data = {
        "status": "Frozen before compiling or running this separately instrumented diagnostic",
        "source_record": rejection["source_record"], "source_record_sha256": rejection["source_record_sha256"],
        "rejection_record": rejection_path.name, "rejection_record_sha256": sha(rejection_path),
        "original_probe": str(original_probe.relative_to(E)), "original_probe_sha256": sha(original_probe),
        "probe": probe.name, "probe_sha256": sha(probe), "patch": patch.name, "patch_sha256": sha(patch),
        "runner": "diagnose_cache_state_algebraic_stages.py",
        "runner_sha256": sha(E / "diagnose_cache_state_algebraic_stages.py"),
        "generator_sha256": sha(Path(__file__)),
        "output": "cache-state-algebraic-stage-diagnostic-v2000", "build_command": build,
        "format_command": format_command,
        "libraries_sha256": summary["libraries_sha256"],
        "command": [str(binary), "cost", "4", "4", "0", "false", "64"],
        "address_space_bytes": 192 * 1024 * 1024, "cpu_seconds": [2, 3], "wall_seconds": 4,
        "cpu": min(os.sched_getaffinity(0)), "clock_ticks_per_second": os.sysconf("SC_CLK_TCK"),
        "prelude": warmup["build_command"][-1], "prelude_sha256": warmup["prelude_sha256"],
        "prelude_iterations": 50_000_000, "diagnostic_processes": 1,
        "limits": [
            "One candidate-only instrumented run under the original caps; no retry, larger-cap substitution, matched benchmark or acceptance decision.",
            "Keep all 64 fourth-root/four-factor fixtures, their warm precision -16384, sign query and independent zero/sample assertions. The original fixture functions are byte-identical.",
            "Markers read Linux process CPU ticks and wall time and write stderr. Instrumentation has overhead; phase differences include it and are diagnostic observations only.",
            "This run may identify its own unfinished stage or observed work distribution. It cannot identify the stage of the earlier uninstrumented failure retroactively.",
            "The retained baseline executable is intact, but its former shared hyperreal rlib has been replaced by subsequent candidate builds. This uses only the current hash-verified candidate libraries and makes no baseline stage comparison.",
            "The v1943 failure, unretained selector, v1836 retained runtime, thirteen open proof groups and full properties-plan scope remain unchanged.",
        ],
    }
    protocol_path.write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps({"prepared": protocol_path.name, "diagnostic_processes": 1, "executed": False}))


if __name__ == "__main__":
    main()
