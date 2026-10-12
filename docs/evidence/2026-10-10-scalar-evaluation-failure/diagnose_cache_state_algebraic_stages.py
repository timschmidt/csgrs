"""Run the frozen original-cap stage diagnostic once and preserve every outcome."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import resource
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    protocol_path = E / "cache-state-algebraic-stage-protocol-v2000.json"
    protocol = read(protocol_path)
    protocol_hash = sha(protocol_path)
    out = E / protocol["output"]
    assert not out.exists()
    source_record = E / protocol["source_record"]
    assert sha(source_record) == protocol["source_record_sha256"]
    source = json.loads(gzip.decompress(source_record.read_bytes()))
    main_guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))

    def guard():
        assert sha(protocol_path) == protocol_hash
        assert sha(Path(__file__)) == protocol["runner_sha256"]
        assert sha(E / protocol["probe"]) == protocol["probe_sha256"]
        assert sha(E / protocol["original_probe"]) == protocol["original_probe_sha256"]
        assert sha(E / protocol["rejection_record"]) == protocol["rejection_record_sha256"]
        assert sha(Path(protocol["prelude"])) == protocol["prelude_sha256"]
        for name, digest in protocol["libraries_sha256"].items():
            assert sha(Path(name)) == digest
        for name, digest in source.items():
            assert sha(SOURCE / name) == digest
        for name, digest in main_guard.items():
            assert sha(ROOT / name) == digest

    guard()
    assert protocol["diagnostic_processes"] == 1
    assert protocol["cpu"] in os.sched_getaffinity(0)
    assert os.sysconf("SC_CLK_TCK") == protocol["clock_ticks_per_second"]
    assert protocol["command"][1:] == ["cost", "4", "4", "0", "false", "64"]
    out.mkdir()
    (out / "runner.py").write_bytes(Path(__file__).read_bytes())
    (out / "protocol.json").write_bytes(protocol_path.read_bytes())
    (out / "probe.rs").write_bytes((E / protocol["probe"]).read_bytes())
    binary = Path(protocol["command"][0])
    assert not binary.exists()
    binary.parent.mkdir(parents=True, exist_ok=True)
    build = subprocess.run(protocol["build_command"], cwd=ROOT, capture_output=True, text=True)
    (out / "build.stdout").write_text(build.stdout)
    (out / "build.stderr").write_text(build.stderr)
    (out / "build.json").write_text(json.dumps({"command": protocol["build_command"], "exit_code": build.returncode}, indent=2) + "\n")
    assert build.returncode == 0 and build.stderr == ""
    binary_hash = sha(binary)

    def limits():
        os.sched_setaffinity(0, {protocol["cpu"]})
        resource.setrlimit(resource.RLIMIT_AS, (protocol["address_space_bytes"],) * 2)
        resource.setrlimit(resource.RLIMIT_CPU, tuple(protocol["cpu_seconds"]))
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

    prelude = subprocess.run([protocol["prelude"]], cwd=ROOT, capture_output=True, text=True,
                             preexec_fn=limits, timeout=protocol["wall_seconds"])
    (out / "prelude.json").write_text(json.dumps({"command": [protocol["prelude"]],
        "exit_code": prelude.returncode, "stdout": prelude.stdout, "stderr": prelude.stderr,
        "binary_sha256": protocol["prelude_sha256"]}, indent=2) + "\n")
    assert prelude.returncode == 0 and prelude.stderr == ""
    started = time.monotonic()
    timed_out = False
    with (out / "probe.stdout").open("wb") as stdout, (out / "probe.stderr").open("wb") as stderr:
        try:
            result = subprocess.run(protocol["command"], cwd=ROOT, stdout=stdout, stderr=stderr,
                                    preexec_fn=limits, timeout=protocol["wall_seconds"])
            exit_code = result.returncode
        except subprocess.TimeoutExpired:
            timed_out = True
            exit_code = None
    receipt = {"command": protocol["command"], "exit_code": exit_code,
        "wall_timeout": timed_out, "wall_seconds": time.monotonic() - started,
        "binary_sha256": binary_hash, "cpu": protocol["cpu"],
        "address_space_bytes": protocol["address_space_bytes"],
        "cpu_seconds_soft_hard": protocol["cpu_seconds"], "wall_seconds_limit": protocol["wall_seconds"]}
    (out / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    assert sha(binary) == binary_hash
    guard()
    expected = [("start", None)]
    for index in range(64):
        expected.extend((stage, index) for stage in ["before_fixture", "after_fixture", "after_warmup"])
    expected.extend([( "before_queries", None), ("after_queries", None)])
    markers, other_stderr = [], []
    for line in (out / "probe.stderr").read_text().splitlines():
        try:
            marker = json.loads(line)
        except json.JSONDecodeError:
            other_stderr.append(line)
            continue
        if isinstance(marker, dict) and "diagnostic_stage" in marker:
            markers.append(marker)
        else:
            other_stderr.append(line)
    assert [(row["diagnostic_stage"], row["iteration"]) for row in markers] == expected[:len(markers)]
    assert markers and len(markers) <= len(expected)
    for before, after in zip(markers, markers[1:]):
        assert int(after["wall_ns"]) >= int(before["wall_ns"])
        assert after["user_ticks"] >= before["user_ticks"] and after["system_ticks"] >= before["system_ticks"]
    observations = {"fixture": [], "warmup": [], "queries": []}
    for before, after in zip(markers, markers[1:]):
        pair = (before["diagnostic_stage"], after["diagnostic_stage"])
        category = {("before_fixture", "after_fixture"): "fixture",
                    ("after_fixture", "after_warmup"): "warmup",
                    ("before_queries", "after_queries"): "queries"}.get(pair)
        if category:
            observations[category].append({"iteration": after["iteration"],
                "wall_ns": int(after["wall_ns"]) - int(before["wall_ns"]),
                "cpu_ticks": after["user_ticks"] + after["system_ticks"] - before["user_ticks"] - before["system_ticks"]})
    output = (out / "probe.stdout").read_text()
    completed = exit_code == 0 and not timed_out
    cost_receipt = json.loads(output) if completed else None
    if completed:
        assert len(markers) == 195 and not other_stderr
        assert cost_receipt["stage"] == "cost" and cost_receipt["iterations"] == cost_receipt["certified"] == 64
    totals = {category: {"completed_intervals": len(rows),
        "wall_ns": sum(row["wall_ns"] for row in rows), "cpu_ticks": sum(row["cpu_ticks"] for row in rows)}
        for category, rows in observations.items()}
    summary = {
        "status": "Separately instrumented stage diagnostic completed" if completed else "Separately instrumented stage diagnostic interrupted",
        "source_record_sha256": protocol["source_record_sha256"], "protocol_sha256": protocol_hash,
        "runner_sha256": protocol["runner_sha256"], "guards_unchanged": True,
        "diagnostic_completed": completed, "receipt": receipt, "cost_receipt": cost_receipt,
        "markers": markers, "stage_observations": observations, "stage_totals": totals,
        "clock_ticks_per_second": protocol["clock_ticks_per_second"],
        "last_marker": markers[-1], "next_expected_marker": expected[len(markers)] if len(markers) < len(expected) else None,
        "other_stderr": other_stderr, "runtime_retained": False,
        "original_cost_failure_still_failed": True, "limits": protocol["limits"],
        "artifacts_sha256": {path.name: sha(path) for path in sorted(out.iterdir()) if path.is_file()},
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"diagnostic_completed": completed, "exit_code": exit_code,
        "stage_totals": totals, "last_marker": markers[-1],
        "original_cost_failure_still_failed": True, "runtime_retained": False}))


if __name__ == "__main__":
    main()
