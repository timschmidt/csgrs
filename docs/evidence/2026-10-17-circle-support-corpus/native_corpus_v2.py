"""Replay the full native library after the repair; reuse unchanged consumer checks."""
import gzip
import hashlib
import json
import os
import re
import resource
import signal
import subprocess
import time
from pathlib import Path

W = Path(__file__).resolve().parent
B = W / "native-build-v2"
O = W / "native-corpus-v2"


def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def read(p):
    return json.loads(Path(p).read_text())


def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2) + "\n")


assert read(W / "curve-all-v1/result.json")["complete"]
assert read(W / "curve-all-v1/result.json")["cargo_target_closed"]
assert read(W / "storage-validation-v1/result.json")["command_failures"] == []
assert read(B / "result.json")["complete"] and read(B / "result.json")["command_failures"] == []
protocol = read(B / "protocol.json")
old_protocol = read(W / "native-build-v1/protocol.json")
assert protocol["read_only_dependency_sha256"] == old_protocol["read_only_dependency_sha256"]
compiled = read(B / "binary.json")
binary = Path(compiled["path"])
assert sha(binary) == compiled["sha256"]
runtime = Path(protocol["runtime_source_mirror"])
with gzip.open(protocol["source_manifest"], "rt") as f:
    inputs = json.load(f)
with gzip.open(old_protocol["source_manifest"], "rt") as f:
    old_inputs = json.load(f)
repair = read(W / "storage-preparation-v1/result.json")["implementation_sha256"]
assert {p: h for p, h in inputs.items() if h != old_inputs[p]} == repair
for p in repair:
    assert (runtime / p).read_text().split("#[cfg(test)]", 1)[0] == (Path(old_protocol["runtime_source_mirror"]) / p).read_text().split("#[cfg(test)]", 1)[0]
assert all(sha(runtime / p) == h for p, h in inputs.items())
assert all(sha(p) == h for p, h in protocol["read_only_dependency_sha256"].items())
prior_commands = read(W / "native-corpus-v1/commands.json")
reused = [r for r in prior_commands if r["name"] != "library"]
assert len(reused) == 8 and all(r["exit_code"] == 0 for r in reused)
for row in reused:
    for suffix in ["stdout", "stderr", "resources"]:
        assert sha(W / "native-corpus-v1" / (row["name"] + "." + suffix)) == row[suffix + "_sha256"]
consumer_protocol = read(W / "native-corpus-v1/protocol.json")
for artifact in consumer_protocol["integration_binaries"]:
    assert sha(artifact["retained"]) == artifact["sha256"]
for path, artifact in consumer_protocol["compiled_cli_paths_used_by_integration_tests"].items():
    assert sha(path) == artifact["sha256"]
O.mkdir(exist_ok=False)
scratch = O / "selected-temporary-directory"
scratch.mkdir()
control = scratch / "write-control"
control.write_bytes(b"Full native replay uses configured temporary storage.\n")
env = os.environ.copy()
env.update(protocol["environment"])
env.update({"TMPDIR": str(scratch), "RUST_BACKTRACE": "0"})
pins = {str(p): sha(p) for p in [
    Path(__file__), W / "storage-contract-v1.md", W / "curve-all-v1/result.json",
    W / "storage-validation-v1/result.json", W / "storage-preparation-v1/result.json",
    B / "protocol.json", B / "commands.json", B / "result.json", B / "binary.json",
    W / "native-corpus-v1/commands.json", W / "native-corpus-v1/protocol.json", control,
]}
command = [str(binary), "--nocapture", "--test-threads=1"]
dump(O / "protocol.json", {
    "input_sha256": pins, "library_binary": compiled, "native_build_protocol": str(B / "protocol.json"),
    "source": str(runtime), "command": command, "TMPDIR": str(scratch),
    "source_changes_since_first_native_run": repair, "production_prefixes_unchanged": True,
    "reused_integration_and_doc_commands": reused,
    "reuse_proof": "All production source and read-only dependencies are identical; only five fixture path constructors and an unused test import differ. The seven consumer executables, their CLI and the three doc tests already passed in native-corpus-v1.",
    "features": [], "cargo_all_features_equivalent": True,
    "selection": "Entire native library, including all three original workload gates; no filters or added skips.",
    "curve_checks_and_other_task_builds_terminal_before_start": True,
    "external_timeout_seconds": 1800, "address_space_bytes": 8 * 1024**3,
    "minimum_repository_free_bytes": 768 * 1024**2, "cargo_targets_reopened": False,
})


def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


start = time.monotonic()
space = []
reason = None
print("START complete repaired native library", flush=True)
with (O / "library.stdout").open("wb") as out, (O / "library.stderr").open("wb") as err:
    proc = subprocess.Popen(
        ["/usr/bin/time", "-v", "-o", str(O / "library.resources"), *command],
        cwd=runtime, env=env, stdout=out, stderr=err, preexec_fn=limits, start_new_session=True,
    )
    dump(O / "library.process.json", {"pid": proc.pid, "command": command, "started_unix": time.time()})
    while proc.poll() is None:
        fs = os.statvfs(W)
        available = fs.f_bavail * fs.f_frsize
        elapsed = time.monotonic() - start
        space.append({"elapsed_seconds": elapsed, "available_bytes": available})
        if available < 768 * 1024**2 or elapsed > 1800:
            reason = "repository-space-reserve" if available < 768 * 1024**2 else "external-timeout"
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
            break
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass
    code = proc.wait()
assert all(sha(runtime / p) == h for p, h in inputs.items())
assert all(sha(p) == h for p, h in pins.items())
assert all(sha(p) == h for p, h in protocol["read_only_dependency_sha256"].items())
assert sha(binary) == compiled["sha256"]
summaries = [
    {"passed": int(p), "failed": int(f), "ignored": int(i), "measured": int(m), "filtered_out": int(x)}
    for p, f, i, m, x in re.findall(
        r"test result: \w+\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;",
        (O / "library.stdout").read_text(),
    )
]
row = {
    "name": "library", "command": command, "exit_code": code, "external_stop_reason": reason,
    "elapsed_seconds": time.monotonic() - start, "free_space_observations": space, "test_summaries": summaries,
    **{s + "_sha256": sha(O / ("library." + s)) for s in ["stdout", "stderr", "resources"]},
}
dump(O / "commands.json", [row])
generated = {
    str(p.relative_to(runtime)): sha(p) for p in runtime.rglob("*")
    if p.is_file() and str(p.relative_to(runtime)) not in inputs
}
dump(O / "result.json", {
    "complete": True, "command_failures": [] if code == 0 else ["library"],
    "passed_library_tests": sum(s["passed"] for s in summaries),
    "failed_library_tests": sum(s["failed"] for s in summaries),
    "reused_passed_consumer_and_doc_tests": sum(s["passed"] for r in reused for s in r["test_summaries"]),
    "source_and_binaries_and_dependencies_unchanged": True,
    "generated_regression_outputs": generated,
    "selected_temporary_directory_cleaned": list(scratch.iterdir()) == [control],
    "temporary_leftovers": [str(p) for p in scratch.iterdir() if p != control],
    "original_performance_gates_included": True, "original_failures_retained": str(W / "native-corpus-v1/result.json"),
    "cargo_targets_reopened": False, "cache_admitted": False, "full_plan_complete": False,
})
print(json.dumps({"exit_code": code, "elapsed_seconds": row["elapsed_seconds"], "test_summaries": summaries}), flush=True)
raise SystemExit(code)
