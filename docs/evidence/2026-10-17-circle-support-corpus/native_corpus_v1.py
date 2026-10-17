"""Exercise the complete native library, integrations and docs with fresh scratch."""
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
OLD = W.parent / "circle-support-index/candidate-qualification-v2"
B = W / "native-build-v1"
O = W / "native-corpus-v1"


def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def read(p):
    return json.loads(Path(p).read_text())


def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2) + "\n")


assert read(W / "default-corpus-v1/result.json")["complete"], "Do not overlap the original native timing gates with the default corpus."
assert read(W / "curve-all-v1/build-phase-result.json")["complete"], "Do not overlap the native timing gates with the independent all-feature build."
assert read(B / "result.json")["complete"] and not read(B / "result.json")["command_failures"]
protocol = read(B / "protocol.json")
compiled = read(B / "binary.json")
binary = Path(compiled["path"])
assert sha(binary) == compiled["sha256"]
runtime = Path(protocol["runtime_source_mirror"])
with gzip.open(protocol["source_manifest"], "rt") as f:
    inputs = json.load(f)
assert all(sha(runtime / p) == h for p, h in inputs.items())
assert all(sha(p) == h for p, h in protocol["read_only_dependency_sha256"].items())
previous_build = next(r for r in read(OLD / "commands.json") if r["name"] == "native-build")
assert previous_build["exit_code"] == 0
integration = [
    a for a in previous_build["binaries"].values()
    if a["profile"]["test"] and a["target"]["kind"] != ["lib"]
]
assert len(integration) == 7
for a in integration:
    assert sha(a["retained"]) == a["sha256"]
cli = {
    p: a for p, a in previous_build["binaries"].items()
    if not a["profile"]["test"] and a["target"]["kind"] == ["bin"]
}
assert len(cli) == 1
for p, a in cli.items():
    assert sha(p) == a["sha256"] and sha(a["retained"]) == a["sha256"]
O.mkdir(exist_ok=False)
scratch = O / "selected-temporary-directory"
scratch.mkdir()
control = scratch / "write-control"
control.write_bytes(b"Full native corpus has writable temporary storage.\n")
env = os.environ.copy()
for key in ["CARGO_TARGET_DIR", "RUSTFLAGS", "CARGO_ENCODED_RUSTFLAGS", "RUSTC_WRAPPER", "RUSTC_WORKSPACE_WRAPPER", "RUSTDOCFLAGS"]:
    env.pop(key, None)
env.update(protocol["environment"])
env.update({"TMPDIR": str(scratch), "RUST_BACKTRACE": "0"})
pins = {str(p): sha(p) for p in [
    Path(__file__), W / "contract-v1.md", W / "default-corpus-v1/result.json",
    B / "protocol.json", B / "commands.json", B / "result.json", B / "binary.json",
    W / "curve-all-v1/build-phase-result.json",
    OLD / "commands.json", control,
]}
pins.update({p: a["sha256"] for p, a in cli.items()})
dump(O / "protocol.json", {
    "input_sha256": pins, "library_binary": compiled, "integration_binaries": integration,
    "compiled_cli_paths_used_by_integration_tests": cli,
    "native_build_protocol": str(B / "protocol.json"), "cwd": str(runtime), "TMPDIR": str(scratch),
    "full_library_selection": "Entire test binary, including all three original performance gates; no added skips or filters.",
    "integration_reuse_proof": "Production implementation, all integration source, fixtures, dependency lock and package features match the successful E38 build. Only three library cfg(test) storage sections and the ledger changed.",
    "features": [], "cargo_all_features_equivalent": True,
    "default_corpus_terminal_before_start": True,
    "existing_generated_test_environment": {k: v for k, v in env.items() if k.startswith("PROPTEST_")},
    "external_timeout_seconds": 1800, "address_space_bytes": 8 * 1024**3,
    "minimum_repository_free_bytes": 768 * 1024**2,
    "cargo_targets_reopened": False, "cache_admitted": False,
})
rows = []


def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def run(name, command):
    print("START " + name, flush=True)
    start = time.monotonic()
    space = []
    reason = None
    with (O / (name + ".stdout")).open("wb") as out, (O / (name + ".stderr")).open("wb") as err:
        proc = subprocess.Popen(
            ["/usr/bin/time", "-v", "-o", str(O / (name + ".resources")), *command],
            cwd=runtime, env=env, stdout=out, stderr=err, preexec_fn=limits, start_new_session=True,
        )
        dump(O / (name + ".process.json"), {"pid": proc.pid, "command": command, "started_unix": time.time()})
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
    output = (O / (name + ".stdout")).read_text()
    summaries = [
        {"passed": int(p), "failed": int(f), "ignored": int(i), "measured": int(m), "filtered_out": int(x)}
        for p, f, i, m, x in re.findall(
            r"test result: \w+\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;",
            output,
        )
    ]
    row = {
        "name": name, "command": command, "exit_code": code, "external_stop_reason": reason,
        "elapsed_seconds": time.monotonic() - start, "free_space_observations": space,
        "test_summaries": summaries,
        **{s + "_sha256": sha(O / (name + "." + s)) for s in ["stdout", "stderr", "resources"]},
    }
    rows.append(row)
    dump(O / "commands.json", rows)
    print(json.dumps({k: row[k] for k in ["name", "exit_code", "elapsed_seconds", "test_summaries"]}), flush=True)
    return code


run("library", [str(binary), "--nocapture", "--test-threads=1"])
for a in integration:
    run("integration-" + a["target"]["name"], [a["retained"], "--nocapture", "--test-threads=1"])
build = read(B / "commands.json")[0]["command"]
doc = ["rustdoc", "--crate-name", "hyperdrc", "--edition=2024", "--test",
       str(runtime / "crates/hyperdrc/src/lib.rs"), "-D", "warnings",
       "--test-args", "--test-threads=1"]
for i, arg in enumerate(build):
    if arg in ["-L", "--extern"]:
        doc += [arg, build[i + 1]]
old_artifacts = [json.loads(line) for line in (OLD / "native-build.stdout").read_text().splitlines() if line.startswith("{")]
library = [
    p for a in old_artifacts
    if a.get("reason") == "compiler-artifact" and a["target"]["name"] == "hyperdrc" and a["target"]["kind"] == ["lib"]
    for p in a["filenames"] if p.endswith(".rlib")
]
assert len(library) == 1 and library[0] in protocol["read_only_dependency_sha256"]
doc += ["--extern", "hyperdrc=" + library[0]]
run("doctests", doc)
assert sha(binary) == compiled["sha256"]
assert all(sha(a["retained"]) == a["sha256"] for a in integration)
assert all(sha(p) == h for p, h in protocol["read_only_dependency_sha256"].items())
generated = {
    str(p.relative_to(runtime)): sha(p) for p in runtime.rglob("*")
    if p.is_file() and str(p.relative_to(runtime)) not in inputs
}
dump(O / "result.json", {
    "complete": True, "command_failures": [r["name"] for r in rows if r["exit_code"]],
    "passed_tests": sum(s["passed"] for r in rows for s in r["test_summaries"]),
    "failed_tests": sum(s["failed"] for r in rows for s in r["test_summaries"]),
    "source_and_binaries_and_dependencies_unchanged": True,
    "generated_regression_outputs": generated,
    "selected_temporary_directory_cleaned": list(scratch.iterdir()) == [control],
    "temporary_leftovers": [str(p) for p in scratch.iterdir() if p != control],
    "original_performance_gates_included": True,
    "cargo_targets_reopened": False, "cache_admitted": False, "full_plan_complete": False,
})
raise SystemExit(101 if any(r["exit_code"] for r in rows) else 0)
