"""Independently qualify the additional test-only storage repair."""
import gzip
import hashlib
import json
import os
import resource
import signal
import subprocess
import time
from pathlib import Path

W = Path(__file__).resolve().parent
OLD = W.parent / "circle-support-recovery"
S = W / "storage-source-v1"
O = W / "storage-validation-v1"
T = W / "storage-lint-target-v1"


def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def read(p):
    return json.loads(Path(p).read_text())


def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2) + "\n")


prep = read(W / "storage-preparation-v1/result.json")
assert prep["complete"] and prep["production_prefixes_unchanged"]
manifest = W / "storage-preparation-v1/source-hashes.json.gz"
assert sha(manifest) == prep["source_hashes_sha256"]
with gzip.open(manifest, "rt") as f:
    inputs = json.load(f)
assert all(sha(S / p) == h for p, h in inputs.items())
previous = read(OLD / "temp-validation-v3/protocol.json")
dependency_pins = previous["read_only_dependency_sha256"]
assert all(sha(p) == h for p, h in dependency_pins.items())
prior = read(OLD / "temp-replay-v4/protocol.json")
assert read(OLD / "temp-replay-v4/result.json")["command_failures"] == []
assert subprocess.check_output(["rustc", "-vV"], text=True) == previous["rustc"]
O.mkdir(exist_ok=False)
T.mkdir(exist_ok=False)
scratch = O / "compiler-scratch"
scratch.mkdir()
selected = O / "selected-temporary-directory"
selected.mkdir()
control = selected / "write-control"
control.write_bytes(b"Current-main tests honor selected temporary storage.\n")
env = os.environ.copy()
for key in ["RUSTFLAGS", "CARGO_ENCODED_RUSTFLAGS", "RUSTC_WRAPPER", "RUSTC_WORKSPACE_WRAPPER", "RUSTDOCFLAGS"]:
    env.pop(key, None)
env.update(prior["environment"])
env.update({
    "CARGO_HOME": "/home/tim/.cargo", "CARGO_TARGET_DIR": str(T), "CARGO_BUILD_JOBS": "2",
    "CARGO_NET_OFFLINE": "true", "CARGO_INCREMENTAL": "0", "CARGO_TERM_COLOR": "never",
    "TMPDIR": str(scratch), "RUST_BACKTRACE": "0",
    "CARGO_MANIFEST_DIR": str(S / "crates/hyperdrc"),
    "CARGO_MANIFEST_PATH": str(S / "crates/hyperdrc/Cargo.toml"),
})
command = read(OLD / "temp-replay-v4/commands.json")[0]["command"].copy()
command = [arg.replace(prior["source"], str(S)) for arg in command]
binary = O / "hyperdrc-tests"
command[command.index("-o") + 1] = str(binary)
pins = {str(p): sha(p) for p in [
    Path(__file__), W / "storage-contract-v1.md", W / "storage-preparation-v1/result.json", manifest,
    OLD / "temp-validation-v3/protocol.json", OLD / "temp-replay-v4/protocol.json",
    OLD / "temp-replay-v4/commands.json", OLD / "temp-replay-v4/result.json", control,
]}
dump(O / "protocol.json", {
    "input_sha256": pins, "source": str(S), "target": str(T), "target_initially_empty": not list(T.iterdir()),
    "read_only_dependency_sha256": dependency_pins, "source_equivalence": str(W / "storage-reproduction-v1/protocol.json"),
    "environment": {k: v for k, v in env.items() if k.startswith("CARGO_") or k in ["TMPDIR", "RUST_BACKTRACE"]},
    "external_timeout_seconds": 1800, "address_space_bytes": 8 * 1024**3,
    "minimum_repository_free_bytes": 768 * 1024**2,
    "cache_candidate_included": False, "closed_cargo_targets_reopened": False,
})
rows = []
space_stop = False


def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def run(name, cmd, overrides=None):
    global space_stop
    print("START " + name, flush=True)
    start = time.monotonic()
    space = []
    reason = None
    with (O / (name + ".stdout")).open("wb") as out, (O / (name + ".stderr")).open("wb") as err:
        proc = subprocess.Popen(["/usr/bin/time", "-v", "-o", str(O / (name + ".resources")), *cmd],
                                cwd=S, env=env | (overrides or {}), stdout=out, stderr=err,
                                preexec_fn=limits, start_new_session=True)
        dump(O / (name + ".process.json"), {"pid": proc.pid, "command": cmd, "started_unix": time.time()})
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
    assert all(sha(S / p) == h for p, h in inputs.items())
    assert all(sha(p) == h for p, h in pins.items())
    row = {
        "name": name, "command": cmd, "environment_overrides": overrides or {}, "exit_code": code,
        "elapsed_seconds": time.monotonic() - start, "external_stop_reason": reason,
        "free_space_observations": space,
        **{suffix + "_sha256": sha(O / (name + "." + suffix)) for suffix in ["stdout", "stderr", "resources"]},
    }
    rows.append(row)
    dump(O / "commands.json", rows)
    space_stop |= reason == "repository-space-reserve"
    print(json.dumps({k: row[k] for k in ["name", "exit_code", "elapsed_seconds"]}), flush=True)
    return code


run("format", ["cargo", "fmt", "--all", "--", "--check"])
code = run("compile", command)
if code == 0 and not space_stop:
    dump(O / "binary.json", {"path": str(binary), "sha256": sha(binary), "bytes": binary.stat().st_size})
    run("app-tests", [str(binary), "app::tests::", "--nocapture", "--test-threads=1"], {"TMPDIR": str(selected)})
    run("arrow-test", [str(binary), "arrow_report::tests::writes_readable_arrow_report", "--exact", "--nocapture", "--test-threads=1"], {"TMPDIR": str(selected)})
if not space_stop:
    run("clippy", ["cargo", "clippy", "--offline", "--locked", "-p", "hyperdrc", "--release", "--all-features", "--all-targets", "--", "-D", "warnings"])
assert all(sha(p) == h for p, h in dependency_pins.items())
assert list(selected.iterdir()) == [control]
dump(O / "result.json", {
    "complete": True, "command_failures": [r["name"] for r in rows if r["exit_code"]],
    "source_and_read_only_dependencies_unchanged": True, "selected_temporary_directory_cleaned": True,
    "cargo_target_closed": True, "cache_candidate_included": False, "source_changes_applied": False,
    "production_prefixes_unchanged": True, "implementation_sha256": prep["implementation_sha256"],
})
raise SystemExit(101 if any(r["exit_code"] for r in rows) else 0)
