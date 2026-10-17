"""Run a fresh Cargo all-feature curve qualification within available disk space."""
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
S = W / "candidate-source-v1"
O = W / "curve-all-v1"
T = W / "curve-all-target-v1"


def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def read(p):
    return json.loads(Path(p).read_text())


def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2) + "\n")


prep = read(W / "preparation-v1/result.json")
assert prep["complete"]
manifest = W / "preparation-v1/source-hashes.json.gz"
assert sha(manifest) == prep["source_hashes_sha256"]
with gzip.open(manifest, "rt") as f:
    inputs = json.load(f)
assert all(sha(S / p) == h for p, h in inputs.items())
fs = os.statvfs(W)
assert fs.f_bavail * fs.f_frsize > 2 * 1024**3, "Need build headroom in addition to the free-space reserve."
O.mkdir(exist_ok=False)
T.mkdir(exist_ok=False)
scratch = O / "scratch"
scratch.mkdir()
env = os.environ.copy()
for k in ["RUSTFLAGS", "CARGO_ENCODED_RUSTFLAGS", "RUSTC_WRAPPER", "RUSTC_WORKSPACE_WRAPPER", "RUSTDOCFLAGS"]:
    env.pop(k, None)
env.update({
    "CARGO_HOME": "/home/tim/.cargo", "CARGO_TARGET_DIR": str(T), "CARGO_BUILD_JOBS": "2",
    "CARGO_NET_OFFLINE": "true", "CARGO_INCREMENTAL": "0", "CARGO_TERM_COLOR": "never",
    "TMPDIR": str(scratch), "RUST_BACKTRACE": "0",
})
pins = {str(p): sha(p) for p in [
    Path(__file__), manifest, W / "preparation-v1/result.json", W / "contract-v1.md",
]}
dump(O / "protocol.json", {
    "input_sha256": pins, "source": str(S), "target": str(T), "target_initially_empty": not list(T.iterdir()),
    "rustc": subprocess.check_output(["rustc", "-vV"], text=True),
    "cargo": subprocess.check_output(["cargo", "-V"], text=True),
    "hardware": subprocess.check_output(["lscpu"], text=True),
    "environment": {k: env[k] for k in ["CARGO_HOME", "CARGO_TARGET_DIR", "CARGO_BUILD_JOBS", "CARGO_NET_OFFLINE", "CARGO_INCREMENTAL", "TMPDIR"]},
    "existing_generated_test_environment": {k: v for k, v in env.items() if k.startswith("PROPTEST_")},
    "external_timeout_seconds_per_command": 1800, "address_space_bytes_per_process": 8 * 1024**3,
    "minimum_repository_free_bytes": 768 * 1024**2, "space_check_interval_seconds": 5,
    "build_estimate": "An earlier matching all-feature Cargo build lists 1,333,395,887 bytes of retained compiler artifacts. This is a planning observation, not an internal memory or disk bound.",
    "execution_order": "The independent all-feature build may overlap the default functional suite. Native timing gates wait for this build to finish; all-feature tests and Clippy wait for the native corpus to finish.",
    "completed_cargo_targets_reopened": False,
    "scope": "Entire Hypercurve all-feature library, integration and documentation corpus, plus all-feature/all-target strict Clippy.",
})
rows = []
stop = False


def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def run(name, command):
    global stop
    print("START " + name, flush=True)
    start = time.monotonic()
    free = []
    reason = None
    with (O / (name + ".stdout")).open("wb") as out, (O / (name + ".stderr")).open("wb") as err:
        proc = subprocess.Popen(
            ["/usr/bin/time", "-v", "-o", str(O / (name + ".resources")), *command],
            cwd=S, env=env, stdout=out, stderr=err, preexec_fn=limits, start_new_session=True,
        )
        dump(O / (name + ".process.json"), {"pid": proc.pid, "command": command, "started_unix": time.time()})
        while proc.poll() is None:
            fs = os.statvfs(W)
            available = fs.f_bavail * fs.f_frsize
            elapsed = time.monotonic() - start
            free.append({"elapsed_seconds": elapsed, "available_bytes": available})
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
    binaries = {}
    for line in (O / (name + ".stdout")).read_text().splitlines():
        if not line.startswith("{"):
            continue
        artifact = json.loads(line)
        if artifact.get("reason") == "compiler-artifact" and artifact.get("executable"):
            p = Path(artifact["executable"])
            binaries[str(p)] = {
                "sha256": sha(p), "bytes": p.stat().st_size, "features": artifact["features"],
                "target": artifact["target"], "profile": artifact["profile"],
            }
    row = {
        "name": name, "command": command, "exit_code": code, "external_stop_reason": reason,
        "elapsed_seconds": time.monotonic() - start, "free_space_observations": free, "binaries": binaries,
        **{s + "_sha256": sha(O / (name + "." + s)) for s in ["stdout", "stderr", "resources"]},
    }
    rows.append(row)
    dump(O / "commands.json", rows)
    if reason == "repository-space-reserve":
        stop = True
    print(json.dumps({k: row[k] for k in ["name", "exit_code", "elapsed_seconds", "external_stop_reason"]}), flush=True)
    return code


build = ["cargo", "test", "--offline", "--locked", "-p", "hypercurve", "--release", "--all-features"]
code = run("build", [*build, "--no-run", "--message-format=json"])
dump(O / "build-phase-result.json", {
    "complete": True, "exit_code": code, "command_record": str(O / "commands.json"),
    "next_build_or_test_waits_for_native_corpus": True,
})
if not stop:
    print("WAIT for the native corpus before all-feature tests or Clippy", flush=True)
    wait_started = time.monotonic()
    while not (W / "native-corpus-v1/result.json").exists():
        assert time.monotonic() - wait_started < 3600, "Native corpus did not finish within the external orchestration wait."
        time.sleep(5)
    assert read(W / "native-corpus-v1/result.json")["complete"]
    dump(O / "native-gates-before-tests.json", {
        "result_path": str(W / "native-corpus-v1/result.json"),
        "result_sha256": sha(W / "native-corpus-v1/result.json"),
        "wait_seconds": time.monotonic() - wait_started,
    })
if code == 0 and not stop:
    run("tests", [*build, "--no-fail-fast", "--", "--test-threads=1"])
if not stop:
    run("clippy", ["cargo", "clippy", "--offline", "--locked", "-p", "hypercurve", "--release", "--all-features", "--all-targets", "--", "-D", "warnings"])
generated = {
    str(p.relative_to(S)): sha(p) for p in S.rglob("*")
    if p.is_file() and str(p.relative_to(S)) not in inputs
}
dump(O / "result.json", {
    "complete": True, "command_failures": [r["name"] for r in rows if r["exit_code"]],
    "sources_unchanged": True, "generated_outputs": generated, "cargo_target_closed": True,
    "repository_space_stop": stop, "cache_admitted": False, "full_plan_complete": False,
})
raise SystemExit(101 if any(r["exit_code"] for r in rows) else 0)
