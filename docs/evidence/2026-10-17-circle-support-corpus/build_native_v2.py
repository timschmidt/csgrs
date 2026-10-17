"""Build current candidate native tests using verified, read-only Cargo artifacts."""
import gzip
import hashlib
import json
import os
import resource
import shutil
import signal
import subprocess
import time
import tomllib
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
OLD = W.parent / "circle-support-index"
S = W / "candidate-source-v2"
O = W / "native-build-v2"


def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def read(p):
    return json.loads(Path(p).read_text())


def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2) + "\n")


prep = read(W / "preparation-v2/result.json")
assert prep["complete"] and prep["production_implementation_identical_to_e38_candidate"]
manifest = W / "preparation-v2/source-hashes.json.gz"
assert sha(manifest) == prep["source_hashes_sha256"]
with gzip.open(manifest, "rt") as f:
    inputs = json.load(f)
assert all(sha(S / p) == h for p, h in inputs.items())
previous = OLD / "candidate-qualification-v2"
prior = read(previous / "result.json")
assert prior["complete"] and prior["cargo_target_closed"]
previous_protocol = read(previous / "protocol.json")
rustc = subprocess.check_output(["rustc", "-vV"], text=True)
cargo = subprocess.check_output(["cargo", "-V"], text=True)
assert rustc == previous_protocol["rustc"] and cargo == previous_protocol["cargo"]
previous_build = next(r for r in read(previous / "commands.json") if r["name"] == "native-build")
assert previous_build["exit_code"] == 0
build_log = previous / "native-build.stdout"
assert sha(build_log) == previous_build["stdout_sha256"]
artifacts = [json.loads(line) for line in build_log.read_text().splitlines() if line.startswith("{")]
package = tomllib.loads((S / "crates/hyperdrc/Cargo.toml").read_text())
assert not package.get("features")
lock = tomllib.loads((S / "Cargo.lock").read_text())
locked = next(p["dependencies"] for p in lock["package"] if p["name"] == "hyperdrc")
versions = {s.split()[0]: s.split()[1] for s in locked if len(s.split()) > 1}
externs = {}
for dependency in [*package["dependencies"], *package["dev-dependencies"]]:
    name = dependency.replace("-", "_")
    paths = {
        p for a in artifacts
        if a.get("reason") == "compiler-artifact" and a["target"]["name"] == name
        and (dependency not in versions or a["package_id"].endswith("@" + versions[dependency]))
        for p in a["filenames"] if p.endswith(".rlib")
    }
    assert len(paths) == 1, (name, paths)
    externs[name] = paths.pop()
deps = Path(previous_protocol["target"]) / "release/deps"
linked_paths = sorted({
    p for a in artifacts if a.get("reason") == "build-script-executed"
    for p in a.get("linked_paths", [])
})
dependency_files = {
    p for p in deps.iterdir() if p.is_file() and p.suffix in [".rlib", ".rmeta", ".so"]
}
for linked in linked_paths:
    directory = Path(linked.split("=", 1)[-1])
    assert directory.is_dir(), linked
    dependency_files.update(p for p in directory.rglob("*") if p.is_file())
expectations_path = ROOT / "docs/evidence/2026-10-17-circle-support-index/artifact-expectations-v1.json.gz"
with gzip.open(expectations_path, "rt") as f:
    expected = json.load(f)["added_physical_artifacts_sha256"]
dependency_pins = {}
for p in sorted(dependency_files):
    h = sha(p)
    assert expected[str(p)] == h, p
    dependency_pins[str(p)] = h
assert all(path in dependency_pins for path in externs.values())
O.mkdir(exist_ok=False)
runtime = O / "runtime-source-mirror"
runtime.mkdir()
for p, h in inputs.items():
    dest = runtime / p
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(S / p, dest)
    assert sha(dest) == h
scratch = O / "compiler-scratch"
scratch.mkdir()
package = package["package"]
env = os.environ.copy()
for key in ["CARGO_TARGET_DIR", "RUSTFLAGS", "CARGO_ENCODED_RUSTFLAGS", "RUSTC_WRAPPER", "RUSTC_WORKSPACE_WRAPPER", "RUSTDOCFLAGS"]:
    env.pop(key, None)
env.update({
    "TMPDIR": str(scratch), "RUST_BACKTRACE": "0",
    "CARGO_MANIFEST_DIR": str(runtime / "crates/hyperdrc"),
    "CARGO_MANIFEST_PATH": str(runtime / "crates/hyperdrc/Cargo.toml"),
    "CARGO_PKG_NAME": package["name"], "CARGO_CRATE_NAME": "hyperdrc",
    "CARGO_PKG_VERSION": package["version"],
    "CARGO_PKG_AUTHORS": ":".join(package.get("authors", [])),
    **{"CARGO_PKG_" + name.upper(): package.get(name.replace("_", "-"), "")
       for name in ["description", "homepage", "repository", "license", "license_file", "rust_version"]},
})
major, minor, patch = package["version"].split(".")
env.update({"CARGO_PKG_VERSION_MAJOR": major, "CARGO_PKG_VERSION_MINOR": minor,
            "CARGO_PKG_VERSION_PATCH": patch, "CARGO_PKG_VERSION_PRE": ""})
binary = O / "hyperdrc-tests"
command = [
    "rustc", "--crate-name", "hyperdrc", "--edition=2024", "--test",
    str(runtime / "crates/hyperdrc/src/lib.rs"),
    "-C", "opt-level=3", "-C", "codegen-units=16", "-C", "embed-bitcode=no",
    "-D", "warnings", "-L", "dependency=" + str(deps), "-o", str(binary),
]
for name, path in externs.items():
    command += ["--extern", name + "=" + path]
for path in linked_paths:
    command += ["-L", path]
pins = {str(p): sha(p) for p in [
    Path(__file__), manifest, W / "preparation-v2/result.json", W / "contract-v1.md",
    previous / "protocol.json", previous / "commands.json", previous / "result.json",
    build_log, expectations_path,
]}
dump(O / "protocol.json", {
    "input_sha256": pins, "source": str(S), "runtime_source_mirror": str(runtime),
    "source_manifest": str(manifest), "source_equivalence": prep["changes_from_e38_candidate"],
    "read_only_dependency_sha256": dependency_pins, "externs": externs,
    "rustc": rustc, "cargo": cargo, "command": command, "features": [],
    "environment": {k: v for k, v in env.items()
                    if k.startswith("CARGO_PKG_") or k in ["CARGO_MANIFEST_DIR", "CARGO_MANIFEST_PATH", "CARGO_CRATE_NAME", "TMPDIR"]},
    "external_timeout_seconds": 1800, "address_space_bytes": 8 * 1024**3,
    "minimum_repository_free_bytes": 768 * 1024**2,
    "cargo_targets_reopened": False, "cache_candidate_included": True,
    "production_source_matches_successful_native_cargo_build": True,
    "only_test_code_differs_from_e38": "The three independently admitted E39 edits plus the independently qualified two-file E40 test-storage repair.",
})


def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


start = time.monotonic()
space = []
reason = None
print("START compile full native test harness", flush=True)
with (O / "compile.stdout").open("wb") as out, (O / "compile.stderr").open("wb") as err:
    proc = subprocess.Popen(
        ["/usr/bin/time", "-v", "-o", str(O / "compile.resources"), *command],
        cwd=runtime, env=env, stdout=out, stderr=err, preexec_fn=limits, start_new_session=True,
    )
    dump(O / "process.json", {"pid": proc.pid, "command": command, "started_unix": time.time()})
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
assert all(sha(S / p) == h and sha(runtime / p) == h for p, h in inputs.items())
assert all(sha(p) == h for p, h in dependency_pins.items())
assert all(sha(p) == h for p, h in pins.items())
row = {
    "name": "compile", "command": command, "exit_code": code, "external_stop_reason": reason,
    "elapsed_seconds": time.monotonic() - start, "free_space_observations": space,
    **{s + "_sha256": sha(O / ("compile." + s)) for s in ["stdout", "stderr", "resources"]},
}
dump(O / "commands.json", [row])
if code == 0:
    dump(O / "binary.json", {"path": str(binary), "sha256": sha(binary), "bytes": binary.stat().st_size})
dump(O / "result.json", {
    "complete": True, "command_failures": [] if code == 0 else ["compile"],
    "source_and_read_only_dependencies_unchanged": True,
    "cargo_targets_reopened": False, "cache_admitted": False,
})
print(json.dumps({"complete": True, "exit_code": code, "elapsed_seconds": row["elapsed_seconds"]}), flush=True)
raise SystemExit(code)
