"""Reproduce the five remaining storage failures with the current-main harness."""
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
ROOT = W.parents[2]
PREVIOUS = W.parent / "circle-support-recovery"
O = W / "storage-reproduction-v1"


def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def read(p):
    return json.loads(Path(p).read_text())


def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2) + "\n")


assert read(W / "native-corpus-v1/result.json")["complete"]
compiled = read(PREVIOUS / "temp-replay-v4/binary.json")
binary = Path(compiled["path"])
assert sha(binary) == compiled["sha256"]
with gzip.open(PREVIOUS / "temp-preparation-v1/source-hashes.json.gz", "rt") as f:
    old_inputs = json.load(f)
with gzip.open(W / "preparation-v1/main-input-hashes.json.gz", "rt") as f:
    main = json.load(f)
differences = {p for p, h in old_inputs.items() if main[p] != h}
assert differences == {"docs/properties-status.json"}
assert all(sha(ROOT / p) == h for p, h in main.items())
assert all(sha(PREVIOUS / "temp-candidate-source-v1" / p) == h for p, h in old_inputs.items())
O.mkdir(exist_ok=False)
scratch = O / "selected-temporary-directory"
scratch.mkdir()
control = scratch / "write-control"
control.write_bytes(b"Selected test temporary storage is writable.\n")
tests = [
    ("app::tests::load_all_layers_discovers_all_input_channels_and_marks_conversion_origin", "/tmp/hyperdrc-load-all-layers-{pid}"),
    ("app::tests::load_all_layers_discovers_extracted_package_archives_recursively", "/tmp/hyperdrc-archive-layers-{pid}"),
    ("app::tests::load_layers_preserves_gerber_x2_file_metadata_for_manifest", "/tmp/hyperdrc-x2-metadata-{pid}.gbr"),
    ("app::tests::load_all_layers_wraps_parse_errors_with_file_path_context", "/tmp/hyperdrc-load-all-layers-parse-{pid}"),
    ("arrow_report::tests::writes_readable_arrow_report", "/tmp/hyperdrc-report-{pid}.arrow"),
]
pins = {str(p): sha(p) for p in [
    Path(__file__), W / "storage-contract-v1.md", W / "native-corpus-v1/result.json",
    PREVIOUS / "temp-replay-v4/binary.json", binary, control,
    PREVIOUS / "temp-preparation-v1/source-hashes.json.gz",
]}
dump(O / "protocol.json", {
    "input_sha256": pins, "binary": compiled, "source_differences_from_current_main": sorted(differences),
    "implementation_matches_current_main": True, "cache_candidate_included": False,
    "TMPDIR": str(scratch), "timeout_seconds_per_test": 60, "address_space_bytes": 8 * 1024**3,
    "closed_sources_and_targets_used_read_only": True,
})
rows = []
roots = []
for index, (test, template) in enumerate(tests):
    name = "test-" + str(index + 1)
    command = [str(binary), test, "--exact", "--nocapture", "--test-threads=1"]

    def limits():
        resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
        assert not os.path.lexists(template.format(pid=os.getpid())), "Do not overwrite a retained earlier PID's fixture."

    start = time.monotonic()
    with (O / (name + ".stdout")).open("wb") as out, (O / (name + ".stderr")).open("wb") as err:
        proc = subprocess.Popen(command, cwd=O, env=os.environ | {"TMPDIR": str(scratch), "RUST_BACKTRACE": "0"},
                                stdout=out, stderr=err, preexec_fn=limits, start_new_session=True)
        pid = proc.pid
        try:
            code = proc.wait(timeout=60)
        except subprocess.TimeoutExpired:
            os.killpg(pid, signal.SIGTERM)
            try:
                code = proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(pid, signal.SIGKILL)
                code = proc.wait()
    root = Path(template.format(pid=pid))
    if root.exists():
        roots.append(str(root))
    row = {
        "name": name, "test": test, "command": command, "pid": pid, "exit_code": code,
        "elapsed_seconds": time.monotonic() - start,
        "quota_error": "Disk quota exceeded" in (O / (name + ".stderr")).read_text(),
        "hardcoded_temporary_path": str(root),
        **{s + "_sha256": sha(O / (name + "." + s)) for s in ["stdout", "stderr"]},
    }
    rows.append(row)
    dump(O / "commands.json", rows)
native_output = (W / "native-corpus-v1/library.stderr").read_text()
native_pid = int(read(W / "native-corpus-v1/library.process.json")["pid"])
# The recorded PID belongs to /usr/bin/time. The Arrow diagnostic identifies
# the native executable's process ID, shared by all five failed fixture paths.
import re
match = re.search(r"/tmp/hyperdrc-report-(\d+)\.arrow", native_output)
assert match
native_test_pid = int(match.group(1))
native_roots = [template.format(pid=native_test_pid) for _, template in tests]
assert all(Path(p).exists() for p in native_roots)
roots.extend(native_roots)
artifacts = {}
for root in map(Path, roots):
    for p in ([root] if root.is_file() else root.rglob("*")):
        if p.is_file():
            artifacts[str(p)] = {"sha256": sha(p), "bytes": p.stat().st_size}
assert all(sha(p) == h for p, h in pins.items())
assert list(scratch.iterdir()) == [control]
dump(O / "result.json", {
    "complete": True, "reproduced_failures": sum(row["exit_code"] == 101 and row["quota_error"] for row in rows),
    "commands": rows, "selected_temporary_directory_ignored": True,
    "temporary_roots": roots, "raw_failed_artifacts": artifacts,
    "native_time_supervisor_pid": native_pid, "native_test_pid": native_test_pid,
    "source_and_binary_unchanged": True, "cache_candidate_included": False,
})
print(json.dumps({"reproduced_failures": sum(row["exit_code"] == 101 and row["quota_error"] for row in rows),
                  "retained_temporary_roots": len(roots)}))
