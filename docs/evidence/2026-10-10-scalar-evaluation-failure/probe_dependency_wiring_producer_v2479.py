"""Consume only the reusable builder's bound import arguments in the full scalar preview."""
import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
PROPOSAL = E / "dependency-wiring-proposal-v2476"
BUILD = E / "dependency-wiring-cargo-v2478"
PREVIEW = E / "approximation-bound-candidate-v2460"
OUT = E / "dependency-wiring-producer-v2479"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    assert not OUT.exists()
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    build, build_protocol = read(BUILD / "result.json"), read(BUILD / "protocol.json")
    assert build["gate_passed"] and build["compiler_result"]["verified"] == 453
    assert build["compiler_result"]["warnings"] == 0
    dependencies = read(BUILD / "dependency-reports/dependencies.json")
    assert dependencies["verus_arguments"] == build["verus_arguments"]
    assert build["dependency_report_sha256"] == sha(BUILD / "dependency-reports/dependencies.json")
    assert all(sha(PROPOSAL / n) == h for n, h in build_protocol["proposal_files_sha256"].items())
    preview = read(PREVIEW / "protocol.json")
    assert all(sha(PREVIEW / n) == h for n, h in preview["preview_files_sha256"].items())
    base = read(E / "approximation-bound-proof-v2461/protocol.json")["command"]
    prefix = base[:base.index("--import")]
    command = prefix + dependencies["verus_arguments"] + [str(PREVIEW / "proof.rs")]
    assert command.count("--import") == 1 and "--no-cheating" in command
    OUT.mkdir()
    protocol = {"status": "Frozen before checking the full preview with the reusable builder's exact bound arguments",
        "runner_sha256": sha(Path(__file__)), "command": command,
        "retained_checkpoint_sha256": sha(checkpoint_path), "source_record_sha256": checkpoint["source_record_sha256"],
        "build_result_sha256": sha(BUILD / "result.json"), "build_protocol_sha256": sha(BUILD / "protocol.json"),
        "dependency_report_sha256": sha(BUILD / "dependency-reports/dependencies.json"),
        "preview_protocol_sha256": sha(PREVIEW / "protocol.json"), "preview_files_sha256": preview["preview_files_sha256"],
        "proposal_files_sha256": build_protocol["proposal_files_sha256"], "environment": build_protocol["environment"],
        "scope": "The reusable dependency builder supplies both the Cargo num library and its bound num-bigint VIR import. The complete existing scalar preview and exact shared producer consume those arguments without separate dependency reconstruction. Run the proposed identity checker before and after the proof. This remains a conditional static producer proof under normalization/range and approximation-truth premises, not SOURCE integration, whole-node coverage or runtime admission."}
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    sys.path.insert(0, str(PROPOSAL))
    spec = importlib.util.spec_from_file_location("wiring_dependencies_v2479", PROPOSAL / "verus_dependencies.py")
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    saved = os.environ.copy()
    os.environ.update(build_protocol["environment"])
    start = time.monotonic()
    try:
        helper.check_dependency_identity(dependencies)
        run = subprocess.run(command, cwd=ROOT, env=os.environ, capture_output=True, text=True)
        (OUT / "stdout.json").write_text(run.stdout)
        (OUT / "stderr.log").write_text(run.stderr)
        helper.check_dependency_identity(dependencies)
    finally:
        os.environ.clear()
        os.environ.update(saved)
    assert all(sha(PROPOSAL / n) == h for n, h in build_protocol["proposal_files_sha256"].items())
    assert all(sha(PREVIEW / n) == h for n, h in preview["preview_files_sha256"].items())
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    try:
        verified = json.loads(run.stdout).get("verification-results", {})
    except json.JSONDecodeError:
        verified = {}
    result = {"exit_code": run.returncode, "seconds": time.monotonic() - start, "verification_results": verified,
        "gate_passed": run.returncode == 0 and not run.stderr and verified.get("verified") == 737 and verified.get("errors") == 0,
        "protocol_sha256": sha(OUT / "protocol.json"), "stdout_sha256": sha(OUT / "stdout.json"),
        "stderr_sha256": sha(OUT / "stderr.log"), "source_unchanged": True, "main_unchanged": True,
        "proposal_unchanged": True, "preview_unchanged": True, "builder_identity_checks_passed": True,
        "retained_coverage_changed": False, "runtime_admitted": False}
    (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
