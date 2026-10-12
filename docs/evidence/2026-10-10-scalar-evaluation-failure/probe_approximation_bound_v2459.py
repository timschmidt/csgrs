"""Verify the proposed shared bound producer with actual imported BigInt metadata."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
PREVIEW = E / "approximation-bound-candidate-v2458"
OUT = E / "approximation-bound-proof-v2459"
CARGO = E / "bigint-cargo-export-v2454"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    assert not OUT.exists()
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    main_guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    preview = read(PREVIEW / "protocol.json")
    assert all(sha(PREVIEW / n) == h for n, h in preview["preview_files_sha256"].items())
    cargo, cargo_protocol = read(CARGO / "result.json"), read(CARGO / "protocol.json")
    dependencies = read(CARGO / "dependency-reports/dependencies.json")
    assert cargo["gate_passed"] and cargo["compiler_result"]["warnings"] == 0
    assert cargo["compiler_result"]["verification_results"]["verified"] == 453
    fixture = Path(cargo_protocol["work"])
    assert all(sha(fixture / n) == h for n, h in cargo_protocol["work_files_sha256"].items())
    assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
    metadata = fixture / "artifacts/num_bigint.vir"
    assert sha(metadata) == cargo["export_sha256"]
    tool = Path(read(CARGO / "wrapper-config.json")["verus"]).parent
    assert all(sha(tool / n) == h for n, h in cargo_protocol["tool_sha256"].items())
    OUT.mkdir()
    command = [str(tool / "verus"), "--edition=2024", "--crate-type=lib", "--crate-name", "approximation_bound_preview",
        "--no-cheating", "--output-json", "-D", "warnings", "--check-cfg", "cfg(verus_keep_ghost)",
        "--check-cfg", "cfg(verus_keep_ghost_body)", "--import", "num_bigint=" + str(metadata),
        *dependencies["verus_arguments"], str(PREVIEW / "proof.rs")]
    protocol = {"status": "Frozen before the complete scalar proof target plus proposed producer",
        "runner_sha256": sha(Path(__file__)), "command": command,
        "retained_checkpoint_sha256": sha(checkpoint_path), "source_record_sha256": checkpoint["source_record_sha256"],
        "preview": PREVIEW.name, "preview_protocol_sha256": sha(PREVIEW / "protocol.json"),
        "preview_files_sha256": preview["preview_files_sha256"], "cargo_export_result_sha256": sha(CARGO / "result.json"),
        "dependency_report_sha256": sha(CARGO / "dependency-reports/dependencies.json"),
        "dependency_artifacts_sha256": dependencies["artifacts"], "exported_metadata_sha256": sha(metadata),
        "tool_sha256": cargo_protocol["tool_sha256"],
        "scope": "Verify the same proposed shared associated-function file prepared for the node facade, with the current scalar proof bodies and the Cargo-built actual BigInt/BigUint export. The static owner's graph fields are unused and not imported. Exact metadata output, early Unknown and conditional real-denotation certificates are attempted under explicit representation/range premises. SOURCE and main remain unchanged; full node/approximation existence, premise discharge, runtime qualification and retained coverage remain separate."}
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    start = time.monotonic()
    run = subprocess.run(command, cwd=ROOT, env=os.environ, capture_output=True, text=True)
    (OUT / "stdout.json").write_text(run.stdout)
    (OUT / "stderr.log").write_text(run.stderr)
    assert all(sha(PREVIEW / n) == h for n, h in preview["preview_files_sha256"].items())
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    assert all(sha(fixture / n) == h for n, h in cargo_protocol["work_files_sha256"].items())
    assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
    assert sha(metadata) == cargo["export_sha256"]
    assert all(sha(tool / n) == h for n, h in cargo_protocol["tool_sha256"].items())
    try:
        parsed = json.loads(run.stdout).get("verification-results", {})
    except json.JSONDecodeError:
        parsed = {}
    result = {"exit_code": run.returncode, "seconds": time.monotonic() - start, "verification_results": parsed,
        "protocol_sha256": sha(OUT / "protocol.json"), "stdout_sha256": sha(OUT / "stdout.json"),
        "stderr_sha256": sha(OUT / "stderr.log"), "source_unchanged": True, "main_unchanged": True,
        "preview_unchanged": True, "dependency_inputs_unchanged": True, "tooling_unchanged": True,
        "retained_coverage_changed": False, "runtime_admitted": False}
    (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
