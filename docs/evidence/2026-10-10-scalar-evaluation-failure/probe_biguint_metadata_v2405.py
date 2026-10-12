"""Verify the exact shared source files in the frozen BigUint dependency preview."""
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
PREVIEW = E / "biguint-metadata-candidate-v2404"
OUT = E / "biguint-metadata-proof-v2405"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    assert not OUT.exists()
    checkpoint_path = E / "typed-api-prototype-v2386-state.json"
    assert sha(checkpoint_path) == "6dabcb25b03a7a8769c432b926b3c4a35288ed43fd172e7a46d27503d8f68cb9"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    preview_protocol = read(PREVIEW / "protocol.json")
    assert all(sha(PREVIEW / n) == h for n, h in preview_protocol["preview_files_sha256"].items())
    assert sha(PREVIEW / "upstream.patch") == preview_protocol["upstream_patch_sha256"]
    inputs = {str(p.relative_to(PREVIEW)): sha(p) for p in sorted(PREVIEW.rglob("*")) if p.is_file()}
    pin = read(SOURCE / "crates/hyperreal/verification/toolchain.json")
    tool = ROOT / "crates/hyperreal/target/verus" / pin["version"] / "verus-x86-linux"
    tooling = read(E / "typed-api-prototype-v2380-word-scan-verus/proof-reports/tooling.json")
    assert all(sha(tool / n) == h for n, h in tooling["tool_sha256"].items())
    OUT.mkdir()
    command = [str(tool / "verus"), "--edition=2024", "--crate-type=lib", "--no-cheating",
               "--output-json", str(PREVIEW / "proof.rs")]
    protocol = {
        "status": "Frozen before verifying the shared dependency preview files",
        "command": command, "runner_sha256": sha(Path(__file__)),
        "retained_checkpoint": checkpoint_path.name, "retained_checkpoint_sha256": sha(checkpoint_path),
        "source_record_sha256": checkpoint["source_record_sha256"], "preview": PREVIEW.name,
        "preview_files_sha256": inputs, "pinned_toolchain": pin, "tool_sha256": tooling["tool_sha256"],
        "scope": "Compile the exact shared BigUint representation, public metadata methods and six helpers for each native width alongside the retained arithmetic models. Eighteen new executable function bodies are attempted; imported proof units are not new coverage. Normalization/range caller premises, allocator policy and full dependency/Cargo/runtime admission remain open. No SOURCE/main mutation, local trust escape hatch or retained coverage increase is authorized by this support run.",
    }
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    start = time.monotonic()
    result = subprocess.run(command, cwd=ROOT, env=os.environ | {"PYTHONDONTWRITEBYTECODE": "1"},
                            capture_output=True, text=True)
    (OUT / "stdout.json").write_text(result.stdout)
    (OUT / "stderr.log").write_text(result.stderr)
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    assert {str(p.relative_to(PREVIEW)): sha(p) for p in sorted(PREVIEW.rglob("*")) if p.is_file()} == inputs
    assert all(sha(tool / n) == h for n, h in tooling["tool_sha256"].items())
    parsed = json.loads(result.stdout)
    data = {
        "exit_code": result.returncode, "seconds": time.monotonic() - start,
        "verification_results": parsed.get("verification-results"),
        "protocol_sha256": sha(OUT / "protocol.json"), "stdout_sha256": sha(OUT / "stdout.json"),
        "stderr_sha256": sha(OUT / "stderr.log"), "source_unchanged": True, "main_unchanged": True,
        "preview_unchanged": True, "tooling_unchanged": True, "retained_proof_coverage_changed": False,
        "new_executable_functions_attempted": 18, "runtime_admitted": False,
    }
    (OUT / "result.json").write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps(data))


if __name__ == "__main__":
    main()
