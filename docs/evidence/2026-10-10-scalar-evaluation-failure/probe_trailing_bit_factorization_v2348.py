"""Probe word-plus-one factorization without changing the candidate."""
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
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))

PROBE = (E / "trailing-bit-factorization-proposal-v2348.rs").read_text()


def main():
    out = E / "trailing-bit-factorization-v2348"
    assert not out.exists()
    checkpoint_path = E / "typed-api-prototype-v2345-state.json"
    assert sha(checkpoint_path) == "a75220d56a289d2e83e36a426fb6f1f04b808e684b0889e5d360924408d940b9"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    pin = read(SOURCE / "crates/hyperreal/verification/toolchain.json")
    tool = ROOT / "crates/hyperreal/target/verus" / pin["version"] / "verus-x86-linux"
    tooling = read(E / "typed-api-prototype-v2343-approximation-metadata-final-verus/proof-reports/tooling.json")
    assert all(sha(tool / n) == h for n, h in tooling["tool_sha256"].items())
    out.mkdir()
    (out / "probe.rs").write_text(PROBE)
    command = [str(tool / "verus"), "--edition=2024", "--crate-type=lib", "--no-cheating",
               "--output-json", str(out / "probe.rs")]
    protocol = {"status": "Frozen before primitive support probing", "command": command,
        "runner_sha256": sha(Path(__file__)), "probe_sha256": sha(out / "probe.rs"),
        "retained_checkpoint": checkpoint_path.name, "retained_checkpoint_sha256": sha(checkpoint_path),
        "source_record_sha256": checkpoint["source_record_sha256"], "pinned_toolchain": pin,
        "verifier_sha256": sha(tool / "verus"),
        "question": "Do the pinned word bit-count contracts prove that a word plus one factors into the reported power of two and a positive odd quotient, including full all-ones words, for u32 and u64?",
        "scope": "Use only the pinned vstd standard-library axioms already supplied by Verus. Introduce no external method specification, assumption or trusted arithmetic axiom. This support probe is not a production BigInt proof, a SOURCE change, a new retained theorem or runtime admission. Preserve a failure without changing completed artifacts."}
    (out / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    start = time.monotonic()
    result = subprocess.run(command, cwd=ROOT, env=os.environ | {"PYTHONDONTWRITEBYTECODE": "1"},
                            capture_output=True, text=True)
    (out / "stdout.json").write_text(result.stdout)
    (out / "stderr.log").write_text(result.stderr)
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    assert all(sha(tool / n) == h for n, h in tooling["tool_sha256"].items())
    data = {"exit_code": result.returncode, "seconds": time.monotonic() - start,
        "protocol_sha256": sha(out / "protocol.json"), "stdout_sha256": sha(out / "stdout.json"),
        "stderr_sha256": sha(out / "stderr.log"), "source_unchanged": True, "main_unchanged": True,
        "tooling_unchanged": True, "retained_proof_coverage_changed": False,
        "next_step": "Inspect the exact result before using any primitive contract in a metadata model. BigUint storage, normalization, aggregate scans, length ranges and production caller refinement remain separate."}
    (out / "result.json").write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps(data))


if __name__ == "__main__":
    main()
