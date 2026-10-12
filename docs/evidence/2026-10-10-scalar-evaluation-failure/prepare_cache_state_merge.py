"""Freeze the current diagnostic source before extracting the full cache-state match."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / "cache-state-merge-baseline-v1918"
    assert not out.exists()
    review_path = E / "bound-merge-static-review-v1917.json"
    review = read(review_path)
    assert not review["runtime_retained"] and review["timed_processes_for_variant"] == 0
    record = E / review["current_source_record"]
    assert sha(record) == review["current_source_record_sha256"]
    current = json.loads(gzip.decompress(record.read_bytes()))
    for name, digest in current.items():
        assert sha(SOURCE / name) == digest, name
    retained_path = E / review["retained_runtime_checkpoint"]
    assert sha(retained_path) == review["retained_runtime_checkpoint_sha256"]
    retained = read(retained_path)
    original = json.loads(gzip.decompress((E / retained["source_record"]).read_bytes()))
    changed = {name: digest for name, digest in current.items() if original[name] != digest}
    assert len(changed) == 6 and len(current) == 24746
    guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    for name, digest in guard.items():
        assert sha(ROOT / name) == digest, name
    out.mkdir()
    (out / "prepare.py").write_bytes(Path(__file__).read_bytes())
    for name, digest in changed.items():
        target = out / "inputs" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes((SOURCE / name).read_bytes())
        assert sha(target) == digest
    inputs = ["bound-merge-proof-validation-v1840.json", "bound-merge-broad-validation-v1849.json",
        "bound-merge-cost-rejection-v1857.json", "bound-merge-cost-review-v1857.json",
        "bound-merge-cost-followup-strategy-v1857.json", "bound-merge-cost-protocol-v1850.json",
        "bound-merge-replay-protocol-v1858.json", "bound-merge-static-review-v1917.json",
        "bound-merge-static-diagnosis-v1914/summary.json", "flat-merge-generated-code-v1916/summary.json",
        "merge-expression-support-v1917/result.json", "merge-expression-support-v1917/stderr.log"]
    data = {
        "status": "Frozen before source changes for the direct cache-state merge proposal",
        "base_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "retained_runtime_checkpoint": retained_path.name, "retained_runtime_checkpoint_sha256": sha(retained_path),
        "previous_diagnostic_source_record": review["current_source_record"],
        "previous_diagnostic_source_record_sha256": sha(record),
        "source_files_verified": len(current), "main_guarded_files_verified": len(guard),
        "archived_sources_sha256": changed, "inputs_sha256": {name: sha(E / name) for name in inputs},
        "proposal": "Move the included pure merge method to BoundCache and match (retained cache state, incoming bound) directly. Invalid or Unknown retains the incoming value; other cases preserve the existing total selection. AtomicFacts::publish_bound calls that method after the unchanged diagnostics. Preserve the initial load/test hook and encode/mask/CAS suffix exactly.",
        "proof_boundary": "Connect the existing denotation, compatibility, certificate-retention and domain-retention lemmas to the actual BoundCache::merge contract for Valid inputs. The executable contract also specifies Invalid inputs exactly. Adapt existing production mutations to their new syntax and add an independent invalid-cache mutation. Do not add assume_specification, axioms, external bodies or other bypasses for Option::or. Producer validity, real existence, diagnostics, atomics and callers remain outside this conditional proof.",
        "initial_gate": "Format and release-build the new source; compile the same unchanged atan probe without timing it and inspect publication code against retained v1836 and both prior variants. If useful, refresh the full no-cheating proof and acceptance/rejection suite before runtime qualification.",
        "retention_gates": "Preserve all nine runtime commands, seven cost families with all 52 original full-size fixtures, 468 standalone processes, 936 alternating paired processes, original process caps and the frozen family-wide 27-pair repeat rule. The cost baseline stays v1836. Then preserve all 39 capped phases, 4129 numerical cases, 648 public schedules, deterministic receipt comparisons and observed-fact audits. No prior failed comparison, fixed baseline, threshold, fixture or process cap is reset.",
        "main_runtime_admitted": False,
        "scope": "An isolated proposal within the unchanged 39 properties, 14 targets and crate future work; all thirteen whole-crate proof groups remain open.",
    }
    (out / "baseline.json").write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps({"baseline": out.name, "archived_sources": len(changed),
        "source_files_verified": len(current), "retained_checkpoint": retained_path.name}))


if __name__ == "__main__":
    main()
