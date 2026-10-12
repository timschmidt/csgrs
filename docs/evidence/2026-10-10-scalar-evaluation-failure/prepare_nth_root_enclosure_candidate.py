"""Freeze the failed selector and source before changing adjacent root bounds."""
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
    out = E / "nth-root-enclosure-baseline-v2001"
    assert not out.exists()
    review_path = E / "cache-state-algebraic-stage-review-v2000.json"
    review = read(review_path)
    rejection = read(E / review["rejection_record"])
    assert not review["runtime_retained"] and review["original_cost_failure_still_failed"]
    assert not rejection["cost_gate_passed"] and not rejection["runtime_retained"]
    record = E / review["source_record"]
    assert sha(record) == review["source_record_sha256"]
    current = json.loads(gzip.decompress(record.read_bytes()))
    for name, digest in current.items():
        assert sha(SOURCE / name) == digest
    retained_path = E / rejection["retained_runtime_checkpoint"]
    assert sha(retained_path) == rejection["retained_runtime_checkpoint_sha256"]
    retained = read(retained_path)
    original = json.loads(gzip.decompress((E / retained["source_record"]).read_bytes()))
    changed = {name: digest for name, digest in current.items() if original[name] != digest}
    assert len(current) == 24746 and len(changed) == 6
    guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    for name, digest in guard.items():
        assert sha(ROOT / name) == digest
    expected = read(E.parent / "2026-10-10-artifact-numeric-keys/retention.json")["staged_verification"]["preexisting_status"]
    status = subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=all"], cwd=ROOT, text=True).splitlines()
    assert [line for line in status if not line[3:].startswith(str(E.relative_to(ROOT)) + "/")] == expected
    assert len(expected) == 49
    planned = ["crates/hyperreal/src/computable/approximation/exp_sqrt.rs",
               "crates/hyperreal/src/computable/approximation/precision_tests.rs"]
    archived = changed | {name: current[name] for name in planned}
    assert len(archived) == 8
    proof = read(E / "cache-state-merge-proof-validation-v1926.json")
    assert set(planned).isdisjoint(proof["proof_and_guard_inputs_sha256"])
    for name, digest in proof["proof_and_guard_inputs_sha256"].items():
        assert sha(SOURCE / name) == digest
    out.mkdir()
    (out / "prepare.py").write_bytes(Path(__file__).read_bytes())
    for name, digest in archived.items():
        path = out / "inputs" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes((SOURCE / name).read_bytes())
        assert sha(path) == digest
    inputs = ["cache-state-merge-proof-validation-v1926.json",
        "cache-state-merge-broad-validation-v1935.json",
        "cache-state-merge-cost-protocol-v1936.json",
        "cache-state-merge-cost-followup-strategy-v1943.json",
        "cache-state-merge-partial-cost-review-v1943.json",
        "cache-state-merge-cost-rejection-v1943.json",
        "cache-state-algebraic-stage-protocol-v2000.json",
        "cache-state-algebraic-stage-diagnostic-v2000/summary.json",
        "cache-state-algebraic-stage-review-v2000.json",
        "cache-state-merge-replay-protocol-v1944.json"]
    data = {
        "status": "Frozen before changing the direct nth-root enclosure",
        "base_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "current_source_record": str(record.relative_to(E)), "current_source_record_sha256": sha(record),
        "retained_runtime_checkpoint": retained_path.name, "retained_runtime_checkpoint_sha256": sha(retained_path),
        "source_files_verified": len(current), "main_guarded_files_verified": len(guard),
        "original_worktree_entries_preserved": 49,
        "archived_sources_sha256": archived, "planned_changed_files": planned,
        "inputs_sha256": {name: sha(E / name) for name in inputs},
        "proof_and_guard_inputs_sha256": proof["proof_and_guard_inputs_sha256"],
        "proposal": review["proposal_not_implemented"],
        "focused_checks": [
            "Check exact floor/ceiling enclosure inequalities for every small input A from 2 through 1024 and every supported direct degree, 3 through 9.",
            "Compare both integer endpoints with the old two-root construction at wide perfect powers and their neighbors, including the original failing fixture's operand widths.",
            "Run the existing public MPFR nth-root oracle cases unchanged; preserve all domain, precision and cancellation branches.",
        ],
        "proof_scope": "The two planned runtime/test files are outside the 44 frozen Verus and guard inputs. Any reused proof evidence must check all of those bytes and retain its original conditional scope; it does not formally verify the new bigint kernel. All thirteen whole-crate proof groups remain open.",
        "retention_gates": "The combined isolated proposal still compares with retained v1836, not the rejected selector. Run the unchanged nine broad commands (including the two new tests), seven cost families and all 52 full-size fixtures under their original caps and frozen family-wide repeat rule. Preserve the v1943 failure and every older failure/tradeoff. Only after those pass, run the 39 capped phases, 4129 numerical cases, 648 public schedules and receipt/fact audits. No fixture, threshold, cap, fixed baseline or main admission requirement is relaxed.",
        "runtime_retained": False, "main_runtime_admitted": False,
        "scope": "All 39 properties, 14 targets and crate future work remain in scope.",
    }
    (out / "baseline.json").write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps({"baseline": out.name, "archived_sources": len(archived),
                      "planned_changed_files": planned, "source_files_verified": len(current)}))


if __name__ == "__main__":
    main()
