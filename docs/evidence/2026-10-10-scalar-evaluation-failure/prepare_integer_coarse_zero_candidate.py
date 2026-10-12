"""Freeze the failed combined source before a charged integer coarse-zero shortcut."""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / "integer-coarse-zero-baseline-v2079"
    assert not out.exists()
    failure_path = E / "nth-root-enclosure-cost-rejection-v2021.json"
    failure = read(failure_path)
    assert not failure["runtime_retained"] and not failure["cost_gate_passed"]
    assert failure["successful_cost_processes"] == 2160 and failure["separate_cpu_preludes"] == 1692
    record = E / failure["source_record"]
    assert sha(record) == failure["source_record_sha256"]
    sources = json.loads(gzip.decompress(record.read_bytes()))
    assert len(sources) == 24746 and all(sha(SOURCE / name) == digest for name, digest in sources.items())
    guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert len(guard) == 1167 and all(sha(ROOT / name) == digest for name, digest in guard.items())
    static_path = E / "nth-root-enclosure-integer-layout-v2078/summary.json"
    static = read(static_path)
    assert static["source_record_sha256"] == failure["source_record_sha256"] and static["timed_processes"] == 0
    assert all(sha(static_path.parent / name) == digest for name, digest in static["artifacts_sha256"].items())
    assert all(row["normalized_instruction_shapes_equal"] for row in static["comparisons"].values())
    proof_path = E / "nth-root-enclosure-proof-validation-v2004.json"
    proof = read(proof_path)
    name = "crates/hyperreal/src/computable/node/approximation_queries.rs"
    assert name not in proof["proof_and_guard_inputs_sha256"]
    expected = read(E.parent / "2026-10-10-artifact-numeric-keys/retention.json")["staged_verification"]["preexisting_status"]
    status = subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True).splitlines()
    assert [line for line in status if not line[3:].startswith(str(E.relative_to(ROOT)) + "/")] == expected
    out.mkdir()
    shutil.copyfile(__file__, out / "prepare.py")
    archived = {**proof["changed_sources_sha256"], name: sources[name]}
    for path, digest in archived.items():
        target = out / "inputs" / path
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(SOURCE / path, target)
        assert sha(target) == digest
    data = {
        "status": "Frozen before adding the exact-integer coarse-zero branch",
        "base_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "current_source_record": str(record.relative_to(E)), "current_source_record_sha256": sha(record),
        "retained_runtime_checkpoint": failure["retained_runtime_checkpoint"],
        "retained_runtime_checkpoint_sha256": failure["retained_runtime_checkpoint_sha256"],
        "source_files_verified": len(sources), "main_guarded_files_verified": len(guard),
        "original_worktree_entries_preserved": len(expected),
        "archived_sources_sha256": archived, "planned_changed_files": [name],
        "prior_rejection": failure_path.name, "prior_rejection_sha256": sha(failure_path),
        "static_review": str(static_path.relative_to(E)), "static_review_sha256": sha(static_path),
        "prior_proof_record": proof_path.name, "prior_proof_record_sha256": sha(proof_path),
        "proof_and_guard_inputs_sha256": proof["proof_and_guard_inputs_sha256"],
        "proposal": "After the existing cancellation/exhaustion checks, exact-zero shortcut, precision validation and successful one-unit work charge in Approximation::Int, return zero when p > 0 and u64(p) > value.bits(). This is the identical predicate and result used by the called scaling helper; avoid entering that helper for this case. All remaining integer requests retain the original scaling path.",
        "rationale": "The preserved integer probe, evaluator and scaling helper have equal normalized instruction shapes relative to retained v1836; this does not explain timing causality. Static inspection separately identifies a real removable call: a 1238-byte scaling helper saves six registers and 88 stack bytes before its coarse-zero result. Improve that source path, rather than retrying the unchanged failed proposal.",
        "focused_checks": [
            "Run all five existing integer_scale tests unchanged: signed limb boundaries and halfway rounding, public/raw allocation controls through 1000001-bit inputs, allowance exhaustion/retries/retained signs, precision limits and cancellation precedence.",
            "Retain all three direct_nth_root tests unchanged and verify that only the planned production file changes.",
            "Refresh the unchanged conditional Verus target and acceptance checks. Any reused negative guards must retain exact proof/guard, dependency and verifier identities; no proof of this new integer branch is implied.",
        ],
        "retention_gates": "The candidate remains the combined kernel/outlined-selector proposal and compares against retained v1836. Require unchanged broad commands, all 52 full-size cost fixtures under original caps and the same predeclared family-repeat rule, then all 39 caps, 4129 numerical cases and 648 public schedules and their audits. Preserve the v2021 failed comparison and every earlier failure; no favorable retry, reduced fixture or relaxed threshold replaces them.",
        "allocation_scope": "Existing integer allocation controls measure the public and raw paths. The prior 84-case copied-root study remains evidence only for its unchanged extracted helper and num dependency, not a new measurement of the proposed public path.",
        "runtime_retained": False, "main_runtime_admitted": False,
        "scope": "All thirteen whole-crate proof groups, the fixed pre-Verus baseline, main/native/full-board admission and all 39 properties/14 targets/crate future work remain open.",
    }
    (out / "baseline.json").write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps({"baseline_frozen": True, "archived_files": len(archived), "source_files_verified": len(sources)}))


if __name__ == "__main__":
    main()
