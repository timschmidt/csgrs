"""Record unsuccessful static follow-ups without changing the retained baseline."""
import copy
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / "bound-merge-static-review-v1917.json"
    audit = E / "bound-merge-static-ledger-v1917.json"
    assert not out.exists() and not audit.exists()
    phase = E / "typed-api-prototype-v1916-flat-merge-codegen"
    result = read(phase / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    diagnostic = E / "flat-merge-generated-code-v1916/summary.json"
    generated = read(diagnostic)
    source_record = phase / "source-hashes.json.gz"
    assert generated["source_record_sha256"] == sha(source_record)
    assert generated["single_selection_expression_change_verified"] and generated["timed_processes"] == 0
    assert all(row["exit_code"] == 0 for row in read(diagnostic.parent / "commands.json"))
    assert all(sha(diagnostic.parent / name) == digest for name, digest in generated["artifacts_sha256"].items())
    assert generated["functions"]["publish"]["bytes"] == 860
    assert generated["functions"]["publish"]["retained_bytes"] == 517
    assert generated["functions"]["publish"]["rejected_bytes"] == 860
    support_path = E / "merge-expression-support-v1917/result.json"
    support = read(support_path)
    assert support["exit_code"] == 1 and support["verifier_unchanged"]
    assert all(sha(support_path.parent / name) == digest for name, digest in support["artifacts_sha256"].items())
    assert "`core::option::impl&%0::or` is not supported" in (support_path.parent / "stderr.log").read_text()
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    assert len(sources) == 24746
    for name, digest in sources.items():
        assert sha(ROOT / "target/scalar-evaluation-candidate/source" / name) == digest, name
    rejection = read(E / "bound-merge-cost-rejection-v1857.json")
    next_priority = (
        "Prototype a pure helper that matches the original BoundCache/incoming-bound tuple "
        "directly, preserving Invalid and Unknown handling within that proved body. Keep "
        "supported executable selections and prove their contracts without adding an assumed "
        "Option::or specification. Freeze and inspect generated code before the full unchanged "
        "proof, runtime, original-cap cost and replay gates. The v1857 rejection remains decisive "
        "for its source, and v1836 remains the retained runtime."
    )
    record = {
        "status": "Set aside the caller-only variant: its intended publication-code reduction did not occur",
        "current_source_record": str(source_record.relative_to(E)), "current_source_record_sha256": sha(source_record),
        "source_files_verified": len(sources),
        "retained_runtime_checkpoint": rejection["retained_runtime_checkpoint"],
        "retained_runtime_checkpoint_sha256": rejection["retained_runtime_checkpoint_sha256"],
        "static_diagnosis": "bound-merge-static-diagnosis-v1914/summary.json",
        "static_diagnosis_sha256": sha(E / "bound-merge-static-diagnosis-v1914/summary.json"),
        "variant_baseline": "flat-merge-baseline-v1915/baseline.json",
        "variant_baseline_sha256": sha(E / "flat-merge-baseline-v1915/baseline.json"),
        "variant_codegen": str(diagnostic.relative_to(E)), "variant_codegen_sha256": sha(diagnostic),
        "expression_support_probe": str(support_path.relative_to(E)), "expression_support_probe_sha256": sha(support_path),
        "observations": [
            "The frozen source changes exactly one combined-bound selection expression from the rejected proposal. Workspace/included-file formatting, the release build and unchanged probe compilation pass; the 860-byte publication body does not shrink toward the 517-byte retained body.",
            "No timing samples were taken for this caller-only variant, so its performance is unknown. It is set aside on the stated static hypothesis, not rejected by a fabricated timing result. The 2484-process failed v1857 cost comparison applies only to its original source.",
            "The standalone generic Option::or plus i32::min proof probe stops at unsupported Option::or. It establishes no result for i32::min or production merging. No assumption/specification was added to bypass the failure.",
            "The current candidate directory holds the unretained caller-only source for a reproducible handoff. The next variant must freeze its own source and validation identities. Prepared v1913 retention and earlier replay protocols remain unexecuted; all thirteen whole-crate groups and main/native/full-board admission remain open.",
        ],
        "timed_processes_for_variant": 0, "runtime_retained": False, "main_runtime_admitted": False,
        "next_priority": next_priority,
    }
    ledger = ROOT / "docs/properties-status.json"
    before_bytes = ledger.read_bytes()
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == before_bytes
    before = json.loads(before_bytes)
    after = copy.deepcopy(before)
    assert len(after["properties"]) == 39 and len(after["targets"]) == 14
    p4 = next(row for row in after["properties"] if row["id"] == "P4")
    path = str(out.relative_to(ROOT))
    assert path not in p4["evidence"]
    p4["evidence"].append(path)
    update = (
        "The v1914-v1917 static follow-up preserves the failed cost experiment. A caller-only "
        "branching variant passes formatting/release/probe compilation but leaves publication "
        "at 860 bytes versus the retained 517 bytes, so it is set aside without timing. A "
        "separate original-expression proof probe fails because Option::or is unsupported; "
        "no assumption was added and i32::min was not independently verified. A direct "
        "cache-state helper is the next hypothesis. The retained runtime remains v1836; "
        "all property/target statuses, fixed baselines and admission obligations are unchanged."
    )
    p4["remaining"] += " " + update
    old = rejection["next_priority"]
    assert after["next_actions"][0].count(old) == 1
    after["next_actions"][0] = after["next_actions"][0].replace(old, next_priority, 1)
    after["next_actions"][0] += " " + update
    unchanged = copy.deepcopy(after)
    old_p4 = next(row for row in before["properties"] if row["id"] == "P4")
    same_p4 = next(row for row in unchanged["properties"] if row["id"] == "P4")
    same_p4["evidence"], same_p4["remaining"] = old_p4["evidence"], old_p4["remaining"]
    unchanged["next_actions"][0] = before["next_actions"][0]
    assert unchanged == before
    out.write_text(json.dumps(record, indent=2) + "\n")
    ledger.write_text(json.dumps(after, indent=2) + "\n")
    audit.write_text(json.dumps({"before_sha256": hashlib.sha256(before_bytes).hexdigest(),
        "after_sha256": sha(ledger), "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
        "all_39_property_and_14_target_requirements_and_statuses_preserved": True,
        "static_review_sha256": sha(out), "runtime_retained": False}, indent=2) + "\n")
    print(json.dumps({"status": record["status"], "source_record_sha256": sha(source_record),
        "runtime_retained": False, "variant_timing_processes": 0}))


if __name__ == "__main__":
    main()
