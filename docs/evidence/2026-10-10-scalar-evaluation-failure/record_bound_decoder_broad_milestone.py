"""Bind completed runtime checks to the proved decoder without claiming retention."""
import copy
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    output = E / "bound-decoder-broad-milestone-v2180.json"
    ledger_output = E / "bound-decoder-broad-ledger-v2180.json"
    assert not output.exists() and not ledger_output.exists()
    proof_path = E / "bound-decoder-proof-validation-v2171.json"
    proof = read(proof_path)
    broad_path = E / "bound-decoder-broad-validation-v2180.json"
    broad = read(broad_path)
    validation_path = E / "bound-decoder-candidate-validation-v2180.json"
    validation = read(validation_path)
    expected = proof["source_record_sha256"]
    assert broad["source_record_sha256"] == validation["source_record_sha256"] == expected
    assert validation["complete"] and len(validation["phases"]) == 9
    for name, row in validation["phases"].items():
        assert read(E / name / "result.json") == row["result"]
        assert row["result"]["exit_code"] == 0 and row["result"]["source_unchanged"] and row["result"]["main_unchanged"]
        assert sha(E / name / "source-hashes.json.gz") == expected
    assert broad["strict_clippy_warning_lines"] == 0 and len(broad["inherited_stack_warning_lines"]) == 14
    assert broad["ignored_test_names_unchanged"] and broad["included_rustfmt_files"] == 108
    assert proof["verified_units"] == 655 and proof["fresh_rejected_mutations"] == 245
    assert proof["fresh_passing_guard_baselines"] == 4 and proof["fresh_acceptance_tests"] == 10
    source_record = E / "typed-api-prototype-v2174-bound-decoder-release-build/source-hashes.json.gz"
    assert sha(source_record) == expected
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    main = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert len(sources) == 24746 and all(sha(SOURCE / n) == h for n, h in sources.items())
    assert len(main) == 1167 and all(sha(ROOT / n) == h for n, h in main.items())
    comparison_path = E / "bound-decoder-binary-comparison-protocol-v2181.json"
    assert read(comparison_path)["source_record_sha256"] == expected
    record = {"status": "All nine runtime commands pass on the proved decoder source; binary, cost and replay qualification pending",
        "source_record": str(source_record.relative_to(E)), "source_record_sha256": expected,
        "source_files_verified": len(sources), "main_files_verified": len(main),
        "broad_validation": broad_path.name, "broad_validation_sha256": sha(broad_path),
        "validation_controller": validation_path.name, "validation_controller_sha256": sha(validation_path),
        "proof_record": proof_path.name, "proof_record_sha256": sha(proof_path),
        "retained_runtime_checkpoint": proof["retained_runtime_checkpoint"],
        "retained_runtime_checkpoint_sha256": proof["retained_runtime_checkpoint_sha256"],
        "next_binary_comparison_protocol": comparison_path.name,
        "next_binary_comparison_protocol_sha256": sha(comparison_path),
        "runtime_retained": False, "main_runtime_admitted": False,
        "limits": [
            "The five-crate test suite covers hyperreal, hyperlimit, hyperlattice, hypercurve and hyperdrc. All 33 ignored cases and 14 inherited Cargo example-filename warning lines match the retained baseline. This is not whole-repository CI, a full-board check or main admission.",
            "The fresh conditional proof reports 655 units, 98 production contracts and 135 model theorems; all 245 negative mutations, four positive baselines and ten acceptance tests pass afresh. The added contract covers the actual all-word decoder and five connected field lemmas. Producer truth, existence, atomic/caller ordering and all thirteen whole-crate groups remain open.",
            "The seven retained baseline binaries are frozen and byte-identical to their historical versions. The prepared candidate comparison has not yet run. Reuse fixture cost evidence only for exact binary identity; otherwise run fresh original-cap comparisons. Existing failures and the fixed pre-Verus baseline remain unchanged.",
            "The remaining capped/numerical/public replay and fact audits remain separate gates. No runtime retention or property/target completion follows from this milestone alone.",
        ]}
    next_priority = (
        "All nine decoder runtime commands now pass. Execute the frozen seven-probe binary "
        "comparison, bind any reused cost evidence to exact executable/workload identity, "
        "and run original-cap cost controls for changed binaries. Complete the remaining "
        "capped/numerical/public replay and fact audits before retaining the decoder. Preserve "
        "all earlier failures, the fixed pre-Verus baseline and open producer/existence/atomic "
        "and main/native/full-board requirements."
    )
    record["next_priority"] = next_priority
    ledger = ROOT / "docs/properties-status.json"
    before_bytes = ledger.read_bytes()
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == before_bytes
    before, after = json.loads(before_bytes), json.loads(before_bytes)
    p4 = next(row for row in after["properties"] if row["id"] == "P4")
    for name in [broad_path.name, output.name]:
        p4["evidence"].append(str(E.relative_to(ROOT) / name))
    update = (
        "The v2172-v2180 decoder checks pass on the exact v2169 proof source: 1074 scalar "
        "tests plus one child, 5858 consumer tests plus three children, 1059 default-feature "
        "tests plus three children and 1074 debug tests plus one child. All 33 ignored cases "
        "and 14 inherited warning lines are unchanged. Both strict Clippy configurations, "
        "release build, workspace formatting and 108 included-file format checks pass. "
        "The decoder's 655-unit conditional proof and 245 fresh negative guards retain "
        "their scope. The prepared compiled-binary comparison, cost qualification and "
        "remaining replays are still pending; v1836 remains retained. All property/target "
        "statuses, thirteen whole-crate groups and main/native/full-board requirements "
        "remain unchanged."
    )
    p4["remaining"] += " " + update
    assert after["next_actions"][0].count(proof["next_priority"]) == 1
    after["next_actions"][0] = after["next_actions"][0].replace(proof["next_priority"], next_priority, 1)
    after["next_actions"][0] += " " + update
    same = copy.deepcopy(after)
    old = next(row for row in before["properties"] if row["id"] == "P4")
    compare = next(row for row in same["properties"] if row["id"] == "P4")
    compare["evidence"], compare["remaining"] = old["evidence"], old["remaining"]
    same["next_actions"][0] = before["next_actions"][0]
    assert same == before
    output.write_text(json.dumps(record, indent=2) + "\n")
    ledger.write_text(json.dumps(after, indent=2) + "\n")
    ledger_output.write_text(json.dumps({"before_sha256": hashlib.sha256(before_bytes).hexdigest(),
        "after_sha256": sha(ledger), "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
        "all_39_property_and_14_target_requirements_and_statuses_preserved": True,
        "milestone_record_sha256": sha(output), "runtime_retention_pending": True}, indent=2) + "\n")
    print(json.dumps({"runtime_commands_passed": 9, "runtime_retention_pending": True}))


if __name__ == "__main__":
    main()
