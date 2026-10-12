"""Record completed runtime checks while keeping cost and replay admission open."""
import copy
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    broad = E / "integer-coarse-zero-broad-validation-v2092.json"
    data = read(broad)
    validation = read(E / "integer-coarse-zero-candidate-validation-v2092.json")
    proof = read(E / "integer-coarse-zero-proof-validation-v2083.json")
    assert validation["complete"] and len(validation["phases"]) == 9
    assert proof["verified_units"] == 651 and proof["fresh_acceptance_tests"] == 10
    assert proof["focused_tests"] == 5 and proof["root_control_tests"] == 3
    assert proof["fresh_rejected_mutants"] == proof["fresh_passing_guard_baselines"] == 0
    assert proof["reused_rejected_mutants"] == 247 and proof["reused_passing_guard_baselines"] == 4
    assert data["source_record_sha256"] == validation["source_record_sha256"] == proof["source_record_sha256"]
    assert all(row["result"]["exit_code"] == 0 and row["result"]["source_unchanged"]
               and row["result"]["main_unchanged"] for row in validation["phases"].values())
    assert data["strict_clippy_warning_lines"] == 0 and len(data["inherited_stack_warning_lines"]) == 14
    assert data["included_rustfmt_files"] == 108 and data["ignored_test_names_unchanged"]
    out = E / "integer-coarse-zero-broad-milestone-v2092.json"
    ledger_record = E / "integer-coarse-zero-broad-ledger-v2092.json"
    assert not out.exists() and not ledger_record.exists()
    record = {
        "status": "All nine runtime validation commands pass; cost and replay checks remain pending before retaining the combined proposal",
        "source_record": "typed-api-prototype-v2086-integer-coarse-zero-release-build/source-hashes.json.gz",
        "source_record_sha256": data["source_record_sha256"],
        "broad_validation": broad.name, "broad_validation_sha256": sha(broad),
        "proof_record": "integer-coarse-zero-proof-validation-v2083.json",
        "proof_record_sha256": sha(E / "integer-coarse-zero-proof-validation-v2083.json"),
        "retained_runtime_checkpoint": "typed-api-prototype-v1836-state.json",
        "retained_runtime_checkpoint_sha256": sha(E / "typed-api-prototype-v1836-state.json"),
        "next_cost_protocol": "integer-coarse-zero-cost-protocol-v2093.json",
        "next_replay_protocol": "integer-coarse-zero-replay-protocol-v2101.json",
        "prepared_receipt_audits_sha256": {name: sha(E / name) for name in [
            "compare_integer_zero_queries.py", "compare_integer_zero_replays.py",
            "audit_integer_zero_fact_retention.py", "integer-coarse-zero-replay-protocol-v2101.json"]},
        "limits": [
            "The five-crate test suite covers hyperreal, hyperlimit, hyperlattice, hypercurve and hyperdrc, with 33 unchanged ignored tests and 14 inherited Cargo example-filename warning lines. This is not all-workspace/CI or main admission.",
            "Source hashes match the fresh conditional verification (651 units) and ten fresh acceptance tests. The 247 rejected mutations and four passing guard baselines are reused from v1926 because all 44 proof/guard inputs and verifier identities are unchanged; zero mutations were rerun. The modified integer branch and bigint root kernel are outside Verus coverage. All thirteen whole-crate groups remain open.",
            "The combined integer/kernel/selector change is still a proposal. Preserve all original capped fixtures, public reuse schedules, fixed cost baselines and older cost failures/tradeoffs; proof coverage alone cannot justify a runtime regression.",
            "The new receipt/fact audits are prepared, not executed. They will compare all deterministic observations and independently audit concurrent fact retention without requiring race-dependent outcome counts to match.",
        ],
    }
    ledger = ROOT / "docs/properties-status.json"
    before_bytes = ledger.read_bytes()
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == before_bytes
    before = json.loads(before_bytes)
    after = copy.deepcopy(before)
    assert len(after["properties"]) == 39 and len(after["targets"]) == 14
    p4 = next(row for row in after["properties"] if row["id"] == "P4")
    for name in [broad.name, out.name]:
        path = str(E.relative_to(ROOT) / name)
        assert path not in p4["evidence"]
        p4["evidence"].append(path)
    update = (
        "The v2084-v2092 broad checks pass on the exact v2082 source snapshot: "
        "1076 all-feature scalar tests plus one child test, 5860 consumer tests plus three "
        "child tests, 1061 default-feature tests plus three child tests, and 1076 debug "
        "scalar tests plus one child test. The 33 ignored cases and 14 inherited Cargo "
        "warning lines are unchanged. Both strict Clippy configurations, the release "
        "build, workspace formatting and all 108 included-file formatting checks pass. "
        "The retained runtime remains v1836 until the frozen costs, 39 capped phases, "
        "4129 numerical cases and 648 public reuse schedules pass their separate gates. "
        "Receipt comparisons and race-aware fact audits are prepared only. All existing "
        "property/target statuses, whole-crate proof obligations and main/native/full-board "
        "admission limits remain unchanged."
    )
    p4["remaining"] += " " + update
    old = ("The charged integer coarse-zero prototype passes existing focused tests and refreshed conditional verification. "
           "Next run the nine broad commands, freeze the new cost/replay protocols, and evaluate the combined "
           "proposal against v1836 under every unchanged full-size fixture and original cap. Preserve all "
           "earlier failures and do not retain it before all gates pass.")
    assert after["next_actions"][0].count(old) == 1
    after["next_actions"][0] = after["next_actions"][0].replace(old,
        "The nine runtime validation commands pass on the combined integer/kernel/selector source. Next run the frozen "
        "cost controls, capped/numerical/public replays and receipt audits before deciding retention.", 1)
    after["next_actions"][0] += " " + update
    comparison = copy.deepcopy(after)
    old_p4 = next(row for row in before["properties"] if row["id"] == "P4")
    compare_p4 = next(row for row in comparison["properties"] if row["id"] == "P4")
    compare_p4["evidence"], compare_p4["remaining"] = old_p4["evidence"], old_p4["remaining"]
    comparison["next_actions"][0] = before["next_actions"][0]
    assert comparison == before
    out.write_text(json.dumps(record, indent=2) + "\n")
    ledger.write_text(json.dumps(after, indent=2) + "\n")
    ledger_record.write_text(json.dumps({
        "before_sha256": hashlib.sha256(before_bytes).hexdigest(), "after_sha256": sha(ledger),
        "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
        "all_39_property_and_14_target_requirements_and_statuses_preserved": True,
        "milestone_record_sha256": sha(out), "runtime_retention_pending": True,
    }, indent=2) + "\n")
    print(json.dumps({"runtime_checks_passed": 9, "source_record_sha256": data["source_record_sha256"],
                      "runtime_retention_pending": True}))


if __name__ == "__main__":
    main()
