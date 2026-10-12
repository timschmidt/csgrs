"""Publish scoped cost evidence without skipping the pending replay gates."""
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
    costs_path = E / "domain-predicate-cost-review-v1780.json"
    costs = read(costs_path)
    initial = read(E / costs["initial_record"])
    broad = read(E / "domain-predicate-broad-milestone-v1772.json")
    assert costs["source_record_sha256"] == initial["source_record_sha256"] == broad["source_record_sha256"]
    assert sha(E / costs["initial_record"]) == costs["initial_record_sha256"]
    assert costs["successful_cost_processes_including_standalone"] == 1566
    assert costs["separate_cpu_preparation_processes"] == 1098 and costs["original_process_caps_preserved"]
    assert initial["paired_complete"] and initial["complete_fixtures"] == 52
    assert initial["failed_attempt"] is None and not initial["partial_fixtures"]
    assert all(item["non_timing_outputs_identical"] for item in costs["families"].values())
    assert all(item["delta_bytes"] < 0 for item in costs["executable_size_comparison"].values())
    assert all(item["median_percent_change"] <= 5 for item in costs["followup"]["cases"]["inverse_trig"])
    out = E / "domain-predicate-cost-milestone-v1780.json"
    ledger_record = E / "domain-predicate-cost-ledger-v1780.json"
    assert not out.exists() and not ledger_record.exists()
    record = {
        "status": "All frozen local cost controls and the prescribed repeat complete under original caps; runtime retention awaits replays",
        "source_record": broad["source_record"], "source_record_sha256": costs["source_record_sha256"],
        "cost_record": costs_path.name, "cost_record_sha256": sha(costs_path),
        "retained_runtime_checkpoint": broad["retained_runtime_checkpoint"],
        "retained_runtime_checkpoint_sha256": broad["retained_runtime_checkpoint_sha256"],
        "successful_cost_processes": 1566, "separate_cpu_preludes": 1098,
        "original_paired_fixtures": 52, "original_paired_processes": 936,
        "followup_processes": 162, "failed_cost_processes": 0,
        "initial_and_followup_pi_percent": costs["initial_and_followup_pi_percent"],
        "next_replay_protocol": "domain-predicate-replay-protocol-v1781.json",
        "next_replay_runner": "run_domain_predicate_replays.py",
        "limits": [
            "The 5.0501 percent initial pi increase and 1.0191 percent 27-pair repeat are both retained. No causal attribution or uniform speedup is established.",
            "All non-timing outputs are unchanged. Seven probe executables shrink by 48 to 152 bytes; these local sizes do not establish whole-board size parity.",
            "The cost baseline is the retained v1754 runtime. Older v1638/v1710 failures and the integer-control tradeoff remain separate evidence; the fixed pre-Verus acceptance baseline is not reset.",
            "The extraction remains unretained pending all 39 capped phases, 4129 numerical cases, 648 public reuse schedules and their receipt/fact audits. Main/native/full-board admission and all thirteen whole-crate proof groups remain open.",
        ],
    }
    ledger = ROOT / "docs/properties-status.json"
    before_bytes = ledger.read_bytes()
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == before_bytes
    before = json.loads(before_bytes)
    after = copy.deepcopy(before)
    assert len(after["properties"]) == 39 and len(after["targets"]) == 14
    p4 = next(row for row in after["properties"] if row["id"] == "P4")
    for name in [costs["initial_record"], costs_path.name, out.name]:
        path = str(E.relative_to(ROOT) / name)
        assert path not in p4["evidence"]
        p4["evidence"].append(path)
    update = (
        "The v1773-v1780 cost gates complete on the unchanged proved extraction: 468 "
        "standalone processes and all 936 original-cap paired processes cover 52 fixtures. "
        "The single initial increase above five percent is the 1024-bit pi control at "
        "5.0501 percent; the frozen 27-pair follow-up measures 1.0191 percent, with atan "
        "and acos controls at 0.4078 and 0.5842 percent. All 1566 cost processes and 1098 "
        "separate CPU preludes pass the original limits. Samples, allocations, work counts "
        "and checksums are unchanged; probe executables shrink 48-152 bytes. Both initial "
        "and follow-up observations remain recorded without a uniform-speedup claim. "
        "The retained runtime stays v1754 pending the frozen capped, numerical and public "
        "replays and fact audits. Older failures/tradeoff, the fixed pre-Verus baseline, "
        "all property/target statuses and main/native/full-board obligations remain unchanged."
    )
    p4["remaining"] += " " + update
    old = ("Next run the frozen cost controls, capped/numerical/public replays and receipt audits "
           "before deciding retention.")
    assert after["next_actions"][0].count(old) == 1
    after["next_actions"][0] = after["next_actions"][0].replace(old,
        "The frozen local cost controls and prescribed follow-up now pass. Next run all "
        "capped/numerical/public replays and receipt/fact audits before deciding retention.", 1)
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
    print(json.dumps({"cost_processes_passed": 1566, "cpu_preludes_passed": 1098,
                      "runtime_retention_pending": True}))


if __name__ == "__main__":
    main()
