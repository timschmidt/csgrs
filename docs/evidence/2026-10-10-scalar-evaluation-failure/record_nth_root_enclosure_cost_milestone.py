"""Publish passing local cost evidence while retaining the pending replay gates."""
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
    costs_path = E / "nth-root-enclosure-cost-review-v2021.json"
    costs = read(costs_path)
    initial_path = E / costs["initial_record"]
    initial = read(initial_path)
    broad = read(E / "nth-root-enclosure-broad-milestone-v2013.json")
    controller_path = E / "nth-root-enclosure-cost-validation-v2021.json"
    controller = read(controller_path)
    assert controller["complete"] and costs["local_timing_and_process_gates_passed"]
    assert not costs["persistent_above_five_percent_increases"]
    assert len({row["source_record_sha256"] for row in [costs, initial, broad, controller]}) == 1
    assert sha(initial_path) == costs["initial_record_sha256"]
    assert initial["paired_complete"] and initial["complete_fixtures"] == 52
    assert initial["successful_paired_processes"] == 936 and initial["standalone_successful_processes"] == 468
    assert initial["failed_attempt"] is None and not initial["partial_fixtures"]
    assert costs["original_process_caps_preserved"]
    assert all(item["non_timing_outputs_identical"] for item in costs["families"].values())
    followup_processes = sum(row["successful_processes"] for row in costs["followups"])
    assert all(row["failed_attempts"] == 0 for row in costs["followups"])
    processes, preludes = 1404 + followup_processes, 936 + followup_processes
    assert costs["successful_cost_processes_including_standalone"] == processes
    assert costs["separate_cpu_preparation_processes"] == preludes
    assert costs["original_run"]["failed_attempts"] == 0
    for row in controller["steps"]:
        assert row["exit_code"] == 0
        assert sha(Path(row["command"][1])) == row["helper_sha256"]
        assert sha(E / row["stdout"]) == row["stdout_sha256"]
        assert sha(E / row["stderr"]) == row["stderr_sha256"]
    strategy = E / costs["followup_strategy"]
    assert sha(strategy) == controller["followup_strategy_sha256"] == costs["followup_strategy_sha256"]
    out = E / "nth-root-enclosure-cost-milestone-v2021.json"
    ledger_record = E / "nth-root-enclosure-cost-ledger-v2021.json"
    assert not out.exists() and not ledger_record.exists()
    record = {
        "status": "Frozen local cost controls and any prescribed family repeats pass; runtime retention awaits replays",
        "source_record": broad["source_record"], "source_record_sha256": costs["source_record_sha256"],
        "cost_record": costs_path.name, "cost_record_sha256": sha(costs_path),
        "cost_controller": controller_path.name, "cost_controller_sha256": sha(controller_path),
        "retained_runtime_checkpoint": broad["retained_runtime_checkpoint"],
        "retained_runtime_checkpoint_sha256": broad["retained_runtime_checkpoint_sha256"],
        "successful_cost_processes": processes, "separate_cpu_preludes": preludes,
        "standalone_processes": 468, "original_paired_fixtures": 52, "original_paired_processes": 936,
        "followup_processes": followup_processes, "failed_cost_processes": 0,
        "initial_and_followup_percent": costs["initial_and_followup_percent"],
        "executable_size_comparison": costs["executable_size_comparison"],
        "next_replay_protocol": "nth-root-enclosure-replay-protocol-v2022.json",
        "next_replay_runner": "run_nth_root_replays.py",
        "limits": [
            "Every initial observation and any family-wide 27-pair repeat are retained. The repeat rule was frozen before candidate measurements. No causal attribution or uniform speedup is established.",
            "Certification counts, checksums and all non-timing output fields are unchanged. These probes do not measure allocations or consumed evaluation work. Probe executable sizes are separate local observations and do not establish whole-board size parity.",
            "The cost baseline is retained v1836. Older original-cap failures at v1638/v1710, the earlier integer-control tradeoff and the fixed pre-Verus acceptance baseline remain separate and unchanged.",
            "The combined proposal remains unretained pending all 39 capped phases, 4129 numerical cases, 648 public reuse schedules and their receipt/fact audits. Main/native/full-board admission and all thirteen whole-crate proof groups remain open.",
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
    increases = costs["initial_and_followup_percent"]
    timing = ("No original nine-pair fixture exceeds the five-percent repeat threshold. "
              if not increases else
              f"The {len(increases)} initial increases above five percent trigger the predeclared "
              "27-pair repeats of every fixture in each affected family; none of those repeated "
              "fixtures has a remaining increase above five percent. ")
    update = (
        "The v2014-v2021 local cost gates pass on the unchanged combined kernel/selector source: "
        "468 standalone processes and all 936 original-cap paired processes cover 52 fixtures. "
        + timing
        + f"All {processes} cost processes and {preludes} separate CPU preludes pass their original "
        "limits. Certification counts, checksums and non-timing output fields are unchanged. These probes do not measure allocations or consumed evaluation work. All timing "
        "samples and executable sizes remain recorded without a causal or uniform-speedup claim. "
        "The retained runtime stays v1836 pending the frozen capped, numerical and public replays "
        "and fact audits. Older failures/tradeoff, the fixed pre-Verus baseline, all property/target "
        "statuses and main/native/full-board obligations remain unchanged."
    )
    p4["remaining"] += " " + update
    old = ("Next run the frozen cost controls, capped/numerical/public replays and receipt audits "
           "before deciding retention.")
    assert after["next_actions"][0].count(old) == 1
    after["next_actions"][0] = after["next_actions"][0].replace(old,
        "The frozen local cost controls and prescribed repeat rule pass. Next run all "
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
    print(json.dumps({"cost_processes_passed": processes, "cpu_preludes_passed": preludes,
                      "runtime_retention_pending": True}))


if __name__ == "__main__":
    main()
