"""Reject the extraction after its prescribed cost repeat; preserve all evidence."""
import copy
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
    out = E / "bound-merge-cost-rejection-v1857.json"
    ledger_record = E / "bound-merge-rejection-ledger-v1857.json"
    assert not out.exists() and not ledger_record.exists()
    costs_path = E / "bound-merge-cost-review-v1857.json"
    costs = read(costs_path)
    initial_path = E / costs["initial_record"]
    initial = read(initial_path)
    controller_path = E / "bound-merge-cost-validation-v1857.json"
    controller = read(controller_path)
    proof = read(E / "bound-merge-proof-validation-v1840.json")
    broad = read(E / "bound-merge-broad-validation-v1849.json")
    expected = costs["source_record_sha256"]
    assert expected == initial["source_record_sha256"] == controller["source_record_sha256"]
    assert expected == proof["source_record_sha256"] == broad["source_record_sha256"]
    assert sha(initial_path) == costs["initial_record_sha256"]
    assert not costs["local_timing_and_process_gates_passed"] and not controller["complete"]
    assert [(row["label"], row["exit_code"]) for row in controller["steps"]] == [
        ("standalone", 0), ("paired", 0), ("initial-review", 0),
        ("freeze-followup", 0), ("followup-0", 0), ("final-review", 1)]
    for row in controller["steps"]:
        assert sha(Path(row["command"][1])) == row["helper_sha256"]
        assert sha(E / row["stdout"]) == row["stdout_sha256"]
        assert sha(E / row["stderr"]) == row["stderr_sha256"]
    assert "Persistent timing increases prevent cost-gate acceptance" in (E / controller["steps"][-1]["stderr"]).read_text()
    assert initial["paired_complete"] and initial["complete_fixtures"] == 52
    assert initial["successful_paired_processes"] == 936 and initial["standalone_successful_processes"] == 468
    assert initial["failed_attempt"] is None and not initial["partial_fixtures"]
    assert costs["successful_cost_processes_including_standalone"] == 2484
    assert costs["separate_cpu_preparation_processes"] == 2016 and costs["original_process_caps_preserved"]
    assert all(row["non_timing_outputs_identical"] for row in costs["families"].values())
    assert len(costs["followups"]) == 1
    repeat = costs["followups"][0]
    assert repeat["successful_processes"] == 1080 and repeat["failed_attempts"] == 0
    assert sum(map(len, repeat["cases"].values())) == 20
    persistent = costs["persistent_above_five_percent_increases"]
    assert {(row["family"], tuple(row["arguments"])) for row in persistent} == {
        ("atan", ("cached_reciprocal_one", "100000")),
        ("atan", ("cached_reciprocal_many", "100000"))}
    assert all(row["median_percent_change"] > 5 for row in persistent)
    assert sha(E / costs["followup_strategy"]) == costs["followup_strategy_sha256"] == controller["followup_strategy_sha256"]
    for phase_name in [initial["paired_phase"], repeat["phase"]]:
        phase = E / phase_name
        summary = read(phase / "summary.json")
        assert summary["guards_unchanged"] and not (phase / "failed-attempt.json").exists()
        assert all(sha(phase / name) == digest for name, digest in summary["artifacts_sha256"].items())
    source_record = E / "typed-api-prototype-v1843-bound-merge-release-build/source-hashes.json.gz"
    assert sha(source_record) == expected
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    assert len(sources) == 24746
    assert all(sha(SOURCE / name) == digest for name, digest in sources.items())
    assert sha(E / proof["retained_runtime_checkpoint"]) == proof["retained_runtime_checkpoint_sha256"]
    assert not (E / "bound-merge-cost-milestone-v1857.json").exists()
    assert not (E / "bound-merge-replay-validation-v1911.json").exists()
    assert not (E / "typed-api-prototype-v1913-state.json").exists()
    next_priority = (
        "Investigate the two persistent cached-reciprocal atan increases using the preserved "
        "binaries and unchanged full-size controls. Any revised extraction needs a new frozen "
        "source/protocol and all required proof, runtime, cost and replay gates. Preserve this "
        "failed comparison; do not retry unchanged timings until a favorable result appears."
    )
    record = {
        "status": "Reject the isolated bound-merge extraction: two prescribed repeat controls exceed the frozen timing threshold",
        "source_record": str(source_record.relative_to(E)), "source_record_sha256": expected,
        "cost_record": costs_path.name, "cost_record_sha256": sha(costs_path),
        "cost_controller": controller_path.name, "cost_controller_sha256": sha(controller_path),
        "retained_runtime_checkpoint": proof["retained_runtime_checkpoint"],
        "retained_runtime_checkpoint_sha256": proof["retained_runtime_checkpoint_sha256"],
        "proposal_patch": proof["proposal_patch"], "proposal_patch_sha256": proof["proposal_patch_sha256"],
        "persistent_increases": persistent,
        "initial_and_followup_percent": costs["initial_and_followup_percent"],
        "successful_cost_processes": 2484, "separate_cpu_preludes": 2016,
        "failed_capped_cost_processes": 0, "cost_gate_passed": False,
        "original_fixture_count": 52, "family_repeat_fixture_count": 20,
        "executable_size_comparison": costs["executable_size_comparison"],
        "passing_evidence": {"verified_units": 651, "fresh_rejected_mutants": 246,
            "fresh_passing_guard_baselines": 4, "fresh_acceptance_tests": 10,
            "broad_runtime_commands": 9, "all_observed_non_timing_cost_outputs_identical": True},
        "replays_started": False, "runtime_retained": False, "main_runtime_admitted": False,
        "limits": [
            "The gate rejects this candidate based on the frozen local comparison. Timing does not establish a code-level cause or a uniform regression; integer-family initial increases did not persist in the repeat.",
            "The 468 standalone, 936 original paired and 1080 family-repeat processes all pass their original caps, as do 2016 separate CPU preludes. Process success does not override the failed timing gate.",
            "The seven probe executables grow by 320-408 bytes, recorded separately from timing. These sizes do not establish whole-board effects.",
            "The conditional production proof and nine runtime checks remain valid evidence for their source. Prepared cost-milestone/retention helpers and the 39-cap/4129-numerical/648-public replay protocol were not executed; none is an admission result.",
            "The retained runtime remains v1836, and the current candidate directory still holds the rejected proposal for diagnosis. Older failures/tradeoffs, the fixed pre-Verus acceptance baseline, thirteen whole-crate proof groups and all main/native/full-board obligations remain unchanged.",
        ],
        "next_priority": next_priority,
    }
    ledger = ROOT / "docs/properties-status.json"
    before_bytes = ledger.read_bytes()
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == before_bytes
    before = json.loads(before_bytes)
    after = copy.deepcopy(before)
    assert len(after["properties"]) == 39 and len(after["targets"]) == 14
    p4 = next(row for row in after["properties"] if row["id"] == "P4")
    for name in [initial_path.name, costs_path.name, out.name]:
        path = str(E.relative_to(ROOT) / name)
        assert path not in p4["evidence"]
        p4["evidence"].append(path)
    update = (
        "The v1850-v1857 cost gate rejects the bound-merge extraction. All 2484 cost processes "
        "and 2016 separate CPU preludes pass their original caps and preserve non-timing "
        "observations, but the predeclared family-wide 27-pair repeat leaves cached reciprocal "
        "atan increases of 9.1308 and 7.2137 percent. Integer-family initial increases do not "
        "persist. Probe executables grow 320-408 bytes. All original and repeated observations "
        "remain immutable; the 651-unit proof and nine passing runtime commands do not override "
        "the failed cost gate. Replays and the prepared retention helper were not run. The "
        "retained runtime remains v1836 and all property/target statuses, older failures/tradeoff, "
        "fixed pre-Verus baseline and main/native/full-board obligations remain unchanged."
    )
    p4["remaining"] += " " + update
    old = ("Next run the frozen cost controls, capped/numerical/public replays and receipt audits "
           "before deciding retention.")
    assert after["next_actions"][0].count(old) == 1
    after["next_actions"][0] = after["next_actions"][0].replace(old,
        "The frozen cost comparison rejects the bound-merge extraction; retain v1836. " + next_priority, 1)
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
        "rejection_record_sha256": sha(out), "runtime_retained": False,
    }, indent=2) + "\n")
    print(json.dumps({"status": record["status"], "cost_processes_passed": 2484,
        "persistent_increases": [{key: row[key] for key in ["family", "arguments", "median_percent_change"]}
                                  for row in persistent], "retained_checkpoint": proof["retained_runtime_checkpoint"]}))


if __name__ == "__main__":
    main()
