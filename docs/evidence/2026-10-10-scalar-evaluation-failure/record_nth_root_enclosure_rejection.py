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
    out = E / "nth-root-enclosure-cost-rejection-v2021.json"
    ledger_record = E / "nth-root-enclosure-rejection-ledger-v2021.json"
    assert not out.exists() and not ledger_record.exists()
    costs_path = E / "nth-root-enclosure-cost-review-v2021.json"
    costs = read(costs_path)
    initial_path = E / costs["initial_record"]
    initial = read(initial_path)
    controller_path = E / "nth-root-enclosure-cost-validation-v2021.json"
    controller = read(controller_path)
    proof = read(E / "nth-root-enclosure-proof-validation-v2004.json")
    broad = read(E / "nth-root-enclosure-broad-validation-v2013.json")
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
    assert costs["successful_cost_processes_including_standalone"] == 2160
    assert costs["separate_cpu_preparation_processes"] == 1692 and costs["original_process_caps_preserved"]
    assert all(row["non_timing_outputs_identical"] for row in costs["families"].values())
    assert len(costs["followups"]) == 1
    repeat = costs["followups"][0]
    assert repeat["successful_processes"] == 756 and repeat["failed_attempts"] == 0
    assert sum(map(len, repeat["cases"].values())) == 14
    persistent = costs["persistent_above_five_percent_increases"]
    assert {(row["family"], tuple(row["arguments"])) for row in persistent} == {
        ("integer", ("wide_gap", "10000"))}
    assert all(row["median_percent_change"] > 5 for row in persistent)
    assert sha(E / costs["followup_strategy"]) == costs["followup_strategy_sha256"] == controller["followup_strategy_sha256"]
    for phase_name in [initial["paired_phase"], repeat["phase"]]:
        phase = E / phase_name
        summary = read(phase / "summary.json")
        assert summary["guards_unchanged"] and not (phase / "failed-attempt.json").exists()
        assert all(sha(phase / name) == digest for name, digest in summary["artifacts_sha256"].items())
    source_record = E / "typed-api-prototype-v2007-nth-root-enclosure-release-build/source-hashes.json.gz"
    assert sha(source_record) == expected
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    assert len(sources) == 24746
    assert all(sha(SOURCE / name) == digest for name, digest in sources.items())
    assert sha(E / proof["retained_runtime_checkpoint"]) == proof["retained_runtime_checkpoint_sha256"]
    assert not (E / "nth-root-enclosure-cost-milestone-v2021.json").exists()
    assert not (E / "nth-root-enclosure-replay-validation-v2075.json").exists()
    assert not (E / "typed-api-prototype-v2077-state.json").exists()
    allocation_path = E / "nth-root-enclosure-allocation-study-v2021/summary.json"
    allocation = read(allocation_path)
    resource_path = E / "nth-root-enclosure-resource-review-v2021.json"
    resource = read(resource_path)
    assert allocation["source_record_sha256"] == resource["source_record_sha256"] == expected
    assert allocation["guards_unchanged"] and allocation["counts_stable"] and allocation["exact_endpoints_equal"]
    assert allocation["fixtures"] == 84 and allocation["kernel_calls"] == 504 and not allocation["increases"]
    assert resource["cost_controller_sha256"] == sha(controller_path) and not resource["cost_controller_passed"]
    assert resource["allocation_study_sha256"] == sha(allocation_path)
    for name, digest in allocation["artifacts_sha256"].items():
        assert sha(allocation_path.parent / name) == digest
    main_guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert len(main_guard) == 1167 and all(sha(ROOT / name) == digest for name, digest in main_guard.items())
    assert proof["fresh_rejected_mutants"] == proof["fresh_passing_guard_baselines"] == 0
    assert proof["reused_rejected_mutants"] == 247 and proof["reused_passing_guard_baselines"] == 4
    assert sha(E / proof["prior_failure"]) == proof["prior_failure_sha256"]
    assert not read(E / proof["prior_failure"])["runtime_retained"]
    next_priority = (
        "Investigate the prescribed-repeat wide-integer-gap increase using the preserved binaries "
        "and unchanged controls. Inspect generated code and the exact-integer fast path before "
        "proposing a justified source change. Any revised proposal needs a new frozen "
        "source/protocol and all required proof, runtime, cost and replay gates. Preserve this "
        "failed comparison; do not retry unchanged timings until a favorable result appears."
    )
    record = {
        "status": "Reject the isolated combined root/cache proposal: a prescribed repeat control exceeds the frozen timing threshold",
        "source_record": str(source_record.relative_to(E)), "source_record_sha256": expected,
        "cost_record": costs_path.name, "cost_record_sha256": sha(costs_path),
        "cost_controller": controller_path.name, "cost_controller_sha256": sha(controller_path),
        "retained_runtime_checkpoint": proof["retained_runtime_checkpoint"],
        "retained_runtime_checkpoint_sha256": proof["retained_runtime_checkpoint_sha256"],
        "proposal_patch": proof["proposal_patch"], "proposal_patch_sha256": proof["proposal_patch_sha256"],
        "resource_review": resource_path.name, "resource_review_sha256": sha(resource_path),
        "allocation_study": str(allocation_path.relative_to(E)), "allocation_study_sha256": sha(allocation_path),
        "main_guarded_files_verified": len(main_guard),
        "persistent_increases": persistent,
        "initial_and_followup_percent": costs["initial_and_followup_percent"],
        "successful_cost_processes": 2160, "separate_cpu_preludes": 1692,
        "failed_capped_cost_processes": 0, "cost_gate_passed": False,
        "original_fixture_count": 52, "family_repeat_fixture_count": 14,
        "executable_size_comparison": costs["executable_size_comparison"],
        "passing_evidence": {"verified_units": 651, "fresh_rejected_mutants": 0,
            "fresh_passing_guard_baselines": 0, "reused_rejected_mutants": 247,
            "reused_passing_guard_baselines": 4, "fresh_acceptance_tests": 10, "focused_tests": 3,
            "broad_runtime_commands": 9, "all_observed_non_timing_cost_outputs_identical": True},
        "replays_started": False, "runtime_retained": False, "main_runtime_admitted": False,
        "limits": [
            "The gate rejects this candidate based on the frozen family-repeat rule. The three initially flagged cases fall below the threshold on repeat, but the unchanged wide_gap control rises 11.175507824761223 percent in that repeat (its initial change was -6.268224844107484 percent). The predeclared rule checks every fixture in each affected family, not only the initially flagged cases. Timing establishes neither a code-level cause nor a uniform regression.",
            "The 468 standalone, 936 original paired and 756 family-repeat processes all pass their original caps, as do 1692 separate CPU preludes. Process success does not override the failed timing gate.",
            "The seven probe executables shrink by 864-1008 bytes, recorded separately from timing. The copied-kernel allocation study reduces requests in all 84 cases, and the full algebraic fixture has lower observed whole-process wall time. These separate results do not override the failed timing gate or establish whole-board effects.",
            "The refreshed conditional cache proof, reused mutation guards and nine runtime checks remain valid evidence for their declared source and scope. The bigint root helper is outside Verus coverage. Prepared cost-milestone/retention helpers and the 39-cap/4129-numerical/648-public replay protocol were not executed; none is an admission result.",
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
    for name in [initial_path.name, costs_path.name, resource_path.name, str(allocation_path.relative_to(E)), out.name]:
        path = str(E.relative_to(ROOT) / name)
        assert path not in p4["evidence"]
        p4["evidence"].append(path)
    update = (
        "The v2014-v2021 cost gate rejects the combined one-root enclosure and outlined cache "
        "proposal. All 2160 cost processes and 1692 CPU preludes pass their original limits and "
        "preserve every non-timing output. All three initial increases fall below five percent "
        "on the prescribed 27-pair repeat, but another unchanged integer control (wide_gap) "
        "increases 11.1755 percent; its initial change was -6.2682 percent. Every affected-family "
        "control remains part of the frozen gate. All 84 copied-kernel allocation cases use "
        "fewer requests and bytes with identical endpoints; seven probes shrink 864-1008 bytes. "
        "The original algebraic 4/4/0/false/64 fixture passes all paired processes and has "
        "48.0550 percent lower median whole-process wall time, while its timed query loop "
        "changes +1.1863 percent. These observations do not isolate a causal contribution or "
        "override the failed timing gate. All initial/repeated observations and earlier "
        "failures remain immutable. The 651-unit conditional proof, 247 reused mutation "
        "guards and nine broad checks retain their scope; the bigint helper is not formally "
        "verified. Replays and retention were not executed. Retained runtime stays v1836, "
        "with all property/target statuses, fixed pre-Verus baseline and main/native/full-board "
        "obligations unchanged."
    )
    p4["remaining"] += " " + update
    old = ("Next run the frozen cost controls, capped/numerical/public replays and receipt audits "
           "before deciding retention.")
    assert after["next_actions"][0].count(old) == 1
    after["next_actions"][0] = after["next_actions"][0].replace(old,
        "The frozen cost comparison rejects the combined root/cache proposal; retain v1836. " + next_priority, 1)
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
    print(json.dumps({"status": record["status"], "cost_processes_passed": 2160,
        "persistent_increases": [{key: row[key] for key in ["family", "arguments", "median_percent_change"]}
                                  for row in persistent], "retained_checkpoint": proof["retained_runtime_checkpoint"]}))


if __name__ == "__main__":
    main()
