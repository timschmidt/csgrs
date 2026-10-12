"""Audit the interrupted original-cap repeat without treating partial data as a pass."""
from collections import defaultdict
import copy
import gzip
import hashlib
import json
from pathlib import Path
import signal
import statistics
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
nontiming = lambda row: {key: value for key, value in row.items() if key != "elapsed_ns"}


def main():
    outputs = [E / name for name in [
        "cache-state-merge-partial-cost-review-v1943.json",
        "cache-state-merge-cost-rejection-v1943.json",
        "cache-state-merge-cost-rejection-ledger-v1943.json",
    ]]
    assert all(not path.exists() for path in outputs)
    initial_path = E / "cache-state-merge-initial-cost-review-v1943.json"
    initial = read(initial_path)
    protocol_path = E / "cache-state-merge-cost-followup-protocol-v1943.json"
    protocol = read(protocol_path)
    strategy_path = E / protocol["followup_strategy"]
    strategy = read(strategy_path)
    controller_path = E / "cache-state-merge-cost-validation-v1943.json"
    controller = read(controller_path)
    proof_path = E / "cache-state-merge-proof-validation-v1926.json"
    proof = read(proof_path)
    broad = read(E / "cache-state-merge-broad-milestone-v1935.json")
    validation = read(E / "cache-state-merge-candidate-validation-v1935.json")
    assert proof["verified_units"] == 651 and proof["errors"] == 0
    assert proof["fresh_rejected_mutants"] == 247 and proof["fresh_passing_guard_baselines"] == 4
    assert proof["fresh_acceptance_tests"] == 10 and proof["open_obligation_groups"] == 13
    assert validation["complete"] and len(validation["phases"]) == 9
    assert all(row["result"]["exit_code"] == 0 and row["result"]["source_unchanged"]
               and row["result"]["main_unchanged"] for row in validation["phases"].values())
    expected_source = proof["source_record_sha256"]
    assert expected_source == "555cf4a48df68e021119c107412824b5879baf48e252c6fb1c8d2bc22baf1c59"
    assert all(row["source_record_sha256"] == expected_source
               for row in [initial, protocol, strategy, controller, broad, validation])
    assert sha(initial_path) == protocol["original_record_sha256"]
    assert sha(strategy_path) == protocol["followup_strategy_sha256"] == controller["followup_strategy_sha256"]
    assert initial["protocol_sha256"] == strategy["cost_protocol_sha256"]
    assert initial["paired_complete"] and initial["complete_fixtures"] == 52
    assert initial["successful_paired_processes"] == 936 and initial["standalone_successful_processes"] == 468
    assert not initial["partial_fixtures"] and initial["failed_attempt"] is None
    original_phase = E / initial["paired_phase"]
    for name, digest in initial["original_run_artifacts_sha256"].items():
        assert sha(original_phase / name) == digest
    assert not controller["complete"]
    assert [(row["label"], row["exit_code"]) for row in controller["steps"]] == [
        ("standalone", 0), ("paired", 0), ("initial-review", 0),
        ("freeze-followup", 0), ("followup-0", 1)]
    for row in controller["steps"]:
        assert sha(Path(row["command"][1])) == row["helper_sha256"]
        assert sha(E / row["stdout"]) == row["stdout_sha256"]
        assert sha(E / row["stderr"]) == row["stderr_sha256"]
    increases = [row for row in initial["cases"] if row["median_percent_change"] > 5]
    assert increases == initial["above_five_percent_increases"] == protocol["initial_above_five_percent_increases"]
    affected = {row["family"] for row in increases}
    assert len(protocol["jobs"]) == 1
    config = protocol["jobs"][0]
    assert config["selection"] == {family: fixtures for family, fixtures in strategy["family_fixtures"].items() if family in affected}
    assert config["repetitions"] == 27 and config["cpu_seconds"] == [2, 3] and config["wall_seconds"] == 4
    phase = E / config["output"]
    assert not (phase / "summary.json").exists()
    assert not (E / "cache-state-merge-cost-review-v1943.json").exists()
    assert not (E / "cache-state-merge-cost-milestone-v1943.json").exists()
    preparation = read(E / "cache-state-merge-cost-followup-preparation-v1943.json")
    assert sha(phase / "reproduce.py") == sha(E / preparation["runner"]) == preparation["runner_sha256"]
    warmup = read(phase / "preparation.json")
    assert warmup["iterations"] == 50_000_000 and warmup["original_process_caps_preserved"]
    assert warmup["process_cpu_seconds"] == [2, 3] and warmup["process_wall_seconds"] == 4
    assert warmup["cpu"] == min(warmup["available_cpus"])
    assert sha(Path(warmup["build_command"][-1])) == warmup["prelude_sha256"]
    assert sha(phase / "prelude.rs") == sha(E / "acos_cost_cpu_prelude.rs") == warmup["prelude_source_sha256"]

    # Reconstruct the exact planned order, including the original nine-pair
    # fixture ordering repeated three times and per-fixture alternation.
    planned, provenance = [], {}
    family_order = ["atan", "acos", "integer", "inverse_trig", "normal_form", "default", "algebraic"]
    for family in family_order:
        if family not in affected:
            continue
        retained = initial["families"][family]
        selected = {tuple(args) for args in config["selection"][family]}
        commands = {}
        for which, phase_name in retained["phases"].items():
            commands[which] = [row for row in read(E / phase_name / "commands.json")
                               if row["name"] not in {"format", "build"}
                               and tuple(row["command"][1:]) in selected] * 3
            assert len(commands[which]) == 27 * len(selected)
            binary = Path(commands[which][0]["command"][0])
            assert sha(binary) == retained["binary_sha256"][which]
        assert all(sha(E / retained["phases"]["baseline"] / name)
                   == sha(E / retained["phases"]["candidate"] / name)
                   for name in ["probe.rs", "reproduce.py"])
        provenance[family] = retained
        occurrences = defaultdict(int)
        for index, (old, new) in enumerate(zip(commands["baseline"], commands["candidate"])):
            assert old["name"] == new["name"] and old["command"][1:] == new["command"][1:]
            args = tuple(old["command"][1:])
            occurrence = occurrences[args]
            occurrences[args] += 1
            for which in (["baseline", "candidate"] if occurrence % 2 == 0 else ["candidate", "baseline"]):
                command = commands[which][index]
                assert command["address_space_cap_mib"] == 192
                assert command["cpu_seconds"] == [2, 3] and command["timeout_seconds"] == 4
                planned.append({"name": f"{family}-{index}-{which}", "family": family,
                    "which": which, "arguments": list(args), "command": command["command"],
                    "binary_sha256": retained["binary_sha256"][which],
                    "reference": E / retained["phases"][which] / (command["name"] + ".stdout")})
    assert len(planned) == 1296
    rows = read(phase / "receipts.json")
    failure = read(phase / "failed-attempt.json")
    assert len(rows) == 1116 and len(rows) < len(planned)
    samples = defaultdict(lambda: {"baseline": [], "candidate": []})
    for row, expected in zip(rows, planned):
        assert all(row[key] == expected[key] for key in ["name", "command", "binary_sha256"])
        assert row["exit_code"] == 0 and row["address_space_bytes"] == 192 * 1024 * 1024
        assert row["cpu_seconds_soft_hard"] == [2, 3] and row["wall_seconds_limit"] == 4
        assert read(phase / (row["name"] + ".stdout")) == row["receipt"]
        assert (phase / (row["name"] + ".stderr")).read_bytes() == b""
        assert nontiming(row["receipt"]) == nontiming(read(expected["reference"]))
        samples[(expected["family"], tuple(expected["arguments"]))][expected["which"]].append(int(row["receipt"]["elapsed_ns"]))
    next_expected = planned[len(rows)]
    assert all(failure[key] == next_expected[key] for key in ["name", "command", "binary_sha256"])
    assert failure["exit_code"] == -signal.SIGXCPU == -24 and not failure["wall_timeout"]
    assert failure["address_space_bytes"] == 192 * 1024 * 1024
    assert failure["cpu_seconds_soft_hard"] == [2, 3] and failure["wall_seconds_limit"] == 4
    assert failure["command"][1:] == ["cost", "4", "4", "0", "false", "64"]
    assert all((phase / (failure["name"] + suffix)).read_bytes() == b"" for suffix in [".stdout", ".stderr"])
    preludes = read(phase / "prelude-receipts.json")
    assert len(preludes) == len(rows) + 1 == 1117
    assert [row["before"] for row in preludes] == [row["name"] for row in planned[:len(preludes)]]
    assert all(row["exit_code"] == 0 and row["stderr"] == "" for row in preludes)
    assert len({row["stdout"] for row in preludes}) == 1

    cases, complete, partial = [], [], []
    for family, fixtures in config["selection"].items():
        for args in fixtures:
            values = samples[(family, tuple(args))]
            counts = {which: len(items) for which, items in values.items()}
            finished = all(count == 27 for count in counts.values())
            medians = {which: statistics.median(items) if items else None for which, items in values.items()}
            row = {"family": family, "arguments": args, "complete_27_pairs": finished,
                "successful_samples": counts, "elapsed_ns": values, "median_ns": medians,
                "median_percent_change": 100 * (medians["candidate"] / medians["baseline"] - 1)
                    if all(value is not None for value in medians.values()) else None}
            cases.append(row)
            (complete if finished else partial).append(row)
    assert len(complete) == 18 and len(partial) == 6
    assert all(row["family"] in {"atan", "inverse_trig"} for row in complete)
    assert all(row["median_percent_change"] <= 5 for row in complete)
    assert all(row["family"] == "algebraic" for row in partial)

    # The interrupted runner never reached its closing source guards. Check
    # them independently now and distinguish this audit from a completed run.
    source_record = E / proof["source_record"]
    assert sha(source_record) == expected_source
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    main_guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert len(sources) == 24746 and len(main_guard) == 1167
    for name, digest in sources.items():
        assert sha(SOURCE / name) == digest
    for name, digest in main_guard.items():
        assert sha(ROOT / name) == digest
    replay_plan = read(E / "cache-state-merge-replay-protocol-v1944.json")
    assert all(not (E / item["phase"]).exists() for group in ["caps", "numerical_replays", "public_replays"]
               for item in replay_plan[group])
    assert not (E / "cache-state-merge-replay-validation-v1997.json").exists()
    assert not (E / "typed-api-prototype-v1999-state.json").exists()
    successful = 468 + 936 + len(rows)
    prelude_count = 936 + len(preludes)
    artifacts = {path.name: sha(path) for path in sorted(phase.iterdir()) if path.is_file()}
    review = {
        "status": "Original-cap family repeat interrupted by candidate SIGXCPU; cost gate failed",
        "source_record_sha256": expected_source, "runtime_retained": False,
        "initial_record": initial_path.name, "initial_record_sha256": sha(initial_path),
        "followup_protocol": protocol_path.name, "followup_protocol_sha256": sha(protocol_path),
        "followup_strategy": strategy_path.name, "followup_strategy_sha256": sha(strategy_path),
        "controller": controller_path.name, "controller_sha256": sha(controller_path),
        "followup_phase": phase.name, "followup_artifacts_sha256": artifacts,
        "failure": failure, "failure_stage": "Unknown: the probe has no intermediate output",
        "planned_followup_processes": len(planned), "successful_followup_processes": len(rows),
        "failed_followup_processes": 1, "unstarted_followup_processes": len(planned) - len(rows) - 1,
        "successful_cost_processes": successful, "attempted_cost_processes": successful + 1,
        "separate_cpu_preludes": prelude_count,
        "complete_followup_fixtures": len(complete), "partial_followup_fixtures": len(partial),
        "followup_cases": cases, "initial_above_five_percent_increases": increases,
        "family_provenance": provenance, "all_successful_non_timing_outputs_identical": True,
        "independent_post_failure_guard": {"source_files_verified": len(sources), "main_files_verified": len(main_guard)},
        "interpretation": [
            "The original 468 standalone and 936 paired processes passed. The prescribed repeat then stopped at its first failed process. No run is retried, truncated into success or substituted with a larger cap.",
            "All eighteen complete atan/inverse-trig fixtures have repeated median increases at or below five percent. The six algebraic fixtures are partial and do not satisfy their 27-pair requirement; their sample medians are descriptive only.",
            "The algebraic cost probe constructs and warms all 64 values before timing the sign queries. Its only output follows the timed loop. Empty failed output cannot identify construction, warming or query execution as the failing stage.",
            "The cap failure does not establish causal attribution to the selector. Older baseline failures on the same full-size fixture remain relevant and unchanged. All sampled outputs, source guards and proof results have separate scopes.",
        ],
    }
    next_priority = (
        "Diagnose CPU work on the unchanged full-size algebraic fixture, preserving construction, "
        "warming and query stages and every original cap. Keep this outlined selector unretained "
        "until a concrete correction passes a fresh frozen comparison and the pending replays. "
        "A diagnostic with extra instrumentation or a larger cap cannot replace the failed run."
    )
    rejection = {
        "status": "Do not retain the outlined cache-state merge after its capped family repeat failed",
        "source_record": proof["source_record"], "source_record_sha256": expected_source,
        "retained_runtime_checkpoint": proof["retained_runtime_checkpoint"],
        "retained_runtime_checkpoint_sha256": proof["retained_runtime_checkpoint_sha256"],
        "partial_cost_record": outputs[0].name,
        "cost_controller": controller_path.name, "cost_controller_sha256": sha(controller_path),
        "proposal_patch": proof["proposal_patch"], "proposal_patch_sha256": proof["proposal_patch_sha256"],
        "failure": failure, "successful_cost_processes": successful,
        "failed_capped_cost_processes": 1, "separate_cpu_preludes": prelude_count,
        "cost_gate_passed": False, "runtime_retained": False,
        "replays_started": False, "main_runtime_admitted": False,
        "passing_evidence": {"verified_units": 651, "negative_mutations": 247,
            "positive_guard_baselines": 4, "acceptance_tests": 10, "runtime_validation_commands": 9},
        "limits": [
            "The retained runtime remains v1836; the working isolated source is an unretained proposal. All original worktree entries and main runtime are preserved.",
            "The fresh conditional proof remains valid for its exact source. Producer soundness, independent real existence, decoder equivalence, atomic publication, caller obligations and all thirteen whole-crate proof groups remain open.",
            "The earlier v1857 BoundInfo extraction failed its timing gate. The v1638/v1710 original-cap failures, prior retained integer-control tradeoff and fixed pre-Verus baseline remain explicit; none is reset by this attempt.",
            "EvaluationBudget excludes aggregate bytes/time and unmetered algebraic helpers. Whole-workspace/CI, native combination, full-board admission and all 39 properties, 14 targets and crate future work remain in scope.",
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
    for path in outputs[:2]:
        name = str(path.relative_to(ROOT))
        assert name not in p4["evidence"]
        p4["evidence"].append(name)
    update = (
        "The v1943 outlined-selector comparison is not retained: all 468 standalone and 936 "
        "original paired processes passed, but the prescribed family repeat stopped after "
        "1116 successes when candidate algebraic-72 (cost 4 4 0 false 64) hit SIGXCPU at the "
        "unchanged two-second CPU limit. Its empty output does not identify the failing stage. "
        "All 2520 successful cost receipts and 2053 CPU preludes are preserved, together with "
        "the failed attempt and 179 unstarted repeat processes. Eighteen complete atan/inverse-trig "
        "fixtures are below the five-percent repeat threshold; all six algebraic fixtures remain "
        "partial. No capped/numerical/public replays or new retention checkpoint were run. "
        "The proof and nine runtime checks still apply only to their declared scope; v1836 "
        "remains retained, with older failures/tradeoff and the full properties plan unchanged."
    )
    p4["remaining"] += " " + update
    old = "Next run the frozen cost controls, capped/numerical/public replays and receipt audits before deciding retention."
    assert after["next_actions"][0].count(old) == 1
    after["next_actions"][0] = after["next_actions"][0].replace(old,
        "The outlined selector remains unretained after its original-cap repeat failed. " + next_priority, 1)
    after["next_actions"][0] += " " + update
    unchanged = copy.deepcopy(after)
    original_p4 = next(row for row in before["properties"] if row["id"] == "P4")
    compare_p4 = next(row for row in unchanged["properties"] if row["id"] == "P4")
    compare_p4["evidence"], compare_p4["remaining"] = original_p4["evidence"], original_p4["remaining"]
    unchanged["next_actions"][0] = before["next_actions"][0]
    assert unchanged == before
    outputs[0].write_text(json.dumps(review, indent=2) + "\n")
    rejection["partial_cost_record_sha256"] = sha(outputs[0])
    outputs[1].write_text(json.dumps(rejection, indent=2) + "\n")
    ledger.write_text(json.dumps(after, indent=2) + "\n")
    outputs[2].write_text(json.dumps({
        "before_sha256": hashlib.sha256(before_bytes).hexdigest(), "after_sha256": sha(ledger),
        "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
        "all_39_property_and_14_target_requirements_and_statuses_preserved": True,
        "rejection_record_sha256": sha(outputs[1]), "runtime_retained": False,
        "retained_runtime_checkpoint": proof["retained_runtime_checkpoint"],
    }, indent=2) + "\n")
    print(json.dumps({"successful_cost_processes": successful, "failed_cost_processes": 1,
        "cpu_preludes": prelude_count, "complete_repeat_fixtures": len(complete),
        "partial_repeat_fixtures": len(partial), "runtime_retained": False}))


if __name__ == "__main__":
    main()
