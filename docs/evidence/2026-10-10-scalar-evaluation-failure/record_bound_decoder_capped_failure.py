"""Preserve the baseline CPU failure and audit the exact interrupted pair order."""
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
    outputs = [E / name for name in ["bound-decoder-partial-cost-review-v2247.json",
        "bound-decoder-cost-rejection-v2247.json", "bound-decoder-cost-rejection-ledger-v2247.json"]]
    assert not any(path.exists() for path in outputs)
    initial_path = E / "bound-decoder-initial-cost-review-v2247.json"
    initial = read(initial_path)
    controller_path = E / "bound-decoder-cost-validation-v2247.json"
    controller = read(controller_path)
    proof = read(E / "bound-decoder-proof-validation-v2171.json")
    broad = read(E / "bound-decoder-binary-milestone-v2247.json")
    protocol_path = E / "bound-decoder-cost-protocol-v2240.json"
    protocol = read(protocol_path)
    strategy_path = E / "bound-decoder-cost-followup-strategy-v2247.json"
    strategy = read(strategy_path)
    expected_source = proof["source_record_sha256"]
    assert expected_source == "80bb17679a0891ff93eba8ed10b9f63c205f3ccb39e35dbb3e187e23bb0b7162"
    assert all(row["source_record_sha256"] == expected_source for row in [initial, controller, broad, protocol, strategy])
    assert not initial["paired_complete"] and not controller["complete"]
    assert initial["standalone_successful_processes"] == 468 and initial["successful_paired_processes"] == 864
    assert initial["complete_fixtures"] == 48 and initial["prelude_processes"] == 865
    assert initial["protocol_sha256"] == sha(protocol_path) == strategy["cost_protocol_sha256"]
    assert sha(strategy_path) == controller["followup_strategy_sha256"]
    assert [(row["label"], row["exit_code"]) for row in controller["steps"]] == [
        ("standalone", 0), ("paired", 1), ("initial-review", 0)]
    for row in controller["steps"]:
        assert sha(Path(row["command"][1])) == row["helper_sha256"]
        assert sha(E / row["stdout"]) == row["stdout_sha256"]
        assert sha(E / row["stderr"]) == row["stderr_sha256"]
    phase = E / initial["paired_phase"]
    assert not (phase / "summary.json").exists()
    assert all(sha(phase / name) == digest for name, digest in initial["original_run_artifacts_sha256"].items())
    assert sha(phase / "reproduce.py") == protocol["runner_forks"]["replay_bound_decoder_cost.py"]["fork_sha256"]
    preparation = read(phase / "preparation.json")
    assert preparation["iterations"] == 50000000 and preparation["timed_query_inputs_and_caps_unchanged"]
    assert preparation["cpu"] == min(preparation["available_cpus"])
    assert sha(Path(preparation["build_command"][-1])) == preparation["prelude_sha256"]
    assert sha(phase / "prelude.rs") == sha(E / "acos_cost_cpu_prelude.rs") == preparation["prelude_source_sha256"]
    planned = []
    for family in ["atan", "acos", "integer", "inverse_trig", "normal_form", "default", "algebraic"]:
        retained = initial["families"][family]
        commands = {}
        for which, name in retained["phases"].items():
            commands[which] = [row for row in read(E / name / "commands.json") if row["name"] not in {"format", "build"}]
            assert sha(Path(commands[which][0]["command"][0])) == retained["binary_sha256"][which]
        assert all(sha(E / retained["phases"]["baseline"] / name) == sha(E / retained["phases"]["candidate"] / name)
                   for name in ["probe.rs", "reproduce.py"])
        counts = defaultdict(int)
        for index, (before, after) in enumerate(zip(commands["baseline"], commands["candidate"])):
            assert before["name"] == after["name"] and before["command"][1:] == after["command"][1:]
            args = tuple(before["command"][1:])
            occurrence = counts[args]
            counts[args] += 1
            for which in (["baseline", "candidate"] if occurrence % 2 == 0 else ["candidate", "baseline"]):
                command = commands[which][index]
                assert command["address_space_cap_mib"] == 192 and command["cpu_seconds"] == [2, 3]
                assert command["timeout_seconds"] == 4
                planned.append({"name": f"{family}-{index}-{which}", "family": family, "which": which,
                    "arguments": list(args), "command": command["command"], "binary_sha256": retained["binary_sha256"][which],
                    "reference": E / retained["phases"][which] / (command["name"] + ".stdout")})
        assert set(counts) == {tuple(args) for args in strategy["family_fixtures"][family]}
        assert all(count == 9 for count in counts.values())
    assert len(planned) == 936
    rows = read(phase / "receipts.json")
    failure = read(phase / "failed-attempt.json")
    assert len(rows) == 864 and failure == initial["failed_attempt"]
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
    assert next_expected["which"] == "baseline" and failure["name"] == "algebraic-18-baseline"
    assert failure["command"][1:] == ["cost", "4", "4", "0", "false", "64"]
    assert failure["exit_code"] == -signal.SIGXCPU == -24 and not failure["wall_timeout"]
    assert failure["address_space_bytes"] == 192 * 1024 * 1024
    assert failure["cpu_seconds_soft_hard"] == [2, 3] and failure["wall_seconds_limit"] == 4
    assert all((phase / (failure["name"] + suffix)).read_bytes() == b"" for suffix in [".stdout", ".stderr"])
    preludes = read(phase / "prelude-receipts.json")
    assert len(preludes) == 865 and [row["before"] for row in preludes] == [row["name"] for row in planned[:865]]
    assert all(row["exit_code"] == 0 and row["stderr"] == "" for row in preludes)
    assert len({row["stdout"] for row in preludes}) == 1
    cases = []
    for family, fixtures in strategy["family_fixtures"].items():
        for args in fixtures:
            values = samples[family, tuple(args)]
            counts = {which: len(value) for which, value in values.items()}
            complete = all(count == 9 for count in counts.values())
            medians = {which: statistics.median(value) if value else None for which, value in values.items()}
            cases.append({"family": family, "arguments": args, "complete_nine_pairs": complete,
                "successful_samples": counts, "elapsed_ns": values, "median_ns": medians,
                "median_percent_change": 100 * (medians["candidate"] / medians["baseline"] - 1) if complete else None})
    complete = [row for row in cases if row["complete_nine_pairs"]]
    partial = [row for row in cases if not row["complete_nine_pairs"] and sum(row["successful_samples"].values())]
    unstarted = [row for row in cases if not sum(row["successful_samples"].values())]
    assert (len(cases), len(complete), len(partial), len(unstarted)) == (52, 48, 0, 4)
    assert not partial
    wide_gap = next(row for row in complete if row["family"] == "integer" and row["arguments"] == ["wide_gap", "10000"])
    increases = [row for row in complete if row["median_percent_change"] > 5]
    assert [(row["family"], row["arguments"]) for row in increases] == [
        ("integer", ["wide_gap", "10000"]), ("integer", ["wide_fine", "10000"]),
        ("integer", ["wide_identity", "10000"]), ("default", ["structural_cancel", "500"])]
    source_record = E / proof["source_record"]
    assert sha(source_record) == expected_source
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert len(sources) == 24746 and all(sha(SOURCE / name) == digest for name, digest in sources.items())
    assert len(guard) == 1167 and all(sha(ROOT / name) == digest for name, digest in guard.items())
    replay = read(E / "bound-decoder-measured-replay-protocol-v2247.json")
    assert all(not (E / item["phase"]).exists() for group in ["caps", "numerical_replays", "public_replays"] for item in replay[group])
    for name in ["bound-decoder-cost-followup-protocol-v2247.json", "bound-decoder-cost-review-v2247.json",
                 "bound-decoder-cost-milestone-v2247.json", "bound-decoder-replay-validation-v2235.json", "typed-api-prototype-v2248-state.json"]:
        assert not (E / name).exists()
    review = {
        "status": "Original comparison stopped at a retained-baseline CPU failure; cost gate remains failed",
        "source_record_sha256": expected_source, "initial_record": initial_path.name, "initial_record_sha256": sha(initial_path),
        "controller": controller_path.name, "controller_sha256": sha(controller_path),
        "paired_phase": phase.name, "failure": failure, "failure_stage": "Unknown: no intermediate probe output",
        "planned_paired_processes": 936, "successful_paired_processes": 864, "failed_paired_processes": 1,
        "unstarted_paired_processes": 71, "successful_standalone_processes": 468,
        "successful_cost_processes": 1332, "attempted_cost_processes": 1333, "separate_cpu_preludes": 865,
        "complete_fixtures": 48, "observed_partial_fixtures": 0, "fixtures_without_successful_samples": 4,
        "failed_before_first_success_fixtures": 1, "wholly_unattempted_fixtures": 3, "cases": cases,
        "complete_fixture_increases_above_five_percent": increases,
        "wide_gap_complete_comparison": wide_gap,
        "followup_started": False, "replays_started": False, "runtime_retained": False,
        "independent_post_failure_guard": {"source_files_verified": len(sources), "main_files_verified": len(guard)},
        "artifacts_sha256": {path.name: sha(path) for path in sorted(phase.iterdir()) if path.is_file()},
        "limits": [
            "The failure is in the hash-identical retained v1836 baseline executable, not the new candidate. This identifies the failing binary, not an internal stage or a reason for the CPU consumption.",
            "All successful processes preserve non-timing output fields and original caps. Forty-eight fixtures are complete; the failing fixture has no successful paired sample and three further fixtures were never attempted. Four complete fixtures have increases above five percent; they remain unresolved because the original run did not complete. No timing observation passes the failed process gate.",
            "The frozen timing-repeat rule cannot turn an original capped failure into a pass. No family repeats, replays or retention steps were started. Earlier baseline and candidate failures remain unchanged; a diagnostic or larger-cap run cannot replace this failure.",
        ],
    }
    next_priority = (
        "Restore the exact retained v1836 source after archiving the decoder's failed cost gate. "
        "Inspect the frozen baseline/candidate executable differences before choosing a next "
        "proof-compatible implementation experiment. Preserve the full-size algebraic CPU "
        "failure and every original fixture/cap; do not repeat unchanged timing runs until "
        "one passes. Producer/existence/atomic, resource and main/native/full-board work "
        "remain open."
    )
    rejection = {
        "status": "Do not retain the bound-decoder proposal after a baseline capped failure",
        "source_record": proof["source_record"], "source_record_sha256": expected_source,
        "retained_runtime_checkpoint": proof["retained_runtime_checkpoint"],
        "retained_runtime_checkpoint_sha256": proof["retained_runtime_checkpoint_sha256"],
        "proposal_patch": proof["proposal_patch"], "proposal_patch_sha256": proof["proposal_patch_sha256"],
        "partial_cost_record": outputs[0].name, "cost_controller": controller_path.name,
        "cost_controller_sha256": sha(controller_path), "failure": failure,
        "successful_cost_processes": 1332, "failed_capped_cost_processes": 1, "separate_cpu_preludes": 865,
        "cost_gate_passed": False, "runtime_retained": False, "replays_started": False, "main_runtime_admitted": False,
        "passing_evidence": {"verified_units": 655, "listed_production_contracts": 98,
            "listed_model_theorems": 135, "fresh_rejected_mutations": 245,
            "reused_rejected_mutations": 0, "fresh_passing_baselines": 4,
            "fresh_acceptance_tests": 10, "broad_commands": 9},
        "proof_record": "bound-decoder-proof-validation-v2171.json",
        "proof_record_sha256": sha(E / "bound-decoder-proof-validation-v2171.json"),
        "limits": review["limits"] + ["The archived conditional proof verifies the actual decoder and five connected field lemmas, but the implementation remains unretained. Producer truth, existence, atomic/caller ordering, all thirteen whole-crate groups, the fixed pre-Verus acceptance baseline and main/native/full-board admission remain open."],
        "next_priority": next_priority,
    }
    ledger = ROOT / "docs/properties-status.json"
    before_bytes = ledger.read_bytes()
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == before_bytes
    before, after = json.loads(before_bytes), json.loads(before_bytes)
    assert len(after["properties"]) == 39 and len(after["targets"]) == 14
    p4 = next(row for row in after["properties"] if row["id"] == "P4")
    for name in [initial_path.name, outputs[0].name, outputs[1].name]:
        path = str(E.relative_to(ROOT) / name)
        assert path not in p4["evidence"]
        p4["evidence"].append(path)
    update = (
        "The v2240-v2247 decoder cost comparison stops at algebraic-18-baseline: the "
        "retained v1836 executable hits SIGXCPU under the original 192 MiB, CPU 2/3-second "
        "and four-second wall limits, with no intermediate output. All 468 standalone and "
        "864 paired processes before the failure pass, as do 865 preludes. Forty-eight "
        "fixtures have all nine pairs; the failed fixture has no successful paired sample "
        "and three later fixtures are unattempted. Seventy-one paired processes remain "
        "unstarted. Four complete fixtures exceed the five-percent repeat threshold, but "
        "the incomplete original run prevents repeats. The 655-unit conditional proof, "
        "245 fresh negative guards and nine broad checks remain archived; no replay or "
        "runtime retention follows. The candidate stays v1836 and every property/target "
        "status and main/native/full-board requirement remains unchanged."
    )
    p4["remaining"] += " " + update
    old = broad["next_priority"]
    assert after["next_actions"][0].count(old) == 1
    after["next_actions"][0] = after["next_actions"][0].replace(old,
        "The original-cap comparison stopped at a retained-baseline CPU failure; retain v1836. " + next_priority, 1)
    after["next_actions"][0] += " " + update
    unchanged = copy.deepcopy(after)
    prior_p4 = next(row for row in before["properties"] if row["id"] == "P4")
    current_p4 = next(row for row in unchanged["properties"] if row["id"] == "P4")
    current_p4["evidence"], current_p4["remaining"] = prior_p4["evidence"], prior_p4["remaining"]
    unchanged["next_actions"][0] = before["next_actions"][0]
    assert unchanged == before
    outputs[0].write_text(json.dumps(review, indent=2) + "\n")
    rejection["partial_cost_record_sha256"] = sha(outputs[0])
    outputs[1].write_text(json.dumps(rejection, indent=2) + "\n")
    ledger.write_text(json.dumps(after, indent=2) + "\n")
    outputs[2].write_text(json.dumps({"before_sha256": hashlib.sha256(before_bytes).hexdigest(), "after_sha256": sha(ledger),
        "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
        "all_39_property_and_14_target_requirements_and_statuses_preserved": True,
        "rejection_record_sha256": sha(outputs[1]), "runtime_retained": False}, indent=2) + "\n")
    print(json.dumps({"failed_binary": "retained baseline", "successful_cost_processes": 1332,
        "complete_fixtures": 48, "partial_fixtures": 0, "fixtures_without_successful_samples": 4,
        "wholly_unattempted_fixtures": 3, "unstarted_processes": 71, "runtime_retained": False}))


if __name__ == "__main__":
    main()
