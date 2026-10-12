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
    outputs = [E / name for name in ["integer-coarse-zero-partial-cost-review-v2100.json",
        "integer-coarse-zero-cost-rejection-v2100.json", "integer-coarse-zero-cost-rejection-ledger-v2100.json"]]
    assert not any(path.exists() for path in outputs)
    initial_path = E / "integer-coarse-zero-initial-cost-review-v2100.json"
    initial = read(initial_path)
    controller_path = E / "integer-coarse-zero-cost-validation-v2100.json"
    controller = read(controller_path)
    proof = read(E / "integer-coarse-zero-proof-validation-v2083.json")
    broad = read(E / "integer-coarse-zero-broad-milestone-v2092.json")
    protocol_path = E / "integer-coarse-zero-cost-protocol-v2093.json"
    protocol = read(protocol_path)
    strategy_path = E / "integer-coarse-zero-cost-followup-strategy-v2100.json"
    strategy = read(strategy_path)
    expected_source = proof["source_record_sha256"]
    assert expected_source == "e093cd7ed95ae15ecb66d04cd76beb89b04aa205971330f4b2794db4190ded62"
    assert all(row["source_record_sha256"] == expected_source for row in [initial, controller, broad, protocol, strategy])
    assert not initial["paired_complete"] and not controller["complete"]
    assert initial["standalone_successful_processes"] == 468 and initial["successful_paired_processes"] == 880
    assert initial["complete_fixtures"] == 48 and initial["prelude_processes"] == 881
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
    assert sha(phase / "reproduce.py") == protocol["runner_forks"]["replay_integer_coarse_zero_cost.py"]["fork_sha256"]
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
    assert len(rows) == 880 and failure == initial["failed_attempt"]
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
    assert next_expected["which"] == "baseline" and failure["name"] == "algebraic-26-baseline"
    assert failure["command"][1:] == ["cost", "4", "4", "0", "false", "64"]
    assert failure["exit_code"] == -signal.SIGXCPU == -24 and not failure["wall_timeout"]
    assert failure["address_space_bytes"] == 192 * 1024 * 1024
    assert failure["cpu_seconds_soft_hard"] == [2, 3] and failure["wall_seconds_limit"] == 4
    assert all((phase / (failure["name"] + suffix)).read_bytes() == b"" for suffix in [".stdout", ".stderr"])
    preludes = read(phase / "prelude-receipts.json")
    assert len(preludes) == 881 and [row["before"] for row in preludes] == [row["name"] for row in planned[:881]]
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
    assert (len(cases), len(complete), len(partial), len(unstarted)) == (52, 48, 1, 3)
    assert partial[0]["successful_samples"] == {"baseline": 8, "candidate": 8}
    wide_gap = next(row for row in complete if row["family"] == "integer" and row["arguments"] == ["wide_gap", "10000"])
    increases = [row for row in complete if row["median_percent_change"] > 5]
    assert [(row["family"], row["arguments"]) for row in increases] == [("integer", ["small_coarse", "200000"])]
    source_record = E / proof["source_record"]
    assert sha(source_record) == expected_source
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert len(sources) == 24746 and all(sha(SOURCE / name) == digest for name, digest in sources.items())
    assert len(guard) == 1167 and all(sha(ROOT / name) == digest for name, digest in guard.items())
    replay = read(E / "integer-coarse-zero-replay-protocol-v2101.json")
    assert all(not (E / item["phase"]).exists() for group in ["caps", "numerical_replays", "public_replays"] for item in replay[group])
    for name in ["integer-coarse-zero-cost-followup-protocol-v2100.json", "integer-coarse-zero-cost-review-v2100.json",
                 "integer-coarse-zero-cost-milestone-v2100.json", "integer-coarse-zero-replay-validation-v2154.json", "typed-api-prototype-v2156-state.json"]:
        assert not (E / name).exists()
    review = {
        "status": "Original comparison stopped at a retained-baseline CPU failure; cost gate remains failed",
        "source_record_sha256": expected_source, "initial_record": initial_path.name, "initial_record_sha256": sha(initial_path),
        "controller": controller_path.name, "controller_sha256": sha(controller_path),
        "paired_phase": phase.name, "failure": failure, "failure_stage": "Unknown: no intermediate probe output",
        "planned_paired_processes": 936, "successful_paired_processes": 880, "failed_paired_processes": 1,
        "unstarted_paired_processes": 55, "successful_standalone_processes": 468,
        "successful_cost_processes": 1348, "attempted_cost_processes": 1349, "separate_cpu_preludes": 881,
        "complete_fixtures": 48, "observed_partial_fixtures": 1, "unstarted_fixtures": 3, "cases": cases,
        "complete_fixture_increases_above_five_percent": increases,
        "wide_gap_complete_comparison": wide_gap,
        "followup_started": False, "replays_started": False, "runtime_retained": False,
        "independent_post_failure_guard": {"source_files_verified": len(sources), "main_files_verified": len(guard)},
        "artifacts_sha256": {path.name: sha(path) for path in sorted(phase.iterdir()) if path.is_file()},
        "limits": [
            "The failure is in the hash-identical retained v1836 baseline executable, not the new candidate. This identifies the failing binary, not an internal stage or a reason for the CPU consumption.",
            "All successful processes preserve non-timing output fields and original caps. Partial and unstarted fixtures remain incomplete. A completed wide_gap median improvement does not pass the comparison, and the small_coarse increase remains unresolved because the original run did not complete.",
            "The frozen timing-repeat rule cannot turn an original capped failure into a pass. No family repeats, replays or retention steps were started. Earlier baseline and candidate failures remain unchanged; a diagnostic or larger-cap run cannot replace this failure.",
        ],
    }
    next_priority = (
        "Diagnose the retained baseline's full-size algebraic CPU failure and its execution environment "
        "from preserved evidence before proposing a controlled next experiment. Keep every original "
        "fixture and cap, and preserve this failed comparison; do not rerun unchanged timings until "
        "a favorable result appears. The coarse-zero proposal remains unretained and the decoder, "
        "producer/existence/atomic and native/full-board obligations remain open."
    )
    rejection = {
        "status": "Do not retain the combined integer/root/cache proposal after a baseline capped failure",
        "source_record": proof["source_record"], "source_record_sha256": expected_source,
        "retained_runtime_checkpoint": proof["retained_runtime_checkpoint"],
        "retained_runtime_checkpoint_sha256": proof["retained_runtime_checkpoint_sha256"],
        "proposal_patch": proof["proposal_patch"], "proposal_patch_sha256": proof["proposal_patch_sha256"],
        "partial_cost_record": outputs[0].name, "cost_controller": controller_path.name,
        "cost_controller_sha256": sha(controller_path), "failure": failure,
        "successful_cost_processes": 1348, "failed_capped_cost_processes": 1, "separate_cpu_preludes": 881,
        "cost_gate_passed": False, "runtime_retained": False, "replays_started": False, "main_runtime_admitted": False,
        "passing_evidence": {"focused_integer_tests": 5, "root_controls": 3, "verified_units": 651,
            "fresh_rejected_mutations": 0, "reused_rejected_mutations": 247, "reused_passing_baselines": 4,
            "fresh_acceptance_tests": 10, "broad_commands": 9, "fresh_integer_allocation_cases": 64},
        "allocation_review": "integer-coarse-zero-allocation-review-v2092.json",
        "limits": review["limits"] + ["The integer shortcut and bigint root helper remain outside Verus coverage. All thirteen whole-crate groups, the fixed pre-Verus acceptance baseline and main/native/full-board admission remain open."],
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
        "The v2093-v2100 cost comparison stops at baseline algebraic-26-baseline: the retained "
        "v1836 binary hits SIGXCPU under the original 192 MiB, CPU 2/3-second and four-second "
        "wall limits, with no intermediate output. All 468 standalone and 880 paired processes "
        "before that failure pass, as do 881 preludes. Forty-eight fixtures have all nine pairs; "
        "one is partial and three unstarted, leaving 55 processes unstarted. The completed "
        "wide_gap median changes -23.0894 percent, while small_coarse changes +5.3966 percent. "
        "Neither observation resolves the failed process gate; no prescribed repeats or replays "
        "were started. All raw results, the baseline failure and earlier rejections remain "
        "preserved. The nine broad checks, focused tests, scoped proof and allocation evidence "
        "do not admit the runtime. Retained candidate stays v1836 and every property/target "
        "status and main/native/full-board requirement remains unchanged."
    )
    p4["remaining"] += " " + update
    old = "Next run the frozen cost controls, capped/numerical/public replays and receipt audits before deciding retention."
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
    print(json.dumps({"failed_binary": "retained baseline", "successful_cost_processes": 1348,
        "complete_fixtures": 48, "partial_fixtures": 1, "unstarted_fixtures": 3,
        "unstarted_processes": 55, "runtime_retained": False}))


if __name__ == "__main__":
    main()
