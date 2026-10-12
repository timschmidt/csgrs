"""Record complete guards/runtime validation and scoped cost evidence before replay."""
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
    output = E / "eager-bound-decoder-broad-milestone-v2279.json"
    ledger_output = E / "eager-bound-decoder-broad-ledger-v2279.json"
    assert not output.exists() and not ledger_output.exists()
    early_path = E / "eager-bound-decoder-early-milestone-v2265.json"
    early = read(early_path)
    expected = early["source_record_sha256"]
    broad_path = E / "eager-bound-decoder-broad-validation-v2276.json"
    validation_path = E / "eager-bound-decoder-candidate-validation-v2276.json"
    broad, validation = read(broad_path), read(validation_path)
    assert broad["source_record_sha256"] == validation["source_record_sha256"] == expected
    assert validation["complete"] and len(validation["phases"]) == 9
    assert validation["new_runtime_commands"] == 6 and validation["already_completed_runtime_commands"] == 3
    for name, row in validation["phases"].items():
        assert read(E / name / "result.json") == row["result"]
        assert row["result"]["exit_code"] == 0 and row["result"]["source_unchanged"] and row["result"]["main_unchanged"]
        assert sha(E / name / "source-hashes.json.gz") == expected
    assert broad["strict_clippy_warning_lines"] == 0 and len(broad["inherited_stack_warning_lines"]) == 14
    assert broad["ignored_test_names_unchanged"] and broad["included_rustfmt_files"] == 108
    remaining_path = E / "eager-bound-decoder-remaining-guards-v2267.json"
    remaining = read(remaining_path)
    assert remaining["complete"] and remaining["source_record_sha256"] == expected
    guards = []
    tool_hashes = None
    for phase, rejected, positive in [
        ("typed-api-prototype-v2265-eager-bound-decoder-cache-rejections", 41, 1),
        ("typed-api-prototype-v2267-eager-bound-decoder-non-cache-rejections", 204, 3),
    ]:
        p = E / phase
        result = read(p / "result.json")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
        assert sha(p / "source-hashes.json.gz") == expected
        tooling = read(p / "proof-checks/tooling.json")
        calls_path = p / "proof-checks/verus-invocations.jsonl.gz"
        assert sha(calls_path) == tooling["invocation_log_sha256"]
        calls = [json.loads(s) for s in gzip.decompress(calls_path.read_bytes()).splitlines()]
        assert tooling["exit_code"] == 0 and tooling["tool_unchanged"]
        assert len(calls) == tooling["verus_invocations"] == rejected + positive
        assert sum(c["exit_code"] != 0 for c in calls) == tooling["rejected_invocations"] == rejected
        assert sum(c["exit_code"] == 0 for c in calls) == tooling["baseline_successes"] == positive
        if tool_hashes is not None:
            assert tooling["tool_sha256"] == tool_hashes
        tool_hashes = tooling["tool_sha256"]
        guards.append({"phase": phase, "tooling_sha256": sha(p / "proof-checks/tooling.json"),
            "invocation_log_sha256": sha(calls_path), "rejected_mutations": rejected, "passing_baselines": positive})
    selected = E / remaining["phase"] / "proof-checks"
    selection = read(selected / "selection.json")
    original = Path(selection["original_entrypoint"])
    assert sha(original) == selection["original_entrypoint_sha256"]
    assert sha(selected / "selected-entrypoint.py") == selection["selected_entrypoint_sha256"]
    omitted = selection["only_removed_call"]
    assert omitted == "    reject_cache_mutations()\n" and original.read_text().count(omitted) == 1
    assert (selected / "selected-entrypoint.py").read_text() == original.read_text().replace(omitted, "")
    assert early["verified_units"] == 655 and early["fresh_acceptance_tests"] == 10
    assert early["listed_production_contracts"] == 98 and early["listed_model_theorems"] == 135
    assert early["open_obligation_groups"] == 13
    source_record = E / early["source_record"]
    assert sha(source_record) == expected
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    main = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert len(sources) == 24746 and all(sha(SOURCE / n) == h for n, h in sources.items())
    assert len(main) == 1167 and all(sha(ROOT / n) == h for n, h in main.items())
    qualification_path = E / "eager-bound-decoder-computation-review-v2279.json"
    qualification = read(qualification_path)
    assert qualification["source_record_sha256"] == expected and qualification["qualified_for_fresh_replays"]
    assert qualification["historical_matching_successful_processes"] == 1017
    assert len(qualification["historical_failures"]) == 2
    assert not qualification["original_full_file_identity_rule_satisfied"]
    assert not qualification["all_historical_original_caps_pass"] and qualification["fresh_timing_processes"] == 0
    replay_path = E / "eager-bound-decoder-replay-protocol-v2280.json"
    replay = read(replay_path)
    assert replay["source_record_sha256"] == expected and replay["cost_qualification_sha256"] == sha(qualification_path)
    assert all(not (E / row["phase"]).exists() for group in ["caps", "numerical_replays", "public_replays"] for row in replay[group])
    next_priority = (
        "The eager decoder's complete proof/guard/runtime checks and separately frozen "
        "unchanged-computation review now pass. Run the frozen 39 capped commands, 4129 "
        "numerical cases and 648 public schedules, then compare receipts and audit every "
        "retained fact before deciding isolated retention. Whole-file identity remains "
        "false; historical timings remain historical and both later baseline CPU failures "
        "remain unresolved. Preserve all original limits, the fixed pre-Verus baseline "
        "and open producer/existence/atomic, resource and main/native/full-board obligations."
    )
    inputs = [early_path, broad_path, validation_path, remaining_path, qualification_path, replay_path]
    record = {"status": "All nine current-source runtime commands and all proof guards pass; unchanged replay gates are next",
        "source_record": early["source_record"], "source_record_sha256": expected,
        "source_files_verified": len(sources), "main_files_verified": len(main),
        "changed_sources_sha256": early["changed_sources_sha256"],
        "retained_runtime_checkpoint": early["retained_runtime_checkpoint"],
        "retained_runtime_checkpoint_sha256": early["retained_runtime_checkpoint_sha256"],
        "inputs_sha256": {str(p.relative_to(E)): sha(p) for p in inputs},
        "verified_units": 655, "listed_production_contracts": 98, "listed_model_theorems": 135,
        "open_obligation_groups": 13, "fresh_acceptance_tests": 10, "fresh_rejected_mutations": 245,
        "fresh_passing_guard_baselines": 4, "guard_groups": guards,
        "runtime_commands_passed": 9, "early_runtime_commands": 3, "remaining_runtime_commands": 6,
        "identical_instruction_probe_families": 7, "identical_complete_executables": 0,
        "cost_evidence_method": qualification["method"], "historical_matching_successful_processes": 1017,
        "later_baseline_cpu_failures_preserved": 2, "fresh_timing_processes": 0,
        "runtime_retained": False, "main_runtime_admitted": False, "next_priority": next_priority,
        "limits": [
            "The five-crate suite passes 5858 top-level tests and three children, with 33 ignored cases and 14 inherited example-name warnings. All-feature scalar and debug suites each pass 1074 plus one child; default features pass 1059 plus three. Both strict Clippy configurations, release build, workspace fmt and 108 included-file format checks pass. This is not complete repository CI.",
            "The 655-unit conditional proof covers the actual all-word decoder and connected field lemmas; no new whole-crate group is closed. All 245 negative guards, four baselines and ten acceptance checks executed on this candidate. Cache guards ran separately; only their already completed call was omitted from the other-groups entrypoint.",
            "Static instruction/layout identity and a 194-reference diagnostic audit support the scoped differential-cost method. The 1017 older matching successes keep their original identities. The two later same-baseline SIGXCPU failures remain failed; no fresh timing pass, absolute cap guarantee, uniform performance claim or full executable equivalence is inferred.",
            "Replays and fact audits are prepared but unexecuted. Runtime stays v1836. All 39 properties, 14 targets, future work, producer/existence/atomic obligations and fixed-baseline/native/full-board requirements remain unchanged.",
        ]}
    ledger = ROOT / "docs/properties-status.json"
    before_bytes = ledger.read_bytes()
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == before_bytes
    before, after = json.loads(before_bytes), json.loads(before_bytes)
    p4 = next(row for row in after["properties"] if row["id"] == "P4")
    for p in [broad_path, qualification_path, replay_path, output]:
        p4["evidence"].append(str(p.relative_to(ROOT)))
    update = (
        "The v2267-v2279 eager-decoder milestone completes all 245 negative mutations, "
        "four positive baselines and nine runtime commands on the current source. Scalar "
        "and debug suites each pass 1074 plus one child; the five-crate suite passes 5858 "
        "plus three, and default features pass 1059 plus three. The 33 ignored cases and "
        "14 inherited warnings are unchanged; both strict Clippy checks and all build/format "
        "checks pass. The seven instruction-identical probes have 194 direct diagnostic "
        "references to the changed line records. A separately frozen computation review "
        "permits fresh unchanged replays, citing 1017 historical matching executions while "
        "preserving both later baseline CPU failures. Whole-file identity still fails and "
        "no fresh timing pass is claimed. The first reader's missing-field failure and its "
        "schema correction remain archived. All 39 capped commands, 4129 numerical cases "
        "and 648 public schedules are frozen and pending. Runtime remains v1836; proof "
        "scope and every property/target/native/full-board requirement remain unchanged."
    )
    p4["remaining"] += " " + update
    assert after["next_actions"][0].count(early["next_priority"]) == 1
    after["next_actions"][0] = after["next_actions"][0].replace(early["next_priority"], next_priority, 1)
    after["next_actions"][0] += " " + update
    same = copy.deepcopy(after)
    old = next(row for row in before["properties"] if row["id"] == "P4")
    changed = next(row for row in same["properties"] if row["id"] == "P4")
    changed["remaining"], changed["evidence"] = old["remaining"], old["evidence"]
    same["next_actions"][0] = before["next_actions"][0]
    assert same == before
    output.write_text(json.dumps(record, indent=2) + "\n")
    ledger.write_text(json.dumps(after, indent=2) + "\n")
    ledger_output.write_text(json.dumps({"before_sha256": hashlib.sha256(before_bytes).hexdigest(),
        "after_sha256": sha(ledger), "milestone_record_sha256": sha(output),
        "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
        "all_39_property_and_14_target_requirements_and_statuses_preserved": True,
        "runtime_retention_pending": True}, indent=2) + "\n")
    print(json.dumps({"runtime_commands_passed": 9, "fresh_rejected_mutations": 245,
                      "fresh_passing_baselines": 4, "runtime_retention_pending": True}))


if __name__ == "__main__":
    main()
