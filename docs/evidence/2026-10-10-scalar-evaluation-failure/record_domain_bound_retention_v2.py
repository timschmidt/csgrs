"""Check the completed evidence and retain the isolated magnitude-domain extension."""
import hashlib
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
from parse_test_results_v3 import parse_phase
from record_domain_sign_retention_v3 import compact_summary, direct_test_counts

E = Path(__file__).resolve().parent
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
SOURCE_HASH = "77f0d0c9d5232b48c71dc1db236e398579486cc4b4b9fb0935ab3d6e4f698eb7"


def phase(version):
    paths = [p for p in E.glob(f"typed-api-prototype-v{version}-*") if p.is_dir()]
    assert len(paths) == 1, (version, paths)
    return paths[0]


def validation(path):
    result = read(path / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    assert sha(path / "source-hashes.json.gz") == SOURCE_HASH
    return result


completed = read(E / "domain-bound-candidate-validation-v1702.json")
remaining = read(E / "domain-bound-remaining-validation-v1753.json")
focused = read(E / "domain-bound-candidate-caps-v1693.json")
replay = read(E / "domain-bound-replay-orchestration-v1745.json")
comparison = read(E / "domain-bound-budget-replay-comparison-v1745.json")
queries = read(E / "domain-bound-query-comparison-v1693.json")
costs = read(E / "domain-bound-cost-review-v1710.json")
proof = read(E / "domain-bound-proof-scope-v1698.json")
for record in [completed, remaining, focused, replay, comparison, queries, costs, proof]:
    assert record["source_record_sha256"] == SOURCE_HASH
assert remaining["complete"] and replay["complete"] and focused["complete"]
assert len(completed["phases"]) == 9 and len(remaining["results"]) == 38
assert len(focused["tests"]) == 9
validations = {}
for version in [1684, *range(1694, 1703)]:
    path = phase(version)
    validations[path.name] = {"result": validation(path), "tests": parse_phase(path)}
for version, expected in {1694: (1074, 1, 0), 1699: (5858, 3, 33),
                          1700: (1059, 3, 0), 1701: (1074, 1, 0)}.items():
    tests = validations[phase(version).name]["tests"]
    assert tuple(tests[k] for k in ["top_level_passed", "child_process_passed", "top_level_ignored"]) == expected
    assert tests["top_level_failed"] == tests["child_process_failed"] == 0
warnings = lambda p: sorted(line for line in (p / "stderr.log").read_text().splitlines()
                            if line.startswith("warning:"))
stack_warnings = warnings(phase(1699))
assert stack_warnings == warnings(phase(1627)) and len(stack_warnings) == 14
assert not warnings(phase(1695)) and not warnings(phase(1702))
assert parse_phase(phase(1699))["ignored_test_lines"] == parse_phase(phase(1627))["ignored_test_lines"]

caps = {row["phase"]: row for row in focused["tests"] + remaining["results"][:30]}
assert len(caps) == 39
for name, row in caps.items():
    path = E / name
    assert validation(path) == row["result"]
    cap = row["cap"]
    assert cap["exit_code"] == 0 and not cap["wall_timeout"]
    assert cap["address_space_bytes"] == 192 * 1024 * 1024
    assert cap["cpu_seconds_soft_hard"] == [2, 3] and cap["wall_seconds_limit"] == 4
    row["test_counts"] = direct_test_counts(path)
for subset, expected in [(focused["tests"], (9, 0)),
                         (remaining["results"][:11], (11, 0)),
                         (remaining["results"][11:30], (136, 1))]:
    assert tuple(sum(caps[row["phase"]]["test_counts"][k] for row in subset)
                 for k in ["top_level_passed", "child_process_passed"]) == expected

counts = [384, 384, 288, 144, 384, 415, 60, 78, 600, 384, 168, 456, 384]
summaries = {}
for version, count in zip(range(1741, 1754), counts):
    path = phase(version)
    summary = read(path / "summary.json")
    assert summary["source_hashes_sha256"] == SOURCE_HASH and summary["guards_unchanged"]
    if count == 415:
        assert summary["ordinary_cases"] == 400 and summary["typed_interruption_errors"] == 300
        assert len(summary["extremes"]) == 6 and len(summary["cache"]) == 9
        assert all(r["valid"] for r in summary["extremes"] + summary["cache"])
        assert all(r["exit_code"] == 0 for r in summary["commands"])
    else:
        assert summary["cases"] == count
    for key, value in summary.items():
        if key.startswith("invalid_") or key in {"incomplete_cases", "unexpected_result_shapes", "nonzero_exit_cases", "typed_errors"}:
            assert not value, (path.name, key, value)
    summaries[path.name] = {"cases": count, "summary_sha256": sha(path / "summary.json"),
                            "summary": compact_summary(summary)}
assert sum(counts) == 4129
assert comparison["cases"] == 1584 and not comparison["unreviewed"]
assert len(comparison["changes"]) == 16
assert all(c["classification"] == "work-accounting change" for c in comparison["changes"])
consumption = [c for c in comparison["changes"] if c["path"].endswith(".consumed")]
assert len(consumption) == 8 and all(c["after"] == c["before"] - 6 for c in consumption)
assert queries["public_queries"] == 1584 and queries["successful_queries_before_after"] == [415, 453]
assert len(queries["public_completion_gains"]) == 38
assert queries["public_completion_losses"] == queries["changed_previous_successful_samples"] == queries["lost_or_contradictory_definite_signs"] == 0
assert queries["cheap_private_successes_before_after"] == [4, 56]
assert queries["all_private_successes_allocate_zero_rust_bytes"]
assert queries["endpoint_invalid_and_upper_bound_controls_identical"] == 48
assert queries["fine_retries_identical"] == 36 and queries["zero_work_cached_retry_queries"] == 144
public_consumption = [r["changes"]["consumed"] for r in queries["changed_public_rows"] if "consumed" in r["changes"]]
assert len(public_consumption) == 404
assert sum(r["after"] < r["before"] for r in public_consumption) == 396
assert sum(r["after"] > r["before"] for r in public_consumption) == 8
assert costs["all_successful_cost_processes_including_standalone"] == 1602
assert costs["separate_cpu_preparation_processes"] == 1135
assert costs["original_fixed_cpu_run"]["failed_attempt"]["exit_code"] == -24
assert costs["initial_and_followup_acos_percent"][0] > 5 > costs["initial_and_followup_acos_percent"][1]
assert not costs["separate_larger_cap_diagnostic"]["passes_original_two_second_guard"]
assert len(proof["unchanged_proof_and_guard_inputs_sha256"]) == 44
assert not proof["fresh_verus_run"] and proof["open_whole_crate_obligation_groups"] == 13
source = E.parents[2] / "target/scalar-evaluation-candidate/source"
for name, digest in proof["unchanged_proof_and_guard_inputs_sha256"].items():
    assert sha(source / name) == digest
continuation = read(E / "domain-bound-continuation-v1753/results.json")
assert continuation["complete"]
assert [r["label"] for r in continuation["steps"] if r["exit_code"] != 0] == ["paired-costs"]

record = {
    "status": "Retain the isolated cached magnitude-domain completeness extension; main runtime admission and the original cost guard remain open",
    "source_record": "typed-api-prototype-v1696-domain-bound-release-build/source-hashes.json.gz",
    "source_record_sha256": SOURCE_HASH,
    "baseline": "domain-bound-baseline-v1683.json",
    "implementation": "domain-bound-focused-implementation-v1698.json",
    "fresh_cargo_and_format_validations": validations, "fresh_capped_validations": caps,
    "capped_test_counts": {"new_focused": 9, "retained_domain_sign": 11,
        "other_retained": 136, "retained_child": 1, "total_top_level": 156},
    "replays": summaries, "replay_cases": 4129,
    "focused_replay_comparison": {"record": "domain-bound-budget-replay-comparison-v1745.json",
        "cases": 1584, "all_non_accounting_observations_identical": True,
        "changed_accounting_fields": 16, "reduced_consumption_rows": 8, "increased_consumption_rows": 0},
    "public_domain_comparison": {"record": "domain-bound-query-comparison-v1693.json",
        "cases": 1584, "successful_queries_before_after": [415, 453],
        "new_completions": 38, "lost_completions_or_definite_signs": 0,
        "reduced_consumption_rows": 396, "increased_consumption_rows": 8},
    "private_domain_completions_before_after": [4, 56],
    "stack_warning_comparison": {"baseline_phase": phase(1627).name, "unchanged_warning_lines": stack_warnings},
    "cost_record": "domain-bound-cost-review-v1710.json",
    "initial_and_followup_acos_percent": costs["initial_and_followup_acos_percent"],
    "decision": "After independent existence checks, retained upper bounds prove strict unit-range membership and retained positive exact binades prove x >= 1 before coefficient recognition. Keep the measured completeness gain: 52 previously exhausting private proof queries and 38 additional public query schedules complete at unchanged allowances. All prior successful samples and definite signs remain. This implements the accepted strengthening-only cache rule; main admission stays separate.",
    "limits": [
        "The new nine test bodies were frozen against the retained v1672 runtime before implementation. The three expected baseline proof-order failures remain archived. Private raw-wrapper diagnostics and their internal lower-bound existence preparation are not public API coverage.",
        "The public domain survey explicitly prewarms pi; other global constants evolve in its frozen process order. Its 1584 queries, 36 fine retries and 144 zero-work cached queries use independent directed numerical enclosures, without a globally cold-process claim.",
        "The original fixed-CPU comparison stopped on the retained v1672 baseline SIGXCPU after 864 successful processes. The interrupted stage is unknown. The equal larger-cap algebraic diagnostic does not pass or replace that failure; the earlier v1638 baseline failure also remains preserved.",
        "Cost evidence comprises 1602 successful processes, one failed baseline process and 1135 separate CPU preparations. The sole initial above-five-percent increase changes from 8.37 to 2.45 percent in the predeclared 27-pair repeat; keep both observations. No uniform speedup is claimed, and the older v1672 integer tradeoff remains retained.",
        "Public domain-query consumption decreases in 396 rows and increases in eight; 434 allocation observations change as more evaluation becomes reachable. The zero-allocation claim applies only to the 56 focused private proof queries. EvaluationBudget excludes bytes, time and unmetered algebraic helpers.",
        "The 78 private-kernel replay cases check result shapes. The 4129-case total is not a count of independent numerical oracle checks. The separate 1584-case retained replay comparison changes only sixteen accounting fields, all eight consumption rows decreasing by six.",
        "Full-stack tests retain 33 ignored tests and fourteen unchanged Cargo example-filename warnings. Both strict Clippy configurations pass without warnings. The checks cover the recorded five-crate stack, not every CI job or full-board admission.",
        "The prior log-range warmup comment is corrected without changing any setup, expression, order or assertion. All previously archived source snapshots remain immutable.",
        "All 44 prior conditional proof and guard inputs remain unchanged. No fresh Verus run is claimed. The new cached-bound helper and runtime domain traversal are outside that proof target; all thirteen whole-crate obligation groups remain open.",
        "Main retains its original scalar failure and all 49 preexisting worktree entries. Native combination, whole-board admission, all 39 properties, 14 targets and crate future work remain open. No push occurred.",
    ],
    "remaining_source_review": "domain-bound-deferred-review-v1702.json",
    "next_priority": "Public concurrent and reconstructed high-precision proof reuse with independent enclosures, spent shared allowances and retained facts; distinguish thread stress from exhaustive interleavings.",
}
out = E / "domain-bound-retention-v1754.json"
assert not out.exists()
out.write_text(json.dumps(record, indent=2) + "\n")
print(json.dumps({"status": record["status"], "cargo_and_format_phases": len(validations),
                  "capped_phases": len(caps), "capped_tests": 156,
                  "replay_cases": sum(counts), "public_completion_gains": 38}))
