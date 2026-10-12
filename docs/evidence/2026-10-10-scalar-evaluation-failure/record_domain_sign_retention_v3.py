"""Check completed evidence and retain the isolated domain-sign extension."""
import hashlib
import json
from pathlib import Path
import re
import sys

sys.dont_write_bytecode = True
from parse_test_results_v3 import parse_phase

E = Path(__file__).resolve().parent
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
SOURCE_HASH = "f8dbb363116f762ef866231905590de0b21a020a5eedbe3f2d7fcc8d8c8ccd0d"


def phase(version):
    paths = [p for p in E.glob(f"typed-api-prototype-v{version}-*") if p.is_dir()]
    assert len(paths) == 1 and paths[0].is_dir(), (version, paths)
    return paths[0]


def validation(path):
    result = read(path / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    assert sha(path / "source-hashes.json.gz") == SOURCE_HASH
    return result


def direct_test_counts(path):
    # The Cargo parser requires Cargo's stderr target headers. These capped
    # commands invoke the frozen test binary directly, so count their nested
    # libtest batches without inventing a Cargo target header or editing logs.
    text = (path / "stdout.log").read_text()
    pattern = r"test result:\s*(\w+)\. (\d+) passed; (\d+) failed; (\d+) ignored; [^\n]*"
    text = re.sub(pattern, lambda m: "\n" + re.sub(r"result:\s*", "result: ", m[0]) + "\n", text)
    stack, top, child = [], [], []
    for line in text.splitlines():
        start = re.fullmatch(r"running (\d+) tests?", line)
        if start:
            stack.append(int(start[1]))
        end = re.fullmatch(pattern, line)
        if end:
            assert stack and end[1] == "ok" and int(end[3]) == int(end[4]) == 0
            count = stack.pop()
            assert count == int(end[2])
            (child if stack else top).append(count)
    assert not stack and len(top) == 1 and top[0] > 0
    return {"top_level_passed": sum(top), "child_process_passed": sum(child)}


def compact_summary(summary):
    return {key: value for key, value in summary.items()
            if isinstance(value, (str, int, float, bool, type(None)))
            and key not in {"build_record", "source_record"}}


def write_new(name, data):
    path = E / name
    assert not path.exists(), path
    path.write_text(json.dumps(data, indent=2) + "\n")


def main():
    completed = read(E / "domain-sign-candidate-validation-v1630.json")
    remaining = read(E / "domain-sign-remaining-validation-v1671.json")
    focused = read(E / "domain-sign-retaining-caps-v1621.json")
    replay = read(E / "domain-sign-replay-orchestration-v1644.json")
    comparison = read(E / "domain-sign-budget-replay-comparison-v1644.json")
    costs = read(E / "domain-sign-cost-review-v1639.json")
    proof = read(E / "domain-sign-proof-scope-v1630.json")
    for record in [completed, remaining, focused, replay, comparison, costs, proof]:
        assert record["source_record_sha256"] == SOURCE_HASH
    assert remaining["complete"] and replay["complete"]
    assert len(completed["phases"]) == 9 and len(remaining["results"]) == 27

    validations = {}
    for version in [1610, *range(1622, 1631)]:
        path = phase(version)
        validations[path.name] = {"result": validation(path), "tests": parse_phase(path)}
    expected_counts = {1622: (1065, 1, 0), 1627: (5849, 3, 33),
                       1628: (1059, 3, 0), 1629: (1065, 1, 0)}
    for version, expected in expected_counts.items():
        tests = validations[phase(version).name]["tests"]
        assert tuple(tests[k] for k in ["top_level_passed", "child_process_passed", "top_level_ignored"]) == expected
        assert tests["top_level_failed"] == tests["child_process_failed"] == 0

    warnings = lambda p: sorted(line for line in (p / "stderr.log").read_text().splitlines()
                                if line.startswith("warning:"))
    stack_warnings = warnings(phase(1627))
    assert stack_warnings == warnings(phase(1372)) and len(stack_warnings) == 14
    assert not warnings(phase(1623)) and not warnings(phase(1630))

    cap_rows = dict(focused["tests"])
    cap_rows.update({row["phase"]: row for row in remaining["results"][:19]})
    assert len(cap_rows) == 30
    for name, row in cap_rows.items():
        path = E / name
        assert validation(path) == row["result"]
        cap = row["cap"]
        assert cap["exit_code"] == 0 and not cap["wall_timeout"]
        assert cap["address_space_bytes"] == 192 * 1024 * 1024
        assert cap["cpu_seconds_soft_hard"] == [2, 3] and cap["wall_seconds_limit"] == 4
        row["test_counts"] = direct_test_counts(path)
    for subset, expected in [(list(focused["tests"]), (11, 0)),
                             ([r["phase"] for r in remaining["results"][:19]], (136, 1))]:
        assert tuple(sum(cap_rows[n]["test_counts"][k] for n in subset)
                     for k in ["top_level_passed", "child_process_passed"]) == expected

    summaries = {}
    counts = [384, 384, 288, 144, 384, 415, 60, 78, 600, 384, 168, 456, 384]
    replay_phases = [phase(v) for v in [*range(1640, 1645), *range(1664, 1672)]]
    for path, count in zip(replay_phases, counts):
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
    assert len(comparison["changes"]) == 926
    assert all(c["classification"] == "work-accounting change" for c in comparison["changes"])
    consumption = [c for c in comparison["changes"] if c["path"].endswith(".consumed")]
    assert len(consumption) == 463
    charge_reductions = sum(c["after"] < c["before"] for c in consumption)
    charge_increases = sum(c["after"] > c["before"] for c in consumption)
    assert (charge_reductions, charge_increases) == (459, 4)

    assert costs["all_successful_cost_processes_including_standalone"] == 1767
    assert costs["separate_cpu_preparation_processes"] == 1300
    assert costs["original_fixed_cpu_run"]["failed_attempt"]["exit_code"] == -24
    assert costs["persistent_integer_cost"]["median_percent_change"] > 18
    proof_inputs = proof["unchanged_proof_and_guard_inputs_sha256"]
    assert len(proof_inputs) == 44
    source = E.parents[2] / "target/scalar-evaluation-candidate/source"
    for name, digest in proof_inputs.items():
        assert sha(source / name) == digest

    record = {
        "status": "Retain the corrected isolated domain-sign completeness extension with the measured performance tradeoff; main runtime admission remains open",
        "source_record": "typed-api-prototype-v1624-domain-sign-release-build/source-hashes.json.gz",
        "source_record_sha256": SOURCE_HASH,
        "baseline": "domain-sign-baseline-v1593.json",
        "implementation": "domain-sign-focused-implementation-v1626.json",
        "rejected_first_variant": "domain-sign-publication-correction-v1621.json",
        "fresh_cargo_and_format_validations": validations,
        "fresh_capped_validations": cap_rows,
        "capped_test_counts": {"focused": 11, "retained": 136, "retained_child": 1},
        "replays": summaries,
        "replay_cases": 4129,
        "focused_replay_comparison": {
            "record": "domain-sign-budget-replay-comparison-v1644.json", "cases": 1584,
            "all_non_accounting_observations_identical": True, "changed_accounting_fields": 926,
            "reduced_consumption_rows": charge_reductions, "increased_consumption_rows": charge_increases},
        "stack_warning_comparison": {"baseline_phase": phase(1372).name, "unchanged_warning_lines": stack_warnings},
        "cost_record": "domain-sign-cost-review-v1639.json",
        "cost_tradeoff": costs["persistent_integer_cost"],
        "decision": "Canonical literal and retained sign proofs avoid coefficient recognition after independent existence has been established. Eight public queries change from Exhausted to valid samples at the same eight-unit allowance, and earlier zero-work child sign proofs remain available. Under the governing exactness > completeness > performance ladder, keep this isolated completeness gain while reporting the persistent 18.76 percent narrow integer-control slowdown. No uniform speedup or main-runtime admission is claimed.",
        "limits": [
            "The first candidate lost child proof publication and is rejected. Its failure, the original domain exhaustion failures and the corrected 12-case zero-work proof diagnostic remain archived.",
            "All earlier scalar test bodies and full-size fixtures remain unchanged. The eleven added test functions cover 88 domain observations, 241 log survey observations, thirteen unlimited retries and 52 zero-work cached queries.",
            "The original fixed-CPU cost run stopped on a retained-baseline SIGXCPU after 867 successful processes. Its failure remains unresolved under the original cap; the 108-process equal larger-cap algebraic diagnostic does not replace it. The stopped internal stage is unknown.",
            "Cost evidence comprises 1767 successful processes, one failed baseline process and 1300 separate CPU preparation processes. Timing is local evidence, not a universal performance claim.",
            "The 78 private-kernel cases check result shapes; the 4129-case total is not a count of independent numerical oracle checks. The 1584 focused receipt comparison changes only accounting, including four increased consumption rows.",
            "Full-stack tests retain 33 ignored tests and fourteen inherited Cargo example-filename collision warnings. Both strict Clippy configurations passed without warnings.",
            "The log survey shared-constant warmup comment is misleading; the setup calls the direct rational ln1p path. Global constants evolve in the frozen process order. Correct this comment before runtime admission, preserving the archived original evidence.",
            "EvaluationBudget excludes aggregate byte/time bounds and unmetered algebraic helpers. The million-bit public ln1p query still requests 875240 Rust allocation bytes. No byte/time contract violation is claimed.",
            "All 44 prior conditional proof and guard inputs remain identical. No fresh Verus run was needed for that unchanged target, and it does not prove the modified runtime domain traversal. All thirteen whole-crate obligation groups remain open.",
            "Public Real log routes extract a Computable payload through public serialization; no native Real budget API is claimed. Private domain diagnostics are identified separately from public query cases.",
            "Main retains its original scalar failure and all 49 preexisting worktree entries. Native combination, whole-board admission, all 39 properties, 14 targets and crate future work remain open. No push occurred."
        ],
        "remaining_source_review": "domain-sign-deferred-review-v1630.json",
    }
    write_new("domain-sign-retention-v1672.json", record)
    print(json.dumps({"status": record["status"], "cargo_and_format_phases": len(validations),
                      "capped_phases": len(cap_rows), "replay_cases": sum(counts)}))


if __name__ == "__main__":
    main()
