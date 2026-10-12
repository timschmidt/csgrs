"""Audit the original cost controls and the predeclared family-wide repeats."""
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import statistics

E = Path(__file__).resolve().parent
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
nontiming = lambda row: {key: value for key, value in row.items() if key != "elapsed_ns"}


def main():
    out = E / "integer-coarse-zero-cost-review-v2100.json"
    assert not out.exists()
    original_path = E / "integer-coarse-zero-initial-cost-review-v2100.json"
    original = read(original_path)
    assert original["paired_complete"] and original["successful_paired_processes"] == 936
    assert original["standalone_successful_processes"] == 468 and original["complete_fixtures"] == 52
    assert not original["partial_fixtures"] and original["failed_attempt"] is None
    initial_phase = E / original["paired_phase"]
    assert all(sha(initial_phase / name) == digest for name, digest in original["original_run_artifacts_sha256"].items())
    protocol_path = E / "integer-coarse-zero-cost-followup-protocol-v2100.json"
    protocol = read(protocol_path)
    strategy_path = E / protocol["followup_strategy"]
    strategy = read(strategy_path)
    assert protocol["followup_strategy_sha256"] == sha(strategy_path)
    assert protocol["original_record_sha256"] == sha(original_path)
    assert protocol["source_record_sha256"] == original["source_record_sha256"] == strategy["source_record_sha256"]
    assert original["protocol_sha256"] == strategy["cost_protocol_sha256"]
    initial_increases = [row for row in original["cases"] if row["median_percent_change"] > 5]
    assert initial_increases == protocol["initial_above_five_percent_increases"] == original["above_five_percent_increases"]
    affected = {row["family"] for row in initial_increases}
    expected_selection = {family: rows for family, rows in strategy["family_fixtures"].items() if family in affected}
    assert len(protocol["jobs"]) == (1 if affected else 0)
    followups, repeated = [], {}
    process_count = 0
    for config in protocol["jobs"]:
        assert config["selection"] == expected_selection
        assert config["repetitions"] == 27 and config["cpu_seconds"] == [2, 3] and config["wall_seconds"] == 4
        phase = E / config["output"]
        assert not (phase / "failed-attempt.json").exists()
        data = read(phase / "summary.json")
        assert data["configuration"] == config and data["protocol_sha256"] == sha(protocol_path)
        assert data["source_record_sha256"] == original["source_record_sha256"] and data["guards_unchanged"]
        count = 2 * 27 * sum(map(len, expected_selection.values()))
        assert data["processes"] == data["cpu_preparation"]["prelude_processes"] == count
        assert all(sha(phase / name) == digest for name, digest in data["artifacts_sha256"].items())
        preparation = read(E / "integer-coarse-zero-cost-followup-preparation-v2100.json")
        assert sha(phase / "reproduce.py") == sha(E / preparation["runner"]) == preparation["runner_sha256"]
        assert set(data["provenance"]) == set(data["cases"]) == affected
        expected = {}
        for family, provenance in data["provenance"].items():
            retained = original["families"][family]
            assert provenance["phases"] == retained["phases"]
            assert provenance["binary_sha256_at_paired_run"] == retained["binary_sha256"]
            assert all(sha(Path(path)) == retained["binary_sha256"][which] for which, path in provenance["binaries"].items())
            reference = E / retained["phases"]["candidate"]
            for command in read(reference / "commands.json"):
                if command["name"] not in {"format", "build"}:
                    key = (family, tuple(command["command"][1:]))
                    expected[key] = nontiming(read(reference / (command["name"] + ".stdout")))
        rows, preludes = read(phase / "receipts.json"), read(phase / "prelude-receipts.json")
        assert len(rows) == len(preludes) == count
        assert all(row["exit_code"] == 0 and row["stderr"] == "" for row in preludes)
        assert len({row["stdout"] for row in preludes}) == 1
        samples = defaultdict(lambda: {"baseline": [], "candidate": []})
        for row in rows:
            family, index, which = row["name"].rsplit("-", 2)
            assert row["exit_code"] == 0 and row["binary_sha256"] == original["families"][family]["binary_sha256"][which]
            assert row["address_space_bytes"] == 192 * 1024 * 1024
            assert row["cpu_seconds_soft_hard"] == [2, 3] and row["wall_seconds_limit"] == 4
            key = (family, tuple(row["command"][1:]))
            assert nontiming(row["receipt"]) == expected[key]
            assert read(phase / (row["name"] + ".stdout")) == row["receipt"]
            assert (phase / (row["name"] + ".stderr")).read_text() == ""
            samples[key][which].append(int(row["receipt"]["elapsed_ns"]))
        required_keys = {(family, tuple(args)) for family, args_list in expected_selection.items() for args in args_list}
        assert set(samples) == required_keys
        for family, cases in data["cases"].items():
            assert {tuple(row["arguments"]) for row in cases} == {tuple(args) for args in expected_selection[family]}
            for row in cases:
                key = (family, tuple(row["arguments"]))
                assert row["elapsed_ns"] == samples[key]
                assert all(len(values) == 27 for values in samples[key].values())
                medians = {which: statistics.median(values) for which, values in samples[key].items()}
                assert row["median_ns"] == medians
                assert row["median_percent_change"] == 100 * (medians["candidate"] / medians["baseline"] - 1)
                repeated[key] = {"family": family, **row}
        followups.append({"phase": phase.name, "protocol": protocol_path.name,
            "protocol_sha256": sha(protocol_path), "summary_sha256": sha(phase / "summary.json"),
            "successful_processes": count, "cases": data["cases"], "failed_attempts": 0})
        process_count += count
    persistent = [row for row in repeated.values() if row["median_percent_change"] > 5]
    passed = not persistent
    comparisons = [{"family": row["family"], "arguments": row["arguments"],
        "initial_percent": row["median_percent_change"],
        "followup_percent": repeated[(row["family"], tuple(row["arguments"]))]["median_percent_change"]}
        for row in initial_increases]
    sizes = {family: {"bytes": item["binary_bytes"],
        "delta_bytes": item["binary_bytes"]["candidate"] - item["binary_bytes"]["baseline"],
        "percent_change": 100 * (item["binary_bytes"]["candidate"] / item["binary_bytes"]["baseline"] - 1)}
        for family, item in original["families"].items()}
    result = {
        "status": "Original-cap local cost controls and prescribed repeats pass" if passed else "Persistent timing increases require review; do not retain the combined proposal",
        "local_timing_and_process_gates_passed": passed,
        "source_record_sha256": original["source_record_sha256"],
        "retained_runtime_checkpoint": original["retained_runtime_checkpoint"],
        "initial_record": original_path.name, "initial_record_sha256": sha(original_path),
        "families": original["families"], "executable_size_comparison": sizes,
        "original_run": {"phase": initial_phase.name, "successful_processes": 936,
            "complete_fixtures": 52, "cases": original["cases"], "failed_attempts": 0},
        "followups": followups, "followup_strategy": strategy_path.name,
        "followup_strategy_sha256": sha(strategy_path),
        "initial_and_followup_percent": comparisons,
        "persistent_above_five_percent_increases": persistent,
        "successful_cost_processes_including_standalone": 1404 + process_count,
        "separate_cpu_preparation_processes": 936 + process_count,
        "original_process_caps_preserved": True,
        "interpretation": [
            "Retain every original observation and every predeclared family-wide repeat. The repeat covers all unchanged controls in affected families; timing alone establishes neither causation nor a uniform speedup.",
            "Every completed process preserves all frozen non-timing observations, including certification counts, checksums and output fields. CPU preludes are separate processes and all original caps remain in force.",
            "Probe executable sizes are recorded separately for review; these local measurements do not establish whole-board size or performance parity.",
            "The local baseline is retained v1836. Older original-cap failures at v1638/v1710, the earlier integer-control tradeoff and the fixed pre-Verus acceptance baseline remain unchanged.",
            "The combined proposal still awaits capped, numerical and public replays and receipt/fact audits. General graph/atomic proofs, all thirteen whole-crate groups, main/native/full-board admission and the full properties plan remain open.",
        ],
    }
    out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"local_timing_and_process_gates_passed": passed,
        "successful_cost_processes": result["successful_cost_processes_including_standalone"],
        "cpu_preludes": result["separate_cpu_preparation_processes"],
        "initial_and_followup_percent": comparisons,
        "persistent_increases": [{key: row[key] for key in ["family", "arguments", "median_percent_change"]} for row in persistent],
        "executable_size_changes": sizes}))
    assert passed, "Persistent timing increases prevent cost-gate acceptance"


if __name__ == "__main__":
    main()
