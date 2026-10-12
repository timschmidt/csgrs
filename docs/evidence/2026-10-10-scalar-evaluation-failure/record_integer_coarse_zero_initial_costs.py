"""Retain the complete or interrupted original-cap cost comparison honestly."""
import gzip
import hashlib
import json
from pathlib import Path
import statistics

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
nontiming = lambda value: {key: item for key, item in value.items() if key != "elapsed_ns"}


def main():
    out = E / "integer-coarse-zero-initial-cost-review-v2100.json"
    assert not out.exists()
    protocol_path = E / "integer-coarse-zero-cost-protocol-v2093.json"
    protocol = read(protocol_path)
    source_record = E / protocol["source_record"]
    expected_source = protocol["source_record_sha256"]
    assert sha(source_record) == expected_source
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert all(sha(SOURCE / name) == digest for name, digest in sources.items())
    assert all(sha(ROOT / name) == digest for name, digest in guard.items())
    standalone = read(E / "integer-coarse-zero-cost-orchestration-v2099.json")
    assert standalone["completed"] and standalone["source_record_sha256"] == expected_source
    checkpoint = read(E / protocol["retained_runtime_checkpoint"])["artifacts_sha256"]
    families, observations = {}, {}
    for item in standalone["results"]:
        family = item["family"]
        frozen = protocol["families"][family]
        old, new = E / frozen["baseline_phase"], E / item["phase"]
        assert new.name == frozen["candidate_phase"]
        assert sha(old / "summary.json") == checkpoint[old.name + "/summary.json"]
        assert sha(new / "summary.json") == item["summary_sha256"]
        assert sha(old / "probe.rs") == sha(new / "probe.rs") == frozen["probe_sha256"]
        assert sha(old / "reproduce.py") == sha(new / "reproduce.py") == frozen["runner_sha256"]
        commands = {which: [row for row in read(path / "commands.json") if row["name"] not in {"format", "build"}]
                    for which, path in [("baseline", old), ("candidate", new)]}
        assert len(commands["baseline"]) == len(commands["candidate"]) == item["processes"]
        binaries = {which: Path(rows[0]["command"][0]) for which, rows in commands.items()}
        hashes = {which: sha(path) for which, path in binaries.items()}
        assert hashes["baseline"] == frozen["binary_sha256"]
        summary = read(new / "summary.json")
        assert summary["source_hashes_sha256"] == expected_source and summary["guards_unchanged"]
        assert all(sha(Path(name)) == digest for name, digest in summary["libraries_sha256"].items())
        for a, b in zip(commands["baseline"], commands["candidate"]):
            assert a["name"] == b["name"] and a["command"][1:] == b["command"][1:]
            for command in [a, b]:
                assert command["exit_code"] == 0 and not command.get("wall_timeout", False)
                assert command["address_space_cap_mib"] == 192 and command["cpu_seconds"] == [2, 3]
                assert command["timeout_seconds"] == 4
            before, after = read(old / (a["name"] + ".stdout")), read(new / (b["name"] + ".stdout"))
            assert nontiming(before) == nontiming(after)
            key = (family, tuple(a["command"][1:]))
            assert key not in observations or observations[key] == nontiming(before)
            observations[key] = nontiming(before)
        families[family] = {
            "phases": {"baseline": old.name, "candidate": new.name}, "processes": item["processes"],
            "binaries": {which: str(path) for which, path in binaries.items()}, "binary_sha256": hashes,
            "binary_bytes": {which: path.stat().st_size for which, path in binaries.items()},
            "non_timing_outputs_identical": True,
        }
    assert sum(item["processes"] for item in families.values()) == 468 and len(observations) == 52
    phase = E / "integer-coarse-zero-preconditioned-cost-v2100"
    assert sha(phase / "reproduce.py") == protocol["runner_forks"]["replay_integer_coarse_zero_cost.py"]["fork_sha256"]
    complete = (phase / "summary.json").exists()
    failure = read(phase / "failed-attempt.json") if (phase / "failed-attempt.json").exists() else None
    assert complete != (failure is not None)
    rows = read(phase / "receipts.json")
    preludes = read(phase / "prelude-receipts.json")
    assert all(item["exit_code"] == 0 and item["stderr"] == "" for item in preludes)
    assert len({item["stdout"] for item in preludes}) == 1
    samples = {}
    for row in rows:
        family, index, which = row["name"].rsplit("-", 2)
        assert row["exit_code"] == 0 and row["binary_sha256"] == families[family]["binary_sha256"][which]
        assert row["address_space_bytes"] == 192 * 1024 * 1024
        assert row["cpu_seconds_soft_hard"] == [2, 3] and row["wall_seconds_limit"] == 4
        key = (family, tuple(row["command"][1:]))
        assert nontiming(row["receipt"]) == observations[key]
        assert read(phase / (row["name"] + ".stdout")) == row["receipt"]
        assert (phase / (row["name"] + ".stderr")).read_text() == ""
        samples.setdefault(key, {"baseline": [], "candidate": []})[which].append(int(row["receipt"]["elapsed_ns"]))
    if complete:
        summary = read(phase / "summary.json")
        assert summary["guards_unchanged"] and summary["source_record_sha256"] == expected_source
        assert summary["processes"] == len(rows) == len(preludes) == 936
        assert all(sha(phase / name) == digest for name, digest in summary["artifacts_sha256"].items())
    else:
        family, index, which = failure["name"].rsplit("-", 2)
        assert failure["binary_sha256"] == families[family]["binary_sha256"][which]
        assert failure["address_space_bytes"] == 192 * 1024 * 1024
        assert failure["cpu_seconds_soft_hard"] == [2, 3] and failure["wall_seconds_limit"] == 4
        assert failure["exit_code"] != 0 and len(preludes) == len(rows) + 1
    cases, partial = [], []
    for (family, arguments), values in samples.items():
        assert all(0 <= len(value) <= 9 for value in values.values())
        item = {"family": family, "arguments": list(arguments), "elapsed_ns": values}
        if all(len(value) == 9 for value in values.values()):
            medians = {which: statistics.median(value) for which, value in values.items()}
            item.update(median_ns=medians, median_percent_change=100 * (medians["candidate"] / medians["baseline"] - 1))
            cases.append(item)
        else:
            partial.append(item)
    if complete:
        assert len(cases) == 52 and not partial
    increases = [item for item in cases if item["median_percent_change"] > 5]
    result = {
        "status": "Original-cap comparison completed" if complete else "Original-cap comparison stopped; preserve the failure and every completed observation",
        "source_record_sha256": expected_source, "protocol_sha256": sha(protocol_path),
        "retained_runtime_checkpoint": protocol["retained_runtime_checkpoint"],
        "standalone_successful_processes": 468, "families": families,
        "paired_phase": phase.name, "paired_complete": complete,
        "successful_paired_processes": len(rows), "prelude_processes": len(preludes),
        "complete_fixtures": len(cases), "cases": cases, "partial_fixtures": partial,
        "failed_attempt": failure, "above_five_percent_increases": increases,
        "original_run_artifacts_sha256": {path.name: sha(path) for path in sorted(phase.iterdir()) if path.is_file()},
        "source_files_verified_after_run": len(sources), "main_guarded_files_verified_after_run": len(guard),
        "limits": [
            "All successful processes preserve every frozen non-timing output, including certification counts, checksums and output fields. Only elapsed_ns is excluded from equality.",
            "Each complete fixture contains nine alternating-order pairs. Any increase above five percent requires the frozen follow-up rule; partial fixtures remain partial.",
            "An original process failure is not passed by a separate larger-cap diagnostic. A missing final receipt does not identify the interrupted internal stage.",
            "The combined integer/kernel/selector proposal provides no new completeness gain to justify a runtime regression. The fixed pre-Verus acceptance baseline, earlier cost failures/tradeoff and main/full-board admission remain separate open obligations.",
        ],
    }
    out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"paired_complete": complete, "successful_paired_processes": len(rows),
                      "complete_fixtures": len(cases), "partial_fixtures": len(partial),
                      "above_five_percent_increases": [{k: row[k] for k in ["family", "arguments", "median_percent_change"]} for row in increases],
                      "failed_attempt": failure}))


if __name__ == "__main__":
    main()
