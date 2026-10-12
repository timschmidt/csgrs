"""Retain initial and prescribed follow-up costs without claiming runtime admission."""
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
nontiming = lambda value: {key: item for key, item in value.items() if key != "elapsed_ns"}


def main():
    out = E / "domain-predicate-cost-review-v1780.json"
    assert not out.exists()
    original_path = E / "domain-predicate-initial-cost-review-v1780.json"
    original = read(original_path)
    assert original["paired_complete"] and original["successful_paired_processes"] == 936
    assert original["standalone_successful_processes"] == 468 and original["complete_fixtures"] == 52
    assert not original["partial_fixtures"] and original["failed_attempt"] is None
    initial_phase = E / original["paired_phase"]
    assert all(sha(initial_phase / name) == digest for name, digest in original["original_run_artifacts_sha256"].items())
    protocol_path = E / "domain-predicate-cost-followup-protocol-v1780.json"
    protocol = read(protocol_path)
    assert protocol["original_record_sha256"] == sha(original_path)
    assert protocol["source_record_sha256"] == original["source_record_sha256"]
    assert len(protocol["jobs"]) == 1
    config = protocol["jobs"][0]
    assert config["repetitions"] == 27 and config["cpu_seconds"] == [2, 3] and config["wall_seconds"] == 4
    phase = E / config["output"]
    data = read(phase / "summary.json")
    assert data["configuration"] == config and data["protocol_sha256"] == sha(protocol_path)
    assert data["source_record_sha256"] == original["source_record_sha256"] and data["guards_unchanged"]
    assert data["processes"] == data["cpu_preparation"]["prelude_processes"] == 162
    assert all(sha(phase / name) == digest for name, digest in data["artifacts_sha256"].items())
    preparation = read(E / "domain-predicate-cost-followup-preparation-v1780.json")
    assert sha(phase / "reproduce.py") == sha(E / preparation["runner"]) == preparation["runner_sha256"]
    expected = {}
    for family, provenance in data["provenance"].items():
        retained = original["families"][family]
        assert provenance["phases"] == retained["phases"]
        assert provenance["binary_sha256_at_paired_run"] == retained["binary_sha256"]
        assert all(sha(Path(path)) == retained["binary_sha256"][which] for which, path in provenance["binaries"].items())
        reference = E / retained["phases"]["candidate"]
        for command in read(reference / "commands.json"):
            if command["name"] not in {"format", "build"}:
                expected[(family, tuple(command["command"][1:]))] = nontiming(read(reference / (command["name"] + ".stdout")))
    rows = read(phase / "receipts.json")
    preludes = read(phase / "prelude-receipts.json")
    assert len(rows) == len(preludes) == 162
    assert all(row["exit_code"] == 0 and row["stderr"] == "" for row in preludes)
    assert len({row["stdout"] for row in preludes}) == 1
    for row in rows:
        family, index, which = row["name"].rsplit("-", 2)
        assert row["exit_code"] == 0 and row["binary_sha256"] == original["families"][family]["binary_sha256"][which]
        assert row["address_space_bytes"] == 192 * 1024 * 1024
        assert row["cpu_seconds_soft_hard"] == [2, 3] and row["wall_seconds_limit"] == 4
        assert nontiming(row["receipt"]) == expected[(family, tuple(row["command"][1:]))]
        assert read(phase / (row["name"] + ".stdout")) == row["receipt"]
        assert (phase / (row["name"] + ".stderr")).read_text() == ""
    initial = original["above_five_percent_increases"]
    assert len(initial) == 1 and initial[0]["arguments"] == ["cost", "pi", "1024", "64"]
    current = data["cases"]["inverse_trig"]
    assert len(current) == 3 and all(len(value) == 27 for case in current for value in case["elapsed_ns"].values())
    assert all(case["median_percent_change"] <= 5 for case in current)
    pi = next(case for case in current if case["arguments"] == initial[0]["arguments"])
    sizes = {family: {"bytes": item["binary_bytes"],
                      "delta_bytes": item["binary_bytes"]["candidate"] - item["binary_bytes"]["baseline"],
                      "percent_change": 100 * (item["binary_bytes"]["candidate"] / item["binary_bytes"]["baseline"] - 1)}
             for family, item in original["families"].items()}
    result = {
        "status": "Original-cap local cost controls and the prescribed follow-up pass; capped/numerical/public replay gates remain pending",
        "source_record_sha256": original["source_record_sha256"],
        "retained_runtime_checkpoint": original["retained_runtime_checkpoint"],
        "initial_record": original_path.name, "initial_record_sha256": sha(original_path),
        "families": original["families"], "executable_size_comparison": sizes,
        "original_run": {"phase": initial_phase.name, "successful_processes": 936,
                         "complete_fixtures": 52, "cases": original["cases"], "failed_attempts": 0},
        "followup": {"phase": phase.name, "protocol": protocol_path.name,
                     "protocol_sha256": sha(protocol_path), "summary_sha256": sha(phase / "summary.json"),
                     "successful_processes": 162, "cases": data["cases"], "failed_attempts": 0},
        "initial_and_followup_pi_percent": [initial[0]["median_percent_change"], pi["median_percent_change"]],
        "successful_cost_processes_including_standalone": 1566,
        "separate_cpu_preparation_processes": 1098,
        "original_process_caps_preserved": True,
        "interpretation": [
            "The one initial increase above five percent is the 1024-bit pi control. Its 5.0501 percent median increase is 1.0191 percent in the frozen 27-pair follow-up; same-width atan and acos controls are 0.4078 and 0.5842 percent. Both observations remain retained, with no causal attribution or uniform-speedup claim.",
            "All 1566 fresh cost processes pass the original 192 MiB, CPU 2/3-second and four-second wall limits. All successful observations preserve non-timing samples, allocations, work counts and checksums. The 1098 CPU preludes are separate processes.",
            "Executable sizes are recorded for each frozen probe. These are local controls and include executable metadata; they do not establish whole-board size or performance parity.",
            "The baseline is the retained v1754 runtime. This does not erase older original-cap failures at v1638/v1710 or the earlier integer-control tradeoff, nor reset the fixed pre-Verus acceptance baseline.",
            "The extraction is not yet retained or admitted to main. Complete all frozen capped/numerical/public replays and fact audits; general graph/atomic proofs, all thirteen whole-crate groups and full-board admission remain separate open obligations.",
        ],
    }
    out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"successful_cost_processes": 1566, "cpu_prelude_processes": 1098,
                      "pi_original_followup_percent": result["initial_and_followup_pi_percent"],
                      "executable_size_changes": sizes}))


if __name__ == "__main__":
    main()
