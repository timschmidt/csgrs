"""Freeze and review a scoped unchanged-computation cost method, including failures."""
import gzip
import hashlib
import json
from pathlib import Path
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
PROTOCOL = E / "eager-bound-decoder-computation-protocol-v2279.json"
REVIEW = E / "eager-bound-decoder-computation-review-v2279.json"
INPUTS = [
    "eager-bound-decoder-load-image-review-v2263.json",
    "eager-bound-decoder-location-uses-v2277/summary.json",
    "eager-bound-decoder-binary-comparison-v2262/summary.json",
    "eager-bound-decoder-early-milestone-v2265.json",
    "domain-predicate-cost-review-v1780.json",
    "integer-coarse-zero-cost-rejection-v2100.json",
    "bound-decoder-cost-rejection-v2247.json",
]


def prepare():
    assert not PROTOCOL.exists() and not REVIEW.exists()
    milestone = read(E / INPUTS[3])
    PROTOCOL.write_text(json.dumps({
        "status": "Prospective qualification method; no replay or new cost workload started",
        "predecessor_protocol": "eager-bound-decoder-computation-protocol-v2278.json",
        "predecessor_protocol_sha256": sha(E / "eager-bound-decoder-computation-protocol-v2278.json"),
        "reader_failure": "eager-bound-decoder-computation-reader-failure-v2278.json",
        "reader_failure_sha256": sha(E / "eager-bound-decoder-computation-reader-failure-v2278.json"),
        "schema_correction": "Use the preserved paired provenance binding each original summary hash to its executable hash; additionally check direct summary hashes where present. No fixture, cap, historical result, static condition or qualification criterion changes.",
        "source_record": milestone["source_record"],
        "source_record_sha256": milestone["source_record_sha256"],
        "runner_sha256": sha(Path(__file__)), "inputs_sha256": {n: sha(E / n) for n in INPUTS},
        "method": "unchanged-computation-with-reviewed-diagnostic-metadata",
        "reason": "The source change extends proof coverage and restores the retained instruction bytes in all seven original cost probes. Proportionate differential cost evidence can therefore be static, provided all remaining loaded differences are explicitly accounted for and diagnostic use is reviewed. This is a new method, not a pass under the earlier complete-file identity rule.",
        "required_static_evidence": [
            "All seven frozen probe hashes, complete executable sections and executable load segments match the retained computation; program headers and dynamic relocation payloads are identical.",
            "Loaded differences are restricted to section-table offset metadata, build-id and six source-line fields at unchanged statements. Full files may differ; no binary is rewritten for execution.",
            "All detected direct references to changed location records feed resolved panic routines. Preserve the reference-analysis limits and the possible diagnostic line-number change.",
            "Probe file sizes do not grow. The complete source change, proof scope and library-size change remain explicit; this does not prove code-generation identity for other compilers, targets or build options.",
        ],
        "historical_evidence_rule": "Verify and reference the retained candidate side of all seven v1780 standalone families and both paired campaigns. Keep the other side distinct. Historical timing, allocation and output receipts keep their original binary/source identities; they are supporting observations, never measurements of this candidate or universal timing guarantees.",
        "failure_rule": "Both later v1836 baseline SIGXCPU results at v2100 and v2247 remain failed under the original 192 MiB, CPU 2/3-second and four-second wall caps. Include their exact binary identities and commands. Earlier failures, unresolved changes for rejected variants and the fixed pre-Verus baseline remain unchanged. Do not rerun or loosen a failed cap to obtain approval.",
        "decision": "If static and historical integrity checks pass, permit fresh unchanged correctness/replay/fact gates under this scoped differential-cost method. Runtime retention additionally requires all nine broad commands, the full conditional proof, every mutation/acceptance guard, all 39 capped commands, 4129 numerical cases and 648 public schedules, with their original comparisons. The method makes no absolute resource-cap claim and does not repair baseline failures.",
        "original_full_file_identity_rule_satisfied": False,
        "planned_new_timing_processes": 0, "runtime_retained": False,
    }, indent=2) + "\n")
    print(json.dumps({"protocol": PROTOCOL.name, "new_timing_processes": 0}))


def review():
    assert not REVIEW.exists()
    protocol = read(PROTOCOL)
    assert protocol["runner_sha256"] == sha(Path(__file__))
    assert all(sha(E / n) == h for n, h in protocol["inputs_sha256"].items())
    assert sha(E / protocol["source_record"]) == protocol["source_record_sha256"]
    sources = json.loads(gzip.decompress((E / protocol["source_record"]).read_bytes()))
    guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    images, uses, binaries, milestone, historical, failure_a, failure_b = [read(E / n) for n in INPUTS]
    assert images["source_record_sha256"] == uses["source_record_sha256"] == binaries["source_record_sha256"] == protocol["source_record_sha256"]
    assert not binaries["all_probe_binaries_identical"] and not images["qualification"]["full_executable_identity"]
    assert milestone["verified_units"] == 655 and milestone["open_obligation_groups"] == 13
    assert len(images["families"]) == len(uses["families"]) == len(binaries["results"]) == 7
    old_hashes = {n: r["binary_sha256"]["candidate"] for n, r in historical["families"].items()}
    for row in images["families"]:
        assert row["binary_sha256"]["baseline"] == old_hashes[row["family"]]
        assert row["binary_size_delta"] <= 0
        for segment in row["load_segments"]:
            if segment["executable"]:
                assert segment["baseline_sha256"] == segment["candidate_sha256"]
        assert len(row["source_location_records"]) == 6
    for row in uses["families"]:
        assert row["direct_reference_count"] == len(row["direct_references"]) > 0
        assert not row["data_relocations_into_records"]
        assert sha(E / "eager-bound-decoder-location-uses-v2277" / row["stdout_archive"]) == row["stdout_archive_sha256"]
    for row in binaries["results"]:
        assert sha(Path(row["binary"])) == row["binary_sha256"]
        assert row["baseline_binary_sha256"] == old_hashes[row["family"]]
    verified_artifacts, standalone, paired = {}, [], []

    def artifacts(phase):
        summary = read(phase / "summary.json")
        for name, value in summary["artifacts_sha256"].items():
            assert sha(phase / name) == value, str(phase / name)
            verified_artifacts[str((phase / name).relative_to(E))] = value
        verified_artifacts[str((phase / "summary.json").relative_to(E))] = sha(phase / "summary.json")
        return summary

    for family, row in historical["families"].items():
        phase = E / row["phases"]["candidate"]
        summary = artifacts(phase)
        paired_summary = read(E / historical["original_run"]["phase"] / "summary.json")
        provenance = paired_summary["provenance"][family]
        assert provenance["original_summary_sha256"]["candidate"] == sha(phase / "summary.json")
        assert provenance["binary_sha256_at_paired_run"]["candidate"] == old_hashes[family]
        if "binary_sha256" in summary:
            assert summary["binary_sha256"] == old_hashes[family]
        assert sha(Path(row["binaries"]["candidate"])) == old_hashes[family]
        commands = [c for c in read(phase / "commands.json") if c["name"] not in {"format", "build"}]
        assert len(commands) == len(read(phase / "receipts.json")) == summary["processes"]
        for command in commands:
            assert command["command"][0] == row["binaries"]["candidate"] and command["exit_code"] == 0
            assert command["address_space_cap_mib"] == 192 and command["cpu_seconds"] == [2, 3] and command["timeout_seconds"] == 4
            assert (phase / (command["name"] + ".stderr")).read_bytes() == b""
        standalone.append({"family": family, "phase": phase.name, "binary_sha256": old_hashes[family],
                           "historical_successful_processes": len(commands)})
    for part in ["original_run", "followup"]:
        phase = E / historical[part]["phase"]
        summary = artifacts(phase)
        receipts = read(phase / "receipts.json")
        selected = [r for r in receipts if r["name"].endswith("-candidate")]
        assert len(receipts) == summary["processes"] == 2 * len(selected)
        for receipt in selected:
            family = next(n for n in old_hashes if receipt["name"].startswith(n + "-"))
            assert receipt["binary_sha256"] == old_hashes[family] and receipt["exit_code"] == 0
            assert receipt["address_space_bytes"] == 192 * 1024 * 1024
            assert receipt["cpu_seconds_soft_hard"] == [2, 3] and receipt["wall_seconds_limit"] == 4
            assert (phase / (receipt["name"] + ".stderr")).read_bytes() == b""
        paired.append({"phase": phase.name, "historical_matching_processes": len(selected),
                       "other_version_processes_not_credited": len(receipts) - len(selected)})
    failures = []
    for name, record in zip(INPUTS[-2:], [failure_a, failure_b]):
        assert not record["cost_gate_passed"] and not record["runtime_retained"]
        assert sha(E / record["partial_cost_record"]) == record["partial_cost_record_sha256"]
        failed = record["failure"]
        assert failed["binary_sha256"] == old_hashes["algebraic"] and failed["exit_code"] == -24
        assert failed["command"][1:] == ["cost", "4", "4", "0", "false", "64"]
        assert failed["address_space_bytes"] == 192 * 1024 * 1024
        assert failed["cpu_seconds_soft_hard"] == [2, 3] and failed["wall_seconds_limit"] == 4
        failures.append({"record": name, "record_sha256": sha(E / name), "failure": failed,
                         "original_gate_still_failed": True})
    assert sum(r["historical_successful_processes"] for r in standalone) == 468
    assert sum(r["historical_matching_processes"] for r in paired) == 549
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    REVIEW.write_text(json.dumps({
        "status": "Scoped differential-cost evidence qualifies for fresh replays; historical cap failures remain unresolved",
        "protocol": PROTOCOL.name, "protocol_sha256": sha(PROTOCOL),
        "source_record_sha256": protocol["source_record_sha256"], "method": protocol["method"],
        "qualified_for_fresh_replays": True, "original_full_file_identity_rule_satisfied": False,
        "static_inputs_sha256": protocol["inputs_sha256"], "standalone": standalone, "paired": paired,
        "historical_matching_successful_processes": 1017, "other_version_processes_not_credited": 549,
        "historical_failures": failures, "historical_artifacts_sha256": verified_artifacts,
        "fresh_timing_processes": 0, "fresh_timing_gate_passed": False,
        "historical_receipts_relabelled_as_candidate_runs": 0,
        "all_historical_original_caps_pass": False, "runtime_retained": False,
        "interpretation": "Identical computation and reviewed diagnostic metadata support retaining the added conditional proof without evidence of added computational work in these seven builds, subject to all fresh correctness/replay/fact gates. The 1017 older observations remain historical v1836 executions. Both later failures of that same algebraic program are inherited limitations, not overridden by the earlier passes. No speedup, current timing pass, whole-program equivalence, absolute resource bound, other-target parity or main/native admission is claimed.",
        "source_files_verified": len(sources), "main_files_verified": len(guard),
    }, indent=2) + "\n")
    print(json.dumps({"qualified_for_fresh_replays": True, "historical_successes": 1017,
                      "later_baseline_failures_preserved": 2, "fresh_timing_processes": 0}))


if __name__ == "__main__":
    assert len(sys.argv) == 2 and sys.argv[1] in {"prepare", "review"}
    {"prepare": prepare, "review": review}[sys.argv[1]]()
