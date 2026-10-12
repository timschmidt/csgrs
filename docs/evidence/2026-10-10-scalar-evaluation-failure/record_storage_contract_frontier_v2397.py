"""Record actual storage-contract limits and the guarded executable alternative."""
import gzip
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    out = E / "storage-contract-frontier-v2397.json"
    assert not out.exists()
    checkpoint_path = E / "typed-api-prototype-v2386-state.json"
    assert sha(checkpoint_path) == "6dabcb25b03a7a8769c432b926b3c4a35288ed43fd172e7a46d27503d8f68cb9"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    names = [
        ("storage-forward-probe-v2387", 1, "only variables are supported here, not general patterns"),
        ("storage-reverse-probe-v2388", 1, "only variables are supported here, not general patterns"),
        ("storage-shrink-probe-v2389", 1, "disambiguate by wrapping it in either `old` or `final`"),
        ("storage-truncate-probe-v2390", 1, "disambiguate by wrapping it in either `old` or `final`"),
        ("storage-forward-probe-v2391", 1, "::position` is not supported"),
        ("storage-reverse-probe-v2392", 1, "::rposition` is not supported"),
        ("storage-shrink-probe-v2393", 1, "::shrink_to_fit` is not supported"),
        ("storage-truncate-probe-v2394", 0, ""),
        ("borrowed-metadata-probe-v2395", 0, ""),
    ]
    phases = {}
    for name, expected_exit, diagnostic in names:
        path = E / name
        result = read(path / "result.json")
        protocol = read(path / "protocol.json")
        verification = read(path / "stdout.json")["verification-results"]
        stderr = (path / "stderr.log").read_text()
        assert result["exit_code"] == expected_exit and result["source_unchanged"] and result["main_unchanged"]
        assert result["tooling_unchanged"] and not result["retained_proof_coverage_changed"]
        assert protocol["retained_checkpoint_sha256"] == sha(checkpoint_path)
        assert protocol["source_record_sha256"] == checkpoint["source_record_sha256"]
        assert protocol["probe_sha256"] == sha(path / "probe.rs")
        assert diagnostic in stderr
        for key, filename in [("protocol_sha256", "protocol.json"), ("stdout_sha256", "stdout.json"), ("stderr_sha256", "stderr.log")]:
            assert result[key] == sha(path / filename)
        if expected_exit:
            assert not verification["success"] and verification.get("verified", 0) == 0
        else:
            assert not stderr and verification["success"] and verification["errors"] == 0
        phases[name] = {"result_sha256": sha(path / "result.json"), "probe_sha256": sha(path / "probe.rs"),
                        "exit_code": result["exit_code"], "verification_results": verification,
                        "diagnostic": diagnostic, "seconds": result["seconds"]}
    assert "::capacity` is not supported" in (E / "storage-shrink-probe-v2393/stderr.log").read_text()
    assert phases["storage-truncate-probe-v2394"]["verification_results"]["verified"] == 440
    assert phases["borrowed-metadata-probe-v2395"]["verification_results"]["verified"] == 454
    controls_path = E / "borrowed-metadata-guards-v2396"
    controls = read(controls_path / "result.json")
    controls_protocol = read(controls_path / "protocol.json")
    assert controls["complete"] and len(controls["runs"]) == len(controls_protocol["runs"]) == 13
    assert controls["source_unchanged"] and controls["main_unchanged"] and controls["tooling_unchanged"]
    assert not controls["retained_proof_coverage_changed"]
    assert controls["protocol_sha256"] == sha(controls_path / "protocol.json")
    assert controls_protocol["retained_checkpoint_sha256"] == sha(checkpoint_path)
    assert controls_protocol["source_record_sha256"] == checkpoint["source_record_sha256"]
    original = (E / "borrowed-metadata-proposal-v2395.rs").read_text()
    assert controls_protocol["proposal_sha256"] == sha(E / "borrowed-metadata-proposal-v2395.rs")
    for protocol, result in zip(controls_protocol["runs"], controls["runs"]):
        directory = controls_path / protocol["directory"]
        assert result == read(directory / "result.json") and result["gate_passed"]
        assert result["source_sha256"] == protocol["source_sha256"] == sha(directory / "probe.rs")
        assert result["stdout_sha256"] == sha(directory / "stdout.json")
        assert result["stderr_sha256"] == sha(directory / "stderr.log")
        text = original
        if protocol["mutation"]:
            name, before, after = protocol["mutation"]
            start = text.index("fn " + name + "(")
            end = text.index("\n}\n", start) + 3
            assert text[start:end].count(before) == 1
            text = text[:start] + text[start:end].replace(before, after) + text[end:]
            assert result["exit_code"] != 0 and result["verification_results"]["errors"] > 0
        else:
            assert result["exit_code"] == 0 and result["verification_results"]["verified"] == 454
        assert hashlib.sha256(text.encode()).hexdigest() == protocol["source_sha256"]
    dep_review_path = E / "numerical-bound-metadata-review-v2279-v2.json"
    assert sha(dep_review_path) == checkpoint["artifacts_sha256"][dep_review_path.name]
    dep_review = read(dep_review_path)
    assert dep_review["dependency"]["version"] == "0.4.6" and dep_review["dependency_files_match_locked_archive"]
    assert dep_review["lock_sha256"] == sha(SOURCE / "Cargo.lock")
    dependency_sources = {row["path"]: row["sha256"] for row in dep_review["dependency_sections_read"]}
    assert all(sha(Path(n)) == h for n, h in dependency_sources.items())
    preparation = read(E / "storage-contract-probe-preparation-v2387.json")
    for name, row in preparation["pinned_specification_sources_inspected"].items():
        assert sha(Path(f"/tmp/csgrs-pinned-verus-{name}-671956e.rs")) == row["sha256"]
    pin = read(SOURCE / "crates/hyperreal/verification/toolchain.json")
    tool = ROOT / "crates/hyperreal/target/verus" / pin["version"] / "verus-x86-linux"
    tooling = read(E / "typed-api-prototype-v2380-word-scan-verus/proof-reports/tooling.json")
    assert all(sha(tool / n) == h for n, h in tooling["tool_sha256"].items())
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    out.write_text(json.dumps({
        "status": "Direct dependency-call proof limits established; borrowed-slice prototype and its controls pass",
        "runner_sha256": sha(Path(__file__)), "retained_checkpoint": checkpoint_path.name,
        "retained_checkpoint_sha256": sha(checkpoint_path), "retained_source_record_sha256": checkpoint["source_record_sha256"],
        "candidate_sources_changed": False, "main_sources_changed": False,
        "source_files_verified": len(sources), "main_files_verified": len(guard),
        "retained_verified_units": 736, "retained_production_contracts": 98, "retained_model_theorems": 178,
        "retained_negative_controls": 306, "retained_positive_baselines": 7,
        "whole_crate_obligation_groups_open": 13, "new_retained_proof_units": 0,
        "new_retained_model_theorems": 0, "new_retained_production_contracts": 0,
        "phases": phases, "prototype_guard_protocol_sha256": sha(controls_path / "protocol.json"),
        "prototype_guard_result_sha256": sha(controls_path / "result.json"),
        "prototype_rejected_mutations": 12, "prototype_positive_baselines": 1,
        "prototype_executable_functions": 12,
        "dependency_source_review": dep_review_path.name, "dependency_source_review_sha256": sha(dep_review_path),
        "dependency_sources_sha256": dependency_sources,
        "pinned_specification_sources_inspected": preparation["pinned_specification_sources_inspected"],
        "findings": [
            "After separately resolving reference-pattern syntax, both native widths fail at the actual slice iterator position/rposition methods: the pinned verifier supplies no supported method contract for these calls.",
            "After explicitly naming the mutable post-state, the actual capacity and shrink_to_fit calls also report unsupported methods. No local assume_specification or other proof escape hatch was added.",
            "The direct Vec::truncate wrapper connects its existing pinned contract to the retained typed zero-suffix proof for both widths. Its 440-unit run includes existing imports and proves two harness functions, not the complete BigUint normalize implementation.",
            "A separate twelve-function indexed-slice prototype verifies first/last matches, logical normalization and three native bit counts. Its 454-unit run includes existing imports. All twelve executable mutation controls fail with proof errors rather than frontend errors, while the unmutated baseline passes.",
            "The indexed prototype is new code. It does not prove the original dependency iterator implementation, preserve its allocation/shrink policy by construction, establish the BigUint storage view or discharge actual metadata range/caller premises. It has not replaced runtime code or passed fixed-baseline runtime/cost/resource admission."
        ],
        "next_action": "Use the proved executable prototype to make a concrete same-source dependency integration candidate, or establish the missing iterator/allocator method proofs without local trust escape hatches. Bind the selected path to actual BigUint storage and allocation behavior, discharge count-range/caller premises, and run original fixed-baseline correctness, cost, memory and size gates before admitting any runtime change. Keep the current v2335 runtime and v2386 proof checkpoint until those gates pass.",
        "fresh_runtime_test_processes": 0, "fresh_cost_processes": 0, "main_runtime_admitted": False,
        "all_39_properties_14_targets_and_crate_future_work_in_scope": True,
        "limits": "This milestone records capability failures and a guarded executable support prototype only. Retained counts and all thirteen whole-crate groups are unchanged. Preserve every failed attempt, historical resource/cost failure, pre-Verus baseline, original worktree entry, and main/native/full-board requirement. No property, target, backend or whole-crate completion is claimed.",
    }, indent=2) + "\n")
    print(json.dumps({"storage_call_probe_phases": len(phases), "prototype_negative_controls": 12,
        "prototype_executable_functions": 12, "retained_verified_units": 736, "source_changed": False}))


if __name__ == "__main__":
    main()
