"""Record proved eager decoding and scoped static/guard evidence without retention."""
import copy
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    output = E / "eager-bound-decoder-early-milestone-v2265.json"
    audit = E / "eager-bound-decoder-early-ledger-v2265.json"
    assert not output.exists() and not audit.exists()
    baseline_path = E / "eager-bound-decoder-baseline-v2257/protocol.json"
    baseline = read(baseline_path)
    expected = baseline["source_record_sha256"]
    early_path = E / "eager-bound-decoder-early-validation-v2261.json"
    early = read(early_path)
    guards_path = E / "eager-bound-decoder-focused-guards-v2265.json"
    guards = read(guards_path)
    assert early["complete"] and len(early["phases"]) == 4
    assert guards["complete"] and len(guards["phases"]) == 2
    assert early["source_record_sha256"] == guards["source_record_sha256"] == expected
    for name, row in list(early["phases"].items()) + [(r["phase"], r) for r in guards["phases"]]:
        phase = E / name
        assert read(phase / "result.json") == row["result"]
        assert row["result"]["exit_code"] == 0 and row["result"]["source_unchanged"] and row["result"]["main_unchanged"]
        assert sha(phase / "source-hashes.json.gz") == expected
    proof_phase = E / "typed-api-prototype-v2260-eager-bound-decoder-verus"
    report = read(proof_phase / "proof-reports/verus.json")
    result = report["verification-results"]
    assert result["success"] and result["verified"] == 655 and result["errors"] == 0
    assert result["is-verifying-entire-crate"] and not result["encountered-error"] and not result["encountered-vir-error"]
    scope = read(SOURCE / "crates/hyperreal/verification/scope.json")
    assert len(scope["verified_executable_contracts"]) == 98 and len(scope["verified_model_theorems"]) == 135
    assert len(scope["obligations"]) == 13
    assert "hyperreal_proofs::computable_cache::AtomicFacts::decode_bound" in report["func-details"]
    for key in ["verified_executable_contracts", "verified_model_theorems"]:
        assert all("hyperreal_proofs::" + name in report["func-details"] for name in scope[key])
    acceptance = E / guards["phases"][0]["phase"]
    assert re.search(r"Ran 10 tests in .*\n\nOK\n", (acceptance / "stderr.log").read_text())
    cache = E / guards["phases"][1]["phase"] / "proof-checks"
    tooling = read(cache / "tooling.json")
    calls = [json.loads(s) for s in gzip.decompress((cache / "verus-invocations.jsonl.gz").read_bytes()).splitlines()]
    assert tooling["exit_code"] == 0 and tooling["tool_unchanged"]
    assert len(calls) == tooling["verus_invocations"] == 42
    assert sum(row["exit_code"] == 0 for row in calls) == tooling["baseline_successes"] == 1
    assert sum(row["exit_code"] != 0 for row in calls) == tooling["rejected_invocations"] == 41
    source_record = proof_phase / "source-hashes.json.gz"
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    retained = json.loads(gzip.decompress((E / baseline["retained_source_record"]).read_bytes()))
    assert len(sources) == 24746 and sources.keys() == retained.keys()
    changed = {n: h for n, h in sources.items() if retained[n] != h}
    assert changed == baseline["candidate_sources_sha256"] and len(changed) == 5
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    main = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert len(main) == 1167 and all(sha(ROOT / n) == h for n, h in main.items())
    comparison_path = E / "eager-bound-decoder-binary-comparison-v2262/summary.json"
    comparison = read(comparison_path)
    static_path = E / "eager-bound-decoder-load-image-review-v2263.json"
    static = read(static_path)
    assert comparison["source_record_sha256"] == static["source_record_sha256"] == expected
    assert comparison["builds_passed"] == 7 and not comparison["all_probe_binaries_identical"]
    assert comparison["new_timing_processes"] == 0
    assert static["comparison_record_sha256"] == sha(comparison_path)
    assert not static["qualification"]["full_executable_identity"] and not static["qualification"]["cost_gate_passed"]
    assert static["qualification"]["reused_cost_receipts"] == static["qualification"]["new_workload_processes"] == 0
    next_priority = (
        "Freeze and review an explicit qualification method for the eager decoder's identical "
        "instruction bytes and narrowly identified diagnostic/ELF metadata differences before "
        "any further runtime cost work. The whole-executable identity condition remains failed; "
        "do not reuse receipts under it or repeat the failed baseline comparison unchanged. "
        "Keep the original caps and failures visible, and complete the remaining applicable "
        "runtime, mutation, replay and fact-retention gates before retention. Continue producer, "
        "existence, atomic/caller and resource work toward native/full-board admission."
    )
    record = {"status": "Eager decoder proved with matching instruction bytes; full identity and runtime qualification remain open",
        "source_record": str(source_record.relative_to(E)), "source_record_sha256": expected,
        "source_files_verified": len(sources), "main_files_verified": len(main), "changed_sources_sha256": changed,
        "retained_runtime_checkpoint": baseline["retained_runtime_checkpoint"],
        "retained_runtime_checkpoint_sha256": baseline["retained_runtime_checkpoint_sha256"],
        "inputs_sha256": {str(p.relative_to(E)): sha(p) for p in [baseline_path, early_path, guards_path, comparison_path, static_path]},
        "verified_units": 655, "listed_production_contracts": 98, "listed_model_theorems": 135,
        "open_obligation_groups": 13, "fresh_acceptance_tests": 10,
        "fresh_cache_rejected_mutations": 41, "fresh_cache_passing_baselines": 1,
        "other_mutation_suites_executed": 0, "other_mutations_credited_as_reused": 0,
        "fresh_format_proof_build_commands": 4, "fresh_scalar_or_consumer_runtime_tests": 0,
        "compiled_probe_families": 7, "identical_executable_instruction_segments": True,
        "identical_complete_executables": 0, "new_timing_processes": 0, "cost_receipts_reused": 0,
        "runtime_retained": False, "main_runtime_admitted": False, "next_priority": next_priority,
        "limits": [
            "The actual all-u64 decoder and five connected field lemmas are verified. Certificate producer truth, independent existence, atomic/caller ordering and all thirteen whole-crate groups remain open.",
            "Only ten acceptance tests and the 41 cache mutations plus one positive baseline were run afresh after this payload change. Earlier 245-mutation and nine-command reports describe the rejected source, not this one.",
            "The static comparison preserves complete executable instruction segments but changes source locations, build identifiers and private symbol metadata. This is not full executable identity, a whole-program equivalence theorem or a fresh cost pass. All prior CPU and timing failures remain recorded.",
            "The runtime remains unretained. Broad tests, applicable proof guards, resource/replay/fact checks, the fixed pre-Verus baseline and main/native/full-board admission remain separate requirements. No property/target status changes follow.",
        ]}
    ledger = ROOT / "docs/properties-status.json"
    before_bytes = ledger.read_bytes()
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == before_bytes
    before, after = json.loads(before_bytes), json.loads(before_bytes)
    p4 = next(row for row in after["properties"] if row["id"] == "P4")
    for p in [static_path, guards_path, output]:
        p4["evidence"].append(str(p.relative_to(ROOT)))
    update = (
        "The v2257-v2265 eager-payload decoder experiment computes the magnitude before "
        "selecting Some/None. Four fresh formatting/proof/build checks pass, including 655 "
        "conditional proof units. Ten acceptance tests and 41 cache mutations plus one "
        "positive baseline pass; other mutation suites and runtime tests are not credited "
        "for this source. All seven probe instruction segments match the retained binaries. "
        "Remaining loaded-byte differences are ELF metadata and six unchanged statements' "
        "source-line records; complete executable identity still fails. No timing processes "
        "run and no cost receipts are reused. Runtime retention and every prior obligation "
        "remain open; v1836 is still the retained checkpoint."
    )
    p4["remaining"] += " " + update
    previous = read(E / "decoder-retained-restoration-review-v2255.json")["next_priority"]
    assert after["next_actions"][0].count(previous) == 1
    after["next_actions"][0] = after["next_actions"][0].replace(previous, next_priority, 1)
    after["next_actions"][0] += " " + update
    same = copy.deepcopy(after)
    old_p4 = next(row for row in before["properties"] if row["id"] == "P4")
    compare = next(row for row in same["properties"] if row["id"] == "P4")
    compare["remaining"], compare["evidence"] = old_p4["remaining"], old_p4["evidence"]
    same["next_actions"][0] = before["next_actions"][0]
    assert same == before
    output.write_text(json.dumps(record, indent=2) + "\n")
    ledger.write_text(json.dumps(after, indent=2) + "\n")
    audit.write_text(json.dumps({"before_sha256": hashlib.sha256(before_bytes).hexdigest(),
        "after_sha256": sha(ledger), "milestone_record_sha256": sha(output),
        "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
        "all_39_property_and_14_target_requirements_and_statuses_preserved": True,
        "runtime_retention_pending": True}, indent=2) + "\n")
    print(json.dumps({"verified_units": 655, "fresh_cache_mutations": 41,
                      "instruction_identical_probes": 7, "runtime_retention_pending": True}))


if __name__ == "__main__":
    main()
