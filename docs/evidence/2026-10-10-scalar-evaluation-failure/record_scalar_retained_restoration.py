"""Bind fresh restoration checks and prior evidence to the exact retained source."""
import copy
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess

from parse_test_results_v3 import parse_phase

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    output = E / "scalar-retained-restoration-review-v2166.json"
    ledger_output = E / "scalar-retained-restoration-ledger-v2166.json"
    assert not output.exists() and not ledger_output.exists()
    directory = E / "scalar-retained-restoration-v2159"
    protocol = read(directory / "protocol.json")
    controller_path = E / "scalar-retained-restoration-validation-v2165.json"
    controller = read(controller_path)
    assert controller["complete"] and len(controller["phases"]) == 6
    expected = protocol["retained_source_record_sha256"]
    assert controller["source_record_sha256"] == expected
    assert controller["protocol_sha256"] == sha(directory / "protocol.json")
    for row, frozen in zip(controller["phases"], protocol["commands"]):
        assert row["phase"] == frozen["phase"] and row["command"] == frozen["command"]
        phase = E / row["phase"]
        assert read(phase / "result.json") == row["result"]
        assert row["result"]["exit_code"] == 0 and row["result"]["source_unchanged"] and row["result"]["main_unchanged"]
        assert sha(phase / "source-hashes.json.gz") == expected
        assert read(phase / "command.json")["command"] == frozen["command"]
    source_record = E / controller["phases"][0]["phase"] / "source-hashes.json.gz"
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    assert len(sources) == 24746 and all(sha(SOURCE / n) == h for n, h in sources.items())
    main = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert len(main) == 1167 and all(sha(ROOT / n) == h for n, h in main.items())
    assert sha(directory / "restored-source-hashes.json.gz") == expected
    assert len(protocol["restorations"]) == 9
    for name, row in protocol["restorations"].items():
        assert sha(directory / "rejected" / name) == row["rejected_sha256"]
        assert sha(E / row["retained_archive"]) == sha(SOURCE / name) == row["retained_sha256"]
    subprocess.run(["git", "apply", "--reverse", "--check", str(directory / "restoration.patch")], cwd=SOURCE, check=True)
    counts = parse_phase(E / controller["phases"][0]["phase"])
    assert counts["top_level_passed"] == 1074 and counts["child_process_passed"] == 1
    assert counts["top_level_failed"] == counts["child_process_failed"] == counts["top_level_ignored"] == 0
    assert len(controller["phases"][3]["command"][8:]) == 108
    proof = E / controller["phases"][4]["phase"]
    acceptance = E / controller["phases"][5]["phase"]
    assert re.search(r"Ran 10 tests in .*\n\nOK\n", (acceptance / "stderr.log").read_text())
    report = read(proof / "proof-reports/verus.json")
    result = report["verification-results"]
    assert result["success"] and result["verified"] == 645 and result["errors"] == 0
    assert result["is-verifying-entire-crate"] and not result["encountered-error"] and not result["encountered-vir-error"]
    evidence = read(proof / "proof-reports/evidence.json")
    assert "--no-cheating" in evidence["command"]
    scope = read(SOURCE / "crates/hyperreal/verification/scope.json")
    assert scope == evidence["scope"]
    assert len(scope["verified_executable_contracts"]) == 97
    assert len(scope["verified_model_theorems"]) == 130 and len(scope["obligations"]) == 13
    for key in ["verified_executable_contracts", "verified_model_theorems"]:
        assert all("hyperreal_proofs::" + name in report["func-details"] for name in scope[key])
    prior_proof_path = E / "domain-predicate-proof-validation-v1763.json"
    prior_proof = read(prior_proof_path)
    assert prior_proof["source_record_sha256"] == expected
    rejection_phase = E / prior_proof["fresh_rejection_phase"]
    assert sha(rejection_phase / "source-hashes.json.gz") == expected
    old_tooling = read(rejection_phase / "proof-checks/tooling.json")
    calls = [json.loads(line) for line in gzip.decompress((rejection_phase / "proof-checks/verus-invocations.jsonl.gz").read_bytes()).splitlines()]
    assert len(calls) == old_tooling["verus_invocations"] == 230
    assert sum(row["exit_code"] == 0 for row in calls) == old_tooling["baseline_successes"] == 4
    assert sum(row["exit_code"] != 0 for row in calls) == old_tooling["rejected_invocations"] == 226
    assert old_tooling["tool_unchanged"] and old_tooling["exit_code"] == 0
    for phase, subdir in [(proof, "proof-reports"), (acceptance, "proof-checks")]:
        tooling = read(phase / subdir / "tooling.json")
        assert tooling["tool_unchanged"] and tooling["exit_code"] == 0
        assert tooling["tool_sha256"] == old_tooling["tool_sha256"]
    dependencies = proof / "proof-reports/dependencies.json"
    prior_dependencies = E / prior_proof["fresh_verification_phase"] / "proof-reports/dependencies.json"
    assert dependencies.read_bytes() == prior_dependencies.read_bytes()
    inputs = {"scalar-retained-restoration-v2159/protocol.json": sha(directory / "protocol.json"),
              "scalar-retained-restoration-v2159/restoration.json": sha(directory / "restoration.json"),
              controller_path.name: sha(controller_path), prior_proof_path.name: sha(prior_proof_path)}
    for name in ["bound-merge-cost-rejection-v1857.json", "cache-state-merge-cost-rejection-v1943.json",
                 "nth-root-enclosure-cost-rejection-v2021.json", "integer-coarse-zero-cost-rejection-v2100.json"]:
        assert not read(E / name)["runtime_retained"]
        inputs[name] = sha(E / name)
    next_priority = (
        "Connect the actual retained cache-word decoder to its specification, first checking "
        "bool::then_some and masked unreachable-arm support without assumed specifications. "
        "Preserve all u64 inputs, signs, signed exponent limits, absent magnitudes and unrelated "
        "bits; separate encoding correctness from producer validity, independent existence and "
        "atomic publication. Keep the merge/root/integer proposals rejected and every original "
        "cost failure preserved. Continue the remaining resource, native and unchanged full-board "
        "admission work; all 39 properties, 14 targets and crate future work remain in scope."
    )
    record = {"status": "Exact v1836 source restored; six fresh restoration commands pass",
        "retained_runtime_checkpoint": protocol["checkpoint"], "retained_runtime_checkpoint_sha256": protocol["checkpoint_sha256"],
        "source_record": str(source_record.relative_to(E)), "source_record_sha256": expected,
        "source_files_verified": len(sources), "main_files_verified": len(main), "restored_files": 9,
        "inputs_sha256": inputs, "fresh_commands": 6, "scalar_counts": counts,
        "included_rustfmt_files": 108, "verified_units": 645, "errors": 0,
        "listed_production_contracts": 97, "listed_model_theorems": 130, "open_obligation_groups": 13,
        "fresh_acceptance_tests": 10, "fresh_rejected_mutations": 0, "reused_rejected_mutations": 226,
        "reused_guard_baselines": 4, "reused_rejection_phase": rejection_phase.name,
        "reused_rejection_tooling_sha256": sha(rejection_phase / "proof-checks/tooling.json"),
        "proof_dependency_identity_sha256": sha(dependencies), "new_runtime_admission": False,
        "main_runtime_admitted": False, "limits": protocol["limits"], "next_priority": next_priority}
    ledger = ROOT / "docs/properties-status.json"
    before_bytes = ledger.read_bytes()
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == before_bytes
    before, after = json.loads(before_bytes), json.loads(before_bytes)
    p4 = next(row for row in after["properties"] if row["id"] == "P4")
    p4["evidence"].append(str(output.relative_to(ROOT)))
    update = (
        "The v2159-v2166 restoration returns all 24746 isolated source files to the exact v1836 "
        "checkpoint. Nine rejected source files and their complete patch remain archived. Six "
        "fresh commands pass: 1074 scalar units plus one child, release build, workspace and "
        "108 included-input format checks, 645 conditional proof units and ten acceptance tests. "
        "The 226 negative mutations and four passing baselines are reused under identical "
        "source, dependency and verifier identities. The discarded merge's 651-unit report "
        "remains historical evidence for that unretained source. Every cost rejection, including "
        "the v2100 baseline SIGXCPU, remains failed. Prior v1836 broad/replay/cost evidence is "
        "reused for identical retained bytes, not rerun here. Main and all property/target "
        "statuses are unchanged; decoder, producer/existence/atomic and native/full-board "
        "obligations remain open."
    )
    p4["remaining"] += " " + update
    prior_priority = read(E / "integer-coarse-zero-baseline-profile-review-v2158.json")["next_priority"]
    assert after["next_actions"][0].count(prior_priority) == 1
    after["next_actions"][0] = after["next_actions"][0].replace(prior_priority, next_priority, 1)
    after["next_actions"][0] += " " + update
    same = copy.deepcopy(after)
    original_p4 = next(row for row in before["properties"] if row["id"] == "P4")
    compare_p4 = next(row for row in same["properties"] if row["id"] == "P4")
    compare_p4["remaining"], compare_p4["evidence"] = original_p4["remaining"], original_p4["evidence"]
    same["next_actions"][0] = before["next_actions"][0]
    assert same == before
    output.write_text(json.dumps(record, indent=2) + "\n")
    ledger.write_text(json.dumps(after, indent=2) + "\n")
    ledger_output.write_text(json.dumps({"before_sha256": hashlib.sha256(before_bytes).hexdigest(),
        "after_sha256": sha(ledger), "review_sha256": sha(output),
        "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
        "all_39_property_and_14_target_requirements_and_statuses_preserved": True}, indent=2) + "\n")
    print(json.dumps({"restored_files": 9, "fresh_commands": 6, "scalar_tests": 1074,
                      "child_tests": 1, "conditional_proof_units": 645, "new_runtime_admission": False}))


if __name__ == "__main__":
    main()
