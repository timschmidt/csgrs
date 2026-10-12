"""Bind focused endpoint tests and unchanged formal coverage to the new proposal."""
import copy
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    output = E / "integer-coarse-zero-proof-validation-v2083.json"
    patch = E / "integer-coarse-zero-proposal-v2083.patch"
    ledger_record = E / "integer-coarse-zero-proof-ledger-v2083.json"
    assert all(not path.exists() for path in [output, patch, ledger_record])
    baseline_dir = E / "integer-coarse-zero-baseline-v2079"
    baseline = read(baseline_dir / "baseline.json")
    prior_path = E / "nth-root-enclosure-proof-validation-v2004.json"
    prior = read(prior_path)
    expected = "e093cd7ed95ae15ecb66d04cd76beb89b04aa205971330f4b2794db4190ded62"
    focused = E / "typed-api-prototype-v2080-integer-coarse-zero-focused"
    proof = E / "typed-api-prototype-v2082-integer-coarse-zero-verus"
    acceptance = E / "typed-api-prototype-v2083-integer-coarse-zero-acceptance"
    root_controls = E / "typed-api-prototype-v2081-integer-coarse-zero-root-controls"
    for phase in [focused, root_controls, proof, acceptance]:
        result = read(phase / "result.json")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
        assert sha(phase / "source-hashes.json.gz") == expected
    assert "test result: ok. 5 passed; 0 failed; 0 ignored;" in (focused / "stdout.log").read_text()
    assert "test result: ok. 3 passed; 0 failed; 0 ignored;" in (root_controls / "stdout.log").read_text()
    assert re.search(r"Ran 10 tests in .*\n\nOK\n", (acceptance / "stderr.log").read_text())
    source_record = proof / "source-hashes.json.gz"
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    previous = json.loads(gzip.decompress((E / baseline["current_source_record"]).read_bytes()))
    retained = read(E / baseline["retained_runtime_checkpoint"])
    original = json.loads(gzip.decompress((E / retained["source_record"]).read_bytes()))
    assert sources.keys() == previous.keys() == original.keys() and len(sources) == 24746
    root_changes = {name for name in sources if sources[name] != previous[name]}
    changes = {name for name in sources if sources[name] != original[name]}
    assert root_changes == set(baseline["planned_changed_files"]) and len(changes) == 9
    assert changes == root_changes | set(prior["changed_sources_sha256"])
    for name, digest in sources.items():
        assert sha(SOURCE / name) == digest
    for name, digest in baseline["archived_sources_sha256"].items():
        assert sha(baseline_dir / "inputs" / name) == digest
    for key in ["prior_rejection", "static_review", "prior_proof_record"]:
        assert sha(E / baseline[key]) == baseline[key + "_sha256"]
    main_guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    for name, digest in main_guard.items():
        assert sha(ROOT / name) == digest

    # The production kernel is outside the unchanged conditional proof target.
    # Reuse negative results only with all actual proof/guard inputs, dependency
    # artifacts and verifier binaries unchanged; do not call those results fresh.
    assert len(prior["proof_and_guard_inputs_sha256"]) == 44
    assert root_changes.isdisjoint(prior["proof_and_guard_inputs_sha256"])
    assert baseline["proof_and_guard_inputs_sha256"] == prior["proof_and_guard_inputs_sha256"]
    for name, digest in prior["proof_and_guard_inputs_sha256"].items():
        assert sha(SOURCE / name) == digest
    report = read(proof / "proof-reports/verus.json")
    evidence = read(proof / "proof-reports/evidence.json")
    result = report["verification-results"]
    assert result["success"] and result["verified"] == 651 and result["errors"] == 0
    assert result["is-verifying-entire-crate"] and not result["encountered-error"] and not result["encountered-vir-error"]
    assert "--no-cheating" in evidence["command"]
    scope = read(SOURCE / "crates/hyperreal/verification/scope.json")
    assert scope == evidence["scope"]
    assert len(scope["verified_executable_contracts"]) == 98
    assert len(scope["verified_model_theorems"]) == 135 and len(scope["obligations"]) == 13
    for key in ["verified_executable_contracts", "verified_model_theorems"]:
        assert all("hyperreal_proofs::" + name in report["func-details"] for name in scope[key])
    rejection = E / prior["reused_rejection_phase"]
    old_tooling = read(rejection / "proof-checks/tooling.json")
    calls = [json.loads(line) for line in gzip.decompress((rejection / "proof-checks/verus-invocations.jsonl.gz").read_bytes()).splitlines()]
    assert old_tooling["tool_unchanged"] and old_tooling["exit_code"] == 0
    assert old_tooling["verus_invocations"] == len(calls) == 251
    assert sum(row["exit_code"] == 0 for row in calls) == old_tooling["baseline_successes"] == 4
    assert sum(row["exit_code"] != 0 for row in calls) == old_tooling["rejected_invocations"] == 247
    for phase, directory in [(proof, "proof-reports"), (acceptance, "proof-checks")]:
        tooling = read(phase / directory / "tooling.json")
        assert tooling["tool_unchanged"] and tooling["exit_code"] == 0
        assert tooling["tool_sha256"] == old_tooling["tool_sha256"]
    dependencies = proof / "proof-reports/dependencies.json"
    old_dependencies = E / prior["fresh_verification_phase"] / "proof-reports/dependencies.json"
    assert dependencies.read_bytes() == old_dependencies.read_bytes()
    query_name = "crates/hyperreal/src/computable/node/approximation_queries.rs"
    old_query = (baseline_dir / "inputs" / query_name).read_text()
    new_query = (SOURCE / query_name).read_text()
    inserted = (
        "                // A gap beyond the integer's bit length rounds to zero for\n"
        "                // either sign. Resolve the scaling helper's same shortcut\n"
        "                // here, after the precision checks and work charge.\n"
        "                if p > 0 && u64::from(p as u32) > value.bits() {\n"
        "                    return Ok(BigInt::zero());\n"
        "                }\n")
    assert new_query.count(inserted) == 1 and new_query.replace(inserted, "", 1) == old_query
    prefix = new_query.split(inserted, 1)[0]
    assert prefix.endswith('                if !charge_evaluation_work() {\n                    return Err(Problem::Exhausted);\n                }\n')
    assert old_query.count('let p = checked_approximation_precision(p)?;') == 1
    diff = []
    for name in sorted(changes):
        candidates = [directory / "inputs" / name for directory in
            [baseline_dir, E / "nth-root-enclosure-baseline-v2001", E / "bound-merge-baseline-v1837"]]
        matches = [path for path in candidates if path.is_file() and sha(path) == original[name]]
        assert matches, name
        original_file = matches[0]
        diff.extend(difflib.unified_diff(original_file.read_text().splitlines(True),
            (SOURCE / name).read_text().splitlines(True), fromfile="a/" + name, tofile="b/" + name))
    record = {
        "status": "Charged integer coarse-zero proposal passes existing focused tests and unchanged conditional verification; runtime gates pending",
        "retained_runtime_checkpoint": baseline["retained_runtime_checkpoint"],
        "retained_runtime_checkpoint_sha256": baseline["retained_runtime_checkpoint_sha256"],
        "baseline": str((baseline_dir / "baseline.json").relative_to(E)), "baseline_sha256": sha(baseline_dir / "baseline.json"),
        "source_record": str(source_record.relative_to(E)), "source_record_sha256": expected,
        "source_files_verified": len(sources), "main_guarded_files_verified": len(main_guard),
        "changed_sources_sha256": {name: sources[name] for name in sorted(changes)},
        "changed_since_rejected_proposal": sorted(root_changes), "proposal_patch": patch.name,
        "focused_phase": focused.name, "focused_tests": 5,
        "root_control_phase": root_controls.name, "root_control_tests": 3, "new_tests_added": 0,
        "fresh_verification_phase": proof.name, "fresh_acceptance_phase": acceptance.name,
        "verified_units": 651, "errors": 0, "listed_production_contracts": 98,
        "listed_model_theorems": 135, "open_obligation_groups": 13, "fresh_acceptance_tests": 10,
        "fresh_rejected_mutants": 0, "fresh_passing_guard_baselines": 0,
        "reused_rejected_mutants": 247, "reused_passing_guard_baselines": 4,
        "reused_rejection_phase": rejection.name, "reused_proof_record": prior["reused_proof_record"],
        "reused_proof_record_sha256": prior["reused_proof_record_sha256"],
        "prior_proposal_proof_record": prior_path.name, "prior_proposal_proof_record_sha256": sha(prior_path),
        "reused_rejection_tooling_sha256": sha(rejection / "proof-checks/tooling.json"),
        "proof_and_guard_inputs_sha256": prior["proof_and_guard_inputs_sha256"],
        "proof_dependency_identity_sha256": sha(dependencies),
        "prior_failure": "nth-root-enclosure-cost-rejection-v2021.json",
        "prior_failure_sha256": sha(E / "nth-root-enclosure-cost-rejection-v2021.json"),
        "implementation": "The exact Int branch returns zero for p > 0 and u64(p) > value.bits() after the existing precision checks and successful evaluation-work charge. This is the called scaling helper's identical strict predicate and result. All other evaluator bytes, five existing integer tests, root code/tests and proof/guard inputs remain unchanged.",
        "limits": [
            "The changed integer evaluator branch and bigint nth-root helper are outside Verus coverage. The 651 verified units refresh the existing 98 contracts and 135 model theorems; they do not formally prove either runtime change or complete any open group.",
            "The 247 negative mutations and four passing guard baselines are reused, not rerun. All 44 proof/guard inputs, dependency identity bytes and verifier binaries are unchanged. Ten acceptance tests do not satisfy the pending fixed pre-Verus acceptance report.",
            "The combined proposal includes the unretained outlined selector and one-root kernel. The v2021 wide_gap timing rejection, v1943 CPU-limit failure, v1857 timing rejection, earlier failures/tradeoff and fixed baseline remain unchanged. Fresh broad, cost and replay qualification is required against retained v1836.",
            "Five unchanged integer tests cover signed rounding, million-bit allocation controls, work allowances/retries, retained signs, precision limits and cancellation precedence. Three unchanged root controls pass. This is finite evidence; no performance or size improvement is inferred from the new branch alone. The earlier copied-root allocation study retains only its original extracted-helper scope.",
            "EvaluationBudget still excludes aggregate bytes/time and unmetered algebraic helpers. Main/native/full-board admission, all 39 properties, 14 targets and crate future work remain open.",
        ],
        "runtime_retained": False, "main_runtime_admitted": False,
    }
    ledger = ROOT / "docs/properties-status.json"
    before_bytes = ledger.read_bytes()
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == before_bytes
    before = json.loads(before_bytes)
    after = copy.deepcopy(before)
    assert len(after["properties"]) == 39 and len(after["targets"]) == 14
    p4 = next(row for row in after["properties"] if row["id"] == "P4")
    for name in [record["baseline"], output.name, patch.name, proof.name + "/proof-reports/evidence.json"]:
        path = str(E.relative_to(ROOT) / name)
        assert path not in p4["evidence"]
        p4["evidence"].append(path)
    update = (
        "The v2079-v2083 isolated proposal handles the scaling helper's strict coarse-zero case "
        "directly in the exact-integer evaluator, after the original precision and work checks. "
        "Five unchanged integer tests and three unchanged root controls pass. The other evaluator "
        "bytes, root kernel and test sources are unchanged. Refreshed conditional verification "
        "reports 651 units with zero errors, and ten acceptance tests pass. The 247 negative "
        "guards and four positive baselines are reused under unchanged proof/guard/dependency/tool "
        "identities; the integer branch and bigint root kernel remain outside formal coverage. "
        "The v2021 timing rejection remains failed, and retained runtime stays v1836 pending "
        "fresh broad commands, all original-cap cost controls and replays."
    )
    p4["remaining"] += " " + update
    old = read(E / "nth-root-enclosure-cost-rejection-v2021.json")["next_priority"]
    assert after["next_actions"][0].count(old) == 1
    after["next_actions"][0] = after["next_actions"][0].replace(old,
        "The charged integer coarse-zero prototype passes existing focused tests and refreshed conditional verification. "
        "Next run the nine broad commands, freeze the new cost/replay protocols, and evaluate the "
        "combined proposal against v1836 under every unchanged full-size fixture and original cap. "
        "Preserve all earlier failures and do not retain it before all gates pass.", 1)
    after["next_actions"][0] += " " + update
    unchanged = copy.deepcopy(after)
    old_p4 = next(row for row in before["properties"] if row["id"] == "P4")
    compare_p4 = next(row for row in unchanged["properties"] if row["id"] == "P4")
    compare_p4["evidence"], compare_p4["remaining"] = old_p4["evidence"], old_p4["remaining"]
    unchanged["next_actions"][0] = before["next_actions"][0]
    assert unchanged == before
    patch.write_text("".join(diff))
    subprocess.run(["git", "apply", "--reverse", "--check", str(patch)], cwd=SOURCE, check=True)
    record["proposal_patch_sha256"] = sha(patch)
    output.write_text(json.dumps(record, indent=2) + "\n")
    ledger.write_text(json.dumps(after, indent=2) + "\n")
    ledger_record.write_text(json.dumps({
        "before_sha256": hashlib.sha256(before_bytes).hexdigest(), "after_sha256": sha(ledger),
        "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
        "all_39_property_and_14_target_requirements_and_statuses_preserved": True,
        "proof_record_sha256": sha(output), "runtime_retained": False,
    }, indent=2) + "\n")
    print(json.dumps({"focused_tests": 5, "root_control_tests": 3, "verified_units": 651, "acceptance_tests": 10,
        "reused_negative_mutations": 247, "reused_guard_baselines": 4,
        "changed_sources_since_retained": 9, "runtime_retained": False}))


if __name__ == "__main__":
    main()
