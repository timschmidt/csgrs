"""Bind the production merge proof to its frozen inputs without runtime admission."""
import copy
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

sys.dont_write_bytecode = True
from record_domain_predicate_proof import mutation_lists

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def publication(text):
    start = text.index("    fn publish_bound(")
    end = text.index("    // Explicit replacement is reserved for test fixture setup.", start)
    return text[start:end]


def main():
    out = E / "cache-state-merge-proof-validation-v1926.json"
    patch = E / "cache-state-merge-proposal-v1926.patch"
    ledger_record = E / "cache-state-merge-proof-ledger-v1926.json"
    assert not any(path.exists() for path in [out, patch, ledger_record])
    baseline_dir = E / "bound-merge-baseline-v1837"
    baseline = read(baseline_dir / "baseline.json")
    proof = E / "typed-api-prototype-v1924-cache-state-merge-verus"
    acceptance = E / "typed-api-prototype-v1925-cache-state-merge-acceptance"
    rejection = E / "typed-api-prototype-v1926-cache-state-merge-rejections"
    source_record = proof / "source-hashes.json.gz"
    expected = "555cf4a48df68e021119c107412824b5879baf48e252c6fb1c8d2bc22baf1c59"
    assert sha(source_record) == expected
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    previous = json.loads(gzip.decompress((E / baseline["source_record"]).read_bytes()))
    assert sources.keys() == previous.keys() and len(sources) == 24746
    changes = {name for name in sources if sources[name] != previous[name]}
    expected_changes = {"crates/hyperreal/" + name for name in [
        "src/computable/node/bounds.rs", "src/computable/node/representation.rs",
        "verification/bound_denotation.rs", "verification/test_bound_rejections.py",
        "verification/scope.json", "verification/README.md"]}
    assert changes == expected_changes
    assert all(sha(SOURCE / name) == digest for name, digest in sources.items())
    main_guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert all(sha(ROOT / name) == digest for name, digest in main_guard.items())
    for name, digest in baseline["baseline_inputs_sha256"].items():
        assert sha(baseline_dir / "inputs" / name) == digest
        if name not in changes:
            assert sha(SOURCE / name) == digest
    for name, digest in baseline["retained_records_sha256"].items():
        assert sha(E / name) == digest
    for phase in [proof, acceptance, rejection]:
        result = read(phase / "result.json")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
        assert sha(phase / "source-hashes.json.gz") == expected
    evidence = read(proof / "proof-reports/evidence.json")
    report = read(proof / "proof-reports/verus.json")
    verified = report["verification-results"]
    assert verified["success"] and verified["verified"] == 651 and verified["errors"] == 0
    assert verified["is-verifying-entire-crate"] and not verified["encountered-error"] and not verified["encountered-vir-error"]
    assert "--no-cheating" in evidence["command"]
    scope = read(SOURCE / "crates/hyperreal/verification/scope.json")
    previous_scope = read(baseline_dir / "inputs/crates/hyperreal/verification/scope.json")
    assert evidence["scope"] == scope and scope["obligations"] == previous_scope["obligations"]
    assert len(scope["verified_executable_contracts"]) == 98
    assert len(scope["verified_model_theorems"]) == 135 and len(scope["obligations"]) == 13
    for key in ["verified_executable_contracts", "verified_model_theorems"]:
        assert scope[key][:len(previous_scope[key])] == previous_scope[key]
        assert all("hyperreal_proofs::" + name in report["func-details"] for name in scope[key])
    assert re.search(r"Ran 10 tests in .*\n\nOK\n", (acceptance / "stderr.log").read_text())
    tooling = read(rejection / "proof-checks/tooling.json")
    calls = [json.loads(line) for line in gzip.decompress((rejection / "proof-checks/verus-invocations.jsonl.gz").read_bytes()).splitlines()]
    assert tooling["exit_code"] == 0 and tooling["tool_unchanged"]
    assert tooling["verus_invocations"] == len(calls) == 251
    assert tooling["baseline_successes"] == sum(call["exit_code"] == 0 for call in calls) == 4
    assert tooling["rejected_invocations"] == sum(call["exit_code"] != 0 for call in calls) == 247
    for phase, directory in [(proof, "proof-reports"), (acceptance, "proof-checks")]:
        other = read(phase / directory / "tooling.json")
        assert other["tool_unchanged"] and other["tool_sha256"] == tooling["tool_sha256"]
    guards = Path("crates/hyperreal/verification/test_bound_rejections.py")
    prior_mutations = mutation_lists(baseline_dir / "inputs" / guards)
    current_mutations = mutation_lists(SOURCE / guards)
    for name, count in [("MUTATIONS", 12), ("MODEL_MUTATIONS", 9)]:
        assert current_mutations[name][:len(prior_mutations[name])] == prior_mutations[name]
        assert len(current_mutations[name]) - len(prior_mutations[name]) == count
    representation = Path("crates/hyperreal/src/computable/node/representation.rs")
    old_publication = publication((baseline_dir / "inputs" / representation).read_text())
    new_publication = publication((SOURCE / representation).read_text())
    diagnostic = r"debug_assert(?:_eq)?!\(.*?\);"
    old_checks = re.findall(diagnostic, old_publication, re.S)
    new_checks = re.findall(diagnostic, new_publication, re.S)
    assert old_checks == new_checks and len(old_checks) == 5
    assert old_publication.split("        loop {", 1)[0] == new_publication.split("        loop {", 1)[0]
    suffix = "            let encoded = Self::encode_bound(BoundCache::Valid(combined));"
    assert old_publication[old_publication.index(suffix):] == new_publication[new_publication.index(suffix):]
    diff = []
    for name in sorted(changes):
        original = baseline_dir / "inputs" / name
        diff.extend(difflib.unified_diff(original.read_text().splitlines(True),
            (SOURCE / name).read_text().splitlines(True), fromfile="a/" + name, tofile="b/" + name))
    prior_proof = read(E / "domain-predicate-proof-validation-v1763.json")
    diagnostics = {}
    for name in ["cache-state-merge-generated-code-v1919", "inlined-cache-merge-generated-code-v1921",
                 "outlined-cache-merge-generated-code-v1923"]:
        path = E / name / "summary.json"
        item = read(path)
        assert item["timed_processes"] == 0
        assert all(sha(path.parent / artifact) == digest for artifact, digest in item["artifacts_sha256"].items())
        diagnostics[name] = {"summary_sha256": sha(path), "source_record_sha256": item["source_record_sha256"],
            "binary_bytes": item["binary_bytes"], "functions": item["functions"], "timed_processes": 0}
    final_codegen = read(E / "outlined-cache-merge-generated-code-v1923/summary.json")
    assert final_codegen["source_record_sha256"] == expected
    assert final_codegen["only_merge_outlining_attribute_changed"]
    assert final_codegen["publication_plus_selector_bytes"] == 520
    assert diagnostics["cache-state-merge-generated-code-v1919"]["functions"]["publish"]["bytes"] == 1684
    assert diagnostics["inlined-cache-merge-generated-code-v1921"]["functions"]["publish"]["bytes"] == 1684
    experiments = ["bound-merge-cost-rejection-v1857.json", "bound-merge-static-review-v1917.json",
        "cache-state-merge-baseline-v1918/baseline.json", "cache-merge-inlining-baseline-v1920/baseline.json",
        "cache-merge-outlining-baseline-v1922/baseline.json"]
    assert not read(E / experiments[0])["cost_gate_passed"]
    assert not read(E / experiments[1])["runtime_retained"]
    record = {
        "status": "Fresh conditional production merge proof and all guards pass; runtime extraction remains a proposal",
        "retained_runtime_checkpoint": baseline["runtime_checkpoint"],
        "retained_runtime_checkpoint_sha256": baseline["runtime_checkpoint_sha256"],
        "baseline": str((baseline_dir / "baseline.json").relative_to(E)),
        "baseline_sha256": sha(baseline_dir / "baseline.json"),
        "fresh_verification_phase": proof.name, "fresh_acceptance_phase": acceptance.name,
        "fresh_rejection_phase": rejection.name,
        "source_record": str(source_record.relative_to(E)), "source_record_sha256": expected,
        "source_files_verified": len(sources), "main_guarded_files_verified": len(main_guard),
        "changed_sources_sha256": {name: sources[name] for name in sorted(changes)},
        "proposal_patch": patch.name,
        "verified_units": 651, "errors": 0, "listed_production_contracts": 98,
        "listed_model_theorems": 135, "open_obligation_groups": 13,
        "fresh_acceptance_tests": 10, "fresh_rejected_mutants": 247,
        "fresh_passing_guard_baselines": 4, "reused_rejected_mutants": 0,
        "new_negative_checks": {name: current_mutations[name][len(prior_mutations[name]):] for name in current_mutations},
        "proof_and_guard_inputs_sha256": {name: sha(SOURCE / name) for name in prior_proof["proof_and_guard_inputs_sha256"]},
        "publication_source_comparison": {
            "identical_debug_assertion_macros": len(old_checks),
            "initial_load_and_test_hook_identical": True,
            "encoding_mask_and_CAS_retry_suffix_identical": True,
            "scope": "Source comparisons and review preserve the diagnostics and load/hook/encode/mask/CAS operations. The separate debug match has equivalent branch guards; this comparison does not formally verify the atomic loop or its inputs.",
        },
        "failed_attempts": [],
        "prior_experiments_sha256": {name: sha(E / name) for name in experiments},
        "compile_only_diagnostics": diagnostics,
        "outlined_publication_plus_selector_bytes": 520,
        "codegen_interpretation": "The direct-match and forced-inline variants each produce 1684 publication bytes. Outlining produces 292 wrapper bytes plus 228 selector bytes, compared with 517 publication bytes in retained v1836. The probe executable is 232 bytes larger than that baseline. No timings were measured for these variants; the extra call and complete runtime behavior still require all frozen gates.",
        "semantics": "The actual included BoundCache::merge body implements the frozen total cache-state selection, including Invalid and Unknown states. The outlined helper uses the same proved body in ordinary Rust. Five lemmas prove exact-binade/upper compatibility, compatibility of bounds for one real, merged real denotation, retention of both compatible bounds and preservation of either input's accepted domain certificates. Combining independent sign and magnitude evidence can establish an additional domain certificate.",
        "limits": [
            "Both input certificates must be valid for the same immutable real to obtain mathematical compatibility. Producers, independent real existence, production caller obligations, debug diagnostics and the atomic storage/CAS invariant remain separate open obligations.",
            "The retained runtime is v1836. Only two runtime files change: the pure selection is extracted into bounds.rs and called by the existing publication loop; the other four changed files are proof, guard, scope and documentation inputs. All prior runtime test bodies and full-size fixtures remain byte-identical.",
            "Fresh scalar/consumer/default/debug/build/format/strict Clippy checks, original-cap cost controls, 39 capped phases, 4129 numerical cases, 648 public schedules and receipt/fact audits remain required before retention. Proof coverage alone cannot justify a runtime regression.",
            "All thirteen whole-crate groups, the fixed pre-Verus acceptance baseline, older original cost failures and the retained integer tradeoff remain explicit. Acceptance guard tests do not satisfy the pending acceptance report.",
            "EvaluationBudget excludes aggregate bytes/time and unmetered algebraic helpers. Main/native/full-board admission and all 39 properties, 14 targets and crate future work remain open; no completion status changes.",
        ],
    }
    ledger = ROOT / "docs/properties-status.json"
    original_bytes = ledger.read_bytes()
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == original_bytes
    before = json.loads(original_bytes)
    after = copy.deepcopy(before)
    assert len(after["properties"]) == 39 and len(after["targets"]) == 14
    p4 = next(row for row in after["properties"] if row["id"] == "P4")
    for name in [record["baseline"], out.name, patch.name, proof.name + "/proof-reports/evidence.json", rejection.name + "/proof-checks/tooling.json"]:
        path = str(E.relative_to(ROOT) / name)
        assert path not in p4["evidence"]
        p4["evidence"].append(path)
    update = (
        "The v1924/v1926 cache-state merge proposal passes the full conditional proof target with "
        "651 verified units, zero errors, 98 production contracts and 135 model theorems. "
        "The included production merge and five new lemmas retain both compatible inputs' "
        "sign, zero/nonzero, upper and exact-magnitude certificates and preserve their "
        "accepted domain answers. Validity for one real establishes compatibility; neither "
        "producer validity nor atomic publication is assumed proved by this extraction. "
        "All 247 negative mutations (226 original and 21 merge checks), four positive guard baselines "
        "and ten acceptance/dependency tests pass. Five original debug assertion macros, "
        "the initial load/test hook and encoding/mask/CAS suffix remain identical. "
        "The extraction remains unretained pending the frozen runtime, cost and replay "
        "gates; v1836 remains the retained isolated runtime. All thirteen whole-crate "
        "groups, fixed acceptance baseline, earlier cost failures/tradeoff, main/native/"
        "full-board admission and all property/target statuses remain unchanged."
    )
    update += (
        " The full cache-state selection now includes Invalid inputs, with an independent "
        "negative mutation for losing incoming evidence there. Direct matching and forced "
        "inlining both generated 1684 publication bytes; outlining yields 520 combined wrapper/"
        "selector bytes versus 517 retained bytes. This is static evidence only. The earlier "
        "v1857 timing rejection remains failed, and none of the new sources has a timing pass."
    )
    p4["remaining"] += " " + update
    old = read(E / "bound-merge-static-review-v1917.json")["next_priority"]
    assert after["next_actions"][0].count(old) == 1
    after["next_actions"][0] = after["next_actions"][0].replace(old,
        "The outlined cache-state merge now passes conditional Verus and all mutation/acceptance guards. "
        "Next validate its runtime extraction against unchanged scalar/consumer tests, cost "
        "controls and capped/numerical/public replays before deciding retention. Bound producers, "
        "independent existence, diagnostics and the atomic publication invariant remain open.", 1)
    after["next_actions"][0] += " " + update
    unchanged = copy.deepcopy(after)
    old_p4 = next(row for row in before["properties"] if row["id"] == "P4")
    compare_p4 = next(row for row in unchanged["properties"] if row["id"] == "P4")
    compare_p4["evidence"], compare_p4["remaining"] = old_p4["evidence"], old_p4["remaining"]
    unchanged["next_actions"][0] = before["next_actions"][0]
    assert unchanged == before
    patch.write_text("".join(diff))
    record["proposal_patch_sha256"] = sha(patch)
    subprocess.run(["git", "apply", "--reverse", "--check", str(patch)], cwd=SOURCE, check=True)
    out.write_text(json.dumps(record, indent=2) + "\n")
    ledger.write_text(json.dumps(after, indent=2) + "\n")
    ledger_record.write_text(json.dumps({
        "before_sha256": hashlib.sha256(original_bytes).hexdigest(), "after_sha256": sha(ledger),
        "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
        "all_39_property_and_14_target_requirements_and_statuses_preserved": True,
        "proof_record_sha256": sha(out), "retained_runtime_checkpoint": baseline["runtime_checkpoint"],
        "main_runtime_admitted": False,
    }, indent=2) + "\n")
    print(json.dumps({"verified_units": 651, "negative_mutations": 247,
        "positive_guard_baselines": 4, "acceptance_tests": 10, "runtime_retention_pending": True}))


if __name__ == "__main__":
    main()
