"""Record conditional production proofs without admitting their runtime extraction."""
import ast
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


def mutation_lists(path):
    nodes = []
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id in {"MUTATIONS", "MODEL_MUTATIONS"}
            for target in node.targets
        ):
            nodes.append(node)
        elif isinstance(node, ast.AugAssign) and isinstance(node.target, ast.Name) and node.target.id in {"MUTATIONS", "MODEL_MUTATIONS"}:
            nodes.append(node)
    namespace = {}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), "exec"), namespace)
    return {name: namespace[name] for name in ["MUTATIONS", "MODEL_MUTATIONS"]}


def main():
    out = E / "domain-predicate-proof-validation-v1763.json"
    patch = E / "domain-predicate-proposal-v1763.patch"
    ledger_record = E / "domain-predicate-proof-ledger-v1763.json"
    assert not any(path.exists() for path in [out, patch, ledger_record])
    baseline_dir = E / "domain-predicate-baseline-v1759"
    baseline = read(baseline_dir / "baseline.json")
    proof = E / "typed-api-prototype-v1761-domain-predicate-verus"
    acceptance = E / "typed-api-prototype-v1762-domain-predicate-acceptance"
    rejection = E / "typed-api-prototype-v1763-domain-predicate-rejections"
    failed = E / "typed-api-prototype-v1760-domain-predicate-verus"
    source_record = proof / "source-hashes.json.gz"
    expected = sha(source_record)
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    previous = json.loads(gzip.decompress((E / baseline["source_record"]).read_bytes()))
    assert sources.keys() == previous.keys()
    changes = {name for name in sources if sources[name] != previous[name]}
    assert changes == set(baseline["baseline_inputs_sha256"])
    assert all(sha(SOURCE / name) == digest for name, digest in sources.items())
    main_guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert all(sha(ROOT / name) == digest for name, digest in main_guard.items())
    for phase in [proof, acceptance, rejection]:
        result = read(phase / "result.json")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
        assert sha(phase / "source-hashes.json.gz") == expected
    failed_result = read(failed / "result.json")
    assert failed_result["exit_code"] == 1 and failed_result["source_unchanged"] and failed_result["main_unchanged"]
    assert "marked `open` but not marked `pub`" in (failed / "proof-reports/verus.stderr").read_text()
    evidence = read(proof / "proof-reports/evidence.json")
    verified = read(proof / "proof-reports/verus.json")["verification-results"]
    assert verified["success"] and verified["verified"] == 645 and verified["errors"] == 0
    assert not verified["encountered-error"] and not verified["encountered-vir-error"]
    assert "--no-cheating" in evidence["command"]
    scope = read(SOURCE / "crates/hyperreal/verification/scope.json")
    previous_scope = read(baseline_dir / "inputs/crates/hyperreal/verification/scope.json")
    assert evidence["scope"] == scope and scope["obligations"] == previous_scope["obligations"]
    assert len(scope["verified_executable_contracts"]) == 97
    assert len(scope["verified_model_theorems"]) == 130 and len(scope["obligations"]) == 13
    accepted = (acceptance / "stderr.log").read_text()
    assert re.search(r"Ran 10 tests in .*\n\nOK\n", accepted)
    tooling = read(rejection / "proof-checks/tooling.json")
    calls = [json.loads(line) for line in gzip.decompress((rejection / "proof-checks/verus-invocations.jsonl.gz").read_bytes()).splitlines()]
    assert tooling["exit_code"] == 0 and tooling["tool_unchanged"]
    assert tooling["verus_invocations"] == len(calls) == 230
    assert tooling["baseline_successes"] == sum(call["exit_code"] == 0 for call in calls) == 4
    assert tooling["rejected_invocations"] == sum(call["exit_code"] != 0 for call in calls) == 226
    for phase, directory in [(proof, "proof-reports"), (acceptance, "proof-checks")]:
        other = read(phase / directory / "tooling.json")
        assert other["tool_unchanged"] and other["tool_sha256"] == tooling["tool_sha256"]
    guards = Path("crates/hyperreal/verification/test_bound_rejections.py")
    prior_mutations = mutation_lists(baseline_dir / "inputs" / guards)
    current_mutations = mutation_lists(SOURCE / guards)
    for name, count in [("MUTATIONS", 9), ("MODEL_MUTATIONS", 4)]:
        assert current_mutations[name][:len(prior_mutations[name])] == prior_mutations[name]
        assert len(current_mutations[name]) - len(prior_mutations[name]) == count
    diff = []
    for name in sorted(changes):
        original = baseline_dir / "inputs" / name
        assert sha(original) == baseline["baseline_inputs_sha256"][name]
        diff.extend(difflib.unified_diff(original.read_text().splitlines(True),
                                       (SOURCE / name).read_text().splitlines(True),
                                       fromfile="a/" + name, tofile="b/" + name))
    patch.write_text("".join(diff))
    record = {
        "status": "Fresh conditional production proof and all negative guards pass; runtime extraction remains a proposal pending runtime, replay and cost gates",
        "retained_runtime_checkpoint": baseline["runtime_checkpoint"],
        "retained_runtime_checkpoint_sha256": baseline["runtime_checkpoint_sha256"],
        "baseline": str((baseline_dir / "baseline.json").relative_to(E)),
        "baseline_sha256": sha(baseline_dir / "baseline.json"),
        "fresh_verification_phase": proof.name, "fresh_acceptance_phase": acceptance.name,
        "fresh_rejection_phase": rejection.name,
        "source_record": str(source_record.relative_to(E)), "source_record_sha256": expected,
        "source_files_verified": len(sources), "main_guarded_files_verified": len(main_guard),
        "changed_sources_sha256": {name: sources[name] for name in sorted(changes)},
        "proposal_patch": patch.name, "proposal_patch_sha256": sha(patch),
        "verified_units": 645, "errors": 0, "listed_production_contracts": 97,
        "listed_model_theorems": 130, "open_obligation_groups": 13,
        "fresh_acceptance_tests": 10, "fresh_rejected_mutants": 226,
        "fresh_passing_guard_baselines": 4, "reused_rejected_mutants": 0,
        "new_negative_checks": {name: current_mutations[name][len(prior_mutations[name]):] for name in current_mutations},
        "proof_and_guard_inputs_sha256": {name: sha(SOURCE / name) for name in baseline["unchanged_prior_proof_inputs_sha256"]},
        "failed_attempt": {"phase": failed.name, "exit_code": 1,
                           "reason": "A private spec function incorrectly used open visibility. The next immutable phase removes that annotation; no contract or theorem was weakened."},
        "semantics": "The runtime cached-bound wrapper calls the included BoundInfo::proves_domain body. Its typed contract is exact for every metadata state and requested domain. Three denotation lemmas establish strict unit upper bounds, positive exact lower binades and the accepted predicate's real-domain implication, conditional on a valid bound for an independently established real. False means no certificate from this predicate, not invalidity.",
        "limits": [
            "Independent operand existence, sound incoming bound producers, atomic publication and production caller ordering are premises or separate obligations. This does not prove general graph convergence, all cache interleavings or whole-crate correctness.",
            "All thirteen whole-crate obligation groups and the fixed pre-Verus acceptance baseline remain unchanged. Passing acceptance unit tests does not satisfy the pending acceptance report.",
            "The only runtime source changes are the private enum relocation, pure predicate extraction and wrapper call. All prior runtime test bodies and full-size fixtures remain byte-identical; runtime equivalence, replay and cost checks still must run.",
            "The retained runtime remains v1754. This proposal is not admitted into main, and no runtime regression can be justified solely by added proof coverage. Older original cost guard failures and the prior integer tradeoff remain explicit.",
            "All 39 properties, 14 targets and crate-specific future work remain in scope; no existing completion status changes.",
        ],
    }
    ledger = ROOT / "docs/properties-status.json"
    original_bytes = ledger.read_bytes()
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == original_bytes
    before = json.loads(original_bytes)
    after = copy.deepcopy(before)
    assert len(after["properties"]) == 39 and len(after["targets"]) == 14
    p4 = next(row for row in after["properties"] if row["id"] == "P4")
    for name in [out.name, patch.name, proof.name + "/proof-reports/evidence.json", rejection.name + "/proof-checks/tooling.json"]:
        path = str(E.relative_to(ROOT) / name)
        assert path not in p4["evidence"]
        p4["evidence"].append(path)
    update = (
        "The isolated v1761/v1763 domain-predicate proposal now has a fresh included-production "
        "proof: 645 verified units, zero errors, 97 production contracts and 130 model theorems. "
        "The exact typed predicate contract and three conditional real-domain lemmas preserve "
        "strict upper bounds, probability positivity and exact lower-binade requirements. "
        "All 226 negative mutations (213 existing plus 13 new), four positive guard baselines "
        "and ten acceptance/dependency tests pass. The first private-spec visibility failure "
        "is retained separately. Incoming certificate validity, independent existence, atomic "
        "publication and caller ordering remain unproved by this addition. All thirteen "
        "whole-crate groups and the fixed acceptance baseline remain open. The extraction "
        "is not yet retained: v1754 remains the accepted isolated runtime while unchanged "
        "runtime tests, capped replays, public reuse corpora and cost controls run. Proof "
        "coverage alone cannot justify a runtime regression. Main/native/full-board admission "
        "and all existing property/target statuses remain unchanged."
    )
    p4["remaining"] += " " + update
    old = ("Next bind the cached magnitude-domain predicate to an included production Verus contract, "
           "preserving independent existence, explicit atomic/caller proof obligations and the fixed cost baselines.")
    assert after["next_actions"][0].count(old) == 1
    after["next_actions"][0] = after["next_actions"][0].replace(old,
        "The included magnitude-domain predicate now passes conditional Verus and mutation checks. "
        "Next validate its runtime extraction against the unchanged tests, public/capped replays "
        "and retained cost controls before deciding retention.", 1)
    after["next_actions"][0] += " " + update
    comparison = copy.deepcopy(after)
    old_p4 = next(row for row in before["properties"] if row["id"] == "P4")
    compare_p4 = next(row for row in comparison["properties"] if row["id"] == "P4")
    compare_p4["evidence"], compare_p4["remaining"] = old_p4["evidence"], old_p4["remaining"]
    comparison["next_actions"][0] = before["next_actions"][0]
    assert comparison == before
    out.write_text(json.dumps(record, indent=2) + "\n")
    ledger.write_text(json.dumps(after, indent=2) + "\n")
    ledger_record.write_text(json.dumps({
        "before_sha256": hashlib.sha256(original_bytes).hexdigest(), "after_sha256": sha(ledger),
        "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
        "all_39_property_and_14_target_requirements_and_statuses_preserved": True,
        "proof_record_sha256": sha(out), "retained_runtime_checkpoint": baseline["runtime_checkpoint"],
        "main_runtime_admitted": False,
    }, indent=2) + "\n")
    print(json.dumps({"verified_units": 645, "negative_mutations": 226,
                      "positive_guard_baselines": 4, "acceptance_tests": 10,
                      "runtime_retention_pending": True}))


if __name__ == "__main__":
    main()
