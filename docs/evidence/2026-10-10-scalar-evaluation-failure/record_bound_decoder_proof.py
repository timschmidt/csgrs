"""Record the actual decoder contract, its connected lemmas and fresh guard results."""
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
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def mutations(path):
    groups = {}
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            name = node.targets[0].id
            if name in {"MUTATIONS", "MODEL_MUTATIONS"}:
                groups[name] = ast.literal_eval(node.value)
        elif isinstance(node, ast.AugAssign) and isinstance(node.target, ast.Name):
            if node.target.id in groups:
                assert isinstance(node.op, ast.Add)
                groups[node.target.id] += ast.literal_eval(node.value)
    return groups


def main():
    output = E / "bound-decoder-proof-validation-v2171.json"
    patch = E / "bound-decoder-proposal-v2171.patch"
    ledger_output = E / "bound-decoder-proof-ledger-v2171.json"
    assert not any(p.exists() for p in [output, patch, ledger_output])
    base = E / "bound-decoder-baseline-v2168"
    baseline = read(base / "protocol.json")
    baseline_builds = read(base / "summary.json")
    assert baseline_builds["builds_passed"] == 7 and all(r["matches_historical_executable_bytes"] for r in baseline_builds["results"])
    assert all(sha(Path(r["binary"])) == r["binary_sha256"] for r in baseline_builds["results"])
    proof = E / "typed-api-prototype-v2169-bound-decoder-verus"
    acceptance = E / "typed-api-prototype-v2170-bound-decoder-acceptance"
    rejection = E / "typed-api-prototype-v2171-bound-decoder-rejections"
    focused = read(E / "bound-decoder-focused-validation-v2171.json")
    assert focused["complete"] and len(focused["phases"]) == 2
    expected = "80bb17679a0891ff93eba8ed10b9f63c205f3ccb39e35dbb3e187e23bb0b7162"
    for phase in [proof, acceptance, rejection]:
        result = read(phase / "result.json")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
        assert sha(phase / "source-hashes.json.gz") == expected
    assert re.search(r"Ran 10 tests in .*\n\nOK\n", (acceptance / "stderr.log").read_text())
    source_record = proof / "source-hashes.json.gz"
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    retained = json.loads(gzip.decompress((E / baseline["source_record"]).read_bytes()))
    assert sources.keys() == retained.keys() and len(sources) == 24746
    changed = sorted(n for n in sources if sources[n] != retained[n])
    assert set(changed) == set(baseline["planned_changed_files"]) and len(changed) == 5
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    for n, h in baseline["archived_sources_sha256"].items():
        assert sha(base / "inputs" / n) == h == retained[n]
    main = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert len(main) == 1167 and all(sha(ROOT / n) == h for n, h in main.items())
    representation = "crates/hyperreal/src/computable/node/representation.rs"
    original_body = (base / "inputs" / representation).read_text()
    current_body = (SOURCE / representation).read_text()
    header = '''    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == bound_word(value),
    ))]
    fn decode_bound(value: u64) -> BoundCache {
        #[cfg(verus_keep_ghost)]
        proof! {
            assert(Self::MSD_PRESENT == 16u64) by (bit_vector);
            assert(Self::EXACT_MSD == 32u64) by (bit_vector);
            assert(value & 3u64 <= 3u64) by (bit_vector);
            assert((value >> 2u32) & 3u64 <= 3u64) by (bit_vector);
        }
'''
    new_msd = '''                let msd = if value & Self::MSD_PRESENT != 0 {
                    Some(((value >> Self::MSD_SHIFT) as u32) as i32)
                } else {
                    None
                };'''
    old_msd = '''                let msd = (value & Self::MSD_PRESENT != 0)
                    .then_some(((value >> Self::MSD_SHIFT) as u32) as i32);'''
    assert current_body.count(header) == current_body.count(new_msd) == 1
    reverted = current_body.replace(header, '    #[cfg(not(verus_keep_ghost))]\n    fn decode_bound(value: u64) -> BoundCache {\n', 1).replace(new_msd, old_msd, 1)
    assert reverted == original_body
    old_scope = read(base / "inputs/crates/hyperreal/verification/scope.json")
    scope = read(SOURCE / "crates/hyperreal/verification/scope.json")
    assert len(scope["verified_executable_contracts"]) == 98
    assert len(scope["verified_model_theorems"]) == 135 and len(scope["obligations"]) == 13
    unchanged_scope = copy.deepcopy(scope)
    new_contract = unchanged_scope["verified_executable_contracts"].pop()
    new_theorems = unchanged_scope["verified_model_theorems"][-5:]
    del unchanged_scope["verified_model_theorems"][-5:]
    assert unchanged_scope == old_scope and new_contract == "computable_cache::AtomicFacts::decode_bound"
    old_mutants = mutations(base / "inputs/crates/hyperreal/verification/test_cache_rejections.py")
    new_mutants = mutations(SOURCE / "crates/hyperreal/verification/test_cache_rejections.py")
    assert all(new_mutants[k][:len(v)] == v for k, v in old_mutants.items())
    assert len(new_mutants["MUTATIONS"]) - len(old_mutants["MUTATIONS"]) == 14
    assert len(new_mutants["MODEL_MUTATIONS"]) - len(old_mutants["MODEL_MUTATIONS"]) == 5
    report = read(proof / "proof-reports/verus.json")
    verification = report["verification-results"]
    assert verification["success"] and verification["verified"] == 655 and verification["errors"] == 0
    assert verification["is-verifying-entire-crate"] and not verification["encountered-error"] and not verification["encountered-vir-error"]
    evidence = read(proof / "proof-reports/evidence.json")
    assert evidence["scope"] == scope and "--no-cheating" in evidence["command"]
    for key in ["verified_executable_contracts", "verified_model_theorems"]:
        assert all("hyperreal_proofs::" + n in report["func-details"] for n in scope[key])
    calls = [json.loads(line) for line in gzip.decompress((rejection / "proof-checks/verus-invocations.jsonl.gz").read_bytes()).splitlines()]
    tooling = read(rejection / "proof-checks/tooling.json")
    assert tooling["tool_unchanged"] and tooling["exit_code"] == 0
    assert len(calls) == tooling["verus_invocations"] == 249
    assert sum(r["exit_code"] == 0 for r in calls) == tooling["baseline_successes"] == 4
    assert sum(r["exit_code"] != 0 for r in calls) == tooling["rejected_invocations"] == 245
    for phase, subdir in [(proof, "proof-reports"), (acceptance, "proof-checks")]:
        fresh = read(phase / subdir / "tooling.json")
        assert fresh["tool_unchanged"] and fresh["exit_code"] == 0
        assert fresh["tool_sha256"] == tooling["tool_sha256"]
    chunks = []
    for name in changed:
        chunks.extend(difflib.unified_diff((base / "inputs" / name).read_text().splitlines(True),
            (SOURCE / name).read_text().splitlines(True), fromfile="a/" + name, tofile="b/" + name))
    record = {"status": "Actual all-word decoder and five connected lemmas pass conditional proof and fresh guards; runtime qualification pending",
        "retained_runtime_checkpoint": baseline["retained_runtime_checkpoint"],
        "retained_runtime_checkpoint_sha256": baseline["retained_runtime_checkpoint_sha256"],
        "baseline": "bound-decoder-baseline-v2168/protocol.json", "baseline_sha256": sha(base / "protocol.json"),
        "baseline_builds": "bound-decoder-baseline-v2168/summary.json", "baseline_builds_sha256": sha(base / "summary.json"),
        "source_record": str(source_record.relative_to(E)), "source_record_sha256": expected,
        "changed_sources_sha256": {n: sources[n] for n in changed}, "source_files_verified": len(sources),
        "main_files_verified": len(main), "proposal_patch": patch.name,
        "fresh_verification_phase": proof.name, "fresh_acceptance_phase": acceptance.name,
        "fresh_rejection_phase": rejection.name, "verified_units": 655, "errors": 0,
        "listed_production_contracts": 98, "listed_model_theorems": 135, "open_obligation_groups": 13,
        "new_contract": new_contract, "new_connected_theorems": new_theorems,
        "fresh_acceptance_tests": 10, "fresh_rejected_mutations": 245, "fresh_passing_guard_baselines": 4,
        "reused_rejected_mutations": 0, "new_production_mutations": 14, "new_model_mutations": 5,
        "proof_and_guard_inputs_sha256": {n: sources[n] for n in baseline["proof_and_guard_inputs_sha256"]},
        "implementation": "The actual decode_bound body now has an exact bound_word contract for every u64. Ghost-only masked bounds prove both panic arms unreachable. The only ordinary Rust change is the equivalent pure optional-magnitude if expression; all other representation bytes, including publication and caller bodies, are unchanged.",
        "limits": [
            "The original bool::then_some support probe fails; no assumed standard-library specification was added. The actual production body is included by the pinned verifier. All u64 words include malformed-but-decodable metadata; matching the wire specification does not establish a valid mathematical certificate.",
            "Five lemmas connect actual decoder results to encoder round trips, irrelevant bits and bound/sign/finite-real word updates. Incoming producers, independent existence, pointer ownership, atomic publication, caller ordering and all thirteen whole-crate groups remain open.",
            "All 226 older negative checks remain unchanged and run afresh alongside nineteen decoder checks, with four passing baselines. The registry gains one production contract and five model theorems; no obligation status or fixed acceptance baseline changes.",
            "The seven freshly rebuilt retained probes match their historical executables. These are baseline builds only: no candidate binary comparison, cost or public replay qualification has run. The runtime stays unretained until proportionate broad, compiled-code, cost and replay evidence is complete. Earlier failed gates remain failed.",
        ], "runtime_retained": False, "main_runtime_admitted": False}
    ledger = ROOT / "docs/properties-status.json"
    before_bytes = ledger.read_bytes()
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == before_bytes
    before, after = json.loads(before_bytes), json.loads(before_bytes)
    assert len(after["properties"]) == 39 and len(after["targets"]) == 14
    p4 = next(p for p in after["properties"] if p["id"] == "P4")
    for name in [baseline["expression_support"], record["baseline"], output.name, patch.name]:
        path = str(E.relative_to(ROOT) / name)
        assert path not in p4["evidence"]
        p4["evidence"].append(path)
    update = (
        "The v2167 support probe rejects bool::then_some without an assumed specification. "
        "The v2169-v2171 proposal instead includes the actual decoder with an equivalent pure "
        "if expression. Its contract matches the word specification for every u64, and four "
        "ghost mask assertions discharge the unreachable arms. Five new lemmas connect actual "
        "decoded results to round trips and bound/sign/finite-real field updates. The full "
        "conditional target reports 655 units with zero errors, 98 production contracts and "
        "135 model theorems; ten acceptance tests and all 245 negative mutations plus four "
        "passing baselines pass afresh. All 226 prior negative checks and every other "
        "representation byte remain unchanged. The seven rebuilt retained probes match their "
        "historical executables; candidate compiled-code, runtime, cost and replay gates are "
        "pending. This establishes encoding behavior, not producer truth or atomic publication. "
        "Retained runtime remains v1836 and all thirteen whole-crate groups, fixed acceptance "
        "baseline and main/native/full-board obligations remain open."
    )
    p4["remaining"] += " " + update
    previous = read(E / "scalar-retained-restoration-review-v2166.json")["next_priority"]
    assert after["next_actions"][0].count(previous) == 1
    next_priority = (
        "The actual decoder now passes its all-word conditional proof and fresh guards. Run "
        "the nine unchanged broad commands and rebuild the seven consumer probes for a byte "
        "comparison with the frozen retained baseline. Reuse prior cost receipts only for "
        "identical executables and workloads; changed binaries require fresh original-cap "
        "controls. Complete the remaining capped/numerical/public replay and fact audits "
        "before retention. Preserve all prior failures and keep producer/existence/atomic, "
        "fixed pre-Verus and main/native/full-board obligations open."
    )
    after["next_actions"][0] = after["next_actions"][0].replace(previous, next_priority, 1)
    after["next_actions"][0] += " " + update
    same = copy.deepcopy(after)
    old = next(p for p in before["properties"] if p["id"] == "P4")
    compare = next(p for p in same["properties"] if p["id"] == "P4")
    compare["evidence"], compare["remaining"] = old["evidence"], old["remaining"]
    same["next_actions"][0] = before["next_actions"][0]
    assert same == before
    patch.write_text("".join(chunks))
    subprocess.run(["git", "apply", "--reverse", "--check", str(patch)], cwd=SOURCE, check=True)
    record["proposal_patch_sha256"] = sha(patch)
    record["next_priority"] = next_priority
    output.write_text(json.dumps(record, indent=2) + "\n")
    ledger.write_text(json.dumps(after, indent=2) + "\n")
    ledger_output.write_text(json.dumps({"before_sha256": hashlib.sha256(before_bytes).hexdigest(),
        "after_sha256": sha(ledger), "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
        "all_39_property_and_14_target_requirements_and_statuses_preserved": True,
        "proof_record_sha256": sha(output), "runtime_retained": False}, indent=2) + "\n")
    print(json.dumps({"verified_units": 655, "fresh_rejected_mutations": 245,
        "fresh_passing_baselines": 4, "actual_decoder_contract_verified": True, "runtime_retained": False}))


if __name__ == "__main__":
    main()
