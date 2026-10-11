#!/usr/bin/env python3
"""Account for fresh positive verification and unchanged negative proof inputs."""
import gzip
import hashlib
import json
from pathlib import Path

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"


def read(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    old = EVIDENCE / "typed-api-prototype-v250-scaled-denotation-verus"
    new = EVIDENCE / "typed-api-prototype-v1101-budget-verus"
    rejection = EVIDENCE / "typed-api-prototype-v251-proof-rejections"
    acceptance = EVIDENCE / "typed-api-prototype-v1102-proof-acceptance-tests"
    for phase in [old, new, rejection, acceptance]:
        result = read(phase / "result.json")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    before = read(old / "proof-reports/evidence.json")
    after = read(new / "proof-reports/evidence.json")
    assert after["verification_results"]["success"]
    assert after["verification_results"]["verified"] == 638
    assert after["verification_results"]["errors"] == 0
    assert before["command"] == after["command"] and "--no-cheating" in after["command"]
    assert before["scope"] == after["scope"]
    assert before["dependencies"] == after["dependencies"]
    old_tools = read(rejection / "proof-checks/tooling.json")
    new_tools = read(new / "proof-reports/tooling.json")
    assert old_tools["tool_sha256"] == new_tools["tool_sha256"]
    assert old_tools["pinned_toolchain"] == new_tools["pinned_toolchain"]
    assert old_tools["tool_unchanged"] and new_tools["tool_unchanged"]
    assert old_tools["verus_invocations"] == 217
    assert old_tools["baseline_successes"] == 4 and old_tools["rejected_invocations"] == 213
    old_sources = json.loads(gzip.decompress((rejection / "source-hashes.json.gz").read_bytes()))
    new_sources = json.loads(gzip.decompress((new / "source-hashes.json.gz").read_bytes()))
    prefix = "crates/hyperreal/"
    # The complete verification tree includes every model, mutation script,
    # toolchain pin and scope declaration. The production entry point imports
    # verified/mod.rs and structural.rs; its two include! fragments are listed
    # explicitly. All verification runners and dependency declarations match.
    proof_inputs = {
        name: value for name, value in new_sources.items()
        if name.startswith(tuple(prefix + p for p in ["verification/", "scripts/", "src/verified/"]))
        or name in {prefix + p for p in ["src/structural.rs", "src/computable/node/bounds.rs",
                                       "src/computable/node/representation.rs", "Cargo.toml", "Cargo.lock"]}
    }
    assert proof_inputs
    for name, value in proof_inputs.items():
        assert old_sources[name] == value and digest(SOURCE / name) == value, name
    changed = {name: value for name, value in new_sources.items() if old_sources.get(name) != value}
    assert set(changed).isdisjoint(proof_inputs)
    assert changed
    assert "Ran 10 tests" in (acceptance / "stderr.log").read_text()
    assert (acceptance / "stderr.log").read_text().rstrip().endswith("OK")
    assert digest(acceptance / "source-hashes.json.gz") == digest(new / "source-hashes.json.gz")
    out = EVIDENCE / "algebraic-preflight-proof-reuse-v1133.json"
    assert not out.exists()
    out.write_text(json.dumps({
        "fresh_verification_phase": new.name,
        "fresh_acceptance_phase": acceptance.name,
        "source_record_sha256": digest(new / "source-hashes.json.gz"),
        "verified_units": 638, "errors": 0,
        "listed_production_contracts": len(after["scope"]["verified_executable_contracts"]),
        "listed_model_theorems": len(after["scope"]["verified_model_theorems"]),
        "open_obligation_groups": len(after["scope"]["obligations"]),
        "reused_rejection_phase": rejection.name,
        "reused_rejected_mutants": 213, "reused_passing_guard_baselines": 4,
        "new_rejection_invocations": 0, "fresh_acceptance_tests": 10,
        "proof_and_guard_inputs_sha256": proof_inputs,
        "toolchain_and_tool_bytes_identical": True,
        "proof_dependency_sources_packages_artifacts_and_arguments_identical": True,
        "changed_sources_outside_proof_target": changed,
        "limits": "The 213 defect rejections are reused from v251 because their entire proof, model, mutation, runner, dependency and tool inputs remain byte-identical. A fresh full positive target and ten acceptance tests pass. The changed runtime/test files listed in this record are outside this conditional proof target; no convergence, transcendental identity, domain-graph or whole-crate proof is claimed. All thirteen whole-crate obligation groups remain open.",
    }, indent=2) + "\n")
    print(json.dumps({"unchanged_proof_and_guard_inputs": len(proof_inputs),
                      "fresh_verified_units": 638, "reused_rejections": 213,
                      "new_rejection_invocations": 0}))


if __name__ == "__main__":
    main()
