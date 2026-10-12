"""Freeze the retained predicate checkpoint before extracting bound selection."""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / "bound-merge-baseline-v1837"
    assert not out.exists()
    state_path = E / "typed-api-prototype-v1836-state.json"
    assert sha(state_path) == "41c1a789a3705059067a9e1cbaef9f9c3090744f00a5490b65ba7736bca4c627"
    state = read(state_path)
    source = json.loads(gzip.decompress((E / state["source_record"]).read_bytes()))
    assert sha(E / state["source_record"]) == state["source_record_sha256"]
    for name, digest in source.items():
        assert sha(SOURCE / name) == digest, name
    proof = read(E / "domain-predicate-proof-validation-v1763.json")
    paths = set(proof["proof_and_guard_inputs_sha256"])
    paths.update("crates/hyperreal/src/computable/node/" + name for name in [
        "cache_publication_tests.rs", "structural_analysis.rs", "domain_certificates.rs", "domain_validation.rs"])
    for name in sorted(paths):
        target = out / "inputs" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(SOURCE / name, target)
        assert sha(target) == source[name]
    records = ["domain-predicate-retention-v1836.json", "domain-predicate-proof-validation-v1763.json",
        "domain-predicate-cost-protocol-v1773.json", "domain-predicate-cost-review-v1780.json",
        "domain-predicate-replay-protocol-v1781.json", "domain-predicate-public-fact-audit-v1835.json"]
    record = {
        "status": "Baseline frozen before changing the production bound merge or proof inputs",
        "baseline_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "runtime_checkpoint": state_path.name, "runtime_checkpoint_sha256": sha(state_path),
        "source_record": state["source_record"], "source_record_sha256": state["source_record_sha256"],
        "source_files_checked": len(source),
        "baseline_inputs_sha256": {name: sha(SOURCE / name) for name in sorted(paths)},
        "retained_records_sha256": {name: sha(E / name) for name in records},
        "prior_verified_units": 645, "prior_production_contracts": 97,
        "prior_model_theorems": 130, "open_whole_crate_groups": 13,
        "reviewed_source_sections": {
            "representation.rs": "AtomicFacts::publish_bound complete function: snapshot decode, compatibility diagnostics, merge selection, field preservation and weak compare-exchange retry.",
            "bounds.rs": "BoundInfo variants, existing pure contract/specification pattern and all domain predicate branches.",
            "bound_denotation.rs": "Bound/sign/upper/exact-binade semantics and exponent order lemmas; existing conditional producer and domain boundary.",
            "cache_encoding_model.rs": "Existing field-mask, packing and word-update lemmas before initial_cache_word_preserves_all_fields; these do not include the production merge or CAS loop.",
            "cache_publication_tests.rs": "Delayed Unknown publication, independent exact-magnitude and tighter-upper publication tests, plus their separately justified incoming bounds.",
        },
        "intended_change": "Extract the exact bound selection into a pure included BoundInfo::merge body. Keep existing conflict diagnostics at the AtomicFacts publication site and retain the same load/hook/CAS retry ordering and masks. Prove its typed selection contract, denotation and retention of both compatible certificates; prove valid bounds for the same real meet the compatibility premise.",
        "invariants": [
            "Preserve every typed selection, including unavailable exponents, exact flags without exponents, Unknown, zero, sign-only and both publication orders. Debug conflict checks and messages remain; no cache or public query behavior change is intended.",
            "The actual production method must be included by Verus. A copied model is not implementation evidence; incoming producer soundness, independent existence, all atomic interleavings and caller obligations remain open.",
            "All prior 226 negative mutations, four positive guard baselines, ten acceptance/dependency tests, runtime test bodies, full-size fixtures and original process caps remain unchanged. Add meaningful selection and premise mutations for the new contract and lemmas.",
            "No source edits during active validation or measurement. Preserve failed attempts, fork corrections, and never rewrite completed phases or checkpoint-covered artifacts.",
            "Retention requires fresh full positive Verus, all mutation and acceptance guards, scalar/consumer/default/debug/build/format/strict Clippy checks, original-cap cost controls, all 39 capped phases, 4129 numerical cases, 648 public schedules and receipt/fact audits. Proof coverage alone cannot justify a runtime regression.",
            "The retained runtime stays v1836 until all gates pass. Keep older cost failures/tradeoff, the fixed pre-Verus baseline, thirteen whole-crate groups, main/native/full-board admission and all 39 properties, 14 targets and crate future work open where not yet proved.",
        ],
    }
    (out / "baseline.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({"baseline": str(out.relative_to(E)), "source_files_checked": len(source),
                      "frozen_inputs": len(paths), "runtime_checkpoint": state_path.name}))


if __name__ == "__main__":
    main()
