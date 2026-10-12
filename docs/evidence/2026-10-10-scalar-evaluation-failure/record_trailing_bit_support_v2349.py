"""Record primitive support and the remaining dependency refinement boundary."""
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())


def main():
    out = E / "trailing-bit-support-milestone-v2349.json"
    assert not out.exists()
    phases = {}
    for name, expected, units in [("trailing-bit-support-v2346", 0, 4),
                                  ("trailing-bit-factorization-v2347", 1, 0),
                                  ("trailing-bit-factorization-v2348", 0, 16)]:
        phase = E / name
        result = read(phase / "result.json")
        proof = read(phase / "stdout.json")["verification-results"]
        assert result["exit_code"] == expected
        assert result["source_unchanged"] and result["main_unchanged"] and result["tooling_unchanged"]
        assert not result["retained_proof_coverage_changed"]
        assert proof["verified"] == units
        assert proof["success"] == (expected == 0)
        assert (phase / "stderr.log").read_text() == "" if expected == 0 else True
        phases[name] = {"result_sha256": sha(phase / "result.json"), "result": result,
                       "verification_results": proof, "protocol_sha256": sha(phase / "protocol.json")}
    previous = E / "typed-api-prototype-v2345-state.json"
    assert sha(previous) == "a75220d56a289d2e83e36a426fb6f1f04b808e684b0889e5d360924408d940b9"
    out.write_text(json.dumps({"status": "Standalone word contracts support the next metadata model; candidate unchanged",
        "runner_sha256": sha(Path(__file__)), "retained_checkpoint": previous.name,
        "retained_checkpoint_sha256": sha(previous), "source_record_sha256": read(previous)["source_record_sha256"],
        "phases": phases, "reader_correction": "trailing-bit-support-reader-failure-v2348.json",
        "reader_correction_sha256": sha(E / "trailing-bit-support-reader-failure-v2348.json"),
        "findings": [
            "The pinned standard-library specification equates trailing ones to trailing zeros of the complement for every u32 and u64. Actual primitive method calls satisfy that specification in the support target.",
            "For each word width, the arithmetic target proves that word + 1, interpreted as an unbounded natural, equals a positive odd quotient times 2 raised to the trailing-one count. The all-ones word is covered without machine-width addition.",
            "The first factorization attempt stopped at three unresolved library names before verifying any units. The corrected target changes only those names, passes 16 verification units across four proof functions and emits no warnings. Both attempts remain intact.",
            "The proof uses existing pinned vstd word contracts and arithmetic lemmas. It adds no external arithmetic method specification or new trusted axiom. These standalone results have not been integrated into the candidate's conditional proof catalog."
        ],
        "dependency_review": "numerical-bound-metadata-review-v2279-v2.json",
        "dependency_review_sha256": sha(E / "numerical-bound-metadata-review-v2279-v2.json"),
        "next_obligations": [
            "Derive the all-ones and power-of-two flags from bit length and positive odd-factor metadata using unbounded arithmetic.",
            "Connect those facts to normalized little-endian digit scans at both 32- and 64-bit widths, including empty storage, all-maximal limbs, the first nonmaximal limb and the first nonzero limb.",
            "Discharge the actual BigUint storage/import and normalization invariant, BigInt sign/magnitude relation, iterator first-match semantics, usize conversion and u64 bit-count range. The existing mathematical limb count is unbounded; it does not prove the dependency's length-times-digit-width arithmetic cannot overflow.",
            "Preserve the producer's early Unknown return and its lazy trailing-zero scan when the exponent is absent. Do not extract metadata in a way that adds eager work.",
            "Integrate any successful model extension with explicit scope, negative controls, current full proof/formatting and proportionate unchanged-input or runtime evidence before retaining it. Then continue approximation truth, caller, independent-existence and atomic publication obligations."
        ],
        "candidate_sources_changed": False, "retained_verified_units": 662,
        "retained_production_contracts": 98, "retained_model_theorems": 137,
        "whole_crate_obligation_groups_open": 13, "fresh_runtime_or_cost_processes": 0,
        "main_runtime_admitted": False,
        "limits": "These are separate support targets with 4 and 16 verification units, including auxiliary assertion obligations; they add no retained production proof coverage. No BigUint method, aggregate scan, normalization invariant, producer caller or concurrent publication is verified here. All earlier resource failures and the full 39-property, 14-target and crate-future-work scope remain unchanged."}, indent=2) + "\n")
    print(json.dumps({"support_targets_passed": 2, "frontend_failure_preserved": 1,
        "candidate_source_changed": False, "retained_units_unchanged": 662}))


if __name__ == "__main__":
    main()
