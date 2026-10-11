#!/usr/bin/env python3
"""Retain the scoped source review and carry forward unresolved requirements."""
import copy
import gzip
import hashlib
import json
from pathlib import Path

EVIDENCE = Path(__file__).resolve().parent
SOURCE = EVIDENCE.parents[2] / "target/scalar-evaluation-candidate/source"


def main():
    metadata = json.loads((EVIDENCE / "typed-api-prototype-v267-metadata.json").read_text())
    assert metadata["formal_validation"]["rejected_mutants"] == 213
    previous = json.loads((EVIDENCE / "magnitude-resource-followup-v245.json").read_text())
    record = EVIDENCE / "typed-api-prototype-v250-scaled-denotation-verus/source-hashes.json.gz"
    hashes = json.loads(gzip.decompress(record.read_bytes()))
    descriptions = {
        "src/verified/word.rs": "Existing checked_bit_length_msd and new upper_bit_length_msd bodies and contracts. Full u64 bit-count differences plus signed offsets are computed before narrowing; the new helper distinguishes lower clamping from upper overflow.",
        "src/rational/arithmetic/queries_conversion.rs": "msd_exact_with_offset, msd_upper_with_offset and shared magnitude_with_offset: borrowed bit lengths, aligned magnitude comparison, ceiling carry and the separate checked floor/clamped upper conversions.",
        "src/real/arithmetic/facts.rs": "Scaled structural magnitude maps the helper's binade-preservation flag into exact_msd. A clamped exponent remains only an upper certificate even for a dyadic rational scale.",
        "src/computable/node/scaled_lower_magnitude_tests.rs": "Complete 16-sequence compact-Offset regression, independently valid test-private publications, public rational multiplication, clone facts, CBOR reconstruction and numerical precision rejection controls.",
        "src/computable/node/magnitude_upper_bound_tests.rs": "Updated 84-case scaled_upper_exponents_round_before_the_final_range_check matrix plus zero control. Other matrices retain the previous source review and workload.",
        "verification/bound_denotation.rs": "Existing upper predicate and binade-ordering proof, plus new upper-monotonicity and clamped-exponent certificate lemmas. The second new lemma calls the actual checked helper contract and still requires a valid incoming mathematical upper certificate.",
        "verification/test_rejections.py": "Five new executable defect mutations cover clamped exactness, lower/upper classification, wrong clamp, dropped lower certificate and fabricated upper-overflow certificate. All prior mutation slots remain.",
        "verification/test_bound_rejections.py": "Three new denotation mutations weaken the monotonicity premise, remove the unclamped exactness guard and claim an invalid tighter upper certificate. Existing slots remain.",
        "src/computable/approximation/trig.rs": "Read-only follow-up: cos, cos_rational and small_angle_quotient fine-precision fixed-point loops assume reduced arguments; their coarse sin/cos/sinc/cosc global bounds do not establish fine-loop rounding/truncation contracts outside that range.",
        "src/computable/node/domain_validation.rs": "Read-only follow-up: required-domain classifier, refinement loop and dynamic tangent/cotangent denominator construction. Domain validation certifies function existence; exact deferred poles and private convergence ranges remain the next executable probes.",
    }
    slices = {}
    for relative, description in descriptions.items():
        name = "crates/hyperreal/" + relative
        digest = hashlib.sha256((SOURCE / name).read_bytes()).hexdigest()
        assert hashes[name] == digest
        slices[name] = {"sha256": digest, "scope": description}
    remaining = copy.deepcopy(previous["remaining_source_derived_work"][1:])
    assert len(remaining) == 5
    remaining[0]["evidence_level"] = "Inherited source-derived designs, with the listed kernel/domain slices reread at this checkpoint. No new fine-precision kernel or exact-pole workload is claimed here."
    remaining[2]["action"] = "Connect the actual BigInt endpoint producer and rational bit-length/alignment/ceiling producer to the upper-bound premises; prove incoming graph/domain validity and cache compatibility, then atomic publication loops and memory ownership. The new full-width upper-exponent conversion and its conditional clamping denotation are verified. Preserve all 13 open whole-crate obligation groups and the immutable acceptance gate."
    remaining[3]["evidence_level"] = "Executed matrices are listed in magnitude-regression-coverage-v245.json and scaled-lower-regression-coverage-v267.json; no exhaustive schedule claim."
    report = {
        "status": "Source review of the isolated scaled-view lower-exponent repair; executed validation is recorded separately in typed-api-prototype-v267-state.json",
        "previous_review": "magnitude-resource-followup-v245.json",
        "contract_decision": "p4-contract-decision-v173.json",
        "source_record": str(record.relative_to(EVIDENCE)),
        "source_record_sha256": hashlib.sha256(record.read_bytes()).hexdigest(),
        "reviewed_source_slices": slices,
        "resolved_previous_witness": {
            "area": previous["remaining_source_derived_work"][0]["area"],
            "baseline": "typed-api-prototype-v246-scaled-lower-regression",
            "finding": "12 of the 16 original sequences lose a valid scaled upper fact after a stronger base certificate arrives; the four non-underflowing controls retain their facts.",
            "repair": "Retain i32::MIN as an inexact coarse upper exponent on lower overflow. Leave upper overflow unavailable and exact/floor exponent requests unchanged.",
            "regression_record": "scaled-lower-regression-coverage-v267.json",
            "proof_scope": "96 listed production contracts, 127 model theorems and 638 verification units pass. The new helper contract is directly checked; two conditional lemmas preserve incoming upper certificates and guard exactness. Eight new defects join the previous 205 rejections.",
            "limit": "The publications are test-private, independently valid certificates. Public Real scaling shares their node, but this does not assert that numerical refinement can reach the guarded i32::MIN precision.",
        },
        "remaining_source_derived_work": remaining,
        "scope_preserved": "All 39 properties, 14 targets and crate future work remain in scope. Main runtime and its 49 pre-existing worktree entries stay unchanged. No push or runtime admission is claimed.",
        "unadmitted": True,
    }
    out = EVIDENCE / "scaled-lower-resource-followup-v267.json"
    assert not out.exists()
    out.write_text(json.dumps(report, indent=2) + "\n")
    print("Recorded the executed lower-exponent witness and five remaining work areas.")


if __name__ == "__main__":
    main()
