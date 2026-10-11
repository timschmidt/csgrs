#!/usr/bin/env python3
"""Retain source-review boundaries separately from executed validation."""
import gzip
import hashlib
import json
from pathlib import Path

EVIDENCE = Path(__file__).resolve().parent
SOURCE = EVIDENCE.parents[2] / "target/scalar-evaluation-candidate/source"


def main():
    record = "typed-api-prototype-v281-kernel-verus/source-hashes.json.gz"
    sources = json.loads(gzip.decompress((EVIDENCE / record).read_bytes()))
    scopes = {
        "src/computable/approximation/inverse_trig.rs": "atan_computable and atan_deferred range checking/reduction, integral +/-1 endpoint handling and the neighboring small rational kernel; no exhaustive inverse-trig audit claimed.",
        "src/computable/approximation/inverse_hyperbolic.rs": "Complete approximation slice, including new asinh_outside_series, both asinh Taylor guards and preserved atanh guards. Larger asinh inputs use a separated sign and positive logarithm identity before coarsening.",
        "src/computable/node/roots_inverse_hyperbolic.rs": "atan_reduced and the new half-unit range helper plus existing one-eighth helper. Half-unit proof accepts certified upper exponents, exact direct leaves or a one-unit rough sample; it does not expand arbitrary graph rationals.",
        "src/computable/node/domain_validation.rs": "Complete required-domain classifier and iterative existence traversal, including the new exact-zero certificate lookup before numeric nonzero refinement and unchanged observed-stop propagation.",
        "src/computable/node/trig_domain.rs": "Complete sufficient pi-multiple zero recognizer. It recognizes integer sine and odd-half-integer cosine zeros, retains wrapper zero identities, and limits traversal and coefficient growth before allocation. Its rational subqueries have their own bounded work, not a shared global budget.",
        "src/computable/node/private_kernel_domain_tests.rs": "All seven tests: directed inverse/coarse/near-pole enclosures, exact-pole errors, clone/public/CBOR views, observed-stop sweeps, 594 exact lattice cases, retained zero facts, bounded extreme-shift recognition and undefined-operand control.",
        "src/computable/node/domain_certificates.rs": "Existence certificate variants and auxiliary sign/rational/unit-interval branches, especially raw tangent and cotangent. Existence remains separate from private convergence-range certification.",
        "src/computable/node/structural_analysis.rs": "exact_sign cache upgrades and iterative direct branches; complete bounded_rational_value recognizer and its coefficient/visit/depth limits. No new whole-file review claimed.",
        "src/rational/arithmetic/queries_conversion.rs": "compare_magnitude used by the half-unit leaf guard: equal-denominator and equal-wide-numerator shortcuts, followed by exact BigUint cross-product comparison. No floating demotion.",
        "src/computable/approximation/trig.rs": "cos, sin, rational variants and small_angle_quotient fixed-point series and error budgets; coarse global bounds remain valid, but arbitrary unreduced fine inputs still need further work evidence.",
        "src/computable/approximation/exp_sqrt.rs": "exp and expm1 demand-time range guards and square-root entry/reduction, reviewed for context. Existing repairs remain unchanged.",
        "src/computable/approximation/logarithms.rs": "ln/ln_rational half-unit series checks and binary-scaled rational combination entry, reviewed for remaining loose-upper-bound planning and resource work. Existing repairs remain unchanged.",
        "verification/lib.rs": "Complete entry point and module/include closure used to delimit the unchanged conditional proof target; new runtime helpers are outside it.",
    }
    reviewed = {}
    for name, scope in scopes.items():
        name = "crates/hyperreal/" + name
        digest = hashlib.sha256((SOURCE / name).read_bytes()).hexdigest()
        assert digest == sources[name]
        reviewed[name] = {"sha256": digest, "scope": scope}
    out = EVIDENCE / "private-kernel-resource-followup-v300.json"
    assert not out.exists()
    out.write_text(json.dumps({
        "status": "Scoped source review; executed coverage belongs to private-kernel-regression-coverage-v300.json and typed-api-prototype-v300-state.json",
        "previous_review": "scaled-lower-resource-followup-v267.json",
        "contract_decision": "p4-contract-decision-v173.json",
        "source_record": record,
        "source_record_sha256": hashlib.sha256((EVIDENCE / record).read_bytes()).hexdigest(),
        "reviewed_source_slices": reviewed,
        "resolved_witnesses": [
            "The original deferred tangent/cotangent exact-pole matrix is now executed and repaired for recognized rational pi multiples, including zero cotangent; nearby valid controls remain.",
            "Raw inverse Taylor inputs at +/-2 and +/-8 hit CPU caps. A half-unit demand-time guard routes them through existing atan reduction or a stable asinh identity.",
            "Raw generic and rational asinh produced 12 invalid coarse zero samples. Coarse evaluation now establishes a value before rescaling outside the proved small interval.",
        ],
        "remaining_source_derived_work": [
            {
                "area": "Full-range private fine kernels",
                "observation": "The 600 moderate trig cases all pass. The fine sin/cos/sinc/cosc loops still have precision-derived term/error budgets and assume reduced arguments, while serialized arguments can have arbitrary magnitude.",
                "action": "Extend capped raw/serialized versus public-control workloads to much larger arguments with independent directed enclosures. Retain all moderate cases. Do not infer numerical failure merely from comments or call bounded test coverage a global guarantee.",
                "evidence_level": "Moderate coverage executed; larger full-range workload is a source-derived next action, not a reproduced failure at this checkpoint.",
            },
            {
                "area": "Exact-domain completeness and total proof work",
                "observation": "The new recognizer deliberately leaves coefficients beyond 4096 bits, extreme inner shifts, excessive traversal and unrecognized algebraic identities unresolved. The exact zero fact is retained when proved. Auxiliary rational/sign searches still use independent budgets.",
                "action": "Test wider/deeper equivalent poles and known-finite controls under aggregate work limits, preserving the 740-term domain fixtures. Expand certificates where justified; Unknown must never become a false validity or invalidity fact.",
                "evidence_level": "Two extreme-shift controls establish bounded recognition only; complete evaluation and global accounting remain open.",
            },
            {
                "area": "Loose upper bounds and planning cost",
                "action": "Retain the full signed 96-addition and 740-term fixtures when evaluating composed ln/exp/inverse-hyperbolic construction and approximation. Upper magnitude bounds cannot prove that a value is large. Measure total work without replacing hard cases by proxies.",
                "evidence_level": "Inherited open source-derived cost follow-up; no new matched performance claim.",
            },
            {
                "area": "Implementation-connected proof and cache compatibility",
                "action": "Connect BigInt endpoint and rational upper-exponent producers, graph/domain validity, publication compatibility and atomic ownership to proofs. Extend public concurrent and serialized query schedules. All thirteen whole-crate proof groups remain open; the new transcendental guards and identities are not verified by the unchanged target.",
                "evidence_level": "638 fresh conditional units and ten acceptance checks pass; reuse of 213 unchanged negative guards is explicitly recorded.",
            },
            {
                "area": "Aggregate resources and runtime admission",
                "action": "Continue shared arithmetic/proof/coefficient/allocation/time accounting, combine the independent native prototype, and retain full-board reports plus proportionate timing/allocation/size evidence before admission. Preserve every original hard case and replay scalar controls after runtime changes.",
                "evidence_level": "Open requirements; scoped caps and unit checks are not matched performance evidence or production admission.",
            },
        ],
        "scope_preserved": "All 39 properties, 14 targets and crate future work remain in scope. Main runtime and all 49 original worktree entries remain unchanged. P4 permits stronger retained facts, never contradictions or weaker available facts.",
        "unadmitted": True,
    }, indent=2) + "\n")
    print(json.dumps({"reviewed_slices": len(reviewed), "unadmitted": True}))


if __name__ == "__main__":
    main()
