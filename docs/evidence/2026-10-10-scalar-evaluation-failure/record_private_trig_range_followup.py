#!/usr/bin/env python3
"""Keep the reviewed source slices and unproved boundaries explicit."""
import gzip
import hashlib
import json
from pathlib import Path

EVIDENCE = Path(__file__).resolve().parent
SOURCE = EVIDENCE.parents[2] / "target/scalar-evaluation-candidate/source"


def main():
    record = "typed-api-prototype-v309-trig-range-verus/source-hashes.json.gz"
    sources = json.loads(gzip.decompress((EVIDENCE / record).read_bytes()))
    scopes = {
        "src/computable/approximation/trig.rs": "Generic/rational sine and cosine series entry guards, the complete small_angle_quotient kernel and quotient fallback, and their retained error-budget comments. No whole-file review or universal error proof claimed.",
        "src/computable/approximation/dispatch.rs": "Deferred sine/cosine/tangent/log dispatch and the new globally bounded coarse sin/cos shortcut. The dispatcher checks cancellation before the shortcut.",
        "src/computable/node/exp_trig.rs": "Checked literal/Offset magnitude planning, demand-time small sin/cos certificate helper, existing bounded pi magnitude recognizer, and public/fallible sine/cosine reducers. Nested Negate keeps the same signal and propagates typed failures. Tangent's neighboring public-recursive branches were read as a remaining cancellation-review boundary.",
        "src/computable/node/structural_analysis.rs": "Optional power-of-two coefficient materialization, shared_constant_term and integer_pi_plus_rational. The new pre-allocation limit avoids MIN negation and huge shifts but is not a proof of completeness of oversized symbolic forms.",
        "src/computable/node/algebra.rs": "Every changed optional power-of-two caller: Laurent, atan/acos, retained pi coefficients, canonical sine coefficients and bounded rational offset forms. Read the retained pi-multiple and canonical sine branches in context; not a new whole-file audit.",
        "src/computable/node/exp_relation.rs": "Changed guarded Offset call only; existing exponent-magnitude limit 1023 remains tighter than the shared materializer ceiling.",
        "src/computable/node/approximation_queries.rs": "approx_signal entry, stop/precision checks, finite-real validation before numeric cache reuse, and the direct kernel dispatch/publication slice. This justifies the coarse deferred shortcut's operand-existence precondition.",
        "src/computable/node/domain_validation.rs": "RequiredRealDomain predicates, exact/retained domain checks and iterative child-before-parent existence traversal through all deferred sin/cos nodes. New coarse results do not bypass invalid descendant validation.",
        "src/computable/node/private_trig_range_tests.rs": "Complete five-test file: 4752 directed enclosures, 448 valid cache-rounding schedules, 28 observed-stop fixtures, four extreme reduction controls, eight retained-fact inputs, 32 global coarse controls and two undefined-descendant controls.",
        "src/computable/node/tests.rs": "Only the pre-existing bounded fixture's updated unwrap at the now-optional power_of_two_rational helper; no new whole-file reading claimed.",
    }
    reviewed = {}
    for name, scope in scopes.items():
        name = "crates/hyperreal/" + name
        digest = hashlib.sha256((SOURCE / name).read_bytes()).hexdigest()
        assert digest == sources[name]
        reviewed[name] = {"sha256": digest, "scope": scope}
    out = EVIDENCE / "private-trig-range-resource-followup-v330.json"
    assert not out.exists()
    out.write_text(json.dumps({
        "status": "Scoped source review; execution counts belong to private-trig-range-regression-coverage-v330.json and typed-api-prototype-v330-state.json",
        "previous_review": "private-kernel-resource-followup-v300.json",
        "contract_decision": "p4-contract-decision-v173.json",
        "source_record": record,
        "source_record_sha256": hashlib.sha256((EVIDENCE / record).read_bytes()).hexdigest(),
        "reviewed_source_slices": reviewed,
        "resolved_witnesses": [
            "The larger serialized trig workload reproduces 24 CPU-limit failures at +/-1048576; all completed baseline samples and all public controls were valid. Demand-time range guards route unreduced inputs to existing fallible reduction.",
            "Extreme Offset construction exposed unchecked MSD addition, then MIN negation and enormous coefficient allocation. Checked planning and bounded optional materialization keep the tested paths typed.",
            "A stop observed inside a negated Add was swallowed by public sine fallback. Nested sine/cosine negation now remains fallible; the entire 28-fixture stop sweep is retained.",
        ],
        "remaining_source_derived_work": [
            {
                "area": "Compressed certificates for wide and deep symbolic forms",
                "observation": "Optional power-of-two expansion now declines shifts larger than MAX_APPROXIMATION_BITS. Exact Offset graphs and observed input signs survive, but recognizers returning None are not evidence that every old symbolic simplification remains available. The existing exact-pole recognizer has separate 4096-bit/depth/work limits.",
                "action": "Compare baseline and candidate wide/deep exact pi and dyadic identities under unchanged workloads. Prefer compressed exponent/parity certificates when they can preserve exact facts without coefficient expansion. Retain known-finite and invalid-descendant controls; test fine queries separately from coarse global bounds.",
                "evidence_level": "Scoped retained-fact/coarse/typed controls passed. Universal symbolic completeness and fine evaluation at extreme scales are open, not claimed repaired.",
            },
            {
                "area": "Remaining private kernels and nested cancellation",
                "observation": "Six sin/cos/sinc/cosc families now have independent finite-matrix enclosures and range guards. Other private variants and nested public constructors, including neighboring tangent reductions, have not acquired exhaustive stop or domain coverage.",
                "action": "Continue serialized/public controls, all observed-stop prefixes and independent enclosures at domain and reduction boundaries. Preserve every earlier case and avoid inferring numeric failure from a resource timeout or source comment.",
                "evidence_level": "984 moderate/wide trig replay cases and the new unit matrices are scoped tests, not an all-kernel theorem.",
            },
            {
                "area": "Loose upper bounds and aggregate work",
                "observation": "Upper bounds can certify smallness but cannot prove largeness. The new materializer ceiling applies to an individual exponent expansion, not composed coefficient products or aggregate graph work.",
                "action": "Retain signed 96-addition graphs, 740-term domain fixtures and 192000-bit series while measuring composed ln/exp/inverse-hyperbolic planning. Connect arithmetic, proof, coefficient, allocation and time accounting across nested operations.",
                "evidence_level": "Inherited open cost and accounting work; local caps are not matched performance or global budget evidence.",
            },
            {
                "area": "Implementation-connected proof and cache compatibility",
                "action": "Connect BigInt and rational bound producers, graph/domain validity, publication compatibility, range-error arguments and atomic ownership to proofs. Extend public concurrent and reconstructed query schedules. Preserve Unknown-to-Certified strengthening and all reusable prior proofs.",
                "evidence_level": "638 conditional units and ten acceptance tests are fresh; 213 negative guards reuse identical inputs. New runtime guard/reduction paths remain outside the target; thirteen whole-crate groups stay open.",
            },
            {
                "area": "Runtime admission and full plan scope",
                "action": "Combine the independently evaluated native prototype and retain full-board reports plus proportionate timing/allocation/size evidence before admission. Replay scalar controls after runtime edits. Main runtime's original failure remains until a candidate is admitted.",
                "evidence_level": "Candidate only. All 39 properties, 14 targets and crate future work remain required; no push or full pre-push validation claim.",
            },
        ],
        "scope_preserved": "P4 permits stronger retained facts, never contradictions or weakened available facts. Main runtime and all 49 original worktree entries are unchanged.",
        "unadmitted": True,
    }, indent=2) + "\n")
    print(json.dumps({"reviewed_slices": len(reviewed), "unadmitted": True}))


if __name__ == "__main__":
    main()
