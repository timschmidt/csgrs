#!/usr/bin/env python3
"""Update only P4 and its next action after the private-kernel checks pass."""
import copy
import hashlib
import json
from pathlib import Path

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[2]


def main():
    metadata = json.loads((EVIDENCE / "typed-api-prototype-v300-metadata.json").read_text())
    assert metadata["test_summaries"]["five_crate"]["passed"] == 5721
    assert metadata["formal_validation"]["reused_rejected_mutants"] == 213
    assert metadata["formal_validation"]["new_rejection_invocations"] == 0
    path = ROOT / "docs/properties-status.json"
    before_bytes = path.read_bytes()
    before = json.loads(before_bytes)
    after = copy.deepcopy(before)
    assert len(after["properties"]) == 39 and len(after["targets"]) == 14
    prop = next(row for row in after["properties"] if row["id"] == "P4")
    assert prop["status"] == "in-progress"
    prefix = str(EVIDENCE.relative_to(ROOT)) + "/"
    for name in ["private-kernel-regression-coverage-v300.json", "private-kernel-proof-reuse-v300.json",
                 "private-kernel-resource-followup-v300.json", "typed-api-prototype-v300-state.json"]:
        assert prefix + name not in prop["evidence"]
        prop["evidence"].append(prefix + name)
    marker = "Private fine-kernel convergence, deferred poles, wider cache/serialization schedules"
    assert prop["remaining"].count(marker) == 1
    prop["remaining"] = prop["remaining"][:prop["remaining"].index(marker)] + (
        "The v300 isolated checkpoint executes the inherited private-kernel and exact-pole witnesses. Raw atan/asinh at +/-2 and +/-8 hit CPU caps; generic/rational coarse asinh produced 12 one-unit violations; exact tangent/cotangent poles timed out or returned Exhausted. The repair certifies the half-unit series range, uses existing atan reduction or a stable positive asinh identity, and recognizes bounded rational pi-multiple zero denominators after proving operand existence. Exact zero facts are retained, with observed cancellation preserved. "
        "Seven regressions cover independent directed MPFR enclosures, cloned/shared caches, public controls, CBOR reconstruction, exact and nearby poles, cancellation sweeps, 594 pi-lattice checks and bounded extreme-shift recognition. All 600 moderate raw/public trig cases pass 2400 independent exact-series checks; these passing cases are not credited as a prior numerical defect or arbitrary-range guarantee. "
        "Final validation passes 5721 five-crate and 942 default-scalar tests/doctests, with two child tests counted separately in each; 937 debug scalar units, strict Clippy, workspace and 80 included-file formatting checks, 25 capped tests and all 475 original/precision, 78 private-kernel and 600 moderate trig replays pass. Fourteen inherited Cargo filename warnings remain unchanged. A fresh pinned proof run passes 638 conditional units, and ten acceptance checks pass. The 213 prior negative guards are explicitly reused after all 44 proof/guard inputs, tools and dependency identities match; no new negative invocations or proof of the new transcendental/domain implementations is claimed. "
        "Arbitrary-range private kernels, wider exact-domain recognition, loose-upper-bound planning cost, concurrent/reconstructed cache compatibility, shared budgets, implementation-connected proofs, native-prototype combination and production admission remain open. All thirteen whole-crate proof groups remain open. Main runtime and all 49 original worktree entries are preserved. See private-kernel-resource-followup-v300.json and typed-api-prototype-v300-state.json in the scalar evaluation evidence directory."
    )
    after["next_actions"][0] = (
        "Continue the isolated fallible scalar evaluator from typed-api-prototype-v300-state.json before production admission. P4 permits Unknown-to-Certified strengthening and reuse of prior proofs while forbidding contradictions or weaker available facts; the strict-equality experiment remains rejected. "
        "The identified magnitude and cache defects, inverse-series convergence/coarse-asinh defects and recognized exact tangent/cotangent pole failures are repaired in the candidate. Preserve all original signed 96-addition, 740-term domain, 192000-bit series, concurrent publication and lower-exponent fixtures. "
        "Validation passes 5721 five-crate and 942 default-scalar tests/doctests plus two child tests in each, 937 debug scalar units, strict Clippy, workspace and 80 included-file formatting checks, 25 capped regressions and 1153 retained replay cases. The 600 moderate trig cases include 2400 independent enclosure observations. Fourteen inherited Cargo filename warnings remain unchanged. A fresh proof target passes 638 conditional units; ten acceptance checks pass and 213 negative guards are reused with unchanged proof, mutation, tool and dependency inputs. All thirteen whole-crate proof groups remain open; the new transcendental guards and identities are outside the proof target. "
        "Next extend capped serialized fine-kernel tests to much larger arguments with independent enclosures and unchanged public controls; passing moderate arguments do not establish arbitrary-range work bounds. Cover wider/deeper equivalent poles, composed ln/exp/inverse-hyperbolic planning from loose upper bounds, public concurrent and serialized queries, and remaining constructor/fact/algebra paths. Local limits still do not form shared arithmetic, proof, coefficient, allocation or time budgets. "
        "Use private-kernel-resource-followup-v300.json and the inherited reviews. Preserve original workloads, combine independently with the native prototype, and retain full-board report/timing/allocation/size evidence before admission; repeat scalar replays after runtime changes. Main runtime still has the original scalar failure and all 49 pre-existing worktree entries remain preserved. All 39 properties, 14 targets, crate future work and full pre-push verification remain required."
    )
    for key in before:
        if key not in {"properties", "next_actions"}:
            assert after[key] == before[key], key
    for old, new in zip(before["properties"], after["properties"]):
        if old["id"] != "P4":
            assert old == new
        else:
            assert old["requirement"] == new["requirement"]
    assert after["next_actions"][1:] == before["next_actions"][1:]
    out = EVIDENCE / "private-kernel-ledger-update-v300.json"
    assert not out.exists()
    path.write_text(json.dumps(after, indent=2) + "\n")
    out.write_text(json.dumps({
        "path": str(path.relative_to(ROOT)),
        "before_sha256": hashlib.sha256(before_bytes).hexdigest(),
        "after_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "changed_scope": ["P4 evidence and remaining work", "next_actions[0]"],
        "properties_preserved": 39, "targets_preserved": 14,
        "future_work_unchanged": True, "other_property_rows_unchanged": True,
        "remaining_next_actions_unchanged": True,
    }, indent=2) + "\n")
    print("Updated only P4 and its next action; all 39 properties, 14 targets and future work preserved.")


if __name__ == "__main__":
    main()
