#!/usr/bin/env python3
"""Update only P4 and its next action after the magnitude checks pass."""
import copy
import hashlib
import json
from pathlib import Path

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[2]


def main():
    metadata = json.loads((EVIDENCE / "typed-api-prototype-v245-metadata.json").read_text())
    assert metadata["test_summaries"]["five_crate"]["passed"] == 5713
    assert metadata["formal_validation"]["rejected_mutants"] == 205
    path = ROOT / "docs/properties-status.json"
    before_bytes = path.read_bytes()
    before = json.loads(before_bytes)
    after = copy.deepcopy(before)
    assert len(after["properties"]) == 39 and len(after["targets"]) == 14
    prop = next(row for row in after["properties"] if row["id"] == "P4")
    assert prop["status"] == "in-progress"
    evidence_prefix = str(EVIDENCE.relative_to(ROOT)) + "/"
    for name in ["magnitude-regression-coverage-v245.json", "magnitude-resource-followup-v245.json",
                 "typed-api-prototype-v245-state.json"]:
        entry = evidence_prefix + name
        assert entry not in prop["evidence"]
        prop["evidence"].append(entry)
    old = "Public inexact magnitude semantics still need a consistent certificate contract; current Verus denotation constrains only exact MSDs. Private fine-kernel convergence, deferred poles, wider cache/serialization schedules, aggregate budgets and production admission remain open. See publication-resource-followup-v203.json and typed-api-prototype-v213-state.json in the scalar evaluation evidence directory."
    assert prop["remaining"].endswith(old)
    new = "The v245 isolated checkpoint defines available magnitude exponents as strict upper bounds and repairs the identified carry, closed-endpoint, rational-scaling and merge defects. Four retained baseline regressions expose 130 invalid graph facts, 14 weakened and eight missing graph facts, 80 numeric endpoint failures, 28 scaled-radical failures and 100 losses of tighter upper bounds. Six new tests cover 64 exact-rational graphs in two query orders, 128 CBOR round-trips, 60 scaled-radical observations, 525 numerical endpoint observations, 240 proof permutations, 84 final-range rounding cases and eight delayed loose/tighter publication schedules. All original hard workloads remain. Final validation passes 5713 five-crate and 935 default-scalar tests/doctests (two child tests separately counted in each), 929 debug scalar unit tests, strict Clippy, formatting of 76 changed Rust files, 17 resource-capped tests and all 475 original/precision replays. The pinned full proof target passes 635 verification units, with 95 listed production contracts and 125 model theorems; 205 deliberate defects are rejected and ten acceptance/dependency guard tests pass. Inexact magnitude propagation/export is now covered conditionally, and the finite-real cache-word mask is repaired. Incoming graph/domain certificates, BigInt endpoint production, rational rounding and atomic CAS loops are not thereby proved; all 13 whole-crate proof obligation groups stay open. A source-derived lower-exponent scaled-view witness remains unexecuted: tightening the base upper bound may push its transformed exponent below i32::MIN and drop an earlier valid fact. The separate 16-case draft is explicitly uncompiled. That witness, private fine-kernel convergence, deferred poles, wider cache/serialization schedules, aggregate budgets, independent native-prototype combination and production admission remain open. Main runtime and all 49 original worktree entries are preserved. See magnitude-resource-followup-v245.json and typed-api-prototype-v245-state.json in the scalar evaluation evidence directory."
    prop["remaining"] = prop["remaining"][:-len(old)] + new
    after["next_actions"][0] = "Continue the isolated fallible scalar evaluator from typed-api-prototype-v245-state.json before production admission. P4 allows Unknown-to-Certified strengthening and reuse of prior proofs while forbidding contradictions or weaker available facts; the strict-equality experiment remains archived and rejected. The identified upper-magnitude propagation, scaling, numerical endpoint and monotone merge defects are repaired in the candidate. Six new regressions preserve the full 96-addition graph, 740-term domain inputs and all earlier identities, perturbations and publication schedules. Validation passes 5713 five-crate and 935 default-scalar tests/doctests plus two child tests in each, 929 debug scalar units, strict Clippy, workspace and 76 included-file formatting checks, 17 capped regressions and all 475 retained replay cases. Fourteen inherited Cargo filename warnings remain unchanged. The pinned proof target passes 635 units with 95 listed production contracts and 125 model theorems; 205 negative guards and ten acceptance/dependency guard tests pass. These are conditional proof boundaries, with all 13 full-crate obligation groups still open. First execute the lower-exponent scaled-view witness and separate uncompiled 16-case draft in magnitude-resource-followup-v245.json: a stronger base upper bound may make the scaled exponent unavailable while its earlier coarse bound remains valid. Preserve valid facts if that witness is confirmed, without falsely marking a clamped bound exact or reducing the original 84 rounding cases. Continue raw/serialized private-kernel fine convergence, exact deferred tangent/cotangent poles, concurrent queries, serialization and remaining constructor/fact/algebra paths using that review and domain-resource-followup-v155.json. Local limits still do not form shared arithmetic, proof, coefficient, allocation or time budgets. Preserve all original failures and workloads, combine independently with the native prototype, and retain full-board report/timing/allocation/size evidence before admission; repeat scalar replays after runtime changes. Main runtime still has the original scalar failure and all 49 pre-existing worktree entries are preserved. All 39 properties, 14 targets, crate future work and full pre-push verification remain required."
    for key in before:
        if key not in {"properties", "next_actions"}:
            assert after[key] == before[key], key
    for old_row, new_row in zip(before["properties"], after["properties"]):
        if old_row["id"] != "P4":
            assert old_row == new_row
        else:
            assert old_row["requirement"] == new_row["requirement"]
    assert after["next_actions"][1:] == before["next_actions"][1:]
    out = EVIDENCE / "magnitude-ledger-update-v245.json"
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
    print("Updated only P4 evidence/remaining work and next_actions[0]; all plan scope preserved.")


if __name__ == "__main__":
    main()
