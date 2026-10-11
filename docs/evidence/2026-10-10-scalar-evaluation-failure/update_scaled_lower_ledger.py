#!/usr/bin/env python3
"""Update only P4 and its next action after the scaled lower-bound checks pass."""
import copy
import hashlib
import json
from pathlib import Path

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[2]


def main():
    metadata = json.loads((EVIDENCE / "typed-api-prototype-v267-metadata.json").read_text())
    assert metadata["test_summaries"]["five_crate"]["passed"] == 5714
    assert metadata["formal_validation"]["rejected_mutants"] == 213
    path = ROOT / "docs/properties-status.json"
    before_bytes = path.read_bytes()
    before = json.loads(before_bytes)
    after = copy.deepcopy(before)
    assert len(after["properties"]) == 39 and len(after["targets"]) == 14
    prop = next(row for row in after["properties"] if row["id"] == "P4")
    assert prop["status"] == "in-progress"
    evidence_prefix = str(EVIDENCE.relative_to(ROOT)) + "/"
    for name in ["scaled-lower-regression-coverage-v267.json", "scaled-lower-resource-followup-v267.json",
                 "typed-api-prototype-v267-state.json"]:
        entry = evidence_prefix + name
        assert entry not in prop["evidence"]
        prop["evidence"].append(entry)
    marker = "The v245 isolated checkpoint defines available magnitude exponents"
    assert prop["remaining"].count(marker) == 1
    assert prop["remaining"].endswith("See magnitude-resource-followup-v245.json and typed-api-prototype-v245-state.json in the scalar evaluation evidence directory.")
    prop["remaining"] = prop["remaining"][:prop["remaining"].index(marker)] + (
        "The v245 isolated checkpoint defines available magnitude exponents as strict upper bounds and repairs the identified carry, closed-endpoint, rational-scaling and merge defects. Its six regressions retain the full 96-addition graph, 740-term domain inputs, 64 exact-rational graphs in two orders, 128 CBOR round-trips, 60 scaled-radical observations, 525 numerical endpoint observations, 240 proof permutations and eight delayed publication schedules. "
        "The v267 checkpoint executes the formerly uncompiled lower-exponent scaled-view witness: 12 of 16 sequences lost a valid upper certificate after stronger base proofs arrived. The repaired upper-exponent conversion retains i32::MIN as a coarse bound and clears exactness when clamped. All 16 sequences now pass, including cloned views, 16 CBOR round-trips and 16 unchanged OutOfRange precision controls; the 84 previous rounding cases plus zero remain. These independently valid test-private proof publications do not establish a public numerical cache schedule at i32::MIN. "
        "Final validation passes 5714 five-crate and 935 default-scalar tests/doctests, with two child tests counted separately in each; 930 debug scalar units, strict Clippy, workspace and 78 included-file formatting checks, 18 capped regressions and all 475 original/precision replays pass. The pinned target verifies 638 units with 96 listed production contracts and 127 model theorems; 213 deliberate defects are rejected and ten acceptance/dependency guard tests pass. "
        "The production exponent conversion and conditional upper-certificate preservation are proved. Actual rational bit-length/comparison rounding, BigInt endpoint production, graph/domain validity and atomic CAS loops remain open, as do all 13 whole-crate proof obligation groups. Private fine-kernel convergence, deferred poles, wider cache/serialization schedules, aggregate budgets, independent native-prototype combination and production admission remain open. Main runtime and all 49 original worktree entries are preserved. See scaled-lower-resource-followup-v267.json and typed-api-prototype-v267-state.json in the scalar evaluation evidence directory."
    )
    after["next_actions"][0] = (
        "Continue the isolated fallible scalar evaluator from typed-api-prototype-v267-state.json before production admission. P4 allows Unknown-to-Certified strengthening and reuse of prior proofs while forbidding contradictions or weaker available facts; the strict-equality experiment remains archived and rejected. "
        "The identified upper-magnitude propagation, scaling, numerical endpoint and monotone merge defects are repaired in the candidate. The lower-exponent scaled-view witness is now confirmed and repaired: 12 baseline losses across 16 valid proof sequences become retained coarse bounds without false exactness. Clones, CBOR reconstruction, unchanged typed precision rejection and all 84 earlier rounding cases remain covered. "
        "Validation passes 5714 five-crate and 935 default-scalar tests/doctests plus two child tests in each, 930 debug scalar units, strict Clippy, workspace and 78 included-file formatting checks, 18 capped regressions and 475 retained replay cases. Fourteen inherited Cargo filename warnings remain unchanged. The pinned target passes 638 units with 96 listed production contracts and 127 model theorems; 213 negative guards and ten acceptance/dependency guard tests pass. All 13 whole-crate proof obligation groups remain open. "
        "Next execute the raw/serialized private-kernel fine-convergence and exact deferred tangent/cotangent pole regressions with independent enclosures and unchanged constructor-valid controls, using scaled-lower-resource-followup-v267.json and domain-resource-followup-v155.json. Continue concurrent queries, serialization and remaining constructor/fact/algebra paths. Local limits still do not form shared arithmetic, proof, coefficient, allocation or time budgets. "
        "Preserve all original failures and workloads, combine independently with the native prototype, and retain full-board report/timing/allocation/size evidence before admission; repeat scalar replays after runtime changes. Main runtime still has the original scalar failure and all 49 pre-existing worktree entries are preserved. All 39 properties, 14 targets, crate future work and full pre-push verification remain required."
    )
    for key in before:
        if key not in {"properties", "next_actions"}:
            assert after[key] == before[key], key
    for old_row, new_row in zip(before["properties"], after["properties"]):
        if old_row["id"] != "P4":
            assert old_row == new_row
        else:
            assert old_row["requirement"] == new_row["requirement"]
    assert after["next_actions"][1:] == before["next_actions"][1:]
    out = EVIDENCE / "scaled-lower-ledger-update-v267.json"
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
