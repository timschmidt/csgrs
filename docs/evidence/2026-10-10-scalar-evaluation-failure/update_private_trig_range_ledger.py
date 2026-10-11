#!/usr/bin/env python3
"""Update only P4 and its next action after trig-range validation completes."""
import copy
import hashlib
import json
from pathlib import Path

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[2]


def main():
    metadata = json.loads((EVIDENCE / "typed-api-prototype-v330-metadata.json").read_text())
    assert metadata["test_summaries"]["five_crate"]["passed"] == 5726
    assert metadata["formal_validation"]["reused_rejected_mutants"] == 213
    assert metadata["formal_validation"]["new_rejection_invocations"] == 0
    assert sum(row["cases"] for row in metadata["final_source_replays"].values()) == 1537
    path = ROOT / "docs/properties-status.json"
    before_bytes = path.read_bytes()
    before = json.loads(before_bytes)
    after = copy.deepcopy(before)
    assert len(after["properties"]) == 39 and len(after["targets"]) == 14
    prop = next(row for row in after["properties"] if row["id"] == "P4")
    assert prop["status"] == "in-progress"
    prefix = str(EVIDENCE.relative_to(ROOT)) + "/"
    for name in ["private-trig-range-regression-coverage-v330.json", "private-trig-range-proof-reuse-v330.json",
                 "private-trig-range-resource-followup-v330.json", "typed-api-prototype-v330-state.json"]:
        assert prefix + name not in prop["evidence"]
        prop["evidence"].append(prefix + name)
    marker = "The v300 isolated checkpoint executes the inherited private-kernel and exact-pole witnesses."
    assert prop["remaining"].count(marker) == 1
    prop["remaining"] = prop["remaining"][:prop["remaining"].index(marker)] + (
        "The v330 isolated checkpoint retains the v300 inverse-series, coarse-asinh and recognized exact-pole repairs and executes the larger serialized trig follow-up. Of 384 baseline cases, 24 serialized +/-1048576 cases hit CPU caps; all public controls completed and all 1440 completed numeric observations were valid. This demonstrates cost failures, not mathematical divergence or numerical error in completed results. "
        "Demand-time sin/cos/sinc/cosc range certificates now route larger inputs through fallible reduction. Checked magnitude arithmetic and optional coefficient expansion address extreme Offset overflow/allocation; fallible negation reduction preserves observed stops. Deferred sin/cos use global coarse bounds after validating operand existence. Five new regressions cover 4752 directed MPFR enclosures, 448 valid cache schedules with 1792 observations, 28 observed-stop fixtures, typed extreme reductions, retained input facts, global coarse values and undefined descendants. "
        "Validation passes 5726 five-crate and 947 default-scalar tests/doctests, with two child tests counted separately in each; 942 debug scalar units, strict Clippy, workspace and 81 included-file formatting checks, 30 capped tests and all 1537 retained replay cases pass. The 600 moderate and 384 wide trig replays include 2400 and 1536 independent enclosure observations respectively. Fourteen inherited Cargo filename warnings remain unchanged. A fresh pinned proof run passes 638 conditional units and ten acceptance checks pass; 213 negative guards are reused after all 44 proof/guard inputs, tools and dependency identities match. No new negative invocations or proof of the new transcendental implementations is claimed. "
        "Optional power-of-two materialization now declines shifts beyond the existing numeric precision ceiling. Scoped tests retain input facts and coarse values, but fine extreme-scale evaluation and preservation of every symbolic simplification remain open admission requirements; compressed exponent/parity certificates should be evaluated. Other private kernels, wider exact-domain recognition, loose-upper-bound planning, concurrent/reconstructed cache compatibility, shared budgets, implementation-connected proofs, native-prototype combination and production admission remain open. All thirteen whole-crate proof groups remain open. Main runtime and all 49 original worktree entries are preserved. See private-trig-range-resource-followup-v330.json and typed-api-prototype-v330-state.json."
    )
    after["next_actions"][0] = (
        "Continue the isolated fallible scalar evaluator from typed-api-prototype-v330-state.json before production admission. P4 permits Unknown-to-Certified strengthening and reuse of prior proofs while forbidding contradictions or weaker available facts; the strict-equality experiment remains rejected. "
        "Retain the earlier magnitude/cache, inverse-series/coarse-asinh and recognized exact-pole repairs plus the new serialized trig range, extreme-offset and observed-stop fixes. Preserve the full signed 96-addition, 740-term domain, 192000-bit series, concurrent publication and lower-exponent fixtures. "
        "Validation passes 5726 five-crate and 947 default-scalar tests/doctests plus two child tests in each, 942 debug scalar units, strict Clippy, workspace and 81 included-file formatting checks, 30 capped regressions and 1537 replay cases. All 384 wide cases now complete with 1536 valid directed-enclosure observations, including the original 24 CPU-capped cases; the 600 moderate cases retain 2400 valid observations. Fourteen inherited Cargo filename warnings remain unchanged. A fresh proof target passes 638 conditional units; ten acceptance checks pass and 213 negative guards are reused with identical proof, mutation, tool and dependency inputs. All thirteen whole-crate proof groups remain open. "
        "Next compare baseline and candidate wide/deep exact pi and dyadic identities, preserving known-finite controls. Optional coefficient expansion limits require compressed exponent/parity certificates wherever symbolic completeness would otherwise weaken; coarse-value and retained-input-sign tests do not prove fine evaluation or all identities. Extend remaining serialized private variants, nested cancellation, composed ln/exp/inverse-hyperbolic planning, public concurrent and reconstructed queries, and constructor/fact/algebra paths. Local caps are not shared arithmetic/proof/coefficient/allocation/time budgets. "
        "Use private-trig-range-resource-followup-v330.json and inherited reviews. Preserve original workloads, combine independently with the native prototype, and retain full-board report/timing/allocation/size evidence before admission; repeat scalar replays after runtime changes. Main runtime still has the original scalar failure and all 49 pre-existing worktree entries remain preserved. All 39 properties, 14 targets, crate future work and full pre-push verification remain required."
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
    out = EVIDENCE / "private-trig-range-ledger-update-v330.json"
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
