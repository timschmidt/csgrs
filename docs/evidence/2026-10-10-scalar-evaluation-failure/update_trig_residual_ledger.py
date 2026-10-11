#!/usr/bin/env python3
"""Preserve the complete properties plan while recording residual-angle progress."""
import copy
import hashlib
import json
from pathlib import Path

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[2]


def main():
    metadata = json.loads((EVIDENCE / "typed-api-prototype-v403-metadata.json").read_text())
    assert metadata["test_summaries"]["five_crate"]["passed"] == 5737
    assert metadata["formal_validation"]["reused_rejected_mutants"] == 213
    assert metadata["formal_validation"]["new_rejection_invocations"] == 0
    assert sum(row["cases"] for row in metadata["final_source_replays"].values()) == 2161
    path = ROOT / "docs/properties-status.json"
    before_bytes = path.read_bytes()
    before = json.loads(before_bytes)
    after = copy.deepcopy(before)
    assert len(after["properties"]) == 39 and len(after["targets"]) == 14
    prop = next(row for row in after["properties"] if row["id"] == "P4")
    assert prop["status"] == "in-progress"
    prefix = str(EVIDENCE.relative_to(ROOT)) + "/"
    for name in ["trig-residual-regression-coverage-v403.json", "trig-residual-proof-reuse-v403.json",
                 "trig-residual-resource-followup-v403.json", "typed-api-prototype-v403-state.json"]:
        assert prefix + name not in prop["evidence"]
        prop["evidence"].append(prefix + name)
    marker = "The v365 isolated checkpoint retains all earlier magnitude/cache"
    assert prop["remaining"].count(marker) == 1
    prop["remaining"] = prop["remaining"][:prop["remaining"].index(marker)] + (
        "The v403 isolated checkpoint retains all earlier magnitude/cache, inverse-series, coarse-asinh, exact-pole, serialized trig range and compressed lattice repairs, and extends bounded periodic reduction to non-lattice angles with rational residuals at extreme binary scales. The 456-case baseline produced 36 CPU-limit failures and 896 Exhausted/OutOfRange observations; its 819 returned numeric samples were all valid. "
        "The shared total-arithmetic form keeps bounded coefficients separate from binary exponents, computes modular pi residues and reconstructs bounded rational residuals. Exact quadrant and angle-addition identities produce canonical small pi arguments and dedicated rational trig nodes. An unchanged medium coarse-cache regression exposed a rewrite cycle during validation; dedicated rational nodes removed it, and medium-residual coverage was added without changing any earlier test source. Finite-domain validation, observed-stop propagation and cache publication remain authoritative. "
        "Six new regressions cover 3648 base directed MPFR enclosures, 1183 exact small-form cases, 192 wrapper/cancellation enclosures, 2304 medium-residual enclosures, three fixed points, five Unknown controls, 96 invalid-descendant errors and 12 observed-stop fixtures. Numerical checks preserve existing signs or allow Unknown to strengthen, and verify definite signs against independent enclosures. All 456 baseline cases now pass 1824 numeric observations with identical probe, harness, oracles, parameters, order and caps. "
        "Final validation passes 5737 five-crate and 958 default-scalar tests/doctests plus two separately counted child tests in each, 953 debug scalar units, strict Clippy, workspace and 85 included-file formatting checks, 41 capped tests and all 2161 retained replay cases. Fourteen inherited Cargo filename warnings remain unchanged. The fresh pinned proof target passes 638 conditional units and ten acceptance checks; 213 negative guards are reused after all 44 proof/guard inputs, tool bytes and dependency identities match. The new runtime identity and termination code is outside that target; thirteen whole-crate proof groups remain open. "
        "Unsupported finite residuals/factors, extreme negative scales, oversized coefficients and deeper/wider equivalent forms remain open. The 4096-bit coefficients/residuals, 16 terms, depth 128 and 256 visit/product units are local limits, not shared evaluator budgets. Loose-upper-bound planning, public concurrent/reconstructed cache compatibility, implementation-connected proofs, native-prototype combination and production admission remain open. Main runtime and all 49 original worktree entries are preserved. See trig-residual-resource-followup-v403.json and typed-api-prototype-v403-state.json."
    )
    after["next_actions"][0] = (
        "Continue the isolated fallible scalar evaluator from typed-api-prototype-v403-state.json before production admission. P4 allows Unknown-to-Certified strengthening and reuse of prior proofs while forbidding contradictions or weaker available facts; the strict-equality experiment remains rejected. "
        "Retain all earlier magnitude/cache, inverse-series/coarse-asinh, exact-pole, serialized range, observed-stop and compressed lattice repairs, plus exact periodic reduction for bounded rational residuals. Keep signed 96-addition graphs, 740-term domain fixtures, 192000-bit series, concurrent publication and lower-exponent fixtures, every prior replay case, and the unchanged medium coarse-cache test that caught a reducer cycle. "
        "Final validation passes 5737 five-crate and 958 default-scalar tests/doctests plus two child tests in each, 953 debug scalar units, strict Clippy, workspace and 85 included-file formatting checks, 41 capped regressions and 2161 replay cases. All 456 residual-angle cases pass 1824 independent enclosures under unchanged inputs and caps, including the 36 former CPU-limit failures. Fourteen inherited Cargo filename warnings remain unchanged. A fresh target passes 638 conditional units; ten acceptance checks pass and 213 negative guards reuse identical proof, mutation, tool and dependency inputs. New runtime code is outside this target; thirteen whole-crate proof groups remain open. "
        "Next cover unsupported finite residuals/factors, wider/deeper equivalent forms, other private variants and nested cancellation while preserving all existing recognition paths and available facts. Connect compressed normalization, modular residues, rational reconstruction, trig identities, termination, finite-domain and publication arguments to implementation proofs. Continue composed ln/exp/inverse-hyperbolic planning from loose upper bounds and public concurrent/reconstructed query schedules. Local limits remain separate from shared arithmetic/proof/coefficient/allocation/time budgets. "
        "Use trig-residual-resource-followup-v403.json and inherited reviews. Establish matched cost evidence, combine independently with the native prototype and retain full-board report/timing/allocation/size evidence before admission; repeat scalar replays after runtime changes. Main runtime still has the original scalar failure and all 49 pre-existing worktree entries remain preserved. All 39 properties, 14 targets, crate future work and full pre-push verification remain required."
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
    out = EVIDENCE / "trig-residual-ledger-update-v403.json"
    assert not out.exists()
    path.write_text(json.dumps(after, indent=2) + "\n")
    out.write_text(json.dumps({
        "path": str(path.relative_to(ROOT)),
        "before_sha256": hashlib.sha256(before_bytes).hexdigest(),
        "after_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "changed_scope": ["P4 evidence and remaining work", "next_actions[0]"],
        "properties_preserved": 39, "targets_preserved": 14, "future_work_unchanged": True,
        "other_property_rows_unchanged": True, "remaining_next_actions_unchanged": True,
    }, indent=2) + "\n")
    print("Updated only P4 and its next action; all 39 properties, 14 targets and future work preserved.")


if __name__ == "__main__":
    main()
