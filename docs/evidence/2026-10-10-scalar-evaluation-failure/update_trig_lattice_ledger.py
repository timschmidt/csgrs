#!/usr/bin/env python3
"""Keep the full properties plan while recording the validated lattice repair."""
import copy
import hashlib
import json
from pathlib import Path

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[2]


def main():
    metadata = json.loads((EVIDENCE / "typed-api-prototype-v365-metadata.json").read_text())
    assert metadata["test_summaries"]["five_crate"]["passed"] == 5731
    assert metadata["formal_validation"]["reused_rejected_mutants"] == 213
    assert metadata["formal_validation"]["new_rejection_invocations"] == 0
    assert sum(row["cases"] for row in metadata["final_source_replays"].values()) == 1705
    path = ROOT / "docs/properties-status.json"
    before_bytes = path.read_bytes()
    before = json.loads(before_bytes)
    after = copy.deepcopy(before)
    assert len(after["properties"]) == 39 and len(after["targets"]) == 14
    prop = next(row for row in after["properties"] if row["id"] == "P4")
    assert prop["status"] == "in-progress"
    prefix = str(EVIDENCE.relative_to(ROOT)) + "/"
    for name in ["trig-lattice-regression-coverage-v365.json", "trig-lattice-proof-reuse-v365.json",
                 "trig-lattice-resource-followup-v365.json", "typed-api-prototype-v365-state.json"]:
        assert prefix + name not in prop["evidence"]
        prop["evidence"].append(prefix + name)
    marker = "The v330 isolated checkpoint retains the v300 inverse-series, coarse-asinh and recognized exact-pole repairs"
    assert prop["remaining"].count(marker) == 1
    prop["remaining"] = prop["remaining"][:prop["remaining"].index(marker)] + (
        "The v365 isolated checkpoint retains all earlier magnitude/cache, inverse-series, coarse-asinh, exact-pole and serialized trig range repairs and executes the wide symbolic lattice follow-up. The 168-case baseline produced 14 CPU-limit failures and 296 unexpected Exhausted/OutOfRange samples at exact values or poles; every returned numeric sample was valid. "
        "A bounded sparse form now retains rational coefficients separately from binary exponents, combines equal powers with normalized carries and computes the half-pi residue by modular exponentiation. Recognized total finite expressions yield exact trig values or pole errors; exact sign facts are retained and all existing recognition paths remain as fallbacks. Unsupported partial functions cannot be erased by zero factors. Original coarse-value fixtures accept the exact value's one-unit contract when stronger recognition turns cosine into one. "
        "Five new regressions include 3024 lattice enclosure checks, 1008 typed-pole checks, 504 retained sign checks, 12675 independent integer modular comparisons, 28 carry/cancellation queries, 24 invalid-descendant checks, three Unknown controls, 16 observed-stop fixtures and 192 nearby directed MPFR enclosures. All 168 baseline cases now pass 672 observations under unchanged probe/harness/oracle/case/cap inputs. "
        "Validation passes 5731 five-crate and 952 default-scalar tests/doctests, plus two separately counted child tests in each; 947 debug scalar units, strict Clippy, workspace and 83 included-file formatting checks, 35 capped tests and all 1705 retained replay cases pass. Fourteen inherited Cargo filename warnings remain unchanged. A fresh pinned proof target passes 638 conditional units and ten acceptance checks; 213 negative guards are reused after all 44 proof/guard inputs, tools and dependency identities match. New runtime normalization/modular/domain/publication arguments are not proved by that target; all thirteen whole-crate groups remain open. "
        "General non-lattice periodic reduction at extreme scales, unsupported factors and wider/deeper equivalent forms remain open. Nearby finite controls use exponents 1, 8 and 16, not i32::MAX. The recognizer's 4096-bit coefficients, 16 terms, depth 128 and 256 visit/product units are local limits, not shared evaluator budgets. Loose-upper-bound planning, concurrent/reconstructed cache compatibility, implementation-connected proofs, native-prototype combination and production admission remain open. Main runtime and all 49 original worktree entries are preserved. See trig-lattice-resource-followup-v365.json and typed-api-prototype-v365-state.json."
    )
    after["next_actions"][0] = (
        "Continue the isolated fallible scalar evaluator from typed-api-prototype-v365-state.json before production admission. P4 permits Unknown-to-Certified strengthening and reuse of prior proofs while forbidding contradictions or weaker available facts; the strict-equality experiment remains rejected. "
        "Retain all earlier magnitude/cache, inverse-series/coarse-asinh, recognized pole, serialized trig range and observed-stop repairs, plus compressed half-pi lattice recognition. Preserve signed 96-addition graphs, 740-term domain fixtures, 192000-bit series, concurrent publication and lower-exponent fixtures, and every previous replay case. "
        "Validation passes 5731 five-crate and 952 default-scalar tests/doctests plus two child tests in each, 947 debug scalar units, strict Clippy, workspace and 83 included-file formatting checks, 35 capped regressions and 1705 replay cases. The 168 symbolic lattice cases pass all 672 exact observations, including all original CPU-capped and unexpected resource/range-error cases. Fourteen inherited Cargo filename warnings remain unchanged. A fresh target passes 638 conditional units; ten acceptance checks pass and 213 negative guards reuse identical proof, mutation, tool and dependency inputs. New runtime certificate code is outside the proof target; thirteen whole-crate groups remain open. "
        "Next extend periodic reduction to finite non-lattice residuals at extreme scales with independent controls and unchanged public/serialized workloads. Cover wider/deeper equivalent forms, unsupported factors, other private variants and nested cancellation. Preserve all existing exact recognition paths and available facts. Connect compressed normalization, modular residues, finite-domain and publication arguments to implementation proofs; continue composed ln/exp/inverse-hyperbolic planning from loose upper bounds and public concurrent/reconstructed query schedules. Local limits remain separate from shared arithmetic/proof/coefficient/allocation/time budgets. "
        "Use trig-lattice-resource-followup-v365.json and inherited reviews. Combine independently with the native prototype and retain full-board report/timing/allocation/size evidence before admission; repeat scalar replays after runtime changes. Main runtime still has the original scalar failure and all 49 pre-existing worktree entries remain preserved. All 39 properties, 14 targets, crate future work and full pre-push verification remain required."
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
    out = EVIDENCE / "trig-lattice-ledger-update-v365.json"
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
