#!/usr/bin/env python3
"""Preserve the complete properties plan while recording finite-operand progress."""
import copy
import hashlib
import json
from pathlib import Path

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[2]


def main():
    metadata = json.loads((EVIDENCE / "typed-api-prototype-v436-metadata.json").read_text())
    assert metadata["test_summaries"]["five_crate"]["passed"] == 5743
    assert metadata["formal_validation"]["reused_rejected_mutants"] == 213
    assert metadata["formal_validation"]["new_rejection_invocations"] == 0
    assert sum(row["cases"] for row in metadata["final_source_replays"].values()) == 2545
    path = ROOT / "docs/properties-status.json"
    before_bytes = path.read_bytes()
    before = json.loads(before_bytes)
    after = copy.deepcopy(before)
    assert len(after["properties"]) == 39 and len(after["targets"]) == 14
    prop = next(row for row in after["properties"] if row["id"] == "P4")
    assert prop["status"] == "in-progress"
    prefix = str(EVIDENCE.relative_to(ROOT)) + "/"
    for name in ["trig-operand-regression-coverage-v436.json", "trig-operand-proof-reuse-v436.json",
                 "trig-operand-resource-followup-v436.json", "typed-api-prototype-v436-state.json"]:
        assert prefix + name not in prop["evidence"]
        prop["evidence"].append(prefix + name)
    marker = "The v403 isolated checkpoint retains all earlier magnitude/cache"
    assert prop["remaining"].count(marker) == 1
    prop["remaining"] = prop["remaining"][:prop["remaining"].index(marker)] + (
        "The v436 isolated checkpoint retains all earlier magnitude/cache, inverse-series, coarse-asinh, exact-pole, serialized trig range, compressed lattice and rational-residual repairs. Periodic reduction now retains finite non-rational operands, including roots, logs, exponentials, inverse trig and nonlinear products. The preserved 384-case baseline had 36 CPU-limit failures and 1056 Exhausted/OutOfRange samples; all 374 returned numeric samples were valid. "
        "A conditional form extracts periodic pi coefficients and retains unsupported atoms, while raw public fallback nodes keep the original graph reachable. Domain validation precedes cancellation and exact-pole publication. Existing exact algebraic constructor branches stay ahead of the new fallback. A direct numerical quotient and guarded pi subtraction feed the bounded sine/cosine series without reconstructing an equivalent trig node. "
        "Six new regressions cover 3072 independent MPFR enclosures across shared/public/CBOR views, 144 nonlinear-atom/wrapper enclosures, 576 finite cancellation/pole checks, 216 invalid-descendant errors, 48 conditional rational-factor enclosures, three existing exact algebraic certificates, six Unknown controls and 12 observed-stop fixtures. Every numerical check retains previous definite signs or permits Unknown to strengthen. All 384 baseline cases now complete 1536 valid numeric observations with unchanged probe, harness, oracles, parameters, order and caps. No prior test source changed. "
        "Final validation passes 5743 five-crate and 964 default-scalar tests/doctests plus two separately counted child tests in each, 959 debug scalar units, strict Clippy, workspace and 88 included-file formatting checks, 47 capped tests and all 2545 replay cases. Fourteen inherited Cargo filename warnings remain unchanged. The fresh pinned proof target passes 638 conditional units and ten acceptance checks; 213 negative guards reuse identical proof/guard inputs, tool bytes and dependency identities. New runtime code is outside that proof target; thirteen whole-crate groups remain open. "
        "Nested periodic operands, wider/deeper forms, extreme negative scales and other private variants remain open. Local coefficient/term/visit limits do not account for structural atom comparison, original domain walks or nested arithmetic; shared arithmetic/proof/coefficient/allocation/time budgets remain required. Continue loose-upper-bound planning, public concurrent/reconstructed cache schedules, implementation-connected proofs, native-prototype combination and full-board admission evidence. Main runtime and all 49 original worktree entries remain preserved. See trig-operand-resource-followup-v436.json and typed-api-prototype-v436-state.json."
    )
    after["next_actions"][0] = (
        "Continue the isolated fallible scalar evaluator from typed-api-prototype-v436-state.json before production admission. P4 allows Unknown-to-Certified strengthening and reuse of prior proofs while forbidding contradictions or weaker available facts; the strict-equality experiment remains rejected. "
        "Retain all earlier magnitude/cache, inverse-series/coarse-asinh, exact-pole, serialized range, observed-stop, compressed lattice and rational-residual repairs, plus conditional periodic reduction with finite non-rational operands. Keep signed 96-addition graphs, 740-term domain fixtures, 192000-bit series, concurrent publication and lower-exponent fixtures, every prior replay case and test source, and the unchanged medium coarse-cache test that caught a prior reducer cycle. "
        "Final validation passes 5743 five-crate and 964 default-scalar tests/doctests plus two child tests in each, 959 debug scalar units, strict Clippy, workspace and 88 included-file formatting checks, 47 capped regressions and 2545 replay cases. All 384 finite-operand periodic cases pass 1536 independent enclosures under unchanged inputs and caps, including 36 former CPU-limit failures. Fourteen inherited Cargo filename warnings remain unchanged. A fresh target passes 638 conditional units; ten acceptance checks pass and 213 negative guards reuse identical proof, mutation, tool and dependency inputs. New runtime code is outside this target; thirteen whole-crate proof groups remain open. "
        "Next prioritize shared resource accounting for structural atom comparisons, original domain walks and nested arithmetic; local extraction limits do not bound these costs. Budget exhaustion must retain available facts and return typed uncertainty without partial publication. Use nested periodic operands, wider/deeper equivalent forms, extreme negative scales, other private variants and concurrent/reconstructed facts as adversarial coverage while preserving every certificate and domain obligation. Connect conditional extraction, compressed modular arithmetic, domain-preserving cancellation, numerical quotient/subtraction, bounded-series error, termination and publication to implementation proofs. Continue composed ln/exp/inverse-hyperbolic planning under loose upper bounds. "
        "Use trig-operand-resource-followup-v436.json and inherited reviews. Establish matched cost evidence, combine independently with the native prototype and retain unchanged full-board report/timing/allocation/size evidence before admission; repeat scalar replays after runtime changes. Main runtime still has the original scalar failure and all 49 pre-existing worktree entries remain preserved. All 39 properties, 14 targets, crate future work and full pre-push verification remain required."
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
    out = EVIDENCE / "trig-operand-ledger-update-v436.json"
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
