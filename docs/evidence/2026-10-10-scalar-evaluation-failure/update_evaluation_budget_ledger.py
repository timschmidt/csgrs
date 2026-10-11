#!/usr/bin/env python3
"""Advance only P4's evidence and next action after budget validation passes."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[2]


def main():
    metadata = json.loads((EVIDENCE / "typed-api-prototype-v504-metadata.json").read_text())
    assert metadata["previous_checkpoint"] == "typed-api-prototype-v436-state.json"
    assert sum(row["cases"] for row in metadata["final_source_replays"].values()) == 2929
    assert sum(row["passed"] for row in metadata["capped_unit_regressions"].values()) == 54
    rust_count = metadata["included_rust_fragments_checked"]
    path = ROOT / "docs/properties-status.json"
    before_bytes = path.read_bytes()
    before = json.loads(before_bytes)
    after = deepcopy(before)
    assert len(after["properties"]) == 39 and len(after["targets"]) == 14
    prop = next(row for row in after["properties"] if row["id"] == "P4")
    assert prop["status"] == "in-progress"
    prefix = str(EVIDENCE.relative_to(ROOT)) + "/"
    for name in ["evaluation-budget-regression-coverage-v504.json", "evaluation-budget-proof-reuse-v504.json",
                 "evaluation-budget-resource-followup-v504.json", "evaluation-budget-cost-comparison-v442.json",
                 "typed-api-prototype-v504-state.json"]:
        assert prefix + name not in prop["evidence"]
        prop["evidence"].append(prefix + name)
    marker = "The v436 isolated checkpoint retains all earlier magnitude/cache"
    assert prop["remaining"].count(marker) == 1
    validation = (
        f"Final validation passes 5750 five-crate and 971 default-scalar tests/doctests plus two separately counted child tests in each, 966 debug scalar units, strict Clippy, workspace and {rust_count} included-file formatting checks, 54 capped tests and all 2929 replay cases. Fourteen inherited Cargo filename warnings remain unchanged. "
        "The fresh pinned proof target passes 638 conditional units and ten acceptance checks; 213 negative guards reuse identical proof/guard inputs, tool bytes and dependency identities. The budget implementation is outside that proof target; thirteen whole-crate groups remain open. "
    )
    prop["remaining"] = prop["remaining"][:prop["remaining"].index(marker)] + (
        "The v504 isolated checkpoint retains all earlier magnitude/cache, inverse-series, coarse-asinh, exact-pole, serialized trig range, compressed lattice, rational-residual and finite non-rational operand repairs. "
        "Caller-owned EvaluationBudget allowances now share cooperative work units across clones, threads and successive queries. Synchronous nested calls inherit the active allowance; nested scopes cannot hide limits. Evaluator, domain, numerical-loop and selected structural visits debit that shared allowance. Resource failure is sticky for one request, preserves completed facts, and cannot poison later proof retries or publish partial root approximations. Periodic atom comparison now reports budget exhaustion as Unknown, while the existing exact comparator retains its semantics. "
        "Seven new tests cover allowance boundaries on independent sqrt(2)+ln(7/5) roots, successive requests, a retained high-precision sign at zero budget, 512-deep structural atoms, negative-cache retry, eight-thread sharing of 97 units, and nested/unwinding scopes. A new 384-case public/serialized replay shares one million units across four precisions, yielding 1536 independently enclosed samples and 384 later zero-budget cache/sign checks. All 2545 earlier replay cases and prior numerical regression tests remain unchanged. The initial release run caught four missing GMP API classifications; the corrected inventory adds work-policy/accounting entries and preserves all assertions. That failed run remains archived. "
        + validation +
        "Six frozen default-API cost workloads preserve checksums and iteration counts; nine repetitions each observed median overhead of about 0.3% to 5.0%. These are local timing observations, not whole-application, allocation or binary-size admission evidence. "
        "The allowance counts cooperative checkpoints, not arbitrary-precision bit work, bytes or elapsed time. Unmetered algebraic helpers and legacy exact structural comparisons can still do substantial work between checkpoints. Complete shared arithmetic/proof/coefficient/allocation/time accounting remains required, together with kernel-specific budget boundary matrices and public concurrent/reconstructed cache schedules. Continue composed ln/exp/inverse-hyperbolic planning, implementation-connected proofs, native-prototype combination and full-board admission evidence. Main runtime and all 49 original worktree entries remain preserved. See evaluation-budget-resource-followup-v504.json and typed-api-prototype-v504-state.json."
    )
    after["next_actions"][0] = (
        "Continue the isolated fallible scalar evaluator from typed-api-prototype-v504-state.json before production admission. P4 allows Unknown-to-Certified strengthening and reuse of prior proofs while forbidding contradictions or weaker available facts; the strict-equality experiment remains rejected. "
        "Retain all earlier magnitude/cache, inverse-series/coarse-asinh, exact-pole, serialized range, observed-stop, compressed lattice, rational-residual and finite non-rational operand repairs. Keep signed 96-addition graphs, 740-term domain fixtures, 192000-bit series, concurrent publication and lower-exponent fixtures, every prior replay case and test source, and the unchanged medium coarse-cache test that caught a prior reducer cycle. "
        "The new caller-owned EvaluationBudget shares cooperative checkpoints across nested/successive queries and threads. Retained proofs and root numeric caches remain reusable at zero budget; interrupted searches cannot create permanent negative caches. Seven unit tests and 384 budgeted public/serialized cases cover basic accounting, retry and fact retention. "
        + validation +
        "Next complete shared resource accounting for unmetered algebraic helpers, legacy structural comparisons, arbitrary-precision coefficient work, allocations and time. The current allowance does not bound these costs. Add kernel-specific budget cutpoints and public concurrent/reconstructed schedules, preserving every available certificate and original-domain obligation. Connect atomic allowances, scope nesting/unwinding, sticky failure and publication to implementation proofs, alongside existing extraction/modular/domain/series obligations. Continue composed ln/exp/inverse-hyperbolic planning under loose upper bounds. "
        "Use evaluation-budget-resource-followup-v504.json and inherited reviews. Six matched micro-workloads preserve checksums and show roughly 0.3% to 5.0% median default-API overhead; whole-application costs, allocations and binary size remain open. Combine independently with the native prototype and retain unchanged full-board report/timing/allocation/size evidence before admission; repeat scalar replays after runtime changes. Main runtime still has the original scalar failure and all 49 pre-existing worktree entries remain preserved. All 39 properties, 14 targets, crate future work and full pre-push verification remain required."
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
    out = EVIDENCE / "evaluation-budget-ledger-update-v504.json"
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
