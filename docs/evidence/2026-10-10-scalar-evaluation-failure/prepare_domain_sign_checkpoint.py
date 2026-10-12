"""Update only P4 evidence/frontier and prepare the final immutable checkpoint."""
import copy
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
retention = read(E / "domain-sign-retention-v1672.json")
source_hash = retention["source_record_sha256"]
assert source_hash == "f8dbb363116f762ef866231905590de0b21a020a5eedbe3f2d7fcc8d8c8ccd0d"
assert retention["replay_cases"] == 4129
assert len(retention["fresh_cargo_and_format_validations"]) == 10
assert len(retention["fresh_capped_validations"]) == 30
assert read(E / "domain-sign-focused-implementation-v1626.json")["public_completion_before_after"] == [4, 12]

ledger = ROOT / "docs/properties-status.json"
before_bytes = ledger.read_bytes()
assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == before_bytes
before = json.loads(before_bytes)
after = copy.deepcopy(before)
assert len(before["properties"]) == 39 and len(before["targets"]) == 14
p4 = next(row for row in after["properties"] if row["id"] == "P4")
evidence = [
    "log-range-survey-v1583.json", "domain-sign-baseline-v1593.json",
    "domain-sign-publication-correction-v1621.json", "domain-sign-focused-implementation-v1626.json",
    "domain-sign-candidate-validation-v1630.json", "domain-sign-deferred-review-v1630.json",
    "domain-sign-proof-scope-v1630.json", "domain-sign-cost-review-v1639.json",
    "domain-sign-budget-replay-comparison-v1644.json", "domain-sign-remaining-validation-v1671.json",
    "domain-sign-retention-v1672.json", "typed-api-prototype-v1672-state.json",
]
for name in evidence:
    assert name.endswith("state.json") or (E / name).is_file()
    path = str(E.relative_to(ROOT) / name)
    assert path not in p4["evidence"]
    p4["evidence"].append(path)
update = (
    "The v1672 isolated checkpoint retains a domain-sign completeness extension to v1404/v1576. "
    "After independent operand-existence checks, canonical literal or retained signs precede rational "
    "recognition; unreduced rationals keep their metered path. Eight of twelve public serialized sqrt/ln1p "
    "queries that previously exhausted now return valid coarse samples at the same eight-unit allowance. "
    "The first variant lost child proof publication and is rejected; the corrected variant preserves all "
    "twelve wide-leaf zero-work child sign queries. All 88 domain observations pass. The 241 log survey "
    "answers, thirteen unlimited retries and 52 zero-work cache queries preserve the measured contracts. "
    "Fresh validation passes 1065 scalar tests plus one child, 5849 five-crate tests plus three children "
    "with 33 ignored, default/debug tests, strict all-feature and minimal Clippy, formatting, eleven new "
    "capped tests, nineteen retained capped modules (136 tests plus one child), and all 4129 retained "
    "replay cases. The 1584 focused replay receipts differ only in 926 accounting fields: 459 consumption "
    "rows decrease and four increase; all numerical observations and definite facts agree. Fourteen "
    "inherited Cargo example-filename warnings remain unchanged. The 78 private-kernel replay cases "
    "check result shapes. Seven cost families retain exact outputs across 1767 successful processes, "
    "one failed retained-baseline process and 1300 separate CPU preparation processes. The original "
    "fixed-CPU comparison stopped on a baseline SIGXCPU; a separate equal larger-cap algebraic diagnostic "
    "does not replace that failure. A narrow wide-coarse integer control remains 18.76 percent slower "
    "(48.73 to 57.87 ns per call); this lower-rung tradeoff is explicit for the completeness extension, "
    "with no uniform-speedup or main-admission claim. All 44 prior conditional proof inputs are unchanged; "
    "the new runtime domain traversal is outside their proof scope and all thirteen whole-crate obligation "
    "groups remain open. The log setup's shared-constant warmup comment is misleading and must be "
    "corrected before runtime admission; the actual frozen process order is disclosed. EvaluationBudget "
    "does not bound bytes/time or unmetered algebraic helpers. Continue magnitude-domain ordering, "
    "log factoring, public constructor/tower/Laurent paths, concurrent/reconstructed proof reuse, "
    "implementation-connected proofs and unchanged native/full-board admission. Main retains its original "
    "scalar failure and all 49 original worktree entries. All 39 properties, 14 targets and crate future "
    "work remain in scope."
)
p4["remaining"] += " " + update
lead = (
    "Continue the isolated fallible scalar evaluator from typed-api-prototype-v1576-state.json before "
    "production admission. Its retained runtime is byte-identical to v1404; both acos and both atan "
    "allocation variants remain rejected."
)
assert after["next_actions"][0].startswith(lead)
after["next_actions"][0] = after["next_actions"][0].replace(lead,
    "Continue the isolated fallible scalar evaluator from typed-api-prototype-v1672-state.json before "
    "production admission. The domain-sign completeness extension is retained with its measured narrow "
    "integer-control cost; both acos and both atan allocation variants remain rejected.", 1)
old = "numeric atan has now been measured; log-range fallbacks and retained quadratic-tower copies remain source-only candidates."
assert old in after["next_actions"][0]
after["next_actions"][0] = after["next_actions"][0].replace(old,
    "numeric atan and public log-range fallbacks have now been measured; the domain-sign extension is "
    "retained in the isolated candidate, and retained quadratic-tower copies remain a source-review target.")
after["next_actions"][0] += " " + update
comparison = copy.deepcopy(after)
original = next(row for row in before["properties"] if row["id"] == "P4")
row = next(row for row in comparison["properties"] if row["id"] == "P4")
row["evidence"] = original["evidence"]
row["remaining"] = original["remaining"]
comparison["next_actions"][0] = before["next_actions"][0]
assert comparison == before

out = E / "domain-sign-ledger-update-v1672.json"
metadata_path = E / "typed-api-prototype-v1672-metadata.json"
assert not out.exists() and not metadata_path.exists()
ledger.write_text(json.dumps(after, indent=2) + "\n")
out.write_text(json.dumps({
    "before_sha256": hashlib.sha256(before_bytes).hexdigest(), "after_sha256": sha(ledger),
    "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
    "all_39_property_and_14_target_requirements_and_statuses_preserved": True,
    "other_next_actions_and_crate_future_work_preserved": True,
}, indent=2) + "\n")

metadata = {
    "previous_checkpoint": "typed-api-prototype-v1576-state.json",
    "status": retention["status"],
    "contract_decision": "p4-contract-decision-v173.json",
    "accepted_cache_rule": "Warming may strengthen Unknown into Certified and reuse higher-precision proofs at zero work; no contradiction or weakening of an available certificate.",
    "evidence_commits": ["335170220", "11776868d", "65a97a212"],
    "retention_record": "domain-sign-retention-v1672.json",
    "retention_record_sha256": sha(E / "domain-sign-retention-v1672.json"),
    "production_patch": "domain-sign-retaining.patch",
    "production_patch_sha256": sha(E / "domain-sign-retaining.patch"),
    "rejected_first_variant": "domain-sign-publication-correction-v1621.json",
    "fresh_validation_phases": {"cargo_and_format": 10, "separately_verified_capped_binary": 30},
    "cap_record_scope": "The retention record verifies all thirty direct-binary capped phases, their immutable source hashes, limits, exit codes and nested test counts. The checkpoint's Cargo parser receives the ten Cargo/format phases only, because direct-binary runs have no Cargo target headers.",
    "fresh_stack_counts": {"top_level_passed": 5849, "child_process_passed": 3, "ignored": 33},
    "fresh_scalar_counts": {"top_level_passed": 1065, "child_process_passed": 1},
    "fresh_replay_cases": 4129,
    "cost_record": "domain-sign-cost-review-v1639.json",
    "cost_tradeoff": retention["cost_tradeoff"],
    "original_cost_guard_failure_preserved": True,
    "coverage_limits": retention["limits"],
    "conditional_proof_scope": "domain-sign-proof-scope-v1630.json",
    "main_runtime_admitted": False,
    "all_39_properties_14_targets_and_crate_future_work_in_scope": True,
    "remaining_source_review": "domain-sign-deferred-review-v1630.json",
    "remaining_requirements": [
        "Continue bounded-query constructor call-site and lazy/scaled coefficient review without weakening ordinary constructor exactness. Both prior acos and both atan allocation variants remain rejected.",
        "Measure bounded Laurent coefficient-copy ordering without changing the eight-term recognition limit; establish public proof warming before measuring retained quadratic towers and fine-cache misses.",
        "Trace direct Real inverse-trig, radical and tower reachability from public Computable allowance wrappers, keeping serialization-payload diagnostics distinct from native Real API contracts.",
        "Review unshared symbolic growth, construction-time structural comparison, algebraic metadata and vector copies, exact-operation CPU and native memory with unchanged adversarial fixtures.",
        "Continue kernel-specific interruption cutpoints, concurrent/reconstructed and cross-root schedules, implementation-connected proofs and composed ln/exp/inverse-hyperbolic planning.",
        "Investigate available magnitude bounds before unit-interval/at-least-one rational domain recognition, independently proving operand existence and preserving published facts.",
        "Measure smooth-prime logarithm factoring and power-of-two extraction on unchanged full-size public fixtures; EvaluationBudget does not bound this unmetered algebraic work or per-operation bytes/time.",
        "Correct the log survey's misleading shared-constant warmup comment before runtime admission, preserving all original evidence and accurately describing shared global cache state.",
        "The original fixed-CPU baseline cost cap remains a failure. The larger-cap diagnostic is separate; retain the persistent integer-control slowdown when evaluating later combinations and full-board costs.",
        "All thirteen whole-crate proof obligation groups remain open. The unchanged conditional proof target does not establish runtime domain traversal or general concurrent cache correctness.",
        "Native-prototype combination and unchanged full-board admission remain necessary before main runtime changes. Main retains the original scalar failure and all 49 preexisting entries.",
        "All 39 properties, 14 targets and crate future work remain in scope. No property or target completion status changes at this scalar checkpoint."
    ],
}
metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")
print(json.dumps({"ledger_sha256": sha(ledger), "metadata_sha256": sha(metadata_path),
                  "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"]}))
