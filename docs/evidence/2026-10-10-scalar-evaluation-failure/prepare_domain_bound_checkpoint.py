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
retention = read(E / "domain-bound-retention-v1754.json")
source_hash = retention["source_record_sha256"]
assert source_hash == "77f0d0c9d5232b48c71dc1db236e398579486cc4b4b9fb0935ab3d6e4f698eb7"
assert retention["replay_cases"] == 4129
assert len(retention["fresh_cargo_and_format_validations"]) == 10
assert len(retention["fresh_capped_validations"]) == 39
assert retention["public_domain_comparison"]["successful_queries_before_after"] == [415, 453]

ledger = ROOT / "docs/properties-status.json"
before_bytes = ledger.read_bytes()
assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == before_bytes
before = json.loads(before_bytes)
after = copy.deepcopy(before)
assert len(before["properties"]) == 39 and len(before["targets"]) == 14
p4 = next(row for row in after["properties"] if row["id"] == "P4")
evidence = [
    "domain-bound-baseline-v1683.json", "domain-bound-query-comparison-v1693.json",
    "domain-bound-focused-implementation-v1698.json", "domain-bound-proof-scope-v1698.json",
    "domain-bound-broad-validation-v1702.json", "domain-bound-deferred-review-v1702.json",
    "domain-bound-cost-review-v1710.json", "domain-bound-budget-replay-comparison-v1745.json",
    "domain-bound-remaining-validation-v1753.json", "domain-bound-retention-v1754.json",
    "typed-api-prototype-v1754-state.json",
]
for name in evidence:
    assert name.endswith("state.json") or (E / name).is_file()
    path = str(E.relative_to(ROOT) / name)
    assert path not in p4["evidence"]
    p4["evidence"].append(path)
update = (
    "The v1754 isolated checkpoint retains cached magnitude-domain proof reuse after independent "
    "operand-existence checks. Available upper bounds prove strict unit-range membership, and positive "
    "exact binades prove x >= 1 before coefficient recognition. Frozen private proof queries improve "
    "from four to 56 completions at the same allowance, with zero measured Rust allocations on all "
    "56 successful private fast paths. In 1584 public asin/acos/atanh schedules, successful queries "
    "increase from 415 to 453 with no lost earlier successful samples or definite signs. Forty-eight "
    "endpoint/invalid/upper-bound controls, 36 fine retries and 144 zero-work cached queries preserve "
    "their contracts. Pi is explicitly warmed; other shared constants evolve in the frozen process "
    "order. Public-query work decreases in 396 rows and increases in eight, with 434 changed allocation "
    "observations; no universal allocation improvement is claimed. Fresh checks pass 1074 scalar tests "
    "plus one child, 5858 five-crate tests plus three children with 33 ignored, default/debug tests, "
    "strict all-feature and minimal Clippy, formatting including 108 changed Rust files, 39 capped "
    "groups containing 156 tests plus one child, and all 4129 retained replay cases. The separate "
    "1584-case retained receipt comparison changes sixteen accounting fields across eight queries, "
    "each consuming six fewer units; all other observations agree. Fourteen inherited Cargo "
    "example-filename warnings remain unchanged, and the 78 private-kernel replay cases check result "
    "shapes. Cost evidence includes 1602 successful processes, one failed retained-baseline process "
    "and 1135 CPU preparations. The original two-second baseline SIGXCPU remains unresolved; a "
    "separate equal larger-cap algebraic diagnostic does not replace it. The one initial above-five-"
    "percent increase changes from 8.37 to 2.45 percent in the predeclared 27-pair follow-up. Both "
    "observations and the older v1672 integer cost tradeoff remain retained, without a uniform "
    "speedup claim. The earlier log-warmup comment is corrected with all test behavior unchanged. "
    "All 44 conditional proof inputs remain identical; the new runtime domain helper is outside "
    "that target and all thirteen whole-crate obligation groups remain open. EvaluationBudget "
    "excludes bytes, time and unmetered algebraic helpers. Next exercise public concurrent and "
    "reconstructed high-precision proof reuse with independent enclosures and shared spent allowances; "
    "continue constructor/tower/Laurent, log factoring, composed interruption, implementation-connected "
    "proof and unchanged native/full-board admission work. Main retains its original scalar failure "
    "and all 49 preexisting worktree entries. All 39 properties, 14 targets and crate future work remain "
    "in scope; no completion or production-admission status changes."
)
p4["remaining"] += " " + update
lead = (
    "Continue the isolated fallible scalar evaluator from typed-api-prototype-v1672-state.json before "
    "production admission. The domain-sign completeness extension is retained with its measured narrow "
    "integer-control cost; both acos and both atan allocation variants remain rejected."
)
assert after["next_actions"][0].startswith(lead)
after["next_actions"][0] = after["next_actions"][0].replace(lead,
    "Continue the isolated fallible scalar evaluator from typed-api-prototype-v1754-state.json before "
    "production admission. Cached magnitude-domain proof reuse adds 38 public query completions "
    "without losing prior facts. Next prioritize public concurrent and reconstructed high-precision "
    "proof reuse. The earlier domain-sign integer cost tradeoff remains explicit; both acos and both "
    "atan allocation variants remain rejected.", 1)
old = (
    "numeric atan and public log-range fallbacks have now been measured; the domain-sign extension is "
    "retained in the isolated candidate, and retained quadratic-tower copies remain a source-review target."
)
assert old in after["next_actions"][0]
after["next_actions"][0] = after["next_actions"][0].replace(old,
    "numeric atan and public log-range fallbacks have now been measured; domain-sign and cached "
    "magnitude-domain completeness extensions are retained in the isolated candidate, and retained "
    "quadratic-tower copies remain a source-review target.", 1)
after["next_actions"][0] += " " + update
comparison = copy.deepcopy(after)
original = next(row for row in before["properties"] if row["id"] == "P4")
row = next(row for row in comparison["properties"] if row["id"] == "P4")
row["evidence"] = original["evidence"]
row["remaining"] = original["remaining"]
comparison["next_actions"][0] = before["next_actions"][0]
assert comparison == before

out = E / "domain-bound-ledger-update-v1754.json"
metadata_path = E / "typed-api-prototype-v1754-metadata.json"
assert not out.exists() and not metadata_path.exists()
metadata = {
    "previous_checkpoint": "typed-api-prototype-v1672-state.json",
    "status": retention["status"], "contract_decision": "p4-contract-decision-v173.json",
    "accepted_cache_rule": "Warming may strengthen Unknown into Certified and reuse higher-precision proofs at zero work; no contradiction or weakening of an available certificate.",
    "evidence_commits": ["079ddf437", "3f1cfbdac"],
    "retention_record": "domain-bound-retention-v1754.json",
    "retention_record_sha256": sha(E / "domain-bound-retention-v1754.json"),
    "production_patch": "domain-bound-retaining.patch",
    "production_patch_sha256": sha(E / "domain-bound-retaining.patch"),
    "fresh_validation_phases": {"cargo_and_format": 10, "separately_verified_capped_binary": 39},
    "cap_record_scope": "The retention record checks all 39 direct-binary capped phases, source hashes, limits, exit codes and nested test counts. The checkpoint Cargo parser receives the ten Cargo/format phases only; direct-binary logs have no Cargo target headers.",
    "fresh_stack_counts": {"top_level_passed": 5858, "child_process_passed": 3, "ignored": 33},
    "fresh_scalar_counts": {"top_level_passed": 1074, "child_process_passed": 1},
    "fresh_replay_cases": 4129,
    "public_domain_comparison": retention["public_domain_comparison"],
    "private_domain_completions_before_after": [4, 56],
    "cost_record": "domain-bound-cost-review-v1710.json",
    "original_cost_guard_failure_preserved": True,
    "coverage_limits": retention["limits"],
    "conditional_proof_scope": "domain-bound-proof-scope-v1698.json",
    "main_runtime_admitted": False,
    "all_39_properties_14_targets_and_crate_future_work_in_scope": True,
    "remaining_source_review": "domain-bound-deferred-review-v1702.json",
    "remaining_requirements": [
        "Prioritize public concurrent/reconstructed high-precision proof reuse, independent enclosures, retained signs/magnitudes and shared spent allowances. Thread stress is not an exhaustive interleaving proof.",
        "Continue bounded-query constructor and lazy/scaled coefficient review without weakening ordinary constructor exactness; both prior acos and both atan allocation variants remain rejected.",
        "Measure bounded Laurent coefficient-copy ordering without changing the eight-term recognition limit; establish public proof warming before measuring retained quadratic towers and fine-cache misses.",
        "Trace direct Real inverse-trig, radical and tower reachability from public Computable allowance wrappers, keeping serialization-payload diagnostics distinct from native Real API contracts.",
        "Review unshared symbolic growth, construction-time structural comparison, algebraic metadata and vector copies, exact-operation CPU and native memory with unchanged adversarial fixtures.",
        "Continue kernel-specific interruption cutpoints, cross-root schedules, implementation-connected proofs and composed ln/exp/inverse-hyperbolic planning. Rational-returning interval witnesses and separate sign/range consumers remain distinct from the new cached-domain predicate.",
        "Measure smooth-prime logarithm factoring and power-of-two extraction on unchanged full-size public fixtures; EvaluationBudget does not bound this unmetered algebraic work or per-operation bytes/time.",
        "The original fixed-CPU baseline cost failures remain unresolved under their original caps. Separate larger-cap diagnostics cannot replace them; retain the earlier narrow integer tradeoff for later combinations and full-board costs.",
        "All thirteen whole-crate proof obligation groups remain open. The unchanged conditional proof target does not establish runtime domain traversal, the new cached-bound helper or general concurrent cache correctness.",
        "Native-prototype combination and unchanged full-board admission remain necessary before main runtime changes. Main retains its original scalar failure and all 49 preexisting entries.",
        "All 39 properties, 14 targets and crate future work remain in scope. No property or target completion status changes at this scalar checkpoint.",
    ],
}
ledger.write_text(json.dumps(after, indent=2) + "\n")
out.write_text(json.dumps({
    "before_sha256": hashlib.sha256(before_bytes).hexdigest(), "after_sha256": sha(ledger),
    "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
    "all_39_property_and_14_target_requirements_and_statuses_preserved": True,
    "other_next_actions_and_crate_future_work_preserved": True,
}, indent=2) + "\n")
metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")
print(json.dumps({"ledger_sha256": sha(ledger), "metadata_sha256": sha(metadata_path),
                  "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"]}))
