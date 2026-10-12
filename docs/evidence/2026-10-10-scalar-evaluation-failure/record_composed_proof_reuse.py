"""Audit cross-root fact retention and retain the unchanged-runtime corpus."""
from collections import Counter
import copy
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
PHASE = E / "typed-api-prototype-v1757-composed-proof-reuse"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
summary = read(PHASE / "summary.json")
protocol = read(PHASE / "protocol.json")
rows = read(PHASE / "receipts.json")
assert summary["cases"] == len(rows) == len(protocol["cases"]) == 522
assert summary["cases_by_mode"] == protocol["cases_by_mode"] == {"composed": 468, "invalid": 54}
assert summary["guards_unchanged"] and summary["incomplete_cases"] == summary["invalid_samples"] == 0
assert all(sha(PHASE / name) == digest for name, digest in summary["artifacts_sha256"].items())
assert sha(E / "composed-proof-reuse-protocol-v1757.json") == summary["protocol_sha256"]
assert sha(E / protocol["probe"]) == sha(PHASE / "probe.rs") == protocol["probe_sha256"]
assert sha(E / "reproduce_composed_proof_reuse.py") == sha(PHASE / "reproduce.py") == protocol["runner_sha256"]
assert all(sha(Path(name)) == digest for name, digest in summary["libraries_sha256"].items())
assert all(row["exit_code"] == 0 and not row["wall_timeout"] for row in read(PHASE / "commands.json"))
source_record = E / "typed-api-prototype-v1696-domain-bound-release-build/source-hashes.json.gz"
assert sha(source_record) == summary["source_hashes_sha256"] == protocol["source_record_sha256"]
sources = json.loads(gzip.decompress(source_record.read_bytes()))
guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
assert all(sha(SOURCE / name) == digest for name, digest in sources.items())
assert all(sha(ROOT / name) == digest for name, digest in guard.items())


def retains(before, after):
    if before["sign"] is not None:
        assert after["sign"] == before["sign"]
    if before["zero"] != "Unknown":
        assert after["zero"] == before["zero"]
    if before["exact_rational"]:
        assert after["exact_rational"]
    old, new = before["magnitude"], after["magnitude"]
    if old is not None:
        assert new is not None and new["msd"] <= old["msd"]
        if old["exact_msd"]:
            assert new == old


counts = Counter()
by_root = {name: Counter() for name in protocol["roots"]}
by_units = {}
for row, arguments in zip(rows, protocol["cases"]):
    assert row["arguments"] == arguments and row["complete"]
    command = row["command"]
    assert command["exit_code"] == 0 and not command["wall_timeout"]
    assert command["address_space_cap_mib"] == 192
    assert command["cpu_seconds"] == [2, 3] and command["timeout_seconds"] == 4
    assert sha(Path(command["command"][0])) == summary["binary_sha256"]
    assert read(PHASE / (row["name"] + ".stdout")) == row["records"][0]
    assert (PHASE / (row["name"] + ".stderr")).read_text() == ""
    record = row["records"][0]
    mode, result = record["mode"], record["result"]
    assert result["serialized_expressions_unchanged"]
    counts[mode + "_cases"] += 1
    if mode == "invalid":
        assert len(result["rows"]) == 7 and result["consumed"] <= result["units"]
        errors = {"Err(DivideByZero)", "Err(SqrtNegative)", "Err(NotANumber)"}
        for event in result["rows"]:
            assert event["bounded"] in errors | {"Err(Exhausted)"} and event["retry"] in errors
            counts["invalid_bounded:" + event["bounded"]] += 1
            counts["invalid_retry:" + event["retry"]] += 1
        continue
    assert result["root_names"] == protocol["roots"]
    assert len(result["events"]) == 49
    assert result["consumed_after_drain"] == result["units"]
    assert result["consumed_before_drain"] <= result["units"]
    assert result["base_child_shared_with_roots"] == (not result["reconstructed"])
    prior = copy.deepcopy(result["initial_facts"])
    child_prior = None
    for event in result["events"]:
        index, stage = event["root"], event["stage"]
        counts["events:" + stage] += 1
        approximation = event["retry"] if stage == "finer_retry" else event["approximation"]
        if prior[index]["sign"] is not None or ("sample" in approximation and abs(int(approximation["sample"])) > 1):
            assert event["sign"]["sign"] == result["oracles"][index]["expected_sign"]
            counts["available_sign_query_checks"] += 1
        if stage == "bounded":
            answer = "Ok" if "sample" in approximation else approximation["error"]
            assert answer in {"Ok", "Exhausted"}
            counts["bounded:" + answer] += 1
            by_root[result["root_names"][index]][answer] += 1
            by_units.setdefault(result["units"], Counter())[answer] += 1
            assert 0 <= event["consumed_before"] <= event["consumed_after_sample"] <= event["consumed_after_sign"] <= result["units"]
        if stage == "finer_retry":
            counts["finer:" + ("Ok" if "sample" in event["finer_attempt"] else event["finer_attempt"]["error"])] += 1
            assert all("sample" in event[key] for key in ["retained_sample", "retry", "cached_retry"])
        if stage in {"unlimited_retry", "cached"}:
            assert "sample" in approximation
        if "all_facts" in event:
            assert len(event["all_facts"]) == 7
            for before, after in zip(prior, event["all_facts"]):
                retains(before, after)
                counts["root_fact_retention_comparisons"] += 1
            prior = event["all_facts"]
        else:
            retains(prior[index], event["facts"])
            prior[index] = event["facts"]
            counts["root_fact_retention_comparisons"] += 1
        if "child_facts" in event:
            if child_prior is not None:
                retains(child_prior, event["child_facts"])
                counts["child_fact_retention_comparisons"] += 1
            child_prior = event["child_facts"]
    for before, after in zip(prior, result["final_facts"]):
        retains(before, after)
        counts["root_fact_retention_comparisons"] += 1
assert counts["composed_cases"] == 468 and counts["invalid_cases"] == 54
assert counts["bounded:Ok"] == 770 and counts["bounded:Exhausted"] == 2506
assert counts["finer:Ok"] == 288 and counts["finer:Exhausted"] == 2988
assert counts["events:unlimited_retry"] == counts["events:finer_retry"] == 3276
assert counts["events:cached"] == 13104
assert counts["root_fact_retention_comparisons"] == 85176
assert counts["child_fact_retention_comparisons"] == 9360
assert sum(value for key, value in counts.items() if key.startswith("invalid_retry:")) == 378
assert summary["numeric_samples_checked"] == 27266
assert summary["definite_sign_observations_checked"] == 68862
proof = read(E / "domain-bound-proof-scope-v1698.json")
assert len(proof["unchanged_proof_and_guard_inputs_sha256"]) == 44
assert all(sha(SOURCE / name) == digest for name, digest in proof["unchanged_proof_and_guard_inputs_sha256"].items())
out = E / "composed-proof-reuse-retention-v1758.json"
ledger_record = E / "composed-proof-reuse-ledger-update-v1758.json"
assert not out.exists() and not ledger_record.exists()
record = {
    "status": "Retain public composed-query coverage on the unchanged isolated v1754 runtime; no broad admission or new formal proof",
    "runtime_checkpoint": protocol["runtime_checkpoint"],
    "runtime_checkpoint_sha256": protocol["runtime_checkpoint_sha256"],
    "source_record_sha256": sha(source_record), "source_files_verified": len(sources),
    "main_guarded_files_verified": len(guard), "runtime_sources_changed": 0,
    "phase": PHASE.name, "summary_sha256": sha(PHASE / "summary.json"),
    "protocol": "composed-proof-reuse-protocol-v1757.json", "protocol_sha256": summary["protocol_sha256"],
    "cases": 522, "cases_by_mode": summary["cases_by_mode"], "root_identities": protocol["identities"],
    "numeric_samples_checked": 27266, "definite_sign_observations_checked": 68862,
    "observed_counts": dict(counts), "bounded_outcomes_by_root": by_root, "bounded_outcomes_by_allowance": by_units,
    "validation": "The self-contained external probe passes rustfmt and rustc -D warnings. All 522 processes pass the original 192 MiB, CPU 2/3 second and four-second wall limits. Rust validates samples/facts and Python Fraction independently rechecks numerical containment and sign/zero/magnitude certificates using the same directed MPFR base endpoints and stated composition identities.",
    "fact_audit": "All recorded cross-root and observed-child fact sequences retain prior signs, zero knowledge, exact-rational availability and magnitude strength. Later sign queries retain already observed signs or signs certified by their immediately preceding nonzero numeric samples. Exact zero is never inferred from a numeric zero sample alone.",
    "proof_scope": "All 44 previous conditional proof/guard inputs remain unchanged. No new Verus run or formal proof is claimed; all thirteen whole-crate obligation groups remain open.",
    "limits": protocol["limits"] + [
        "This corpus has 3276 first bounded queries: 770 complete and 2506 exhaust. Every unlimited retry completes. It measures semantic outcomes under the implementation-defined cooperative allowance, not aggregate construction or exact-operation cost.",
        "The 68862 definite-sign observations include repeated structural snapshots and query results; they are not distinct mathematical theorems or a coverage percentage.",
        "No runtime source changed, so earlier v1754 Cargo, Clippy, formatting, replay and cost evidence remains applicable without repeating unrelated builds. Original cost guard failures and the prior integer tradeoff remain explicit.",
        "Main runtime admission, native/full-board combination, general graph semantics, larger arguments and implementation-connected proofs remain open. All 49 preexisting worktree entries and all property/target statuses remain preserved.",
    ],
    "source_review_and_next_proof_work": "composed-proof-reuse-followup-v1758.json",
}
ledger = ROOT / "docs/properties-status.json"
before_bytes = ledger.read_bytes()
assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == before_bytes
before = json.loads(before_bytes)
after = copy.deepcopy(before)
assert len(after["properties"]) == 39 and len(after["targets"]) == 14
p4 = next(r for r in after["properties"] if r["id"] == "P4")
for name in ["composed-proof-reuse-protocol-v1757.json", PHASE.name + "/summary.json",
             "composed-proof-reuse-followup-v1758.json", out.name]:
    path = str(E.relative_to(ROOT) / name)
    assert path not in p4["evidence"]
    p4["evidence"].append(path)
update = (
    "The v1757/v1758 corpus checks seven public ln/exp/expm1/inverse-hyperbolic/cancellation "
    "compositions sharing a child and cooperative allowance, plus independent root reconstruction. "
    "All 522 processes pass: 468 valid schedules span three 80/1100-bit directed residual families, "
    "both signs, exact-zero controls, three root orders and six allowances; 54 schedules carry invalid "
    "descendants through all seven compositions. The 3276 first bounded queries return 770 valid "
    "samples and 2506 Exhausted results. Every unlimited retry completes; 13104 later cached samples "
    "reuse spent allowances. Finer requests include 2988 typed misses and 288 cache hits, followed by "
    "successful retention/retry checks. All 27266 samples satisfy the stated identity and directed "
    "enclosure contracts. The independent receipt audit retains all 85176 root-fact and 9360 observed "
    "child-fact comparisons; invalid controls retain typed errors in all 378 unlimited retries. "
    "Constructor and structural-inspection work occurs outside query allowances, and local caches "
    "are discarded on reconstruction; no globally cold or byte/time claim follows. This remains "
    "finite near-zero sequential coverage, distinct from prior thread stress and formal proofs. "
    "Production sources and all 44 prior proof inputs are unchanged. The next implementation-connected "
    "proof target is the retained magnitude-domain predicate: share its actual pure production body "
    "with the existing bound verifier and prove its conditional real-domain implication, while keeping "
    "existence producers, atomic publication and caller obligations explicit. Preserve the fixed "
    "acceptance baseline; proof coverage alone cannot justify a runtime regression. All thirteen "
    "whole-crate groups remain open, and all 39 properties, 14 targets and crate future work remain "
    "in scope with every existing status unchanged. Native/full-board admission and the older cost "
    "failures/tradeoff remain open."
)
p4["remaining"] += " " + update
old = "Next extend cross-root and composed-function interruption coverage and implementation-connected proofs."
assert after["next_actions"][0].count(old) == 1
after["next_actions"][0] = after["next_actions"][0].replace(old,
    "The v1757 composed-query corpus now passes. Next bind the cached magnitude-domain predicate "
    "to an included production Verus contract, preserving independent existence, explicit atomic/caller "
    "proof obligations and the fixed cost baselines.", 1)
after["next_actions"][0] += " " + update
comparison = copy.deepcopy(after)
original = next(r for r in before["properties"] if r["id"] == "P4")
copy_p4 = next(r for r in comparison["properties"] if r["id"] == "P4")
copy_p4["evidence"], copy_p4["remaining"] = original["evidence"], original["remaining"]
comparison["next_actions"][0] = before["next_actions"][0]
assert comparison == before
out.write_text(json.dumps(record, indent=2) + "\n")
ledger.write_text(json.dumps(after, indent=2) + "\n")
ledger_record.write_text(json.dumps({
    "before_sha256": hashlib.sha256(before_bytes).hexdigest(), "after_sha256": sha(ledger),
    "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
    "all_39_property_and_14_target_requirements_and_statuses_preserved": True,
    "other_next_actions_and_crate_future_work_preserved": True,
}, indent=2) + "\n")
print(json.dumps({"record": out.name, "cases": 522, "samples": 27266,
                  "root_fact_comparisons": 85176, "child_fact_comparisons": 9360,
                  "available_sign_checks": counts["available_sign_query_checks"],
                  "runtime_sources_changed": 0, "ledger_sha256": sha(ledger)}))
