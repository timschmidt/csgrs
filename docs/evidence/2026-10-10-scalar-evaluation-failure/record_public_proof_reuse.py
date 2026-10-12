"""Audit public observations, including all facts retained after joining workers."""
import copy
from fractions import Fraction
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
PHASE = E / "typed-api-prototype-v1755-public-proof-reuse"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
summary = read(PHASE / "summary.json")
protocol = read(PHASE / "protocol.json")
rows = read(PHASE / "receipts.json")
assert summary["cases"] == len(rows) == 126
assert summary["cases_by_mode"] == {"sequential": 24, "warm": 24, "race": 72, "invalid": 6}
assert summary["guards_unchanged"] and summary["incomplete_cases"] == summary["invalid_samples"] == 0
assert all(sha(PHASE / name) == digest for name, digest in summary["artifacts_sha256"].items())
assert sha(E / "public-proof-reuse-protocol-v1755.json") == summary["protocol_sha256"]
assert sha(E / protocol["probe"]) == protocol["probe_sha256"]
assert sha(E / "reproduce_public_proof_reuse.py") == protocol["runner_sha256"]
assert sha(PHASE / "probe.rs") == protocol["probe_sha256"]
assert sha(PHASE / "reproduce.py") == protocol["runner_sha256"]
assert all(sha(Path(name)) == digest for name, digest in summary["libraries_sha256"].items())
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


counts = {"sequential_cold_unknown": 0, "sequential_cold_exhausted": 0,
          "sequential_finer_exhausted": 0, "warm_worker_samples": 0,
          "race_worker_samples": 0, "race_worker_exhausted": 0,
          "race_unknown_signs": 0, "known_sign_query_retention_checks": 0,
          "joined_final_fact_comparisons": 0, "invalid_budget_queries": 0,
          "invalid_unlimited_retries": 0}
for row, args in zip(rows, protocol["cases"]):
    assert row["arguments"] == args and row["complete"]
    command = row["command"]
    assert command["exit_code"] == 0 and not command["wall_timeout"]
    assert command["address_space_cap_mib"] == 192
    assert command["cpu_seconds"] == [2, 3] and command["timeout_seconds"] == 4
    assert sha(Path(command["command"][0])) == summary["binary_sha256"]
    assert read(PHASE / (row["name"] + ".stdout")) == row["records"][0]
    assert (PHASE / (row["name"] + ".stderr")).read_text() == ""
    record = row["records"][0]
    mode, result = record["mode"], record["result"]
    assert result["serialized_expression_unchanged"]
    if mode == "sequential":
        assert result["cold_sign"]["sign"] is None
        assert result["cold_sample"]["error"] == "Exhausted"
        assert result["finer_attempt"]["error"] == "Exhausted"
        assert result["spent_units"] == 31
        counts["sequential_cold_unknown"] += 1
        counts["sequential_cold_exhausted"] += 1
        counts["sequential_finer_exhausted"] += 1
        retains(result["cold_facts"], result["warm_facts"])
        prior = result["warm_facts"]
        for query in result["cached_queries"]:
            retains(prior, query["facts"])
            assert query["sign"]["sign"] == record["oracle"]["expected_sign"]
            prior = query["facts"]
        retains(prior, result["final_facts"])
    elif mode in {"warm", "race"}:
        assert result["workers"] == result["rounds_per_worker"] == 8
        assert len(result["rows"]) == 64
        assert result["shared_consumed_after_drain"] == result["shared_limit"]
        assert result["shared_consumed_before_drain"] <= result["shared_limit"]
        assert result["independent_consumed"] <= 1_000_000
        prior = {worker: result["before_facts"] for worker in range(8)}
        if mode == "warm":
            assert result["shared_limit"] == result["shared_consumed_before_drain"] == 31
            assert result["independent_consumed"] == 0
        for observation in result["rows"]:
            before = prior[observation["worker"]]
            after = observation["facts"]
            retains(before, after)
            approximation = observation["approximation"]
            if "sample" in approximation:
                counts[mode + "_worker_samples"] += 1
            else:
                assert mode == "race" and approximation["error"] == "Exhausted"
                counts["race_worker_exhausted"] += 1
            answer = observation["sign"]["sign"]
            if answer is None:
                assert mode == "race"
                counts["race_unknown_signs"] += 1
            # A definite fact observed in an earlier completed call, or a newly
            # returned numeric sample with |n| > 1, must survive this later sign
            # query even when its explicit allowance is spent.
            if before["sign"] is not None or ("sample" in approximation and abs(int(approximation["sample"])) > 1):
                assert answer == record["oracle"]["expected_sign"]
                counts["known_sign_query_retention_checks"] += 1
            retains(after, result["final_facts"])
            counts["joined_final_fact_comparisons"] += 1
            prior[observation["worker"]] = after
    else:
        assert mode == "invalid" and len(result["rows"]) == 16
        assert result["unlimited_retries"] == 4
        assert result["shared_consumed"] <= 32
        expected = "Err(" + result["expected"] + ")"
        assert all(r["answer"] in {expected, "Err(Exhausted)"} for r in result["rows"])
        counts["invalid_budget_queries"] += 16
        counts["invalid_unlimited_retries"] += 4
assert counts == {
    "sequential_cold_unknown": 24, "sequential_cold_exhausted": 24,
    "sequential_finer_exhausted": 24, "warm_worker_samples": 1536,
    "race_worker_samples": 4587, "race_worker_exhausted": 21,
    "race_unknown_signs": 142, "known_sign_query_retention_checks": 5758,
    "joined_final_fact_comparisons": 6144, "invalid_budget_queries": 96,
    "invalid_unlimited_retries": 24,
}
assert summary["numeric_samples_checked"] == 6459
assert summary["definite_sign_observations_checked"] == 12572
# The recorded observed counts describe this run, not deterministic scheduling
# requirements for future replays of the unchanged probe.
out = E / "public-proof-reuse-retention-v1756.json"
ledger_record = E / "public-proof-reuse-ledger-update-v1756.json"
assert not out.exists() and not ledger_record.exists()
record = {
    "status": "Retain new public proof-reuse coverage on the unchanged isolated v1754 runtime; no general concurrency or main-admission claim",
    "runtime_checkpoint": "typed-api-prototype-v1754-state.json",
    "runtime_checkpoint_sha256": sha(E / "typed-api-prototype-v1754-state.json"),
    "runtime_commit": "d31f20cc28e0439040a1688eb01234d089f2cfed",
    "source_record_sha256": sha(source_record), "source_files_verified": len(sources),
    "main_guarded_files_verified": len(guard), "runtime_sources_changed": 0,
    "phase": PHASE.name, "summary_sha256": sha(PHASE / "summary.json"),
    "protocol": "public-proof-reuse-protocol-v1755.json", "protocol_sha256": summary["protocol_sha256"],
    "probe_sha256": protocol["probe_sha256"], "runner_sha256": protocol["runner_sha256"],
    "cases": 126, "cases_by_mode": summary["cases_by_mode"],
    "numeric_samples_checked": 6459, "definite_sign_observations_checked": 12572,
    "observed_run_counts": counts,
    "posthoc_audit": "The frozen probe already checks each thread's fact sequence. This additional audit checks that every final joined snapshot retains every worker observation, and that a sign available from a prior completed observation or the immediately preceding numeric sample survives the later budgeted sign query. All 6144 final comparisons and 5758 applicable sign queries pass.",
    "validation": "The external probe passes rustfmt and rustc -D warnings, followed by all 126 processes under 192 MiB, CPU 2/3 second and four-second wall limits. Candidate source and retained libraries are unchanged; the v1754 Cargo, lint, format and replay evidence remains applicable without repeating unrelated builds.",
    "proof_scope": "No proof input or production source changed. Prior conditional Verus evidence remains limited to its old target; all thirteen whole-crate groups remain open.",
    "limits": protocol["limits"] + [
        "The numerical reference is a directed MPFR enclosure; Python independently checks rational containment using those same endpoints. It is not a second independent transcendental implementation.",
        "Counts of Unknown and Exhausted outcomes in mixed schedules can vary with scheduling. The preserved receipts describe this run; future replays must satisfy the same semantic invariants, not identical interleavings.",
        "The 12572 sign observations include repeated query and structural-fact observations, not distinct theorems. No new P4 completion status is asserted.",
        "Public malformed serialized graphs, cross-root composed functions, cancellation signals and implementation-connected concurrent proofs remain outside this corpus.",
        "Original baseline CPU-cap failures, the older integer-control cost tradeoff, native combination and unchanged full-board admission remain open. Main and all 49 preexisting worktree entries are preserved.",
    ],
    "next_priority": "Extend public cross-root and composed ln/exp/inverse-hyperbolic interruption and proof-reuse coverage; preserve the full constructor, coefficient, domain, formal and native/full-board frontier.",
}
ledger = ROOT / "docs/properties-status.json"
before_bytes = ledger.read_bytes()
assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == before_bytes
before = json.loads(before_bytes)
after = copy.deepcopy(before)
assert len(after["properties"]) == 39 and len(after["targets"]) == 14
p4 = next(r for r in after["properties"] if r["id"] == "P4")
for name in ["public-proof-reuse-protocol-v1755.json", PHASE.name + "/summary.json", out.name]:
    path = str(E.relative_to(ROOT) / name)
    assert path not in p4["evidence"]
    p4["evidence"].append(path)
update = (
    "The v1755/v1756 public proof-reuse corpus passes on the unchanged v1754 isolated runtime. "
    "Public pi, sqrt(2) and sin(1/3) residuals at 80 and 1100 bits, both signs and constructor/roundtrip "
    "origins, cover 24 sequential schedules, 24 warm eight-thread schedules, 72 mixed eight-thread "
    "schedules and six invalid-domain controls. All 126 capped processes pass with 6459 samples "
    "checked against directed 2560-bit enclosures. All 24 cold zero-work signs are Unknown and "
    "become available after fine warming; all 24 finer zero-work misses preserve prior proofs and "
    "allow unlimited retry. The warm workers return 1536 valid cached samples without additional "
    "work after spending their shared allowance. Mixed schedules retain 4587 successful samples, "
    "21 Exhausted samples and 142 Unknown signs in this observed run. A separate receipt audit "
    "checks 5758 later sign queries with already available proofs and all 6144 joined final facts "
    "against worker observations, with no contradiction or weakening. Invalid controls retain "
    "typed errors across 96 bounded queries and 24 unlimited retries. Serialization bytes remain "
    "unchanged; reconstructed objects deliberately start with fresh caches. Scheduling outcomes "
    "may vary and this finite thread stress is not an exhaustive or formal concurrency proof. "
    "No production source or prior proof input changed. Continue cross-root/composed interruption, "
    "constructor/coefficient review, implementation-connected proofs and native/full-board admission. "
    "All 39 properties, 14 targets, thirteen whole-crate proof groups and crate future work remain open "
    "as previously recorded; no completion status changes."
)
p4["remaining"] += " " + update
lead = "Next prioritize public concurrent and reconstructed high-precision proof reuse."
assert after["next_actions"][0].count(lead) == 1
after["next_actions"][0] = after["next_actions"][0].replace(lead,
    "Public concurrent and reconstructed high-precision proof reuse now passes the v1755 corpus. "
    "Next extend cross-root and composed-function interruption coverage and implementation-connected proofs.", 1)
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
print(json.dumps({"record": out.name, "cases": 126, "samples": 6459,
                  "known_sign_query_checks": 5758, "joined_fact_comparisons": 6144,
                  "runtime_sources_changed": 0, "ledger_sha256": sha(ledger)}))
