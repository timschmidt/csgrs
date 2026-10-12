"""Audit every retained public fact sequence after the runtime extraction."""
from collections import Counter
import copy
import hashlib
import json
from pathlib import Path
import sys

sys.set_int_max_str_digits(20000)
E = Path(__file__).resolve().parent
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()

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

def audit_public(PHASE, summary, protocol, rows):
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
    assert counts["sequential_cold_unknown"] == counts["sequential_cold_exhausted"] == 24
    assert counts["sequential_finer_exhausted"] == 24
    assert counts["warm_worker_samples"] == 1536
    assert counts["race_worker_samples"] + counts["race_worker_exhausted"] == 4608
    assert 0 <= counts["race_unknown_signs"] <= 4608
    assert 1536 <= counts["known_sign_query_retention_checks"] <= 6144
    assert counts["joined_final_fact_comparisons"] == 6144
    assert counts["invalid_budget_queries"] == 96 and counts["invalid_unlimited_retries"] == 24
    return {"observed_counts": counts}


def audit_composed(PHASE, summary, protocol, rows):
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
    assert counts["bounded:Ok"] + counts["bounded:Exhausted"] == 3276
    assert counts["finer:Ok"] + counts["finer:Exhausted"] == 3276
    assert counts["events:unlimited_retry"] == counts["events:finer_retry"] == 3276
    assert counts["events:cached"] == 13104
    assert counts["root_fact_retention_comparisons"] == 85176
    assert counts["child_fact_retention_comparisons"] == 9360
    assert sum(value for key, value in counts.items() if key.startswith("invalid_retry:")) == 378
    return {"observed_counts": dict(counts), "bounded_outcomes_by_root": by_root, "bounded_outcomes_by_allowance": by_units}


def main():
    plan_path = E / "domain-predicate-replay-protocol-v1781.json"
    plan = read(plan_path)
    validation = read(E / "domain-predicate-replay-validation-v1834.json")
    assert validation["complete"] and validation["protocol_sha256"] == sha(plan_path)
    assert validation["source_record_sha256"] == plan["source_record_sha256"]
    out = E / "domain-predicate-public-fact-audit-v1835.json"
    assert not out.exists()
    reports = {}
    for label, item, audit in zip(["public", "composed"], plan["public_replays"], [audit_public, audit_composed]):
        phase = E / item["phase"]
        summary = read(phase / "summary.json")
        protocol = read(phase / "protocol.json")
        rows = read(phase / "receipts.json")
        assert summary["cases"] == len(rows) == len(protocol["cases"]) == item["expected_cases"]
        assert summary["source_hashes_sha256"] == protocol["source_record_sha256"] == plan["source_record_sha256"]
        assert summary["guards_unchanged"] and summary["incomplete_cases"] == summary["invalid_samples"] == 0
        assert sha(phase / "protocol.json") == sha(E / item["protocol"]) == item["protocol_sha256"]
        assert sha(phase / "probe.rs") == sha(E / protocol["probe"]) == item["probe_sha256"]
        assert sha(phase / "reproduce.py") == sha(E / item["runner"]) == item["runner_sha256"]
        assert all(sha(phase / name) == digest for name, digest in summary["artifacts_sha256"].items())
        assert all(row["exit_code"] == 0 and not row["wall_timeout"] for row in read(phase / "commands.json"))
        reports[label] = {"phase": phase.name, "summary_sha256": sha(phase / "summary.json"),
                          "numeric_samples_checked": summary["numeric_samples_checked"],
                          "definite_sign_observations_checked": summary["definite_sign_observations_checked"],
                          **audit(phase, summary, protocol, rows)}
    out.write_text(json.dumps({
        "status": "Every audited public and composed fact sequence retains available certificates",
        "source_record_sha256": plan["source_record_sha256"], "protocol_sha256": sha(plan_path),
        "reports": reports,
        "scope": "Reuse the exact retained receipt-audit loops, with current observation counts. Check each worker/root fact sequence, joined final snapshots and subsequent sign queries after available proofs, including exhausted allowances and finer retries. Frozen probes and Python Fraction oracle checks establish the numerical observations separately.",
        "limits": [
            "Race-dependent sample, Exhausted and Unknown counts describe this run; they are not required to match an earlier schedule.",
            "Composed child snapshots start with the first serialized observation. The frozen Rust probe also checks its initial child state, which is not serialized in this corpus.",
            "Exact-rational flags are checked for retention, not general mathematical soundness. Constructors and structural inspections lie outside query allowances.",
            "Finite thread stress and composition schedules do not prove all interleavings, general graph semantics, aggregate byte/time bounds or whole-crate correctness. Runtime retention still depends on the other gates.",
        ],
    }, indent=2) + "\n")
    print(json.dumps({key: value["observed_counts"] for key, value in reports.items()}))


if __name__ == "__main__":
    main()
