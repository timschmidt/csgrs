"""Keep whole-process observations and copied-kernel allocations in their scopes."""
import hashlib
import json
from pathlib import Path
import statistics

E = Path(__file__).resolve().parent
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / "nth-root-enclosure-resource-review-v2021.json"
    assert not out.exists()
    initial_path = E / "nth-root-enclosure-initial-cost-review-v2021.json"
    initial = read(initial_path)
    assert initial["paired_complete"] and initial["successful_paired_processes"] == 936
    assert initial["complete_fixtures"] == 52 and initial["failed_attempt"] is None
    phase = E / initial["paired_phase"]
    assert all(sha(phase / name) == digest for name, digest in initial["original_run_artifacts_sha256"].items())
    arguments = ["cost", "4", "4", "0", "false", "64"]
    rows = [row for row in read(phase / "receipts.json")
            if row["name"].startswith("algebraic-") and row["command"][1:] == arguments]
    assert len(rows) == 18
    samples = {}
    for which in ["baseline", "candidate"]:
        selected = [row for row in rows if row["name"].endswith("-" + which)]
        assert len(selected) == 9
        assert all(row["exit_code"] == 0 and row["address_space_bytes"] == 192 * 1024 * 1024
                   and row["cpu_seconds_soft_hard"] == [2, 3] and row["wall_seconds_limit"] == 4
                   and row["receipt"]["certified"] == 64 for row in selected)
        wall = [row["wall_seconds"] for row in selected]
        query = [int(row["receipt"]["elapsed_ns"]) for row in selected]
        samples[which] = {"whole_process_wall_seconds": wall,
            "median_whole_process_wall_seconds": statistics.median(wall),
            "maximum_whole_process_wall_seconds": max(wall),
            "query_elapsed_ns": query, "median_query_elapsed_ns": statistics.median(query)}
    for key in ["median_whole_process_wall_seconds", "median_query_elapsed_ns"]:
        assert samples["baseline"][key] > 0
    controller_path = E / "nth-root-enclosure-cost-validation-v2021.json"
    controller = read(controller_path)
    assert controller["complete"] or any(row["exit_code"] != 0 for row in controller["steps"])
    allocation_path = E / "nth-root-enclosure-allocation-study-v2021/summary.json"
    allocation = read(allocation_path)
    assert allocation["source_record_sha256"] == controller["source_record_sha256"] == initial["source_record_sha256"]
    assert allocation["guards_unchanged"] and allocation["counts_stable"] and allocation["exact_endpoints_equal"]
    assert allocation["fixtures"] == 84 and allocation["kernel_calls"] == 504
    assert all(sha(allocation_path.parent / name) == digest for name, digest in allocation["artifacts_sha256"].items())
    allocation_ranges = {}
    for index, field in enumerate(allocation["measurement_fields"]):
        cases = allocation["comparisons"]
        allocation_ranges[field] = {
            "lower_cases": sum(row["candidate"][index] < row["baseline"][index] for row in cases),
            "equal_cases": sum(row["candidate"][index] == row["baseline"][index] for row in cases),
            "higher_cases": sum(row["candidate"][index] > row["baseline"][index] for row in cases),
            "baseline_range": [min(row["baseline"][index] for row in cases), max(row["baseline"][index] for row in cases)],
            "candidate_range": [min(row["candidate"][index] for row in cases), max(row["candidate"][index] for row in cases)],
        }
    prior = E / "cache-state-merge-cost-rejection-v1943.json"
    assert not read(prior)["runtime_retained"]
    data = {
        "status": "Separate resource observations for the combined proposal; retention still requires every gate",
        "source_record_sha256": initial["source_record_sha256"],
        "initial_cost_review": initial_path.name, "initial_cost_review_sha256": sha(initial_path),
        "cost_controller": controller_path.name, "cost_controller_sha256": sha(controller_path),
        "cost_controller_passed": controller["complete"],
        "original_algebraic_fixture": {"arguments": arguments, "pairs": 9, "samples": samples,
            "median_whole_process_wall_percent_change": 100 * (samples["candidate"]["median_whole_process_wall_seconds"] / samples["baseline"]["median_whole_process_wall_seconds"] - 1),
            "median_query_percent_change": 100 * (samples["candidate"]["median_query_elapsed_ns"] / samples["baseline"]["median_query_elapsed_ns"] - 1)},
        "allocation_study": str(allocation_path.relative_to(E)), "allocation_study_sha256": sha(allocation_path),
        "allocation_observations": allocation_ranges,
        "prior_failed_run": prior.name, "prior_failed_run_sha256": sha(prior),
        "limits": [
            "The algebraic whole-process wall observations include construction, warmup and the query loop; the query timer excludes construction and warmup. Wall time is not CPU time and does not identify an internal stage. This comparison changes both the outlined selector and root enclosure relative to retained v1836, so it cannot isolate either change's causal contribution.",
            "The fixed acceptance rule still uses the original query timing comparisons, every full-size fixture and all original process caps. These supplemental wall observations neither relax that rule nor replace any failure or prescribed repeat.",
            allocation["limits"],
            "Earlier capped failures remain failed. The single instrumented v2000 observation identified warmup as dominant in that diagnostic only; it did not retroactively identify the stage of the uninstrumented v1943 failure.",
            "The combined runtime remains unretained until cost qualification and all capped, numerical and public replays and fact audits pass. A failed cost gate prevents starting those replays. Main/native/full-board admission, the fixed pre-Verus baseline and every whole-crate proof group remain open.",
        ],
    }
    out.write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps({"whole_process_wall_percent": data["original_algebraic_fixture"]["median_whole_process_wall_percent_change"],
                      "query_percent": data["original_algebraic_fixture"]["median_query_percent_change"],
                      "allocation_observations": allocation_ranges}))


if __name__ == "__main__":
    main()
