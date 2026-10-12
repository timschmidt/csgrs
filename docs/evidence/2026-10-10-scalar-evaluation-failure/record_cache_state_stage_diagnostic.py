"""Record stage observations and a source-based hypothesis without retaining runtime."""
import copy
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / "cache-state-algebraic-stage-review-v2000.json"
    ledger_record = E / "cache-state-algebraic-stage-ledger-v2000.json"
    assert not out.exists() and not ledger_record.exists()
    protocol_path = E / "cache-state-algebraic-stage-protocol-v2000.json"
    protocol = read(protocol_path)
    phase = E / protocol["output"]
    summary_path = phase / "summary.json"
    summary = read(summary_path)
    rejection_path = E / protocol["rejection_record"]
    rejection = read(rejection_path)
    assert sha(rejection_path) == protocol["rejection_record_sha256"]
    assert not rejection["runtime_retained"] and not rejection["cost_gate_passed"]
    assert summary["source_record_sha256"] == rejection["source_record_sha256"] == protocol["source_record_sha256"]
    assert summary["protocol_sha256"] == sha(protocol_path)
    assert summary["runner_sha256"] == protocol["runner_sha256"] == sha(E / protocol["runner"])
    assert summary["guards_unchanged"] and summary["original_cost_failure_still_failed"]
    assert summary["diagnostic_completed"] and summary["receipt"]["exit_code"] == 0
    assert not summary["receipt"]["wall_timeout"] and not summary["runtime_retained"]
    assert len(summary["markers"]) == 195 and summary["other_stderr"] == []
    assert summary["receipt"]["address_space_bytes"] == 192 * 1024 * 1024
    assert summary["receipt"]["cpu_seconds_soft_hard"] == [2, 3]
    assert summary["receipt"]["wall_seconds_limit"] == 4
    for name, digest in summary["artifacts_sha256"].items():
        assert sha(phase / name) == digest
    assert sha(Path(protocol["command"][0])) == summary["receipt"]["binary_sha256"]
    totals = summary["stage_totals"]
    assert {name: row["completed_intervals"] for name, row in totals.items()} == {
        "fixture": 64, "warmup": 64, "queries": 1}
    for name, row in totals.items():
        observations = summary["stage_observations"][name]
        assert row["wall_ns"] == sum(item["wall_ns"] for item in observations)
        assert row["cpu_ticks"] == sum(item["cpu_ticks"] for item in observations)
    source_hashes = json.loads(gzip.decompress((E / protocol["source_record"]).read_bytes()))
    reviewed = {
        "crates/hyperreal/src/computable/node/approximation_queries.rs": "approx_signal entry, finite-real validation, cached value/tower dispatch and iterative Add/Negate/Offset traversal; sign_until refinement and algebraic-separation decision.",
        "crates/hyperreal/src/computable/node/quadratic_tower.rs": "cached_quadratic_tower_approximation, including its reuse-only contract; inspected supported NthRoot degree-two arms, without claiming a runtime trace.",
        "crates/hyperreal/src/computable/node/domain_validation.rs": "Bottom-up finite-real traversal and the bounded-degree/nonnegative NthRoot domain check.",
        "crates/hyperreal/src/computable/approximation/dispatch.rs": "Approximation::approximate entry and NthRoot dispatch.",
        "crates/hyperreal/src/computable/approximation/exp_sqrt.rs": "Complete direct nth_root kernel, including range/sign/precision guards, fallback and adjacent integer-root enclosure.",
    }
    for name in reviewed:
        assert sha(SOURCE / name) == source_hashes[name]
    kernel = (SOURCE / "crates/hyperreal/src/computable/approximation/exp_sqrt.rs").read_text()
    body = kernel[kernel.index("fn nth_root("):kernel.index("// Compute cosine of |c| < 1")]
    assert body.count(".nth_root(degree)") == 2 and body.count("upper.pow(degree)") == 1
    next_priority = (
        "Prototype deriving the adjacent direct nth-root enclosure from one integer root, "
        "after freezing the current unretained source. Prove equivalence of both integer "
        "endpoints, retain every range/domain/precision guard and independent existence check, "
        "and exercise perfect-power neighbors, small magnitudes, all supported degrees and "
        "wide inputs. The stage diagnostic identifies aggregate warming work, not the cost "
        "of an individual kernel. Evaluate any concrete change on the unchanged full-size "
        "fixtures and original caps, preserving the v1943 failure and all earlier failures. "
        "Keep the outlined selector unretained until all fresh cost and replay gates pass."
    )
    record = {
        "status": "Candidate-only stage diagnostic and source review; no runtime retention",
        "source_record": protocol["source_record"], "source_record_sha256": protocol["source_record_sha256"],
        "protocol": protocol_path.name, "protocol_sha256": sha(protocol_path),
        "summary": str(summary_path.relative_to(E)), "summary_sha256": sha(summary_path),
        "rejection_record": rejection_path.name, "rejection_record_sha256": sha(rejection_path),
        "diagnostic_processes": 1, "separate_cpu_preludes": 1,
        "original_process_caps_preserved": True, "stage_totals": totals,
        "clock_ticks_per_second": summary["clock_ticks_per_second"],
        "warmup_share_of_measured_stage_wall_time": totals["warmup"]["wall_ns"] / sum(row["wall_ns"] for row in totals.values()),
        "reviewed_sources": {name: {"sha256": source_hashes[name], "scope": scope} for name, scope in reviewed.items()},
        "reviewed_nth_root_body": body,
        "proposal_not_implemented": {
            "current_operations": "For integer sample A > 1, the kernel computes floor_root(A - 1), floor_root(A + 1), and one power to obtain ceil_root(A + 1).",
            "candidate_operations": "For A >= 2 and degree d >= 3, compute r = floor_root(A + 1) and q = r^d once. The lower endpoint is r if q < A, otherwise r - 1; the upper endpoint is r if q = A + 1, otherwise r + 1.",
            "mathematical_argument_to_verify": "When q < A, q <= A - 1 and (r + 1)^d > A + 1, so r is the lower floor root. Otherwise q >= A >= 2 implies r >= 2; the gap r^d - (r - 1)^d is at least 3, so (r - 1)^d <= q - 3 <= A - 2 < A - 1 < q, giving lower root r - 1. The upper endpoint follows from the definition of r. These are the same endpoints as the current kernel; no approximation tolerance is changed.",
            "limits": "This is a source-based optimization hypothesis, not an implemented change, a production proof or measured kernel attribution. Preserve the A <= 1 branch and all existing constructor, domain and extreme-precision behavior.",
        },
        "retained_runtime_checkpoint": rejection["retained_runtime_checkpoint"],
        "runtime_retained": False, "original_cost_failure_still_failed": True,
        "limits": protocol["limits"] + [
            "Observed stage wall totals include instrumentation overhead and are not a matched performance comparison. CPU ticks have the recorded host clock resolution.",
            "The warmup marker brackets the complete approx/sample call, including validation, cache work and all kernels. It does not isolate direct nth-root execution or retroactively locate the original SIGXCPU.",
            "The diagnostic changes no candidate source. The conditional merge proof and nine runtime checks still have their prior scope; decoder/producer/existence/atomic/caller obligations, all thirteen proof groups, main/native/full-board admission and the full 39-property/14-target/crate-future-work scope remain open.",
        ],
        "next_priority": next_priority,
    }
    ledger = ROOT / "docs/properties-status.json"
    before_bytes = ledger.read_bytes()
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == before_bytes
    before = json.loads(before_bytes)
    after = copy.deepcopy(before)
    assert len(after["properties"]) == 39 and len(after["targets"]) == 14
    p4 = next(row for row in after["properties"] if row["id"] == "P4")
    for path in [protocol_path, summary_path, out]:
        name = str(path.relative_to(ROOT))
        assert name not in p4["evidence"]
        p4["evidence"].append(name)
    update = (
        "A separately instrumented v2000 diagnostic completes the unchanged 64-value "
        "fourth-root fixture under its original caps. Its measured stage wall totals are "
        f"{totals['fixture']['wall_ns']} ns for construction, {totals['warmup']['wall_ns']} ns "
        f"for warming and {totals['queries']['wall_ns']} ns for sign queries. These candidate-only "
        "observations include instrumentation overhead and do not identify the stage of the "
        "earlier uninstrumented failure. Source review identifies two neighboring integer-root "
        "computations in the direct kernel and records an unimplemented one-root enclosure "
        "proposal. No source or admission status changes; v1943 remains failed and v1836 retained."
    )
    p4["remaining"] += " " + update
    old = rejection["next_priority"]
    assert after["next_actions"][0].count(old) == 1
    after["next_actions"][0] = after["next_actions"][0].replace(old, next_priority, 1)
    after["next_actions"][0] += " " + update
    unchanged = copy.deepcopy(after)
    original_p4 = next(row for row in before["properties"] if row["id"] == "P4")
    compare_p4 = next(row for row in unchanged["properties"] if row["id"] == "P4")
    compare_p4["evidence"], compare_p4["remaining"] = original_p4["evidence"], original_p4["remaining"]
    unchanged["next_actions"][0] = before["next_actions"][0]
    assert unchanged == before
    out.write_text(json.dumps(record, indent=2) + "\n")
    ledger.write_text(json.dumps(after, indent=2) + "\n")
    ledger_record.write_text(json.dumps({
        "before_sha256": hashlib.sha256(before_bytes).hexdigest(), "after_sha256": sha(ledger),
        "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
        "all_39_property_and_14_target_requirements_and_statuses_preserved": True,
        "diagnostic_review_sha256": sha(out), "runtime_retained": False,
    }, indent=2) + "\n")
    print(json.dumps({"diagnostic_processes": 1, "stage_totals": totals,
                      "original_cost_failure_still_failed": True, "source_unchanged": True}))


if __name__ == "__main__":
    main()
