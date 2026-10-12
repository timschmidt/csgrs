"""Review the preserved profiles without converting diagnostics into acceptance."""
import copy
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    output = E / "integer-coarse-zero-baseline-profile-review-v2158.json"
    ledger_output = E / "integer-coarse-zero-baseline-profile-ledger-v2158.json"
    assert not output.exists() and not ledger_output.exists()
    failure = read(E / "integer-coarse-zero-cost-rejection-v2100.json")
    initial = read(E / "integer-coarse-zero-baseline-profile-v2157/summary.json")
    current = read(E / "integer-coarse-zero-baseline-profile-v2158/summary.json")
    inputs = {}
    for summary in [initial, current]:
        protocol_path = E / summary["protocol"]
        assert sha(protocol_path) == summary["protocol_sha256"]
        protocol = read(protocol_path)
        assert sha(E / protocol["runner"]) == protocol["runner_sha256"]
        assert summary["source_record_sha256"] == failure["source_record_sha256"]
        assert summary["before_guard"] == summary["after_guard"] == {
            "source_files_verified": 24746, "main_files_verified": 1167, "executables_verified": 2}
        directory = E / protocol["output"]
        assert all(sha(directory / n) == h for n, h in summary["artifacts_sha256"].items())
        inputs[str(protocol_path.relative_to(E))] = sha(protocol_path)
        inputs[str((directory / "summary.json").relative_to(E))] = sha(directory / "summary.json")
    assert initial["attempted_profiles"] == 1 and initial["attempts"][0]["exit_code"] == 127
    assert initial["attempts"][0]["receipt"] is None and initial["attempts"][0]["perf_data_bytes"] == 0
    assert current["attempted_profiles"] == 2
    profiles = []
    for row in current["attempts"]:
        assert row["exit_code"] == row["report_exit_code"] == 0 and not row["wall_timeout"]
        assert row["workload_launcher_ran"] and row["effective_workload_limits"]["cpu_seconds"] == [2, 3]
        assert row["effective_workload_limits"]["address_space_bytes"] == [201326592] * 2
        assert row["effective_workload_limits"]["affinity"] == [0]
        assert sha(Path(row["effective_workload_limits"]["command"][0])) == row["binary_sha256"]
        report = E / "integer-coarse-zero-baseline-profile-v2158" / (row["which"] + "-report.stdout")
        text = report.read_text()
        samples = int(re.search(r"^# Samples: (\d+)", text, re.M)[1])
        lost = int(re.search(r"^# Total Lost Samples: (\d+)", text, re.M)[1])
        weights = {symbol.strip(): float(weight) for weight, symbol in re.findall(
            r"^\s+([0-9.]+)%\s+\[\.\]\s+(.*?)\s+-\s+-\s*$", text, re.M)}
        dominant = {symbol: weights[symbol] for symbol in [
            "num_bigint::biguint::division::div_rem_core", "num_bigint::biguint::multiplication::mac3"]}
        assert sum(dominant.values()) > 90 and lost == 0
        profiles.append({"which": row["which"], "binary_sha256": row["binary_sha256"],
            "samples_including_launcher": samples, "lost_samples": lost,
            "exclusive_symbol_percent": dominant, "all_reported_symbol_percent": weights,
            "wall_seconds_including_profiler": row["wall_seconds"],
            "child_tree_user_seconds": row["child_tree_user_seconds"],
            "child_tree_system_seconds": row["child_tree_system_seconds"],
            "receipt": row["receipt"], "effective_workload_limits": row["effective_workload_limits"]})
    assert [p["samples_including_launcher"] for p in profiles] == [180, 95]
    source_record = E / failure["source_record"]
    assert sha(source_record) == failure["source_record_sha256"]
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    next_priority = (
        "Restore all nine unretained source changes to the exact v1836 checkpoint, preserving "
        "the proposals, proofs, allocation studies and every failed cost gate. Verify restored "
        "source identity and run scalar/release/format/proof checks. Then connect the actual "
        "cache-word decoder to its specification from that retained base; do not start another "
        "unchanged cost trial or treat the successful profiles as admission. The decoder, "
        "producer/existence/atomic and native/full-board obligations remain open."
    )
    review = {"status": "Diagnostic profiles explain sampled work only; original cost gate remains failed",
        "source_record": failure["source_record"], "source_record_sha256": failure["source_record_sha256"],
        "inputs_sha256": inputs, "setup_loader_failures": 1, "successful_workload_profiles": 2,
        "separate_cpu_preludes": 3, "profiles": profiles, "source_files_verified": len(sources),
        "main_files_verified": len(guard), "retained_runtime_checkpoint": failure["retained_runtime_checkpoint"],
        "original_cost_gate_passed": False, "runtime_retained": False,
        "observations": [
            "The first perf parent failed in the dynamic loader at 192 MiB, before launching a workload. The fork places the original limits on the actual executable via a recorded exec wrapper and bounds the profiler separately.",
            "The baseline and candidate each complete the same 64-value fixture with all original numerical and sign assertions. Sampled userspace instruction pointers mostly name bigint division and multiplication in both runs. The original executable bytes are unchanged.",
            "Contemporary snapshots report powersave with balance_performance and variable frequency; visible cgroup ancestors report cpu.max=max 100000. These observations neither recover historical CPU frequency nor prove why the original process crossed its CPU limit.",
            "DWARF call chains include unresolved numeric addresses. The report supports exclusive instruction-pointer symbol weights; it does not reliably isolate a caller, individual root kernel, construction/warmup/query stage or the earlier failure stage.",
        ],
        "limits": read(E / current["protocol"])["limits"] + [
            "One diagnostic per executable, with launcher/profiler overhead and only 180/95 samples including launcher events. No significance test, steady-state timing pass, peak-memory measurement, uniform speedup or historical failure attribution is established.",
            "The completed wide_gap improvement and unresolved small_coarse increase remain partial cost evidence. All earlier gates and failures remain unchanged. Restoring v1836 is the next action, not yet performed by this recorder.",
        ], "next_priority": next_priority}
    ledger = ROOT / "docs/properties-status.json"
    before_bytes = ledger.read_bytes()
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == before_bytes
    before, after = json.loads(before_bytes), json.loads(before_bytes)
    p4 = next(p for p in after["properties"] if p["id"] == "P4")
    p4["evidence"].append(str(output.relative_to(ROOT)))
    update = (
        "The v2157 profiler fails in its loader before launching a workload; v2158 corrects "
        "the process receiving the memory limit and preserves that setup failure. One profile "
        "of each immutable executable completes the original full algebraic fixture under "
        "192 MiB, CPU 2/3-second and four-second wall limits. Division and multiplication "
        "account for 92.22 and 94.74 percent of 180 and 95 sampled userspace events, including "
        "launcher events. These sparse diagnostic observations and contemporary CPU policy "
        "snapshots do not identify the earlier failure's cause, override v2100, or admit the "
        "proposal. Restore v1836 before continuing the actual decoder proof; all thirteen "
        "whole-crate groups and main/native/full-board requirements remain open."
    )
    p4["remaining"] += " " + update
    assert after["next_actions"][0].count(failure["next_priority"]) == 1
    after["next_actions"][0] = after["next_actions"][0].replace(failure["next_priority"], next_priority, 1)
    after["next_actions"][0] += " " + update
    same = copy.deepcopy(after)
    old = next(p for p in before["properties"] if p["id"] == "P4")
    row = next(p for p in same["properties"] if p["id"] == "P4")
    row["remaining"], row["evidence"] = old["remaining"], old["evidence"]
    same["next_actions"][0] = before["next_actions"][0]
    assert same == before
    output.write_text(json.dumps(review, indent=2) + "\n")
    ledger.write_text(json.dumps(after, indent=2) + "\n")
    ledger_output.write_text(json.dumps({"before_sha256": hashlib.sha256(before_bytes).hexdigest(),
        "after_sha256": sha(ledger), "review_sha256": sha(output),
        "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
        "all_39_property_and_14_target_requirements_and_statuses_preserved": True}, indent=2) + "\n")
    print(json.dumps({"successful_profiles": 2, "samples": [180, 95], "runtime_retained": False}))


if __name__ == "__main__":
    main()
