"""Run the frozen runtime replay protocol sequentially, retaining every result."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    protocol_path = E / "integer-coarse-zero-replay-protocol-v2101.json"
    protocol = read(protocol_path)
    expected = protocol["source_record_sha256"]
    validation = read(E / "integer-coarse-zero-candidate-validation-v2092.json")
    assert validation["complete"] and validation["source_record_sha256"] == expected
    assert all(row["result"]["exit_code"] == 0 for row in validation["phases"].values())
    costs = read(E / "integer-coarse-zero-cost-orchestration-v2099.json")
    assert costs["completed"] and costs["source_record_sha256"] == expected
    cost_review = read(E / "integer-coarse-zero-cost-review-v2100.json")
    controller = read(E / "integer-coarse-zero-cost-validation-v2100.json")
    assert controller["complete"] and cost_review["local_timing_and_process_gates_passed"]
    assert cost_review["source_record_sha256"] == controller["source_record_sha256"] == expected
    build, unit = E / protocol["build_phase"], E / protocol["unit_phase"]
    assert sha(build / "source-hashes.json.gz") == sha(unit / "source-hashes.json.gz") == expected
    paths = re.findall(r"Running unittests src/lib\.rs \(([^)]+)\)", (unit / "stderr.log").read_text())
    assert len(paths) == 1
    binary = Path(paths[0])
    binary_hash = sha(binary)
    out = E / "integer-coarse-zero-replay-validation-v2154.json"
    assert not out.exists()
    rows = []

    def save(complete=False):
        out.write_text(json.dumps({
            "source_record_sha256": expected, "protocol_sha256": sha(protocol_path),
            "runner_sha256": sha(Path(__file__)), "complete": complete, "results": rows,
        }, indent=2) + "\n")

    for item in protocol["caps"]:
        phase = E / item["phase"]
        assert not phase.exists() and sha(E / "run_limited_unit_test.py") == item["runner_sha256"]
        command = [sys.executable, str(E / "run_limited_unit_test.py"), str(binary), binary_hash, *item["arguments"]]
        subprocess.run([sys.executable, str(E / "validate_candidate_v2.py"), phase.name,
                        "--exec", *command], cwd=ROOT, check=True)
        result = read(phase / "result.json")
        log = (phase / "stdout.log").read_text()
        cap = json.loads(log[log.rfind('{"command":'):])
        rows.append({"kind": "cap", "phase": phase.name, "baseline_phase": item["baseline_phase"],
                     "result": result, "cap": cap})
        save()
        assert result["exit_code"] == cap["exit_code"] == 0 and not cap["wall_timeout"]
        assert result["source_unchanged"] and result["main_unchanged"]
        assert sha(phase / "source-hashes.json.gz") == expected
        assert any(int(count) > 0 for count in re.findall(r"test result: ok\. (\d+) passed", log))
        print(phase.name, "passed", flush=True)

    for kind, items in [("numerical", protocol["numerical_replays"]), ("public", protocol["public_replays"])]:
        for item in items:
            phase = E / item["phase"]
            assert not phase.exists()
            command = item["command"]
            assert sha(Path(command[1])) == item["runner_sha256"]
            started = time.monotonic()
            result = subprocess.run(command, cwd=ROOT)
            row = {"kind": kind, "phase": phase.name, "command": command,
                   "exit_code": result.returncode, "seconds": time.monotonic() - started,
                   "expected_cases": item["expected_cases"]}
            rows.append(row)
            save()
            assert result.returncode == 0, row
            summary = read(phase / "summary.json")
            assert summary["source_hashes_sha256"] == expected and summary["guards_unchanged"]
            if item["expected_cases"] == 415:
                assert summary["ordinary_cases"] == 400 and summary["typed_interruption_errors"] == 300
                assert summary["invalid_first_contracts"] == summary["invalid_retries"] == 0
                assert len(summary["extremes"]) == 6 and len(summary["cache"]) == 9
                assert all(value["valid"] for value in summary["extremes"] + summary["cache"])
                assert all(value["exit_code"] == 0 for value in summary["commands"])
            else:
                assert summary["cases"] == item["expected_cases"]
                for key in ["incomplete_cases", "invalid_numeric_samples", "invalid_samples",
                            "unexpected_result_shapes", "nonzero_exit_cases", "invalid_numeric_cases", "typed_errors"]:
                    if key in summary:
                        assert not summary[key], (phase.name, key, summary[key])
            if kind == "numerical":
                assert sha(phase / "reproduce.py") == sha(E / item["baseline_phase"] / "reproduce.py")
                for name, digest in item["baseline_rust_inputs_sha256"].items():
                    assert sha(phase / name) == digest
            else:
                assert sha(phase / "probe.rs") == item["probe_sha256"]
                assert sha(phase / "protocol.json") == item["protocol_sha256"]
                assert sha(phase / "reproduce.py") == item["runner_sha256"]
            row["summary_sha256"] = sha(phase / "summary.json")
            save()
            print(phase.name, item["expected_cases"], "cases passed", flush=True)
    assert len(rows) == 54
    save(True)


if __name__ == "__main__":
    main()
