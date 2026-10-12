"""Run cost measurements and their frozen audits sequentially after runtime validation."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    validation = read(E / "integer-coarse-zero-candidate-validation-v2092.json")
    broad = read(E / "integer-coarse-zero-broad-validation-v2092.json")
    protocol = read(E / "integer-coarse-zero-cost-protocol-v2093.json")
    assert validation["complete"] and len(validation["phases"]) == 9
    assert all(row["result"]["exit_code"] == 0 for row in validation["phases"].values())
    expected = validation["source_record_sha256"]
    assert expected == broad["source_record_sha256"] == protocol["source_record_sha256"]
    strategy = E / "integer-coarse-zero-cost-followup-strategy-v2100.json"
    assert read(strategy)["source_record_sha256"] == expected
    out = E / "integer-coarse-zero-cost-validation-v2100.json"
    logs = E / "integer-coarse-zero-cost-controller-v2100"
    assert not out.exists() and not logs.exists()
    logs.mkdir()
    (logs / "runner.py").write_bytes(Path(__file__).read_bytes())
    rows = []

    def save(complete=False):
        out.write_text(json.dumps({"source_record_sha256": expected,
            "runner_sha256": sha(Path(__file__)), "followup_strategy_sha256": sha(strategy),
            "complete": complete, "steps": rows,
            "scope": "Sequential standalone and paired original-cap costs, original receipt audit, predeclared family repeats when needed, and final cost audit. No concurrent builds/tests/measurements or source edits; all unsuccessful steps remain recorded."}, indent=2) + "\n")

    def run(label, helper, *arguments):
        command = [sys.executable, str(E / helper), *arguments]
        print("Starting", label, flush=True)
        started = time.monotonic()
        stdout, stderr = logs / (label + ".stdout"), logs / (label + ".stderr")
        with stdout.open("wb") as output, stderr.open("wb") as errors:
            result = subprocess.run(command, cwd=ROOT, stdout=output, stderr=errors)
        row = {"label": label, "command": command, "helper_sha256": sha(E / helper),
            "exit_code": result.returncode, "seconds": time.monotonic() - started,
            "stdout": str(stdout.relative_to(E)), "stdout_sha256": sha(stdout),
            "stderr": str(stderr.relative_to(E)), "stderr_sha256": sha(stderr)}
        rows.append(row)
        save()
        print("Completed", label, "exit", result.returncode, flush=True)
        return result.returncode

    save()
    assert run("standalone", "run_integer_coarse_zero_costs.py") == 0
    paired = run("paired", "replay_integer_coarse_zero_cost.py")
    # The initial recorder preserves a capped interruption and every successful
    # receipt. A failed original run still stops qualification after that audit.
    reviewed = run("initial-review", "record_integer_coarse_zero_initial_costs.py")
    assert reviewed == 0 and paired == 0, "Original-cap failure remains failed"
    assert run("freeze-followup", "prepare_integer_coarse_zero_cost_followup.py") == 0
    followup = read(E / "integer-coarse-zero-cost-followup-protocol-v2100.json")
    for index, config in enumerate(followup["jobs"]):
        assert config["repetitions"] == 27 and config["cpu_seconds"] == [2, 3] and config["wall_seconds"] == 4
        assert run(f"followup-{index}", "replay_integer_coarse_zero_followup.py", str(index)) == 0
    assert run("final-review", "record_integer_coarse_zero_costs.py") == 0
    result = read(E / "integer-coarse-zero-cost-review-v2100.json")
    assert result["source_record_sha256"] == expected and result["local_timing_and_process_gates_passed"]
    save(complete=True)
    print("All frozen cost steps completed.", flush=True)


if __name__ == "__main__":
    main()
