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
    validation = read(E / "bound-merge-candidate-validation-v1849.json")
    broad = read(E / "bound-merge-broad-validation-v1849.json")
    protocol = read(E / "bound-merge-cost-protocol-v1850.json")
    assert validation["complete"] and len(validation["phases"]) == 9
    assert all(row["result"]["exit_code"] == 0 for row in validation["phases"].values())
    expected = validation["source_record_sha256"]
    assert expected == broad["source_record_sha256"] == protocol["source_record_sha256"]
    strategy = E / "bound-merge-cost-followup-strategy-v1857.json"
    assert read(strategy)["source_record_sha256"] == expected
    out = E / "bound-merge-cost-validation-v1857.json"
    logs = E / "bound-merge-cost-controller-v1857"
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
    assert run("standalone", "run_bound_merge_costs.py") == 0
    paired = run("paired", "replay_bound_merge_cost.py")
    # The initial recorder preserves a capped interruption and every successful
    # receipt. A failed original run still stops qualification after that audit.
    reviewed = run("initial-review", "record_bound_merge_initial_costs.py")
    assert reviewed == 0 and paired == 0, "Original-cap failure remains failed"
    assert run("freeze-followup", "prepare_bound_merge_cost_followup.py") == 0
    followup = read(E / "bound-merge-cost-followup-protocol-v1857.json")
    for index, config in enumerate(followup["jobs"]):
        assert config["repetitions"] == 27 and config["cpu_seconds"] == [2, 3] and config["wall_seconds"] == 4
        assert run(f"followup-{index}", "replay_bound_merge_followup.py", str(index)) == 0
    assert run("final-review", "record_bound_merge_costs.py") == 0
    result = read(E / "bound-merge-cost-review-v1857.json")
    assert result["source_record_sha256"] == expected and result["local_timing_and_process_gates_passed"]
    save(complete=True)
    print("All frozen cost steps completed.", flush=True)


if __name__ == "__main__":
    main()
