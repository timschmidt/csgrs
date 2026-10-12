"""Freeze one diagnostic per original executable; no timing retries or new build."""
import ast
import hashlib
import json
import os
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())


def main():
    out = E / "integer-coarse-zero-baseline-profile-protocol-v2157.json"
    assert not out.exists()
    runner = E / "profile_integer_coarse_zero_baseline.py"
    ast.parse(runner.read_text())
    initial = read(E / "integer-coarse-zero-initial-cost-review-v2100.json")
    reject_path = E / "integer-coarse-zero-cost-rejection-v2100.json"
    rejection = read(reject_path)
    assert not rejection["cost_gate_passed"] and rejection["failure"]["exit_code"] == -24
    preparation = read(E / "integer-coarse-zero-preconditioned-cost-v2100/preparation.json")
    cpu = preparation["cpu"]
    assert cpu == min(os.sched_getaffinity(0)) == 0
    output = "integer-coarse-zero-baseline-profile-v2157"
    perf = Path("/usr/bin/perf")
    runs = []
    family = initial["families"]["algebraic"]
    for which in ["baseline", "candidate"]:
        phase = E / family["phases"][which]
        original = next(r for r in read(phase / "commands.json") if r["name"] not in {"format", "build"})
        binary = Path(original["command"][0])
        assert sha(binary) == family["binary_sha256"][which]
        args = ["cost", "4", "4", "0", "false", "64"]
        command = [str(perf), "record", "-q", "-e", "cpu-clock:u", "-F", "99",
                   "--call-graph", "dwarf,8192", "-m", "64", "-o",
                   str(E / output / (which + ".perf.data")), "--", str(binary)] + args
        runs.append({"which": which, "binary": str(binary), "binary_sha256": sha(binary),
                     "arguments": args, "original_probe": str((phase / "probe.rs").relative_to(E)),
                     "original_probe_sha256": sha(phase / "probe.rs"), "command": command})
    assert runs[0]["original_probe_sha256"] == runs[1]["original_probe_sha256"]
    protocol = {"status": "Frozen before either diagnostic; no runtime or acceptance change",
                "source_record": rejection["source_record"],
                "source_record_sha256": rejection["source_record_sha256"],
                "rejection_record": reject_path.name, "rejection_record_sha256": sha(reject_path),
                "runner": runner.name, "runner_sha256": sha(runner),
                "generator_sha256": sha(Path(__file__)), "output": output,
                "perf": str(perf), "perf_sha256": sha(perf),
                "perf_version": subprocess.check_output([str(perf), "version"], text=True).strip(),
                "cpu": cpu, "address_space_bytes": 201326592, "cpu_seconds": [2, 3], "wall_seconds": 4,
                "prelude": preparation["build_command"][-1], "prelude_sha256": preparation["prelude_sha256"],
                "prelude_iterations": 50000000, "runs": runs,
                "limits": [
                    "Exactly one planned software-event profile per hash-identical original executable, baseline then candidate; no rebuild or source changes, workload reduction, cap increase, retry, host-policy change or original-gate override.",
                    "Both use the unchanged 64-value fourth-root/four-factor workload and the original numerical and exact-zero assertions. Each workload and its perf parent inherit the original address-space and CPU caps; the four-second supervisor deadline also includes profiler startup and shutdown.",
                    "Sampling userspace CPU-clock at 99 Hz with DWARF stack capture is diagnostic instrumentation with overhead. Symbol weights apply only to these runs, not the interrupted earlier run, and do not establish an uninstrumented speedup or a kernel's exclusive causal contribution.",
                    "Aggregate child-tree user/system time includes the profiler and workload. It is not the workload's standalone CPU time. Snapshots are contemporary host observations, not historical telemetry or proof of the earlier failure's cause.",
                    "An unavailable profiler is recorded and stops the diagnostic, without escalation or a substitute favorable attempt. A successful diagnostic cannot replace the failed original comparison or trigger retention. All earlier failures and all thirteen whole-crate proof groups remain open.",
                ]}
    out.write_text(json.dumps(protocol, indent=2) + "\n")
    print(json.dumps({"protocol": out.name, "runs_frozen": len(runs), "runtime_retained": False}))


if __name__ == "__main__":
    main()
