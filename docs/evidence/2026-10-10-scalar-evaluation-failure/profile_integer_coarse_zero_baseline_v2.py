"""One frozen software-event profile per immutable algebraic executable."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import resource
import signal
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
PROTOCOL = E / "integer-coarse-zero-baseline-profile-protocol-v2158.json"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def snapshot(cpu):
    names = ["/proc/self/cgroup", "/proc/pressure/cpu", "/proc/loadavg",
             "/proc/sys/kernel/perf_event_paranoid", "/sys/fs/cgroup/cpu.stat",
             "/sys/fs/cgroup/cpuset.cpus.effective"]
    for name in ["scaling_driver", "scaling_governor", "scaling_cur_freq",
                 "scaling_min_freq", "scaling_max_freq", "energy_performance_preference",
                 "boost", "cpuinfo_avg_freq"]:
        names.append(f"/sys/devices/system/cpu/cpu{cpu}/cpufreq/{name}")
    cgroup = Path("/proc/self/cgroup").read_text().strip().split(":", 2)[2]
    current = Path("/sys/fs/cgroup") / cgroup.lstrip("/")
    while current.is_relative_to("/sys/fs/cgroup"):
        for name in ["cpu.max", "cpu.stat", "cpuset.cpus.effective"]:
            names.append(str(current / name))
        if current == Path("/sys/fs/cgroup"):
            break
        current = current.parent
    result = {"wall_time_ns": time.time_ns(), "affinity": sorted(os.sched_getaffinity(0)),
              "uname": list(os.uname()), "files": {}}
    for name in sorted(set(names)):
        try:
            result["files"][name] = {"text": Path(name).read_text()}
        except OSError as error:
            result["files"][name] = {"error": str(error)}
    return result


def guard(protocol):
    record = E / protocol["source_record"]
    assert sha(record) == protocol["source_record_sha256"]
    sources = json.loads(gzip.decompress(record.read_bytes()))
    main = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert len(sources) == 24746 and all(sha(SOURCE / n) == h for n, h in sources.items())
    assert len(main) == 1167 and all(sha(ROOT / n) == h for n, h in main.items())
    for run in protocol["runs"]:
        assert sha(Path(run["binary"])) == run["binary_sha256"]
        assert sha(E / run["original_probe"]) == run["original_probe_sha256"]
    assert sha(E / protocol["rejection_record"]) == protocol["rejection_record_sha256"]
    assert sha(Path(protocol["prelude"])) == protocol["prelude_sha256"]
    return {"source_files_verified": len(sources), "main_files_verified": len(main),
            "executables_verified": len(protocol["runs"])}


def main():
    assert sys.argv[1:] == ["run"]
    protocol = read(PROTOCOL)
    assert sha(Path(__file__)) == protocol["runner_sha256"]
    assert sha(Path(protocol["perf"])) == protocol["perf_sha256"]
    assert sha(E / protocol["workload_launcher"]) == protocol["workload_launcher_sha256"]
    output = E / protocol["output"]
    assert not output.exists()
    before_guard = guard(protocol)
    output.mkdir()
    before_environment = snapshot(protocol["cpu"])
    (output / "environment-before.json").write_text(json.dumps(before_environment, indent=2) + "\n")
    results = []
    for run in protocol["runs"]:
        assert run["arguments"] == ["cost", "4", "4", "0", "false", "64"]

        def limits():
            os.sched_setaffinity(0, {protocol["cpu"]})
            resource.setrlimit(resource.RLIMIT_AS, (protocol["address_space_bytes"],) * 2)
            resource.setrlimit(resource.RLIMIT_CPU, tuple(protocol["cpu_seconds"]))

        prelude_command = [protocol["prelude"], str(protocol["prelude_iterations"])]
        prelude = subprocess.run(prelude_command, capture_output=True, text=True,
                                 preexec_fn=limits, timeout=protocol["wall_seconds"])
        (output / (run["which"] + "-prelude.stdout")).write_text(prelude.stdout)
        (output / (run["which"] + "-prelude.stderr")).write_text(prelude.stderr)
        assert prelude.returncode == 0 and prelude.stderr == ""
        command = run["command"]
        assert command[-5:] == ["--", sys.executable, str(E / protocol["workload_launcher"]), "exec", run["which"]]
        before = resource.getrusage(resource.RUSAGE_CHILDREN)
        start = time.monotonic()
        def profiler_limits():
            os.sched_setaffinity(0, {protocol["cpu"]})
            resource.setrlimit(resource.RLIMIT_AS, (protocol["profiler_address_space_bytes"],) * 2)
            resource.setrlimit(resource.RLIMIT_CPU, tuple(protocol["profiler_cpu_seconds"]))

        proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                preexec_fn=profiler_limits, start_new_session=True)
        timed_out = False
        try:
            stdout, stderr = proc.communicate(timeout=protocol["wall_seconds"])
        except subprocess.TimeoutExpired:
            timed_out = True
            os.killpg(proc.pid, signal.SIGKILL)
            stdout, stderr = proc.communicate()
        elapsed = time.monotonic() - start
        after = resource.getrusage(resource.RUSAGE_CHILDREN)
        (output / (run["which"] + ".stdout")).write_bytes(stdout)
        (output / (run["which"] + ".stderr")).write_bytes(stderr)
        row = {"which": run["which"], "command": command, "binary_sha256": run["binary_sha256"],
               "exit_code": proc.returncode, "wall_timeout": timed_out, "wall_seconds": elapsed,
               "child_tree_user_seconds": after.ru_utime - before.ru_utime,
               "child_tree_system_seconds": after.ru_stime - before.ru_stime,
               "prelude_exit_code": prelude.returncode, "receipt": None,
               "environment_after": snapshot(protocol["cpu"])}
        if stdout.strip():
            receipt = json.loads(stdout)
            assert {k: v for k, v in receipt.items() if k != "elapsed_ns"} == {
                "stage": "cost", "iterations": 64, "certified": 64}
            row["receipt"] = receipt
        data_path = output / (run["which"] + ".perf.data")
        limits_path = output / (run["which"] + "-effective-workload-limits.json")
        row["workload_launcher_ran"] = limits_path.exists()
        if limits_path.exists():
            effective = read(limits_path)
            assert effective["address_space_bytes"] == [201326592, 201326592]
            assert effective["cpu_seconds"] == [2, 3] and effective["affinity"] == [0]
            assert effective["command"] == [run["binary"]] + run["arguments"]
            row["effective_workload_limits"] = effective
        row["perf_data_bytes"] = data_path.stat().st_size if data_path.exists() else 0
        if row["perf_data_bytes"]:
            report_command = [protocol["perf"], "report", "--stdio", "--no-children",
                              "--sort", "symbol", "--percent-limit", "0.5", "-i", str(data_path)]
            report = subprocess.run(report_command, capture_output=True, timeout=30)
            (output / (run["which"] + "-report.stdout")).write_bytes(report.stdout)
            (output / (run["which"] + "-report.stderr")).write_bytes(report.stderr)
            row["report_command"] = report_command
            row["report_exit_code"] = report.returncode
        results.append(row)
        (output / "attempts.json").write_text(json.dumps(results, indent=2) + "\n")
        if proc.returncode != 0 and row["receipt"] is None and row["perf_data_bytes"] == 0:
            # Preserve an unavailable profiler without another attempt or escalation.
            break
    after_guard = guard(protocol)
    summary = {"protocol": PROTOCOL.name, "protocol_sha256": sha(PROTOCOL),
               "source_record_sha256": protocol["source_record_sha256"],
               "before_guard": before_guard, "after_guard": after_guard,
               "attempted_profiles": len(results), "planned_profiles": 2, "attempts": results,
               "original_cost_gate_passed": False, "runtime_retained": False,
               "limits": protocol["limits"],
               "artifacts_sha256": {p.name: sha(p) for p in sorted(output.iterdir()) if p.is_file()}}
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"attempts": len(results), "exits": [r["exit_code"] for r in results],
                      "original_cost_gate_passed": False, "runtime_retained": False}))


if __name__ == "__main__":
    main()
