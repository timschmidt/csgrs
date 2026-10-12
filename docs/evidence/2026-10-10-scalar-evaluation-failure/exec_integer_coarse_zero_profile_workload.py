"""Apply the original limits to the profiled workload, then exec its original binary."""
import json
import os
from pathlib import Path
import resource
import sys

E = Path(__file__).resolve().parent
protocol = json.loads((E / "integer-coarse-zero-baseline-profile-protocol-v2158.json").read_text())
assert len(sys.argv) == 3 and sys.argv[1] == "exec"
run = next(row for row in protocol["runs"] if row["which"] == sys.argv[2])
os.sched_setaffinity(0, {protocol["cpu"]})
resource.setrlimit(resource.RLIMIT_AS, (protocol["address_space_bytes"],) * 2)
resource.setrlimit(resource.RLIMIT_CPU, tuple(protocol["cpu_seconds"]))
record = E / protocol["output"] / (run["which"] + "-effective-workload-limits.json")
assert not record.exists()
usage = resource.getrusage(resource.RUSAGE_SELF)
record.write_text(json.dumps({
    "command": [run["binary"]] + run["arguments"], "pid": os.getpid(),
    "address_space_bytes": list(resource.getrlimit(resource.RLIMIT_AS)),
    "cpu_seconds": list(resource.getrlimit(resource.RLIMIT_CPU)),
    "affinity": sorted(os.sched_getaffinity(0)),
    "wrapper_user_seconds_before_exec": usage.ru_utime,
    "wrapper_system_seconds_before_exec": usage.ru_stime,
    "limits_apply_to_original_executable_after_exec": True,
}, indent=2) + "\n")
os.execv(run["binary"], [run["binary"]] + run["arguments"])
