"""Shared external containment for this fresh consumer qualification."""
import gzip
import hashlib
import json
import os
import resource
import signal
import subprocess
import time
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
V = Path("/dev/shm/csgrs-circle-support-consumers-v1")


def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def read(p):
    return json.loads(Path(p).read_text())


def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2) + "\n")


def quota_observation():
    command = ["quota", "-w", "-v", "-p", "--show-mntpoint", "--hide-device", "-f", "/dev/shm"]
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=10)
    line = next(line for line in result.stdout.splitlines() if line.split() and line.split()[0] == "/dev/shm")
    fields = line.split()
    used, soft, hard = (int(value.rstrip("*")) * 1024 for value in fields[1:4])
    assert hard > 0
    return {"command_exit_code": result.returncode, "used_bytes": used, "soft_limit_bytes": soft,
            "hard_limit_bytes": hard, "available_bytes": hard - used}


def free_bytes(path):
    fs = os.statvfs(path)
    return fs.f_bavail * fs.f_frsize


def memory_available():
    line = next(line for line in Path("/proc/meminfo").read_text().splitlines() if line.startswith("MemAvailable:"))
    return int(line.split()[1]) * 1024


class Runner:
    def __init__(self, name, target, helper, virtual_gib=8, runtime_root=None):
        self.prep = read(V / "preparation-v1/result.json")
        assert self.prep["complete"]
        self.source = Path(self.prep["source"])
        self.manifest = V / "preparation-v1/source-hashes.json.gz"
        assert sha(self.manifest) == self.prep["source_hashes_sha256"]
        with gzip.open(self.manifest, "rt") as f:
            self.inputs = json.load(f)
        assert not self.source_mismatches()
        self.output = (runtime_root or V) / name
        self.target = target
        self.runtime_root = runtime_root
        self.virtual_gib = virtual_gib
        self.rows = []
        self.not_run = []
        self.stop = None
        self.output.mkdir(exist_ok=False)
        self.target.mkdir(exist_ok=False)
        self.scratch = self.output / "scratch"
        self.scratch.mkdir()
        self.env = os.environ.copy()
        for key in ["RUSTFLAGS", "CARGO_ENCODED_RUSTFLAGS", "RUSTC_WRAPPER", "RUSTC_WORKSPACE_WRAPPER", "RUSTDOCFLAGS", "NODE_OPTIONS"]:
            self.env.pop(key, None)
        self.env.update({
            "CARGO_HOME": "/home/tim/.cargo", "CARGO_TARGET_DIR": str(self.target),
            "CARGO_BUILD_JOBS": "2", "CARGO_NET_OFFLINE": "true", "CARGO_INCREMENTAL": "0",
            "CARGO_TERM_COLOR": "never", "TMPDIR": str(self.scratch),
            "RUST_BACKTRACE": "0", "PYTHONDONTWRITEBYTECODE": "1",
        })
        self.pins = {str(p): sha(p) for p in [
            Path(__file__), helper, W / "contract-v1.md", self.manifest, V / "preparation-v1/result.json",
        ]}
        dump(self.output / "protocol.json", {
            "input_sha256": self.pins, "source": str(self.source), "target": str(self.target),
            "target_initially_empty": not list(self.target.iterdir()), "base_commit": self.prep["base_commit"],
            "rustc": subprocess.check_output(["rustc", "-vV"], text=True),
            "cargo": subprocess.check_output(["cargo", "-V"], text=True),
            "hardware": subprocess.check_output(["lscpu"], text=True),
            "environment": {key: self.env[key] for key in [
                "CARGO_HOME", "CARGO_TARGET_DIR", "CARGO_BUILD_JOBS", "CARGO_NET_OFFLINE",
                "CARGO_INCREMENTAL", "CARGO_TERM_COLOR", "TMPDIR", "RUST_BACKTRACE", "PYTHONDONTWRITEBYTECODE",
            ]},
            "existing_generated_test_environment": {k: v for k, v in self.env.items() if k.startswith("PROPTEST_")},
            "initial_resources": self.resources(), "external_timeout_seconds_per_command": 1800,
            "address_space_bytes_per_process": virtual_gib * 1024**3,
            "minimum_repository_free_bytes": 768 * 1024**2,
            "minimum_shm_user_quota_free_bytes": 512 * 1024**2,
            "minimum_host_available_memory_bytes": 2 * 1024**3,
            "minimum_runtime_filesystem_free_bytes": 256 * 1024**2 if runtime_root else None,
            "resource_probe_interval_seconds": 5, "completed_targets_reopened": False,
            "internal_resource_bound_claimed": False, "duplicate_executable_copies_created": False,
        })

    def source_mismatches(self):
        return {p: sha(self.source / p) if (self.source / p).is_file() else None
                for p, h in self.inputs.items() if not (self.source / p).is_file() or sha(self.source / p) != h}

    def resources(self):
        value = {
            "repository_free_bytes": free_bytes(ROOT), "shm_quota": quota_observation(),
            "host_available_memory_bytes": memory_available(),
        }
        if self.runtime_root:
            value["runtime_filesystem_free_bytes"] = free_bytes(self.runtime_root)
        return value

    @staticmethod
    def violation(value):
        if value["repository_free_bytes"] < 768 * 1024**2:
            return "repository-space-reserve"
        if value["shm_quota"]["available_bytes"] < 512 * 1024**2:
            return "shm-user-quota-reserve"
        if value["host_available_memory_bytes"] < 2 * 1024**3:
            return "host-memory-reserve"
        if value.get("runtime_filesystem_free_bytes", 1024**4) < 256 * 1024**2:
            return "runtime-filesystem-space-reserve"
        return None

    def omit(self, name, reason):
        self.not_run.append({"name": name, "reason": reason})

    def run(self, name, command, docs=False, extra=None):
        if self.stop:
            self.omit(name, self.stop)
            return 125
        assert not self.source_mismatches()
        initial = self.resources()
        if reason := self.violation(initial):
            self.stop = reason
            self.omit(name, reason)
            return 125
        print("START " + name, flush=True)
        start = time.monotonic()
        observations = [{"elapsed_seconds": 0, **initial}]
        reason = None
        command_env = self.env | ({"RUSTDOCFLAGS": "-D warnings"} if docs else {}) | (extra or {})

        def limits():
            ceiling = self.virtual_gib * 1024**3
            resource.setrlimit(resource.RLIMIT_AS, (ceiling, ceiling))
            resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

        with (self.output / (name + ".stdout")).open("wb") as out, (self.output / (name + ".stderr")).open("wb") as err:
            proc = subprocess.Popen(
                ["/usr/bin/time", "-v", "-o", str(self.output / (name + ".resources")), *command],
                cwd=self.source, env=command_env, stdout=out, stderr=err,
                preexec_fn=limits, start_new_session=True,
            )
            dump(self.output / (name + ".process.json"), {
                "pid": proc.pid, "command": command, "started_unix": time.time(),
            })
            probe_failures = 0
            while proc.poll() is None:
                try:
                    proc.wait(timeout=5)
                    continue
                except subprocess.TimeoutExpired:
                    pass
                elapsed = time.monotonic() - start
                try:
                    observed = self.resources()
                    observations.append({"elapsed_seconds": elapsed, **observed})
                    probe_failures = 0
                    reason = self.violation(observed)
                except Exception as error:
                    observations.append({"elapsed_seconds": elapsed, "resource_probe_error": repr(error)})
                    probe_failures += 1
                    if probe_failures >= 3:
                        reason = "resource-probe-unavailable"
                if elapsed > 1800:
                    reason = "external-timeout"
                if reason:
                    os.killpg(proc.pid, signal.SIGTERM)
                    try:
                        proc.wait(timeout=20)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                    break
            code = proc.wait()
        stderr = (self.output / (name + ".stderr")).read_text(errors="replace")
        if not reason and code and any(s in stderr for s in ["Disk quota exceeded", "No space left on device"]):
            reason = "reported-storage-exhaustion"
        mismatches = self.source_mismatches()
        if mismatches:
            self.stop = "source-input-mutated"
        elif reason and reason != "external-timeout":
            self.stop = reason
        assert all(sha(p) == h for p, h in self.pins.items())
        binaries = {}
        if "--message-format=json" in command:
            for line in (self.output / (name + ".stdout")).read_text(errors="replace").splitlines():
                if not line.startswith("{"):
                    continue
                artifact = json.loads(line)
                if artifact.get("reason") == "compiler-artifact" and artifact.get("executable"):
                    p = Path(artifact["executable"])
                    binaries[str(p)] = {
                        "sha256": sha(p), "bytes": p.stat().st_size, "features": artifact["features"],
                        "target": artifact["target"], "profile": artifact["profile"],
                    }
        row = {
            "name": name, "command": command, "exit_code": code, "elapsed_seconds": time.monotonic() - start,
            "external_stop_reason": reason, "resource_observations": observations, "source_input_mismatches": mismatches,
            "binaries": binaries, "environment_overrides": extra or {}, "rustdocflags": command_env.get("RUSTDOCFLAGS"),
            **{s + "_sha256": sha(self.output / (name + "." + s)) for s in ["stdout", "stderr", "resources"]},
        }
        self.rows.append(row)
        dump(self.output / "commands.json", self.rows)
        print(json.dumps({k: row[k] for k in ["name", "exit_code", "elapsed_seconds", "external_stop_reason"]}), flush=True)
        return code

    def finish(self, extra=None):
        mismatches = self.source_mismatches()
        generated = {
            str(p.relative_to(self.source)): sha(p) for p in self.source.rglob("*")
            if p.is_file() and str(p.relative_to(self.source)) not in self.inputs
        }
        result = {
            "complete": True, "command_failures": [r["name"] for r in self.rows if r["exit_code"]],
            "commands_not_run": self.not_run, "external_stop_reason": self.stop,
            "sources_unchanged": not mismatches, "source_input_mismatches": mismatches,
            "generated_outputs": generated, "cargo_target_closed": True,
            "main_sources_modified": False, "cache_admitted": False, "full_plan_complete": False,
            "full_pre_push_matrix_executed": False, "published_package_pipeline_executed": False,
            **(extra or {}),
        }
        dump(self.output / "result.json", result)
        print(json.dumps({"complete": True, "failures": result["command_failures"],
                          "commands_not_run": self.not_run, "external_stop_reason": self.stop}), flush=True)
        return 101 if result["command_failures"] or self.not_run or mismatches else 0
