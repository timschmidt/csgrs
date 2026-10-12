"""Correct profiler placement after the first protocol's loader failure."""
import ast
import difflib
import hashlib
import json
from pathlib import Path
import sys

E = Path(__file__).resolve().parent
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    prior_path = E / "integer-coarse-zero-baseline-profile-v2157/summary.json"
    prior = read(prior_path)
    assert prior["attempted_profiles"] == 1
    row = prior["attempts"][0]
    assert row["exit_code"] == 127 and row["perf_data_bytes"] == 0 and row["receipt"] is None
    stderr = E / "integer-coarse-zero-baseline-profile-v2157/baseline.stderr"
    assert "error while loading shared libraries" in stderr.read_text()
    old = E / "profile_integer_coarse_zero_baseline.py"
    new = E / "profile_integer_coarse_zero_baseline_v2.py"
    protocol_path = E / "integer-coarse-zero-baseline-profile-protocol-v2158.json"
    patch_path = E / "integer-coarse-zero-baseline-profile-correction-v2158.patch"
    assert not any(p.exists() for p in [new, protocol_path, patch_path])
    source = old.read_text().replace("profile-protocol-v2157.json", "profile-protocol-v2158.json")
    replacements = [
        ('    assert sha(Path(protocol["perf"])) == protocol["perf_sha256"]',
         '    assert sha(Path(protocol["perf"])) == protocol["perf_sha256"]\n'
         '    assert sha(E / protocol["workload_launcher"]) == protocol["workload_launcher_sha256"]'),
        ('        assert command[-7:] == [run["binary"]] + run["arguments"]',
         '        assert command[-5:] == ["--", sys.executable, str(E / protocol["workload_launcher"]), "exec", run["which"]]'),
        ('        proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,\n'
         '                                preexec_fn=limits, start_new_session=True)',
         '        def profiler_limits():\n'
         '            os.sched_setaffinity(0, {protocol["cpu"]})\n'
         '            resource.setrlimit(resource.RLIMIT_AS, (protocol["profiler_address_space_bytes"],) * 2)\n'
         '            resource.setrlimit(resource.RLIMIT_CPU, tuple(protocol["profiler_cpu_seconds"]))\n\n'
         '        proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,\n'
         '                                preexec_fn=profiler_limits, start_new_session=True)'),
        ('        row["perf_data_bytes"] = data_path.stat().st_size if data_path.exists() else 0',
         '        limits_path = output / (run["which"] + "-effective-workload-limits.json")\n'
         '        row["workload_launcher_ran"] = limits_path.exists()\n'
         '        if limits_path.exists():\n'
         '            effective = read(limits_path)\n'
         '            assert effective["address_space_bytes"] == [201326592, 201326592]\n'
         '            assert effective["cpu_seconds"] == [2, 3] and effective["affinity"] == [0]\n'
         '            assert effective["command"] == [run["binary"]] + run["arguments"]\n'
         '            row["effective_workload_limits"] = effective\n'
         '        row["perf_data_bytes"] = data_path.stat().st_size if data_path.exists() else 0'),
    ]
    for before, after in replacements:
        assert source.count(before) == 1
        source = source.replace(before, after)
    ast.parse(source)
    launcher = E / "exec_integer_coarse_zero_profile_workload.py"
    ast.parse(launcher.read_text())
    new.write_text(source)
    patch_path.write_text("".join(difflib.unified_diff(old.read_text().splitlines(True), source.splitlines(True), fromfile=old.name, tofile=new.name)))
    protocol = read(E / "integer-coarse-zero-baseline-profile-protocol-v2157.json")
    protocol["status"] = "Frozen diagnostic harness correction before launching either workload"
    protocol["output"] = "integer-coarse-zero-baseline-profile-v2158"
    protocol["runner"], protocol["runner_sha256"] = new.name, sha(new)
    protocol["generator_sha256"] = sha(Path(__file__))
    protocol["prior_setup_failure"] = str(prior_path.relative_to(E))
    protocol["prior_setup_failure_sha256"] = sha(prior_path)
    protocol["predecessor_runner"] = old.name
    protocol["predecessor_runner_sha256"] = sha(old)
    protocol["correction_patch"], protocol["correction_patch_sha256"] = patch_path.name, sha(patch_path)
    protocol["workload_launcher"], protocol["workload_launcher_sha256"] = launcher.name, sha(launcher)
    protocol["profiler_address_space_bytes"], protocol["profiler_cpu_seconds"] = 1073741824, [4, 5]
    for run in protocol["runs"]:
        command = run["command"]
        separator = command.index("--")
        assert command[separator + 1:] == [run["binary"]] + run["arguments"]
        run["command"] = [arg.replace("baseline-profile-v2157", "baseline-profile-v2158") for arg in command[:separator + 1]]
        run["command"] += [sys.executable, str(launcher), "exec", run["which"]]
    protocol["limits"][1] = (
        "Both use the unchanged 64-value fourth-root/four-factor workload and original numerical/zero assertions. "
        "The launcher applies the original 192 MiB and CPU 2/3-second caps immediately before exec of the "
        "hash-identical workload; its small startup CPU consumption also counts against that same limit. "
        "The profiler separately has a 1 GiB address-space and CPU 4/5-second limit. The unchanged four-second "
        "supervisor deadline includes profiler startup, the complete workload and profiler shutdown."
    )
    protocol["limits"].append(
        "The first profiler process failed in the dynamic loader at its mistakenly shared 192 MiB cap, "
        "before it could launch the baseline. Preserve that failure. This fork corrects which process "
        "receives each limit; it does not rerun a completed or failed benchmark process, change the "
        "original acceptance protocol, or remove profiler/wrapper overhead from the diagnostic."
    )
    protocol_path.write_text(json.dumps(protocol, indent=2) + "\n")
    print(json.dumps({"protocol": protocol_path.name, "workload_cpu_seconds": [2, 3],
                      "workload_address_space_bytes": 201326592, "runtime_retained": False}))


if __name__ == "__main__":
    main()
