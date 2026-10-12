"""Compare freshly compiled decoder probes with their frozen retained executables."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    protocol_path = E / "eager-bound-decoder-binary-comparison-protocol-v2262.json"
    protocol = read(protocol_path)
    validation_path = E / "eager-bound-decoder-early-validation-v2261.json"
    validation = read(validation_path)
    assert validation["complete"] and len(validation["phases"]) == 4
    expected = protocol["source_record_sha256"]
    assert validation["source_record_sha256"] == expected
    for phase, row in validation["phases"].items():
        assert read(E / phase / "result.json") == row["result"]
        assert row["result"]["exit_code"] == 0 and row["result"]["source_unchanged"] and row["result"]["main_unchanged"]
        assert sha(E / phase / "source-hashes.json.gz") == expected
    for key in ["baseline_protocol", "baseline_summary", "retained_library_archive"]:
        assert sha(E / protocol[key]) == protocol[key + "_sha256"]
    original_library = gzip.decompress((E / protocol["retained_library_archive"]).read_bytes())
    assert hashlib.sha256(original_library).hexdigest() == protocol["retained_library_sha256"]
    assert len(original_library) == protocol["retained_library_bytes"]
    source_record = E / protocol["source_record"]
    assert sha(source_record) == expected
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    out = E / "eager-bound-decoder-binary-comparison-v2262"
    binaries = ROOT / "target/eager-bound-decoder-binary-comparison-v2262"
    assert not out.exists() and not binaries.exists()
    libraries = {}
    for row in protocol["build_commands"]:
        assert sha(E / row["probe"]) == row["probe_sha256"]
        assert sha(Path(row["baseline_binary"])) == row["baseline_binary_sha256"]
        assert sha(Path(row["original_binary"])) == row["original_binary_sha256"] == row["baseline_binary_sha256"]
        for name, old_digest in row["baseline_libraries_sha256"].items():
            current = sha(Path(name))
            if name != protocol["retained_library"]:
                assert current == old_digest, name
            libraries[name] = {"retained_sha256": old_digest, "candidate_sha256": current,
                               "identical": current == old_digest, "candidate_bytes": Path(name).stat().st_size}
    out.mkdir()
    binaries.mkdir()
    (out / "preparation.json").write_text(json.dumps({"protocol_sha256": sha(protocol_path),
        "validation_sha256": sha(validation_path), "runner_sha256": sha(Path(__file__)),
        "libraries": libraries, "no_timing_processes": True}, indent=2) + "\n")
    rows = []
    for item in protocol["build_commands"]:
        start = time.monotonic()
        result = subprocess.run(item["command"], cwd=ROOT, capture_output=True, timeout=120)
        (out / (item["family"] + ".stdout")).write_bytes(result.stdout)
        (out / (item["family"] + ".stderr")).write_bytes(result.stderr)
        executable = Path(item["command"][item["command"].index("-o") + 1])
        row = {"family": item["family"], "command": item["command"], "exit_code": result.returncode,
            "compile_seconds": time.monotonic() - start, "binary": str(executable),
            "binary_sha256": sha(executable) if executable.exists() else None,
            "baseline_binary_sha256": item["baseline_binary_sha256"],
            "baseline_bytes": Path(item["baseline_binary"]).stat().st_size,
            "candidate_bytes": executable.stat().st_size if executable.exists() else None}
        row["identical_executable_bytes"] = row["binary_sha256"] == row["baseline_binary_sha256"]
        rows.append(row)
        (out / "build-results.json").write_text(json.dumps(rows, indent=2) + "\n")
        assert result.returncode == 0 and result.stderr == b"", item["family"]
    assert all(sha(Path(n)) == h["candidate_sha256"] for n, h in libraries.items())
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    summary = {"protocol": protocol_path.name, "protocol_sha256": sha(protocol_path),
        "source_record_sha256": expected, "builds_passed": len(rows), "results": rows, "libraries": libraries,
        "all_probe_binaries_identical": all(r["identical_executable_bytes"] for r in rows),
        "changed_probe_families": [r["family"] for r in rows if not r["identical_executable_bytes"]],
        "new_timing_processes": 0, "original_cost_failures_preserved": True, "runtime_retained": False,
        "source_files_verified": len(sources), "main_files_verified": len(guard),
        "limits": [protocol["decision"],
            "This compares complete seven-probe executable bytes, not only instruction shapes or selected functions. A matching probe may reuse its own unchanged fixtures' historical receipts; it is not evidence for unrelated programs or execution environments.",
            "Library identity is recorded independently. No current timing, allocation, memory-pressure, native/full-board or pre-Verus performance pass is inferred. The old retained-baseline CPU failures remain part of the resource evidence."],
        "artifacts_sha256": {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file()}}
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"builds_passed": len(rows), "identical_binaries": sum(r["identical_executable_bytes"] for r in rows),
        "changed_families": summary["changed_probe_families"], "new_timing_processes": 0, "runtime_retained": False}))


if __name__ == "__main__":
    main()
