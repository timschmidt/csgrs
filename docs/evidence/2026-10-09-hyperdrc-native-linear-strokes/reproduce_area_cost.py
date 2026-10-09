"""Matched area costs after isolated validation and production replay finish."""

import hashlib
import json
from pathlib import Path
import shutil
import statistics
import subprocess
import time

ROOT = Path(__file__).resolve().parent
REPLAY = ROOT / "area-compaction-production-replay"
OUT = ROOT / "area-compaction-cost"
OUT.mkdir()
record = json.loads((REPLAY / "manifest.json").read_text())
assert record["source_unchanged"]
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
variants = record["variants"]
for variant in variants.values():
    variant["hashes"] = json.loads(Path(variant["source_record"]).read_text())["source_hashes"]


def verify():
    assert all(digest(Path(name)) == value["sha256"] for name, value in record["additional_fixture_inputs"].items())
    for variant in variants.values():
        assert all(digest(Path(variant["source_directory"]) / name) == value for name, value in variant["hashes"].items())
        assert all(digest(Path(name)) == value["sha256"] for name, value in variant["libraries"].items())


shutil.copyfile(ROOT / "area_compaction_cost_probe.rs", OUT / "probe.rs")
shutil.copyfile(__file__, OUT / "reproduce.py")
commands = []


def run(name, command):
    verify()
    start = time.monotonic()
    with (OUT / (name + ".jsonl")).open("w") as stdout, (OUT / (name + ".log")).open("w") as stderr:
        result = subprocess.run(command, stdout=stdout, stderr=stderr, check=False)
    commands.append({"name": name, "command": command, "exit_code": result.returncode, "wall_seconds": time.monotonic() - start})
    (OUT / "commands.json").write_text(json.dumps(commands, indent=2) + "\n")
    verify()
    assert result.returncode == 0, name
    print(name, "passed", flush=True)
    return (OUT / (name + ".jsonl")).read_text()


binaries = {}
for name, variant in variants.items():
    libraries = {crate: next(Path(path) for path in variant["libraries"] if Path(path).name.startswith("lib" + crate)) for crate in ["hypercurve", "hyperlimit"]}
    for mode in ["time", "memory"]:
        binary = Path("/tmp") / ("drc-area-cost-" + name + "-" + mode)
        command = ["rustc", "--edition=2024", "-O", "-D", "warnings", str(OUT / "probe.rs"), "-o", str(binary)]
        for crate, library in libraries.items():
            command += ["--extern", crate + "=" + str(library), "-L", "dependency=" + str(library.parent)]
            if (library.parent / "deps").is_dir():
                command += ["-L", "dependency=" + str(library.parent / "deps")]
        if mode == "memory":
            command += ["--cfg", 'feature="count-alloc"']
        run("build-" + name + "-" + mode, command)
        binaries[name, mode] = binary
for (name, mode), binary in binaries.items():
    run("preflight-" + name + "-" + mode, ["taskset", "-c", "0", str(binary), "1"])

trials = []
for mode, count in [("time", 128), ("memory", 64)]:
    for index in range(1, 6):
        for name in (["baseline", "candidate"] if index % 2 else ["candidate", "baseline"]):
            text = run(f"{mode}-{index}-{name}", ["taskset", "-c", "0", "/usr/bin/time", "-v", str(binaries[name, mode]), str(count)])
            rows = [json.loads(line) for line in text.splitlines()]
            assert len(rows) == 63
            witnesses = [row for row in rows if "witness" in row]
            assert len(witnesses) == 9 and all(row["all_areas_exact"] and row["count"] == count for row in witnesses)
            trials.append({"mode": mode, "index": index, "variant": name, "rows": rows})
summaries = []
for mode in ["time", "memory"]:
    selected = [trial for trial in trials if trial["mode"] == mode]
    witness = [row for row in selected[0]["rows"] if "witness" in row]
    assert all([row for row in trial["rows"] if "witness" in row] == witness for trial in selected)
    labels = [row["name"] for row in selected[0]["rows"] if "name" in row]
    assert len(labels) == 54
    for label in labels:
        values = {}
        for name in ["baseline", "candidate"]:
            rows = [next(row for row in trial["rows"] if row.get("name") == label) for trial in selected if trial["variant"] == name]
            fields = ["nanos"] if mode == "time" else ["allocation_calls", "allocated_bytes", "retained_bytes_delta", "peak_bytes_delta"]
            values[name] = {field: {"samples": [row[field] for row in rows], "median": statistics.median(row[field] for row in rows)} for field in fields}
        summaries.append({"mode": mode, "name": label, "variants": values})
binary_info = {}
for (name, mode), binary in binaries.items():
    sections = subprocess.check_output(["size", "-A", str(binary)], text=True)
    (OUT / (name + "-" + mode + "-size.log")).write_text(sections)
    binary_info[name + "-" + mode] = {"path": str(binary), "bytes": binary.stat().st_size, "sha256": digest(binary)}
manifest = {
    "scope": "Focused matched construction, first-per-object and repeated area queries for nine established exact rectangle/circle/capsule families. Five fresh CPU-0-pinned processes per variant and mode, alternating order. Timing executables use the system allocator; separate allocation executables count requested Rust heap bytes, excluding allocator metadata and native C heaps. Shared constant caches can warm within a process; this is not a guarantee of globally cold scalar state or a broad/competitive speedup.",
    "source_records": {name: variant["source_record"] for name, variant in variants.items()},
    "source_unchanged": True,
    "commands": commands,
    "trials": trials,
    "summaries": summaries,
    "binaries": binary_info,
    "probe_sha256": digest(OUT / "probe.rs"),
}
(OUT / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print("matched area cost probe complete", flush=True)
