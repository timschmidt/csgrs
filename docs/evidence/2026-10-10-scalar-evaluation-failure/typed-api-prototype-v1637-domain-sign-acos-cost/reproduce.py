"""Measure public coarse-warmed rational acos with frozen inputs and raw trials."""
import gzip
import hashlib
import json
from pathlib import Path
import resource
import shutil
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
TARGET = ROOT / "target/scalar-evaluation-candidate/target"
BUILD = EVIDENCE / sys.argv[1]
OUT = EVIDENCE / sys.argv[2]
WORK = ROOT / "target" / sys.argv[2]
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
assert read(BUILD / "result.json")["exit_code"] == 0
sources = json.loads(gzip.decompress((BUILD / "source-hashes.json.gz").read_bytes()))
main = json.loads(gzip.decompress((EVIDENCE / "candidate-preparation/main-hashes.json.gz").read_bytes()))
assert not OUT.exists() and not WORK.exists()
OUT.mkdir(); WORK.mkdir()

matches = [path for path in (TARGET / "release/deps").glob("libhyperreal-*.rlib")
           if sha(path) == sha(TARGET / "release/libhyperreal.rlib")]
assert len(matches) == 1
scalar = matches[0]
fingerprints = TARGET / "release/.fingerprint"
metadata = read(fingerprints / scalar.stem.removeprefix("lib") / "lib-hyperreal.json")
identity = next(row[3] for row in metadata["deps"] if row[1] == "serde_json")
matches = []
for path in (TARGET / "release/deps").glob("libserde_json-*.rlib"):
    receipt = fingerprints / path.stem.removeprefix("lib") / "lib-serde_json"
    if receipt.is_file() and int.from_bytes(bytes.fromhex(receipt.read_text()), "little") == identity:
        matches.append(path)
assert len(matches) == 1
libraries = {"hyperreal": scalar, "serde_json": matches[0]}
hashes = {str(path): sha(path) for path in libraries.values()}
shutil.copyfile(__file__, OUT / "reproduce.py")
shutil.copyfile(EVIDENCE / "acos_warm_cost_probe_v2.rs", OUT / "probe.rs")
frozen = {path: sha(path) for path in OUT.iterdir()}


def verify():
    assert all(sha(SOURCE / name) == value for name, value in sources.items())
    assert all(sha(ROOT / name) == value for name, value in main.items())
    assert all(sha(Path(name)) == value for name, value in hashes.items())
    assert all(sha(path) == value for path, value in frozen.items())


def cap():
    resource.setrlimit(resource.RLIMIT_AS, (192 * 1024 * 1024, 192 * 1024 * 1024))
    resource.setrlimit(resource.RLIMIT_CPU, (2, 3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


commands = []


def run(name, command, limited=False):
    command = list(map(str, command))
    started = time.monotonic()
    with (OUT / (name + ".stdout")).open("w") as stdout, (OUT / (name + ".stderr")).open("w") as stderr:
        try:
            result = subprocess.run(command, cwd=ROOT, stdout=stdout, stderr=stderr,
                                    timeout=4 if limited else 120, preexec_fn=cap if limited else None)
        except subprocess.TimeoutExpired:
            (OUT / "failed-attempt.json").write_text(json.dumps({"name": name, "command": command,
                "wall_timeout": True, "wall_seconds": time.monotonic() - started}, indent=2) + "\n")
            raise
    receipt = {"name": name, "command": command, "exit_code": result.returncode,
               "wall_seconds": time.monotonic() - started, "timeout_seconds": 4 if limited else 120,
               "address_space_cap_mib": 192 if limited else None, "cpu_seconds": [2, 3] if limited else None}
    commands.append(receipt)
    (OUT / "commands.json").write_text(json.dumps(commands, indent=2) + "\n")
    assert result.returncode == 0, receipt
    return (OUT / (name + ".stdout")).read_text()


verify()
run("format", ["rustfmt", "--edition", "2024", "--check", OUT / "probe.rs"])
command = ["rustc", "--edition=2024", "-O", "-D", "warnings", OUT / "probe.rs", "-L", "dependency=" + str(TARGET / "release/deps")]
for name, path in libraries.items():
    command += ["--extern", name + "=" + str(path)]
run("build", command + ["-o", WORK / "probe"])
frozen[WORK / "probe"] = sha(WORK / "probe")
print("Cost probe built; repeats run sequentially without overlapping workloads.", flush=True)
cases = [("small_positive", 2000), ("small_negative", 2000),
         ("mid_positive", 300), ("mid_negative", 300),
         ("wide_positive", 4), ("wide_negative", 4),
         ("cached_positive", 20000), ("cached_negative", 20000)]
rows = []
for repeat in range(9):
    for case, count in cases:
        name = f"repeat-{repeat}-{case}"
        row = json.loads(run(name, [WORK / "probe", case, count], True))
        assert row["case"] == case and row["iterations"] == count
        rows.append(row | {"repeat": repeat})
        (OUT / "receipts.json").write_text(json.dumps(rows, indent=2) + "\n")
    print("Repeat", repeat, "completed", flush=True)
verify()
summary = {"source_hashes_sha256": sha(BUILD / "source-hashes.json.gz"), "build_phase": BUILD.name,
           "libraries_sha256": hashes, "guards_unchanged": True, "processes": len(rows),
           "cases": {case: {"iterations": count,
               "elapsed_ns": [int(row["elapsed_ns"]) for row in rows if row["case"] == case],
               "median_ns": statistics.median(int(row["elapsed_ns"]) for row in rows if row["case"] == case),
               "checksums": sorted({row["checksum"] for row in rows if row["case"] == case}),
               "output_bytes": sorted({row["output_bytes"] for row in rows if row["case"] == case}),
               "reference": sorted({row["reference"] for row in rows if row["case"] == case})}
               for case, count in cases},
           "scope": "Eight public rational-acos micro-workloads with nine separate-process repetitions. Coarse-warmed independent roots then finer numeric misses, plus completed-cache controls. Timed approximation and complete decimal-output checksum only; fixture/reference creation and warming are excluded. All trials and failures retained. This is local timing evidence, not an independent numeric oracle or a full-board/admission/resource bound."}
assert all(len(value["checksums"]) == 1 for value in summary["cases"].values())
summary["binary_sha256"] = sha(WORK / "probe")
summary["binary_bytes"] = (WORK / "probe").stat().st_size
summary["artifacts_sha256"] = {str(path.relative_to(OUT)): sha(path) for path in OUT.iterdir() if path.is_file()}
(OUT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
print(json.dumps({case: values["median_ns"] for case, values in summary["cases"].items()}), flush=True)
