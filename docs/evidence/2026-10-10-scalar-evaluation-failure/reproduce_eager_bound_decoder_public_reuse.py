"""Check public concurrent/reconstructed cache reuse against directed enclosures."""
from fractions import Fraction
import gzip
import hashlib
import json
from pathlib import Path
import re
import resource
import shutil
import subprocess
import sys
import time

sys.set_int_max_str_digits(20000)
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
TARGET = ROOT / "target/scalar-evaluation-candidate/target"
BUILD = E / sys.argv[1]
OUT = E / sys.argv[2]
WORK = ROOT / "target" / sys.argv[2]
UNIT = E / sys.argv[3]
PROTOCOL = E / sys.argv[4]
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
config = read(PROTOCOL)
assert not OUT.exists() and not WORK.exists()
assert read(BUILD / "result.json")["exit_code"] == 0
assert sha(BUILD / "source-hashes.json.gz") == config["source_record_sha256"]
assert sha(E / config["probe"]) == config["probe_sha256"]
assert sha(Path(__file__)) == config["runner_sha256"]
source_hashes = json.loads(gzip.decompress((BUILD / "source-hashes.json.gz").read_bytes()))
main_hashes = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
OUT.mkdir()
WORK.mkdir()


def dependency(parent, name):
    fingerprints = TARGET / "release/.fingerprint"
    metadata = read(fingerprints / parent.stem.removeprefix("lib") /
                    ("lib-" + parent.stem.split("-")[0].removeprefix("lib") + ".json"))
    identity = next(row[3] for row in metadata["deps"] if row[1] == name)
    paths = []
    for path in (TARGET / "release/deps").glob("lib" + name + "-*.rlib"):
        receipt = fingerprints / path.stem.removeprefix("lib") / ("lib-" + name)
        if receipt.is_file() and int.from_bytes(bytes.fromhex(receipt.read_text()), "little") == identity:
            paths.append(path)
    assert len(paths) == 1, (name, paths)
    return paths[0]


matches = [p for p in (TARGET / "release/deps").glob("libhyperreal-*.rlib")
           if sha(p) == sha(TARGET / "release/libhyperreal.rlib")]
assert len(matches) == 1
scalar = matches[0]
assert read(UNIT / "result.json")["exit_code"] == 0
assert sha(UNIT / "source-hashes.json.gz") == config["source_record_sha256"]
unit_binaries = re.findall(r"Running unittests src/lib\.rs \(([^)\n]*/hyperreal-[^)]+)\)",
                          (UNIT / "stderr.log").read_text())
assert len(unit_binaries) == 1
unit_fingerprint = TARGET / "release/.fingerprint" / Path(unit_binaries[0]).name / "test-lib-hyperreal.json"
rug_identity = next(row[3] for row in read(unit_fingerprint)["deps"] if row[1] == "rug")
rug_matches = []
for path in (TARGET / "release/deps").glob("librug-*.rlib"):
    receipt = TARGET / "release/.fingerprint" / path.stem.removeprefix("lib") / "lib-rug"
    if receipt.is_file() and int.from_bytes(bytes.fromhex(receipt.read_text()), "little") == rug_identity:
        rug_matches.append(path)
assert len(rug_matches) == 1
libraries = {"hyperreal": scalar, "serde_json": dependency(scalar, "serde_json"), "rug": rug_matches[0]}
library_hashes = {str(p): sha(p) for p in libraries.values()}
prior_libraries = {name: row["candidate_sha256"] for name, row in read(E / "eager-bound-decoder-binary-comparison-v2262/summary.json")["libraries"].items()}
assert prior_libraries[str(scalar)] == sha(scalar)
(OUT / "library-selection.json").write_text(json.dumps({
    "unit_phase": UNIT.name, "unit_fingerprint": str(unit_fingerprint),
    "unit_fingerprint_sha256": sha(unit_fingerprint), "rug_dependency_identity": rug_identity,
}, indent=2) + "\n")
shutil.copyfile(__file__, OUT / "reproduce.py")
shutil.copyfile(PROTOCOL, OUT / "protocol.json")
shutil.copyfile(E / config["probe"], OUT / "probe.rs")
frozen = {p: sha(p) for p in OUT.iterdir() if p.is_file()}


def verify():
    assert all(sha(SOURCE / name) == value for name, value in source_hashes.items())
    assert all(sha(ROOT / name) == value for name, value in main_hashes.items())
    assert all(sha(Path(name)) == value for name, value in library_hashes.items())
    assert all(sha(p) == value for p, value in frozen.items())
    assert sha(PROTOCOL) == sha(OUT / "protocol.json")


def cap():
    resource.setrlimit(resource.RLIMIT_AS, (192 * 1024 * 1024, 192 * 1024 * 1024))
    resource.setrlimit(resource.RLIMIT_CPU, (2, 3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


commands = []


def run(name, command, capped=False):
    command = list(map(str, command))
    start = time.monotonic()
    timeout = False
    with (OUT / (name + ".stdout")).open("w") as stdout, (OUT / (name + ".stderr")).open("w") as stderr:
        try:
            result = subprocess.run(command, cwd=ROOT, stdout=stdout, stderr=stderr,
                                    timeout=4 if capped else 120, preexec_fn=cap if capped else None)
            code = result.returncode
        except subprocess.TimeoutExpired:
            code = None
            timeout = True
    receipt = {"name": name, "command": command, "exit_code": code,
               "wall_seconds": time.monotonic() - start, "wall_timeout": timeout,
               "timeout_seconds": 4 if capped else 120,
               "address_space_cap_mib": 192 if capped else None,
               "cpu_seconds": [2, 3] if capped else None}
    commands.append(receipt)
    (OUT / "commands.json").write_text(json.dumps(commands, indent=2) + "\n")
    return receipt, (OUT / (name + ".stdout")).read_text()


def verify_samples(record):
    if record["mode"] == "invalid":
        assert len(record["result"]["rows"]) == 16
        assert record["result"]["unlimited_retries"] == 4
        return 0, 0
    oracle = record["oracle"]
    lo, hi = Fraction(oracle["lower"]), Fraction(oracle["upper"])
    assert lo <= hi
    assert (hi < 0) if record["negative"] else (lo > 0)
    expected = "Negative" if record["negative"] else "Positive"
    assert oracle["expected_sign"] == expected and oracle["precision_bits"] == 2560
    numeric = definite_signs = 0

    def visit(value):
        nonlocal numeric, definite_signs
        if isinstance(value, list):
            for child in value:
                visit(child)
        elif isinstance(value, dict):
            if "sample" in value:
                unit = Fraction(2) ** value["precision"]
                center = int(value["sample"]) * unit
                assert center - unit <= lo <= hi <= center + unit
                numeric += 1
            if "sign" in value and isinstance(value["sign"], str):
                assert value["sign"] == expected
                definite_signs += 1
            for child in value.values():
                visit(child)
    visit(record["result"])
    return numeric, definite_signs


verify()
assert run("format", ["rustfmt", "--edition", "2024", "--check", OUT / "probe.rs"])[0]["exit_code"] == 0
command = ["rustc", "--edition=2024", "-O", "-D", "warnings", OUT / "probe.rs",
           "-L", "dependency=" + str(TARGET / "release/deps")]
for name, path in libraries.items():
    command += ["--extern", name + "=" + str(path)]
assert run("build", command + ["-o", WORK / "probe"])[0]["exit_code"] == 0
frozen[WORK / "probe"] = sha(WORK / "probe")
verify()
print("Public proof-reuse probe built; no builds during capped workloads.", flush=True)

rows = []
total_numeric = total_definite_signs = 0
for args in config["cases"]:
    name = "case-" + str(len(rows))
    command, output = run(name, [WORK / "probe", *args], True)
    records = [json.loads(line) for line in output.splitlines()]
    complete = command["exit_code"] == 0 and len(records) == 1
    row = {"name": name, "arguments": args, "command": command, "complete": complete, "records": records}
    rows.append(row)
    (OUT / "receipts.json").write_text(json.dumps(rows, indent=2) + "\n")
    assert complete, (name, command)
    record = records[0]
    assert record["mode"] == args[0]
    numeric, definite = verify_samples(record)
    total_numeric += numeric
    total_definite_signs += definite
    if len(rows) % 12 == 0:
        print("Completed", len(rows), "public proof-reuse processes", flush=True)
assert len(rows) == len(config["cases"]) == 126
verify()
summary = {
    "cases": len(rows), "cases_by_mode": {mode: sum(r["arguments"][0] == mode for r in rows)
        for mode in ["sequential", "warm", "race", "invalid"]},
    "numeric_samples_checked": total_numeric, "definite_sign_observations_checked": total_definite_signs,
    "source_hashes_sha256": config["source_record_sha256"], "build_phase": BUILD.name,
    "libraries_sha256": library_hashes, "binary_sha256": sha(WORK / "probe"),
    "protocol_sha256": sha(PROTOCOL), "guards_unchanged": True,
    "incomplete_cases": 0, "invalid_samples": 0,
    "scope": config["scope"], "limits": config["limits"],
    "artifacts_sha256": {str(p.relative_to(OUT)): sha(p) for p in OUT.iterdir() if p.is_file()},
}
(OUT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
print(json.dumps({k: v for k, v in summary.items() if k in {
    "cases", "cases_by_mode", "numeric_samples_checked", "definite_sign_observations_checked", "guards_unchanged"}}), flush=True)
