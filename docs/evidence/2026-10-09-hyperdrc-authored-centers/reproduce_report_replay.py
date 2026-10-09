"""Replay the eight affected checks on both full boards, with fixed source paths.

Run after validation, consumer checks and cost jobs have exited:
python3 reproduce_report_replay.py COST_OUTPUT REPORT_OUTPUT
The baseline input files must be present at the recorded fixed paths. Extract
the retained archives there if needed; their complete inventory is checked.
"""
import gzip
import hashlib
import json
import pathlib
import re
import subprocess
import sys

root = pathlib.Path.cwd()
record = pathlib.Path(__file__).resolve().parent
cost, out = [pathlib.Path(p).resolve() for p in sys.argv[1:3]]
out.mkdir(parents=True, exist_ok=False)
load = lambda path: json.loads(path.read_bytes())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
info = lambda path: dict(bytes=path.stat().st_size, sha256=digest(path))
manifest = load(cost / "manifest.json")
assert manifest["source_unchanged"] and manifest["fixture_exit_code"] == 0
source_hashes = manifest["source_hashes"]
current_sources = lambda: {p: digest(pathlib.Path(p)) for p in source_hashes}
assert current_sources() == source_hashes
baseline = load(record / "baseline-reports/manifest.json")
inputs = pathlib.Path(baseline["command"][-2])
input_files = lambda: {str(p.relative_to(inputs)): info(p) for p in sorted(inputs.rglob("*")) if p.is_file()}
assert input_files() == baseline["input_files"]
for path, expected in baseline["archives"].items():
    assert info(root / path) == expected

artifacts = [json.loads(line) for line in (cost / "library-build.jsonl").read_text().splitlines()]
def rlib(name):
    return next(pathlib.Path(f) for a in artifacts
                if a.get("reason") == "compiler-artifact" and a["target"]["name"] == name
                for f in a["filenames"] if f.endswith(".rlib"))

linked = {name: rlib(name) for name in ["hyperdrc", "clap", "serde_json"]}
assert info(linked["hyperdrc"]) == manifest["binaries"][str(linked["hyperdrc"])]
commands = []
for stem, names, flags in [
    ("full_report_replay", ["hyperdrc", "clap", "serde_json"], []),
]:
    command = ["rustc", "--edition=2024", "-O", str(record / f"{stem}.rs"),
               "-L", f"dependency={root}/target/release/deps", *flags]
    for name in names:
        command += ["--extern", f"{name}={linked[name]}"]
    command += ["-o", str(out / stem)]
    with (out / f"{stem}-build.log").open("w") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    commands.append(dict(command=command, exit_code=0))
trials = []
for trial in [1, 2]:
    command = ["taskset", "-c", "0", "/usr/bin/time", "-v", str(out / "full_report_replay"),
               str(inputs), str(out / f"trial-{trial}")]
    with (out / f"trial-{trial}.log").open("w") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    log = (out / f"trial-{trial}.log").read_text()
    trials.append(dict(command=command, exit_code=0,
                       peak_rss_kib=int(re.search(r"Maximum resident set size \(kbytes\): (\d+)", log)[1])))
    print(f"complete affected-check report trial {trial}: passed", flush=True)

reports = {}
for label in ["cparti-fpga", "hvp109a"]:
    paths = [out / f"trial-{trial}/{label}.json" for trial in [1, 2]]
    raw = paths[0].read_bytes()
    assert raw == paths[1].read_bytes()
    before = gzip.decompress((record / f"baseline-reports/{label}.json.gz").read_bytes())
    version_count = before.count(b'"check_version":13')
    expected = before.replace(b'"check_version":13', b'"check_version":14')
    assert version_count > 0
    report = json.loads(raw)
    assert len(report["violations"]) == report["violation_count"]
    for trial, path in enumerate(paths, 1):
        (out / f"trial-{trial}-{label}.json.gz").write_bytes(gzip.compress(path.read_bytes(), mtime=0))
    reports[label] = dict(**info(paths[0]), identical_complete_bytes=True,
                          baseline_only_version_changed=raw == expected,
                          baseline_version_fields=version_count,
                          violations=report["violation_count"], summary=report["summary"],
                          compressed_artifacts={f"trial-{trial}-{label}.json.gz": info(out / f"trial-{trial}-{label}.json.gz") for trial in [1, 2]})

result = dict(scope="Two fresh processes serialize the complete public Report for all eight affected authored-center and selector checks on both full KiCad boards at fixed input paths. No candidate report normalization. Baseline comparison records version and presentation deltas; audit_report_deltas.py permits only the six recorded connector fixed-decimal messages and eight version fields per board. Every other byte remains checked. No universal or path-independent report determinism claim.",
              commands=commands, trials=trials, reports=reports,
              input_files=baseline["input_files"], archives=baseline["archives"],
              sources={name: info(record / f"{name}.rs") for name in ["full_report_replay"]},
              binaries={str(out / name): info(out / name) for name in ["full_report_replay"]},
              linked_libraries={str(p): info(p) for p in linked.values()},
              source_hashes=source_hashes, source_unchanged=current_sources() == source_hashes,
              cost_library_build=info(cost / "library-build.jsonl"))
(out / "manifest.json").write_text(json.dumps(result, indent=2) + "\n")
assert result["source_unchanged"] and input_files() == baseline["input_files"]
print(json.dumps(reports, indent=2), flush=True)
subprocess.run([sys.executable, str(record / "audit_report_deltas.py"), str(out)], check=True)
