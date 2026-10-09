"""Replay complete reports in two fresh processes with identical source paths.

Run after validation, consumer checks, and cost jobs have exited:
python3 reproduce_report_replay.py /tmp/layer-order-cost /tmp/layer-order-reports
Only RunOutcome.report is serialized; RunOutcome.elapsed is not a report field.
No report field, identifier, row, coordinate, or source path is normalized.
"""
import gzip
import hashlib
import json
import pathlib
import re
import shutil
import subprocess
import sys

root = pathlib.Path.cwd()
record = pathlib.Path(__file__).resolve().parent
cost = pathlib.Path(sys.argv[1]).resolve()
out = pathlib.Path(sys.argv[2]).resolve()
out.mkdir(parents=True, exist_ok=False)
cost_manifest = json.loads((cost / "manifest.json").read_text())
assert cost_manifest["source_unchanged"] and cost_manifest["fixture_exit_code"] == 0
source_hashes = cost_manifest["source_hashes"]
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
info = lambda path: dict(bytes=path.stat().st_size, sha256=digest(path))
current_sources = lambda: {path: digest(pathlib.Path(path)) for path in source_hashes}
assert current_sources() == source_hashes

artifacts = [json.loads(line) for line in (cost / "library-build.jsonl").read_text().splitlines()]


def rlib(name):
    files = [pathlib.Path(filename) for artifact in artifacts
             if artifact.get("reason") == "compiler-artifact" and artifact["target"]["name"] == name
             for filename in artifact["filenames"] if filename.endswith(".rlib")]
    assert len(files) == 1, (name, files)
    return files[0]


linked = {name: rlib(name) for name in ["hyperdrc", "hyperlimit", "serde_json", "clap"]}
assert info(linked["hyperdrc"]) == cost_manifest["binaries"][str(linked["hyperdrc"]) ]
commands = []
for stem, dependencies in [
    ("full_board_witness", ["hyperdrc", "hyperlimit", "serde_json"]),
    ("full_report_replay", ["hyperdrc", "serde_json", "clap"]),
]:
    command = ["rustc", "--edition=2024", "-O", str(record / f"{stem}.rs"),
               "-L", f"dependency={root / 'target/release/deps'}"]
    for name in dependencies:
        command += ["--extern", f"{name}={linked[name]}"]
    command += ["-o", str(out / stem)]
    with (out / f"{stem}-build.log").open("w") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    commands.append(dict(command=command, exit_code=0))

inputs = out / "inputs"
archives = {}
for label, archive in [("cparti-fpga", "CPArti FPGA dev board.zip"), ("hvp109a", "HVP109A.zip")]:
    source = root / "crates/hyperdrc/docs" / archive
    destination = inputs / label
    destination.mkdir(parents=True)
    command = ["unzip", "-q", str(source), "-d", str(destination)]
    subprocess.run(command, check=True)
    commands.append(dict(command=command, exit_code=0))
    archives[str(source.relative_to(root))] = info(source)
subset = inputs / "cparti-fpga/gerber-smoke-subset"
subset.mkdir()
edge = "CPArti FPGA dev board-Edge_Cuts.gbr"
shutil.copyfile(inputs / "cparti-fpga/gerbers" / edge, subset / edge)
input_files = {str(path.relative_to(inputs)): info(path) for path in sorted(inputs.rglob("*")) if path.is_file()}

command = [str(out / "full_board_witness"), str(inputs / "cparti-fpga/CPArti FPGA dev board.kicad_pcb")]
with (out / "witness.json").open("w") as stdout, (out / "witness.log").open("w") as stderr:
    subprocess.run(command, stdout=stdout, stderr=stderr, check=True)
commands.append(dict(command=command, exit_code=0))
before = json.loads((record / "baseline-full-board/witness.json").read_text())
after = json.loads((out / "witness.json").read_text())
canonical = next(row for row in before["input_order_variants"] if row["layers"] == ["B.Cu", "In1.Cu"])
assert after["input_order_results"] == [canonical, canonical]

trials = []
for trial in [1, 2]:
    command = ["taskset", "-c", "0", "/usr/bin/time", "-v", str(out / "full_report_replay"),
               str(inputs), str(out / f"trial-{trial}")]
    with (out / f"trial-{trial}.log").open("w") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    text = (out / f"trial-{trial}.log").read_text()
    trials.append(dict(command=command, exit_code=0,
                       peak_rss_kib=int(re.search(r"Maximum resident set size \(kbytes\): (\d+)", text)[1])))
    print(f"complete report trial {trial}: passed", flush=True)

reports = {}
for label in ["cparti-fpga", "hvp109a", "cparti-gerber"]:
    paths = [out / f"trial-{trial}/{label}.json" for trial in [1, 2]]
    assert paths[0].read_bytes() == paths[1].read_bytes(), label
    report = json.loads(paths[0].read_bytes())
    assert len(report["violations"]) == report["violation_count"]
    for trial, path in enumerate(paths, 1):
        (out / f"trial-{trial}-{label}.json.gz").write_bytes(gzip.compress(path.read_bytes(), mtime=0))
    reports[label] = dict(**info(paths[0]), identical_complete_bytes=True,
                          violations=report["violation_count"], inputs=len(report["inputs"]),
                          summary=report["summary"],
                          compressed_artifacts={f"trial-{trial}-{label}.json.gz": info(out / f"trial-{trial}-{label}.json.gz") for trial in [1, 2]})
assert input_files == {str(path.relative_to(inputs)): info(path) for path in sorted(inputs.rglob("*")) if path.is_file()}
manifest = dict(scope="Two fresh processes serialize the complete public Report with identical input paths, all fields and findings retained, for both full KiCad boards and the unchanged single-edge-cuts Gerber smoke subset. No normalization. This is fixture/check-specific determinism, not path-independent IDs or universal report determinism.",
                commands=commands, trials=trials, archives=archives, input_files=input_files,
                reports=reports, canonical_duplicate_matches_baseline_variant=True,
                sources={name: info(record / f"{name}.rs") for name in ["full_board_witness", "full_report_replay"]},
                binaries={str(out / name): info(out / name) for name in ["full_board_witness", "full_report_replay"]},
                linked_libraries={str(path): info(path) for path in linked.values()},
                cost_library_build=info(cost / "library-build.jsonl"),
                baseline_witness=info(record / "baseline-full-board/witness.json"),
                source_hashes=source_hashes, source_unchanged=current_sources() == source_hashes)
(out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
assert manifest["source_unchanged"]
print(json.dumps(reports, indent=2), flush=True)
