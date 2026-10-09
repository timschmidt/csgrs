"""Full demo inputs, five fresh release processes for both adapters.

Run from the repository root after reproduce_validation.py:
python3 reproduce_cost.py VALIDATION_OUTPUT COST_OUTPUT
No Cargo builds or production source edits may overlap this runner.
"""
import hashlib
import json
import re
import shutil
import statistics
import subprocess
import sys
from pathlib import Path

record = Path(__file__).resolve().parent
root = Path.cwd()
validation, output = map(Path, sys.argv[1:3])
output.mkdir(parents=True, exist_ok=False)
demo = root / "crates/hypercurve/examples/hypercurve_ui"
geometry_path = "crates/hypercurve/examples/hypercurve_ui/src/geometry.rs"
inputs = json.loads((record / "cost-inputs.json").read_text())


def info(path):
    data = path.read_bytes()
    return dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())


def sources():
    return {
        str(p.relative_to(root)): info(p)["sha256"]
        for p in sorted((root / "crates/hypercurve").rglob("*.rs"))
        if "target" not in p.parts
    }


frozen = sources()
artifacts = [json.loads(line) for line in (validation / "library-build.jsonl").read_text().splitlines()]


def rlib(name):
    return next(Path(f) for a in artifacts if a.get("reason") == "compiler-artifact"
                and a["target"]["name"] == name
                # The demo's target dependencies unify serde's rc feature;
                # the separate build-host serde artifact is incompatible.
                and (name != "serde" or "rc" in a["features"])
                for f in a["filenames"] if f.endswith(".rlib"))


linked = {name: rlib(name) for name in ["hypercurve", "geo", "serde", "serde_json"]}
linked_before = {str(p): info(p) for p in linked.values()}
baseline = output / "baseline-source"
baseline.mkdir()
baseline_geometry = baseline / "geometry.rs"
baseline_geometry.write_bytes(subprocess.check_output([
    "git", "show", f'{inputs["base_revision"]}:{geometry_path}'
]))
assert info(baseline_geometry)["sha256"] == inputs["baseline_geometry"]["sha256"]
assert info(record / "shared-blocker.json")["sha256"] == inputs["shared_fixture_sha256"]
versions = {}
for version, source in [("baseline", baseline_geometry), ("candidate", root / geometry_path)]:
    directory = output / version
    directory.mkdir()
    shutil.copy2(source, directory / "geometry.rs")
    if source.with_suffix("").is_dir():
        shutil.copytree(source.with_suffix(""), directory / "geometry")
    wrapper = directory / "wrapper.rs"
    wrapper.write_text('#![allow(dead_code)]\n'
                       'mod geometry;\n'
                       f'include!({json.dumps(str(record / "cost_probe.rs"))});\n')
    binary = directory / "probe"
    command = ["rustc", "--edition=2024", "-O", str(wrapper),
               "-L", f"dependency={demo / 'target/release/deps'}"]
    for name, path in linked.items():
        command += ["--extern", f"{name}={path}"]
    command += ["-o", str(binary)]
    with (directory / "build.log").open("w") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    versions[version] = dict(build_command=command, binary=info(binary), observations=[])

for trial in range(1, 6):
    for version in versions:
        directory = output / version
        run = ["taskset", "-c", "0", "/usr/bin/time", "-v", str(directory / "probe"),
               str(record / "shared-blocker.json"), str(directory / f"previews-{trial}")]
        with (directory / f"trial-{trial}.json").open("w") as stdout, \
                (directory / f"trial-{trial}.log").open("w") as stderr:
            subprocess.run(run, stdout=stdout, stderr=stderr, check=True)
        rows = json.loads((directory / f"trial-{trial}.json").read_text())
        assert len(rows) == 12
        for row in rows:
            if row["status"] == "decided":
                row["preview"] = info(directory / f"previews-{trial}" / f'{row["name"]}.json')
            if version == "candidate":
                assert row["status"] == "decided", row
                assert row["retains_exact_region"] and row["segmented_retains_exact_region"], row
        rss = int(re.search(r"Maximum resident set size \(kbytes\): (\d+)",
                           (directory / f"trial-{trial}.log").read_text())[1])
        versions[version]["observations"].append(dict(rows=rows, peak_rss_kib=rss))
        print(version, trial, rss, flush=True)


def identities(observation):
    return [{k: v for k, v in row.items() if k != "elapsed_ns"} for row in observation["rows"]]


for version, data in versions.items():
    observations = data["observations"]
    assert all(identities(o) == identities(observations[0]) for o in observations)
    data["rows"] = identities(observations[0])
    data["medians_ns"] = {
        row["name"]: statistics.median(o["rows"][index]["elapsed_ns"] for o in observations)
        for index, row in enumerate(observations[0]["rows"])
    }
    (output / version / "rows.json").write_text(
        json.dumps(data["rows"], sort_keys=True, separators=(",", ":")) + "\n")

deltas = []
for before, after in zip(versions["baseline"]["rows"], versions["candidate"]["rows"]):
    assert before["name"] == after["name"]
    if before["status"] == "decided":
        assert before["preview"] == after["preview"], (before, after)
    deltas.append(dict(name=before["name"], before=before, after=after))
assert sources() == frozen, "production Rust sources changed during cost capture"
assert linked_before == {str(p): info(p) for p in linked.values()}
manifest = dict(
    scope="Capability cost for the unchanged full default, multi and shared demo pairs. "
          "Twelve operation rows, five sequential fresh processes per adapter, CPU 0. "
          "Input construction and independent source-retention checks are outside timers but in RSS. "
          "Previously successful serialized previews must match exactly. Retained-source booleans "
          "may change from false to true and newly recovered rows have no matched speedup. "
          "RSS and standalone probe sizes are proxies, not allocation counts or demo/library sizes.",
    inputs=inputs, probe_source=info(record / "cost_probe.rs"), source_hashes=frozen,
    sources_unchanged=True, linked_libraries=linked_before, versions=versions, deltas=deltas,
)
(output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print("cost capture passed", flush=True)
