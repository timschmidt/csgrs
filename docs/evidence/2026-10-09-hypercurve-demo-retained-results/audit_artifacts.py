"""Check source archives, raw preview witnesses and the evidence inventory."""
import hashlib
import json
import tarfile
from pathlib import Path

record = Path(__file__).resolve().parent
root = record.parents[2]
read = lambda name: json.loads((record / name).read_text())
digest = lambda data: hashlib.sha256(data).hexdigest()
manifest = read("manifest.json")
for name, expected in manifest["artifacts"].items():
    data = (record / name).read_bytes()
    assert len(data) == expected["bytes"] and digest(data) == expected["sha256"], name
for archive, metadata in [
    ("initial-focused-sources.tar.gz", "initial-focused.json"),
    ("initial-regression-assertion/sources.tar.gz", "initial-regression-assertion/manifest.json"),
]:
    expected = read(metadata)["source_hashes"]
    with tarfile.open(record / archive) as saved:
        actual = {m.name: digest(saved.extractfile(m).read()) for m in saved if m.isfile()}
    assert actual == expected, archive
frozen = read("validation/sources.json")
assert frozen["unchanged"]
assert "test result: ok. 42 passed; 0 failed" in (record / "validation/demo-tests.log").read_text()
for name, expected in frozen["source_hashes"].items():
    assert digest((root / name).read_bytes()) == expected, name
reuse = read("reused-source-audit.json")
for name, expected in reuse["unchanged_non_demo_sources"].items():
    assert digest((root / name).read_bytes()) == expected, name
cost = read("cost/manifest.json")
assert cost["sources_unchanged"] and cost["source_hashes"] == frozen["source_hashes"]
assert digest((record / "cost_probe.rs").read_bytes()) == cost["probe_source"]["sha256"]
preview_count = 0
for version, data in cost["versions"].items():
    for trial, observation in enumerate(data["observations"], 1):
        raw = read(f"cost/{version}/trial-{trial}.json")
        assert raw == [{k:v for k,v in row.items() if k != "preview"} for row in observation["rows"]]
        assert len(raw) == 12
        for row in observation["rows"]:
            if row["status"] == "decided":
                preview = (record / f"cost/{version}/previews-{trial}/{row['name']}.json").read_bytes()
                assert len(preview) == row["preview"]["bytes"]
                assert digest(preview) == row["preview"]["sha256"]
                preview_count += 1
            if version == "candidate":
                assert row["status"] == "decided"
                assert row["retains_exact_region"] and row["segmented_retains_exact_region"]
for delta in cost["deltas"]:
    if delta["before"]["status"] == "decided":
        assert delta["before"]["preview"] == delta["after"]["preview"]
result = dict(status="passed", artifacts=len(manifest["artifacts"]),
              frozen_sources=len(frozen["source_hashes"]), raw_previews=preview_count,
              unchanged_non_demo_sources=len(reuse["unchanged_non_demo_sources"]),
              source_archives=2, recovered_operation_rows=7, preserved_preview_rows=5)
(record / "artifact-audit.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result))
