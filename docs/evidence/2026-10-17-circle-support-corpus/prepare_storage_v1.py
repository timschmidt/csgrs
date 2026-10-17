"""Prepare only the five reproduced fixture-path corrections."""
import gzip
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
S = W / "storage-source-v1"
O = W / "storage-preparation-v1"


def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def read(p):
    return json.loads(Path(p).read_text())


assert read(W / "storage-reproduction-v1/result.json")["reproduced_failures"] == 5
with gzip.open(W / "preparation-v1/main-input-hashes.json.gz", "rt") as f:
    main = json.load(f)
assert all(sha(ROOT / p) == h for p, h in main.items())
S.mkdir(exist_ok=False)
O.mkdir(exist_ok=False)
for p in main:
    dest = S / p
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ROOT / p, dest)
replacements = {
    "crates/hyperdrc/src/app.rs": [
        ('PathBuf::from(format!("/tmp/hyperdrc-load-all-layers-{process_id}"))',
         'std::env::temp_dir().join(format!("hyperdrc-load-all-layers-{process_id}"))'),
        ('PathBuf::from(format!(\n            "/tmp/hyperdrc-archive-layers-{}",',
         'std::env::temp_dir().join(format!(\n            "hyperdrc-archive-layers-{}",'),
        ('PathBuf::from(format!("/tmp/hyperdrc-x2-metadata-{process_id}.gbr"))',
         'std::env::temp_dir().join(format!("hyperdrc-x2-metadata-{process_id}.gbr"))'),
        ('PathBuf::from(format!("/tmp/hyperdrc-load-all-layers-parse-{process_id}"))',
         'std::env::temp_dir().join(format!("hyperdrc-load-all-layers-parse-{process_id}"))'),
    ],
    "crates/hyperdrc/src/arrow_report.rs": [
        ('PathBuf::from(format!("/tmp/hyperdrc-report-{}.arrow", std::process::id()))',
         'std::env::temp_dir().join(format!("hyperdrc-report-{}.arrow", std::process::id()))'),
        ("    use std::path::PathBuf;\n", ""),
    ],
}
for p, substitutions in replacements.items():
    original = (S / p).read_text()
    updated = original
    for old, new in substitutions:
        assert updated.count(old) == 1, (p, old)
        updated = updated.replace(old, new)
    assert original.split("#[cfg(test)]", 1)[0] == updated.split("#[cfg(test)]", 1)[0]
    (S / p).write_text(updated)
with (O / "format.stdout").open("wb") as out, (O / "format.stderr").open("wb") as err:
    result = subprocess.run(["cargo", "fmt", "--all"], cwd=S, stdout=out, stderr=err)
assert result.returncode == 0
current = {p: sha(S / p) for p in main}
changed = {p: h for p, h in current.items() if main[p] != h}
assert changed.keys() == replacements.keys()
for p in changed:
    assert (ROOT / p).read_text().split("#[cfg(test)]", 1)[0] == (S / p).read_text().split("#[cfg(test)]", 1)[0]
with gzip.open(O / "source-hashes.json.gz", "wt") as f:
    json.dump(current, f, sort_keys=True)
patches = []
for p in changed:
    diff = subprocess.run(["git", "diff", "--no-index", "--", str(ROOT / p), str(S / p)], stdout=subprocess.PIPE)
    assert diff.returncode == 1
    patches.append(diff.stdout)
(O / "test-storage.patch").write_bytes(b"\n".join(patches))
(O / "result.json").write_text(json.dumps({
    "complete": True, "source": str(S), "source_inputs": len(current),
    "source_hashes_sha256": sha(O / "source-hashes.json.gz"),
    "implementation_sha256": changed, "production_prefixes_unchanged": True,
    "exact_replacements": replacements, "format_exit_code": result.returncode,
    "cache_candidate_included": False, "main_sources_modified": False,
    "input_sha256": {str(p): sha(p) for p in [
        Path(__file__), W / "storage-contract-v1.md", W / "storage-reproduction-v1/result.json",
        W / "preparation-v1/main-input-hashes.json.gz",
    ]},
}, indent=2) + "\n")
assert all(sha(ROOT / p) == h for p, h in main.items())
print(json.dumps({"changed_test_files": list(changed), "production_prefixes_unchanged": True}))
