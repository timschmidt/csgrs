"""Combine the unchanged cache proposal with the independently qualified test repair."""
import ast
import gzip
import hashlib
import json
import shutil
from pathlib import Path

W = Path(__file__).resolve().parent
S = W / "candidate-source-v2"
O = W / "preparation-v2"


def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def read(p):
    return json.loads(Path(p).read_text())


qualified = read(W / "storage-validation-v1/result.json")
assert qualified["complete"] and qualified["command_failures"] == [] and qualified["cargo_target_closed"]
repair = read(W / "storage-preparation-v1/result.json")["implementation_sha256"]
prior = read(W / "preparation-v1/result.json")
with gzip.open(W / "preparation-v1/source-hashes.json.gz", "rt") as f:
    inputs = json.load(f)
assert all(sha(W / "candidate-source-v1" / p) == h for p, h in inputs.items())
assert all(sha(W / "storage-source-v1" / p) == h for p, h in repair.items())
S.mkdir(exist_ok=False)
O.mkdir(exist_ok=False)
for p in inputs:
    dest = S / p
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(W / "candidate-source-v1" / p, dest)
for p in repair:
    original = (S / p).read_text()
    replacement = (W / "storage-source-v1" / p).read_text()
    assert original.split("#[cfg(test)]", 1)[0] == replacement.split("#[cfg(test)]", 1)[0]
    shutil.copy2(W / "storage-source-v1" / p, S / p)
current = {p: sha(S / p) for p in inputs}
assert {p for p, h in current.items() if h != inputs[p]} == repair.keys()
with gzip.open(O / "source-hashes.json.gz", "wt") as f:
    json.dump(current, f, sort_keys=True)
changes = prior["changes_from_e38_candidate"] | {
    p: {"previous": inputs[p], "current": h} for p, h in repair.items()
}
(O / "result.json").write_text(json.dumps({
    "complete": True, "base_commit": prior["base_commit"], "source": str(S),
    "source_inputs": len(current), "source_hashes_sha256": sha(O / "source-hashes.json.gz"),
    "proposal_sha256": prior["proposal_sha256"], "test_storage_repair_sha256": repair,
    "changes_from_e38_candidate": changes,
    "production_implementation_identical_to_e38_candidate": True,
    "production_prefixes_unchanged": True, "main_sources_modified": False,
    "cache_admitted": False,
    "input_sha256": {str(p): sha(p) for p in [
        Path(__file__), W / "storage-contract-v1.md", W / "storage-validation-v1/result.json",
        W / "storage-preparation-v1/result.json", W / "preparation-v1/result.json",
        W / "preparation-v1/source-hashes.json.gz",
    ]},
}, indent=2) + "\n")
template = (W / "build_native_v1.py").read_text()
for old, new in [
    ('"candidate-source-v1"', '"candidate-source-v2"'),
    ('"native-build-v1"', '"native-build-v2"'),
    ('"preparation-v1/', '"preparation-v2/'),
    ("The three independently admitted E39 test-storage edits.",
     "The three independently admitted E39 edits plus the independently qualified two-file E40 test-storage repair."),
]:
    assert old in template
    template = template.replace(old, new)
dest = W / "build_native_v2.py"
assert not dest.exists()
ast.parse(template)
dest.write_text(template)
print(json.dumps({"source_inputs": len(current), "only_added_changes": list(repair), "native_builder": str(dest)}))
