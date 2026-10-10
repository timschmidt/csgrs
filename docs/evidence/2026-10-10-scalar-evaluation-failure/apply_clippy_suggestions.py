"""Apply a frozen Clippy report's reviewed, exact source edits after it exits."""
import gzip
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
phase = EVIDENCE / sys.argv[1]
out = EVIDENCE / sys.argv[2]
out.mkdir()
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
hashes = json.loads(gzip.decompress((phase / "source-hashes.json.gz").read_bytes()))
assert (phase / "result.json").is_file(), "validation must have exited"
assert all(sha(SOURCE / p) == h for p, h in hashes.items())
edits = set()
for line in (phase / "stdout.log").read_text().splitlines():
    item = json.loads(line)
    if item.get("reason") != "compiler-message":
        continue
    message = item["message"]
    if message.get("code", {}).get("code") != "clippy::needless_question_mark":
        continue
    for child in message["children"]:
        for span in child["spans"]:
            if span["suggestion_applicability"] == "MachineApplicable":
                assert span["suggested_replacement"] == ""
                edits.add((span["file_name"], span["byte_start"], span["byte_end"]))
records = []
for name in sorted({e[0] for e in edits}):
    path = SOURCE / name
    data = path.read_bytes()
    before = sha(path)
    ceiling = len(data)
    for _, start, end in sorted((e for e in edits if e[0] == name), reverse=True):
        assert end <= ceiling
        assert data[start:end] in [b"Ok(", b"?)"]
        data = data[:start] + data[end:]
        ceiling = start
    path.write_bytes(data)
    records.append({"path": name, "before": before, "after": sha(path)})
(out / "runner.py").write_bytes(Path(__file__).read_bytes())
(out / "result.json").write_text(json.dumps({
    "input_phase": str(phase), "edits": sorted(edits), "changed_sources": records,
}, indent=2) + "\n")
print("Applied", len(edits), "exact edits in", len(records), "files.")
