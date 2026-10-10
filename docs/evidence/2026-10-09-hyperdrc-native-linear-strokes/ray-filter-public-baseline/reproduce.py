"""Replay exact affine lens centers against the admitted production library."""
import hashlib
import json
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / "ray-filter-public-baseline"
OUT.mkdir()
WORK = ROOT / "target/ray-filter-overflow"
record = json.loads((EVIDENCE / "bbox-initial-pair-production-cost/manifest.json").read_text())
library = record["libraries"]["candidate"]
lib = Path(library["path"])
assert hashlib.sha256(lib.read_bytes()).hexdigest() == library["sha256"]
probe = EVIDENCE / "ray_filter_public_probe.rs"
(OUT / "probe.rs").write_bytes(probe.read_bytes())
commands = []
for name, command in [
    ("build", ["rustc", "--edition=2024", "-O", "-D", "warnings", str(OUT / "probe.rs"), "-L", "dependency=" + str(lib.parent), "-L", "dependency=" + str(lib.parent / "deps"), "--extern", "hypercurve=" + str(lib), "-o", str(WORK / "public-baseline")]),
    ("run", ["timeout", "180", str(WORK / "public-baseline")]),
]:
    print("start", name, flush=True)
    start = time.monotonic()
    with (OUT / (name + ".log")).open("w") as log, (OUT / (name + ".jsonl")).open("w") as rows:
        result = subprocess.run(command, cwd=ROOT, stdout=rows, stderr=log, check=False)
    commands.append({"name": name, "command": command, "exit_code": result.returncode, "wall_seconds": time.monotonic() - start})
    (OUT / "commands.json").write_text(json.dumps(commands, indent=2) + "\n")
    assert hashlib.sha256(lib.read_bytes()).hexdigest() == library["sha256"]
    if name == "build": assert result.returncode == 0
rows = [json.loads(line) for line in (OUT / "run.jsonl").read_text().splitlines()]
(OUT / "manifest.json").write_text(json.dumps({"library": library, "source_record": record["candidate_source_record"], "probe_sha256": hashlib.sha256(probe.read_bytes()).hexdigest(), "commands": commands, "rows": rows, "main_runtime_unchanged": True}, indent=2) + "\n")
(OUT / "reproduce.py").write_bytes(Path(__file__).read_bytes())
print("Public baseline exit:", result.returncode, "completed cases:", len(rows), "failures:", sum(not row["passed"] for row in rows), flush=True)
