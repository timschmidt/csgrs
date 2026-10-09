"""Rebuild baseline report witnesses on a clean base checkout.

Usage: python3 reproduce_baseline_reports.py BASE_CHECKOUT OUTPUT
This is a fresh reconstruction. The originally retained baseline binaries used
the verified prior campaign's library, as documented in baseline-reports.
"""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys

record = Path(__file__).resolve().parent
checkout, out = [Path(p).resolve() for p in sys.argv[1:3]]
base = json.loads((record / "baseline.json").read_text())["production_commit"]
assert subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=checkout, text=True).strip() == base
assert not subprocess.check_output(["git", "status", "--porcelain"], cwd=checkout, text=True).strip()
out.mkdir(parents=True, exist_ok=False)
command = ["cargo", "build", "--locked", "--release", "-p", "hyperdrc", "--lib", "--message-format=json"]
with (out / "library-build.jsonl").open("w") as stdout, (out / "library-build.log").open("w") as stderr:
    subprocess.run(command, cwd=checkout, stdout=stdout, stderr=stderr, check=True)
artifacts = [json.loads(line) for line in (out / "library-build.jsonl").read_text().splitlines()]
def rlib(name):
    return next(Path(f) for a in artifacts
                if a.get("reason") == "compiler-artifact" and a["target"]["name"] == name
                for f in a["filenames"] if f.endswith(".rlib"))

for stem, names in [("full_report_replay", ["hyperdrc", "clap", "serde_json"])]:
    command = ["rustc", "--edition=2024", "-O", str(record / f"{stem}.rs"),
               "-L", f"dependency={rlib('hyperreal').parent}"]
    for name in names:
        command += ["--extern", f"{name}={rlib(name)}"]
    command += ["-o", str(out / stem)]
    subprocess.run(command, cwd=checkout, check=True)
before = json.loads((record / "baseline-reports/manifest.json").read_text())
inputs = Path(before["command"][-2])
for label, archive in [("cparti-fpga", "CPArti FPGA dev board.zip"), ("hvp109a", "HVP109A.zip")]:
    destination = inputs / label
    if not destination.exists():
        destination.mkdir(parents=True)
        subprocess.run(["unzip", "-q", str(checkout / "crates/hyperdrc/docs" / archive), "-d", str(destination)], check=True)
info = lambda p: dict(bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest())
assert {str(p.relative_to(inputs)): info(p) for p in sorted(inputs.rglob("*")) if p.is_file()} == before["input_files"]
subprocess.run([str(out / "full_report_replay"), str(inputs), str(out / "reports")], check=True)
for label in before["reports"]:
    assert (out / f"reports/{label}.json").read_bytes() == gzip.decompress((record / f"baseline-reports/{label}.json.gz").read_bytes())
print("Reproduced full baseline report bytes.")
