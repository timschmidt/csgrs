"""Run against a separate clean checkout of the recorded production commit.

Usage: python3 reproduce_baseline.py /path/to/clean/checkout /tmp/area-baseline
The regression source and module registration are temporary and restored even
when the deliberately failing test command exits nonzero.
"""
import hashlib
import json
import pathlib
import subprocess
import sys

record = pathlib.Path(__file__).resolve().parent
manifest = json.loads((record / "baseline.json").read_text())
checkout = pathlib.Path(sys.argv[1]).resolve()
out = pathlib.Path(sys.argv[2]).resolve()
assert subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=checkout, text=True).strip() == manifest["production_commit"]
assert not subprocess.check_output(["git", "status", "--porcelain"], cwd=checkout, text=True).strip()
out.mkdir(parents=True, exist_ok=False)
lib = checkout / "crates/hyperdrc/src/lib.rs"
tests = checkout / "crates/hyperdrc/src/region/area_regressions.rs"
assert not tests.exists()
original = lib.read_bytes()
anchor = b"#[cfg(test)]\npub(crate) mod test_support {"
assert original.count(anchor) == 1
try:
    lib.write_bytes(original.replace(anchor, manifest["test_module_registration"].encode() + b"\n\n" + anchor))
    tests.write_bytes((record / "baseline-regressions.rs").read_bytes())
    for path, digest in manifest["source_hashes"].items():
        assert hashlib.sha256((checkout / path).read_bytes()).hexdigest() == digest, path
    with (out / "baseline.log").open("w") as log:
        result = subprocess.run(manifest["command"], cwd=checkout, stdout=log, stderr=subprocess.STDOUT)
    text = (out / "baseline.log").read_text()
    assert result.returncode == manifest["exit_code"]
    assert "1 passed; 9 failed;" in text
finally:
    lib.write_bytes(original)
    tests.unlink(missing_ok=True)
print("Reproduced nine failures and one passing retained-failure control; checkout restored.")
