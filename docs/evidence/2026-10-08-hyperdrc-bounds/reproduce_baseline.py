"""Replay the 14 failures and three controls in a separate clean base checkout.

Usage: python3 reproduce_baseline.py /path/to/base-checkout /tmp/bounds-baseline
Both temporary registrations and test files are restored on exit.
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
checks = checkout / "crates/hyperdrc/src/checks/mod.rs"
tests = checkout / "crates/hyperdrc/src/region/bounds_regressions.rs"
spatial_tests = checkout / "crates/hyperdrc/src/checks/bounds_regressions.rs"
assert not tests.exists() and not spatial_tests.exists()
original = lib.read_bytes()
original_checks = checks.read_bytes()
anchor = b"#[cfg(test)]\npub(crate) mod test_support {"
assert original.count(anchor) == 1
try:
    lib.write_bytes(original.replace(anchor, manifest["test_module_registration"].encode() + b"\n\n" + anchor))
    checks.write_bytes(original_checks + manifest["checks_test_registration"].encode())
    tests.write_bytes((record / "baseline-regressions.rs").read_bytes())
    spatial_tests.write_bytes((record / "baseline-spatial-regressions.rs").read_bytes())
    for path, digest in manifest["source_hashes"].items():
        assert hashlib.sha256((checkout / path).read_bytes()).hexdigest() == digest, path
    with (out / "baseline.log").open("w") as log:
        result = subprocess.run(manifest["command"], cwd=checkout, stdout=log, stderr=subprocess.STDOUT)
    text = (out / "baseline.log").read_text()
    assert result.returncode == manifest["exit_code"]
    assert "3 passed; 14 failed;" in text
finally:
    lib.write_bytes(original)
    checks.write_bytes(original_checks)
    tests.unlink(missing_ok=True)
    spatial_tests.unlink(missing_ok=True)
print("Reproduced fourteen failures and three passing controls; checkout restored.")
