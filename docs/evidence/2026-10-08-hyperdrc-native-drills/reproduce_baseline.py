"""Replay ten failures and two controls on a separate clean base checkout."""
import hashlib
import json
import pathlib
import subprocess
import sys

record = pathlib.Path(__file__).resolve().parent
manifest = json.loads((record / "baseline.json").read_text())
checkout, out = map(lambda p: pathlib.Path(p).resolve(), sys.argv[1:3])
assert subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=checkout, text=True).strip() == manifest["production_commit"]
assert not subprocess.check_output(["git", "status", "--porcelain"], cwd=checkout, text=True).strip()
out.mkdir(parents=True, exist_ok=False)
module = checkout / "crates/hyperdrc/src/checks/mod.rs"
tests = checkout / "crates/hyperdrc/src/checks/drill_native_regressions.rs"
assert not tests.exists()
original = module.read_bytes()
anchor = b"pub mod drill;\n"
assert original.count(anchor) == 1
try:
    module.write_bytes(original.replace(anchor, anchor + manifest["test_module_registration"].encode()))
    tests.write_bytes((record / "baseline-regressions.rs").read_bytes())
    for path, digest in manifest["source_hashes"].items():
        assert hashlib.sha256((checkout / path).read_bytes()).hexdigest() == digest, path
    with (out / "baseline.log").open("w") as log:
        result = subprocess.run(manifest["command"], cwd=checkout, stdout=log, stderr=subprocess.STDOUT)
    assert result.returncode == manifest["exit_code"]
    assert "2 passed; 10 failed;" in (out / "baseline.log").read_text()
finally:
    module.write_bytes(original)
    tests.unlink(missing_ok=True)
print("Reproduced ten failures and two controls; checkout restored.")
