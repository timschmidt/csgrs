"""Freeze the reusable dependency builder proposal and run identity/receipt controls."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
PROPOSAL = E / "dependency-wiring-proposal-v2476"
OUT = E / "dependency-wiring-units-v2477"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    assert not OUT.exists()
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    proposal = {p.name: sha(p) for p in sorted(PROPOSAL.glob("*.py"))}
    assert set(proposal) == {"verus_dependencies.py", "verus_bigint.py", "verus_rustc_wrapper.py", "test_verified_export.py"}
    old_tests = SOURCE / "crates/hyperreal/verification/test_dependency_identity.py"
    driver = """import importlib.util
import pathlib
import sys
import unittest
sys.dont_write_bytecode = True
proposal, legacy = map(pathlib.Path, sys.argv[1:])
sys.path.insert(0, str(proposal))
import verus_dependencies
assert pathlib.Path(verus_dependencies.__file__).resolve() == (proposal / 'verus_dependencies.py').resolve()
import test_verified_export
spec = importlib.util.spec_from_file_location('legacy_identity', legacy)
old = importlib.util.module_from_spec(spec)
spec.loader.exec_module(old)
assert old.dependencies is verus_dependencies
suite = unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromModule(old), unittest.defaultTestLoader.loadTestsFromModule(test_verified_export)])
assert suite.countTestCases() == 44, suite.countTestCases()
result = unittest.TextTestRunner(verbosity=2).run(suite)
raise SystemExit(0 if result.wasSuccessful() else 1)
"""
    OUT.mkdir()
    (OUT / "driver.py").write_text(driver)
    command = [sys.executable, "-B", str(OUT / "driver.py"), str(PROPOSAL), str(old_tests)]
    protocol = {"status": "Frozen before forty-four source-identity and export-binding tests",
        "runner_sha256": sha(Path(__file__)), "driver_sha256": sha(OUT / "driver.py"),
        "retained_checkpoint_sha256": sha(checkpoint_path), "source_record_sha256": checkpoint["source_record_sha256"],
        "proposal": str(PROPOSAL), "proposal_files_sha256": proposal, "legacy_tests_sha256": sha(old_tests),
        "command": command, "expected_legacy_tests": 19, "expected_new_tests": 25,
        "scope": "Exercise the unchanged nineteen production/proof-resolution identity controls against the proposed reusable builder, plus twenty-five synthetic export-receipt controls. The synthetic records establish file, path, source, helper, tool, report and Cargo-library rejection behavior; they are not arithmetic proof evidence. The actual Cargo build, imported producer proof and candidate integration remain separate gates. SOURCE, main and retained proof/runtime coverage are unchanged."}
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    start = time.monotonic()
    run = subprocess.run(command, cwd=ROOT, env=os.environ, capture_output=True, text=True)
    (OUT / "stdout.log").write_text(run.stdout)
    (OUT / "stderr.log").write_text(run.stderr)
    assert {p.name: sha(p) for p in sorted(PROPOSAL.glob("*.py"))} == proposal
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    result = {"exit_code": run.returncode, "seconds": time.monotonic() - start,
        "gate_passed": run.returncode == 0 and "Ran 44 tests" in run.stderr and run.stderr.rstrip().endswith("OK"),
        "protocol_sha256": sha(OUT / "protocol.json"), "stdout_sha256": sha(OUT / "stdout.log"),
        "stderr_sha256": sha(OUT / "stderr.log"), "source_unchanged": True, "main_unchanged": True,
        "proposal_unchanged": True, "retained_coverage_changed": False, "runtime_admitted": False}
    (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
