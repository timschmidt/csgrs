#!/usr/bin/env python3
"""Regression tests for admitting evidence, independent of geometry algorithms."""

import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

sys.dont_write_bytecode = True
DRIVER = Path(__file__).with_name("capability_matrix.py")
spec = importlib.util.spec_from_file_location("capability_matrix", DRIVER)
matrix = importlib.util.module_from_spec(spec)
spec.loader.exec_module(matrix)


class MatrixEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="hypercurve-matrix-test-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        catalog = matrix.load_catalog(matrix.DEFAULT_CATALOG)
        self.catalog = {**catalog, "fixtures": [copy.deepcopy(catalog["fixtures"][0])]}
        row = self.catalog["fixtures"][0]
        row.update(id="witness", target="fixture", test="witness", timeout_seconds=1)
        self.path = self.root / "catalog.json"
        self.path.write_text(json.dumps(self.catalog))

    def reject(self, change):
        change(self.catalog)
        self.path.write_text(json.dumps(self.catalog))
        with self.assertRaises(ValueError):
            matrix.load_catalog(self.path)

    def test_catalog_rejects_unknown_claims(self):
        for key, value in [("stage", "almost_exact"), ("outcome", "completed"), ("stage", [])]:
            with self.subTest(key=key, value=value):
                original = copy.deepcopy(self.catalog)
                self.reject(lambda c: c["fixtures"][0]["claims"][0].update({key: value}))
                self.catalog = original

    def test_catalog_rejects_ambiguous_witnesses(self):
        self.reject(lambda c: c["fixtures"].append(copy.deepcopy(c["fixtures"][0])))

    def test_catalog_rejects_missing_scope_and_bad_limits(self):
        for key, value in [("domain", ""), ("timeout_seconds", True), ("timeout_seconds", 0),
                           ("policies", ["DEFAULT"]), ("families", ["AllCurves"])]:
            with self.subTest(key=key):
                original = copy.deepcopy(self.catalog)
                self.reject(lambda c: c["fixtures"][0].update({key: value}))
                self.catalog = original

    def test_duplicate_json_keys_are_rejected(self):
        self.path.write_text('{"scope": "exact", "scope": "unsupported"}')
        with self.assertRaisesRegex(ValueError, "duplicate JSON key"):
            matrix.load_catalog(self.path)

    def test_zero_tests_or_ignored_tests_are_not_success(self):
        result = {"status": "exited", "returncode": 0}
        for summary in [
            "test result: ok. 0 passed; 0 failed; 0 ignored; 0 measured; 1 filtered out; finished in 0s",
            "test result: ok. 0 passed; 0 failed; 1 ignored; 0 measured; 0 filtered out; finished in 0s",
            "test result: ok. 2 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0s",
        ]:
            with self.subTest(summary=summary):
                self.assertFalse(matrix.witness_passed(result, summary))

    def test_blocker_witness_does_not_become_completed_geometry(self):
        witness = copy.deepcopy(self.catalog["fixtures"][0])
        witness["claims"] = [{"stage": "region_admission", "outcome": "unsupported",
                              "assertion": "Public Unsupported reason"}]
        records = [{"id": "witness", "status": "witness_passed", "witness": witness}]
        stages = matrix.stage_matrix(records, True)[0]["stages"]
        self.assertEqual(stages["region_admission"], witness["claims"])
        self.assertEqual(stages["exact_replay"], [])
        self.assertTrue(all(not claims for claims in matrix.stage_matrix(records, False)[0]["stages"].values()))
        records[0]["status"] = "test_failed"
        self.assertTrue(all(not claims for claims in matrix.stage_matrix(records, True)[0]["stages"].values()))

    def fake_tools(self, mode):
        tools = self.root / "tools"
        tools.mkdir()
        binary = self.root / "fixture"
        binary.write_text(f"#!{sys.executable}\n" + '''
import os, signal, subprocess, sys, time
from pathlib import Path
mode = os.environ['FIXTURE_MODE']
if '--list' in sys.argv:
    if '--ignored' not in sys.argv or mode == 'ignored':
        print(('different' if mode == 'missing' else 'witness') + ': test')
    sys.exit(0)
if mode == 'timeout':
    child = subprocess.Popen([sys.executable, '-c', 'import signal,time; signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(30)'])
    Path(os.environ['FIXTURE_PID']).write_text(str(child.pid))
    time.sleep(30)
if mode == 'binary_changed':
    with Path(__file__).open('a') as stream:
        stream.write('\\n# changed after build\\n')
if mode == 'catalog_changed':
    with Path(os.environ['FIXTURE_CATALOG']).open('a') as stream:
        stream.write('\\n')
passed = 0 if mode == 'zero' else 1
print(f'test result: ok. {passed} passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s')
sys.exit(2 if mode == 'failed' else 0)
''')
        binary.chmod(0o755)
        cargo = tools / "cargo"
        cargo.write_text(f"#!{sys.executable}\n" + '''
import json, os, sys
if os.environ['FIXTURE_MODE'] == 'build_failed':
    sys.exit(17)
print(json.dumps({'reason': 'compiler-artifact', 'target': {'name': 'fixture', 'kind': ['test']}, 'executable': os.environ['FIXTURE_BINARY']}))
''')
        cargo.chmod(0o755)
        rustc = tools / "rustc"
        rustc.write_text(f"#!{sys.executable}\nprint('controlled test compiler')\n")
        rustc.chmod(0o755)
        return {**os.environ, "PATH": str(tools) + os.pathsep + os.environ["PATH"],
                "FIXTURE_MODE": mode, "FIXTURE_BINARY": str(binary),
                "FIXTURE_CATALOG": str(self.path), "FIXTURE_PID": str(self.root / "child.pid"),
                "PYTHONDONTWRITEBYTECODE": "1"}

    def invoke(self, mode):
        result = subprocess.run([sys.executable, str(DRIVER), "--catalog", str(self.path),
                                 "--output", str(self.root / "report")],
                                env=self.fake_tools(mode), capture_output=True, text=True, timeout=30)
        report = json.loads((self.root / "report/report.json").read_text())
        return result, report

    def test_one_registered_success_is_admitted(self):
        result, report = self.invoke("pass")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(report["status"], "witnesses_passed")
        self.assertTrue(report["evidence_inputs_stable"])
        self.assertEqual(report["fixtures"][0]["status"], "witness_passed")
        self.assertIn("--locked", report["build"]["command"])
        self.assertIn("--release", report["build"]["command"])
        self.assertIn("--all-features", report["build"]["command"])
        self.assertTrue(report["binaries"]["fixture"]["sha256"])

    def check_failure(self, mode, expected):
        result, report = self.invoke(mode)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(report["fixtures"][0]["status"], expected)
        self.assertTrue(all(not claims for claims in report["matrix"][0]["stages"].values()))
        return report

    def test_missing_registration_is_rejected(self):
        self.check_failure("missing", "unregistered")

    def test_ignored_registration_is_rejected(self):
        self.check_failure("ignored", "ignored")

    def test_zero_test_success_is_rejected(self):
        self.check_failure("zero", "test_failed")

    def test_nonzero_exit_is_rejected_even_with_success_summary(self):
        self.check_failure("failed", "test_failed")

    def test_changed_binary_is_rejected(self):
        report = self.check_failure("binary_changed", "binary_changed")
        self.assertFalse(report["evidence_inputs_stable"])

    def test_catalog_rewrite_after_capture_cannot_admit_other_bytes(self):
        original = self.path.read_bytes()
        changed = json.loads(original)
        changed["fixtures"][0]["domain"] = "rewritten after parsing"
        calls = 0

        def snapshot():
            nonlocal calls
            calls += 1
            if calls == 1:
                self.path.write_text(json.dumps(changed))
            return {"untracked_inputs": [], "revision": "controlled"}, [], b""

        args = SimpleNamespace(catalog=self.path, fixture=None,
                               output=self.root / "report", build_timeout=30)
        with patch.dict(os.environ, self.fake_tools("pass")), patch.object(
            matrix.evidence, "source_snapshot", side_effect=snapshot,
        ):
            code = matrix.run(args)
        report = json.loads((args.output / "report.json").read_text())
        self.assertNotEqual(code, 0)
        self.assertEqual(report["status"], "evidence_inputs_changed")
        self.assertEqual(report["catalog_sha256"], hashlib.sha256(original).hexdigest())
        self.assertEqual((args.output / "catalog.json").read_bytes(), original)
        self.assertEqual(report["fixtures"][0]["witness"], json.loads(original)["fixtures"][0])
        self.assertTrue(all(not claims for claims in report["matrix"][0]["stages"].values()))

    def test_changed_catalog_invalidates_claims(self):
        result, report = self.invoke("catalog_changed")
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(report["status"], "evidence_inputs_changed")
        self.assertTrue(all(not claims for claims in report["matrix"][0]["stages"].values()))

    def test_failed_build_cannot_admit_a_fixture(self):
        result, report = self.invoke("build_failed")
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(report["status"], "driver_failed")
        self.assertEqual(report["fixtures"], [])
        self.assertEqual(report["build"]["returncode"], 17)

    def test_timeout_is_incomplete_and_cleans_descendants(self):
        report = self.check_failure("timeout", "timeout")
        self.assertEqual(report["fixtures"][0]["run"]["status"], "timeout")
        self.assert_child_stopped()

    def assert_child_stopped(self):
        pid = (self.root / "child.pid").read_text()
        state = Path('/proc') / pid / 'stat'
        if state.exists():
            self.assertIn(state.read_text().split()[2], ['Z', 'X'])

    def test_interrupted_driver_cleans_descendants(self):
        environment = self.fake_tools("timeout")
        with (self.root / "driver.log").open("w") as log:
            driver = subprocess.Popen([sys.executable, str(DRIVER), "--catalog", str(self.path),
                                       "--output", str(self.root / "report")],
                                      env=environment, stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 15
                while not (self.root / "child.pid").exists():
                    self.assertIsNone(driver.poll(), "driver ended before interruption")
                    self.assertLess(time.monotonic(), deadline)
                    time.sleep(0.01)
                driver.send_signal(signal.SIGTERM)
                self.assertNotEqual(driver.wait(timeout=15), 0)
            finally:
                if driver.poll() is None:
                    driver.kill()
                    driver.wait()
        report = json.loads((self.root / "report/report.json").read_text())
        self.assertEqual(report["status"], "interrupted")
        self.assertEqual(report["matrix"], [])
        self.assert_child_stopped()


if __name__ == "__main__":
    unittest.main()
