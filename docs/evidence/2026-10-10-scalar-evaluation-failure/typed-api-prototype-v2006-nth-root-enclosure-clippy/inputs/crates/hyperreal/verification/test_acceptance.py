#!/usr/bin/env python3
"""Exercise the completion gate's stale, missing and invalid evidence boundaries."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

# Keep dependency identity checks on the existing CI acceptance-test entry point.
from test_dependency_identity import DependencyIdentityTests  # noqa: F401

MODULE = Path(__file__).resolve().parents[1] / "scripts/check_verification_acceptance.py"
SPEC = importlib.util.spec_from_file_location("acceptance", MODULE)
acceptance = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(acceptance)


class AcceptanceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root / "src").mkdir()
        (self.root / "src/lib.rs").write_text("pub fn fixture() -> u32 { 7 }\n")
        (self.root / "verification").mkdir()
        (self.root / "measurements.json").write_text('{"fixture": true}\n')
        self.report = {
            "schema": 1, "baseline": acceptance.BASELINE,
            "priority_order": acceptance.PRIORITIES, "status": "accepted", "unresolved": [],
            "candidate_sources": acceptance.qualification_inputs(self.root),
            "evidence": {"measurements.json": acceptance.sha256(self.root / "measurements.json")},
            "assessments": {priority: {"status": "qualified", "rationale": "Schema test fixture only.",
                                      "evidence": ["measurements.json"]} for priority in acceptance.PRIORITIES},
        }

    def check(self, report=None):
        (self.root / "verification/acceptance.json").write_text(json.dumps(self.report if report is None else report))
        acceptance.check_acceptance(self.root)

    def test_complete_fixture_and_self_reference_exclusion(self):
        self.check()
        self.assertEqual(acceptance.qualification_inputs(self.root), self.report["candidate_sources"])

    def test_status_reference_order_and_unresolved_items(self):
        for key, value in [("schema", 2), ("baseline", "0" * 40), ("priority_order", list(reversed(acceptance.PRIORITIES))),
                           ("status", "pending"), ("unresolved", ["timing regression"]), ("unresolved", None)]:
            with self.subTest(key=key, value=value), self.assertRaises(RuntimeError):
                report = copy.deepcopy(self.report)
                report[key] = value
                self.check(report)

    def test_changed_added_and_removed_sources(self):
        path = self.root / "src/lib.rs"
        path.write_text("pub fn fixture() -> u32 { 8 }\n")
        with self.assertRaisesRegex(RuntimeError, "stale"):
            self.check()
        path.write_text("pub fn fixture() -> u32 { 7 }\n")
        extra = self.root / "src/extra.rs"
        extra.write_text("pub fn extra() {}\n")
        with self.assertRaisesRegex(RuntimeError, "stale"):
            self.check()
        extra.unlink()
        path.unlink()
        with self.assertRaisesRegex(RuntimeError, "stale"):
            self.check()

    def test_workload_and_build_changes_invalidate_evidence(self):
        for name in ["benches/input.rs", "tests/oracle.rs", "fuzz/fuzz_targets/real.rs", "Cargo.toml", "scripts/check.sh", ".github/workflows/ci.yml", ".cargo/config.toml", "verification/dependencies/Cargo.lock", "verification/dependencies/src/lib.rs"]:
            with self.subTest(name=name):
                path = self.root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("changed qualification input\n")
                with self.assertRaisesRegex(RuntimeError, "stale"):
                    self.check()
                path.unlink()

    def test_source_edit_during_artifact_check_is_rejected(self):
        original = acceptance.sha256

        def change_source_after_reading_evidence(path):
            result = original(path)
            if path.name == "measurements.json":
                (self.root / "src/lib.rs").write_text("pub fn fixture() -> u32 { 9 }\n")
            return result

        with patch.object(acceptance, "sha256", side_effect=change_source_after_reading_evidence):
            with self.assertRaisesRegex(RuntimeError, "changed while checking"):
                self.check()

    def test_changed_and_missing_artifacts(self):
        artifact = self.root / "measurements.json"
        artifact.write_text("modified\n")
        with self.assertRaisesRegex(RuntimeError, "artifact changed"):
            self.check()
        artifact.unlink()
        with self.assertRaisesRegex(RuntimeError, "evidence path"):
            self.check()

    def test_external_artifacts_and_incomplete_assessments(self):
        for evidence in [{}, {str(self.root / "measurements.json"): "bad"}, {"../outside": "bad"}]:
            with self.subTest(evidence=evidence), self.assertRaises(RuntimeError):
                report = copy.deepcopy(self.report)
                report["evidence"] = evidence
                self.check(report)
        for value in [{}, {"status": "pending"}, {"status": "qualified", "rationale": " "},
                      {"status": "qualified", "rationale": "reason", "evidence": ["absent.json"]}]:
            with self.subTest(assessment=value), self.assertRaises(RuntimeError):
                report = copy.deepcopy(self.report)
                report["assessments"]["performance"] = value
                self.check(report)
        report = copy.deepcopy(self.report)
        del report["assessments"]["code_size"]
        with self.assertRaisesRegex(RuntimeError, "all six"):
            self.check(report)


if __name__ == "__main__":
    unittest.main()
