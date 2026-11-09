"""Independent negative controls for the experimental process protocol."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("native_collector", HERE / "native.py")
NATIVE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NATIVE)


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=HERE, prefix="protocol-test-")
        self.folder = Path(self.temp.name)
        self.source = self.folder / "source.scad"
        self.source.write_text("cube(1);")
        self.receipt = self.folder / "receipt.json"

    def tearDown(self):
        self.temp.cleanup()

    def envelope(self, comparison):
        comparison.setdefault("dimension", 2)
        self.receipt.write_text(json.dumps({"protocol_version": 1, "policy": "STRICT", "curve_principal_certainty": "Certified",
            "source": str(self.source), "result": {"status": "native_and_reference_observed",
            "native": {"diagnostics": ["operation was ignored"]}, "comparison": comparison}}))

    def test_missing_receipt_is_failure(self):
        self.assertEqual(NATIVE.classify(self.receipt, self.source)["status"], "native_receipt_missing")

    def test_invalid_json_is_failure(self):
        self.receipt.write_text("{broken")
        self.assertEqual(NATIVE.classify(self.receipt, self.source)["status"], "native_receipt_invalid")

    def test_wrong_source_is_failure(self):
        self.envelope({"status": "represented_sets_equal", "continuation_identity_empty": True})
        self.assertEqual(NATIVE.classify(self.receipt, self.folder / "other.scad")["status"], "native_receipt_invalid")

    def test_missing_continuation_is_failure(self):
        self.envelope({"status": "represented_sets_equal"})
        self.assertEqual(NATIVE.classify(self.receipt, self.source)["status"], "native_continuation_identity_failed")

    def test_false_continuation_is_failure(self):
        self.envelope({"status": "represented_sets_equal", "continuation_identity_empty": False})
        self.assertEqual(NATIVE.classify(self.receipt, self.source)["status"], "native_continuation_identity_failed")

    def test_warning_does_not_become_source_conformance(self):
        self.envelope({"status": "represented_sets_equal", "continuation_identity_empty": True})
        result = NATIVE.classify(self.receipt, self.source)
        self.assertEqual(result["status"], "represented_sets_equal")
        self.assertTrue(result["native_diagnostics_present"])
        self.assertTrue(result["source_conformance"].startswith("unestablished"))

    def test_missing_certainty_is_not_exact_agreement(self):
        self.envelope({"status": "represented_sets_equal", "continuation_identity_empty": True})
        value = json.loads(self.receipt.read_text())
        del value["curve_principal_certainty"]
        self.receipt.write_text(json.dumps(value))
        self.assertEqual(NATIVE.classify(self.receipt, self.source)["status"], "native_receipt_invalid")

    def test_boolean_protocol_version_is_invalid(self):
        self.envelope({"status": "represented_sets_equal", "continuation_identity_empty": True})
        value = json.loads(self.receipt.read_text())
        value["protocol_version"] = True
        self.receipt.write_text(json.dumps(value))
        self.assertEqual(NATIVE.classify(self.receipt, self.source)["status"], "native_receipt_invalid")

    def test_partial_native_failure_is_not_empty_agreement(self):
        self.receipt.write_text(json.dumps({"protocol_version": 1, "policy": "STRICT",
            "source": str(self.source), "result": {"status": "native_failed", "native": {
            "parts": [{"dimension": 3}], "failures": [{"kind": "unsupported"}], "diagnostics": []},
            "comparison": None}}))
        result = NATIVE.classify(self.receipt, self.source)
        self.assertEqual(result["status"], "native_failed")
        self.assertEqual(len(result["receipt"]["result"]["native"]["parts"]), 1)
        self.assertEqual(len(result["receipt"]["result"]["native"]["failures"]), 1)

    def test_missing_mesh_certainty_is_not_inferred(self):
        self.envelope({"status": "represented_sets_equal", "dimension": 3, "continuation_identity_empty": True})
        self.assertEqual(NATIVE.classify(self.receipt, self.source)["status"], "native_receipt_invalid")

    def fake(self, body):
        worker = self.folder / "fake-worker"
        worker.write_text("#!/usr/bin/python3\n" + body)
        worker.chmod(0o700)
        return worker

    def reference_case(self, body, name):
        collector = NATIVE.COLLECTOR
        previous = collector.MANIFEST, collector.OUT, collector.BACKEND
        try:
            collector.MANIFEST = {"root": str(self.folder)}
            collector.OUT = self.folder / name
            collector.BACKEND = self.fake(body)
            return collector.case({"path": self.source.name,
                                   "sha256": collector.digest(self.source)})
        finally:
            collector.MANIFEST, collector.OUT, collector.BACKEND = previous

    def test_reference_crash_overrides_obj_empty_marker(self):
        result = self.reference_case(
            'import os, signal\nprint("Current top level object is empty.", flush=True)\n'
            'os.kill(os.getpid(), signal.SIGKILL)\n', "obj-crash")
        self.assertEqual(result["status"], "reference_process_failed")
        self.assertEqual(len(result["attempts"]), 1)
        self.assertEqual(result["attempts"][0]["exit_code"], 137)

    def test_reference_crash_overrides_svg_empty_marker(self):
        result = self.reference_case(
            'import os, signal, sys\nfrom pathlib import Path\n'
            'if Path(sys.argv[sys.argv.index("-o") + 1]).suffix == ".obj":\n'
            '    print("Current top level object is not a 3D object.", flush=True)\n'
            '    raise SystemExit(1)\n'
            'print("Current top level object is empty.", flush=True)\n'
            'os.kill(os.getpid(), signal.SIGKILL)\n', "svg-crash")
        self.assertEqual(result["status"], "reference_process_failed")
        self.assertEqual([x["exit_code"] for x in result["attempts"]], [1, 137])

    def test_reference_crash_does_not_retry_dimension_marker(self):
        result = self.reference_case(
            'import os, signal\n'
            'print("Current top level object is not a 3D object.", flush=True)\n'
            'os.kill(os.getpid(), signal.SIGKILL)\n', "dimension-crash")
        self.assertEqual(result["status"], "reference_process_failed")
        self.assertEqual(len(result["attempts"]), 1)

    def test_reference_normal_empty_markers_still_work(self):
        for stage in ["obj", "svg"]:
            for code in [0, 1]:
                with self.subTest(stage=stage, exit_code=code):
                    body = 'import sys\nfrom pathlib import Path\n'
                    if stage == "svg":
                        body += (
                            'if Path(sys.argv[sys.argv.index("-o") + 1]).suffix == ".obj":\n'
                            '    print("Current top level object is not a 3D object.")\n'
                            '    raise SystemExit(1)\n')
                    body += ('print("Current top level object is empty.")\n'
                             f'raise SystemExit({code})\n')
                    result = self.reference_case(body, f"normal-{stage}-{code}")
                    self.assertEqual(result["status"], "reference_reported_empty")
                    self.assertEqual(len(result["attempts"]), 1 if stage == "obj" else 2)

    def test_native_address_space_limit_is_enforced_before_evaluation(self):
        self.check_address_space_limit(reference=False)

    def test_reference_address_space_limit_is_enforced_before_evaluation(self):
        self.check_address_space_limit(reference=True)

    def check_address_space_limit(self, reference):
        collector = NATIVE.COLLECTOR
        previous_limit = getattr(collector, "ADDRESS_SPACE_BYTES", None)
        previous_backend = collector.BACKEND
        limit = 96 * 1024 * 1024
        body = """import json, resource, sys
from pathlib import Path
observed = {"limit": resource.getrlimit(resource.RLIMIT_AS)}
try:
    data = bytearray(128 * 1024 * 1024)
    observed["allocation_denied"] = False
except MemoryError:
    observed["allocation_denied"] = True
Path(__file__).with_name("limit.json").write_text(json.dumps(observed))
raise SystemExit(23)
"""
        worker = self.fake(body)
        try:
            collector.ADDRESS_SPACE_BYTES = limit
            if reference:
                collector.BACKEND = worker
                result, _ = collector.attempt(self.source, self.folder, "obj", 30)
            else:
                result = NATIVE.invoke(self.source, None, self.folder, worker=worker)
            observed = json.loads((self.folder / "limit.json").read_text())
            self.assertEqual(observed["limit"], [limit, limit])
            self.assertTrue(observed["allocation_denied"])
            self.assertEqual(result["exit_code"], 23)
            self.assertEqual(result["external_address_space_bytes"], limit)
            if not reference:
                self.assertEqual(result["status"], "native_process_failed")
        finally:
            collector.ADDRESS_SPACE_BYTES = previous_limit
            collector.BACKEND = previous_backend

    def test_nonzero_exit_overrides_even_valid_receipt(self):
        self.envelope({"status": "represented_sets_equal", "continuation_identity_empty": True})
        result = NATIVE.invoke(self.source, None, self.folder, worker=self.fake("raise SystemExit(17)\n"))
        self.assertEqual(result["status"], "native_process_failed")
        self.assertEqual(result["exit_code"], 17)

    def test_timeout_overrides_even_valid_receipt(self):
        self.envelope({"status": "represented_sets_equal", "continuation_identity_empty": True})
        result = NATIVE.invoke(self.source, None, self.folder, seconds=0.1,
                              worker=self.fake("import time\ntime.sleep(30)\n"))
        self.assertEqual(result["status"], "native_external_containment")
        self.assertTrue(result["external_containment_reached"])

    def test_no_receipt_after_clean_exit_is_failure(self):
        result = NATIVE.invoke(self.source, None, self.folder, worker=self.fake("pass\n"))
        self.assertEqual(result["status"], "native_receipt_missing")


if __name__ == "__main__":
    unittest.main(verbosity=2)
