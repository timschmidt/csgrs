#!/usr/bin/env python3
"""Exercise corruption and path-boundary checks on disposable import fixtures."""

import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


spec = importlib.util.spec_from_file_location("verify_import", Path(__file__).with_name("verify-import.py"))
verify_import = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verify_import)


class ImportTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="comparison-import-test-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root / "integration").mkdir()
        (self.root / "audits").mkdir()
        self.payload = b"original\r\n\x00payload\n"
        (self.root / "audits/probe").write_bytes(self.payload)
        self.records = [{
            "path": "audits/probe", "mode": "100644",
            "git_oid": hashlib.sha1(f"blob {len(self.payload)}\0".encode() + self.payload).hexdigest(),
        }]
        self.save()

    def save(self, adaptations=()):
        data = "".join(json.dumps(row) + "\n" for row in self.records).encode()
        (self.root / "integration/upstream-tree.jsonl").write_bytes(data)
        (self.root / "integration/provenance.json").write_text(json.dumps({
            "revision": "fixture", "inventory_sha256": hashlib.sha256(data).hexdigest(),
            "adaptations": list(adaptations), "legacy_migration_discrepancies": [],
        }))

    def test_exact_bytes_and_executable_bit_are_required(self):
        self.assertEqual(verify_import.verify(self.root)["verified"]["files"], 1)
        file = self.root / "audits/probe"
        file.write_bytes(self.payload.replace(b"\r\n", b"\n"))
        with self.assertRaisesRegex(ValueError, "content digest changed"):
            verify_import.verify(self.root)
        file.write_bytes(self.payload)
        file.chmod(0o755)
        with self.assertRaisesRegex(ValueError, "executable bit changed"):
            verify_import.verify(self.root)

    def test_lfs_pointer_cannot_replace_verified_payload(self):
        self.records[0]["lfs"] = {"sha256": hashlib.sha256(self.payload).hexdigest(), "bytes": len(self.payload)}
        self.save()
        self.assertEqual(verify_import.verify(self.root)["verified"]["lfs_payloads"], 1)
        (self.root / "audits/probe").write_text("version https://git-lfs.github.com/spec/v1\n")
        with self.assertRaisesRegex(ValueError, "LFS payload size"):
            verify_import.verify(self.root)

    def test_symlink_is_checked_without_following_its_target(self):
        file = self.root / "audits/probe"
        file.unlink()
        target = b"../../unavailable-historical-source"
        file.symlink_to(target.decode())
        self.records[0] = {"path": "audits/probe", "mode": "120000", "git_oid": hashlib.sha1(f"blob {len(target)}\0".encode() + target).hexdigest()}
        self.save()
        self.assertEqual(verify_import.verify(self.root)["verified"]["symlinks"], 1)
        file.unlink()
        file.symlink_to("different-target")
        with self.assertRaisesRegex(ValueError, "symlink target changed"):
            verify_import.verify(self.root)

    def test_historical_changes_cannot_be_declared_as_adaptations(self):
        self.save([{"path": "audits/probe", "sha256": "irrelevant"}])
        with self.assertRaisesRegex(ValueError, "historical archive cannot be adapted"):
            verify_import.verify(self.root)

    def test_inventory_drift_and_parent_traversal_are_rejected(self):
        (self.root / "integration/upstream-tree.jsonl").write_text("changed")
        with self.assertRaisesRegex(ValueError, "inventory digest changed"):
            verify_import.verify(self.root)
        self.records[0]["path"] = "../outside"
        self.save()
        with self.assertRaisesRegex(ValueError, "unsafe or repeated"):
            verify_import.verify(self.root)


if __name__ == "__main__":
    unittest.main()
