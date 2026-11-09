import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("corpus_cli", HERE / "run.py")
CLI = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CLI)


class ManifestTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name).resolve()
        self.path = self.root / "manifest.json"
        (self.root / "one.scad").write_text("cube(1);")
        self.item = {"path": "one.scad", "sha256": hashlib.sha256(b"cube(1);").hexdigest()}

    def tearDown(self):
        self.temp.cleanup()

    def load(self, items, count):
        self.path.write_text(json.dumps({"count": count, "files": items}))
        return CLI.load_manifest(self.path, self.root, count)

    def test_original_source_and_complete_inventory_are_required(self):
        self.assertEqual(self.load([self.item], 1)["files"], [self.item])
        (self.root / "one.scad").write_text("cube(2);")
        with self.assertRaises(ValueError):
            self.load([self.item], 1)

    def test_omitted_case_is_rejected(self):
        (self.root / "unlisted.scad").write_text("sphere(1);")
        with self.assertRaises(ValueError):
            self.load([self.item], 1)

    def test_duplicate_case_is_rejected(self):
        with self.assertRaises(ValueError):
            self.load([self.item, self.item], 2)

    def test_parent_escape_is_rejected(self):
        with self.assertRaises(ValueError):
            self.load([dict(self.item, path="../one.scad")], 1)

    def test_absolute_path_is_rejected(self):
        with self.assertRaises(ValueError):
            self.load([dict(self.item, path=str(self.root / "one.scad"))], 1)

    def test_symlink_escape_is_rejected(self):
        with tempfile.TemporaryDirectory() as outside:
            source = Path(outside) / "outside.scad"
            source.write_text("cube(1);")
            (self.root / "one.scad").unlink()
            (self.root / "one.scad").symlink_to(source)
            with self.assertRaises(ValueError):
                self.load([self.item], 1)

    def test_normal_cleanup_does_not_cancel_following_engine(self):
        collector = CLI.native.COLLECTOR
        collector.STOP.clear()
        collector.cleanup()
        self.assertFalse(collector.STOP.is_set())


if __name__ == "__main__":
    unittest.main(verbosity=2)
