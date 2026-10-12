"""Exercise whole-catalog source identity with real temporary validation commands."""
import contextlib
import gzip
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.dont_write_bytecode = True
import validate_candidate_v3 as snapshots  # noqa: E402


class CandidateSnapshotTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.source = self.root / 'source'
        self.vendor = self.source / 'crates/hyperreal/vendor/num-bigint'
        self.vendor.mkdir(parents=True)
        self.manifest = self.source / 'Cargo.toml'
        self.manifest.write_text('[workspace]\n')
        self.original = {'Cargo.toml': snapshots.digest(self.manifest)}

    def capture(self):
        return snapshots.source_hashes(self.source, self.original)

    def test_original_workspace_inputs_are_kept(self):
        self.assertEqual(self.capture(), self.original)

    def test_vendor_inputs_do_not_depend_on_extensions(self):
        for name in ['LICENSE-MIT', 'Cargo.toml.orig', '.cargo_vcs_info.json', 'table.bin', 'data.inc']:
            (self.vendor / name).write_bytes(b'input\x00bytes')
        captured = self.capture()
        self.assertEqual(len(captured), 6)
        for path in self.vendor.iterdir():
            self.assertEqual(captured[str(path.relative_to(self.source))], snapshots.digest(path))

    def test_non_vendor_scalar_inputs_are_also_included(self):
        extra = self.source / 'crates/hyperreal/verification-table'
        extra.write_bytes(b'new scalar input')
        self.assertIn(str(extra.relative_to(self.source)), self.capture())

    def test_added_file_changes_catalog(self):
        before = self.capture()
        (self.vendor / 'LICENSE').write_text('new input\n')
        self.assertNotEqual(before, self.capture())

    def test_changed_file_changes_catalog(self):
        path = self.vendor / 'LICENSE'
        path.write_text('old input\n')
        before = self.capture()
        path.write_text('new input\n')
        self.assertNotEqual(before, self.capture())

    def test_removed_introduced_file_changes_catalog(self):
        path = self.vendor / 'LICENSE'
        path.write_text('input\n')
        before = self.capture()
        path.unlink()
        self.assertNotEqual(before, self.capture())

    def test_removed_original_file_cannot_be_hashed(self):
        self.manifest.unlink()
        with self.assertRaises(FileNotFoundError):
            self.capture()

    def test_generated_trees_are_pruned(self):
        before = self.capture()
        for name in ['target', '.git', '__pycache__']:
            directory = self.vendor / name / 'nested'
            directory.mkdir(parents=True)
            (directory / 'data').write_text('generated\n')
        self.assertEqual(before, self.capture())

    def test_symlink_file_is_rejected(self):
        (self.vendor / 'input').symlink_to(self.manifest)
        with self.assertRaisesRegex(RuntimeError, 'Symbolic-link scalar input'):
            self.capture()

    def test_symlink_directory_is_rejected(self):
        (self.vendor / 'hidden').symlink_to(self.root, target_is_directory=True)
        with self.assertRaisesRegex(RuntimeError, 'Symbolic-link scalar input'):
            self.capture()

    def run_command(self, name, code, expect_failure):
        evidence = self.root / 'evidence'
        preparation = evidence / 'candidate-preparation'
        preparation.mkdir(parents=True)
        main = self.root / 'main'
        main.mkdir()
        (main / 'guard').write_text('unchanged main\n')
        for filename, value in [('source-hashes.json.gz', self.original),
                                ('main-hashes.json.gz', {'guard': snapshots.digest(main / 'guard')})]:
            (preparation / filename).write_bytes(gzip.compress((json.dumps(value, sort_keys=True) + '\n').encode(), mtime=0))
        arguments = ['validate_candidate_v3.py', name, '--exec', sys.executable, '-c', code]
        with patch.multiple(snapshots, ROOT=main, SOURCE=self.source, EVIDENCE=evidence, TARGET=self.root / 'target'), \
                patch.object(sys, 'argv', arguments), contextlib.redirect_stdout(io.StringIO()):
            if expect_failure:
                with self.assertRaises(AssertionError):
                    snapshots.main()
            else:
                snapshots.main()
        result = json.loads((evidence / name / 'result.json').read_text())
        self.assertEqual(result['source_unchanged'], not expect_failure)
        self.assertTrue(result['main_unchanged'])
        self.assertTrue(result['source_catalog_rechecked'])
        self.assertEqual(result['exit_code'], 0)
        return result

    def test_real_command_with_unchanged_inputs_passes(self):
        self.run_command('unchanged', 'pass', False)

    def test_real_command_adding_an_extensionless_input_fails(self):
        path = self.vendor / 'LICENSE'
        self.run_command('addition', f'from pathlib import Path\nPath({str(path)!r}).write_text("added")\n', True)

    def test_real_command_removing_an_original_input_records_failure(self):
        result = self.run_command('removal', f'from pathlib import Path\nPath({str(self.manifest)!r}).unlink()\n', True)
        self.assertIsNotNone(result['source_snapshot_error'])


if __name__ == '__main__':
    unittest.main()
