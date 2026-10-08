#!/usr/bin/env python3
"""Exercise migration failure/recovery boundaries using disposable fixtures."""
import contextlib
import importlib.util
import io
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('migration', Path(__file__).with_name('consolidate-audits.py'))
migration = importlib.util.module_from_spec(spec)
spec.loader.exec_module(migration)


class MigrationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='comparison-migration-test-')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.archive = self.root / 'archive'
        self.mock = patch.object(migration, 'ARCHIVE', self.archive)
        self.mock.start()
        self.addCleanup(self.mock.stop)
        self.records = []

    def source(self, name, data=b'original\n', mode=0o644):
        old = self.root / 'old' / name
        old.parent.mkdir(parents=True, exist_ok=True)
        old.write_bytes(data)
        old.chmod(mode)
        self.records.append({'source': str(old), 'destination': 'tmp/' + name,
                             'sha256': migration.digest(old), 'bytes': len(data),
                             'mode': mode, 'kind': 'code'})
        return old, self.archive / 'tmp' / name

    def manifest(self):
        return {'files': self.records, 'existing_links': [], 'excluded': []}

    def apply(self):
        with contextlib.redirect_stdout(io.StringIO()):
            migration.apply(self.manifest())

    def test_preserves_binary_bytes_and_executable_mode_and_resumes(self):
        old, new = self.source('path with spaces/probe', b'\0\xffpayload\n', 0o755)
        self.apply()
        self.assertTrue(old.is_symlink())
        self.assertFalse(os.path.isabs(os.readlink(old)))
        self.assertEqual(new.read_bytes(), b'\0\xffpayload\n')
        self.assertEqual(new.stat().st_mode & 0o777, 0o755)
        self.apply()

    def test_preflight_detects_later_source_change_before_any_move(self):
        first, _ = self.source('first')
        second, _ = self.source('second')
        second.write_bytes(b'changed')
        with self.assertRaisesRegex(RuntimeError, 'source changed'):
            self.apply()
        self.assertFalse(first.is_symlink())
        self.assertFalse(self.archive.exists())

    def test_destination_conflict_preserves_both_files(self):
        old, new = self.source('probe')
        new.parent.mkdir(parents=True)
        new.write_bytes(b'another experiment')
        with self.assertRaisesRegex(RuntimeError, 'destination conflict'):
            self.apply()
        self.assertEqual(old.read_bytes(), b'original\n')
        self.assertEqual(new.read_bytes(), b'another experiment')

    def test_resumes_after_copy_before_link(self):
        old, new = self.source('probe')
        new.parent.mkdir(parents=True)
        new.write_bytes(old.read_bytes())
        self.apply()
        self.assertTrue(old.is_symlink())

    def test_resumes_after_temporary_link_before_replace(self):
        old, new = self.source('probe')
        new.parent.mkdir(parents=True)
        new.write_bytes(old.read_bytes())
        alias = old.with_name(old.name + '.audit-migration-link')
        alias.symlink_to(os.path.relpath(new, old.parent))
        self.apply()
        self.assertTrue(old.is_symlink())
        self.assertFalse(alias.is_symlink())

    def test_rejects_destination_escape(self):
        old, _ = self.source('probe')
        self.records[0]['destination'] = '../escape'
        with self.assertRaisesRegex(RuntimeError, 'destination outside archive'):
            self.apply()
        self.assertFalse(old.is_symlink())

    def test_verifier_rejects_modified_archive_and_wrong_alias(self):
        old, new = self.source('probe')
        self.apply()
        new.write_bytes(b'changed')
        with self.assertRaisesRegex(RuntimeError, 'archive content mismatch'):
            migration.verify(self.manifest())
        new.write_bytes(b'original\n')
        old.unlink()
        old.symlink_to('missing')
        with self.assertRaisesRegex(RuntimeError, 'compatibility link mismatch'):
            migration.verify(self.manifest())

    def test_git_mode_verification_accepts_only_permission_bits_git_does_not_retain(self):
        _, new = self.source('probe', mode=0o600)
        self.apply()
        new.chmod(0o644)
        with contextlib.redirect_stdout(io.StringIO()):
            migration.verify(self.manifest(), require_links=False, git_modes=True)
        with self.assertRaisesRegex(RuntimeError, 'archive mode mismatch'):
            migration.verify(self.manifest(), require_links=False)
        new.chmod(0o755)
        with self.assertRaisesRegex(RuntimeError, 'archive mode mismatch'):
            migration.verify(self.manifest(), require_links=False, git_modes=True)


if __name__ == '__main__':
    unittest.main()
