#!/usr/bin/env python3
"""Check that changed dependencies cannot retain their recorded proof identity."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import verus_dependencies as dependencies  # noqa: E402


class DependencyIdentityTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        package = self.root / 'package'
        package.mkdir()
        (package / 'Cargo.toml').write_text('[package]\nname="fixture"\nversion="0.1.0"\n')
        self.source = package / 'lib.rs'
        self.source.write_text('pub enum Sign { Minus, Zero, Plus }\n')
        self.binary = self.root / 'fixture.rlib'
        self.binary.write_bytes(b'original fixture artifact')
        packages = [{'id': 'fixture', 'manifest_path': str(package / 'Cargo.toml')}]
        self.evidence = {'packages': packages, 'sources': dependencies.dependency_sources(packages),
                         'artifacts': {str(self.binary): dependencies.sha256(self.binary)}}

    def test_source_changes_and_additions_invalidate_identity(self):
        dependencies.check_dependency_identity(self.evidence)
        self.source.write_text('pub enum Sign { Minus, Zero, Plus, Extra }\n')
        with self.assertRaisesRegex(RuntimeError, 'sources changed'):
            dependencies.check_dependency_identity(self.evidence)
        self.source.write_text('pub enum Sign { Minus, Zero, Plus }\n')
        self.source.with_name('build.rs').write_text('fn main() {}\n')
        with self.assertRaisesRegex(RuntimeError, 'sources changed'):
            dependencies.check_dependency_identity(self.evidence)

    def test_artifact_changes_invalidate_identity(self):
        self.binary.write_bytes(b'replacement fixture artifact')
        with self.assertRaisesRegex(RuntimeError, 'artifacts changed'):
            dependencies.check_dependency_identity(self.evidence)

    def test_wrong_pin_is_rejected_before_cargo(self):
        folder = self.root / 'verification/dependencies'
        folder.mkdir(parents=True)
        (folder / 'Cargo.toml').write_text('[package]\nname="fixture"\nversion="0.1.0"\n')
        lock = '[[package]]\nname="num"\nversion="0.4.3"\nsource="registry+fixture"\nchecksum="original"\n'
        (self.root / 'Cargo.lock').write_text(lock)
        (folder / 'Cargo.lock').write_text(lock.replace('original', 'changed'))
        with patch.object(dependencies.subprocess, 'check_output') as cargo:
            with self.assertRaisesRegex(RuntimeError, 'does not match production lock'):
                dependencies.prepare_dependencies(self.root, {}, self.root / 'reports')
            cargo.assert_not_called()


if __name__ == '__main__':
    unittest.main()
