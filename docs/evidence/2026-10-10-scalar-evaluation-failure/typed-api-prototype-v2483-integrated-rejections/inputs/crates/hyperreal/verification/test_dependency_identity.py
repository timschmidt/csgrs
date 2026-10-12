#!/usr/bin/env python3
"""Reject stale local-package, graph, source, configuration and binary identities."""
import copy
import json
import os
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
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.manifest = self.root / 'Cargo.toml'
        self.manifest.write_text('[package]\nname="production"\nversion="0.1.0"\n')
        self.proof_manifest = self.root / 'verification/dependencies/Cargo.toml'
        self.proof_manifest.parent.mkdir(parents=True)
        self.proof_manifest.write_text('[package]\nname="proof"\nversion="0.1.0"\n')
        self.package = self.root / 'package'
        self.package.mkdir()
        (self.package / 'Cargo.toml').write_text('[package]\nname="fixture"\nversion="0.1.0"\n')
        self.source = self.package / 'lib.rs'
        self.source.write_text('pub enum Sign { Minus, Zero, Plus }\n')
        self.binary = self.root / 'fixture.rlib'
        self.binary.write_bytes(b'original fixture artifact')
        self.lock = {'package': [{'name': 'fixture', 'version': '0.1.0'}]}
        lock_text = '[[package]]\nname="fixture"\nversion="0.1.0"\n'
        self.production_lock = self.root / 'Cargo.lock'
        self.production_lock.write_text(lock_text)
        self.proof_manifest.with_name('Cargo.lock').write_text(lock_text)
        self.dependency = {'id': 'dependency', 'name': 'fixture', 'version': '0.1.0', 'source': None,
                           'manifest_path': str(self.package / 'Cargo.toml')}
        self.proof = self.metadata('proof', self.proof_manifest)
        self.production = self.metadata('production', self.manifest)
        self.overrides = {'CARGO_HOME': str(self.root / 'cargo-home')}
        packages, bindings = self.bind()
        self.evidence = {
            'packages': packages, 'bindings': bindings,
            'sources': dependencies.dependency_sources(packages),
            'artifacts': {str(self.binary): dependencies.sha256(self.binary)},
            'metadata_commands': [['cargo', 'proof'], ['cargo', 'production']],
            'resolution': {
                'root': str(self.root), 'production_lock': str(self.production_lock),
                'proof_manifest': str(self.proof_manifest), 'production_manifest': str(self.manifest),
                'environment_overrides': self.overrides,
                'configuration_sources': dependencies.configuration_sources(self.root, self.production_lock, os.environ | self.overrides),
            },
        }

    def metadata(self, name, manifest):
        package = {'id': name, 'name': name, 'version': '0.1.0', 'source': None, 'manifest_path': str(manifest)}
        return {'packages': [package, copy.deepcopy(self.dependency)],
                'resolve': {'nodes': [{'id': name, 'deps': [{'pkg': 'dependency'}]},
                                      {'id': 'dependency', 'deps': []}]}}

    def bind(self, proof=None, production=None):
        return dependencies.bind_resolution(proof or self.proof, production or self.production,
            self.proof_manifest, self.manifest, self.lock)

    def check(self, proof=None, production=None):
        results = [json.dumps(proof or self.proof), json.dumps(production or self.production)]
        with patch.object(dependencies.subprocess, 'check_output', side_effect=results) as cargo:
            dependencies.check_dependency_identity(self.evidence)
            self.assertEqual(cargo.call_count, 2)

    def test_local_dependency_is_bound_and_included(self):
        packages, bindings = self.bind()
        self.assertEqual([p['id'] for p in packages], ['dependency'])
        self.assertEqual(bindings[0]['manifest_path'], str(self.package / 'Cargo.toml'))
        self.assertIsNone(bindings[0]['source'])
        self.check()

    def test_same_name_and_version_at_another_path_is_rejected(self):
        production = copy.deepcopy(self.production)
        production['packages'][1]['manifest_path'] = str(self.root / 'different/Cargo.toml')
        with self.assertRaisesRegex(RuntimeError, 'does not match production resolution'):
            self.bind(production=production)

    def test_unreachable_workspace_package_is_not_a_production_dependency(self):
        production = copy.deepcopy(self.production)
        production['resolve']['nodes'][0]['deps'] = []
        with self.assertRaisesRegex(RuntimeError, 'does not match production resolution'):
            self.bind(production=production)

    def test_transitive_path_dependency_is_reachable(self):
        for metadata in [self.proof, self.production]:
            bridge = copy.deepcopy(metadata['packages'][1])
            bridge.update(id='bridge', name='bridge', manifest_path=str(self.root / 'bridge/Cargo.toml'))
            metadata['packages'].append(bridge)
            metadata['resolve']['nodes'][0]['deps'] = [{'pkg': 'bridge'}]
            metadata['resolve']['nodes'].append({'id': 'bridge', 'deps': [{'pkg': 'dependency'}]})
        self.lock['package'].append({'name': 'bridge', 'version': '0.1.0'})
        packages, _ = self.bind()
        self.assertEqual({p['id'] for p in packages}, {'bridge', 'dependency'})

    def test_registry_source_identity_is_preserved(self):
        self.proof['packages'][1]['source'] = 'registry+fixture'
        self.production['packages'][1]['source'] = 'registry+fixture'
        self.lock['package'][0]['source'] = 'registry+fixture'
        self.bind()
        self.production['packages'][1]['source'] = 'registry+replacement'
        with self.assertRaisesRegex(RuntimeError, 'does not match production resolution'):
            self.bind()

    def test_missing_resolved_node_is_rejected(self):
        self.proof['resolve']['nodes'].pop()
        with self.assertRaisesRegex(RuntimeError, 'Incomplete Cargo dependency graph'):
            self.bind()

    def test_unlocked_local_dependency_is_rejected(self):
        self.lock['package'].clear()
        with self.assertRaisesRegex(RuntimeError, 'Unexpected resolved proof dependency'):
            self.bind()

    def test_source_changes_and_additions_invalidate_identity(self):
        self.check()
        self.source.write_text('pub enum Sign { Minus, Zero, Plus, Extra }\n')
        with self.assertRaisesRegex(RuntimeError, 'sources changed'):
            self.check()
        self.source.write_text('pub enum Sign { Minus, Zero, Plus }\n')
        (self.package / 'build.rs').write_text('fn main() {}\n')
        with self.assertRaisesRegex(RuntimeError, 'sources changed'):
            self.check()

    def test_extensionless_and_non_rust_inputs_are_hashed(self):
        (self.package / 'LICENSE').write_text('license input\n')
        (self.package / 'generated.inc').write_text('generated data\n')
        captured = dependencies.dependency_sources([self.dependency])['dependency']
        self.assertIn('LICENSE', captured)
        self.assertIn('generated.inc', captured)
        with self.assertRaisesRegex(RuntimeError, 'sources changed'):
            self.check()

    def test_source_removal_invalidates_identity(self):
        self.source.unlink()
        with self.assertRaisesRegex(RuntimeError, 'sources changed'):
            self.check()

    def test_generated_target_outputs_are_excluded(self):
        target = self.package / 'target'
        target.mkdir()
        (target / 'generated.bin').write_bytes(b'build output')
        self.check()

    def test_symbolic_link_input_is_rejected(self):
        (self.package / 'hidden').symlink_to(self.root, target_is_directory=True)
        with self.assertRaisesRegex(RuntimeError, 'Symbolic-link dependency input'):
            self.check()

    def test_artifact_changes_and_removal_invalidate_identity(self):
        self.binary.write_bytes(b'replacement fixture artifact')
        with self.assertRaisesRegex(RuntimeError, 'artifacts changed'):
            self.check()
        self.binary.unlink()
        with self.assertRaisesRegex(RuntimeError, 'artifacts changed'):
            self.check()

    def test_new_configuration_file_invalidates_identity_before_cargo(self):
        directory = self.root / '.cargo'
        directory.mkdir()
        (directory / 'config.toml').write_text('[build]\ntarget = "different-target"\n')
        with patch.object(dependencies.subprocess, 'check_output') as cargo:
            with self.assertRaisesRegex(RuntimeError, 'configuration changed'):
                dependencies.check_dependency_identity(self.evidence)
            cargo.assert_not_called()

    def test_changed_lock_invalidates_identity_before_cargo(self):
        self.production_lock.write_text('# changed lock\n')
        with patch.object(dependencies.subprocess, 'check_output') as cargo:
            with self.assertRaisesRegex(RuntimeError, 'configuration changed'):
                dependencies.check_dependency_identity(self.evidence)
            cargo.assert_not_called()

    def test_missing_resolution_binding_is_rejected(self):
        del self.evidence['resolution']
        with self.assertRaisesRegex(RuntimeError, 'resolution binding is missing'):
            self.check()

    def test_re_resolved_path_drift_is_rejected(self):
        proof, production = copy.deepcopy(self.proof), copy.deepcopy(self.production)
        for metadata in [proof, production]:
            metadata['packages'][1]['manifest_path'] = str(self.root / 'replacement/Cargo.toml')
        with self.assertRaisesRegex(RuntimeError, 'resolution changed'):
            self.check(proof, production)

    def test_binary_change_during_resolution_is_rejected(self):
        outputs = iter([json.dumps(self.proof), json.dumps(self.production)])

        def metadata(*_args, **_kwargs):
            self.binary.write_bytes(b'changed during metadata')
            return next(outputs)

        with patch.object(dependencies.subprocess, 'check_output', side_effect=metadata):
            with self.assertRaisesRegex(RuntimeError, 'artifacts changed while rechecking'):
                dependencies.check_dependency_identity(self.evidence)

    def test_wrong_registry_pin_is_rejected_before_cargo(self):
        lock = '[[package]]\nname="num"\nversion="0.4.3"\nsource="registry+fixture"\nchecksum="original"\n'
        self.production_lock.write_text(lock)
        self.proof_manifest.with_name('Cargo.lock').write_text(lock.replace('original', 'changed'))
        with patch.object(dependencies.subprocess, 'check_output') as cargo:
            with self.assertRaisesRegex(RuntimeError, 'does not match production lock'):
                dependencies.prepare_dependencies(self.root, {}, self.root / 'reports')
            cargo.assert_not_called()


if __name__ == '__main__':
    unittest.main()
