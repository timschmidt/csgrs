"""Exercise export identity checks with synthetic receipts, not arithmetic proofs."""
import copy
import json
from pathlib import Path
import tempfile
import sys
import unittest

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

import verus_bigint as binding
import verus_dependencies as dependencies


class VerifiedExportTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.vendor = self.root / 'vendor'
        (self.vendor / 'src').mkdir(parents=True)
        (self.vendor / 'src/lib.rs').write_text('fixture source\n')
        self.tool = self.root / 'tool'
        self.tool.mkdir()
        (self.tool / 'verus').write_text('fixture tool\n')
        self.model = self.root / 'model.rs'
        self.model.write_text('fixture model\n')
        self.helper = self.root / 'helper.py'
        self.helper.write_text('fixture helper\n')
        self.library = self.root / 'libnum_bigint-fixture.rlib'
        self.library.write_bytes(b'fixture library')
        self.metadata = self.root / 'libnum_bigint-fixture.rmeta'
        self.metadata.write_bytes(b'fixture metadata')
        self.export = self.root / 'num_bigint.vir'
        self.export.write_bytes(b'fixture export; this is not a proof')
        self.config = {
            'vendor': str(self.vendor), 'vendor_files': binding.catalog(self.vendor),
            'tool': str(self.tool / 'verus'), 'tool_directory': str(self.tool), 'tool_files': binding.catalog(self.tool),
            'model_sources': {str(self.model): binding.sha256(self.model)},
            'helpers': {str(self.helper): binding.sha256(self.helper)}, 'package_id': 'fixture-package',
            'pin': {'version': 'fixture-version', 'commit': 'fixture-commit'}, 'export': str(self.export),
            'result': str(self.root / 'result.json'), 'invocation': str(self.root / 'invocation.json'),
            'stdout': str(self.root / 'stdout.json'), 'stderr': str(self.root / 'stderr.json'),
        }
        config_path = self.root / 'config.json'
        self.write(config_path, self.config)
        self.context = {'config': str(config_path), 'config_sha256': binding.sha256(config_path),
                        'package_id': 'fixture-package'}
        self.arguments = ['--crate-name', 'num_bigint', '--crate-type', 'lib', str(self.vendor / 'src/lib.rs')]
        self.invocation = {'command': [self.config['tool'], '--no-cheating', '--output-json', '--compile',
            '--export', self.config['export'], *self.arguments], 'cargo_arguments': self.arguments,
            'manifest_directory': str(self.vendor), 'config_sha256': self.context['config_sha256']}
        self.write(self.config['invocation'], self.invocation)
        self.report = {
            'verification-results': {'verified': binding.EXPECTED_UNITS, 'errors': 0, 'success': True,
                'is-verifying-entire-crate': True, 'encountered-error': False, 'encountered-vir-error': False},
            'func-details': {name: {} for name in binding.REQUIRED_FUNCTIONS}, 'verus': self.config['pin'],
        }
        self.write(self.config['stdout'], self.report)
        Path(self.config['stderr']).write_text('')
        self.result = {'gate_passed': True, 'exit_code': 0, 'verified': binding.EXPECTED_UNITS, 'warnings': 0,
            'config_sha256': self.context['config_sha256'], 'export_sha256': binding.sha256(self.export),
            'library_artifacts': {str(p): binding.sha256(p) for p in [self.library, self.metadata]}}
        self.refresh_result()
        self.artifacts = [{'package_id': 'fixture-package', 'target': {'name': 'num_bigint', 'kind': ['lib']},
            'filenames': [str(self.library), str(self.metadata)]}]

    def write(self, path, value):
        Path(path).write_text(json.dumps(value, sort_keys=True) + '\n')

    def refresh_result(self):
        for field, path in [('stdout_sha256', 'stdout'), ('stderr_sha256', 'stderr'), ('invocation_sha256', 'invocation')]:
            self.result[field] = binding.sha256(self.config[path])
        self.write(self.config['result'], self.result)

    def bind(self):
        return binding.bind_export(self.context, self.artifacts)

    def evidence(self):
        return self.bind()[1]

    def test_valid_receipt_binds_exact_library_and_export(self):
        arguments, evidence = self.bind()
        self.assertEqual(arguments, ['--import', 'num_bigint=' + str(self.export)])
        self.assertEqual(evidence['library_artifacts'], self.result['library_artifacts'])
        binding.check_export(evidence)

    def test_missing_compiler_execution_is_rejected(self):
        Path(self.config['result']).unlink()
        with self.assertRaisesRegex(RuntimeError, 'did not run'):
            self.bind()

    def test_different_cargo_library_path_is_rejected_even_for_equal_bytes(self):
        alternate = self.root / 'other.rlib'
        alternate.write_bytes(self.library.read_bytes())
        self.artifacts[0]['filenames'][0] = str(alternate)
        with self.assertRaisesRegex(RuntimeError, 'does not match the proved compilation'):
            self.bind()

    def test_wrong_cargo_package_is_rejected(self):
        self.artifacts[0]['package_id'] = 'another-package'
        with self.assertRaisesRegex(RuntimeError, 'Expected one verified'):
            self.bind()

    def test_duplicate_cargo_artifacts_are_rejected(self):
        self.artifacts.append(copy.deepcopy(self.artifacts[0]))
        with self.assertRaisesRegex(RuntimeError, 'Expected one verified'):
            self.bind()

    def test_missing_rlib_is_rejected(self):
        self.artifacts[0]['filenames'] = [str(self.metadata)]
        with self.assertRaisesRegex(RuntimeError, 'library is missing'):
            self.bind()

    def test_changed_export_is_rejected(self):
        evidence = self.evidence()
        self.export.write_bytes(b'another export')
        with self.assertRaisesRegex(RuntimeError, 'proof export changed'):
            binding.check_export(evidence)

    def test_equal_export_at_another_path_is_rejected(self):
        evidence = self.evidence()
        alternate = self.root / 'other.vir'
        alternate.write_bytes(self.export.read_bytes())
        evidence['export'] = str(alternate)
        with self.assertRaisesRegex(RuntimeError, 'export path changed'):
            binding.check_export(evidence)

    def test_changed_library_is_rejected(self):
        evidence = self.evidence()
        self.library.write_bytes(b'another library')
        with self.assertRaisesRegex(RuntimeError, 'library changed'):
            binding.check_export(evidence)

    def test_changed_compiler_result_is_rejected(self):
        evidence = self.evidence()
        self.result['gate_passed'] = False
        self.refresh_result()
        with self.assertRaisesRegex(RuntimeError, 'compiler result changed'):
            binding.check_export(evidence)

    def test_changed_invocation_is_rejected(self):
        evidence = self.evidence()
        self.invocation['command'].remove('--no-cheating')
        self.write(self.config['invocation'], self.invocation)
        with self.assertRaisesRegex(RuntimeError, 'compiler receipt changed'):
            binding.check_export(evidence)

    def test_semantically_invalid_invocation_is_rejected_before_binding(self):
        self.invocation['command'].remove('--no-cheating')
        self.write(self.config['invocation'], self.invocation)
        self.refresh_result()
        with self.assertRaisesRegex(RuntimeError, 'Compiler arguments changed'):
            self.bind()

    def test_wrong_manifest_in_receipt_is_rejected_before_binding(self):
        self.invocation['manifest_directory'] = str(self.root / 'other-package')
        self.write(self.config['invocation'], self.invocation)
        self.refresh_result()
        with self.assertRaisesRegex(RuntimeError, 'Compiler package path changed'):
            self.bind()

    def test_changed_configuration_is_rejected(self):
        evidence = self.evidence()
        self.config['export'] = 'other-export'
        self.write(self.context['config'], self.config)
        with self.assertRaisesRegex(RuntimeError, 'configuration changed'):
            binding.check_export(evidence)

    def test_changed_source_is_rejected(self):
        evidence = self.evidence()
        (self.vendor / 'src/lib.rs').write_text('different source')
        with self.assertRaisesRegex(RuntimeError, 'source catalog changed'):
            binding.check_export(evidence)

    def test_added_source_file_is_rejected(self):
        evidence = self.evidence()
        (self.vendor / 'new-input.bin').write_bytes(b'new source input')
        with self.assertRaisesRegex(RuntimeError, 'source catalog changed'):
            binding.check_export(evidence)

    def test_changed_scalar_model_is_rejected(self):
        evidence = self.evidence()
        self.model.write_text('different model')
        with self.assertRaisesRegex(RuntimeError, 'proof models changed'):
            binding.check_export(evidence)

    def test_changed_helper_is_rejected(self):
        evidence = self.evidence()
        self.helper.write_text('different build helper')
        with self.assertRaisesRegex(RuntimeError, 'build helpers changed'):
            binding.check_export(evidence)

    def test_changed_tool_is_rejected(self):
        evidence = self.evidence()
        (self.tool / 'verus').write_text('different verifier')
        with self.assertRaisesRegex(RuntimeError, 'tool catalog changed'):
            binding.check_export(evidence)

    def test_symbolic_link_source_is_rejected(self):
        evidence = self.evidence()
        (self.vendor / 'alias').symlink_to(self.model)
        with self.assertRaisesRegex(RuntimeError, 'Symbolic-link verification input'):
            binding.check_export(evidence)

    def test_missing_required_function_is_rejected(self):
        self.report['func-details'].pop(next(iter(binding.REQUIRED_FUNCTIONS)))
        self.write(self.config['stdout'], self.report)
        self.refresh_result()
        with self.assertRaisesRegex(RuntimeError, 'proof report is incomplete'):
            self.bind()

    def test_partial_module_proof_is_rejected(self):
        self.report['verification-results']['is-verifying-entire-crate'] = False
        self.write(self.config['stdout'], self.report)
        self.refresh_result()
        with self.assertRaisesRegex(RuntimeError, 'proof report is incomplete'):
            self.bind()

    def test_wrong_verifier_in_report_is_rejected(self):
        self.report['verus'] = self.config['pin'] | {'commit': 'other-commit'}
        self.write(self.config['stdout'], self.report)
        self.refresh_result()
        with self.assertRaisesRegex(RuntimeError, 'another verifier'):
            self.bind()

    def test_compiler_warning_is_rejected_even_if_summary_says_zero(self):
        self.write(self.config['stderr'], {'level': 'warning', 'message': 'fixture warning'})
        self.refresh_result()
        with self.assertRaisesRegex(RuntimeError, 'compiler reported warnings'):
            self.bind()

    def test_num_bigint_identity_requires_verified_metadata(self):
        with self.assertRaisesRegex(RuntimeError, 'metadata binding is missing'):
            dependencies.check_dependency_identity({'packages': [{'name': 'num-bigint'}]})


if __name__ == '__main__':
    unittest.main()
