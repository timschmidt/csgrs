"""Qualify the final consumer migration and complete the unchanged curve corpus."""
import sys
sys.dont_write_bytecode = True
from consumer_common_v1 import Validation, read, WORK

assert read(WORK / 'circuit-baseline-v1/result.json')['baseline_compile_failure_reproduced']
assert read(WORK / 'candidate-validation-v4/result.json')['unexpected_exits'] == ['curve-all', 'circuit-all', 'clippy']
assert read(WORK / 'candidate-preparation-v5/result.json')['stroke_sources_unchanged_from_v4']
v = Validation('candidate', __file__)
v.run('format', ['cargo', 'fmt', '--all', '--', '--check'])
v.run('circuit-build', ['cargo', 'test', '--offline', '--locked', '-p', 'hypercircuit', '--release', '--all-features', '--no-run', '--message-format=json'])
v.run('text-fabrication', ['cargo', 'test', '--offline', '--locked', '-p', 'hypercircuit', '--release', '--all-features', '--test', 'fabrication', '--', '--nocapture', '--test-threads=1'])
v.run('circuit-all', ['cargo', 'test', '--no-fail-fast', '--offline', '--locked', '-p', 'hypercircuit', '--release', '--all-features', '--', '--test-threads=1'])
v.run('curve-all-build', ['cargo', 'test', '--offline', '--locked', '-p', 'hypercurve', '--release', '--all-features', '--no-run', '--message-format=json'])
v.run('curve-all', ['cargo', 'test', '--no-fail-fast', '--offline', '--locked', '-p', 'hypercurve', '--release', '--all-features', '--', '--test-threads=1'])
v.run('clippy', ['cargo', 'clippy', '--offline', '--locked', '-p', 'hypercurve', '-p', 'csgrs', '-p', 'hyperdrc', '-p', 'hypercircuit', '--release', '--all-features', '--all-targets', '--', '-D', 'warnings'])
v.run('circuit-docs', ['cargo', 'doc', '--offline', '--locked', '-p', 'hypercircuit', '--release', '--all-features', '--no-deps'], docs=True)
v.run('native-cli', ['cargo', 'build', '--offline', '--locked', '-p', 'hyperdrc', '--release', '--all-features', '--message-format=json'])
sys.exit(v.finish({'production_snapshot': 'candidate-source-v5',
    'preserved_incomplete_attempt': 'candidate-validation-v4',
    'v4_unchanged_dependency_closures': ['hypercurve', 'csgrs', 'hyperdrc'],
    'curve_test_command_unchanged': True, 'original_workload_gates_unchanged': True,
    'text_failure_contract_migration': 'Selected font or glyph failure returns typed Construction error, with no materialization report.',
    'full_pre_push_matrix_executed': False, 'published_package_pipeline_executed': False}))
