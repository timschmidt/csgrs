"""Qualify source fidelity before the unchanged whole-board resource gates."""
import sys
sys.dont_write_bytecode = True
from validation_common_v4 import Validation, read, WORK

baseline = read(WORK / 'baseline-acceptance-v1/result.json')
assert baseline['complete']
assert baseline['counts']['kicad_native_strokes']['counts'] == [['0', '10', '0']]
assert baseline['counts']['kicad_track_failures']['counts'] == [['38', '1', '0']]
v = Validation('candidate', __file__)
v.run('format', ['cargo', 'fmt', '--all', '--', '--check'])
if v.run('native-build', ['cargo', 'test', '--offline', '--locked', '-p', 'hyperdrc', '--release', '--all-features', '--no-run', '--message-format=json']):
    sys.exit(v.finish({'dependent_runtime_checks_attempted': False, 'reason': 'native build failed'}))
focus = []
focus.append(v.run('cold-zero-budget', ['cargo', 'test', '--offline', '--locked', '-p', 'hyperdrc', '--release', '--all-features', '--test', 'kicad_native_strokes', 'fresh_zero_budget_load_retains_native_rational_direction_certificates', '--', '--exact', '--nocapture', '--test-threads=1']))
focus.append(v.run('native-strokes', ['cargo', 'test', '--offline', '--locked', '-p', 'hyperdrc', '--release', '--all-features', '--test', 'kicad_native_strokes', '--', '--nocapture', '--test-threads=1']))
focus.append(v.run('source-failures', ['cargo', 'test', '--offline', '--locked', '-p', 'hyperdrc', '--release', '--all-features', '--test', 'kicad_track_failures', '--', '--nocapture', '--test-threads=1']))
focus.append(v.run('native-aggregation', ['cargo', 'test', '--offline', '--locked', '-p', 'hyperdrc', '--release', '--all-features', '--test', 'kicad_native_aggregation', '--', '--nocapture', '--test-threads=1']))
if any(focus):
    sys.exit(v.finish({'workload_gates_attempted': False, 'reason': 'source-fidelity prerequisite failed'}))
for name, test in [('full-board', 'kicad_board_completes_smoke_check_suite'),
                   ('copper-neck', 'min_copper_neck_width_completes_on_copper_layers'),
                   ('gerber', 'gerber_package_completes_smoke_check_suite')]:
    v.run(name, ['cargo', 'test', '--offline', '--locked', '-p', 'hyperdrc', '--release', '--all-features', '--lib',
        'zz_complex_project_performance_tests::' + test, '--', '--exact', '--nocapture', '--test-threads=1'])
v.run('native-tests', ['cargo', 'test', '--no-fail-fast', '--offline', '--locked', '-p', 'hyperdrc', '--release', '--all-features', '--', '--test-threads=1',
    '--skip', 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite',
    '--skip', 'zz_complex_project_performance_tests::min_copper_neck_width_completes_on_copper_layers',
    '--skip', 'zz_complex_project_performance_tests::gerber_package_completes_smoke_check_suite'])
v.run('clippy', ['cargo', 'clippy', '--offline', '--locked', '-p', 'hyperdrc', '--release', '--all-features', '--all-targets', '--', '-D', 'warnings'])
v.run('docs', ['cargo', 'doc', '--offline', '--locked', '-p', 'hyperdrc', '--release', '--all-features', '--no-deps'], docs=True)
v.run('native-cli', ['cargo', 'build', '--offline', '--locked', '-p', 'hyperdrc', '--release', '--all-features', '--message-format=json'])
sys.exit(v.finish({'source_fidelity_migration': 'Exact source coordinates and round caps replace finite rectangles.',
    'readiness_version_migration': [21, 22], 'original_workload_gates_unchanged': True,
    'full_pre_push_matrix_executed': False, 'published_package_pipeline_executed': False,
    'downstream_hypercircuit_validation_complete': False}))
