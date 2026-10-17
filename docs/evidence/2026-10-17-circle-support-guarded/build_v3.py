"""Keep all source bytes; compile relative paths so proptest can persist failures."""
import os
import shutil
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True
from runner_v1 import RUNTIME, W, dump, read, run, sha

old = RUNTIME / 'build-v2'
prior = read(old / 'protocol.json')
built = read(old / 'result.json')
assert built['complete'] and built['output_closed'] and not built['command_failures']
assert read(RUNTIME / 'corpus-v1/result.json')['exit_code'] == -15
assert read(W / 'corpus-stop-v1.json')['requested_signal'] == 'SIGTERM'
output = RUNTIME / 'build-v3'
output.mkdir(exist_ok=False)
mirror = output / 'runtime-source-mirror'
mirror.mkdir()
for path, digest in prior['sources']['candidate']['sha256'].items():
    original = Path(prior['sources']['candidate']['path']) / path
    assert sha(original) == digest
    dest = mirror / path
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(original, dest)
pins = {str(path): sha(path) for path in [Path(__file__), W / 'runner_v1.py', W / 'contract-v1.md',
    old / 'result.json', old / 'protocol.json', W / 'corpus-stop-v1.json', RUNTIME / 'corpus-v1/result.json']}
dump(output / 'protocol.json', prior | {
    'input_sha256': prior['input_sha256'] | pins,
    'only_change_from_v2': 'Both variants use a relative Rust crate source argument and no path remap. Production, tests, dependencies, flags and policies otherwise match v2.',
    'relative_crate_source': 'crates/hypercurve/src/lib.rs', 'source_path_remapping': False,
    'runtime_source_mirror_sha256': prior['sources']['candidate']['sha256'],
    'existing_generated_test_environment': {key: value for key, value in os.environ.items() if key.startswith('PROPTEST_')},
    'baseline_reused_read_only': False,
})
shutil.copy2(old / 'main-input-hashes.json.gz', output / 'main-input-hashes.json.gz')
commands, binaries = [], {}
for label, description in prior['sources'].items():
    source = Path(description['path'])
    assert all(sha(source / path) == digest for path, digest in description['sha256'].items())
    binary = output / (label + '-tests')
    command = ['rustc', '--crate-name', 'hypercurve', '--edition=2024', '--test',
        'crates/hypercurve/src/lib.rs', '-C', 'opt-level=3', '-C', 'codegen-units=16',
        '-C', 'embed-bitcode=no', '-D', 'warnings',
        '-L', 'dependency=' + str(Path(prior['externs']['hyperreal']).parent), '-o', str(binary)]
    for name, path in prior['externs'].items():
        command += ['--extern', name + '=' + path]
    row = run(output, label, command, source)
    commands.append(row)
    dump(output / 'commands.json', commands)
    if row['exit_code'] != 0:
        if row['external_stop_reason']:
            break
        continue
    binaries[label] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    commands.append(run(output, label + '-contracts', [str(binary),
        'curve_intersection::circle_support_cache_tests::', '--nocapture', '--test-threads=1'], mirror))
    commands.append(run(output, label + '-property-persistence', [str(binary),
        'algebraic_image_tests::linear_coordinate_images_match_exact_midpoint_values',
        '--exact', '--nocapture', '--test-threads=1'], mirror))
    if commands[-1]['exit_code'] == 0:
        assert 'test result: ok. 1 passed; 0 failed;' in (output / (label + '-property-persistence.stdout')).read_text()
        assert 'FileFailurePersistence' not in (output / (label + '-property-persistence.stderr')).read_text()
    lint = command.copy()
    lint[0] = 'clippy-driver'
    lint[lint.index('-o') + 1] = str(output / (label + '.rmeta'))
    lint.append('--emit=metadata')
    commands.append(run(output, label + '-clippy', lint, source))
    dump(output / 'commands.json', commands)
assert all(sha(path) == digest for path, digest in (prior['input_sha256'] | pins | prior['read_only_dependencies_sha256']).items())
assert all(sha(Path(value['path']) / path) == digest
           for value in prior['sources'].values() for path, digest in value['sha256'].items())
assert all(sha(mirror / path) == digest for path, digest in prior['sources']['candidate']['sha256'].items())
generated = {str(path.relative_to(mirror)): sha(path) for path in mirror.rglob('*')
             if path.is_file() and str(path.relative_to(mirror)) not in prior['sources']['candidate']['sha256']}
dump(output / 'result.json', {'complete': True, 'output_closed': True, 'binaries': binaries,
    'command_failures': [row['name'] for row in commands if row['exit_code'] != 0],
    'sources_and_dependencies_unchanged': True, 'main_sources_unchanged': True,
    'runtime_mirror_unchanged': True, 'generated_outputs': generated,
    'property_persistence_warnings': False, 'cache_admitted': False})
raise SystemExit(0 if len(binaries) == 2 and not any(row['exit_code'] for row in commands) else 101)
