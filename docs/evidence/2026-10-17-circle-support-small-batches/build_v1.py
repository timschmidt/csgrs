"""Build unchanged guarded production with one identical repeated-batch probe."""
import gzip
import json
import shutil
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True
from runner_v1 import ROOT, RUNTIME, W, dump, read, resources, run, sha, violation

base = '6479d3ba982777a1e9ecc8ce24ef83752d2e474b'
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == base
subprocess.run(['git', 'diff', '--quiet'], cwd=ROOT, check=True)
subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=ROOT, check=True)
assert violation(resources()) is None
old = Path('/run/user/1000/csgrs-circle-support-guarded-v1/build-v3')
prior = read(old / 'protocol.json')
previous = read(old / 'result.json')
assert previous['complete'] and previous['output_closed'] and not previous['command_failures']
assert subprocess.check_output(['rustc', '-vV'], text=True) == prior['rustc']
assert all(sha(path) == digest for path, digest in prior['read_only_dependencies_sha256'].items())
with gzip.open(old / 'main-input-hashes.json.gz', 'rt') as stream:
    old_main = json.load(stream)
main = {path: sha(ROOT / path) for path in old_main}
assert {path for path in main if main[path] != old_main[path]} == {'docs/properties-status.json'}
output = RUNTIME / 'build-v1'
output.mkdir(exist_ok=False)
probe = W / 'small_batch_probe_v1.rs'
commands = [run(output, 'probe-format', ['rustfmt', '--edition', '2024', '--config',
                'skip_children=true', str(probe)], ROOT)]
assert commands[-1]['exit_code'] == 0
pins = {str(path): sha(path) for path in [Path(__file__), W / 'runner_v1.py',
    W / 'contract-v1.md', probe, old / 'protocol.json', old / 'result.json']}
sources = {}
hook = '\n#[cfg(test)]\n#[path = "circle_support_small_batch_probe.rs"]\nmod circle_support_small_batch_probe;\n'
intersection = 'crates/hypercurve/src/curve_intersection.rs'
for label, description in prior['sources'].items():
    source = output / (label + '-source')
    source.mkdir()
    for path, digest in description['sha256'].items():
        original = Path(description['path']) / path
        assert sha(original) == digest
        dest = source / path
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(original, dest)
    shutil.copy2(probe, source / 'crates/hypercurve/src/circle_support_small_batch_probe.rs')
    path = source / intersection
    path.write_text(path.read_text() + hook)
    assert path.read_text().removesuffix(hook) == (Path(description['path']) / intersection).read_text()
    sources[label] = {'path': str(source), 'sha256': {
        str(path.relative_to(source)): sha(path) for path in source.rglob('*') if path.is_file()}}
with gzip.open(output / 'main-input-hashes.json.gz', 'wt') as stream:
    json.dump(main, stream, sort_keys=True)
dump(output / 'protocol.json', {
    'base_commit': base, 'sources': sources, 'input_sha256': pins,
    'read_only_dependencies_sha256': prior['read_only_dependencies_sha256'],
    'externs': prior['externs'], 'rustc': prior['rustc'], 'hardware': prior['hardware'],
    'features': prior['features'], 'main_input_hashes_sha256': sha(output / 'main-input-hashes.json.gz'),
    'production_bytes_unchanged_from_e42': True,
    'only_new_source_is_identical_diagnostic': True,
    'completed_targets_reopened': False, 'external_timeout_seconds': 1800,
    'address_space_bytes': 8 * 1024**3, 'initial_resources': resources(),
    'clippy_version': subprocess.check_output(['clippy-driver', '--version'], text=True),
    'direct_clippy_scope': 'No-feature diagnostic test harness, not the full Cargo target/feature/manifest matrix.',
})
binaries = {}
for label, description in sources.items():
    source = Path(description['path'])
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
        break
    binaries[label] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    commands.append(run(output, label + '-contracts', [str(binary),
        'curve_intersection::circle_support_cache_tests::', '--nocapture', '--test-threads=1'], source))
    lint = command.copy()
    lint[0] = 'clippy-driver'
    lint[lint.index('-o') + 1] = str(output / (label + '.rmeta'))
    lint.append('--emit=metadata')
    commands.append(run(output, label + '-clippy', lint, source))
    dump(output / 'commands.json', commands)
    if any(row['exit_code'] != 0 for row in commands):
        break
assert all(sha(path) == digest for path, digest in (pins | prior['read_only_dependencies_sha256']).items())
assert all(sha(Path(value['path']) / path) == digest
           for value in sources.values() for path, digest in value['sha256'].items())
assert all(sha(Path(value['path']) / path) == digest
           for value in prior['sources'].values() for path, digest in value['sha256'].items())
assert all(sha(ROOT / path) == digest for path, digest in main.items())
failures = [row['name'] for row in commands if row['exit_code'] != 0]
dump(output / 'result.json', {'complete': True, 'output_closed': True, 'binaries': binaries,
    'command_failures': failures, 'main_sources_unchanged': True,
    'frozen_sources_and_dependencies_unchanged': True, 'cache_admitted': False})
raise SystemExit(0 if len(binaries) == 2 and not failures else 101)
