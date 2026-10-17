"""Qualify the all-feature library against independently pinned frozen dependencies."""
import gzip
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True
from runner_v1 import ROOT, RUNTIME, W, dump, read, run, sha

built = read(RUNTIME / 'build-v3/result.json')
current = read(RUNTIME / 'build-v3/protocol.json')
assert built['complete'] and built['output_closed'] and not built['command_failures']
old = W.parent / 'circle-support-corpus'
assert read(old / 'curve-all-v1/result.json')['cargo_target_closed']
assert not read(old / 'curve-all-v1/result.json')['command_failures']
old_build = next(row for row in read(old / 'curve-all-v1/commands.json') if row['name'] == 'build')
assert sha(old / 'curve-all-v1/build.stdout') == old_build['stdout_sha256']
artifacts = [json.loads(line) for line in (old / 'curve-all-v1/build.stdout').read_text().splitlines() if line.startswith('{')]
features = next(row['features'] for row in artifacts if row.get('reason') == 'compiler-artifact'
                and row['target']['name'] == 'hypercurve' and row.get('executable'))
assert features == ['comparative-benchmarks', 'default', 'dispatch-trace', 'hershey', 'svg', 'triangulation']
externs = {}
for name in ['hyperreal', 'hypersolve', 'hyperlimit', 'hypertri', 'num', 'cavalier_contours',
             'curvo', 'i_overlay', 'nalgebra', 'svg', 'svgtypes', 'geo', 'proptest']:
    paths = {path for row in artifacts if row.get('reason') == 'compiler-artifact'
             and row['target']['name'] == name and (name != 'i_overlay' or row['package_id'].endswith('@7.0.3'))
             for path in row['filenames'] if path.endswith('.rlib')}
    assert len(paths) == 1, (name, paths)
    externs[name] = paths.pop()
dependency_dir = Path(externs['hyperreal']).parent
with gzip.open(ROOT / 'docs/evidence/2026-10-17-circle-support-corpus/artifact-expectations-v1.json.gz', 'rt') as stream:
    expected = json.load(stream)['added_physical_artifacts_sha256']
dependencies = {str(path): sha(path) for path in dependency_dir.iterdir()
                if path.is_file() and path.suffix in ['.rlib', '.rmeta', '.so']}
assert all(expected[path] == digest for path, digest in dependencies.items())
with gzip.open(old / 'preparation-v1/source-hashes.json.gz', 'rt') as stream:
    prior_source = json.load(stream)
dependency_sources = {path: digest for path, digest in prior_source.items()
    if path in ['Cargo.toml', 'Cargo.lock'] or path.startswith(tuple(
        'crates/' + name + '/' for name in ['hyperreal', 'hypersolve', 'hyperlimit', 'hypertri']))}
assert all(sha(ROOT / path) == digest for path, digest in dependency_sources.items())
source = current['sources']['candidate']
assert all(sha(Path(source['path']) / path) == digest for path, digest in source['sha256'].items())
output = RUNTIME / 'all-feature-library-v1'
output.mkdir(exist_ok=False)
mirror = output / 'runtime-source-mirror'
mirror.mkdir()
for path, digest in source['sha256'].items():
    dest = mirror / path
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(Path(source['path']) / path, dest)
pins = {str(path): sha(path) for path in [Path(__file__), W / 'runner_v1.py', W / 'contract-v1.md',
    RUNTIME / 'build-v3/result.json', RUNTIME / 'build-v3/protocol.json',
    old / 'curve-all-v1/build.stdout', old / 'curve-all-v1/result.json', old / 'preparation-v1/source-hashes.json.gz']}
dump(output / 'protocol.json', {
    'input_sha256': pins, 'read_only_dependencies_sha256': dependencies,
    'dependency_source_sha256': dependency_sources, 'source': source, 'externs': externs, 'features': features,
    'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
    'clippy_version': subprocess.check_output(['clippy-driver', '--version'], text=True),
    'library_only': True, 'integration_doc_bench_or_downstream_coverage_claimed': False,
    'existing_generated_test_environment': {key: value for key, value in os.environ.items() if key.startswith('PROPTEST_')},
    'relative_source_paths_preserve_property_failure_persistence': True,
    'functional_run_may_overlap_independent_no_feature_library_run': True,
    'performance_measurement_waits_until_both_runs_are_terminal': True,
    'completed_cargo_targets_reopened': False,
})
binary = output / 'hypercurve-tests'
command = ['rustc', '--crate-name', 'hypercurve', '--edition=2024', '--test', 'crates/hypercurve/src/lib.rs',
           '-C', 'opt-level=3', '-C', 'codegen-units=16', '-C', 'embed-bitcode=no', '-D', 'warnings',
           '-L', 'dependency=' + str(dependency_dir), '-o', str(binary)]
for feature in features:
    command += ['--cfg', 'feature="' + feature + '"']
for name, path in externs.items():
    command += ['--extern', name + '=' + path]
commands = [run(output, 'build', command, Path(source['path']))]
dump(output / 'commands.json', commands)
binaries = {}
if commands[-1]['exit_code'] == 0:
    binaries[str(binary)] = {'sha256': sha(binary), 'bytes': binary.stat().st_size}
    commands.append(run(output, 'tests', [str(binary), '--nocapture', '--test-threads=1'], mirror, {'RUST_BACKTRACE': '0'}))
    dump(output / 'commands.json', commands)
    lint = command.copy()
    lint[0] = 'clippy-driver'
    lint[lint.index('-o') + 1] = str(output / 'hypercurve.rmeta')
    lint.append('--emit=metadata')
    commands.append(run(output, 'clippy', lint, Path(source['path'])))
    dump(output / 'commands.json', commands)
assert all(sha(path) == digest for path, digest in (pins | dependencies).items())
assert all(sha(ROOT / path) == digest for path, digest in dependency_sources.items())
assert all(sha(Path(source['path']) / path) == digest for path, digest in source['sha256'].items())
unchanged = all(sha(mirror / path) == digest for path, digest in source['sha256'].items())
generated = {str(path.relative_to(mirror)): sha(path) for path in mirror.rglob('*')
             if path.is_file() and str(path.relative_to(mirror)) not in source['sha256']}
text = (output / 'tests.stdout').read_text() if (output / 'tests.stdout').exists() else ''
stderr = (output / 'tests.stderr').read_text() if (output / 'tests.stderr').exists() else ''
summaries = re.findall(r'test result: (\w+)\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;', text)
dump(output / 'result.json', {'complete': True, 'output_closed': True,
     'command_failures': [row['name'] for row in commands if row['exit_code'] != 0],
     'source_mirror_unchanged': unchanged, 'generated_outputs': generated, 'binaries': binaries,
     'property_persistence_warnings': 'FileFailurePersistence' in stderr,
     'test_summaries': summaries, 'library_only': True, 'all_inputs_unchanged': True,
     'cache_admitted': False, 'full_plan_complete': False})
raise SystemExit(0 if len(commands) == 3 and not any(row['exit_code'] != 0 for row in commands)
                 and unchanged and 'FileFailurePersistence' not in stderr else 101)
