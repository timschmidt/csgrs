"""Replay both complete unmodified test targets against final expected artifacts."""
import gzip, hashlib, json, os, re, resource, subprocess, time
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
O = W / 'snapshot-requalification-v1'
SOURCE = W / 'candidate-source-v6'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2) + '\n')

native = read(W / 'consumer-validation-v1/result.json')
assert native['complete'] and native['unexpected_exits'] == ['circuit-all'], native
assert read(W / 'candidate-preparation-v6/result.json')['production_code_and_test_bodies_unchanged_from_v5']
with gzip.open(W / 'candidate-preparation-v6/source-hashes.json.gz', 'rt') as f:
    inputs = json.load(f)
with gzip.open(W / 'candidate-preparation-v5/source-hashes.json.gz', 'rt') as f:
    compiled = json.load(f)
assert inputs.keys() == compiled.keys()
assert {p for p in inputs if inputs[p] != compiled[p]} == set(read(W / 'candidate-preparation-v6/result.json')['delta_from_v5'])
assert all(sha(SOURCE / p) == h for p, h in inputs.items())
assert all(sha(W / 'candidate-source-v5' / p) == h for p, h in compiled.items())
build = W / 'consumer-validation-v1/circuit-build.stdout'
commands = read(W / 'consumer-validation-v1/commands.json')
build_row = next(r for r in commands if r['name'] == 'circuit-build')
assert build_row['exit_code'] == 0 and sha(build) == build_row['stdout_sha256']
artifacts = [json.loads(line) for line in build.read_text().splitlines() if line.startswith('{')]
library_names = ['hypercircuit', 'hyperreal', 'hypersolve', 'semver', 'hyperlattice', 'hyperlimit',
    'hyperpath', 'hypercurve', 'csgrs', 'hyperdrc', 'hyperphysics', 'serde', 'serde_json', 'sha2',
    'toml', 'ring', 'zip', 'quick_xml', 'unicode_normalization', 'proptest']
libraries = {name: next(Path(p) for row in artifacts if row.get('reason') == 'compiler-artifact'
    and row['target']['name'] == name and (name != 'zip' or row['package_id'].endswith('#zip@2.4.2')) for p in row['filenames'] if p.endswith('.rlib')) for name in library_names}
features = next(row['features'] for row in artifacts if row.get('reason') == 'compiler-artifact'
    and row['target']['name'] == 'hypercircuit' and 'lib' in row['target']['kind'])
assert set(features) == {'behavior-async', 'default', 'dispatch-trace', 'drc', 'geometry', 'interchange', 'layout', 'lceda'}
retention = read(W / 'snapshot-replay-libraries-v1/result.json')
assert retention['complete'] and retention['completed_build_stdout_sha256'] == sha(build)
libraries = {name: Path(retention['retained'][str(path)]['retained']) for name, path in libraries.items()}
deps = W / 'snapshot-replay-libraries-v1/libraries'
readonly = {r['retained']: r['sha256'] for r in retention['retained'].values()}
assert all(sha(p) == h for p, h in readonly.items())
O.mkdir(exist_ok=False)
env = os.environ.copy()
for key in ['RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER',
            'UPDATE_HYPERCIRCUIT_EASYDUINO_SNAPSHOTS', 'UPDATE_HYPERCIRCUIT_GOLDEN']:
    env.pop(key, None)
env['CARGO_MANIFEST_DIR'] = str(SOURCE / 'crates/hypercircuit')
flags = ['rustc', '--edition=2024', '--test', '-D', 'warnings', '-C', 'opt-level=3',
    '-C', 'embed-bitcode=no', '-C', 'strip=debuginfo', '-L', 'dependency=' + str(deps)]
for feature in features:
    flags += ['--cfg', 'feature="' + feature + '"']
for name, path in libraries.items():
    flags += ['--extern', name + '=' + str(path)]
pins = {str(p): sha(p) for p in [Path(__file__), W / 'metadata-contract-v1.md',
    W / 'metadata-migration-v2/result.json', W / 'candidate-preparation-v6/source-hashes.json.gz',
    W / 'consumer-validation-v1/result.json', W / 'snapshot-replay-libraries-v1/result.json', build]}
dump(O / 'protocol.json', {'input_sha256': pins, 'source': str(SOURCE),
    'source_inputs_sha256': inputs, 'read_only_library_sha256': readonly,
    'libraries': {n: str(p) for n, p in libraries.items()}, 'features': features,
    'cargo_manifest_dir': env['CARGO_MANIFEST_DIR'], 'snapshot_update_environment_disabled': True,
    'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
    'timeout_seconds_per_command': 1800, 'address_space_bytes': 8 * 1024**3,
    'completed_cargo_target_reopened': False,
    'scope': 'Recompile unchanged easyduino/workflow integration test sources against completed v5 all-features libraries. Only seven checked expected artifacts differ in the final snapshot; every production source and test body is unchanged.'})
rows = []

def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

def run(name, command):
    print('START snapshot ' + name, flush=True)
    begun = time.monotonic()
    with (O / (name + '.stdout')).open('wb') as out, (O / (name + '.stderr')).open('wb') as err:
        result = subprocess.run(['/usr/bin/time', '-v', '-o', str(O / (name + '.resources')),
            'timeout', '--kill-after=20s', '1800s', *command], cwd=SOURCE, env=env,
            stdout=out, stderr=err, preexec_fn=limits, start_new_session=True)
    row = {'name': name, 'command': command, 'exit_code': result.returncode,
        'elapsed_seconds': time.monotonic() - begun,
        **{s + '_sha256': sha(O / (name + '.' + s)) for s in ['stdout', 'stderr', 'resources']}}
    rows.append(row)
    dump(O / 'commands.json', rows)
    print(json.dumps({k: row[k] for k in ['name', 'exit_code', 'elapsed_seconds']}), flush=True)
    assert result.returncode == 0, row

run('format', ['cargo', 'fmt', '--all', '--', '--check'])
passing = {}
for name in ['easyduino', 'workflow']:
    source = SOURCE / 'crates/hypercircuit/tests' / (name + '.rs')
    binary = O / name
    run(name + '-compile', [*flags, '--crate-name', name, str(source), '-o', str(binary)])
    run(name, [str(binary), '--nocapture', '--test-threads=1'])
    output = (O / (name + '.stdout')).read_text()
    counts = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', output)
    assert counts == [('9', '0', '0')], counts
    passing[name] = {'passed': 9, 'failed': 0, 'ignored': 0,
        'test_names': re.findall(r'^test (\S+) \.\.\. ok$', output, re.M),
        'binary': str(binary), 'binary_sha256': sha(binary), 'binary_bytes': binary.stat().st_size}
    assert len(passing[name]['test_names']) == 9
old_failures = set(re.findall(r'^test (\S+) \.\.\. FAILED$',
    (W / 'consumer-validation-v1/circuit-all.stdout').read_text(), re.M))
assert len(old_failures) == 7
assert old_failures <= {n for p in passing.values() for n in p['test_names']}
assert all(sha(SOURCE / p) == h for p, h in inputs.items())
assert all(sha(p) == h for p, h in readonly.items())
assert all(sha(p) == h for p, h in pins.items())
dump(O / 'result.json', {'complete': True, 'commands': len(rows), 'targets': passing,
    'all_seven_prior_failures_replayed_successfully': True,
    'all_source_and_library_inputs_unchanged': True, 'test_bodies_and_original_fixture_inputs_unchanged': True,
    'snapshot_update_environment_disabled': True, 'main_sources_modified': False,
    'original_cargo_suite_exit_code': 101, 'original_cargo_suite_relabelled_as_success': False})
print(json.dumps({'complete': True, 'passed': 18, 'failed': 0, 'original_suite_preserved_as_failed': True}), flush=True)
