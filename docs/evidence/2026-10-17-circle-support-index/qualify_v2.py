"""Run the unchanged affected native corpus on the final isolated cache patch."""
import gzip
import hashlib
import json
import os
import resource
import shutil
import subprocess
import sys
import time
from pathlib import Path

W = Path(__file__).resolve().parent
S = W / 'candidate-source-v2'
O = W / 'candidate-qualification-v2'
T = Path('/tmp/csgrs-circle-support-qualification-v2')

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, v):
    Path(p).write_text(json.dumps(v, indent=2) + '\n')

prep = read(W / 'candidate-preparation-v2/result.json')
assert prep['complete']
manifest = W / 'candidate-preparation-v2/source-hashes.json.gz'
assert sha(manifest) == prep['source_hashes_sha256']
with gzip.open(manifest, 'rt') as f:
    inputs = json.load(f)
assert all(sha(S / p) == h for p, h in inputs.items())
O.mkdir(exist_ok=False)
T.mkdir(exist_ok=False)
scratch = O / 'scratch'
scratch.mkdir()
retained = O / 'executables'
retained.mkdir()
env = os.environ.copy()
for key in ['RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER', 'RUSTDOCFLAGS', 'NODE_OPTIONS']:
    env.pop(key, None)
env.update({'CARGO_HOME': '/home/tim/.cargo', 'CARGO_TARGET_DIR': str(T), 'CARGO_BUILD_JOBS': '2',
            'CARGO_NET_OFFLINE': 'true', 'CARGO_INCREMENTAL': '0', 'TMPDIR': str(scratch),
            'CARGO_TERM_COLOR': 'never', 'RUST_BACKTRACE': '0'})
pins = {str(p): sha(p) for p in [Path(__file__), manifest, W / 'candidate-preparation-v2/result.json', W / 'contract-v1.md']}
dump(O / 'protocol.json', {
    'input_sha256': pins, 'source': str(S), 'target': str(T), 'base_commit': prep['base_commit'],
    'target_initially_empty': not list(T.iterdir()),
    'rustc': subprocess.check_output(['rustc', '-vV'], text=True), 'cargo': subprocess.check_output(['cargo', '-V'], text=True),
    'hardware': subprocess.check_output(['lscpu'], text=True),
    'environment': {k: env[k] for k in ['CARGO_HOME', 'CARGO_TARGET_DIR', 'CARGO_BUILD_JOBS', 'CARGO_NET_OFFLINE', 'CARGO_INCREMENTAL', 'TMPDIR']},
    'external_timeout_seconds_per_command': 1800, 'address_space_bytes_per_process': 8 * 1024**3,
    'build_and_execution_contained_separately': True,
    'original_workload_gates_unchanged': True, 'internal_resource_bound_claimed': False,
})
rows = []

def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

def run(name, command, docs=False):
    assert all(sha(S / p) == h for p, h in inputs.items())
    print('START ' + name, flush=True)
    start = time.monotonic()
    command_env = env | ({'RUSTDOCFLAGS': '-D warnings'} if docs else {})
    with (O / (name + '.stdout')).open('wb') as out, (O / (name + '.stderr')).open('wb') as err:
        r = subprocess.run(['/usr/bin/time', '-v', '-o', str(O / (name + '.resources')),
                            'timeout', '--signal=TERM', '--kill-after=20s', '1800s', *command],
                           cwd=S, env=command_env, stdout=out, stderr=err, preexec_fn=limits, start_new_session=True)
    assert all(sha(S / p) == h for p, h in inputs.items())
    assert all(sha(p) == h for p, h in pins.items())
    binaries = {}
    if '--message-format=json' in command:
        for line in (O / (name + '.stdout')).read_text(errors='replace').splitlines():
            if not line.startswith('{'):
                continue
            a = json.loads(line)
            if a.get('reason') != 'compiler-artifact' or not a.get('executable'):
                continue
            path = Path(a['executable'])
            digest = sha(path)
            dest = retained / digest / path.name
            dest.parent.mkdir(exist_ok=True)
            if not dest.exists():
                shutil.copy2(path, dest)
            assert sha(dest) == digest
            binaries[str(path)] = {'retained': str(dest), 'sha256': digest, 'bytes': path.stat().st_size,
                                   'target': a['target'], 'profile': a['profile']}
    row = {'name': name, 'command': command, 'exit_code': r.returncode, 'elapsed_seconds': time.monotonic() - start,
           'binaries': binaries, 'rustdocflags': command_env.get('RUSTDOCFLAGS'),
           **{suffix + '_sha256': sha(O / (name + '.' + suffix)) for suffix in ['stdout', 'stderr', 'resources']}}
    rows.append(row)
    dump(O / 'commands.json', rows)
    print(json.dumps({k: row[k] for k in ['name', 'exit_code', 'elapsed_seconds']}), flush=True)
    return r.returncode

def cargo(action, package, *args):
    return ['cargo', action, '--offline', '--locked', '-p', package, '--release', *args]

run('format', ['cargo', 'fmt', '--all', '--', '--check'])
run('clippy', ['cargo', 'clippy', '--offline', '--locked', '-p', 'hypercurve', '-p', 'csgrs', '-p', 'hyperdrc', '-p', 'hypercircuit',
               '--release', '--all-features', '--all-targets', '--', '-D', 'warnings'])
if not any(row['exit_code'] for row in rows):
    run('native-build', cargo('test', 'hyperdrc', '--all-features', '--no-run', '--message-format=json'))
    gates = {
        'full-board': 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite',
        'copper-neck': 'zz_complex_project_performance_tests::min_copper_neck_width_completes_on_copper_layers',
        'gerber': 'zz_complex_project_performance_tests::gerber_package_completes_smoke_check_suite',
    }
    for name, gate in gates.items():
        run(name, cargo('test', 'hyperdrc', '--all-features', '--lib', gate, '--', '--exact', '--nocapture', '--test-threads=1'))
    skips = [arg for gate in gates.values() for arg in ['--skip', gate]]
    run('native-tests', cargo('test', 'hyperdrc', '--all-features', '--no-fail-fast', '--', '--test-threads=1', *skips))
    run('curve-all-build', cargo('test', 'hypercurve', '--all-features', '--no-run', '--message-format=json'))
    run('curve-all', cargo('test', 'hypercurve', '--all-features', '--no-fail-fast', '--', '--test-threads=1'))
    run('root-all-build', cargo('test', 'csgrs', '--all-features', '--no-run', '--message-format=json'))
    run('root-all', cargo('test', 'csgrs', '--all-features', '--no-fail-fast', '--', '--test-threads=1'))
    run('root-minimal', cargo('test', 'csgrs', '--no-default-features', '--no-fail-fast', '--', '--test-threads=1'))
    run('circuit-build', cargo('test', 'hypercircuit', '--all-features', '--no-run', '--message-format=json'))
    run('circuit-all', cargo('test', 'hypercircuit', '--all-features', '--no-fail-fast', '--', '--test-threads=1'))
    run('docs', ['cargo', 'doc', '--offline', '--locked', '-p', 'hypercurve', '-p', 'csgrs', '-p', 'hyperdrc', '-p', 'hypercircuit',
                 '--release', '--all-features', '--no-deps'], docs=True)
    run('native-cli', cargo('build', 'hyperdrc', '--all-features', '--message-format=json'))
dump(O / 'result.json', {'complete': True, 'command_failures': [r['name'] for r in rows if r['exit_code']],
    'sources_unchanged': True, 'cargo_target_closed': True, 'native_runtime_admitted': False, 'full_plan_complete': False,
    'main_sources_modified': False, 'original_workload_gates_unchanged': True, 'full_pre_push_matrix_executed': False,
    'published_package_pipeline_executed': False})
print(json.dumps({'complete': True, 'failures': [r['name'] for r in rows if r['exit_code']]}), flush=True)
sys.exit(101 if any(r['exit_code'] for r in rows) else 0)
