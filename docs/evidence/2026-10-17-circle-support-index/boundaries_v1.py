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
O = W / 'boundary-validation-v1'
T = Path('/tmp/csgrs-circle-support-boundaries-v1')

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

def run(name, command, docs=False, extra=None, wasm=False):
    assert all(sha(S / p) == h for p, h in inputs.items())
    print('START ' + name, flush=True)
    start = time.monotonic()
    command_env = env | ({'RUSTDOCFLAGS': '-D warnings'} if docs else {}) | (extra or {})
    with (O / (name + '.stdout')).open('wb') as out, (O / (name + '.stderr')).open('wb') as err:
        r = subprocess.run(['/usr/bin/time', '-v', '-o', str(O / (name + '.resources')),
                            'timeout', '--signal=TERM', '--kill-after=20s', '1800s', *command],
                           cwd=S, env=command_env, stdout=out, stderr=err, preexec_fn=wasm_limits if wasm else limits, start_new_session=True)
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
           'binaries': binaries, 'rustdocflags': command_env.get('RUSTDOCFLAGS'), 'environment_overrides': extra or {}, 'address_space_bytes': (64 if wasm else 8) * 1024**3,
           **{suffix + '_sha256': sha(O / (name + '.' + suffix)) for suffix in ['stdout', 'stderr', 'resources']}}
    rows.append(row)
    dump(O / 'commands.json', rows)
    print(json.dumps({k: row[k] for k in ['name', 'exit_code', 'elapsed_seconds']}), flush=True)
    return r.returncode

def cargo(action, package, *args):
    return ['cargo', action, '--offline', '--locked', '-p', package, '--release', *args]


def wasm_limits():
    resource.setrlimit(resource.RLIMIT_AS, (64 * 1024**3, 64 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

old = W.parent / 'export-projection-contracts'
tool = read(old / 'wasm-tools-v1/result.json')
assert tool['exit_code'] == 0 and tool['version'] == 'wasm-bindgen 0.2.126'
assert sha(tool['binary']) == tool['binary_sha256']
typescript = old / 'wasm-source-v1/tests/wasm/node_modules/typescript'
assert read(typescript / 'package.json')['version'] == '5.7.3'
tools = {str(p): sha(p) for p in typescript.rglob('*') if p.is_file()}
tools[tool['binary']] = tool['binary_sha256']
dump(O / 'tools.json', {'read_only_sha256': tools, 'wasm_limits': '64 GiB address space for V8 reservations; not a heap bound',
                       'node': subprocess.check_output(['node', '--version'], text=True),
                       'tsc': str(typescript / 'bin/tsc'), 'wasm_bindgen': tool['binary']})

run('format', ['cargo', 'fmt', '--all', '--', '--check'])
run('scalar-value-hash', cargo('test', 'hyperreal', 'unreduced_arithmetic_preserves_canonical_queries_and_hashes', '--', '--nocapture', '--test-threads=1'))
run('ffi-build', cargo('build', 'csgrs-ffi', '--message-format=json'))
run('ffi-tests', cargo('test', 'csgrs-ffi', '--no-fail-fast', '--', '--test-threads=1'))
run('ffi-clippy', cargo('clippy', 'csgrs-ffi', '--all-targets', '--', '-D', 'warnings'))
library = T / 'release/libcsgrs_ffi.so'
assert library.is_file()
shared = {'CSGRS_LIBRARY': str(library), 'PYTHONDONTWRITEBYTECODE': '1'}
for name in ['empty_composition', 'curve_constructor_failures', 'planar_operation_failures', 'exact_linear_modeling']:
    for language, cc, standard in [('c', 'cc', '-std=c11'), ('cpp', 'c++', '-std=c++17')]:
        source = S / 'tests/ffi' / (name + '.' + language)
        if not source.exists():
            continue
        binary = O / (name + '-' + language)
        run(name + '-' + language + '-build', [cc, standard, '-Wall', '-Wextra', '-Werror', '-I', str(S / 'crates/csgrs-ffi/include'),
             '-I', str(S / 'bindings/cpp'), str(source), '-L', str(T / 'release'), '-lcsgrs_ffi',
             '-Wl,-rpath,' + str(T / 'release'), '-o', str(binary)])
        run(name + '-' + language, [str(binary)])
    source = S / 'tests/ffi' / (name + '.py')
    if source.exists():
        run(name + '-python', ['python3', str(source)], extra=shared)
run('wasm-shell-syntax', ['bash', '-n', str(S / 'scripts/test-wasm-bindings.sh')])
run('wasm-consumers', ['bash', str(S / 'scripts/test-wasm-bindings.sh')],
    extra={'WASM_BINDGEN': tool['binary'], 'TSC': str(typescript / 'bin/tsc')}, wasm=True)
run('wasm-clippy', ['cargo', 'clippy', '--offline', '--locked', '--profile', 'wasm', '--target', 'wasm32-unknown-unknown',
    '--no-default-features', '--features', 'wasm', '--lib', '--', '-D', 'warnings'], wasm=True)
assert all(sha(p) == h for p, h in tools.items())
artifacts = [library, T / 'wasm32-unknown-unknown/wasm/csgrs.wasm', *sorted((T / 'wasm-bindings-node').glob('*'))]
dump(O / 'result.json', {'complete': True, 'command_failures': [r['name'] for r in rows if r['exit_code']],
    'sources_unchanged': True, 'cargo_target_closed': True, 'native_runtime_admitted': False, 'full_plan_complete': False,
    'main_sources_modified': False, 'full_pre_push_matrix_executed': False, 'published_package_pipeline_executed': False,
    'read_only_tools_unchanged': True, 'artifacts': {str(p): {'sha256': sha(p), 'bytes': p.stat().st_size} for p in artifacts if p.is_file()}})
print(json.dumps({'complete': True, 'failures': [r['name'] for r in rows if r['exit_code']]}), flush=True)
sys.exit(101 if any(r['exit_code'] for r in rows) else 0)
