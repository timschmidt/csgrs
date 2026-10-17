"""Validate test-only storage repair with frozen main dependencies and fresh lint target."""
import gzip
import hashlib
import json
import os
import resource
import signal
import subprocess
import sys
import time
import tomllib
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
S = W / 'temp-candidate-source-v1'
O = W / 'temp-validation-v1'
T = W / 'temp-lint-target-v1'
OLD = W.parent / 'native-stroke-completion'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, v):
    Path(p).write_text(json.dumps(v, indent=2) + '\n')

prep = read(W / 'temp-preparation-v1/result.json')
assert prep['complete'] and not prep['cache_index_included']
manifest = W / 'temp-preparation-v1/source-hashes.json.gz'
assert sha(manifest) == prep['source_hashes_sha256']
with gzip.open(manifest, 'rt') as f:
    inputs = json.load(f)
with gzip.open(W / 'temp-preparation-v1/main-input-hashes.json.gz', 'rt') as f:
    main = json.load(f)
e37 = read(ROOT / 'docs/evidence/2026-10-17-native-track-cost-attribution/stage-observations-v1/protocol.json')
old_inputs = e37['sources']['baseline']['inputs']
differences = {p: {'baseline': h, 'main': main[p]} for p, h in old_inputs.items() if main[p] != h}
admitted = read(ROOT / 'docs/evidence/2026-10-17-native-stroke-completion/checkpoint-v1.json')['implementation_sha256']
fixtures = {p for p in admitted if p.startswith('crates/hypercircuit/tests/fixtures/')}
assert len(fixtures) == 7 and set(differences) == fixtures | {'docs/properties-status.json'}
assert all(differences[p]['main'] == admitted[p] for p in fixtures)
assert all(sha(S / p) == h for p, h in inputs.items())
build = OLD / 'consumer-validation-v1/native-cli.stdout'
test_build = OLD / 'consumer-validation-v1/circuit-build.stdout'
artifacts = [json.loads(s) for s in build.read_text().splitlines() if s.startswith('{')]
test_artifacts = [json.loads(s) for s in test_build.read_text().splitlines() if s.startswith('{')]
package = tomllib.loads((S / 'crates/hyperdrc/Cargo.toml').read_text())
externs = {}
for dependency in [*package['dependencies'], *package['dev-dependencies']]:
    name = dependency.replace('-', '_')
    records = test_artifacts if dependency == 'proptest' else artifacts
    paths = {p for a in records if a.get('reason') == 'compiler-artifact' and a['target']['name'] == name
             for p in a['filenames'] if p.endswith(('.rlib', '.so'))}
    assert len(paths) == 1, (name, paths)
    externs[name] = paths.pop()
deps = OLD / 'consumer-target-v1/release/deps'
dependency_pins = {str(p): sha(p) for p in sorted(deps.iterdir()) if p.is_file() and p.suffix in ['.rlib', '.rmeta', '.so']}
assert all(str(p) in dependency_pins for p in externs.values())
assert all(sha(p) == h for p, h in e37['read_only_sha256'].items() if str(deps) in p)
for p in [O, T]:
    p.mkdir(exist_ok=False)
scratch = O / 'scratch'
scratch.mkdir()
test_tmp = O / 'selected-temporary-directory'
test_tmp.mkdir()
control = test_tmp / 'write-control'
control.write_bytes(b'Explicit test storage is writable.\n')
env = os.environ.copy()
for k in ['RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER', 'RUSTDOCFLAGS']:
    env.pop(k, None)
env.update({'CARGO_HOME': '/home/tim/.cargo', 'CARGO_TARGET_DIR': str(T), 'CARGO_BUILD_JOBS': '2', 'CARGO_NET_OFFLINE': 'true',
            'CARGO_INCREMENTAL': '0', 'CARGO_TERM_COLOR': 'never', 'TMPDIR': str(scratch), 'RUST_BACKTRACE': '0',
            'CARGO_MANIFEST_DIR': str(S / 'crates/hyperdrc'), 'CARGO_PKG_VERSION': package['package']['version'],
            'CARGO_PKG_NAME': package['package']['name']})
pins = {str(p): sha(p) for p in [Path(__file__), manifest, W / 'temp-preparation-v1/result.json', build, test_build, control]}
dump(O / 'protocol.json', {
    'input_sha256': pins, 'source': str(S), 'fresh_clippy_target': str(T), 'target_initially_empty': not list(T.iterdir()),
    'source_equivalence_to_main_dependencies': differences, 'read_only_dependency_sha256': dependency_pins, 'externs': externs,
    'rustc': subprocess.check_output(['rustc', '-vV'], text=True), 'cargo': subprocess.check_output(['cargo', '-V'], text=True),
    'hardware': subprocess.check_output(['lscpu'], text=True), 'environment': {k: env[k] for k in ['CARGO_HOME', 'CARGO_TARGET_DIR', 'CARGO_BUILD_JOBS', 'CARGO_NET_OFFLINE', 'CARGO_INCREMENTAL', 'TMPDIR', 'CARGO_MANIFEST_DIR', 'CARGO_PKG_VERSION']},
    'external_timeout_seconds': 1800, 'address_space_bytes': 8 * 1024**3,
    'minimum_repository_free_bytes': 768 * 1024**2, 'space_check_interval_seconds': 5,
    'containment': 'External limits and repository free-space reserve, not a production resource bound.',
    'completed_cargo_targets_reopened': False, 'cache_index_included': False,
})
rows = []

def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

def run(name, command, extra=None):
    print('START ' + name, flush=True)
    start = time.monotonic()
    reason = None
    free = []
    with (O / (name + '.stdout')).open('wb') as out, (O / (name + '.stderr')).open('wb') as err:
        proc = subprocess.Popen(['/usr/bin/time', '-v', '-o', str(O / (name + '.resources')), *command], cwd=S,
                                env=env | (extra or {}), stdout=out, stderr=err, preexec_fn=limits, start_new_session=True)
        while proc.poll() is None:
            fs = os.statvfs(W)
            available = fs.f_bavail * fs.f_frsize
            free.append({'elapsed_seconds': time.monotonic() - start, 'available_bytes': available})
            if available < 768 * 1024**2 or time.monotonic() - start > 1800:
                reason = 'repository-free-space-reserve' if available < 768 * 1024**2 else 'external-timeout'
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                break
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
        code = proc.wait()
    assert all(sha(S / p) == h for p, h in inputs.items())
    assert all(sha(p) == h for p, h in pins.items())
    row = {'name': name, 'command': command, 'environment_overrides': extra or {}, 'exit_code': code,
           'elapsed_seconds': time.monotonic() - start, 'external_stop_reason': reason, 'free_space_observations': free,
           **{s + '_sha256': sha(O / (name + '.' + s)) for s in ['stdout', 'stderr', 'resources']}}
    rows.append(row)
    dump(O / 'commands.json', rows)
    print(json.dumps({k: row[k] for k in ['name', 'exit_code', 'elapsed_seconds', 'external_stop_reason']}), flush=True)
    return code

run('format', ['cargo', 'fmt', '--all', '--', '--check'])
binary = O / 'hyperdrc-tests'
command = ['rustc', '--crate-name', 'hyperdrc', '--edition=2024', '--test', str(S / 'crates/hyperdrc/src/lib.rs'),
           '-C', 'opt-level=3', '-C', 'codegen-units=16', '-C', 'embed-bitcode=no', '-D', 'warnings',
           '-L', 'dependency=' + str(deps), '-o', str(binary)]
for name, path in externs.items():
    command += ['--extern', name + '=' + path]
for path in sorted({p for a in artifacts if a.get('reason') == 'build-script-executed' for p in a.get('linked_paths', [])}):
    command += ['-L', path]
rc = run('compile-tests', command)
if rc == 0:
    dump(O / 'binary.json', {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size})
    selected = {'TMPDIR': str(test_tmp)}
    run('conversion-tests', [str(binary), 'conversion::tests::', '--nocapture', '--test-threads=4'], selected)
    run('parquet-test', [str(binary), 'parquet_report::tests::writes_readable_parquet_report', '--exact', '--nocapture', '--test-threads=1'], selected)
    run('sqlite-test', [str(binary), 'sqlite_report::tests::writes_queryable_sqlite_report', '--exact', '--nocapture', '--test-threads=1'], selected)
    assert list(test_tmp.iterdir()) == [control], list(test_tmp.iterdir())
run('clippy', ['cargo', 'clippy', '--offline', '--locked', '-p', 'hyperdrc', '--release', '--all-features', '--all-targets', '--', '-D', 'warnings'])
assert all(sha(p) == h for p, h in dependency_pins.items())
dump(O / 'result.json', {
    'complete': True, 'command_failures': [r['name'] for r in rows if r['exit_code']],
    'source_and_read_only_dependencies_unchanged': True, 'cargo_target_closed': True,
    'selected_temporary_directory_cleaned': list(test_tmp.iterdir()) == [control],
    'cache_index_included': False, 'production_prefixes_unchanged': True, 'source_changes_applied': False,
    'native_runtime_admitted': False, 'full_plan_complete': False,
})
sys.exit(101 if any(r['exit_code'] for r in rows) else 0)
