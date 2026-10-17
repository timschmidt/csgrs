"""Complete the unchanged tests with Cargo's full package metadata environment."""
import gzip
import hashlib
import json
import os
import resource
import subprocess
import sys
import time
import tomllib
from pathlib import Path

W = Path(__file__).resolve().parent
OLD = W / 'temp-validation-v3'
O = W / 'temp-replay-v4'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, v):
    Path(p).write_text(json.dumps(v, indent=2) + '\n')

prior = read(OLD / 'result.json')
assert prior['complete'] and prior['cargo_target_closed'] and prior['command_failures'] == ['compile-tests']
assert 'CARGO_PKG_AUTHORS' in (OLD / 'compile-tests.stderr').read_text()
protocol = read(OLD / 'protocol.json')
S = Path(protocol['source'])
with gzip.open(W / 'temp-preparation-v1/source-hashes.json.gz', 'rt') as f:
    inputs = json.load(f)
assert all(sha(S / p) == h for p, h in inputs.items())
assert all(sha(p) == h for p, h in protocol['read_only_dependency_sha256'].items())
original = next(r for r in read(OLD / 'commands.json') if r['name'] == 'compile-tests')
O.mkdir(exist_ok=False)
scratch = O / 'compiler-scratch'
scratch.mkdir()
test_tmp = O / 'selected-temporary-directory'
test_tmp.mkdir()
control = test_tmp / 'write-control'
control.write_bytes(b'Selected temporary storage remains writable.\n')
package = tomllib.loads((S / 'crates/hyperdrc/Cargo.toml').read_text())['package']
env = os.environ.copy()
for k in ['CARGO_TARGET_DIR', 'RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER', 'RUSTDOCFLAGS']:
    env.pop(k, None)
env.update({
    'TMPDIR': str(scratch), 'RUST_BACKTRACE': '0', 'CARGO_MANIFEST_DIR': str(S / 'crates/hyperdrc'),
    'CARGO_MANIFEST_PATH': str(S / 'crates/hyperdrc/Cargo.toml'), 'CARGO_PKG_NAME': package['name'],
    'CARGO_CRATE_NAME': 'hyperdrc', 'CARGO_PKG_VERSION': package['version'],
    'CARGO_PKG_AUTHORS': ':'.join(package.get('authors', [])),
    **{'CARGO_PKG_' + name.upper(): package.get(name.replace('_', '-'), '') for name in ['description', 'homepage', 'repository', 'license', 'license_file', 'rust_version']},
})
major, minor, patch = package['version'].split('.')
env.update({'CARGO_PKG_VERSION_MAJOR': major, 'CARGO_PKG_VERSION_MINOR': minor, 'CARGO_PKG_VERSION_PATCH': patch, 'CARGO_PKG_VERSION_PRE': ''})
pins = {str(p): sha(p) for p in [Path(__file__), OLD / 'protocol.json', OLD / 'commands.json', OLD / 'result.json', W / 'temp-preparation-v1/source-hashes.json.gz', control]}
dump(O / 'protocol.json', {
    'input_sha256': pins, 'source': str(S), 'reused_dependency_protocol': str(OLD / 'protocol.json'),
    'environment': {k: v for k, v in env.items() if k.startswith('CARGO_PKG_') or k in ['CARGO_MANIFEST_DIR', 'CARGO_MANIFEST_PATH', 'CARGO_CRATE_NAME', 'TMPDIR']},
    'correction': 'Supply Cargo package metadata, including empty AUTHORS for this manifest, to the CLI derive macro in the direct test build.',
    'source_changes_since_passed_format_and_clippy': False, 'completed_cargo_targets_reopened': False,
    'external_timeout_seconds': 1800, 'address_space_bytes': 8 * 1024**3,
    'cache_index_included': False,
})
rows = []

def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

def run(name, command, extra=None):
    print('START ' + name, flush=True)
    assert os.statvfs(W).f_bavail * os.statvfs(W).f_frsize > 768 * 1024**2
    start = time.monotonic()
    with (O / (name + '.stdout')).open('wb') as out, (O / (name + '.stderr')).open('wb') as err:
        result = subprocess.run(['/usr/bin/time', '-v', '-o', str(O / (name + '.resources')), 'timeout', '--kill-after=20s', '1800s', *command],
                                cwd=S, env=env | (extra or {}), stdout=out, stderr=err, preexec_fn=limits, start_new_session=True)
    assert all(sha(S / p) == h for p, h in inputs.items())
    row = {'name': name, 'command': command, 'environment_overrides': extra or {}, 'exit_code': result.returncode,
           'elapsed_seconds': time.monotonic() - start,
           **{s + '_sha256': sha(O / (name + '.' + s)) for s in ['stdout', 'stderr', 'resources']}}
    rows.append(row)
    dump(O / 'commands.json', rows)
    print(json.dumps({k: row[k] for k in ['name', 'exit_code', 'elapsed_seconds']}), flush=True)
    return result.returncode

binary = O / 'hyperdrc-tests'
command = original['command'].copy()
command[command.index('-o') + 1] = str(binary)
rc = run('compile-tests', command)
if rc == 0:
    dump(O / 'binary.json', {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size})
    selected = {'TMPDIR': str(test_tmp)}
    run('conversion-tests', [str(binary), 'conversion::tests::', '--nocapture', '--test-threads=4'], selected)
    run('parquet-test', [str(binary), 'parquet_report::tests::writes_readable_parquet_report', '--exact', '--nocapture', '--test-threads=1'], selected)
    run('sqlite-test', [str(binary), 'sqlite_report::tests::writes_queryable_sqlite_report', '--exact', '--nocapture', '--test-threads=1'], selected)
assert all(sha(p) == h for p, h in pins.items())
assert all(sha(p) == h for p, h in protocol['read_only_dependency_sha256'].items())
dump(O / 'result.json', {
    'complete': True, 'command_failures': [r['name'] for r in rows if r['exit_code']],
    'selected_temporary_directory_cleaned': list(test_tmp.iterdir()) == [control],
    'source_and_read_only_dependencies_unchanged': True, 'clippy_and_format_reused_from_identical_source': str(OLD / 'result.json'),
    'cache_index_included': False, 'production_prefixes_unchanged': True, 'source_changes_applied': False,
    'native_runtime_admitted': False, 'full_plan_complete': False,
})
assert list(test_tmp.iterdir()) == [control], list(test_tmp.iterdir())
sys.exit(101 if any(r['exit_code'] for r in rows) else 0)
