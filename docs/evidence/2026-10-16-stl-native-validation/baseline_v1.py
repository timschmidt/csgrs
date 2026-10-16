"""Reproduce STL failures using qualified, read-only current-revision libraries."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import resource
import subprocess
import time

ROOT = Path('/home/tim/Documents/GitHub/workspace/csgrs')
E = ROOT / 'docs/evidence/2026-10-16-stl-native-validation'
E24 = ROOT / 'docs/evidence/2026-10-16-physical-build-requalification'
E25 = ROOT / 'docs/evidence/2026-10-16-obj-index-validation'
WORK = ROOT / 'target/properties/stl-native-validation'
OUT = WORK / 'baseline-v1'
OLD = Path('/dev/shm/csgrs-obj-index-contracts')
DEPS = OLD / 'candidate-target-v1/release/deps'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


OUT.mkdir(exist_ok=False)
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == '7a1f0c7e1f4f69499feb3b471f739dfb50f4776c'
with gzip.open(E25 / 'candidate-validation-v1/source-hashes.json.gz', 'rt') as stream:
    inputs = json.load(stream)
for name, digest in inputs.items():
    if name != 'docs/properties-status.json':
        assert sha(ROOT / name) == digest, name
inputs['docs/properties-status.json'] = sha(ROOT / 'docs/properties-status.json')
with gzip.open(OUT / 'source-hashes.json.gz', 'wt') as stream:
    json.dump(inputs, stream, sort_keys=True)
bindings = json.loads((E25 / 'candidate-validation-v1/probe-bindings.json').read_text())['candidate']
for record in bindings.values():
    assert sha(record['retained']) == record['sha256']
libraries = {str(p): sha(p) for p in sorted(DEPS.iterdir()) if p.suffix in {'.rlib', '.rmeta', '.so'} and p.is_file()}
dump(OUT / 'dependency-hashes.json', libraries)
probes = {
    'format': E24 / 'format_probe_v1.rs',
    'topology': E24 / 'stl-topology-v1/probe_v1.rs',
    'acceptance': E / 'stl_import_contracts_v1.rs',
}
protocol = {
    'runner_sha256': sha(__file__), 'head': '7a1f0c7e1f4f69499feb3b471f739dfb50f4776c',
    'source_hashes_sha256': sha(OUT / 'source-hashes.json.gz'),
    'bindings': bindings, 'dependencies': str(DEPS),
    'dependency_hashes_sha256': sha(OUT / 'dependency-hashes.json'),
    'probes': {name: {'path': str(path), 'sha256': sha(path)} for name, path in probes.items()},
    'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
    'elapsed_limit_seconds_per_command': 120,
    'address_space_limit_bytes_per_process': 8 * 1024**3,
    'target_reused_read_only': True, 'completed_targets_modified': False,
}
dump(OUT / 'protocol.json', protocol)


def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


rows = []
environment = os.environ | {'TMPDIR': str(OUT), 'RUST_BACKTRACE': '0'}
for name, path in probes.items():
    binary = OUT / (name + '-probe')
    command = ['rustc', '--edition=2024', '--test', '-C', 'opt-level=3', '-C', 'debug-assertions=off',
               '--cfg', 'feature="stl-io"', '-L', 'dependency=' + str(DEPS)]
    for crate, binding in bindings.items():
        command += ['--extern', crate + '=' + binding['retained']]
    command += [str(path), '-o', str(binary)]
    for stage, argv in [('compile', command), ('test', [str(binary), '--test-threads=1', '--nocapture'])]:
        label = name + '-' + stage
        start = time.monotonic()
        with (OUT / (label + '.stdout')).open('wb') as stdout, (OUT / (label + '.stderr')).open('wb') as stderr:
            result = subprocess.run(['/usr/bin/time', '-v', '-o', str(OUT / (label + '.resources')),
                                     'timeout', '--signal=TERM', '--kill-after=10s', '120s', *argv],
                                    cwd=ROOT, env=environment, stdout=stdout, stderr=stderr,
                                    preexec_fn=limits, start_new_session=True)
        row = {'name': label, 'command': argv, 'exit_code': result.returncode,
               'elapsed_seconds': time.monotonic() - start,
               **{s + '_sha256': sha(OUT / (label + '.' + s)) for s in ['stdout', 'stderr', 'resources']}}
        if binary.exists():
            row['binary_sha256'] = sha(binary)
        rows.append(row)
        dump(OUT / 'commands.json', rows)
        print(json.dumps({k: row[k] for k in ['name', 'exit_code', 'elapsed_seconds']}), flush=True)
        if stage == 'compile' and result.returncode:
            break
assert all(sha(ROOT / name) == digest for name, digest in inputs.items())
assert all(sha(path) == digest for path, digest in libraries.items())
assert all(sha(row['retained']) == row['sha256'] for row in bindings.values())
assert all(sha(row['path']) == row['sha256'] for row in protocol['probes'].values())
assert sha(__file__) == protocol['runner_sha256']
dump(OUT / 'result.json', {'complete': True, 'commands': len(rows),
                         'compile_failures': [row['name'] for row in rows if row['name'].endswith('-compile') and row['exit_code']],
                         'sources_and_all_observed_libraries_unchanged': True,
                         'native_runtime_admitted': False, 'full_plan_complete': False})
