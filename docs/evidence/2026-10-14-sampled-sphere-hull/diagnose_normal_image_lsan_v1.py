"""Run one frozen phase of a standalone sanitizer environment comparison."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'normal-image-lsan-environment-v1'
WORK = ROOT / 'target/normal-image-lsan-environment-v1'
MODES = ['single', 'threads', 'leak']
ENV = {
    'RUSTC_BOOTSTRAP': '1',
    'ASAN_OPTIONS': 'detect_leaks=1:halt_on_error=1:abort_on_error=0:exitcode=66',
    'LSAN_OPTIONS': 'verbosity=1:log_threads=1',
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def guard(binary=False):
    protocol = read(OUT / 'protocol.json')
    assert all(sha(E / name) == digest for name, digest in protocol['inputs_sha256'].items())
    roots = json.loads(gzip.decompress((OUT / 'root-source-hashes.json.gz').read_bytes()))
    assert all(sha(ROOT / name) == digest for name, digest in roots.items())
    prior = E / 'sphere-normal-image-v1'
    source = Path(read(prior / 'protocol.json')['source'])
    mapping = json.loads(gzip.decompress((prior / 'source-hashes.json.gz').read_bytes()))
    assert all(sha(source / name) == digest for name, digest in mapping.items())
    failure = read(E / 'normal-image-asan-closure-v1.json')
    for row in [failure['executed_control_binary'], failure['executed_test_binary']]:
        assert sha(row['path']) == row['sha256']
    if binary:
        row = read(OUT / 'binary.json')
        assert sha(row['path']) == row['sha256']
        assert Path(row['path']).stat().st_size == row['bytes']


def run(name, command, seconds, permitted):
    guard()
    assert not (OUT / (name + '.json')).exists()
    print(json.dumps({'start': name}), flush=True)
    started = time.monotonic()
    with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
        code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=5s', str(seconds), *command],
                              cwd=ROOT, env=os.environ | ENV, stdout=stdout, stderr=stderr).returncode
    row = {'name': name, 'command': command, 'exit_code': code,
           'process_wall_seconds': time.monotonic() - started,
           'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
    dump(OUT / (name + '.json'), row)
    print(json.dumps(row), flush=True)
    guard()
    if code not in permitted:
        dump(OUT / (name + '-unexpected-exit.json'), {'complete': False, 'command': row})
        raise SystemExit(1)
    return row


def prepare():
    assert not OUT.exists() and not WORK.exists()
    closure = read(E / 'normal-image-asan-closure-v1.json')
    assert closure['trial_closed'] and not closure['complete']
    assert closure['failed_command_exit_code'] == 66
    inputs = {name: sha(E / name) for name in [
        'diagnose_normal_image_lsan_v1.py', 'normal_image_lsan_control_v1.rs',
        'normal-image-lsan-environment-scope-v1.json', 'normal-image-asan-closure-v1.json',
        'sphere-normal-image-v1/protocol.json', 'sphere-normal-image-v1/source-hashes.json.gz',
    ]}
    for name, digest in closure['artifacts_sha256'].items():
        assert sha(E / 'normal-image-asan-v1' / name) == digest
        inputs['normal-image-asan-v1/' + name] = digest
    baseline = json.loads(gzip.decompress((E / 'mesh-identity-preparation-v1/source-hashes.json.gz').read_bytes()))
    roots = {name: sha(ROOT / name) for name in baseline}
    assert all(roots[name] == digest for name, digest in baseline.items() if name != 'docs/properties-status.json')
    compiler = subprocess.check_output(['rustc', '-Vv'], text=True)
    assert compiler == read(E / 'normal-image-asan-v1/protocol.json')['compiler']
    OUT.mkdir()
    WORK.mkdir()
    (OUT / 'root-source-hashes.json.gz').write_bytes(gzip.compress(
        (json.dumps(roots, sort_keys=True) + '\n').encode(), mtime=0))
    dump(OUT / 'protocol.json', {'compiler': compiler, 'inputs_sha256': inputs, 'environment': ENV,
        'source_files': 10136, 'root_source_files': len(roots),
        'contexts': ['sandbox', 'unrestricted'], 'modes': MODES,
        'native_executions': 6, 'native_wall_limit_seconds': 30,
        'runtime_admitted': False, 'plan_complete': False})
    source = E / 'normal_image_lsan_control_v1.rs'
    run('format', ['rustfmt', '--edition', '2024', '--check', str(source)], 30, {0})
    binary = WORK / 'lsan-control'
    run('build', ['rustc', '--edition', '2024', '--target', 'x86_64-unknown-linux-gnu',
                  '-Zsanitizer=address', '-Cforce-frame-pointers=yes', '-Copt-level=1',
                  '-Cdebuginfo=1', str(source), '-o', str(binary)], 180, {0})
    dump(OUT / 'binary.json', {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size})
    guard(binary=True)


def context(name):
    assert name in ['sandbox', 'unrestricted']
    if name == 'unrestricted':
        assert (OUT / 'sandbox-results.json').exists()
    assert not (OUT / (name + '-results.json')).exists()
    guard(binary=True)
    binary = read(OUT / 'binary.json')['path']
    results = []
    for mode in MODES:
        guard(binary=True)
        row = run(name + '-' + mode, [binary, mode], 30, {0, 66})
        stdout = (OUT / (row['name'] + '.stdout')).read_text()
        stderr = (OUT / (row['name'] + '.stderr')).read_text()
        assert stdout == f'lsan_control={mode} completed\n'
        results.append(row | {
            'mode': mode,
            'fatal_leak_scanner_error': 'LeakSanitizer has encountered a fatal error.' in stderr,
            'leak_report': 'ERROR: LeakSanitizer: detected memory leaks' in stderr,
            'address_error': 'ERROR: AddressSanitizer:' in stderr,
        })
    guard(binary=True)
    dump(OUT / (name + '-results.json'), results)
    print(json.dumps({'context_complete': name, 'codes': [row['exit_code'] for row in results]}), flush=True)


def assess():
    assert not (OUT / 'result.json').exists()
    guard(binary=True)
    results = {name: read(OUT / (name + '-results.json')) for name in ['sandbox', 'unrestricted']}
    for name, rows in results.items():
        assert [row['mode'] for row in rows] == MODES
        for row in rows:
            for stream in ['stdout', 'stderr']:
                assert sha(OUT / (row['name'] + '.' + stream)) == row[stream + '_sha256']
    restricted = all(row['exit_code'] == 66 and row['fatal_leak_scanner_error'] for row in results['sandbox'])
    freed = all(row['exit_code'] == 0 and not row['fatal_leak_scanner_error'] and not row['leak_report']
                and not row['address_error'] for row in results['unrestricted'][:2])
    leak = results['unrestricted'][2]
    leak_detected = leak['exit_code'] == 66 and leak['leak_report'] and not leak['fatal_leak_scanner_error']
    assert not leak['address_error']
    dump(OUT / 'result.json', {
        'complete': True, 'standalone_control_executions': 6, 'results': results,
        'sandbox_scanning_failure_reproduced': restricted,
        'unrestricted_cleanup_controls_passed': freed,
        'unrestricted_deliberate_leak_detected': leak_detected,
        'sandbox_associated_restriction_demonstrated': restricted and freed and leak_detected,
        'native_library_qualification_reexecuted': False,
        'artifacts_sha256': {path.name: sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'limits': read(E / 'normal-image-lsan-environment-scope-v1.json')['limits'],
        'runtime_admitted': False, 'plan_complete': False,
    })
    print(json.dumps({'complete': True, 'sandbox_failure': restricted,
                     'unrestricted_cleanup': freed, 'unrestricted_leak_detected': leak_detected,
                     'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    assert len(sys.argv) == 2
    phase = sys.argv[1]
    if phase == 'prepare':
        prepare()
    elif phase in ['sandbox', 'unrestricted']:
        context(phase)
    else:
        assert phase == 'assess'
        assess()
