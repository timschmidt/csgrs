"""Prepare, then independently run two frozen MPFR cache-lifecycle controls."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'normal-image-mpfr-cache-v1'
WORK = ROOT / 'target/normal-image-mpfr-cache-v1'
RELEASE = ROOT / 'target/normal-image-asan-v2/target/x86_64-unknown-linux-gnu/release'
ENV = {'RUSTC_BOOTSTRAP': '1',
       'ASAN_OPTIONS': 'detect_leaks=1:halt_on_error=1:abort_on_error=0:exitcode=66',
       'LSAN_OPTIONS': ''}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def guard(binary=False):
    protocol = read(OUT / 'protocol.json')
    assert all(sha(E / name) == digest for name, digest in protocol['inputs_sha256'].items())
    assert all(sha(row['path']) == row['sha256'] for row in protocol['libraries'])
    roots = json.loads(gzip.decompress((OUT / 'root-source-hashes.json.gz').read_bytes()))
    assert all(sha(ROOT / name) == digest for name, digest in roots.items())
    prior = E / 'sphere-normal-image-v1'
    source = Path(read(prior / 'protocol.json')['source'])
    mapping = json.loads(gzip.decompress((prior / 'source-hashes.json.gz').read_bytes()))
    assert all(sha(source / name) == digest for name, digest in mapping.items())
    for row in protocol['preserved_executed_binaries']:
        assert sha(row['path']) == row['sha256']
    if binary:
        row = read(OUT / 'binary.json')
        assert sha(row['path']) == row['sha256']


def run(name, command, seconds, expected):
    guard()
    assert not (OUT / (name + '.json')).exists()
    print(json.dumps({'start': name, 'expected_exit_code': expected}), flush=True)
    started = time.monotonic()
    with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
        code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=5s', str(seconds), *command],
                              cwd=ROOT, env=os.environ | ENV, stdout=stdout, stderr=stderr).returncode
    row = {'name': name, 'command': command, 'exit_code': code, 'expected_exit_code': expected,
           'process_wall_seconds': time.monotonic() - started,
           'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
    dump(OUT / (name + '.json'), row)
    print(json.dumps(row), flush=True)
    guard()
    if code != expected:
        dump(OUT / 'failure.json', {'complete': False, 'failed_command': row})
        raise SystemExit(1)


def prepare():
    assert not OUT.exists() and not WORK.exists()
    failed = read(E / 'normal-image-asan-closure-v2.json')
    assert failed['trial_closed'] and not failed['complete']
    assert failed['leak_bytes'] == 163672 and failed['leak_allocations'] == 560
    inputs = {name: sha(E / name) for name in [
        'probe_normal_image_mpfr_cache_v1.py', 'normal_image_mpfr_cache_probe_v1.rs',
        'normal-image-mpfr-cache-scope-v1.json', 'normal-image-asan-closure-v2.json',
        'sphere-normal-image-v1/protocol.json', 'sphere-normal-image-v1/source-hashes.json.gz',
    ]}
    for name, digest in failed['artifacts_sha256'].items():
        assert sha(E / 'normal-image-asan-v2' / name) == digest
        inputs['normal-image-asan-v2/' + name] = digest
    compiler = subprocess.check_output(['rustc', '-Vv'], text=True)
    assert compiler == read(E / 'normal-image-asan-v2/protocol.json')['compiler']
    rug = list((RELEASE / 'deps').glob('librug-*.rlib'))
    outputs = list(RELEASE.glob('build/gmp-mpfr-sys-*/output'))
    assert len(rug) == len(outputs) == 1
    searches = re.findall(r'^cargo:rustc-link-search=native=(.+)$', outputs[0].read_text(), re.MULTILINE)
    assert len(searches) == 1
    native = Path(searches[0])
    assert {path.name for path in native.glob('*.a')} == {'libgmp.a', 'libmpfr.a'}
    libraries = sorted((RELEASE / 'deps').glob('*.rlib')) + sorted(native.glob('*.a')) + outputs
    records = [{'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size} for path in libraries]
    baseline = json.loads(gzip.decompress((E / 'mesh-identity-preparation-v1/source-hashes.json.gz').read_bytes()))
    roots = {name: sha(ROOT / name) for name in baseline}
    assert all(roots[name] == digest for name, digest in baseline.items() if name != 'docs/properties-status.json')
    OUT.mkdir()
    WORK.mkdir()
    (OUT / 'root-source-hashes.json.gz').write_bytes(gzip.compress(
        (json.dumps(roots, sort_keys=True) + '\n').encode(), mtime=0))
    dump(OUT / 'protocol.json', {
        'inputs_sha256': inputs, 'compiler': compiler, 'environment': ENV,
        'libraries': records, 'rug_rlib': str(rug[0]), 'native_search_path': str(native),
        'preserved_executed_binaries': [failed['executed_control_binary'], *failed['executed_test_binaries']],
        'source_files': 10136, 'root_source_files': len(roots),
        'commands': 4, 'expected_exit_codes': [0, 0, 66, 0],
        'runtime_admitted': False, 'plan_complete': False,
    })
    source = E / 'normal_image_mpfr_cache_probe_v1.rs'
    run('format', ['rustfmt', '--edition', '2024', '--check', str(source)], 30, 0)
    binary = WORK / 'mpfr-cache-probe'
    run('build', ['rustc', '--edition', '2024', '--target', 'x86_64-unknown-linux-gnu',
                  '-Zsanitizer=address', '-Cforce-frame-pointers=yes', '-Copt-level=1',
                  '-Cdebuginfo=1', '-Dwarnings', str(source), '--extern', 'rug=' + str(rug[0]),
                  '-L', 'dependency=' + str(RELEASE / 'deps'), '-L', 'native=' + str(native),
                  '-o', str(binary)], 180, 0)
    dump(OUT / 'binary.json', {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size})
    guard(binary=True)


def compare():
    assert not (OUT / 'result.json').exists()
    guard(binary=True)
    binary = read(OUT / 'binary.json')['path']
    run('retain', [binary, 'retain'], 30, 66)
    stderr = (OUT / 'retain.stderr').read_text()
    assert 'ERROR: LeakSanitizer: detected memory leaks' in stderr
    assert '__gmp_default_' in stderr
    assert 'LeakSanitizer has encountered a fatal error.' not in stderr
    assert 'ERROR: AddressSanitizer:' not in stderr
    match = re.search(r'SUMMARY: AddressSanitizer: (\d+) byte\(s\) leaked in (\d+) allocation\(s\)', stderr)
    assert match and int(match[1]) > 0 and int(match[2]) > 0
    guard(binary=True)
    run('cleanup', [binary, 'cleanup'], 30, 0)
    assert (OUT / 'cleanup.stderr').read_bytes() == b''
    witness = (OUT / 'retain.stdout').read_bytes()
    assert witness == (OUT / 'cleanup.stdout').read_bytes()
    assert len(witness.splitlines()) == 2
    guard(binary=True)
    commands = [read(OUT / (name + '.json')) for name in ['format', 'build', 'retain', 'cleanup']]
    assert [row['exit_code'] for row in commands] == [0, 0, 66, 0]
    dump(OUT / 'result.json', {
        'complete': True, 'commands_matching_expected_exit': 4, 'native_processes': 2,
        'retain_leak_bytes': int(match[1]), 'retain_leak_allocations': int(match[2]),
        'cleanup_exit_code': 0, 'cleanup_stderr_empty': True,
        'both_numeric_witnesses_byte_identical': True,
        'witness_sha256': hashlib.sha256(witness).hexdigest(),
        'repository_crates_linked': False,
        'all_frozen_inputs_libraries_old_binaries_and_sources_unchanged': True,
        'artifacts_sha256': {path.name: sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'limits': read(E / 'normal-image-mpfr-cache-scope-v1.json')['limits'],
        'runtime_admitted': False, 'plan_complete': False,
    })
    print(json.dumps({'complete': True, 'retain_leak_bytes': int(match[1]),
                     'retain_leak_allocations': int(match[2]), 'cleanup_exit_code': 0,
                     'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    assert len(sys.argv) == 2
    if sys.argv[1] == 'prepare':
        prepare()
    else:
        assert sys.argv[1] == 'compare'
        compare()
