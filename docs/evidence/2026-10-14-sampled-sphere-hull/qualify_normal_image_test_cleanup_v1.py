"""Freeze and qualify test-only cleanup without modifying earlier executed sources."""
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
OUT = E / 'normal-image-test-cleanup-v1'
WORK = ROOT / 'target/normal-image-test-cleanup-v1'
SOURCE = WORK / 'source'
PRIOR = E / 'sphere-normal-image-v1'
TARGET = 'x86_64-unknown-linux-gnu'
FLAGS = ['-Zsanitizer=address', '-Cforce-frame-pointers=yes']
ENV = {
    'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
    'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never',
    'CARGO_TARGET_DIR': str(WORK / 'target'), 'RUSTC_BOOTSTRAP': '1',
    'RUSTFLAGS': ' '.join(FLAGS),
    'ASAN_OPTIONS': 'detect_leaks=1:halt_on_error=1:abort_on_error=0:exitcode=66',
    'LSAN_OPTIONS': '',
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def mapping(path):
    return json.loads(gzip.decompress(path.read_bytes()))


def guard():
    protocol = read(OUT / 'protocol.json')
    assert all(sha(E / name) == digest for name, digest in protocol['inputs_sha256'].items())
    for source, table in [
        (ROOT, mapping(OUT / 'root-source-hashes.json.gz')),
        (SOURCE, mapping(OUT / 'source-hashes.json.gz')),
        (Path(protocol['parent_source']), mapping(PRIOR / 'source-hashes.json.gz')),
    ]:
        assert all(sha(source / name) == digest for name, digest in table.items())
    assert all(sha(row['path']) == row['sha256'] for row in protocol['preserved_libraries_and_binaries'])
    if (OUT / 'binaries.json').exists():
        assert all(sha(row['path']) == row['sha256'] for row in read(OUT / 'binaries.json').values())


def prepare():
    assert not OUT.exists() and SOURCE.exists() and not (WORK / 'target').exists()
    scope = read(E / 'normal-image-test-cleanup-scope-v1.json')
    parent = Path(read(PRIOR / 'protocol.json')['source'])
    old = mapping(PRIOR / 'source-hashes.json.gz')
    actual = {str(path.relative_to(SOURCE)): sha(path) for path in SOURCE.rglob('*') if path.is_file()}
    assert len(old) == 10136 and len(actual) == scope['source_files'] == 10137
    assert actual == old | scope['expected_delta_sha256']
    assert len(scope['expected_delta_sha256']) == 13
    # Every pre-existing cleanup target is also identical in the admitted root.
    # This permits separate review of the test repair without admitting the runtime prototype.
    assert all(sha(ROOT / name) == old[name] for name in scope['expected_delta_sha256'] if name in old)
    baseline = mapping(E / 'mesh-identity-preparation-v1/source-hashes.json.gz')
    roots = {name: sha(ROOT / name) for name in baseline}
    assert all(roots[name] == digest for name, digest in baseline.items() if name != 'docs/properties-status.json')
    cache = E / 'normal-image-mpfr-cache-v1'
    cache_result = read(cache / 'result.json')
    assert cache_result['complete'] and cache_result['cleanup_stderr_empty']
    failed = read(E / 'normal-image-asan-closure-v2.json')
    assert failed['trial_closed'] and not failed['complete']
    inputs = {name: sha(E / name) for name in [
        'qualify_normal_image_test_cleanup_v1.py', 'normal-image-test-cleanup-scope-v1.json',
        'normal-image-test-cleanup-v1.patch', 'normal_image_mpfr_thread_exit_v1.rs',
        'normal_image_asan_control_v1.rs', 'normal-image-mpfr-cache-closure-v1.json',
        'normal-image-asan-closure-v2.json', 'normal-image-mpfr-cache-v1/result.json',
        'sphere-normal-image-v1/protocol.json', 'sphere-normal-image-v1/source-hashes.json.gz',
    ]}
    for directory, artifacts in [
        ('normal-image-mpfr-cache-v1', cache_result['artifacts_sha256']),
        ('normal-image-asan-v2', failed['artifacts_sha256']),
    ]:
        for name, digest in artifacts.items():
            assert sha(E / directory / name) == digest
            inputs[directory + '/' + name] = digest
    cache_protocol = read(cache / 'protocol.json')
    records = [*cache_protocol['libraries'], *cache_protocol['preserved_executed_binaries'],
               read(cache / 'binary.json'), *read(PRIOR / 'binaries.json').values(),
               *read(PRIOR / 'libraries.json').values()]
    compiler = subprocess.check_output(['rustc', '-Vv'], text=True)
    assert compiler == cache_protocol['compiler']
    OUT.mkdir()
    for name, table in [('source-hashes.json.gz', actual), ('root-source-hashes.json.gz', roots)]:
        (OUT / name).write_bytes(gzip.compress((json.dumps(table, sort_keys=True) + '\n').encode(), mtime=0))
    dump(OUT / 'protocol.json', {
        'source': str(SOURCE), 'parent_source': str(parent),
        'source_files': len(actual), 'root_source_files': len(roots),
        'inputs_sha256': inputs, 'compiler': compiler, 'environment': ENV,
        'preserved_libraries_and_binaries': records,
        'rug_rlib': cache_protocol['rug_rlib'], 'native_search_path': cache_protocol['native_search_path'],
        'commands': 12, 'expected_exit_codes': [0, 0, 0, 0, 0, 66, 0, 0, 0, 0, 0, 0],
        'test_only_delta': True, 'runtime_admitted': False, 'plan_complete': False,
    })
    guard()
    print(json.dumps({'prepared': True, 'source_files': len(actual), 'changed_files': 13}), flush=True)


def qualify():
    assert not (OUT / 'commands.json').exists() and not (OUT / 'result.json').exists()
    guard()
    protocol = read(OUT / 'protocol.json')
    scope = read(E / 'normal-image-test-cleanup-scope-v1.json')
    commands, binaries = [], {}

    def run(name, command, expected=0, seconds=1800):
        guard()
        print(json.dumps({'start': name, 'expected_exit_code': expected}), flush=True)
        start = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', str(seconds), *command],
                                  cwd=SOURCE, env=os.environ | ENV, stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'expected_exit_code': expected, 'exit_code': code,
               'process_wall_seconds': time.monotonic() - start,
               'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        print(json.dumps(row), flush=True)
        if name in ['default-tests', 'all-features-tests', 'native-hull-tests']:
            for path in re.findall(r'Running .* \(([^()\n]+)\)', (OUT / (name + '.stderr')).read_text()):
                save_binary(name + '/' + Path(path).name, Path(path))
        guard()
        if code != expected:
            dump(OUT / 'failure.json', {'complete': False, 'failed_command': row,
                                      'runtime_admitted': False, 'plan_complete': False})
            raise SystemExit(1)

    def save_binary(name, binary):
        binaries[name] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
        dump(OUT / 'binaries.json', binaries)

    run('format', ['cargo', 'fmt', '--all', '--', '--check'])
    for name, features in [('default-clippy', []), ('all-features-clippy', ['--all-features'])]:
        run(name, ['cargo', 'clippy', '--locked', '--release', '--target', TARGET,
                   '-p', 'hyperreal', '--all-targets', *features, '--', '-D', 'warnings'])
    overflow = E / 'normal_image_asan_control_v1.rs'
    thread_exit = E / 'normal_image_mpfr_thread_exit_v1.rs'
    run('controls-format', ['rustfmt', '--edition', '2024', '--check', str(overflow), str(thread_exit)], seconds=30)
    control = WORK / 'asan-control'
    run('overflow-build', ['rustc', '--edition', '2024', '--target', TARGET, *FLAGS,
                           '-Copt-level=1', '-Dwarnings', str(overflow), '-o', str(control)], seconds=180)
    save_binary('overflow-control', control)
    run('overflow-expected-failure', [str(control)], expected=66, seconds=30)
    assert 'ERROR: AddressSanitizer: heap-buffer-overflow' in (OUT / 'overflow-expected-failure.stderr').read_text()
    helper = WORK / 'thread-exit-control'
    run('thread-exit-build', ['rustc', '--edition', '2024', '--target', TARGET, *FLAGS,
                             '-Copt-level=1', '-Cdebuginfo=1', '-Dwarnings', str(thread_exit),
                             '--extern', 'rug=' + protocol['rug_rlib'],
                             '-L', 'dependency=' + str(Path(protocol['rug_rlib']).parent),
                             '-L', 'native=' + protocol['native_search_path'], '-o', str(helper)], seconds=180)
    save_binary('thread-exit-control', helper)
    for mode in ['normal', 'unwind']:
        run('thread-exit-' + mode, [str(helper), mode], seconds=30)
        assert (OUT / ('thread-exit-' + mode + '.stderr')).read_bytes() == b''
    witness = (E / 'normal-image-mpfr-cache-v1/retain.stdout').read_bytes()
    assert (OUT / 'thread-exit-normal.stdout').read_bytes() == witness
    assert (OUT / 'thread-exit-unwind.stdout').read_bytes() == witness.splitlines(keepends=True)[0] + b'expected_worker_panics=1\n'

    packages = [arg for package in ['hyperreal', 'hyperlimit', 'hypermesh'] for arg in ['-p', package]]
    for name, features in [('default-tests', []), ('all-features-tests', ['--all-features'])]:
        run(name, ['cargo', 'test', '--locked', '--release', '--target', TARGET, *packages,
                   '--lib', '--tests', *features, '--', '--test-threads=2', '--show-output'])
    run('native-hull-tests', ['cargo', 'test', '--locked', '--release', '--target', TARGET,
                             '-p', 'csgrs', '--no-default-features', '--test', 'native_hull_determinism',
                             '--', '--test-threads=1', '--show-output'])
    summaries = {}
    for name in ['default-tests', 'all-features-tests', 'native-hull-tests']:
        raw = (OUT / (name + '.stdout')).read_text()
        stderr = (OUT / (name + '.stderr')).read_text()
        rows = [tuple(map(int, row)) for row in re.findall(
            r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;', raw, re.MULTILINE)]
        assert rows and all(row[1] == row[3] == row[4] == 0 for row in rows)
        counts = dict(zip(['passed', 'failed', 'ignored', 'measured', 'filtered_out'], map(sum, zip(*rows))))
        counts['completed_test_summaries'] = len(rows)
        summaries[name] = counts
        assert 'ERROR: AddressSanitizer:' not in stderr and 'LeakSanitizer' not in stderr
        if name == 'native-hull-tests':
            assert counts['passed'] == 1 and counts['ignored'] == 0
        else:
            for test in ['cyclotomic_root_images_reject_only_certified_nonzero_residues',
                         'cyclotomic_root_collisions_and_bad_units_keep_exact_zero_search']:
                assert re.search(r'^test .*::' + test + r' \.\.\. ok$', raw, re.MULTILINE)
        if name == 'all-features-tests':
            assert re.search(r'^test .*::dispatch_snapshot_merges_equal_labels_from_distinct_static_storage \.\.\. ok$', raw, re.MULTILINE)
    guard()
    assert len(commands) == 12 and [row['exit_code'] for row in commands] == protocol['expected_exit_codes']
    dump(OUT / 'result.json', {
        'complete': True, 'commands_matching_expected_exit': 12, 'commands_zero': 11,
        'intentional_overflow_control_failures': 1, 'thread_exit_normal_and_unwind_clean': True,
        'normal_numeric_witness_matches_prior_control': True,
        'test_counts': summaries, 'test_only_changed_files': 13,
        'all_frozen_sources_inputs_libraries_and_binaries_rechecked': True,
        'artifacts_sha256': {path.name: sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'limits': scope['limits'], 'runtime_admitted': False, 'plan_complete': False,
    })
    print(json.dumps({'complete': True, 'test_counts': summaries,
                      'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    assert len(sys.argv) == 2
    if sys.argv[1] == 'prepare':
        prepare()
    else:
        assert sys.argv[1] == 'qualify'
        qualify()
