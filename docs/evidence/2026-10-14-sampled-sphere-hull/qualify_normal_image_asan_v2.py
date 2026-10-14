"""Unrestricted sanitizer qualification after the preserved sandbox failure."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PRIOR = E / 'sphere-normal-image-v1'
OUT = E / 'normal-image-asan-v2'
WORK = ROOT / 'target/normal-image-asan-v2'
TARGET = 'x86_64-unknown-linux-gnu'
FLAGS = ['-Zsanitizer=address', '-Cforce-frame-pointers=yes']
ENV = {
    'CARGO_HOME': '/tmp/csgrs-properties-cargo',
    'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
    'CARGO_NET_OFFLINE': 'true',
    'CARGO_TERM_COLOR': 'never',
    'CARGO_TARGET_DIR': str(WORK / 'target'),
    'RUSTC_BOOTSTRAP': '1',
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


def main():
    assert not OUT.exists() and not WORK.exists()
    prerequisites = {
        'sphere-normal-image-qualification-v1/result.json': ('commands_all_zero', 12),
        'normal-image-native-cost-assessment-v1.json': ('commands_all_zero', 58),
        'normal-image-resources-assessment-v1.json': ('commands_all_zero', 29),
    }
    inputs = {name: sha(E / name) for name in [
        'qualify_normal_image_asan_v2.py', 'normal_image_asan_control_v1.rs',
        'normal-image-asan-scope-v2.json', 'normal-image-asan-closure-v1.json',
        'normal-image-lsan-environment-closure-v1.json',
        'normal-image-lsan-environment-v1/result.json', *prerequisites,
    ]}
    for name, (key, value) in prerequisites.items():
        result = read(E / name)
        assert result['complete'] and result[key] == value
        if 'artifacts_sha256' in result:
            for artifact, digest in result['artifacts_sha256'].items():
                path = (E / name).parent / artifact
                assert sha(path) == digest
                inputs[str(path.relative_to(E))] = digest
    cost = read(E / 'normal-image-native-cost-assessment-v1.json')
    assert not any(case['reproducible_cost_increase_flagged'] for case in cost['cases'])
    for directory, assessment_name, count in [
        ('normal-image-native-costs-v1', 'normal-image-native-cost-assessment-v1.json', 58),
        ('normal-image-resources-v1', 'normal-image-resources-assessment-v1.json', 29),
    ]:
        result_path = E / directory / 'result.json'
        result = read(result_path)
        assert result['complete'] and result['commands_all_zero'] == count
        assert sha(result_path) == read(E / assessment_name)['result_sha256']
        inputs[str(result_path.relative_to(E))] = sha(result_path)
        for artifact, digest in result['artifacts_sha256'].items():
            path = result_path.parent / artifact
            assert sha(path) == digest
            inputs[str(path.relative_to(E))] = digest
    protocol = read(PRIOR / 'protocol.json')
    source = Path(protocol['source'])
    mapping = json.loads(gzip.decompress((PRIOR / 'source-hashes.json.gz').read_bytes()))
    baseline = json.loads(gzip.decompress((E / 'mesh-identity-preparation-v1/source-hashes.json.gz').read_bytes()))
    roots = {name: sha(ROOT / name) for name in baseline}
    assert all(roots[name] == digest for name, digest in baseline.items() if name != 'docs/properties-status.json')
    inputs |= {str((PRIOR / name).relative_to(E)): sha(PRIOR / name)
               for name in ['protocol.json', 'source-hashes.json.gz', 'binaries.json', 'libraries.json']}
    records = [*read(PRIOR / 'binaries.json').values(), *read(PRIOR / 'libraries.json').values()]
    failed = read(E / 'normal-image-asan-closure-v1.json')
    records.extend([failed['executed_control_binary'], failed['executed_test_binary'],
                    read(E / 'normal-image-lsan-environment-v1/binary.json')])
    for name, digest in failed['artifacts_sha256'].items():
        path = E / 'normal-image-asan-v1' / name
        assert sha(path) == digest
        inputs[str(path.relative_to(E))] = digest

    def guard():
        assert all(sha(E / name) == digest for name, digest in inputs.items())
        assert all(sha(source / name) == digest for name, digest in mapping.items())
        assert all(sha(ROOT / name) == digest for name, digest in roots.items())
        assert all(sha(row['path']) == row['sha256'] for row in records)

    guard()
    compiler = subprocess.check_output(['rustc', '-Vv'], text=True)
    assert compiler == protocol['compiler']
    scope = read(E / 'normal-image-asan-scope-v2.json')
    assert scope['commands'] == 6 and scope['expected_control_exit_code'] == 66
    restriction = read(E / 'normal-image-lsan-environment-v1/result.json')
    assert restriction['complete'] and restriction['sandbox_associated_restriction_demonstrated']
    assert sha(E / 'normal-image-lsan-environment-v1/result.json') == scope['environment_diagnostic_result_sha256']
    assert sha(E / 'normal-image-asan-closure-v1.json') == scope['preserved_failed_qualification_closure_sha256']
    for name, digest in restriction['artifacts_sha256'].items():
        path = E / 'normal-image-lsan-environment-v1' / name
        assert sha(path) == digest
        inputs[str(path.relative_to(E))] = digest
    OUT.mkdir()
    WORK.mkdir()
    (OUT / 'root-source-hashes.json.gz').write_bytes(gzip.compress(
        (json.dumps(roots, sort_keys=True) + '\n').encode(), mtime=0))
    dump(OUT / 'protocol.json', {
        'source': str(source), 'source_files': len(mapping), 'root_source_files': len(roots),
        'source_mapping_sha256': sha(PRIOR / 'source-hashes.json.gz'),
        'inputs_sha256': inputs, 'compiler': compiler, 'environment': ENV,
        'target': TARGET, 'command_wall_limit_seconds': 1800,
        'required_execution_context': scope['execution_context'],
        'standard_library_rebuilt_with_sanitizer': False,
        'host_proc_macros_or_native_dependencies_claimed_instrumented': False,
        'limits': scope['limits'], 'runtime_admitted': False, 'plan_complete': False,
    })
    commands = []

    def run(name, command, expected=0):
        guard()
        print(json.dumps({'start': name, 'expected_exit_code': expected}), flush=True)
        started = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800s', *command],
                                  cwd=source, env=os.environ | ENV, stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'expected_exit_code': expected, 'exit_code': code,
               'process_wall_seconds': time.monotonic() - started,
               'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        print(json.dumps(row), flush=True)
        guard()
        if code != expected:
            dump(OUT / 'failure.json', {'complete': False, 'failed_command': row,
                                      'runtime_admitted': False, 'plan_complete': False})
            raise SystemExit(1)

    control = E / 'normal_image_asan_control_v1.rs'
    binary = WORK / 'asan-control'
    run('control-format', ['rustfmt', '--edition', '2024', '--check', str(control)])
    run('control-build', ['rustc', '--edition', '2024', '--target', TARGET, *FLAGS,
                          '-Copt-level=1', str(control), '-o', str(binary)])
    dump(OUT / 'control-binary.json', {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size})
    run('control-expected-failure', [str(binary)], expected=66)
    control_error = (OUT / 'control-expected-failure.stderr').read_text()
    assert 'ERROR: AddressSanitizer: heap-buffer-overflow' in control_error
    assert 'WRITE of size 1' in control_error

    packages = [arg for package in ['hyperreal', 'hyperlimit', 'hypermesh'] for arg in ['-p', package]]
    for name, features in [('default-tests', []), ('all-features-tests', ['--all-features'])]:
        run(name, ['cargo', 'test', '--locked', '--release', '--target', TARGET, *packages,
                   '--lib', '--tests', *features, '--', '--test-threads=2', '--show-output'])
    run('native-hull-tests', ['cargo', 'test', '--locked', '--release', '--target', TARGET,
                             '-p', 'csgrs', '--no-default-features', '--test', 'native_hull_determinism',
                             '--', '--test-threads=1', '--show-output'])
    summaries = {}
    pattern = r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;'
    for name in ['default-tests', 'all-features-tests', 'native-hull-tests']:
        raw = (OUT / (name + '.stdout')).read_text()
        rows = [tuple(map(int, row)) for row in re.findall(pattern, raw, re.MULTILINE)]
        assert rows and all(row[1] == row[3] == row[4] == 0 for row in rows)
        counts = dict(zip(['passed', 'failed', 'ignored', 'measured', 'filtered_out'], map(sum, zip(*rows))))
        counts['completed_test_summaries'] = len(rows)
        summaries[name] = counts
        if name == 'native-hull-tests':
            assert counts['passed'] == 1 and counts['ignored'] == 0
        else:
            for test in ['cyclotomic_root_images_reject_only_certified_nonzero_residues',
                         'cyclotomic_root_collisions_and_bad_units_keep_exact_zero_search']:
                assert re.search(r'^test .*::' + test + r' \.\.\. ok$', raw, re.MULTILINE)
        assert 'ERROR: AddressSanitizer:' not in (OUT / (name + '.stderr')).read_text()
    guard()
    assert len(commands) == 6 and [row['exit_code'] for row in commands] == [0, 0, 66, 0, 0, 0]
    dump(OUT / 'result.json', {
        'complete': True, 'commands_matching_expected_exit': 6, 'commands_zero': 5,
        'intentional_control_failures': 1, 'expected_control_exit_code': 66,
        'control_heap_buffer_overflow_detected': True, 'test_counts': summaries,
        'source_files_rechecked': len(mapping), 'root_source_files_rechecked': len(roots),
        'all_frozen_sources_inputs_and_original_binaries_unchanged': True,
        'artifacts_sha256': {path.name: sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'limits': scope['limits'], 'runtime_admitted': False, 'plan_complete': False,
    })
    print(json.dumps({'complete': True, 'test_counts': summaries, 'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    main()
