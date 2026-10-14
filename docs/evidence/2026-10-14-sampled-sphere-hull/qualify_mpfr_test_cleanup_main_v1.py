"""Qualify the same test-only cleanup against admitted main runtime sources."""
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
OUT = E / 'mpfr-test-cleanup-main-v1'
WORK = ROOT / 'target/mpfr-test-cleanup-main-v1'
SOURCE = WORK / 'source'
TARGET = 'x86_64-unknown-linux-gnu'
ENV = {
    'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
    'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never',
    'CARGO_TARGET_DIR': str(WORK / 'target'), 'RUSTC_BOOTSTRAP': '1',
    'RUSTFLAGS': '-Zsanitizer=address -Cforce-frame-pointers=yes',
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


def main():
    assert not OUT.exists() and SOURCE.exists() and not (WORK / 'target').exists()
    scope = read(E / 'mpfr-test-cleanup-main-scope-v1.json')
    prior = E / 'normal-image-test-cleanup-v1'
    closure = read(E / 'normal-image-test-cleanup-closure-v1.json')
    result = read(prior / 'result.json')
    assert closure['complete'] and closure['trial_closed'] and result['complete']
    assert sha(prior / 'result.json') == closure['result_sha256']
    assert closure['removing_declared_test_edits_reconstructs_exact_parent_files']
    assert closure['thread_exit_normal_and_unwind_controls_clean']
    baseline = mapping(E / 'mesh-identity-preparation-v1/source-hashes.json.gz')
    actual = {str(path.relative_to(SOURCE)): sha(path) for path in SOURCE.rglob('*') if path.is_file()}
    assert actual == baseline | scope['expected_delta_sha256'] | scope['candidate_ledger_sha256']
    assert len(actual) == 10130
    roots = {name: sha(ROOT / name) for name in baseline}
    assert all(roots[name] == digest for name, digest in baseline.items() if name != 'docs/properties-status.json')
    for name, digest in scope['expected_delta_sha256'].items():
        if name in baseline:
            assert roots[name] == baseline[name]
        else:
            assert not (ROOT / name).exists()
        assert sha(Path(read(prior / 'protocol.json')['source']) / name) == digest
    inputs = {name: sha(E / name) for name in [
        'qualify_mpfr_test_cleanup_main_v1.py', 'mpfr-test-cleanup-main-scope-v1.json',
        'normal-image-test-cleanup-scope-v1.json', 'normal-image-test-cleanup-v1.patch',
        'normal-image-test-cleanup-closure-v1.json', 'normal-image-test-cleanup-v1/result.json',
        'mesh-identity-preparation-v1/source-hashes.json.gz',
    ]}
    for name, digest in result['artifacts_sha256'].items():
        assert sha(prior / name) == digest
        inputs['normal-image-test-cleanup-v1/' + name] = digest
    prior_protocol = read(prior / 'protocol.json')
    preserved = [*prior_protocol['preserved_libraries_and_binaries'], *read(prior / 'binaries.json').values()]
    prior_sources = mapping(prior / 'source-hashes.json.gz')
    prior_source = Path(prior_protocol['source'])
    parent = Path(prior_protocol['parent_source'])
    parent_sources = mapping(E / 'sphere-normal-image-v1/source-hashes.json.gz')
    compiler = subprocess.check_output(['rustc', '-Vv'], text=True)
    assert compiler == prior_protocol['compiler']
    binaries, commands = {}, []

    def guard():
        assert all(sha(E / name) == digest for name, digest in inputs.items())
        for base, table in [(ROOT, roots), (SOURCE, actual), (prior_source, prior_sources), (parent, parent_sources)]:
            assert all(sha(base / name) == digest for name, digest in table.items())
        assert all(sha(row['path']) == row['sha256'] for row in [*preserved, *binaries.values()])

    guard()
    OUT.mkdir()
    for name, table in [('source-hashes.json.gz', actual), ('root-source-hashes.json.gz', roots)]:
        (OUT / name).write_bytes(gzip.compress((json.dumps(table, sort_keys=True) + '\n').encode(), mtime=0))
    dump(OUT / 'protocol.json', {
        'source': str(SOURCE), 'source_files': len(actual), 'root_source_files': len(roots),
        'inputs_sha256': inputs, 'compiler': compiler, 'environment': ENV,
        'preserved_libraries_and_binaries': preserved,
        'commands': 5, 'command_wall_limit_seconds': 1800,
        'test_patch_identical_to_prior_qualification': True,
        'unadmitted_scalar_or_mesh_runtime_prototypes_included': False,
        'test_cleanup_admitted': False, 'plan_complete': False,
    })

    def run(name, command):
        guard()
        print(json.dumps({'start': name}), flush=True)
        start = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800s', *command],
                                  cwd=SOURCE, env=os.environ | ENV, stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'exit_code': code,
               'process_wall_seconds': time.monotonic() - start,
               'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        if name.endswith('-tests'):
            for path in re.findall(r'Running .* \(([^()\n]+)\)', (OUT / (name + '.stderr')).read_text()):
                binaries[name + '/' + Path(path).name] = {'path': path, 'sha256': sha(path), 'bytes': Path(path).stat().st_size}
            dump(OUT / 'binaries.json', binaries)
        print(json.dumps(row), flush=True)
        guard()
        if code:
            dump(OUT / 'failure.json', {'complete': False, 'failed_command': row,
                                      'test_cleanup_admitted': False, 'plan_complete': False})
            raise SystemExit(1)

    run('format', ['cargo', 'fmt', '--all', '--', '--check'])
    for name, features in [('default-clippy', []), ('all-features-clippy', ['--all-features'])]:
        run(name, ['cargo', 'clippy', '--locked', '--release', '--target', TARGET,
                   '-p', 'hyperreal', '--all-targets', *features, '--', '-D', 'warnings'])
    counts = {}
    for name, features in [('default-tests', []), ('all-features-tests', ['--all-features'])]:
        run(name, ['cargo', 'test', '--locked', '--release', '--target', TARGET,
                   '-p', 'hyperreal', '--lib', '--tests', *features, '--', '--test-threads=2', '--show-output'])
        stdout = (OUT / (name + '.stdout')).read_text()
        stderr = (OUT / (name + '.stderr')).read_text()
        rows = [tuple(map(int, row)) for row in re.findall(
            r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;', stdout, re.MULTILINE)]
        assert rows and all(row[1] == row[3] == row[4] == 0 for row in rows)
        counts[name] = dict(zip(['passed', 'failed', 'ignored', 'measured', 'filtered_out'], map(sum, zip(*rows))))
        counts[name]['completed_test_summaries'] = len(rows)
        assert 'ERROR: AddressSanitizer:' not in stderr and 'LeakSanitizer' not in stderr
        for test in ['large_signed_rational_powers_match_mpfr',
                     'chudnovsky_pi_matches_mpfr_from_one_bit_through_many_digits',
                     'pi_and_square_root_match_directed_mpfr_reference_brackets',
                     'core2_zimmermann_exp_inputs_match_mpfr_at_256_bits']:
            assert re.search(r'^test (?:.*::)?' + test + r' \.\.\. ok$', stdout, re.MULTILINE)
        if name == 'all-features-tests':
            assert re.search(r'^test .*::dispatch_snapshot_merges_equal_labels_from_distinct_static_storage \.\.\. ok$', stdout, re.MULTILINE)
    guard()
    assert len(commands) == 5 and all(row['exit_code'] == 0 for row in commands)
    dump(OUT / 'result.json', {
        'complete': True, 'commands_all_zero': 5, 'test_counts': counts,
        'test_only_changed_files': 13, 'test_patch_identical_to_prior_qualification': True,
        'unadmitted_scalar_or_mesh_runtime_prototypes_included': False,
        'all_frozen_sources_inputs_libraries_and_binaries_rechecked': True,
        'artifacts_sha256': {path.name: sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'limits': scope['limits'], 'test_cleanup_admitted': False, 'plan_complete': False,
    })
    print(json.dumps({'complete': True, 'test_counts': counts, 'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    main()
