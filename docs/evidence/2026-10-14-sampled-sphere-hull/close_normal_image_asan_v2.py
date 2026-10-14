"""Preserve the failed unrestricted qualification and characterize its report."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'normal-image-asan-v2'
DESTINATION = E / 'normal-image-asan-closure-v2.json'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def main():
    assert not DESTINATION.exists() and not (OUT / 'result.json').exists()
    protocol = read(OUT / 'protocol.json')
    failure = read(OUT / 'failure.json')
    commands = read(OUT / 'commands.json')
    assert [row['name'] for row in commands] == [
        'control-format', 'control-build', 'control-expected-failure', 'default-tests']
    assert [row['exit_code'] for row in commands] == [0, 0, 66, 66]
    assert not failure['complete'] and failure['failed_command'] == commands[-1]
    for row in commands:
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (row['name'] + '.' + stream)) == row[stream + '_sha256']
    assert all(sha(E / name) == digest for name, digest in protocol['inputs_sha256'].items())
    roots = json.loads(gzip.decompress((OUT / 'root-source-hashes.json.gz').read_bytes()))
    mapping = json.loads(gzip.decompress((E / 'sphere-normal-image-v1/source-hashes.json.gz').read_bytes()))
    assert len(roots) == 10129 and len(mapping) == 10136
    assert all(sha(ROOT / name) == digest for name, digest in roots.items())
    assert all(sha(Path(protocol['source']) / name) == digest for name, digest in mapping.items())
    for name in ['binaries.json', 'libraries.json']:
        assert all(sha(row['path']) == row['sha256']
                   for row in read(E / 'sphere-normal-image-v1' / name).values())

    stdout = (OUT / 'default-tests.stdout').read_text()
    stderr = (OUT / 'default-tests.stderr').read_text()
    pattern = r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;'
    counts = [tuple(map(int, row)) for row in re.findall(pattern, stdout, re.MULTILINE)]
    assert len(counts) == 24 and list(map(sum, zip(*counts))) == [1379, 0, 7, 0, 0]
    assert counts[-1] == (795, 0, 0, 0, 0)
    for test in ['cyclotomic_root_images_reject_only_certified_nonzero_residues',
                 'cyclotomic_root_collisions_and_bad_units_keep_exact_zero_search']:
        assert re.search(r'^test .*::' + test + r' \.\.\. ok$', stdout, re.MULTILINE)
    assert 'ERROR: LeakSanitizer: detected memory leaks' in stderr
    assert 'ERROR: AddressSanitizer:' not in stderr
    assert 'LeakSanitizer has encountered a fatal error.' not in stderr
    assert 'SUMMARY: AddressSanitizer: 163672 byte(s) leaked in 560 allocation(s).' in stderr
    leaks = []
    for block in re.split(r'\n\n+', stderr):
        match = re.search(r'(Direct|Indirect) leak of (\d+) byte\(s\) in (\d+) object\(s\)', block)
        if match:
            leaks.append({'kind': match[1], 'bytes': int(match[2]), 'objects': int(match[3]),
                          'gmp_allocator_in_stack': '__gmp_default_' in block})
    assert len(leaks) == 412
    assert sum(row['bytes'] for row in leaks) == 163672
    assert sum(row['objects'] for row in leaks) == 560
    assert all(row['kind'] == 'Direct' and row['gmp_allocator_in_stack'] for row in leaks)
    executed = [Path(name) for name in re.findall(r'Running .* \(([^()\n]+)\)', stderr)]
    assert len(executed) == len(set(executed)) == 24
    binaries = [{'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size} for path in executed]
    control = read(OUT / 'control-binary.json')
    assert sha(control['path']) == control['sha256']
    installed_docs = Path('/tmp/csgrs-properties-cargo/registry/src/index.crates.io-1949cf8c6b5b557f/rug-1.30.0/src/float/mod.rs')
    assert 'pub fn free_cache(which: FreeCache)' in installed_docs.read_text()
    closure = {
        'complete': False, 'trial_closed': True, 'driver_terminal_exit_code': 1,
        'execution_context': 'exec_command require_escalated; automatic review approved execution.',
        'commands_completed': 4, 'commands_matching_expected_exit': 3,
        'failed_command': 'default-tests', 'failed_command_exit_code': 66,
        'completed_test_counts': {'passed': 1379, 'failed': 0, 'ignored': 7,
                                  'measured': 0, 'filtered_out': 0, 'summaries': 24},
        'hyperreal_unit_tests_passed': 795,
        'both_new_normal_form_tests_passed': True,
        'leak_bytes': 163672, 'leak_allocations': 560, 'direct_leak_report_blocks': 412,
        'all_reported_allocation_stacks_include_gmp_allocator': True,
        'leak_report_blocks': leaks,
        'executed_test_binaries': binaries, 'executed_control_binary': control,
        'source_files_rechecked': len(mapping), 'root_source_files_rechecked': len(roots),
        'all_frozen_sources_inputs_original_libraries_and_binaries_rechecked': True,
        'artifacts_sha256': {path.name: sha(path) for path in sorted(OUT.iterdir()) if path.is_file()},
        'driver_stdout_sha256': sha(E / 'normal-image-asan-driver-v2.stdout'),
        'driver_stderr_sha256': sha(E / 'normal-image-asan-driver-v2.stderr'),
        'closure_reader_sha256': sha(__file__),
        'installed_rug_cache_documentation': {'path': str(installed_docs), 'sha256': sha(installed_docs)},
        'interpretation': 'The unrestricted process completed its assertions but failed leak checking. All report blocks include GMP allocation, and Rug is a dev-dependency used by test oracles/benchmarks. MPFR thread-local cache cleanup is a hypothesis supported by installed Rug documentation; it has not yet been demonstrated as the cause of this whole report.',
        'remaining_unexecuted': 'Later default Hyperreal integration tests, all-feature tests and original native sphere replay.',
        'next_step': 'Preserve this failed run and independently compare a standalone Rug-only threaded program with and without explicit local cache cleanup. Do not suppress leaks, alter these executed inputs, or attribute the report to the new filter without evidence.',
        'runtime_admitted': False, 'plan_complete': False,
    }
    DESTINATION.write_text(json.dumps(closure, indent=2) + '\n')
    ledger_path = ROOT / 'docs/properties-status.json'
    ledger = read(ledger_path)
    base = json.loads(subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT))
    assert {key: value for key, value in ledger.items() if key != 'current_work'} == {
        key: value for key, value in base.items() if key != 'current_work'}
    entry = ledger['current_work']['sampled_sphere_hull']
    prefix = str(E.relative_to(ROOT)) + '/'
    entry.update({
        'status': 'normal-form-image-filter-gmp-leak-report-preserved',
        'sphere_normal_image_asan_v2_executed': True,
        'sphere_normal_image_asan_v2_complete': False,
        'sphere_normal_image_asan_v2_failure': prefix + 'normal-image-asan-v2/failure.json',
        'sphere_normal_image_asan_v2_closure': prefix + DESTINATION.name,
        'sphere_normal_image_asan_v2_partial_counts': closure['completed_test_counts'],
        'sphere_normal_image_asan_v2_leak_bytes': 163672,
        'sphere_normal_image_asan_v2_leak_allocations': 560,
        'sphere_normal_image_asan_v2_all_reported_stacks_gmp': True,
        'sphere_normal_image_runtime_admitted': False, 'plan_complete': False,
    })
    ledger_path.write_text(json.dumps(ledger, indent=2) + '\n')
    print(json.dumps({'trial_closed': True, 'complete': False,
                      'leak_bytes': 163672, 'leak_allocations': 560,
                      'closure_sha256': sha(DESTINATION)}))


if __name__ == '__main__':
    main()
