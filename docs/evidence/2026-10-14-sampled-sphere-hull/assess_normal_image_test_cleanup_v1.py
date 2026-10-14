"""Independently verify the finite cleanup qualification and its test-only delta."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'normal-image-test-cleanup-v1'
DESTINATION = E / 'normal-image-test-cleanup-closure-v1.json'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def mapping(path):
    return json.loads(gzip.decompress(path.read_bytes()))


def main():
    assert not DESTINATION.exists() and not (OUT / 'failure.json').exists()
    result = read(OUT / 'result.json')
    protocol = read(OUT / 'protocol.json')
    scope = read(E / 'normal-image-test-cleanup-scope-v1.json')
    assert result['complete'] and result['commands_matching_expected_exit'] == 12
    assert all(sha(OUT / name) == digest for name, digest in result['artifacts_sha256'].items())
    assert all(sha(E / name) == digest for name, digest in protocol['inputs_sha256'].items())
    commands = read(OUT / 'commands.json')
    assert [row['name'] for row in commands] == [
        'format', 'default-clippy', 'all-features-clippy', 'controls-format', 'overflow-build',
        'overflow-expected-failure', 'thread-exit-build', 'thread-exit-normal', 'thread-exit-unwind',
        'default-tests', 'all-features-tests', 'native-hull-tests']
    assert [row['exit_code'] for row in commands] == [0, 0, 0, 0, 0, 66, 0, 0, 0, 0, 0, 0]
    for row in commands:
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (row['name'] + '.' + stream)) == row[stream + '_sha256']
    sources = mapping(OUT / 'source-hashes.json.gz')
    roots = mapping(OUT / 'root-source-hashes.json.gz')
    parent_mapping = mapping(E / 'sphere-normal-image-v1/source-hashes.json.gz')
    source = Path(protocol['source'])
    parent = Path(protocol['parent_source'])
    assert len(sources) == 10137 and len(roots) == 10129 and len(parent_mapping) == 10136
    assert sources == parent_mapping | scope['expected_delta_sha256']
    for base, table in [(ROOT, roots), (source, sources), (parent, parent_mapping)]:
        assert all(sha(base / name) == digest for name, digest in table.items())
    binaries = read(OUT / 'binaries.json')
    for row in [*protocol['preserved_libraries_and_binaries'], *binaries.values()]:
        assert sha(row['path']) == row['sha256']

    # Reconstruct the exact parent text from only the declared test edits.
    registrations = 0
    helper = 'crates/hyperreal/tests/support/mpfr.rs'
    assert helper not in parent_mapping
    for name in scope['expected_delta_sha256']:
        if name == helper:
            continue
        text = (source / name).read_text()
        text, count = re.subn(r'(?m)^[ \t]*(?:crate::mpfr_test|mpfr)::cleanup_on_thread_exit\(\);\n', '', text)
        registrations += count
        if name.startswith('crates/hyperreal/tests/'):
            assert text.startswith('#[path = "support/mpfr.rs"]\nmod mpfr;\n\n')
            text = text.removeprefix('#[path = "support/mpfr.rs"]\nmod mpfr;\n\n')
        if name.endswith('/src/lib.rs'):
            suffix = '\n#[cfg(test)]\n#[path = "../tests/support/mpfr.rs"]\nmod mpfr_test;\n'
            assert text.endswith(suffix)
            text = text.removesuffix(suffix)
        if name.endswith('/src/dispatch_trace.rs'):
            new = ('        static LABELS: [[u8; 4]; 6] = [*b"same"; 6];\n'
                   '        let label = |index: usize| std::str::from_utf8(&LABELS[index]).unwrap();\n'
                   '        let first = (label(0), label(1), label(2));\n'
                   '        let second = (label(3), label(4), label(5));\n')
            old = ('        let label = || -> &\'static str { Box::leak(String::from("same").into_boxed_str()) };\n'
                   '        let first = (label(), label(), label());\n'
                   '        let second = (label(), label(), label());\n')
            assert text.count(new) == 1
            text = text.replace(new, old)
        assert text.encode() == (parent / name).read_bytes(), name
    assert registrations == 31
    witness = (E / 'normal-image-mpfr-cache-v1/retain.stdout').read_bytes()
    assert (OUT / 'thread-exit-normal.stdout').read_bytes() == witness
    assert (OUT / 'thread-exit-unwind.stdout').read_bytes() == witness.splitlines(keepends=True)[0] + b'expected_worker_panics=1\n'
    assert all((OUT / ('thread-exit-' + mode + '.stderr')).read_bytes() == b'' for mode in ['normal', 'unwind'])
    assert 'ERROR: AddressSanitizer: heap-buffer-overflow' in (OUT / 'overflow-expected-failure.stderr').read_text()
    counts = {}
    for name in ['default-tests', 'all-features-tests', 'native-hull-tests']:
        stdout = (OUT / (name + '.stdout')).read_text()
        stderr = (OUT / (name + '.stderr')).read_text()
        rows = [tuple(map(int, row)) for row in re.findall(
            r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;', stdout, re.MULTILINE)]
        assert rows and all(row[1] == row[3] == row[4] == 0 for row in rows)
        counts[name] = dict(zip(['passed', 'failed', 'ignored', 'measured', 'filtered_out'], map(sum, zip(*rows))))
        counts[name]['completed_test_summaries'] = len(rows)
        assert 'ERROR: AddressSanitizer:' not in stderr and 'LeakSanitizer' not in stderr
        for path in re.findall(r'Running .* \(([^()\n]+)\)', stderr):
            row = binaries[name + '/' + Path(path).name]
            assert row['path'] == path and sha(path) == row['sha256']
    assert counts == result['test_counts']
    closure = {
        'complete': True, 'trial_closed': True, 'driver_terminal_exit_code': 0,
        'commands_matching_expected_exit': 12, 'test_counts': counts,
        'test_only_changed_files': 13, 'mpfr_registration_call_sites': 31,
        'removing_declared_test_edits_reconstructs_exact_parent_files': True,
        'production_arithmetic_and_geometry_unchanged_from_parent': True,
        'thread_exit_normal_and_unwind_controls_clean': True,
        'old_failed_runs_remain_closed_and_unchanged': True,
        'all_frozen_sources_inputs_libraries_and_binaries_rechecked': True,
        'result_sha256': sha(OUT / 'result.json'), 'closure_reader_sha256': sha(__file__),
        'driver_stdout_sha256': sha(E / 'normal-image-test-cleanup-driver-v1.stdout'),
        'driver_stderr_sha256': sha(E / 'normal-image-test-cleanup-driver-v1.stderr'),
        'interpretation': 'The selected complete default/all-feature test suites and original native hull test pass with ASan and leak detection enabled after test-only cache and fixture cleanup. The previous reports remain failures of their own frozen runs.',
        'limits': scope['limits'], 'runtime_admitted': False, 'plan_complete': False,
    }
    DESTINATION.write_text(json.dumps(closure, indent=2) + '\n')
    ledger_path = ROOT / 'docs/properties-status.json'
    ledger = read(ledger_path)
    base = json.loads(subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT))
    assert {k: v for k, v in ledger.items() if k != 'current_work'} == {
        k: v for k, v in base.items() if k != 'current_work'}
    entry = ledger['current_work']['sampled_sphere_hull']
    prefix = str(E.relative_to(ROOT)) + '/'
    entry.update({
        'status': 'normal-form-image-filter-test-cleanup-sanitizers-complete',
        'sphere_normal_image_test_cleanup_scope': prefix + 'normal-image-test-cleanup-scope-v1.json',
        'sphere_normal_image_test_cleanup_protocol': prefix + 'qualify_normal_image_test_cleanup_v1.py',
        'sphere_normal_image_test_cleanup_result': prefix + 'normal-image-test-cleanup-v1/result.json',
        'sphere_normal_image_test_cleanup_closure': prefix + DESTINATION.name,
        'sphere_normal_image_test_cleanup_complete': True,
        'sphere_normal_image_test_cleanup_counts': counts,
        'sphere_normal_image_test_cleanup_admitted': False,
        'sphere_normal_image_runtime_admitted': False, 'plan_complete': False,
    })
    ledger_path.write_text(json.dumps(ledger, indent=2) + '\n')
    print(json.dumps({'complete': True, 'test_counts': counts, 'closure_sha256': sha(DESTINATION)}))


if __name__ == '__main__':
    main()
