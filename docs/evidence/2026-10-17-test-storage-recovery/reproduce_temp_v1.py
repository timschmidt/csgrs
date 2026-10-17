"""Reproduce the current test-path restriction with writable TMPDIR selected."""
import hashlib
import json
import os
import resource
import signal
import subprocess
import time
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
O = W / 'temp-reproduction-v1'
record = json.loads((W / 'reused-native-artifacts-v1.json').read_text())
binary = Path(record['lib_tests']['retained'])

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def dump(p, v):
    Path(p).write_text(json.dumps(v, indent=2) + '\n')

assert sha(binary) == record['lib_tests']['sha256']
assert all(sha(ROOT / p) == h for p, h in record['relevant_test_sources_identical_to_current_main'].items())
O.mkdir(exist_ok=False)
scratch = O / 'selected-temporary-directory'
scratch.mkdir()
control = scratch / 'write-control'
control.write_bytes(b'Writable storage selected for the unchanged tests.\n')
tests = [
    'conversion::tests::convert_executes_successful_converter_and_returns_output_paths',
    'conversion::tests::convert_executes_successful_kicad_cli_converter_and_returns_output_paths',
    'conversion::tests::convert_reports_directory_creation_failure',
    'conversion::tests::converter_provenance_hashes_inputs_and_manifests_outputs',
    'parquet_report::tests::writes_readable_parquet_report',
    'sqlite_report::tests::writes_queryable_sqlite_report',
]
pins = {str(p): sha(p) for p in [Path(__file__), binary, W / 'reused-native-artifacts-v1.json', W / 'contract-v1.md', control]}
dump(O / 'protocol.json', {
    'input_sha256': pins, 'tests': tests, 'TMPDIR': str(scratch), 'custom_storage_write_succeeded': True,
    'containment': 'Python Popen process group, 60 second deadline, 8 GiB address space, core dumps disabled.',
    'current_test_source_equivalence': record['relevant_test_sources_identical_to_current_main'],
    'library_scope': 'Reuse E38 executable; only the isolated cache proposal differs in geometry dependencies. The reproduced failures occur in the unchanged filesystem fixture setup/output, not geometric decisions.',
    'quota_before': subprocess.check_output(['quota', '-s'], text=True),
})
rows = []
artifacts = {}

def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

for i, test in enumerate(tests):
    name = str(i) + '-' + test.rsplit('::', 1)[1]
    command = [str(binary), test, '--exact', '--nocapture', '--test-threads=1']
    start = time.monotonic()
    with (O / (name + '.stdout')).open('wb') as out, (O / (name + '.stderr')).open('wb') as err:
        proc = subprocess.Popen(command, cwd=record['source'], env=os.environ | {'TMPDIR': str(scratch), 'RUST_BACKTRACE': '0'},
                                stdout=out, stderr=err, preexec_fn=limits, start_new_session=True)
        timed_out = False
        try:
            code = proc.wait(timeout=60)
        except subprocess.TimeoutExpired:
            timed_out = True
            os.killpg(proc.pid, signal.SIGKILL)
            code = proc.wait()
    roots = list(Path('/tmp').glob('*' + str(proc.pid) + '*'))
    for root in roots:
        for p in ([root] if root.is_file() else root.rglob('*')):
            if p.is_file():
                artifacts[str(p)] = {'sha256': sha(p), 'bytes': p.stat().st_size, 'test': test}
    row = {'name': name, 'test': test, 'command': command, 'pid': proc.pid, 'exit_code': code,
           'timed_out': timed_out, 'elapsed_seconds': time.monotonic() - start, 'temporary_roots': [str(p) for p in roots],
           'stdout_sha256': sha(O / (name + '.stdout')), 'stderr_sha256': sha(O / (name + '.stderr'))}
    rows.append(row)
    dump(O / 'commands.json', rows)
    print(json.dumps({'test': test, 'exit_code': code}), flush=True)
assert [r['exit_code'] for r in rows] == [0, 101, 101, 101, 101, 101]
assert all(not r['timed_out'] for r in rows)
assert all('Disk quota exceeded' in (O / (r['name'] + '.stderr')).read_text() for r in rows[1:5])
assert 'disk I/O error' in (O / (rows[-1]['name'] + '.stderr')).read_text()
assert list(scratch.iterdir()) == [control]
assert all(sha(p) == h for p, h in pins.items())
dump(O / 'result.json', {
    'complete': True, 'expected_control_passed': 1, 'expected_file_failures_reproduced': 5,
    'selected_TMPDIR_writable_but_unused': True, 'raw_failed_artifacts': artifacts,
    'temporary_roots': [p for r in rows for p in r['temporary_roots']],
    'source_changes_applied': False, 'main_sources_unchanged': True,
})
