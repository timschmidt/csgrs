"""Audit complete production validation without claiming resource admission."""
import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
PRODUCTION = EVIDENCE / 'laurent-root-production'
OUT = EVIDENCE / 'laurent-root-production-validation'
SOURCE = ROOT / 'target/laurent-root-production/source'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
source = read(PRODUCTION / 'sources.json')
base = read(Path(source['base_source_record']))
native = read(Path(source['main_source_record']))
assert digest(Path(source['base_source_record'])) == source['base_source_record_sha256']
assert digest(Path(source['main_source_record'])) == source['main_source_record_sha256']
hashes = base['source_hashes'] | source['changed_source_hashes'] | source['additional_source_hashes']
assert len(hashes) == source['source_files']
assert all(digest(SOURCE / name) == value for name, value in hashes.items())
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
for filename, expected in [('source-delta.tar.gz', source['changed_source_hashes']), ('restored-inputs.tar.gz', source['additional_source_hashes'])]:
    with tarfile.open(PRODUCTION / filename) as archive:
        assert set(archive.getnames()) == set(expected)
        for name in archive.getnames():
            assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == expected[name]
audit = read(PRODUCTION / 'committed-input-audit.json')
assert audit['mismatches'] == []
assert len(audit['restored_missing_inputs']) == 261
assert all(not name.endswith('.rs') for name in audit['restored_missing_inputs'])
status = read(OUT / 'status.json')
assert [row['name'] for row in status] == [
    'format', 'scalar-default-tests', 'scalar-all-tests', 'scalar-clippy', 'scalar-doc',
    'curve-clippy', 'curve-tests', 'curve-doc', 'build-drc-tests', 'drc-tests',
    'drc-clippy', 'drc-doc', 'full-board-smoke',
]
assert all(row['exit_code'] == 0 for row in status)
tests = {}
for name in ['scalar-default-tests', 'scalar-all-tests', 'curve-tests', 'drc-tests']:
    log = (OUT / (name + '.log')).read_text()
    rows = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored', log)
    assert rows and all(int(row[1]) == 0 for row in rows)
    tests[name] = {'binaries': len(rows), 'passed': sum(int(row[0]) for row in rows), 'ignored': sum(int(row[2]) for row in rows)}
assert tests == {
    'scalar-default-tests': {'binaries': 1, 'passed': 768, 'ignored': 0},
    'scalar-all-tests': {'binaries': 20, 'passed': 950, 'ignored': 0},
    'curve-tests': {'binaries': 49, 'passed': 2430, 'ignored': 28},
    'drc-tests': {'binaries': 1, 'passed': 1559, 'ignored': 0},
}
integration = EVIDENCE / 'laurent-root-production-integration'
integration_status = read(integration / 'status.json')
assert len(integration_status) == 1 and integration_status[0]['exit_code'] == 0
assert read(integration / 'source-verification.json')['all_source_hashes_unchanged']
rows = re.findall(r'test result: ok\. (\d+) passed; 0 failed; (\d+) ignored', (integration / 'tests.log').read_text())
assert len(rows) == 2 and sum(int(row[0]) for row in rows) == 35 and all(int(row[1]) == 0 for row in rows)
tests['drc-integration-tests'] = {'binaries': 2, 'passed': 35, 'ignored': 0}
binary = read(OUT / 'binary.json')
assert digest(Path(binary['path'])) == binary['sha256']
verification = read(OUT / 'source-verification.json')
assert verification['source_record_sha256'] == digest(PRODUCTION / 'sources.json')
assert verification['all_candidate_and_main_source_hashes_unchanged']
log = (OUT / 'full-board-smoke.log').read_text()
assert 'skipping complex project fixture test' not in log
assert log.count('hyperdrc: finished emit report') == 2
assert 'test result: ok. 1 passed; 0 failed' in log
counts = list(map(int, re.findall(r'finished run \((\d+) active finding\(s\)', log)))
assert counts == [12, 201]
checks = [
    {'check': name, 'findings': int(count), 'wall_seconds': float(seconds)}
    for name, count, seconds in re.findall(r'finished check ([^ ]+) \((\d+) finding\(s\), ([\d.]+)s\)', log)
]
assert len(checks) == 8
time_log = (OUT / 'full-board-smoke-time.log').read_text()
resource = lambda label: re.search(re.escape(label) + r': ([^\n]+)', time_log)[1]
record = {
    'decision': 'production-only root memo omission passes correctness validation; exact report/capability replay and matched resource gates remain required before runtime admission',
    'scope': source['scope'],
    'source_record': str(PRODUCTION / 'sources.json'),
    'source_files_verified': {'production_candidate': len(hashes), 'main': len(native['source_hashes'])},
    'tracked_inputs_compared_with_committed_blobs': audit['tracked_rust_build_and_board_inputs_checked'],
    'inherited_snapshot_restoration': {'initial_audit_preserved': True, 'missing_inputs_restored_exactly': 261, 'rust_sources_restored_or_changed_by_restoration': 0},
    'only_modified_runtime_source': source['only_modified_runtime_source'],
    'tests': tests,
    'test_count_scope': 'Scalar default/all-feature invocations overlap; these are not counts of distinct tests. Curve count is the complete triangulation-enabled library/integration suite. DRC library and its two admitted integration binaries are counted separately.',
    'strict_checks': ['workspace formatting', 'scalar all-feature/all-target release Clippy', 'scalar all-feature release rustdoc', 'curve triangulation/all-target release Clippy', 'curve all-feature release rustdoc', 'DRC all-target release Clippy', 'DRC all-feature release rustdoc'],
    'board_gate': {
        'passed': True, 'worker_cpu_limit_seconds': 120,
        'worker_cpu_seconds': 'not printed on success',
        'process_cpu_seconds': round(float(resource('User time (seconds)')) + float(resource('System time (seconds)')), 2),
        'wall_elapsed': resource('Elapsed (wall clock) time (h:mm:ss or m:ss)'),
        'peak_rss_kib': int(resource('Maximum resident set size (kbytes)')),
        'counts': {'cparti-fpga': counts[0], 'hvp109a': counts[1]}, 'checks': checks,
    },
    'main_runtime_unchanged': True,
    'next_step': 'Run matched scalar time/allocation/size probes and replay all 231 retained capability witnesses plus complete twelve-check reports for both boards, preserving adverse samples and every byte.',
    'limitations': 'A successful board smoke gate is not complete report preservation or matched application performance evidence. The larger native-stroke prototype remains unadmitted and still fails its original runtime gate. This does not complete the repository pre-push matrix or any full property/target.',
}
paths = [path for path in sorted(OUT.iterdir()) if path.is_file()]
paths += [path for path in sorted(integration.iterdir()) if path.is_file()]
paths += [PRODUCTION / name for name in ['sources.json', 'committed-input-audit.json', 'initial-committed-input-audit.json', 'preparation-failure.json', 'source-delta.tar.gz', 'restored-inputs.tar.gz']]
paths += [EVIDENCE / name for name in ['reproduce_laurent_root_production_validation.py', 'reproduce_laurent_root_production_integration.py', 'analyze_laurent_root_production_validation.py']]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-66.json').write_text(json.dumps(record, indent=2) + '\n')
print('Production suites:', tests)
print('Board gate:', {key: value for key, value in record['board_gate'].items() if key != 'checks'})
