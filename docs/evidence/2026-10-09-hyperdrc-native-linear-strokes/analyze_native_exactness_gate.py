"""Preserve the full native gate result without weakening its existing limit."""
import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'native-exactness-gate'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
source = read(OUT / 'sources.json')
assert all(digest(ROOT / name) == value for name, value in source['source_hashes'].items())
assert digest(Path(source['base_source_record'])) == source['base_source_record_sha256']
assert digest(OUT / 'source-delta.tar.gz') == source['source_delta_sha256']
with tarfile.open(OUT / 'source-delta.tar.gz') as archive:
    assert set(archive.getnames()) == set(source['delta_files'])
    for name in archive.getnames():
        assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == source['source_hashes'][name]
status = read(OUT / 'status.json')
assert len(status) == 7 and all(row['exit_code'] == 0 for row in status[:-1])
assert status[-1]['name'] == 'full-board-smoke' and status[-1]['exit_code'] in [0, 101]
focused = {}
for name, expected in [('contact-index-tests', 4), ('ray-filter-unit', 2), ('ray-filter-public', 2)]:
    log = (OUT / (name + '.log')).read_text()
    rows = re.findall(r'test result: ok\. (\d+) passed; 0 failed; 0 ignored', log)
    assert len(rows) == 1 and int(rows[0]) == expected
    focused[name] = expected
binary = read(OUT / 'binary.json')
assert digest(Path(binary['path'])) == binary['sha256']
passed = status[-1]['exit_code'] == 0
log = (OUT / 'full-board-smoke.log').read_text()
assert 'skipping complex project fixture test' not in log and log.count('hyperdrc: finished emit report') == 2
worker = re.search(r'complex project KiCad smoke suite took ([\d.]+)s of worker time', log)
assert passed or (worker and float(worker[1]) > 120)
counts = list(map(int, re.findall(r'finished run \((\d+) active finding\(s\)', log)))
assert counts == [12, 197]
checks = [{'check': name, 'findings': int(count), 'wall_seconds': float(seconds)} for name, count, seconds in re.findall(r'finished check ([^ ]+) \((\d+) finding\(s\), ([\d.]+)s\)', log)]
assert len(checks) == 8
time_log = (OUT / 'full-board-smoke-time.log').read_text()
resource = lambda label: re.search(re.escape(label) + r': ([^\n]+)', time_log)[1]
record = {
    'decision': 'broader native-stroke prototype remains unadmitted pending complete source/report/resource review' if passed else 'broader native-stroke prototype remains unadmitted: original full-board runtime gate still fails',
    'scope': source['scope'],
    'revision': source['head'],
    'source_files_verified': len(source['source_hashes']),
    'source_changes_since_diagnostics_42': source['delta_files'],
    'focused_tests_passed': focused,
    'board_gate': {
        'passed': passed,
        'worker_cpu_seconds': float(worker[1]) if worker else 'not printed on success',
        'worker_cpu_limit_seconds': 120,
        'process_cpu_seconds': round(float(resource('User time (seconds)')) + float(resource('System time (seconds)')), 2),
        'wall_elapsed': resource('Elapsed (wall clock) time (h:mm:ss or m:ss)'),
        'peak_rss_kib': int(resource('Maximum resident set size (kbytes)')),
        'counts': {'cparti-fpga': counts[0], 'hvp109a': counts[1]},
        'checks': checks,
    },
    'next_step': 'Evaluate sharing of closed polynomial values in the existing per-call normal-form memo. Preserve opaque atom ownership, memo eligibility, exact coefficient order and all recognition frontiers; require independent identities, matched resource evidence and the original complete fixture.',
    'limitations': 'This run repeats only the eight newly admitted exactness regressions and the original board gate on the complete native prototype. It does not reuse or claim full native suites. Single gate timings are not a matched speedup measurement. Complete source/report oracle audits, resource gates, readiness snapshots and all remaining properties/targets stay open.',
}
paths = [path for path in sorted(OUT.iterdir()) if path.is_file()]
paths += [EVIDENCE / name for name in ['reproduce_native_exactness_gate.py', 'analyze_native_exactness_gate.py']]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-59.json').write_text(json.dumps(record, indent=2) + '\n')
print('Focused regressions:', focused)
print('Board gate:', {key: value for key, value in record['board_gate'].items() if key != 'checks'})
