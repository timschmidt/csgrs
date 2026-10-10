"""Audit scoped endpoint production validation before complete report replay."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
PRODUCTION = EVIDENCE / 'arc-endpoint-certificate'
OUT = EVIDENCE / 'arc-endpoint-production-validation'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
records = {variant: read(PRODUCTION / (variant + '-sources.json')) for variant in ['baseline', 'candidate']}
maps = {}
for variant, record in records.items():
    path = Path(record['source_hashes_file'])
    assert digest(path) == record['source_hashes_file_sha256']
    maps[variant] = json.loads(gzip.decompress(path.read_bytes()))
    source = Path(record['source_directory'])
    assert all(digest(source / name) == value for name, value in maps[variant].items())
native = read(EVIDENCE / 'native-exactness-gate/sources.json')['source_hashes']
assert all(digest(ROOT / name) == value for name, value in native.items())
changed = [name for name in maps['baseline'] if maps['baseline'][name] != maps['candidate'][name]]
assert changed == ['crates/hypercurve/src/segment.rs']
with tarfile.open(PRODUCTION / 'candidate-source-delta.tar.gz') as archive:
    assert archive.getnames() == changed
    for name in changed:
        assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == maps['candidate'][name]
status = read(OUT / 'status.json')
assert [row['name'] for row in status] == ['format', 'curve-clippy', 'curve-tests', 'curve-doc', 'build-drc-tests', 'drc-tests', 'drc-integration-tests', 'drc-clippy', 'drc-doc', 'full-board-smoke']
assert all(row['exit_code'] == 0 for row in status)
tests = {}
for name in ['curve-tests', 'drc-tests', 'drc-integration-tests']:
    raw = (OUT / (name + '.log')).read_text()
    rows = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored', raw)
    assert rows and all(int(row[1]) == 0 for row in rows)
    tests[name] = {'binaries': len(rows), 'passed': sum(int(row[0]) for row in rows), 'ignored': sum(int(row[2]) for row in rows)}
assert tests == {'curve-tests': {'binaries': 49, 'passed': 2430, 'ignored': 28}, 'drc-tests': {'binaries': 1, 'passed': 1559, 'ignored': 0}, 'drc-integration-tests': {'binaries': 2, 'passed': 35, 'ignored': 0}}
binary = read(OUT / 'binary.json')
assert digest(Path(binary['path'])) == binary['sha256']
verification = read(OUT / 'source-verification.json')
assert verification['source_record_sha256'] == digest(PRODUCTION / 'candidate-sources.json')
assert verification['all_production_and_main_source_hashes_unchanged']
raw = (OUT / 'full-board-smoke.log').read_text()
assert 'skipping complex project fixture test' not in raw
assert raw.count('hyperdrc: finished emit report') == 2
assert 'test result: ok. 1 passed; 0 failed' in raw
counts = list(map(int, re.findall(r'finished run \((\d+) active finding\(s\)', raw)))
assert counts == [12, 201]
checks = [{'check': name, 'findings': int(count), 'wall_seconds': float(seconds)} for name, count, seconds in re.findall(r'finished check ([^ ]+) \((\d+) finding\(s\), ([\d.]+)s\)', raw)]
assert len(checks) == 8
time_log = (OUT / 'full-board-smoke-time.log').read_text()
resource = lambda label: re.search(re.escape(label) + r': ([^\n]+)', time_log)[1]
record = {
    'decision': 'production endpoint candidate passes full scoped correctness and original board gate; complete capability/report replay remains required before runtime admission',
    'scope': 'Isolated admitted production with only local exact-negative arc endpoint certificate reuse; no test-only diagnostic Rust.',
    'source_files_verified': {'production_baseline': len(maps['baseline']), 'production_candidate': len(maps['candidate']), 'main': len(native)},
    'only_runtime_change': changed[0], 'tests': tests,
    'strict_checks': ['workspace formatting', 'curve triangulation/all-target release Clippy', 'curve all-feature release rustdoc', 'DRC all-target release Clippy', 'DRC all-feature release rustdoc'],
    'board_gate': {'passed': True, 'worker_cpu_limit_seconds': 120, 'worker_cpu_seconds': 'not printed on success', 'process_cpu_seconds': round(float(resource('User time (seconds)')) + float(resource('System time (seconds)')), 2), 'wall_elapsed': resource('Elapsed (wall clock) time (h:mm:ss or m:ss)'), 'peak_rss_kib': int(resource('Maximum resident set size (kbytes)')), 'counts': {'cparti-fpga': counts[0], 'hvp109a': counts[1]}, 'checks': checks},
    'earlier_scoped_gates': ['diagnostics-77.json', 'diagnostics-78.json'],
    'main_runtime_unchanged': True,
    'next_step': 'Add a focused effective-final-coordinate abort probe after this process exits; the earlier abort flag was attached before coordinate construction. Then replay all 231 retained historical capability witnesses in five fresh processes per variant and all bytes of the original complete twelve-check reports in two fresh processes per variant; preserve every discrepancy.',
    'limitations': 'The board smoke gate is not complete report preservation or a matched application speedup. No fresh scalar suite is claimed because scalar runtime is unchanged. The larger native-stroke prototype remains unadmitted and requires a separate recheck. This does not complete the repository pre-push matrix, explicit shared-work-budget sweep, or any entire property/target.',
}
paths = [path for path in sorted(OUT.iterdir()) if path.is_file()]
paths += [EVIDENCE / name for name in ['reproduce_arc_endpoint_production_validation.py', 'analyze_arc_endpoint_production_validation.py']]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-79.json').write_text(json.dumps(record, indent=2) + '\n')
print('Production suites:', tests)
print('Board gate:', {key: value for key, value in record['board_gate'].items() if key != 'checks'})
