"""Preserve native correctness and original resource-gate outcomes without admission."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'arc-endpoint-native'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
records = {variant: read(OUT / (variant + '-sources.json')) for variant in ['baseline', 'candidate']}
maps = {}
for variant, record in records.items():
    path = Path(record['source_hashes_file'])
    assert digest(path) == record['source_hashes_file_sha256']
    maps[variant] = json.loads(gzip.decompress(path.read_bytes()))
    assert all(digest(Path(record['source_directory']) / name) == value for name, value in maps[variant].items())
native = read(EVIDENCE / 'native-exactness-gate/sources.json')['source_hashes']
assert all(digest(ROOT / name) == value for name, value in native.items())
changed = [name for name in maps['baseline'] if maps['baseline'][name] != maps['candidate'][name]]
assert changed == ['crates/hypercurve/src/segment.rs']
with tarfile.open(OUT / 'candidate-source-delta.tar.gz') as archive:
    assert archive.getnames() == changed
    assert hashlib.sha256(archive.extractfile(changed[0]).read()).hexdigest() == maps['candidate'][changed[0]]
status = read(OUT / 'status.json')
steps = ['format', 'curve-clippy', 'point-regressions', 'segment-regressions', 'curve-integrations', 'drc-native-regressions', 'drc-clippy', 'build-board-smoke']
assert [(row['variant'], row['name']) for row in status] == [(variant, name) for variant in records for name in steps] + [(variant, 'full-board-smoke') for variant in records]
assert all(row['exit_code'] == 0 for row in status[:-2])
assert read(OUT / 'source-verification.json')['all_source_and_binary_hashes_unchanged']
binaries = read(OUT / 'binaries.json')
focused, gates = {}, {}
for variant in records:
    focused[variant] = {}
    for name, expected in [('point-regressions', 14), ('segment-regressions', 7), ('curve-integrations', 25), ('drc-native-regressions', 26)]:
        rows = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored', (OUT / (variant + '-' + name + '.log')).read_text())
        counts = [sum(int(row[column]) for row in rows) for column in range(3)]
        assert counts == [expected, 0, 0]
        focused[variant][name] = {'passed': expected, 'binaries': len(rows), 'ignored': 0}
    assert digest(Path(binaries[variant]['path'])) == binaries[variant]['sha256']
    row = next(row for row in status if row['variant'] == variant and row['name'] == 'full-board-smoke')
    assert row['exit_code'] in [0, 101]
    raw = (OUT / (variant + '-full-board-smoke.log')).read_text()
    assert 'skipping complex project fixture test' not in raw and raw.count('hyperdrc: finished emit report') == 2
    worker = re.search(r'complex project KiCad smoke suite took ([\d.]+)s of worker time', raw)
    assert row['exit_code'] == 0 or (worker and float(worker[1]) >= 120)
    counts = list(map(int, re.findall(r'finished run \((\d+) active finding\(s\)', raw)))
    assert counts == [12, 197]
    checks = [{'check': name, 'findings': int(count), 'wall_seconds': float(seconds)} for name, count, seconds in re.findall(r'finished check ([^ ]+) \((\d+) finding\(s\), ([\d.]+)s\)', raw)]
    assert len(checks) == 8
    time_log = (OUT / (variant + '-full-board-smoke-time.log')).read_text()
    resource = lambda label: re.search(re.escape(label) + r': ([^\n]+)', time_log)[1]
    gates[variant] = {
        'passed': row['exit_code'] == 0, 'exit_code': row['exit_code'],
        'worker_cpu_limit_seconds': 120, 'worker_cpu_seconds': float(worker[1]) if worker else 'not printed on success',
        'process_cpu_seconds': round(float(resource('User time (seconds)')) + float(resource('System time (seconds)')), 2),
        'wall_elapsed': resource('Elapsed (wall clock) time (h:mm:ss or m:ss)'),
        'peak_rss_kib': int(resource('Maximum resident set size (kbytes)')),
        'counts': {'cparti-fpga': counts[0], 'hvp109a': counts[1]}, 'checks': checks,
        'executable_bytes': binaries[variant]['bytes'],
    }
assert [(row['check'], row['findings']) for row in gates['baseline']['checks']] == [(row['check'], row['findings']) for row in gates['candidate']['checks']]
comparison = {key + '_candidate_over_baseline': gates['candidate'][key] / gates['baseline'][key] for key in ['process_cpu_seconds', 'peak_rss_kib', 'executable_bytes']}
if all(isinstance(gate['worker_cpu_seconds'], float) for gate in gates.values()):
    comparison['worker_cpu_seconds_candidate_over_baseline'] = gates['candidate']['worker_cpu_seconds'] / gates['baseline']['worker_cpu_seconds']
record = {
    'decision': 'native focused checks and unchanged board outcomes recorded; final cross-workload tradeoff review remains required and no runtime change is admitted',
    'scope': 'One fresh complete native board-smoke process per variant after sequential focused validation and all builds. The only runtime difference is previously tested exact-negative arc endpoint certificate reuse. Original native prototype, board fixtures, selected checks and 120-second worker CPU assertion are unchanged.',
    'source_files_verified': {variant: len(hashes) for variant, hashes in maps.items()} | {'main': len(native)},
    'focused_tests': focused,
    'strict_checks_each_variant': ['workspace formatting', 'curve triangulation/all-target release Clippy', 'DRC all-target release Clippy'],
    'board_gates': gates, 'descriptive_single_pair': comparison,
    'main_runtime_unchanged': True, 'native_admission': False,
    'preceding_production_gates': ['diagnostics-77.json', 'diagnostics-78.json', 'diagnostics-79.json', 'diagnostics-80.json', 'diagnostics-81.json', 'diagnostics-82.json'],
    'limitations': 'A single baseline-then-candidate pair has no uncertainty estimate and is not a robust speedup claim. Matching smoke findings/check counts do not establish complete native report-byte identity or full native correctness. No full native suites, new scalar suite or repository pre-push matrix are claimed. Broad production workload slowdowns remain part of retention review; no further sampling extension of those binaries is planned. All property/target and native-source coverage gaps remain open.',
}
paths = [path for path in sorted(OUT.iterdir()) if path.is_file()]
paths += [EVIDENCE / name for name in ['prepare_arc_endpoint_native.py', 'reproduce_arc_endpoint_native.py', 'analyze_arc_endpoint_native.py']]
paths += sorted((EVIDENCE / 'arc-endpoint-native-preparation-correction').iterdir())
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-83.json').write_text(json.dumps(record, indent=2) + '\n')
print('Focused tests:', focused)
print('Board gates:', {variant: {key: value for key, value in gate.items() if key != 'checks'} for variant, gate in gates.items()})
print('Single-pair ratios:', comparison)
