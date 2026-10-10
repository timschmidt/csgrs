"""Record the native candidate's broader tests and unchanged board gate."""
import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'laurent-root-native-validation'
CANDIDATE = EVIDENCE / 'laurent-root-memo-omission'
SOURCE = ROOT / 'target/laurent-root-memo-omission/candidate/source'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
source = read(CANDIDATE / 'candidate-sources.json')
native = read(Path(source['native_source_record']))
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
assert all(digest(SOURCE / name) == value for name, value in source['source_hashes'].items())
with tarfile.open(CANDIDATE / 'candidate-source-delta.tar.gz') as archive:
    assert set(archive.getnames()) == set(source['only_modified_sources'])
    for name in archive.getnames():
        assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == source['source_hashes'][name]
retained = read(EVIDENCE / 'laurent-root-memo-omission-cost/retained-libraries.json')
assert all(digest(Path(row['retained_path'])) == row['sha256'] for row in retained['libraries'].values())
status = read(OUT / 'status.json')
assert [row['name'] for row in status] == [
    'scalar-tests', 'curve-focused', 'curve-clippy', 'build-board-smoke',
    'drc-clippy', 'full-board-smoke',
]
assert all(row['exit_code'] == 0 for row in status[:-1])
assert status[-1]['exit_code'] in [0, 101]
tests = {}
for name in ['scalar-tests', 'curve-focused']:
    log = (OUT / (name + '.log')).read_text()
    rows = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored', log)
    assert rows and all(int(failed) == 0 for _, failed, _ in rows)
    tests[name] = {
        'binaries': len(rows), 'passed': sum(int(row[0]) for row in rows),
        'ignored': sum(int(row[2]) for row in rows),
    }
assert tests == {
    'scalar-tests': {'binaries': 19, 'passed': 880, 'ignored': 0},
    'curve-focused': {'binaries': 3, 'passed': 12, 'ignored': 0},
}
binary = read(OUT / 'binary.json')
assert digest(Path(binary['path'])) == binary['sha256']
passed = status[-1]['exit_code'] == 0
log = (OUT / 'full-board-smoke.log').read_text()
assert 'skipping complex project fixture test' not in log
assert log.count('hyperdrc: finished emit report') == 2
worker = re.search(r'complex project KiCad smoke suite took ([\d.]+)s of worker time', log)
assert passed or (worker and float(worker[1]) > 120)
counts = list(map(int, re.findall(r'finished run \((\d+) active finding\(s\)', log)))
assert counts == [12, 197]
checks = [
    {'check': name, 'findings': int(count), 'wall_seconds': float(seconds)}
    for name, count, seconds in re.findall(
        r'finished check ([^ ]+) \((\d+) finding\(s\), ([\d.]+)s\)', log
    )
]
assert len(checks) == 8
time_log = (OUT / 'full-board-smoke-time.log').read_text()
resource = lambda label: re.search(re.escape(label) + r': ([^\n]+)', time_log)[1]
record = {
    'decision': 'root-entry omission passes broader native correctness checks; production isolation and output/resource gates still required',
    'candidate': 'diagnostics-64.json',
    'native_baseline_gate': 'diagnostics-59.json',
    'scope': source['scope'],
    'source_files_verified': {'main': len(native['source_hashes']), 'candidate': len(source['source_hashes'])},
    'measured_microbenchmark_libraries_retained_and_verified': True,
    'tests': tests,
    'strict_release_all_target_clippy_passed': ['hypercurve', 'hyperdrc'],
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
    'main_runtime_unchanged': True,
    'next_step': 'Isolate only the root-entry omission against admitted production source, then validate scalar/curve/DRC behavior, historical capability witnesses, complete report bytes and scoped resource costs before retention.',
    'limitations': 'The scalar run enables serde; it is not an all-feature matrix. Only three curve integration binaries were run. Full native curve/DRC suites and complete native source/report/resource review remain open. A single board run is not a matched speedup measurement. The board limit is unchanged, and a failure remains a failed gate. The complete properties.md objective remains open.',
}
paths = [path for path in sorted(OUT.iterdir()) if path.is_file()]
paths += [EVIDENCE / name for name in [
    'reproduce_laurent_root_native_validation.py', 'analyze_laurent_root_native_validation.py',
    'laurent-root-memo-omission/candidate-sources.json',
    'laurent-root-memo-omission/candidate-source-delta.tar.gz',
    'laurent-root-memo-omission-cost/retained-libraries.json',
]]
record['artifacts'] = {
    str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)}
    for path in paths
}
(EVIDENCE / 'diagnostics-65.json').write_text(json.dumps(record, indent=2) + '\n')
print('Tests:', tests)
print('Board gate:', {key: value for key, value in record['board_gate'].items() if key != 'checks'})
