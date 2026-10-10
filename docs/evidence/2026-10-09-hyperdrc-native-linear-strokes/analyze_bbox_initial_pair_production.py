"""Validate the production-isolated source audit, suites and unchanged board gate."""
import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'bbox-initial-pair-production'
SOURCE = ROOT / 'target/bbox-initial-pair-production/source'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
source = read(OUT / 'sources.json')
assert all(digest(SOURCE / name) == value for name, value in source['source_hashes'].items())
audit = read(OUT / 'committed-source-audit.json')
assert audit['all_baseline_inputs_match_git'] and audit['all_package_rust_inputs_accounted_for']
assert audit['candidate_existing_differences'] == ['crates/hypercurve/src/bbox.rs']
assert audit['candidate_new_sources'] == ['crates/hypercurve/tests/bbox_initial_pair.rs']
assert audit['checked_committed_inputs'] == len(audit['committed_source_hashes']) == 990
assert all(digest(SOURCE / name) == value for name, value in audit['committed_source_hashes'].items() if name not in audit['candidate_existing_differences'])
with tarfile.open(OUT / 'source-delta.tar.gz') as archive:
    assert set(archive.getnames()) == set(source['only_modified_sources'])
    for name in archive.getnames():
        assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == source['source_hashes'][name]
status = read(OUT / 'status.json')
assert len(status) == 8 and all(command['exit_code'] == 0 for command in status)
suites = {}
for name in ['curve-tests', 'drc-tests']:
    log = (OUT / (name + '.log')).read_text()
    rows = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored', log)
    assert rows and all(int(row[1]) == 0 for row in rows)
    suites[name] = {'test_binaries': len(rows), 'passed': sum(int(row[0]) for row in rows), 'failed': 0, 'ignored': sum(int(row[2]) for row in rows)}
assert suites['curve-tests']['passed'] == 2422 and suites['curve-tests']['ignored'] == 28
assert suites['drc-tests']['passed'] == 1559 and suites['drc-tests']['ignored'] == 0
binary = read(OUT / 'binary.json')
assert digest(Path(binary['path'])) == binary['sha256']
log = (OUT / 'full-board-smoke.log').read_text()
assert 'skipping complex project fixture test' not in log
assert log.count('hyperdrc: finished emit report') == 2
assert 'test result: ok. 1 passed; 0 failed; 0 ignored' in log
counts = list(map(int, re.findall(r'finished run \((\d+) active finding\(s\)', log)))
assert counts == [12, 201]
time = (OUT / 'full-board-smoke-time.log').read_text()
number = lambda pattern: float(re.search(pattern, time)[1])
board = {'passed': True, 'original_worker_cpu_limit_seconds': 120, 'worker_cpu_seconds': 'not printed on success', 'process_user_seconds': number(r'User time \(seconds\): ([0-9.]+)'), 'process_system_seconds': number(r'System time \(seconds\): ([0-9.]+)'), 'maximum_rss_kib': int(number(r'Maximum resident set size \(kbytes\): (\d+)')), 'counts': {'cparti-fpga': counts[0], 'hvp109a': counts[1]}}
board['process_cpu_seconds'] = board['process_user_seconds'] + board['process_system_seconds']
record = {
    'decision': 'isolated production correctness and runtime gates pass; matched production costs and complete report replay remain pending before retention',
    'production_baseline_revision': source['production_baseline_revision'],
    'only_changed_sources': source['only_modified_sources'],
    'source_hashes_verified': len(source['source_hashes']),
    'committed_inputs_checked': audit['checked_committed_inputs'],
    'package_roots': audit['package_roots'],
    'suites': suites, 'original_board_gate': board,
    'format_strict_clippy_and_docs_passed': True,
    'earlier_focused_costs': ['diagnostics-38.json', 'diagnostics-39.json'],
    'unimplemented_followup_candidates': 'comparison-optimization-candidates.json',
    'limitations': 'The full curve suite retains 28 existing ignored cases. This is not the complete workspace/pre-push matrix. The separate native-stroke prototype and its failed runtime gate remain unadmitted. No full-application speedup, cold-start improvement or properties.md completion is claimed.',
}
paths = [path for path in sorted(OUT.iterdir()) if path.is_file()]
paths += [EVIDENCE / 'reproduce_bbox_initial_pair_production.py', EVIDENCE / 'comparison-optimization-candidates.json', Path(__file__)]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-40.json').write_text(json.dumps(record, indent=2) + '\n')
print('Production suites:', suites)
print('Original board gate:', board)
