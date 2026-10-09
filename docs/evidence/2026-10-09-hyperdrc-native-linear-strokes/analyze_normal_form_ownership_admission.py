"""Verify the finished isolated ownership trial and write its admission record.

Run from the repository root after every validation and measurement has exited.
The successful replay's compressed reports are retained; executable artifacts
may be removed only after checking the hashes recorded by their drivers.
"""
import gzip
import hashlib
import json
from pathlib import Path
import re
import statistics
import subprocess
import tarfile

root = Path(__file__).resolve().parent
repo = Path.cwd()
read = lambda path: json.loads(path.read_text())
digest = lambda data: hashlib.sha256(data).hexdigest()
info = lambda path: dict(bytes=path.stat().st_size, sha256=digest(path.read_bytes()))
validation = root / 'normal-form-ownership-isolated'
cost = root / 'normal-form-ownership-production-cost'
replay = root / 'normal-form-ownership-production-replay-corrected'
baseline = read(root / 'circular-support-guard-fix-isolated/sources.json')
candidate = read(validation / 'sources.json')
name = 'crates/hyperreal/src/computable/node/algebra.rs'
assert candidate['unchanged']
assert baseline['source_hashes'].keys() == candidate['source_hashes'].keys()
assert [p for p, h in candidate['source_hashes'].items()
        if h != baseline['source_hashes'][p]] == [name]
assert digest((repo / name).read_bytes()) == candidate['source_hashes'][name]
with tarfile.open(validation / 'source-delta.tar.gz') as archive:
    assert archive.getnames() == [name]
    assert digest(archive.extractfile(name).read()) == candidate['source_hashes'][name]
statuses = read(validation / 'status.json')
assert len(statuses) == 12 and all(row['exit_code'] == 0 for row in statuses)
tests = {}
for path in validation.glob('*.log'):
    matches = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored', path.read_text())
    if matches:
        tests[path.name] = {key: sum(int(row[i]) for row in matches)
                            for i, key in enumerate(['passed', 'failed', 'ignored'])}
board_log = (validation / 'full-board-smoke.log').read_text()
assert 'skipping complex project fixture test' not in board_log
assert board_log.count('hyperdrc: finished emit report') == 2
assert all(row['failed'] == row['ignored'] == 0 for row in tests.values())
board_time = (validation / 'full-board-smoke-time.log').read_text()
number = lambda label: float(re.search(re.escape(label) + r': ([\d.]+)', board_time)[1])
costs = read(cost / 'manifest.json')
assert costs['source_unchanged'] and costs['source_change'] == [name]
assert len(costs['commands']) == 28 and all(row['exit_code'] == 0 for row in costs['commands'])
assert len(costs['trials']) == 20
for mode in ['time', 'memory']:
    trials = [t for t in costs['trials'] if t['mode'] == mode]
    assert len(trials) == 10
    witness = lambda trial: [r for r in trial['rows'] if 'witness' in r or 'fixture' in r]
    assert len(witness(trials[0])) == 24
    assert all(witness(t) == witness(trials[0]) for t in trials)
memory = [row for row in costs['summaries'] if row['mode'] == 'memory']
assert len(memory) == 24
assert all(len(set(field['samples'])) == 1
           for row in memory for variant in row['variants'].values() for field in variant.values())
for row in memory:
    before, after = (row['variants'][v] for v in ['baseline', 'candidate'])
    assert before['retained_bytes_delta'] == after['retained_bytes_delta']
    if row['name'].endswith('_construction'):
        assert before == after
oracle = read(root / 'normal-form-ownership-oracle.json')
assert oracle['conjugate_reciprocal_verified'] and oracle['identities'] == len(oracle['rows']) == 126
totals = {}
for variant in ['baseline', 'candidate']:
    samples = [sum(r['nanos'] for r in t['rows'] if 'nanos' in r)
               for t in costs['trials'] if t['mode'] == 'time' and t['variant'] == variant]
    totals[variant] = dict(samples_ns=samples, median_ns=statistics.median(samples))
replayed = read(replay / 'manifest.json')
assert replayed['source_unchanged'] and replayed['only_modified_source'] == name
assert len(replayed['commands']) == 20
assert all(row['exit_code'] == 0 for row in replayed['commands'])
assert len(replayed['cost_trials']) == 10
rows = read(replay / 'rows.json')
assert len(rows) == 231 and info(replay / 'rows.json') == replayed['canonical_rows']
for trial in replayed['cost_trials']:
    assert [{k: v for k, v in row.items() if k != 'elapsed_ns'} for row in trial['rows']] == rows
for board, report in replayed['reports'].items():
    blobs = [gzip.decompress((replay / f'{v}-reports-{i}-{board}.json.gz').read_bytes())
             for v in ['baseline', 'candidate'] for i in [1, 2]]
    assert all(blob == blobs[0] for blob in blobs)
    assert len(blobs[0]) == report['bytes'] and digest(blobs[0]) == report['sha256']
    parsed = json.loads(blobs[0])
    assert len(parsed['violations']) == report['violation_count']
record = dict(
    decision='kept',
    scope='Only the three ownership changes in bounded Laurent/radical recognition are admitted. All other native-source prototypes remain provisional.',
    previous_diagnostic='diagnostics-16.json',
    production_baseline_revision='86b6212d9',
    source_change=dict(path=name, before_sha256=baseline['source_hashes'][name],
                       after_sha256=candidate['source_hashes'][name],
                       baseline_bytes=len(subprocess.check_output(['git', 'show', '86b6212d9:' + name])),
                       candidate_bytes=(repo / name).stat().st_size),
    contract='normal-form-ownership-contract.json',
    implementation='Move uniquely owned coefficients instead of copying their vectors during scaling, multiplication by one term, and inversion. Arithmetic/insertion order, recognition limits, fallback, API, layout, persistent caches and decision strata are unchanged.',
    validation=dict(command_count=len(statuses), tests=tests, source_files=len(candidate['source_hashes']),
                    passed_test_invocations_excluding_board=sum(t['passed'] for p, t in tests.items() if p != 'full-board-smoke.log'),
                    full_board_original_120_worker_cpu_second_gate_passed=True,
                    full_board_worker_cpu_seconds=None,
                    full_board_process_user_seconds=number('User time (seconds)'),
                    full_board_process_system_seconds=number('System time (seconds)'),
                    full_board_maximum_rss_kib=int(number('Maximum resident set size (kbytes)'))),
    independent_oracle='normal-form-ownership-oracle.json',
    independently_derived_identities=oracle['identities'],
    exact_query_witnesses_equal=True,
    allocation_samples_repeat_identically=True,
    first_query_allocations=[row for row in memory if row['name'].endswith('_first')],
    focused_time_totals=totals,
    binaries={k: {field: v[field] for field in ['bytes', 'sha256']} for k, v in costs['binaries'].items()},
    libraries=costs['libraries'],
    canonical_capability_rows=231,
    historical_row_deltas=replayed['historical_row_deltas'],
    complete_reports=replayed['reports'],
    timing_claim='No broad speedup or statistical significance is claimed. Short operation timings vary, including unchanged construction and already-rational query controls.',
    allocation_limit='Counts requested Rust allocation bytes/operations, excluding allocator metadata and native C/GMP heaps. Four first-query families reduce traffic; two are unchanged. Construction and retained bytes are unchanged.',
    tooling_failures=['Initial isolated preparation omitted the hash helper before validation; corrected before checks.',
                      'Initial scalar probe used i32 instead of i64; corrected before measurements.',
                      'Initial production replay used release instead of release/deps as the dependency search path; corrected before measurements.'],
    remaining='The combined native-source candidate still fails the unchanged 120 worker CPU second gate (209.577392864 seconds). Its report changes and broader admission remain unresolved. All 39 properties, 14 targets and crate future work remain in scope.',
)
directories = [validation, cost, root / 'normal-form-ownership-production-replay', replay]
record['artifacts'] = {str(p.relative_to(root)): info(p)
                       for directory in directories for p in sorted(directory.rglob('*')) if p.is_file()}
record['artifacts'][Path(__file__).name] = info(Path(__file__))
record['artifacts']['normal-form-ownership-oracle.json'] = info(root / 'normal-form-ownership-oracle.json')
(root / 'diagnostics-17.json').write_text(json.dumps(record, indent=2) + '\n')
print('isolated normal-form ownership admission verified')
