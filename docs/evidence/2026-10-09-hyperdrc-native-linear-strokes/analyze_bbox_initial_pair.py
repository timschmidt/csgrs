"""Validate the isolated initial-pair trial and its complete paired measurements."""
import hashlib
import json
from pathlib import Path
import re
import statistics
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
CANDIDATE = EVIDENCE / 'bbox-initial-pair-candidate'
COST = EVIDENCE / 'bbox-initial-pair-cost-corrected'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
source = read(CANDIDATE / 'sources.json')
native = read(Path(source['native_source_record']))
isolated = ROOT / 'target/native-bbox-initial-pair/source'
assert all(digest(isolated / name) == value for name, value in source['source_hashes'].items())
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
with tarfile.open(CANDIDATE / 'source-delta.tar.gz') as archive:
    assert set(archive.getnames()) == set(source['only_changed_sources'])
    for name in archive.getnames():
        assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == source['source_hashes'][name]
status = read(CANDIDATE / 'status.json')
assert len(status) == 6 and all(command['exit_code'] == 0 for command in status)
tests = {}
for name, expected in [('bbox-unit', 7), ('bbox-public', 12), ('curve-regressions', 10)]:
    counts = re.findall(r'test result: ok\. (\d+) passed; 0 failed; 0 ignored', (CANDIDATE / (name + '.log')).read_text())
    tests[name] = sum(map(int, counts))
    assert tests[name] == expected
cost = read(COST / 'manifest.json')
assert len(cost['commands']) == 29 and all(command['exit_code'] == 0 for command in cost['commands'])
assert len(cost['trials']) == 20 and len(cost['summaries']) == 192
assert cost['probe_sha256'] == digest(COST / 'probe.rs') == digest(EVIDENCE / 'bbox_initial_pair_probe.rs')
assert all(digest(Path(row['path'])) == row['sha256'] for row in cost['libraries'].values())
assert all(digest(COST / name) == row['sha256'] for name, row in cost['binaries'].items())
for trial in cost['trials']:
    count = 1024 if trial['mode'] == 'time' else 256
    assert len(trial['rows']) == 168
    measurements = [row for row in trial['rows'] if 'name' in row]
    assert len(measurements) == 96 and all(row['iterations'] == count for row in measurements)
    witnesses = [row for row in trial['rows'] if 'witness' in row]
    assert len(witnesses) == 72 and all(row['decided'] == count for row in witnesses)
for mode in ['time', 'memory']:
    matching = [trial for trial in cost['trials'] if trial['mode'] == mode]
    witnesses = [row for row in matching[0]['rows'] if 'witness' in row]
    assert all([row for row in trial['rows'] if 'witness' in row] == witnesses for trial in matching)
for summary in cost['summaries']:
    for variant, fields in summary['variants'].items():
        rows = [next(row for row in trial['rows'] if row.get('name') == summary['name']) for trial in cost['trials'] if trial['mode'] == summary['mode'] and trial['variant'] == variant]
        assert len(rows) == 5
        for field, stats in fields.items():
            assert stats['samples'] == [row[field] for row in rows]
            assert stats['median'] == statistics.median(stats['samples'])
            if summary['mode'] == 'memory': assert len(set(stats['samples'])) == 1

timings = []
for summary in cost['summaries']:
    if summary['mode'] != 'time': continue
    old = summary['variants']['baseline']['nanos']['median']
    new = summary['variants']['candidate']['nanos']['median']
    timings.append({'name': summary['name'], 'baseline_ns_per_box': old / 1024, 'candidate_ns_per_box': new / 1024, 'candidate_over_baseline': new / old})
affected = [row for row in timings if '_construction' not in row['name'] ]
controls = [row for row in timings if row not in affected]
allocations = []
for summary in cost['summaries']:
    if summary['mode'] != 'memory': continue
    row = {'name': summary['name']}
    for variant in ['baseline', 'candidate']:
        row[variant] = {field: stats['median'] for field, stats in summary['variants'][variant].items()}
    if row['baseline'] != row['candidate']: allocations.append(row)
size = {}
for name, row in cost['binaries'].items():
    sections = {line.split()[0]: int(line.split()[1]) for line in row['sections'].splitlines() if line.startswith('.')}
    size[name] = {'file_bytes': row['bytes'], 'text_bytes': sections['.text'], 'rodata_bytes': sections['.rodata']}
failure = read(EVIDENCE / 'bbox-initial-pair-cost/failure.json')
assert all(digest(EVIDENCE / 'bbox-initial-pair-cost' / name) == row['sha256'] for name, row in failure['artifacts'].items())
record = {
    'decision': 'focused isolated correctness passes; initial costs show broad query improvements and unresolved short rational-pair timing, so fresh-process focused measurements and production admission remain pending',
    'tests_passed': tests, 'format_and_strict_clippy_passed': True,
    'independent_extrema_matrix': {'boxes': 864, 'exact_extremum_assertions': 3456, 'coverage': 'Two policies, four signed/extreme scales, cold/warm input intervals, all six three-point permutations and nine independent coordinate/tie patterns. Additional empty/singleton/duplicate/near-coincident rational controls.'},
    'matched_probe': {'trials': 20, 'witness_rows_per_process': 72, 'all_witnesses_identical': True, 'memory_samples_repeat_exactly': True},
    'affected_query_timing_rows': affected, 'unchanged_construction_timing_controls': controls,
    'changed_allocation_rows': allocations, 'binary_sizes': size,
    'source_unchanged': True, 'main_runtime_unchanged': True,
    'next_step': 'Repeat the short rational two-point scenario alone in fresh processes at the original and a larger batch size; also recheck the small pi eight-point cloned timing difference. Earlier singleton allocations can affect later allocator state, but that explanation remains a hypothesis pending the focused run.',
    'original_harness_failure': 'bbox-initial-pair-cost/failure.json',
    'limitations': 'Only bounding-box initialization changes. No persistent cache, scalar proof-budget change, full-application speedup, broad native admission or property-completion claim. Timing controls and retained-memory costs must be considered alongside affected query rows. Production isolation and downstream validation remain required before retention.',
}
paths = [path for folder in [CANDIDATE, COST, EVIDENCE / 'bbox-initial-pair-cost'] for path in sorted(folder.iterdir()) if path.is_file() and path.name not in cost['binaries']]
paths += [EVIDENCE / name for name in ['bbox_initial_pair_regressions.rs', 'bbox_initial_pair_probe.rs', 'reproduce_bbox_initial_pair_candidate.py', 'reproduce_bbox_initial_pair_cost.py', 'analyze_bbox_initial_pair.py']]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-38.json').write_text(json.dumps(record, indent=2) + '\n')
print('Focused tests:', sum(tests.values()))
print('Affected query rows:', len(affected), 'median-ratio range:', min(row['candidate_over_baseline'] for row in affected), max(row['candidate_over_baseline'] for row in affected))
print('Affected regressions:', [row for row in affected if row['candidate_over_baseline'] > 1])
print('Changed allocation rows:', len(allocations))
print('Binary sizes:', size)
