"""Validate the isolated initial-pair trial and its complete paired measurements."""
import hashlib
import json
from pathlib import Path
import re
import statistics
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
CANDIDATE = EVIDENCE / 'bbox-initial-pair-production'
COST = EVIDENCE / 'bbox-initial-pair-production-cost'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
source = read(CANDIDATE / 'sources.json')
native = read(Path(source['main_source_record']))
isolated = ROOT / 'target/bbox-initial-pair-production/source'
assert all(digest(isolated / name) == value for name, value in source['source_hashes'].items())
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
with tarfile.open(CANDIDATE / 'source-delta.tar.gz') as archive:
    assert set(archive.getnames()) == set(source['only_modified_sources'])
    for name in archive.getnames():
        assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == source['source_hashes'][name]
status = read(CANDIDATE / 'status.json')
assert len(status) == 8 and all(command['exit_code'] == 0 for command in status)
validation = read(EVIDENCE / 'diagnostics-40.json')
assert validation['original_board_gate']['passed'] and validation['format_strict_clippy_and_docs_passed']
tests = {name: row['passed'] for name, row in validation['suites'].items()}
cost = read(COST / 'manifest.json')
assert len(cost['commands']) == 30 and all(command['exit_code'] == 0 for command in cost['commands'])
assert len(cost['trials']) == 20 and len(cost['summaries']) == 192
assert cost['probe_sha256'] == digest(COST / 'probe.rs') == digest(EVIDENCE / 'bbox_initial_pair_production_probe.rs')
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
replay_folder = EVIDENCE / 'bbox-initial-pair-production-replay'
replay = read(replay_folder / 'manifest.json')
assert len(replay['commands']) == 20 and all(command['exit_code'] == 0 for command in replay['commands'])
assert len(replay['cost_trials']) == 10 and replay['historical_row_deltas'] == 0
canonical = read(replay_folder / 'rows.json')
assert len(canonical) == 231
for trial in replay['cost_trials']:
    assert [{key: value for key, value in row.items() if key != 'elapsed_ns'} for row in trial['rows']] == canonical
for variant in replay['variants'].values():
    hashes = read(Path(variant['source_record']))['source_hashes']
    assert all(digest(Path(variant['source_directory']) / name) == value for name, value in hashes.items())
    assert all(digest(Path(name)) == row['sha256'] for name, row in variant['libraries'].items())
    assert all(digest(Path(row['path'])) == row['sha256'] for row in variant['binaries'].values())
assert all(digest(Path(name)) == row['sha256'] for name, row in replay['additional_fixture_inputs'].items())
previous_reports = read(EVIDENCE / 'area-compaction-production-replay/manifest.json')['reports']
import gzip
for board, row in replay['reports'].items():
    assert row['complete_bytes_identical_across_both_variants_and_repetitions']
    assert row['sha256'] == previous_reports[board]['sha256'] and row['bytes'] == previous_reports[board]['bytes']
    for variant in ['baseline', 'candidate']:
        for index in [1, 2]:
            raw = gzip.decompress((replay_folder / f'{variant}-reports-{index}-{board}.json.gz').read_bytes())
            assert hashlib.sha256(raw).hexdigest() == row['sha256'] and len(raw) == row['bytes']
regressions = [row for row in affected if row['candidate_over_baseline'] > 1]
memory_increases = [row for row in allocations if any(row['candidate'][field] > row['baseline'][field] for field in row['baseline'])]
assert len(regressions) == 3 and max(row['candidate_over_baseline'] for row in regressions) < 1.021
assert {row['name'] for row in memory_increases} == {'tiny_quadratic_2_points_cloned', 'tiny_quadratic_3_points_warm', 'tiny_quadratic_8_points_warm'}
assert all(row['candidate']['allocation_calls'] <= row['baseline']['allocation_calls'] and row['candidate']['allocated_bytes'] <= row['baseline']['allocated_bytes'] for row in allocations)
record = {
    'decision': 'kept for scoped production admission: exact outputs and complete reports are preserved, with broad query-time improvement and explicitly accepted resource tradeoffs',
    'tests_passed': tests, 'format_and_strict_clippy_passed': True,
    'production_validation_record': 'diagnostics-40.json',
    'complete_report_replay': replay['reports'],
    'historical_capability_rows_preserved': 231,
    'independent_extrema_matrix': {'boxes': 864, 'exact_extremum_assertions': 3456, 'coverage': 'Two policies, four signed/extreme scales, cold/warm input intervals, all six three-point permutations and nine independent coordinate/tie patterns. Additional empty/singleton/duplicate/near-coincident rational controls.'},
    'matched_probe': {'trials': 20, 'witness_rows_per_process': 72, 'all_witnesses_identical': True, 'memory_samples_repeat_exactly': True},
    'affected_query_timing_rows': affected, 'unchanged_construction_timing_controls': controls,
    'changed_allocation_rows': allocations, 'binary_sizes': size,
    'source_unchanged_during_validation_and_measurement': True, 'main_runtime_unchanged_when_measured': True,
    'resource_review': {
        'query_medians_improved': len(affected) - len(regressions),
        'query_medians_slower': regressions,
        'timing_interpretation': 'Three production medians are 1.3-2.0% slower, with overlapping five-sample ranges and variable construction controls. The earlier focused large-batch/CPU-conditioned comparisons improve these questioned cases. Retention does not assert a uniform improvement or cold-start benefit; all unfavorable samples remain recorded.',
        'allocation_increases': memory_increases,
        'memory_interpretation': 'Three tiny-quadratic rows retain 12288/8960/8960 additional bytes per 256 results (peak increases 11904/8960/8960). Their query medians improve 49.6%/27.3%/10.8%, and requested allocation traffic decreases in each. These repeatable memory tradeoffs are accepted under performance before memory. No allocation-call or allocated-byte row increases.',
        'binary_interpretation': 'Timing/allocation executables shrink by 2896/5152 bytes; text shrinks by 976/2896 bytes and rodata by 160 bytes in both.',
    },
    'next_step': 'Apply and stage only the two audited production source files, verify their staged hashes against the tested source archive, then rerun the original full-board gate on the broader native prototype. Its prior failed gate and remaining admission requirements remain open.',
    'limitations': 'Production timing follows fixed 100 ms integer-only CPU conditioning before geometry construction, and does not claim cold-start latency improvement. The original unconditioned and focused trials remain in diagnostics-38/39. No persistent cache, scalar proof-budget change, full-application speedup, broad native admission or property-completion claim. Twenty-eight existing curve ignores and the complete workspace/pre-push matrix remain outside this scoped admission.',
}
paths = [path for folder in [COST, replay_folder] for path in sorted(folder.rglob('*')) if path.is_file() and path.name not in cost['binaries'] and not any(part.startswith(('baseline-reports-', 'candidate-reports-')) for part in path.relative_to(folder).parts[:-1])]
paths += [EVIDENCE / name for name in ['bbox_initial_pair_production_probe.rs', 'reproduce_bbox_initial_pair_production_replay.py', 'reproduce_bbox_initial_pair_production_cost.py', 'analyze_bbox_initial_pair_production_cost.py']]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-41.json').write_text(json.dumps(record, indent=2) + '\n')
print('Production tests:', sum(tests.values()))
print('Affected query rows:', len(affected), 'median-ratio range:', min(row['candidate_over_baseline'] for row in affected), max(row['candidate_over_baseline'] for row in affected))
print('Affected regressions:', [row for row in affected if row['candidate_over_baseline'] > 1])
print('Changed allocation rows:', len(allocations))
print('Binary sizes:', size)

print('Report bytes:', {name: row['bytes'] for name, row in replay['reports'].items()})
