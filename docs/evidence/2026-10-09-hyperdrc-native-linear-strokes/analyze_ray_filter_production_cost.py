"""Validate matched ray-filter recovery, controls and complete report replay."""
import gzip
import hashlib
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
COST = EVIDENCE / 'ray-filter-production-cost'
REPLAY = EVIDENCE / 'ray-filter-production-replay'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
validation = read(EVIDENCE / 'diagnostics-45.json')
assert validation['original_board_gate']['passed'] and validation['format_strict_clippy_and_docs_passed']
source = read(EVIDENCE / 'ray-filter-finite-guard/sources.json')
native = read(Path(source['main_source_record']))
assert all(digest(ROOT / 'target/ray-filter-finite-guard/source' / name) == value for name, value in source['source_hashes'].items())
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
cost = read(COST / 'manifest.json')
assert len(cost['commands']) == 30 and all(command['exit_code'] == 0 for command in cost['commands'])
assert len(cost['trials']) == 20 and len(cost['summaries']) == 128
assert cost['probe_sha256'] == digest(COST / 'probe.rs') == digest(EVIDENCE / 'ray_filter_cost_probe.rs')
assert all(digest(Path(row['path'])) == row['sha256'] for row in cost['libraries'].values())
assert all(digest(COST / name) == row['sha256'] for name, row in cost['binaries'].items())
canonical = {}
for mode, count in [('time', 128), ('memory', 64)]:
    canonical[mode] = {}
    for variant in ['baseline', 'candidate']:
        trials = [trial for trial in cost['trials'] if trial['mode'] == mode and trial['variant'] == variant]
        assert len(trials) == 5
        witness = None
        for trial in trials:
            assert len(trial['rows']) == 128
            measures = [row for row in trial['rows'] if 'name' in row]
            rows = [row for row in trial['rows'] if 'witness' in row]
            assert len(measures) == len(rows) == 64
            assert all(row['iterations'] == count for row in measures)
            assert all(row['count'] == count for row in rows)
            if witness is None: witness = rows
            else: assert rows == witness
            if variant == 'candidate':
                for row in rows:
                    if row['kind'] == 'query': assert row['correct'] == count and row['counts'] == [count//4, count//4, count//2, 0]
                    else: assert row['constructed'] == row['nonempty'] == row['loops'] == row['exact_area_matches'] == count
        canonical[mode][variant] = {row['witness']: row for row in witness}
for summary in cost['summaries']:
    for variant, fields in summary['variants'].items():
        rows = [next(row for row in trial['rows'] if row.get('name') == summary['name']) for trial in cost['trials'] if trial['mode'] == summary['mode'] and trial['variant'] == variant]
        assert len(rows) == 5
        for field, stats in fields.items():
            assert stats['samples'] == [row[field] for row in rows]
            assert stats['median'] == statistics.median(stats['samples'])
            if summary['mode'] == 'memory': assert len(set(stats['samples'])) == 1
deltas = {}
for mode in ['time', 'memory']:
    old, new = canonical[mode]['baseline'], canonical[mode]['candidate']
    assert old.keys() == new.keys()
    deltas[mode] = [{'name': name, 'baseline': old[name], 'candidate': new[name]} for name in old if old[name] != new[name]]
recovered = {row['name'] for row in deltas['time']}
assert recovered == {row['name'] for row in deltas['memory']}
timings, allocations = [], []
for summary in cost['summaries']:
    if summary['mode'] == 'time':
        old = summary['variants']['baseline']['nanos']['median']
        new = summary['variants']['candidate']['nanos']['median']
        timings.append({'name': summary['name'], 'witness_changed_for_correctness': summary['name'] in recovered, 'baseline_ns_per_operation': old/128, 'candidate_ns_per_operation': new/128, 'candidate_over_baseline': new/old})
    else:
        row = {'name': summary['name'], 'witness_changed_for_correctness': summary['name'] in recovered}
        for variant in ['baseline', 'candidate']: row[variant] = {field: stats['median'] for field, stats in summary['variants'][variant].items()}
        allocations.append(row)
sizes = {}
for name, row in cost['binaries'].items():
    sections = {line.split()[0]: int(line.split()[1]) for line in row['sections'].splitlines() if line.startswith('.')}
    sizes[name] = {'file_bytes': row['bytes'], 'text_bytes': sections['.text'], 'rodata_bytes': sections['.rodata']}
replay = read(REPLAY / 'manifest.json')
correction = read(REPLAY / 'scope-correction.json')
original = read(REPLAY / correction['original_manifest'])
assert digest(REPLAY / correction['original_manifest']) == correction['original_manifest_sha256']
assert digest(REPLAY / 'manifest.json') == correction['corrected_manifest_sha256']
assert digest(REPLAY / correction['original_executed_runner']) == correction['original_runner_sha256']
assert digest(EVIDENCE / 'reproduce_ray_filter_production_replay.py') == correction['corrected_runner_sha256']
assert {key: value for key, value in replay.items() if key != 'scope'} == {key: value for key, value in original.items() if key != 'scope'}
assert len(replay['commands']) == 20 and all(command['exit_code'] == 0 for command in replay['commands'])
assert len(replay['cost_trials']) == 10 and replay['historical_row_deltas'] == 0
rows = read(REPLAY / 'rows.json'); assert len(rows) == 231
for trial in replay['cost_trials']:
    assert [{key: value for key, value in row.items() if key != 'elapsed_ns'} for row in trial['rows']] == rows
for variant in replay['variants'].values():
    hashes = read(Path(variant['source_record']))['source_hashes']
    assert all(digest(Path(variant['source_directory']) / name) == value for name, value in hashes.items())
    assert all(digest(Path(name)) == row['sha256'] for name, row in variant['libraries'].items())
    assert all(digest(Path(row['path'])) == row['sha256'] for row in variant['binaries'].values())
assert all(digest(Path(name)) == row['sha256'] for name, row in replay['additional_fixture_inputs'].items())
previous = read(EVIDENCE / 'bbox-initial-pair-production-replay/manifest.json')['reports']
for board, row in replay['reports'].items():
    assert row['complete_bytes_identical_across_both_variants_and_repetitions']
    assert row['sha256'] == previous[board]['sha256'] and row['bytes'] == previous[board]['bytes']
    for variant in ['baseline', 'candidate']:
        for index in [1, 2]:
            raw = gzip.decompress((REPLAY / f'{variant}-reports-{index}-{board}.json.gz').read_bytes())
            assert hashlib.sha256(raw).hexdigest() == row['sha256'] and len(raw) == row['bytes']
unchanged = [row for row in timings if not row['witness_changed_for_correctness']]
assert len(recovered) == 16 and len(unchanged) == 48
assert all(row['baseline'] == row['candidate'] for row in allocations if not row['witness_changed_for_correctness'])
record = {
    'decision': 'kept as an exactness repair: four incorrectly empty affine lens configurations recover their exact nonempty geometry, area and classification witnesses',
    'counterexample_record': 'diagnostics-43.json', 'focused_record': 'diagnostics-44.json', 'production_validation_record': 'diagnostics-45.json',
    'production_suites': validation['suites'], 'original_production_gate_passed': True,
    'historical_capability_rows_preserved': 231, 'complete_reports': replay['reports'],
    'replay_scope_metadata_correction': 'ray-filter-production-replay/scope-correction.json',
    'matched_probe': {'trials': 20, 'witness_rows_per_process': 64, 'candidate_witnesses_all_correct': True, 'witnesses_repeat_per_variant': True, 'allocation_samples_repeat_exactly': True, 'timing_iterations': 128, 'allocation_iterations': 64},
    'correctness_witness_deltas': deltas, 'timing_rows': timings, 'allocation_rows': allocations, 'binary_sizes': sizes,
    'main_runtime_unchanged_when_measured': True,
    'resource_review': {
        'preserved_witness_rows': len(unchanged),
        'preserved_witness_median_ratio': statistics.median(row['candidate_over_baseline'] for row in unchanged),
        'preserved_witness_ratio_range': [min(row['candidate_over_baseline'] for row in unchanged), max(row['candidate_over_baseline'] for row in unchanged)],
        'preserved_witness_allocations_all_identical': True,
        'correctness_tradeoff': 'Four retained-chart construction rows previously returned empty regions, losing all area and boundary evidence; all now retain one correct loop. Their first/warm/cloned queries account for the other twelve corrected rows. Correct classification and retained geometry require substantially more query work and memory than the former empty-result path. These costs are accepted under exactness before performance and memory, without a uniform speedup claim.',
        'example': 'For the positive 1e308 translation, construction changes from 426.6 to 393.5 microseconds per region, while the first query changes from an incorrect empty-region lookup at 20.2 ns to a correct result at 119.2 microseconds. Per 64 constructed fixtures retained bytes increase from 918984 to 1686528; first-query retained bytes increase from 64 to 698944. All samples are retained.',
        'binary_tradeoff': 'Timing/allocation ELF files grow by 536/544 bytes; text grows by 640 bytes in both, while rodata shrinks by 64/48 bytes.',
    },
    'unmeasured_followup': 'contact-index-range-review.json',
    'next_step': 'Admit only the guard and its two test modules after staged source-hash verification. Investigate the separately recorded contact-index range-rounding hypothesis after current validation and measurement are terminal; no change for that hypothesis is included here.',
    'limitations': 'Focused matched probes use 100 ms CPU conditioning and do not measure cold-start latency or establish a full-application speedup. A cheaper incorrect baseline result is not a valid optimization target. The 28 existing curve ignores, complete workspace/pre-push matrix, full floating-point error proof and broader native-prototype admission remain open.',
}
paths = [path for folder in [COST, REPLAY] for path in sorted(folder.rglob('*')) if path.is_file() and path.name not in cost['binaries'] and not any(part.startswith(('baseline-reports-', 'candidate-reports-')) for part in path.relative_to(folder).parts[:-1])]
paths += [EVIDENCE / name for name in ['ray_filter_cost_probe.rs', 'reproduce_ray_filter_production_cost.py', 'reproduce_ray_filter_production_replay.py', 'analyze_ray_filter_production_cost.py', 'contact-index-range-review.json']]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-46.json').write_text(json.dumps(record, indent=2)+'\n')
unchanged = [row for row in timings if not row['witness_changed_for_correctness']]
print('Corrected witness rows:', len(recovered), 'of 64')
print('Preserved-witness timing ratio range:', min(row['candidate_over_baseline'] for row in unchanged), max(row['candidate_over_baseline'] for row in unchanged))
print('Binary sizes:', sizes)
print('Complete report bytes:', {name: row['bytes'] for name, row in replay['reports'].items()})
