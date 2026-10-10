"""Audit exact replay and retain every broad workload timing comparison."""
import gzip
import hashlib
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'arc-endpoint-production-replay'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
manifest = read(OUT / 'manifest.json')
assert manifest['source_unchanged'] and manifest['historical_row_deltas'] == 0
assert all(row['exit_code'] == 0 for row in manifest['commands'])
trials = manifest['cost_trials']
assert len(trials) == 10 and all(len(trial['rows']) == 231 for trial in trials)
canonical = read(OUT / 'rows.json')
assert len(canonical) == 231
features = {}
for variant in ['baseline', 'candidate']:
    source = manifest['variants'][variant]['source_directory']
    features[variant] = sorted((event['package_id'].replace(source, '<SOURCE>'), event['target']['name'], event['target']['kind'], event['features']) for line in (OUT / (variant + '-library-build.log')).read_text().splitlines() if line.startswith('{') and (event := json.loads(line)).get('reason') == 'compiler-artifact')
assert features['baseline'] == features['candidate']
timings = []
for witness in canonical:
    values = {variant: [next(row['elapsed_ns'] for row in trial['rows'] if row['name'] == witness['name']) for trial in trials if trial['variant'] == variant] for variant in ['baseline', 'candidate']}
    assert all(len(samples) == 5 for samples in values.values())
    baseline, candidate = values['baseline'], values['candidate']
    timings.append({'name': witness['name'], 'iterations': witness['iterations'], 'baseline_samples_ns': baseline, 'candidate_samples_ns': candidate, 'candidate_over_baseline_median': statistics.median(candidate) / statistics.median(baseline), 'candidate_all_samples_slower': min(candidate) > max(baseline), 'candidate_all_samples_faster': max(candidate) < min(baseline), 'sample_ranges_overlap': max(min(baseline), min(candidate)) <= min(max(baseline), max(candidate))})
(OUT / 'timing-comparison.json').write_text(json.dumps(timings, indent=2) + '\n')
expected = {'cparti-fpga': (1543306, 2605, '5be27023735a5ee22624a3a517368d1d735da15c73b8974fc1947d7bff69bd1a'), 'hvp109a': (291822, 654, 'e2d82bbd8d193b8889d32d4f4799185b51fdbf72c4db6ad1e587c05b866ca8e3')}
for board, (length, count, sha) in expected.items():
    report = manifest['reports'][board]
    assert (report['bytes'], report['violation_count'], report['sha256']) == (length, count, sha)
    assert report['complete_bytes_identical_across_versions_repetitions_and_admitted_report']
    for variant in ['baseline', 'candidate']:
        for index in [1, 2]:
            raw = gzip.decompress((OUT / f'{variant}-reports-{index}-{board}.json.gz').read_bytes())
            assert len(raw) == length and hashlib.sha256(raw).hexdigest() == sha
record = {
    'decision': 'complete historical witnesses and report bytes preserved; inspect broad timing comparisons and perform native recheck before runtime admission',
    'scope': manifest['scope'],
    'historical_witnesses_per_process': 231, 'fresh_witness_processes_each_variant': 5,
    'historical_witness_deltas': 0, 'complete_report_processes_each_variant': 2,
    'reports': manifest['reports'],
    'matching_cargo_feature_artifacts': len(features['baseline']),
    'broad_workload_timing_comparisons': 'arc-endpoint-production-replay/timing-comparison.json',
    'timing_overview': {'rows': len(timings), 'medians_slower': sum(row['candidate_over_baseline_median'] > 1 for row in timings), 'medians_faster': sum(row['candidate_over_baseline_median'] < 1 for row in timings), 'median_of_row_ratios': statistics.median(row['candidate_over_baseline_median'] for row in timings), 'maximum_row_ratio': max(row['candidate_over_baseline_median'] for row in timings), 'nonoverlapping_slower_rows': [row['name'] for row in timings if row['candidate_all_samples_slower']], 'nonoverlapping_faster_rows': [row['name'] for row in timings if row['candidate_all_samples_faster']]},
    'preceding_gates': ['diagnostics-77.json', 'diagnostics-78.json', 'diagnostics-79.json', 'diagnostics-80.json'],
    'source_unchanged': True, 'main_runtime_unchanged': True,
    'limitations': 'Complete report bytes and 231 historical witnesses are preserved on the admitted-production input set. The separate native-stroke prototype is not admitted. Five-sample timing ranges are descriptive and every adverse sample is retained; this record does not infer a universal speedup or zero regression from matching witnesses. Native recheck, final tradeoff review and explicit retention remain outstanding. Full workspace/pre-push validation, all representations and the complete property/target plan are not supplied by this replay.',
}
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in sorted(OUT.rglob('*')) if path.is_file() and not any(parent.name.startswith(('baseline-reports-', 'candidate-reports-')) for parent in path.parents if parent != OUT)}
for name in ['reproduce_arc_endpoint_production_replay.py', 'analyze_arc_endpoint_production_replay.py', 'compress_arc_endpoint_replay_evidence.py']:
    path = EVIDENCE / name
    record['artifacts'][name] = {'bytes': path.stat().st_size, 'sha256': digest(path)}
(EVIDENCE / 'diagnostics-81.json').write_text(json.dumps(record, indent=2) + '\n')
print('Complete report bytes and all 231 historical witnesses preserved.')
print(json.dumps(record['timing_overview'], indent=2))
