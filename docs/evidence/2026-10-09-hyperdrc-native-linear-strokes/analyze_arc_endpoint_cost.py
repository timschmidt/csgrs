"""Describe every paired endpoint time/allocation/size tradeoff without admission."""
import hashlib
import json
from pathlib import Path
import statistics

EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'arc-endpoint-cost'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
manifest = read(OUT / 'manifest.json')
assert manifest['all_production_and_main_source_hashes_unchanged']
assert len(manifest['trials']) == 20
assert all(row['exit_code'] == 0 for row in manifest['commands'])
timing, memory = [], []
for row in manifest['summaries']:
    baseline, candidate = row['variants']['baseline'], row['variants']['candidate']
    if row['mode'] == 'time':
        baseline, candidate = baseline['nanos'], candidate['nanos']
        timing.append({
            'name': row['name'], 'candidate_over_baseline_median': candidate['median'] / baseline['median'],
            'baseline_samples_ns': baseline['samples'], 'candidate_samples_ns': candidate['samples'],
            'baseline_median_ns': baseline['median'], 'candidate_median_ns': candidate['median'],
            'sample_ranges_overlap': max(min(baseline['samples']), min(candidate['samples'])) <= min(max(baseline['samples']), max(candidate['samples'])),
            'candidate_all_samples_faster': max(candidate['samples']) < min(baseline['samples']),
            'candidate_all_samples_slower': min(candidate['samples']) > max(baseline['samples']),
        })
    else:
        fields = ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']
        assert all(len(set(values[field]['samples'])) == 1 for values in [baseline, candidate] for field in fields), row['name']
        memory.append({'name': row['name'], 'baseline': {field: baseline[field]['median'] for field in fields}, 'candidate': {field: candidate[field]['median'] for field in fields}, 'deltas': {field: candidate[field]['median'] - baseline[field]['median'] for field in fields}})
assert len(timing) == len(memory) == 48
sizes = {}
for mode in ['time', 'memory']:
    baseline, candidate = [manifest['binaries'][variant + '-' + mode] for variant in ['baseline', 'candidate']]
    sizes[mode] = {'baseline_bytes': baseline['bytes'], 'candidate_bytes': candidate['bytes'], 'candidate_minus_baseline_bytes': candidate['bytes'] - baseline['bytes']}
summary = {
    'decision': 'early production resource gate passed; advance to complete production curve/DRC validation and replay; no runtime admission',
    'scope': manifest['scope'], 'timing_rows': timing, 'allocation_rows': memory, 'executable_sizes': sizes,
    'timing_overview': {
        'medians_slower': sum(row['candidate_over_baseline_median'] > 1 for row in timing),
        'medians_faster': sum(row['candidate_over_baseline_median'] < 1 for row in timing),
        'median_of_row_ratios': statistics.median(row['candidate_over_baseline_median'] for row in timing),
        'minimum_row_ratio': min(row['candidate_over_baseline_median'] for row in timing),
        'maximum_row_ratio': max(row['candidate_over_baseline_median'] for row in timing),
        'nonoverlapping_faster_rows': [row['name'] for row in timing if row['candidate_all_samples_faster']],
        'nonoverlapping_slower_rows': [row['name'] for row in timing if row['candidate_all_samples_slower']],
    },
    'allocation_overview': {'calls_saved_rows': sum(row['deltas']['allocation_calls'] < 0 for row in memory), 'calls_added_rows': sum(row['deltas']['allocation_calls'] > 0 for row in memory), 'peak_increased_rows': sum(row['deltas']['peak_bytes_delta'] > 0 for row in memory), 'retained_increased_rows': sum(row['deltas']['retained_bytes_delta'] > 0 for row in memory)},
    'matched_witnesses_per_process': 48,
    'independent_geometry_basis': 'The five optimizing circle families use the unit-circle points (3/5,4/5), (sqrt(2)/2,sqrt(2)/2), or (-3/5,4/5), with exact translation/positive scale and reversed-authoring invariance. Endpoint aliases/copies are inclusive. (2,0) and (1-1e-40,0) are off the unit circle. The symbolic sin(e)^2+cos(e)^2-1 input is mathematically zero but its strict result stays typed uncertain; the approximate case explicitly consumes the terminal. The production probe asserts results and certainty for every operation; all recorded witnesses match in every paired process. See the separate Fraction/similarity oracle and frontier record for broader geometry coverage.',
    'main_runtime_unchanged': True,
    'assessment': 'The fifteen first/warm/cloned rows in the five intended non-endpoint circle families improve 18.4% to 42.3%; fourteen have nonoverlapping five-sample ranges. Ten of all 48 timing medians are slower, at most 4.68%, but every slower row has overlapping ranges. This does not prove zero regression. Fifteen rows save allocation calls/requested bytes, with no increases in calls, peak or retained bytes. The modest +168/+136-byte timing/allocation executable growth is lower priority than the observed query improvements. Constructor and unchanged-control timing shifts are not attributed to certificate reuse. The evidence justifies the next application gate, not final retention.',
    'limitations': manifest['limitations'],
}
summary['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in sorted(OUT.iterdir()) if path.is_file()}
for name in ['arc_endpoint_cost_probe.rs', 'reproduce_arc_endpoint_cost.py', 'analyze_arc_endpoint_cost.py']:
    path = EVIDENCE / name
    summary['artifacts'][name] = {'bytes': path.stat().st_size, 'sha256': digest(path)}
(EVIDENCE / 'diagnostics-78.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps({'timing': summary['timing_overview'], 'allocation': summary['allocation_overview'], 'sizes': sizes}, indent=2))
