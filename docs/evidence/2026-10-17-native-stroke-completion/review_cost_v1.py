"""Classify every measured artifact difference and retain unfavorable costs."""
import hashlib, json, re, statistics
from pathlib import Path

W = Path(__file__).resolve().parent
O = W / 'cost-review-v1.json'
assert not O.exists()

def read(p):
    return json.loads(Path(p).read_text())

def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

cost = read(W / 'comparison-v1/result.json')
structure = read(W / 'structure-metrics-v1/result.json')
assert cost['complete'] and structure['complete']
assert len(cost['all_samples']) == structure['phase_observations'] == 240
assert not cost['candidate_incomplete_cases'] and not cost['repeated_snapshot_differences']
assert not any(cost['raw_full_board_report_differences'].values())
assert cost['exact_scalar_serialization_errors'] == 0
variants = {v: [r for r in cost['all_samples'] if r['variant'] == v] for v in ['baseline', 'candidate']}
completion = {v: {'phases': len(rows), 'certified_complete': sum(r['certified_completion'] for r in rows)} for v, rows in variants.items()}
assert completion == {'baseline': {'phases': 120, 'certified_complete': 80}, 'candidate': {'phases': 120, 'certified_complete': 120}}
matched = [r for r in cost['paired_summary'] if r['matched_completed_semantic_contract']]
expanded = [r for r in cost['paired_summary'] if r['candidate_always_completes'] and not r['baseline_always_completes']]
assert len(matched) == 16 and len(expanded) == 8
reviewed = []
allowed = re.compile(r'^\$\[[01]\]\.regions\[[01]\]\.(?:area\.actual(?:\.|$)|bounds\[[0-3]\]\.actual(?:\.|$)|native\.material\[0\]$)')
for row in cost['artifact_differences']:
    path = W / 'comparison-v1' / row['path']
    assert sha(path) == row['sha256']
    differences = read(path)
    assert len(differences) == row['difference_count']
    strengthened = []
    for difference in differences:
        key = difference['path']
        if key.endswith('.area.certified_equal'):
            assert difference['baseline'] is False and difference['candidate'] is True
            strengthened.append(key)
        else:
            assert allowed.match(key), key
    assert len(strengthened) == (4 if row['cap'] == 'round' and row['case'] >= 2 else 0)
    counts = {}
    for variant in ['baseline', 'candidate']:
        snapshot = read(W / 'comparison-v1' / f'pair-00-case-{row["case"]}-{row["cap"]}-{variant}.snapshot.json')
        counts[variant] = [[len(region['native']['material'][0]) for region in phase['regions']] for phase in snapshot]
    reviewed.append({'case': row['case'], 'cap': row['cap'], 'difference_count': len(differences),
        'strengthened_area_claims': strengthened, 'native_edge_counts': counts,
        'all_other_facts_and_independent_query_results_identical': True})
assert all(r['difference_count'] == 0 for r in reviewed if r['cap'] == 'butt' or r['case'] == 0)
matched_rows = []
for row in matched:
    pairs = row['all_pairs']
    matched_rows.append({k: row[k] for k in ['case', 'label', 'cap', 'phase', 'median_elapsed_ratio']} | {
        'median_requested_allocation_byte_change_percent': 100 * statistics.median(p['allocated_bytes_delta'] / p['baseline']['allocated_bytes'] for p in pairs),
        'all_allocation_call_deltas': [p['allocation_calls_delta'] for p in pairs],
        'all_peak_requested_byte_deltas': [p['peak_extra_requested_bytes_delta'] for p in pairs]})
for row in structure['all_paired_rows']:
    for before, after in zip(row['baseline'], row['candidate']):
        assert all(after[k] <= before[k] for k in before), (row['label'], row['cap'])
record = {'complete': True, 'runner_sha256': sha(__file__),
    'comparison_sha256': sha(W / 'comparison-v1/result.json'),
    'structure_sha256': sha(W / 'structure-metrics-v1/result.json'),
    'completion': completion, 'matched_completed_rows': len(matched), 'expanded_completion_rows': len(expanded),
    'reviewed_artifact_differences': reviewed, 'repeated_snapshot_differences': 0,
    'matched_completed_cost_rows': matched_rows,
    'largest_median_paired_elapsed_ratio_on_matched_rows': max(r['median_elapsed_ratio'] for r in matched),
    'largest_median_requested_allocation_byte_increase_percent_on_matched_rows': max(r['median_requested_allocation_byte_change_percent'] for r in matched_rows),
    'all_recorded_retained_expression_metrics_nonincreasing': True,
    'cli_byte_delta': cost['sizes']['candidate']['cli_bytes'] - cost['sizes']['baseline']['cli_bytes'],
    'probe_byte_delta': cost['sizes']['candidate']['probe_bytes'] - cost['sizes']['baseline']['probe_bytes'],
    'interpretation': 'Retain for expanded certified completion. Small adverse butt-stroke timings and allocation increases are retained alongside favorable round-stroke observations. Ratios for newly completed cases compare different completion outcomes and are not speedup evidence.',
    'artifact_migration': 'Five non-axis round-stroke cases use six regularized native edges instead of eight; equivalent scalar expression forms change in the four newly completed cases. Independent expected area, bounds and point classification establish the tested semantics separately from these serialization differences.',
    'limitations': 'Five instrumented pairs per case/cap. No universal speedup, uninstrumented latency, unique live graph size, peak intermediate expression growth, internal work bound or whole-program heap bound is established.'}
O.write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({k: record[k] for k in ['complete', 'completion', 'matched_completed_rows', 'expanded_completion_rows',
    'largest_median_paired_elapsed_ratio_on_matched_rows', 'largest_median_requested_allocation_byte_increase_percent_on_matched_rows',
    'cli_byte_delta', 'probe_byte_delta']}), flush=True)
