"""Report the initial, independent follow-up and combined timing evidence."""
import hashlib
import json
from pathlib import Path
import statistics

EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'arc-endpoint-broad-followup'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
manifest = read(OUT / 'manifest.json')
initial = read(EVIDENCE / 'arc-endpoint-production-replay/manifest.json')
assert digest(Path(manifest['plan']['initial_record'])) == manifest['plan']['initial_record_sha256']
assert manifest['all_sources_and_binaries_unchanged'] and manifest['all_231_witnesses_preserved_each_process']
assert len(manifest['additional_trials']) == len(manifest['commands']) == 20
assert all(row['exit_code'] == 0 for row in manifest['commands'])
trials = initial['cost_trials'] + manifest['additional_trials']
rows = []
for combined in manifest['comparison']:
    name = combined['name']
    parts = {}
    for label, selected in [('initial_five', initial['cost_trials']), ('additional_ten', manifest['additional_trials']), ('combined_fifteen', trials)]:
        by_pair = {}
        values = {'baseline': [], 'candidate': []}
        for trial in selected:
            elapsed = next(row['elapsed_ns'] for row in trial['rows'] if row['name'] == name)
            values[trial['variant']].append(elapsed)
            by_pair.setdefault(trial['index'], {})[trial['variant']] = elapsed
        baseline, candidate = values['baseline'], values['candidate']
        paired = [pair['candidate'] / pair['baseline'] for _, pair in sorted(by_pair.items())]
        parts[label] = {'candidate_over_baseline_median': statistics.median(candidate) / statistics.median(baseline), 'paired_ratios': paired, 'median_paired_ratio': statistics.median(paired), 'pairs_candidate_slower': sum(ratio > 1 for ratio in paired), 'pairs': len(paired), 'candidate_all_samples_slower': min(candidate) > max(baseline), 'candidate_all_samples_faster': max(candidate) < min(baseline), 'sample_ranges_overlap': max(min(baseline), min(candidate)) <= min(max(baseline), max(candidate))}
    rows.append({'name': name, 'iterations': combined['iterations'], 'stages': parts})
(OUT / 'split-comparison.json').write_text(json.dumps(rows, indent=2) + '\n')
overview = {}
for label in ['initial_five', 'additional_ten', 'combined_fifteen']:
    values = [row['stages'][label] for row in rows]
    overview[label] = {'medians_slower': sum(row['candidate_over_baseline_median'] > 1 for row in values), 'medians_faster': sum(row['candidate_over_baseline_median'] < 1 for row in values), 'median_of_row_ratios': statistics.median(row['candidate_over_baseline_median'] for row in values), 'maximum_row_ratio': max(row['candidate_over_baseline_median'] for row in values), 'nonoverlapping_slower_rows': [row['name'] for row in rows if row['stages'][label]['candidate_all_samples_slower']], 'nonoverlapping_faster_rows': [row['name'] for row in rows if row['stages'][label]['candidate_all_samples_faster']]}
original_controls = [row for row in rows if row['name'] in manifest['plan']['initial_nonoverlapping_slower_rows']]
assert len(original_controls) == 4
record = {
    'decision': 'one fixed follow-up completed; persistent control median shifts remain for final tradeoff review; native recheck is next and no runtime change is admitted',
    'scope': manifest['scope'], 'overview': overview,
    'initial_adverse_controls': original_controls,
    'largest_combined_median_ratios': [{'name': row['name'], 'stages': {label: {'candidate_over_baseline_median': stage['candidate_over_baseline_median'], 'pairs_candidate_slower': stage['pairs_candidate_slower'], 'pairs': stage['pairs']} for label, stage in row['stages'].items()}} for row in sorted(rows, key=lambda row: row['stages']['combined_fifteen']['candidate_over_baseline_median'], reverse=True)[:10]],
    'all_231_historical_witnesses_preserved': True,
    'all_thirty_processes_in_combined_analysis': True,
    'no_adaptive_extension_or_discarded_samples': True,
    'all_sources_and_binaries_unchanged': True,
    'main_runtime_unchanged': True,
    'assessment': 'The independent ten-pair extension has no nonoverlapping slower ranges, but overlap is not treated as proof of no regression. Three initially adverse controls retain combined median ratios of about 1.060 (huge-center proximity), 1.030 (huge-center pair return), and 1.066 (pi decimal formatting), with 11/15, 13/15 and 12/15 slower paired observations. The thermal fixture narrows to about 1.011 overall and 1.002 in the independent extension. Other adverse rows remain visible: sparse fixture coverage is about 1.104 overall, and overlapping duplicate-layer area is about 1.043 with 14/15 slower pairs, including all ten new pairs. Across all 231 rows, the combined median of ratios is about 1.0016. These persistent local costs must remain visible alongside the targeted endpoint savings. No further sampling extension is planned; the intended native workload still requires a separate correctness/resource check before final retention review.',
    'limitations': 'Paired ratios and separate initial/additional results expose persistent shifts even when wider combined ranges overlap. The descriptive min/max ranges are not confidence intervals and overlap does not prove absence of regression. These timings are correlated within each process and span 231 comparisons; no statistical-significance claim is inferred from row counts. No source rewrite, native admission or universal performance claim follows from this analysis.',
}
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in sorted(OUT.iterdir()) if path.is_file()}
for name in ['reproduce_arc_endpoint_broad_followup.py', 'analyze_arc_endpoint_broad_followup.py', 'compress_arc_endpoint_replay_evidence.py']:
    path = EVIDENCE / name
    record['artifacts'][name] = {'bytes': path.stat().st_size, 'sha256': digest(path)}
(EVIDENCE / 'diagnostics-82.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps(overview, indent=2))
for row in original_controls:
    print(row['name'], {label: (round(values['candidate_over_baseline_median'], 5), values['pairs_candidate_slower'], values['pairs'], values['sample_ranges_overlap']) for label, values in row['stages'].items()})
