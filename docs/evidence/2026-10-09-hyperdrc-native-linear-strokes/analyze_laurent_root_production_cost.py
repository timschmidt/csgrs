"""Summarize every production timing and allocation row without hiding regressions."""
import gzip
import hashlib
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
PRODUCTION = EVIDENCE / 'laurent-root-production'
COST = EVIDENCE / 'laurent-root-production-cost'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
source = read(PRODUCTION / 'sources.json')
base = read(Path(source['base_source_record']))
native = read(Path(source['main_source_record']))
hashes = base['source_hashes'] | source['changed_source_hashes'] | source['additional_source_hashes']
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
assert all(digest(ROOT / 'target/laurent-root-production/source' / name) == value for name, value in hashes.items())
assert all(digest(ROOT / 'target/contact-index-directed-width/source' / name) == value for name, value in base['source_hashes'].items())
wrapper = read(COST / 'wrapper-status.json')
assert wrapper['exit_code'] == 0 and wrapper['all_production_and_baseline_inputs_unchanged']
assert digest(Path(wrapper['production_validation'])) == wrapper['production_validation_sha256']
assert digest(Path(wrapper['reused_independent_oracle'])) == wrapper['oracle_sha256']
assert read(Path(wrapper['reused_independent_oracle']))['identities'] == 126
manifest = read(COST / 'manifest.json')
assert manifest['source_change'] == [source['only_modified_runtime_source']]
assert len(manifest['trials']) == 20 and len(manifest['commands']) == 28
assert all(row['exit_code'] == 0 for row in manifest['commands'])
for name in ['baseline', 'candidate']:
    adapter = Path(manifest[name + '_source'])
    archived = gzip.decompress((PRODUCTION / 'scalar-cost-inputs' / (name + '-sources.json.gz')).read_bytes())
    assert archived == adapter.read_bytes()
    expected = base['source_hashes'] if name == 'baseline' else {path: hashes[path] for path in base['source_hashes']}
    assert json.loads(archived)['source_hashes'] == expected
witnesses = lambda trial: [row for row in trial['rows'] if 'witness' in row or 'fixture' in row]
for mode in ['time', 'memory']:
    trials = [trial for trial in manifest['trials'] if trial['mode'] == mode]
    assert len(trials) == 10 and len(witnesses(trials[0])) == 24
    assert all(witnesses(trial) == witnesses(trials[0]) for trial in trials)
retained = read(COST / 'retained-libraries.json')
for name, row in retained['libraries'].items():
    assert row['sha256'] == manifest['libraries'][name]['sha256']
    assert digest(Path(row['retained_path'])) == row['sha256']
for name, row in manifest['binaries'].items():
    assert digest(COST / name) == row['sha256']
timing = []
allocation = []
for row in manifest['summaries']:
    old = row['variants']['baseline']
    new = row['variants']['candidate']
    if row['mode'] == 'time':
        before, after = old['nanos'], new['nanos']
        timing.append({
            'name': row['name'], 'ratio': after['median'] / before['median'],
            'baseline_median_batch_ns': before['median'], 'candidate_median_batch_ns': after['median'],
            'baseline_median_ns_per_fixture': before['median'] / 4096,
            'candidate_median_ns_per_fixture': after['median'] / 4096,
            'baseline_range_batch_ns': [min(before['samples']), max(before['samples'])],
            'candidate_range_batch_ns': [min(after['samples']), max(after['samples'])],
            'nonoverlapping_ranges': max(before['samples']) < min(after['samples']) or max(after['samples']) < min(before['samples']),
        })
    else:
        allocation.append({
            'name': row['name'],
            'candidate_minus_baseline': {field: new[field]['median'] - old[field]['median'] for field in old},
            'all_five_samples_identical_per_variant': all(len(set(stats['samples'])) == 1 for fields in [old, new] for stats in fields.values()),
        })
assert len(timing) == len(allocation) == 24
ratios = [row['ratio'] for row in timing]
assert all(row['iterations'] == (4096 if trial['mode'] == 'time' else 1000) for trial in manifest['trials'] for row in trial['rows'] if 'name' in row)
conjugate = {row['name']: row for row in timing if row['name'].startswith('conjugate_product_')}
assert conjugate['conjugate_product_first']['ratio'] > 1.10
assert conjugate['conjugate_product_cloned']['ratio'] > 1.04
assert all(conjugate[name]['nonoverlapping_ranges'] for name in ['conjugate_product_first', 'conjugate_product_cloned', 'conjugate_product_construction'])
record = {
    'decision': 'not retained: production timing regressions outweigh allocation savings under performance before memory',
    'production_validation_record': 'diagnostics-66.json',
    'recognition_frontier_record_reused': 'diagnostics-64.json',
    'only_modified_runtime_source': source['only_modified_runtime_source'],
    'source_inputs_frozen': {'candidate': len(hashes), 'baseline': len(base['source_hashes']), 'main': len(native['source_hashes'])},
    'input_scope': 'The restored 261 non-Rust configuration/fixture files are frozen in the candidate but excluded from the generic adapter comparison. The scalar probe consumes none of those files. All shared inherited inputs are compared; only algebra.rs differs.',
    'independent_identity_oracle_reused': wrapper['reused_independent_oracle'],
    'matched_processes_per_variant_per_mode': 5,
    'fixtures_per_timing_batch': 4096,
    'fixtures_per_allocation_batch': 1000,
    'witness_rows_preserved_per_mode': 24,
    'timing_summary': {'rows': 24, 'slower_medians': sum(ratio > 1 for ratio in ratios), 'ratio_min': min(ratios), 'ratio_median': statistics.median(ratios), 'ratio_max': max(ratios)},
    'timing_rows': timing,
    'allocation_rows': allocation,
    'allocation_metrics_all_nonincreasing': all(delta <= 0 for row in allocation for delta in row['candidate_minus_baseline'].values()),
    'allocation_saving_rows': sum(row['candidate_minus_baseline']['allocation_calls'] < 0 for row in allocation),
    'allocation_samples_repeat_exactly': all(row['all_five_samples_identical_per_variant'] for row in allocation),
    'binary_delta_bytes': {mode: manifest['binaries']['candidate-' + mode]['bytes'] - manifest['binaries']['baseline-' + mode]['bytes'] for mode in ['time', 'memory']},
    'resource_review': 'Fifteen of 24 production timing medians are slower, with a median ratio of 1.0199 and maximum 1.1053. Conjugate construction, first and cloned queries regress 3.5%, 10.5% and 4.6% with nonoverlapping sample ranges; the warm median is 9.6% slower with overlap. Candidate outliers and all individual samples remain retained. Five cloned rows each save 667 allocations and 42,688 requested bytes per 1,000 fixtures, with no higher retained/peak bytes. Both executables grow 3,344 bytes. The production timing evidence rejects this implementation despite the earlier native microbenchmark gate and passing correctness suites. No causal explanation for the timing differences is established.',
    'production_report_replay_run': False,
    'production_report_replay_not_run_reason': 'The matched production resource gate rejects the implementation before the next admission stage. Its unused reproduction script remains available; no report preservation is claimed for this candidate.',
    'main_runtime_unchanged': True,
    'next_step': 'Keep the tested root omission out of production. Evaluate a new local-memo index borrowing candidate against the original admitted root behavior, with focused frontier/resource gates before broader validation; see memo-index-candidate.json. A runtime boolean root flag is a separate unmeasured alternative to the current const specialization, not an established fix.',
    'limitations': 'These are scoped scalar identities and resource samples, not complete application or cross-library measurements. Sample ranges do not establish statistical significance. Every adverse sample is retained. No property/target or native-stroke admission is completed.',
}
paths = [COST / name for name in ['manifest.json', 'wrapper-status.json', 'retained-libraries.json', 'commands.json', 'probe.rs', 'reproduce.py', 'wrapper.py']]
paths += [PRODUCTION / 'scalar-cost-inputs' / name for name in ['status.json', 'libraries.json', 'baseline-sources.json.gz', 'candidate-sources.json.gz', 'baseline-library.log', 'candidate-library.log']]
paths += [EVIDENCE / name for name in ['analyze_laurent_root_production_cost.py', 'reproduce_laurent_root_production_cost.py', 'diagnostics-66.json']]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-67.json').write_text(json.dumps(record, indent=2) + '\n')
print('Timing summary:', record['timing_summary'])
print('Nonoverlapping sample ranges:', [row for row in timing if row['nonoverlapping_ranges']])
print('Allocation savings:', [row for row in allocation if any(row['candidate_minus_baseline'].values())])
print('Binary deltas:', record['binary_delta_bytes'])
