"""Summarize every production timing and allocation row without hiding regressions."""
import gzip
import hashlib
import json
from pathlib import Path
import statistics
import subprocess

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
PRODUCTION = EVIDENCE / 'laurent-shared-production'
COST = EVIDENCE / 'laurent-shared-production-cost'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
review = read(COST / 'review.json')
assert review['decision'] in ['advance to full production validation; no runtime admission', 'not retained: production resource tradeoff fails']
source = read(PRODUCTION / 'sources.json')
base = read(Path(source['base_source_record']))
native = read(Path(source['main_source_record']))
hashes = base['source_hashes'] | source['changed_source_hashes'] | source['additional_source_hashes']
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
assert all(digest(ROOT / 'target/laurent-shared-production/source' / name) == value for name, value in hashes.items())
assert all(digest(ROOT / 'target/contact-index-directed-width/source' / name) == value for name, value in base['source_hashes'].items())
wrapper = read(COST / 'wrapper-status.json')
assert wrapper['exit_code'] == 0 and wrapper['all_production_and_baseline_inputs_unchanged']
assert digest(Path(wrapper['focused_validation'])) == wrapper['focused_validation_sha256']
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
    for trial in trials:
        raw = COST / f"{mode}-{trial['index']}-{trial['variant']}.jsonl"
        assert [json.loads(line) for line in raw.read_text().splitlines()] == trial['rows']
retained = read(COST / 'retained-libraries.json')
for name, row in retained['libraries'].items():
    assert row['sha256'] == manifest['libraries'][name]['sha256']
    assert digest(Path(row['retained_path'])) == row['sha256']
for name, row in manifest['binaries'].items():
    assert digest(COST / name) == row['sha256']
layout = read(COST / 'code-layout.json')
for variant in layout['variants'].values():
    assert digest(ROOT / variant['command'][-1]) == variant['binary_sha256']
    raw = subprocess.check_output(variant['command'], cwd=ROOT, text=True)
    symbols = []
    for line in raw.splitlines():
        if any(pattern in line for pattern in ['pi_laurent_polynomial_with_consumer', 'pi_laurent_polynomial_with_memo', 'pi_laurent_compound_polynomial', 'PiLaurentConsumer']):
            address, size, kind, symbol = line.split(maxsplit=3)
            symbols.append({'address_hex': address, 'bytes': int(size, 16), 'kind': kind, 'symbol': symbol})
    assert symbols == variant['symbols']
    assert sum(row['bytes'] for row in symbols) == variant['matching_symbol_bytes']
assert not (ROOT / 'target/laurent-shared-production/source' / source['removed_test_only_source']).exists()
focused = read(Path(source['trial_source_record']))
assert (ROOT / 'target/laurent-shared-production/source' / source['only_modified_runtime_source']).read_text() + source['removed_test_only_declaration'] == (Path(focused['candidate_source_directory']) / source['only_modified_runtime_source']).read_text()
timing = []
allocation = []
for row in manifest['summaries']:
    for variant, fields in row['variants'].items():
        for field, stats in fields.items():
            samples = [next(item[field] for item in trial['rows'] if item.get('name') == row['name']) for trial in manifest['trials'] if trial['variant'] == variant and trial['mode'] == row['mode']]
            assert samples == stats['samples'] and statistics.median(samples) == stats['median']
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
record = {
    'decision': review['decision'],
    'focused_validation_record': 'diagnostics-74.json',
    'recognition_frontier_record_reused': 'diagnostics-74.json',
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
    'resource_review': review['resource_review'],
    'full_production_validation_run': False,
    'production_report_replay_run': False,
    'production_source_has_no_diagnostic_module': not (ROOT / 'target/laurent-shared-production/source' / source['removed_test_only_source']).exists(),
    'code_layout': read(COST / 'code-layout.json'),
    'main_runtime_unchanged': True,
    'next_step': review['next_step'],
    'limitations': 'These are scoped scalar identities and resource samples, not complete application or cross-library measurements. Sample ranges do not establish statistical significance. Every adverse sample is retained. No property/target or native-stroke admission is completed.',
}
paths = [path for path in sorted(COST.iterdir()) if path.is_file() and path.name not in manifest['binaries']]
paths += [path for path in sorted((PRODUCTION / 'scalar-cost-inputs').iterdir()) if path.is_file()]
paths += [EVIDENCE / name for name in ['analyze_laurent_shared_production_cost.py', 'reproduce_laurent_shared_production_cost.py', 'diagnostics-74.json']]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-75.json').write_text(json.dumps(record, indent=2) + '\n')
print('Timing summary:', record['timing_summary'])
print('Nonoverlapping sample ranges:', [(row['name'], round(row['ratio'], 4)) for row in timing if row['nonoverlapping_ranges']])
print('Allocation savings:', [(row['name'], row['candidate_minus_baseline']) for row in allocation if any(row['candidate_minus_baseline'].values())])
print('Binary deltas:', record['binary_delta_bytes'])
