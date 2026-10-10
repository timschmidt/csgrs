"""Audit the initial static-consumer resource probe and its one fixed follow-up."""
import gzip
import hashlib
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
FOCUSED = EVIDENCE / 'laurent-memo-consumer'
COST = EVIDENCE / 'laurent-memo-consumer-cost'
FOLLOWUP = EVIDENCE / 'laurent-memo-consumer-confirmation'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
directories = {
    'baseline': ROOT / 'target/laurent-memo-index/baseline/source',
    'candidate': ROOT / 'target/laurent-memo-consumer/candidate/source',
}
records = {name: read(FOCUSED / (name + '-sources.json')) for name in directories}
hashes = {}
for name, record in records.items():
    assert digest(Path(record['source_hashes_file'])) == record['source_hashes_file_sha256']
    hashes[name] = json.loads(gzip.decompress(Path(record['source_hashes_file']).read_bytes()))
    assert len(hashes[name]) == record['source_files']
    assert all(digest(directories[name] / path) == value for path, value in hashes[name].items())
native = read(Path(records['baseline']['main_source_record']))
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
focused = read(EVIDENCE / 'diagnostics-70.json')
assert focused['frontier_receipts_byte_identical'] == 19140
for path, record in focused['artifacts'].items():
    assert digest(EVIDENCE / path) == record['sha256']
wrapper = read(COST / 'wrapper-status.json')
assert wrapper['exit_code'] == 0 and wrapper['all_baseline_candidate_and_main_sources_unchanged']
assert digest(Path(wrapper['frontier_record'])) == wrapper['frontier_record_sha256']
assert digest(Path(wrapper['reused_independent_oracle'])) == wrapper['oracle_sha256']
assert read(Path(wrapper['reused_independent_oracle']))['identities'] == 126
for name, files in wrapper['source_adapters'].items():
    for record in files.values():
        assert digest(Path(record['path'])) == record['sha256']
    assert read(Path(files['sources.json']['path']))['source_hashes'] == hashes[name]
initial = read(COST / 'manifest.json')
followup = read(FOLLOWUP / 'manifest.json')
assert initial['source_change'] == ['crates/hyperreal/src/computable/node/algebra.rs']
assert len(initial['trials']) == 20 and len(initial['commands']) == 28
assert len(followup['trials']) == len(followup['commands']) == 20
assert all(row['exit_code'] == 0 for record in [initial, followup] for row in record['commands'])
assert digest(COST / 'manifest.json') == followup['initial_record_sha256']
assert followup['source_and_binary_hashes_unchanged']
witness = lambda trial: [row for row in trial['rows'] if 'witness' in row or 'fixture' in row]
for mode in ['time', 'memory']:
    trials = [trial for trial in initial['trials'] if trial['mode'] == mode]
    expected = witness(trials[0])
    assert len(trials) == 10 and len(expected) == 24
    for folder, group in [(COST, trials), (FOLLOWUP, followup['trials'] if mode == 'time' else [])]:
        for trial in group:
            path = folder / f"{mode}-{trial['index']}-{trial['variant']}.jsonl"
            assert [json.loads(line) for line in path.read_text().splitlines()] == trial['rows']
            assert witness(trial) == expected
            assert all(row['iterations'] == (4096 if mode == 'time' else 1000) for row in trial['rows'] if 'name' in row)
retained = read(COST / 'retained-libraries.json')
for name, record in retained['libraries'].items():
    assert record['sha256'] == initial['libraries'][name]['sha256']
    assert digest(Path(record['retained_path'])) == record['sha256']
for name, record in initial['binaries'].items():
    assert digest(COST / name) == record['sha256']
for name, record in followup['binaries'].items():
    assert record['sha256'] == initial['binaries'][name + '-time']['sha256']


def timing_rows(trials):
    names = [row['name'] for row in trials[0]['rows'] if 'name' in row]
    rows = []
    for name in names:
        samples = {
            variant: [next(row['nanos'] for row in trial['rows'] if row.get('name') == name)
                      for trial in trials if trial['variant'] == variant]
            for variant in directories
        }
        before, after = samples['baseline'], samples['candidate']
        rows.append({
            'name': name, 'samples_batch_ns': samples,
            'ratio': statistics.median(after) / statistics.median(before),
            'medians_ns_per_fixture': {key: statistics.median(values) / 4096 for key, values in samples.items()},
            'nonoverlapping_ranges': max(before) < min(after) or max(after) < min(before),
        })
    return rows


def summary(rows):
    ratios = [row['ratio'] for row in rows]
    return {
        'rows': len(rows), 'improved_medians': sum(value < 1 for value in ratios),
        'slower_medians': sum(value > 1 for value in ratios),
        'ratio_min': min(ratios), 'ratio_median': statistics.median(ratios), 'ratio_max': max(ratios),
        'nonoverlapping_regressions': [row['name'] for row in rows if row['ratio'] > 1 and row['nonoverlapping_ranges']],
    }


time_trials = [trial for trial in initial['trials'] if trial['mode'] == 'time']
initial_time = timing_rows(time_trials)
followup_time = timing_rows(followup['trials'])
combined_time = timing_rows(time_trials + followup['trials'])
for row, archived in zip(followup_time, followup['summaries'], strict=True):
    assert row['name'] == archived['name'] and row['samples_batch_ns'] == archived['samples_batch_ns']
    assert row['ratio'] == archived['ratio'] and row['nonoverlapping_ranges'] == archived['nonoverlapping_ranges']
allocations = []
for row in initial['summaries']:
    if row['mode'] != 'memory':
        continue
    before, after = row['variants']['baseline'], row['variants']['candidate']
    for variant, fields in row['variants'].items():
        for field, stats in fields.items():
            samples = [next(item[field] for item in trial['rows'] if item.get('name') == row['name'])
                       for trial in initial['trials'] if trial['mode'] == 'memory' and trial['variant'] == variant]
            assert samples == stats['samples'] and statistics.median(samples) == stats['median']
            assert len(set(samples)) == 1
    allocations.append({
        'name': row['name'],
        'candidate_minus_baseline_per_1000_fixtures': {field: after[field]['median'] - before[field]['median'] for field in before},
    })
assert len(initial_time) == len(followup_time) == len(combined_time) == len(allocations) == 24
assert all(value <= 0 for row in allocations for value in row['candidate_minus_baseline_per_1000_fixtures'].values())
assert sum(row['candidate_minus_baseline_per_1000_fixtures']['allocation_calls'] < 0 for row in allocations) == 9
assert all(row['candidate_minus_baseline_per_1000_fixtures'][field] == 0
           for row in allocations for field in ['retained_bytes_delta', 'peak_bytes_delta'])
record = {
    'decision': 'advance static consumers to full production validation and fresh production resource/report gates; no runtime admission',
    'focused_correctness_record': 'diagnostics-70.json',
    'only_runtime_change': initial['source_change'],
    'frontier_receipts_preserved': 19140, 'independent_fraction_identities': 126,
    'initial_processes_per_variant_per_mode': 5, 'additional_timing_processes_per_variant': 10,
    'fixtures_per_timing_batch': 4096, 'fixtures_per_allocation_batch': 1000,
    'witness_rows_preserved_per_mode': 24,
    'initial_timing_summary': summary(initial_time), 'initial_timing_rows': initial_time,
    'followup_timing_summary': summary(followup_time), 'followup_timing_rows': followup_time,
    'combined_timing_summary': summary(combined_time), 'combined_timing_rows': combined_time,
    'allocation_rows': allocations,
    'allocation_samples_identical_per_variant': True,
    'binary_delta_bytes': {mode: initial['binaries']['candidate-' + mode]['bytes'] - initial['binaries']['baseline-' + mode]['bytes'] for mode in ['time', 'memory']},
    'resource_review': 'Initial medians improve in 17/24 rows, but radical-Laurent construction regresses 3.3% with nonoverlapping ranges. One fixed ten-pair follow-up uses unchanged executables, workload and CPU, reverses the first pair, and retains every row and outlier. Its construction ratio is 1.0093 with overlap; the combined ratio is 1.0242. Across all 15 samples per variant, 16/24 medians improve, eight worsen, and ratios range from 0.9629 to 1.0246 with overlapping ranges throughout. This neither establishes statistical significance nor dismisses adverse rows. Nine first/warm/cloned query rows save 1-3 allocations per fixture; retained and peak bytes never change. Both executables grow 13,808 bytes. This is sufficient to investigate the unchanged implementation under broader production gates, not sufficient to admit it or claim an application speedup.',
    'next_step': 'Remove only the test-only frontier module in a new isolated source tree. Run full default/all-feature scalar, curve and DRC tests, strict lint/docs/format, the original board gate, then fresh production costs and complete historical/report replay. Preserve this exact implementation and both initial/follow-up records.',
    'broader_validation_or_report_replay_run': False,
    'main_runtime_unchanged': True,
    'limitations': 'Focused correctness and six identity families do not establish full representation coverage. Timing measurements are scoped and noisy; no further repetitions of this probe are planned to seek favorable results. The original native gate, wider prototype and all remaining plan scope stay open.',
}
paths = [path for folder in [COST, FOLLOWUP] for path in sorted(folder.iterdir()) if path.is_file() and path.name not in initial['binaries']]
paths += [EVIDENCE / name for name in ['diagnostics-70.json', 'reproduce_laurent_memo_consumer_cost.py', 'reproduce_laurent_memo_consumer_confirmation.py', 'analyze_laurent_memo_consumer_cost.py']]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-71.json').write_text(json.dumps(record, indent=2) + '\n')
print('Static-consumer combined timing:', record['combined_timing_summary'])
print('Nine allocation-saving rows; unchanged retained/peak bytes. ELF deltas:', record['binary_delta_bytes'])
