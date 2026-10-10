"""Retain the indexed-memo resource regression and all favorable/adverse rows."""
import gzip
import hashlib
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
FOCUSED = EVIDENCE / 'laurent-memo-index'
COST = EVIDENCE / 'laurent-memo-index-cost'
WORK = ROOT / 'target/laurent-memo-index'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
records = {name: read(FOCUSED / (name + '-sources.json')) for name in ['baseline', 'candidate']}
hashes = {}
for name, record in records.items():
    assert digest(Path(record['source_hashes_file'])) == record['source_hashes_file_sha256']
    hashes[name] = json.loads(gzip.decompress(Path(record['source_hashes_file']).read_bytes()))
    assert all(digest(WORK / name / 'source' / path) == value for path, value in hashes[name].items())
native = read(Path(records['baseline']['main_source_record']))
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
focused = read(EVIDENCE / 'diagnostics-68.json')
assert focused['candidate_frontier_rows_byte_identical'] == 19140
wrapper = read(COST / 'wrapper-status.json')
assert wrapper['exit_code'] == 0 and wrapper['all_baseline_candidate_and_main_sources_unchanged']
assert digest(Path(wrapper['frontier_record'])) == wrapper['frontier_record_sha256']
assert digest(Path(wrapper['reused_independent_oracle'])) == wrapper['oracle_sha256']
assert read(Path(wrapper['reused_independent_oracle']))['identities'] == 126
for name, files in wrapper['source_adapters'].items():
    for filename, record in files.items():
        assert digest(Path(record['path'])) == record['sha256']
    assert read(Path(files['sources.json']['path']))['source_hashes'] == hashes[name]
manifest = read(COST / 'manifest.json')
assert manifest['source_change'] == ['crates/hyperreal/src/computable/node/algebra.rs']
assert len(manifest['trials']) == 20 and len(manifest['commands']) == 28
assert all(row['exit_code'] == 0 for row in manifest['commands'])
for mode in ['time', 'memory']:
    trials = [trial for trial in manifest['trials'] if trial['mode'] == mode]
    witness = lambda trial: [row for row in trial['rows'] if 'witness' in row or 'fixture' in row]
    assert len(trials) == 10 and len(witness(trials[0])) == 24
    assert all(witness(trial) == witness(trials[0]) for trial in trials)
    assert all(row['iterations'] == (4096 if mode == 'time' else 1000) for trial in trials for row in trial['rows'] if 'name' in row)
retained = read(COST / 'retained-libraries.json')
for name, record in retained['libraries'].items():
    assert record['sha256'] == manifest['libraries'][name]['sha256']
    assert digest(Path(record['retained_path'])) == record['sha256']
for name, record in manifest['binaries'].items():
    assert digest(COST / name) == record['sha256']
timing = []
allocation = []
for row in manifest['summaries']:
    old, new = row['variants']['baseline'], row['variants']['candidate']
    if row['mode'] == 'time':
        before, after = old['nanos'], new['nanos']
        timing.append({
            'name': row['name'], 'ratio': after['median'] / before['median'],
            'baseline_median_ns_per_fixture': before['median'] / 4096,
            'candidate_median_ns_per_fixture': after['median'] / 4096,
            'baseline_range_batch_ns': [min(before['samples']), max(before['samples'])],
            'candidate_range_batch_ns': [min(after['samples']), max(after['samples'])],
            'nonoverlapping_ranges': max(before['samples']) < min(after['samples']) or max(after['samples']) < min(before['samples']),
        })
    else:
        allocation.append({
            'name': row['name'],
            'candidate_minus_baseline_per_1000_fixtures': {field: new[field]['median'] - old[field]['median'] for field in old},
            'all_five_samples_identical_per_variant': all(len(set(stats['samples'])) == 1 for fields in [old, new] for stats in fields.values()),
        })
ratios = [row['ratio'] for row in timing]
assert len(timing) == len(allocation) == 24 and sum(ratio > 1 for ratio in ratios) == 20
assert all(row['all_five_samples_identical_per_variant'] for row in allocation)
assert all(delta <= 0 for row in allocation for delta in row['candidate_minus_baseline_per_1000_fixtures'].values())
assert sum(row['candidate_minus_baseline_per_1000_fixtures']['allocation_calls'] < 0 for row in allocation) == 9
record = {
    'decision': 'not retained: indexed local-memo borrowing reduces allocations but substantially regresses production query timing',
    'focused_correctness_record': 'diagnostics-68.json',
    'only_runtime_change': manifest['source_change'],
    'fresh_production_baseline': True,
    'original_root_memo_behavior_preserved': True,
    'frontier_receipts_preserved': 19140,
    'independent_identity_oracle_reused': wrapper['reused_independent_oracle'],
    'matched_processes_per_variant_per_mode': 5,
    'fixtures_per_timing_batch': 4096, 'fixtures_per_allocation_batch': 1000,
    'witness_rows_preserved_per_mode': 24,
    'timing_summary': {'rows': 24, 'slower_medians': 20, 'ratio_min': min(ratios), 'ratio_median': statistics.median(ratios), 'ratio_max': max(ratios)},
    'timing_rows': timing,
    'allocation_rows': allocation,
    'binary_delta_bytes': {mode: manifest['binaries']['candidate-' + mode]['bytes'] - manifest['binaries']['baseline-' + mode]['bytes'] for mode in ['time', 'memory']},
    'resource_review': 'Twenty of 24 medians are slower; the median ratio is 1.1398 and maximum 1.2739. Thirteen query rows regress with nonoverlapping sample ranges. The already-reduced radical-transport query controls improve and are retained alongside all adverse rows. First/warm/cloned queries save two allocations per multiradical fixture, three per pi-transport fixture, and one per opaque-linear fixture. Other allocation metrics never increase; retained and peak bytes remain identical in all rows. These lower-priority savings do not admit the measured performance regression. No causal explanation or statistical significance is claimed.',
    'broader_validation_or_report_replay_run': False,
    'main_runtime_unchanged': True,
    'next_step': 'Keep the index-return carrier rejected. A statically selected result consumer is an unimplemented alternative that might avoid runtime return-carrier dispatch while preserving closed-only borrowing. Treat it as a new experiment with the same original-root, frontier and resource gates; do not assume the source-level hypothesis explains these timings.',
    'analysis_count_correction': 'laurent-memo-index-cost/analysis-correction.json',
    'limitations': 'Four focused tests per variant and deterministic frontier/identity corpora do not establish complete correctness. This rejected candidate was not subjected to full production/native suites or complete reports. The larger native prototype and all remaining properties/targets stay open.',
}
assert sum(row['ratio'] > 1 and row['nonoverlapping_ranges'] for row in timing) == 13
paths = [COST / name for name in ['manifest.json', 'wrapper-status.json', 'retained-libraries.json', 'commands.json', 'probe.rs', 'reproduce.py', 'wrapper.py', 'analysis-correction.json', 'analysis-before-count-correction.py']]
paths += [EVIDENCE / name for name in ['diagnostics-68.json', 'reproduce_laurent_memo_index_cost.py', 'analyze_laurent_memo_index_cost.py']]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-69.json').write_text(json.dumps(record, indent=2) + '\n')
print('Rejected memo-index borrowing:', record['timing_summary'])
print('Nine allocation-saving rows; unchanged retained/peak bytes. ELF deltas:', record['binary_delta_bytes'])
