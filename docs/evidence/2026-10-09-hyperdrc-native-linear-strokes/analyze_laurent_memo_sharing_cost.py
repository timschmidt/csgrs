"""Reject the first sharing representation using its completed matched resource gate."""
import hashlib
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'laurent-memo-sharing-cost'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
manifest = read(OUT / 'manifest.json')
assert len(manifest['trials']) == 20 and len(manifest['commands']) == 28
assert all(row['exit_code'] == 0 for row in manifest['commands'])
witnesses = lambda trial: [row for row in trial['rows'] if 'witness' in row or 'fixture' in row]
for mode in ['time', 'memory']:
    trials = [trial for trial in manifest['trials'] if trial['mode'] == mode]
    assert len(witnesses(trials[0])) == 24
    assert all(witnesses(trial) == witnesses(trials[0]) for trial in trials)
ratios = [row['variants']['candidate']['nanos']['median'] / row['variants']['baseline']['nanos']['median'] for row in manifest['summaries'] if row['mode'] == 'time']
assert len(ratios) == 24 and sum(ratio > 1 for ratio in ratios) == 20
for row in manifest['summaries']:
    if row['mode'] == 'memory':
        assert all(len(set(value[field]['samples'])) == 1 for value in row['variants'].values() for field in value)
assert read(EVIDENCE / 'laurent-memo-sharing/independent-identities.json')['identities'] == 126
native = read(EVIDENCE / 'native-exactness-gate/sources.json')
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
for value in manifest['libraries'].values():
    assert digest(Path(value['path'])) == value['sha256']
for name, value in manifest['binaries'].items():
    assert digest(OUT / name) == value['sha256']
record = {
    'decision': 'not retained: term-vector sharing does not improve the measured performance/resource tradeoff',
    'focused_record': 'diagnostics-60.json',
    'independent_exact_fraction_identities': 126,
    'matched_processes_per_variant_per_mode': 5,
    'witness_rows_preserved_per_mode': 24,
    'timing_rows': {'count': 24, 'slower_medians': 20, 'ratio_min': min(ratios), 'ratio_median': statistics.median(ratios), 'ratio_max': max(ratios)},
    'decisive_rows': [
        {'name': row['name'], 'mode': row['mode'], 'variants': row['variants']}
        for row in manifest['summaries'] if row['name'] in ['opaque_linear_atom_cloned', 'quadratic_reciprocal_first', 'conjugate_product_first', 'multiradical_square_first', 'pi_parameter_transport_first']
    ],
    'allocation_review': 'All five samples repeat exactly. Multiradical-square and pi-transport queries save one allocation per query, but quadratic-reciprocal and conjugate-product queries add one. Every changed query family increases peak requested bytes; retained byte deltas are unchanged.',
    'timing_review': 'Twenty of twenty-four medians regress. Opaque cloned queries are 7.45% slower with nonoverlapping sample ranges; most other ranges overlap. The overall matched result does not justify keeping this performance experiment.',
    'probe_elf_bytes': {key: value['bytes'] for key, value in manifest['binaries'].items()},
    'binary_delta_bytes': {mode: manifest['binaries']['candidate-' + mode]['bytes'] - manifest['binaries']['baseline-' + mode]['bytes'] for mode in ['time', 'memory']},
    'main_runtime_unchanged': True,
    'candidate_full_suites_or_board_gate_run': False,
    'analysis_note': 'An initial inline analysis compared timing witnesses (4096 inputs) with allocation witnesses (1000 inputs), so its assertion failed before writing a record. Correct comparisons are within each mode; all original measured output is preserved.',
    'next_step': 'Evaluate sharing the complete immutable polynomial result at memo boundaries, preserving the existing owned Vec operations and opaque ownership. This is a new unmeasured hypothesis, not an attributed cause or promised speedup.',
    'limitations': 'The exact identities and differential frontier corpus are scoped evidence. No full-application claim or native admission is made. The original native board gate still fails and all broader properties/targets remain open.',
}
paths = [OUT / 'manifest.json', OUT / 'wrapper-status.json', EVIDENCE / 'reproduce_laurent_memo_sharing_cost.py', EVIDENCE / 'laurent-memo-sharing/independent-identities.json', Path(__file__)]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-61.json').write_text(json.dumps(record, indent=2) + '\n')
print('Recorded rejection: twenty slower medians, two query families adding allocations; no runtime change retained.')
