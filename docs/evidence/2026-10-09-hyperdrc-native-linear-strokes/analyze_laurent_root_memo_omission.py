"""Verify unused root memo omission before broader native validation."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import statistics
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'laurent-root-memo-omission'
COST = EVIDENCE / 'laurent-root-memo-omission-cost'
SOURCE = ROOT / 'target/laurent-root-memo-omission/candidate/source'
read = lambda path: json.loads(path.read_text())
sha = lambda data: hashlib.sha256(data).hexdigest()
digest = lambda path: sha(path.read_bytes())
source = read(OUT / 'candidate-sources.json')
native = read(Path(source['native_source_record']))
baseline = read(Path(source['base_source_record']))
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
assert all(digest(SOURCE / name) == value for name, value in source['source_hashes'].items())
changes = [name for name, value in source['source_hashes'].items() if baseline['source_hashes'].get(name) != value]
assert set(changes) == {'crates/hyperreal/src/computable/node/algebra.rs', 'crates/hyperreal/src/computable/node/normal_form_memo_frontier_probe.rs'}
with tarfile.open(OUT / 'candidate-source-delta.tar.gz') as archive:
    assert archive.getnames() == changes
    for name in changes: assert sha(archive.extractfile(name).read()) == source['source_hashes'][name]
status = read(OUT / 'status.json')
assert len(status) == 4 and all(row['exit_code'] == 0 for row in status)
frontiers = read(OUT / 'frontiers.json')
for variant, record in frontiers.items():
    compressed = OUT / (variant + '-frontiers.jsonl.gz')
    assert digest(compressed) == record['compressed_sha256']
    data = gzip.decompress(compressed.read_bytes())
    assert len(data) == record['uncompressed_bytes'] and sha(data) == record['uncompressed_sha256']
comparison = read(OUT / 'frontier-comparison.json')
assert comparison['rows'] == 19140
assert comparison['unexpected_different_rows'] == 0
assert comparison['raw_different_rows'] == comparison['final_root_entries_omitted'] == 5220
helper = 'crates/hyperreal/src/computable/node/normal_form_memo_frontier_probe.rs'
old_helper = (ROOT / 'target/laurent-memo-sharing/baseline/source' / helper).read_text()
assert (SOURCE / helper).read_text() == old_helper.replace(source['probe_only_change']['before'], source['probe_only_change']['after'])
log = gzip.decompress((OUT / 'candidate-normal-form-tests.log.gz').read_bytes())
assert sha(log) == frontiers['candidate']['raw_log']['sha256']
assert re.findall(rb'test result: ok\. (\d+) passed; 0 failed; 0 ignored', log) == [b'4']
assert comparison['unexpected_different_rows'] == 0
manifest = read(COST / 'manifest.json')
assert len(manifest['trials']) == 20 and len(manifest['commands']) == 28
assert all(row['exit_code'] == 0 for row in manifest['commands'])
witnesses = lambda trial: [row for row in trial['rows'] if 'witness' in row or 'fixture' in row]
for mode in ['time', 'memory']:
    trials = [trial for trial in manifest['trials'] if trial['mode'] == mode]
    assert len(witnesses(trials[0])) == 24
    assert all(witnesses(trial) == witnesses(trials[0]) for trial in trials)
for value in manifest['libraries'].values():
    assert digest(Path(value['path'])) == value['sha256']
for name, value in manifest['binaries'].items():
    assert digest(COST / name) == value['sha256']
ratios = [row['variants']['candidate']['nanos']['median'] / row['variants']['baseline']['nanos']['median'] for row in manifest['summaries'] if row['mode'] == 'time']
assert len(ratios) == 24 and sum(value > 1 for value in ratios) == 11
memory = {row['name']: row['variants'] for row in manifest['summaries'] if row['mode'] == 'memory'}
previous = {row['name']: row['variants'] for row in read(EVIDENCE / 'laurent-memo-sharing-cost/manifest.json')['summaries'] if row['mode'] == 'memory'}
assert all(row['candidate'][field]['median'] <= row['baseline'][field]['median'] for row in memory.values() for field in row['baseline'])
assert all(row['candidate']['retained_bytes_delta']['median'] == row['baseline']['retained_bytes_delta']['median'] for row in memory.values())
assert all(row['candidate']['peak_bytes_delta']['median'] == row['baseline']['peak_bytes_delta']['median'] for row in memory.values())
assert sum(row['candidate']['allocation_calls']['median'] < row['baseline']['allocation_calls']['median'] for row in memory.values()) == 5
record = {
    'decision': 'advance unused root memo omission to broader validation; no runtime change admitted',
    'previous_experiment': 'diagnostics-63.json',
    'native_baseline_gate': 'diagnostics-59.json',
    'source_change': changes,
    'only_runtime_change': source['only_modified_runtime_source'],
    'semantic_argument': source['argument'],
    'focused_tests_passed': 4,
    'strict_clippy_and_format_passed': True,
    'frontier_outputs_work_and_recursive_entries_preserved': 19140,
    'unused_final_root_entries_omitted': 5220,
    'frontier_comparison': comparison,
    'frontier_sha256': frontiers['candidate']['uncompressed_sha256'],
    'baseline_frontier_validation_reused': 'diagnostics-60.json',
    'independent_identity_oracle_reused': 'laurent-memo-sharing/independent-identities.json',
    'matched_processes_per_variant_per_mode': 5,
    'witness_rows_preserved_per_mode': 24,
    'timing_rows': {'count': 24, 'slower_medians': 11, 'ratio_min': min(ratios), 'ratio_median': statistics.median(ratios), 'ratio_max': max(ratios)},
    'timing_review': 'Thirteen of twenty-four timing medians improve. Ratios range from 0.9499 to 1.0401, with median 0.9919. All five cloned-query families with allocation savings improve, but four of those sample ranges overlap. The conjugate warm query is 4.0% slower with overlapping ranges; two already-reduced radical-transport controls are about 0.06 ns slower with nonoverlapping ranges. No uniform or full-application speedup is claimed.',
    'allocation_review': 'No allocation metric regresses. Each of five cloned-query families removes 667 allocation calls and 42,688 requested bytes per 1,000 fixtures; their two-thirds non-leaf input fraction explains the count. Retained and peak requested bytes are unchanged in every row. All five memory samples repeat exactly.',
    'binary_delta_bytes': {mode: manifest['binaries']['candidate-' + mode]['bytes'] - manifest['binaries']['baseline-' + mode]['bytes'] for mode in ['time', 'memory']},
    'main_runtime_unchanged': True,
    'candidate_full_suites_or_board_gate_run': False,
    'next_step': 'Run the complete scalar suites, focused native curve regressions and the original unchanged full-board gate on the frozen isolated native candidate. Separate production-isolated validation and matched application evidence still gate any runtime admission.',
    'limitations': 'This is scoped differential, exact-identity and microbenchmark evidence. Earlier sharing candidates remain rejected; this trial restores the baseline owned representation and changes only root memoization. Full native admission, its original runtime gate and the complete properties.md objective remain open.',
}
paths = [path for path in sorted(OUT.iterdir()) if path.is_file()]
paths += [COST / 'manifest.json', COST / 'wrapper-status.json']
paths += [EVIDENCE / name for name in ['prepare_laurent_root_memo_omission.py', 'reproduce_laurent_root_memo_omission_validation.py', 'reproduce_laurent_root_memo_omission_cost.py', 'analyze_laurent_root_memo_omission.py']]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-64.json').write_text(json.dumps(record, indent=2) + '\n')
print('Unused root memo omission advances: 19,140 preserved outputs/work/recursive entries, 5,220 expected root-entry omissions and five allocation-saving query rows.')
