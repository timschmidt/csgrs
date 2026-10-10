"""Record caller-directed sharing and its unfavorable matched timing result."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import statistics
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'laurent-memo-use-sharing'
COST = EVIDENCE / 'laurent-memo-use-sharing-cost'
SOURCE = ROOT / 'target/laurent-memo-use-sharing/candidate/source'
read = lambda path: json.loads(path.read_text())
sha = lambda data: hashlib.sha256(data).hexdigest()
digest = lambda path: sha(path.read_bytes())
source = read(OUT / 'candidate-sources.json')
native = read(Path(source['native_source_record']))
baseline = read(Path(source['base_source_record']))
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
assert all(digest(SOURCE / name) == value for name, value in source['source_hashes'].items())
changes = [name for name, value in source['source_hashes'].items() if baseline['source_hashes'].get(name) != value]
assert changes == ['crates/hyperreal/src/computable/node/algebra.rs']
with tarfile.open(OUT / 'candidate-source-delta.tar.gz') as archive:
    assert archive.getnames() == changes
    assert sha(archive.extractfile(changes[0]).read()) == source['source_hashes'][changes[0]]
status = read(OUT / 'status.json')
assert len(status) == 4 and all(row['exit_code'] == 0 for row in status)
frontiers = read(OUT / 'frontiers.json')
for variant, record in frontiers.items():
    compressed = OUT / (variant + '-frontiers.jsonl.gz')
    assert digest(compressed) == record['compressed_sha256']
    data = gzip.decompress(compressed.read_bytes())
    assert len(data) == record['uncompressed_bytes'] and sha(data) == record['uncompressed_sha256']
assert frontiers['baseline']['uncompressed_sha256'] == frontiers['candidate']['uncompressed_sha256']
log = gzip.decompress((OUT / 'candidate-normal-form-tests.log.gz').read_bytes())
assert sha(log) == frontiers['candidate']['raw_log']['sha256']
assert re.findall(rb'test result: ok\. (\d+) passed; 0 failed; 0 ignored', log) == [b'4']
assert read(OUT / 'frontier-comparison.json')['different_rows'] == 0
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
assert len(ratios) == 24 and sum(value > 1 for value in ratios) == 19
memory = {row['name']: row['variants'] for row in manifest['summaries'] if row['mode'] == 'memory'}
previous = {row['name']: row['variants'] for row in read(EVIDENCE / 'laurent-memo-sharing-cost/manifest.json')['summaries'] if row['mode'] == 'memory'}
assert all(row['candidate']['allocation_calls']['median'] <= row['baseline']['allocation_calls']['median'] for row in memory.values())
assert all(row['candidate']['retained_bytes_delta']['median'] == row['baseline']['retained_bytes_delta']['median'] for row in memory.values())
record = {
    'decision': 'not retained: caller-directed sharing removes allocation regressions but worsens measured query performance',
    'previous_experiment': 'diagnostics-62.json',
    'native_baseline_gate': 'diagnostics-59.json',
    'source_change': changes,
    'focused_tests_passed': 4,
    'strict_clippy_and_format_passed': True,
    'frontier_rows_identical': 19140,
    'frontier_sha256': frontiers['candidate']['uncompressed_sha256'],
    'baseline_frontier_validation_reused': 'diagnostics-60.json',
    'independent_identity_oracle_reused': 'laurent-memo-sharing/independent-identities.json',
    'matched_processes_per_variant_per_mode': 5,
    'witness_rows_preserved_per_mode': 24,
    'timing_rows': {'count': 24, 'slower_medians': 19, 'ratio_min': min(ratios), 'ratio_median': statistics.median(ratios), 'ratio_max': max(ratios)},
    'timing_review': 'Nineteen of twenty-four medians regress, with median ratio 1.0334 and range 0.9840 to 1.0778. Quadratic-reciprocal first/warm and opaque-atom first/warm queries are slower with nonoverlapping sample ranges. No performance improvement is established.',
    'allocation_review': 'No row adds allocation calls or retained bytes. Multiradical queries save one allocation each and pi-transport queries save two; the two earlier extra-allocation families now match baseline counts. Nine query rows still add 40 peak requested bytes. Lower allocation traffic does not outweigh the measured performance regressions under the governing ladder.',
    'binary_delta_bytes': {mode: manifest['binaries']['candidate-' + mode]['bytes'] - manifest['binaries']['baseline-' + mode]['bytes'] for mode in ['time', 'memory']},
    'main_runtime_unchanged': True,
    'candidate_full_suites_or_board_gate_run': False,
    'next_step': 'The normalizer entry point discards its per-call memo after receiving the final result. Evaluate omitting only the final root memo insertion with the existing owned representation; preserve every recursive memo entry, depth/work limit and exact output. This dead-store hypothesis is unimplemented and unmeasured.',
    'limitations': 'This is scoped differential, exact-identity and microbenchmark evidence. Each sharing candidate was paired against the same frozen baseline in a separate session; candidates were not directly paired against each other. Full native admission, its original runtime gate and the complete properties.md objective remain open.',
}
paths = [path for path in sorted(OUT.iterdir()) if path.is_file()]
paths += [COST / 'manifest.json', COST / 'wrapper-status.json']
paths += [EVIDENCE / name for name in ['prepare_laurent_memo_use_sharing.py', 'reproduce_laurent_memo_use_sharing_validation.py', 'reproduce_laurent_memo_use_sharing_cost.py', 'analyze_laurent_memo_use_sharing.py']]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-63.json').write_text(json.dumps(record, indent=2) + '\n')
print('Caller-directed sharing rejected: recognition receipts match and allocation regressions are removed, but query timings worsen.')
