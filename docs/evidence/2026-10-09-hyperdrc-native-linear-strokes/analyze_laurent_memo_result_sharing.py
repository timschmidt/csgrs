"""Combine the whole-result sharing frontier and resource checks."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import statistics
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'laurent-memo-result-sharing'
COST = EVIDENCE / 'laurent-memo-result-sharing-cost'
SOURCE = ROOT / 'target/laurent-memo-result-sharing/candidate/source'
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
assert len(ratios) == 24 and sum(value > 1 for value in ratios) == 14
memory = {row['name']: row['variants'] for row in manifest['summaries'] if row['mode'] == 'memory'}
previous = {row['name']: row['variants'] for row in read(EVIDENCE / 'laurent-memo-sharing-cost/manifest.json')['summaries'] if row['mode'] == 'memory'}
assert memory == previous
record = {
    'decision': 'not retained: whole-result sharing preserves semantics but does not establish an overall resource improvement',
    'previous_experiment': 'diagnostics-61.json',
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
    'timing_rows': {'count': 24, 'slower_medians': 14, 'ratio_min': min(ratios), 'ratio_median': statistics.median(ratios), 'ratio_max': max(ratios)},
    'timing_review': 'Fourteen medians regress; all three quadratic-reciprocal and all three opaque-atom query medians are slower. Most ranges overlap. The clearly faster three radical-transport queries operate on inputs already reduced by construction, at about four nanoseconds per query; they do not demonstrate faster normalization. Opaque-atom construction is 2.6% slower with nonoverlapping sample ranges. These measurements do not justify retaining the added representation.',
    'allocation_review': 'All 24 allocation rows, both variants and all repeated samples, are identical to the first representation trial. The extra allocation per quadratic-reciprocal/conjugate-product query and higher peak requested bytes remain.',
    'binary_delta_bytes': {mode: manifest['binaries']['candidate-' + mode]['bytes'] - manifest['binaries']['baseline-' + mode]['bytes'] for mode in ['time', 'memory']},
    'main_runtime_unchanged': True,
    'candidate_full_suites_or_board_gate_run': False,
    'next_step': 'Evaluate caller-directed ownership in this per-call memo. A newly shared result that is immediately consumed must currently clone because the memo retains another reference. Preserve ordinary owned return/storage for consuming callers and consider sharing only borrowed results or borrowed memo hits. Keep opaque ownership, key order and all work/depth/term frontiers unchanged; this next design is unimplemented and unmeasured.',
    'limitations': 'This is scoped differential, exact-identity and microbenchmark evidence. The two candidates were each paired against the same frozen baseline in separate sessions, not directly paired against each other. Full native admission, its original runtime gate and the complete properties.md objective remain open.',
}
paths = [path for path in sorted(OUT.iterdir()) if path.is_file()]
paths += [COST / 'manifest.json', COST / 'wrapper-status.json']
paths += [EVIDENCE / name for name in ['prepare_laurent_memo_result_sharing.py', 'reproduce_laurent_memo_result_sharing_validation.py', 'reproduce_laurent_memo_result_sharing_cost.py', 'analyze_laurent_memo_result_sharing.py']]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-62.json').write_text(json.dumps(record, indent=2) + '\n')
print('Whole-result sharing rejected: 19,140 identical frontier receipts, but the allocation regressions persist and no overall speedup is established.')
