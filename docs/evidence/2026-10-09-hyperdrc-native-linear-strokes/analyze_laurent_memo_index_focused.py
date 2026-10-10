"""Audit the production memo-index candidate's focused correctness gate."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'laurent-memo-index'
WORK = ROOT / 'target/laurent-memo-index'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
sources = {name: read(OUT / (name + '-sources.json')) for name in ['baseline', 'candidate']}
hashes = {}
for name, record in sources.items():
    compressed = Path(record['source_hashes_file'])
    assert digest(compressed) == record['source_hashes_file_sha256']
    hashes[name] = json.loads(gzip.decompress(compressed.read_bytes()))
    assert len(hashes[name]) == record['source_files']
    assert all(digest(WORK / name / 'source' / filename) == value for filename, value in hashes[name].items())
    with tarfile.open(OUT / (name + '-source-delta.tar.gz')) as archive:
        assert set(archive.getnames()) == set(record['changes_from_parent_or_baseline'])
        for filename in archive.getnames():
            assert hashlib.sha256(archive.extractfile(filename).read()).hexdigest() == hashes[name][filename]
native = read(Path(sources['baseline']['main_source_record']))
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
changes = [name for name, value in hashes['candidate'].items() if hashes['baseline'].get(name) != value]
assert changes == ['crates/hyperreal/src/computable/node/algebra.rs']
helper = 'crates/hyperreal/src/computable/node/normal_form_memo_frontier_probe.rs'
assert hashes['baseline'][helper] == hashes['candidate'][helper] == digest(EVIDENCE / 'normal_form_memo_frontier_probe.rs')
status = read(OUT / 'status.json')
assert len(status) == 8 and all(row['exit_code'] == 0 for row in status)
complete = read(OUT / 'validation-complete.json')
assert complete['frontier_rows_identical'] == complete['historical_baseline_rows_identical'] == 19140
assert read(OUT / 'frontier-comparison.json')['different_rows'] == 0
assert read(OUT / 'historical-baseline-comparison.json')['different_rows'] == 0
frontiers = read(OUT / 'frontiers.json')
for name, record in frontiers.items():
    compressed = OUT / (name + '-frontiers.jsonl.gz')
    assert digest(compressed) == record['compressed_sha256']
    raw = gzip.decompress(compressed.read_bytes())
    assert len(raw) == record['uncompressed_bytes']
    assert hashlib.sha256(raw).hexdigest() == record['uncompressed_sha256']
    log = gzip.decompress((OUT / (name + '-normal-form-tests.log.gz')).read_bytes())
    assert hashlib.sha256(log).hexdigest() == record['raw_log']['sha256']
    assert re.findall(rb'test result: ok\. (\d+) passed; 0 failed; 0 ignored', log) == [b'4']
assert frontiers['baseline']['uncompressed_sha256'] == frontiers['candidate']['uncompressed_sha256']
record = {
    'decision': 'advance production memo-index borrowing to matched resource measurements; no runtime admission',
    'scope': sources['candidate']['scope'],
    'preceding_rejection': 'diagnostics-67.json',
    'source_files_verified_per_variant': len(hashes['baseline']),
    'main_source_files_verified': len(native['source_hashes']),
    'only_runtime_change': changes,
    'original_root_memo_behavior_preserved': True,
    'diagnostic_corpus_unchanged': True,
    'focused_tests_passed_per_variant': 4,
    'strict_serde_release_all_target_clippy_and_workspace_format_passed': True,
    'fresh_production_baseline_frontier_rows': 19140,
    'candidate_frontier_rows_byte_identical': 19140,
    'historical_native_baseline_frontier_rows_byte_identical': 19140,
    'frontier_sha256': frontiers['baseline']['uncompressed_sha256'],
    'ownership_argument': 'Only nonempty forms with no opaque atoms can be borrowed from an existing local memo entry. Entries are appended and addressed by stable indices; references are created only after recursion completes and do not cross later insertion. Consuming returns and all opaque cloning retain their owned path. The original root insertion and all memo keys/order and recognition limits remain present.',
    'main_runtime_unchanged': True,
    'next_step': 'Measure fresh production baseline/candidate identity, timing, allocation and executable-size rows before considering broader suites or complete report replay.',
    'limitations': 'The deterministic frontier corpus and source argument are scoped checks, not a formal proof or exhaustive graph/representation coverage. Full production and native suites, complete report preservation, application performance and the full properties.md objective remain open.',
}
paths = [path for path in sorted(OUT.iterdir()) if path.is_file()]
paths += [EVIDENCE / name for name in ['prepare_laurent_memo_index.py', 'reproduce_laurent_memo_index_validation.py', 'analyze_laurent_memo_index_focused.py']]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-68.json').write_text(json.dumps(record, indent=2) + '\n')
print('Memo-index focused gate: four tests per variant, 19,140 identical receipts, unchanged original root behavior.')
