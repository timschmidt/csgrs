"""Audit shared-expansion frontiers before reviewing resource measurements."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'laurent-shared-expansion'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
records = {name: read(OUT / (name + '-sources.json')) for name in ['baseline', 'candidate']}
directories = {'baseline': ROOT / 'target/laurent-memo-index/baseline/source', 'candidate': ROOT / 'target/laurent-shared-expansion/candidate/source'}
hashes = {}
for name, record in records.items():
    assert digest(Path(record['source_hashes_file'])) == record['source_hashes_file_sha256']
    hashes[name] = json.loads(gzip.decompress(Path(record['source_hashes_file']).read_bytes()))
    assert len(hashes[name]) == record['source_files']
    assert all(digest(directories[name] / path) == value for path, value in hashes[name].items())
native = read(Path(records['baseline']['main_source_record']))
assert all(digest(ROOT / path) == value for path, value in native['source_hashes'].items())
changes = [name for name, value in hashes['candidate'].items() if hashes['baseline'].get(name) != value]
assert changes == ['crates/hyperreal/src/computable/node/algebra.rs']
with tarfile.open(OUT / 'candidate-source-delta.tar.gz') as archive:
    assert archive.getnames() == changes
    for path in changes:
        assert hashlib.sha256(archive.extractfile(path).read()).hexdigest() == hashes['candidate'][path]
helper = 'crates/hyperreal/src/computable/node/normal_form_memo_frontier_probe.rs'
assert hashes['baseline'][helper] == hashes['candidate'][helper] == digest(EVIDENCE / 'normal_form_memo_frontier_probe.rs')
status = read(OUT / 'status.json')
assert len(status) == 4 and all(row['exit_code'] == 0 for row in status)
baseline_status = read(OUT / 'baseline-status.json')
assert len(baseline_status) == 4 and all(row['exit_code'] == 0 for row in baseline_status)
assert records['baseline'] == read(EVIDENCE / 'laurent-memo-index/baseline-sources.json')
assert digest(Path(records['candidate']['base_source_record'])) == records['candidate']['base_source_record_sha256']
assert read(OUT / 'validation-complete.json')['frontier_rows_identical'] == 19140
assert read(OUT / 'frontier-comparison.json')['different_rows'] == 0
frontiers = read(OUT / 'frontiers.json')
for name, record in frontiers.items():
    path = OUT / (name + '-frontiers.jsonl.gz')
    assert digest(path) == record['compressed_sha256']
    raw = gzip.decompress(path.read_bytes())
    assert len(raw) == record['uncompressed_bytes'] and hashlib.sha256(raw).hexdigest() == record['uncompressed_sha256']
    log = gzip.decompress((OUT / (name + '-normal-form-tests.log.gz')).read_bytes())
    assert hashlib.sha256(log).hexdigest() == record['raw_log']['sha256']
    assert re.findall(rb'test result: ok\. (\d+) passed; 0 failed; 0 ignored', log) == [b'4']
assert frontiers['baseline']['uncompressed_sha256'] == frontiers['candidate']['uncompressed_sha256']
record = {
    'decision': 'advance shared compound expansion to early production-source resource measurements; no runtime admission',
    'scope': records['candidate']['scope'],
    'preceding_rejection': 'diagnostics-73.json',
    'source_files_verified_per_variant': len(hashes['baseline']),
    'main_source_files_verified': len(native['source_hashes']),
    'only_runtime_change': changes,
    'candidate_focused_tests_passed': 4,
    'baseline_validation_reused': 'diagnostics-68.json; four focused tests and strict checks on unchanged baseline source',
    'candidate_strict_serde_release_all_target_clippy_and_format_passed': True,
    'diagnostic_corpus_unchanged': True,
    'original_root_memo_behavior_preserved': True,
    'frontier_receipts_byte_identical': 19140,
    'frontier_sha256': frontiers['baseline']['uncompressed_sha256'],
    'ownership_argument': 'Owned recursive consumers keep the ordinary polynomial return type. Closed nonempty results can be consumed through a reference to the existing memo. Opaque results retain owned cloning. A child entry is completed before the parent consumer can fail, keeping normalization fallback and memo contents separate from parent arithmetic failure. No reference crosses insertion or recursive traversal. The nongeneric helper contains exactly the preceding compound match and opaque fallback; cheap recognition, memo completion and budget charging remain in the generic caller.',
    'main_runtime_unchanged': True,
    'next_step': 'Prepare a separate production source tree by removing only the unchanged diagnostic module, then run the same independent identity and matched time/allocation/size probe on that production library before broader validation or report replay.',
    'limitations': 'This is scoped differential evidence and a source argument, not a formal proof or complete representation/concurrency coverage. No allocation saving, timing gain, full-suite pass, native gate pass or report preservation is claimed here. All properties/targets remain open.',
}
paths = [path for path in sorted(OUT.iterdir()) if path.is_file()]
paths += [EVIDENCE / name for name in ['prepare_laurent_shared_expansion.py', 'reproduce_laurent_shared_expansion_validation.py', 'analyze_laurent_shared_expansion_focused.py']]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-74.json').write_text(json.dumps(record, indent=2) + '\n')
print('Shared-expansion focused gate: four fresh tests, strict checks, 19,140 identical receipts; baseline validation reused with verified hashes.')
