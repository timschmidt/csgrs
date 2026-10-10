"""Check the isolated sharing experiment's focused semantic evidence."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'laurent-memo-sharing'
WORK = ROOT / 'target/laurent-memo-sharing'
read = lambda path: json.loads(path.read_text())
sha = lambda data: hashlib.sha256(data).hexdigest()
digest = lambda path: sha(path.read_bytes())
native = read(EVIDENCE / 'native-exactness-gate/sources.json')
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
sources = {variant: read(OUT / (variant + '-sources.json')) for variant in ['baseline', 'candidate']}
for variant, source in sources.items():
    assert all(digest(WORK / variant / 'source' / name) == value for name, value in source['source_hashes'].items())
    with tarfile.open(OUT / (variant + '-source-delta.tar.gz')) as archive:
        assert set(archive.getnames()) == set(source['only_modified_sources'])
        for name in archive.getnames():
            assert sha(archive.extractfile(name).read()) == source['source_hashes'][name]
changes = [name for name, value in sources['candidate']['source_hashes'].items() if sources['baseline']['source_hashes'].get(name) != value]
assert changes == ['crates/hyperreal/src/computable/node/algebra.rs']
status = read(OUT / 'status.json')
assert len(status) == 8 and all(row['exit_code'] == 0 for row in status)
frontiers = read(OUT / 'frontiers.json')
for variant, record in frontiers.items():
    path = OUT / (variant + '-frontiers.jsonl.gz')
    assert digest(path) == record['compressed_sha256']
    data = gzip.decompress(path.read_bytes())
    assert len(data) == record['uncompressed_bytes'] and sha(data) == record['uncompressed_sha256']
    assert len(data.splitlines()) == 19140
    log_path = OUT / (variant + '-normal-form-tests.log.gz')
    assert digest(log_path) == record['raw_log']['gzip_sha256']
    log = gzip.decompress(log_path.read_bytes())
    assert len(log) == record['raw_log']['bytes'] and sha(log) == record['raw_log']['sha256']
    assert re.findall(rb'test result: ok\. (\d+) passed; 0 failed; 0 ignored', log) == [b'4']
assert frontiers['baseline']['uncompressed_sha256'] == frontiers['candidate']['uncompressed_sha256']
comparison = read(OUT / 'frontier-comparison.json')
assert comparison['rows'] == 19140 and comparison['different_rows'] == 0
libraries = read(OUT / 'libraries.json')
assert all(digest(Path(record['path'])) == record['sha256'] for record in libraries.values())
record = {
    'decision': 'advance isolated closed-result sharing to matched resource measurements; no runtime change admitted',
    'native_baseline_gate': 'diagnostics-59.json',
    'scope': 'A per-call storage experiment on the complete native prototype. The baseline and candidate use the identical test-only diagnostic module; only algebra.rs differs between them.',
    'changed_runtime_source': changes,
    'ownership_constraint': 'Only nonempty polynomial term vectors whose coefficients contain no Computable atoms are shared. Every mutation first restores owned storage. Opaque terms retain ordinary deep cloning, so their expression-node ownership and the existing Arc strong-count memo eligibility remain unchanged.',
    'unchanged_contracts': ['node layout', 'public API', 'depth/work/term recognition limits', 'arithmetic evaluation and coefficient insertion order', 'persistent caches and serialization'],
    'focused_tests_per_variant': 4,
    'format_and_strict_all_target_scalar_clippy': True,
    'frontier_corpus': {
        'fixtures': 58, 'stages': ['fresh', 'warm', 'cloned'],
        'depths': [0, 1, 2, 7, 8, 47, 48, 63, 64, 65],
        'work_budgets': [0, 1, 2, 7, 8, 31, 127, 255, 511, 512, 513],
        'rows': 19140, 'different_rows': 0,
        'bytes_per_variant': frontiers['baseline']['uncompressed_bytes'],
        'sha256': frontiers['baseline']['uncompressed_sha256'],
        'compared': comparison['compared'],
    },
    'main_source_hashes_unchanged': len(native['source_hashes']),
    'frozen_files_per_variant': len(sources['baseline']['source_hashes']),
    'limitations': 'The receipt comparison preserves behavior for this deterministic corpus; it is not an independent mathematical proof, exhaustive DAG enumeration or full scalar/curve/DRC validation. Public algebraic identities, matched timing/allocation/size evidence and the original board fixture are subsequent gates. The native runtime limit and all broader properties/targets remain open.',
}
paths = [path for path in sorted(OUT.iterdir()) if path.is_file()]
paths += [EVIDENCE / name for name in ['prepare_laurent_memo_sharing.py', 'normal_form_memo_frontier_probe.rs', 'reproduce_laurent_memo_sharing_validation.py', 'analyze_laurent_memo_sharing_focused.py']]
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in paths}
(EVIDENCE / 'diagnostics-60.json').write_text(json.dumps(record, indent=2) + '\n')
print('Focused sharing evidence verified: 4 tests per variant, strict checks, 19,140 identical frontier receipts; main source unchanged.')
