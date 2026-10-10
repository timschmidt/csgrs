"""Freeze the isolated checked formatter before running any validation."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'checked-scientific'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
assert not (OUT / 'status.json').exists()
parent = read(OUT / 'parent.json')
source = Path(parent['source_directory'])
record_path = Path(parent['parent_source_record'])
assert digest(record_path) == parent['parent_source_record_sha256']
record = read(record_path)
hash_file = Path(record['source_hashes_file'])
assert digest(hash_file) == record['source_hashes_file_sha256']
original = json.loads(gzip.decompress(hash_file.read_bytes()))
base = Path(record['source_directory'])
assert all(digest(base / name) == value for name, value in original.items())
native_path = EVIDENCE.parent / '2026-10-09-hyperdrc-native-linear-strokes/native-exactness-gate/sources.json'
native = read(native_path)['source_hashes']
assert all(digest(ROOT / name) == value for name, value in native.items())
new = ['crates/hyperreal/src/real/arithmetic/scientific.rs', 'crates/hyperreal/tests/checked_scientific.rs']
hashes = {name: digest(source / name) for name in sorted(set(original) | set(new))}
changed = [name for name in hashes if original.get(name) != hashes[name]]
expected = {'crates/hyperreal/README.md', 'crates/hyperreal/src/lib.rs', 'crates/hyperreal/src/real/arithmetic.rs', 'crates/hyperreal/src/problem.rs', 'crates/hyperreal/tests/gmp_api_coverage.rs', *new}
assert set(changed) == expected
compressed = OUT / 'candidate-source-hashes.json.gz'
compressed.write_bytes(gzip.compress((json.dumps(hashes, indent=2) + '\n').encode(), mtime=0))
with tarfile.open(OUT / 'candidate-source-delta.tar.gz', 'w:gz') as archive:
    for name in changed:
        archive.add(source / name, arcname=name)
patch = ''
for name in changed:
    before = (base / name).read_text() if name in original else ''
    patch += ''.join(difflib.unified_diff(before.splitlines(True), (source / name).read_text().splitlines(True), 'a/' + name if name in original else '/dev/null', 'b/' + name))
(OUT / 'candidate.patch').write_text(patch)
record = parent | {
    'source_hashes_file': str(compressed), 'source_hashes_file_sha256': digest(compressed),
    'source_files': len(hashes), 'changed_files': changed,
    'source_delta_sha256': digest(OUT / 'candidate-source-delta.tar.gz'),
    'main_source_record': str(native_path), 'main_source_record_sha256': digest(native_path),
    'scope': 'Isolated checked scientific-formatting API with certified nearest-even enclosure rounding, exact rational ties, caller refinement floor, finite formatter guards and explicit typed failure. Existing formatting traits and main runtime unchanged.',
    'public_api': ['Real::try_scientific', 'ScientificCase::{Lower,Upper}'],
    'focused_tests': 'Twelve integration tests, including 4608 exact-rational outputs checked against directed MPFR bounds, pi/square-root brackets, exact ties/carries, original unresolved-hypot inputs, final-value aborts and excessive-request guards.',
    'limitations': 'Own precision requests and formatter arithmetic are bounded; underlying expression evaluation is not given a total-work or wall-time bound. This checked path does not complete general infallible formatting or the property plan.',
}
(OUT / 'sources.json').write_text(json.dumps(record, indent=2) + '\n')
shutil.copyfile(__file__, OUT / 'freeze.py')
print('Frozen checked scientific candidate:', len(hashes), 'inputs,', len(changed), 'changed files.', flush=True)
