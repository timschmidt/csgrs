"""Freeze a separate exact-class precheck candidate; preserve the first candidate."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'class-precheck'
WORK = ROOT / 'target/hyperreal-scientific-class-precheck'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
parent_path = EVIDENCE / 'checked-scientific/sources.json'
parent = read(parent_path)
hash_file = Path(parent['source_hashes_file'])
assert digest(hash_file) == parent['source_hashes_file_sha256']
original = json.loads(gzip.decompress(hash_file.read_bytes()))
native = read(Path(parent['main_source_record']))['source_hashes']
base = Path(parent['source_directory'])
assert all(digest(base / name) == value for name, value in original.items())
assert all(digest(ROOT / name) == value for name, value in native.items())
OUT.mkdir()
WORK.mkdir()
source, target = WORK / 'source', WORK / 'target'
for before, after in [(base, source), (Path(parent['target_directory']), target)]:
    print('copy', before, 'to', after, flush=True)
    subprocess.run(['cp', '-a', '--reflink=auto', str(before), str(after)], check=True)
assert all(digest(source / name) == value for name, value in original.items())
name = 'crates/hyperreal/src/real/arithmetic/scientific.rs'
old = (source / name).read_text()
before = '''        let zero = self.structural_facts().zero;
        if zero == ZeroKnowledge::Zero {
'''
after = '''        // Every classified inner value is certified nonzero; its rational
        // scale carries zero and sign. Only an unclassified expression needs
        // a structural zero proof before numeric refinement.
        if self.definitely_zero()
            || (matches!(&self.class, super::Class::Irrational)
                && self.structural_facts().zero == ZeroKnowledge::Zero)
        {
'''
assert old.count(before) == 1
(source / name).write_text(old.replace(before, after))
hashes = {item: digest(source / item) for item in original}
assert [item for item in hashes if hashes[item] != original[item]] == [name]
compressed = OUT / 'candidate-source-hashes.json.gz'
compressed.write_bytes(gzip.compress((json.dumps(hashes, indent=2) + '\n').encode(), mtime=0))
with tarfile.open(OUT / 'candidate-source-delta.tar.gz', 'w:gz') as archive:
    archive.add(source / name, arcname=name)
(OUT / 'candidate.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True), (source / name).read_text().splitlines(True), 'a/' + name, 'b/' + name)))
record = {
    'parent_source_record': str(parent_path),
    'parent_source_record_sha256': digest(parent_path),
    'source_directory': str(source), 'target_directory': str(target),
    'source_hashes_file': str(compressed), 'source_hashes_file_sha256': digest(compressed),
    'source_files': len(hashes), 'changed_files': [name],
    'source_delta_sha256': digest(OUT / 'candidate-source-delta.tar.gz'),
    'main_source_record': parent['main_source_record'],
    'main_source_record_sha256': parent['main_source_record_sha256'],
    'scope': 'Only the zero precheck changes: exact zero scale remains sufficient, and only Class::Irrational requires structural zero analysis. All numeric refinement, rounding, typed failures, limits and abort handling remain unchanged.',
    'proof_basis': 'Class invariants in real/arithmetic/classification.rs certify every non-Irrational inner value as positive and nonzero. Real.rational scales it and carries its sign. definitely_zero checks that exact scale. A false definitely_zero result alone is never used as a nonzero proof.',
    'hypothesis': 'Avoid unnecessary structural facts and retained rational metadata for already-classified nonzero values. The preceding measurements do not establish causality.',
    'gates': ['complete scalar tests, strict Clippy/docs/formatting', 'same 3816 checked receipts in both process orders', 'five fresh alternating samples of all 28 timing/allocation rows against first candidate', 'downstream compatibility before runtime admission'],
    'scope_unchanged': 'Existing formatting traits, native source work and the full property/target plan remain open.',
}
(OUT / 'sources.json').write_text(json.dumps(record, indent=2) + '\n')
shutil.copyfile(__file__, OUT / 'prepare.py')
assert all(digest(base / item) == value for item, value in original.items())
assert all(digest(ROOT / item) == value for item, value in native.items())
print('Frozen exact-class candidate:', len(hashes), 'source inputs; one runtime file differs.', flush=True)
