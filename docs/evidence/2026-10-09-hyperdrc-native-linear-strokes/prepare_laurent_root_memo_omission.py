"""Omit the root's unused memo copy while retaining the original owned form."""
import difflib
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'laurent-root-memo-omission'
WORK = ROOT / 'target/laurent-root-memo-omission/candidate'
SOURCE = WORK / 'source'
BASE = ROOT / 'target/laurent-memo-sharing/baseline'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
base_record = EVIDENCE / 'laurent-memo-sharing/baseline-sources.json'
base = read(base_record)
native = read(EVIDENCE / 'native-exactness-gate/sources.json')
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
assert all(digest(BASE / 'source' / name) == value for name, value in base['source_hashes'].items())
OUT.mkdir()
WORK.mkdir(parents=True)
for folder in ['source', 'target']:
    with (OUT / ('copy-' + folder + '.log')).open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(BASE / folder), str(WORK / folder)], stdout=log, stderr=subprocess.STDOUT, check=True)
algebra = 'crates/hyperreal/src/computable/node/algebra.rs'
helper = 'crates/hyperreal/src/computable/node/normal_form_memo_frontier_probe.rs'
path = SOURCE / algebra
text = path.read_text()
before = '        self.pi_laurent_polynomial_with_memo(budget, &mut remaining_work, &mut memo)'
after = '''        // This fresh memo is dropped as soon as the root result is returned.
        // Only recursive results can be reused during the traversal.
        self.pi_laurent_polynomial_with_memo_impl::<false>(budget, &mut remaining_work, &mut memo)'''
assert text.count(before) == 1
text = text.replace(before, after)
before = '''    fn pi_laurent_polynomial_with_memo(
        &self,
        budget: usize,
        remaining_work: &mut usize,
        memo: &mut Option<Vec<(usize, Option<PiLaurentPolynomial>)>>,
    ) -> Option<PiLaurentPolynomial> {
        let key = Arc::as_ptr(&self.internal) as usize;
        let shared = Arc::strong_count(&self.internal) > 1;'''
after = '''    fn pi_laurent_polynomial_with_memo(
        &self,
        budget: usize,
        remaining_work: &mut usize,
        memo: &mut Option<Vec<(usize, Option<PiLaurentPolynomial>)>>,
    ) -> Option<PiLaurentPolynomial> {
        self.pi_laurent_polynomial_with_memo_impl::<true>(budget, remaining_work, memo)
    }

    fn pi_laurent_polynomial_with_memo_impl<const MEMOIZE_CURRENT: bool>(
        &self,
        budget: usize,
        remaining_work: &mut usize,
        memo: &mut Option<Vec<(usize, Option<PiLaurentPolynomial>)>>,
    ) -> Option<PiLaurentPolynomial> {
        let key = Arc::as_ptr(&self.internal) as usize;
        let shared = MEMOIZE_CURRENT && Arc::strong_count(&self.internal) > 1;'''
assert text.count(before) == 1
path.write_text(text.replace(before, after))
probe = SOURCE / helper
text = probe.read_text()
before = 'query.pi_laurent_polynomial_with_memo(depth, &mut remaining, &mut memo)'
after = 'query.pi_laurent_polynomial_with_memo_impl::<false>(depth, &mut remaining, &mut memo)'
assert text.count(before) == 1
probe.write_text(text.replace(before, after))
environment = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': str(WORK / 'target')}
with (OUT / 'prepare-format.log').open('w') as log:
    subprocess.run(['cargo', 'fmt', '--all'], cwd=SOURCE, env=environment, stdout=log, stderr=subprocess.STDOUT, check=True)
assert probe.read_text() == (BASE / 'source' / helper).read_text().replace(before, after)
initial = {name: digest(SOURCE / name) for name in base['source_hashes']}
changes = [name for name, value in initial.items() if base['source_hashes'][name] != value]
assert set(changes) == {algebra, helper}, changes
record = {
    'scope': 'Isolated omission of the unused final root memo entry. The original owned polynomial and recursive memo behavior are retained. The diagnostic corpus changes only its call target so it exercises the same root specialization as production.',
    'base_source_record': str(base_record),
    'native_source_record': str(EVIDENCE / 'native-exactness-gate/sources.json'),
    'source_hashes': initial,
    'only_modified_sources': changes,
    'only_modified_runtime_source': algebra,
    'probe_only_change': {'before': before, 'after': after},
    'main_production_unchanged': True,
    'argument': 'The entry point creates an empty local memo. The expression is an acyclic DAG, so a descendant cannot revisit the root. Root memo insertion occurs after all recursive calls and arithmetic; the memo is immediately dropped on return. Omitting that insertion cannot change any later lookup in this traversal. Recursive ownership, limits, insertion order and result calculations are unchanged.',
}
(OUT / 'candidate-sources.json').write_text(json.dumps(record, indent=2) + '\n')
with tarfile.open(OUT / 'candidate-source-delta.tar.gz', 'w:gz') as archive:
    for name in changes:
        archive.add(SOURCE / name, arcname=name)
patch = []
for name in changes:
    patch.extend(difflib.unified_diff((BASE / 'source' / name).read_text().splitlines(True), (SOURCE / name).read_text().splitlines(True), 'a/' + name, 'b/' + name))
(OUT / 'candidate.patch').write_text(''.join(patch))
shutil.copyfile(__file__, OUT / 'prepare.py')
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
print('Prepared unused root memo omission; main runtime unchanged.')
