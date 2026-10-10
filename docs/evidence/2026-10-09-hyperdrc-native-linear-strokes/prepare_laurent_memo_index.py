"""Prepare production baseline/candidate trees for closed local-memo borrowing."""
import difflib
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'laurent-memo-index'
WORK = ROOT / 'target/laurent-memo-index'
PARENT = ROOT / 'target/laurent-root-production'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
parent_record = EVIDENCE / 'laurent-root-production/sources.json'
parent = read(parent_record)
base = read(Path(parent['base_source_record']))
native = read(Path(parent['main_source_record']))
parent_hashes = base['source_hashes'] | parent['changed_source_hashes'] | parent['additional_source_hashes']
assert read(EVIDENCE / 'diagnostics-67.json')['decision'].startswith('not retained:')
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
assert all(digest(PARENT / 'source' / name) == value for name, value in parent_hashes.items())
OUT.mkdir()
WORK.mkdir()
algebra = 'crates/hyperreal/src/computable/node/algebra.rs'
helper = 'crates/hyperreal/src/computable/node/normal_form_memo_frontier_probe.rs'
module = '\n#[cfg(test)]\nmod normal_form_memo_frontier_probe;\n'
original = subprocess.check_output(['git', 'show', 'HEAD:' + algebra], cwd=ROOT).decode()
assert hashlib.sha256(original.encode()).hexdigest() == base['source_hashes'][algebra]
baseline = WORK / 'baseline'
baseline.mkdir()
for folder in ['source', 'target']:
    with (OUT / ('baseline-copy-' + folder + '.log')).open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(PARENT / folder), str(baseline / folder)], stdout=log, stderr=subprocess.STDOUT, check=True)
(baseline / 'source' / algebra).write_text(original + module)
shutil.copyfile(EVIDENCE / 'normal_form_memo_frontier_probe.rs', baseline / 'source' / helper)
candidate = WORK / 'candidate'
candidate.mkdir()
for folder in ['source', 'target']:
    with (OUT / ('candidate-copy-' + folder + '.log')).open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(baseline / folder), str(candidate / folder)], stdout=log, stderr=subprocess.STDOUT, check=True)
path = candidate / 'source' / algebra
text = path.read_text()
marker = '// This normal form is a bounded recognition fast path, not an exactness'
assert text.count(marker) == 1
text = text.replace(marker, '''type PiLaurentMemo = Option<Vec<(usize, Option<PiLaurentPolynomial>)>>;

enum PiLaurentValue {
    Owned(PiLaurentPolynomial),
    Memoized(usize),
}

impl PiLaurentValue {
    fn as_polynomial<'a>(&'a self, memo: &'a PiLaurentMemo) -> &'a PiLaurentPolynomial {
        match self {
            Self::Owned(value) => value,
            // Entries are only appended during a traversal. An index is
            // returned only after inserting a successful closed result.
            Self::Memoized(index) => memo.as_ref()
                .expect("a memoized result has a local memo")[*index].1.as_ref()
                .expect("a memoized result names a successful entry"),
        }
    }

    fn into_owned(self, memo: &PiLaurentMemo) -> PiLaurentPolynomial {
        match self {
            Self::Owned(value) => value,
            value => value.as_polynomial(memo).clone(),
        }
    }
}

''' + marker)
first = text.index('impl PiLaurentPolynomial {')
last = text.index('impl PiAtanTermsForm {', first)
body = text[first:last]
before = '    fn multiply(self, other: Self) -> Option<Self> {'
assert body.count(before) == 1
body = body.replace(before, '    fn multiply(self, other: &Self) -> Option<Self> {')
before = '    const MAX_TERMS: usize = 8;'
body = body.replace(before, before + '''

    fn can_borrow_from_memo(&self) -> bool {
        !self.terms.is_empty()
            && self.terms.iter().all(|(_, coefficient)| coefficient.atoms.is_empty())
    }
''')
text = text[:first] + body + text[last:]
before = '''    fn pi_laurent_polynomial_with_memo(
        &self,
        budget: usize,
        remaining_work: &mut usize,
        memo: &mut Option<Vec<(usize, Option<PiLaurentPolynomial>)>>,
    ) -> Option<PiLaurentPolynomial> {
        let key ='''
after = '''    fn pi_laurent_polynomial_with_memo(
        &self,
        budget: usize,
        remaining_work: &mut usize,
        memo: &mut PiLaurentMemo,
    ) -> Option<PiLaurentPolynomial> {
        self.pi_laurent_polynomial_for_use(budget, remaining_work, memo, true)
            .map(|value| value.into_owned(memo))
    }

    fn pi_laurent_polynomial_for_use(
        &self,
        budget: usize,
        remaining_work: &mut usize,
        memo: &mut PiLaurentMemo,
        needs_owned: bool,
    ) -> Option<PiLaurentValue> {
        let key ='''
assert text.count(before) == 1
text = text.replace(before, after)
first = text.index('    fn pi_laurent_polynomial_for_use(')
last = text.index('    pub(crate) fn bounded_laurent_rational', first)
body = text[first:last]
before = '''            && let Some((_, value)) = memo
                .as_ref()
                .and_then(|memo| memo.iter().find(|(candidate, _)| *candidate == key))
        {
            return value.clone();
        }'''
after = '''            && let Some((index, (_, value))) = memo
                .as_ref()
                .and_then(|memo| memo.iter().enumerate().find(|(_, (candidate, _))| *candidate == key))
        {
            return value.as_ref().map(|value| {
                if !needs_owned && value.can_borrow_from_memo() {
                    PiLaurentValue::Memoized(index)
                } else {
                    PiLaurentValue::Owned(value.clone())
                }
            });
        }'''
assert body.count(before) == 1
body = body.replace(before, after)
first_branch = body.index('                Approximation::Multiply(left, right) =>')
last_branch = body.index('                Approximation::Negate(child) =>', first_branch)
body = body[:first_branch] + '''                Approximation::Multiply(left, right) => {
                    let left = left.pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?;
                    let right = right.pi_laurent_polynomial_for_use(budget - 1, remaining_work, memo, false)?;
                    left.multiply(right.as_polynomial(memo))
                }
''' + body[last_branch:]
before = '''                Approximation::Square(child) => {
                    let child =
                        child.pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?;
                    child.clone().multiply(child)
                }
                Approximation::Sqrt(child) => {
                    let child =
                        child.pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?;'''
after = '''                Approximation::Square(child) => {
                    let value = child.pi_laurent_polynomial_for_use(budget - 1, remaining_work, memo, false)?;
                    let child = value.as_polynomial(memo);
                    child.clone().multiply(child)
                }
                Approximation::Sqrt(child) => {
                    let value = child.pi_laurent_polynomial_for_use(budget - 1, remaining_work, memo, false)?;
                    let child = value.as_polynomial(memo);'''
assert body.count(before) == 1
body = body.replace(before, after)
before = '''        if shared {
            memo.get_or_insert_with(|| Vec::with_capacity(8))
                .push((key, result.clone()));
        }
        result
'''
after = '''        if shared {
            let entries = memo.get_or_insert_with(|| Vec::with_capacity(8));
            if !needs_owned && result.as_ref().is_some_and(PiLaurentPolynomial::can_borrow_from_memo) {
                let index = entries.len();
                entries.push((key, result));
                return Some(PiLaurentValue::Memoized(index));
            }
            entries.push((key, result.clone()));
        }
        result.map(PiLaurentValue::Owned)
'''
assert body.count(before) == 1
text = text[:first] + body.replace(before, after) + text[last:]
path.write_text(text)
records = {}
for name in ['baseline', 'candidate']:
    work = WORK / name
    env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_TARGET_DIR': str(work / 'target'), 'CARGO_NET_OFFLINE': 'true'}
    with (OUT / (name + '-format.log')).open('w') as log:
        subprocess.run(['cargo', 'fmt', '--all'], cwd=work / 'source', env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    hashes = {filename: digest(work / 'source' / filename) for filename in sorted(set(parent_hashes) | {helper})}
    expected = parent_hashes if name == 'baseline' else records['baseline']
    changes = [filename for filename, value in hashes.items() if expected.get(filename) != value]
    assert changes == ([algebra, helper] if name == 'baseline' else [algebra]), changes
    records[name] = hashes
    raw = (json.dumps(hashes, indent=2) + '\n').encode()
    compressed = OUT / (name + '-source-hashes.json.gz')
    compressed.write_bytes(gzip.compress(raw, mtime=0))
    record = {
        'scope': 'Production source with the original root memo behavior and unchanged test-only frontier corpus.' if name == 'baseline' else 'Borrow closed nonempty local-memo results by stable index for multiplication, square and square-root reads. Preserve consuming and opaque ownership, original root insertion, ordered memo entries and all work/recognition limits.',
        'parent_source_record': str(parent_record), 'parent_source_record_sha256': digest(parent_record),
        'main_source_record': parent['main_source_record'],
        'source_hashes_file': str(compressed), 'source_hashes_file_sha256': digest(compressed),
        'source_files': len(hashes), 'changes_from_parent_or_baseline': changes,
        'only_runtime_difference_from_admitted_source': None if name == 'baseline' else algebra,
        'diagnostic_helper_sha256': hashes[helper], 'main_runtime_unchanged': True,
    }
    (OUT / (name + '-sources.json')).write_text(json.dumps(record, indent=2) + '\n')
    with tarfile.open(OUT / (name + '-source-delta.tar.gz'), 'w:gz') as archive:
        for filename in changes:
            archive.add(work / 'source' / filename, arcname=filename)
(OUT / 'candidate.patch').write_text(''.join(difflib.unified_diff(
    (baseline / 'source' / algebra).read_text().splitlines(True), path.read_text().splitlines(True),
    'a/' + algebra, 'b/' + algebra,
)))
shutil.copyfile(__file__, OUT / 'prepare.py')
assert (baseline / 'source' / helper).read_bytes() == (candidate / 'source' / helper).read_bytes()
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
print('Prepared production baseline and memo-index candidate; main runtime unchanged.', flush=True)
