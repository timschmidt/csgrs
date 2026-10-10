"""Prepare static consumers without the rejected indexed return carrier."""
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
OUT = EVIDENCE / 'laurent-memo-consumer'
WORK = ROOT / 'target/laurent-memo-consumer/candidate'
BASE = ROOT / 'target/laurent-memo-index/baseline'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
base_record = EVIDENCE / 'laurent-memo-index/baseline-sources.json'
base = read(base_record)
assert digest(Path(base['source_hashes_file'])) == base['source_hashes_file_sha256']
hashes = json.loads(gzip.decompress(Path(base['source_hashes_file']).read_bytes()))
native = read(Path(base['main_source_record']))
assert read(EVIDENCE / 'diagnostics-69.json')['decision'].startswith('not retained:')
assert all(digest(BASE / 'source' / name) == value for name, value in hashes.items())
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
OUT.mkdir()
WORK.mkdir(parents=True)
for folder in ['source', 'target']:
    with (OUT / ('copy-' + folder + '.log')).open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(BASE / folder), str(WORK / folder)], stdout=log, stderr=subprocess.STDOUT, check=True)
name = 'crates/hyperreal/src/computable/node/algebra.rs'
path = WORK / 'source' / name
original = path.read_text()
text = original
marker = '// This normal form is a bounded recognition fast path, not an exactness'
assert text.count(marker) == 1
text = text.replace(marker, '''trait PiLaurentConsumer: Sized {
    const NEEDS_OWNED: bool = false;

    fn owned(self, value: PiLaurentPolynomial) -> Option<PiLaurentPolynomial> {
        self.borrowed(&value)
    }

    fn borrowed(self, value: &PiLaurentPolynomial) -> Option<PiLaurentPolynomial>;
}

struct OwnPiLaurent;

impl PiLaurentConsumer for OwnPiLaurent {
    const NEEDS_OWNED: bool = true;

    fn owned(self, value: PiLaurentPolynomial) -> Option<PiLaurentPolynomial> {
        Some(value)
    }

    fn borrowed(self, value: &PiLaurentPolynomial) -> Option<PiLaurentPolynomial> {
        Some(value.clone())
    }
}

struct MultiplyPiLaurent(PiLaurentPolynomial);

impl PiLaurentConsumer for MultiplyPiLaurent {
    fn borrowed(self, right: &PiLaurentPolynomial) -> Option<PiLaurentPolynomial> {
        self.0.multiply(right)
    }
}

struct SquarePiLaurent;

impl PiLaurentConsumer for SquarePiLaurent {
    fn borrowed(self, value: &PiLaurentPolynomial) -> Option<PiLaurentPolynomial> {
        value.clone().multiply(value)
    }
}

struct SqrtPiLaurent;

impl PiLaurentConsumer for SqrtPiLaurent {
    fn borrowed(self, value: &PiLaurentPolynomial) -> Option<PiLaurentPolynomial> {
        let radicand = match value.terms.as_slice() {
            [] => Rational::zero(),
            [(0, coefficient)]
                if coefficient.radicals.is_empty() && coefficient.atoms.is_empty() =>
            {
                coefficient.rational.clone()
            }
            _ => return None,
        };
        if radicand.sign() == Sign::Minus {
            return None;
        }
        Some(PiLaurentPolynomial::monomial(
            0,
            RadicalCoefficient::radical(Rational::one(), radicand),
        ))
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
        memo: &mut Option<Vec<(usize, Option<PiLaurentPolynomial>)>>,
    ) -> Option<PiLaurentPolynomial> {
        self.pi_laurent_polynomial_with_consumer(budget, remaining_work, memo, OwnPiLaurent)
    }

    fn pi_laurent_polynomial_with_consumer<C: PiLaurentConsumer>(
        &self,
        budget: usize,
        remaining_work: &mut usize,
        memo: &mut Option<Vec<(usize, Option<PiLaurentPolynomial>)>>,
        consumer: C,
    ) -> Option<PiLaurentPolynomial> {
        let key ='''
assert text.count(before) == 1
text = text.replace(before, after)
first = text.index('    fn pi_laurent_polynomial_with_consumer<')
last = text.index('    pub(crate) fn bounded_laurent_rational', first)
body = text[first:last]
before = '            return value.clone();'
assert body.count(before) == 1
body = body.replace(before, '''            return value.as_ref().and_then(|value| {
                if !C::NEEDS_OWNED && value.can_borrow_from_memo() {
                    consumer.borrowed(value)
                } else {
                    consumer.owned(value.clone())
                }
            });''')
start = body.index('                Approximation::Multiply(left, right) =>')
end = body.index('                Approximation::Negate(child) =>', start)
body = body[:start] + '''                Approximation::Multiply(left, right) => {
                    let left = left.pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?;
                    right.pi_laurent_polynomial_with_consumer(
                        budget - 1, remaining_work, memo, MultiplyPiLaurent(left),
                    )
                }
''' + body[end:]
start = body.index('                Approximation::Square(child) =>')
end = body.index('                Approximation::LinearCombination3(combination) =>', start)
body = body[:start] + '''                Approximation::Square(child) => child.pi_laurent_polynomial_with_consumer(
                    budget - 1, remaining_work, memo, SquarePiLaurent,
                ),
                Approximation::Sqrt(child) => child.pi_laurent_polynomial_with_consumer(
                    budget - 1, remaining_work, memo, SqrtPiLaurent,
                ),
''' + body[end:]
before = '''        if shared {
            memo.get_or_insert_with(|| Vec::with_capacity(8))
                .push((key, result.clone()));
        }
        result
'''
after = '''        if shared {
            let entries = memo.get_or_insert_with(|| Vec::with_capacity(8));
            if !C::NEEDS_OWNED && result.as_ref().is_some_and(PiLaurentPolynomial::can_borrow_from_memo) {
                entries.push((key, result));
                // The child's memo entry is complete before its parent
                // arithmetic can fail. No reference crosses an insertion.
                let value = entries.last().expect("just inserted a local memo entry")
                    .1.as_ref().expect("inserted a successful closed result");
                return consumer.borrowed(value);
            }
            entries.push((key, result.clone()));
        }
        result.and_then(|value| consumer.owned(value))
'''
assert body.count(before) == 1
text = text[:first] + body.replace(before, after) + text[last:]
path.write_text(text)
env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_TARGET_DIR': str(WORK / 'target'), 'CARGO_NET_OFFLINE': 'true'}
with (OUT / 'prepare-format.log').open('w') as log:
    subprocess.run(['cargo', 'fmt', '--all'], cwd=WORK / 'source', env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
current = {filename: digest(WORK / 'source' / filename) for filename in hashes}
changes = [filename for filename, value in current.items() if hashes[filename] != value]
assert changes == [name], changes
compressed = OUT / 'candidate-source-hashes.json.gz'
compressed.write_bytes(gzip.compress((json.dumps(current, indent=2) + '\n').encode(), mtime=0))
record = {
    'scope': 'Static owned/borrowed consumers for the existing local normal-form memo. Only nonempty closed results may be borrowed; the original root behavior, opaque ownership, recognition limits and ordered memo remain unchanged. No indexed return carrier or persistent cache.',
    'base_source_record': str(base_record), 'base_source_record_sha256': digest(base_record),
    'main_source_record': base['main_source_record'],
    'source_hashes_file': str(compressed), 'source_hashes_file_sha256': digest(compressed),
    'source_files': len(current), 'only_modified_sources': changes,
    'baseline_source_directory': str(BASE / 'source'),
    'candidate_source_directory': str(WORK / 'source'),
    'main_runtime_unchanged': True,
}
(OUT / 'candidate-sources.json').write_text(json.dumps(record, indent=2) + '\n')
with tarfile.open(OUT / 'candidate-source-delta.tar.gz', 'w:gz') as archive:
    archive.add(path, arcname=name)
(OUT / 'candidate.patch').write_text(''.join(difflib.unified_diff(original.splitlines(True), path.read_text().splitlines(True), 'a/' + name, 'b/' + name)))
shutil.copyfile(base_record, OUT / 'baseline-sources.json')
shutil.copyfile(__file__, OUT / 'prepare.py')
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
print('Prepared static-consumer candidate; baseline and main runtime unchanged.', flush=True)
