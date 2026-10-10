"""Prepare whole-result sharing while retaining ordinary owned Vec arithmetic."""
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
PREVIOUS = EVIDENCE / 'laurent-memo-sharing'
OUT = EVIDENCE / 'laurent-memo-result-sharing'
BASE = ROOT / 'target/laurent-memo-sharing/baseline'
WORK = ROOT / 'target/laurent-memo-result-sharing/candidate'
SOURCE = WORK / 'source'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
base = read(PREVIOUS / 'baseline-sources.json')
native = read(EVIDENCE / 'native-exactness-gate/sources.json')
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
assert all(digest(BASE / 'source' / name) == value for name, value in base['source_hashes'].items())
OUT.mkdir()
WORK.mkdir(parents=True)
for folder in ['source', 'target']:
    with (OUT / ('copy-' + folder + '.log')).open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(BASE / folder), str(WORK / folder)], stdout=log, stderr=subprocess.STDOUT, check=True)
name = 'crates/hyperreal/src/computable/node/algebra.rs'
path = SOURCE / name
text = path.read_text()
before = '''#[derive(Clone)]
struct PiLaurentPolynomial {
    terms: Vec<(i16, RadicalCoefficient)>,
}'''
after = before + '''

// Graph-free memo values can share their immutable polynomial. Opaque values
// keep ordinary cloning so expression-node reference counts are unchanged.
#[derive(Clone)]
enum MemoizedPiLaurentPolynomial {
    Owned(PiLaurentPolynomial),
    Shared(std::rc::Rc<PiLaurentPolynomial>),
}

impl MemoizedPiLaurentPolynomial {
    fn into_owned(self) -> PiLaurentPolynomial {
        match self {
            Self::Owned(polynomial) => polynomial,
            Self::Shared(polynomial) => std::rc::Rc::unwrap_or_clone(polynomial),
        }
    }
}

impl std::ops::Deref for MemoizedPiLaurentPolynomial {
    type Target = PiLaurentPolynomial;

    fn deref(&self) -> &Self::Target {
        match self {
            Self::Owned(polynomial) => polynomial,
            Self::Shared(polynomial) => polynomial,
        }
    }
}'''
assert text.count(before) == 1
text = text.replace(before, after)
start = text.index('impl PiLaurentPolynomial {')
end = text.index('\nimpl PiAtanTermsForm {', start)
body = text[start:end]
assert body.count('fn multiply(self, other: Self)') == 1
body = body.replace('fn multiply(self, other: Self)', 'fn multiply(self, other: &Self)')
text = text[:start] + body + text[end:]
assert text.count('-> Option<PiLaurentPolynomial>') == 2
text = text.replace('-> Option<PiLaurentPolynomial>', '-> Option<MemoizedPiLaurentPolynomial>')
before = 'memo: &mut Option<Vec<(usize, Option<PiLaurentPolynomial>)>>,'
assert text.count(before) == 1
text = text.replace(before, before.replace('Option<PiLaurentPolynomial>', 'Option<MemoizedPiLaurentPolynomial>'))
start = text.index('    fn pi_laurent_polynomial_with_memo(')
end = text.index('    pub(crate) fn bounded_laurent_rational', start)
body = text[start:end]
first = body.index('                Approximation::Add(left, right) =>')
last = body.index('                Approximation::Sqrt(child) =>', first)
body = body[:first] + '''                Approximation::Add(left, right) => left
                    .pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?
                    .into_owned()
                    .add(right.pi_laurent_polynomial_with_memo(
                        budget - 1,
                        remaining_work,
                        memo,
                    )?.into_owned()),
                Approximation::Multiply(left, right) => {
                    let left = left.pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?;
                    let right = right.pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?;
                    left.into_owned().multiply(&right)
                }
                Approximation::Negate(child) => Some(
                    child
                        .pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?
                        .into_owned()
                        .scaled(Rational::new(-1)),
                ),
                Approximation::Offset(child, shift) => Some(
                    child
                        .pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?
                        .into_owned()
                        .scaled(Self::power_of_two_rational(*shift)),
                ),
                Approximation::Inverse(child) => child
                    .pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?
                    .into_owned()
                    .inverse_monomial(),
                Approximation::Square(child) => {
                    let child = child.pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?;
                    child.clone().into_owned().multiply(&child)
                }
''' + body[last:]
before = '''                                .pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?
                                .scaled(value.clone()),'''
assert body.count(before) == 1
body = body.replace(before, before.replace('.scaled(value.clone())', '.into_owned().scaled(value.clone())'))
before = '''        if shared {
            memo.get_or_insert_with(|| Vec::with_capacity(8))'''
after = '''        let result = result.map(|polynomial| {
            if shared
                && !polynomial.terms.is_empty()
                && polynomial.terms.iter().all(|(_, coefficient)| coefficient.atoms.is_empty())
            {
                MemoizedPiLaurentPolynomial::Shared(std::rc::Rc::new(polynomial))
            } else {
                MemoizedPiLaurentPolynomial::Owned(polynomial)
            }
        });
        if shared {
            memo.get_or_insert_with(|| Vec::with_capacity(8))'''
assert body.count(before) == 1
body = body.replace(before, after)
text = text[:start] + body + text[end:]
before = '        for (power, coefficient) in polynomial.terms {'
assert text.count(before) == 1
text = text.replace(before, before.replace('polynomial.terms', 'polynomial.into_owned().terms'))
path.write_text(text)
environment = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': str(WORK / 'target')}
with (OUT / 'prepare-format.log').open('w') as log:
    subprocess.run(['cargo', 'fmt', '--all'], cwd=SOURCE, env=environment, stdout=log, stderr=subprocess.STDOUT, check=True)
initial = {name: digest(SOURCE / name) for name in base['source_hashes']}
changes = [name for name, value in initial.items() if base['source_hashes'][name] != value]
assert changes == [name], changes
record = {'scope': 'Isolated whole-result sharing in the existing per-call memo. Only nonempty closed polynomials share storage; arithmetic restores owned inputs only where consumed. Opaque cloning, Node layout, proof limits and main production source remain unchanged. The baseline frontier probe is reused verbatim.', 'base_source_record': str(PREVIOUS / 'baseline-sources.json'), 'native_source_record': str(EVIDENCE / 'native-exactness-gate/sources.json'), 'source_hashes': initial, 'only_modified_sources': changes, 'main_production_unchanged': True}
(OUT / 'candidate-sources.json').write_text(json.dumps(record, indent=2) + '\n')
with tarfile.open(OUT / 'candidate-source-delta.tar.gz', 'w:gz') as archive:
    archive.add(path, arcname=name)
(OUT / 'candidate.patch').write_text(''.join(difflib.unified_diff((BASE / 'source' / name).read_text().splitlines(True), path.read_text().splitlines(True), 'a/' + name, 'b/' + name)))
shutil.copyfile(__file__, OUT / 'prepare.py')
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
print('Prepared whole-result sharing candidate; main runtime unchanged.')
