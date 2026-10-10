"""Keep computed consuming operands owned; share closed results only for reads."""
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
OUT = EVIDENCE / 'laurent-memo-use-sharing'
WORK = ROOT / 'target/laurent-memo-use-sharing/candidate'
SOURCE = WORK / 'source'
PREVIOUS = ROOT / 'target/laurent-memo-result-sharing/candidate'
BASE = ROOT / 'target/laurent-memo-sharing/baseline/source'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
base_record = EVIDENCE / 'laurent-memo-sharing/baseline-sources.json'
previous_record = EVIDENCE / 'laurent-memo-result-sharing/candidate-sources.json'
base = read(base_record)
previous = read(previous_record)
native = read(EVIDENCE / 'native-exactness-gate/sources.json')
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
assert all(digest(PREVIOUS / 'source' / name) == value for name, value in previous['source_hashes'].items())
OUT.mkdir()
WORK.mkdir(parents=True)
for folder in ['source', 'target']:
    with (OUT / ('copy-' + folder + '.log')).open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(PREVIOUS / folder), str(WORK / folder)], stdout=log, stderr=subprocess.STDOUT, check=True)
name = 'crates/hyperreal/src/computable/node/algebra.rs'
path = SOURCE / name
text = path.read_text()
before = '''impl MemoizedPiLaurentPolynomial {
    fn into_owned(self) -> PiLaurentPolynomial {'''
after = '''impl MemoizedPiLaurentPolynomial {
    fn share_closed(self) -> Self {
        match self {
            Self::Owned(polynomial)
                if !polynomial.terms.is_empty()
                    && polynomial.terms.iter().all(|(_, coefficient)| coefficient.atoms.is_empty()) =>
            {
                Self::Shared(std::rc::Rc::new(polynomial))
            }
            value => value,
        }
    }

    fn into_owned(self) -> PiLaurentPolynomial {'''
assert text.count(before) == 1
text = text.replace(before, after)
before = '''    fn pi_laurent_polynomial_with_memo(
        &self,
        budget: usize,
        remaining_work: &mut usize,
        memo: &mut Option<Vec<(usize, Option<MemoizedPiLaurentPolynomial>)>>,
    ) -> Option<MemoizedPiLaurentPolynomial> {
        let key ='''
after = '''    fn pi_laurent_polynomial_with_memo(
        &self,
        budget: usize,
        remaining_work: &mut usize,
        memo: &mut Option<Vec<(usize, Option<MemoizedPiLaurentPolynomial>)>>,
    ) -> Option<MemoizedPiLaurentPolynomial> {
        self.pi_laurent_polynomial_for_use(budget, remaining_work, memo, false)
    }

    fn pi_laurent_polynomial_for_use(
        &self,
        budget: usize,
        remaining_work: &mut usize,
        memo: &mut Option<Vec<(usize, Option<MemoizedPiLaurentPolynomial>)>>,
        needs_owned: bool,
    ) -> Option<MemoizedPiLaurentPolynomial> {
        let key ='''
assert text.count(before) == 1
text = text.replace(before, after)
start = text.index('    fn pi_laurent_polynomial_for_use(')
end = text.index('    pub(crate) fn bounded_laurent_rational', start)
body = text[start:end]
before = '''            && let Some((_, value)) = memo
                .as_ref()
                .and_then(|memo| memo.iter().find(|(candidate, _)| *candidate == key))
        {
            return value.clone();
        }'''
after = '''            && let Some((_, value)) = memo
                .as_mut()
                .and_then(|memo| memo.iter_mut().find(|(candidate, _)| *candidate == key))
        {
            if !needs_owned {
                *value = value.take().map(MemoizedPiLaurentPolynomial::share_closed);
            }
            return value.clone();
        }'''
assert body.count(before) == 1
body = body.replace(before, after)
first = body.index('                Approximation::Add(left, right) =>')
last = body.index('                Approximation::Sqrt(child) =>', first)
body = body[:first] + '''                Approximation::Add(left, right) => left
                    .pi_laurent_polynomial_for_use(budget - 1, remaining_work, memo, true)?
                    .into_owned()
                    .add(right.pi_laurent_polynomial_for_use(
                        budget - 1, remaining_work, memo, true,
                    )?.into_owned()),
                Approximation::Multiply(left, right) => {
                    let left = left.pi_laurent_polynomial_for_use(budget - 1, remaining_work, memo, true)?;
                    let right = right.pi_laurent_polynomial_for_use(budget - 1, remaining_work, memo, false)?;
                    left.into_owned().multiply(&right)
                }
                Approximation::Negate(child) => Some(
                    child.pi_laurent_polynomial_for_use(budget - 1, remaining_work, memo, true)?
                        .into_owned().scaled(Rational::new(-1)),
                ),
                Approximation::Offset(child, shift) => Some(
                    child.pi_laurent_polynomial_for_use(budget - 1, remaining_work, memo, true)?
                        .into_owned().scaled(Self::power_of_two_rational(*shift)),
                ),
                Approximation::Inverse(child) => child
                    .pi_laurent_polynomial_for_use(budget - 1, remaining_work, memo, true)?
                    .into_owned().inverse_monomial(),
                Approximation::Square(child) => {
                    let child = child.pi_laurent_polynomial_for_use(budget - 1, remaining_work, memo, false)?;
                    child.clone().into_owned().multiply(&child)
                }
''' + body[last:]
before = '.pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?'
assert body.count(before) == 2
# The sqrt branch reads its coefficient; the linear combination consumes it.
body = body.replace(before, '.pi_laurent_polynomial_for_use(budget - 1, remaining_work, memo, false)?', 1)
body = body.replace(before, '.pi_laurent_polynomial_for_use(budget - 1, remaining_work, memo, true)?', 1)
first = body.index('        let result = result.map(|polynomial| {')
last = body.index('        if shared {', first)
body = body[:first] + '''        let result = result.map(|polynomial| {
            let value = MemoizedPiLaurentPolynomial::Owned(polynomial);
            if shared && !needs_owned {
                value.share_closed()
            } else {
                value
            }
        });
''' + body[last:]
assert 'pi_laurent_polynomial_with_memo(' not in body
text = text[:start] + body + text[end:]
path.write_text(text)
environment = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': str(WORK / 'target')}
with (OUT / 'prepare-format.log').open('w') as log:
    subprocess.run(['cargo', 'fmt', '--all'], cwd=SOURCE, env=environment, stdout=log, stderr=subprocess.STDOUT, check=True)
initial = {name: digest(SOURCE / name) for name in base['source_hashes']}
changes = [name for name, value in initial.items() if base['source_hashes'][name] != value]
assert changes == [name], changes
record = {'scope': 'Isolated caller-directed ownership for the existing per-call memo. Computed consumed values retain owned Vec storage; only borrowed nonempty closed results or borrowed memo hits share. Opaque cloning, graph memo eligibility and all recognition budgets stay unchanged. The baseline diagnostic module is unchanged.', 'base_source_record': str(base_record), 'previous_source_record': str(previous_record), 'native_source_record': str(EVIDENCE / 'native-exactness-gate/sources.json'), 'source_hashes': initial, 'only_modified_sources': changes, 'main_production_unchanged': True}
(OUT / 'candidate-sources.json').write_text(json.dumps(record, indent=2) + '\n')
with tarfile.open(OUT / 'candidate-source-delta.tar.gz', 'w:gz') as archive:
    archive.add(path, arcname=name)
(OUT / 'candidate.patch').write_text(''.join(difflib.unified_diff((BASE / name).read_text().splitlines(True), path.read_text().splitlines(True), 'a/' + name, 'b/' + name)))
shutil.copyfile(__file__, OUT / 'prepare.py')
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
print('Prepared caller-directed sharing candidate; main runtime unchanged.')
