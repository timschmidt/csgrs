"""Prepare an isolated experiment; run only after the native gate is terminal.

Closed memoized term vectors may be shared because they own no Computable nodes.
Mutation first restores an owned vector. Opaque terms retain their original deep
clone behavior, so graph reference counts and memo eligibility remain unchanged.
"""
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
OUT = EVIDENCE / 'laurent-memo-sharing'
WORK = ROOT / 'target/laurent-memo-sharing'
BASE = ROOT / 'target/contact-index-directed-width'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
native = read(EVIDENCE / 'native-exactness-gate/sources.json')
gate = read(EVIDENCE / 'native-exactness-gate/status.json')
assert len(gate) == 7 and gate[-1]['name'] == 'full-board-smoke'
assert all(row['exit_code'] == 0 for row in gate[:-1]) and gate[-1]['exit_code'] in [0, 101]
base = read(EVIDENCE / 'contact-index-directed-width/sources.json')
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
assert all(digest(BASE / 'source' / name) == value for name, value in base['source_hashes'].items())
OUT.mkdir()
WORK.mkdir()
algebra = 'crates/hyperreal/src/computable/node/algebra.rs'
helper = 'crates/hyperreal/src/computable/node/normal_form_memo_frontier_probe.rs'
for variant in ['baseline', 'candidate']:
    directory = WORK / variant
    directory.mkdir()
    for folder in ['source', 'target']:
        with (OUT / (variant + '-copy-' + folder + '.log')).open('w') as log:
            subprocess.run(['cp', '-a', '--reflink=auto', str(BASE / folder), str(directory / folder)], stdout=log, stderr=subprocess.STDOUT, check=True)
    source = directory / 'source'
    for name, expected in native['source_hashes'].items():
        path = source / name
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / name, path)
        assert digest(path) == expected
    path = source / algebra
    text = path.read_text()
    if variant == 'candidate':
        before = '''#[derive(Clone)]
struct PiLaurentPolynomial {
    terms: Vec<(i16, RadicalCoefficient)>,
}'''
        after = '''// Only graph-free, immutable memo results use shared storage. Opaque
// coefficients preserve ordinary cloning and its Computable reference counts.
#[derive(Clone)]
enum PiLaurentTerms {
    Owned(Vec<(i16, RadicalCoefficient)>),
    Shared(std::rc::Rc<Vec<(i16, RadicalCoefficient)>>),
}

impl PiLaurentTerms {
    fn into_vec(self) -> Vec<(i16, RadicalCoefficient)> {
        match self {
            Self::Owned(terms) => terms,
            Self::Shared(terms) => std::rc::Rc::unwrap_or_clone(terms),
        }
    }

    fn share_closed(&mut self) {
        if matches!(self, Self::Owned(_))
            && !self.is_empty()
            && self.iter().all(|(_, coefficient)| coefficient.atoms.is_empty())
        {
            let previous = std::mem::replace(self, Self::Owned(Vec::new()));
            *self = Self::Shared(std::rc::Rc::new(previous.into_vec()));
        }
    }
}

impl std::ops::Deref for PiLaurentTerms {
    type Target = Vec<(i16, RadicalCoefficient)>;

    fn deref(&self) -> &Self::Target {
        match self {
            Self::Owned(terms) => terms,
            Self::Shared(terms) => terms,
        }
    }
}

impl std::ops::DerefMut for PiLaurentTerms {
    fn deref_mut(&mut self) -> &mut Self::Target {
        if matches!(self, Self::Shared(_)) {
            let previous = std::mem::replace(self, Self::Owned(Vec::new()));
            *self = Self::Owned(previous.into_vec());
        }
        match self {
            Self::Owned(terms) => terms,
            Self::Shared(_) => unreachable!("shared terms were restored to owned storage"),
        }
    }
}

impl From<Vec<(i16, RadicalCoefficient)>> for PiLaurentTerms {
    fn from(terms: Vec<(i16, RadicalCoefficient)>) -> Self {
        Self::Owned(terms)
    }
}

#[derive(Clone)]
struct PiLaurentPolynomial {
    terms: PiLaurentTerms,
}'''
        assert text.count(before) == 1
        text = text.replace(before, after)
        start = text.index('impl PiLaurentPolynomial {')
        end = text.index('\nimpl PiAtanTermsForm {', start)
        body = text[start:end]
        body = body.replace('.collect(),', '.collect::<Vec<_>>().into(),')
        body = body.replace('terms: Vec::new()', 'terms: Vec::new().into()')
        body = body.replace('in other.terms {', 'in other.terms.into_vec() {')
        body = body.replace('in self.terms {', 'in self.terms.into_vec() {')
        body = body.replace('in &mut self.terms {', 'in self.terms.iter_mut() {')
        body = body.replace('in &other.terms {', 'in other.terms.iter() {')
        text = text[:start] + body + text[end:]
        before = '        let result = if let Some(rational) = self.exact_rational() {'
        assert text.count(before) == 1
        text = text.replace(before, before.replace('let result', 'let mut result'))
        before = '''        if shared {
            memo.get_or_insert_with(|| Vec::with_capacity(8))'''
        after = '''        if shared {
            if let Some(polynomial) = &mut result {
                polynomial.terms.share_closed();
            }
            memo.get_or_insert_with(|| Vec::with_capacity(8))'''
        assert text.count(before) == 1
        text = text.replace(before, after)
        before = '        for (power, coefficient) in polynomial.terms {'
        assert text.count(before) == 1
        text = text.replace(before, before.replace('polynomial.terms', 'polynomial.terms.into_vec()'))
    text += '\n#[cfg(test)]\nmod normal_form_memo_frontier_probe;\n'
    path.write_text(text)
    shutil.copyfile(EVIDENCE / 'normal_form_memo_frontier_probe.rs', source / helper)
    environment = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': str(directory / 'target')}
    with (OUT / (variant + '-prepare-format.log')).open('w') as log:
        subprocess.run(['cargo', 'fmt', '--all'], cwd=source, env=environment, stdout=log, stderr=subprocess.STDOUT, check=True)
    expected = base['source_hashes'] | native['source_hashes']
    initial = {name: digest(source / name) for name in sorted(set(expected) | {helper})}
    differences = [name for name, value in initial.items() if expected.get(name) != value]
    assert differences == sorted([algebra, helper]), differences
    record = {
        'scope': 'Frozen native prototype plus a test-only frontier probe' if variant == 'baseline' else 'Isolated closed-result sharing in the existing per-call normal-form memo, plus the identical test-only frontier probe. No persistent cache or budget changes.',
        'native_source_record': str(EVIDENCE / 'native-exactness-gate/sources.json'),
        'fallback_source_record': str(EVIDENCE / 'contact-index-directed-width/sources.json'),
        'source_hashes': initial,
        'only_modified_sources': differences,
        'main_production_unchanged': True,
    }
    (OUT / (variant + '-sources.json')).write_text(json.dumps(record, indent=2) + '\n')
    with tarfile.open(OUT / (variant + '-source-delta.tar.gz'), 'w:gz') as archive:
        for name in differences:
            archive.add(source / name, arcname=name)
baseline_path = WORK / 'baseline/source' / algebra
candidate_path = WORK / 'candidate/source' / algebra
(OUT / 'candidate.patch').write_text(''.join(difflib.unified_diff(baseline_path.read_text().splitlines(True), candidate_path.read_text().splitlines(True), 'a/' + algebra, 'b/' + algebra)))
shutil.copyfile(__file__, OUT / 'prepare.py')
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
print('Prepared baseline/candidate sources; no main production source changed.')
