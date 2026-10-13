"""Freeze whole-polynomial retained views and the unchanged owned baseline."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile
import traceback

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PARENT = ROOT / 'docs/evidence/2026-10-13-native-tower-sign-retention'
PREVIOUS = ROOT / 'docs/evidence/2026-10-13-owned-radical-scratch'
WORK = ROOT / 'target/retained-polynomial-views-v2'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
compact = lambda body: ''.join(body.split())
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def change(body, before, after, edits):
    assert body.count(before) == 1, before
    edits.append({'before': before, 'after': after})
    return body.replace(before, after)


def candidate_edits(algebra, representation):
    edits = {'algebra': [], 'representation': []}
    algebra = change(algebra, '#[derive(Clone)]\nstruct PiAtanLinearForm {',
        'mod retained_polynomial;\nuse retained_polynomial::PiLaurentValue;\n\n#[derive(Clone)]\nstruct PiAtanLinearForm {', edits['algebra'])
    start = algebra.index('    fn multiply(self, other: &Self) -> Option<Self> {')
    end = algebra.index('    fn inverse(self) -> Option<Self> {', start)
    original = algebra[start:end]
    signature = '    fn multiply(self, other: &Self) -> Option<Self> {\n'
    body = original.removeprefix(signature)
    for field in ['rational', 'radicals', 'atoms']:
        body = body.replace('&other.' + field, 'other_' + field)
        body = body.replace('other.' + field, 'other_' + field)
    replacement = signature + '        self.multiply_parts(&other.rational, &other.radicals, &other.atoms)\n    }\n\n'
    replacement += '    fn multiply_parts(\n        self,\n        other_rational: &Rational,\n        other_radicals: &[(Rational, Rational)],\n        other_atoms: &[(Computable, Rational)],\n    ) -> Option<Self> {\n' + body
    algebra = change(algebra, original, replacement, edits['algebra'])
    start = algebra.index('    fn pi_laurent_polynomial_retained<const RETAIN: bool>(')
    end = algebra.index('    pub(crate) fn bounded_laurent_rational(', start)
    original = algebra[start:end]
    replacement = original.replace('pi_laurent_polynomial_retained', 'pi_laurent_value_retained')
    replacement = replacement.replace('    ) -> Option<PiLaurentPolynomial> {', '    ) -> Option<PiLaurentValue> {', 1)
    replacement = replacement.replace('self.internal.cache.pi_laurent_polynomial()', 'self.internal.cache.pi_laurent_value()')
    replacement = replacement.replace('            return value.clone();', '            return value.clone().map(PiLaurentValue::Owned);')
    replacement = replacement.replace('                    let radicand = match child.terms.as_slice() {',
        '                    let child = child.into_polynomial();\n                    let radicand = match child.terms.as_slice() {')
    assert replacement.endswith('        result\n    }\n\n')
    replacement = replacement[:-len('        result\n    }\n\n')] + '        result.map(PiLaurentValue::Owned)\n    }\n\n'
    wrapper = '''    fn pi_laurent_polynomial_retained<const RETAIN: bool>(
        &self,
        budget: usize,
        remaining_work: &mut usize,
        memo: &mut Option<Vec<(usize, Option<PiLaurentPolynomial>)>>,
        reused: &mut bool,
    ) -> Option<PiLaurentPolynomial> {
        self.pi_laurent_value_retained::<RETAIN>(budget, remaining_work, memo, reused)
            .map(PiLaurentValue::into_polynomial)
    }

'''
    correction = json.loads((E / 'scope-v2.json').read_text())['formatting_correction']
    assert replacement.count(correction['before']) == correction['occurrences'] == 3
    replacement = replacement.replace(correction['before'], correction['after'])
    algebra = change(algebra, original, wrapper + replacement, edits['algebra'])
    representation = change(representation, 'pi_laurent_polynomial: Option<Box<ClosedPiLaurentPolynomial>>,',
        'pi_laurent_polynomial: Option<Arc<ClosedPiLaurentPolynomial>>,', edits['representation'])
    representation = change(representation, '    fn pi_laurent_polynomial(&self) -> Option<PiLaurentPolynomial> {', '''    fn pi_laurent_value(&self) -> Option<PiLaurentValue> {
        let guard = self
            .cell()?
            .read()
            .unwrap_or_else(|error| error.into_inner());
        Some(PiLaurentValue::Retained(Arc::clone(
            guard.pi_laurent_polynomial.as_ref()?,
        )))
    }

    #[cfg(test)]
    fn pi_laurent_polynomial(&self) -> Option<PiLaurentPolynomial> {''', edits['representation'])
    representation = change(representation, 'guard.pi_laurent_polynomial = Some(Box::new(closed));',
        'guard.pi_laurent_polynomial = Some(Arc::new(closed));', edits['representation'])
    return algebra, representation, edits


def verify_borrowed_algorithm(parent_algebra):
    start = parent_algebra.index('    fn multiply(self, other: Self) -> Option<Self> {', parent_algebra.index('impl PiLaurentPolynomial {'))
    end = parent_algebra.index('    fn inverse_monomial(', start)
    owned = parent_algebra[start:end]
    helper = (E / 'retained_polynomial.rs').read_text()
    start = helper.index('    fn multiply_closed(')
    end = helper.index('\n}\n\n#[cfg(test)]', start)
    closed = helper[start:end]
    normalize = lambda body: compact(re.sub(r'//[^\n]*', '', body))
    closed = normalize(closed)
    closed = closed.replace('fnmultiply_closed(self,other:&ClosedPiLaurentPolynomial)', 'fnmultiply(self,other:Self)')
    closed = closed.replace('.multiply_parts(&right_coefficient.rational,&right_coefficient.radicals,&[],)', '.multiply(right_coefficient)')
    assert closed == normalize(owned), 'borrowed polynomial operations must match the owned algorithm'


def main():
    parent = json.loads((PARENT / 'preparation-v1/protocol.json').read_text())
    review = json.loads((E / 'scope-v2.json').read_text())
    assert sha(PARENT / 'preparation-v1/protocol.json') == review['parent_preparation_sha256']
    assert sha(PREVIOUS / 'decision-v2.json') == review['previous_decision_sha256']
    assert all(sha(ROOT / r['path']) == r['sha256'] for r in review['source_review'])
    assert sha(E / 'preparation-failure-assessment-v1.json') == review['preparation_failure_sha256']
    frozen_v1 = {}
    for variant, digest in review['v1_formatted_sources_sha256'].items():
        path = E / 'failed-preparation-v1' / (variant + '-source-hashes.json.gz')
        assert sha(path) == digest
        frozen_v1[variant] = json.loads(gzip.decompress(path.read_bytes()))
        assert all(sha(ROOT / 'target/retained-polynomial-views' / variant / 'source' / name) == value for name, value in frozen_v1[variant].items())
    original = json.loads(gzip.decompress((PARENT / 'preparation-v1/source-hashes.json.gz').read_bytes()))
    source = Path(parent['source'])
    guard, pending = sources(), parent['pending_native_sha256']
    assert len(original) == 10131 and len(guard) == 1134 and len(pending) == 35
    assert guard == parent['main_source_sha256']
    assert all(sha(source / p) == h for p, h in original.items())
    assert all(sha(ROOT / p) == h for p, h in pending.items())
    for name in ['memo_publication_tests.rs', 'shared_radical_tests.rs']:
        assert sha(E / name) == sha(PREVIOUS / name)
    inventory = json.loads((E / 'layout-inventory-edit.json').read_text())
    current_test = compact((E / 'temporary_storage_tests.rs').read_text())
    assert current_test.count(compact(inventory['after'])) == 1
    assert current_test.replace(compact(inventory['after']), compact(inventory['before'])) == compact((ROOT / inventory['origin_path']).read_text())
    algebra = 'crates/hyperreal/src/computable/node/algebra.rs'
    representation = 'crates/hyperreal/src/computable/node/representation.rs'
    helper = 'crates/hyperreal/src/computable/node/retained_polynomial.rs'
    tests = ['crates/hyperreal/src/computable/node/' + name for name in ['temporary_storage_tests.rs', 'memo_publication_tests.rs', 'shared_radical_tests.rs']]
    prefix = '#[cfg(test)]\nmod shared_radical_tests;\n\n#[cfg(test)]\nmod memo_publication_tests;\n\n#[cfg(test)]\nmod temporary_storage_tests;\n\n'
    verify_borrowed_algorithm((source / algebra).read_text())
    WORK.mkdir(exist_ok=False)
    out = E / 'preparation-v2'
    out.mkdir(exist_ok=False)
    (out / 'parent-source-hashes.json.gz').write_bytes(gzip.compress((json.dumps(original, sort_keys=True) + '\n').encode(), mtime=0))
    variants = {}
    for variant in ['baseline', 'candidate']:
        directory = WORK / variant
        current_source = directory / 'source'
        current_source.mkdir(parents=True)
        for name in original:
            destination = current_source / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source / name, destination)
        for test in tests:
            shutil.copyfile(E / Path(test).name, current_source / test)
        bodies = {algebra: (source / algebra).read_text(), representation: (source / representation).read_text()}
        edits = {path: [] for path in bodies}
        if variant == 'candidate':
            bodies[algebra], bodies[representation], changes = candidate_edits(bodies[algebra], bodies[representation])
            edits = {algebra: changes['algebra'], representation: changes['representation']}
            shutil.copyfile(E / 'retained_polynomial.rs', current_source / helper)
        for name, body in bodies.items():
            (current_source / name).write_text((prefix if name == algebra else '') + body)
        with (out / (variant + '-format.log')).open('w') as log:
            subprocess.run(['rustfmt', '--edition', '2024', '--config-path', str(source / 'crates/.rustfmt.toml'), str(current_source / algebra), str(current_source / representation)], stdout=log, stderr=subprocess.STDOUT, check=True)
        for name in bodies:
            recovered = compact((current_source / name).read_text())
            if name == algebra:
                recovered = recovered.removeprefix(compact(prefix))
            for edit in reversed(edits[name]):
                before, after = compact(edit['before']), compact(edit['after'])
                assert recovered.count(after) == 1, edit
                recovered = recovered.replace(after, before)
            assert recovered == compact((source / name).read_text()), name
        assert all(sha(current_source / test) == sha(E / Path(test).name) for test in tests)
        current = {p: sha(current_source / p) for p in sorted(set(original) | set(tests) | ({helper} if variant == 'candidate' else set()))}
        assert len(current) == (10135 if variant == 'candidate' else 10134)
        assert current == frozen_v1[variant], 'v2 must reproduce every preserved formatted v1 source byte'
        assert {str(p.relative_to(current_source)) for p in current_source.rglob('*') if p.is_file()} == set(current)
        changes = {p: h for p, h in current.items() if original.get(p) != h}
        assert set(changes) == {algebra, *tests} | ({representation, helper} if variant == 'candidate' else set())
        assert sha(current_source / 'Cargo.lock') == original['Cargo.lock']
        if variant == 'candidate':
            assert sha(current_source / helper) == sha(E / 'retained_polynomial.rs')
        else:
            assert (current_source / algebra).read_text().removeprefix(prefix) == (source / algebra).read_text()
        (out / (variant + '-edits.json')).write_text(json.dumps(edits, indent=2) + '\n')
        (out / (variant + '-source-hashes.json.gz')).write_bytes(gzip.compress((json.dumps(current, sort_keys=True) + '\n').encode(), mtime=0))
        with tarfile.open(out / (variant + '-delta.tar.gz'), 'w:gz') as archive:
            for name in changes:
                archive.add(current_source / name, arcname=name)
        (out / (variant + '.patch')).write_text(''.join(''.join(difflib.unified_diff((source / p).read_text().splitlines(True) if p in original else [], (current_source / p).read_text().splitlines(True), 'a/' + p, 'b/' + p)) for p in changes))
        variants[variant] = {'source': str(current_source), 'target': str(directory / 'qualification-target'), 'cost_target': str(directory / 'cost-target'), 'source_files': len(current), 'source_changes_sha256': changes, 'source_hashes_sha256': sha(out / (variant + '-source-hashes.json.gz')), 'edits_sha256': sha(out / (variant + '-edits.json')), 'delta_archive_sha256': sha(out / (variant + '-delta.tar.gz')), 'patch_sha256': sha(out / (variant + '.patch'))}
    baseline = json.loads(gzip.decompress((out / 'baseline-source-hashes.json.gz').read_bytes()))
    candidate = json.loads(gzip.decompress((out / 'candidate-source-hashes.json.gz').read_bytes()))
    assert {p for p in set(baseline) | set(candidate) if baseline.get(p) != candidate.get(p)} == {algebra, representation, helper}
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)), 'source_review_sha256': sha(E / 'scope-v2.json'), 'preparation_failure_sha256': review['preparation_failure_sha256'], 'v1_formatted_sources_sha256': review['v1_formatted_sources_sha256'], 'fresh_sources_identical_to_formatted_v1': True, 'qualification_runner_sha256': sha(E / 'qualify_v2.py'), 'qualification_assessor_sha256': sha(E / 'assess_qualification_v2.py'), 'input_sha256': {name: sha(E / name) for name in ['temporary_storage_tests.rs', 'memo_publication_tests.rs', 'shared_radical_tests.rs', 'retained_polynomial.rs', 'layout-inventory-edit.json']}, 'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(), 'previous_decision_sha256': sha(PREVIOUS / 'decision-v2.json'), 'parent_preparation_sha256': sha(PARENT / 'preparation-v1/protocol.json'), 'parent_source': str(source), 'parent_source_count': len(original), 'parent_hashes_sha256': sha(out / 'parent-source-hashes.json.gz'), 'main_source_sha256': guard, 'pending_native_sha256': pending, 'variants': variants, 'manifests_and_lockfiles_unchanged': True, 'borrowed_polynomial_tokens_match_owned_algorithm': True, 'shared_mathematical_tests_unchanged': True, 'shared_storage_test_functions': 8, 'shared_memo_test_functions': 3, 'shared_alias_test_functions': 3, 'new_candidate_helper_test_functions': 5, 'original_limits': parent['original_limits'], 'runtime_admitted': False, 'plan_complete': False}
    assert sources() == guard
    assert all(sha(ROOT / p) == h for p, h in pending.items())
    assert all(sha(source / p) == h for p, h in original.items())
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    print(json.dumps({'variants': list(variants), 'candidate_runtime_files': 3, 'shared_mathematical_tests': 14, 'new_candidate_helper_tests': 5, 'borrowed_operation_order_verified': True, 'main_and_parent_unchanged': True}))


if __name__ == '__main__':
    try:
        main()
    except Exception:
        failure = E / 'preparation-failure-v2.json'
        assert not failure.exists()
        failure.write_text(json.dumps({'runner_sha256': sha(Path(__file__)), 'traceback': traceback.format_exc(), 'partial_sources_frozen': True}, indent=2) + '\n')
        raise
