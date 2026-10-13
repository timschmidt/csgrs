"""Freeze a baseline and a candidate that omits already-published memo copies."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import traceback

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PARENT = ROOT / 'docs/evidence/2026-10-13-native-tower-sign-retention'
PREVIOUS = ROOT / 'docs/evidence/2026-10-13-radical-union-storage'
WORK = ROOT / 'target/closed-certificate-memo'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def change(body, before, after, edits):
    assert body.count(before) == 1, before
    edits.append({'before': before, 'after': after})
    return body.replace(before, after)


def main():
    parent = json.loads((PARENT / 'preparation-v1/protocol.json').read_text())
    review = json.loads((E / 'scope-v1.json').read_text())
    assert review['parent_preparation_sha256'] == sha(PARENT / 'preparation-v1/protocol.json')
    assert all(sha(ROOT / item['path']) == item['sha256'] for item in review['source_review'])
    source = Path(parent['source'])
    original = json.loads(gzip.decompress((PARENT / 'preparation-v1/source-hashes.json.gz').read_bytes()))
    guard, pending = sources(), parent['pending_native_sha256']
    assert len(original) == 10131 and len(guard) == 1134 and len(pending) == 35
    assert guard == parent['main_source_sha256']
    assert all(sha(source / p) == h for p, h in original.items())
    assert all(sha(ROOT / p) == h for p, h in pending.items())
    assert sha(E / 'temporary_storage_tests.rs') == sha(PREVIOUS / 'temporary_storage_tests.rs')
    assert json.loads((PREVIOUS / 'decision-v1.json').read_text())['status'] == 'reject-radical-union-storage-feature-only-change-not-admitted'
    WORK.mkdir(exist_ok=False)
    out = E / 'preparation-v1'
    out.mkdir(exist_ok=False)
    (out / 'parent-source-hashes.json.gz').write_bytes(gzip.compress((json.dumps(original, sort_keys=True) + '\n').encode(), mtime=0))
    algebra = 'crates/hyperreal/src/computable/node/algebra.rs'
    representation = 'crates/hyperreal/src/computable/node/representation.rs'
    tests = ['crates/hyperreal/src/computable/node/' + name for name in ['temporary_storage_tests.rs', 'memo_publication_tests.rs']]
    prefix = '#[cfg(test)]\nmod memo_publication_tests;\n\n#[cfg(test)]\nmod temporary_storage_tests;\n\n'
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
        bodies = {name: (source / name).read_text() for name in [algebra, representation]}
        edits = {name: [] for name in bodies}
        if variant == 'candidate':
            old = '''        if shared {
            memo.get_or_insert_with(|| Vec::with_capacity(8))
                .push((key, result.clone()));
            if RETAIN
'''
            new = '''        if shared {
            if RETAIN
'''
            bodies[algebra] = change(bodies[algebra], old, new, edits[algebra])
            old = '''                && let Some(polynomial) = &result
            {
                self.internal.cache.store_pi_laurent_polynomial(polynomial);
            }
        }
        result
'''
            new = '''                && let Some(polynomial) = &result
                && self.internal.cache.store_pi_laurent_polynomial(polynomial)
            {
                // Retained visits consult the published certificate before
                // the query memo. Closed coefficients own no expression nodes,
                // so omitting this duplicate does not change node sharing.
                return result;
            }
            memo.get_or_insert_with(|| Vec::with_capacity(8))
                .push((key, result.clone()));
        }
        result
'''
            bodies[algebra] = change(bodies[algebra], old, new, edits[algebra])
            old = '''    fn store_pi_laurent_polynomial(&self, polynomial: &PiLaurentPolynomial) {
        let Some(closed) = ClosedPiLaurentPolynomial::from_polynomial(polynomial) else {
            return;
        };
'''
            new = '''    fn store_pi_laurent_polynomial(&self, polynomial: &PiLaurentPolynomial) -> bool {
        let Some(closed) = ClosedPiLaurentPolynomial::from_polynomial(polynomial) else {
            return false;
        };
'''
            bodies[representation] = change(bodies[representation], old, new, edits[representation])
            old = '''        if guard.pi_laurent_polynomial.is_none() {
            guard.pi_laurent_polynomial = Some(Box::new(closed));
        }
    }
'''
            new = '''        if guard.pi_laurent_polynomial.is_none() {
            guard.pi_laurent_polynomial = Some(Box::new(closed));
        }
        true
    }
'''
            bodies[representation] = change(bodies[representation], old, new, edits[representation])
        for name, body in bodies.items():
            (current_source / name).write_text((prefix if name == algebra else '') + body)
        with (out / (variant + '-format.log')).open('w') as log:
            subprocess.run(['rustfmt', '--edition', '2024', '--config-path', str(source / 'crates/.rustfmt.toml'), str(current_source / algebra), str(current_source / representation)], stdout=log, stderr=subprocess.STDOUT, check=True)
        compact = lambda s: ''.join(s.split())
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
        current = {p: sha(current_source / p) for p in sorted(set(original) | set(tests))}
        assert len(current) == 10133
        assert {str(p.relative_to(current_source)) for p in current_source.rglob('*') if p.is_file()} == set(current)
        changes = {p: h for p, h in current.items() if original.get(p) != h}
        assert set(changes) == {algebra, *tests} | ({representation} if variant == 'candidate' else set())
        assert sha(current_source / 'Cargo.lock') == original['Cargo.lock']
        if variant == 'baseline':
            assert (current_source / algebra).read_text().removeprefix(prefix) == (source / algebra).read_text()
        (out / (variant + '-edits.json')).write_text(json.dumps(edits, indent=2) + '\n')
        (out / (variant + '-source-hashes.json.gz')).write_bytes(gzip.compress((json.dumps(current, sort_keys=True) + '\n').encode(), mtime=0))
        with tarfile.open(out / (variant + '-delta.tar.gz'), 'w:gz') as archive:
            for name in changes:
                archive.add(current_source / name, arcname=name)
        patch = ''.join(''.join(difflib.unified_diff(
            (source / p).read_text().splitlines(True) if p in original else [],
            (current_source / p).read_text().splitlines(True), 'a/' + p, 'b/' + p)) for p in changes)
        (out / (variant + '.patch')).write_text(patch)
        variants[variant] = {'source': str(current_source), 'target': str(directory / 'qualification-target'),
                             'cost_target': str(directory / 'cost-target'), 'source_files': len(current),
                             'source_changes_sha256': changes,
                             'source_hashes_sha256': sha(out / (variant + '-source-hashes.json.gz')),
                             'edits_sha256': sha(out / (variant + '-edits.json')),
                             'delta_archive_sha256': sha(out / (variant + '-delta.tar.gz')),
                             'patch_sha256': sha(out / (variant + '.patch'))}
    baseline = json.loads(gzip.decompress((out / 'baseline-source-hashes.json.gz').read_bytes()))
    candidate = json.loads(gzip.decompress((out / 'candidate-source-hashes.json.gz').read_bytes()))
    assert {p for p in baseline if baseline[p] != candidate[p]} == {algebra, representation}
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
                'source_review_sha256': sha(E / 'scope-v1.json'),
                'qualification_runner_sha256': sha(E / 'qualify_v1.py'),
                'qualification_assessor_sha256': sha(E / 'assess_qualification_v1.py'),
                'test_input_sha256': sha(E / 'temporary_storage_tests.rs'),
                'memo_test_input_sha256': sha(E / 'memo_publication_tests.rs'),
                'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                'previous_decision_sha256': sha(PREVIOUS / 'decision-v1.json'),
                'shared_test_origin': {'path': str(PREVIOUS / 'temporary_storage_tests.rs'), 'sha256': sha(PREVIOUS / 'temporary_storage_tests.rs')},
                'parent_preparation_sha256': sha(PARENT / 'preparation-v1/protocol.json'),
                'parent_source': str(source), 'parent_source_count': len(original),
                'parent_hashes_sha256': sha(out / 'parent-source-hashes.json.gz'),
                'main_source_sha256': guard, 'pending_native_sha256': pending, 'variants': variants,
                'manifests_and_lockfiles_unchanged': True,
                'shared_storage_test_functions': 8, 'new_memo_test_functions': 3,
                'original_limits': parent['original_limits'],
                'runtime_admitted': False, 'plan_complete': False}
    assert sources() == guard
    assert all(sha(ROOT / p) == h for p, h in pending.items())
    assert all(sha(source / p) == h for p, h in original.items())
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    print(json.dumps({'variants': list(variants), 'candidate_runtime_files': 2,
                      'shared_storage_tests': 8, 'new_memo_tests': 3,
                      'baseline_runtime_body_unchanged': True, 'main_and_parent_unchanged': True}))


if __name__ == '__main__':
    try:
        main()
    except Exception:
        failure = E / 'preparation-failure-v1.json'
        assert not failure.exists()
        failure.write_text(json.dumps({'runner_sha256': sha(Path(__file__)),
                                       'traceback': traceback.format_exc(),
                                       'partial_sources_frozen': True}, indent=2) + '\n')
        raise
