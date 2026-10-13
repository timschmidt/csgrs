"""Freeze owned scratch and immutable retained radical storage from the endpoint/P4 parent."""
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
PREVIOUS = ROOT / 'docs/evidence/2026-10-13-shared-radical-storage'
WORK = ROOT / 'target/owned-radical-scratch-v2'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def change(body, before, after, edits):
    assert body.count(before) == 1, before
    edits.append({'before': before, 'after': after})
    return body.replace(before, after)


def main():
    parent = json.loads((PARENT / 'preparation-v1/protocol.json').read_text())
    review = json.loads((E / 'scope-v2.json').read_text())
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
    assert json.loads((PREVIOUS / 'decision-v1.json').read_text())['status'] == 'reject-shared-radical-storage-on-scalar-performance-and-memory-costs'
    inventory_edit = json.loads((E / 'alias-inventory-edit.json').read_text())
    compact_test = lambda body: ''.join(body.split())
    actual_alias = compact_test((E / 'shared_radical_tests.rs').read_text())
    assert actual_alias.count(compact_test(inventory_edit['after'])) == 1
    assert actual_alias.replace(compact_test(inventory_edit['after']), compact_test(inventory_edit['before'])) == compact_test((PREVIOUS / 'shared_radical_tests.rs').read_text())
    correction = review['helper_correction']
    assert sha(E / 'radical_storage.rs') == correction['old_sha256']
    old_helper = (E / 'radical_storage.rs').read_text()
    assert old_helper.count(correction['before']) == 1
    assert old_helper.replace(correction['before'], correction['after']) == (E / 'radical_storage_v2.rs').read_text()
    assert sha(E / 'qualification-failure-v1.json') == review['qualification_failure_v1_sha256']
    WORK.mkdir(exist_ok=False)
    out = E / 'preparation-v2'
    out.mkdir(exist_ok=False)
    (out / 'parent-source-hashes.json.gz').write_bytes(gzip.compress((json.dumps(original, sort_keys=True) + '\n').encode(), mtime=0))
    algebra = 'crates/hyperreal/src/computable/node/algebra.rs'
    helper = 'crates/hyperreal/src/computable/node/radical_storage.rs'
    tests = ['crates/hyperreal/src/computable/node/' + name for name in ['temporary_storage_tests.rs', 'memo_publication_tests.rs', 'shared_radical_tests.rs']]
    prefix = '#[cfg(test)]\nmod shared_radical_tests;\n\n#[cfg(test)]\nmod memo_publication_tests;\n\n#[cfg(test)]\nmod temporary_storage_tests;\n\n'
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
        bodies = {algebra: (source / algebra).read_text()}
        edits = {name: [] for name in bodies}
        if variant == 'candidate':
            shutil.copyfile(E / 'radical_storage_v2.rs', current_source / helper)
            replacements = [
                ('#[derive(Clone)]\nstruct PiAtanLinearForm {',
                 'mod radical_storage;\nuse radical_storage::{ClosedRadicalTerms, RadicalTerms};\n\n#[derive(Clone)]\nstruct PiAtanLinearForm {'),
                ('    radicals: Vec<(Rational, Rational)>,\n    atoms:',
                 '    radicals: RadicalTerms,\n    atoms:'),
                ('struct ClosedRadicalCoefficient {\n    rational: Rational,\n    radicals: Vec<(Rational, Rational)>,',
                 'struct ClosedRadicalCoefficient {\n    rational: Rational,\n    radicals: ClosedRadicalTerms,'),
                ('            rational: value,\n            radicals: Vec::new(),',
                 '            rational: value,\n            radicals: RadicalTerms::new(),'),
                ('            radicals: vec![(radicand, value)],',
                 '            radicals: vec![(radicand, value)].into(),'),
                ('            radicals: Vec::new(),\n            atoms: vec![(value, Rational::one())],',
                 '            radicals: RadicalTerms::new(),\n            atoms: vec![(value, Rational::one())],'),
                ('                    radicals: vec![(radicand.clone(), -coefficient * inverse_norm)],',
                 '                    radicals: vec![(radicand.clone(), -coefficient * inverse_norm)].into(),'),
                ('ClosedRadicalCoefficient {\n                            rational: coefficient.rational.clone(),\n                            radicals: coefficient.radicals.clone(),',
                 'ClosedRadicalCoefficient {\n                            rational: coefficient.rational.clone(),\n                            radicals: coefficient.radicals.retained(),'),
                ('RadicalCoefficient {\n                            rational: coefficient.rational.clone(),\n                            radicals: coefficient.radicals.clone(),\n                            atoms: Vec::new(),',
                 'RadicalCoefficient {\n                            rational: coefficient.rational.clone(),\n                            radicals: coefficient.radicals.to_terms(),\n                            atoms: Vec::new(),'),
            ]
            for before, after in replacements:
                bodies[algebra] = change(bodies[algebra], before, after, edits[algebra])
        for name, body in bodies.items():
            (current_source / name).write_text((prefix if name == algebra else '') + body)
        with (out / (variant + '-format.log')).open('w') as log:
            subprocess.run(['rustfmt', '--edition', '2024', '--config-path', str(source / 'crates/.rustfmt.toml'), str(current_source / algebra)], stdout=log, stderr=subprocess.STDOUT, check=True)
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
        current = {p: sha(current_source / p) for p in sorted(set(original) | set(tests) | ({helper} if variant == 'candidate' else set()))}
        assert len(current) == (10135 if variant == 'candidate' else 10134)
        assert {str(p.relative_to(current_source)) for p in current_source.rglob('*') if p.is_file()} == set(current)
        changes = {p: h for p, h in current.items() if original.get(p) != h}
        assert set(changes) == {algebra, *tests} | ({helper} if variant == 'candidate' else set())
        assert sha(current_source / 'Cargo.lock') == original['Cargo.lock']
        if variant == 'candidate':
            assert sha(current_source / helper) == sha(E / 'radical_storage_v2.rs')
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
    assert {p for p in set(baseline) | set(candidate) if baseline.get(p) != candidate.get(p)} == {algebra, helper}
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
                'source_review_sha256': sha(E / 'scope-v2.json'),
                'helper_correction': correction,
                'qualification_failure_v1_sha256': sha(E / 'qualification-failure-v1.json'),
                'qualification_runner_sha256': sha(E / 'qualify_v2.py'),
                'qualification_assessor_sha256': sha(E / 'assess_qualification_v2.py'),
                'test_input_sha256': sha(E / 'temporary_storage_tests.rs'),
                'memo_test_input_sha256': sha(E / 'memo_publication_tests.rs'),
                'alias_test_input_sha256': sha(E / 'shared_radical_tests.rs'),
                'alias_inventory_edit_sha256': sha(E / 'alias-inventory-edit.json'),
                'alias_test_origin': {'path': str(PREVIOUS / 'shared_radical_tests.rs'), 'sha256': sha(PREVIOUS / 'shared_radical_tests.rs')},
                'storage_helper_input_sha256': sha(E / 'radical_storage_v2.rs'),
                'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                'previous_decision_sha256': sha(PREVIOUS / 'decision-v1.json'),
                'shared_test_origin': {'path': str(PREVIOUS / 'temporary_storage_tests.rs'), 'sha256': sha(PREVIOUS / 'temporary_storage_tests.rs')},
                'parent_preparation_sha256': sha(PARENT / 'preparation-v1/protocol.json'),
                'parent_source': str(source), 'parent_source_count': len(original),
                'parent_hashes_sha256': sha(out / 'parent-source-hashes.json.gz'),
                'main_source_sha256': guard, 'pending_native_sha256': pending, 'variants': variants,
                'manifests_and_lockfiles_unchanged': True,
                'shared_storage_test_functions': 8, 'shared_memo_test_functions': 3,
                'shared_alias_test_functions': 3, 'new_candidate_helper_test_functions': 2,
                'original_limits': parent['original_limits'],
                'runtime_admitted': False, 'plan_complete': False}
    assert sources() == guard
    assert all(sha(ROOT / p) == h for p, h in pending.items())
    assert all(sha(source / p) == h for p, h in original.items())
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    print(json.dumps({'variants': list(variants), 'candidate_runtime_files': 2,
                      'shared_storage_tests': 8, 'shared_memo_tests': 3,
                      'shared_alias_tests': 3, 'new_candidate_helper_tests': 2,
                      'baseline_runtime_body_unchanged': True, 'main_and_parent_unchanged': True}))


if __name__ == '__main__':
    try:
        main()
    except Exception:
        failure = E / 'preparation-failure-v2.json'
        assert not failure.exists()
        failure.write_text(json.dumps({'runner_sha256': sha(Path(__file__)),
                                       'traceback': traceback.format_exc(),
                                       'partial_sources_frozen': True}, indent=2) + '\n')
        raise
