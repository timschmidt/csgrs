"""Freeze baseline, recursive-view control and direct-projection candidate."""
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
PRIOR = ROOT / 'docs/evidence/2026-10-13-retained-polynomial-views'
WORK = ROOT / 'target/retained-polynomial-projection'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
compact = lambda body: ''.join(body.split())
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources

ALGEBRA = 'crates/hyperreal/src/computable/node/algebra.rs'
REPRESENTATION = 'crates/hyperreal/src/computable/node/representation.rs'
STRUCTURAL = 'crates/hyperreal/src/computable/node/structural_analysis.rs'
HELPER = 'crates/hyperreal/src/computable/node/retained_polynomial.rs'
TEST_NAMES = ['temporary_storage_tests.rs', 'memo_publication_tests.rs', 'shared_radical_tests.rs', 'projection_tests.rs']
TESTS = ['crates/hyperreal/src/computable/node/' + name for name in TEST_NAMES]
PREFIX = '#[cfg(test)]\nmod projection_tests;\n\n'
INPUTS = [*TEST_NAMES, 'layout-inventory-edit.json', 'projection-edits-v1.json',
          'retained_polynomial_recursive.rs', 'retained_polynomial_projected.rs',
          'algebra_projected.rs', 'structural_analysis_projected.rs']


def verify_inputs(prior):
    for name in TEST_NAMES[:3]:
        assert sha(E / name) == sha(PRIOR / name)
    assert sha(E / 'layout-inventory-edit.json') == sha(PRIOR / 'layout-inventory-edit.json')
    recursive = Path(prior['variants']['candidate']['source'])
    assert sha(E / 'retained_polynomial_recursive.rs') == sha(recursive / HELPER)
    edits = json.loads((E / 'projection-edits-v1.json').read_text())
    for path, name in [(ALGEBRA, 'algebra_projected.rs'), (STRUCTURAL, 'structural_analysis_projected.rs')]:
        expected = (recursive / path).read_text()
        for edit in edits['runtime'][path]:
            assert expected.count(edit['before']) == 1
            expected = expected.replace(edit['before'], edit['after'])
        assert expected == (E / name).read_text()
    recovered = compact((E / 'retained_polynomial_projected.rs').read_text())
    for edit in reversed(edits['helper']):
        assert recovered.count(compact(edit['after'])) == 1
        recovered = recovered.replace(compact(edit['after']), compact(edit['before']))
    assert recovered == compact((E / 'retained_polynomial_recursive.rs').read_text())
    assert (E / 'projection_tests.rs').read_text().count('#[test]') == 6
    assert (E / 'retained_polynomial_recursive.rs').read_text().count('#[test]') == 5
    assert (E / 'retained_polynomial_projected.rs').read_text().count('#[test]') == 8
    return edits


def main():
    parent = json.loads((PARENT / 'preparation-v1/protocol.json').read_text())
    prior = json.loads((PRIOR / 'preparation-v2/protocol.json').read_text())
    review = json.loads((E / 'scope-v1.json').read_text())
    assert sha(PARENT / 'preparation-v1/protocol.json') == review['parent_preparation_sha256']
    assert sha(PRIOR / 'preparation-v2/protocol.json') == review['recursive_preparation_sha256']
    assert sha(PRIOR / 'decision-v2.json') == review['recursive_decision_sha256']
    assert all(sha(ROOT / r['path']) == r['sha256'] for r in review['source_review'])
    assert json.loads((PRIOR / 'correctness-v2.json').read_text())['all_26_commands_passed']
    assert json.loads((PRIOR / 'decision-v2.json').read_text())['status'] == 'reject-recursive-retained-polynomial-views-on-scalar-performance-and-retention-costs'
    original = json.loads(gzip.decompress((PARENT / 'preparation-v1/source-hashes.json.gz').read_bytes()))
    source = Path(parent['source'])
    guard, pending = sources(), parent['pending_native_sha256']
    assert len(original) == 10131 and len(guard) == 1134 and len(pending) == 35
    assert guard == parent['main_source_sha256'] == prior['main_source_sha256']
    assert all(sha(source / p) == h for p, h in original.items())
    assert all(sha(ROOT / p) == h for p, h in pending.items())
    origin_maps = {}
    for variant, row in prior['variants'].items():
        path = PRIOR / 'preparation-v2' / (variant + '-source-hashes.json.gz')
        assert sha(path) == row['source_hashes_sha256']
        origin_maps[variant] = json.loads(gzip.decompress(path.read_bytes()))
        assert all(sha(Path(row['source']) / p) == h for p, h in origin_maps[variant].items())
    edits = verify_inputs(prior)
    WORK.mkdir(exist_ok=False)
    out = E / 'preparation-v1'
    out.mkdir(exist_ok=False)
    (out / 'parent-source-hashes.json.gz').write_bytes(gzip.compress((json.dumps(original, sort_keys=True) + '\n').encode(), mtime=0))
    variants = {}
    for variant in ['baseline', 'recursive', 'projected']:
        directory = WORK / variant
        current_source = directory / 'source'
        current_source.mkdir(parents=True)
        for name in original:
            destination = current_source / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source / name, destination)
        origin_name = 'baseline' if variant == 'baseline' else 'candidate'
        origin = Path(prior['variants'][origin_name]['source'])
        algebra = (origin / ALGEBRA).read_text()
        if variant == 'projected':
            algebra = (E / 'algebra_projected.rs').read_text()
            shutil.copyfile(E / 'structural_analysis_projected.rs', current_source / STRUCTURAL)
        (current_source / ALGEBRA).write_text(PREFIX + algebra)
        shutil.copyfile(origin / REPRESENTATION, current_source / REPRESENTATION)
        if variant != 'baseline':
            shutil.copyfile(E / ('retained_polynomial_' + variant + '.rs'), current_source / HELPER)
        for test in TESTS:
            shutil.copyfile(E / Path(test).name, current_source / test)
        with (out / (variant + '-format.log')).open('w') as log:
            subprocess.run(['rustfmt', '--edition', '2024', '--config-path', str(source / 'crates/.rustfmt.toml'), str(current_source / ALGEBRA), str(current_source / REPRESENTATION)], stdout=log, stderr=subprocess.STDOUT, check=True)
        for path in [ALGEBRA, STRUCTURAL]:
            body = (current_source / path).read_text()
            if path == ALGEBRA:
                assert body.startswith(PREFIX)
                body = body.removeprefix(PREFIX)
            if variant == 'projected':
                recovered = compact(body)
                for edit in reversed(edits['runtime'][path]):
                    assert recovered.count(compact(edit['after'])) == 1
                    recovered = recovered.replace(compact(edit['after']), compact(edit['before']))
                assert recovered == compact((origin / path).read_text()), path
            else:
                assert body == (origin / path).read_text(), path
        assert sha(current_source / REPRESENTATION) == sha(origin / REPRESENTATION)
        assert all(sha(current_source / test) == sha(E / Path(test).name) for test in TESTS)
        if variant != 'baseline':
            assert sha(current_source / HELPER) == sha(E / ('retained_polynomial_' + variant + '.rs'))
        names = set(original) | set(TESTS) | ({HELPER} if variant != 'baseline' else set())
        assert {str(p.relative_to(current_source)) for p in current_source.rglob('*') if p.is_file()} == names
        current = {p: sha(current_source / p) for p in sorted(names)}
        assert len(current) == (10135 if variant == 'baseline' else 10136)
        changes = {p: h for p, h in current.items() if original.get(p) != h}
        expected = {ALGEBRA, *TESTS}
        if variant != 'baseline':
            expected |= {REPRESENTATION, HELPER}
        if variant == 'projected':
            expected.add(STRUCTURAL)
        assert set(changes) == expected
        assert sha(current_source / 'Cargo.lock') == original['Cargo.lock']
        assert all(h == original[p] for p, h in current.items() if p.endswith('Cargo.toml'))
        (out / (variant + '-source-hashes.json.gz')).write_bytes(gzip.compress((json.dumps(current, sort_keys=True) + '\n').encode(), mtime=0))
        with tarfile.open(out / (variant + '-delta.tar.gz'), 'w:gz') as archive:
            for name in changes:
                archive.add(current_source / name, arcname=name)
        (out / (variant + '.patch')).write_text(''.join(''.join(difflib.unified_diff((source / p).read_text().splitlines(True) if p in original else [], (current_source / p).read_text().splitlines(True), 'a/' + p, 'b/' + p)) for p in changes))
        variants[variant] = {'source': str(current_source), 'target': str(directory / 'qualification-target'), 'cost_target': str(directory / 'cost-target'), 'source_files': len(current), 'source_changes_sha256': changes, 'source_hashes_sha256': sha(out / (variant + '-source-hashes.json.gz')), 'delta_archive_sha256': sha(out / (variant + '-delta.tar.gz')), 'patch_sha256': sha(out / (variant + '.patch'))}
    maps = {v: json.loads(gzip.decompress((out / (v + '-source-hashes.json.gz')).read_bytes())) for v in variants}
    assert {p for p in set(maps['baseline']) | set(maps['recursive']) if maps['baseline'].get(p) != maps['recursive'].get(p)} == {ALGEBRA, REPRESENTATION, HELPER}
    assert {p for p in maps['projected'] if maps['recursive'][p] != maps['projected'][p]} == {ALGEBRA, STRUCTURAL, HELPER}
    protocol = {
        'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
        'source_review_sha256': sha(E / 'scope-v1.json'),
        'qualification_runner_sha256': sha(E / 'qualify_v1.py'),
        'qualification_assessor_sha256': sha(E / 'assess_qualification_v1.py'),
        'input_sha256': {name: sha(E / name) for name in INPUTS},
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'parent_preparation_sha256': sha(PARENT / 'preparation-v1/protocol.json'),
        'parent_source': str(source), 'parent_source_count': len(original),
        'parent_hashes_sha256': sha(out / 'parent-source-hashes.json.gz'),
        'recursive_preparation_sha256': sha(PRIOR / 'preparation-v2/protocol.json'),
        'recursive_correctness_sha256': sha(PRIOR / 'correctness-v2.json'),
        'recursive_decision_sha256': sha(PRIOR / 'decision-v2.json'),
        'main_source_sha256': guard, 'pending_native_sha256': pending, 'variants': variants,
        'manifests_and_lockfiles_unchanged': True,
        'baseline_runtime_and_recursive_control_preserved': True,
        'only_declared_projection_changes': True, 'shared_mathematical_tests_unchanged': True,
        'shared_storage_test_functions': 8, 'shared_memo_test_functions': 3,
        'shared_alias_test_functions': 3, 'shared_projection_test_functions': 6,
        'helper_test_functions': {'baseline': 0, 'recursive': 5, 'projected': 8},
        'original_limits': parent['original_limits'], 'runtime_admitted': False, 'plan_complete': False,
    }
    assert sources() == guard
    assert all(sha(ROOT / p) == h for p, h in pending.items())
    assert all(sha(source / p) == h for p, h in original.items())
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    print(json.dumps({'variants': list(variants), 'source_counts': {k: v['source_files'] for k, v in variants.items()}, 'shared_mathematical_tests': 20, 'baseline_and_recursive_runtime_preserved': True, 'only_declared_projection_changes': True, 'main_and_parent_unchanged': True}))


if __name__ == '__main__':
    try:
        main()
    except Exception:
        failure = E / 'preparation-failure-v1.json'
        assert not failure.exists()
        failure.write_text(json.dumps({'runner_sha256': sha(Path(__file__)), 'traceback': traceback.format_exc(), 'partial_sources_frozen': True}, indent=2) + '\n')
        raise
