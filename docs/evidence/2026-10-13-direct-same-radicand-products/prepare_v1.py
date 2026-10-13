"""Freeze a shared-test parent and a direct same-radicand product candidate."""
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
CONTROL = ROOT / 'docs/evidence/2026-10-13-cache-local-polynomial-products'
COVERAGE = ROOT / 'docs/evidence/2026-10-13-same-radicand-product-coverage'
WORK = ROOT / 'target/direct-same-radicand-products'
NODE = 'crates/hyperreal/src/computable/node/'
ALGEBRA = NODE + 'algebra.rs'
OLD_TESTS = ['temporary_storage_tests.rs', 'memo_publication_tests.rs',
             'shared_radical_tests.rs', 'projection_tests.rs']
TEST_NAMES = OLD_TESTS + ['same_radicand_product_tests.rs']
INPUTS = TEST_NAMES + ['runtime-edits-v1.json', 'algebra_baseline.rs',
                      'algebra_candidate.rs', 'measure_v1.py', 'assess_cost_v1.py']
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
compact = lambda s: ''.join(s.split())
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def method(text, head):
    start = text.index(head)
    end = text.index('{', start) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


def verify_inputs():
    scope = json.loads((E / 'scope-v1.json').read_text())
    assert sha(PARENT / 'preparation-v1/protocol.json') == scope['parent_preparation_sha256']
    assert sha(CONTROL / 'preparation-v1/protocol.json') == scope['baseline_preparation_sha256']
    assert sha(COVERAGE / 'assessment-v2.json') == scope['coverage_assessment_sha256']
    assert sha(COVERAGE / 'decision-v2.json') == scope['coverage_decision_sha256']
    assert sha(COVERAGE / 'next-kernel-review-v1.json') == scope['kernel_review_sha256']
    assert all(sha(ROOT / row['path']) == row['sha256'] for row in scope['source_review'])
    control = json.loads((CONTROL / 'preparation-v1/protocol.json').read_text())
    baseline = Path(control['variants']['baseline']['source'])
    for name in OLD_TESTS:
        assert sha(E / name) == sha(CONTROL / name) == sha(baseline / NODE / name)
    edits = json.loads((E / 'runtime-edits-v1.json').read_text())
    before = (E / 'algebra_baseline.rs').read_text()
    after = (E / 'algebra_candidate.rs').read_text()
    assert before == edits['test_module_prefix'] + (baseline / ALGEBRA).read_text()
    assert before.count(edits['coefficient_method_before']) == 1
    assert after.count(edits['coefficient_method_after']) == 1
    assert after.replace(edits['coefficient_method_after'], edits['coefficient_method_before']) == before
    recovered = edits['coefficient_method_after'].replace(edits['inserted_branch'], '')
    assert recovered.replace('fn multiply(mut self,', 'fn multiply(self,', 1) == edits['coefficient_method_before']
    tests = (E / 'same_radicand_product_tests.rs').read_text()
    oracle = method(tests, '    fn original_product(').replace('fn original_product(', 'fn multiply(', 1)
    assert compact(oracle) == compact(edits['coefficient_method_before'])
    assert tests.count('#[test]') == 8
    return control


def main():
    control = verify_inputs()
    parent = json.loads((PARENT / 'preparation-v1/protocol.json').read_text())
    original = json.loads(gzip.decompress((PARENT / 'preparation-v1/source-hashes.json.gz').read_bytes()))
    parent_source = Path(parent['source'])
    guard, pending = sources(), parent['pending_native_sha256']
    assert len(original) == 10131 and len(guard) == 1134 and len(pending) == 35
    assert guard == parent['main_source_sha256'] == control['main_source_sha256']
    assert all(sha(parent_source / p) == h for p, h in original.items())
    assert all(sha(ROOT / p) == h for p, h in pending.items())
    prior_map_path = CONTROL / 'preparation-v1/baseline-source-hashes.json.gz'
    assert sha(prior_map_path) == control['variants']['baseline']['source_hashes_sha256']
    prior_map = json.loads(gzip.decompress(prior_map_path.read_bytes()))
    assert all(sha(Path(control['variants']['baseline']['source']) / p) == h for p, h in prior_map.items())
    WORK.mkdir(exist_ok=False)
    out = E / 'preparation-v1'
    out.mkdir(exist_ok=False)
    (out / 'parent-source-hashes.json.gz').write_bytes(gzip.compress((json.dumps(original, sort_keys=True) + '\n').encode(), mtime=0))
    variants, maps = {}, {}
    for variant in ['baseline', 'candidate']:
        directory = WORK / variant
        source = directory / 'source'
        source.mkdir(parents=True)
        for name in original:
            destination = source / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(parent_source / name, destination)
        shutil.copyfile(E / ('algebra_' + variant + '.rs'), source / ALGEBRA)
        for name in TEST_NAMES:
            shutil.copyfile(E / name, source / NODE / name)
        with (out / (variant + '-format.log')).open('w') as log:
            subprocess.run(['rustfmt', '--check', '--edition', '2024',
                            '--config-path', str(parent_source / 'crates/.rustfmt.toml'), str(source / ALGEBRA)],
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        names = set(original) | {NODE + n for n in TEST_NAMES}
        assert {str(p.relative_to(source)) for p in source.rglob('*') if p.is_file()} == names
        current = {p: sha(source / p) for p in sorted(names)}
        assert len(current) == 10136
        changes = {p: h for p, h in current.items() if original.get(p) != h}
        assert set(changes) == {ALGEBRA, *(NODE + n for n in TEST_NAMES)}
        assert current['Cargo.lock'] == original['Cargo.lock']
        assert all(h == original[p] for p, h in current.items() if p.endswith('Cargo.toml'))
        (out / (variant + '-source-hashes.json.gz')).write_bytes(gzip.compress((json.dumps(current, sort_keys=True) + '\n').encode(), mtime=0))
        with tarfile.open(out / (variant + '-delta.tar.gz'), 'w:gz') as archive:
            for p in sorted(changes):
                archive.add(source / p, arcname=p)
        (out / (variant + '.patch')).write_text(''.join(''.join(difflib.unified_diff(
            (parent_source / p).read_text().splitlines(True) if p in original else [],
            (source / p).read_text().splitlines(True), 'a/' + p, 'b/' + p)) for p in sorted(changes)))
        maps[variant] = current
        variants[variant] = {
            'source': str(source), 'target': str(directory / 'qualification-target'),
            'cost_target': str(directory / 'cost-target'), 'source_files': len(current),
            'source_changes_sha256': changes,
            'source_hashes_sha256': sha(out / (variant + '-source-hashes.json.gz')),
            'delta_archive_sha256': sha(out / (variant + '-delta.tar.gz')),
            'patch_sha256': sha(out / (variant + '.patch')),
        }
    assert {p for p in maps['candidate'] if maps['baseline'][p] != maps['candidate'][p]} == {ALGEBRA}
    protocol = {
        'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
        'source_review_sha256': sha(E / 'scope-v1.json'),
        'qualification_runner_sha256': sha(E / 'qualify_v1.py'),
        'qualification_assessor_sha256': sha(E / 'assess_qualification_v1.py'),
        'input_sha256': {name: sha(E / name) for name in INPUTS},
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'parent_preparation_sha256': sha(PARENT / 'preparation-v1/protocol.json'),
        'parent_source': str(parent_source), 'parent_source_count': len(original),
        'parent_hashes_sha256': sha(out / 'parent-source-hashes.json.gz'),
        'main_source_sha256': guard, 'pending_native_sha256': pending, 'variants': variants,
        'baseline_preparation_sha256': sha(CONTROL / 'preparation-v1/protocol.json'),
        'manifests_and_lockfiles_unchanged': True, 'baseline_runtime_preserved': True,
        'original_general_fallback_recovered': True, 'original_test_oracle_verified': True,
        'shared_test_functions': 28, 'all_shared_tests_byte_identical_between_variants': True,
        'original_limits': parent['original_limits'], 'runtime_admitted': False, 'plan_complete': False,
    }
    assert sources() == guard
    assert all(sha(ROOT / p) == h for p, h in pending.items())
    verify_inputs()
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    print(json.dumps({'source_counts': {k: v['source_files'] for k, v in variants.items()},
                      'original_fallback_recovered': True, 'shared_tests': 28,
                      'main_and_parent_unchanged': True}))


if __name__ == '__main__':
    try:
        main()
    except Exception:
        failure = E / 'preparation-failure-v1.json'
        assert not failure.exists()
        failure.write_text(json.dumps({'runner_sha256': sha(Path(__file__)),
            'traceback': traceback.format_exc(), 'partial_sources_frozen': True}, indent=2) + '\n')
        raise
