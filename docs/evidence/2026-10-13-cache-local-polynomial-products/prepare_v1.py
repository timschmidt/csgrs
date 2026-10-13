"""Freeze unchanged parent and typed cache-local multiplication sources."""
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
CONTROL = ROOT / 'docs/evidence/2026-10-13-retained-polynomial-projection'
WORK = ROOT / 'target/cache-local-polynomial-products'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
compact = lambda s: ''.join(s.split())
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources

NODE = 'crates/hyperreal/src/computable/node/'
ALGEBRA, REPRESENTATION, HELPER = [NODE + p for p in ['algebra.rs', 'representation.rs', 'cached_product.rs']]
TEST_NAMES = ['temporary_storage_tests.rs', 'memo_publication_tests.rs', 'shared_radical_tests.rs', 'projection_tests.rs']
INPUTS = [*TEST_NAMES, 'draft-edits-v1.json', 'runtime-edits-v1.json', 'algebra_candidate.rs', 'representation_candidate.rs', 'cached_product.rs']


def method(s, needle):
    start = s.index(needle)
    brace = s.index('{', start)
    depth, end = 1, brace + 1
    while depth:
        depth += (s[end] == '{') - (s[end] == '}')
        end += 1
    return s[start:end]


def verify_inputs(control):
    baseline = Path(control['variants']['baseline']['source'])
    for name in TEST_NAMES:
        assert sha(E / name) == sha(CONTROL / name) == sha(baseline / NODE / name)
    edits = json.loads((E / 'runtime-edits-v1.json').read_text())
    declared = json.loads((E / 'draft-edits-v1.json').read_text())
    original = (baseline / ALGEBRA).read_text()
    candidate = (E / 'algebra_candidate.rs').read_text()
    assert candidate.startswith(edits['algebra'][0]['after'])
    recovered = candidate.removeprefix(edits['algebra'][0]['after'])
    for item in edits['algebra'][1:]:
        assert recovered.count(item['after']) == 1
        recovered = recovered.replace(item['after'], item['before'])
    assert recovered == original
    header = '    fn pi_laurent_polynomial_retained<const RETAIN: bool>('
    assert compact(method(candidate, header)) == compact(declared['wrapper'])
    uncached = method(candidate, '    fn pi_laurent_polynomial_uncached<const RETAIN: bool>(')
    assert uncached.startswith(declared['uncached_header'])
    body = compact(uncached[len(declared['uncached_header']):])
    assert body.count(compact(declared['multiply_after'])) == 1
    body = body.replace(compact(declared['multiply_after']), compact(declared['multiply_before']))
    assert compact(method(original, header)) == compact(declared['normalizer_header'] + declared['cache_check']) + body
    entry = edits['representation']
    rep = (E / 'representation_candidate.rs').read_text()
    assert rep.count(entry['after']) == 1
    assert rep.replace(entry['after'], entry['before']) == (baseline / REPRESENTATION).read_text()
    review = json.loads((E / 'scope-v1.json').read_text())
    assert all(sha(ROOT / row['path']) == row['sha256'] for row in review['source_review'] + review['borrowed_kernel_review'])
    recursive = (ROOT / review['borrowed_kernel_review'][0]['path']).read_text()
    start = recursive.index('    fn multiply(self, other: &Self)')
    end = recursive.index('    fn inverse(self)', start)
    assert compact(recursive[start:end]) == compact(declared['coefficient_after'])
    prior_helper = (ROOT / review['borrowed_kernel_review'][1]['path']).read_text()
    assert method(prior_helper, '    fn multiply_closed(') == method((E / 'cached_product.rs').read_text(), '    fn multiply_closed(')
    assert (E / 'cached_product.rs').read_text().count('#[test]') == 8


def main():
    review = json.loads((E / 'scope-v1.json').read_text())
    parent = json.loads((PARENT / 'preparation-v1/protocol.json').read_text())
    control = json.loads((CONTROL / 'preparation-v2/protocol.json').read_text())
    assert sha(PARENT / 'preparation-v1/protocol.json') == review['parent_preparation_sha256']
    assert sha(CONTROL / 'preparation-v2/protocol.json') == review['baseline_preparation_sha256']
    assert sha(CONTROL / 'decision-v2.json') == review['projection_decision_sha256']
    assert json.loads((CONTROL / 'correctness-v2.json').read_text())['all_42_commands_passed']
    baseline = Path(control['variants']['baseline']['source'])
    control_map_path = CONTROL / 'preparation-v2/baseline-source-hashes.json.gz'
    assert sha(control_map_path) == control['variants']['baseline']['source_hashes_sha256']
    control_map = json.loads(gzip.decompress(control_map_path.read_bytes()))
    assert all(sha(baseline / p) == h for p, h in control_map.items())
    original = json.loads(gzip.decompress((PARENT / 'preparation-v1/source-hashes.json.gz').read_bytes()))
    parent_source = Path(parent['source'])
    guard, pending = sources(), parent['pending_native_sha256']
    assert len(original) == 10131 and len(guard) == 1134 and len(pending) == 35
    assert guard == parent['main_source_sha256'] == control['main_source_sha256']
    assert all(sha(parent_source / p) == h for p, h in original.items())
    assert all(sha(ROOT / p) == h for p, h in pending.items())
    verify_inputs(control)
    WORK.mkdir(exist_ok=False)
    out = E / 'preparation-v1'
    out.mkdir(exist_ok=False)
    (out / 'parent-source-hashes.json.gz').write_bytes(gzip.compress((json.dumps(original, sort_keys=True) + '\n').encode(), mtime=0))
    variants = {}
    for variant in ['baseline', 'candidate']:
        directory = WORK / variant
        source = directory / 'source'
        source.mkdir(parents=True)
        for name in original:
            destination = source / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(parent_source / name, destination)
        shutil.copyfile(baseline / ALGEBRA, source / ALGEBRA)
        for name in TEST_NAMES:
            shutil.copyfile(E / name, source / NODE / name)
        if variant == 'candidate':
            for path, name in [(ALGEBRA, 'algebra_candidate.rs'), (REPRESENTATION, 'representation_candidate.rs'), (HELPER, 'cached_product.rs')]:
                shutil.copyfile(E / name, source / path)
        # Inputs were formatted in a complete draft tree, so this check never edits a frozen source.
        with (out / (variant + '-format.log')).open('w') as log:
            subprocess.run(['rustfmt', '--check', '--edition', '2024', '--config-path', str(parent_source / 'crates/.rustfmt.toml'), str(source / ALGEBRA), str(source / REPRESENTATION)], stdout=log, stderr=subprocess.STDOUT, check=True)
        names = set(original) | {NODE + n for n in TEST_NAMES} | ({HELPER} if variant == 'candidate' else set())
        assert {str(p.relative_to(source)) for p in source.rglob('*') if p.is_file()} == names
        current = {p: sha(source / p) for p in sorted(names)}
        assert len(current) == (10135 if variant == 'baseline' else 10136)
        if variant == 'baseline':
            assert current == control_map
        changes = {p: h for p, h in current.items() if original.get(p) != h}
        assert set(changes) == {ALGEBRA, *(NODE + n for n in TEST_NAMES)} | ({REPRESENTATION, HELPER} if variant == 'candidate' else set())
        assert current['Cargo.lock'] == original['Cargo.lock']
        assert all(h == original[p] for p, h in current.items() if p.endswith('Cargo.toml'))
        (out / (variant + '-source-hashes.json.gz')).write_bytes(gzip.compress((json.dumps(current, sort_keys=True) + '\n').encode(), mtime=0))
        with tarfile.open(out / (variant + '-delta.tar.gz'), 'w:gz') as archive:
            for p in sorted(changes):
                archive.add(source / p, arcname=p)
        (out / (variant + '.patch')).write_text(''.join(''.join(difflib.unified_diff((parent_source / p).read_text().splitlines(True) if p in original else [], (source / p).read_text().splitlines(True), 'a/' + p, 'b/' + p)) for p in sorted(changes)))
        variants[variant] = {'source': str(source), 'target': str(directory / 'qualification-target'), 'cost_target': str(directory / 'cost-target'), 'source_files': len(current), 'source_changes_sha256': changes, 'source_hashes_sha256': sha(out / (variant + '-source-hashes.json.gz')), 'delta_archive_sha256': sha(out / (variant + '-delta.tar.gz')), 'patch_sha256': sha(out / (variant + '.patch'))}
    maps = {v: json.loads(gzip.decompress((out / (v + '-source-hashes.json.gz')).read_bytes())) for v in variants}
    assert {p for p in maps['candidate'] if maps['baseline'].get(p) != maps['candidate'][p]} == {ALGEBRA, REPRESENTATION, HELPER}
    protocol = {
        'scope': __doc__, 'runner_sha256': sha(Path(__file__)), 'source_review_sha256': sha(E / 'scope-v1.json'),
        'qualification_runner_sha256': sha(E / 'qualify_v1.py'), 'qualification_assessor_sha256': sha(E / 'assess_qualification_v1.py'),
        'input_sha256': {name: sha(E / name) for name in INPUTS},
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'parent_preparation_sha256': sha(PARENT / 'preparation-v1/protocol.json'), 'parent_source': str(parent_source),
        'parent_hashes_sha256': sha(out / 'parent-source-hashes.json.gz'), 'parent_source_count': len(original),
        'baseline_preparation_sha256': sha(CONTROL / 'preparation-v2/protocol.json'), 'baseline_correctness_sha256': sha(CONTROL / 'correctness-v2.json'),
        'projection_decision_sha256': sha(CONTROL / 'decision-v2.json'),
        'main_source_sha256': guard, 'pending_native_sha256': pending, 'variants': variants,
        'manifests_and_lockfiles_unchanged': True, 'baseline_runtime_preserved': True,
        'original_recognizer_body_recovered': True, 'coefficient_kernel_matches_qualified_control': True,
        'shared_mathematical_tests_unchanged': True, 'shared_test_functions': 20, 'helper_test_functions': {'baseline': 0, 'candidate': 8},
        'original_limits': parent['original_limits'], 'runtime_admitted': False, 'plan_complete': False,
    }
    assert sources() == guard
    assert all(sha(ROOT / p) == h for p, h in pending.items())
    verify_inputs(control)
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    print(json.dumps({'source_counts': {k: v['source_files'] for k, v in variants.items()}, 'original_recognizer_recovered': True, 'shared_tests_unchanged': True, 'main_and_parent_unchanged': True}))


if __name__ == '__main__':
    try:
        main()
    except Exception:
        failure = E / 'preparation-failure-v1.json'
        assert not failure.exists()
        failure.write_text(json.dumps({'runner_sha256': sha(Path(__file__)), 'traceback': traceback.format_exc(), 'partial_sources_frozen': True}, indent=2) + '\n')
        raise
