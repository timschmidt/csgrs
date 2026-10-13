"""Freeze fresh baseline, direct and outlined product sources with identical controls."""
import difflib
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import traceback

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PRIOR = ROOT / 'docs/evidence/2026-10-13-direct-same-radicand-products'
WORK = ROOT / 'target/outlined-same-radicand-products'
NODE = 'crates/hyperreal/src/computable/node/'
ALGEBRA = NODE + 'algebra.rs'
TEST_NAMES = ['temporary_storage_tests.rs', 'memo_publication_tests.rs',
              'shared_radical_tests.rs', 'projection_tests.rs', 'same_radicand_product_tests.rs']
VARIANTS = ['baseline', 'direct', 'outlined']
INPUTS = TEST_NAMES + ['algebra_' + v + '.rs' for v in VARIANTS] + [
    'runtime-edits-v1.json', 'draft-format-review-v1.json', 'measure_v1.py', 'assess_cost_v1.py']
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
load = lambda p: json.loads(p.read_text())
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def verify_inputs():
    scope = load(E / 'scope-v1.json')
    for name, digest in scope['prior_evidence_sha256'].items():
        assert sha(PRIOR / name) == digest, name
    for row in scope['source_review']:
        assert sha(ROOT / row['path']) == row['sha256']
    prior = load(PRIOR / 'preparation-v2/protocol.json')
    spec = importlib.util.spec_from_file_location('_prior_direct_qualification', PRIOR / 'qualify_v2.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.verify(prior)
    assert load(PRIOR / 'correctness-v2.json')['all_28_commands_passed']
    assert load(PRIOR / 'decision-v1.json')['status'] == 'reject-direct-same-radicand-products-on-scalar-timing-costs'
    baseline = Path(prior['variants']['baseline']['source'])
    direct = Path(prior['variants']['candidate']['source'])
    for name in TEST_NAMES:
        assert sha(E / name) == sha(baseline / NODE / name) == sha(direct / NODE / name)
    assert sum((E / n).read_text().count('#[test]') for n in TEST_NAMES) == 28
    assert (E / 'algebra_baseline.rs').read_bytes() == (baseline / ALGEBRA).read_bytes()
    assert (E / 'algebra_direct.rs').read_bytes() == (direct / ALGEBRA).read_bytes()
    edits = load(E / 'runtime-edits-v1.json')
    before, control, after = [(E / ('algebra_' + v + '.rs')).read_text() for v in VARIANTS]
    assert control.count(edits['direct_method']) == 1
    assert after.count(edits['helper'] + edits['outlined_method']) == 1
    assert control.replace(edits['direct_method'], edits['original_method']) == before
    assert after.replace(edits['helper'] + edits['outlined_method'], edits['direct_method']) == control
    assert edits['outlined_method'].replace(edits['outlined_branch'], '') == edits['original_method']
    assert edits['direct_method'].replace(edits['direct_branch'], '').replace('fn multiply(mut self,', 'fn multiply(self,', 1) == edits['original_method']
    assert after.count('multiply_same_radicand') == 2
    assert '#[inline(never)]\n    fn multiply_same_radicand' in edits['helper']
    # Independently compare the arithmetic suffix, allowing only indentation and return wrapping.
    body = edits['direct_branch'].split('            let rational =', 1)[1].rsplit('        }', 1)[0]
    body = '            let rational =' + body
    body = '\n'.join(line[4:] if line.startswith('    ') else line for line in body.splitlines()) + '\n'
    body = body.replace('return Some(Self::rational(rational));', 'return Self::rational(rational);').replace('        return Some(self);\n', '        self\n')
    assert edits['helper'].split('        let rational =', 1)[1].rsplit('    }\n\n', 1)[0] == body.split('        let rational =', 1)[1]
    return prior


def main():
    prior = verify_inputs()
    source_parent = Path(prior['variants']['baseline']['source'])
    parent_hash_path = PRIOR / 'preparation-v2/baseline-source-hashes.json.gz'
    assert sha(parent_hash_path) == prior['variants']['baseline']['source_hashes_sha256']
    original = json.loads(gzip.decompress(parent_hash_path.read_bytes()))
    guard, pending = sources(), prior['pending_native_sha256']
    assert len(original) == 10136 and len(guard) == 1134 and len(pending) == 35
    assert guard == prior['main_source_sha256']
    assert all(sha(ROOT / p) == h for p, h in pending.items())
    WORK.mkdir(exist_ok=False)
    out = E / 'preparation-v1'
    out.mkdir(exist_ok=False)
    shutil.copyfile(parent_hash_path, out / 'parent-source-hashes.json.gz')
    variants, maps = {}, {}
    for variant in VARIANTS:
        directory = WORK / variant
        source = directory / 'source'
        source.mkdir(parents=True)
        for name in original:
            destination = source / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source_parent / name, destination)
        shutil.copyfile(E / ('algebra_' + variant + '.rs'), source / ALGEBRA)
        with (out / (variant + '-format.log')).open('w') as log:
            subprocess.run(['rustfmt', '--check', '--edition', '2024',
                '--config-path', str(source_parent / 'crates/.rustfmt.toml'), str(source / ALGEBRA)],
                stdout=log, stderr=subprocess.STDOUT, check=True)
        assert {str(p.relative_to(source)) for p in source.rglob('*') if p.is_file()} == set(original)
        current = {p: sha(source / p) for p in sorted(original)}
        changes = {p: h for p, h in current.items() if original[p] != h}
        assert set(changes) == (set() if variant == 'baseline' else {ALGEBRA})
        if variant != 'outlined':
            previous_variant = 'baseline' if variant == 'baseline' else 'candidate'
            assert current == json.loads(gzip.decompress((PRIOR / 'preparation-v2' / (previous_variant + '-source-hashes.json.gz')).read_bytes()))
        assert current['Cargo.lock'] == original['Cargo.lock']
        assert all(h == original[p] for p, h in current.items() if p.endswith('Cargo.toml'))
        (out / (variant + '-source-hashes.json.gz')).write_bytes(gzip.compress((json.dumps(current, sort_keys=True) + '\n').encode(), mtime=0))
        with tarfile.open(out / (variant + '-delta.tar.gz'), 'w:gz') as archive:
            for p in sorted(changes):
                archive.add(source / p, arcname=p)
        patch = ''.join(''.join(difflib.unified_diff((source_parent / p).read_text().splitlines(True),
            (source / p).read_text().splitlines(True), 'a/' + p, 'b/' + p)) for p in sorted(changes))
        (out / (variant + '.patch')).write_text(patch)
        maps[variant] = current
        variants[variant] = {'source': str(source), 'target': str(directory / 'qualification-target'),
            'cost_target': str(directory / 'cost-target'), 'source_files': len(current),
            'source_changes_sha256': changes,
            'source_hashes_sha256': sha(out / (variant + '-source-hashes.json.gz')),
            'delta_archive_sha256': sha(out / (variant + '-delta.tar.gz')),
            'patch_sha256': sha(out / (variant + '.patch'))}
    assert {p for p in original if maps['direct'][p] != maps['outlined'][p]} == {ALGEBRA}
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
        'source_review_sha256': sha(E / 'scope-v1.json'),
        'qualification_runner_sha256': sha(E / 'qualify_v1.py'),
        'qualification_assessor_sha256': sha(E / 'assess_qualification_v1.py'),
        'input_sha256': {name: sha(E / name) for name in INPUTS},
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'prior_preparation_sha256': sha(PRIOR / 'preparation-v2/protocol.json'),
        'parent_source': str(source_parent), 'parent_source_count': len(original),
        'parent_hashes_sha256': sha(out / 'parent-source-hashes.json.gz'),
        'main_source_sha256': guard, 'pending_native_sha256': pending, 'variants': variants,
        'manifests_and_lockfiles_unchanged': True, 'baseline_and_direct_sources_byte_identical_to_prior': True,
        'outlined_arithmetic_suffix_recovered': True, 'original_general_fallback_recovered': True,
        'shared_test_functions': 28, 'all_shared_tests_byte_identical_between_variants': True,
        'original_limits': prior['original_limits'], 'runtime_admitted': False, 'plan_complete': False}
    verify_inputs()
    assert sources() == guard and all(sha(ROOT / p) == h for p, h in pending.items())
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    print(json.dumps({'source_counts': {v: row['source_files'] for v, row in variants.items()},
        'original_fallback_and_arithmetic_recovered': True, 'shared_tests': 28}))


if __name__ == '__main__':
    try:
        main()
    except Exception:
        failure = E / 'preparation-failure-v1.json'
        assert not failure.exists()
        failure.write_text(json.dumps({'runner_sha256': sha(Path(__file__)),
            'traceback': traceback.format_exc(), 'partial_sources_frozen': True}, indent=2) + '\n')
        raise
