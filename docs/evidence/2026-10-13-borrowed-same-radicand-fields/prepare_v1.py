"""Freeze fresh baseline, outlined and borrowed product sources with identical controls."""
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
PRIOR = ROOT / 'docs/evidence/2026-10-13-outlined-same-radicand-products'
WORK = ROOT / 'target/borrowed-same-radicand-fields'
NODE = 'crates/hyperreal/src/computable/node/'
ALGEBRA = NODE + 'algebra.rs'
TEST_NAMES = ['temporary_storage_tests.rs', 'memo_publication_tests.rs',
              'shared_radical_tests.rs', 'projection_tests.rs', 'same_radicand_product_tests.rs']
VARIANTS = ['baseline', 'outlined', 'borrowed']
INPUTS = TEST_NAMES + ['algebra_' + v + '.rs' for v in VARIANTS] + [
    'runtime-edits-v1.json', 'draft-format-v1.json', 'measure_v1.py', 'assess_cost_v1.py', 'inspect_codegen_v1.py']
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
    prior = load(PRIOR / 'preparation-v1/protocol.json')
    spec = importlib.util.spec_from_file_location('_prior_outlined_qualification', PRIOR / 'qualify_v1.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.verify(prior)
    assert load(PRIOR / 'correctness-v1.json')['all_42_commands_passed']
    assert load(PRIOR / 'decision-v1.json')['status'] == 'reject-outlined-same-radicand-products-on-unresolved-scalar-costs'
    for variant in ['baseline', 'outlined']:
        source = Path(prior['variants'][variant]['source'])
        assert (E / ('algebra_' + variant + '.rs')).read_bytes() == (source / ALGEBRA).read_bytes()
        for name in TEST_NAMES:
            assert sha(E / name) == sha(source / NODE / name)
    assert sum((E / n).read_text().count('#[test]') for n in TEST_NAMES) == 28
    edits = load(E / 'runtime-edits-v1.json')
    before, control, after = [(E / ('algebra_' + v + '.rs')).read_text() for v in VARIANTS]
    assert control.count(edits['outlined_helper'] + edits['outlined_method']) == 1
    assert after.count(edits['borrowed_helper'] + edits['borrowed_method']) == 1
    assert control.replace(edits['outlined_helper'] + edits['outlined_method'], edits['original_method']) == before
    assert after.replace(edits['borrowed_helper'] + edits['borrowed_method'], edits['outlined_helper'] + edits['outlined_method']) == control
    assert edits['borrowed_method'].replace(edits['borrowed_branch'], '').replace('fn multiply(mut self,', 'fn multiply(self,', 1) == edits['original_method']
    assert edits['outlined_method'].replace(edits['outlined_branch'], '') == edits['original_method']
    assert after.count('multiply_same_radicand_fields') == 2
    assert 'radicals[' not in edits['borrowed_helper'] and 'unsafe' not in edits['borrowed_helper']
    prior_edits = load(PRIOR / 'runtime-edits-v1.json')
    assert edits['outlined_helper'] == prior_edits['helper']
    original = '        let rational =' + edits['outlined_helper'].split('        let rational =', 1)[1].rsplit('    }\n\n', 1)[0]
    assert original == edits['outlined_arithmetic_suffix']
    transformed = original
    for item in edits['arithmetic_substitutions']:
        assert transformed.count(item['before']) == item['count']
        transformed = transformed.replace(item['before'], item['after'])
    assert transformed == edits['borrowed_arithmetic_suffix']
    actual = '        let rational =' + edits['borrowed_helper'].split('        let rational =', 1)[1].rsplit('    }\n\n', 1)[0]
    assert ''.join(actual.split()) == ''.join(transformed.split())
    return prior

def main():
    prior = verify_inputs()
    source_parent = Path(prior['variants']['baseline']['source'])
    parent_hash_path = PRIOR / 'preparation-v1/baseline-source-hashes.json.gz'
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
        if variant != 'borrowed':
            previous_variant = variant
            assert current == json.loads(gzip.decompress((PRIOR / 'preparation-v1' / (previous_variant + '-source-hashes.json.gz')).read_bytes()))
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
    assert {p for p in original if maps['outlined'][p] != maps['borrowed'][p]} == {ALGEBRA}
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
        'source_review_sha256': sha(E / 'scope-v1.json'),
        'qualification_runner_sha256': sha(E / 'qualify_v1.py'),
        'qualification_assessor_sha256': sha(E / 'assess_qualification_v1.py'),
        'input_sha256': {name: sha(E / name) for name in INPUTS},
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'prior_preparation_sha256': sha(PRIOR / 'preparation-v1/protocol.json'),
        'parent_source': str(source_parent), 'parent_source_count': len(original),
        'parent_hashes_sha256': sha(out / 'parent-source-hashes.json.gz'),
        'main_source_sha256': guard, 'pending_native_sha256': pending, 'variants': variants,
        'manifests_and_lockfiles_unchanged': True, 'baseline_and_outlined_sources_byte_identical_to_prior': True,
        'borrowed_arithmetic_suffix_verified': True, 'original_general_fallback_recovered': True,
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
