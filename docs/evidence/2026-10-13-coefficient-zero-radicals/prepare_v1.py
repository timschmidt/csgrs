"""Freeze an unchanged baseline and a two-guard zero-radical candidate.

Both copies add the same seven tests. The candidate only skips the two radical
linear loops when their opposite rational constant is stored zero. Every other
runtime statement is unchanged. Source and Cargo target trees remain separate.
"""
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PARENT = ROOT / 'docs/evidence/2026-10-12-endpoint-coordinate-reuse'
DECISION = ROOT / 'docs/evidence/2026-10-13-coefficient-zero-contributions/decision-v1.json'
WORK = ROOT / 'target/coefficient-zero-radicals'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    WORK.mkdir(exist_ok=False)
    out = E / 'preparation-v1'
    out.mkdir(exist_ok=False)
    parent = json.loads((PARENT / 'preparation.json').read_text())
    original = json.loads(gzip.decompress((PARENT / 'baseline-source-hashes.json.gz').read_bytes())) | parent['source_changes_sha256']
    source = Path(parent['candidate_source'])
    guard = parent['main_source_sha256']
    pending = json.loads((ROOT / 'docs/evidence/2026-10-12-existing-worktree/geometry-validation-closure/result.json').read_text())['pending_native_sha256']
    assert len(original) == 10130 and len(pending) == 35
    assert sources() == guard
    assert all(sha(ROOT / name) == digest for name, digest in pending.items())
    assert all(sha(source / name) == digest for name, digest in original.items())
    assert json.loads(DECISION.read_text())['decision'] == 'advance-isolated-zero-radical-contribution-guard-experiment'
    (out / 'parent-source-hashes.json.gz').write_bytes(gzip.compress((json.dumps(original, sort_keys=True) + '\n').encode(), mtime=0))
    algebra = 'crates/hyperreal/src/computable/node/algebra.rs'
    test = 'crates/hyperreal/src/computable/node/coefficient_zero_radical_tests.rs'
    prefix = '#[cfg(test)]\nmod coefficient_zero_radical_tests;\n\n'
    original_text = (source / algebra).read_text()
    variants = {}
    for variant in ['baseline', 'candidate']:
        directory = WORK / variant
        directory.mkdir()
        current_source = directory / 'source'
        with (out / (variant + '-copy.log')).open('w') as log:
            subprocess.run(['cp', '-a', '--reflink=auto', str(source), str(current_source)], stdout=log, stderr=subprocess.STDOUT, check=True)
        shutil.copyfile(E / 'coefficient_zero_radical_tests.rs', current_source / test)
        body = prefix + original_text
        replacements = []
        if variant == 'candidate':
            for vector, scale in [('self', 'other'), ('other', 'self')]:
                before = (f'        for (radicand, coefficient) in &{vector}.radicals {{\n'
                          f'            result.insert_radical(radicand.clone(), coefficient * &{scale}.rational)?;\n'
                          '        }')
                after = (f'        if !{scale}.rational.is_zero() {{\n'
                         + '\n'.join('    ' + line for line in before.splitlines()) + '\n        }')
                assert body.count(before) == 1
                body = body.replace(before, after)
                replacements.append((before, after))
        (current_source / algebra).write_text(body)
        with (out / (variant + '-format.log')).open('w') as log:
            subprocess.run(['rustfmt', '--edition', '2024', str(current_source / algebra)], stdout=log, stderr=subprocess.STDOUT, check=True)
        recovered = (current_source / algebra).read_text().removeprefix(prefix)
        for before, after in replacements:
            assert recovered.count(after) == 1
            recovered = recovered.replace(after, before)
        assert recovered == original_text
        current = {name: sha(current_source / name) for name in sorted(set(original) | {test})}
        changes = {name: digest for name, digest in current.items() if original.get(name) != digest}
        assert set(changes) == {algebra, test}
        with tarfile.open(out / (variant + '-delta.tar.gz'), 'w:gz') as archive:
            for name in changes:
                archive.add(current_source / name, arcname=name)
        patch = ''.join(''.join(difflib.unified_diff(
            (source / name).read_text().splitlines(True) if name in original else [],
            (current_source / name).read_text().splitlines(True), 'a/' + name, 'b/' + name)) for name in changes)
        (out / (variant + '.patch')).write_text(patch)
        variants[variant] = {'source': str(current_source), 'target': str(directory / 'qualification-target'),
                             'cost_target': str(directory / 'cost-target'), 'source_changes_sha256': changes,
                             'delta_archive_sha256': sha(out / (variant + '-delta.tar.gz')),
                             'patch_sha256': sha(out / (variant + '.patch'))}
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
                'test_input_sha256': sha(E / 'coefficient_zero_radical_tests.rs'),
                'validation_runner_sha256': sha(E / 'validate_v1.py'),
                'validation_assessor_sha256': sha(E / 'assess_validation_v1.py'),
                'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                'zero_contribution_decision_sha256': sha(DECISION), 'parent_preparation_sha256': sha(PARENT / 'preparation.json'),
                'parent_source': str(source), 'parent_source_count': len(original),
                'main_source_sha256': guard, 'pending_native_sha256': pending, 'variants': variants,
                'qualification_order': ['baseline', 'candidate'], 'new_regression_functions': 7,
                'original_body_recovered_after_removing_only_two_guards_and_test_module': True,
                'runtime_admitted': False}
    assert sources() == guard
    assert all(sha(ROOT / name) == digest for name, digest in pending.items())
    assert all(sha(source / name) == digest for name, digest in original.items())
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    print(json.dumps({'variants': list(variants), 'changed_files_per_variant': 2,
                      'original_body_recovered_after_removing_two_guards': True, 'main_unchanged': True}))


if __name__ == '__main__':
    main()
