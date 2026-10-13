"""Freeze baseline, right-scalar and bidirectional coefficient-scaling sources.

The baseline adds the same test module as both candidates; its runtime body is
unchanged. Every tree has its own qualification and cost Cargo targets. Neither
diagnostic counters nor rejected merge/prepass code is inherited.
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
DECISION = ROOT / 'docs/evidence/2026-10-13-coefficient-multiplication-shapes/decision-v1.json'
WORK = ROOT / 'target/coefficient-scaling'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def replace(text, before, after):
    assert text.count(before) == 1, before
    return text.replace(before, after)


def main():
    WORK.mkdir(exist_ok=False)
    out = E / 'preparation-v1'
    out.mkdir(exist_ok=False)
    parent = json.loads((PARENT / 'preparation.json').read_text())
    original = json.loads(gzip.decompress((PARENT / 'baseline-source-hashes.json.gz').read_bytes())) | parent['source_changes_sha256']
    source = Path(parent['candidate_source'])
    guard = parent['main_source_sha256']
    pending = json.loads((ROOT / 'docs/evidence/2026-10-12-existing-worktree/geometry-validation-closure/result.json').read_text())['pending_native_sha256']
    assert len(original) == 10130
    assert sources() == guard
    assert all(sha(ROOT / name) == digest for name, digest in pending.items())
    assert all(sha(source / name) == digest for name, digest in original.items())
    assert json.loads(DECISION.read_text())['decision'] == 'advance-two-isolated-scaling-variants-with-separate-cost-results'
    (out / 'parent-source-hashes.json.gz').write_bytes(gzip.compress((json.dumps(original, sort_keys=True) + '\n').encode(), mtime=0))
    algebra = 'crates/hyperreal/src/computable/node/algebra.rs'
    test = 'crates/hyperreal/src/computable/node/coefficient_scale_tests.rs'
    variants = {}
    for variant in ['baseline', 'right', 'both']:
        directory = WORK / variant
        directory.mkdir()
        current_source = directory / 'source'
        with (out / (variant + '-copy.log')).open('w') as log:
            subprocess.run(['cp', '-a', '--reflink=auto', str(source), str(current_source)], stdout=log, stderr=subprocess.STDOUT, check=True)
        shutil.copyfile(E / 'coefficient_scale_tests.rs', current_source / test)
        body = (current_source / algebra).read_text()
        body = '#[cfg(test)]\nmod coefficient_scale_tests;\n\n' + body
        signature = '    fn multiply(self, other: &Self) -> Option<Self> {'
        if variant != 'baseline':
            optimized = '''    fn multiply(mut self, other: &Self) -> Option<Self> {
        if other.radicals.is_empty() && other.atoms.is_empty() {
            self.scale_in_place(&other.rational);
            return Some(self);
        }
'''
            if variant == 'both':
                optimized += '''        if self.radicals.is_empty() && self.atoms.is_empty() {
            let mut scaled = other.clone();
            scaled.scale_in_place(&self.rational);
            return Some(scaled);
        }
'''
            body = replace(body, signature + '\n', optimized)
        (current_source / algebra).write_text(body)
        with (out / (variant + '-format.log')).open('w') as log:
            subprocess.run(['rustfmt', '--edition', '2024', str(current_source / algebra)], stdout=log, stderr=subprocess.STDOUT, check=True)
        current = {name: sha(current_source / name) for name in sorted(set(original) | {test})}
        changes = {name: digest for name, digest in current.items() if original.get(name) != digest}
        assert set(changes) == {algebra, test}
        if variant == 'baseline':
            baseline_text = (current_source / algebra).read_text()
            assert baseline_text.removeprefix('#[cfg(test)]\nmod coefficient_scale_tests;\n\n') == (source / algebra).read_text()
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
                'test_input_sha256': sha(E / 'coefficient_scale_tests.rs'),
                'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                'shape_decision_sha256': sha(DECISION), 'parent_preparation_sha256': sha(PARENT / 'preparation.json'),
                'parent_source': str(source), 'parent_source_count': len(original),
                'main_source_sha256': guard, 'pending_native_sha256': pending, 'variants': variants,
                'qualification_order': ['baseline', 'right', 'both'],
                'new_regression_functions': 4, 'runtime_admitted': False}
    assert sources() == guard
    assert all(sha(ROOT / name) == digest for name, digest in pending.items())
    assert all(sha(source / name) == digest for name, digest in original.items())
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    print(json.dumps({'variants': list(variants), 'changed_files_per_variant': 2,
                      'baseline_runtime_body_unchanged': True, 'main_unchanged': True}))


if __name__ == '__main__':
    main()
