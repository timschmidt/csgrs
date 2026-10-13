"""Freeze a matched Vec baseline and three private temporary-storage variants.

All trees inherit the qualified endpoint/P4 combination, not instrumentation or
rejected coefficient experiments. Only temporary radical/term vectors change;
persistent certificates and opaque atom vectors retain their Vec storage.
"""
import copy
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tomllib

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PARENT = ROOT / 'docs/evidence/2026-10-13-native-tower-sign-retention'
DECISION = ROOT / 'docs/evidence/2026-10-13-retained-quadratic-projection/decision-v1.json'
WORK = ROOT / 'target/temporary-laurent-storage'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def change(body, before, after, edits):
    assert body.count(before) == 1, before
    edits.append({'before': before, 'after': after})
    return body.replace(before, after)


def main():
    parent = json.loads((PARENT / 'preparation-v1/protocol.json').read_text())
    source = Path(parent['source'])
    original = json.loads(gzip.decompress((PARENT / 'preparation-v1/source-hashes.json.gz').read_bytes()))
    guard, pending = sources(), parent['pending_native_sha256']
    assert len(original) == 10131 and len(guard) == 1134 and len(pending) == 35
    assert guard == parent['main_source_sha256']
    assert all(sha(source / p) == h for p, h in original.items())
    assert all(sha(ROOT / p) == h for p, h in pending.items())
    decision = json.loads(DECISION.read_text())
    assert decision['next_experiment']['parent_source'] == str(source)
    assert decision['next_experiment']['parent_preparation_sha256'] == sha(PARENT / 'preparation-v1/protocol.json')
    lock = tomllib.loads((source / 'Cargo.lock').read_text())
    smallvec = [p for p in lock['package'] if p['name'] == 'smallvec']
    assert len(smallvec) == 1 and smallvec[0]['version'] == '1.15.1'
    assert smallvec[0]['checksum'] == decision['next_experiment']['dependency']['registry_checksum']
    expected_lock = copy.deepcopy(lock)
    packages = [p for p in expected_lock['package'] if p['name'] == 'hyperreal']
    assert len(packages) == 1 and 'smallvec' not in packages[0]['dependencies']
    packages[0]['dependencies'].append('smallvec')
    WORK.mkdir(exist_ok=False)
    out = E / 'preparation-v1'
    out.mkdir(exist_ok=False)
    (out / 'parent-source-hashes.json.gz').write_bytes(gzip.compress((json.dumps(original, sort_keys=True) + '\n').encode(), mtime=0))
    algebra = 'crates/hyperreal/src/computable/node/algebra.rs'
    test = 'crates/hyperreal/src/computable/node/temporary_storage_tests.rs'
    manifest = 'crates/hyperreal/Cargo.toml'
    variants = {}
    for variant in ['baseline', 'radicals', 'terms', 'both']:
        directory = WORK / variant
        current_source = directory / 'source'
        current_source.mkdir(parents=True)
        for name in original:
            destination = current_source / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source / name, destination)
        shutil.copyfile(E / 'temporary_storage_tests.rs', current_source / test)
        manifest_body = (source / manifest).read_text()
        assert manifest_body.count('[dependencies]\n') == 1
        (current_source / manifest).write_text(manifest_body.replace('[dependencies]\n', '[dependencies]\nsmallvec = { version = "1.15.1", default-features = false }\n'))
        lock_body = (source / 'Cargo.lock').read_text()
        package_start = lock_body.index('[[package]]\nname = "hyperreal"\n')
        package_end = lock_body.index('\n[[package]]', package_start + 1)
        block = lock_body[package_start:package_end]
        assert block.count(' "serde_json",\n') == 1
        lock_body = lock_body[:package_start] + block.replace(' "serde_json",\n', ' "serde_json",\n "smallvec",\n') + lock_body[package_end:]
        assert tomllib.loads(lock_body) == expected_lock
        (current_source / 'Cargo.lock').write_text(lock_body)
        body = (source / algebra).read_text()
        edits = []
        if variant in ['radicals', 'both']:
            body = change(body, 'struct RadicalCoefficient {\n    rational: Rational,\n    radicals: Vec<(Rational, Rational)>,', 'struct RadicalCoefficient {\n    rational: Rational,\n    radicals: smallvec::SmallVec<[(Rational, Rational); 1]>,', edits)
            for before, after in [
                ('rational: value,\n            radicals: Vec::new(),', 'rational: value,\n            radicals: smallvec::SmallVec::new(),'),
                ('radicals: vec![(radicand, value)],', 'radicals: smallvec::SmallVec::from_buf([(radicand, value)]),'),
                ('rational: Rational::zero(),\n            radicals: Vec::new(),', 'rational: Rational::zero(),\n            radicals: smallvec::SmallVec::new(),'),
                ('radicals: vec![(radicand.clone(), -coefficient * inverse_norm)],', 'radicals: smallvec::SmallVec::from_buf([(\n                        radicand.clone(),\n                        -coefficient * inverse_norm,\n                    )]),'),
                ('ClosedRadicalCoefficient {\n                            rational: coefficient.rational.clone(),\n                            radicals: coefficient.radicals.clone(),', 'ClosedRadicalCoefficient {\n                            rational: coefficient.rational.clone(),\n                            radicals: coefficient.radicals.to_vec(),'),
                ('RadicalCoefficient {\n                            rational: coefficient.rational.clone(),\n                            radicals: coefficient.radicals.clone(),\n                            atoms:', 'RadicalCoefficient {\n                            rational: coefficient.rational.clone(),\n                            radicals: coefficient.radicals.iter().cloned().collect(),\n                            atoms:'),
            ]:
                body = change(body, before, after, edits)
        if variant in ['terms', 'both']:
            body = change(body, 'struct PiLaurentPolynomial {\n    terms: Vec<(i16, RadicalCoefficient)>,', 'struct PiLaurentPolynomial {\n    terms: smallvec::SmallVec<[(i16, RadicalCoefficient); 1]>,', edits)
            body = change(body, 'let mut product = Self { terms: Vec::new() };', 'let mut product = Self {\n            terms: smallvec::SmallVec::new(),\n        };', edits)
        prefix = '#[cfg(test)]\nmod temporary_storage_tests;\n\n'
        (current_source / algebra).write_text(prefix + body)
        with (out / (variant + '-format.log')).open('w') as log:
            subprocess.run(['rustfmt', '--edition', '2024', '--config-path', str(source / 'crates/.rustfmt.toml'), str(current_source / algebra)], stdout=log, stderr=subprocess.STDOUT, check=True)
        # Check exact token recovery. Whitespace is immaterial here; no optional
        # punctuation or operation is allowed to disappear from the parent.
        compact = lambda s: ''.join(s.split())
        recovered = compact((current_source / algebra).read_text()).removeprefix(compact(prefix))
        for edit in reversed(edits):
            before, after = compact(edit['before']), compact(edit['after'])
            assert recovered.count(after) == 1, edit
            recovered = recovered.replace(after, before)
        assert recovered == compact((source / algebra).read_text())
        assert sha(current_source / test) == sha(E / 'temporary_storage_tests.rs')
        current = {p: sha(current_source / p) for p in sorted(set(original) | {test})}
        assert len(current) == 10132
        assert {str(p.relative_to(current_source)) for p in current_source.rglob('*') if p.is_file()} == set(current)
        changes = {p: h for p, h in current.items() if original.get(p) != h}
        assert set(changes) == {algebra, test, manifest, 'Cargo.lock'}
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
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
                'qualification_runner_sha256': sha(E / 'qualify_v1.py'),
                'test_input_sha256': sha(E / 'temporary_storage_tests.rs'),
                'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                'decision_sha256': sha(DECISION), 'parent_preparation_sha256': sha(PARENT / 'preparation-v1/protocol.json'),
                'parent_source': str(source), 'parent_source_count': len(original),
                'parent_hashes_sha256': sha(out / 'parent-source-hashes.json.gz'),
                'main_source_sha256': guard, 'pending_native_sha256': pending, 'variants': variants,
                'locked_package_inventory_unchanged': True, 'smallvec_package': smallvec[0],
                'new_features_requested': [], 'new_regression_functions': 8,
                'original_limits': parent['original_limits'],
                'runtime_admitted': False, 'plan_complete': False}
    assert sources() == guard
    assert all(sha(ROOT / p) == h for p, h in pending.items())
    assert all(sha(source / p) == h for p, h in original.items())
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    print(json.dumps({'variants': list(variants), 'changed_files_per_variant': 4, 'new_tests': 8,
                      'baseline_runtime_body_unchanged': True, 'main_and_parent_unchanged': True}))


if __name__ == '__main__':
    main()
