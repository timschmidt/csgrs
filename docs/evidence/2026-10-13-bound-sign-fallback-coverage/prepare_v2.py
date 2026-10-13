"""Freeze existing-bound-sign counters at the unchanged quadratic fallback."""
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
PARENT = ROOT / 'docs/evidence/2026-10-13-native-tower-sign-retention'
WORK = ROOT / 'target/bound-sign-fallback-coverage-v2'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    parent = json.loads((PARENT / 'preparation-v1/protocol.json').read_text())
    scope = json.loads((E / 'scope-v1.json').read_text())
    assert scope['parent_preparation_sha256'] == sha(PARENT / 'preparation-v1/protocol.json')
    original = Path(parent['source'])
    expected = json.loads(gzip.decompress((PARENT / 'preparation-v1/source-hashes.json.gz').read_bytes()))
    guard, pending = sources(), parent['pending_native_sha256']
    assert len(expected) == 10131 and len(guard) == 1134 and len(pending) == 35
    assert guard == parent['main_source_sha256']
    assert all(sha(original / p) == h for p, h in expected.items())
    assert all(sha(ROOT / p) == h for p, h in pending.items())
    profile = ROOT / 'docs/evidence/2026-10-13-native-normal-form-attribution'
    assert sha(profile / 'decision-v1.json') == scope['profile_decision_sha256']
    assert sha(profile / 'next-diagnostic-v1.json') == scope['next_diagnostic_sha256']
    WORK.mkdir(exist_ok=False)
    out = E / 'preparation-v2'
    out.mkdir(exist_ok=False)
    source = WORK / 'source'
    source.mkdir()
    for name in expected:
        destination = source / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(original / name, destination)
    helper = 'crates/hyperreal/src/bound_sign_diagnostic.rs'
    tests = 'crates/hyperreal/src/computable/node/bound_sign_site_tests.rs'
    reference_tests = 'crates/hyperreal/src/computable/node/bound_sign_reference_tests.rs'
    reference_source = WORK / 'reference-source'
    shutil.copytree(original, reference_source)
    edits_by_copy = json.loads((E / 'runtime-edits-v1.json').read_text())
    reference_edits = edits_by_copy['reference']
    edits = edits_by_copy['diagnostic']
    for directory, replacements, added in [(reference_source, reference_edits, [reference_tests]), (source, edits, [helper, tests, reference_tests])]:
        for name in added:
            shutil.copyfile(E / ('bound_sign_reference_tests_v2.rs' if name == reference_tests else Path(name).name), directory / name)
        for name, rows in replacements.items():
            text = (directory / name).read_text()
            for row in rows:
                assert text.count(row['before']) == 1, name
                text = text.replace(row['before'], row['after'])
            (directory / name).write_text(text)
            for row in reversed(rows):
                assert text.count(row['after']) == 1, name
                text = text.replace(row['after'], row['before'])
            assert text == (original / name).read_text(), name
    with (out / 'format.log').open('w') as log:
        subprocess.run(['cargo', 'fmt', '--all', '--', '--check'], cwd=source,
                       stdout=log, stderr=subprocess.STDOUT, check=True)
        subprocess.run(['cargo', 'fmt', '--all', '--', '--check'], cwd=reference_source,
                       stdout=log, stderr=subprocess.STDOUT, check=True)
        subprocess.run(['rustfmt', '--edition', '2024', '--check',
                        *[str(source / name) for name in [helper, tests, reference_tests]]],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    current = {p: sha(source / p) for p in sorted(set(expected) | {helper, tests, reference_tests})}
    changes = {p: h for p, h in current.items() if expected.get(p) != h}
    assert set(changes) == set(edits) | {helper, tests, reference_tests}
    reference_hashes = {p: sha(reference_source / p) for p in sorted(set(expected) | {reference_tests})}
    assert len(current) == 10134 and len(reference_hashes) == 10132
    for label, data in [('parent-source', expected), ('source', current), ('reference-source', reference_hashes)]:
        (out / (label + '-hashes.json.gz')).write_bytes(
            gzip.compress((json.dumps(data, sort_keys=True) + '\n').encode(), mtime=0))
    with tarfile.open(out / 'source-delta.tar.gz', 'w:gz') as archive:
        for name in changes:
            archive.add(source / name, arcname=name)
    patch = ''.join(''.join(difflib.unified_diff(
        (original / p).read_text().splitlines(True) if p in expected else [],
        (source / p).read_text().splitlines(True), 'a/' + p, 'b/' + p)) for p in changes)
    (out / 'source.patch').write_text(patch)
    inputs = ['draft-format-v1.json', 'scope-v1.json', 'runtime-edits-v1.json', 'bound_sign_diagnostic.rs', 'bound_sign_reference_tests_v2.rs', 'bound_sign_site_tests.rs', 'diagnose_v2.py', 'assess_v2.py', 'preparation-failure-v1.json']
    protocol = {
        'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
        'inputs_sha256': {name: sha(E / name) for name in inputs},
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'parent_preparation_sha256': sha(PARENT / 'preparation-v1/protocol.json'),
        'parent_source': str(original), 'reference_source': str(reference_source), 'reference_target': str(WORK / 'reference-target'), 'reference_hashes_sha256': sha(out / 'reference-source-hashes.json.gz'), 'source': str(source), 'target': str(WORK / 'target'),
        'parent_files': len(expected), 'diagnostic_files': len(current),
        'main_source_sha256': guard, 'pending_native_sha256': pending,
        'source_changes_sha256': changes,
        'parent_hashes_sha256': sha(out / 'parent-source-hashes.json.gz'),
        'source_hashes_sha256': sha(out / 'source-hashes.json.gz'),
        'archive_sha256': sha(out / 'source-delta.tar.gz'),
        'patch_sha256': sha(out / 'source.patch'),
        'exact_original_bytes_recovered_from_explicit_replacements': True,
        'frozen_sources_checked_for_formatting_without_editing': True, 'reference_changes_from_v1': 'Only shared-test formatting under the crate configuration; no runtime or measurement change.',
        'runtime_admitted': False, 'plan_complete': False,
    }
    assert sources() == guard
    assert all(sha(ROOT / p) == h for p, h in pending.items())
    assert all(sha(original / p) == h for p, h in expected.items())
    assert all(sha(source / p) == h for p, h in current.items())
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    print(json.dumps({'source_files': len(current), 'changed_files': len(changes),
                      'original_bytes_recovered': True, 'parent_and_main_unchanged': True}))


if __name__ == '__main__':
    main()
