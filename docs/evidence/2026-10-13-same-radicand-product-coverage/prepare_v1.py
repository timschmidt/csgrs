"""Freeze counters at unchanged single-radical multiplication operations."""
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
WORK = ROOT / 'target/same-radicand-product-coverage'
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
    for row in scope['source_review']:
        assert sha(ROOT / row['path']) == row['sha256']
    prior = ROOT / 'docs/evidence/2026-10-13-cache-local-polynomial-products/decision-v1.json'
    assert sha(prior) == scope['rejected_cache_local_decision_sha256']
    WORK.mkdir(exist_ok=False)
    out = E / 'preparation-v1'
    out.mkdir(exist_ok=False)
    source = WORK / 'source'
    source.mkdir()
    for name in expected:
        destination = source / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(original / name, destination)
    helper = 'crates/hyperreal/src/same_radicand_diagnostic.rs'
    tests = 'crates/hyperreal/src/computable/node/same_radicand_site_tests.rs'
    for name in [helper, tests]:
        shutil.copyfile(E / Path(name).name, source / name)
    edits = json.loads((E / 'runtime-edits-v1.json').read_text())
    assert set(edits) == {'crates/hyperreal/src/lib.rs',
                          'crates/hyperreal/src/computable/node/algebra.rs',
                          'crates/hyperdrc/src/app.rs'}
    for name, rows in edits.items():
        text = (source / name).read_text()
        for row in rows:
            assert text.count(row['before']) == 1, name
            text = text.replace(row['before'], row['after'])
        (source / name).write_text(text)
        for row in reversed(rows):
            assert text.count(row['after']) == 1, name
            text = text.replace(row['after'], row['before'])
        assert text == (original / name).read_text(), name
    with (out / 'format.log').open('w') as log:
        subprocess.run(['rustfmt', '--edition', '2024', '--check',
                        *[str(source / name) for name in edits]],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    current = {p: sha(source / p) for p in sorted(set(expected) | {helper, tests})}
    changes = {p: h for p, h in current.items() if expected.get(p) != h}
    assert set(changes) == set(edits) | {helper, tests}
    for label, data in [('parent-source', expected), ('source', current)]:
        (out / (label + '-hashes.json.gz')).write_bytes(
            gzip.compress((json.dumps(data, sort_keys=True) + '\n').encode(), mtime=0))
    with tarfile.open(out / 'source-delta.tar.gz', 'w:gz') as archive:
        for name in changes:
            archive.add(source / name, arcname=name)
    patch = ''.join(''.join(difflib.unified_diff(
        (original / p).read_text().splitlines(True) if p in expected else [],
        (source / p).read_text().splitlines(True), 'a/' + p, 'b/' + p)) for p in changes)
    (out / 'source.patch').write_text(patch)
    inputs = ['scope-v1.json', 'runtime-edits-v1.json', 'draft-format-review-v1.json', 'same_radicand_diagnostic.rs',
              'same_radicand_site_tests.rs', 'diagnose_v1.py', 'assess_v1.py']
    protocol = {
        'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
        'inputs_sha256': {name: sha(E / name) for name in inputs},
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'parent_preparation_sha256': sha(PARENT / 'preparation-v1/protocol.json'),
        'parent_source': str(original), 'source': str(source), 'target': str(WORK / 'target'),
        'parent_files': len(expected), 'diagnostic_files': len(current),
        'main_source_sha256': guard, 'pending_native_sha256': pending,
        'source_changes_sha256': changes,
        'parent_hashes_sha256': sha(out / 'parent-source-hashes.json.gz'),
        'source_hashes_sha256': sha(out / 'source-hashes.json.gz'),
        'archive_sha256': sha(out / 'source-delta.tar.gz'),
        'patch_sha256': sha(out / 'source.patch'),
        'exact_original_bytes_recovered_from_explicit_replacements': True,
        'frozen_sources_formatted_before_preparation_and_checked_without_editing': True,
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
