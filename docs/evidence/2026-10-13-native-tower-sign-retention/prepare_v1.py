"""Freeze the endpoint parent plus exactly the three admitted P4 repair files."""
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
REPAIR = ROOT / 'docs/evidence/2026-10-13-quadratic-tower-sign-retention'
WORK = ROOT / 'target/native-tower-sign-retention'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    parent = json.loads((PARENT / 'preparation.json').read_text())
    original = json.loads(gzip.decompress((PARENT / 'baseline-source-hashes.json.gz').read_bytes())) | parent['source_changes_sha256']
    source = Path(parent['candidate_source'])
    repair = json.loads((REPAIR / 'retention-v1.json').read_text())
    inputs = json.loads((REPAIR / 'next-native-inputs-v1.json').read_text())
    pending = json.loads((ROOT / 'docs/evidence/2026-10-12-existing-worktree/geometry-validation-closure/result.json').read_text())['pending_native_sha256']
    main_guard = sources()
    assert len(original) == 10130 and len(pending) == 35 and len(main_guard) == 1134
    assert repair['runtime_admitted'] and not repair['native_runtime_admitted']
    assert repair['files_sha256'] == inputs['admitted_repair_files_sha256']
    assert sha(PARENT / 'preparation.json') == inputs['parent_preparation_sha256']
    assert inputs['parent_source'] == str(source)
    assert all(sha(ROOT / p) == h for p, h in pending.items())
    assert all(sha(source / p) == h for p, h in original.items())
    assert all(sha(ROOT / p) == h for p, h in repair['files_sha256'].items())
    for p, h in inputs['parent_existing_runtime_files_match_pre_repair_main'].items():
        assert original[p] == h
    assert inputs['new_test_absent_in_parent'] not in original
    WORK.mkdir(exist_ok=False)
    out = E / 'preparation-v1'
    out.mkdir(exist_ok=False)
    current_source = WORK / 'source'
    current_source.mkdir()
    for name in original:
        destination = current_source / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source / name, destination)
    for name in repair['files_sha256']:
        destination = current_source / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / name, destination)
    current = {p: sha(current_source / p) for p in sorted(set(original) | set(repair['files_sha256']))}
    assert len(current) == 10131
    changes = {p: h for p, h in current.items() if original.get(p) != h}
    assert changes == repair['files_sha256']
    assert {str(p.relative_to(current_source)) for p in current_source.rglob('*') if p.is_file()} == set(current)
    for name, data in [('parent-source-hashes.json.gz', original), ('source-hashes.json.gz', current)]:
        (out / name).write_bytes(gzip.compress((json.dumps(data, sort_keys=True) + '\n').encode(), mtime=0))
    with tarfile.open(out / 'source-delta.tar.gz', 'w:gz') as archive:
        for name in changes:
            archive.add(current_source / name, arcname=name)
    patch = ''.join(''.join(difflib.unified_diff(
        (source / p).read_text().splitlines(True) if p in original else [],
        (current_source / p).read_text().splitlines(True), 'a/' + p, 'b/' + p)) for p in changes)
    (out / 'source.patch').write_text(patch)
    protocol = {
        'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
        'qualification_runner_sha256': sha(E / 'qualify_v1.py'),
        'qualification_assessor_sha256': sha(E / 'assess_qualification_v1.py'),
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'parent_preparation_sha256': sha(PARENT / 'preparation.json'),
        'repair_retention_sha256': sha(REPAIR / 'retention-v1.json'),
        'next_native_inputs_sha256': sha(REPAIR / 'next-native-inputs-v1.json'),
        'parent_source': str(source), 'source': str(current_source),
        'target': str(WORK / 'qualification-target'),
        'parent_files': len(original), 'combined_files': len(current),
        'source_changes_sha256': changes,
        'main_source_sha256': main_guard, 'pending_native_sha256': pending,
        'parent_hashes_sha256': sha(out / 'parent-source-hashes.json.gz'),
        'source_hashes_sha256': sha(out / 'source-hashes.json.gz'),
        'delta_archive_sha256': sha(out / 'source-delta.tar.gz'),
        'patch_sha256': sha(out / 'source.patch'),
        'original_limits': inputs['original_limits'],
        'admitted_scalar_repair': True, 'native_runtime_admitted': False,
        'plan_complete': False,
    }
    assert sources() == main_guard
    assert all(sha(ROOT / p) == h for p, h in pending.items())
    assert all(sha(source / p) == h for p, h in original.items())
    assert all(sha(current_source / p) == h for p, h in current.items())
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    print(json.dumps({'combined_files': len(current), 'changed_files': len(changes),
                      'parent_and_main_unchanged': True, 'native_runtime_admitted': False}))


if __name__ == '__main__':
    main()
