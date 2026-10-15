"""Share identical closed, non-executable libraries; retain paths and bytes.

This does not resume a target, remove a unique artifact, or restore a lost one.
The read-only plan pins every candidate to the earlier preservation manifest.
"""
import hashlib
import json
import os
from pathlib import Path
import stat
import time

E = Path(__file__).resolve().parent
PLAN = E / 'closed-artifact-consolidation-plan-v1.json'
OUT = E / 'closed-artifact-consolidation-v1.json'
JOURNAL = E / 'closed-artifact-consolidation-v1.jsonl'
assert not OUT.exists() and not JOURNAL.exists()
plan = json.loads(PLAN.read_text())


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        while block := stream.read(1 << 20):
            digest.update(block)
    return digest.hexdigest()


def inspect(path, group, inode):
    assert str(path).startswith('/run/user/1000/csgrs-')
    assert path.suffix in {'.rlib', '.rmeta'}
    assert not any(part.is_symlink() for part in [path, *path.parents])
    info = path.stat()
    assert stat.S_ISREG(info.st_mode) and not info.st_mode & 0o111
    assert (info.st_dev, info.st_ino, info.st_mode, info.st_uid, info.st_gid) == (
        group['device'], inode, group['mode'], group['uid'], group['gid'])
    assert sha(path) == group['sha256'], path
    return {'inode': info.st_ino, 'size': info.st_size, 'mtime_ns': info.st_mtime_ns,
            'nlink': info.st_nlink, 'device': info.st_dev, 'mode': info.st_mode,
            'uid': info.st_uid, 'gid': info.st_gid}


# Verify every input before the first replacement, including the original
# checkpoint's digest. These completed targets are never reused for builds.
checkpoint = json.loads((E.parent / '2026-10-15-quotient-factor-proofs/checkpoint-v1.json').read_text())
pinned = checkpoint['preserved_artifacts_sha256']
inputs = {}
for group in plan['groups']:
    entries = [{'path': group['source'], 'inode': group['source_inode']}, *group['destinations']]
    for entry in entries:
        path = Path(entry['path'])
        assert pinned[str(path)] == group['sha256']
        assert str(path) not in inputs
        inputs[str(path)] = inspect(path, group, entry['inode'])

before = os.statvfs('/run/user/1000')
started = time.time()
replaced = 0
with JOURNAL.open('x') as journal:
    for group in plan['groups']:
        source = Path(group['source'])
        for entry in group['destinations']:
            destination = Path(entry['path'])
            # Recheck the identity and bytes immediately before replacement.
            old = inspect(destination, group, entry['inode'])
            inspect(source, group, group['source_inode'])
            temporary = destination.with_name(destination.name + '.consolidation-v1')
            assert not temporary.exists()
            os.link(source, temporary, follow_symlinks=False)
            os.replace(temporary, destination)
            new = inspect(destination, group, group['source_inode'])
            journal.write(json.dumps({'path': str(destination), 'source': str(source),
                'sha256': group['sha256'], 'before': old, 'after': new}) + '\n')
            journal.flush()
            os.fsync(journal.fileno())
            replaced += 1

verified = {}
for group in plan['groups']:
    for name in [group['source'], *(entry['path'] for entry in group['destinations'])]:
        inspect(Path(name), group, group['source_inode'])
        verified[name] = group['sha256']
after = os.statvfs('/run/user/1000')
OUT.write_text(json.dumps({'scope': plan['scope'], 'started_unix': started,
    'finished_unix': time.time(), 'plan_sha256': sha(PLAN),
    'runner_sha256': sha(Path(__file__)), 'journal_sha256': sha(JOURNAL),
    'replaced_paths': replaced, 'verified_paths_sha256': verified,
    'all_input_metadata': inputs,
    'free_bytes_before': before.f_bavail * before.f_frsize,
    'free_bytes_after': after.f_bavail * after.f_frsize,
    'unique_artifacts_deleted': 0, 'executables_changed': 0,
    'historical_missing_artifacts_restored': 0}, indent=2) + '\n')
print(json.dumps({'replaced_paths': replaced, 'verified_paths': len(verified),
    'available_bytes': after.f_bavail * after.f_frsize}))
