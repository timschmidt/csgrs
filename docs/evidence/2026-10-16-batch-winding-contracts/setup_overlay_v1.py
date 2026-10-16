"""Create a private copy-on-write validation target over the preserved build."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
LOWER = E.parents[2] / 'target/properties/minkowski-contracts/benchmark-repair-v1'
PHASE = sys.argv[1]
TARGET = E.parents[2] / 'target/properties/batch-winding-contracts' / PHASE
STORAGE = Path('/dev/shm/csgrs-batch-winding-contracts') / PHASE
OUT = E / PHASE


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


assert not TARGET.exists() and not STORAGE.exists()
assert os.statvfs(STORAGE.parent.parent).f_bavail * os.statvfs(STORAGE.parent.parent).f_frsize > 1024**3
lower_hashes = {}
for path in LOWER.rglob('*'):
    assert not path.is_symlink(), path
    if path.is_file():
        lower_hashes[str(path)] = sha(path)
with gzip.open(OUT / 'overlay-lower-sha256.json.gz', 'wt') as stream:
    json.dump(lower_hashes, stream, sort_keys=True)
TARGET.mkdir(parents=True)
upper, work = STORAGE / 'upper', STORAGE / 'work'
upper.mkdir(parents=True)
work.mkdir()
command = ['fuse-overlayfs', '-o', f'lowerdir={LOWER},upperdir={upper},workdir={work}', str(TARGET)]
subprocess.run(command, check=True)
assert os.path.ismount(TARGET)
# Writing an identical byte must copy up this existing binary, preserving both
# its bytes and the original lower file's inode/mtime. It is not a new test run.
relative = 'boolean-failures-c'
original = LOWER / relative
before = original.stat()
with (TARGET / relative).open('r+b') as stream:
    byte = stream.read(1)
    stream.seek(0)
    stream.write(byte)
    stream.flush()
    os.fsync(stream.fileno())
assert (upper / relative).is_file()
assert sha(upper / relative) == sha(original) == lower_hashes[str(original)]
assert original.stat().st_mtime_ns == before.st_mtime_ns
assert (upper / relative).stat().st_ino != before.st_ino
record = {
    'lower': str(LOWER), 'upper': str(upper), 'work': str(work), 'target': str(TARGET),
    'command': command, 'lower_files': len(lower_hashes),
    'lower_hashes_sha256': sha(OUT / 'overlay-lower-sha256.json.gz'),
    'copy_up_probe': relative, 'copy_up_probe_sha256': sha(original),
    'fuse_overlayfs_sha256': sha(Path('/usr/bin/fuse-overlayfs')),
    'version': subprocess.check_output(['fuse-overlayfs', '--version'], text=True),
    'quota': subprocess.check_output(['quota', '-s'], text=True),
    'storage': subprocess.check_output(['df', '-h', str(STORAGE), str(TARGET)], text=True),
    'scope': 'The lower build is read-only through the overlay. New and changed artifacts go to a private upper directory. All lower hashes are checked again at closure. No prior target or failed partial copy is removed.',
}
(OUT / 'overlay.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({'overlay_ready': str(TARGET), 'lower_files': len(lower_hashes),
                  'copy_up_verified': True}), flush=True)
