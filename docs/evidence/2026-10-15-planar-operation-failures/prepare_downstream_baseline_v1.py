"""Reconstruct the starting revision plus the unchanged 35 pending native files.

Excluded comparisons are not a dependency or workspace member for Hypercircuit.
No live source or expected snapshot is modified. The isolated source is retained.
"""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
DEST = ROOT / 'target/planar-operation-failures/downstream-baseline-source-v1'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert not DEST.exists()
    DEST.mkdir()
    prior = ROOT / 'docs/evidence/2026-10-15-sampled-profile-failures/closure-v1.json'
    pending = json.loads(prior.read_text())['pending_native_sha256']
    assert all(sha(ROOT / name) == digest for name, digest in pending.items())
    base = '227792db1'
    names = subprocess.check_output([
        'git', 'ls-tree', '-r', '--name-only', base, '--', 'src', 'crates', 'tests',
        'examples', 'benchmarks/rust', 'Cargo.toml', 'Cargo.lock', 'readme.md',
        '.cargo', 'build.rs', 'assets', 'docs/csg.png', 'docs/rounded_rectangle.png',
    ], cwd=ROOT, text=True).splitlines()
    names = sorted(set(name for name in names if not name.startswith('crates/comparisons/')) | set(pending))
    changed = set(subprocess.check_output(['git', 'diff', '--name-only', base], cwd=ROOT, text=True).splitlines())
    hashes = {}
    restored = []
    for name in names:
        path = DEST / name
        path.parent.mkdir(parents=True, exist_ok=True)
        if name in changed and name not in pending:
            data = subprocess.check_output(['git', 'show', base + ':' + name], cwd=ROOT)
            restored.append(name)
        else:
            data = (ROOT / name).read_bytes()
        path.write_bytes(data)
        hashes[name] = sha(path)
    assert all(sha(DEST / name) == digest for name, digest in pending.items())
    with gzip.open(E / 'downstream-baseline-source-v1.json.gz', 'wt') as stream:
        json.dump(hashes, stream, sort_keys=True)
    (E / 'downstream-baseline-source-v1.json').write_text(json.dumps({
        'base_commit': base, 'source_directory': str(DEST),
        'runner_sha256': sha(Path(__file__)), 'pending_native_sha256': pending,
        'restored_to_base': restored, 'source_count': len(hashes),
        'source_manifest_sha256': sha(E / 'downstream-baseline-source-v1.json.gz'),
        'scope': 'Original tracked build, fixture and runtime inputs plus unchanged pending native files; no planar-operation implementation edits. Excluded comparisons workspace is not part of this dependency graph.',
    }, indent=2) + '\n')
    print(json.dumps({'baseline_files': len(hashes), 'restored_to_base': restored}))


if __name__ == '__main__':
    main()
