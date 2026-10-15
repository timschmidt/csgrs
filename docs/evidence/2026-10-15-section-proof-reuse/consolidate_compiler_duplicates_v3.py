"""Reconcile all closed duplicate-link manifests, retaining the most shared inode.

V2 chose the newer copy by lexical target ordering, reclaimed no bytes and
split older inode relationships. Bytes stayed unchanged. This restores every
recorded alias relationship and consolidates the new closed Cargo seed too.
"""
import gzip
import hashlib
import json
import os
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PRIOR = ROOT / 'docs/evidence/2026-10-15-slice-projection-contract'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / 'compiler-consolidation-v3.json'
    assert not out.exists()
    prior = json.loads((PRIOR / 'closure-v1.json').read_text())
    protected = dict(prior['preserved_artifacts_sha256'])
    for phase in ['baseline-v1', 'baseline-v2', 'candidate-v1', 'cargo-v1']:
        for row in json.loads((E / phase / 'result.json').read_text())['commands']:
            protected.update(row['binaries_sha256'])
    for phase in ['baseline-v1', 'baseline-v2', 'candidate-v1', 'cost-v1']:
        for path in (ROOT / 'target/section-proof-reuse' / phase).iterdir():
            if path.is_file(): protected[str(path.relative_to(ROOT))] = sha(path)
    assert all(sha(ROOT / p) == h for p, h in protected.items())
    manifests = sorted(PRIOR.glob('linked-compiler-duplicates-v*.json.gz')) + sorted(E.glob('linked-compiler-duplicates-v*.json.gz'))
    recorded = {}
    for manifest in manifests:
        for name, row in json.loads(gzip.decompress(manifest.read_bytes())).items():
            for path in [name, row['retained_original']]:
                assert recorded.setdefault(path, row['sha256']) == row['sha256']
    groups = {}
    for name, digest in recorded.items():
        path = Path(name); stat = path.stat()
        assert path.is_relative_to(ROOT / 'target') and not path.is_symlink()
        assert path.suffix in {'.rlib', '.rmeta', '.d'} and not stat.st_mode & 0o111
        assert sha(path) == digest
        groups.setdefault((digest, stat.st_size, stat.st_dev, stat.st_mode), []).append(path)
    linked = {}
    for (digest, size, _, _), paths in groups.items():
        origin = max(paths, key=lambda p: (p.stat().st_nlink, 'section-proof-reuse' not in p.parts, str(p)))
        for path in paths:
            if path.stat().st_ino != origin.stat().st_ino:
                linked[str(path)] = {'retained_original': str(origin), 'sha256': digest, 'bytes': size}
    manifest = E / 'linked-compiler-duplicates-v3.json.gz'
    with gzip.open(manifest, 'wt') as stream:
        json.dump(linked, stream, sort_keys=True)
    reclaimed = 0
    for name, row in linked.items():
        path = Path(name); origin = Path(row['retained_original'])
        assert sha(path) == sha(origin) == row['sha256']
        if path.stat().st_nlink == 1: reclaimed += path.stat().st_size
        temporary = path.with_name(path.name + '.section-reuse-dedupe-v3')
        assert not temporary.exists()
        os.link(origin, temporary); os.replace(temporary, path)
    for previous in manifests + [manifest]:
        for name, row in json.loads(gzip.decompress(previous.read_bytes())).items():
            origin = Path(row['retained_original'])
            assert sha(Path(name)) == sha(origin) == row['sha256']
            assert Path(name).stat().st_ino == origin.stat().st_ino
    assert all(sha(ROOT / p) == h for p, h in protected.items())
    result = {'runner_sha256': sha(Path(__file__)), 'manifest_sha256': sha(manifest),
              'linked_paths': len(linked), 'duplicate_bytes_consolidated': reclaimed,
              'protected_artifacts_verified': len(protected),
              'all_prior_link_manifests_verified': [str(m.relative_to(ROOT)) for m in manifests],
              'scope': 'Closed non-executable compiler artifacts only; all paths and bytes preserved. Reconciles the ineffective v2 linking order and verifies every old alias relationship. New mutable trials still require independent inodes.'}
    out.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
