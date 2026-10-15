"""Reclaim only byte-identical, non-executable compiler copies in closed trials."""
import gzip
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / 'compiler-reclamation-v2.json'
    assert not out.exists()
    previous = ROOT / 'docs/evidence/2026-10-15-metaball-failures/closure-v1.json'
    protected = json.loads(previous.read_text())['preserved_executables_sha256']
    for record in sorted(E.glob('*/result.json')):
        for row in json.loads(record.read_text())['commands']:
            protected.update(row['binaries_sha256'])
    assert all(sha(ROOT / p) == digest for p, digest in protected.items())
    targets = [ROOT / 'target/metaball-failures/baseline-v1', ROOT / 'target/metaball-failures/candidate-v2', Path('/tmp/csgrs-curve-metaball-failures/wrappers-baseline-v1'), ROOT / 'target/metaball-failures/native-v1']
    # Preserve the newest usable seed and every unique file. Only closed trials
    # enter this scan; no Cargo process may be active in these directories.
    seen, removed = {}, {}
    for target in reversed(targets):
        record_dir = E if str(target).startswith('/tmp/') else previous.parent
        assert (record_dir / target.name / 'result.json').exists()
        for path in sorted(target.rglob('*')):
            if path.suffix not in {'.rlib', '.rmeta', '.d'} or not path.is_file():
                continue
            assert not path.is_symlink() and str(path) not in protected
            if path.stat().st_mode & 0o111:
                continue
            digest = sha(path)
            key = (digest, path.stat().st_size)
            if key in seen and target != ROOT / 'target/metaball-failures/native-v1':
                removed[str(path)] = {'retained_original': seen[key], 'sha256': digest, 'bytes': key[1]}
            else:
                seen.setdefault(key, str(path))
    manifest = E / 'removed-compiler-duplicates-v2.json.gz'
    with gzip.open(manifest, 'wt') as f:
        json.dump(removed, f, sort_keys=True)
    for name, row in removed.items():
        assert sha(Path(row['retained_original'])) == row['sha256']
        assert sha(Path(name)) == row['sha256']
        Path(name).unlink()
    assert all(sha(ROOT / p) == digest for p, digest in protected.items())
    record = {
        'runner_sha256': sha(Path(__file__)),
        'protected_executables_verified': len(protected),
        'duplicate_count': len(removed),
        'duplicate_bytes_removed': sum(row['bytes'] for row in removed.values()),
        'manifest_sha256': sha(manifest),
        'reason': 'Storage quota blocked duplicated seeds. All executable, unique compiler, source and log artifacts remain. Only byte-identical non-executable .rlib/.rmeta/.d copies in closed trials were removed, with retained originals mapped by digest.'
    }
    out.write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(record), flush=True)


if __name__ == '__main__':
    main()
