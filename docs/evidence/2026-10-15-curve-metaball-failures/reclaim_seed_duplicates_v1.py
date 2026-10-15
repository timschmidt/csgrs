"""Remove only unexecuted partial or byte-identical seed copies after terminal preparation failure."""
import gzip, hashlib, json, shutil
from pathlib import Path
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    out = E / 'seed-reclamation-v1.json'
    assert not out.exists()
    prior = json.loads((ROOT / 'docs/evidence/2026-10-15-metaball-failures/closure-v1.json').read_text())
    protected = dict(prior['preserved_executables_sha256'])
    baseline = json.loads((E / 'baseline-v1/result.json').read_text())
    for row in baseline['commands']: protected.update(row['binaries_sha256'])
    assert all(sha(ROOT / p) == digest for p, digest in protected.items())
    preparation = json.loads((E / 'candidate-v2/preparation.json').read_text())
    assert preparation['exit_code'] == 1 and not (E / 'candidate-v2/protocol.json').exists()
    partial = Path('/tmp/csgrs-curve-metaball-failures/candidate-v2')
    assert partial.is_dir() and not partial.is_symlink()
    partial_bytes = sum(p.stat().st_size for p in partial.rglob('*') if p.is_file())
    shutil.rmtree(partial)
    # The baseline compiled only csgrs and ran one test executable. No seeded
    # dependency build script or other copied test binary ran in this phase.
    stderr = (E / 'baseline-v1/regressions.stderr').read_text().splitlines()
    compile_lines = [line.strip() for line in stderr if 'Compiling ' in line]
    assert len(compile_lines) == 1 and compile_lines[0].startswith('Compiling csgrs ')
    seed = ROOT / 'target/metaball-failures/native-v1/release'
    copied = Path('/tmp/csgrs-curve-metaball-failures/baseline-v1/release')
    observed = {str((ROOT / name).resolve()) for name in protected}
    removed = {}
    total = 0
    for path in copied.rglob('*'):
        if not path.is_file() or path.is_symlink() or str(path.resolve()) in observed: continue
        original = seed / path.relative_to(copied)
        if original.is_file():
            digest = sha(original)
            if sha(path) == digest:
                removed[str(path)] = {'retained_original': str(original.relative_to(ROOT)), 'sha256': digest, 'bytes': path.stat().st_size}
                total += path.stat().st_size
    with gzip.open(E / 'removed-seed-duplicates-v1.json.gz', 'wt') as f: json.dump(removed, f, sort_keys=True)
    for path, row in removed.items():
        assert sha(ROOT / row['retained_original']) == row['sha256']
        Path(path).unlink()
    assert all(sha(ROOT / p) == digest for p, digest in protected.items())
    record = {'runner_sha256': sha(Path(__file__)), 'partial_candidate': str(partial), 'partial_bytes_removed': partial_bytes, 'duplicate_count': len(removed), 'duplicate_bytes_removed': total, 'duplicate_manifest_sha256': sha(E / 'removed-seed-duplicates-v1.json.gz'), 'protected_executables_verified': len(protected), 'reason': 'Both candidate preparations exited before compilation. Baseline duplicate seed files retain identical originals in the preceding qualified target; its executed test binary and unique compiler output remain preserved. No executed artifact or source snapshot was removed.'}
    out.write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(record), flush=True)
if __name__ == '__main__': main()
