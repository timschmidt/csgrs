"""Fork the integer-leaf source for metadata-first duplicate validation.

Only the existing conjunction operands change at runtime. Source equality uses
immutable integer records; geometry comparison remains the second predicate.
The separate arrangement-index validation and all error messages stay intact.
"""
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
PARENT = ROOT / 'docs/evidence/2026-10-12-small-integer-certificates'
BASE = ROOT / 'target/small-integer-certificates/source'
WORK = ROOT / 'target/region-metadata-first'
SOURCE = WORK / 'source'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    original = json.loads(gzip.decompress((PARENT / 'baseline-source-hashes.json.gz').read_bytes()))
    baseline = original | json.loads((PARENT / 'validation-v1/protocol.json').read_text())['source_changes_sha256']
    guard = json.loads((PARENT / 'preparation.json').read_text())['main_source_sha256']
    assert sources() == guard
    assert all(sha(BASE / name) == digest for name, digest in baseline.items())
    WORK.mkdir(exist_ok=False)
    with (E / 'copy.log').open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(BASE), str(SOURCE)],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    region = 'crates/hypercurve/src/bezier_region.rs'
    helper = 'crates/hypercurve/src/bezier_region/metadata_order_tests.rs'
    path = SOURCE / region
    text = path.read_text()
    before = '            boundary_loop.fragments == candidate.fragments\n                && boundary_loop.arrangement_sources == candidate.arrangement_sources'
    after = '            boundary_loop.arrangement_sources == candidate.arrangement_sources\n                && boundary_loop.fragments == candidate.fragments'
    assert text.count(before) == 1
    text = text.replace(before, after)
    before = '#[cfg(test)]\nmod tests;'
    assert text.count(before) == 1
    text = text.replace(before, before + '\n\n#[cfg(test)]\nmod metadata_order_tests;')
    path.write_text(text)
    shutil.copyfile(E / 'metadata_order_tests.rs', SOURCE / helper)
    with (E / 'prepare-format.log').open('w') as log:
        subprocess.run(['rustfmt', '--edition', '2024', str(path)], stdout=log,
                       stderr=subprocess.STDOUT, check=True)
    changed = [region, helper]
    current = {name: sha(SOURCE / name) for name in sorted(set(baseline) | {helper})}
    assert {name for name, digest in current.items() if baseline.get(name) != digest} == set(changed)
    with tarfile.open(E / 'candidate-delta.tar.gz', 'w:gz') as archive:
        for name in changed:
            archive.add(SOURCE / name, arcname=name)
    patch = ''.join(''.join(difflib.unified_diff(
        (BASE / name).read_text().splitlines(True) if name in baseline else [],
        (SOURCE / name).read_text().splitlines(True), 'a/' + name, 'b/' + name)) for name in changed)
    (E / 'candidate.patch').write_text(patch)
    with (E / 'baseline-source-hashes.json.gz').open('xb') as raw:
        with gzip.GzipFile(fileobj=raw, mode='wb', mtime=0, filename='') as stream:
            stream.write((json.dumps(baseline, indent=2) + '\n').encode())
    record = {
        'scope': __doc__,
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'runner_sha256': sha(Path(__file__)),
        'diagnostic_assessment_sha256': sha(ROOT / 'docs/evidence/2026-10-12-region-source-comparisons/assessment-v1.json'),
        'main_source_sha256': guard,
        'baseline_source': str(BASE), 'candidate_source': str(SOURCE),
        'baseline_hashes_sha256': sha(E / 'baseline-source-hashes.json.gz'),
        'source_changes_sha256': {name: current[name] for name in changed},
        'candidate_archive_sha256': sha(E / 'candidate-delta.tar.gz'),
        'candidate_patch_sha256': sha(E / 'candidate.patch'),
        'copied_files': len(baseline), 'runtime_admitted': False,
    }
    assert sources() == guard
    assert all(sha(BASE / name) == digest for name, digest in baseline.items())
    (E / 'preparation.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps({'copied_files': len(baseline), 'changed': changed, 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
