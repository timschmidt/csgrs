"""Prepare fresh isolated native sources for a closed-certificate cache trial.

Start from the explicitly authorized current main worktree, including the
compact line correction. Do not import the separate scalar evaluation prototype
or rely on its now-historical main-worktree guard.
"""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = ROOT / 'target/persistent-laurent-cache'
W = ROOT / 'docs/evidence/2026-10-12-existing-worktree'
sys.path.insert(0, str(W))
from validate import sources

sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    closure = json.loads((W / 'geometry-validation-closure/result.json').read_text())
    assert closure['source_unchanged'] and not closure['runtime_admitted']
    assert sources() == json.loads((W / 'geometry-layout-validation/protocol.json').read_text())['sources_sha256']
    for name, expected in closure['pending_native_sha256'].items():
        assert sha(ROOT / name) == expected
    main_sources = sources()
    names = subprocess.check_output(['git', 'ls-files', '-z', '--cached', '--others', '--exclude-standard'], cwd=ROOT).decode().split('\0')
    names = sorted({name for name in names if name
                    and not name.startswith(('docs/evidence/', '.git/', 'target/', 'crates/comparisons/'))
                    and (ROOT / name).is_file()})
    original = {name: sha(ROOT / name) for name in names}
    WORK.mkdir(exist_ok=False)
    for variant in ['baseline', 'candidate']:
        source = WORK / variant / 'source'
        source.mkdir(parents=True)
        for name in names:
            path = source / name
            path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / name, path)
            assert sha(path) == original[name]
        print(json.dumps({'variant': variant, 'copied_files': len(names)}), flush=True)
    assert sources() == main_sources
    assert all(sha(ROOT / name) == expected for name, expected in original.items())
    with (E / 'initial-source-hashes.json.gz').open('xb') as raw:
        with gzip.GzipFile(fileobj=raw, mode='wb', mtime=0, filename='') as stream:
            stream.write((json.dumps(original, indent=2) + '\n').encode())
    record = {
        'scope': __doc__,
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'runner_sha256': sha(Path(__file__)),
        'previous_geometry_closure_sha256': sha(W / 'geometry-validation-closure/result.json'),
        'source_files_copied_per_variant': len(names),
        'source_hashes_sha256': sha(E / 'initial-source-hashes.json.gz'),
        'main_source_sha256': main_sources,
        'excluded_paths': ['docs/evidence/', '.git/', 'target/', 'crates/comparisons/'],
        'excluded_comparisons_reason': 'This package is excluded from the root Cargo workspace; its external oracle audit artifacts are not required for the initial scalar/native prototype checks. No comparison-package validation is claimed.',
        'source_directories': {variant: str(WORK / variant / 'source') for variant in ['baseline', 'candidate']},
        'cache_contract': 'Only successful graph-free certificates may be retained. Warming may strengthen an Unknown but must preserve every available definite fact. Opaque atoms, failed recognition and abort outcomes must not be cached as proofs.',
        'qualification_required': [
            'Cold/warm/cloned and concurrent facts at depth, work and term boundaries, including unchanged failed-query fallback.',
            'Opaque expression ownership and cache reclamation, approximation/error contracts and original per-pass recognition limits.',
            'Independent mathematical identities, fresh scalar and consumer tests, strict Clippy, formatting and documentation.',
            'Matched time/allocation/size controls, full board fixtures with unchanged 120-second worker and 60-second copper-neck limits, and complete report replay.',
        ],
        'runtime_admitted': False,
        'main_runtime_unchanged': True,
    }
    (E / 'preparation.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps({'prepared': True, 'source_files': len(names), 'main_runtime_unchanged': True}), flush=True)


if __name__ == '__main__':
    main()
