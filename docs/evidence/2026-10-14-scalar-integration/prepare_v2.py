"""Integrate the frozen fallible scalar prototype with the current main inputs.

This creates a new, isolated source tree. Neither ancestor is edited and no
runtime admission follows from a textual merge. Conflicts remain explicit.
"""
import gzip
import hashlib
import io
import json
from pathlib import Path
import shutil
import subprocess
import tarfile

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OLD = ROOT / 'docs/evidence/2026-10-10-scalar-evaluation-failure'
MAIN_RECORD = ROOT / 'docs/evidence/2026-10-14-directed-scalar-search/main-admission-v3/source-hashes-after.json.gz'
BASE = ROOT / 'target/artifact-numeric-keys-candidate-v2/source'
CANDIDATE = ROOT / 'target/scalar-evaluation-candidate/source'
DEST = ROOT / 'target/scalar-main-integration-v2/source'
OUT = E / 'preparation-v2'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_map(path):
    return json.loads(gzip.decompress(path.read_bytes()))


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def inventory(root):
    return {str(p.relative_to(root)): sha(p) for p in root.rglob('*') if p.is_file()}


def archive(path, root, names):
    with path.open('xb') as raw, gzip.GzipFile(fileobj=raw, mode='wb', mtime=0) as packed:
        with tarfile.open(fileobj=packed, mode='w') as tar:
            for name in sorted(names):
                data = (root / name).read_bytes()
                info = tarfile.TarInfo(name)
                info.size = len(data)
                info.mode = (root / name).stat().st_mode & 0o777
                tar.addfile(info, io.BytesIO(data))


def main():
    assert not OUT.exists() and not DEST.exists()
    assert not subprocess.check_output(['git', 'diff', '--cached', '--name-only'], cwd=ROOT)
    records = {
        'main': MAIN_RECORD,
        'base': OLD / 'candidate-preparation/source-hashes.json.gz',
        'candidate': OLD / 'typed-api-prototype-v2492-integrated-verus/source-hashes.json.gz',
    }
    maps = {key: read_map(path) for key, path in records.items()}
    assert (len(maps['main']), len(maps['base']), len(maps['candidate'])) == (10170, 24697, 24831)
    for label, root in [('main', ROOT), ('base', BASE), ('candidate', CANDIDATE)]:
        assert all(sha(root / name) == digest for name, digest in maps[label].items()), label
    assert maps['base'].keys() <= maps['candidate'].keys()
    delta = {name: digest for name, digest in maps['candidate'].items() if maps['base'].get(name) != digest}
    assert len(delta) == 214
    assert all(name.startswith('crates/hyperreal/') or name in {
        'Cargo.toml', 'Cargo.lock', 'crates/hypercurve/src/spline_periodic.rs',
        'crates/hyperlimit/tests/real_representations.rs'} for name in delta)
    native_record = ROOT / 'docs/evidence/2026-10-12-existing-worktree/geometry-validation-closure/result.json'
    native = json.loads(native_record.read_text())['pending_native_sha256']
    assert len(native) == 35 and not set(native).intersection(delta)
    assert all(sha(ROOT / name) == digest for name, digest in native.items())
    OUT.mkdir()
    DEST.mkdir(parents=True)
    for name in maps['main']:
        path = DEST / name
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / name, path)
    review_path = E / 'proof-additions-review-v2.json'
    review = json.loads(review_path.read_text())
    expected = {name for name in delta if name not in maps['base'] and name in maps['main']
                and maps['main'][name] != delta[name]}
    assert set(review) == expected and len(review) == 9
    for name, row in review.items():
        assert row['main_sha256'] == maps['main'][name] and row['candidate_sha256'] == delta[name]
        if row['shared_ancestor']:
            prior = read_map(OLD / row['shared_ancestor']['source_record'])
            assert prior[name] == maps['main'][name]
    old_scope = json.loads((ROOT / 'crates/hyperreal/verification/scope.json').read_text())
    new_scope = json.loads((CANDIDATE / 'crates/hyperreal/verification/scope.json').read_text())
    for key in ['objective', 'obligations']:
        assert old_scope[key] == new_scope[key]
    for key in ['verified_executable_contracts', 'verified_model_theorems']:
        assert set(old_scope[key]) <= set(new_scope[key])
    merges = []
    for name, digest in delta.items():
        path = DEST / name
        path.parent.mkdir(parents=True, exist_ok=True)
        current = maps['main'].get(name)
        if current is None or current in {maps['base'].get(name), digest}:
            shutil.copy2(CANDIDATE / name, path)
            continue
        if name in review:
            shutil.copy2(CANDIDATE / name, path)
            continue
        assert name in maps['base'], ('add/add requires explicit handling', name)
        command = ['git', 'merge-file', '-p', '-L', 'current-main', '-L', 'prototype-base',
                   '-L', 'fallible-prototype', str(ROOT / name), str(BASE / name), str(CANDIDATE / name)]
        result = subprocess.run(command, capture_output=True, cwd=ROOT)
        assert 0 <= result.returncode <= 127 and not result.stderr, (name, result.returncode, result.stderr)
        path.write_bytes(result.stdout)
        merges.append({'path': name, 'exit_code': result.returncode,
                       'main_sha256': current, 'base_sha256': maps['base'][name],
                       'candidate_sha256': digest, 'merged_sha256': sha(path)})
    after = inventory(DEST)
    changed = {name: digest for name, digest in after.items() if maps['main'].get(name) != digest}
    record = OUT / 'source-hashes.json.gz'
    record.write_bytes(gzip.compress((json.dumps(after, sort_keys=True) + '\n').encode(), mtime=0))
    archive(OUT / 'overlay.tar.gz', DEST, changed)
    for label, root in [('main', ROOT), ('base', BASE), ('candidate', CANDIDATE)]:
        assert all(sha(root / name) == digest for name, digest in maps[label].items()), label
    assert all(sha(DEST / name) == digest for name, digest in native.items())
    dump(OUT / 'result.json', {
        'status': 'textual integration only; conflicts require review; no runtime admission',
        'main_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'source_directory': str(DEST), 'source_files': len(after),
        'source_records': {label: {'path': str(path.relative_to(ROOT)), 'sha256': sha(path)}
                           for label, path in records.items()},
        'runner_sha256': sha(Path(__file__)), 'prototype_delta_files': len(delta),
        'reviewed_proof_additions': review, 'proof_additions_review_sha256': sha(review_path),
        'merged_paths': merges, 'conflicted_paths': [row['path'] for row in merges if row['exit_code']],
        'changed_from_main_sha256': changed, 'source_record_sha256': sha(record),
        'overlay_sha256': sha(OUT / 'overlay.tar.gz'),
        'main_and_frozen_ancestors_unchanged': True, 'pending_native_files_preserved': len(native),
        'runtime_admitted': False, 'plan_complete': False,
    })
    print(json.dumps({'source_files': len(after), 'changed_from_main': len(changed),
                      'three_way_merges': len(merges),
                      'conflicts': [row['path'] for row in merges if row['exit_code']]}), flush=True)


if __name__ == '__main__':
    main()
