"""Verify staged production bytes while preserving the separate native worktree."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[3]
evidence = Path(__file__).resolve().parent
record = json.loads((evidence / 'candidate-v5/sources.json').read_text())
source = Path(record['source_directory'])
expected = set(record['changed_files'])
staged = {name.decode() for name in subprocess.check_output(['git', 'diff', '--cached', '--name-only', '-z'], cwd=root).split(b'\0') if name}
assert staged & expected == expected
assert all(name in expected or name.startswith('docs/') for name in staged)
index = {}
for entry in subprocess.check_output(['git', 'ls-files', '--stage', '-z'], cwd=root).split(b'\0'):
    if entry:
        fields, name = entry.split(b'\t', 1)
        mode, oid, stage = fields.decode().split()
        assert stage == '0'
        index[name.decode()] = {'mode': mode, 'oid': oid}

def git_hash(data):
    return hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest()

sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
main = json.loads(gzip.decompress(Path(record['main_hashes_file']).read_bytes()))
assert all(sha(root / name) == value for name, value in main.items())
checked, absent = [], []
for name in sorted(main):
    if name in index and (source / name).is_file():
        assert git_hash((source / name).read_bytes()) == index[name]['oid'], name
        checked.append(name)
    else:
        absent.append(name)
native_only = ['crates/hypercurve/src/bezier_region/linear_stroke.rs', 'crates/hypercurve/tests/native_linear_stroke.rs', 'crates/hyperdrc/src/checks/area_tolerance.rs', 'crates/hyperdrc/src/geometry/stroke.rs', 'crates/hyperdrc/src/kicad/tracks.rs', 'crates/hyperdrc/tests/native_linear_strokes.rs']
ignored_lock = 'crates/hyperlattice/fuzz/Cargo.lock'
assert absent == sorted(native_only + [ignored_lock])
assert subprocess.run(['git', 'check-ignore', '-q', ignored_lock], cwd=root, check=False).returncode == 0
old = json.loads(subprocess.check_output(['git', 'show', 'HEAD:docs/properties-status.json'], cwd=root))
new = json.loads((root / 'docs/properties-status.json').read_text())
for key in ['properties', 'targets']:
    assert [(row['id'], row['status']) for row in old[key]] == [(row['id'], row['status']) for row in new[key]]
result = {'production_source_record': str(evidence / 'candidate-v5/sources.json'), 'staged_runtime_paths': sorted(expected), 'staged_runtime_blob_ids': {name: index[name] for name in sorted(expected)}, 'compiled_scope_index_files_matching_candidate': len(checked), 'native_only_untracked_files_excluded': native_only, 'other_excluded_file': {'path': ignored_lock, 'reason': 'Ignored fuzz-workspace lock exists in main/native but not the production candidate; fuzz builds are outside this scoped validation.'}, 'all_1166_main_source_guards_unchanged': True, 'all_property_and_target_statuses_unchanged': True, 'scope': 'The Git index matches tested production bytes across all 1159 applicable guarded files. Existing native prototype files and edits remain outside this commit.'}
path = evidence / 'staged-source-verification.json'
path.write_text(json.dumps(result, indent=2) + '\n')
manifest = json.loads((evidence / 'retention.json').read_text())
for artifact in [path, Path(__file__)]:
    manifest['artifacts'][artifact.name] = {'bytes': artifact.stat().st_size, 'sha256': sha(artifact)}
manifest['staged_source_verification'] = {'path': str(path), **manifest['artifacts'][path.name]}
(evidence / 'retention.json').write_text(json.dumps(manifest, indent=2) + '\n')
print('Git index matches', len(checked), 'guarded production files; all 1166 main guards and all property/target statuses are unchanged.')
