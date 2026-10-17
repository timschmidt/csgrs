"""Check which completed v4 observations retain byte-identical dependency inputs."""
import gzip, hashlib, json, subprocess
from pathlib import Path

W = Path(__file__).resolve().parent
O = W / 'v4-qualification-reuse-v1'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def gz(p):
    with gzip.open(p, 'rt') as f:
        return json.load(f)

old = gz(W / 'candidate-preparation-v4/source-hashes.json.gz')
new = gz(W / 'candidate-preparation-v5/source-hashes.json.gz')
assert old.keys() == new.keys()
assert all(sha(W / 'candidate-source-v4' / p) == h for p, h in old.items())
assert all(sha(W / 'candidate-source-v5' / p) == h for p, h in new.items())
delta = {p: {'v4': old[p], 'v5': new[p]} for p in old if old[p] != new[p]}
assert set(delta) == {'crates/hypercircuit/src/materialize.rs', 'crates/hypercircuit/tests/fabrication.rs'}
O.mkdir(exist_ok=False)
command = ['cargo', 'metadata', '--offline', '--locked', '--format-version=1', '--no-deps', '--all-features']
result = subprocess.run(command, cwd=W / 'candidate-source-v5', capture_output=True)
(O / 'metadata.stdout').write_bytes(result.stdout)
(O / 'metadata.stderr').write_bytes(result.stderr)
assert result.returncode == 0
metadata = json.loads(result.stdout)
packages = {p['name']: p for p in metadata['packages']}
edges = {n: sorted({d['name'] for d in p['dependencies'] if d['name'] in packages}) for n, p in packages.items()}

def reaches(start, sought):
    pending, seen = [start], set()
    while pending:
        current = pending.pop()
        if current == sought:
            return True
        if current not in seen:
            seen.add(current)
            pending.extend(edges[current])
    return False

assert all(not reaches(p, 'hypercircuit') for p in ['hypercurve', 'csgrs', 'hyperdrc'])
commands = read(W / 'candidate-validation-v4/commands.json')
names = ['stroke-regressions', 'curve-default', 'root-all', 'root-minimal',
    'native-tests', 'full-board', 'copper-neck', 'gerber', 'docs']
reused = []
for name in names:
    row = next(r for r in commands if r['name'] == name)
    assert row['exit_code'] == 0
    assert all(sha(W / 'candidate-validation-v4' / (name + '.' + s)) == row[s + '_sha256']
               for s in ['stdout', 'stderr', 'resources'])
    reused.append({'name': name, 'command': row['command'], 'exit_code': row['exit_code'],
        'input_scope': 'All snapshot files are byte-identical except two Hypercircuit-owned files; no conservative local dependency path from the tested package reaches Hypercircuit.'})
record = {'complete': True, 'runner_sha256': sha(__file__), 'command': command,
    'metadata_stdout_sha256': sha(O / 'metadata.stdout'), 'metadata_stderr_sha256': sha(O / 'metadata.stderr'),
    'source_manifest_sha256': {str(p): sha(p) for p in [W / 'candidate-preparation-v4/source-hashes.json.gz', W / 'candidate-preparation-v5/source-hashes.json.gz']},
    'changed_files': delta, 'conservative_workspace_dependency_edges': edges,
    'reused_completed_observations': reused,
    'whole_v4_passed': False, 'hypercircuit_or_combined_lint_reused': False,
    'final_source_qualified': False, 'main_sources_modified': False}
(O / 'result.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({'complete': True, 'unchanged_dependency_observations': len(reused), 'changed_paths': list(delta)}), flush=True)
