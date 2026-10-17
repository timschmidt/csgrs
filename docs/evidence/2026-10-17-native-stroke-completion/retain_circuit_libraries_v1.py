"""Retain completed circuit-build libraries before later target feature variants."""
import hashlib, json, shutil
from pathlib import Path

W = Path(__file__).resolve().parent
O = W / 'snapshot-replay-libraries-v1'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

build = W / 'consumer-validation-v1/circuit-build.stdout'
rows = json.loads((W / 'consumer-validation-v1/commands.json').read_text())
row = next(r for r in rows if r['name'] == 'circuit-build')
assert row['exit_code'] == 0 and sha(build) == row['stdout_sha256']
assert any(r['name'] == 'text-fabrication' and r['exit_code'] == 0 for r in rows)
assert not any(r['name'] in ['clippy', 'native-cli'] for r in rows)
artifacts = [json.loads(s) for s in build.read_text().splitlines() if s.startswith('{')]
paths = {Path(p) for r in artifacts if r.get('reason') == 'compiler-artifact'
    for p in r['filenames'] if p.endswith(('.rlib', '.so'))}
O.mkdir(exist_ok=False)
(O / 'libraries').mkdir()
retained = {}
for p in sorted(paths):
    before = p.stat()
    h = sha(p)
    dest = O / 'libraries' / p.name
    if dest.exists():
        assert sha(dest) == h
    else:
        shutil.copy2(p, dest)
    after = p.stat()
    assert all(getattr(before, k) == getattr(after, k) for k in ['st_ino', 'st_size', 'st_mtime_ns', 'st_ctime_ns'])
    assert sha(dest) == h
    retained[str(p)] = {'retained': str(dest), 'sha256': h, 'bytes': before.st_size}
record = {'complete': True, 'runner_sha256': sha(__file__),
    'completed_build_stdout_sha256': sha(build), 'retained': retained,
    'scope': 'Completed Hypercircuit feature variant before combined lint/native-cli variants. Source target remains in its original ongoing validation session; retained library copies are frozen for read-only test replay.',
    'retained_files': len(retained), 'retained_bytes': sum(r['bytes'] for r in retained.values()),
    'source_target_modified_by_retention': False}
(O / 'result.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({k: record[k] for k in ['complete', 'retained_files', 'retained_bytes']}), flush=True)
