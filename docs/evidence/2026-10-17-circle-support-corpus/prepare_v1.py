"""Freeze current main plus exactly the preserved cache proposal."""
import gzip
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
OLD = W.parent / 'circle-support-index'
O = W / 'preparation-v1'
S = W / 'candidate-source-v1'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, v):
    Path(p).write_text(json.dumps(v, indent=2) + '\n')

base = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
assert base == '0ed5a970cf0d9204e9f4c5c73e93b16a6a13bc89'
subprocess.run(['git', 'diff', '--quiet'], cwd=ROOT, check=True)
parent = read(ROOT / 'docs/evidence/2026-10-17-test-storage-recovery/checkpoint-v1.json')
assert all(sha(ROOT / p) == h for p, h in parent['implementation_sha256'].items())
assert all(sha(ROOT / p) == h for p, h in parent['evidence_files_sha256'].items())
proposal = read(OLD / 'decision-v1.json')['prototype_sha256']
with gzip.open(OLD / 'baseline-preparation-v1/main-input-hashes.json.gz', 'rt') as f:
    names = json.load(f)
before = {p: sha(ROOT / p) for p in names}
O.mkdir(exist_ok=False)
S.mkdir(exist_ok=False)
for p in names:
    dest = S / p
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ROOT / p, dest)
for p, h in proposal.items():
    src = OLD / 'candidate-source-v2' / p
    assert sha(src) == h
    dest = S / p
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dest)
after = {p: sha(S / p) for p in names.keys() | proposal.keys()}
with gzip.open(OLD / 'candidate-preparation-v2/source-hashes.json.gz', 'rt') as f:
    previous = json.load(f)
changes = {p: {'previous': previous.get(p), 'current': h} for p, h in after.items() if previous.get(p) != h}
assert set(changes) == set(parent['implementation_sha256']) | {'docs/properties-status.json'}
assert all(after[p] == h for p, h in parent['implementation_sha256'].items())
with (O / 'format.stdout').open('wb') as out, (O / 'format.stderr').open('wb') as err:
    result = subprocess.run(['cargo', 'fmt', '--all', '--', '--check'], cwd=S, stdout=out, stderr=err)
assert result.returncode == 0
with gzip.open(O / 'main-input-hashes.json.gz', 'wt') as f:
    json.dump(before, f, sort_keys=True)
with gzip.open(O / 'source-hashes.json.gz', 'wt') as f:
    json.dump(after, f, sort_keys=True)
dump(O / 'result.json', {
    'complete': True, 'base_commit': base, 'source': str(S), 'source_inputs': len(after),
    'source_hashes_sha256': sha(O / 'source-hashes.json.gz'), 'main_input_hashes_sha256': sha(O / 'main-input-hashes.json.gz'),
    'proposal_sha256': proposal, 'changes_from_e38_candidate': changes,
    'production_implementation_identical_to_e38_candidate': True,
    'test_storage_repair_included': True, 'main_sources_modified': False, 'cache_admitted': False,
})
print(json.dumps({'complete': True, 'source_inputs': len(after), 'only_e38_differences': list(changes)}), flush=True)
