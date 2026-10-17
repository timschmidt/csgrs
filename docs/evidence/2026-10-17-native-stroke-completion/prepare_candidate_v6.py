"""Freeze only the independently verified readiness-metadata expectation migration."""
import gzip, hashlib, json, shutil
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

audit = read(W / 'metadata-migration-v2/result.json')
assert audit['complete'] and audit['geometry_findings_and_other_artifacts_unchanged']
with gzip.open(W / 'candidate-preparation-v5/source-hashes.json.gz', 'rt') as f:
    prior = json.load(f)
with gzip.open(W / 'candidate-preparation-v1/baseline-source-hashes.json.gz', 'rt') as f:
    baseline = json.load(f)
assert all(sha(W / 'candidate-source-v5' / p) == h for p, h in prior.items())
assert all(sha(ROOT / p) == h for p, h in baseline.items())
source, out = W / 'candidate-source-v6', W / 'candidate-preparation-v6'
source.mkdir(exist_ok=False)
out.mkdir(exist_ok=False)
for p, h in prior.items():
    dest = source / p
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(W / 'candidate-source-v5' / p, dest)
    assert sha(dest) == h
for fixture in audit['fixtures']:
    name = fixture['fixture']
    assert sha(source / name) == fixture['expected_sha256']
    if name.endswith('.zip'):
        replacement = W / 'metadata-migration-v2/independently-rebuilt-release.zip'
    else:
        slug = Path(name).stem
        replacement = W / 'artifact-mismatches-v1' / (slug + '_runs_end_to_end_through_hyperdrc') / 'actual.json'
    assert sha(replacement) == fixture['actual_sha256']
    shutil.copy2(replacement, source / name)
final = {p: sha(source / p) for p in prior}
delta = {p: h for p, h in final.items() if h != prior[p]}
assert set(delta) == {r['fixture'] for r in audit['fixtures']} and len(delta) == 7
assert all(final[p] == h for p, h in read(W / 'candidate-preparation-v5/result.json')['changes'].items())
changes = {p: h for p, h in final.items() if baseline.get(p) != h}
assert len(final) == 2835 and len(changes) == 14
with gzip.open(out / 'source-hashes.json.gz', 'wt') as f:
    json.dump(final, f, sort_keys=True)
record = {'complete': True, 'base_commit': 'b137381247a03342ce5b6184710475fcc22c9d63',
    'source_inputs': len(final), 'source_hashes_sha256': sha(out / 'source-hashes.json.gz'),
    'changes': changes, 'delta_from_v5': delta, 'runner_sha256': sha(__file__),
    'metadata_migration_audit_sha256': sha(W / 'metadata-migration-v2/result.json'),
    'production_code_and_test_bodies_unchanged_from_v5': True,
    'original_source_fixtures_unchanged': True, 'main_sources_modified': False}
(out / 'result.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({'source_inputs': len(final), 'changed_paths': len(changes),
    'expected_artifact_migrations': len(delta), 'production_code_unchanged_from_v5': True}), flush=True)
