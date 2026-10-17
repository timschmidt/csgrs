"""Freeze the candidate without diagnostic test instrumentation."""
import gzip
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

W = Path(__file__).resolve().parent
S = W / 'candidate-source-v2'
O = W / 'candidate-preparation-v2'
OLD = W / 'candidate-source-v1'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, v):
    Path(p).write_text(json.dumps(v, indent=2) + '\n')

assert read(W / 'candidate-validation-v1/result.json')['command_failures'] == []
assert read(W / 'baseline-validation-v1/result.json')['command_failures'] == []
prep = read(W / 'candidate-preparation-v1/result.json')
manifest = W / 'candidate-preparation-v1/source-hashes.json.gz'
assert sha(manifest) == prep['source_hashes_sha256']
with gzip.open(manifest, 'rt') as f:
    original = json.load(f)
assert all(sha(OLD / p) == h for p, h in original.items())
S.mkdir(exist_ok=False)
O.mkdir(exist_ok=False)
diagnostic = 'crates/hypercurve/src/circle_support_cost_probe.rs'
for p in original:
    if p == diagnostic:
        continue
    dest = S / p
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(OLD / p, dest)
curve = 'crates/hypercurve/src/curve_intersection.rs'
decl = '\n#[cfg(test)]\n#[path = "circle_support_cost_probe.rs"]\nmod circle_support_cost_probe;\n'
before = (S / curve).read_text()
assert before.count(decl) == 1
(S / curve).write_text(before.replace(decl, ''))
readme = S / 'crates/hypercurve/README.md'
text = readme.read_text()
anchor = '- `CurvePreviewOptions` owns finite display tolerances separately.'
addition = '''- Native intersection batches retain owned circle-support relations for one
  synchronous operation under one predicate policy. Exact rational centers and
  squared radii use an ordered value key; symbolic supports retain the existing
  equality proofs and first-match order. Hits reuse the same relation and ordered
  witnesses. The cache has no eviction or configurable disable switch and is
  released when the batch is dropped. Its entry count, key storage, and scalar
  payloads have no internal heap-byte or total-work bound; it is not shared
  between independent operations or threads.
'''
assert text.count(anchor) == 1
readme.write_text(text.replace(anchor, addition + anchor))
with (O / 'format.stdout').open('wb') as out, (O / 'format.stderr').open('wb') as err:
    result = subprocess.run(['cargo', 'fmt', '--all'], cwd=S, stdout=out, stderr=err)
assert result.returncode == 0
assert (S / curve).read_text() == before.replace(decl, '')
final = {p: sha(S / p) for p in original if p != diagnostic}
assert final['crates/hypercurve/src/circle_support_cache_tests.rs'] == original['crates/hypercurve/src/circle_support_cache_tests.rs']
with gzip.open(O / 'source-hashes.json.gz', 'wt') as f:
    json.dump(final, f, sort_keys=True)
dump(O / 'result.json', {
    'complete': True, 'base_commit': prep['base_commit'],
    'source': str(S), 'source_inputs': len(final),
    'source_hashes_sha256': sha(O / 'source-hashes.json.gz'),
    'runner_sha256': sha(__file__), 'main_sources_modified': False,
    'removed_diagnostic_test_only': diagnostic,
    'production_implementation_and_semantic_test_bytes_unchanged_from_v1': True,
    'changes_from_v1': {p: {'before': original[p], 'after': h} for p, h in final.items() if h != original[p]},
    'native_runtime_admitted': False,
})
print(json.dumps({'complete': True, 'source_inputs': len(final), 'production_bytes_unchanged': True}), flush=True)
