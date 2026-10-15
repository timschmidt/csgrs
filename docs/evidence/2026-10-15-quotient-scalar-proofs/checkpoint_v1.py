"""Preserve this tested candidate and its next frontier without admitting it."""
from pathlib import Path
import gzip, hashlib, json, re, subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
memo = {}


def sha(path):
    path = Path(path); st = path.stat(); key = (st.st_dev, st.st_ino, st.st_size, st.st_mtime_ns)
    if key not in memo:
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024*1024), b''): digest.update(block)
        memo[key] = digest.hexdigest()
    return memo[key]


def span(text):
    start = re.search(r'"current_work"\s*:\s*', text).end()
    value, length = json.JSONDecoder().raw_decode(text[start:])
    return start, start+length, value


assert not (E/'checkpoint-v1.json').exists()
previous_path = ROOT/'docs/evidence/2026-10-15-affine-scalar-proofs/checkpoint-v1.json'
previous = json.loads(previous_path.read_text())
assert all(sha(ROOT/p) == digest for p, digest in previous['evidence_files_sha256'].items())
pending = previous['pending_native_sha256']
assert len(pending) == 35 and all(sha(ROOT/p) == digest for p, digest in pending.items())
current = json.loads(gzip.decompress((E/'scalar-and-drill-v1/source-hashes.json.gz').read_bytes()))
assert all(sha(ROOT/p) == digest for p, digest in current.items())
assert json.loads((E/'scalar-and-drill-v1/result.json').read_text())['passed']
protected = dict(previous['preserved_artifacts_sha256'])
for result_path in E.glob('*/result.json'):
    folder = result_path.parent
    result = json.loads(result_path.read_text())
    protocol = json.loads((folder/'protocol.json').read_text())
    assert protocol['runner_sha256'] in [sha(E/'validate_v1.py')]
    protected.update(protocol['preserved_prior_executables_sha256'])
    for row in result['commands']:
        assert row['sources_unchanged']
        for suffix in ['stdout', 'stderr', 'resources']:
            assert sha(folder/(row['name']+'.'+suffix)) == row[suffix+'_sha256']
        protected.update(row['binaries_sha256'])
for target in [Path('/tmp/csgrs-quotient-scalar-proofs'), Path('/run/user/1000/csgrs-quotient-scalar-proofs')]:
    for path in target.rglob('*'):
        if path.is_file(): protected[str(path)] = sha(path)
print('Verifying preserved prior and current artifacts.', flush=True)
assert all(sha(ROOT/p) == digest for p, digest in protected.items())
sources = {p: sha(ROOT/p) for p in [
    'crates/hyperreal/src/computable/node.rs',
    'crates/hyperreal/src/computable/node/ring_polynomial.rs',
    'crates/hyperreal/src/computable/node/ring_quotient.rs',
    'crates/hyperreal/src/computable/node/ring_quotient_tests.rs',
    'crates/hyperreal/src/real/arithmetic/facts.rs',
    'crates/hyperreal/tests/quotient_polynomial_cancellation.rs',
]}
ledger = ROOT/'docs/properties-status.json'; raw = ledger.read_text(); before = sha(ledger)
reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT, text=True)
start, end, work = span(raw); rstart, rend, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rstart], reference[rend:])
work['quotient_scalar_proof_work'] = {
    'status': 'candidate: guarded quotient proofs pass; shared-expression frontier and full admission remain open',
    'baseline_commit': 'a9e3fbb29',
    'progress': 'docs/evidence/2026-10-15-quotient-scalar-proofs/progress-v1.md',
    'validation': 'docs/evidence/2026-10-15-quotient-scalar-proofs/scalar-and-drill-v1/result.json',
    'diagnosis': 'docs/evidence/2026-10-15-quotient-scalar-proofs/orientation-algebra-v1.json',
    'checkpoint': 'docs/evidence/2026-10-15-quotient-scalar-proofs/checkpoint-v1.json',
    'implementation_sha256': sources,
    'qualification': '15 commands pass; 906 default and 1013 all-feature scalar tests, strict scalar Clippy, formatting, 28 doctests, CSGRS curve build and eight original drill observations. Configurations overlap; probe success is not strict geometry completion.',
    'workflow': '8/24 segments remain Certified. 16/32 strict pass coordinate equality and stop at retained line orientation; approximate results remain qualified. Captured zero determinants have 648 arithmetic visits, exceeding the 512-node attempt limit.',
    'remaining': 'Investigate shared subexpression proof reuse without raising limits; preserve prior successful proofs, complete original strict drills, semantic/topology/composition acceptance, broad downstream qualification and matched completed-contract costs before retention.',
    'runtime_admitted': False,
    'full_plan_complete': False,
}
work['plan_complete'] = False; work['updated_on'] = '2026-10-15'
ledger.write_text(raw[:start]+json.dumps(work, indent=2).replace('\n', '\n  ')+raw[end:])
updated = ledger.read_text(); ustart, uend, _ = span(updated)
assert (updated[:ustart], updated[uend:]) == (reference[:rstart], reference[rend:])
x, y = json.loads(updated), json.loads(reference); x.pop('current_work'); y.pop('current_work'); assert x == y
files = {str(p.relative_to(ROOT)): sha(p) for p in E.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
record = {'status': 'tested candidate, not final admission', 'previous_closure_sha256': sha(previous_path),
          'evidence_files_sha256': files, 'preserved_artifacts_sha256': protected,
          'implementation_sha256': sources, 'pending_native_sha256': pending,
          'ledger_before_sha256': before, 'ledger_after_sha256': sha(ledger),
          'only_current_work_changed': True, 'runtime_admitted': False, 'full_plan_complete': False}
(E/'checkpoint-v1.json').write_text(json.dumps(record, indent=2)+'\n')
print(json.dumps({'preserved_artifacts': len(protected), 'evidence_files_including_checkpoint': len(files)+1, 'native_files_unchanged': 35, 'runtime_admitted': False}), flush=True)
