"""Close the factor candidate; retain and disclose all historical losses."""
from pathlib import Path
import gzip
import hashlib
import json
import re
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
memo = {}


def sha(path):
    path = Path(path)
    st = path.stat()
    key = (st.st_dev, st.st_ino, st.st_size, st.st_mtime_ns)
    if key not in memo:
        h = hashlib.sha256()
        with path.open('rb') as f:
            while block := f.read(1024 * 1024):
                h.update(block)
        memo[key] = h.hexdigest()
    return memo[key]


def span(text):
    start = re.search(r'"current_work"\s*:\s*', text).end()
    value, length = json.JSONDecoder().raw_decode(text[start:])
    return start, start + length, value


assert not (E / 'checkpoint-v1.json').exists()
assert not (E / 'artifact-audit-v1.json.gz').exists()
previous_path = E.parent / '2026-10-15-quotient-factor-proofs/checkpoint-v1.json'
previous = json.loads(previous_path.read_text())
assert all(sha(ROOT / p) == v for p, v in previous['evidence_files_sha256'].items())
pending = previous['pending_native_sha256']
assert len(pending) == 35 and all(sha(ROOT / p) == v for p, v in pending.items())
current = json.loads(gzip.decompress((E / 'factor-qualification-v1/source-hashes.json.gz').read_bytes()))
assert all(sha(ROOT / p) == v for p, v in current.items())
for phase in ['factor-qualification-v1', 'diagnostic-v3', 'analysis-v2']:
    assert json.loads((E / phase / 'result.json').read_text())['passed']
candidate = json.loads(gzip.decompress((E / 'factor-candidate-v2/source-hashes.json.gz').read_bytes()))
assert candidate == current
model = json.loads((E / 'factor-candidate-v2/result.json').read_text())
assert len(model['commands']) == 22 and not model['passed']
assert all(row['exit_code'] == 0 for row in model['commands'][:-1])
assert model['commands'][-1]['name'] == 'workflow-composition'
assert model['commands'][-1]['exit_code'] == 101
composition = (E / 'factor-candidate-v2/workflow-composition.stdout').read_text()
assert '3 passed; 1 failed; 0 ignored' in composition
assert composition.count('tests/strict_drilled_workflows.rs:99:10') == 1
totals = {}
for name, expected in [('scalar-default', 919), ('scalar-all-features', 1026), ('scalar-docs', 28)]:
    log = (E / 'factor-qualification-v1' / (name + '.stdout')).read_text()
    rows = [tuple(map(int, r)) for r in re.findall(
        r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', log)]
    assert sum(r[0] for r in rows) == expected and all(r[1] == 0 for r in rows)
    totals[name] = expected

expected = dict(previous['preserved_artifacts_sha256'])
for result_path in E.glob('*/result.json'):
    folder = result_path.parent
    result = json.loads(result_path.read_text())
    protocol = json.loads((folder / 'protocol.json').read_text())
    assert protocol['runner_sha256'] in [sha(E / f'validate_v{i}.py') for i in range(1, 5)]
    expected.update(protocol['preserved_prior_executables_sha256'])
    for row in result['commands']:
        assert row['sources_unchanged']
        for suffix in ['stdout', 'stderr', 'resources']:
            assert sha(folder / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
        expected.update(row['binaries_sha256'])
for target in [ROOT / 'target/properties/polynomial-quotient-reduction',
               Path('/run/user/1000/csgrs-polynomial-quotient-reduction')]:
    for path in target.rglob('*'):
        if path.is_file():
            expected[str(path)] = sha(path)
verified, missing = {}, {}
print('Auditing recorded survivors and losses.', flush=True)
for name, digest in expected.items():
    path = ROOT / name
    if not path.exists():
        missing[name] = digest
    else:
        assert sha(path) == digest, name
        verified[name] = digest
loss = json.loads((E / 'external-artifact-loss-v1.json').read_text())['missing_artifacts_sha256']
historical_missing = {p: v for p, v in missing.items() if p in previous['preserved_artifacts_sha256']}
assert historical_missing == loss
additional_missing = {p: v for p, v in missing.items() if p not in loss}
assert all(p.startswith('/tmp/csgrs-polynomial-quotient-reduction/diagnostic-v1') for p in additional_missing)
audit = {'expected_artifacts_sha256': expected, 'verified_artifacts_sha256': verified,
         'missing_artifacts_sha256': missing,
         'historical_missing_count': len(loss), 'lost_diagnostic_inputs_count': len(additional_missing),
         'hash_mismatches': 0, 'historical_preservation_complete': False}
with gzip.open(E / 'artifact-audit-v1.json.gz', 'wt') as stream:
    json.dump(audit, stream, sort_keys=True)
with gzip.open(E / 'artifact-audit-v1.json.gz', 'rt') as stream:
    assert json.load(stream) == audit
source_names = set(previous['implementation_sha256']) | {
    'crates/hyperreal/src/computable/node/ring_quotient_reduction.rs',
    'crates/hyperreal/src/computable/node/ring_quotient_reduction_tests.rs',
    'crates/hyperreal/tests/quotient_factor_reduction.rs',
}
sources = {p: sha(ROOT / p) for p in sorted(source_names)}
assert all(current[p] == v for p, v in sources.items())

ledger = ROOT / 'docs/properties-status.json'
raw = ledger.read_text()
before = sha(ledger)
reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT, text=True)
start, end, work = span(raw)
rstart, rend, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rstart], reference[rend:])
prefix = str(E.relative_to(ROOT)) + '/'
work['polynomial_quotient_reduction_work'] = {
    'status': 'tested candidate: original 16-segment full chain is now Certified; 32-segment subtraction and admission remain open',
    'baseline_commit': 'cfcd73640',
    'progress': prefix + 'progress-v1.md',
    'validation': prefix + 'factor-qualification-v1/result.json',
    'model_acceptance': prefix + 'factor-candidate-v2/result.json',
    'current_remaining_diagnosis': prefix + 'diagnostic-v3/result.json',
    'independent_algebra': prefix + 'remaining-algebra-v1.json',
    'checkpoint': prefix + 'checkpoint-v1.json',
    'artifact_audit': prefix + 'artifact-audit-v1.json.gz',
    'implementation_sha256': sources,
    'qualification': '919 default and 1026 all-feature scalar tests, 28 doctests, selected scalar/root Clippy with warnings denied and formatting pass. The original 8/16/24/32 membership, volume-enclosure and topology checks pass. Counts overlap with direct tests.',
    'workflow': '8/16/24 complete Certified empty accumulation, subtraction to canonical empty and reunion. 32 retains a failing full test at complete subtraction with typed PredicateUndecided. Fresh diagnosis identifies a later 64-term limit; independent algebra needs 145 terms for an exact-zero degree-eight expression with shared factors.',
    'resource_contract': 'Optional full-denominator division after arithmetic declines, with strict existing divisor witnesses and the same remaining recognition budget. No cap, policy, fixture or authoritative expression change. External containment is not an internal total-work or heap-byte bound.',
    'historical_preservation': 'Incomplete: 26957 inherited /tmp artifacts and additional pinned diagnostic source copies were externally lost. Every expected hash is retained, separately from verified survivors. No recovery claim.',
    'remaining': 'Investigate bounded common-factor cancellation for the current 32-segment predicate. Broad downstream, matched cost/allocation/artifact, external-consumer and release checks, and the 35 pending native files remain open.',
    'runtime_admitted': False,
    'full_plan_complete': False,
}
work['updated_on'] = '2026-10-15'
work['plan_complete'] = False
ledger.write_text(raw[:start] + json.dumps(work, indent=2).replace('\n', '\n  ') + raw[end:])
updated = ledger.read_text()
ustart, uend, _ = span(updated)
assert (updated[:ustart], updated[uend:]) == (reference[:rstart], reference[rend:])
x, y = json.loads(updated), json.loads(reference)
x.pop('current_work'); y.pop('current_work')
assert x == y
files = {str(p.relative_to(ROOT)): sha(p) for p in E.rglob('*')
         if p.is_file() and '__pycache__' not in p.parts}
record = {'status': 'tested factor-recovery candidate, not final admission',
          'previous_closure_sha256': sha(previous_path), 'evidence_files_sha256': files,
          'artifact_audit': prefix + 'artifact-audit-v1.json.gz',
          'artifact_audit_sha256': sha(E / 'artifact-audit-v1.json.gz'),
          'expected_artifact_paths': len(expected), 'verified_artifact_paths': len(verified),
          'missing_artifact_paths': len(missing), 'historical_preservation_complete': False,
          'implementation_sha256': sources, 'pending_native_sha256': pending,
          'test_totals': totals, 'ledger_before_sha256': before,
          'ledger_after_sha256': sha(ledger), 'only_current_work_changed': True,
          'runtime_admitted': False, 'full_plan_complete': False}
(E / 'checkpoint-v1.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({k: record[k] for k in ['expected_artifact_paths', 'verified_artifact_paths',
    'missing_artifact_paths', 'test_totals', 'runtime_admitted']}), flush=True)
