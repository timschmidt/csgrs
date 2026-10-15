"""Close the zero-quotient candidate without claiming complete admission."""
from pathlib import Path
import gzip, hashlib, json, re, subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
memo = {}


def sha(path):
    path = Path(path)
    st = path.stat()
    key = (st.st_dev, st.st_ino, st.st_size, st.st_mtime_ns)
    if key not in memo:
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        memo[key] = digest.hexdigest()
    return memo[key]


def span(text):
    start = re.search(r'"current_work"\s*:\s*', text).end()
    value, length = json.JSONDecoder().raw_decode(text[start:])
    return start, start + length, value


assert not (E / 'checkpoint-v1.json').exists()
previous_path = ROOT / 'docs/evidence/2026-10-15-shared-scalar-proofs/checkpoint-v1.json'
previous = json.loads(previous_path.read_text())
assert all(sha(ROOT / p) == digest for p, digest in previous['evidence_files_sha256'].items())
pending = previous['pending_native_sha256']
assert len(pending) == 35 and all(sha(ROOT / p) == digest for p, digest in pending.items())
current = json.loads(gzip.decompress((E / 'zero-qualification-v2/source-hashes.json.gz').read_bytes()))
assert all(sha(ROOT / p) == digest for p, digest in current.items())
assert json.loads((E / 'zero-qualification-v2/result.json').read_text())['passed']
assert json.loads((E / 'factor-analysis-v1/result.json').read_text())['passed']
model = json.loads((E / 'zero-candidate-v1/result.json').read_text())
assert len(model['commands']) == 20 and not model['passed']
assert all(row['exit_code'] == 0 for row in model['commands'][:-1])
assert model['commands'][-1]['name'] == 'workflow-composition'
assert model['commands'][-1]['exit_code'] == 101
composition = (E / 'zero-candidate-v1/workflow-composition.stdout').read_text()
assert '2 passed; 2 failed; 0 ignored' in composition
assert composition.count('tests/strict_drilled_workflows.rs:99:10') == 2
candidate = json.loads(gzip.decompress((E / 'zero-candidate-v1/source-hashes.json.gz').read_bytes()))
assert set(current) == set(candidate)
assert {p for p in current if current[p] != candidate[p]} == {
    'crates/hyperreal/tests/quotient_polynomial_cancellation.rs'
}  # Only the additional serde assertion followed the direct downstream run.
totals = {}
for name, expected in [('scalar-default', 912), ('scalar-all-features', 1019), ('scalar-docs', 28)]:
    log = (E / 'zero-qualification-v2' / (name + '.stdout')).read_text()
    rows = [tuple(map(int, row)) for row in re.findall(
        r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', log)]
    assert sum(row[0] for row in rows) == expected
    assert all(row[1] == 0 for row in rows)
    totals[name] = expected

protected = dict(previous['preserved_artifacts_sha256'])
for result_path in E.glob('*/result.json'):
    folder = result_path.parent
    result = json.loads(result_path.read_text())
    protocol = json.loads((folder / 'protocol.json').read_text())
    assert protocol['runner_sha256'] in [sha(E / 'validate_v1.py'), sha(E / 'validate_v2.py')]
    protected.update(protocol['preserved_prior_executables_sha256'])
    for row in result['commands']:
        assert row['sources_unchanged']
        for suffix in ['stdout', 'stderr', 'resources']:
            assert sha(folder / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
        protected.update(row['binaries_sha256'])
for target in [Path('/tmp/csgrs-quotient-factor-proofs'), Path('/run/user/1000/csgrs-quotient-factor-proofs')]:
    for path in target.rglob('*'):
        if path.is_file():
            protected[str(path)] = sha(path)
print('Verifying preserved prior and current artifacts.', flush=True)
assert all(sha(ROOT / p) == digest for p, digest in protected.items())
sources = {p: sha(ROOT / p) for p in previous['implementation_sha256']}
assert all(current[p] == digest for p, digest in sources.items())

ledger = ROOT / 'docs/properties-status.json'
raw = ledger.read_text()
before = sha(ledger)
reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT, text=True)
start, end, work = span(raw)
rstart, rend, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rstart], reference[rend:])
work['zero_quotient_proof_work'] = {
    'status': 'candidate: all original drills now compose with empty; complete 16/32 subtraction and admission remain open',
    'baseline_commit': '3eca3cce8',
    'progress': 'docs/evidence/2026-10-15-quotient-factor-proofs/progress-v1.md',
    'validation': 'docs/evidence/2026-10-15-quotient-factor-proofs/zero-qualification-v2/result.json',
    'model_acceptance': 'docs/evidence/2026-10-15-quotient-factor-proofs/zero-candidate-v1/result.json',
    'production_baseline_diagnosis': 'docs/evidence/2026-10-15-quotient-factor-proofs/diagnostic-v4/result.json',
    'conditional_algebra': 'docs/evidence/2026-10-15-quotient-factor-proofs/factor-reduction-algebra-v1.json',
    'checkpoint': 'docs/evidence/2026-10-15-quotient-factor-proofs/checkpoint-v1.json',
    'implementation_sha256': sources,
    'qualification': '912 default and 1019 all-feature scalar tests, 28 scalar doctests, scalar all-feature/all-target and root curve/all-target Clippy with warnings denied, and workspace formatting pass. Direct downstream context and independent polygonal membership/volume-enclosure/topology checks pass. Counts overlap.',
    'workflow': '8/24 still complete empty accumulation, complete subtraction and reunion. 16/32 now pass Certified empty accumulation and subsequent membership, then fail complete subtraction with typed Real sign classification Unknown. The original full-resolution tests remain failing.',
    'resource_contract': 'Discard proved-zero denominators only after both operands pass expansion and inverse-domain guards. No recognition limit or policy change; no persistent cache or authoritative expression rewrite. External containment is not an internal total-work or heap-byte bound.',
    'remaining': 'Reproduce the remaining subtraction decline on this candidate and investigate exact factor cancellation or retained construction proofs without weakening limits. Broad downstream, matched cost/allocation/artifact comparisons, release and external user-path validation remain open.',
    'runtime_admitted': False,
    'full_plan_complete': False,
}
work['plan_complete'] = False
work['updated_on'] = '2026-10-15'
ledger.write_text(raw[:start] + json.dumps(work, indent=2).replace('\n', '\n  ') + raw[end:])
updated = ledger.read_text()
ustart, uend, _ = span(updated)
assert (updated[:ustart], updated[uend:]) == (reference[:rstart], reference[rend:])
x, y = json.loads(updated), json.loads(reference)
x.pop('current_work')
y.pop('current_work')
assert x == y
files = {str(p.relative_to(ROOT)): sha(p) for p in E.rglob('*')
         if p.is_file() and '__pycache__' not in p.parts}
record = {
    'status': 'tested zero-quotient candidate, not final admission',
    'previous_closure_sha256': sha(previous_path),
    'evidence_files_sha256': files,
    'preserved_artifacts_sha256': protected,
    'implementation_sha256': sources,
    'pending_native_sha256': pending,
    'test_totals': totals,
    'ledger_before_sha256': before,
    'ledger_after_sha256': sha(ledger),
    'only_current_work_changed': True,
    'runtime_admitted': False,
    'full_plan_complete': False,
}
(E / 'checkpoint-v1.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({'preserved_artifacts': len(protected),
                  'evidence_files_including_checkpoint': len(files) + 1,
                  'native_files_unchanged': 35, 'runtime_admitted': False}), flush=True)
