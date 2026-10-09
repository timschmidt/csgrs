"""Certify the two existing demo failures against an isolated committed checkout.

Usage: python3 audit_demo_baseline.py BASELINE_TREE BASELINE_LOG VALIDATION OUTPUT
The tree is produced by git archive at baseline-reuse.json's base revision.
"""
import hashlib, json, re, shutil, subprocess, sys
from pathlib import Path

record = Path(__file__).resolve().parent
base, log, validation, out = map(Path, sys.argv[1:])
out.mkdir(parents=True, exist_ok=False)
info = lambda p: dict(bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest())
verification = dict(base_revision=json.loads((record/'baseline-reuse.json').read_text())['base_revision'],
                    source_hashes={})
paths = ['crates/'+c for c in ['hypercurve','hyperreal','hyperlattice','hyperlimit','hypersolve','hypertri']]
rows = subprocess.check_output(['git','ls-tree','-r','-z',verification['base_revision'],'--',*paths]).split(b'\0')
count = 0
for row in rows:
    if not row: continue
    meta, path = row.split(b'\t', 1)
    _, kind, witness = meta.split()
    assert kind == b'blob'
    data = (base/path.decode()).read_bytes()
    assert hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest().encode() == witness
    if path.endswith(b'.rs'):
        verification['source_hashes'][path.decode()] = hashlib.sha256(data).hexdigest()
    count += 1
verification['verified_git_blobs'] = count
before, after = log.read_text(), (validation/'demo-tests.log').read_text()
failed = lambda s: sorted(re.findall(r'^test ([^ ]+) \.\.\. FAILED$', s, re.M))
expected = sorted(['scenes::tests::multi_boolean_defaults_resolve_all_boolean_modes',
                   'scenes::tests::shared_polyline_boolean_state_resolves_all_boolean_modes'])
assert failed(before) == failed(after) == expected
diagnostics = lambda s: [line for line in s.splitlines() if 'the demo cannot yet edit a ' in line]
assert diagnostics(before) == diagnostics(after)
counts = lambda s: tuple(map(int, re.search(r'test result: FAILED\. (\d+) passed; (\d+) failed; (\d+) ignored;', s).groups()))
assert counts(before) == (1, 2, 0) and counts(after) == (35, 2, 0)
shutil.copy(log, out/'baseline.log')
shutil.copy(validation/'demo-tests.log', out/'candidate.log')
(out/'source-verification.json').write_text(json.dumps(verification, indent=2)+'\n')
result = dict(status='matched-existing-failures', base_revision=verification['base_revision'],
              verified_git_blobs=count, baseline_sources=len(verification['source_hashes']),
              command=['cargo','test','--locked','--no-fail-fast','all_boolean_modes'],
              cwd=str(base/'crates/hypercurve/examples/hypercurve_ui'),
              environment={'CARGO_TARGET_DIR':str(Path.cwd()/'crates/hypercurve/examples/hypercurve_ui/target')},
              baseline_exit_code=101, candidate_exit_code=101,
              baseline_counts=counts(before), candidate_counts=counts(after),
              failures=expected, identical_diagnostics=diagnostics(after),
              baseline_log=info(log), candidate_log=info(validation/'demo-tests.log'),
              scope='Two existing demo editability failures only. The full candidate demo has 35 passes and these two failures. This attributes existing failures for the local checkpoint; the demo test gate remains failed. General restricted-fragment editability remains in FUTURE_WORK.md.')
(out/'manifest.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps({k:result[k] for k in ['status','failures','baseline_counts','candidate_counts']}, indent=2))
