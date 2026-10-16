"""Close a fully executed E12 qualification without rewriting failed trials."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
QUALIFICATION = sys.argv[1]
BINDINGS = sys.argv[2]
memo = {}


def sha(path):
    path = Path(path)
    stat = path.stat()
    key = (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns)
    if key not in memo:
        h = hashlib.sha256()
        with path.open('rb') as stream:
            while block := stream.read(1024 * 1024):
                h.update(block)
        memo[key] = h.hexdigest()
    return memo[key]


def span(text):
    start = re.search(r'"current_work"\s*:\s*', text).end()
    value, length = json.JSONDecoder().raw_decode(text[start:])
    return start, start + length, value


assert not (E / 'checkpoint-v1.json').exists()
assert not (E / 'artifact-audit-v1.json.gz').exists()
previous_path = E.parent / '2026-10-15-polynomial-quotient-reduction/checkpoint-v1.json'
previous = json.loads(previous_path.read_text())
assert all(sha(ROOT / p) == v for p, v in previous['evidence_files_sha256'].items())
assert sha(ROOT / previous['artifact_audit']) == previous['artifact_audit_sha256']
with gzip.open(ROOT / previous['artifact_audit'], 'rt') as stream:
    prior_audit = json.load(stream)
pending = previous['pending_native_sha256']
assert len(pending) == 35 and all(sha(ROOT / p) == v for p, v in pending.items())
with gzip.open(E / QUALIFICATION / 'source-hashes.json.gz', 'rt') as stream:
    qualified = json.load(stream)
qualification = json.loads((E / QUALIFICATION / 'result.json').read_text())
assert len(qualification['commands']) == 14
assert all(row['exit_code'] == 0 for row in qualification['commands'][:-1])
assert qualification['commands'][-1]['name'] == 'wasm'
assert qualification['commands'][-1]['exit_code'] == 1 and not qualification['passed']
assert 'Use wasm-bindgen-cli 0.2.126, matching Cargo.lock' in (
    E / QUALIFICATION / 'wasm.stderr').read_text()
bindings = json.loads((E / BINDINGS / 'result.json').read_text())
assert bindings['passed'] and len(bindings['commands']) == 2
with gzip.open(E / BINDINGS / 'source-hashes.json.gz', 'rt') as stream:
    current = json.load(stream)
assert set(current) == set(qualified)
consumer_changes = [name for name in current if current[name] != qualified[name]]
assert consumer_changes == ['tests/wasm/geometry_certainty.cjs']
assert all(sha(ROOT / p) == v for p, v in current.items())
for phase, count in [('candidate-v2', 22), ('cost-build-v1', 11),
                     ('cost-measurement-v1', 22), ('adapter-certainty-v1', 5),
                     ('residual-policy-v1', 5), ('representation-policy-v1', 5),
                     ('curve-semantics-v2', 5), ('boundary-certainty-v1', 2),
                     ('wasm-tools-v1', 4), ('wasm-rotation-v1', 2)]:
    result = json.loads((E / phase / 'result.json').read_text())
    assert result['passed'] and len(result['commands']) == count
assert '4 passed; 0 failed; 0 ignored' in (E / 'candidate-v2/workflow-composition.stdout').read_text()
assert 'WASM certainty, query status, geometry, and invalid-input checks passed' in (
    E / BINDINGS / 'wasm.stdout').read_text()
cost = json.loads((E / 'cost-summary-v1.json').read_text())
assert len(cost['sample_rows']) == 380 and len(cost['comparisons']) == 38
cost_contract = json.loads((E / 'cost-contract-v1.json').read_text())
assert all(current['crates/hyperreal/' + name] == digest
           for name, digest in cost_contract['sources_sha256']['candidate'].items())
failed_binding = json.loads((E / 'wasm-bindings-v1/result.json').read_text())
assert not failed_binding['passed'] and failed_binding['commands'][-1]['exit_code'] == 1
assert 'Missing expected exception' in (E / 'wasm-bindings-v1/wasm.stderr').read_text()

totals = {}
for row in qualification['commands']:
    rows = [tuple(map(int, match)) for match in re.findall(
        r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;',
        (E / QUALIFICATION / (row['name'] + '.stdout')).read_text())]
    if rows:
        assert all(r[1] == 0 for r in rows)
        totals[row['name']] = {'passed': sum(r[0] for r in rows),
                               'ignored': sum(r[2] for r in rows)}
for name, count in [('scalar-default', 924), ('scalar-all-features', 1031),
                    ('foundations', 1545), ('native-mesh', 240),
                    ('curve-native', 2493), ('root-minimal', 102),
                    ('root-curve', 237), ('root-all-features', 344),
                    ('scalar-docs', 28)]:
    assert totals[name]['passed'] == count, (name, totals[name])

aggregate = {
    'passed': True,
    'rust_commands': [{'phase': QUALIFICATION, 'name': row['name'],
                       'result_sha256': sha(E / QUALIFICATION / 'result.json')}
                      for row in qualification['commands'][:-1]],
    'binding_commands': [{'phase': BINDINGS, 'name': row['name'],
                          'result_sha256': sha(E / BINDINGS / 'result.json')}
                         for row in bindings['commands']],
    'identical_rust_and_other_compiled_sources': True,
    'binding_consumer_test_changes': consumer_changes,
    'source_hashes_sha256': sha(E / QUALIFICATION / 'source-hashes.json.gz'),
    'final_source_hashes_sha256': sha(E / BINDINGS / 'source-hashes.json.gz'),
    'retained_failures': 'The original qualification remains failed: its last command rejected installed wasm-bindgen 0.2.122 before building. With locked CLI 0.2.126, wasm-bindings-v1 reproduced a stale curve uncertainty expectation. Only the CJS consumer changed: preserve the original certified point and add the independently checked unresolved nonzero-coordinate boundary. All Rust/other compiled source hashes are identical; their passed checks are reused. Both failed binding phases remain unchanged.',
    'test_totals': totals,
}
aggregate_path = E / 'qualification-summary-v1.json'
if aggregate_path.exists():
    assert json.loads(aggregate_path.read_text()) == aggregate
else:
    aggregate_path.write_text(json.dumps(aggregate, indent=2) + '\n')

expected = dict(prior_audit['expected_artifacts_sha256'])


def retain(name, digest):
    if name in expected:
        assert expected[name] == digest, name
    expected[name] = digest


runner_hashes = {sha(path) for path in E.glob('validate_v*.py')}
phases = {}
live_source_pins = {}
for result_path in sorted(E.glob('*/result.json')):
    folder = result_path.parent
    result = json.loads(result_path.read_text())
    protocol = json.loads((folder / 'protocol.json').read_text())
    assert protocol['runner_sha256'] in runner_hashes
    phases[folder.name] = {'passed': result['passed'],
                           'commands': len(result['commands']),
                           'failures': [(r['name'], r['exit_code']) for r in result['commands']
                                        if r['exit_code'] != 0]}
    for name, digest in protocol['preserved_prior_executables_sha256'].items():
        if name == 'Cargo.lock':
            # This phase pin guards a live input during execution; its content
            # is versioned in Git and in the source snapshot. Do not pretend a
            # live repository path is an immutable historical build artifact.
            assert current[name] == digest and sha(ROOT / name) == digest
            live_source_pins[name] = digest
        else:
            retain(name, digest)
    for row in result['commands']:
        assert row['sources_unchanged']
        for suffix in ['stdout', 'stderr', 'resources']:
            assert sha(folder / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
        for name, digest in row['binaries_sha256'].items():
            retain(name, digest)
print('Auditing every E12 target and inherited artifact expectation.', flush=True)
targets = {ROOT / json.loads(path.read_text()) for path in E.glob('*-target.json')}
targets.add(Path('/tmp/csgrs-factored-product-proofs/cost-v1-source'))
unused_targets = []
for target in sorted(targets):
    if not target.exists():
        assert target == Path('/tmp/csgrs-factored-product-proofs/analysis-v1')
        assert json.loads((E / 'analysis-v1.json').read_text()) == [
            ['remaining-algebra', ['python3', 'docs/evidence/2026-10-15-factored-product-proofs/analyze_remaining_v1.py']]]
        assert not any((ROOT / name).is_relative_to(target) for name in expected)
        # This analysis writes evidence beside its script, with no build output.
        unused_targets.append(str(target))
        continue
    assert target.is_dir(), target
    for path in target.rglob('*'):
        assert not path.is_symlink(), path
        if path.is_file():
            retain(str(path), sha(path))
verified, missing = {}, {}
for name, digest in expected.items():
    path = ROOT / name
    if not path.exists():
        missing[name] = digest
    else:
        assert sha(path) == digest, name
        verified[name] = digest
assert missing == prior_audit['missing_artifacts_sha256']
assert len(missing) == 27093
audit = {'expected_artifacts_sha256': expected, 'verified_artifacts_sha256': verified,
         'missing_artifacts_sha256': missing, 'hash_mismatches': 0,
         'new_missing_artifacts': 0, 'historical_preservation_complete': False,
         'declared_but_unused_target_directories': unused_targets,
         'loss': 'Inherited external cleanup: 26957 historical artifacts and 136 pinned diagnostic inputs. Expected hashes are retained; no recovery is claimed.'}
with gzip.open(E / 'artifact-audit-v1.json.gz', 'wt') as stream:
    json.dump(audit, stream, sort_keys=True)
with gzip.open(E / 'artifact-audit-v1.json.gz', 'rt') as stream:
    assert json.load(stream) == audit

source_names = set(previous['implementation_sha256']) | {
    'crates/hyperreal/src/computable/node/ring_product_sum.rs',
    'crates/hyperreal/src/computable/node/ring_product_sum_tests.rs',
    'crates/hypersolve/src/residual_replay.rs',
    'crates/hypersolve/tests/real_representations.rs',
    'src/curve/native.rs', 'tests/geometry_context.rs',
    'tests/wasm/geometry_certainty.cjs',
}
sources = {name: sha(ROOT / name) for name in sorted(source_names)}
assert all(current[name] == digest for name, digest in sources.items())
ledger = ROOT / 'docs/properties-status.json'
raw = ledger.read_text()
before = sha(ledger)
reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'],
                                    cwd=ROOT, text=True)
start, end, work = span(raw)
rstart, rend, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rstart], reference[rend:])
prefix = str(E.relative_to(ROOT)) + '/'
work['factored_product_proof_work'] = {
    'status': 'certified original 8/16/24/32 strict composition corpus; focused qualification and cost evidence retained; full admission remains open',
    'baseline_commit': '045621eba',
    'progress': prefix + 'progress-v1.md',
    'validation': prefix + 'qualification-summary-v1.json',
    'rust_validation': prefix + QUALIFICATION + '/result.json',
    'binding_validation': prefix + BINDINGS + '/result.json',
    'model_acceptance': prefix + 'candidate-v2/result.json',
    'cost_contract': prefix + 'cost-contract-v1.json',
    'cost_summary': prefix + 'cost-summary-v1.json',
    'checkpoint': prefix + 'checkpoint-v1.json',
    'artifact_audit': prefix + 'artifact-audit-v1.json.gz',
    'implementation_sha256': sources,
    'test_totals': totals,
    'workflow': 'All original full-resolution 8/16/24/32 drill workflows now complete Certified empty accumulation, subtraction to canonical empty and reunion. Independent point membership, polygonal-volume enclosure and separate manifold checks pass.',
    'certainty': 'Original e/pi dense and all 22 representation solver cases, crescent, rack boundary, 16-segment scalar/JavaScript drill and rotated VRML fixtures remain; obsolete permanent-Unknown and redundant-vertex assertions are replaced with stronger receipts and semantic checks. Additional genuinely unresolved trigonometric fixtures preserve strict/approximate/strict behavior.',
    'resource_contract': 'Shared-factor zero recovery reuses the same remaining depth/node/term/degree/coefficient/work budget and validates every operand. Query-local storage, no larger caps or persistent negative cache. External containment is not an internal full-query work or heap-byte bound.',
    'cost_scope': 'Five fixed alternating instrumented scalar pairs against 7d69ef51d; all 380 raw rows and unfavorable results retained. Completion is separate from timing. This is a completeness extension, not a downstream or cross-library speedup claim.',
    'historical_preservation': 'Incomplete: inherited 27093 missing artifact paths retain expected hashes. No new loss or mismatch at this checkpoint; no historical recovery claim.',
    'remaining': 'Scalar Boolean adapter typed-error propagation, wider supported workloads and release/external-consumer validation; full-query resources/provenance/formal/coverage obligations remain. The 35 pending native files retain their prior hashes and are not admitted by these checks.',
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
files = {str(path.relative_to(ROOT)): sha(path) for path in E.rglob('*')
         if path.is_file() and '__pycache__' not in path.parts}
record = {'status': 'focused scalar and downstream qualification; full admission remains open',
          'previous_closure_sha256': sha(previous_path), 'evidence_files_sha256': files,
          'artifact_audit': prefix + 'artifact-audit-v1.json.gz',
          'artifact_audit_sha256': sha(E / 'artifact-audit-v1.json.gz'),
          'expected_artifact_paths': len(expected), 'verified_artifact_paths': len(verified),
          'missing_artifact_paths': len(missing), 'historical_preservation_complete': False,
          'implementation_sha256': sources, 'pending_native_sha256': pending,
          'live_source_dependency_pins_sha256': live_source_pins,
          'test_totals': totals, 'phases': phases, 'ledger_before_sha256': before,
          'ledger_after_sha256': sha(ledger), 'only_current_work_changed': True,
          'runtime_admitted': False, 'full_plan_complete': False}
(E / 'checkpoint-v1.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({key: record[key] for key in ['expected_artifact_paths',
    'verified_artifact_paths', 'missing_artifact_paths', 'test_totals', 'runtime_admitted']}), flush=True)
