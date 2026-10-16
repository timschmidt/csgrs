"""Close admitted Minkowski evidence and change only the raw current_work value."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OLD = E.parent / '2026-10-16-polyhedral-construction'
sys.path.insert(0, str(OLD))
from load_artifact_audit_v1 import load

PREFIX = str(E.relative_to(ROOT)) + '/'
memo = {}

def sha(path):
    path = Path(path)
    stat = path.stat()
    key = (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns)
    if key not in memo:
        with path.open('rb') as stream:
            memo[key] = hashlib.file_digest(stream, 'sha256').hexdigest()
    return memo[key]

def gz(path):
    with gzip.open(path, 'rt') as stream:
        return json.load(stream)

def span(raw):
    start = re.search(r'"current_work"\s*:\s*', raw).end()
    value, length = json.JSONDecoder().raw_decode(raw[start:])
    return start, start + length, value

assert not (E / 'checkpoint-v1.json').exists()
assert not (E / 'artifact-audit-v1.json.gz').exists()
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == 'adef9a64a91a7524006ba41527e427e2edb79fbe'
previous_path = OLD / 'checkpoint-v1.json'
previous = json.loads(previous_path.read_text())
assert all(sha(ROOT / p) == v for p, v in previous['evidence_files_sha256'].items())
assert sha(ROOT / previous['artifact_audit']) == previous['artifact_audit_sha256']
prior, inherited_missing = load(ROOT / previous['artifact_audit'])
assert len(prior) == 515223 and len(inherited_missing) == 27093
pending = previous['pending_native_sha256']
assert len(pending) == 35 and all(sha(ROOT / p) == v for p, v in pending.items())

ledger = ROOT / 'docs/properties-status.json'
raw = ledger.read_text()
before = sha(ledger)
assert before == previous['ledger_after_sha256']
reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT, text=True)
start, end, work = span(raw)
rstart, rend, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rstart], reference[rend:])

summary = json.loads((E / 'qualification-summary-v1.json').read_text())
assert summary['passed'] and len(summary['timing_rows']) == 78 and len(summary['allocation_rows']) == 26
assert (E / 'progress-v1.md').is_file()
qualified = gz(E / 'qualification-v1/source-hashes.json.gz')
current = gz(E / 'resources-v1/source-hashes.json.gz')
assert current == gz(E / 'benchmark-repair-v1/source-hashes.json.gz')
adjustment = json.loads((E / 'wasm-count-adjustment-v1.json').read_text())
late_changes = {p for p in set(qualified) | set(current) if qualified.get(p) != current.get(p)}
assert late_changes == {'src/wasm/mesh_js.rs', 'tests/wasm/geometry_certainty.cjs', 'readme.md'}
assert set(adjustment['sources']) == late_changes
for p in late_changes:
    assert adjustment['sources'][p] == {'before_sha256': qualified[p], 'after_sha256': current[p]}
assert all(sha(ROOT / p) == v for p, v in current.items())
baseline = gz(E / 'baseline-v1/source-hashes.json.gz')
old = gz(OLD / 'resources-v1/source-hashes.json.gz')
def implementation_sources(values):
    return {p: v for p, v in values.items() if not p.startswith('docs/evidence/')}
assert implementation_sources(baseline) == implementation_sources(old)
changed = sorted(p for p in set(baseline) | set(current)
                 if not p.startswith('docs/evidence/') and baseline.get(p) != current.get(p))
host = json.loads((E / 'cost-host-v1.json').read_text())
assert changed == sorted(host['source_sizes']), changed
assert not set(changed) & pending.keys()
implementation = {p: current[p] for p in changed}
inherited = previous['inherited_implementation_sha256'] | previous['implementation_sha256']
inherited = {p: v for p, v in inherited.items() if p not in changed}
assert all(sha(ROOT / p) == v for p, v in inherited.items())
extra_inputs = previous['extra_unchanged_inputs_sha256']
assert all(current[p] == v for p, v in extra_inputs.items())
dependency_pins = previous['live_source_dependency_pins_sha256']
assert all(sha(ROOT / p) == v for p, v in dependency_pins.items())

expected = dict(prior)
def retain(name, digest):
    if name in expected: assert expected[name] == digest, name
    expected[name] = digest

targets = {ROOT / json.loads((E / (name + '-target.json')).read_text())
           for name, phase in summary['phases'].items() if not phase.get('incomplete')}
base_runner = E.parent / '2026-10-15-factored-product-proofs/validate_v8.py'
wrappers = {sha(E / f'validate_v{n}.py') for n in [1, 2, 3, 4]}
for name, phase in summary['phases'].items():
    folder = E / name
    if phase.get('incomplete'):
        assert name == 'candidate-v4' and not (folder / 'protocol.json').exists()
        continue
    protocol = json.loads((folder / 'protocol.json').read_text())
    assert protocol['runner_sha256'] == sha(base_runner)
    assert protocol['wrapper_sha256'] in wrappers
    assert protocol['base_commit'] == 'adef9a64a91a7524006ba41527e427e2edb79fbe'
    specification = json.loads((E / (name + '.json')).read_text())
    assert protocol['commands'] == specification
    for path, digest in protocol['preserved_prior_executables_sha256'].items(): retain(path, digest)
    result = json.loads((folder / 'result.json').read_text())
    assert [row['name'] for row in result['commands']] == [row[0] for row in specification]
    assert phase['passed'] == result['passed']
    for row in result['commands']:
        assert row['sources_unchanged']
        for suffix in ['stdout', 'stderr', 'resources']:
            assert sha(folder / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
        for path, digest in row['binaries_sha256'].items(): retain(path, digest)
    if (folder / 'overlay.json').exists():
        overlay = json.loads((folder / 'overlay.json').read_text())
        hashes = folder / 'overlay-lower-sha256.json.gz'
        assert sha(hashes) == overlay['lower_hashes_sha256']
        for path, digest in gz(hashes).items():
            assert sha(ROOT / path) == digest, path
            retain(path, digest)
        assert Path(overlay['target']).is_mount()
        targets.add(Path(overlay['upper']))

print('Auditing inherited artifacts, new targets and immutable overlay lowers.', flush=True)
for target in sorted(targets):
    assert target.is_dir(), target
    for path in target.rglob('*'):
        assert not path.is_symlink(), path
        if path.is_file(): retain(str(path), sha(path))
for path in Path('/dev/shm/csgrs-minkowski-contracts').glob('*.log'):
    if path.name != 'closure-v1.log': retain(str(path), sha(path))
missing = {}
for index, (name, digest) in enumerate(expected.items()):
    path = ROOT / name
    if path.exists():
        assert sha(path) == digest, name
    else:
        missing[name] = digest
    if index and index % 100000 == 0: print('Verified expectations:', index, flush=True)
assert missing == inherited_missing
added = {name: digest for name, digest in expected.items() if name not in prior}
audit = {
    'schema_version': 2, 'parent_audit': previous['artifact_audit'],
    'parent_audit_sha256': previous['artifact_audit_sha256'],
    'added_artifacts_sha256': added,
    'expected_artifact_paths': len(expected), 'verified_artifact_paths': len(expected) - len(missing),
    'missing_artifact_paths': len(missing), 'hash_mismatches': 0, 'new_missing_artifacts': 0,
    'historical_preservation_complete': False,
    'scope': 'Full expected map is the hash-pinned parent plus disjoint additions. All available artifacts were verified; inherited missing hashes remain in the parent. The prior schema-2 loader reconstructs this complete map.',
}
with gzip.open(E / 'artifact-audit-v1.json.gz', 'wt') as stream:
    json.dump(audit, stream, sort_keys=True)
assert load(E / 'artifact-audit-v1.json.gz') == (expected, missing)

work['minkowski_contracts_work'] = {
    'status': 'represented-set Minkowski sums and supporting exact scalar proofs qualified locally',
    'baseline_commit': 'adef9a64a',
    'contract': PREFIX + 'implementation-contract-v2.md',
    'baseline': PREFIX + 'baseline-findings-v1.md',
    'progress': PREFIX + 'progress-v1.md',
    'validation': PREFIX + 'qualification-summary-v1.json',
    'checkpoint': PREFIX + 'checkpoint-v1.json',
    'artifact_audit': PREFIX + 'artifact-audit-v1.json.gz', 'artifact_audit_schema': 2,
    'implementation_sha256': implementation, 'test_totals': summary['test_totals'],
    'cost': {'rows': 104, 'timing_rows': 78, 'allocation_rows': 26,
             'costs': summary['costs'], 'library_bytes': summary['library_bytes'],
             'source_sizes': summary['source_sizes'], 'scope': summary['scope'],
             'limits': summary['limitations']},
    'semantics': [
        'Native supporting-plane cells and signed winding preserve disconnected gaps, concavity, cavities, reversal, multiplicity and cancellation; native binary unions accumulate convex pair sums.',
        'Both operands validate before empty success; unused positions do not change the solid. Full dimension has a separate proof and hull failure never becomes empty.',
        'Independent exact volumes, witness membership, affine/repeated composition, topology, tiny gaps, typed failures, limits and recovery pass under the declared policies.',
        'Bounded constant-square and selected eighth-angle scalar proofs certify the original unchanged 16x8 sphere example; symbolic serialization remains intact.',
        'Explicit minkowski_hull preserves the old point-set intent; Rust and JavaScript expose the migration, local certainty and typed errors.',
        'The original cube-sum ordered output remains byte-identical. Scalar proof expansion changes completion and carries no same-completion speedup claim.',
    ],
    'diagnostics': PREFIX + 'diagnostic-findings-v1.md',
    'historical_preservation': '27093 inherited missing paths retain their expected hashes through the pinned parent audit; no new missing artifact or mismatch.',
    'remaining': 'Native 26-input batch union, inherited provenance, complete internal resource contracts, clean external release, full CI/formal/fuzz/coverage and 35 separately unadmitted native files remain open.',
    'full_plan_complete': False,
}
updated = raw[:start] + json.dumps(work, indent=2) + raw[end:]
ustart, uend, _ = span(updated)
assert (updated[:ustart], updated[uend:]) == (reference[:rstart], reference[rend:])
assert {k: v for k, v in json.loads(updated).items() if k != 'current_work'} == {
    k: v for k, v in json.loads(reference).items() if k != 'current_work'}
ledger.write_text(updated)
files = {str(path.relative_to(ROOT)): sha(path) for path in E.rglob('*')
         if path.is_file() and path.name != 'checkpoint-v1.json'}
record = {
    'status': 'focused Minkowski and scalar proof admission; full plan remains open',
    'previous_closure': str(previous_path.relative_to(ROOT)), 'previous_closure_sha256': sha(previous_path),
    'evidence_files_sha256': files,
    'artifact_audit': PREFIX + 'artifact-audit-v1.json.gz',
    'artifact_audit_sha256': sha(E / 'artifact-audit-v1.json.gz'), 'artifact_audit_schema': 2,
    'expected_artifact_paths': len(expected), 'verified_artifact_paths': len(expected) - len(missing),
    'missing_artifact_paths': len(missing), 'historical_preservation_complete': False,
    'implementation_sha256': implementation, 'inherited_implementation_sha256': inherited,
    'extra_unchanged_inputs_sha256': extra_inputs, 'pending_native_sha256': pending,
    'live_source_dependency_pins_sha256': dependency_pins,
    'source_guard_scope': 'Final production/test/asset, scalar fixtures and standalone Rust sources match the 11-command final boundary/benchmark phase and resource trials. Three pinned WASM/test/README changes follow the broad phase; all scalar/native sources remain identical to its passing checks. The real 60-case failing count-boundary reproduction and every failed phase remain retained.',
    'test_totals': summary['test_totals'], 'phases': summary['phases'],
    'ledger_before_sha256': before, 'ledger_after_sha256': sha(ledger),
    'only_current_work_changed': True, 'full_plan_complete': False,
}
(E / 'checkpoint-v1.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({k: record[k] for k in ['expected_artifact_paths', 'verified_artifact_paths',
                                       'missing_artifact_paths', 'full_plan_complete']}), flush=True)
