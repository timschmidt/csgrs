"""Close batch evidence and edit only the ledger's raw current_work value."""
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
OLD = E.parent / '2026-10-16-minkowski-contracts'
sys.path.insert(0, str(E.parent / '2026-10-16-polyhedral-construction'))
from load_artifact_audit_v1 import load

BASE_COMMIT = 'f74a7848805398eb62dddb167b3997662e6f90d5'
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
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == BASE_COMMIT
previous_path = OLD / 'checkpoint-v1.json'
previous = json.loads(previous_path.read_text())
assert all(sha(ROOT / path) == digest for path, digest in previous['evidence_files_sha256'].items())
assert sha(ROOT / previous['artifact_audit']) == previous['artifact_audit_sha256']
prior, inherited_missing = load(ROOT / previous['artifact_audit'])
assert len(prior) == 663576 and len(inherited_missing) == 27093
pending = previous['pending_native_sha256']
assert len(pending) == 35 and all(sha(ROOT / path) == digest for path, digest in pending.items())

ledger = ROOT / 'docs/properties-status.json'
raw = ledger.read_text()
before = sha(ledger)
assert before == previous['ledger_after_sha256']
reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT, text=True)
start, end, work = span(raw)
rstart, rend, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rstart], reference[rend:])

summary = json.loads((E / 'qualification-summary-v1.json').read_text())
assert summary['passed'] and len(summary['timing_rows']) == 66 and len(summary['allocation_rows']) == 22
assert (E / 'progress-v1.md').is_file()
qualified = gz(E / 'qualification-v1/source-hashes.json.gz')
current = gz(E / 'resources-v1/source-hashes.json.gz')
assert current == gz(E / 'fuzz-lint-repair-v1/source-hashes.json.gz')
assert qualified == gz(E / 'candidate-focused-v2/source-hashes.json.gz')
adjustment = json.loads((E / 'fuzz-lint-adjustment-v1.json').read_text())['sources']
late_changes = {path for path in set(qualified) | set(current) if qualified.get(path) != current.get(path)}
assert late_changes == set(adjustment) == {
    'crates/hypertri/fuzz/fuzz_targets/hyperreal_representations.rs',
    'crates/hypertri/fuzz/fuzz_targets/topology_invariants.rs'}
assert all(adjustment[path] == {'before_sha256': qualified[path], 'after_sha256': current[path]}
           for path in late_changes)
assert all(sha(ROOT / path) == digest for path, digest in current.items())
baseline = gz(E / 'baseline-v1/source-hashes.json.gz')
old = gz(OLD / 'resources-v1/source-hashes.json.gz')


def implementation_sources(values):
    return {path: digest for path, digest in values.items() if not path.startswith('docs/evidence/')}


assert implementation_sources(baseline) == implementation_sources(old)
changed = sorted(path for path in set(baseline) | set(current)
                 if not path.startswith('docs/evidence/') and baseline.get(path) != current.get(path))
host = json.loads((E / 'cost-host-v1.json').read_text())
assert changed == sorted(host['source_sizes']) and len(changed) == 8, changed
assert not set(changed) & pending.keys()
implementation = {path: current[path] for path in changed}
inherited = previous['inherited_implementation_sha256'] | previous['implementation_sha256']
inherited = {path: digest for path, digest in inherited.items() if path not in changed}
assert all(sha(ROOT / path) == digest for path, digest in inherited.items())
extra_inputs = previous['extra_unchanged_inputs_sha256']
assert all(current[path] == digest for path, digest in extra_inputs.items())
dependency_pins = previous['live_source_dependency_pins_sha256']
assert all(sha(ROOT / path) == digest for path, digest in dependency_pins.items())
mesh_source = 'crates/hypermesh/src/surface_arrangement.rs'
assert (ROOT / mesh_source).read_bytes() == subprocess.check_output(['git', 'show', 'HEAD:' + mesh_source], cwd=ROOT)

expected = dict(prior)


def retain(name, digest):
    if name in expected:
        assert expected[name] == digest, name
    expected[name] = digest


targets = {ROOT / json.loads((E / (name + '-target.json')).read_text())
           for name in summary['phases']}
base_runner = E.parent / '2026-10-15-factored-product-proofs/validate_v8.py'
wrappers = {sha(E / f'validate_v{number}.py') for number in [1, 2]}
for name, phase in summary['phases'].items():
    folder = E / name
    protocol = json.loads((folder / 'protocol.json').read_text())
    assert protocol['runner_sha256'] == sha(base_runner)
    assert protocol['wrapper_sha256'] in wrappers and protocol['base_commit'] == BASE_COMMIT
    specification = json.loads((E / (name + '.json')).read_text())
    assert protocol['commands'] == specification
    for path, digest in protocol['preserved_prior_executables_sha256'].items():
        retain(path, digest)
    result = json.loads((folder / 'result.json').read_text())
    assert [row['name'] for row in result['commands']] == [row[0] for row in specification]
    assert phase['passed'] == result['passed']
    for row in result['commands']:
        assert row['sources_unchanged']
        for suffix in ['stdout', 'stderr', 'resources']:
            assert sha(folder / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
        for path, digest in row['binaries_sha256'].items():
            retain(path, digest)
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
        if path.is_file():
            retain(str(path), sha(path))
for path in Path('/dev/shm/csgrs-batch-winding-contracts').glob('*.log'):
    if path.name != 'closure-v1.log':
        retain(str(path), sha(path))
missing = {}
for index, (name, digest) in enumerate(expected.items()):
    path = ROOT / name
    if path.exists():
        assert sha(path) == digest, name
    else:
        missing[name] = digest
    if index and index % 100000 == 0:
        print('Verified expectations:', index, flush=True)
assert missing == inherited_missing
added = {name: digest for name, digest in expected.items() if name not in prior}
audit = {
    'schema_version': 2, 'parent_audit': previous['artifact_audit'],
    'parent_audit_sha256': previous['artifact_audit_sha256'], 'added_artifacts_sha256': added,
    'expected_artifact_paths': len(expected), 'verified_artifact_paths': len(expected) - len(missing),
    'missing_artifact_paths': len(missing), 'hash_mismatches': 0, 'new_missing_artifacts': 0,
    'historical_preservation_complete': False,
    'scope': 'Full map is the pinned parent plus disjoint additions. All available artifacts verified; inherited missing expectations retained. The pinned schema-2 loader reconstructs the complete map.',
}
with gzip.open(E / 'artifact-audit-v1.json.gz', 'wt') as stream:
    json.dump(audit, stream, sort_keys=True)
assert load(E / 'artifact-audit-v1.json.gz') == (expected, missing)

work['batch_winding_contracts_work'] = {
    'status': 'canonical common-cell triangulation and original strict batch unions qualified locally',
    'baseline_commit': BASE_COMMIT,
    'contract': PREFIX + 'triangulation-contract-v1.md',
    'baseline': PREFIX + 'baseline-findings-v1.md',
    'diagnostics': PREFIX + 'diagnostic-findings-v1.md',
    'reasoning': PREFIX + 'canonicalization-reasoning-v1.md',
    'progress': PREFIX + 'progress-v1.md',
    'validation': PREFIX + 'qualification-summary-v1.json',
    'checkpoint': PREFIX + 'checkpoint-v1.json',
    'artifact_audit': PREFIX + 'artifact-audit-v1.json.gz', 'artifact_audit_schema': 2,
    'implementation_sha256': implementation, 'test_totals': summary['test_totals'],
    'cost': {'rows': 88, 'timing_rows': 66, 'allocation_rows': 22,
             'original_baseline_rows': 24, 'costs': summary['costs'],
             'library_bytes': summary['library_bytes'], 'source_sizes': summary['source_sizes'],
             'scope': summary['scope'], 'limits': summary['limitations']},
    'semantics': [
        'Identical protected planar cells use one ordered noncrossing edge selection through native exact constraint recovery; no incircle requirement or mesh-winding relaxation.',
        'Independent exhaustive five-point triangulations under all 120 vertex orders, three affine maps and both policies supplement three complete captured face inputs.',
        'Original 26-box volume-124 cavity and minimized five-box volume-46 union complete strictly in batch and merged-shell forms, with reversed/order variants and multiple expression roots.',
        'Exact coordinate planes, closed PWN, exact bounds and every open integer-cell witness establish the represented box sets; downstream intersection, self-difference, empty reuse, accumulation and translation remain certified.',
        'The three completed control artifacts remain byte-identical across both old-build epochs and the candidate. Affected formerly noncanonical outputs have an explicit triangulation/artifact migration.',
    ],
    'historical_preservation': '27093 inherited missing paths retain their expected hashes through the parent audit; no new missing artifact or mismatch.',
    'remaining': 'Inherited provenance, complete internal resource contracts, clean external release, full CI/formal/fuzz/coverage and 35 separately unadmitted native files remain open.',
    'full_plan_complete': False,
}
updated = raw[:start] + json.dumps(work, indent=2) + raw[end:]
ustart, uend, _ = span(updated)
assert (updated[:ustart], updated[uend:]) == (reference[:rstart], reference[rend:])
assert {key: value for key, value in json.loads(updated).items() if key != 'current_work'} == {
    key: value for key, value in json.loads(reference).items() if key != 'current_work'}
ledger.write_text(updated)
files = {str(path.relative_to(ROOT)): sha(path) for path in E.rglob('*')
         if path.is_file() and path.name != 'checkpoint-v1.json'}
record = {
    'status': 'focused batch completion admission; full plan remains open',
    'previous_closure': str(previous_path.relative_to(ROOT)), 'previous_closure_sha256': sha(previous_path),
    'evidence_files_sha256': files, 'artifact_audit': PREFIX + 'artifact-audit-v1.json.gz',
    'artifact_audit_sha256': sha(E / 'artifact-audit-v1.json.gz'), 'artifact_audit_schema': 2,
    'expected_artifact_paths': len(expected), 'verified_artifact_paths': len(expected) - len(missing),
    'missing_artifact_paths': len(missing), 'historical_preservation_complete': False,
    'implementation_sha256': implementation, 'inherited_implementation_sha256': inherited,
    'extra_unchanged_inputs_sha256': extra_inputs, 'pending_native_sha256': pending,
    'live_source_dependency_pins_sha256': dependency_pins,
    'source_guard_scope': 'Final production, non-fuzz tests, assets, fixtures and standalone Rust sources match focused-v2 and the 50-command broad phase. Two pinned fuzz-harness loop repairs follow its sole lint failure; the five-command final affected checks and resource phase match the final source snapshot. All failures and temporary diagnostics remain retained. Evidence helpers are separately hashed here.',
    'test_totals': summary['test_totals'], 'phases': summary['phases'],
    'ledger_before_sha256': before, 'ledger_after_sha256': sha(ledger),
    'only_current_work_changed': True, 'full_plan_complete': False,
}
(E / 'checkpoint-v1.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({key: record[key] for key in ['expected_artifact_paths', 'verified_artifact_paths',
                                            'missing_artifact_paths', 'full_plan_complete']}), flush=True)
