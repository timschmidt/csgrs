"""Close face/loft evidence with a lossless referenced audit and protected ledger."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
sys.dont_write_bytecode = True
from load_artifact_audit_v1 import load

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
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
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).startswith('67a3daae8')
previous_path = E.parent / '2026-10-16-distribution-contracts/checkpoint-v1.json'
previous = json.loads(previous_path.read_text())
assert all(sha(ROOT / p) == v for p, v in previous['evidence_files_sha256'].items())
assert sha(ROOT / previous['artifact_audit']) == previous['artifact_audit_sha256']
prior, inherited_missing = load(ROOT / previous['artifact_audit'])
assert len(prior) == 435075 and len(inherited_missing) == 27093
pending = previous['pending_native_sha256']
assert len(pending) == 35 and all(sha(ROOT / p) == v for p, v in pending.items())

summary = json.loads((E / 'qualification-summary-v1.json').read_text())
assert summary['passed'] and len(summary['timing_rows']) == 24 and len(summary['allocation_rows']) == 8
assert (E / 'progress-v1.md').is_file()
qualified = gz(E / 'qualification-v1/source-hashes.json.gz')
current = gz(E / 'resources-v1/source-hashes.json.gz')
final_docs = gz(E / 'cost-build-v1/source-hashes.json.gz')
assert final_docs == current
assert {p: v for p, v in qualified.items() if p != 'readme.md'} == {
    p: v for p, v in current.items() if p != 'readme.md'}
doc_adjustment = json.loads((E / 'documentation-adjustment-v1.json').read_text())
assert qualified['readme.md'] == doc_adjustment['before_sha256']
assert current['readme.md'] == doc_adjustment['after_sha256']
assert all(sha(ROOT / p) == v for p, v in current.items())
baseline = gz(E / 'reproduction-v1/source-hashes.json.gz')
old = gz(previous_path.parent / 'resources-v1/source-hashes.json.gz')
old_prefix = str(previous_path.parent.relative_to(ROOT)) + '/'
assert {p: v for p, v in baseline.items() if not p.startswith(PREFIX)} == {
    p: v for p, v in old.items() if not p.startswith(old_prefix)}
changed = sorted(p for p in set(baseline) | set(current)
                 if not p.startswith(PREFIX) and baseline.get(p) != current.get(p))
assert changed == sorted([
    'src/solid.rs', 'crates/hypertri/src/earcut.rs', 'crates/hypertri/src/lib.rs',
    'crates/hypertri/tests/simple_ring.rs', 'tests/polyhedral_construction.rs',
    'crates/hypertri/README.md', 'readme.md', 'tests/ffi/polyhedral_construction.c',
    'tests/ffi/polyhedral_construction.cpp', 'tests/ffi/polyhedral_construction.py',
    'bindings/go/polyhedral_construction_test.go',
]), changed
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

targets = {ROOT / json.loads(path.read_text()) for path in E.glob('*-target.json')}
base_runner = E.parent / '2026-10-15-factored-product-proofs/validate_v8.py'
wrapper = sha(E / 'validate_v1.py')
for name, phase in summary['phases'].items():
    folder = E / name
    protocol = json.loads((folder / 'protocol.json').read_text())
    assert protocol['runner_sha256'] == sha(base_runner)
    assert protocol['wrapper_sha256'] == wrapper
    assert protocol['base_commit'].startswith('67a3daae8')
    specification = json.loads((E / (name + '.json')).read_text())
    assert protocol['commands'] == specification
    for path, digest in protocol['preserved_prior_executables_sha256'].items():
        retain(path, digest)
    if phase.get('incomplete'):
        assert name == 'baseline-cost-v1' and (folder / 'result.json').stat().st_size == 0
    else:
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

print('Auditing every inherited artifact, new target and immutable overlay lower.', flush=True)
for target in sorted(targets):
    assert target.is_dir(), target
    for path in target.rglob('*'):
        assert not path.is_symlink(), path
        if path.is_file(): retain(str(path), sha(path))
for path in Path('/dev/shm/csgrs-polyhedral-construction').glob('*-driver.log'):
    retain(str(path), sha(path))
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
    'schema_version': 2,
    'parent_audit': previous['artifact_audit'],
    'parent_audit_sha256': previous['artifact_audit_sha256'],
    'added_artifacts_sha256': added,
    'expected_artifact_paths': len(expected),
    'verified_artifact_paths': len(expected) - len(missing),
    'missing_artifact_paths': len(missing),
    'hash_mismatches': 0, 'new_missing_artifacts': 0,
    'historical_preservation_complete': False,
    'scope': 'Full expected map is the hash-pinned parent plus disjoint additions. Every available artifact was verified; inherited missing hashes remain in the parent. See artifact-format-v1.md.',
}
with gzip.open(E / 'artifact-audit-v1.json.gz', 'wt') as stream:
    json.dump(audit, stream, sort_keys=True)
assert load(E / 'artifact-audit-v1.json.gz') == (expected, missing)

ledger = ROOT / 'docs/properties-status.json'
raw = ledger.read_text()
before = sha(ledger)
assert before == previous['ledger_after_sha256']
reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT, text=True)
start, end, work = span(raw)
rstart, rend, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rstart], reference[rend:])
work['polyhedral_construction_work'] = {
    'status': 'focused simple-face and closed-PWN loft repair qualified; general Minkowski semantics remain open',
    'baseline_commit': '67a3daae8',
    'contract': PREFIX + 'implementation-contract-v1.md',
    'baseline': PREFIX + 'baseline-findings-v1.md',
    'progress': PREFIX + 'progress-v1.md',
    'validation': PREFIX + 'qualification-summary-v1.json',
    'checkpoint': PREFIX + 'checkpoint-v1.json',
    'artifact_audit': PREFIX + 'artifact-audit-v1.json.gz',
    'artifact_audit_schema': 2,
    'implementation_sha256': implementation,
    'test_totals': summary['test_totals'],
    'cost': {'timing_pairs': summary['timing_pairs'], 'allocation_rows': 8,
             'library_bytes': summary['library_bytes'], 'source_sizes': summary['source_sizes'],
             'scope': summary['scope'], 'limits': summary['limitations']},
    'semantics': [
        'Native simple-ring validation preserves authored winding, explicit closure, collinear boundary vertices and qualified predicates; general earcut remains unchanged.',
        'Every cyclic/reversed L face and prism has independently checked orientation/volume. Lofts have volume 10, preserve the notch, validate topology and compose through empty union and complete subtraction.',
        'Consecutive pointwise-equal sections coalesce with consumed certainty; invalid rings and degenerate final meshes fail typed before publication.',
        'Strict-approximate-strict tests retain qualification and native error sources; exact rational and symbolically planar controls remain certified.',
        'Original tetrahedron, square loft and cube-sum ordered artifacts remain byte-identical; full README fixtures retain independently checked volumes.',
        'Real C/C++/Python/Go consumers preserve invalid-face diagnostics and recovery; existing WASM/TypeScript consumers pass.',
    ],
    'diagnostics': PREFIX + 'diagnostic-findings-v1.md',
    'historical_preservation': '27093 inherited missing paths retain their expected hashes through a hash-pinned parent audit. No new missing path or mismatch.',
    'remaining': 'General represented-set Minkowski sum, inherited provenance, complete internal resource contracts, clean external release, full CI/formal/fuzz/coverage and 35 separately unadmitted native files remain open.',
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
    'status': 'focused polyhedral construction and consumer qualification; full plan remains open',
    'previous_closure': str(previous_path.relative_to(ROOT)),
    'previous_closure_sha256': sha(previous_path),
    'evidence_files_sha256': files,
    'artifact_audit': PREFIX + 'artifact-audit-v1.json.gz',
    'artifact_audit_sha256': sha(E / 'artifact-audit-v1.json.gz'),
    'artifact_audit_schema': 2,
    'expected_artifact_paths': len(expected), 'verified_artifact_paths': len(expected) - len(missing),
    'missing_artifact_paths': len(missing), 'historical_preservation_complete': False,
    'implementation_sha256': implementation, 'inherited_implementation_sha256': inherited,
    'extra_unchanged_inputs_sha256': extra_inputs, 'pending_native_sha256': pending,
    'live_source_dependency_pins_sha256': dependency_pins,
    'source_guard_scope': 'All production/test/asset and standalone Rust inputs match the 40-command qualification. A later README table separator correction is pinned separately and passes final README tests, docs and fmt in cost-build-v1; every final input matches that phase and resources-v1. Failed and incomplete phases remain retained.',
    'test_totals': summary['test_totals'], 'phases': summary['phases'],
    'ledger_before_sha256': before, 'ledger_after_sha256': sha(ledger),
    'only_current_work_changed': True, 'full_plan_complete': False,
}
(E / 'checkpoint-v1.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({k: record[k] for k in ['expected_artifact_paths', 'verified_artifact_paths', 'missing_artifact_paths', 'full_plan_complete']}), flush=True)
