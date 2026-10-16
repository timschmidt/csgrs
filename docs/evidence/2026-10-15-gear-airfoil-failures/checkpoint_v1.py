"""Close E16 evidence, preserving every inherited expectation and ledger byte."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
memo = {}


def sha(path):
    path = Path(path)
    stat = path.stat()
    key = (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns)
    if key not in memo:
        with path.open('rb') as stream:
            memo[key] = hashlib.file_digest(stream, 'sha256').hexdigest()
    return memo[key]


def read_gzip(path):
    with gzip.open(path, 'rt') as stream:
        return json.load(stream)


def dump(path, value):
    assert not path.exists(), path
    path.write_text(json.dumps(value, indent=2) + '\n')


def span(text):
    start = re.search(r'"current_work"\s*:\s*', text).end()
    value, length = json.JSONDecoder().raw_decode(text[start:])
    return start, start + length, value


assert not (E / 'checkpoint-v1.json').exists()
assert not (E / 'artifact-audit-v1.json.gz').exists()
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).startswith('f1dab2248')
previous_path = E.parent / '2026-10-15-hilbert-trimming/checkpoint-v1.json'
previous = json.loads(previous_path.read_text())
assert all(sha(ROOT / p) == v for p, v in previous['evidence_files_sha256'].items())
assert sha(ROOT / previous['artifact_audit']) == previous['artifact_audit_sha256']
prior_audit = read_gzip(ROOT / previous['artifact_audit'])
assert len(prior_audit['expected_artifacts_sha256']) == 231894
assert len(prior_audit['verified_artifacts_sha256']) == 204801
pending = previous['pending_native_sha256']
assert len(pending) == 35 and all(sha(ROOT / p) == v for p, v in pending.items())

qualified = read_gzip(E / 'qualification-v1/source-hashes.json.gz')
assert read_gzip(E / 'candidate-v1/source-hashes.json.gz') == qualified
current = read_gzip(E / 'resources-v1/source-hashes.json.gz')
driver = str((E / 'resource-probe/allocation_v1.rs').relative_to(ROOT))
assert set(current) - set(qualified) == {driver}
assert all(current[name] == digest for name, digest in qualified.items())
assert all(sha(ROOT / p) == v for p, v in current.items())
baseline = read_gzip(E / 'reproduction-v1/source-hashes.json.gz')
old = read_gzip(previous_path.parent / 'candidate-v2/source-hashes.json.gz')
old_probe = str((previous_path.parent / 'baseline_probe_v1.rs').relative_to(ROOT))
new_probe = str((E / 'baseline_probe_v1.rs').relative_to(ROOT))
assert set(old) - set(baseline) == {old_probe}
assert set(baseline) - set(old) == {new_probe}
assert all(baseline[name] == old[name] for name in set(old) & set(baseline))
changed = sorted(p for p in set(baseline) | set(qualified)
                 if baseline.get(p) != qualified.get(p))
assert changed == [
    'benchmarks/rust/feature_pipeline.rs', 'benchmarks/rust/profile_primitives.rs',
    'examples/readme_renders.rs', 'readme.md', 'src/curve/native.rs', 'src/solid.rs',
    'tests/gear_airfoil_failures.rs',
], changed
inherited = previous['inherited_implementation_sha256'] | previous['implementation_sha256']
inherited = {p: v for p, v in inherited.items() if p not in changed}
assert all(sha(ROOT / p) == v for p, v in inherited.items())
extra_inputs = previous['extra_unchanged_inputs_sha256']
assert all(current[p] == digest for p, digest in extra_inputs.items())
dependency_pins = previous['live_source_dependency_pins_sha256']
assert all(sha(ROOT / p) == v for p, v in dependency_pins.items())
summary = json.loads((E / 'qualification-summary-v1.json').read_text())
assert summary['passed'] and len(summary['timing_rows']) == 100
assert len(summary['allocation_rows']) == 20 and len(summary['catalog_completion_rows']) == 4
assert (E / 'progress-v1.md').is_file()

expected = dict(prior_audit['expected_artifacts_sha256'])


def retain(name, digest):
    if name in expected:
        assert expected[name] == digest, name
    expected[name] = digest


base_runner = E.parent / '2026-10-15-factored-product-proofs/validate_v8.py'
wrappers = {sha(path) for path in E.glob('validate_v*.py')}
phases = {}
for result_path in sorted(E.glob('*/result.json')):
    folder = result_path.parent
    result = json.loads(result_path.read_text())
    protocol = json.loads((folder / 'protocol.json').read_text())
    assert protocol['runner_sha256'] == sha(base_runner)
    assert protocol['wrapper_sha256'] in wrappers
    assert protocol['base_commit'].startswith('f1dab2248')
    specification = json.loads((E / (folder.name + '.json')).read_text())
    assert protocol['commands'] == specification
    assert [row['name'] for row in result['commands']] == [row[0] for row in specification]
    assert result['passed']
    phases[folder.name] = {'passed': True, 'commands': len(result['commands'])}
    for name, digest in protocol['preserved_prior_executables_sha256'].items():
        retain(name, digest)
    for row in result['commands']:
        assert row['sources_unchanged'] and row['exit_code'] == 0
        for suffix in ['stdout', 'stderr', 'resources']:
            assert sha(folder / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
        for name, digest in row['binaries_sha256'].items():
            retain(name, digest)
assert phases == summary['phases']
targets = {ROOT / json.loads(path.read_text()) for path in E.glob('*-target.json')}
for phase in ['candidate-v1', 'qualification-v1']:
    overlay = json.loads((E / phase / 'overlay.json').read_text())
    lower_path = E / phase / 'overlay-lower-sha256.json.gz'
    assert sha(lower_path) == overlay['lower_hashes_sha256']
    lower = read_gzip(lower_path)
    assert all(sha(ROOT / p) == v for p, v in lower.items())
    for name, digest in lower.items():
        retain(name, digest)
    assert Path(overlay['target']).is_mount()
    targets.add(Path(overlay['upper']))
print('Auditing all inherited expectations, E16 targets, inputs and private overlay uppers.', flush=True)
for target in sorted(targets):
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
assert missing == prior_audit['missing_artifacts_sha256'] and len(missing) == 27093
audit = {
    'expected_artifacts_sha256': expected, 'verified_artifacts_sha256': verified,
    'missing_artifacts_sha256': missing, 'hash_mismatches': 0,
    'new_missing_artifacts': 0, 'historical_preservation_complete': False,
    'loss': 'Inherited external cleanup: 26957 historical artifacts and 136 pinned diagnostic inputs. Expected hashes remain; no recovery is claimed.',
}
with gzip.open(E / 'artifact-audit-v1.json.gz', 'wt') as stream:
    json.dump(audit, stream, sort_keys=True)
assert read_gzip(E / 'artifact-audit-v1.json.gz') == audit

prefix = str(E.relative_to(ROOT)) + '/'
sources = {name: qualified[name] for name in changed}
ledger = ROOT / 'docs/properties-status.json'
raw = ledger.read_text()
before = sha(ledger)
assert before == previous['ledger_after_sha256']
reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT, text=True)
start, end, work = span(raw)
rstart, rend, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rstart], reference[rend:])
work['gear_airfoil_failure_work'] = {
    'status': 'typed sampled gear/rack/airfoil failures and composed certainty; affected semantic and downstream checks passed',
    'baseline_commit': 'f1dab2248',
    'properties': ['P1', 'P5', 'P6', 'P8', 'P9', 'P17', 'P18', 'P19', 'P20', 'P25', 'P27', 'P35', 'P37', 'P39'],
    'contract': prefix + 'contract-v1.md', 'baseline': prefix + 'baseline-findings-v1.md',
    'progress': prefix + 'progress-v1.md', 'validation': prefix + 'qualification-summary-v1.json',
    'checkpoint': prefix + 'checkpoint-v1.json', 'artifact_audit': prefix + 'artifact-audit-v1.json.gz',
    'implementation_sha256': sources, 'test_totals': summary['test_totals'],
    'migration': 'The five gear/rack/airfoil constructors now return Result, with policy-explicit GeometryOutcome forms. All three solid gear wrappers propagate validation failures and consumed certainty. Existing sampled sets, full original resolutions and represented NACA constants/closure are preserved; analytic fidelity is not certified.',
    'semantics': summary['semantic_evidence'], 'certainty': summary['certainty_evidence'],
    'baseline_failures': summary['baseline_failures'],
    'candidate_diagnostics': summary['candidate_diagnostics'],
    'exact_original_coordinate_fixtures': 10,
    'cost': summary['cost'],
    'resources': 'All 100 timing, 20 allocation and 4 catalog completion rows retained. No optimization claim. Checked reservations/capacities do not bound all native work, coefficients, heap bytes or cancellation. Requested allocation bytes exclude allocator overhead. External 1800-second/8-GiB containment only; private /dev/shm uppers preserve the E15 baseline.',
    'historical_preservation': '27093 inherited missing paths retain expected hashes; no new missing artifacts or mismatches.',
    'remaining': 'Legacy text construction failures and remaining adapters; broader resources/provenance, external package/release, formal/fuzz/coverage and full properties obligations. The 35 pending native files remain separately unadmitted and their failed cost gates are not discharged by these root suites.',
    'full_plan_complete': False,
}
work['updated_on'] = '2026-10-15'
work['plan_complete'] = False
ledger.write_text(raw[:start] + json.dumps(work, indent=2).replace('\n', '\n  ') + raw[end:])
updated = ledger.read_text()
ustart, uend, _ = span(updated)
assert (updated[:ustart].encode(), updated[uend:].encode()) == (reference[:rstart].encode(), reference[rend:].encode())
x, y = json.loads(updated), json.loads(reference)
x.pop('current_work'); y.pop('current_work')
assert x == y
files = {str(path.relative_to(ROOT)): sha(path) for path in E.rglob('*')
         if path.is_file() and '__pycache__' not in path.parts}
record = {
    'status': 'focused sampled-profile failure and consumer qualification; full admission remains open',
    'previous_closure_sha256': sha(previous_path), 'evidence_files_sha256': files,
    'artifact_audit': prefix + 'artifact-audit-v1.json.gz', 'artifact_audit_sha256': sha(E / 'artifact-audit-v1.json.gz'),
    'expected_artifact_paths': len(expected), 'verified_artifact_paths': len(verified),
    'missing_artifact_paths': len(missing), 'historical_preservation_complete': False,
    'implementation_sha256': sources, 'inherited_implementation_sha256': inherited,
    'extra_unchanged_inputs_sha256': extra_inputs, 'pending_native_sha256': pending,
    'live_source_dependency_pins_sha256': dependency_pins,
    'source_guard_scope': 'E16 replaces the E15 standalone baseline probe in its evidence-source guard while preserving all shared production inputs. Candidate and qualification snapshots are identical. Only the additional standalone allocation driver is added by resources-v1, guarded by validate_v2.py; it has its own warning-denied builds and per-process calibration.',
    'test_totals': summary['test_totals'], 'phases': phases, 'ledger_before_sha256': before,
    'ledger_after_sha256': sha(ledger), 'only_current_work_changed': True, 'full_plan_complete': False,
}
dump(E / 'checkpoint-v1.json', record)
print(json.dumps({key: record[key] for key in ['expected_artifact_paths', 'verified_artifact_paths',
      'missing_artifact_paths', 'test_totals', 'full_plan_complete']}), flush=True)
