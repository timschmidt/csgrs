"""Close text evidence without changing inherited ledger bytes or expectations."""
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
    s = path.stat()
    key = (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns)
    if key not in memo:
        with path.open('rb') as stream:
            memo[key] = hashlib.file_digest(stream, 'sha256').hexdigest()
    return memo[key]

def gz(path):
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
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).startswith('72cd216cc')
previous_path = E.parent / '2026-10-15-gear-airfoil-failures/checkpoint-v1.json'
previous = json.loads(previous_path.read_text())
assert all(sha(ROOT / p) == v for p, v in previous['evidence_files_sha256'].items())
assert sha(ROOT / previous['artifact_audit']) == previous['artifact_audit_sha256']
prior_audit = gz(ROOT / previous['artifact_audit'])
assert len(prior_audit['expected_artifacts_sha256']) == 260556
assert len(prior_audit['verified_artifacts_sha256']) == 233463
pending = previous['pending_native_sha256']
assert len(pending) == 35 and all(sha(ROOT / p) == v for p, v in pending.items())
summary = json.loads((E / 'qualification-summary-v1.json').read_text())
assert summary['passed'] and len(summary['timing_rows']) == 48 and len(summary['allocation_rows']) == 16
assert (E / 'progress-v1.md').is_file()
qualified = gz(E / summary['qualification_phase'] / 'source-hashes.json.gz')
current = gz(E / summary['resource_phase'] / 'source-hashes.json.gz')
prefix = str(E.relative_to(ROOT)) + '/'
assert all(current.get(p) == v for p, v in qualified.items() if not p.startswith(prefix))
assert all(sha(ROOT / p) == v for p, v in current.items())
baseline = gz(E / 'reproduction-v1/source-hashes.json.gz')
old = gz(previous_path.parent / 'resources-v1/source-hashes.json.gz')
assert all(baseline[p] == old[p] for p in set(old) & set(baseline))
assert set(old) - set(baseline) == {
    str((previous_path.parent / 'baseline_probe_v1.rs').relative_to(ROOT)),
    str((previous_path.parent / 'resource-probe/allocation_v1.rs').relative_to(ROOT)),
}
assert set(baseline) - set(old) == {prefix + 'baseline_probe_v1.rs'}
# The image guard was added during this work; establish the old values from HEAD.
for name in ['docs/truetype.png', 'docs/hershey_strings.png']:
    baseline[name] = hashlib.sha256(subprocess.check_output(['git', 'show', 'HEAD:' + name], cwd=ROOT)).hexdigest()
changed = sorted(p for p in set(baseline) | set(qualified)
                 if not p.startswith(prefix) and baseline.get(p) != qualified.get(p))
assert changed == [
    'Cargo.lock', 'Cargo.toml', 'benchmarks/rust/feature_pipeline.rs',
    'crates/hypercurve/README.md', 'crates/hypercurve/src/hershey.rs', 'docs/truetype.png',
    'examples/README.md', 'examples/assembly_manual.rs', 'examples/readme_renders.rs',
    'readme.md', 'src/curve/native.rs', 'src/curve/truetype.rs', 'src/errors.rs',
    'src/parts/dimension.rs', 'src/solid.rs', 'tests/support/text_font.rs', 'tests/text_contracts.rs',
], changed
inherited = previous['inherited_implementation_sha256'] | previous['implementation_sha256']
inherited = {p: v for p, v in inherited.items() if p not in changed}
assert all(sha(ROOT / p) == v for p, v in inherited.items())
extra_inputs = previous['extra_unchanged_inputs_sha256'] | {'docs/hershey_strings.png': baseline['docs/hershey_strings.png']}
assert all(current[p] == v for p, v in extra_inputs.items())
assert baseline['Cargo.lock'] == previous['live_source_dependency_pins_sha256']['Cargo.lock']
dependency_pins = previous['live_source_dependency_pins_sha256'] | {'Cargo.lock': current['Cargo.lock']}
assert all(sha(ROOT / p) == v for p, v in dependency_pins.items())

expected = dict(prior_audit['expected_artifacts_sha256'])
def retain(name, digest):
    if name in expected:
        assert expected[name] == digest, name
    expected[name] = digest

base_runner = E.parent / '2026-10-15-factored-product-proofs/validate_v8.py'
wrappers = {sha(path) for path in E.glob('validate_v*.py')}
phases = {}
targets = {ROOT / json.loads(path.read_text()) for path in E.glob('*-target.json')}
for result_path in sorted(E.glob('*/result.json')):
    folder = result_path.parent
    result = json.loads(result_path.read_text())
    protocol = json.loads((folder / 'protocol.json').read_text())
    assert protocol['runner_sha256'] == sha(base_runner)
    assert protocol['wrapper_sha256'] in wrappers
    assert protocol['base_commit'].startswith('72cd216cc')
    specification = json.loads((E / (folder.name + '.json')).read_text())
    assert protocol['commands'] == specification
    assert [row['name'] for row in result['commands']] == [row[0] for row in specification]
    phases[folder.name] = {'passed': result['passed'], 'commands': len(result['commands']),
                          'failed_commands': [row['name'] for row in result['commands'] if row['exit_code']]}
    for name, digest in protocol['preserved_prior_executables_sha256'].items():
        retain(name, digest)
    for row in result['commands']:
        assert row['sources_unchanged']
        for suffix in ['stdout', 'stderr', 'resources']:
            assert sha(folder / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
        for name, digest in row['binaries_sha256'].items():
            retain(name, digest)
    if (folder / 'overlay.json').exists():
        overlay = json.loads((folder / 'overlay.json').read_text())
        lower_file = folder / 'overlay-lower-sha256.json.gz'
        assert sha(lower_file) == overlay['lower_hashes_sha256']
        for name, digest in gz(lower_file).items():
            assert sha(ROOT / name) == digest, name
            retain(name, digest)
        assert Path(overlay['target']).is_mount()
        targets.add(Path(overlay['upper']))
assert phases == summary['phases']
print('Auditing inherited artifacts, every text target and each private overlay upper.', flush=True)
for target in sorted(targets):
    assert target.is_dir(), target
    for path in target.rglob('*'):
        assert not path.is_symlink(), path
        if path.is_file(): retain(str(path), sha(path))
verified, missing = {}, {}
for name, digest in expected.items():
    path = ROOT / name
    if path.exists():
        assert sha(path) == digest, name
        verified[name] = digest
    else:
        missing[name] = digest
assert missing == prior_audit['missing_artifacts_sha256'] and len(missing) == 27093
audit = {'expected_artifacts_sha256': expected, 'verified_artifacts_sha256': verified,
         'missing_artifacts_sha256': missing, 'hash_mismatches': 0, 'new_missing_artifacts': 0,
         'historical_preservation_complete': False,
         'loss': 'Inherited cleanup: 26957 historical artifacts and 136 pinned diagnostic inputs; expected hashes retained, no recovery claimed.'}
with gzip.open(E / 'artifact-audit-v1.json.gz', 'wt') as stream:
    json.dump(audit, stream, sort_keys=True)
assert gz(E / 'artifact-audit-v1.json.gz') == audit

implementation = {p: current[p] for p in changed}
ledger = ROOT / 'docs/properties-status.json'
raw = ledger.read_text()
before = sha(ledger)
assert before == previous['ledger_after_sha256']
reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT, text=True)
start, end, work = span(raw)
rstart, rend, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rstart], reference[rend:])
work['text_contract_work'] = {
    'status': 'focused typed text construction, exact native outlines and affected consumers qualified',
    'baseline_commit': '72cd216cc',
    'properties': ['P1', 'P4', 'P5', 'P6', 'P8', 'P9', 'P12', 'P13', 'P14', 'P17', 'P18', 'P19', 'P20', 'P25', 'P26', 'P27', 'P29', 'P35', 'P36', 'P37', 'P39'],
    'contract': prefix + 'contract-v1.md', 'migration': prefix + 'implementation-contract-v1.md',
    'baseline': prefix + 'baseline-findings-v1.md', 'progress': prefix + 'progress-v1.md',
    'validation': prefix + 'qualification-summary-v1.json', 'checkpoint': prefix + 'checkpoint-v1.json',
    'artifact_audit': prefix + 'artifact-audit-v1.json.gz', 'implementation_sha256': implementation,
    'test_totals': summary['test_totals'], 'cost': summary['cost'],
    'semantics': summary['semantics'], 'diagnostics': summary['diagnostics'],
    'resources': 'Checked owned buffers, 48 timing rows and 16 allocation rows in the completed cost phase; all 64 earlier attempts, including 32 successful Hershey measurements, also retained; no total-work, heap-byte or cancellation bound. TrueType output migrates; no performance-only claim.',
    'historical_preservation': '27093 inherited missing artifact paths retain expected hashes; no new loss or mismatch.',
    'remaining': 'Full font-file validation, wider symbolic/native consumers, end-to-end provenance and resource contracts, external release, formal/fuzz/coverage and other properties obligations remain open. The 35 pending native files remain separately unadmitted.',
    'full_plan_complete': False,
}
work['updated_on'] = '2026-10-16'
work['plan_complete'] = False
ledger.write_text(raw[:start] + json.dumps(work, indent=2).replace('\n', '\n  ') + raw[end:])
updated = ledger.read_text()
ustart, uend, _ = span(updated)
assert (updated[:ustart].encode(), updated[uend:].encode()) == (reference[:rstart].encode(), reference[rend:].encode())
x, y = json.loads(updated), json.loads(reference)
x.pop('current_work'); y.pop('current_work')
assert x == y
files = {str(path.relative_to(ROOT)): sha(path) for path in E.rglob('*') if path.is_file() and '__pycache__' not in path.parts}
record = {
    'status': 'focused text contracts and consumer qualification; full admission remains open',
    'previous_closure_sha256': sha(previous_path), 'evidence_files_sha256': files,
    'artifact_audit': prefix + 'artifact-audit-v1.json.gz', 'artifact_audit_sha256': sha(E / 'artifact-audit-v1.json.gz'),
    'expected_artifact_paths': len(expected), 'verified_artifact_paths': len(verified), 'missing_artifact_paths': len(missing),
    'historical_preservation_complete': False, 'implementation_sha256': implementation,
    'inherited_implementation_sha256': inherited, 'extra_unchanged_inputs_sha256': extra_inputs,
    'pending_native_sha256': pending, 'live_source_dependency_pins_sha256': dependency_pins,
    'source_guard_scope': 'All production/test/asset inputs match final qualification. Standalone resource-driver revisions are separately guarded and compiled. All earlier failed phases and immutable lower builds remain retained.',
    'test_totals': summary['test_totals'], 'phases': phases, 'ledger_before_sha256': before,
    'ledger_after_sha256': sha(ledger), 'only_current_work_changed': True, 'full_plan_complete': False,
}
dump(E / 'checkpoint-v1.json', record)
print(json.dumps({k: record[k] for k in ['expected_artifact_paths', 'verified_artifact_paths', 'missing_artifact_paths', 'test_totals', 'full_plan_complete']}), flush=True)
