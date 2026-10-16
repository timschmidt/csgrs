"""Close E13 only after every frozen candidate command has passed."""
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
previous_path = E.parent / '2026-10-15-factored-product-proofs/checkpoint-v1.json'
previous = json.loads(previous_path.read_text())
assert all(sha(ROOT / p) == v for p, v in previous['evidence_files_sha256'].items())
assert sha(ROOT / previous['artifact_audit']) == previous['artifact_audit_sha256']
prior_audit = read_gzip(ROOT / previous['artifact_audit'])
pending = previous['pending_native_sha256']
assert len(pending) == 35 and all(sha(ROOT / p) == v for p, v in pending.items())
assert all(sha(ROOT / p) == v for p, v in previous['implementation_sha256'].items())

phase = 'candidate-v1'
qualified = read_gzip(E / phase / 'source-hashes.json.gz')
result = json.loads((E / phase / 'result.json').read_text())
assert not result['passed'] and len(result['commands']) == 53
assert [(row['name'], row['exit_code']) for row in result['commands'] if row['exit_code']] == [('root-all-feature-clippy', 101)]
assert all(row['sources_unchanged'] for row in result['commands'])
assert 'called `.err().expect()` on a `Result` value' in (E / phase / 'root-all-feature-clippy.stderr').read_text()
final_phase = 'candidate-v3'
current = read_gzip(E / final_phase / 'source-hashes.json.gz')
assert all(sha(ROOT / p) == v for p, v in current.items())
final_result = json.loads((E / final_phase / 'result.json').read_text())
assert final_result['passed'] and len(final_result['commands']) == 6
assert set(current) == set(qualified)
assert [p for p in current if current[p] != qualified[p]] == ['tests/adapter_boolean_failures.rs']
for feature in ['minimal', 'curve', 'all-features']:
    final_typed = (E / final_phase / ('typed-errors-' + feature + '.stdout')).read_text()
    assert 'invalid_input structured=192 total=192' in final_typed
    assert 'undecided structured=32 total=32' in final_typed
old = read_gzip(previous_path.parent / 'wasm-bindings-v2/source-hashes.json.gz')
changed = sorted(p for p in set(old) | set(current) if old.get(p) != current.get(p))
expected_changes = [
    'bindings/README.md', 'bindings/go/boolean_failures_test.go',
    'crates/csgrs-ffi/README.md', 'readme.md', 'src/adapter/mesh.rs',
    'src/adapter/scalar.rs', 'tests/adapter_boolean_failures.rs',
    'tests/ffi/boolean_failures.c', 'tests/ffi/boolean_failures.cpp',
    'tests/ffi/boolean_failures.py',
]
assert changed == expected_changes, changed
typed = (E / phase / 'typed-errors.stdout').read_text()
assert 'invalid_input structured=192 total=192' in typed
assert 'undecided structured=32 total=32' in typed
for language in ['c', 'cpp', 'python']:
    assert 'Boolean statuses: 4/4' in (E / phase / (language + '.stdout')).read_text()
go = (E / phase / 'go.stdout').read_text()
assert '--- PASS: TestBooleanFailureCategoryAndRecovery' in go
assert go.count('--- PASS:') == 5
totals = {}
for row in result['commands']:
    counts = [tuple(map(int, match)) for match in re.findall(
        r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;',
        (E / phase / (row['name'] + '.stdout')).read_text())]
    if counts:
        assert all(count[1] == 0 for count in counts)
        totals[row['name']] = {'passed': sum(count[0] for count in counts),
                               'ignored': sum(count[2] for count in counts)}
for name, count in [('root-minimal', 104), ('root-curve', 239), ('root-all-features', 346)]:
    assert totals[name]['passed'] == count, (name, totals[name])

baseline = json.loads((E / 'baseline-summary-v1.json').read_text())
assert all(sha(ROOT / p) == v for p, v in baseline['binaries_sha256'].items())
assert [r['exit_code'] for r in json.loads((E / 'baseline-bindings-v1/result.json').read_text())['commands']] == [0, 0, 134, 0, 134, 1, 0, 1]
red = (E / 'reproduction-v2/typed-errors.stdout').read_text()
assert 'invalid_input structured=0 total=192' in red
assert 'undecided structured=0 total=32' in red

expected = dict(prior_audit['expected_artifacts_sha256'])


def retain(name, digest):
    if name in expected:
        assert expected[name] == digest, name
    expected[name] = digest


base_runner = previous_path.parent / 'validate_v8.py'
wrappers = {sha(path) for path in E.glob('validate_v*.py')}
phases = {}
for result_path in sorted(E.glob('*/result.json')):
    folder = result_path.parent
    rows = json.loads(result_path.read_text())
    protocol = json.loads((folder / 'protocol.json').read_text())
    assert protocol['runner_sha256'] == sha(base_runner)
    assert protocol['wrapper_sha256'] in wrappers
    phases[folder.name] = {'passed': rows['passed'], 'commands': len(rows['commands']),
                          'failures': [(r['name'], r['exit_code']) for r in rows['commands']
                                       if r['exit_code'] != 0]}
    for name, digest in protocol['preserved_prior_executables_sha256'].items():
        retain(name, digest)
    for row in rows['commands']:
        assert row['sources_unchanged']
        for suffix in ['stdout', 'stderr', 'resources']:
            assert sha(folder / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
        for name, digest in row['binaries_sha256'].items():
            retain(name, digest)

preparation = json.loads((E / 'candidate-v2/preparation.json').read_text())
assert preparation['exit_code'] == 1
assert not (E / 'candidate-v2/protocol.json').exists()
assert not (E / 'candidate-v2/result.json').exists()
assert 'Disk quota exceeded' in (E / 'candidate-v2/seed-copy.stderr').read_text()
for suffix in ['stdout', 'stderr']:
    assert sha(E / ('candidate-v2/seed-copy.' + suffix)) == preparation[suffix + '_sha256']
phases['candidate-v2'] = {'passed': False, 'commands': 0,
                          'failures': [('seed preparation: user quota', 1)]}
overlay = json.loads((E / final_phase / 'overlay.json').read_text())
assert sha(E / final_phase / 'overlay-lower-sha256.json.gz') == overlay['lower_hashes_sha256']
lower_pins = read_gzip(E / final_phase / 'overlay-lower-sha256.json.gz')
assert all(sha(ROOT / p) == v for p, v in lower_pins.items())
for name, digest in lower_pins.items():
    retain(name, digest)
assert Path(overlay['target']).is_mount()

print('Auditing E13 targets and every inherited artifact expectation.', flush=True)
targets = {ROOT / json.loads(p.read_text()) for p in E.glob('*-target.json')}
targets.add(Path(overlay['upper']))
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
assert missing == prior_audit['missing_artifacts_sha256']
assert len(missing) == 27093
audit = {
    'expected_artifacts_sha256': expected, 'verified_artifacts_sha256': verified,
    'missing_artifacts_sha256': missing, 'hash_mismatches': 0,
    'new_missing_artifacts': 0, 'historical_preservation_complete': False,
    'loss': 'Inherited external cleanup: 26957 historical artifacts and 136 pinned diagnostic inputs. Expected hashes are retained; no recovery is claimed.',
}
with gzip.open(E / 'artifact-audit-v1.json.gz', 'wt') as stream:
    json.dump(audit, stream, sort_keys=True)
assert read_gzip(E / 'artifact-audit-v1.json.gz') == audit

prefix = str(E.relative_to(ROOT)) + '/'
sources = {name: current[name] for name in expected_changes}
summary = {
    'passed': True, 'qualification_phase': phase, 'final_phase': final_phase,
    'qualification_commands': 53, 'final_commands': 6, 'test_totals': totals,
    'qualification_result_sha256': sha(E / phase / 'result.json'),
    'final_result_sha256': sha(E / final_phase / 'result.json'),
    'reused_passed_commands': [row['name'] for row in result['commands'] if row['exit_code'] == 0],
    'final_edit': 'Only the new regression helper changes .err().expect() to expect_err() to satisfy Clippy. All production, other test, binding, documentation and dependency sources are identical. The final phase reruns the affected test under minimal/curve/all-feature configurations, workspace formatting and all-feature/all-target Clippy. The original phase remains failed; its unaffected passing checks are reused.',
    'final_storage': 'candidate-v2 stopped at the /tmp user quota before any check ran; its partial copy and diagnostics remain. candidate-v3 uses a copy-on-write overlay with the prior build read-only and private upper storage in /run/user/1000. All lower file hashes remain unchanged; mount specification, upper and view artifacts are retained.',
    'typed_adapter_cases': {'invalid_input': 192, 'undecided': 32},
    'bindings': {'C': '4/4 typed undecided Booleans', 'C++': '4/4 typed undecided Booleans',
                 'Python': '4/4 typed undecided Booleans', 'Go': 'all five binding tests pass'},
    'existing_binding_regressions': 'All C/C++/Python empty composition, primitive, transform, bounds, curve constructor, planar operation and exact linear modeling consumers pass against the rebuilt shared library.',
    'source_delta_from_E12': changed,
    'unchanged_lower_layer_sources': True,
    'prior_lower_layer_qualification': 'docs/evidence/2026-10-15-factored-product-proofs/qualification-summary-v1.json',
    'scope': 'Typed Boolean error forwarding only. Native geometry, policy and certainty implementations are unchanged. Native layer and WASM consumer implementations match E12; their earlier qualification is inherited, not claimed as rerun. No performance, external-package, full-CI, release or full-plan admission.',
    'retained_failures': phases,
}
dump(E / 'qualification-summary-v1.json', summary)
ledger = ROOT / 'docs/properties-status.json'
raw = ledger.read_text()
before = sha(ledger)
reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT, text=True)
start, end, work = span(raw)
rstart, rend, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rstart], reference[rend:])
work['adapter_boolean_failure_work'] = {
    'status': 'typed native Boolean causes preserved; focused Rust and real C/C++/Python/Go qualification passed',
    'baseline_commit': '5080ba4b4', 'properties': ['P6', 'P18', 'P25', 'P30', 'P35', 'P37'],
    'progress': prefix + 'progress-v1.md', 'contract': prefix + 'contract-v2.md',
    'baseline': prefix + 'baseline-summary-v1.json',
    'validation': prefix + 'qualification-summary-v1.json',
    'checkpoint': prefix + 'checkpoint-v1.json', 'artifact_audit': prefix + 'artifact-audit-v1.json.gz',
    'implementation_sha256': sources, 'test_totals': totals,
    'migration': 'All eight ScalarMesh Boolean methods use the shared HypermeshError conversion through ValidationError::Mesh and AdapterError::Construction. Both source links and Display survive. C-family undecided status changes from generic 4 to existing 5; invalid input remains 4, including empty/malformed pairs. No successful model is published on failure.',
    'scope': '192 invalid-input and 32 undecided Rust cases across four scalar families. All four operations exercised in actual C/C++/Python/Go consumers, followed by successful empty composition and recovery. Lower-layer and real WASM sources match E12; their qualification is inherited, not rerun.',
    'resource_contract': 'External 1800-second command and 8-GiB per-process address-space containment. No new internal full-query work, heap-byte or cancellation bound; no performance claim.',
    'historical_preservation': '27093 inherited missing paths retain expected hashes; no new missing artifacts or mismatches.',
    'remaining': 'Other text-only adapter paths, wider workload/provenance/resource contracts, clean external package and release validation. The 35 pending native files remain unchanged and separately unadmitted.',
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
record = {
    'status': 'focused typed Boolean adapter and binding qualification; full admission remains open',
    'previous_closure_sha256': sha(previous_path), 'evidence_files_sha256': files,
    'artifact_audit': prefix + 'artifact-audit-v1.json.gz', 'artifact_audit_sha256': sha(E / 'artifact-audit-v1.json.gz'),
    'expected_artifact_paths': len(expected), 'verified_artifact_paths': len(verified),
    'missing_artifact_paths': len(missing), 'historical_preservation_complete': False,
    'implementation_sha256': sources, 'pending_native_sha256': pending,
    'inherited_implementation_sha256': previous['implementation_sha256'],
    'live_source_dependency_pins_sha256': previous['live_source_dependency_pins_sha256'],
    'test_totals': totals, 'phases': phases, 'ledger_before_sha256': before,
    'ledger_after_sha256': sha(ledger), 'only_current_work_changed': True, 'full_plan_complete': False,
}
dump(E / 'checkpoint-v1.json', record)
print(json.dumps({key: record[key] for key in ['expected_artifact_paths', 'verified_artifact_paths',
      'missing_artifact_paths', 'test_totals', 'full_plan_complete']}), flush=True)
