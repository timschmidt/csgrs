"""Close the exact-path migration while preserving every earlier expectation."""
import csv
import gzip
import hashlib
import io
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
previous_path = E.parent / '2026-10-15-adapter-failure-kinds/checkpoint-v1.json'
previous = json.loads(previous_path.read_text())
assert all(sha(ROOT / p) == v for p, v in previous['evidence_files_sha256'].items())
assert sha(ROOT / previous['artifact_audit']) == previous['artifact_audit_sha256']
prior_audit = read_gzip(ROOT / previous['artifact_audit'])
pending = previous['pending_native_sha256']
assert len(pending) == 35 and all(sha(ROOT / p) == v for p, v in pending.items())
current = read_gzip(E / 'candidate-v1/source-hashes.json.gz')
assert all(sha(ROOT / p) == v for p, v in current.items())
assert read_gzip(E / 'workloads-v1/source-hashes.json.gz') == current
old = read_gzip(previous_path.parent / 'candidate-v3/source-hashes.json.gz')
changed = sorted(p for p in set(old) | set(current) if old.get(p) != current.get(p))
assert changed == [
    'benchmarks/rust/feature_pipeline.rs', 'benchmarks/rust/profile_primitives.rs',
    'examples/readme_renders.rs', 'readme.md', 'src/curve/native.rs',
    'tests/exact_path_failures.rs',
], changed
inherited = previous['inherited_implementation_sha256'] | previous['implementation_sha256']
inherited = {p: v for p, v in inherited.items() if p not in changed}
assert all(sha(ROOT / p) == v for p, v in inherited.items())
qualification = json.loads((E / 'candidate-v1/result.json').read_text())
workloads = json.loads((E / 'workloads-v1/result.json').read_text())
assert qualification['passed'] and len(qualification['commands']) == 10
assert workloads['passed'] and len(workloads['commands']) == 3
totals = {}
for row in qualification['commands']:
    counts = [tuple(map(int, match)) for match in re.findall(
        r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;',
        (E / 'candidate-v1' / (row['name'] + '.stdout')).read_text())]
    if counts:
        assert all(count[1] == 0 for count in counts)
        totals[row['name']] = {'passed': sum(count[0] for count in counts),
                               'ignored': sum(count[2] for count in counts)}
for name, count in [('exact-paths', 6), ('root-curve', 245),
                    ('root-all-features', 352), ('root-doctests', 1)]:
    assert totals[name]['passed'] == count, (name, totals[name])
baseline = (E / 'reproduction-v3/probe.stdout').read_text()
assert baseline.count('primary_empty=true') == 3
assert 'spline degree=0: primary_absent=true checked_kind=Geometry source=false' in baseline
assert baseline.count('unresolved closure') == 3
assert baseline.count('retraced quadratic') == 2
assert baseline.count('empty=true') == 5
assert baseline.count('exact Bezier degree=') == 12
assert baseline.count('exact B-spline degree=') == 12
rows = []
for name in ['profile-workloads', 'feature-catalog']:
    rows += list(csv.DictReader(io.StringIO((E / 'workloads-v1' / (name + '.stdout')).read_text())))
assert len(rows) == 5
assert all(row['iterations'] == '1' for row in rows)
assert {row['case'] for row in rows} == {'bezier', 'bspline', 'cubic_bezier', 'cubic_region', 'catalog'}
extra_inputs = {}
for name in ['benchmarks/support/harness.rs', 'asar.ttf']:
    data = (ROOT / name).read_bytes()
    assert data == subprocess.check_output(['git', 'show', '958d2fa76:' + name], cwd=ROOT)
    extra_inputs[name] = hashlib.sha256(data).hexdigest()

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
    phases[folder.name] = {'passed': result['passed'], 'commands': len(result['commands']),
                          'failures': [(r['name'], r['exit_code']) for r in result['commands']
                                       if r['exit_code'] != 0]}
    for name, digest in protocol['preserved_prior_executables_sha256'].items():
        retain(name, digest)
    for row in result['commands']:
        assert row['sources_unchanged']
        for suffix in ['stdout', 'stderr', 'resources']:
            assert sha(folder / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
        for name, digest in row['binaries_sha256'].items():
            retain(name, digest)
assert phases['reproduction-v2']['failures'] == [('probe-build', 1), ('probe', 127)]
overlay = json.loads((E / 'candidate-v1/overlay.json').read_text())
assert sha(E / 'candidate-v1/overlay-lower-sha256.json.gz') == overlay['lower_hashes_sha256']
lower = read_gzip(E / 'candidate-v1/overlay-lower-sha256.json.gz')
assert all(sha(ROOT / p) == v for p, v in lower.items())
for name, digest in lower.items():
    retain(name, digest)
assert Path(overlay['target']).is_mount()
targets = {ROOT / json.loads(path.read_text()) for path in E.glob('*-target.json')}
targets.add(Path(overlay['upper']))
print('Auditing the current targets and every inherited artifact expectation.', flush=True)
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
    'loss': 'Inherited external cleanup: 26957 historical artifacts and 136 pinned diagnostic inputs. Expected hashes remain; no recovery is claimed.',
}
with gzip.open(E / 'artifact-audit-v1.json.gz', 'wt') as stream:
    json.dump(audit, stream, sort_keys=True)
assert read_gzip(E / 'artifact-audit-v1.json.gz') == audit

prefix = str(E.relative_to(ROOT)) + '/'
sources = {name: current[name] for name in changed}
summary = {
    'passed': True, 'qualification': prefix + 'candidate-v1/result.json',
    'consumer_workloads': prefix + 'workloads-v1/result.json', 'test_totals': totals,
    'changed_sources_from_E13': changed, 'extra_unchanged_inputs_sha256': extra_inputs,
    'extra_input_scope': 'The shared benchmark harness and embedded example font match the target Git revision. These are reconstruction pins verified at closure; the frozen runner watches benchmarks/rust but not benchmarks/support or root TTF files on every command.',
    'workload_rows': rows,
    'workload_scope': 'Original inputs/resolutions, one iteration each. These are real caller completion checks, not matched timing, independent set oracles, or fidelity/certainty certification of the entire mixed feature catalog and its legacy constructors. Semantic evidence is in exact_path_failures.rs.',
    'retained_phases': phases,
    'scope': 'Typed public exact-path and Bezier region construction, with unchanged native geometry and successful corpus. Workspace qualification only; no release/external-package/full-CI or full-plan completion claim.',
}
dump(E / 'qualification-summary-v1.json', summary)
ledger = ROOT / 'docs/properties-status.json'
raw = ledger.read_text()
before = sha(ledger)
reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT, text=True)
start, end, work = span(raw)
rstart, rend, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rstart], reference[rend:])
work['exact_path_failure_work'] = {
    'status': 'typed exact-path and closed-Bezier construction; focused semantics, downstream suites and real caller checks passed',
    'baseline_commit': '958d2fa76', 'properties': ['P6', 'P8', 'P9', 'P16', 'P18', 'P25', 'P30', 'P35', 'P37'],
    'contract': prefix + 'contract-v1.md', 'baseline': prefix + 'baseline-findings-v1.md',
    'progress': prefix + 'progress-v1.md', 'validation': prefix + 'qualification-summary-v1.json',
    'checkpoint': prefix + 'checkpoint-v1.json', 'artifact_audit': prefix + 'artifact-audit-v1.json.gz',
    'implementation_sha256': sources, 'test_totals': totals,
    'migration': 'bezier_path/bspline_path now return Result rather than Option; bezier_region returns Result rather than empty-on-error. The new Bezier path context implementation is shared with closed-region construction. Native Curve/ExactCurve causes and invalid/undecided categories survive. Existing examples, benchmark callers and docs propagate the new result.',
    'semantics': 'Preserved exact midpoint values for four Bezier families and matching polynomial splines at all original display counts; native family tags, reversal and local certainty pass. Cubic area 27/20, independent membership, union with empty and affine area 81/10 pass. A retraced quadratic remains successful Certified empty and composes. The unresolved cubic remains typed Undecided in strict/approximate/strict.',
    'resources': 'Private copy-on-write target over the unchanged E13 lower build; all lower hashes rechecked. External 1800-second/8-GiB per-process containment only. Original workload fixtures run once as completion checks, without performance claims. The /tmp quota remains full; historical targets and failed copies are retained.',
    'historical_preservation': '27093 inherited missing paths retain expected hashes; no new missing artifacts or mismatches.',
    'remaining': 'Hilbert midpoint-only clipping and lossy wrapper; legacy sampled gear/airfoil and text construction failures; remaining adapters; wider resources/provenance, external package/release, formal/fuzz/coverage and full properties obligations. The 35 pending native files remain separately unadmitted.',
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
    'status': 'focused exact-path failure and semantic qualification; full admission remains open',
    'previous_closure_sha256': sha(previous_path), 'evidence_files_sha256': files,
    'artifact_audit': prefix + 'artifact-audit-v1.json.gz', 'artifact_audit_sha256': sha(E / 'artifact-audit-v1.json.gz'),
    'expected_artifact_paths': len(expected), 'verified_artifact_paths': len(verified),
    'missing_artifact_paths': len(missing), 'historical_preservation_complete': False,
    'implementation_sha256': sources, 'inherited_implementation_sha256': inherited,
    'extra_unchanged_inputs_sha256': extra_inputs, 'pending_native_sha256': pending,
    'live_source_dependency_pins_sha256': previous['live_source_dependency_pins_sha256'],
    'test_totals': totals, 'phases': phases, 'ledger_before_sha256': before,
    'ledger_after_sha256': sha(ledger), 'only_current_work_changed': True, 'full_plan_complete': False,
}
dump(E / 'checkpoint-v1.json', record)
print(json.dumps({key: record[key] for key in ['expected_artifact_paths', 'verified_artifact_paths',
      'missing_artifact_paths', 'test_totals', 'full_plan_complete']}), flush=True)
