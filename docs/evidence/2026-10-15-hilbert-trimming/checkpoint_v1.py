"""Close Hilbert admission evidence without changing prior expectations."""
import csv
import gzip
import hashlib
import io
import json
from pathlib import Path
import re
import statistics
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
previous_path = E.parent / '2026-10-15-exact-path-failures/checkpoint-v1.json'
previous = json.loads(previous_path.read_text())
assert all(sha(ROOT / p) == v for p, v in previous['evidence_files_sha256'].items())
assert sha(ROOT / previous['artifact_audit']) == previous['artifact_audit_sha256']
prior_audit = read_gzip(ROOT / previous['artifact_audit'])
pending = previous['pending_native_sha256']
assert len(pending) == 35 and all(sha(ROOT / p) == v for p, v in pending.items())
current = read_gzip(E / 'candidate-v2/source-hashes.json.gz')
assert all(sha(ROOT / p) == v for p, v in current.items())
assert read_gzip(E / 'workloads-v1/source-hashes.json.gz') == current
baseline = read_gzip(E / 'reproduction-v1/source-hashes.json.gz')
old = read_gzip(previous_path.parent / 'candidate-v1/source-hashes.json.gz')
assert all(baseline[name] == digest for name, digest in old.items())
assert set(baseline) - set(old) == {
    'asar.ttf', 'benchmarks/support/harness.rs',
    str((E / 'baseline_probe_v1.rs').relative_to(ROOT)),
}
changed = sorted(p for p in set(baseline) | set(current)
                 if baseline.get(p) != current.get(p))
assert changed == [
    'benchmarks/rust/profile_primitives.rs', 'examples/readme_renders.rs',
    'readme.md', 'src/curve/native.rs', 'tests/hilbert_trimming.rs',
], changed
inherited = previous['inherited_implementation_sha256'] | previous['implementation_sha256']
inherited = {p: v for p, v in inherited.items() if p not in changed}
assert all(sha(ROOT / p) == v for p, v in inherited.items())
extra_inputs = previous['extra_unchanged_inputs_sha256']
assert all(current[p] == digest for p, digest in extra_inputs.items())
assert all(sha(ROOT / p) == v for p, v in previous['live_source_dependency_pins_sha256'].items())
qualification = json.loads((E / 'candidate-v2/result.json').read_text())
workloads = json.loads((E / 'workloads-v1/result.json').read_text())
assert qualification['passed'] and len(qualification['commands']) == 10
assert workloads['passed'] and len(workloads['commands']) == 21
totals = {}
for row in qualification['commands']:
    counts = [tuple(map(int, match)) for match in re.findall(
        r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;',
        (E / 'candidate-v2' / (row['name'] + '.stdout')).read_text())]
    if counts:
        assert all(count[1] == 0 for count in counts)
        totals[row['name']] = {'passed': sum(count[0] for count in counts),
                               'ignored': sum(count[2] for count in counts)}
for name, count in [('hilbert-semantics', 8), ('root-curve', 253),
                    ('root-all-features', 360), ('root-doctests', 1)]:
    assert totals[name]['passed'] == count, (name, totals[name])
assert totals['root-all-features']['ignored'] == 1
probe = (E / 'reproduction-v1/probe.stdout').read_text()
assert probe.count('defect: whole first leg') == 3
assert probe.count('defect: missing top-row') == 3
assert probe.count('kind=InvalidInput error=invalid arguments') == 3
assert probe.count('primary_empty=true') == 4
assert probe.count('original fixture order=') == 2

rows = []
for command in workloads['commands'][1:]:
    name = command['name']
    parsed = list(csv.DictReader(io.StringIO((E / 'workloads-v1' / (name + '.stdout')).read_text())))
    assert len(parsed) == 1
    row = parsed[0]
    temperature, pair, variant = name.split('-')
    assert row['case'] == 'hilbert_curve' and row['temperature'] == temperature
    assert int(row['iterations']) == (1 if temperature == 'cold' else 8)
    rss = re.search(r'Maximum resident set size \(kbytes\): (\d+)',
                    (E / 'workloads-v1' / (name + '.resources')).read_text())
    assert rss
    rows.append(row | {'pair': int(pair), 'variant': variant,
                       'command': name, 'peak_process_rss_kib': int(rss[1]),
                       'nanoseconds_per_iteration': int(row['elapsed_ns']) / int(row['iterations'])})
assert len(rows) == 20
cost = {}
for temperature in ['cold', 'warm']:
    ratios = []
    for pair in range(5):
        b, c = [next(row for row in rows if row['temperature'] == temperature
                     and row['pair'] == pair and row['variant'] == variant)
                for variant in ['baseline', 'candidate']]
        for key in ['work_units', 'output_size', 'checksum', 'iterations']:
            assert b[key] == c[key], (temperature, pair, key)
        ratios.append(c['nanoseconds_per_iteration'] / b['nanoseconds_per_iteration'])
    cost[temperature] = {
        'paired_candidate_over_baseline_ratios': ratios,
        'median_ratio': statistics.median(ratios),
        'latency_ns_per_iteration': {
            variant: [r['nanoseconds_per_iteration'] for r in rows
                      if r['temperature'] == temperature and r['variant'] == variant]
            for variant in ['baseline', 'candidate']},
        'peak_process_rss_kib': {
            variant: [r['peak_process_rss_kib'] for r in rows
                      if r['temperature'] == temperature and r['variant'] == variant]
            for variant in ['baseline', 'candidate']},
    }

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
    assert protocol['base_commit'].startswith('362e157c1')
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
assert phases['candidate-v1']['failures'] == [('hilbert-semantics', 101)]
targets = {ROOT / json.loads(path.read_text()) for path in E.glob('*-target.json')}
for phase in ['candidate-v1', 'candidate-v2']:
    overlay = json.loads((E / phase / 'overlay.json').read_text())
    lower_path = E / phase / 'overlay-lower-sha256.json.gz'
    assert sha(lower_path) == overlay['lower_hashes_sha256']
    lower = read_gzip(lower_path)
    assert all(sha(ROOT / p) == v for p, v in lower.items())
    for name, digest in lower.items():
        retain(name, digest)
    assert Path(overlay['target']).is_mount()
    targets.add(Path(overlay['upper']))
print('Auditing every inherited expectation and both preserved Hilbert targets.', flush=True)
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
    'passed': True, 'qualification': prefix + 'candidate-v2/result.json',
    'caller_measurements': prefix + 'workloads-v1/result.json', 'test_totals': totals,
    'changed_sources_from_E14': changed, 'extra_unchanged_inputs_sha256': extra_inputs,
    'extra_input_scope': 'The shared harness, embedded font and baseline Rust probe are included in every source guard in this phase.',
    'workload_rows': rows, 'cost': cost,
    'workload_scope': 'Fixed five alternating pairs per cold/warm condition; unchanged square-8/order-3/padding-1 caller, all-feature dispatch tracing. Exact interval/grid tests establish semantics independently of count/checksum agreement. Cost disclosure for a correctness migration, not an optimization claim. Peak RSS is whole-process, not allocation or kernel-only memory.',
    'retained_phases': phases,
    'runner_metadata_note': 'The frozen v1 wrapper left the base runner historical missing_acceptance_test list in its protocol; those E12 observations are not Hilbert evidence. contract-v1.md and this summary define this admission. The v2 workload wrapper explicitly replaces that legacy metadata.',
    'scope': 'Exact Hilbert path clipping, retained native trim evidence, fallible geometry-only strings and affected workspace consumers. Render example built; its original Hilbert input executed by semantic tests, not a full rendering campaign. No new binding entry point, external package, release, full-CI or full-plan claim.',
}
dump(E / 'qualification-summary-v1.json', summary)
ledger = ROOT / 'docs/properties-status.json'
raw = ledger.read_text()
before = sha(ledger)
reference = subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT, text=True)
start, end, work = span(raw)
rstart, rend, _ = span(reference)
assert (raw[:start], raw[end:]) == (reference[:rstart], reference[rend:])
work['hilbert_trim_work'] = {
    'status': 'exact native Hilbert clipping and visible failures; affected semantic and downstream checks passed',
    'baseline_commit': '362e157c1',
    'properties': ['P1', 'P6', 'P8', 'P9', 'P15', 'P16', 'P17', 'P18', 'P19', 'P20', 'P25', 'P29', 'P35', 'P37', 'P39'],
    'contract': prefix + 'contract-v1.md', 'baseline': prefix + 'baseline-findings-v1.md',
    'progress': prefix + 'progress-v1.md', 'validation': prefix + 'qualification-summary-v1.json',
    'checkpoint': prefix + 'checkpoint-v1.json', 'artifact_audit': prefix + 'artifact-audit-v1.json.gz',
    'implementation_sha256': sources, 'test_totals': totals,
    'migration': 'hilbert_paths and its context form expose native CurvePathRegionTrim2 evidence. hilbert_strings now returns Result and provides an exact geometry-only view when endpoints have Real coordinates; selected endpoints remain supported by the authoritative path API. Actual contacts replace midpoint-only retention, so segmentation changes intentionally.',
    'semantics': 'Independent directed intervals cover holes, disconnected material, positive-length boundary overlaps and isolated tangency under strict/approximate/strict. Exact symbolic affine coordinates, original order-3/order-4 grid coverage, source-parameter/contact replay, downstream native path use, empty composition, invalid/undecided/unsupported outcomes and qualified approximate-empty receipts pass.',
    'cost': cost,
    'resources': 'All samples retained as a correctness-migration cost disclosure; no speedup claim. The order cap and checked reservations do not bound all native work, coefficients, heap bytes or cancellation. External 1800-second/8-GiB containment only. Private /dev/shm overlay uppers preserve the E14 lower build and all failed trials.',
    'historical_preservation': '27093 inherited missing paths retain expected hashes; no new missing artifacts or mismatches.',
    'remaining': 'Legacy sampled gear/airfoil and text construction failures; remaining adapters; broader resources/provenance, external package/release, formal/fuzz/coverage and full properties obligations. The 35 pending native files remain separately unadmitted.',
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
    'status': 'focused Hilbert semantic and consumer qualification; full admission remains open',
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
