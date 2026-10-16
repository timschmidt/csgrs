"""Check the complete frozen campaign; retain every measurement and cost change."""
import csv
import hashlib
import io
import json
from pathlib import Path
import re
import statistics

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
CASES = ['involute_gear', 'cycloidal_gear', 'involute_rack', 'cycloidal_rack', 'airfoil_naca4']
OUTPUT = E / 'qualification-summary-v1.json'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def phase(name, count):
    result = json.loads((E / name / 'result.json').read_text())
    commands = json.loads((E / (name + '.json')).read_text())
    assert result['passed'] and len(result['commands']) == len(commands) == count, name
    assert [r['name'] for r in result['commands']] == [r[0] for r in commands]
    for row in result['commands']:
        assert row['exit_code'] == 0 and row['sources_unchanged'], (name, row)
        for suffix in ['stdout', 'stderr', 'resources']:
            assert sha(E / name / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
    return result


def text(phase_name, command, suffix='stdout'):
    return (E / phase_name / (command + '.' + suffix)).read_text()


def rss(command):
    match = re.search(r'Maximum resident set size \(kbytes\): (\d+)',
                      text('resources-v1', command, 'resources'))
    assert match, command
    return int(match[1])


assert not OUTPUT.exists()
phases = {name: phase(name, count) for name, count in [
    ('reproduction-v1', 3), ('candidate-v1', 3), ('qualification-v1', 11), ('resources-v1', 46),
]}
totals = {}
for phase_name in ['candidate-v1', 'qualification-v1']:
    for row in phases[phase_name]['commands']:
        counts = [tuple(map(int, match)) for match in re.findall(
            r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', text(phase_name, row['name']))]
        if counts:
            assert all(count[1] == 0 for count in counts)
            totals[row['name']] = {'passed': sum(count[0] for count in counts),
                                   'ignored': sum(count[2] for count in counts)}
assert totals == {
    'gear-airfoil-semantics': {'passed': 7, 'ignored': 0},
    'root-curve': {'passed': 260, 'ignored': 0},
    'root-all-features': {'passed': 367, 'ignored': 1},
    'root-doctests': {'passed': 1, 'ignored': 0},
}, totals
baseline = text('reproduction-v1', 'probe')
candidate = text('qualification-v1', 'public-probe')
assert baseline.count('success_empty=true') == 28
assert baseline.count(': panic') == 5
assert baseline.count('certainty=Certified') == 6
assert candidate.count('kind=InvalidInput') == 18
assert candidate.count('kind=Undecided') == 5
assert candidate.count('kind=ResourceExhausted') == 10
assert 'success_empty=true' not in candidate and ': panic' not in candidate
assert baseline.count('fixture ') == candidate.count('fixture ') == 10
oracle = 'independent rack area=15*pi/4+29*sqrt(3)/24 and four membership witnesses passed'
assert oracle in baseline and oracle in candidate
fixtures = json.loads((E / 'qualification-v1/fixture-comparison.json').read_text())
assert fixtures['passed'] and len(fixtures['rows']) == 10
for row in fixtures['rows']:
    assert row['identical']
    assert Path(row['baseline']).read_bytes() == Path(row['candidate']).read_bytes()
    for variant in ['baseline', 'candidate']:
        assert sha(row[variant]) == row[variant + '_sha256']
        assert Path(row[variant]).stat().st_size == row[variant + '_bytes']

timings, allocations, catalogs = [], [], []
for command in phases['resources-v1']['commands']:
    name = command['name']
    if not name.startswith(('timing-', 'alloc-', 'catalog-')):
        continue
    parsed = list(csv.DictReader(io.StringIO(text('resources-v1', name))))
    if name.startswith('timing-'):
        _, temperature, pair, variant = name.split('-')
        assert sorted(row['case'] for row in parsed) == sorted(CASES), name
        for row in parsed:
            assert row['engine'] == 'csgrs' and row['suite'] == 'profile_primitives'
            assert row['benchmark'] == 'constructor' and row['temperature'] == temperature
            assert int(row['sample']) == int(pair)
            assert int(row['iterations']) == (1 if temperature == 'cold' else 8)
            assert int(row['elapsed_ns']) > 0
            timings.append(row | {'pair': int(pair), 'variant': variant, 'command': name,
                'peak_process_rss_kib': rss(name),
                'nanoseconds_per_iteration': int(row['elapsed_ns']) / int(row['iterations'])})
    elif name.startswith('alloc-'):
        _, temperature, case, variant = name.split('-')
        assert len(parsed) == 1 and parsed[0]['case'] == case and case in CASES
        row = parsed[0]
        assert row['temperature'] == temperature
        converted = {key: (value if key in {'case', 'temperature'} else int(value))
                     for key, value in row.items()}
        assert converted['components'] == 1 and converted['allocations'] > 0
        assert converted['peak_additional_requested_bytes'] >= 0
        allocations.append(converted | {'variant': variant, 'command': name,
                                        'peak_process_rss_kib': rss(name)})
    else:
        assert len(parsed) == 2
        assert sorted(row['benchmark'] for row in parsed) == ['mesh_primitives', 'profile_primitives']
        for row in parsed:
            assert row['case'] == 'catalog' and row['suite'] == 'feature'
            assert row['temperature'] == 'cold' and int(row['iterations']) == 1
            catalogs.append(row | {'variant': name.removeprefix('catalog-'),
                                   'command': name, 'peak_process_rss_kib': rss(name)})
assert (len(timings), len(allocations), len(catalogs)) == (100, 20, 4)
cost = {}
for case in CASES:
    cost[case] = {}
    for temperature in ['cold', 'warm']:
        ratios = []
        for pair in range(5):
            b, c = [next(row for row in timings if row['case'] == case
                         and row['temperature'] == temperature and row['pair'] == pair
                         and row['variant'] == variant) for variant in ['baseline', 'candidate']]
            for key in ['work_units', 'output_size', 'checksum', 'iterations']:
                assert b[key] == c[key], (case, temperature, pair, key)
            ratios.append(c['nanoseconds_per_iteration'] / b['nanoseconds_per_iteration'])
        b, c = [next(row for row in allocations if row['case'] == case
                     and row['temperature'] == temperature and row['variant'] == variant)
                for variant in ['baseline', 'candidate']]
        cost[case][temperature] = {
            'paired_candidate_over_baseline_latency_ratios': ratios,
            'median_latency_ratio': statistics.median(ratios),
            'allocation_candidate_minus_baseline': {
                key: c[key] - b[key] for key in [
                    'allocations', 'reallocations', 'allocated_bytes', 'reallocated_bytes',
                    'live_delta_bytes', 'peak_additional_requested_bytes', 'retained_delta_bytes',
                ]},
        }
for benchmark in ['mesh_primitives', 'profile_primitives']:
    b, c = [next(row for row in catalogs if row['benchmark'] == benchmark and row['variant'] == variant)
            for variant in ['baseline', 'candidate']]
    for key in ['work_units', 'output_size', 'checksum', 'iterations']:
        assert b[key] == c[key], (benchmark, key)

builds = {
    'baseline': ROOT / 'target/properties/hilbert-trimming/candidate-v2/release/deps',
    'candidate': ROOT / 'target/properties/gear-airfoil-failures/qualification-v1/release/deps',
}
binary_sizes = {}
for name in ['libcsgrs.rlib', 'profile_primitives-a860b8425128ee3e', 'feature_pipeline-7fecdce69333ac9d']:
    binary_sizes[name] = {
        variant: {'path': str(build / name), 'bytes': (build / name).stat().st_size,
                  'sha256': sha(build / name)} for variant, build in builds.items()}
    binary_sizes[name]['candidate_minus_baseline_bytes'] = (
        binary_sizes[name]['candidate']['bytes'] - binary_sizes[name]['baseline']['bytes'])

summary = {
    'passed': True, 'test_totals': totals,
    'phases': {key: {'passed': value['passed'], 'commands': len(value['commands'])}
               for key, value in phases.items()},
    'baseline_failures': {'empty_profiles': 22, 'capacity_panics': 5, 'certified_empty_solids': 6},
    'candidate_diagnostics': {'invalid_input': 18, 'undecided': 5, 'resource_exhausted': 10},
    'original_exact_coordinate_fixtures': fixtures,
    'semantic_evidence': 'Independent rack area and membership, translation/empty Boolean/extrusion composition, closed-manifold topology and independent volume, represented NACA ordinate and membership, gear radial witnesses. Source-coordinate and harness checksum agreement are separate preservation checks.',
    'certainty_evidence': 'Ten full original README/benchmark profiles certify under both policies. Actual approximate validation remains Approximate512Consumed through cached rings and all three solid wrappers; strict/approximate/strict diagnostics remain distinct.',
    'timing_rows': timings, 'allocation_rows': allocations, 'catalog_completion_rows': catalogs,
    'cost': cost, 'binary_sizes': binary_sizes,
    'workload_scope': 'Fixed five alternating pairs per cold/warm condition on all five original full-size profile constructor cases. Strict-default completions; all-feature dispatch tracing. Every raw result retained. The two whole-catalog executions are completion checks, not matched cost claims.',
    'allocation_scope': 'Same standalone Rust global allocator driver on pinned baseline/candidate libraries; warning-denied builds and 4096-byte calibration in every process. Requested bytes exclude allocator overhead and allocations outside the Rust allocator. Warm allocation measurements use one construction/drop warmup; timing warmup uses eight iterations. RSS is whole-process and includes untimed work.',
    'limitations': 'Correctness/API migration, no optimization or analytic-fidelity claim. No internal total-work, coefficient-growth, heap-byte or cancellation bound. External 1800-second/8-GiB containment only. Render example built, not a full rendering campaign. No new binding operation, external package, release, full-CI or full-plan admission.',
}
OUTPUT.write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps({'test_totals': totals, 'timing_rows': len(timings),
                  'allocation_rows': len(allocations), 'catalog_rows': len(catalogs), 'cost': cost}, indent=2))
