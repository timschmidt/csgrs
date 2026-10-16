"""Summarize all phases, exact preservation and every fixed cost trial."""
import csv
import hashlib
import io
import json
from pathlib import Path
import re
import statistics

E = Path(__file__).resolve().parent
OUTPUT = E / 'qualification-summary-v1.json'
assert not OUTPUT.exists()


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


phases = {}
for result_path in sorted(E.glob('*/result.json')):
    folder = result_path.parent
    result = json.loads(result_path.read_text())
    commands = json.loads((E / (folder.name + '.json')).read_text())
    assert [r['name'] for r in result['commands']] == [r[0] for r in commands]
    for row in result['commands']:
        assert row['sources_unchanged']
        for suffix in ['stdout', 'stderr', 'resources']:
            assert sha(folder / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
    phases[folder.name] = {'passed': result['passed'], 'commands': len(commands),
                          'failed_commands': [r['name'] for r in result['commands'] if r['exit_code']]}
qualified = sorted((p for p in phases if p.startswith('qualification-') and phases[p]['passed']),
                   key=lambda s: int(s.rsplit('v', 1)[1]))[-1]
assert phases['resources-v1']['passed'] and phases['volume-witness-v4']['passed']
totals = {}
for name in ['native-mesh', 'root-minimal', 'root-curve', 'root-all-features', 'ffi-tests', 'root-doctests', 'native-doctests']:
    rows = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', (E / qualified / (name + '.stdout')).read_text())
    assert rows and all(int(row[1]) == 0 for row in rows)
    totals[name] = {'passed': sum(int(row[0]) for row in rows), 'ignored': sum(int(row[2]) for row in rows)}
measurements = json.loads((E / 'resources-v1/measurements/result.json').read_text())['rows']
assert len(measurements) == 80 and all(row['exit_code'] == 0 for row in measurements)
for row in measurements:
    folder = E / 'resources-v1/measurements'
    for suffix in ['stdout', 'stderr']:
        assert sha(folder / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
    assert list(csv.DictReader(io.StringIO((folder / (row['name'] + '.stdout')).read_text()))) == [row['measurement']]
    assert sha(folder / (row['name'] + '.geometry.json')) == row['geometry_sha256']
timings = [r for r in measurements if r['kind'] == 'timing']
allocations = [r for r in measurements if r['kind'] == 'allocation']
assert (len(timings), len(allocations)) == (60, 20)
ratios, preservation = [], []
for case in ['linear-8', 'grid-4x4', 'arc-12', 'feature-merge-22', 'merge-duplicate']:
    original = Path('/dev/shm/csgrs-distribution-contracts/reproduction-v2/fixtures') / (case + '.json')
    for row in measurements:
        if row['case'] == case:
            assert row['geometry_sha256'] == sha(original)
            assert (E / 'resources-v1/measurements' / (row['name'] + '.geometry.json')).read_bytes() == original.read_bytes()
    preservation.append({'case': case, 'sha256': sha(original), 'bytes': original.stat().st_size,
                         'all_replays_byte_equal': True})
    for temperature in ['cold', 'warm']:
        medians = {v: statistics.median(int(r['measurement']['nanos']) for r in timings
                   if r['case'] == case and r['temperature'] == temperature and r['variant'] == v)
                   for v in ['baseline', 'candidate']}
        ratios.append({'case': case, 'temperature': temperature, 'median_nanos': medians,
                       'candidate_over_baseline': medians['candidate'] / medians['baseline']})
boxes = json.loads((E / 'resources-v1/measurements/independent-box-semantics.json').read_text())
assert len(boxes) == 8 and all(r['exact_unit_face_coverage'] for r in boxes)
benchmark_rows = []
for name in ['kernel_comparison', 'feature_pipeline']:
    benchmark_rows += list(csv.DictReader(io.StringIO((E / qualified / (name + '.stdout')).read_text())))
assert {(r['suite'], r['benchmark'], r['case'], int(r['output_size'])) for r in benchmark_rows} == {
    ('kernel', 'distribute_linear', 'box_8', 96),
    ('kernel', 'distribute_grid', 'box_4x4', 192),
    ('kernel', 'distribute_arc', 'box_12_30_degree_steps', 144),
    ('feature', 'mesh_distribution', 'arc_linear_grid', 264),
}
assert len(benchmark_rows) == 4 and all(int(r['iterations']) == 1 for r in benchmark_rows)

cost = {
    'scope': 'Fixed completed workload construction including inputs; same exact output rows. Correctness extension, no optimization or competitive speedup claim.',
    'timing_pairs': ratios, 'allocation_rows': 20,
    'library_bytes': json.loads((E / 'cost-host-v1.json').read_text())['library_bytes'],
    'source_sizes': json.loads((E / 'cost-host-v1.json').read_text())['source_sizes'],
    'driver_sizes': json.loads((E / 'resources-v1/measurements/sizes.json').read_text()),
    'limits': 'External 1800-second/8-GiB containment; cost children additionally have 120-second walls. Node/WASM has no virtual-address-space bound. Requested allocation bytes exclude allocator overhead. No internal total-work, heap-byte or cancellation contract.',
}
record = {
    'passed': True, 'qualification_phase': qualified, 'resource_phase': 'resources-v1',
    'phases': phases, 'test_totals': totals, 'cost': cost,
    'timing_rows': timings, 'allocation_rows': allocations,
    'preservation_rows': preservation, 'independent_box_semantics': boxes, 'benchmark_rows': benchmark_rows,
    'semantics': [
        'Invalid indices, malformed empties, open, degenerate and unbalanced sources fail before zero/one/disjoint shortcuts under both policies; native causes survive strict and context APIs.',
        'Canonical empty, zero-count and subtraction-to-empty results compose; signed/zero spacing and single-arc midpoint retain their exact meaning.',
        'Original 8/16/12-copy workflows satisfy independent membership, exact per-face volume proofs, strict topology, empty union and complete self subtraction.',
        'Independent exact unit-face coverage establishes the original rational 8/16/22-copy boundaries and duplicate concatenation winding multiplicity on both revisions.',
        'Touching, overlapping, balanced nonmanifold and winding-multiplicity inputs remain supported; concatenation preserves open buffers and unused points.',
        'Native PWN facts preserve combined face/edge certainty and matching buffer identity; approximate facts cannot satisfy strict requests without a new proof.',
        'Cold/warm/disabled/evicted/mutated cache states and strict-approximate-strict sequences retain errors, meanings and receipts.',
        'Count/byte overflow produces typed exhaustion before copy allocation; recovery succeeds. This is not a total resource bound.',
        'All five exact position/index artifacts remain byte-identical across all 80 completed timing/allocation replays.',
    ],
    'diagnostics': 'Every failed compilation, contained child and declined volume query remains retained; see baseline-findings-v1.md, volume-oracle-v1.md and diagnostic-findings-v1.md. No fixture resolution or predicate policy was weakened.',
    'remaining': 'Expanded arc-volume scalar recognition, inherited provenance, general work/heap/cancellation bounds, clean external package/release, full CI, formal/fuzz/coverage and 35 separately unadmitted native files remain open.',
    'full_plan_complete': False,
}
OUTPUT.write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({'qualification': qualified, 'test_totals': totals, 'ratios': ratios}, indent=2))
