"""Summarize complete trials without suppressing failed phases or adverse costs."""
import csv
import io
import hashlib
import json
from pathlib import Path
import re
import statistics

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
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
assert phases['resources-v2']['passed'] and phases['serialization-v1']['passed']
totals = {}
for name in ['truetype-only', 'hershey-only', 'root-curve', 'root-all-features', 'native-hershey', 'root-doctests']:
    rows = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', (E / qualified / (name + '.stdout')).read_text())
    assert rows and all(int(row[1]) == 0 for row in rows)
    totals[name] = {'passed': sum(int(row[0]) for row in rows), 'ignored': sum(int(row[2]) for row in rows)}
full_native = (E / 'qualification-v1/native-hershey.stdout').read_text()
assert 'test result: ok. 1760 passed; 0 failed; 1 ignored;' in full_native
totals['native-full-library'] = {'passed': 1760, 'ignored': 1}
# Full native evidence is reused only when its crate inputs remain identical.
import gzip
def sources(phase):
    with gzip.open(E / phase / 'source-hashes.json.gz', 'rt') as stream:
        return json.load(stream)
old, final = sources('qualification-v1'), sources(qualified)
assert all(final[p] == value for p, value in old.items() if p.startswith('crates/'))
measurements = json.loads((E / 'resources-v2/measurements/result.json').read_text())['rows']
earlier_measurements = json.loads((E / 'resources-v1/measurements/result.json').read_text())['rows']
assert len(earlier_measurements) == 64
assert sum(row['exit_code'] == 0 for row in earlier_measurements) == 32
for phase, rows in [('resources-v1', earlier_measurements), ('resources-v2', measurements)]:
    for row in rows:
        folder = E / phase / 'measurements'
        for suffix in ['stdout', 'stderr']:
            assert sha(folder / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
        if row['exit_code'] == 0:
            assert list(csv.DictReader(io.StringIO((folder / (row['name'] + '.stdout')).read_text()))) == [row['measurement']]
            assert sha(folder / (row['name'] + '.geometry.json')) == row['geometry_sha256']
        else:
            assert phase == 'resources-v1' and row['case'].endswith('truetype')
            assert 'original fixture has native geometry' in (folder / (row['name'] + '.stderr')).read_text()
assert len(measurements) == 64 and all(row['exit_code'] == 0 for row in measurements)
timings = [row for row in measurements if row['kind'] == 'timing']
allocations = [row for row in measurements if row['kind'] == 'allocation']
assert (len(timings), len(allocations)) == (48, 16)
ratios = []
for case in ['readme-truetype', 'bench-truetype', 'readme-hershey', 'bench-hershey']:
    for temperature in ['cold', 'warm']:
        medians = {variant: statistics.median(int(row['measurement']['nanos']) for row in timings
                   if row['case'] == case and row['temperature'] == temperature and row['variant'] == variant)
                   for variant in ['baseline', 'candidate']}
        ratios.append({'case': case, 'temperature': temperature, 'median_nanos': medians,
                       'candidate_over_baseline': medians['candidate'] / medians['baseline'],
                       'same_output_contract': 'hershey' in case})
earlier_ratios = []
for case in ['readme-hershey', 'bench-hershey']:
    for temperature in ['cold', 'warm']:
        medians = {variant: statistics.median(int(row['measurement']['nanos']) for row in earlier_measurements
                   if row['kind'] == 'timing' and row['case'] == case and row['temperature'] == temperature and row['variant'] == variant)
                   for variant in ['baseline', 'candidate']}
        earlier_ratios.append({'case': case, 'temperature': temperature, 'median_nanos': medians,
                               'candidate_over_baseline': medians['candidate'] / medians['baseline'],
                               'same_output_contract': True})
preservation = json.loads((E / 'resources-v2/measurements/preservation.json').read_text())
assert len(preservation) == 4 and all(row['byte_equal'] for row in preservation)
render_dir = ROOT / json.loads((E / (qualified + '-target.json')).read_text()) / 'text-renders'
for name in ['truetype.png', 'hershey_strings.png']:
    assert (render_dir / name).read_bytes() == (ROOT / 'docs' / name).read_bytes()
benchmark = (E / qualified / 'benchmark-text.stdout').read_text()
assert 'truetype_outline' in benchmark and 'hershey_strokes' in benchmark
cost = {
    'scope': 'Focused construction cost; all results retained, no optimization claim. TrueType migrates geometry/units; its ratios are not same-output speedups.',
    'timing_pairs': ratios,
    'earlier_successful_hershey_timing_pairs': earlier_ratios,
    'allocation_rows': 16,
    'earlier_attempts': {'phase': 'resources-v1', 'attempts': 64, 'successful_hershey_rows': 32,
        'failed_truetype_serializations': 32,
        'scope': 'Construction completed, then the helper rejected retained lines. All attempts and successful Hershey costs remain in earlier_measurement_rows; no TrueType cost is credited to failed children.'},
    'library_bytes': json.loads((E / 'cost-host-v2.json').read_text())['library_bytes'],
    'driver_sizes': json.loads((E / 'resources-v2/measurements/sizes.json').read_text()),
    'limits': 'External 1800-second/8-GiB process containment; measurement children additionally have 120-second walls. Requested allocator bytes exclude allocator overhead. No internal total-work, heap-byte or cancellation bound.',
}
record = {
    'passed': True, 'qualification_phase': qualified, 'resource_phase': 'resources-v2', 'phases': phases, 'test_totals': totals,
    'semantics': [
        'Independent cubic callback area 24000 with inside/boundary/outside witnesses; invalid callback retains glyph/scalar cause without publishing a path.',
        'Independent square and quadratic area/membership, exact 127/360 point conversion and scales 1 vs 1+2^-100, pi, and 2^+/-1100.',
        'Independent Hershey Cartesian endpoints, advances and negative/symbolic scale; every record in all 32 compiled fonts decodes.',
        'Blank geometry, malformed fonts/glyphs, missing glyphs, invalid and undecided scales remain distinct typed outcomes; dimension failure is atomic.',
        'GPOS class coverage and spacing witnesses; exact translation/Boolean empty composition; original full text workloads extrude and validate topology under both policies.',
        'STL round trips compare the represented f32 vertex set and independently validate reimported mesh topology; this does not certify original curved-font fidelity.',
        'Four original Hershey coordinate artifacts remain byte-identical; native TrueType geometry and its PNG explicitly migrate. Counts from CurveRegion2::len denote boundary loops, including holes.',
    ],
    'diagnostics': 'All failed compilation, probe and qualification phases remain in phases and raw logs; see diagnostic-findings-v1.md and gpos-findings-v1.md. No policy, input font, full-size fixture or validation was weakened.',
    'earlier_measurement_rows': earlier_measurements,
    'timing_rows': timings, 'allocation_rows': allocations, 'preservation_rows': preservation,
    'cost': cost, 'full_plan_complete': False,
    'remaining': 'Complete font-file validation, CFF2 blank certification, end-to-end provenance/resources, external package/release, full CI, formal/fuzz/coverage and 35 separately unadmitted native files remain open.',
}
OUTPUT.write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({'qualification': qualified, 'test_totals': totals, 'ratios': ratios}, indent=2))
