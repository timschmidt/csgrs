"""Summarize retained qualification and fixed cost evidence without filtering failures."""
import csv
import hashlib
import json
from pathlib import Path
import re
import statistics

E = Path(__file__).resolve().parent
Q = E / 'qualification-v1'
R = E / 'resources-v1'
assert not (E / 'qualification-summary-v1.json').exists()
qualification = json.loads((Q / 'result.json').read_text())
resources = json.loads((R / 'result.json').read_text())
assert qualification['passed'] and len(qualification['commands']) == 40
assert resources['passed'] and len(resources['commands']) == 8
assert json.loads((E / 'cost-build-v1/result.json').read_text())['passed']
rows = json.loads((R / 'measurements/result.json').read_text())['rows']
assert len(rows) == 32 and all(row['exit_code'] == 0 for row in rows)
timing = [row for row in rows if row['kind'] == 'timing']
allocations = [row for row in rows if row['kind'] == 'allocation']
pairs = []
for case in ['tetrahedron', 'loft-square']:
    for temperature in ['cold', 'warm']:
        samples = {variant: [int(row['measurement']['nanos']) for row in timing
                            if (row['variant'], row['case'], row['temperature']) ==
                            (variant, case, temperature)] for variant in ['baseline', 'candidate']}
        assert all(len(values) == 3 for values in samples.values())
        medians = {variant: statistics.median(values) for variant, values in samples.items()}
        pairs.append({'case': case, 'temperature': temperature, 'samples_nanos': samples,
                      'median_nanos': medians,
                      'candidate_over_baseline': medians['candidate'] / medians['baseline']})
totals = {}
for name in ['native-triangulation', 'native-mesh', 'root-minimal', 'root-curve',
             'root-all-features', 'ffi-tests', 'root-doctests', 'native-doctests',
             'triangulation-doctests']:
    results = re.findall(r'test result: ok\. (\d+) passed; 0 failed; (\d+) ignored;',
                         (Q / (name + '.stdout')).read_text())
    assert results, name
    totals[name] = {'passed': sum(int(p) for p, _ in results),
                    'ignored': sum(int(i) for _, i in results)}

replay = Path('/dev/shm/csgrs-polyhedral-construction/resources-v1/fixtures')
baseline = Path('/dev/shm/csgrs-polyhedral-construction/reproduction-v1/fixtures')
preserved = {}
for case in ['tetrahedron', 'loft-square', 'cube-minkowski']:
    before = (baseline / (case + '.json')).read_bytes()
    after = (replay / (case + '.json')).read_bytes()
    assert before == after, case
    preserved[case] = hashlib.sha256(after).hexdigest()
preedit = json.loads((E / 'baseline-cost-v2/measurements.json').read_text())
assert len(preedit) == 16 and all(row['exit_code'] == 0 for row in preedit)
preedit_rows = []
for row in preedit:
    data = list(csv.DictReader((E / 'baseline-cost-v2' / (row['name'] + '.stdout')).open()))
    assert len(data) == 1
    preedit_rows.append({'name': row['name'], 'measurement': data[0]})
phases = {}
for folder in sorted(E.iterdir()):
    if not folder.is_dir() or not (folder / 'protocol.json').exists(): continue
    result = folder / 'result.json'
    if result.stat().st_size == 0:
        assert folder.name == 'baseline-cost-v1'
        phases[folder.name] = {'passed': False, 'incomplete': True,
                              'reason': 'Runner exited 1 with truncated result; raw child evidence retained.'}
    else:
        data = json.loads(result.read_text())
        phases[folder.name] = {'passed': data['passed'], 'commands': len(data['commands']),
                              'failed_commands': [row['name'] for row in data['commands'] if row['exit_code']]}
host = json.loads((E / 'cost-host-v1.json').read_text())
summary = {
    'passed': True, 'qualification_phase': 'qualification-v1', 'resource_phase': 'resources-v1',
    'test_totals': totals, 'phases': phases, 'timing_rows': timing, 'allocation_rows': allocations,
    'timing_pairs': pairs, 'preedit_baseline_rows': preedit_rows,
    'preserved_original_artifacts': preserved,
    'library_bytes': host['library_bytes'], 'source_sizes': host['source_sizes'],
    'driver_sizes': json.loads((R / 'measurements/sizes.json').read_text()),
    'scope': 'Face and loft correctness repair; fixed completed construction costs with exact output preservation. No optimization or full-plan claim.',
    'limitations': 'External 1800-second/8-GiB process containment (Node/WASM has no virtual-address-space bound); cost children additionally have 120-second walls. Requested allocator bytes exclude overhead. No internal total-work, heap-byte or cancellation contract.',
}
(E / 'qualification-summary-v1.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps({'test_totals': totals, 'timing_pairs': pairs,
                  'allocation_rows': [{k: v for k, v in row.items() if k in ['case', 'temperature', 'variant', 'measurement']} for row in allocations]}, indent=2))
