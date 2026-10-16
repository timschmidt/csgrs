"""Summarize completed contracts, raw failures and costs without filtering."""
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
assert len(qualification['commands']) == 48
assert [row['name'] for row in qualification['commands'] if row['exit_code']] == ['benchmark-consumers']
repair = json.loads((E / 'benchmark-repair-v1/result.json').read_text())
assert repair['passed'] and len(repair['commands']) == 11
resources = json.loads((R / 'result.json').read_text())
assert resources['passed'] and len(resources['commands']) == 12
rows = json.loads((R / 'measurements/result.json').read_text())['rows']
assert len(rows) == 104 and all(row['exit_code'] == 0 for row in rows)
timing = [row for row in rows if row['kind'] == 'timing']
allocations = [row for row in rows if row['kind'] == 'allocation']
costs = []
for group, case in sorted({(row['group'], row['case']) for row in rows}):
    for temperature in ['cold', 'warm']:
        samples = {variant: [int(row['measurement']['nanos']) for row in timing
                             if (row['variant'], row['group'], row['case'], row['temperature']) ==
                             (variant, group, case, temperature)]
                   for variant in (['candidate'] if group == 'expanded' else ['baseline', 'candidate'])}
        assert all(len(values) == 3 for values in samples.values())
        medians = {variant: statistics.median(values) for variant, values in samples.items()}
        completion = {variant: sorted({row['measurement'].get('completion', row['measurement'].get('certainty'))
                                      for row in timing if (row['variant'], row['group'], row['case']) ==
                                      (variant, group, case)}) for variant in samples}
        cost = {'group': group, 'case': case, 'temperature': temperature,
                'samples_nanos': samples, 'median_nanos': medians, 'completion': completion}
        same_completion = len(completion) == 2 and completion['baseline'] == completion['candidate']
        if same_completion:
            cost['candidate_over_baseline'] = medians['candidate'] / medians['baseline']
        cost['same_completion'] = same_completion
        costs.append(cost)

totals = {}
for phase, result in [('qualification-v1', qualification), ('benchmark-repair-v1', repair)]:
    for row in result['commands']:
        text = (E / phase / (row['name'] + '.stdout')).read_text()
        results = re.findall(r'test result: ok\. (\d+) passed; 0 failed; (\d+) ignored;', text)
        if results:
            totals[row['name']] = {'passed': sum(int(p) for p, _ in results),
                                   'ignored': sum(int(i) for _, i in results), 'phase': phase}
preedit = []
for row in json.loads((E / 'baseline-v1/result.json').read_text())['commands']:
    if row['name'].startswith(('timing-', 'allocation-')):
        data = list(csv.DictReader((E / 'baseline-v1' / (row['name'] + '.stdout')).open()))
        assert len(data) == 1
        preedit.append({'name': row['name'], 'measurement': data[0]})
assert len(preedit) == 8

phases = {}
for folder in sorted(E.iterdir()):
    if not folder.is_dir(): continue
    if (folder / 'protocol.json').exists():
        result = json.loads((folder / 'result.json').read_text())
        phases[folder.name] = {'passed': result['passed'], 'commands': len(result['commands']),
                               'failed_commands': [row['name'] for row in result['commands'] if row['exit_code']]}
    elif (folder / 'preparation.json').exists():
        phases[folder.name] = {'passed': False, 'incomplete': True,
                               'preparation': json.loads((folder / 'preparation.json').read_text())}

def expression_stats(value):
    result = {'serialized_computable_nodes': 0, 'maximum_computable_depth': 0, 'maximum_rational_bits': 0}
    def walk(v, depth=0):
        if isinstance(v, dict):
            if 'internal' in v:
                depth += 1
                result['serialized_computable_nodes'] += 1
                result['maximum_computable_depth'] = max(result['maximum_computable_depth'], depth)
            if {'sign', 'numerator', 'denominator'} <= v.keys():
                bits = sum(sum(n << (32*i) for i, n in enumerate(v[k])).bit_length()
                           for k in ['numerator', 'denominator'])
                result['maximum_rational_bits'] = max(result['maximum_rational_bits'], bits)
            for child in v.values(): walk(child, depth)
        elif isinstance(v, list):
            for child in v: walk(child, depth)
    walk(value)
    return result

structures = {path.name: expression_stats(json.loads(path.read_text()))
              for path in (E / 'fixtures').glob('*.json')}
for case in ['cube-minkowski', 'concave', 'disconnected', 'hollow']:
    row = next(row for row in rows if row['case'] == case and row['variant'] == 'candidate')
    structures[case] = expression_stats(json.loads((R / 'measurements' / (row['name'] + '.artifact.json')).read_text()))
host = json.loads((E / 'cost-host-v1.json').read_text())
summary = {
    'passed': True,
    'qualification': {'phase': 'qualification-v1', 'commands': 48, 'passed': 47,
                      'failed_harness_command': 'benchmark-consumers',
                      'replacement_phase': 'benchmark-repair-v1', 'replacement_commands_passed': 11,
                      'scope': 'The final phase checks the JavaScript numeric-count boundary fix through affected Rust/all-feature, native/WASM Clippy, docs and actual bindings, and corrects the benchmark harness path. Scalar/native sources are unchanged from the broad phase; no workload is weakened.'},
    'count_boundary': {'baseline_failed_cases': 60, 'final_invalid_count_cases': 96,
                       'source_adjustment': 'wasm-count-adjustment-v1.json',
                       'domain': 'Finite integers 0..4294967295, before unsigned conversion, under either policy and including empty operands.'},
    'resource_phase': 'resources-v1', 'resource_commands_passed': 12,
    'test_totals': totals, 'phases': phases, 'costs': costs,
    'timing_rows': timing, 'allocation_rows': allocations, 'preedit_baseline_rows': preedit,
    'library_bytes': host['library_bytes'], 'source_sizes': host['source_sizes'],
    'executable_bytes': json.loads((R / 'measurements/sizes.json').read_text()),
    'serialized_expression_structure': structures,
    'structure_scope': 'Serialized tree occurrences and rational numerator+denominator bit lengths, not unique DAG nodes or internal proof expansion peaks. Query serialization is checked unchanged; allocation/RSS evidence is separate.',
    'cube_artifact_sha256': next(row['artifact_sha256'] for row in rows if row['group'] == 'cube'),
    'scope': 'Correct represented-set sums and bounded exact proof extensions; no optimization claim. Fixed pre-edit cube workload, retrospective scalar controls/proofs, and new-only completed general sums.',
    'limitations': 'Bare meshes lack inherited provenance. Count limits/cooperative checks do not bound primitive/scalar total work or heap bytes. External 1800-second/8-GiB containment, except the WASM/Node command has wall containment only; cost children have 120-second walls. Requested Rust allocator bytes exclude native allocator internals. Native 26-input batch union, full CI, clean external release, and the full properties plan remain open.',
}
(E / 'qualification-summary-v1.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps({'test_totals': totals, 'costs': costs,
                  'allocation_rows': len(allocations), 'expression_structure': structures}, indent=2))
