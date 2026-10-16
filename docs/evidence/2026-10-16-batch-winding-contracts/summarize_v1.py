"""Summarize all frozen phases and both baseline epochs without filtering."""
import csv
import hashlib
import json
from pathlib import Path
import re
import statistics

E = Path(__file__).resolve().parent
assert not (E / 'qualification-summary-v1.json').exists()
qualification = json.loads((E / 'qualification-v1/result.json').read_text())
assert len(qualification['commands']) == 50 and not qualification['passed']
assert [row['name'] for row in qualification['commands'] if row['exit_code']] == ['hypertri-fuzz-clippy']
repair = json.loads((E / 'fuzz-lint-repair-v1/result.json').read_text())
assert repair['passed'] and len(repair['commands']) == 5
resources = json.loads((E / 'resources-v1/result.json').read_text())
assert resources['passed'] and len(resources['commands']) == 6
replay = json.loads((E / 'resources-v1/measurements/result.json').read_text())['rows']
initial = json.loads((E / 'baseline-probes-v1/measurements/result.json').read_text())['rows']
assert len(replay) == 64 and len(initial) == 24
rows = [dict(row, epoch='original', variant='baseline') for row in initial]
rows += [dict(row, epoch='replay') for row in replay]
assert all(row['exit_code'] == 0 and 'measurement_error' not in row for row in rows)
timing = [row for row in rows if row['kind'] == 'timing']
allocations = [row for row in rows if row['kind'] == 'allocation']
assert len(timing) == 66 and len(allocations) == 22
costs = []
for case in ['overlap', 'coincident', 'touching', 'full26', 'merged26']:
    for temperature in ['cold', 'warm']:
        variants = [('replay', 'candidate')]
        if case in ['overlap', 'coincident', 'touching']:
            variants = [('original', 'baseline'), ('replay', 'baseline')] + variants
        samples = {f'{epoch}-{variant}': [int(row['measurement']['nanos']) for row in timing
                                         if (row['epoch'], row['variant'], row['case'], row['temperature']) ==
                                         (epoch, variant, case, temperature)]
                   for epoch, variant in variants}
        assert all(len(values) == 3 for values in samples.values())
        medians = {name: statistics.median(values) for name, values in samples.items()}
        cost = {'case': case, 'temperature': temperature, 'samples_nanos': samples,
                'median_nanos': medians, 'completion': 'certified_local',
                'same_completion_baseline_available': len(variants) == 3}
        if len(variants) == 3:
            cost['candidate_over_original_baseline'] = medians['replay-candidate'] / medians['original-baseline']
            cost['candidate_over_replayed_baseline'] = medians['replay-candidate'] / medians['replay-baseline']
        costs.append(cost)

phases, totals = {}, {}
for folder in sorted(E.iterdir()):
    if not folder.is_dir() or not (folder / 'protocol.json').exists():
        continue
    result = json.loads((folder / 'result.json').read_text())
    specification = json.loads((E / (folder.name + '.json')).read_text())
    assert len(result['commands']) == len(specification)
    phases[folder.name] = {'passed': result['passed'], 'commands': len(result['commands']),
                           'failed_commands': [row['name'] for row in result['commands'] if row['exit_code']]}
    for row in result['commands']:
        text = (folder / (row['name'] + '.stdout')).read_text()
        results = re.findall(r'test result: (\w+)\. (\d+) passed; (\d+) failed; (\d+) ignored;', text)
        if results:
            totals[folder.name + '/' + row['name']] = {
                'passed': sum(int(p) for _, p, _, _ in results),
                'failed': sum(int(f) for _, _, f, _ in results),
                'ignored': sum(int(i) for _, _, _, i in results),
                'test_binaries_or_doctest_groups': len(results),
            }
assert set(name for name, phase in phases.items() if not phase['passed']) == {
    'triangulation-baseline-v1', 'candidate-focused-v1', 'qualification-v1'}

def expression_stats(value):
    result = {'serialized_computable_nodes': 0, 'maximum_computable_depth': 0,
              'maximum_rational_numerator_plus_denominator_bits': 0}
    def walk(v, depth=0):
        if isinstance(v, dict):
            if 'internal' in v:
                depth += 1
                result['serialized_computable_nodes'] += 1
                result['maximum_computable_depth'] = max(result['maximum_computable_depth'], depth)
            if {'sign', 'numerator', 'denominator'} <= v.keys():
                bits = sum(sum(n << (32*i) for i, n in enumerate(v[k])).bit_length()
                           for k in ['numerator', 'denominator'])
                result['maximum_rational_numerator_plus_denominator_bits'] = max(
                    result['maximum_rational_numerator_plus_denominator_bits'], bits)
            for child in v.values():
                walk(child, depth)
        elif isinstance(v, list):
            for child in v:
                walk(child, depth)
    walk(value)
    return result

structures = {}
for case in ['overlap', 'coincident', 'touching', 'full26', 'merged26']:
    row = next(row for row in replay if row['case'] == case and row['variant'] == 'candidate')
    path = E / 'resources-v1/measurements' / (row['name'] + '.artifact.json')
    structures[case] = expression_stats(json.loads(path.read_text()))
host = json.loads((E / 'cost-host-v1.json').read_text())
benchmark_paths = {
    'prior': E.parent / '2026-10-16-minkowski-contracts/benchmark-repair-v1/feature_pipeline.stdout',
    'current': E / 'qualification-v1/feature_pipeline.stdout',
}
benchmark_rows = {name: list(csv.DictReader(path.open())) for name, path in benchmark_paths.items()}
assert len(benchmark_rows['prior']) == len(benchmark_rows['current']) == 4
benchmark_observations = []
for old_row, new_row in zip(benchmark_rows['prior'], benchmark_rows['current']):
    assert {key: value for key, value in old_row.items() if key != 'elapsed_ns'} == {
        key: value for key, value in new_row.items() if key != 'elapsed_ns'}
    benchmark_observations.append({'prior': old_row, 'current': new_row,
                                   'all_non_time_columns_identical': True,
                                   'current_over_prior': int(new_row['elapsed_ns']) / int(old_row['elapsed_ns'])})
summary = {
    'passed': True,
    'qualification': {'phase': 'qualification-v1', 'commands': 50, 'passed': 49,
                      'failed_command': 'hypertri-fuzz-clippy',
                      'replacement_phase': 'fuzz-lint-repair-v1', 'replacement_commands_passed': 5,
                      'scope': 'Two existing fixed-size triangle loops use as_chunks::<3>().0; all assertions remain. Production and non-fuzz regression sources are identical to the broad passing checks. See fuzz-lint-adjustment-v1.json.'},
    'resource_phase': 'resources-v1', 'resource_commands_passed': 6,
    'test_totals': totals, 'phases': phases, 'costs': costs,
    'timing_rows': timing, 'allocation_rows': allocations,
    'original_baseline_children': 24, 'additional_children': 64,
    'library_bytes': host['library_bytes'], 'source_sizes': host['source_sizes'],
    'executable_bytes': json.loads((E / 'resources-v1/measurements/sizes.json').read_text()),
    'artifacts': json.loads((E / 'resources-v1/measurements/artifact-comparison.json').read_text()),
    'serialized_expression_structure': structures,
    'benchmark_consumer_observations': {
        'files_sha256': {str(path.relative_to(E.parents[2])): hashlib.sha256(path.read_bytes()).hexdigest()
                         for path in benchmark_paths.values()},
        'rows': benchmark_observations,
        'scope': 'Retained original benchmark consumers, one sample each with zero warmup and one iteration. Same recorded non-time columns; counts/checksums are not independent semantic oracles. Different execution epochs and uncontrolled host scheduling; these observations are not a matched performance campaign.',
    },
    'structure_scope': 'Serialized output occurrences and rational numerator+denominator bit lengths; not unique internal DAG nodes or intermediate coefficient peaks. Allocator and process-RSS measurements are separate.',
    'scope': 'Canonical topology repair and strict completion of the original 26-box cavity and deletion-minimal five-box union. Fixed pre-edit probe, both baseline epochs retained, exact control artifacts compared. Small unpinned-host samples; no optimization claim.',
    'measurement_boundary': 'Every timed construction asserts a Certified receipt and materializes the complete native output. That output is dropped after the timing/allocation snapshots. Exact topology, volume, membership and ordered serialization are checked on a separate post-timing replay, as in the unchanged pre-edit driver; the artifact is not a serialization of the first timed object. Cold/warm labels describe the timed construction process state. Independent regression tests establish set semantics separately.',
    'limitations': 'Bare meshes still lack inherited end-to-end provenance. The canonical pair scan has finite combinatorial bounds but no internal scalar-work, heap-byte or cancellation guarantee. Qualification commands use external 1800-second/8-GiB containment except WASM/Node retains only wall containment; cost children have 120-second walls. Requested Rust allocator bytes exclude native allocator internals. Full workflow verification, clean external package/MSRV/release, formal proof, instrumented coverage and directed adversarial campaigns remain open. The 35 inherited native files remain separately unadmitted.',
}
(E / 'qualification-summary-v1.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps({'phases': phases, 'costs': costs, 'timing_rows': len(timing),
                  'allocation_rows': len(allocations), 'expression_structure': structures}, indent=2))
