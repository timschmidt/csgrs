"""Audit the fixed scalar cost trial, separating new proofs from equal answers."""
import gzip
import hashlib
import json
from pathlib import Path
import statistics

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'sphere-cyclotomic-scalar-costs-v1'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def summary(values):
    return {'observations': values, 'minimum': min(values), 'maximum': max(values),
            'median': statistics.median(values)}


def main():
    destination = E / 'sphere-cyclotomic-scalar-costs-assessment-v1.json'
    assert not destination.exists()
    result = json.loads((OUT / 'result.json').read_text())
    assert result['complete'] and result['commands_all_zero'] == 52
    assert result['all_frozen_sources_inputs_and_libraries_unchanged']
    assert all(sha(OUT / name) == digest for name, digest in result['artifacts_sha256'].items())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    assert all(sha(E / name) == digest for name, digest in protocol['inputs_sha256'].items())
    assert all(sha(row['path']) == row['sha256'] for row in protocol['libraries'].values())
    binaries = json.loads((OUT / 'binaries.json').read_text())
    assert all(sha(row['path']) == row['sha256'] for row in binaries.values())
    root_mapping = json.loads(gzip.decompress((OUT / 'root-source-hashes.json.gz').read_bytes()))
    assert all(sha(ROOT / name) == digest for name, digest in root_mapping.items())
    for group, name in protocol['source_mapping_files'].items():
        mapping = json.loads(gzip.decompress((E / name).read_bytes()))
        assert all(sha(Path(protocol['sources'][group]) / path) == digest for path, digest in mapping.items())
    commands = json.loads((OUT / 'commands.json').read_text())
    assert len(commands) == 52 and all(row['exit_code'] == 0 for row in commands)
    for row in commands:
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (row['name'] + '.' + stream)) == row[stream + '_sha256']
    orders = [['baseline', 'late', 'image'], ['late', 'image', 'baseline'], ['image', 'baseline', 'late'],
              ['image', 'late', 'baseline'], ['late', 'baseline', 'image'], ['baseline', 'image', 'late']]
    assert protocol['orders'] == orders
    samples = json.loads((OUT / 'samples.json').read_text())
    schedule = [(mode, number, position, group) for mode in ['time', 'memory']
                for number, order in [(0, ['baseline', 'late', 'image']), *enumerate(orders, 1)]
                for position, group in enumerate(order, 1)]
    assert [(s['mode'], s['round'], s['position'], s['group']) for s in samples] == schedule
    assert [r['name'] for r in commands[10:]] == [s['name'] for s in samples]
    cases = list(protocol['repetitions'])
    rows_by_key = {}
    for sample in samples:
        raw = (OUT / (sample['name'] + '.stdout')).read_text()
        rows = [json.loads(line) for line in raw.splitlines() if line.startswith('{')]
        assert rows == sample['rows']
        assert [(r['case'], r['phase']) for r in rows] == [(c, p) for c in cases for p in ['first', 'repeated']]
        assert [line for line in raw.splitlines() if line.startswith('kind ')] == sample['kinds'] == samples[0]['kinds']
        for row in rows:
            n = 1 if row['phase'] == 'first' else 128 if sample['mode'] == 'memory' else protocol['repetitions'][row['case']]
            assert row['iterations'] == n and 0 <= row['certified'] <= n
            assert row['allocation_accounting'] == (sample['mode'] == 'memory')
            assert row['nanos'] > 0
            assert row['peak_bytes_delta'] >= max(0, row['retained_bytes_delta'])
            if sample['round']:
                key = (sample['mode'], sample['group'], row['case'], row['phase'])
                rows_by_key.setdefault(key, []).append(row)
    stats = {}
    for mode in ['time', 'memory']:
        stats[mode] = {}
        for group in ['baseline', 'late', 'image']:
            stats[mode][group] = {}
            for case in cases:
                stats[mode][group][case] = {}
                for phase in ['first', 'repeated']:
                    rows = rows_by_key[mode, group, case, phase]
                    assert len(rows) == 6
                    fields = ['certified', 'nanos'] if mode == 'time' else [
                        'certified', 'allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']
                    stats[mode][group][case][phase] = {
                        'iterations': rows[0]['iterations'],
                        **{field: summary([row[field] for row in rows]) for field in fields},
                    }
    comparisons = {}
    flags = []
    for reference in ['baseline', 'late']:
        comparisons[reference] = {}
        for case in cases:
            before = rows_by_key['time', reference, case, 'repeated']
            after = rows_by_key['time', 'image', case, 'repeated']
            identical_outcomes = all((a['iterations'], a['certified']) == (b['iterations'], b['certified'])
                                     for a, b in zip(after, before))
            ratios = [a['nanos'] / b['nanos'] for a, b in zip(after, before)]
            median_before = statistics.median(row['nanos'] for row in before)
            median_after = statistics.median(row['nanos'] for row in after)
            stable_scale = min(median_before, median_after) >= 2_000_000
            flagged = identical_outcomes and stable_scale and min(ratios) > 1.10 and statistics.median(ratios) > 1.15
            if flagged:
                flags.append({'reference': reference, 'case': case})
            resources = {}
            for phase in ['first', 'repeated']:
                arows = rows_by_key['memory', 'image', case, phase]
                brows = rows_by_key['memory', reference, case, phase]
                resources[phase] = {metric: summary([a[metric] - b[metric] for a, b in zip(arows, brows)])
                                    for metric in ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']}
            comparisons[reference][case] = {
                'same_repeated_query_certification_counts': identical_outcomes,
                'image_over_reference_wall_ratios': summary(ratios),
                'median_phase_wall_ns': {'reference': median_before, 'image': median_after},
                'median_ns_per_query': {'reference': median_before / before[0]['iterations'],
                                        'image': median_after / after[0]['iterations']},
                'both_median_phase_times_at_least_two_ms': stable_scale,
                'predeclared_overhead_flag': flagged,
                'interpretation': ('flagged overhead on these repeated equal-answer queries' if flagged else
                    'new or different certification capability; ratio is not an optimization comparison' if not identical_outcomes else
                    'inconclusive under the predeclared screen'),
                'image_minus_reference_resource_deltas': resources,
            }
    value = {
        'complete': True, 'assessor_sha256': sha(__file__), 'result_sha256': sha(OUT / 'result.json'),
        'commands_all_zero': 52, 'preflight_processes': 6, 'fixed_sample_processes': 36,
        'query_recipes': 14, 'statistics': stats, 'comparisons': comparisons, 'overhead_flags': flags,
        'binary_raw_bytes': {name: row['bytes'] for name, row in binaries.items()},
        'limits': [
            'No first-query single-call timing or allocator-instrumented timing is used as performance evidence.',
            'Repeated timing includes explicit result and monotonicity checks; this is not a universal API latency bound.',
            'New zero proofs may legitimately strengthen an Unknown; differing capability is kept separate from equal-answer optimization comparisons.',
            'All six fixed samples are included. No sample pooling with native sphere trials or earlier versions.',
            'Requested allocation bytes are not stack-byte usage, allocator usable sizes or RSS.',
            'The fourteen prepared recipes are finite cost cases, not an exhaustive representation matrix, formal proof or directed campaign.',
            'A missing flag means inconclusive under this rule, not no regression. Runtime admission remains open.'
        ],
        'runtime_admitted': False, 'plan_complete': False,
    }
    destination.write_text(json.dumps(value, indent=2) + '\n')
    print(json.dumps({'complete': True, 'overhead_flags': flags, 'assessment_sha256': sha(destination)}))


if __name__ == '__main__':
    main()
