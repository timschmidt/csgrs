"""Verify every fixed process and describe all three complete cost contrasts."""
import gzip
import itertools
import json
from pathlib import Path
import re
import statistics

from cost_common_v1 import E, ROOT, COST_PREP, sha, dump, verify_costs
from cost_rows_v1 import SPECS, parse

GROUPS = ['main', 'full-cache', 'candidate']
CONTRASTS = [('main', 'candidate'), ('full-cache', 'candidate'), ('main', 'full-cache')]
FIELDS = ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']


def main():
    destination = E / 'library-assessment-v1.json'
    assert not destination.exists()
    preparation = json.loads((COST_PREP / 'protocol.json').read_text())
    verify_costs(preparation, full=True)
    out = E / 'library-costs-v1'
    result = json.loads((out / 'result.json').read_text())
    assert result['complete'] and result['commands'] == 254
    assert result['fixed_cost_processes'] == 198 and result['abort_control_processes'] == 9
    assert result['preflight_processes'] == 30
    assert all(sha(out / p) == h for p, h in result['artifact_sha256'].items())
    protocol = json.loads((out / 'protocol.json').read_text())
    assert protocol['assessor_sha256'] == sha(__file__)
    assert protocol['parser_sha256'] == sha(E / 'cost_rows_v1.py')
    assert protocol['runner_sha256'] == sha(E / 'measure_costs_v1.py')
    assert protocol['preparation_sha256'] == sha(COST_PREP / 'protocol.json')
    assert protocol['source_review_sha256'] == sha(E / 'cost-source-review-v1.json')
    assert protocol['groups'] == GROUPS
    orders = protocol['orders']
    assert len(orders) == 6 and set(map(tuple, orders)) == set(itertools.permutations(GROUPS))
    assert all(sum(order[position] == label for order in orders) == 2 for position in range(3) for label in GROUPS)
    binaries = json.loads((out / 'binaries.json').read_text())
    libraries = json.loads((out / 'libraries.json').read_text())
    assert len(binaries) == 33 and set(libraries) == set(GROUPS)
    assert all(sha(r['path']) == r['sha256'] and Path(r['path']).stat().st_size == r['bytes'] for r in binaries.values())
    assert all(sha(r['path']) == r['sha256'] for group in libraries.values() for r in group.values())
    features = [{n: r['features'] for n, r in libraries[g].items()} for g in GROUPS]
    assert all(f == features[0] for f in features)
    commands = json.loads((out / 'commands.json').read_text())
    assert len(commands) == 254 and all(r['exit_code'] == 0 for r in commands)
    assert len({r['name'] for r in commands}) == 254
    raw = {}
    for command in commands:
        for stream in ['stdout', 'stderr']:
            path = out / (command['name'] + '.' + stream + '.gz')
            assert sha(path) == command[stream + '_sha256']
            raw[command['name'] + '.' + stream] = gzip.decompress(path.read_bytes()).decode()
    build_and_control = ['build-libraries-candidate']
    for suite, modes in [('rational', ['time', 'memory']), ('quadratic', ['time', 'memory']),
                         ('integer', ['time', 'memory']), ('arc', ['time', 'memory']),
                         ('broad', ['time']), ('abort', ['control'])]:
        build_and_control += ['link-candidate-' + suite + '-' + mode for mode in modes]
    build_and_control += ['clippy-candidate-metadata', 'build-candidate-metadata']
    build_and_control += ['metadata-regressions-' + group for group in GROUPS]
    build_and_control += ['candidate-endpoint-regressions']
    assert [r['name'] for r in commands[:17]] == build_and_control
    assert all(re.search(r'test result: ok\. 3 passed; 0 failed; 1 ignored;', raw['metadata-regressions-' + g + '.stdout']) for g in GROUPS)
    assert re.search(r'test result: ok\. 5 passed; 0 failed; 0 ignored;', raw['candidate-endpoint-regressions.stdout'])
    expected = []
    for suite in [*SPECS, 'metadata']:
        for mode in ['time', 'memory']:
            for position, group in enumerate(GROUPS, 1):
                expected.append((suite, mode, 2 if suite == 'metadata' else 42, 0, position, group, True))
    for round_index, order in enumerate(orders[:3], 1):
        for position, group in enumerate(order, 1):
            expected.append(('abort', 'control', None, round_index, position, group, False))
    original_schedule = [{'suite': suite, 'mode': mode, 'count': count, 'round': round_index, 'order': order}
        for suite in ['rational', 'quadratic', 'integer', 'arc', 'metadata', 'broad']
        for mode, count in ([('time', None)] if suite == 'broad' else
                           [('time', 8192 if suite == 'arc' else 1024 if suite == 'metadata' else 4096),
                            ('memory', 128 if suite == 'metadata' else 1000)])
        for round_index, order in enumerate(orders, 1)]
    assert protocol['schedule'] == original_schedule and len(original_schedule) == 66
    for row in original_schedule:
        for position, group in enumerate(row['order'], 1):
            expected.append((row['suite'], row['mode'], row['count'], row['round'], position, group, False))
    trials = json.loads(gzip.decompress((out / 'trials.json.gz').read_bytes()))
    assert len(trials) == len(expected) == 237
    identities = {}
    for trial, expected_row, command in zip(trials, expected, commands[17:], strict=True):
        assert tuple(trial[k] for k in ['suite', 'mode', 'count', 'round', 'position', 'variant', 'preflight']) == expected_row
        suite, mode, count, round_index, position, group, preflight = expected_row
        name = ('preflight-' if preflight else 'sample-') + f'{suite}-{mode}-{round_index}-{group}'
        assert trial['command_name'] == command['name'] == name
        rows, identity = parse(suite, raw[name + '.stdout'], count)
        assert rows == trial['rows'] and trial['identity_matches_reference']
        key = (suite, mode, count, preflight)
        assert identity == identities.setdefault(key, identity)
        binary_key = group + '-metadata' if suite == 'metadata' else group + '-' + suite + '-' + mode
        expected_command = ['taskset', '-c', '0', '/usr/bin/time', '-v', binaries[binary_key]['path']]
        if suite == 'metadata':
            expected_command += ['bezier_region::combined_admission_metadata::cost::probe', '--exact', '--ignored', '--test-threads=1', '--nocapture']
            assert command['extra_environment'] == {'REGION_PROBE_MODE': mode, 'REGION_PROBE_ITERATIONS': str(count)}
        elif count is not None:
            expected_command.append(str(count))
        assert command['command'] == expected_command
        rss = re.search(r'Maximum resident set size \(kbytes\): (\d+)', raw[name + '.stderr'])
        assert rss and int(rss[1]) == trial['max_rss_kib']
    assert len(identities['abort', 'control', None, False]) == 210
    selected = [r for r in trials if not r['preflight'] and r['suite'] != 'abort']
    assert len(selected) == 198
    timing, memory, nonrepeat = [], [], []
    for suite in ['rational', 'quadratic', 'integer', 'arc', 'metadata', 'broad']:
        for mode in (['time'] if suite == 'broad' else ['time', 'memory']):
            subset = [r for r in selected if r['suite'] == suite and r['mode'] == mode]
            assert len(subset) == 18
            rows = {}
            for group in GROUPS:
                samples = sorted((r for r in subset if r['variant'] == group), key=lambda r: r['round'])
                assert [r['round'] for r in samples] == list(range(1, 7))
                rows[group] = [{r['name']: r for r in s['rows'] if 'name' in r} for s in samples]
            names = list(rows['main'][0])
            assert all(list(row) == names for group in rows.values() for row in group)
            for name in names:
                iterations = rows['main'][0][name]['iterations']
                assert all(r[name]['iterations'] == iterations for group in rows.values() for r in group)
                if mode == 'memory':
                    for group in GROUPS:
                        for field in FIELDS:
                            values = [r[name][field] for r in rows[group]]
                            if len(set(values)) != 1:
                                nonrepeat.append({'group': group, 'suite': suite, 'name': name, 'field': field, 'values': values})
                for left, right in CONTRASTS:
                    value = {'baseline_group': left, 'candidate_group': right,
                             'suite': suite, 'name': name, 'iterations': iterations}
                    if mode == 'time':
                        a, b = ([r[name]['nanos'] for r in rows[g]] for g in [left, right])
                        assert all(x > 0 for x in a + b)
                        ratios = [y / x for x, y in zip(a, b, strict=True)]
                        value |= {'baseline_ns': a, 'candidate_ns': b,
                                  'baseline_median_ns': statistics.median(a), 'candidate_median_ns': statistics.median(b),
                                  'ratio_of_medians': statistics.median(b) / statistics.median(a),
                                  'paired_ratios': ratios, 'median_paired_ratio': statistics.median(ratios),
                                  'candidate_higher_pairs': sum(y > x for x, y in zip(a, b, strict=True)),
                                  'candidate_lower_pairs': sum(y < x for x, y in zip(a, b, strict=True))}
                        timing.append(value)
                    else:
                        value['fields'] = {}
                        for field in FIELDS:
                            a, b = ([r[name][field] for r in rows[g]] for g in [left, right])
                            value['fields'][field] = {'baseline': a, 'candidate': b,
                                'baseline_median': statistics.median(a), 'candidate_median': statistics.median(b),
                                'paired_deltas': [y - x for x, y in zip(a, b, strict=True)],
                                'candidate_higher_pairs': sum(y > x for x, y in zip(a, b, strict=True)),
                                'candidate_lower_pairs': sum(y < x for x, y in zip(a, b, strict=True))}
                        memory.append(value)
    assert len(timing) == 1095 and len(memory) == 402
    history_path = ROOT / 'docs/evidence/2026-10-09-hyperdrc-kicad-pad-sources/cost/rows.json'
    historical = json.loads(history_path.read_text())
    assert identities['broad', 'time', None, False] == historical
    sizes, summary = [], []
    suffixes = sorted(k.removeprefix('main-') for k in binaries if k.startswith('main-'))
    assert len(suffixes) == 11
    for left, right in CONTRASTS:
        for suffix in suffixes:
            a, b = (binaries[g + '-' + suffix] for g in [left, right])
            sizes.append({'baseline_group': left, 'candidate_group': right, 'probe': suffix,
                          'baseline_bytes': a['bytes'], 'candidate_bytes': b['bytes'],
                          'candidate_minus_baseline_bytes': b['bytes'] - a['bytes']})
        for suite in ['rational', 'quadratic', 'integer', 'arc', 'metadata', 'broad']:
            own = [r for r in timing if (r['baseline_group'], r['candidate_group'], r['suite']) == (left, right, suite)]
            summary.append({'baseline_group': left, 'candidate_group': right, 'suite': suite,
                'timing_rows': len(own), 'candidate_higher_all_six': sum(r['candidate_higher_pairs'] == 6 for r in own),
                'candidate_lower_all_six': sum(r['candidate_lower_pairs'] == 6 for r in own),
                'candidate_higher_at_least_five': sum(r['candidate_higher_pairs'] >= 5 for r in own)})
    assessment = {
        'status': 'borrowed-integer-three-group-library-comparison-complete',
        'assessor_sha256': sha(__file__), 'preparation_sha256': sha(COST_PREP / 'protocol.json'),
        'scope': 'Unchanged main and previous full-cache artifacts versus the same full-cache source with the qualified borrowed-integer helper. All samples are fresh; every prior sample remains closed.',
        'commands_passed': 254, 'fixed_cost_processes': 198, 'preflight_processes': 30, 'abort_control_processes': 9,
        'scalar_tests_reused': 876, 'focused_tests': result['focused_tests'],
        'all_witnesses_representation_inventories_and_210_abort_receipts_match': True,
        'historical_broad_witnesses': {'path': str(history_path.relative_to(ROOT)), 'sha256': sha(history_path), 'all_231_identical': True},
        'timing_summary': summary, 'timing_rows': timing, 'memory_rows': memory, 'binary_sizes': sizes,
        'all_allocation_fields_repeat_within_each_group': not nonrepeat, 'nonrepeating_allocation_fields': nonrepeat,
        'limits': ['Ratios and six-pair directions are descriptive; they do not establish statistical significance or universal performance.',
                   'Requested allocations and phase-relative retention/peak are not allocator RSS or whole-life reclamation.',
                   'The unchanged metadata harness has a TLS allocator wrapper in both modes and is not a production executable.',
                   'The broad probe retains its original setup/windows and does not measure the entire application.',
                   'Current CPU frequency snapshots do not establish historical host conditions or explain any earlier regression.',
                   'No sample is filtered, replaced, extended or pooled with historical data. Prior adverse costs and native failures remain.',
                   'No native gate or CLI report comparison is performed here. No runtime admission or full-plan completion is implied.'],
        'artifacts_sha256': {str(p.relative_to(E)): sha(p) for p in sorted(out.iterdir()) if p.is_file()},
        'runtime_admitted': False, 'plan_complete': False,
    }
    dump(destination, assessment)
    print(json.dumps({'timing_rows': len(timing), 'memory_rows': len(memory), 'summary': summary,
                      'all_allocation_fields_repeat': not nonrepeat, 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
