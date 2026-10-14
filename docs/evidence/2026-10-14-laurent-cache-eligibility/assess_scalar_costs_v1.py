"""Assess all closed scalar cost rows and all three contrasts without selecting samples."""
import gzip
import itertools
import json
from pathlib import Path
import re
import statistics

from eligibility_cost_common_v1 import E, PARENT, COST_PREP, GROUPS, SUITES, ORDERS, sha, dump, verify_costs
from scalar_cost_rows_v1 import SPECS, parse

CONTRASTS = [('main', 'candidate'), ('borrowed', 'candidate'), ('main', 'borrowed')]
FIELDS = ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']


def main():
    destination = E / 'scalar-cost-assessment-v1.json'
    assert not destination.exists()
    prep = json.loads((COST_PREP / 'protocol.json').read_text())
    verify_costs(prep, full=True)
    out = E / 'scalar-costs-v1'
    result = json.loads((out / 'result.json').read_text())
    assert result['complete'] and result['commands'] == 133
    assert result['fixed_cost_processes'] == 108 and result['preflight_processes'] == 18
    assert all(sha(out / p) == h for p, h in result['artifact_sha256'].items())
    assert {p.name for p in out.iterdir() if p.is_file()} == set(result['artifact_sha256']) | {'result.json'}
    protocol = json.loads((out / 'protocol.json').read_text())
    assert protocol['runner_sha256'] == sha(E / 'measure_scalar_costs_v1.py')
    assert protocol['assessor_sha256'] == sha(__file__)
    assert protocol['parser_sha256'] == sha(E / 'scalar_cost_rows_v1.py')
    assert protocol['source_review_sha256'] == sha(E / 'scalar-cost-source-review-v1.json')
    assert protocol['preparation_sha256'] == sha(COST_PREP / 'protocol.json')
    assert protocol['groups'] == GROUPS and protocol['orders'] == ORDERS
    assert set(map(tuple, ORDERS)) == set(itertools.permutations(GROUPS))
    assert all(sum(order[position] == group for order in ORDERS) == 2 for position in range(3) for group in GROUPS)
    libraries = json.loads((out / 'libraries.json').read_text())
    binaries = json.loads((out / 'binaries.json').read_text())
    assert set(libraries) == set(GROUPS) and len(binaries) == 18
    assert all(sha(row['path']) == row['sha256'] and Path(row['path']).stat().st_size == row['bytes'] for row in binaries.values())
    assert all(sha(row['path']) == row['sha256'] for group in libraries.values() for row in group.values())
    features = [{key: row['features'] for key, row in libraries[group].items()} for group in GROUPS]
    assert all(f == features[0] for f in features)
    commands = json.loads((out / 'commands.json').read_text())
    assert len(commands) == len({c['name'] for c in commands}) == 133
    assert all(c['exit_code'] == 0 for c in commands)
    raw = {}
    for command in commands:
        for stream in ['stdout', 'stderr']:
            path = out / (command['name'] + '.' + stream + '.gz')
            assert sha(path) == command[stream + '_sha256']
            raw[command['name'], stream] = gzip.decompress(path.read_bytes()).decode()
    builds = ['build-candidate-production-libraries'] + ['link-candidate-' + suite + '-' + mode
        for suite in SUITES for mode in ['time', 'memory']]
    assert [c['name'] for c in commands[:7]] == builds
    assert commands[0]['command'] == ['cargo', 'build', '--locked', '--release',
        '-p', 'hyperreal', '-p', 'hypersolve', '-p', 'hypercurve', '-p', 'hyperdrc',
        '--all-features', '--lib', '--message-format=json']
    qualified_lib = json.loads((E / 'qualification-v1/library.json').read_text())
    assert libraries['candidate']['hyperreal']['features'] == qualified_lib['features']
    assert result['fresh_scalar_library_byte_identical_to_qualification'] == (libraries['candidate']['hyperreal']['sha256'] == qualified_lib['sha256'])
    expected = [(suite, mode, 42, 0, position, group, True) for suite in SUITES
        for mode in ['time', 'memory'] for position, group in enumerate(GROUPS, 1)]
    schedule = [{'suite': suite, 'mode': mode, 'count': count, 'round': repetition, 'order': order}
        for suite in SUITES for mode, count in [('time', 4096), ('memory', 1000)]
        for repetition, order in enumerate(ORDERS, 1)]
    assert protocol['schedule'] == schedule and len(schedule) == 36
    for row in schedule:
        for position, group in enumerate(row['order'], 1):
            expected.append((row['suite'], row['mode'], row['count'], row['round'], position, group, False))
    trials = json.loads(gzip.decompress((out / 'trials.json.gz').read_bytes()))
    assert len(trials) == len(expected) == 126
    identities = {}
    for trial, scheduled, command in zip(trials, expected, commands[7:], strict=True):
        assert tuple(trial[k] for k in ['suite', 'mode', 'count', 'round', 'position', 'group', 'preflight']) == scheduled
        suite, mode, count, repetition, position, group, preflight = scheduled
        name = ('preflight-' if preflight else 'sample-') + f'{suite}-{mode}-{repetition}-{group}'
        assert trial['command_name'] == command['name'] == name
        rows, identity = parse(suite, raw[name, 'stdout'], count)
        assert rows == trial['rows'] and trial['identity_matches_reference']
        assert identity == identities.setdefault((suite, count), identity)
        assert all(r['allocation_accounting'] == (mode == 'memory') for r in rows if 'name' in r)
        assert command['command'] == ['taskset', '-c', '0', '/usr/bin/time', '-v',
            binaries[group + '-' + suite + '-' + mode]['path'], str(count)]
        assert command['cwd'] == prep['variants'][group]['source']
        rss = re.search(r'Maximum resident set size \(kbytes\): (\d+)', raw[name, 'stderr'])
        assert rss and int(rss[1]) == trial['max_rss_kib']
    selected = [trial for trial in trials if not trial['preflight']]
    assert len(selected) == 108
    timing, memory, nonrepeat = [], [], []
    for suite in SUITES:
        for mode in ['time', 'memory']:
            rows = {}
            for group in GROUPS:
                own = [t for t in selected if (t['suite'], t['mode'], t['group']) == (suite, mode, group)]
                assert [t['round'] for t in own] == list(range(1, 7))
                rows[group] = [{r['name']: r for r in t['rows'] if 'name' in r} for t in own]
            names = list(rows['main'][0])
            assert len(names) == SPECS[suite][1] and all(list(r) == names for group in rows.values() for r in group)
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
                    row = {'baseline_group': left, 'candidate_group': right, 'suite': suite, 'name': name, 'iterations': iterations}
                    if mode == 'time':
                        a, b = ([r[name]['nanos'] for r in rows[g]] for g in [left, right])
                        assert all(n > 0 for n in a + b)
                        paired = [y / x for x, y in zip(a, b, strict=True)]
                        row |= {'baseline_ns': a, 'candidate_ns': b,
                            'baseline_median_ns': statistics.median(a), 'candidate_median_ns': statistics.median(b),
                            'ratio_of_medians': statistics.median(b) / statistics.median(a),
                            'paired_ratios': paired, 'median_paired_ratio': statistics.median(paired),
                            'candidate_higher_pairs': sum(y > x for x, y in zip(a, b, strict=True)),
                            'candidate_lower_pairs': sum(y < x for x, y in zip(a, b, strict=True))}
                        timing.append(row)
                    else:
                        row['fields'] = {}
                        for field in FIELDS:
                            a, b = ([r[name][field] for r in rows[g]] for g in [left, right])
                            row['fields'][field] = {'baseline': a, 'candidate': b,
                                'baseline_median': statistics.median(a), 'candidate_median': statistics.median(b),
                                'paired_deltas': [y - x for x, y in zip(a, b, strict=True)],
                                'candidate_higher_pairs': sum(y > x for x, y in zip(a, b, strict=True)),
                                'candidate_lower_pairs': sum(y < x for x, y in zip(a, b, strict=True))}
                        memory.append(row)
    assert len(timing) == len(memory) == 204
    history = json.loads(gzip.decompress((PARENT / 'library-costs-v1/trials.json.gz').read_bytes()))
    differences = []
    for trial in selected:
        if trial['mode'] != 'memory' or trial['group'] == 'candidate':
            continue
        old_group = 'main' if trial['group'] == 'main' else 'candidate'
        old = [t for t in history if (t['suite'], t['mode'], t['variant'], t['round'], t['preflight']) ==
               (trial['suite'], 'memory', old_group, trial['round'], False)]
        assert len(old) == 1
        identity = lambda rows: [r for r in rows if 'witness' in r or 'fixture' in r]
        assert identity(old[0]['rows']) == identity(trial['rows'])
        for before, after in zip([r for r in old[0]['rows'] if 'name' in r],
                                 [r for r in trial['rows'] if 'name' in r], strict=True):
            assert before['name'] == after['name']
            for field in FIELDS:
                if before[field] != after[field]:
                    differences.append({'group': trial['group'], 'suite': trial['suite'], 'round': trial['round'],
                        'name': before['name'], 'field': field, 'historical': before[field], 'fresh': after[field]})
    sizes, summary = [], []
    for left, right in CONTRASTS:
        for suite in SUITES:
            own = [r for r in timing if (r['baseline_group'], r['candidate_group'], r['suite']) == (left, right, suite)]
            summary.append({'baseline_group': left, 'candidate_group': right, 'suite': suite, 'timing_rows': len(own),
                'candidate_higher_all_six': sum(r['candidate_higher_pairs'] == 6 for r in own),
                'candidate_lower_all_six': sum(r['candidate_lower_pairs'] == 6 for r in own),
                'candidate_higher_at_least_five': sum(r['candidate_higher_pairs'] >= 5 for r in own)})
            for mode in ['time', 'memory']:
                suffix = suite + '-' + mode
                a, b = (binaries[g + '-' + suffix] for g in [left, right])
                sizes.append({'baseline_group': left, 'candidate_group': right, 'probe': suffix,
                    'baseline_bytes': a['bytes'], 'candidate_bytes': b['bytes'], 'candidate_minus_baseline_bytes': b['bytes'] - a['bytes']})
    dump(destination, {'status': 'closed-three-group-scalar-cost-comparison-complete',
        'assessor_sha256': sha(__file__), 'preparation_sha256': sha(COST_PREP / 'protocol.json'),
        'result_sha256': sha(out / 'result.json'), 'commands_passed': 133,
        'fixed_cost_processes': 108, 'preflight_processes': 18, 'qualified_scalar_tests_reused': 879,
        'fresh_scalar_library_byte_identical_to_qualification': result['fresh_scalar_library_byte_identical_to_qualification'],
        'all_witnesses_and_representations_match': True, 'timing_summary': summary,
        'timing_rows': timing, 'memory_rows': memory, 'binary_sizes': sizes,
        'all_allocation_fields_repeat': not nonrepeat, 'nonrepeating_allocation_fields': nonrepeat,
        'all_original_group_allocation_fields_match_closed_history': not differences,
        'historical_allocation_differences': differences,
        'limits': ['All 68 scalar phases and all six fixed orders are retained. No samples are filtered, replaced, extended or pooled with history.',
                   'Ratios and pair directions are descriptive, not statistical significance or universal speed claims.',
                   'Allocation counts and requested/retained/peak bytes are phase-relative instrumentation, not whole-application memory.',
                   'This suite does not quantify how many lookups the new branch removes, or explain code layout or frequency effects.',
                   'Production source/features match qualification; a separately built rlib is identified by its own hash.',
                   'Arc, metadata, broad operations, native gates and CLI output equivalence remain required before runtime admission.'],
        'artifacts_sha256': {str(p.relative_to(E)): sha(p) for p in sorted(out.iterdir()) if p.is_file()},
        'broader_library_costs_pending': True, 'native_gates_run': False,
        'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'timing_rows': len(timing), 'memory_rows': len(memory), 'binary_sizes': len(sizes),
                      'summary': summary, 'allocations_repeat': not nonrepeat,
                      'historical_allocation_differences': len(differences), 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
