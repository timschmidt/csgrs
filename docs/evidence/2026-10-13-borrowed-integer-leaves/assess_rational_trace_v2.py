"""Reparse all diagnostic logs and compare every public-count phase."""
import gzip
import json

from rational_trace_common_v2 import E, TRACE_PREP, GROUPS, ORDERS, sha, dump, verify, rows, identity


def counts(row):
    result = {}

    def flatten(prefix, value):
        if isinstance(value, dict):
            for key, child in value.items():
                flatten(prefix + '/' + key, child)
        else:
            assert isinstance(value, int) and value >= 0
            result[prefix] = value

    flatten('rational', row['rational'])
    for item in row['dispatch']:
        key = 'dispatch:' + json.dumps([item['layer'], item['operation'], item['path']])
        assert key not in result
        result[key] = item['count']
    return result


def main():
    destination = E / 'rational-trace-assessment-v2.json'
    assert not destination.exists()
    prep = json.loads((TRACE_PREP / 'protocol.json').read_text())
    verify(prep, full=True)
    out = E / 'rational-trace-v2'
    result = json.loads((out / 'result.json').read_text())
    protocol = json.loads((out / 'protocol.json').read_text())
    assert result['complete'] and result['commands'] == 17
    assert all(sha(out / p) == h for p, h in result['artifact_sha256'].items())
    assert protocol['assessor_sha256'] == sha(__file__)
    assert protocol['preparation_sha256'] == sha(TRACE_PREP / 'protocol.json')
    assert protocol['orders'] == ORDERS and protocol['count'] == 4096
    commands = json.loads((out / 'commands.json').read_text())
    binaries = json.loads((out / 'binaries.json').read_text())
    samples = json.loads((out / 'samples.json').read_text())
    assert len(commands) == 17 and len(samples) == 8 and set(binaries) == set(GROUPS)
    assert all(c['exit_code'] == 0 for c in commands)
    assert all(sha(r['path']) == r['sha256'] for r in binaries.values())
    for command in commands:
        assert all(sha(out / (command['name'] + '.' + stream + '.gz')) == command[stream + '_sha256']
                   for stream in ['stdout', 'stderr'])
    expected_builds = ['format-header'] + [kind + '-' + group for group in GROUPS for kind in ['clippy', 'link']]
    assert [c['name'] for c in commands[:9]] == expected_builds
    expected = [(repetition, position, group) for repetition, order in enumerate(ORDERS, 1)
                for position, group in enumerate(order, 1)]
    families = ['multiradical_square', 'quadratic_reciprocal', 'pi_parameter_transport',
                'radical_laurent_transport', 'opaque_linear_atom', 'conjugate_product']
    phases = [name + '_' + phase for name in families for phase in ['construction', 'first', 'warm', 'cloned']]
    for sample, (repetition, position, group), command in zip(samples, expected, commands[9:], strict=True):
        assert (sample['repetition'], sample['position'], sample['group']) == (repetition, position, group)
        assert sample['command_name'] == command['name'] == f'trace-{repetition}-{group}'
        assert command['command'] == ['taskset', '-c', '0', binaries[group]['path'], '4096']
        raw = gzip.decompress((out / (command['name'] + '.stdout.gz')).read_bytes())
        assert identity(raw) == prep['identity']
        trace = [r for r in rows(raw) if 'trace_phase' in r]
        assert trace == sample['trace']
        assert [r['trace_phase'] for r in trace] == phases
        assert all(r['iterations'] == 4096 for r in trace)
    by_group = {}
    for group in GROUPS:
        own = [s['trace'] for s in samples if s['group'] == group]
        assert len(own) == 2 and own[0] == own[1]
        by_group[group] = own[0]
    public = lambda trace: [{k: v for k, v in r.items() if k != 'cache_events'} for r in trace]
    assert public(by_group['cache-coverage']) == public(by_group['full-cache'])
    comparisons = []
    for left, right in [('main', 'full-cache'), ('main', 'candidate'), ('full-cache', 'candidate')]:
        for a, b in zip(by_group[left], by_group[right], strict=True):
            x, y = counts(a), counts(b)
            changed = [{'field': key, 'baseline': x.get(key, 0), 'candidate': y.get(key, 0),
                        'candidate_minus_baseline': y.get(key, 0) - x.get(key, 0)}
                       for key in sorted(x.keys() | y.keys()) if x.get(key, 0) != y.get(key, 0)]
            comparisons.append({'baseline_group': left, 'candidate_group': right,
                                'phase': a['trace_phase'], 'all_public_counts_equal': not changed,
                                'changed_counts': changed})
    assert len(comparisons) == 72
    cache = []
    for row in by_group['cache-coverage']:
        event = row['cache_events']
        assert len(event) == 27 and all(isinstance(v, int) and v >= 0 for v in event.values())
        assert event['retained_lookup'] == event['retained_visit']
        assert event['retained_visit'] + event['original_visit'] == sum(event[k] for k in [
            'retained_hit', 'local_memo_hit', 'normalizer_work_visit', 'normalizer_budget_decline'])
        assert event['store_attempt'] == event['closed_copy_attempt']
        assert event['closed_copy_attempt'] == event['closed_copy_declined_opaque'] + event['closed_copy_success']
        assert event['closed_copy_success'] == event['store_published'] + event['store_already_present']
        assert event['evaluation_cell_allocated'] == event['evaluation_cell_published'] + event['evaluation_cell_race_lost']
        cache.append({'phase': row['trace_phase'], 'iterations': row['iterations'], 'events': event})
    assessment = {
        'status': 'original-rational-workload-cache-counts-complete',
        'assessor_sha256': sha(__file__), 'result_sha256': sha(out / 'result.json'),
        'commands_passed': 17, 'fixed_processes': 8, 'trace_windows': 192, 'counter_windows': 48,
        'all_original_witnesses_and_representations_match': True,
        'all_counts_repeat_in_reversed_group_order': True,
        'instrumented_full_cache_public_counts_match_production_full_cache': True,
        'comparisons': comparisons, 'cache_events_per_process': cache,
        'opaque_linear_atom_all_phases': [r for r in cache if r['phase'].startswith('opaque_linear_atom_')],
        'new_library_builds': 0, 'new_runtime_sources': 0,
        'limits': [
            'The cache-event library is the previously qualified instrumented full-cache source, before the borrowed-integer repair. Its arithmetic counts match the actual full-cache library here; no candidate cache-event counts are claimed.',
            'All 24 phases and all four groups are retained. Two runs per group are count-repeatability checks, not cost samples.',
            'Diagnostic counters can perturb execution. No elapsed timings, allocator totals, RSS comparison, performance causality or native deadline claim is derived from these runs.',
            'A cache-event count establishes that the path executed. It does not quantify that path\'s share of a prior timing difference.',
            'Earlier cost results, native failures, runtime non-admission and the full properties.md scope remain unchanged.',
        ],
        'artifacts_sha256': {str(p.relative_to(E)): sha(p) for p in sorted(out.iterdir()) if p.is_file()},
        'runtime_candidate': False, 'runtime_admitted': False, 'plan_complete': False,
    }
    dump(destination, assessment)
    print(json.dumps({'status': assessment['status'], 'comparisons': len(comparisons),
                      'opaque_linear_atom': assessment['opaque_linear_atom_all_phases']}), flush=True)


if __name__ == '__main__':
    main()
