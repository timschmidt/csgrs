"""Verify every fixed count window, public count comparison and lookup partition."""
import gzip
import json
from pathlib import Path
import re

from miss_classification_common_v2 import E, ROOT, PARENT, COVERAGE, BORROWED, PREP, GROUPS, ORDERS, SUITES, sha, dump, verify

CONTRASTS = [('main', 'borrowed'), ('main', 'eligibility'), ('borrowed', 'eligibility')]


def rows(raw):
    return [json.loads(line) for line in raw.splitlines() if line.startswith('{')]


def identity(raw):
    return [r for r in rows(raw) if 'fixture' in r or 'witness' in r]


def public(row):
    return {k: v for k, v in row.items() if k not in ['cache_events', 'lookup_classes']}


def counts(row):
    result = {}

    def flatten(prefix, value):
        if isinstance(value, dict):
            for key, child in value.items():
                flatten(prefix + '/' + key, child)
        else:
            assert type(value) is int and value >= 0
            result[prefix] = value

    flatten('rational', row['rational'])
    for entry in row['dispatch']:
        key = 'dispatch:' + json.dumps([entry['layer'], entry['operation'], entry['path']])
        assert key not in result and type(entry['count']) is int and entry['count'] >= 0
        result[key] = entry['count']
    return result


def main():
    destination = E / 'assessment-v2.json'
    assert not destination.exists()
    prep = json.loads((PREP / 'protocol.json').read_text())
    verify(prep, full=True)
    out = E / 'diagnostic-v2'
    result = json.loads((out / 'result.json').read_text())
    assert result['complete'] and result['commands'] == 53 and result['scalar_tests'] == 881
    assert result['fixed_count_processes'] == 24 and result['trace_windows'] == 544
    assert all(sha(out / p) == h for p, h in result['artifact_sha256'].items())
    assert {p.name for p in out.iterdir() if p.is_file()} == set(result['artifact_sha256']) | {'result.json'}
    protocol = json.loads((out / 'protocol.json').read_text())
    assert protocol['assessor_sha256'] == sha(__file__)
    assert protocol['runner_sha256'] == sha(E / 'measure_v2.py')
    assert protocol['preparation_sha256'] == sha(PREP / 'protocol.json')
    assert protocol['review_sha256'] == sha(E / 'source-review-v2.json')
    assert protocol['groups'] == GROUPS and protocol['orders'] == ORDERS and protocol['suites'] == SUITES
    assert protocol['inputs'] == 4096
    commands = json.loads((out / 'commands.json').read_text())
    assert len(commands) == len({r['name'] for r in commands}) == 53
    assert all(r['exit_code'] == 0 for r in commands)
    raw = {}
    for command in commands:
        for stream in ['stdout', 'stderr']:
            path = out / (command['name'] + '.' + stream + '.gz')
            assert sha(path) == command[stream + '_sha256']
            raw[command['name'], stream] = gzip.decompress(path.read_bytes()).decode()
        if command['name'].startswith('clippy-'):
            assert '-D' in command['command'] and command['command'][command['command'].index('-D') + 1] == 'warnings'
    expected_builds = ['format-header', 'clippy-library', 'build-tests', 'build-library', 'scalar-library']
    expected_builds += [step + '-' + suite + '-' + group for suite in SUITES for group in GROUPS for step in ['clippy', 'link']]
    assert [r['name'] for r in commands[:29]] == expected_builds
    assert '--all-features' in commands[1]['command'] and '--all-targets' in commands[1]['command']
    assert re.search(r'test result: ok\. 881 passed; 0 failed; 0 ignored;', raw['scalar-library', 'stdout'])
    names = set(re.findall(r'^test ([^ ]+) \.\.\. ok$', raw['scalar-library', 'stdout'], re.MULTILINE))
    assert len(names) == 881
    retained_groups = {fragment: sum(fragment in name for name in names) for fragment in [
        'quadratic_tower_sign_retention_tests::', 'persistent_laurent_cache_tests::',
        'laurent_cache_eligibility_tests::', 'borrowed_integer_tests::']}
    assert list(retained_groups.values()) == [11, 8, 3, 2]
    binaries = json.loads((out / 'binaries.json').read_text())
    libraries = json.loads((out / 'libraries.json').read_text())
    assert len(binaries) == 13 and set(libraries) == set(GROUPS)
    assert all(sha(row['path']) == row['sha256'] and Path(row['path']).stat().st_size == row['bytes'] for row in binaries.values())
    assert all(sha(row['path']) == row['sha256'] for group in libraries.values() for row in group.values())
    feature_groups = [{name: row['features'] for name, row in libraries[group].items()} for group in GROUPS]
    assert all(features == feature_groups[0] for features in feature_groups)
    test_artifacts = [r for r in rows(raw['build-tests', 'stdout']) if r.get('reason') == 'compiler-artifact'
                      and r.get('executable') == binaries['library-tests']['path']]
    assert len(test_artifacts) == 1 and test_artifacts[0]['features'] == binaries['library-tests']['features']
    assert test_artifacts[0]['manifest_path'] == binaries['library-tests']['manifest_path']
    for name, library in libraries['classified'].items():
        artifacts = [r for r in rows(raw['build-library', 'stdout']) if r.get('reason') == 'compiler-artifact'
                     and library['path'] in r.get('filenames', [])]
        assert len(artifacts) == 1 and artifacts[0]['features'] == library['features']
        assert artifacts[0]['manifest_path'] == library['manifest_path']
    expected = [(suite, repetition, position, group) for suite in SUITES
                for repetition, order in enumerate(ORDERS, 1) for position, group in enumerate(order, 1)]
    samples = json.loads((out / 'samples.json').read_text())
    assert len(samples) == len(expected) == 24
    references = {suite: gzip.decompress((PARENT / 'scalar-costs-v1' / ('sample-' + suite + '-time-1-main.stdout.gz')).read_bytes()).decode()
                  for suite in SUITES}
    for sample, expected_row, command in zip(samples, expected, commands[29:], strict=True):
        suite, repetition, position, group = expected_row
        assert tuple(sample[k] for k in ['suite', 'repetition', 'position', 'group']) == expected_row
        name = f'count-{suite}-{repetition}-{group}'
        assert sample['command_name'] == command['name'] == name
        assert command['command'] == ['taskset', '-c', '0', binaries[suite + '-' + group]['path'], '4096']
        assert identity(raw[name, 'stdout']) == identity(references[suite]) and sample['identity_matches_closed_cost_trial']
        trace = [r for r in rows(raw[name, 'stdout']) if 'trace_phase' in r]
        phases = [r['name'] for r in rows(references[suite]) if 'name' in r]
        assert trace == sample['trace'] and [r['trace_phase'] for r in trace] == phases
        assert all(r['iterations'] == 4096 for r in trace)
        for row in trace:
            assert (row['cache_events'] is not None) == (group == 'classified')
            assert (row['lookup_classes'] is not None) == (group == 'classified')
    by_group = {}
    for suite in SUITES:
        for group in GROUPS:
            own = [s['trace'] for s in samples if (s['suite'], s['group']) == (suite, group)]
            assert len(own) == 2 and own[0] == own[1]
            by_group[suite, group] = own[0]
        assert [public(r) for r in by_group[suite, 'classified']] == [public(r) for r in by_group[suite, 'borrowed']]
    comparisons = []
    for left, right in CONTRASTS:
        for suite in SUITES:
            for a, b in zip(by_group[suite, left], by_group[suite, right], strict=True):
                x, y = counts(a), counts(b)
                changed = [{'field': key, 'baseline': x.get(key, 0), 'candidate': y.get(key, 0),
                            'candidate_minus_baseline': y.get(key, 0) - x.get(key, 0)}
                           for key in sorted(x.keys() | y.keys()) if x.get(key, 0) != y.get(key, 0)]
                comparisons.append({'baseline_group': left, 'candidate_group': right, 'suite': suite,
                    'phase': a['trace_phase'], 'all_public_counts_equal': not changed, 'changed_counts': changed})
    assert len(comparisons) == 204
    eligible = set(prep['eligible_kinds'])
    kinds = prep['lookup_kind_names']
    outcomes = prep['lookup_outcome_names']
    classified = []
    for suite in SUITES:
        for row in by_group[suite, 'classified']:
            event, classes = row['cache_events'], row['lookup_classes']
            assert len(event) == 27 and all(type(n) is int and n >= 0 for n in event.values())
            assert set(classes) == set(kinds) and len(classes) == 61
            assert all(set(v) == set(outcomes) and all(type(n) is int and n >= 0 for n in v.values()) for v in classes.values())
            assert sum(sum(v.values()) for v in classes.values()) == event['retained_lookup']
            assert sum(v['hit'] for v in classes.values()) == event['retained_hit']
            assert all(n == 0 for n in classes['Unclassified'].values())
            assert all(v['hit'] == 0 for kind, v in classes.items() if kind not in eligible)
            assert event['retained_lookup'] == event['retained_visit']
            assert event['retained_visit'] + event['original_visit'] == sum(event[k] for k in [
                'retained_hit', 'local_memo_hit', 'normalizer_work_visit', 'normalizer_budget_decline'])
            assert event['store_attempt'] == event['closed_copy_attempt']
            assert event['closed_copy_attempt'] == event['closed_copy_declined_opaque'] + event['closed_copy_success']
            assert event['closed_copy_success'] == event['store_published'] + event['store_already_present']
            assert event['evaluation_cell_allocated'] == event['evaluation_cell_published'] + event['evaluation_cell_race_lost']
            groups = {label: {outcome: sum(classes[kind][outcome] for kind in kinds
                        if kind != 'Unclassified' and (kind in eligible) == selected)
                      for outcome in outcomes} for label, selected in [('eligible', True), ('ineligible', False)]}
            classified.append({'suite': suite, 'phase': row['trace_phase'], 'iterations': 4096,
                'events': event, 'classes': classes, 'eligibility_groups': groups,
                'nonzero_kinds': [kind for kind in kinds if any(classes[kind].values())]})
    assert len(classified) == 68
    old_rational = [s['trace'] for s in json.loads((BORROWED / 'rational-trace-v2/samples.json').read_text())
                    if s['group'] == 'cache-coverage']
    old_quadratic = [s['rows'] for s in json.loads((COVERAGE / 'diagnostic-v1/samples.json').read_text())]
    assert len(old_rational) == len(old_quadratic) == 2
    assert old_rational[0] == old_rational[1] and old_quadratic[0] == old_quadratic[1]
    history = []
    for suite, old_rows in [('rational', old_rational[0]), ('quadratic', old_quadratic[0])]:
        for old, new in zip(old_rows, by_group[suite, 'classified'], strict=True):
            assert old['trace_phase'] == new['trace_phase'] and old['iterations'] == new['iterations'] == 4096
            assert old['cache_events'] == new['cache_events']
            history.append({'suite': suite, 'phase': old['trace_phase'], 'all_27_event_counts_match': True})
    assert len(history) == 56
    totals = {suite: {label: {outcome: sum(r['eligibility_groups'][label][outcome] for r in classified if r['suite'] == suite)
                            for outcome in outcomes} for label in ['eligible', 'ineligible']} for suite in SUITES}
    dump(destination, {'status': 'actual-cache-read-classes-verified-for-all-68-phases',
        'assessor_sha256': sha(__file__), 'result_sha256': sha(out / 'result.json'),
        'preparation_sha256': sha(PREP / 'protocol.json'), 'commands_passed': 53, 'scalar_tests': 881,
        'retained_test_groups': retained_groups, 'fixed_count_processes': 24,
        'trace_windows': 544, 'classified_windows': 136, 'distinct_classified_phases': 68,
        'all_counts_repeat_in_reversed_order': True, 'all_witnesses_and_representations_match': True,
        'instrumented_parent_public_counts_equal_in_all_68_phases': True,
        'public_comparisons': comparisons, 'classified_phases': classified,
        'old_27_event_comparisons': history, 'all_56_historical_counter_phases_match': True,
        'per_suite_totals_per_classified_process': totals, 'all_lookup_partitions_match': True,
        'unclassified_public_lookups': 0, 'ineligible_hits': 0,
        'limits': ['Counts identify executed read branches, not their timing, contention or memory cost.',
                   'An initialized miss does not identify which other cache payload caused cell initialization.',
                   'Two fixed reversed passes are retained; no filtering, replacement, extension or pooling.',
                   'The rejected entry guard and all earlier native/library failures remain recorded.',
                   'No new runtime optimization, native gate, allocation sample or performance admission is performed.'],
        'artifacts_sha256': {str(p.relative_to(E)): sha(p) for p in sorted(out.iterdir()) if p.is_file()},
        'runtime_candidate': False, 'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'verified': True, 'phases': 68, 'historical_27_event_phases_match': 56,
        'all_public_and_classified_counts_repeat': True, 'totals': totals, 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
