"""Verify the fixed memo observations against all closed parent trace phases."""
import gzip
import json
from pathlib import Path
import re
from leaf_memo_common_v2 import E, ROOT, PARENT, PREP, GROUPS, ORDERS, SUITES, sha, dump, verify


def rows(raw):
    return [json.loads(line) for line in raw.splitlines() if line.startswith('{')]


def identity(raw):
    return [r for r in rows(raw) if 'fixture' in r or 'witness' in r]


def public(row):
    return {k: v for k, v in row.items() if k not in ['cache_events', 'lookup_classes', 'memo_payloads']}


def names(raw):
    return set(re.findall(r'^test ([^ ]+) \.\.\. ok$', raw, re.MULTILINE))


def main():
    destination = E / 'assessment-v2.json'
    assert not destination.exists()
    prep = json.loads((PREP / 'protocol.json').read_text())
    verify(prep, full=True)
    scope = json.loads((E / 'scope-v2.json').read_text())
    out = E / 'diagnostic-v2'
    result = json.loads((out / 'result.json').read_text())
    assert result['complete'] and result['commands'] == 29 and result['scalar_tests'] == 884
    assert result['fixed_count_processes'] == 12 and result['trace_windows'] == 272
    assert all(sha(out / p) == h for p, h in result['artifact_sha256'].items())
    assert {p.name for p in out.iterdir() if p.is_file()} == set(result['artifact_sha256']) | {'result.json'}
    protocol = json.loads((out / 'protocol.json').read_text())
    assert protocol['assessor_sha256'] == sha(__file__) and protocol['runner_sha256'] == sha(E / 'measure_v2.py')
    assert protocol['preparation_sha256'] == sha(PREP / 'protocol.json')
    assert protocol['review_sha256'] == sha(E / 'source-review-v2.json')
    assert protocol['groups'] == scope['groups'] == GROUPS
    assert protocol['orders'] == scope['orders'] == ORDERS
    assert protocol['suites'] == scope['suites'] == SUITES and protocol['inputs'] == 4096
    commands = json.loads((out / 'commands.json').read_text())
    assert len(commands) == len({r['name'] for r in commands}) == 29
    assert all(r['exit_code'] == 0 for r in commands)
    raw = {}
    for command in commands:
        for stream in ['stdout', 'stderr']:
            path = out / (command['name'] + '.' + stream + '.gz')
            assert sha(path) == command[stream + '_sha256']
            raw[command['name'], stream] = gzip.decompress(path.read_bytes()).decode()
        if command['name'].startswith(('clippy-', 'link-')):
            assert '-D' in command['command'] and command['command'][command['command'].index('-D') + 1] == 'warnings'
    expected_builds = ['format-inputs', 'clippy-library', 'build-tests', 'build-library', 'scalar-library']
    expected_builds += [step + '-' + suite + '-' + group for suite in SUITES for group in GROUPS for step in ['clippy', 'link']]
    assert [r['name'] for r in commands[:17]] == expected_builds
    assert '--all-features' in commands[1]['command'] and '--all-targets' in commands[1]['command']
    assert 'test result: ok. 884 passed; 0 failed; 0 ignored;' in raw['scalar-library', 'stdout']
    old_names = names(gzip.decompress((PARENT / 'diagnostic-v2/scalar-library.stdout.gz').read_bytes()).decode())
    actual = names(raw['scalar-library', 'stdout'])
    tests = json.loads((out / 'scalar-tests.json').read_text())
    assert len(old_names) == 881 and len(actual) == 884
    assert actual == old_names | set(tests['counter_controls']) and len(tests['counter_controls']) == 3
    groups = {fragment: sum(fragment in name for name in actual) for fragment in [
        'quadratic_tower_sign_retention_tests::', 'persistent_laurent_cache_tests::',
        'laurent_cache_eligibility_tests::', 'borrowed_integer_tests::', 'memo_leaf_trace_tests::']}
    assert list(groups.values()) == [11, 9, 3, 2, 2]
    binaries = json.loads((out / 'binaries.json').read_text())
    libraries = json.loads((out / 'libraries.json').read_text())
    assert len(binaries) == 7 and set(libraries) == set(GROUPS)
    assert all(sha(row['path']) == row['sha256'] and Path(row['path']).stat().st_size == row['bytes'] for row in binaries.values())
    assert all(sha(row['path']) == row['sha256'] for group in libraries.values() for row in group.values())
    assert {k: r['features'] for k, r in libraries['borrowed'].items()} == {k: r['features'] for k, r in libraries['diagnostic'].items()}
    test_artifacts = [r for r in rows(raw['build-tests', 'stdout']) if r.get('reason') == 'compiler-artifact'
                      and r.get('executable') == binaries['library-tests']['path']]
    assert len(test_artifacts) == 1 and test_artifacts[0]['features'] == binaries['library-tests']['features']
    assert test_artifacts[0]['manifest_path'] == str(Path(prep['source']) / 'crates/hyperreal/Cargo.toml')
    for name, library in libraries['diagnostic'].items():
        artifacts = [r for r in rows(raw['build-library', 'stdout']) if r.get('reason') == 'compiler-artifact'
                     and library['path'] in r.get('filenames', [])]
        assert len(artifacts) == 1 and artifacts[0]['features'] == library['features']
        assert artifacts[0]['manifest_path'] == library['manifest_path']
    for suite in SUITES:
        for group in GROUPS:
            for step in ['clippy', 'link']:
                cmd = next(r['command'] for r in commands if r['name'] == f'{step}-{suite}-{group}')
                assert str(PREP / (suite + '_probe.rs')) in cmd
                assert ('--cfg' in cmd) == (group == 'diagnostic')
                if group == 'diagnostic':
                    assert cmd[cmd.index('--cfg') + 1] == 'feature="cache-events"'
                for name, library in libraries[group].items():
                    assert name + '=' + library['path'] in cmd
    expected = [(suite, repetition, position, group) for suite in SUITES
                for repetition, order in enumerate(ORDERS, 1) for position, group in enumerate(order, 1)]
    samples = json.loads((out / 'samples.json').read_text())
    assert len(samples) == len(expected) == 12
    references = {suite: gzip.decompress((PARENT / 'diagnostic-v2' / ('count-' + suite + '-1-borrowed.stdout.gz')).read_bytes()).decode() for suite in SUITES}
    for sample, expected_row, command in zip(samples, expected, commands[17:], strict=True):
        suite, repetition, position, group = expected_row
        assert tuple(sample[k] for k in ['suite', 'repetition', 'position', 'group']) == expected_row
        name = f'count-{suite}-{repetition}-{group}'
        assert sample['command_name'] == command['name'] == name
        assert command['command'] == ['taskset', '-c', '0', binaries[suite + '-' + group]['path'], '4096']
        assert identity(raw[name, 'stdout']) == identity(references[suite]) and sample['identity_matches_closed_parent_diagnostic']
        parsed = rows(raw[name, 'stdout'])
        assert len(parsed) == len(raw[name, 'stdout'].splitlines())
        trace = [r for r in parsed if 'trace_phase' in r]
        assert trace == sample['trace']
        assert [r['trace_phase'] for r in trace] == [r['trace_phase'] for r in rows(references[suite]) if 'trace_phase' in r]
        assert all(r['iterations'] == 4096 for r in trace)
        for row in trace:
            assert all((row[key] is not None) == (group == 'diagnostic') for key in ['cache_events', 'lookup_classes', 'memo_payloads'])
    old_samples = json.loads((PARENT / 'diagnostic-v2/samples.json').read_text())
    by_group = {}
    history = []
    for suite in SUITES:
        for group in GROUPS:
            own = [s['trace'] for s in samples if (s['suite'], s['group']) == (suite, group)]
            assert len(own) == 2 and own[0] == own[1]
            by_group[suite, group] = own[0]
            previous_group = 'borrowed' if group == 'borrowed' else 'classified'
            previous = [s['trace'] for s in old_samples if (s['suite'], s['group']) == (suite, previous_group)]
            assert len(previous) == 2 and previous[0] == previous[1]
            assert [{k: v for k, v in row.items() if k != 'memo_payloads'} for row in own[0]] == previous[0]
        assert [public(r) for r in by_group[suite, 'borrowed']] == [public(r) for r in by_group[suite, 'diagnostic']]
        history.extend({'suite': suite, 'phase': r['trace_phase'], 'public_counts_equal': True,
                        'all_27_cache_events_match_parent': True, 'all_61_reader_classes_match_parent': True}
                       for r in by_group[suite, 'diagnostic'])
    assert len(history) == 68
    classified = []
    for suite in SUITES:
        for row in by_group[suite, 'diagnostic']:
            payloads = row['memo_payloads']
            assert set(payloads) == set(scope['buckets'])
            for kind, modes in payloads.items():
                assert set(modes) == set(scope['modes'])
                for mode, events in modes.items():
                    assert set(events) == set(scope['events'])
                    for event, counts in events.items():
                        assert set(counts) == set(scope['fields'])
                        assert all(type(n) is int and n >= 0 for n in counts.values())
                        assert counts['events'] == sum(counts[key] for key in scope['fields'][1:7])
                        assert counts['terms'] >= counts['events'] - counts['none'] - counts['empty']
                        assert counts['radicals'] >= counts['one_radical_term']
                        assert counts['atoms'] >= counts['opaque']
            totals = {mode: {event: {field: sum(payloads[kind][mode][event][field] for kind in scope['buckets'])
                        for field in scope['fields']} for event in scope['events']} for mode in scope['modes']}
            assert sum(totals[mode]['memo_store']['events'] for mode in scope['modes']) == row['cache_events']['local_memo_store']
            assert sum(totals[mode]['memo_hit']['events'] for mode in scope['modes']) == row['cache_events']['local_memo_hit']
            for mode, visit_event in [('retained', 'retained_visit'), ('original', 'original_visit')]:
                assert sum(totals[mode][event]['events'] for event in scope['events']) <= row['cache_events'][visit_event]
            classified.append({'suite': suite, 'phase': row['trace_phase'], 'iterations': 4096,
                'memo_payloads': payloads, 'mode_event_totals': totals,
                'old_local_memo_store': row['cache_events']['local_memo_store'],
                'old_local_memo_hit': row['cache_events']['local_memo_hit']})
    assert len(classified) == 68
    totals = {suite: {kind: {mode: {event: {field: sum(r['memo_payloads'][kind][mode][event][field]
                       for r in classified if r['suite'] == suite) for field in scope['fields']}
                       for event in scope['events']} for mode in scope['modes']}
                       for kind in scope['buckets']} for suite in SUITES}
    dump(destination, {'status': 'local-memo-payload-counts-verified-for-all-68-phases',
        'assessor_sha256': sha(__file__), 'result_sha256': sha(out / 'result.json'),
        'preparation_sha256': sha(PREP / 'protocol.json'), 'commands_passed': 29, 'scalar_tests': 884,
        'prior_881_test_names_retained': True, 'retained_test_groups': groups, 'fixed_count_processes': 12,
        'trace_windows': 272, 'diagnostic_windows': 136, 'distinct_phases': 68,
        'all_counts_repeat_in_reversed_order': True, 'all_witnesses_and_representations_match': True,
        'all_public_counts_equal_to_borrowed': True, 'all_old_cache_and_reader_counts_match_parent': True,
        'all_memo_event_and_shape_partitions_match': True, 'all_phase_comparisons': history,
        'classified_phases': classified, 'per_suite_totals_per_diagnostic_process': totals,
        'shape_definitions': scope['shape_definitions'],
        'limits': ['This classifies copies at the existing local memo sites, not their timing or measured heap cost.',
                   'One rational term is a rational coefficient at any pi power, not necessarily a rational value.',
                   'Observed absent workload categories remain valid zeros. Independent controls exercise other branches.',
                   'The fixed two reversed passes are closed; no filtering, extensions, replacements or pooling.',
                   'All old failures, rejected optimizations and original native limits remain recorded.'],
        'artifacts_sha256': {str(p.relative_to(E)): sha(p) for p in sorted(out.iterdir()) if p.is_file()},
        'new_native_gates_run': False, 'runtime_candidate': False, 'runtime_admitted': False, 'plan_complete': False})
    summary = {suite: {kind: {event: sum(totals[suite][kind][mode][event]['events'] for mode in scope['modes'])
                        for event in scope['events']} for kind in scope['buckets']} for suite in SUITES}
    print(json.dumps({'verified': True, 'phases': 68, 'all_parent_counters_match': True,
                      'all_memo_partitions_match': True, 'events': summary, 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
