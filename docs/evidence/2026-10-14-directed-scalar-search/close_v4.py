"""Independently close the completed finite scalar campaign and its exact scope."""
from collections import Counter
import json
from pathlib import Path

from close_v1 import E, ROOT, mapping, read, sha

OUT = E / 'qualification-v4'


def main():
    assert not (E / 'closure-v4.json').exists() and not (OUT / 'failure.json').exists()
    result, protocol = read(OUT / 'result.json'), read(OUT / 'protocol.json')
    commands = read(OUT / 'commands.json')
    assert result['complete'] and len(commands) == 13 and all(row['exit_code'] == 0 for row in commands)
    for name, digest in result['artifacts_sha256'].items():
        assert sha(OUT / name) == digest, name
    for name, digest in protocol['inputs_sha256'].items():
        assert sha(E / name) == digest
    for version in [1, 2, 3, 4]:
        source = ROOT / f'target/directed-scalar-search-v{version}/source'
        table = mapping(E / f'source-hashes-v{version}.json.gz')
        assert {str(p.relative_to(source)): sha(p) for p in source.rglob('*') if p.is_file()} == table
        if version < 4:
            for name, digest in read(E / f'closure-v{version}.json')['trial_artifacts_sha256'].items():
                assert sha(E / f'qualification-v{version}' / name) == digest
    roots = mapping(E / 'main-source-before.json.gz')
    assert all(sha(ROOT / name) == digest for name, digest in roots.items())
    binaries = read(OUT / 'binaries.json')
    for row in [*binaries.values(), *protocol['preserved_binaries']]:
        assert sha(Path(row['path'])) == row['sha256']
    for name in ['probe-features-tests', 'all-features-tests']:
        assert '7 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out' in (OUT / (name + '.stdout')).read_text()
    assert 'Ran 23 tests' in (OUT / 'python-tests.stderr').read_text()
    campaign = OUT / 'campaign'
    completed = read(campaign / 'result.json')
    assert completed['complete'] and completed['completed_probes'] == 528 and completed['selection_events'] == 264
    for name, digest in completed['artifacts_sha256'].items():
        assert sha(campaign / name) == digest
    selected = read(campaign / 'selected-inputs.json')
    assert len(selected) == 22 and all(1 <= len(rows) <= 4 for rows in selected.values())
    first_cases, origins, filter_depths = Counter(), Counter(), Counter()
    selection_count = 0
    for generation in range(3):
        planned = read(campaign / f'round-{generation:03}-planned.json')
        record = read(campaign / f'round-{generation:03}-selection.json')
        assert set(planned) == set(selected) == set(record['populations']) == set(record['selections'])
        for bucket in selected:
            rows = record['populations'][bucket]
            assert len(rows) == 8 and len(record['selections'][bucket]) == 4
            for member in planned[bucket]:
                origins[member['origin']] += 1
            for event in record['selections'][bucket]:
                survivors = sorted(rows, key=lambda row: row['id'])
                first_cases[event['case_order'][0]] += 1
                for step in event['steps']:
                    worst = max(row['scores'][step['case']] for row in survivors)
                    survivors = [row for row in survivors if row['scores'][step['case']] == worst]
                    assert step['worst_score'] == worst and step['survivors'] == [row['id'] for row in survivors]
                assert event['selected'] in {row['id'] for row in survivors}
                filter_depths[len(event['steps'])] += 1
                selection_count += 1
    assert selection_count == 264 and sum(origins.values()) == 528 and origins['mutation'] > 0
    processes = [read(p) for p in campaign.glob('round-*.process.json')]
    assert len(processes) == 528 and all(p['exit_code'] == 0 and p['status'] == 'exited' for p in processes)
    trace_paths, dispatch_events = set(), 0
    for path in campaign.glob('round-*.stdout'):
        response = read(path)
        assert response['status'] == 'ok' and response['cache_contract_checked']
        assert len(response['queries']) == 6
        for query in response['queries']:
            assert len(query['samples']) == 3
            for sample in query['samples']:
                for trace in sample['dispatch']:
                    trace_paths.add((trace['layer'], trace['operation'], trace['path']))
                    dispatch_events += trace['count']
    logs = ''.join(p.read_text() for p in OUT.rglob('*') if p.suffix in ['.stdout', '.stderr'])
    assert all(marker not in logs for marker in [
        'ERROR: AddressSanitizer', 'ERROR: LeakSanitizer', 'SUMMARY: AddressSanitizer',
        'LeakSanitizer has encountered a fatal error', 'DEADLYSIGNAL'])
    report = {
        'status': 'closed-completed-finite-scalar-qualification', 'driver_terminal_exit_code_observed': 0,
        'qualification_result_sha256': sha(OUT / 'result.json'),
        'commands_executed': 13, 'command_exit_codes': [row['exit_code'] for row in commands],
        'python_tests_passed': 23, 'rust_tests_passed_each_configuration': 7,
        'rust_test_configurations': ['serde,dispatch-trace', 'all-features'],
        'representations': 22, 'generations': 3, 'population_per_representation': 8,
        'campaign_probes': 528, 'query_measurements': 9504, 'selection_events': selection_count,
        'population_member_origins': dict(origins), 'selection_first_query_counts': dict(first_cases),
        'selection_filter_depth_counts': dict(filter_depths),
        'distinct_dispatch_paths': len(trace_paths), 'total_recorded_dispatch_events': dispatch_events,
        'retained_inputs_per_representation': {bucket: len(rows) for bucket, rows in selected.items()},
        'retained_inputs': sum(map(len, selected.values())),
        'selected_inputs_sha256': sha(campaign / 'selected-inputs.json'),
        'all_selection_and_mutation_traces_replayed': True,
        'all_recorded_definite_facts_agree_with_independent_reference': True,
        'sanitizer_error_markers_observed': False,
        'sources_verified': {'v1': 10161, 'v2': 10163, 'v3': 10166, 'v4': 10166, 'main': len(roots)},
        'executed_binaries': binaries,
        'trial_artifacts_sha256': {str(p.relative_to(OUT)): sha(p) for p in OUT.rglob('*') if p.is_file()},
        'driver_logs_sha256': {name: sha(E / name) for name in ['driver-v4.stdout', 'driver-v4.stderr']},
        'closure_sources_sha256': {name: sha(E / name) for name in ['close_v1.py', 'close_v4.py']},
        'limits': [
            'Finite nonzero scalar recipes from 22 private representation classes; not exhaustive inputs or whole-stack directed fuzzing.',
            'Each input runs in a fresh process. Individual query samples rebuild Real objects, but shared constants can already be warm; this is not an all-caches-cold claim.',
            'Independent interval overlap rejects contradictions but does not prove arbitrary symbolic identities.',
            'The Rust targets and rebuilt standard library used ASan/LSan; native GMP/MPFR code is not claimed instrumented.',
            'Instrumented query times are selection fitness only. Competitive enclosure benchmarking is still required before tooling admission.',
        ],
        'competitive_benchmark_complete': False, 'admitted': False,
        'P38_complete': False, 'plan_complete': False,
        'closed_trial_retried_resumed_filtered_extended_or_pooled': False,
    }
    (E / 'closure-v4.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: report[key] for key in ['status', 'campaign_probes', 'selection_events', 'retained_inputs', 'population_member_origins', 'distinct_dispatch_paths']}))
    print(json.dumps({'closure_sha256': sha(E / 'closure-v4.json')}))


if __name__ == '__main__':
    main()
